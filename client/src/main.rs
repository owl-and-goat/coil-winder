use std::{
    collections::HashMap,
    fmt::{self, Display},
    io::{BufRead, ErrorKind, Read},
    net::SocketAddr,
    sync::Arc,
};

use clap::Parser;
use clio::Input;
use eyre::{bail, eyre, Result};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use lexpr::Value;
use rustyline_async::ReadlineEvent;
use tokio::{
    io::{self, AsyncBufReadExt, AsyncWriteExt, BufReader},
    net::{tcp, TcpStream, ToSocketAddrs},
    sync::{mpsc, Mutex},
    task::JoinHandle,
};
use tracing::{debug, info, warn};

const AXES: usize = 4;
const AXIS_LABELS: [char; AXES] = ['X', 'Z', 'C', 'F'];

#[derive(clap::Subcommand, Debug)]
enum Command {
    /// Interactive REPL for sending gcode commands
    Repl,

    /// Send a single Gcode command from the command line
    Oneshot {
        /// Command to send
        #[clap(long, short)]
        command: String,
    },

    /// Run a gcode program
    Run {
        /// Path to gcode program to run
        #[clap(value_parser)]
        program: Input,

        /// Skip verification that the program parses before running
        #[clap(long)]
        no_verify: bool,
    },
}

#[derive(clap::Parser, Debug)]
#[command(version)]
struct Args {
    #[clap(long, short, default_value = "192.168.11.40:1234")]
    addr: SocketAddr,

    #[clap(subcommand)]
    command: Command,
}

fn read_commands(input: impl Read, verify: bool) -> Result<Vec<String>> {
    let reader = std::io::BufReader::new(input);
    let mut res = Vec::new();
    if !verify {
        for line in reader.lines() {
            res.push(line?);
        }
    } else {
        for (i, line) in reader.lines().enumerate() {
            let line = line?;
            let line = line.trim();
            if line.starts_with('(') {
                if !line.ends_with(')') {
                    bail!("line {i}: Mismatched parenthesis in comment");
                }
                continue;
            }
            let line = match line.rsplit_once(';') {
                Some((line, _comment)) => line,
                None => line,
            };
            let mut line = line.to_owned();
            if !line.ends_with('\n') {
                line.push('\n');
            }
            match gcode::parse_single_command(AXIS_LABELS, line.as_bytes()) {
                Ok(_) => {}
                Err(_) => bail!("line {i}: Invalid gcode command: \"{}\"", line.trim()),
            };
            res.push(line);
        }
    }

    Ok(res)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandId(u32);

impl Display for CommandId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ack(Option<CommandId>);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Done(CommandId);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Response {
    Ack(Ack),
    Done(Done),
}

impl Response {
    fn from_sexp(value: lexpr::Value) -> Result<Self, lexpr::Value> {
        match value.get(0) {
            Some(Value::Symbol(s)) if s.as_ref() == "ack" => match value.get(1) {
                None => Ok(Self::Ack(Ack(None))),
                Some(id) => {
                    match id
                        .as_number()
                        .and_then(|v| v.as_u64())
                        .and_then(|v| u32::try_from(v).ok())
                    {
                        Some(id) => Ok(Self::Ack(Ack(Some(CommandId(id as _))))),
                        None => Err(value),
                    }
                }
            },
            Some(Value::Symbol(s)) if s.as_ref() == "done" => {
                match value
                    .get(1)
                    .and_then(|v| v.as_number())
                    .and_then(|v| v.as_u64())
                    .and_then(|v| u32::try_from(v).ok())
                {
                    Some(id) => Ok(Self::Done(Done(CommandId(id as _)))),
                    None => Err(value),
                }
            }
            None => {
                if value.as_str() == Some("ack") {
                    Ok(Self::Ack(Ack(None)))
                } else {
                    Err(value)
                }
            }
            _ => Err(value),
        }
    }
}

#[cfg(test)]
mod response_tests {
    use super::*;

    fn resp_from_sexp(sexp: &str) -> Response {
        let sexp = lexpr::from_str(sexp).unwrap();
        Response::from_sexp(sexp).unwrap()
    }

    #[test]
    fn ack() {
        assert_eq!(resp_from_sexp("(ack)"), Response::Ack(Ack(None)));
    }

    #[test]
    fn done() {
        assert_eq!(
            resp_from_sexp("(done 8)"),
            Response::Done(Done(CommandId(8)))
        );
    }
}

pub struct Client {
    addr: SocketAddr,
    ack_rx: mpsc::Receiver<Ack>,
    ack_tx: mpsc::Sender<Ack>,
    done_tx: mpsc::UnboundedSender<Done>,
    writer: tcp::OwnedWriteHalf,
    reader: JoinHandle<()>,
}

impl Client {
    pub async fn connect(
        addr: impl ToSocketAddrs,
    ) -> Result<(Self, mpsc::UnboundedReceiver<Done>)> {
        let stream = TcpStream::connect(addr).await?;
        stream.set_nodelay(true)?;
        let addr = stream.peer_addr()?;

        let (reader, writer) = stream.into_split();
        let buf_reader = BufReader::new(reader);

        let (ack_tx, ack_rx) = mpsc::channel(1);
        let (done_tx, done_rx) = mpsc::unbounded_channel();
        let reader = Self::spawn_reader(buf_reader, ack_tx.clone(), done_tx.clone());

        Ok((
            Self {
                addr,
                ack_tx,
                ack_rx,
                done_tx,
                writer,
                reader,
            },
            done_rx,
        ))
    }

    fn spawn_reader(
        buf_reader: BufReader<tcp::OwnedReadHalf>,
        ack_tx: mpsc::Sender<Ack>,
        done_tx: mpsc::UnboundedSender<Done>,
    ) -> JoinHandle<()> {
        tokio::spawn(async move {
            let mut lines = buf_reader.lines();
            while let Ok(Some(line)) = lines.next_line().await {
                debug!(line, "got line from server");
                match lexpr::from_str(&line.trim().trim_matches('\0').trim()) {
                    Err(err) => {
                        warn!(%err, "Invalid s-expression from server");
                    }
                    Ok(value) => match Response::from_sexp(value) {
                        Ok(Response::Ack(ack)) => {
                            debug!(?ack);
                            if let Err(error) = ack_tx.send(ack).await {
                                warn!(%error, "ack_tx send error")
                            }
                        }
                        Ok(Response::Done(done)) => {
                            debug!(?done);
                            if let Err(error) = done_tx.send(done) {
                                warn!(%error, "done_tx send error");
                            }
                        }
                        Err(value) => warn!(%value, "Unhandled message from server"),
                    },
                }
            }
            warn!("reader exited");
        })
    }

    async fn reconnect(&mut self) -> io::Result<()> {
        let stream = TcpStream::connect(self.addr).await?;
        stream.set_nodelay(true)?;
        self.addr = stream.peer_addr()?;

        let (reader, writer) = stream.into_split();
        let buf_reader = BufReader::new(reader);

        self.reader.abort();
        self.reader = Self::spawn_reader(buf_reader, self.ack_tx.clone(), self.done_tx.clone());

        self.writer = writer;

        Ok(())
    }

    pub async fn send(&mut self, mut command: String) -> Result<Ack> {
        if !command.ends_with('\n') {
            command.push('\n');
        }

        loop {
            match self.writer.write_all(command.as_bytes()).await {
                Ok(()) => break,
                Err(err) => match err.kind() {
                    ErrorKind::ConnectionReset
                    | ErrorKind::ConnectionAborted
                    | ErrorKind::BrokenPipe
                    | ErrorKind::NotConnected => {
                        self.reconnect().await?;
                        continue;
                    }
                    _ => return Err(err.into()),
                },
            }
        }
        self.writer.flush().await?;

        self.ack_rx
            .recv()
            .await
            .ok_or_else(|| eyre!("ack channel closed"))
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let args = Args::parse();
    let (mut client, mut done_rx) = Client::connect(args.addr).await?;

    match args.command {
        Command::Oneshot { command } => {
            let ack = client.send(command).await?;
            let res = done_rx.recv().await;
            debug!(?res);
            match res {
                Some(Done(CommandId(id))) => match ack.0 {
                    None => info!(id, "done"),
                    Some(CommandId(ack_id)) if ack_id == id => info!(id, "done"),
                    Some(CommandId(ack_id)) => {
                        bail!("got different done id ({id}) than ack id ({ack_id})??")
                    }
                },
                None => bail!("command_rx closed"),
            }
            println!("ok");
            Ok(())
        }
        Command::Run { program, no_verify } => {
            let commands = read_commands(program, !no_verify)?;
            let n = commands.len();
            let multi_progress = MultiProgress::new();
            let upload_bar = multi_progress.add(
                ProgressBar::new(n as u64).with_style(
                    ProgressStyle::with_template("upl [{pos}/{len}]|{bar:50}|[{elapsed}/{eta}]")
                        .unwrap(),
                ),
            );
            let run_bar = Arc::new(
                multi_progress.add(
                    ProgressBar::new(n as u64).with_style(
                        ProgressStyle::with_template(
                            "run [{pos}/{len}]|{bar:50}|[{elapsed}/{eta}]{msg}",
                        )
                        .unwrap(),
                    ),
                ),
            );

            let sent_commands = Arc::new(Mutex::new(HashMap::<CommandId, String>::new()));

            let done_progress = tokio::spawn({
                let sent_commands = Arc::clone(&sent_commands);
                let run_bar = Arc::clone(&run_bar);
                async move {
                    while let Some(Done(command_id)) = done_rx.recv().await {
                        if let Some(command) = sent_commands.lock().await.remove(&command_id) {
                            run_bar.set_message(command.trim().to_owned())
                        } else {
                            warn!(%command_id, "unexpected command id in done msg from server");
                        }
                        run_bar.inc(1);
                    }
                }
            });

            for command in commands {
                upload_bar.set_message(command.trim().to_owned());
                match client.send(command.clone()).await? {
                    Ack(None) => {
                        run_bar.set_message(command.clone());
                        run_bar.inc(1)
                    }
                    Ack(Some(command_id)) => {
                        sent_commands.lock().await.insert(command_id, command);
                    }
                }
                upload_bar.inc(1);
            }

            done_progress.await?;

            println!("Successfully ran {n} commands");
            Ok(())
        }
        Command::Repl => loop {
            let (mut rl, _rlwriter) = rustyline_async::Readline::new(">> ".to_owned())?;

            match rl.readline().await? {
                ReadlineEvent::Eof => break Ok(()),
                ReadlineEvent::Interrupted => continue,
                ReadlineEvent::Line(command) => {
                    client.send(command).await?;
                }
            }
        },
    }
}
