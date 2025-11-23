use std::{
    io::{BufRead, ErrorKind, Read},
    net::SocketAddr,
};

use clap::Parser;
use clio::Input;
use eyre::{Result, bail};
use indicatif::{ProgressBar, ProgressStyle};
use rustyline_async::ReadlineEvent;
use tokio::{
    io::{self, AsyncBufReadExt, AsyncWriteExt, BufReader},
    net::{TcpStream, ToSocketAddrs, tcp},
};
// use rustyline::history::History;
// use rustyline::{error::ReadlineError, DefaultEditor};

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

        /// Print responses from the chip after each command
        #[clap(long)]
        print_responses: bool,
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

pub struct Client {
    addr: SocketAddr,
    reader: BufReader<tcp::OwnedReadHalf>,
    writer: tcp::OwnedWriteHalf,
}

impl Client {
    pub async fn connect(addr: impl ToSocketAddrs) -> io::Result<Self> {
        let stream = TcpStream::connect(addr).await?;
        stream.set_nodelay(true)?;
        let addr = stream.peer_addr()?;

        let (reader, writer) = stream.into_split();
        let reader = BufReader::new(reader);

        Ok(Self {
            addr,
            reader,
            writer,
        })
    }

    async fn reconnect(&mut self) -> io::Result<()> {
        let stream = TcpStream::connect(self.addr).await?;
        stream.set_nodelay(true)?;
        self.addr = stream.peer_addr()?;

        let (reader, writer) = stream.into_split();
        self.reader = BufReader::new(reader);
        self.writer = writer;
        Ok(())
    }

    pub async fn send(&mut self, mut command: String) -> Result<String> {
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
        let mut resp = String::new();
        if self.reader.read_line(&mut resp).await? == 0 {
            self.reconnect().await?;
        }
        Ok(resp)
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let (mut rl, rlwriter) = rustyline_async::Readline::new(">> ".to_owned())?;
    let mut client = Client::connect(args.addr).await?;

    match args.command {
        Command::Oneshot { command } => {
            let resp = client.send(command).await?;
            if resp.len() == 0 {
                bail!("Got EOF from server");
            }
            println!("{resp}");
            Ok(())
        }
        Command::Run {
            program,
            no_verify,
            print_responses,
        } => {
            let commands = read_commands(program, !no_verify)?;
            let n = commands.len();
            let bar = ProgressBar::new(n as u64).with_style(
                ProgressStyle::with_template("[{pos}/{len}][{elapsed}/{eta}]|{bar:50.blue}|{msg}")
                    .unwrap(),
            );
            for command in commands {
                bar.inc(1);
                bar.set_message(command.clone());
                let resp = client.send(command).await?;
                if print_responses {
                    bar.println(resp);
                }
            }

            println!("Successfully ran {n} commands");
            Ok(())
        }
        Command::Repl => loop {
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
