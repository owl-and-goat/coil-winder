use std::fs::File;
use std::io::{self, BufRead, BufReader, ErrorKind, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::path::PathBuf;

use clap::Parser;
use eyre::{bail, Result};
use rustyline::history::History;
use rustyline::{error::ReadlineError, DefaultEditor};

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
        #[clap(long, short)]
        path: PathBuf,

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

fn read_commands_from_file(path: PathBuf, verify: bool) -> Result<Vec<String>> {
    let reader = BufReader::new(File::open(&path)?);
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
                    bail!("{}:{i}: Mismatched parenthesis in comment", path.display());
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
            match gcode::parse_single_command(['X', 'Y', 'Z', 'F'], line.as_bytes()) {
                Ok(_) => {}
                Err(_) => bail!("{}:{i}: Invalid gcode command", path.display()),
            };
            res.push(line);
        }
    }

    Ok(res)
}

pub struct Client {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
}

impl Client {
    pub fn connect(addr: impl ToSocketAddrs) -> io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        let reader = BufReader::new(stream.try_clone()?);
        Ok(Self { stream, reader })
    }

    fn reconnect(&mut self) -> io::Result<()> {
        let addr = self.stream.peer_addr()?;
        self.stream = TcpStream::connect(addr)?;
        self.reader = BufReader::new(self.stream.try_clone()?);
        Ok(())
    }

    pub fn send(&mut self, mut command: String) -> io::Result<String> {
        if !command.ends_with('\n') {
            command.push('\n');
        }

        loop {
            match self.stream.write_all(command.as_bytes()) {
                Ok(()) => break,
                Err(err) => match err.kind() {
                    ErrorKind::ConnectionReset | ErrorKind::BrokenPipe => {
                        self.reconnect()?;
                        continue;
                    }
                    _ => return Err(err.into()),
                },
            }
        }
        self.stream.flush()?;
        let mut resp = String::new();
        if self.reader.read_line(&mut resp)? == 0 {
            self.reconnect()?;
        }
        Ok(resp)
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut rl = DefaultEditor::new()?;
    let mut history = rustyline::history::MemHistory::new();
    let mut client = Client::connect(args.addr)?;

    match args.command {
        Command::Oneshot { command } => {
            let resp = client.send(command)?;
            if resp.len() == 0 {
                bail!("Got EOF from server");
            }
            println!("{resp}");
            Ok(())
        }
        Command::Run {
            path,
            no_verify,
            print_responses,
        } => {
            let commands = read_commands_from_file(path, !no_verify)?;
            let n = commands.len();
            for command in commands {
                let resp = client.send(command)?;
                if print_responses {
                    println!("{resp}");
                }
            }

            println!("Successfully ran {n} commands");

            Ok(())
        }
        Command::Repl => {
            loop {
                match rl.readline(">> ") {
                    Err(ReadlineError::Eof) => break,
                    Err(ReadlineError::Interrupted) => continue,
                    Err(err) => {
                        eprintln!("{err}");
                        break;
                    }
                    Ok(command) => {
                        history.add(&command)?;
                        client.send(command)?;
                    }
                }
            }
            Ok(())
        }
    }
}
