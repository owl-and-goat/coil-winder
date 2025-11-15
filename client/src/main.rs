use std::io::{self, BufRead, BufReader, ErrorKind, Write};
use std::net::{SocketAddr, TcpStream};

use clap::Parser;
use eyre::{bail, Result};
use rustyline::history::History;
use rustyline::{error::ReadlineError, DefaultEditor};

#[derive(clap::Subcommand, Debug)]
enum Command {
    Repl,
    Oneshot {
        /// Command to send
        #[clap(long, short)]
        command: String,
    },
}

#[derive(clap::Parser, Debug)]
#[command(version)]
struct Args {
    #[clap(default_value = "192.168.11.40:1234")]
    addr: SocketAddr,

    #[clap(subcommand)]
    command: Command,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut rl = DefaultEditor::new()?;
    let mut history = rustyline::history::MemHistory::new();

    let mut stream = TcpStream::connect(args.addr)?;
    let mut reader = BufReader::new(stream.try_clone()?);

    match args.command {
        Command::Oneshot { mut command } => {
            command.push('\n');
            stream.write_all(command.as_bytes())?;
            stream.flush()?;
            let mut resp = String::new();
            if reader.read_line(&mut resp)? == 0 {
                bail!("Got EOF from server");
            }
            println!("{resp}");
            Ok(())
        }
        Command::Repl => {
            let reconnect = || -> io::Result<_> {
                let stream = TcpStream::connect(args.addr)?;
                let reader = BufReader::new(stream.try_clone()?);
                Ok((stream, reader))
            };

            loop {
                match rl.readline(">> ") {
                    Err(ReadlineError::Eof) => break,
                    Err(ReadlineError::Interrupted) => continue,
                    Err(err) => {
                        eprintln!("{err}");
                        break;
                    }
                    Ok(mut line) => {
                        history.add(&line)?;
                        line.push('\n');
                        loop {
                            match stream.write_all(line.as_bytes()) {
                                Ok(()) => break,
                                Err(err) => match err.kind() {
                                    ErrorKind::ConnectionReset | ErrorKind::BrokenPipe => {
                                        (stream, reader) = reconnect()?;
                                        continue;
                                    }
                                    _ => return Err(err.into()),
                                },
                            }
                        }
                        stream.flush()?;
                        let mut resp = String::new();
                        if reader.read_line(&mut resp)? == 0 {
                            (stream, reader) = reconnect()?;
                        } else {
                            println!("{resp}");
                        }
                    }
                }
            }
            Ok(())
        }
    }
}
