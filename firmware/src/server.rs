use cyw43::Control;
use defmt::{debug, info, warn};
use embassy_futures::select::{select, Either};
use embassy_net::tcp::TcpSocket;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel};
use embassy_time::Duration;
use embedded_io_async::Write;

use crate::{blink_once, CommandId, MotionStatusMsg, AXES, AXIS_LABELS, COMMAND_BUFFER_SIZE, PORT};

pub struct Server {
    pub stack: embassy_net::Stack<'static>,
    pub control: Control<'static>,
    pub command_tx: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        (CommandId, gcode::Command<AXES>),
        COMMAND_BUFFER_SIZE,
    >,
    pub status_rx:
        channel::Receiver<'static, CriticalSectionRawMutex, MotionStatusMsg, COMMAND_BUFFER_SIZE>,
    pub command_id_gen: u32,
}

impl Server {
    fn gen_command_id(&mut self) -> CommandId {
        self.command_id_gen += 1;
        CommandId(self.command_id_gen)
    }

    pub async fn run(mut self) -> ! {
        let mut rx_buffer = [0; 1024];
        let mut tx_buffer = [0; 1024];
        let mut buf = [0; 2048];

        'accept: loop {
            let mut socket = TcpSocket::new(self.stack, &mut rx_buffer, &mut tx_buffer);
            let mut n = 0;
            socket.set_timeout(Some(Duration::from_secs(10)));

            if let Err(e) = socket.accept(PORT).await {
                warn!("accept error: {}", e);
                continue;
            }

            blink_once(&mut self.control).await;
            loop {
                match select(socket.read(&mut buf[n..]), self.status_rx.receive()).await {
                    Either::Second(MotionStatusMsg::CommandFinished(CommandId(id))) => {
                        debug!("Sending status message");
                        let mut done = [0u8; 64];
                        {
                            use embedded_io::Write;
                            write!(&mut done[..], "(done {id})\n").unwrap();
                        }
                        if let Err(e) = socket.write_all(&done).await {
                            warn!("write error: {}", e);
                        }
                    }
                    Either::First(res) => {
                        match res {
                            Ok(read) => n += read,
                            Err(err) => {
                                warn!("read error: {}", err);
                                continue 'accept;
                            }
                        };
                        debug!("reading command, starting at {}", n);
                        let command = {
                            'read_command: loop {
                                match gcode::parse_single_command(AXIS_LABELS, &buf[..n]) {
                                    Ok((remaining, command)) => {
                                        info!("Got command: {}", &buf[..n]);
                                        let start = usize::try_from(unsafe {
                                            remaining.as_ptr().offset_from(buf.as_ptr())
                                        })
                                        .unwrap();
                                        let end = start + remaining.len();
                                        let len = remaining.len();
                                        buf.copy_within(start..end, 0);
                                        n = len;
                                        break 'read_command command;
                                    }
                                    Err(gcode::Error::Incomplete(_)) => { /* keep reading */ }
                                    Err(gcode::Error::ParseFailed) => {
                                        warn!("parse failed");
                                        if let Err(e) = socket.write_all(b"(parse failed)!\n").await
                                        {
                                            warn!("write error: {}", e);
                                            continue 'accept;
                                        }
                                        continue 'accept;
                                    }
                                };

                                let read = match socket.read(&mut buf[n..]).await {
                                    Ok(0) => {
                                        warn!("read EOF");
                                        continue 'accept;
                                    }
                                    Ok(n) => n,
                                    Err(e) => {
                                        warn!("read error: {}", e);
                                        continue 'accept;
                                    }
                                };
                                n += read;
                            }
                        };

                        blink_once(&mut self.control).await;

                        match command {
                            gcode::Command::Stop => {
                                // TODO(aspen): Also cancel the current command
                                self.command_tx.clear();

                                if let Err(e) = socket.write_all(b"(ack)\n").await {
                                    warn!("write error: {}", e);
                                    continue 'accept;
                                }
                            }
                            command => {
                                let command_id = self.gen_command_id();
                                self.command_tx.send((command_id, command)).await;

                                {
                                    let mut resp_buf = [0u8; 64];
                                    use embedded_io::Write;
                                    resp_buf.fill(0);
                                    write!(&mut resp_buf[..], "(ack {})\n", command_id.0).unwrap();
                                    if let Err(e) = socket.write_all(&resp_buf).await {
                                        warn!("write error: {}", e);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
