use cyw43::Control;
use defmt::{info, warn};
use embassy_net::tcp::TcpSocket;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel};
use embassy_time::Duration;
use embedded_io_async::Write;

use crate::{blink_once, AXES, AXIS_LABELS, COMMAND_BUFFER_SIZE, PORT};

pub async fn run(
    stack: embassy_net::Stack<'static>,
    mut control: Control<'static>,
    command_tx: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        gcode::Command<AXES>,
        COMMAND_BUFFER_SIZE,
    >,
) -> ! {
    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];
    let mut buf = [0; 2048];

    'accept: loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        let mut n = 0;
        socket.set_timeout(Some(Duration::from_secs(10)));

        if let Err(e) = socket.accept(PORT).await {
            warn!("accept error: {}", e);
            continue;
        }

        blink_once(&mut control).await;
        loop {
            let command = {
                'read_command: loop {
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

                    match gcode::parse_single_command(AXIS_LABELS, &buf[..n]) {
                        Ok((remaining, command)) => {
                            let start = usize::try_from(unsafe {
                                remaining.as_ptr().offset_from(buf.as_ptr())
                            })
                            .unwrap();
                            let end = start + remaining.len();
                            let len = remaining.len();
                            buf.copy_within(start..end, 0);
                            n = len;
                            info!("Got command: {}", &buf[..n]);
                            break 'read_command command;
                        }
                        Err(gcode::Error::Incomplete(_)) => continue 'read_command,
                        Err(gcode::Error::ParseFailed) => {
                            warn!("parse failed");
                            if let Err(e) = socket.write_all(b"wtf!\n").await {
                                warn!("write error: {}", e);
                                continue 'accept;
                            }
                            continue 'accept;
                        }
                    };
                }
            };

            blink_once(&mut control).await;

            match command {
                gcode::Command::Stop => {
                    // TODO(aspen): Also cancel the current command
                    command_tx.clear();
                }
                command => {
                    command_tx.send(command).await;
                }
            }

            if let Err(e) = socket.write_all(b"gotcha!\n").await {
                warn!("write error: {}", e);
                continue 'accept;
            }
        }
    }
}
