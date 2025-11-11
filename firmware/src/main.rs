#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{fmt::Write, net::Ipv4Addr};

use cyw43::{Control, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Ipv4Cidr, StackResources};
use embassy_rp::{
    bind_interrupts,
    clocks::RoscRng,
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIO0},
    pio::{InterruptHandler, Pio},
};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use picoserve::make_static;
use {defmt_rtt as _, panic_probe as _};

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const PORT: u16 = 1234;
const AXES: [char; 4] = ['X', 'Y', 'Z', 'F'];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn server_task(stack: embassy_net::Stack<'static>, mut control: Control<'static>) -> ! {
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

                    match gcode::parse_single_command(AXES, &buf[..n]) {
                        Ok((remaining, command)) => {
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
                        Err(gcode::Error::Incomplete(_)) => continue 'read_command,
                        Err(gcode::Error::ParseFailed) => {
                            warn!("parse failed");
                            continue 'accept;
                        }
                    };
                }
            };

            blink_once(&mut control).await;
            let mut resp = Vec::<_, 1024>::new();
            writeln!(&mut resp, "{command:?}").unwrap();
            if let Err(e) = embedded_io_async::Write::write_all(&mut socket, &resp).await {
                warn!("write error: {}", e);
                continue 'accept;
            }
        }
    }
}

async fn blink_once(control: &mut Control<'_>) {
    const DELAY: Duration = Duration::from_millis(500);
    control.gpio_set(0, true).await;
    Timer::after(DELAY).await;
    control.gpio_set(0, false).await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    let state = make_static!(cyw43::State, cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.must_spawn(cyw43_task(runner));

    control.init(clm).await;

    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        net_device,
        // Config::dhcpv4(Default::default())
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 11, 40), 24),
            dns_servers: Vec::new(),
            gateway: Some(Ipv4Addr::new(192, 168, 11, 1)),
        }),
        make_static!(StackResources<3>, StackResources::new()),
        RoscRng.next_u64(),
    );
    spawner.must_spawn(net_task(runner));

    while let Err(err) = control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        info!("join failed with status={}", err.status);
    }

    // let control = make_static!(Mutex<NoopRawMutex, Control>, Mutex::new(control));
    spawner.must_spawn(server_task(stack, control));
}
