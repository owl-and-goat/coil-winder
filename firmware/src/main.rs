#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{net::Ipv4Addr, ptr::addr_of_mut};

use crate::a4988::Direction;
use cyw43::{Control, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_net::{tcp::TcpSocket, Ipv4Cidr, StackResources};
use embassy_rp::{
    bind_interrupts,
    clocks::RoscRng,
    gpio::{Level, Output},
    multicore::Stack,
    peripherals::{DMA_CH0, PIO0},
    pio::{InterruptHandler, Pio},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use heapless::Vec;
use picoserve::make_static;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod a4988;

const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
const PORT: u16 = 1234;
const AXES: usize = 4;
const AXIS_LABELS: [char; AXES] = ['X', 'Y', 'Z', 'F'];

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
async fn server_task(
    stack: embassy_net::Stack<'static>,
    mut control: Control<'static>,
    command_signal: &'static Signal<CriticalSectionRawMutex, gcode::Command<AXES>>,
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

            match &command {
                gcode::Command::RapidMove(pos) | gcode::Command::LinearMove(pos)
                    if pos.0.iter().filter(|p| p.is_some()).count() == 1 =>
                {
                    blink_once(&mut control).await;
                    command_signal.signal(command);
                    if let Err(e) = socket.write_all(b"gotcha!\n").await {
                        warn!("write error: {}", e);
                        continue 'accept;
                    }
                }
                _ => {
                    blink_once(&mut control).await;
                    blink_once(&mut control).await;
                    if let Err(e) = socket.write_all(b"huh??\n").await {
                        warn!("write error: {}", e);
                        continue 'accept;
                    }
                }
            }
        }
    }
}

// TODO(aspen): Move this onto Core 2
#[embassy_executor::task]
async fn stepper_driver_task(
    mut driver: a4988::Driver<AXES>,
    command_signal: &'static Signal<CriticalSectionRawMutex, gcode::Command<AXES>>,
) -> ! {
    loop {
        let command = command_signal.wait().await;
        match command {
            gcode::Command::RapidMove(pos) | gcode::Command::LinearMove(pos) => {
                let Some((axis, dist)) = pos
                    .0
                    .iter()
                    .enumerate()
                    .find_map(|(i, coord)| coord.map(|c| (i, c)))
                else {
                    continue;
                };

                for _ in 0..dist.to_num() {
                    driver.single_step(axis, Direction::Forwards).await;
                }
            }
            _ => continue,
        }
    }
}

async fn blink_once(control: &mut Control<'_>) {
    const DELAY: Duration = Duration::from_millis(500);
    control.gpio_set(0, true).await;
    Timer::after(DELAY).await;
    control.gpio_set(0, false).await;
}

#[embassy_executor::task]
async fn core0(
    pwr: Output<'static>,
    spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    spawner: Spawner,
    command_signal: &'static Signal<CriticalSectionRawMutex, gcode::Command<AXES>>,
) {
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

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
    spawner.must_spawn(server_task(stack, control, command_signal));
}

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

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

    let driver = a4988::Driver::builder()
        .direction_pin(p.PIN_15)
        .step_axis_pins([
            Output::new(p.PIN_16, Level::Low),
            Output::new(p.PIN_17, Level::Low),
            Output::new(p.PIN_18, Level::Low),
            Output::new(p.PIN_19, Level::Low),
        ])
        .build();

    static COMMAND_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, gcode::Command<AXES>>> =
        StaticCell::new();
    let command_signal: &'static _ = COMMAND_SIGNAL.init(Signal::new());

    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| spawner.must_spawn(stepper_driver_task(driver, command_signal)))
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.must_spawn(core0(pwr, spi, spawner, command_signal)))
}
