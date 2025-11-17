#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{net::Ipv4Addr, ptr::addr_of_mut};

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
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{self, Channel},
};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use heapless::Vec;
use picoserve::make_static;
use static_cell::StaticCell;

use crate::motion::{ICoord, MicronsPerStep};

use {defmt_rtt as _, panic_probe as _};

mod a4988;
mod motion;
pub(crate) mod util;

const WIFI_NETWORK: Option<&str> = option_env!("WIFI_NETWORK");
const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");
const PORT: u16 = 1234;
const AXES: usize = 4;
const AXIS_LABELS: [char; AXES] = ['X', 'Z', 'C', 'F'];
pub const COMMAND_BUFFER_SIZE: usize = 32;

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
            command_tx.send(command).await;
            if let Err(e) = socket.write_all(b"gotcha!\n").await {
                warn!("write error: {}", e);
                continue 'accept;
            }
        }
    }
}

// TODO(aspen): Move this onto Core 2
#[embassy_executor::task]
async fn motion_task(
    motion: motion::State,
    driver: a4988::Driver<'static, PIO0, 1, 2, 3>,
    command_rx: channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        gcode::Command<AXES>,
        COMMAND_BUFFER_SIZE,
    >,
) -> ! {
    motion.run(driver, command_rx).await;
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
    command_tx: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        gcode::Command<AXES>,
        COMMAND_BUFFER_SIZE,
    >,
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
        .join(
            WIFI_NETWORK.unwrap_or(""),
            JoinOptions::new(WIFI_PASSWORD.unwrap_or("").as_bytes()),
        )
        .await
    {
        info!("join failed with status={}", err.status);
    }

    // let control = make_static!(Mutex<NoopRawMutex, Control>, Mutex::new(control));
    spawner.must_spawn(server_task(stack, control, command_tx));
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

    // let driver = a4988::Driver::builder()
    //     .direction_pin(p.PIN_15)
    // .step_axis_pins([
    //     Output::new(p.PIN_16, Level::Low),
    // Output::new(p.PIN_17, Level::Low),
    // Output::new(p.PIN_18, Level::Low),
    // Output::new(p.PIN_19, Level::Low),
    // ])
    // .build();

    let prgs = a4988::Programs::new(&mut pio.common);
    let driver = a4988::Driver::new(
        pio.common,
        a4988::config::Axes {
            x_axis: a4988::config::Axis {
                step: p.PIN_10,
                dir: p.PIN_11,
                irq: pio.irq1,
                sm: pio.sm1,
            },
            z_axis: a4988::config::Axis {
                step: p.PIN_12,
                dir: p.PIN_13,
                irq: pio.irq2,
                sm: pio.sm2,
            },
            c_axis: a4988::config::Axis {
                step: p.PIN_14,
                dir: p.PIN_15,
                irq: pio.irq3,
                sm: pio.sm3,
            },
        },
        &prgs,
    );

    static COMMAND_CHANNEL: StaticCell<
        Channel<CriticalSectionRawMutex, gcode::Command<AXES>, COMMAND_BUFFER_SIZE>,
    > = StaticCell::new();
    let command_channel: &'static _ = COMMAND_CHANNEL.init(Channel::new());
    let command_rx = command_channel.receiver();
    let command_tx = command_channel.sender();

    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.must_spawn(motion_task(
                    motion::State::new([
                        /* X */ MicronsPerStep(ICoord::from_num(12)),
                        /* Z */ MicronsPerStep(ICoord::from_num(6)),
                        /* C */ MicronsPerStep(ICoord::from_num(6)),
                    ]),
                    driver,
                    command_rx,
                ))
            })
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| spawner.must_spawn(core0(pwr, spi, spawner, command_tx)))
}
