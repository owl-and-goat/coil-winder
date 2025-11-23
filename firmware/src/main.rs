#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{net::Ipv4Addr, ptr::addr_of_mut};

use cyw43::{Control, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_net::{Ipv4Cidr, StackResources};
use embassy_rp::{
    bind_interrupts,
    clocks::RoscRng,
    gpio::{Level, Output},
    multicore::Stack,
    peripherals::{DMA_CH0, PIN_0, PIO0},
    pio::{InterruptHandler, Pio},
    Peri,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{self, Channel},
};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use picoserve::make_static;
use static_cell::StaticCell;

use crate::motion::ICoord;

use {defmt_rtt as _, panic_probe as _};

mod driver;
mod motion;
mod server;
pub(crate) mod util;

pub(crate) const WIFI_NETWORK: Option<&str> = option_env!("WIFI_NETWORK");
pub(crate) const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");
pub(crate) const PORT: u16 = 1234;
pub(crate) const AXES: usize = 4;
pub(crate) const AXIS_LABELS: [char; AXES] = ['X', 'Z', 'C', 'F'];
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
    control: Control<'static>,
    command_tx: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        gcode::Command<AXES>,
        COMMAND_BUFFER_SIZE,
    >,
) -> ! {
    server::run(stack, control, command_tx).await
}

// TODO(aspen): Move this onto Core 2
#[embassy_executor::task]
async fn motion_task(
    motion: motion::State,
    driver: driver::Driver<'static, PIO0, 1, 2, 3>,
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

    let prgs = driver::Programs::new(&mut pio.common);
    let driver = driver::Driver::new(
        pio.common,
        /* sleep_pin = */ p.PIN_9,
        driver::config::Axes {
            x_axis: driver::config::Axis {
                step: p.PIN_10,
                dir: p.PIN_11,
                zero_limit: Some(p.PIN_6),
                irq: pio.irq1,
                sm: pio.sm1,
            },
            z_axis: driver::config::Axis {
                step: p.PIN_12,
                dir: p.PIN_13,
                zero_limit: Some(p.PIN_7),
                irq: pio.irq2,
                sm: pio.sm2,
            },
            c_axis: driver::config::Axis {
                step: p.PIN_14,
                dir: p.PIN_15,
                zero_limit: None::</* can put anything here, lol */ Peri<PIN_0>>,
                irq: pio.irq3,
                sm: pio.sm3,
            },
        },
        prgs,
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
                        /* X */
                        motion::Axis {
                            microns_per_step: ICoord::from_num(12).into(),
                            degrees_per_step: (ICoord::lit("1.8") / ICoord::from_num(16)).into(),
                            unit: motion::AxisUnit::Millimeters,
                        },
                        /* Z */
                        motion::Axis {
                            microns_per_step: (ICoord::from_num(6)).into(),
                            degrees_per_step: (ICoord::lit("0.9") / ICoord::from_num(16)).into(),
                            unit: motion::AxisUnit::Millimeters,
                        },
                        /* C */
                        motion::Axis {
                            microns_per_step: ICoord::from_num(12).into(),
                            degrees_per_step: (ICoord::lit("1.8") / ICoord::from_num(16)).into(),
                            unit: motion::AxisUnit::Rotations,
                        },
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
