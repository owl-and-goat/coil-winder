#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{net::Ipv4Addr, ptr::addr_of_mut};

use cyw43::{aligned_bytes, Control, JoinOptions};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::Either;
use embassy_net::{Ipv4Cidr, StackResources};
use embassy_rp::{
    bind_interrupts,
    clocks::RoscRng,
    dma,
    gpio::{Drive, Level, Output},
    multicore::Stack,
    peripherals::{DMA_CH0, PIN_0, PIO0},
    pio::{InterruptHandler, Pio},
    Peri,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel};
use embassy_time::{with_timeout, Duration, Timer};
use heapless::Vec;
use static_cell::StaticCell;

use crate::motion::ICoord;

use {defmt_rtt as _, panic_probe as _};

#[macro_use]
pub(crate) mod util;
mod driver;
mod motion;
mod server;

pub(crate) const WIFI_NETWORK: Option<&str> = option_env!("WIFI_NETWORK");
pub(crate) const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");
pub(crate) const PORT: u16 = 1234;
pub(crate) const AXES: usize = 4;
pub(crate) const AXIS_LABELS: [char; AXES] = ['X', 'Z', 'C', 'F'];
pub const COMMAND_BUFFER_SIZE: usize = 32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct CommandId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum MotionStatusMsg {
    CommandFinished(CommandId),
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn server_task(server: server::Server) -> ! {
    server.run().await
}

#[embassy_executor::task]
async fn motion_task(
    motion: motion::State,
    driver: driver::Driver<'static, PIO0, 1, 2, 3>,
    command_rx: channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        (CommandId, gcode::Command<AXES>),
        COMMAND_BUFFER_SIZE,
    >,
    status_tx: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        MotionStatusMsg,
        COMMAND_BUFFER_SIZE,
    >,
) -> ! {
    motion.run(driver, command_rx, status_tx).await;
}

async fn blink_once(control: &mut Control<'_>) {
    const DELAY: Duration = Duration::from_millis(500);
    control.gpio_set(0, true).await;
    Timer::after(DELAY).await;
    control.gpio_set(0, false).await;
}

struct StatusLeds {
    amber: Output<'static>,
    green: Output<'static>,
    red: Output<'static>,
}

#[derive(Clone, Copy, defmt::Format)]
enum Color {
    Amber,
    Green,
    Red,
}

impl StatusLeds {
    pub fn setup(&mut self) {
        self.all_pins()
            .for_each(|p| p.set_drive_strength(Drive::_2mA));
    }

    fn pin(&mut self, color: Color) -> &mut Output<'static> {
        match color {
            Color::Amber => &mut self.amber,
            Color::Green => &mut self.green,
            Color::Red => &mut self.red,
        }
    }

    fn all_pins(&mut self) -> impl Iterator<Item = &mut Output<'static>> {
        [&mut self.amber, &mut self.green, &mut self.red].into_iter()
    }

    pub fn set_exclusive(&mut self, color: Color) {
        self.all_pins().for_each(|p| p.set_low());
        debug!("Setting color: {:?}", color);
        self.pin(color).set_high();
    }
}

#[embassy_executor::task]
async fn net_status(mut status_leds: StatusLeds, stack: &'static embassy_net::Stack<'static>) -> ! {
    if stack.is_link_up() {
        status_leds.set_exclusive(Color::Green);
    } else {
        status_leds.set_exclusive(Color::Amber);
        stack.wait_link_up().await;
    }

    loop {
        stack.wait_link_down().await;
        status_leds.set_exclusive(Color::Amber);
        stack.wait_link_up().await;
        status_leds.set_exclusive(Color::Green);
    }
}

#[embassy_executor::task]
async fn core0(
    pwr: Output<'static>,
    spi: PioSpi<'static, PIO0, 0>,
    spawner: Spawner,
    mut status_leds: StatusLeds,
    command_tx: channel::Sender<
        'static,
        CriticalSectionRawMutex,
        (CommandId, gcode::Command<AXES>),
        COMMAND_BUFFER_SIZE,
    >,
    status_rx: channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        MotionStatusMsg,
        COMMAND_BUFFER_SIZE,
    >,
) -> ! {
    status_leds.setup();

    let fw = aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let state = make_static!(cyw43::State, cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;

    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        net_device,
        // Config::dhcpv4(Default::default())
        embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
            // TODO(aspen): Make these configurable at compile-time
            address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 1, 40), 24),
            dns_servers: Vec::new(),
            gateway: Some(Ipv4Addr::new(192, 168, 1, 1)),
        }),
        make_static!(StackResources<3>, StackResources::new()),
        RoscRng.next_u64(),
    );
    let stack = make_static!(embassy_net::Stack<'static>, stack);
    spawner.spawn(unwrap!(net_task(runner)));

    status_leds.set_exclusive(Color::Amber);
    while let Err(err) = match with_timeout(
        Duration::from_secs(5),
        control.join(
            WIFI_NETWORK.unwrap_or(""),
            JoinOptions::new(WIFI_PASSWORD.unwrap_or("").as_bytes()),
        ),
    )
    .await
    {
        Ok(Ok(())) => Ok(()),
        Err(timeout) => Err(Either::First(timeout)),
        Ok(Err(join_error)) => Err(Either::Second(join_error)),
    } {
        match err {
            Either::First(timeout) => error!("Join timed out, retrying ({})", timeout),
            Either::Second(err) => info!("join failed: {}", err),
        }
        status_leds.set_exclusive(Color::Red);
    }
    spawner.spawn(unwrap!(net_status(status_leds, stack)));

    server::Server {
        stack,
        control,
        command_tx,
        status_rx,
        command_id_gen: 0,
    }
    .run()
    .await
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
        dma::Channel::new(p.DMA_CH0, Irqs),
    );

    let prgs = driver::Programs::new(&mut pio.common);
    let driver = driver::Driver::new(
        pio.common,
        /* sleep_pin = */ p.PIN_15,
        driver::config::Axes {
            x_axis: driver::config::Axis {
                step: p.PIN_4,
                dir: p.PIN_5,
                zero_limit: Some(p.PIN_2),
                irq: pio.irq1,
                sm: pio.sm1,
            },
            z_axis: driver::config::Axis {
                step: p.PIN_20,
                dir: p.PIN_21,
                zero_limit: Some(p.PIN_3),
                irq: pio.irq2,
                sm: pio.sm2,
            },
            c_axis: driver::config::Axis {
                step: p.PIN_16,
                dir: p.PIN_17,
                zero_limit: None::</* can put anything here, lol */ Peri<PIN_0>>,
                irq: pio.irq3,
                sm: pio.sm3,
            },
        },
        prgs,
    );

    static COMMAND_CHANNEL: StaticCell<
        channel::Channel<
            CriticalSectionRawMutex,
            (CommandId, gcode::Command<AXES>),
            COMMAND_BUFFER_SIZE,
        >,
    > = StaticCell::new();
    let command_channel: &'static mut _ = COMMAND_CHANNEL.init(channel::Channel::new());
    let command_tx = command_channel.sender();
    let command_rx = command_channel.receiver();

    static STATUS_CHANNEL: StaticCell<
        channel::Channel<CriticalSectionRawMutex, MotionStatusMsg, COMMAND_BUFFER_SIZE>,
    > = StaticCell::new();
    let status_channel: &'static _ = STATUS_CHANNEL.init(channel::Channel::new());
    let status_tx = status_channel.sender();
    let status_rx = status_channel.receiver();

    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(unwrap!(motion_task(
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
                    status_tx,
                )))
            })
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(core0(
            pwr,
            spi,
            spawner,
            StatusLeds {
                amber: Output::new(p.PIN_7, Level::Low),
                green: Output::new(p.PIN_10, Level::Low),
                red: Output::new(p.PIN_13, Level::Low),
            },
            command_tx,
            status_rx,
        )))
    })
}
