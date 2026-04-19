#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{net::Ipv4Addr, ptr::addr_of_mut};

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::Either;
use embassy_net::StackResources;
use embassy_rp::{
    bind_interrupts,
    clocks::RoscRng,
    gpio::{Input, Level, Output, Pull},
    multicore::Stack,
    peripherals::{DMA_CH0, PIN_0, PIO0},
    pio::{InterruptHandler, Pio},
    Peri,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel};
use embassy_time::{with_timeout, Duration};
use static_cell::StaticCell;

use crate::{
    motion::ICoord,
    status_leds::{Behavior, BlinkSpeed, Status},
};

use {defmt_rtt as _, panic_probe as _};

#[macro_use]
pub(crate) mod util;
mod driver;
mod mdns;
mod motion;
mod server;
mod status_leds;

pub(crate) const WIFI_NETWORK: Option<&str> = option_env!("WIFI_NETWORK");
pub(crate) const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");
pub(crate) const PORT: u16 = 1234;
pub(crate) const MDNS_HOSTNAME: &str = "coil-winder";
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

#[embassy_executor::task]
async fn status_leds_task(runner: status_leds::Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn mdns_task(
    stack: embassy_net::Stack<'static>,
    hostname: &'static str,
    ip: Ipv4Addr,
    rng: RoscRng,
) -> ! {
    mdns::run(stack, hostname, ip, rng).await
}

fn status_behavior(status: Status) -> status_leds::PerColor<Behavior> {
    match status {
        Status::Initializing => status_leds::PerColor {
            amber: Behavior::On,
            ..Default::default()
        },
        Status::WifiConnecting => status_leds::PerColor {
            amber: Behavior::Blink(BlinkSpeed::Fast),
            ..Default::default()
        },
        Status::WaitingForDhcp => status_leds::PerColor {
            amber: Behavior::Blink(BlinkSpeed::Slow),
            ..Default::default()
        },
        Status::Ready => status_leds::PerColor {
            green: Behavior::On,
            ..Default::default()
        },
        Status::Paused => status_leds::PerColor {
            green: Behavior::Blink(BlinkSpeed::Fast),
            ..Default::default()
        },
        Status::WifiError => status_leds::PerColor {
            red: Behavior::On,
            ..Default::default()
        },
        Status::ExecutingCommand => status_leds::PerColor {
            green: Behavior::On,
            amber: Behavior::Blink(BlinkSpeed::Slow),
            ..Default::default()
        },
        Status::XHomingFailed => status_leds::PerColor {
            red: Behavior::Blink(BlinkSpeed::Fast),
            amber: Behavior::On,
            ..Default::default()
        },
        Status::ZHomingFailed => status_leds::PerColor {
            red: Behavior::On,
            amber: Behavior::Blink(BlinkSpeed::Fast),
            ..Default::default()
        },
        Status::BothHomingFailed => status_leds::PerColor {
            red: Behavior::Blink(BlinkSpeed::Fast),
            amber: Behavior::Blink(BlinkSpeed::Fast),
            ..Default::default()
        },
        Status::BadCommand => status_leds::PerColor {
            green: Behavior::On,
            red: Behavior::Blink(BlinkSpeed::Slow),
            ..Default::default()
        },
    }
}

#[embassy_executor::task]
async fn core0(
    pwr: Output<'static>,
    spi: PioSpi<'static, PIO0, 0, DMA_CH0>,
    spawner: Spawner,
    status_leds_runner: status_leds::Runner<'static>,
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
    let status_leds = status_leds_runner.control();
    spawner.must_spawn(status_leds_task(status_leds_runner));
    status_leds.set_status(Status::Initializing);

    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let state = make_static!(cyw43::State, cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.must_spawn(cyw43_task(runner));

    control.init(clm).await;

    control
        .set_power_management(cyw43::PowerManagementMode::Performance)
        .await;

    // Init network stack with DHCP
    let (stack, runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(Default::default()),
        make_static!(StackResources<4>, StackResources::new()),
        RoscRng.next_u64(),
    );
    spawner.must_spawn(net_task(runner));

    status_leds.set_status(Status::WifiConnecting);
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
            Either::Second(err) => {
                status_leds.set_status(Status::WifiError);
                info!("join failed with status={}", err.status)
            }
        }
    }
    // Wait for DHCP to assign an IP
    info!("Waiting for DHCP...");
    status_leds.set_status(Status::WaitingForDhcp);
    stack.wait_config_up().await;
    let our_ip = stack
        .config_v4()
        .map(|c| c.address.address())
        .unwrap_or(Ipv4Addr::UNSPECIFIED);
    info!("DHCP assigned IP: {}", our_ip);
    status_leds.set_status(Status::Ready);

    // Add mDNS multicast MAC address to cyw43 hardware filter
    // IPv4 multicast 224.0.0.251 -> MAC 01:00:5e:00:00:fb
    const MDNS_MULTICAST_MAC: [u8; 6] = [0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb];
    match control.add_multicast_address(MDNS_MULTICAST_MAC).await {
        Ok(_) => info!("Added mDNS multicast MAC filter"),
        Err(_) => error!("Failed to add mDNS multicast MAC"),
    }

    // Start mDNS responder
    spawner.must_spawn(mdns_task(stack, MDNS_HOSTNAME, our_ip, RoscRng));

    server::Server {
        stack,
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

    let status = make_static!(status_leds::Control, status_leds::Control::new());
    let status_leds = status_leds::Runner::new(
        status,
        status_leds::PerColor {
            amber: Output::new(p.PIN_7, Level::Low),
            green: Output::new(p.PIN_10, Level::Low),
            red: Output::new(p.PIN_13, Level::Low),
        },
        status_behavior,
    );

    let status_leds_control = status_leds.control();

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
                spawner.must_spawn(motion_task(
                    motion::State::new(
                        [
                            /* X */
                            motion::Axis {
                                microns_per_step: ICoord::from_num(12).into(),
                                degrees_per_step: (ICoord::lit("1.8") / ICoord::from_num(16))
                                    .into(),
                                unit: motion::AxisUnit::Millimeters,
                                length: ICoord::from_num(140),
                            },
                            /* Z */
                            motion::Axis {
                                microns_per_step: (ICoord::from_num(6)).into(),
                                degrees_per_step: (ICoord::lit("0.9") / ICoord::from_num(16))
                                    .into(),
                                unit: motion::AxisUnit::Millimeters,
                                length: ICoord::from_num(400),
                            },
                            /* C */
                            motion::Axis {
                                microns_per_step: ICoord::from_num(12).into(),
                                degrees_per_step: (ICoord::lit("1.8") / ICoord::from_num(16))
                                    .into(),
                                unit: motion::AxisUnit::Rotations,
                                length: ICoord::from_num(0),
                            },
                        ],
                        status_leds_control,
                        Input::new(p.PIN_22, Pull::Up),
                    ),
                    driver,
                    command_rx,
                    status_tx,
                ))
            })
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(core0(pwr, spi, spawner, status_leds, command_tx, status_rx))
    })
}
