//! Ref: https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf

use defmt::{debug, info, Format};
use embassy_futures::join::{join, join3};
use embassy_rp::{
    gpio::{self, Level, Pull},
    pio::{self, PioPin},
    pio_programs::clock_divider::calculate_pio_clock_divider,
    Peri,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    channel,
};
use embassy_time::Instant;
use fixed::types::extra::U8;

use crate::{CommandId, MotionStatusMsg, COMMAND_BUFFER_SIZE};

const PIO_TARGET_HZ: u32 =
    // 2 μs per cycle
    500_000;

#[derive(Debug, Format, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct StepsPerSecond(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Direction {
    Forwards,
    Backwards,
}

impl From<i32> for Direction {
    fn from(value: i32) -> Self {
        if value > 0 {
            Self::Forwards
        } else {
            Self::Backwards
        }
    }
}

/// The number of instructions per loop of the pio program. Gives a fixed overhead to the incoming
/// "sleeps per cycle" count
const LOOP_OVERHEAD: u32 = 4;

impl StepsPerSecond {
    fn to_sleep_cyles_per_step(self) -> u32 {
        // TODO(aspen): division error?? probably doesn't matter?
        if self.0 == 0 {
            // This doesn't matter (we get 0 speed if we aren't moving), so we return Big Safe
            // Number
            return PIO_TARGET_HZ;
        }
        (PIO_TARGET_HZ / self.0).saturating_sub(LOOP_OVERHEAD)
    }
}

pub struct Programs<'a, T: pio::Instance> {
    home: pio::LoadedProgram<'a, T>,
    move_: pio::LoadedProgram<'a, T>,
}

impl<'a, T: pio::Instance> Programs<'a, T> {
    /// Load the program into the given pio
    pub fn new(common: &mut pio::Common<'a, T>) -> Self {
        let home = common.load_program(&::pio::pio_file!("src/home.s").program);
        let move_ = common.load_program(&::pio::pio_file!("src/move.s").program);
        Self { home, move_ }
    }
}

pub mod config {
    use super::*;
    use embassy_rp::pio::PioPin;

    pub struct Axis<'d, T: pio::Instance, D: PioPin, S: PioPin, ZL: PioPin, const SM: usize> {
        /// Direction pin
        pub dir: Peri<'d, D>,
        /// Step pin
        pub step: Peri<'d, S>,
        /// Zero limit switch input pin
        pub zero_limit: Option<Peri<'d, ZL>>,
        pub irq: Option<pio::Irq<'d, T, SM>>,
        pub sm: pio::StateMachine<'d, T, SM>,
    }

    pub struct Axes<
        'd,
        T: pio::Instance,
        XD: PioPin,
        XS: PioPin,
        XZL: PioPin,
        const XSM: usize,
        ZD: PioPin,
        ZS: PioPin,
        ZZL: PioPin,
        const ZSM: usize,
        CD: PioPin,
        CS: PioPin,
        CZL: PioPin,
        const CSM: usize,
    > {
        pub x_axis: Axis<'d, T, XD, XS, XZL, XSM>,
        pub z_axis: Axis<'d, T, ZD, ZS, ZZL, ZSM>,
        pub c_axis: Axis<'d, T, CD, CS, CZL, CSM>,
    }
}

struct Axis<'d, T: pio::Instance, const SM: usize> {
    sm: pio::StateMachine<'d, T, SM>,
    // irq: pio::Irq<'d, T, SM>,
    dir_pin: pio::Pin<'d, T>,
    step_pin: pio::Pin<'d, T>,
    zero_limit_pin: Option<pio::Pin<'d, T>>,
}

impl<'d, T: pio::Instance, const SM: usize> Axis<'d, T, SM> {
    pub fn new(
        pio: &mut pio::Common<'d, T>,
        axis: config::Axis<'d, T, impl PioPin, impl PioPin, impl PioPin, SM>,
    ) -> Self {
        let config::Axis {
            mut sm,
            step,
            zero_limit,
            dir,
            irq: _,
        } = axis;

        let dir_pin = pio.make_pio_pin(dir);
        let step_pin = pio.make_pio_pin(step);

        sm.set_pin_dirs(pio::Direction::Out, &[&step_pin, &dir_pin]);

        let zero_limit_pin = zero_limit.map(|zero_limit| {
            let mut zero_limit_pin = pio.make_pio_pin(zero_limit);
            zero_limit_pin.set_pull(Pull::Up);
            zero_limit_pin.set_schmitt(true);
            sm.set_pin_dirs(pio::Direction::In, &[&zero_limit_pin]);
            zero_limit_pin
        });

        sm.set_enable(false);

        Self {
            sm,
            dir_pin,
            step_pin,
            zero_limit_pin,
        }
    }

    pub fn configure(
        &mut self,
        clock_divider: fixed::FixedU32<U8>,
        program: &pio::LoadedProgram<'d, T>,
    ) {
        let mut cfg = pio::Config::default();
        cfg.set_set_pins(&[&self.step_pin]);
        cfg.set_out_pins(&[&self.dir_pin]);

        if let Some(zero_limit_pin) = &self.zero_limit_pin {
            cfg.set_jmp_pin(zero_limit_pin);
        }

        cfg.clock_divider = clock_divider;
        cfg.use_program(program, &[&self.step_pin]);
        self.sm.set_config(&cfg);
    }

    pub(self) async fn push_speed(&mut self, speed: StepsPerSecond, direction: Direction) {
        let speed = speed.to_sleep_cyles_per_step();
        // steps.s expects the direction to be the least significant bit of speed - if direction is
        // negative, pin is low, if positive pin is high
        let speed_and_dir = (speed << 1)
            | (match direction {
                Direction::Forwards => 1u32,
                Direction::Backwards => 0u32,
            });
        self.sm.tx().wait_push(speed_and_dir).await;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConfiguredProgram {
    Home,
    Move,
}

#[derive(Clone, Copy)]
pub struct HomeError {
    pub x_failed: bool,
    pub z_failed: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
    Move {
        steps: [i32; 3],
        speeds: [StepsPerSecond; 3],
    },
    SetSleep(bool),
    Home {
        speeds: [(u32, StepsPerSecond); 2],
    },
}

pub const BUFFER_SIZE: usize = 8;

pub struct Channel {
    command: channel::Channel<NoopRawMutex, (CommandId, Command), BUFFER_SIZE>,
    command_started: channel::Channel<NoopRawMutex, CommandStarted, COMMAND_BUFFER_SIZE>,
    home_done: channel::Channel<NoopRawMutex, (), 1>,
    home_result: channel::Channel<NoopRawMutex, Result<(), HomeError>, 1>,
}

impl Channel {
    pub fn new() -> Self {
        Self {
            command: channel::Channel::new(),
            home_done: channel::Channel::new(),
            home_result: channel::Channel::new(),
            command_started: channel::Channel::new(),
        }
    }
}

pub struct Control {
    tx: channel::Sender<'static, NoopRawMutex, (CommandId, Command), BUFFER_SIZE>,
    home_rx: channel::Receiver<'static, NoopRawMutex, Result<(), HomeError>, 1>,
}

impl Control {
    pub async fn set_sleep(&self, command_id: CommandId, sleep: bool) {
        self.tx.send((command_id, Command::SetSleep(sleep))).await;
    }

    pub async fn home(
        &self,
        command_id: CommandId,
        speeds: [(u32, StepsPerSecond); 2],
    ) -> Result<(), HomeError> {
        self.tx.send((command_id, Command::Home { speeds })).await;
        self.home_rx.receive().await
    }

    pub async fn do_move(
        &self,
        command_id: CommandId,
        steps: [i32; 3],
        speeds: [StepsPerSecond; 3],
    ) {
        self.tx
            .send((command_id, Command::Move { steps, speeds }))
            .await;
    }
}

pub enum CommandStarted {
    SetSleep(CommandId),
    Home(CommandId),
    Move(CommandId),
}

pub struct CommandCompletion<
    'd,
    T: pio::Instance,
    const XSM: usize,
    const ZSM: usize,
    const CSM: usize,
> {
    command_rx: channel::Receiver<'static, NoopRawMutex, CommandStarted, COMMAND_BUFFER_SIZE>,
    home_done_tx: channel::Sender<'static, NoopRawMutex, (), 1>,
    irqs: (
        pio::Irq<'d, T, XSM>,
        pio::Irq<'d, T, ZSM>,
        pio::Irq<'d, T, CSM>,
    ),
}

impl<'d, T: pio::Instance, const XSM: usize, const ZSM: usize, const CSM: usize>
    CommandCompletion<'d, T, XSM, ZSM, CSM>
{
    pub async fn run(
        mut self,
        status_tx: channel::Sender<
            'static,
            CriticalSectionRawMutex,
            MotionStatusMsg,
            COMMAND_BUFFER_SIZE,
        >,
    ) -> ! {
        loop {
            let command_id = match self.command_rx.receive().await {
                CommandStarted::SetSleep(command_id) => command_id,
                CommandStarted::Home(command_id) => {
                    join(self.irqs.0.wait(), self.irqs.1.wait()).await;
                    self.home_done_tx.send(()).await;
                    command_id
                }
                CommandStarted::Move(command_id) => {
                    join3(self.irqs.0.wait(), self.irqs.1.wait(), self.irqs.2.wait()).await;
                    command_id
                }
            };
            status_tx
                .send(MotionStatusMsg::CommandFinished(command_id))
                .await;
        }
    }
}

pub struct Driver<'d, T: pio::Instance, const XSM: usize, const ZSM: usize, const CSM: usize> {
    pio: pio::Common<'d, T>,
    sleep_pin: gpio::Output<'d>,
    axes: (Axis<'d, T, XSM>, Axis<'d, T, ZSM>, Axis<'d, T, CSM>),
    configured_program: Option<ConfiguredProgram>,
    programs: Programs<'d, T>,
    clock_divider: fixed::FixedU32<U8>,
    rx: channel::Receiver<'static, NoopRawMutex, (CommandId, Command), BUFFER_SIZE>,
    home_tx: channel::Sender<'static, NoopRawMutex, Result<(), HomeError>, 1>,
    home_done_rx: channel::Receiver<'static, NoopRawMutex, (), 1>,
    command_started_tx: channel::Sender<'static, NoopRawMutex, CommandStarted, COMMAND_BUFFER_SIZE>,
}

macro_rules! each_axis {
    ($self: expr, |$i:tt, $axis:ident|  $body:block ) => {{
        let $i = 0;
        let $axis = &mut $self.axes.0;
        $body;
    }
    {
        let $i = 1;
        let $axis = &mut $self.axes.1;
        $body;
    }
    {
        let $i = 2;
        let $axis = &mut $self.axes.2;
        $body;
    }};
}

impl<'d, T: pio::Instance, const XSM: usize, const ZSM: usize, const CSM: usize>
    Driver<'d, T, XSM, ZSM, CSM>
{
    pub fn new<
        XD: PioPin,
        XS: PioPin,
        XZL: PioPin,
        ZD: PioPin,
        ZS: PioPin,
        ZZL: PioPin,
        CD: PioPin,
        CS: PioPin,
        CZL: PioPin,
    >(
        mut pio: pio::Common<'d, T>,
        sleep_pin: Peri<'d, impl gpio::Pin>,
        mut axes: config::Axes<'d, T, XD, XS, XZL, XSM, ZD, ZS, ZZL, ZSM, CD, CS, CZL, CSM>,
        programs: Programs<'d, T>,
        channel: &'static Channel,
    ) -> (CommandCompletion<'d, T, XSM, ZSM, CSM>, Control, Self) {
        let clock_divider = calculate_pio_clock_divider(PIO_TARGET_HZ);

        let irqs = (
            axes.x_axis.irq.take().expect("X Axis IRQ must be set"),
            axes.z_axis.irq.take().expect("Z Axis IRQ must be set"),
            axes.c_axis.irq.take().expect("C Axis IRQ must be set"),
        );

        let axes = (
            Axis::new(&mut pio, axes.x_axis),
            Axis::new(&mut pio, axes.z_axis),
            Axis::new(&mut pio, axes.c_axis),
        );

        let sleep_pin = gpio::Output::new(sleep_pin, Level::Low);

        (
            CommandCompletion {
                command_rx: channel.command_started.receiver(),
                home_done_tx: channel.home_done.sender(),
                irqs,
            },
            Control {
                tx: channel.command.sender(),
                home_rx: channel.home_result.receiver(),
            },
            Self {
                pio,
                sleep_pin,
                axes,
                configured_program: None,
                clock_divider,
                programs,
                rx: channel.command.receiver(),
                home_tx: channel.home_result.sender(),
                home_done_rx: channel.home_done.receiver(),
                command_started_tx: channel.command_started.sender(),
            },
        )
    }

    pub async fn run(mut self) -> ! {
        loop {
            let (command_id, command) = self.rx.receive().await;
            match command {
                Command::Move { steps, speeds } => self.do_move(command_id, steps, speeds).await,
                Command::SetSleep(sleep) => self.set_sleep(command_id, sleep).await,
                Command::Home { speeds } => {
                    let result = self.home(command_id, speeds).await;
                    self.home_tx.send(result).await
                }
            }
        }
    }

    fn configure_pio(&mut self, which_program: ConfiguredProgram) {
        if self.configured_program == Some(which_program) {
            return;
        }

        let program = match which_program {
            ConfiguredProgram::Home => &self.programs.home,
            ConfiguredProgram::Move => &self.programs.move_,
        };

        each_axis!(self, |_i, axis| {
            axis.configure(self.clock_divider, program);
        });

        self.configured_program = Some(which_program);
    }

    async fn set_sleep(&mut self, command_id: CommandId, sleep: bool) {
        self.sleep_pin
            .set_level(if sleep { Level::Low } else { Level::High });
        self.command_started_tx
            .send(CommandStarted::SetSleep(command_id))
            .await
    }

    async fn home(
        &mut self,
        command_id: CommandId,
        speeds: [(u32, StepsPerSecond); 2],
    ) -> Result<(), HomeError> {
        debug!("homing");
        self.configure_pio(ConfiguredProgram::Home);

        let mut speeds = speeds.into_iter();

        each_axis!(self, |i, axis| {
            if let Some((max_steps, speed)) = speeds.next() {
                if axis.zero_limit_pin.is_some() {
                    info!("will home axis {}", i);
                    axis.sm.tx().wait_push(max_steps).await;
                    axis.push_speed(speed, Direction::Backwards).await;
                }
            }
        });

        debug!("starting home routine");
        self.pio.apply_sm_batch(|batch| {
            each_axis!(self, |_, axis| {
                if axis.zero_limit_pin.is_some() {
                    batch.restart(&mut axis.sm);
                    batch.set_enable(&mut axis.sm, true);
                }
            });
        });

        self.command_started_tx
            .send(CommandStarted::Home(command_id))
            .await;
        self.home_done_rx.receive().await;
        debug!("finished home routine");

        let (x_left, z_left) = (
            self.axes.0.sm.rx().wait_pull().await,
            self.axes.1.sm.rx().wait_pull().await,
        );
        debug!("x_left = {}, z_left = {}", x_left, z_left);

        // the homing routine unconditionally decrements its count register, and
        // returns if that register is zero BEFORE decrement, meaning the
        // register rolls over and we have to check for u32::MAX instead.
        let x_failed = x_left == u32::MAX;
        let z_failed = z_left == u32::MAX;
        if x_failed || z_failed {
            debug!(
                "homing failed on:{}{}",
                if x_failed { " x" } else { "" },
                if z_failed { " z" } else { "" }
            );
            Err(HomeError { x_failed, z_failed })
        } else {
            Ok(())
        }
    }

    async fn do_move(
        &mut self,
        command_id: CommandId,
        steps: [i32; 3],
        speeds: [StepsPerSecond; 3],
    ) {
        info!("driver do_move (t={:tus})", Instant::now());
        self.configure_pio(ConfiguredProgram::Move);

        each_axis!(self, |i, axis| {
            // corresponds to [pull block] instructions in steps.s
            axis.sm.tx().wait_push(steps[i].unsigned_abs()).await;
            axis.push_speed(speeds[i], Direction::from(steps[i])).await;
        });
        info!("pushed speeds (t={:tus})", Instant::now());

        self.command_started_tx
            .send(CommandStarted::Move(command_id))
            .await;
    }
}
