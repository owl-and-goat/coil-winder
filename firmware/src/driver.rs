//! Ref: https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf

use core::cmp::Ordering;

use defmt::info;
use embassy_futures::join::{join, join3};
use embassy_rp::{
    gpio::{self, Level, Pull},
    pio::{self, PioPin},
    pio_programs::clock_divider::calculate_pio_clock_divider,
    Peri,
};
use fixed::types::extra::U8;

const PIO_TARGET_HZ: u32 =
    // 2 μs per cycle
    500_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct StepsPerSecond(pub u32);

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
    steps: pio::LoadedProgram<'a, T>,
}

impl<'a, T: pio::Instance> Programs<'a, T> {
    /// Load the program into the given pio
    pub fn new(common: &mut pio::Common<'a, T>) -> Self {
        let home = common.load_program(&::pio::pio_file!("src/home.s").program);
        let steps = common.load_program(&::pio::pio_file!("src/steps.s").program);
        Self { home, steps }
    }
}

pub mod config {
    use super::*;
    use embassy_rp::{gpio, pio::PioPin};

    pub struct Axis<'d, T: pio::Instance, D: gpio::Pin, S: PioPin, ZL: PioPin, const SM: usize> {
        /// Direction pin
        pub dir: Peri<'d, D>,
        /// Step pin
        pub step: Peri<'d, S>,
        /// Zero limit switch input pin
        pub zero_limit: Option<Peri<'d, ZL>>,
        pub irq: pio::Irq<'d, T, SM>,
        pub sm: pio::StateMachine<'d, T, SM>,
    }

    pub struct Axes<
        'd,
        T: pio::Instance,
        XD: gpio::Pin,
        XS: PioPin,
        XZL: PioPin,
        const XSM: usize,
        ZD: gpio::Pin,
        ZS: PioPin,
        ZZL: PioPin,
        const ZSM: usize,
        CD: gpio::Pin,
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
    irq: pio::Irq<'d, T, SM>,
    dir_pin: gpio::Output<'d>,
    step_pin: pio::Pin<'d, T>,
    zero_limit_pin: Option<pio::Pin<'d, T>>,
}

impl<'d, T: pio::Instance, const SM: usize> Axis<'d, T, SM> {
    pub fn new(
        pio: &mut pio::Common<'d, T>,
        axis: config::Axis<'d, T, impl gpio::Pin, impl PioPin, impl PioPin, SM>,
    ) -> Self {
        let config::Axis {
            mut sm,
            step,
            zero_limit,
            dir,
            irq,
        } = axis;

        let step_pin = pio.make_pio_pin(step);
        sm.set_pin_dirs(pio::Direction::Out, &[&step_pin]);

        let zero_limit_pin = zero_limit.map(|zero_limit| {
            let mut zero_limit_pin = pio.make_pio_pin(zero_limit);
            zero_limit_pin.set_pull(Pull::Up);
            zero_limit_pin.set_schmitt(true);
            sm.set_pin_dirs(pio::Direction::In, &[&zero_limit_pin]);
            zero_limit_pin
        });

        sm.set_enable(false);

        Self {
            sm: sm,
            irq: irq,
            dir_pin: gpio::Output::new(dir, Level::Low),
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

        if let Some(zero_limit_pin) = &self.zero_limit_pin {
            cfg.set_in_pins(&[zero_limit_pin]);
        }

        cfg.clock_divider = clock_divider;
        cfg.use_program(&program, &[]);
        self.sm.set_config(&cfg);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConfiguredProgram {
    Home,
    Steps,
}

pub struct Driver<'d, T: pio::Instance, const XSM: usize, const ZSM: usize, const CSM: usize> {
    pio: pio::Common<'d, T>,
    sleep_pin: gpio::Output<'d>,
    axes: (Axis<'d, T, XSM>, Axis<'d, T, ZSM>, Axis<'d, T, CSM>),
    configured_program: Option<ConfiguredProgram>,
    programs: Programs<'d, T>,
    clock_divider: fixed::FixedU32<U8>,
}

macro_rules! axis {
    ($driver: expr, $i: expr, |$axis: ident| $body: expr) => {
        match $i {
            0 => {
                let $axis = &mut $driver.axes.0;
                $body
            }
            1 => {
                let $axis = &mut $driver.axes.1;
                $body
            }
            2 => {
                let $axis = &mut $driver.axes.2;
                $body
            }
            _ => unreachable!(),
        }
    };
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
        XD: gpio::Pin,
        XS: PioPin,
        XZL: PioPin,
        ZD: gpio::Pin,
        ZS: PioPin,
        ZZL: PioPin,
        CD: gpio::Pin,
        CS: PioPin,
        CZL: PioPin,
    >(
        mut pio: pio::Common<'d, T>,
        sleep_pin: Peri<'d, impl gpio::Pin>,
        axes: config::Axes<'d, T, XD, XS, XZL, XSM, ZD, ZS, ZZL, ZSM, CD, CS, CZL, CSM>,
        programs: Programs<'d, T>,
    ) -> Self {
        let clock_divider = calculate_pio_clock_divider(PIO_TARGET_HZ);

        let axes = (
            Axis::new(&mut pio, axes.x_axis),
            Axis::new(&mut pio, axes.z_axis),
            Axis::new(&mut pio, axes.c_axis),
        );

        let sleep_pin = gpio::Output::new(sleep_pin, Level::Low);

        Self {
            pio,
            sleep_pin,
            axes,
            configured_program: None,
            clock_divider,
            programs,
        }
    }

    fn configure_pio(&mut self, which_program: ConfiguredProgram) {
        if self.configured_program == Some(which_program) {
            return;
        }

        let program = match which_program {
            ConfiguredProgram::Home => &self.programs.home,
            ConfiguredProgram::Steps => &self.programs.steps,
        };

        each_axis!(self, |_i, axis| {
            axis.configure(self.clock_divider, program);
        });

        self.configured_program = Some(which_program);
    }

    pub async fn set_sleep(&mut self, sleep: bool) {
        self.sleep_pin
            .set_level(if sleep { Level::Low } else { Level::High });
    }

    pub async fn home(&mut self, speeds: impl IntoIterator<Item = StepsPerSecond>) {
        self.configure_pio(ConfiguredProgram::Home);

        let mut speeds = speeds.into_iter();
        each_axis!(self, |_, axis| {
            if axis.zero_limit_pin.is_some() {
                axis.dir_pin.set_low();
                axis.sm
                    .tx()
                    .wait_push(speeds.next().unwrap().to_sleep_cyles_per_step())
                    .await;
            }
        });

        self.pio.apply_sm_batch(|batch| {
            each_axis!(self, |_, axis| {
                if axis.zero_limit_pin.is_some() {
                    batch.restart(&mut axis.sm);
                    batch.set_enable(&mut axis.sm, true);
                }
            });
        });

        join(self.axes.0.irq.wait(), self.axes.1.irq.wait()).await;
    }

    pub async fn do_move(&mut self, steps: [i32; 3], speeds: [StepsPerSecond; 3]) {
        self.configure_pio(ConfiguredProgram::Steps);

        for (i, steps) in steps.into_iter().enumerate() {
            match steps.cmp(&0) {
                Ordering::Less => {
                    axis!(self, i, |axis| axis.dir_pin.set_low());
                }
                Ordering::Equal => {}
                Ordering::Greater => {
                    axis!(self, i, |axis| axis.dir_pin.set_high());
                }
            }
        }

        each_axis!(self, |i, axis| {
            let steps = steps[i].unsigned_abs();
            let sleeps = speeds[i].to_sleep_cyles_per_step();

            info!("axis={} steps={} sleeps={}", i, steps, sleeps);
            // corresponds to [pull block] instructions in steps.s
            axis.sm.tx().wait_push(steps).await;
            axis.sm.tx().wait_push(sleeps).await;
        });

        self.pio.apply_sm_batch(|batch| {
            each_axis!(self, |_, axis| {
                batch.restart(&mut axis.sm);
                batch.set_enable(&mut axis.sm, true);
            });
        });

        info!("waiting on irqs");
        join3(
            self.axes.0.irq.wait(),
            self.axes.1.irq.wait(),
            self.axes.2.irq.wait(),
        )
        .await;
        info!("done");

        self.pio.apply_sm_batch(|batch| {
            each_axis!(self, |_, axis| {
                batch.set_enable(&mut axis.sm, false);
            });
        });
        self.pio.apply_sm_batch(|batch| {
            each_axis!(self, |_, axis| {
                batch.restart(&mut axis.sm);
            });
        });
    }
}
