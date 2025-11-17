//! Ref: https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf

use core::cmp::Ordering;

use defmt::info;
use embassy_futures::join::join3;
use embassy_rp::{
    gpio::{self, Level, Output},
    pio::{self, PioPin},
    pio_programs::clock_divider::calculate_pio_clock_divider,
    Peri,
};
use fixed::types::extra::U8;

use crate::util::OnDrop;

const PIO_TARGET_HZ: u32 =
    // 2 μs per cycle
    500_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct StepsPerSecond(pub u32);

impl StepsPerSecond {
    fn to_sleep_cyles_per_step(self) -> u32 {
        // TODO(aspen): division error?? probably doesn't matter?
        PIO_TARGET_HZ / self.0
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

    pub struct Axis<'d, T: pio::Instance, D: gpio::Pin, S: PioPin, const SM: usize> {
        pub dir: Peri<'d, D>,
        pub step: Peri<'d, S>,
        pub irq: pio::Irq<'d, T, SM>,
        pub sm: pio::StateMachine<'d, T, SM>,
    }

    pub struct Axes<
        'd,
        T: pio::Instance,
        XD: gpio::Pin,
        XS: PioPin,
        const XSM: usize,
        ZD: gpio::Pin,
        ZS: PioPin,
        const ZSM: usize,
        CD: gpio::Pin,
        CS: PioPin,
        const CSM: usize,
    > {
        pub x_axis: Axis<'d, T, XD, XS, XSM>,
        pub z_axis: Axis<'d, T, ZD, ZS, ZSM>,
        pub c_axis: Axis<'d, T, CD, CS, CSM>,
    }
}

pub struct Driver<'d, T: pio::Instance, const XSM: usize, const ZSM: usize, const CSM: usize> {
    pio: pio::Common<'d, T>,
    dir_pins: [Output<'d>; 3],
    irqs: (
        pio::Irq<'d, T, XSM>,
        pio::Irq<'d, T, ZSM>,
        pio::Irq<'d, T, CSM>,
    ),
    sms: (
        pio::StateMachine<'d, T, XSM>,
        pio::StateMachine<'d, T, ZSM>,
        pio::StateMachine<'d, T, CSM>,
    ),
}

impl<'d, T: pio::Instance, const XSM: usize, const ZSM: usize, const CSM: usize>
    Driver<'d, T, XSM, CSM, ZSM>
{
    pub fn new<XD: gpio::Pin, XS: PioPin, ZD: gpio::Pin, ZS: PioPin, CD: gpio::Pin, CS: PioPin>(
        mut pio: pio::Common<'d, T>,
        pins: config::Axes<'d, T, XD, XS, XSM, CD, CS, CSM, ZD, ZS, ZSM>,
        programs: &Programs<'d, T>,
    ) -> Self {
        let clock_divider = calculate_pio_clock_divider(PIO_TARGET_HZ);

        fn configure_pio<'d, T: pio::Instance, const SM: usize>(
            pio: &mut pio::Common<'d, T>,
            mut axis: config::Axis<'d, T, impl gpio::Pin, impl PioPin, SM>,
            clock_divider: fixed::FixedU32<U8>,
            program: &pio::LoadedProgram<'d, T>,
        ) -> (
            pio::StateMachine<'d, T, SM>,
            pio::Irq<'d, T, SM>,
            Output<'d>,
        ) {
            let pin = pio.make_pio_pin(axis.step);
            axis.sm.set_pin_dirs(pio::Direction::Out, &[&pin]);

            let mut cfg = pio::Config::default();
            cfg.set_set_pins(&[&pin]);
            cfg.clock_divider = clock_divider;
            cfg.use_program(&program, &[]);
            axis.sm.set_config(&cfg);
            axis.sm.set_enable(false);
            (axis.sm, axis.irq, Output::new(axis.dir, Level::Low))
        }

        let (xsm, xirq, xdir) =
            configure_pio(&mut pio, pins.x_axis, clock_divider, &programs.steps);
        let (zsm, zirq, zdir) =
            configure_pio(&mut pio, pins.z_axis, clock_divider, &programs.steps);
        let (csm, cirq, cdir) =
            configure_pio(&mut pio, pins.c_axis, clock_divider, &programs.steps);

        Self {
            pio,
            dir_pins: [xdir, zdir, cdir],
            irqs: (xirq, zirq, cirq),
            sms: (xsm, zsm, csm),
        }
    }

    pub async fn do_move(&mut self, steps: [i32; 3], speeds: [StepsPerSecond; 3]) {
        for (i, steps) in steps.into_iter().enumerate() {
            match steps.cmp(&0) {
                Ordering::Less => {
                    self.dir_pins[i].set_low();
                }
                Ordering::Equal => {}
                Ordering::Greater => {
                    self.dir_pins[i].set_high();
                }
            }
        }

        macro_rules! each_sm {
            (|$i:tt, $sm:ident|  $body:block ) => {{
                let $i = 0;
                let $sm = &mut self.sms.0;
                $body;
            }
            {
                let $i = 1;
                let $sm = &mut self.sms.1;
                $body;
            }
            {
                let $i = 2;
                let $sm = &mut self.sms.2;
                $body;
            }};
        }

        each_sm!(|i, sm| {
            let steps = steps[i].unsigned_abs();
            let sleeps = speeds[i].to_sleep_cyles_per_step();

            info!("axis={} steps={} sleeps={}", i, steps, sleeps);
            // corresponds to [pull block] instructions in steps.s
            sm.tx().wait_push(steps).await;
            sm.tx().wait_push(sleeps).await;
        });

        self.pio.apply_sm_batch(|batch| {
            each_sm!(|_, sm| {
                batch.restart(sm);
                batch.set_enable(sm, true);
            });
        });

        let drop = OnDrop::new(|| {
            each_sm!(|_, sm| {
                sm.clear_fifos();
                unsafe {
                    sm.exec_instr(
                        ::pio::InstructionOperands::JMP {
                            address: 0,
                            condition: ::pio::JmpCondition::Always,
                        }
                        .encode(),
                    );
                }
            });
        });

        info!("waiting on irqs");
        join3(self.irqs.0.wait(), self.irqs.1.wait(), self.irqs.2.wait()).await;
        info!("done");

        drop.defuse();

        self.pio.apply_sm_batch(|batch| {
            each_sm!(|_, sm| {
                batch.set_enable(sm, false);
            });
        });
        self.pio.apply_sm_batch(|batch| {
            each_sm!(|_, sm| {
                batch.restart(sm);
            });
        });
    }
}
