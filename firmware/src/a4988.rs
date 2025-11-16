//! Ref: https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf

use core::cmp::Ordering;

use embassy_rp::{pio, pio_programs::clock_divider::calculate_pio_clock_divider, Peri};

use crate::util::OnDrop;

// const HIGH_PULSE_WIDTH_US: u64 = 1;

pub struct Program<'a, T: pio::Instance> {
    prg: pio::LoadedProgram<'a, T>,
}

impl<'a, T: pio::Instance> Program<'a, T> {
    /// Load the program into the given pio
    pub fn new(common: &mut pio::Common<'a, T>) -> Self {
        let prg = ::pio::pio_file!("src/steps.s");
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

pub struct Driver<'d, T: pio::Instance, const SM: usize> {
    irq: pio::Irq<'d, T, SM>,
    sm: pio::StateMachine<'d, T, SM>,
}

impl<'d, T: pio::Instance, const SM: usize> Driver<'d, T, SM> {
    pub fn new(
        pio: &mut pio::Common<'d, T>,
        mut sm: pio::StateMachine<'d, T, SM>,
        irq: pio::Irq<'d, T, SM>,
        x_step: Peri<'d, impl pio::PioPin>,
        z_step: Peri<'d, impl pio::PioPin>,
        program: &Program<'d, T>,
    ) -> Self {
        let (x_step, z_step) = (pio.make_pio_pin(x_step), pio.make_pio_pin(z_step));

        sm.set_pin_dirs(pio::Direction::Out, &[&x_step, &z_step]);
        let mut cfg = pio::Config::default();
        cfg.set_set_pins(&[&x_step, &z_step]);
        cfg.clock_divider = calculate_pio_clock_divider(
            40_000, // Hz
        );
        cfg.use_program(&program.prg, &[]);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { irq, sm }
    }

    pub async fn step(&mut self, steps: i32) {
        match steps.cmp(&0) {
            Ordering::Less => self.run((-steps) - 1).await,
            Ordering::Equal => {}
            Ordering::Greater => self.run(steps - 1).await,
        }
    }

    async fn run(&mut self, steps: i32) {
        self.sm.tx().wait_push(steps as u32).await;
        let drop = OnDrop::new(|| {
            self.sm.clear_fifos();
            unsafe {
                self.sm.exec_instr(
                    ::pio::InstructionOperands::JMP {
                        address: 0,
                        condition: ::pio::JmpCondition::Always,
                    }
                    .encode(),
                );
            }
        });
        self.irq.wait().await;
        drop.defuse();
    }
}
