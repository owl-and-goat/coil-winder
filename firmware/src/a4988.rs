//! Ref: https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf

use embassy_rp::{
    gpio::{Level, Output, Pin},
    Peri,
};
use embassy_time::Timer;

const HIGH_PULSE_WIDTH_US: u64 = 1;

pub enum Direction {
    Forwards,
    Backwards,
}

pub struct Driver<const AXES: usize> {
    direction: Output<'static>,
    step_axes: [Output<'static>; AXES],
}

pub struct Builder<const AXES: usize> {
    direction: Option<Output<'static>>,
    step_axes: Option<[Output<'static>; AXES]>,
}

impl<const AXES: usize> Builder<AXES> {
    pub fn new() -> Self {
        Self {
            direction: None,
            step_axes: None,
        }
    }

    pub fn direction_pin(mut self, peri: Peri<'static, impl Pin>) -> Self {
        self.direction = Some(Output::new(peri, Level::Low));
        self
    }

    pub fn step_axis_pins(mut self, outputs: [Output<'static>; AXES]) -> Self {
        self.step_axes = Some(outputs);
        self
    }

    pub fn build(mut self) -> Driver<AXES> {
        Driver {
            direction: self.direction.take().expect("direction_pin was not called"),
            step_axes: self
                .step_axes
                .take()
                .expect("step_axis_pins was not called"),
        }
    }
}

impl<const AXES: usize> Driver<AXES> {
    pub fn builder() -> Builder<AXES> {
        Builder::new()
    }

    fn set_direction(&mut self, direction: Direction) {
        self.direction.set_level(match direction {
            Direction::Forwards => Level::High,
            Direction::Backwards => Level::Low,
        });
    }

    pub async fn single_step(&mut self, axis: usize, direction: Direction) {
        self.set_direction(direction);
        self.step_axes[axis].set_high();
        Timer::after_micros(HIGH_PULSE_WIDTH_US).await;
        self.step_axes[axis].set_low();
        Timer::after_micros(1).await;
    }
}
