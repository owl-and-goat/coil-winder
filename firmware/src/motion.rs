use az::SaturatingCast;
use defmt::info;
use embassy_rp::pio;
use embassy_sync::{blocking_mutex::raw::RawMutex, channel};
use embassy_time::Timer;
use fixed::{types::extra::U10, FixedI32};
use fixed_sqrt::FastSqrt;
use gcode::{Command, UCoord};

use crate::{
    a4988::{self, StepsPerSecond},
    util::ArrayZipWith,
};

pub type ICoord = FixedI32<U10>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MicronsPerStep(pub ICoord);

fn diff(coord1: UCoord, coord2: UCoord) -> ICoord {
    if coord1 > coord2 {
        (coord1 - coord2).saturating_cast()
    } else {
        let abs_diff: ICoord = (coord2 - coord1).saturating_cast();
        abs_diff.saturating_neg()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct MillimetersPerSecond(pub UCoord);

impl MillimetersPerSecond {
    fn to_steps_per_second(self, microns_per_step: MicronsPerStep) -> StepsPerSecond {
        let microns = self.0 * UCoord::from_num(1000);
        let steps = microns / microns_per_step.0.unsigned_abs();
        StepsPerSecond(steps.saturating_cast())
    }
}

const AXES: usize = 3;

pub struct State {
    is_homed: bool,
    /// Feedrate is always in terms of the C axis
    feedrate: MillimetersPerSecond,
    position: [UCoord; 3],
    axis_speeds: [MicronsPerStep; 3],
}

impl State {
    pub fn new(axis_speeds: [MicronsPerStep; AXES]) -> Self {
        Self {
            is_homed: false,
            feedrate: MillimetersPerSecond(UCoord::lit("1")),
            position: [UCoord::ZERO; AXES],
            axis_speeds,
        }
    }

    pub async fn run<
        const BUFFER_SIZE: usize,
        const XSM: usize,
        const CSM: usize,
        const ZSM: usize,
    >(
        mut self,
        mut driver: a4988::Driver<'static, impl pio::Instance, XSM, CSM, ZSM>,
        command_rx: channel::Receiver<
            'static,
            impl RawMutex,
            Command<{ AXES + 1 } /* for F */>,
            BUFFER_SIZE,
        >,
    ) -> ! {
        loop {
            let command = command_rx.receive().await;
            match command {
                Command::Stop => continue,
                Command::Dwell(duration) => {
                    Timer::after_millis(duration.as_millis() as _).await;
                }
                Command::EnableAllSteppers => driver.set_sleep(false).await,
                Command::DisableAllSteppers => {
                    driver.set_sleep(true).await;

                    // If we disable the motors, we have to assume we don't know where we are
                    // anymore
                    self.is_homed = false;
                    for coord in self.position.each_mut() {
                        *coord = UCoord::ZERO;
                    }
                }
                Command::RapidMove(target_pos) | Command::LinearMove(target_pos) => {
                    if let Some(feedrate) = target_pos.0[3 /* feedrate is the last axis */] {
                        self.feedrate = MillimetersPerSecond(feedrate);
                    }

                    let target_pos = [target_pos.0[0], target_pos.0[1], target_pos.0[2]];

                    let dist = self
                        .position
                        .each_mut()
                        .zip_with(target_pos, |p1, p2| match p2 {
                            Some(target_pos) => {
                                let res = diff(target_pos, *p1);
                                // TODO(aspen): Don't update position until after moving, to handle
                                // canceled moves
                                *p1 = target_pos;
                                res
                            }
                            None => p1.saturating_cast(),
                        });

                    let steps = dist.zip_with(
                        self.axis_speeds,
                        |dist, MicronsPerStep(microns_per_step)| -> i32 {
                            let microns = dist * ICoord::from_num(1000);
                            let steps = microns / microns_per_step;
                            steps.saturating_cast()
                        },
                    );

                    let speed = if dist[2].is_zero() {
                        if dist[1].is_zero() {
                            [
                                self.feedrate.to_steps_per_second(self.axis_speeds[0]),
                                StepsPerSecond(0),
                                StepsPerSecond(0),
                            ]
                        } else if dist[0].is_zero() {
                            [
                                StepsPerSecond(0),
                                self.feedrate.to_steps_per_second(self.axis_speeds[1]),
                                StepsPerSecond(0),
                            ]
                        } else {
                            // if c isn't moving, base the feedrate calculation on a triangle
                            let x_fr = {
                                let z_over_x = dist[1].unsigned_abs() / dist[0].unsigned_abs();
                                self.feedrate.0
                                    / (z_over_x * z_over_x + UCoord::from_num(1)).fast_sqrt()
                            };
                            let z_fr = {
                                let x_over_z = dist[0].unsigned_abs() / dist[1].unsigned_abs();
                                self.feedrate.0
                                    / (x_over_z * x_over_z + UCoord::from_num(1)).fast_sqrt()
                            };
                            [
                                MillimetersPerSecond(x_fr).to_steps_per_second(self.axis_speeds[0]),
                                MillimetersPerSecond(z_fr).to_steps_per_second(self.axis_speeds[1]),
                                StepsPerSecond(0),
                            ]
                        }
                    } else {
                        let c_speed = self.feedrate.to_steps_per_second(self.axis_speeds[2]);
                        let dur_s = steps[2].unsigned_abs() / c_speed.0;
                        [
                            StepsPerSecond(steps[0].unsigned_abs() / dur_s),
                            StepsPerSecond(steps[1].unsigned_abs() / dur_s),
                            c_speed,
                        ]
                    };

                    info!(
                        "x_speed={} z_speed={} c_speed={}",
                        speed[0].0, speed[1].0, speed[2].0,
                    );

                    driver.do_move(steps, speed).await;
                }
                Command::Park(_) => {}
                Command::Home => {}
            }
        }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[test]
    fn four_minus_five() {
        let four = UCoord::from_str("4").unwrap();
        let five = UCoord::from_str("4").unwrap();
        let res = diff(four, five);
        assert_eq!(res, ICoord::from_str("-1").unwrap());
    }
}
