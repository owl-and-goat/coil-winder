use az::SaturatingCast;
use defmt::{info, Display2Format};
use embassy_futures::select::select;
use embassy_rp::pio;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex},
    channel,
};
use embassy_time::Timer;
use fixed_sqrt::FastSqrt;
use gcode::{Command, UCoord};
use movement::{
    units::{
        DegreesPerStep, INum, MicronsPerStep, MillimetersPerSecond, MillimetersPerSecondSquared,
    },
    StreamingPlan,
};

use crate::{
    driver::{self, StepsPerSecond},
    util::ArrayZipWith,
    CommandId, MotionStatusMsg, COMMAND_BUFFER_SIZE,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisUnit {
    Millimeters,
    Rotations,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Axis {
    pub microns_per_step: MicronsPerStep,
    pub degrees_per_step: DegreesPerStep,
    pub unit: AxisUnit,
    pub max_accel: MillimetersPerSecondSquared,
}

fn diff(coord1: UCoord, coord2: UCoord) -> INum {
    if coord1 > coord2 {
        (coord1 - coord2).saturating_cast()
    } else {
        let abs_diff: INum = (coord2 - coord1).saturating_cast();
        abs_diff.saturating_neg()
    }
}

impl StepsPerSecond {
    fn from_speeds(
        MillimetersPerSecond(millimeters_per_second): MillimetersPerSecond,
        MicronsPerStep(microns_per_step): MicronsPerStep,
    ) -> StepsPerSecond {
        let microns = millimeters_per_second * UCoord::from_num(1000);
        let steps = microns / microns_per_step;
        StepsPerSecond(steps.saturating_cast())
    }
}

const HOME_SPEED: MillimetersPerSecond = MillimetersPerSecond(UCoord::lit("120"));

const AXES: usize = 3;

pub struct MovementPlans<const AXES: usize>([StreamingPlan; AXES]);

impl<const AXES: usize> MovementPlans<AXES> {
    pub fn new(axes: &[Axis; AXES]) -> Self {
        Self(
            axes.each_ref()
                .map(|ax| StreamingPlan::builder().max_accel(ax.max_accel).build()),
        )
    }
}

pub struct State {
    is_homed: bool,
    /// Feedrate is always in terms of the C axis
    feedrate: MillimetersPerSecond,
    position: [UCoord; AXES],
    movement_plans: MovementPlans<AXES>,
    axes: [Axis; AXES],
}

impl State {
    pub fn new(axes: [Axis; AXES]) -> Self {
        Self {
            is_homed: false,
            feedrate: MillimetersPerSecond(UCoord::lit("1")),
            position: [UCoord::ZERO; AXES],
            movement_plans: MovementPlans::new(&axes),
            axes,
        }
    }

    pub async fn run<const XSM: usize, const CSM: usize, const ZSM: usize>(
        mut self,
        mut driver: driver::Driver<'static, impl pio::Instance, XSM, CSM, ZSM>,
        command_rx: channel::Receiver<
            'static,
            impl RawMutex,
            (CommandId, Command<{ AXES + 1 } /* for F */>),
            COMMAND_BUFFER_SIZE,
        >,
        status_tx: channel::Sender<
            'static,
            CriticalSectionRawMutex,
            MotionStatusMsg,
            COMMAND_BUFFER_SIZE,
        >,
    ) -> ! {
        loop {
            let next_action = select(, command_rx.receive().await);
            let (command_id, command) = command_rx.receive().await;
            info!("got command");
            match command {
                Command::Stop => continue,
                Command::Dwell(duration) => {
                    Timer::after_millis(duration.as_millis() as _).await;
                }
                Command::EnableAllSteppers => {
                    info!("enabling steppers");
                    driver.set_sleep(false).await
                }
                Command::DisableAllSteppers => {
                    info!("disabling steppers");
                    driver.set_sleep(true).await;

                    // If we disable the motors, we have to assume we don't know where we are
                    // anymore
                    self.is_homed = false;
                    for coord in self.position.each_mut() {
                        *coord = UCoord::ZERO;
                    }
                }
                Command::Home => {
                    let speed =
                        [HOME_SPEED; 2].zip_with([self.axes[0], self.axes[1]], |speed, axis| {
                            match axis.unit {
                                AxisUnit::Millimeters => {
                                    StepsPerSecond::from_speeds(speed, axis.microns_per_step)
                                }
                                AxisUnit::Rotations => {
                                    StepsPerSecond(0) /* can't home non-distance axes */
                                }
                            }
                        });
                    driver.home(speed).await;
                    self.is_homed = true;
                    for coord in self.position.each_mut() {
                        *coord = UCoord::ZERO;
                    }
                }
                Command::RapidMove(target_pos) | Command::LinearMove(target_pos) => {
                    if let Some(feedrate) = target_pos.0[3 /* feedrate is the last axis */] {
                        self.feedrate = MillimetersPerSecond(feedrate);
                    }

                    let target_pos = [target_pos.0[0], target_pos.0[1], target_pos.0[2]];

                    let mut dist =
                        self.position
                            .each_mut()
                            .zip_with(target_pos, |p1, p2| match p2 {
                                Some(target_pos) => {
                                    let res = diff(target_pos, *p1);
                                    // TODO(aspen): Don't update position until after moving, to
                                    // handle canceled moves
                                    *p1 = target_pos;
                                    res
                                }
                                None => INum::ZERO,
                            });
                    dist[2] = dist[2].saturating_neg();

                    let steps = dist.zip_with(self.axes, |dist, axis| -> i32 {
                        match axis.unit {
                            AxisUnit::Millimeters => {
                                let microns = dist * INum::from_num(1000);
                                let steps = microns / axis.microns_per_step.0.cast_signed();
                                steps.saturating_cast()
                            }
                            AxisUnit::Rotations => {
                                // Dist is in rotations
                                ((dist * 360) / axis.degrees_per_step.0.cast_signed())
                                    .saturating_cast()
                            }
                        }
                    });

                    let speed = if dist[2].is_zero() {
                        if dist[1].is_zero() {
                            [
                                StepsPerSecond::from_speeds(
                                    self.feedrate,
                                    self.axes[0].microns_per_step,
                                ),
                                StepsPerSecond(0),
                                StepsPerSecond(0),
                            ]
                        } else if dist[0].is_zero() {
                            [
                                StepsPerSecond(0),
                                StepsPerSecond::from_speeds(
                                    self.feedrate,
                                    self.axes[1].microns_per_step,
                                ),
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
                                StepsPerSecond::from_speeds(
                                    MillimetersPerSecond(x_fr),
                                    self.axes[0].microns_per_step,
                                ),
                                StepsPerSecond::from_speeds(
                                    MillimetersPerSecond(z_fr),
                                    self.axes[1].microns_per_step,
                                ),
                                StepsPerSecond(0),
                            ]
                        }
                    } else {
                        let c_speed = StepsPerSecond::from_speeds(
                            self.feedrate,
                            self.axes[2].microns_per_step,
                        );
                        let dur_s =
                            UCoord::from_num(steps[2].unsigned_abs()) / UCoord::from_num(c_speed.0);
                        [
                            StepsPerSecond(
                                (UCoord::from_num(steps[0].unsigned_abs()) / dur_s)
                                    .saturating_to_num(),
                            ),
                            StepsPerSecond(
                                (UCoord::from_num(steps[1].unsigned_abs()) / dur_s)
                                    .saturating_to_num(),
                            ),
                            c_speed,
                        ]
                    };

                    driver.do_move(steps, speed).await;
                }
                Command::GetCurrentPosition => {
                    let [x, z, c] = self.position;
                    let f = self.feedrate;
                    info!(
                        "X{} Z{} C{} F{}",
                        Display2Format(&x),
                        Display2Format(&z),
                        Display2Format(&c),
                        f
                    );
                }
                Command::Park(_) => {}
            }
            info!("command {} done", command_id);
            status_tx
                .send(MotionStatusMsg::CommandFinished(command_id))
                .await;
        }
    }
}
