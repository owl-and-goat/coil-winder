use az::SaturatingCast;
use embassy_rp::pio;
use embassy_sync::{blocking_mutex::raw::RawMutex, channel};
use embassy_time::Timer;
use fixed::{types::extra::U10, FixedI32};
use gcode::{Command, UCoord};

use crate::{
    a4988::{self, StepsPerSecond},
    util::ArrayZipWith,
};

type ICoord = FixedI32<U10>;

// 6 microns per step based on hardware configuration
const MICRONS_PER_STEP: ICoord = ICoord::lit("6");

fn diff(coord1: UCoord, coord2: UCoord) -> ICoord {
    if coord1 > coord2 {
        (coord1 - coord2).saturating_cast()
    } else {
        let abs_diff: ICoord = (coord2 - coord1).saturating_cast();
        abs_diff.saturating_neg()
    }
}

fn mm_to_steps(mm: ICoord) -> i32 {
    let microns = mm * ICoord::from_num(1000);
    let steps = microns / MICRONS_PER_STEP;
    steps.saturating_cast()
}

pub struct State<const AXES: usize> {
    is_homed: bool,
    position: [UCoord; AXES],
}

impl<const AXES: usize> State<AXES> {
    pub fn new() -> Self {
        Self {
            is_homed: false,
            position: [UCoord::ZERO; AXES],
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
        command_rx: channel::Receiver<'static, impl RawMutex, Command<AXES>, BUFFER_SIZE>,
    ) -> ! {
        loop {
            let command = command_rx.receive().await;
            match command {
                Command::Stop => continue,
                Command::Dwell(duration) => {
                    Timer::after_millis(duration.as_millis() as _).await;
                }
                Command::RapidMove(target_pos) | Command::LinearMove(target_pos) => {
                    let dist = self
                        .position
                        .each_mut()
                        .zip_with(target_pos.0, |p1, p2| match p2 {
                            Some(target_pos) => {
                                let res = diff(target_pos, *p1);
                                // TODO(aspen): Don't update position until after moving, to handle
                                // canceled moves
                                *p1 = target_pos;
                                res
                            }
                            None => p1.saturating_cast(),
                        });
                    const TMP_SPEED: StepsPerSecond = StepsPerSecond(2_000);
                    driver
                        .do_move(
                            [
                                mm_to_steps(dist[0]),
                                mm_to_steps(dist[1]),
                                mm_to_steps(dist[2]),
                            ],
                            [TMP_SPEED; 3],
                        )
                        .await;
                }
                Command::Park(_) => {}
                Command::Home => {}
                Command::DisableAllSteppers => {}
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
