use az::{SaturatingCast, WrappingCast};
use embassy_rp::pio;
use embassy_sync::{blocking_mutex::raw::RawMutex, channel};
use embassy_time::Timer;
use fixed::{types::extra::U10, FixedI32};
use gcode::{Command, UCoord};

use crate::{a4988, util::ArrayZipWith};

type ICoord = FixedI32<U10>;

fn diff(coord1: UCoord, coord2: UCoord) -> ICoord {
    coord1.wrapping_sub(coord2).wrapping_cast()
}

pub struct State<const AXES: usize> {
    positions: [UCoord; AXES],
}

impl<const AXES: usize> State<AXES> {
    async fn run<const SM: usize, const BUFFER_SIZE: usize>(
        mut self,
        mut driver: a4988::Driver<'static, impl pio::Instance, SM>,
        command_rx: channel::Receiver<'static, impl RawMutex, Command<AXES>, BUFFER_SIZE>,
    ) {
        loop {
            let command = command_rx.receive().await;
            match command {
                Command::Stop => return,
                Command::Dwell(duration) => {
                    Timer::after_millis(duration.as_millis() as _).await;
                }
                Command::RapidMove(target_pos) => {
                    let dist = self.positions.zip_with(target_pos.0, |p1, p2| match p2 {
                        Some(target_pos) => diff(target_pos, p1),
                        None => p1.saturating_cast(),
                    });
                }
                Command::LinearMove(upos) => todo!(),
                Command::Park(upos) => todo!(),
                Command::Home => todo!(),
                Command::DisableAllSteppers => todo!(),
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
