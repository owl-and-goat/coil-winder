use core::time::Duration;

use fixed::{types::extra::U10, FixedU32};

// TODO(aspen): Consider making this signed after all, in case we want to rotate the spindle
// backwards(?)
pub type UCoord = FixedU32<U10>;

#[cfg_attr(test, derive(Debug, PartialEq, Eq))]
pub struct UPos<const AXES: usize>(pub [Option<UCoord>; AXES]);

impl<const AXES: usize> From<[Option<UCoord>; AXES]> for UPos<AXES> {
    fn from(coordinates: [Option<UCoord>; AXES]) -> Self {
        Self(coordinates)
    }
}

impl<const AXES: usize> From<[UCoord; AXES]> for UPos<AXES> {
    fn from(coordinates: [UCoord; AXES]) -> Self {
        Self(coordinates.map(Some))
    }
}

#[cfg_attr(test, derive(Debug, PartialEq, Eq))]
pub enum Command<const AXES: usize> {
    // G-codes
    /// G0
    RapidMove(UPos<AXES>),
    /// G1
    LinearMove(UPos<AXES>),
    /// G4
    Dwell(Duration),
    /// G27
    Park(Option<UPos<AXES>>),
    /// G28
    Home,

    // M-codes
    /// M0
    Stop,
    /// M18
    DisableAllSteppers,
}
