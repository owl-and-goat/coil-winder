use core::fmt::{self, Display};
use core::time::Duration;

use fixed::{FixedU32, types::extra::U10};

// TODO(aspen): Consider making this signed after all, in case we want to rotate the spindle
// backwards(?)
pub type UCoord = FixedU32<U10>;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
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
    /// G28 [F<speed>]
    Home { f: Option<UCoord> },

    // M-codes
    /// M0
    Stop,
    /// M17
    EnableAllSteppers,
    /// M18
    DisableAllSteppers,
    /// M112
    ForceStop,
    /// M114
    GetCurrentPosition,
    /// M226
    Pause,
}

impl<const AXES: usize> Command<AXES> {
    pub fn display<'a>(&'a self, axis_labels: [char; AXES]) -> DisplayCommand<'a, AXES> {
        DisplayCommand {
            command: self,
            axis_labels,
        }
    }
}

pub struct DisplayCommand<'a, const AXES: usize> {
    command: &'a Command<AXES>,
    axis_labels: [char; AXES],
}

impl<const AXES: usize> Display for DisplayCommand<'_, AXES> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.command {
            Command::RapidMove(pos) => {
                write!(f, "G0")?;
                self.fmt_pos(f, pos)
            }
            Command::LinearMove(pos) => {
                write!(f, "G1")?;
                self.fmt_pos(f, pos)
            }
            Command::Dwell(duration) => write!(f, "G4 P{}", duration.as_millis()),
            Command::Park(None) => write!(f, "G27"),
            Command::Park(Some(pos)) => {
                write!(f, "G27")?;
                self.fmt_pos(f, pos)
            }
            Command::Home { f: None } => write!(f, "G28"),
            Command::Home { f: Some(feedrate) } => write!(f, "G28 F{feedrate}"),
            Command::Stop => write!(f, "M0"),
            Command::EnableAllSteppers => write!(f, "M17"),
            Command::DisableAllSteppers => write!(f, "M18"),
            Command::ForceStop => write!(f, "M112"),
            Command::GetCurrentPosition => write!(f, "M114"),
            Command::Pause => write!(f, "M226"),
        }
    }
}

impl<const AXES: usize> DisplayCommand<'_, AXES> {
    fn fmt_pos(&self, f: &mut fmt::Formatter<'_>, pos: &UPos<AXES>) -> fmt::Result {
        for (i, coord) in pos.0.iter().enumerate() {
            if let Some(c) = coord {
                write!(f, " {}{}", self.axis_labels[i], c)?;
            }
        }
        Ok(())
    }
}
