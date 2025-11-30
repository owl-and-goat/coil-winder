use defmt::{Display2Format, Format};
use derive_more::{From, Into};
use fixed::{types::extra::U10, FixedI32, FixedU32};

pub type UNum = FixedU32<U10>;

pub type INum = FixedI32<U10>;

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, From, Into)]
pub struct UMillimeters(pub UNum);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, From, Into)]
pub struct IMillimeters(pub INum);

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, From, Into)]
pub struct Coord(pub UMillimeters);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, From, Into)]
pub struct MicronsPerStep(pub UNum);

impl Format for MicronsPerStep {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", Display2Format(&self.0))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, From, Into)]
pub struct DegreesPerStep(pub UNum);

impl Format for DegreesPerStep {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", Display2Format(&self.0))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MillimetersPerSecond(pub UNum);

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MillimetersPerSecondSquared(pub UNum);

impl Format for MillimetersPerSecond {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", Display2Format(&self.0))
    }
}
