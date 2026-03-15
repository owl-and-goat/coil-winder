use core::sync::atomic::Ordering;

use embassy_futures::select::{select, Either};
use embassy_rp::gpio::{Level, Output};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use futures::future;
use portable_atomic_enum::atomic_enum;

#[derive(Clone, Copy, defmt::Format)]
pub(crate) enum Color {
    Amber,
    Green,
    Red,
}

impl Color {
    pub fn iter() -> impl Iterator<Item = Self> {
        [Self::Amber, Self::Green, Self::Red].into_iter()
    }
}

#[derive(Clone, Copy, defmt::Format, Default)]
pub struct PerColor<T> {
    pub amber: T,
    pub green: T,
    pub red: T,
}

impl<T> PerColor<T> {
    pub fn values(&self) -> impl Iterator<Item = &T> {
        [&self.amber, &self.green, &self.red].into_iter()
    }

    pub fn values_mut(&mut self) -> impl Iterator<Item = &mut T> {
        [&mut self.amber, &mut self.green, &mut self.red].into_iter()
    }

    pub fn iter(&self) -> impl Iterator<Item = (Color, &T)> {
        Color::iter().map(|c| (c, self.get(c)))
    }

    pub fn get(&self, color: Color) -> &T {
        match color {
            Color::Amber => &self.amber,
            Color::Green => &self.green,
            Color::Red => &self.red,
        }
    }

    pub fn get_mut(&mut self, color: Color) -> &mut T {
        match color {
            Color::Amber => &mut self.amber,
            Color::Green => &mut self.green,
            Color::Red => &mut self.red,
        }
    }
}

#[derive(Clone, Copy, defmt::Format, PartialEq, Eq)]
pub(crate) enum BlinkSpeed {
    Slow,
    Fast,
}

const BLINK_DELAY: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, defmt::Format, Default, PartialEq, Eq)]
pub(crate) enum Behavior {
    #[default]
    Off,
    On,
    Blink(BlinkSpeed),
}

impl Behavior {
    fn is_blink(&self) -> bool {
        matches!(self, Self::Blink(_))
    }

    fn blink_speed(&self) -> Option<BlinkSpeed> {
        match *self {
            Behavior::Blink(blink_speed) => Some(blink_speed),
            Behavior::Off | Behavior::On => None,
        }
    }
}

pub(crate) struct Control {
    signal: Signal<CriticalSectionRawMutex, Status>,
    status: AtomicStatus,
}

impl Control {
    pub(crate) fn new() -> Self {
        Control {
            signal: Signal::new(),
            status: AtomicStatus::from(Status::default()),
        }
    }

    pub(crate) fn set_status(&self, status: Status) {
        self.signal.signal(status);
        self.status.store(status, Ordering::Relaxed);
    }

    pub(crate) fn status(&self) -> Status {
        self.status.load(Ordering::Relaxed)
    }
}

#[atomic_enum]
#[derive(Clone, Copy, defmt::Format, Debug, Default)]
pub(crate) enum Status {
    #[default]
    Initializing,
    WifiConnecting,
    Ready,
    WifiError,
    ExecutingCommand,
}

pub(crate) struct Runner<'a> {
    control: &'a Control,
    pins: PerColor<Output<'static>>,
    status_behavior: fn(Status) -> PerColor<Behavior>,
    behavior: PerColor<Behavior>,
    blink: PerColor<u8>,
}

impl<'a> Runner<'a> {
    pub(crate) fn new(
        control: &'a Control,
        pins: PerColor<Output<'static>>,
        status_behavior: fn(Status) -> PerColor<Behavior>,
    ) -> Self {
        Self {
            control,
            pins,
            status_behavior,
            behavior: Default::default(),
            blink: Default::default(),
        }
    }

    pub(crate) fn control(&self) -> &'a Control {
        self.control
    }

    pub(crate) async fn run(mut self) -> ! {
        self.pins.values_mut().for_each(|p| p.set_low());

        loop {
            match select(
                self.control.signal.wait(),
                if self.is_blinking() {
                    future::Either::Right(Timer::after(BLINK_DELAY))
                } else {
                    future::Either::Left(future::pending())
                },
            )
            .await
            {
                Either::First(status) => {
                    self.behavior = (self.status_behavior)(status);
                    for (color, behavior) in self.behavior.iter() {
                        match behavior {
                            Behavior::Off => self.pins.get_mut(color).set_low(),
                            Behavior::On => self.pins.get_mut(color).set_high(),
                            Behavior::Blink(_) => { /* Handled in select loop */ }
                        }
                    }
                }
                Either::Second(()) => {
                    for color in Color::iter() {
                        if let Some(speed) = self.behavior.get(color).blink_speed() {
                            *self.blink.get_mut(color) = (self.blink.get(color) + 1) % 4;
                            let v = self.blink.get(color);
                            let on = match speed {
                                BlinkSpeed::Slow => v & 2 != 0,
                                BlinkSpeed::Fast => v & 1 != 0,
                            };
                            self.pins.amber.set_level(Level::from(on));
                        }
                    }
                }
            }
        }
    }

    fn is_blinking(&self) -> bool {
        self.behavior.values().any(|b| b.is_blink())
    }
}
