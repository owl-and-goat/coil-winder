use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::RawMutex, watch};

use crate::CommandId;

pub struct Cancellation<M: RawMutex, const N: usize> {
    watch: watch::Watch<M, CommandId, N>,
}

pub struct Sender<'a, M: RawMutex, const N: usize> {
    watch: watch::Sender<'a, M, CommandId, N>,
}

pub struct Receiver<'a, M: RawMutex, const N: usize> {
    watch: watch::Receiver<'a, M, CommandId, N>,
}

pub struct Cancelled(pub CommandId);

pub type Result<T> = core::result::Result<T, Cancelled>;

impl<M: RawMutex, const N: usize> Cancellation<M, N> {
    pub fn new() -> Self {
        Self {
            watch: watch::Watch::new(),
        }
    }

    pub fn sender(&self) -> Sender<'_, M, N> {
        Sender {
            watch: self.watch.sender(),
        }
    }

    pub fn receiver(&self) -> Option<Receiver<'_, M, N>> {
        Some(Receiver {
            watch: self.watch.receiver()?,
        })
    }
}

impl<'a, M: RawMutex, const N: usize> Sender<'a, M, N> {
    pub fn cancel(&self, command_id: CommandId) {
        self.watch.send(command_id);
    }
}

impl<'a, M: RawMutex, const N: usize> Receiver<'a, M, N> {
    pub async fn cancelled(&mut self) -> Cancelled {
        Cancelled(self.watch.changed().await)
    }

    pub async fn try_<F, R>(&mut self, f: F) -> Result<R>
    where
        F: Future<Output = R>,
    {
        match select(f, self.cancelled()).await {
            Either::First(res) => Ok(res),
            Either::Second(cancelled) => Err(cancelled),
        }
    }
}
