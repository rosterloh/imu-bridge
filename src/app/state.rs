use core::sync::atomic::{AtomicBool, Ordering};

pub struct ConnectionState {
    zenoh_ready: AtomicBool,
}

impl ConnectionState {
    pub const fn new() -> Self {
        Self {
            zenoh_ready: AtomicBool::new(false),
        }
    }

    pub fn zenoh_ready(&self) -> bool {
        self.zenoh_ready.load(Ordering::Acquire)
    }

    pub fn set_zenoh_ready(&self, ready: bool) {
        self.zenoh_ready.store(ready, Ordering::Release);
    }
}
