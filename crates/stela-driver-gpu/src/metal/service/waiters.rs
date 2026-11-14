//! The bookkeeping of the platform's completion waiters, one for each
//! device's service ([`super::SharedService::take_completion`]): which
//! waiter of a service is current, whether it waits on its timeline now, and
//! when a service whose waiter left by itself gets another.
//!
//! Each service's waiter is kept apart from every other service's: starting
//! the waiter of one GPU never ends the waiter of another. A waiter that
//! leaves by itself, its waits failing or its file never opening, is started
//! again from a timeline the service exports anew, at most [`RESTARTS`] times
//! in a boot for each service; past that its completions are found by the
//! platform's looks alone, which is reported once.
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

/// The services whose waiters are kept. A service past these gets no
/// waiter: its completions are found by the platform's looks alone.
pub const SERVICES: usize = 16;

/// How many times a service's waiter that left by itself is started again
/// in a boot.
pub const RESTARTS: u32 = 8;

/// What [`Waiters::due`] asks of the platform for a service.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Due {
    /// Nothing: its waiter waits, was replaced, or the GPUs stop.
    Nothing,
    /// Its waiter left by itself: offer its timeline again and start one,
    /// then report [`Waiters::restarted`].
    Restart,
    /// Its waiter left by itself once more than [`RESTARTS`] allow: report
    /// that its completions are found by looks alone. Answered once.
    GiveUp,
}

/// The waiters of every service.
pub struct Waiters {
    /// The last generation given to a waiter, of any service.
    last: AtomicU64,
    /// Each service's current waiter's generation, 0 for none.
    current: [AtomicU64; SERVICES],
    /// The generation of each service's waiter that waits on its timeline
    /// now, 0 for none.
    watching: [AtomicU64; SERVICES],
    /// Each service's current waiter left by itself.
    left: [AtomicBool; SERVICES],
    /// How often each service's waiter was started again.
    restarts: [AtomicU32; SERVICES],
    /// The GPUs stop: no waiter starts again.
    stopped: AtomicBool,
}

impl Default for Waiters {
    fn default() -> Self {
        Self::new()
    }
}

impl Waiters {
    pub const fn new() -> Self {
        Self {
            last: AtomicU64::new(0),
            current: [const { AtomicU64::new(0) }; SERVICES],
            watching: [const { AtomicU64::new(0) }; SERVICES],
            left: [const { AtomicBool::new(false) }; SERVICES],
            restarts: [const { AtomicU32::new(0) }; SERVICES],
            stopped: AtomicBool::new(false),
        }
    }

    /// A waiter of service `index` starts: its generation, which ends the
    /// service's earlier waiter. None for a service past [`SERVICES`], or
    /// once the GPUs stop.
    pub fn begin(&self, index: usize) -> Option<u64> {
        let current = self.current.get(index)?;
        if self.stopped.load(Ordering::Acquire) {
            return None;
        }
        let generation = self.last.fetch_add(1, Ordering::AcqRel) + 1;
        self.left[index].store(false, Ordering::Release);
        current.store(generation, Ordering::Release);
        Some(generation)
    }

    /// Whether the waiter `generation` of service `index` goes on waiting.
    pub fn stays(&self, index: usize, generation: u64) -> bool {
        self.current
            .get(index)
            .is_some_and(|current| current.load(Ordering::Acquire) == generation)
    }

    /// The waiter `generation` of service `index` waits on its timeline.
    pub fn waiting(&self, index: usize, generation: u64) {
        if let Some(watching) = self.watching.get(index) {
            watching.store(generation, Ordering::Release);
        }
    }

    /// The waiter `generation` of service `index` ended. Whether it left by
    /// itself, neither replaced nor stopped: the service then gets another
    /// ([`Self::due`]).
    pub fn ended(&self, index: usize, generation: u64) -> bool {
        let (Some(current), Some(watching)) = (self.current.get(index), self.watching.get(index))
        else {
            return false;
        };
        // A newer waiter may already have stored its own.
        let _ = watching.compare_exchange(generation, 0, Ordering::AcqRel, Ordering::Relaxed);
        let alone = current
            .compare_exchange(generation, 0, Ordering::AcqRel, Ordering::Relaxed)
            .is_ok();
        if alone {
            self.left[index].store(true, Ordering::Release);
        }
        alone
    }

    /// Every waiter leaves and none starts again: the GPUs stop.
    pub fn stop(&self) {
        self.stopped.store(true, Ordering::Release);
        for current in &self.current {
            current.store(0, Ordering::Release);
        }
    }

    /// Whether service `index`'s current waiter waits on its timeline: every
    /// fence of it that signals ends that wait.
    pub fn watching(&self, index: usize) -> bool {
        let Some(current) = self.current.get(index) else {
            return false;
        };
        let generation = current.load(Ordering::Acquire);
        generation != 0 && self.watching[index].load(Ordering::Acquire) == generation
    }

    /// Whether each of the first `count` services' waiters waits on its
    /// timeline, and there is one.
    pub fn all_watching(&self, count: usize) -> bool {
        count != 0 && (0..count).all(|index| self.watching(index))
    }

    /// What the platform does for service `index` now.
    pub fn due(&self, index: usize) -> Due {
        let (Some(left), Some(restarts)) = (self.left.get(index), self.restarts.get(index)) else {
            return Due::Nothing;
        };
        if self.stopped.load(Ordering::Acquire) || !left.load(Ordering::Acquire) {
            return Due::Nothing;
        }
        if restarts.load(Ordering::Acquire) < RESTARTS {
            return Due::Restart;
        }
        match left.swap(false, Ordering::AcqRel) {
            true => Due::GiveUp,
            false => Due::Nothing,
        }
    }

    /// Service `index`'s timeline was offered again for a new waiter, as
    /// [`Due::Restart`] asked: how many times so far.
    pub fn restarted(&self, index: usize) -> u32 {
        let Some(restarts) = self.restarts.get(index) else {
            return 0;
        };
        self.left[index].store(false, Ordering::Release);
        restarts.fetch_add(1, Ordering::AcqRel) + 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_service_keeps_its_own_waiter() {
        let waiters = Waiters::new();
        let first = waiters.begin(0).unwrap();
        waiters.waiting(0, first);
        let second = waiters.begin(1).unwrap();
        waiters.waiting(1, second);
        assert!(waiters.stays(0, first) && waiters.stays(1, second));
        assert!(waiters.all_watching(2));
        // A new timeline of service 0 replaces only service 0's waiter.
        let again = waiters.begin(0).unwrap();
        assert!(!waiters.stays(0, first) && waiters.stays(1, second));
        assert!(
            !waiters.ended(0, first),
            "a replaced waiter did not leave alone"
        );
        assert!(!waiters.watching(0) && waiters.watching(1));
        waiters.waiting(0, again);
        assert!(waiters.all_watching(2));
        assert_eq!(waiters.due(0), Due::Nothing);
    }

    #[test]
    fn the_stop_ends_every_waiter_and_starts_none() {
        let waiters = Waiters::new();
        let (first, second) = (waiters.begin(0).unwrap(), waiters.begin(1).unwrap());
        waiters.stop();
        assert!(!waiters.stays(0, first) && !waiters.stays(1, second));
        assert!(!waiters.ended(0, first) && !waiters.ended(1, second));
        assert_eq!(waiters.due(0), Due::Nothing);
        assert_eq!(waiters.begin(0), None);
        assert!(!waiters.all_watching(2));
    }

    #[test]
    fn a_waiter_that_leaves_by_itself_is_started_again_a_bounded_number_of_times() {
        let waiters = Waiters::new();
        for restart in 1..=RESTARTS {
            let generation = waiters.begin(0).unwrap();
            waiters.waiting(0, generation);
            assert!(waiters.ended(0, generation));
            assert!(!waiters.watching(0));
            assert_eq!(waiters.due(0), Due::Restart);
            // Until the timeline is offered again, the restart stays due.
            assert_eq!(waiters.due(0), Due::Restart);
            assert_eq!(waiters.restarted(0), restart);
            assert_eq!(waiters.due(0), Due::Nothing);
        }
        let generation = waiters.begin(0).unwrap();
        assert!(waiters.ended(0, generation));
        assert_eq!(waiters.due(0), Due::GiveUp);
        assert_eq!(waiters.due(0), Due::Nothing, "given up once");
    }

    #[test]
    fn a_service_past_the_kept_ones_gets_no_waiter() {
        let waiters = Waiters::new();
        assert_eq!(waiters.begin(SERVICES), None);
        assert!(!waiters.stays(SERVICES, 1));
        assert!(!waiters.ended(SERVICES, 1));
        assert!(!waiters.all_watching(SERVICES + 1));
        assert_eq!(waiters.due(SERVICES), Due::Nothing);
    }
}
