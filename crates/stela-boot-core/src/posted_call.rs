//! A call a guest processor posts for the boot processor's Linux tasks to
//! perform, and the bounds of the poster's wait.
//!
//! A guest processor spinning in one exit for longer than about half a
//! second makes XNU panic. A posted call no task has taken by
//! [`WITHDRAW_MS`] is withdrawn: its poster answers the caller
//! `ETIMEDOUT` and reports it, and the task that meets the call later frees
//! it unperformed. A call a task already performs cannot be withdrawn, as
//! the ioctl writes the caller's memory: its poster reports it and waits on,
//! and a call still running at [`STOP_MS`] ends in a named stop rather than
//! a processor spinning forever ([`crate::bounded_spin`]).
use core::sync::atomic::{AtomicU32, Ordering};

/// How long a posted call may wait for a task to take it.
pub const WITHDRAW_MS: u64 = 300;
/// How long a posted call a task performs may run before the boot stops.
pub const STOP_MS: u64 = 10_000;

const POSTED: u32 = 0;
const TAKEN: u32 = 1;
const WITHDRAWN: u32 = 2;

/// Who has a posted call: its poster, a task that performs it, or nobody
/// once the poster withdrew it. Exactly one of [`Self::take`] and
/// [`Self::withdraw`] succeeds.
#[derive(Debug, Default)]
pub struct Claim(AtomicU32);

impl Claim {
    pub const fn new() -> Self {
        Self(AtomicU32::new(POSTED))
    }

    /// A task takes the call to perform it. False when its poster withdrew
    /// it: the poster no longer waits, and the task frees the call's storage
    /// without performing it.
    pub fn take(&self) -> bool {
        match self
            .0
            .compare_exchange(POSTED, TAKEN, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => true,
            Err(state) => state == TAKEN,
        }
    }

    /// The poster withdraws the call, its wait at [`WITHDRAW_MS`]. True when
    /// no task took it: the poster answers its caller without it and leaves
    /// the call's storage to the task that meets it. False when a task
    /// performs it: the poster waits for its result.
    pub fn withdraw(&self) -> bool {
        match self
            .0
            .compare_exchange(POSTED, WITHDRAWN, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => true,
            Err(state) => state == WITHDRAWN,
        }
    }

    /// Whether the poster withdrew the call.
    pub fn withdrawn(&self) -> bool {
        self.0.load(Ordering::Acquire) == WITHDRAWN
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_call_taken_first_cannot_be_withdrawn() {
        let claim = Claim::new();
        assert!(claim.take());
        assert!(!claim.withdraw());
        assert!(!claim.withdrawn());
        assert!(claim.take(), "taking again changes nothing");
    }

    #[test]
    fn a_call_withdrawn_first_is_never_performed() {
        let claim = Claim::new();
        assert!(claim.withdraw());
        assert!(claim.withdrawn());
        assert!(!claim.take());
        assert!(claim.withdraw(), "withdrawing again changes nothing");
    }

    #[test]
    fn the_bounds_stay_inside_xnus_half_second_and_stop_after_it() {
        const _: () = assert!(WITHDRAW_MS < 500 && STOP_MS > 500);
    }
}
