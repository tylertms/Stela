//! Where the dispatcher's steps spend their time: relaxed counters a report
//! swaps out. Four counter reads and a few relaxed adds per step.
//!
//! The step running now ([`RUNNING`]) and the step watchdog's re-arm point
//! ([`WATCH`]) cost three relaxed stores and one compare per step: a
//! watchdog that sees no step return past its limit names the stuck one.
use core::sync::atomic::{AtomicPtr, AtomicU64, Ordering};

/// Time-stamp counter ticks and event counts since the last [`take`].
pub struct StepCounts {
    /// Entry to the context switch of steps that ran a task.
    pub select: AtomicU64,
    /// The task's run, from the switch to its return.
    pub body: AtomicU64,
    /// From the task's return to the step's exit.
    pub after: AtomicU64,
    /// Entry to exit of steps that found nothing to run.
    pub empty: AtomicU64,
    /// Steps that found nothing to run.
    pub empty_steps: AtomicU64,
    /// Job-slot scans for idle tasks.
    pub scans: AtomicU64,
    /// Queue lock acquisitions.
    pub locks: AtomicU64,
}

/// The process-wide counters.
pub static STEP_COUNTS: StepCounts = StepCounts {
    select: AtomicU64::new(0),
    body: AtomicU64::new(0),
    after: AtomicU64::new(0),
    empty: AtomicU64::new(0),
    empty_steps: AtomicU64::new(0),
    scans: AtomicU64::new(0),
    locks: AtomicU64::new(0),
};

/// One snapshot of [`StepCounts`], reset by [`take`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepSnapshot {
    pub select: u64,
    pub body: u64,
    pub after: u64,
    pub empty: u64,
    pub empty_steps: u64,
    pub scans: u64,
    pub locks: u64,
}

/// Swap every counter to zero and return what it held.
pub fn take() -> StepSnapshot {
    let take = |counter: &AtomicU64| counter.swap(0, Ordering::Relaxed);
    let counts = &STEP_COUNTS;
    StepSnapshot {
        select: take(&counts.select),
        body: take(&counts.body),
        after: take(&counts.after),
        empty: take(&counts.empty),
        empty_steps: take(&counts.empty_steps),
        scans: take(&counts.scans),
        locks: take(&counts.locks),
    }
}

pub(super) fn add(counter: &AtomicU64, value: u64) {
    counter.fetch_add(value, Ordering::Relaxed);
}

pub(super) fn now() -> u64 {
    // SAFETY: RDTSC has no memory effects; x86-64 always has it.
    unsafe { core::arch::x86_64::_rdtsc() }
}

/// The task step the dispatcher runs now: the time-stamp counter when its
/// switch began, 0 between steps, and its task and job sequence.
pub struct Running {
    pub began: AtomicU64,
    pub task: AtomicU64,
    pub sequence: AtomicU64,
}

/// The process-wide running step.
pub static RUNNING: Running = Running {
    began: AtomicU64::new(0),
    task: AtomicU64::new(0),
    sequence: AtomicU64::new(0),
};

/// A step watchdog's re-arm point: once a step returns at or past `due`,
/// a time-stamp counter value, the dispatcher calls `hook` with that time,
/// which sets the next. `u64::MAX` while no watchdog is armed.
pub struct Watch {
    due: AtomicU64,
    hook: AtomicPtr<()>,
}

/// The process-wide step watchdog's re-arm point.
pub static WATCH: Watch = Watch {
    due: AtomicU64::new(u64::MAX),
    hook: AtomicPtr::new(core::ptr::null_mut()),
};

impl Watch {
    /// Call `hook` once a step returns at or past `due`.
    pub fn arm(&self, hook: fn(u64), due: u64) {
        self.hook.store(hook as *mut (), Ordering::Relaxed);
        self.due.store(due, Ordering::Relaxed);
    }

    /// Move the re-arm point to `due`; the hook stays.
    pub fn due_at(&self, due: u64) {
        self.due.store(due, Ordering::Relaxed);
    }

    /// No more calls.
    pub fn disarm(&self) {
        self.due.store(u64::MAX, Ordering::Relaxed);
    }

    /// A step returned at `now`: the hook runs once `now` reached the
    /// re-arm point.
    pub fn returned(&self, now: u64) {
        if now < self.due.load(Ordering::Relaxed) {
            return;
        }
        let hook = self.hook.load(Ordering::Relaxed);
        if !hook.is_null() {
            // SAFETY: only `arm` stores this pointer, from a `fn(u64)`.
            let hook: fn(u64) = unsafe { core::mem::transmute::<*mut (), fn(u64)>(hook) };
            hook(now);
        }
    }
}

/// A step of `task`'s job `sequence` begins at `now`.
pub(super) fn begin(task: u64, sequence: u64, now: u64) {
    RUNNING.task.store(task, Ordering::Relaxed);
    RUNNING.sequence.store(sequence, Ordering::Relaxed);
    RUNNING.began.store(now, Ordering::Relaxed);
}

/// The running step returned at `now`.
pub(super) fn end(now: u64) {
    RUNNING.began.store(0, Ordering::Relaxed);
    WATCH.returned(now);
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::AtomicUsize;

    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static SEEN: AtomicU64 = AtomicU64::new(0);

    fn hook(now: u64) {
        CALLS.fetch_add(1, Ordering::Relaxed);
        SEEN.store(now, Ordering::Relaxed);
    }

    #[test]
    fn the_watch_hook_runs_only_once_a_return_reaches_its_point() {
        let watch = Watch {
            due: AtomicU64::new(u64::MAX),
            hook: AtomicPtr::new(core::ptr::null_mut()),
        };
        watch.returned(5);
        watch.arm(hook, 100);
        watch.returned(99);
        assert_eq!(CALLS.load(Ordering::Relaxed), 0);
        watch.returned(100);
        assert_eq!(
            (CALLS.load(Ordering::Relaxed), SEEN.load(Ordering::Relaxed)),
            (1, 100)
        );
        watch.due_at(200);
        watch.returned(150);
        watch.disarm();
        watch.returned(u64::MAX - 1);
        assert_eq!(CALLS.load(Ordering::Relaxed), 1);
    }
}
