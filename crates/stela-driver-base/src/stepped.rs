// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! Long driver work, such as a radio's bring-up or a controller's reset,
//! written once as an `async` function and run either to completion at
//! attach or a bounded step per poll. Nothing here schedules or wakes: a
//! [`Stepped`] task is polled with a no-op waker, and the driver code between
//! two awaits does at most one command or one register poll. [`until`] waits
//! for a deadline without spinning inside the poll, [`yield_now`] ends a step
//! after a command, and an [`Allowance`] ends a step once the waits it made
//! add up to a call's share. The work keeps every wait's lower bound: a
//! deadline that has not passed is never reported as passed.
//!
//! The task is heap-allocated once, when it starts, and pinned in that box;
//! dropping it mid-way drops the work's state at the await it stopped at.
use crate::Deadline;
use alloc::boxed::Box;
use core::{
    cell::Cell,
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
};

/// A future that is pending once, ending the current step.
pub fn yield_now() -> impl Future<Output = ()> {
    let mut yielded = false;
    core::future::poll_fn(move |_| {
        if yielded {
            Poll::Ready(())
        } else {
            yielded = true;
            Poll::Pending
        }
    })
}

/// A future that is ready once `deadline` passes; each step before that
/// ends at once.
pub fn until(deadline: Deadline) -> impl Future<Output = ()> {
    core::future::poll_fn(move |_| {
        if deadline.passed() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
}

/// One poll of `future`, as one step of its work.
pub fn poll_once<F: Future + ?Sized>(future: Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}

/// Run `future` to completion on this CPU, stepping it until it is ready;
/// the attach-time driver of the same work [`Stepped`] runs per poll. Each
/// pending step is a wait, so the [`crate::on_wait`] hook runs between steps.
pub fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    loop {
        if let Poll::Ready(output) = poll_once(future.as_mut()) {
            return output;
        }
        crate::waiting();
        core::hint::spin_loop();
    }
}

/// Work that runs a step per [`Self::step`], allocated once when it starts.
pub struct Stepped<'a, T> {
    future: Pin<Box<dyn Future<Output = T> + 'a>>,
}

impl<'a, T> Stepped<'a, T> {
    pub fn new(future: impl Future<Output = T> + 'a) -> Self {
        Self {
            future: Box::pin(future),
        }
    }

    /// One step: the output once the work is complete.
    pub fn step(&mut self) -> Poll<T> {
        poll_once(self.future.as_mut())
    }
}

impl<T> core::fmt::Debug for Stepped<'_, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("Stepped")
    }
}

/// What one step may still spend waiting, for work whose waits are minimum
/// delays the work makes itself rather than deadlines: each delay is
/// [`Self::spent`], and the step ends once the share is used up. The next
/// step [`Self::refill`]s it. A step then waits at most its share plus the
/// one delay that crossed it.
pub struct Allowance {
    share: u64,
    left: Cell<u64>,
}

impl Allowance {
    pub const fn new(microseconds: u64) -> Self {
        Self {
            share: microseconds,
            left: Cell::new(microseconds),
        }
    }

    /// A new step's share.
    pub fn refill(&self) {
        self.left.set(self.share);
    }

    /// Count a delay of `microseconds` just waited, ending the step if the
    /// share is used up.
    pub async fn spent(&self, microseconds: u64) {
        let left = self.left.get().saturating_sub(microseconds);
        self.left.set(left);
        if left == 0 {
            yield_now().await;
        }
    }
}

impl core::fmt::Debug for Allowance {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Allowance({} of {} us)", self.left.get(), self.share)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Clock, test_clock};
    use alloc::{rc::Rc, vec::Vec};
    use core::cell::RefCell;

    const MODEL: Clock = Clock {
        ticks_per_microsecond: 1,
    };

    /// Two commands of 5 ms each and a 30 ms wait: three steps of work at
    /// most one command or poll each, and the wait never ends early.
    async fn work(log: Rc<RefCell<Vec<u64>>>) -> u32 {
        for _ in 0..2 {
            test_clock::advance(MODEL, 5_000);
            log.borrow_mut().push(MODEL.now_us());
            yield_now().await;
        }
        let deadline = MODEL.deadline(30_000);
        until(deadline).await;
        log.borrow_mut().push(MODEL.now_us());
        7
    }

    #[test]
    fn stepped_work_runs_one_command_or_wait_check_a_step() {
        core::mem::forget(test_clock::virtualize());
        let log = Rc::new(RefCell::new(Vec::new()));
        let mut stepped = Stepped::new(work(log.clone()));
        assert_eq!(stepped.step(), Poll::Pending);
        assert_eq!(log.borrow().len(), 1);
        assert_eq!(stepped.step(), Poll::Pending);
        assert_eq!(log.borrow().len(), 2);
        let waited_from = MODEL.now_us();
        // The wait is pending, a step at a time, until it passes.
        let mut steps = 0;
        let output = loop {
            let start = MODEL.now_us();
            let poll = stepped.step();
            assert!(MODEL.now_us() - start < 1_000);
            steps += 1;
            if let Poll::Ready(output) = poll {
                break output;
            }
            test_clock::advance(MODEL, 4_000);
        };
        assert_eq!(output, 7);
        assert!(steps > 2);
        assert!(log.borrow()[2] - waited_from >= 30_000);
        // The same work runs to completion at once.
        let log = Rc::new(RefCell::new(Vec::new()));
        assert_eq!(block_on(work(log.clone())), 7);
        assert_eq!(log.borrow().len(), 3);
    }

    #[test]
    fn dropped_work_stops_at_its_await() {
        core::mem::forget(test_clock::virtualize());
        let log = Rc::new(RefCell::new(Vec::new()));
        let mut stepped = Stepped::new(work(log.clone()));
        assert_eq!(stepped.step(), Poll::Pending);
        drop(stepped);
        assert_eq!(log.borrow().len(), 1);
        assert_eq!(Rc::strong_count(&log), 1);
    }

    /// Delays of 15 ms against a 40 ms share: each step waits at most the
    /// share plus the delay that crossed it, and every delay is made.
    #[test]
    fn an_allowance_ends_a_step_once_its_share_is_waited() {
        let allowance = Allowance::new(40_000);
        let waited = Cell::new(0u64);
        let mut stepped = Stepped::new(async {
            for _ in 0..10 {
                waited.set(waited.get() + 15_000);
                allowance.spent(15_000).await;
            }
        });
        let mut steps = 0;
        loop {
            allowance.refill();
            let before = waited.get();
            let poll = stepped.step();
            assert!(waited.get() - before <= 40_000 + 15_000);
            steps += 1;
            if poll.is_ready() {
                break;
            }
        }
        assert_eq!(waited.get(), 150_000);
        assert_eq!(steps, 4);
    }
}
