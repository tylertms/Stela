// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2006, Red Hat, Inc., Ingo Molnar
//! Each AP's time-stamp counter, brought into agreement with the BSP's
//! before the AP takes work, after Linux's `arch/x86/kernel/tsc_sync.c`.
//!
//! The engine and the drivers measure intervals as differences of counter
//! reads that may be taken on different cores: a deadline started on one
//! PE's core and checked on another's. Firmware may leave the cores'
//! counters apart. As `check_tsc_sync_source` and `check_tsc_sync_target`
//! do, the BSP and the AP take turns reading their counters under one lock
//! for a round (`stela_boot_core::tsc::sync::round_millis`), each noting
//! the most the shared last value went back. A clean round ends it.
//! Otherwise the AP moves its own counter by the warp through
//! `IA32_TSC_ADJUST`, forward when it saw the counter go back and so ran
//! behind, else back, and tries again. Linux finds the sync mechanism's own
//! overhead leaves at most a third round needed. An AP with no
//! `IA32_TSC_ADJUST`, as every AMD processor, moves the counter itself by
//! writing `IA32_TIME_STAMP_COUNTER`, so that every raw read on it agrees
//! too; Linux, which can fall back to another clock, marks such a counter
//! unstable instead. Where those writes do not converge in [`ROUNDS`], the AP
//! instead adds a software offset to the counter it reads in further
//! rounds, and once those agree counts its guest clock from a base moved by
//! that offset (`tsc::sync::corrected_base`). An AP that agrees in neither
//! way is refused. Every rendezvous point and turn waits at most a round
//! and [`POINT_MILLIS`] for the other core; a core that stops mid-round
//! leaves the other with a refusal instead of a spin with no end.
use crate::Result;
use crate::cpu::msr::{TSC_ADJUST, wrmsr};
use core::{
    arch::{
        asm,
        x86_64::{__cpuid_count, _rdtsc},
    },
    sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, Ordering},
};
use stela_boot_core::tsc::sync::{FIRST_POINT_MILLIS, POINT_MILLIS, Rendezvous, correction};
use stela_contracts::refuse;

/// Rounds of each correction, in the processor and in software.
const ROUNDS: u32 = 4;
/// IA32_TIME_STAMP_COUNTER.
const TIME_STAMP_COUNTER: u32 = 0x10;
const BSP: usize = 0;
const AP: usize = 1;

/// One AP's synchronization with the BSP.
pub(super) struct Sync {
    round_ticks: u64,
    /// The most a point or a turn waits for the other core.
    wait_ticks: u64,
    /// The most the AP waits at its first point (`FIRST_POINT_MILLIS`).
    first_ticks: u64,
    /// The rendezvous points; each needs both cores.
    rendezvous: Rendezvous,
    lock: AtomicBool,
    last: AtomicU64,
    /// The most the shared counter value went back, as the BSP and as the
    /// AP saw it.
    warps: [AtomicU64; 2],
    /// Set by the AP when it has decided, before the last rendezvous.
    finished: AtomicBool,
    /// What the AP added to its `IA32_TSC_ADJUST`, its software offset,
    /// and its rounds.
    adjusted: AtomicI64,
    offset: AtomicI64,
    rounds: AtomicU32,
}

/// How an AP's counter came to agree with the BSP's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Outcome {
    /// Added to its `IA32_TSC_ADJUST`, or to the counter itself where it has
    /// none.
    pub(super) adjusted: i64,
    /// Added in software to every value it reads: its counter base moves by
    /// this much.
    pub(super) offset: i64,
    pub(super) rounds: u32,
}

/// The counter, read after every earlier load and store, as Linux's
/// `rdtsc_ordered`.
fn ordered() -> u64 {
    // SAFETY: fences and RDTSC have no effect beyond ordering.
    unsafe { asm!("mfence", "lfence", options(nostack, preserves_flags)) };
    // SAFETY: RDTSC has no side effects; x86-64 always has it.
    unsafe { _rdtsc() }
}

impl Sync {
    /// A round of `round_millis` (`tsc::sync::round_millis`).
    pub(super) fn new(tsc_hz: u64, round_millis: u64) -> Self {
        Self {
            round_ticks: tsc_hz / 1000 * round_millis,
            wait_ticks: tsc_hz / 1000 * (round_millis + POINT_MILLIS),
            first_ticks: tsc_hz / 1000 * FIRST_POINT_MILLIS,
            rendezvous: Rendezvous::new(),
            lock: AtomicBool::new(false),
            last: AtomicU64::new(0),
            warps: [AtomicU64::new(0), AtomicU64::new(0)],
            finished: AtomicBool::new(false),
            adjusted: AtomicI64::new(0),
            offset: AtomicI64::new(0),
            rounds: AtomicU32::new(0),
        }
    }

    /// Wait until both cores reach rendezvous `point`, counted from one:
    /// whether they met within `wait` ticks.
    fn meet(&self, point: u32, wait: u64) -> bool {
        let started = ordered();
        self.rendezvous
            .meet(point, || ordered().wrapping_sub(started) >= wait)
    }

    /// Read the counter, moved by `offset`, in turn with the other core for
    /// a round, noting the most the shared value went back as `side` saw it:
    /// false when the other core held its turn past the wait.
    fn measure(&self, side: usize, offset: i64) -> bool {
        let started = ordered();
        loop {
            let waiting = ordered();
            while self
                .lock
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                if ordered().wrapping_sub(waiting) >= self.wait_ticks {
                    self.rendezvous.abandon();
                    return false;
                }
                core::hint::spin_loop();
            }
            let previous = self.last.load(Ordering::Relaxed);
            let now = ordered().wrapping_add(offset as u64);
            self.last.store(now, Ordering::Relaxed);
            self.lock.store(false, Ordering::Release);
            if now < previous {
                self.warps[side].fetch_max(previous - now, Ordering::Relaxed);
            }
            if now.wrapping_sub(offset as u64).wrapping_sub(started) >= self.round_ticks {
                return true;
            }
        }
    }

    /// The BSP's part: every round the AP asks for. False when the AP
    /// missed a point or held its turn past the wait.
    pub(super) fn source(&self) -> bool {
        let mut point = 0;
        loop {
            point += 1;
            if !self.meet(point, self.wait_ticks) || !self.measure(BSP, 0) {
                return false;
            }
            point += 1;
            if !self.meet(point, self.wait_ticks) {
                return false;
            }
            point += 1;
            if !self.meet(point, self.wait_ticks) {
                return false;
            }
            if self.finished.load(Ordering::Acquire) {
                return true;
            }
        }
    }

    /// The AP's part, on the AP: rounds until its counter agrees with the
    /// BSP's, correcting it between them in the processor while that
    /// converges, else in software. Returns the software offset.
    pub(super) fn target(&self) -> Result<i64> {
        // CPUID.(EAX=7,ECX=0):EBX[1] enumerates IA32_TSC_ADJUST.
        let adjustable = __cpuid_count(0, 0).eax >= 7 && __cpuid_count(7, 0).ebx & 1 << 1 != 0;
        // SAFETY: enumerated above, and read only then.
        let base = adjustable.then(|| unsafe { TSC_ADJUST.read() });
        let (mut adjusted, mut offset) = (0i64, 0i64);
        let missed = || refuse!("the BSP missed a point of this AP's TSC synchronization");
        let mut point = 0;
        for round in 1..=2 * ROUNDS {
            self.last.store(0, Ordering::Relaxed);
            for warp in &self.warps {
                warp.store(0, Ordering::Relaxed);
            }
            point += 1;
            // The BSP comes to the first point once it is done with the
            // APs before this one.
            let wait = if point == 1 {
                self.first_ticks
            } else {
                self.wait_ticks
            };
            if !self.meet(point, wait) || !self.measure(AP, offset) {
                return Err(missed());
            }
            point += 1;
            if !self.meet(point, self.wait_ticks) {
                return Err(missed());
            }
            let seen_by_bsp = self.warps[BSP].load(Ordering::Relaxed);
            let seen_here = self.warps[AP].load(Ordering::Relaxed);
            let outcome = if seen_by_bsp == 0 && seen_here == 0 {
                Some(Ok(offset))
            } else if round == 2 * ROUNDS {
                Some(Err(refuse!(
                    "AP TSC still differs from the BSP's after hardware and software correction"
                )))
            } else {
                let step = correction(seen_here, seen_by_bsp);
                match base {
                    Some(base) if round < ROUNDS => {
                        adjusted = adjusted.wrapping_add(step);
                        // SAFETY: enumerated; Linux writes the same register.
                        unsafe { TSC_ADJUST.write(base.wrapping_add(adjusted as u64)) };
                    }
                    None if round < ROUNDS => {
                        adjusted = adjusted.wrapping_add(step);
                        // SAFETY: IA32_TIME_STAMP_COUNTER is architectural on
                        // every processor with RDTSC and writable at CPL 0; this
                        // AP runs nothing else yet. The write's own time is
                        // what the next round measures and corrects.
                        unsafe { wrmsr(TIME_STAMP_COUNTER, ordered().wrapping_add(step as u64)) };
                    }
                    _ => offset = offset.wrapping_add(step),
                }
                None
            };
            if let Some(outcome) = outcome {
                self.adjusted.store(adjusted, Ordering::Relaxed);
                self.offset.store(offset, Ordering::Relaxed);
                self.rounds.store(round, Ordering::Relaxed);
                self.finished.store(true, Ordering::Release);
                point += 1;
                if !self.meet(point, self.wait_ticks) {
                    return Err(missed());
                }
                return outcome;
            }
            point += 1;
            if !self.meet(point, self.wait_ticks) {
                return Err(missed());
            }
        }
        unreachable!("the last round decides")
    }

    /// How the AP's counter came to agree, once [`Self::source`] returned.
    pub(super) fn outcome(&self) -> Outcome {
        Outcome {
            adjusted: self.adjusted.load(Ordering::Relaxed),
            offset: self.offset.load(Ordering::Relaxed),
            rounds: self.rounds.load(Ordering::Relaxed),
        }
    }
}
