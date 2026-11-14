//! Naming the part of the host's service that held a processor too long.
//!
//! A PE that serves the host for seconds inside one exit ends the guest in
//! its watchdog's reset, and the periodic report then shows only the
//! longest time of each part. [`SlowParts`] decides when one part's time
//! is worth a line of its own, so that the line can carry where inside the
//! part the time went ([`Split`]) without a line for every slow turn.
use alloc::{format, string::String};
use core::sync::atomic::{AtomicU64, Ordering};

/// The longest time reported for each of `N` parts, in counter ticks.
pub struct SlowParts<const N: usize>([AtomicU64; N]);

impl<const N: usize> SlowParts<N> {
    pub const fn new() -> Self {
        Self([const { AtomicU64::new(0) }; N])
    }

    /// Whether `part` taking `ticks` is reported: when it took `floor` or
    /// more and at least twice what was last reported for it. A part is so
    /// reported a few times a boot at most, each time for a clearly longer
    /// turn, and a call below `floor` costs one comparison. Of two
    /// processors with the same time one reports.
    pub fn worth(&self, part: usize, ticks: u64, floor: u64) -> bool {
        if ticks < floor {
            return false;
        }
        let slot = &self.0[part];
        let mut reported = slot.load(Ordering::Relaxed);
        loop {
            if reported != 0 && ticks < reported.saturating_mul(2) {
                return false;
            }
            match slot.compare_exchange_weak(reported, ticks, Ordering::Relaxed, Ordering::Relaxed)
            {
                Ok(_) => return true,
                Err(now) => reported = now,
            }
        }
    }
}

impl<const N: usize> Default for SlowParts<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Why a service turn ended before its last part ([`Turn`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TurnEnd {
    /// A step's time ran out.
    Step,
    /// An interrupt came for the core.
    Interrupt,
    /// Guest work came for the PE whose core serves: an interrupt for it,
    /// or its own timer.
    Guest,
}

/// Which parts of one host service turn run. A turn runs its parts in
/// order from where the last one ended. It ends before a part once a
/// step's time is up, or, after at least one part ran, once an interrupt
/// or guest work came for the PE whose core serves: that PE answers no
/// interrupt and runs no timer of its own until it is back in its run
/// loop, so guest work waits at most the part that was running. The one
/// part every turn runs keeps the host served however often the guest
/// wants its PE.
#[derive(Clone, Copy, Debug)]
pub struct Turn {
    from: usize,
    started: u64,
    /// A step's time, in counter ticks.
    limit: Option<u64>,
    ran: bool,
    ended: Option<(usize, TurnEnd)>,
}

impl Turn {
    /// A turn from part `from`, started at counter value `started`; a step
    /// ends `limit` ticks after that.
    pub const fn new(from: usize, started: u64, limit: Option<u64>) -> Self {
        Self {
            from,
            started,
            limit,
            ran: false,
            ended: None,
        }
    }

    /// Whether `part` runs, asked in part order at counter value `now`,
    /// with an interrupt for the core and guest work for its PE as seen
    /// then. A part before `from` or after the turn ended does not.
    pub fn due(&mut self, part: usize, now: u64, interrupted: bool, guest: bool) -> bool {
        if self.ended.is_some() || part < self.from {
            return false;
        }
        let end = if self
            .limit
            .is_some_and(|limit| now.wrapping_sub(self.started) >= limit)
        {
            Some(TurnEnd::Step)
        } else if self.ran && interrupted {
            Some(TurnEnd::Interrupt)
        } else if self.ran && guest {
            Some(TurnEnd::Guest)
        } else {
            None
        };
        if let Some(end) = end {
            self.ended = Some((part, end));
            return false;
        }
        self.ran = true;
        true
    }

    /// Whether the turn may go on inside a part between two of its pieces:
    /// no step's time ran out, and no interrupt or guest work came.
    pub fn goes_on(&self, now: u64, interrupted: bool, guest: bool) -> bool {
        self.ended.is_none()
            && !interrupted
            && !guest
            && !self
                .limit
                .is_some_and(|limit| now.wrapping_sub(self.started) >= limit)
    }

    /// The part the next turn starts at and why this one ended early, or
    /// `None` when it ran to its last part.
    pub const fn ended(&self) -> Option<(usize, TurnEnd)> {
        self.ended
    }
}

/// What kept guest work for the serving PE waiting past the floor, as one
/// part ends ([`blame`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Late {
    /// Nothing waited for as long as the floor.
    No,
    /// The part's own time: the ticks of the wait it covered.
    Part(u64),
    /// Time before the part, in earlier parts or in the host's work around
    /// the turn: the whole wait, which the turn reports once.
    Turn(u64),
}

/// Who kept guest work waiting `waited` ticks when a part that took `took`
/// ticks ended, both in the same unit: the part when the wait it covered,
/// from the later of its start and the wait's, reached `floor`, else the
/// turn when the whole wait did. A part of microseconds is never named for
/// a wait that began before it.
pub fn blame(waited: Option<u64>, took: u64, floor: u64) -> Late {
    match waited {
        Some(waited) if waited >= floor => {
            let covered = waited.min(took);
            if covered >= floor {
                Late::Part(covered)
            } else {
                Late::Turn(waited)
            }
        }
        _ => Late::No,
    }
}

/// Where one turn of a part went: the counter ticks of each of its `N`
/// named pieces, added up as the turn runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Split<const N: usize> {
    names: [&'static str; N],
    ticks: [u64; N],
}

impl<const N: usize> Split<N> {
    pub const fn new(names: [&'static str; N]) -> Self {
        Self {
            names,
            ticks: [0; N],
        }
    }

    /// Forget the last turn.
    pub const fn clear(&mut self) {
        self.ticks = [0; N];
    }

    /// `piece` ran from counter value `from` to `to`.
    pub const fn add(&mut self, piece: usize, from: u64, to: u64) {
        self.ticks[piece] = self.ticks[piece].saturating_add(to.wrapping_sub(from));
    }

    /// ` name=…` for each piece in microseconds of a counter of `hz`, then
    /// ` rest=…`: what of the turn's `total` ticks no piece covers.
    pub fn line(&self, total: u64, hz: u64) -> String {
        let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(hz.max(1));
        let mut line = String::new();
        let mut rest = total;
        for (name, &ticks) in self.names.iter().zip(&self.ticks) {
            line.push_str(&format!(" {name}={}", micros(ticks)));
            rest = rest.saturating_sub(ticks);
        }
        line.push_str(&format!(" rest={}", micros(rest)));
        line
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run a turn of parts that take `took[part]` ticks each from part
    /// `from` at tick 0, with guest work arriving at tick `arrives`: the
    /// parts that ran, where the turn ended and when it returned.
    fn run_turn(
        took: &[u64],
        from: usize,
        limit: Option<u64>,
        arrives: u64,
    ) -> (alloc::vec::Vec<usize>, Option<(usize, TurnEnd)>, u64) {
        let mut turn = Turn::new(from, 0, limit);
        let mut now = 0;
        let mut ran = alloc::vec::Vec::new();
        for (part, &ticks) in took.iter().enumerate() {
            if turn.due(part, now, false, now >= arrives) {
                ran.push(part);
                now += ticks;
            }
        }
        (ran, turn.ended(), now)
    }

    #[test]
    fn guest_work_ends_a_turn_at_the_next_part() {
        // Six parts of 1 ms (ticks are microseconds) and a 5 s one, as the
        // USB part of the reported turns, with an IPI at 2.5 ms.
        let took = [1000, 1000, 1000, 5_000_000, 1000, 1000];
        let (ran, ended, returned) = run_turn(&took, 0, None, 2500);
        assert_eq!(ran, [0, 1, 2]);
        assert_eq!(ended, Some((3, TurnEnd::Guest)));
        // The IPI waited until the running part ended: 0.5 ms.
        assert_eq!(returned - 2500, 500);
        // The next turn starts at the part this one did not run.
        let (ran, ended, _) = run_turn(&took, 3, None, u64::MAX);
        assert_eq!(ran, [3, 4, 5]);
        assert_eq!(ended, None);
    }

    #[test]
    fn guest_work_waits_at_most_for_the_part_that_runs() {
        let took = [700, 3000, 200, 1500, 900, 2500];
        let longest = *took.iter().max().unwrap();
        for arrives in (0..10_000).step_by(37) {
            let (ran, _, returned) = run_turn(&took, 0, None, arrives);
            assert!(!ran.is_empty());
            if arrives < returned {
                assert!(
                    returned - arrives <= longest,
                    "work at {arrives} waited until {returned}"
                );
            }
        }
    }

    #[test]
    fn every_turn_runs_one_part_however_often_the_guest_wants_its_pe() {
        let took = [10, 10, 10];
        let mut from = 0;
        let mut served = alloc::vec::Vec::new();
        for _ in 0..3 {
            let (ran, ended, _) = run_turn(&took, from, None, 0);
            assert_eq!(ran.len(), 1, "one part each turn");
            served.extend(ran);
            from = ended.map_or(0, |(next, _)| next);
        }
        assert_eq!(served, [0, 1, 2]);
    }

    #[test]
    fn a_step_ends_on_its_time_and_an_interrupt_after_a_part() {
        let mut turn = Turn::new(0, 100, Some(50));
        assert!(turn.due(0, 100, true, false), "the first part runs");
        assert!(!turn.due(1, 120, true, false));
        assert_eq!(turn.ended(), Some((1, TurnEnd::Interrupt)));
        assert!(!turn.due(2, 130, false, false), "ended once");
        let mut turn = Turn::new(0, 100, Some(50));
        assert!(turn.due(0, 100, false, false));
        assert!(!turn.due(1, 150, false, false));
        assert_eq!(turn.ended(), Some((1, TurnEnd::Step)));
        // Parts before the turn's first are passed by.
        let mut turn = Turn::new(2, 0, None);
        assert!(!turn.due(1, 0, false, false));
        assert!(turn.due(2, 0, false, true));
        assert!(!turn.goes_on(1, false, true));
        assert!(turn.goes_on(1, false, false));
    }

    #[test]
    fn a_part_is_named_only_for_the_wait_its_own_time_covered() {
        // Nothing waited, or not for the floor.
        assert_eq!(blame(None, 1000, 50), Late::No);
        assert_eq!(blame(Some(49), 1000, 50), Late::No);
        // A part of 7 ticks at whose end work had waited 101_226: the
        // reported shape, the turn's and never the part's.
        assert_eq!(blame(Some(101_226), 7, 50), Late::Turn(101_226));
        assert_eq!(blame(Some(101_226), 0, 50), Late::Turn(101_226));
        // A long part that the work waited for: the part, for what of the
        // wait it covered.
        assert_eq!(blame(Some(60), 1000, 50), Late::Part(60));
        assert_eq!(blame(Some(5000), 300, 50), Late::Part(300));
        assert_eq!(blame(Some(5000), 49, 50), Late::Turn(5000));
    }

    #[test]
    fn a_part_below_the_floor_is_never_reported() {
        let slow = SlowParts::<2>::new();
        assert!(!slow.worth(0, 99, 100));
        assert!(!slow.worth(0, 0, 100));
        // And leaves the first slow turn its report.
        assert!(slow.worth(0, 100, 100));
    }

    #[test]
    fn a_slow_part_is_reported_once_until_one_takes_twice_as_long() {
        let slow = SlowParts::<2>::new();
        assert!(slow.worth(1, 130, 100));
        assert!(!slow.worth(1, 130, 100));
        assert!(!slow.worth(1, 259, 100));
        assert!(slow.worth(1, 260, 100));
        assert!(!slow.worth(1, 500, 100));
        // The seconds-long turn after many slow ones still gets its line.
        assert!(slow.worth(1, 5258, 100));
    }

    #[test]
    fn parts_are_reported_apart() {
        let slow = SlowParts::<2>::new();
        assert!(slow.worth(0, 1000, 100));
        assert!(slow.worth(1, 100, 100));
        assert!(!slow.worth(0, 1500, 100));
    }

    #[test]
    fn the_reports_of_a_part_are_few() {
        let slow = SlowParts::<1>::new();
        let reports = (100..100_000u64)
            .filter(|&ticks| slow.worth(0, ticks, 100))
            .count();
        assert_eq!(reports, 10);
    }

    #[test]
    fn a_split_names_each_piece_and_what_no_piece_covers() {
        let mut split = Split::new(["enter", "driver"]);
        split.add(0, 1000, 1010);
        split.add(1, 1010, 1510);
        split.add(0, 2000, 2020);
        // A counter of 1 MHz: ticks are microseconds.
        assert_eq!(
            split.line(5_000_000, 1_000_000),
            " enter=30 driver=500 rest=4999470"
        );
        split.clear();
        assert_eq!(split.line(7, 1_000_000), " enter=0 driver=0 rest=7");
    }

    #[test]
    fn a_split_survives_a_wrapping_counter_and_a_short_total() {
        let mut split = Split::new(["piece"]);
        split.add(0, u64::MAX - 1, 3);
        assert_eq!(split.line(2, 1_000_000), " piece=5 rest=0");
        assert_eq!(split.line(10, 0), " piece=5000000 rest=5000000");
    }
}
