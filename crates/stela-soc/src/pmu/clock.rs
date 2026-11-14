//! One declared PMIC upcount owner shared by its SPMI and SMC interfaces.
use super::RTC_HZ;
use crate::Result;
use crate::counter::FREQUENCY_HZ;
use alloc::rc::Rc;
use core::cell::Cell;
use stela_contracts::refuse;
use stela_soc_data::pmu::WallClock;

/// The upcount's qualified range: 2^32 seconds at 32768 Hz.
pub const LIMIT: u64 = 1 << 47;

/// Declared emulated resource origin, not an observed Apple reset value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtcEpoch {
    pub counter: u64,
    pub ticks: u64,
}

/// What the machine state keeps of the upcount, as a Mac's PMIC keeps
/// counting while the machine is off: the upcount at one moment and the
/// host's Unix seconds then, `None` when no boot that wrote it knew them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtcAnchor {
    pub ticks: u64,
    pub unix_seconds: Option<u64>,
}

impl RtcEpoch {
    /// The origin of this boot's upcount at counter `now`. Without an
    /// anchor (a new machine, or one whose state an older Stela wrote) the
    /// upcount starts at zero, a PMIC that lost power. With one it goes on
    /// from the anchor's ticks by the wall seconds since the anchor, carried
    /// to `now`; never backwards: a wall time before the anchor's, or no wall
    /// time on either side, continues from the anchor's ticks. An anchor at
    /// or past the 47-bit limit, or one the elapsed time carries there,
    /// also starts at zero: the upcount's wrap is unqualified.
    pub fn anchored(now: u64, wall: Option<WallClock>, anchor: Option<RtcAnchor>) -> Self {
        let cold = Self {
            counter: now,
            ticks: 0,
        };
        let Some(anchor) = anchor else {
            return cold;
        };
        let elapsed = match (wall, anchor.unix_seconds) {
            (Some(wall), Some(then)) => {
                // Carry the wall time's ticks from its counter to `now`.
                let rate = |counter: u64| {
                    i128::from(counter) * i128::from(RTC_HZ) / i128::from(FREQUENCY_HZ)
                };
                let seconds = i128::from(wall.unix_seconds) - i128::from(then);
                (seconds * i128::from(RTC_HZ) + rate(now) - rate(wall.counter)).max(0)
            }
            _ => 0,
        };
        match u64::try_from(i128::from(anchor.ticks) + elapsed) {
            Ok(ticks) if ticks < LIMIT => Self {
                counter: now,
                ticks,
            },
            _ => cold,
        }
    }
}

impl RtcAnchor {
    /// The anchor of the upcount `ticks` at counter `counter`, in a boot
    /// whose wall time was `wall` and whose anchor was `loaded`. The Unix
    /// seconds follow the same counter as the ticks, so a counter that lags
    /// the host's time lags both alike and the next boot's wall time makes
    /// up the difference. Without a wall time they follow the loaded
    /// anchor's by the ticks since it, else stay unknown.
    pub fn at(
        counter: u64,
        ticks: u64,
        wall: Option<WallClock>,
        loaded: Option<RtcAnchor>,
    ) -> Self {
        let unix_seconds = match wall {
            Some(wall) => {
                Some(wall.unix_seconds + counter.saturating_sub(wall.counter) / FREQUENCY_HZ)
            }
            None => loaded.and_then(|loaded| {
                Some(loaded.unix_seconds? + ticks.saturating_sub(loaded.ticks) / RTC_HZ)
            }),
        };
        Self {
            ticks,
            unix_seconds,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Sample {
    counter: u64,
    ticks: u64,
}
#[derive(Debug)]
struct Clock {
    epoch: RtcEpoch,
    sample: Cell<Option<Sample>>,
    /// The last counter refused for preceding the publication, with it.
    regression: Cell<Option<(u64, u64)>>,
}

/// Ownership of one PMIC oscillator/epoch. Clones refer to the same
/// publication; no interface owns another wall clock or a default sample.
#[derive(Debug, Clone)]
pub struct RtcClock(Rc<Clock>);
impl RtcClock {
    pub fn new(epoch: RtcEpoch) -> Result<Self> {
        if epoch.ticks >= LIMIT {
            return Err(refuse!(
                "PMU RTC epoch exceeds the qualified 47-bit upcount"
            ));
        }
        let remaining = counter_ticks(LIMIT - epoch.ticks);
        if epoch.counter.checked_add(remaining).is_none() {
            return Err(refuse!(
                "PMU RTC epoch cannot represent its qualified counter horizon"
            ));
        }
        Ok(Self(Rc::new(Clock {
            epoch,
            sample: Cell::new(None),
            regression: Cell::new(None),
        })))
    }

    /// Check an Access.now without publishing or changing any owner.
    pub fn check(&self, now: u64) -> Result<u64> {
        if let Some(sample) = self.0.sample.get()
            && now < sample.counter
        {
            self.0.regression.set(Some((now, sample.counter)));
            return Err(refuse!("PMU RTC counter publication moved backwards"));
        }
        let elapsed = now
            .checked_sub(self.0.epoch.counter)
            .ok_or(refuse!("PMU RTC counter precedes its declared epoch"))?;
        let elapsed = u128::from(elapsed) * u128::from(RTC_HZ) / u128::from(FREQUENCY_HZ);
        let ticks = u128::from(self.0.epoch.ticks) + elapsed;
        if ticks >= u128::from(LIMIT) {
            // 2^32 seconds of upcount: a count this far past the epoch is
            // one no boot reaches, so the board's time itself is wrong.
            let epoch = self.0.epoch.counter;
            return Err(refuse!(
                "PMU RTC upcount past its 47-bit range: the count is over 136 years after the epoch at {epoch:#x}; upcount/alarm wrap behavior is unqualified",
                epoch
            ));
        }
        Ok(ticks as u64)
    }

    /// Board service boundary publication, before ASC receive/poll effects.
    /// Refusal preserves the previous sample. This is model time ownership,
    /// never permission to execute on a host worker.
    pub fn publish(&self, now: u64) -> Result<()> {
        let ticks = self.check(now)?;
        self.0.sample.set(Some(Sample {
            counter: now,
            ticks,
        }));
        Ok(())
    }

    /// The upcount at counter `now`, without a publication and without
    /// recording a regression: `None` before the epoch or past the limit.
    pub fn ticks_at(&self, now: u64) -> Option<u64> {
        let elapsed = now.checked_sub(self.0.epoch.counter)?;
        let elapsed = u128::from(elapsed) * u128::from(RTC_HZ) / u128::from(FREQUENCY_HZ);
        let ticks = u128::from(self.0.epoch.ticks) + elapsed;
        (ticks < u128::from(LIMIT)).then_some(ticks as u64)
    }

    /// The last counter [`Self::check`] refused for moving backwards, and
    /// the publication it preceded: a diagnostic of the stop it causes.
    pub fn regression(&self) -> Option<(u64, u64)> {
        self.0.regression.get()
    }

    /// SMC reads the same published upcount. Missing publication must stop the
    /// service; it cannot be replaced with a successful zero-valued read.
    pub fn published_ticks(&self) -> Result<u64> {
        self.0
            .sample
            .get()
            .map(|sample| sample.ticks)
            .ok_or(refuse!("PMU RTC has no actual counter publication"))
    }

    /// First counter at which the oscillator reaches a tick in this epoch.
    /// None denotes a tick already elapsed before the supplied epoch.
    pub(super) fn counter_for(&self, ticks: u64) -> Option<u64> {
        if ticks >= LIMIT {
            return None;
        }
        let elapsed = ticks.checked_sub(self.0.epoch.ticks)?;
        self.0.epoch.counter.checked_add(counter_ticks(elapsed))
    }
}

fn counter_ticks(ticks: u64) -> u64 {
    (u128::from(ticks) * u128::from(FREQUENCY_HZ)).div_ceil(u128::from(RTC_HZ)) as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    const UNIX: u64 = 1_792_000_000;

    fn wall(unix_seconds: u64, counter: u64) -> Option<WallClock> {
        Some(WallClock {
            unix_seconds,
            counter,
        })
    }

    fn anchor(ticks: u64, unix_seconds: Option<u64>) -> Option<RtcAnchor> {
        Some(RtcAnchor {
            ticks,
            unix_seconds,
        })
    }

    #[test]
    fn without_an_anchor_the_upcount_starts_at_zero_and_its_anchor_follows() {
        let now = 7 * FREQUENCY_HZ;
        let cold = RtcEpoch {
            counter: now,
            ticks: 0,
        };
        // State an older Stela wrote holds no anchor: the old cold origin,
        // with or without a wall time.
        assert_eq!(RtcEpoch::anchored(now, wall(UNIX, now), None), cold);
        assert_eq!(RtcEpoch::anchored(now, None, None), cold);
        // The anchor kept 10 s into that boot: ticks since zero, wall then.
        let clock = RtcClock::new(cold).unwrap();
        let later = now + 10 * FREQUENCY_HZ;
        let ticks = clock.ticks_at(later).unwrap();
        assert_eq!(ticks, 10 * RTC_HZ);
        assert_eq!(
            RtcAnchor::at(later, ticks, wall(UNIX, now), None),
            RtcAnchor {
                ticks: 10 * RTC_HZ,
                unix_seconds: Some(UNIX + 10),
            }
        );
        // Without a wall time and no anchor, its wall seconds stay unknown.
        assert_eq!(RtcAnchor::at(later, ticks, None, None).unix_seconds, None);
        assert_eq!(clock.ticks_at(now - 1), None);
    }

    #[test]
    fn an_anchor_and_a_later_wall_time_continue_the_upcount() {
        let ticks = 1_000 * RTC_HZ;
        // Off for an hour, booted with a smaller counter.
        let now = 3 * FREQUENCY_HZ;
        let epoch = RtcEpoch::anchored(now, wall(UNIX + 3600, now), anchor(ticks, Some(UNIX)));
        assert_eq!(epoch.counter, now);
        assert_eq!(epoch.ticks, ticks + 3600 * RTC_HZ);
        // A wall time sampled at another counter is carried to `now`.
        let carried = RtcEpoch::anchored(
            now,
            wall(UNIX + 3600, 5 * FREQUENCY_HZ),
            anchor(ticks, Some(UNIX)),
        );
        assert_eq!(carried.ticks, ticks + 3598 * RTC_HZ);
        // The anchor of that boot 60 s on, then the next boot a day later.
        let clock = RtcClock::new(epoch).unwrap();
        let later = now + 60 * FREQUENCY_HZ;
        let kept = RtcAnchor::at(
            later,
            clock.ticks_at(later).unwrap(),
            wall(UNIX + 3600, now),
            anchor(ticks, Some(UNIX)),
        );
        assert_eq!(kept.ticks, ticks + 3660 * RTC_HZ);
        assert_eq!(kept.unix_seconds, Some(UNIX + 3660));
        let next = RtcEpoch::anchored(0, wall(UNIX + 3660 + 86_400, 0), Some(kept));
        assert_eq!(next.ticks, ticks + (3660 + 86_400) * RTC_HZ);
        // A lost last write costs nothing with a wall time: the earlier
        // anchor and the same wall time give the same upcount.
        assert_eq!(
            RtcEpoch::anchored(0, wall(UNIX + 3660 + 86_400, 0), anchor(ticks, Some(UNIX))),
            next
        );
    }

    #[test]
    fn an_earlier_or_missing_wall_time_never_moves_the_upcount_back() {
        let ticks = 50_000 * RTC_HZ + 123;
        let continued = RtcEpoch { counter: 9, ticks };
        // The host's clock was set back a day.
        assert_eq!(
            RtcEpoch::anchored(9, wall(UNIX - 86_400, 9), anchor(ticks, Some(UNIX))),
            continued
        );
        // Less than the carry from the wall time's counter back to `now`.
        assert_eq!(
            RtcEpoch::anchored(0, wall(UNIX, FREQUENCY_HZ), anchor(ticks, Some(UNIX))),
            RtcEpoch { counter: 0, ticks }
        );
        // No wall time now, or none when the anchor was kept.
        assert_eq!(
            RtcEpoch::anchored(9, None, anchor(ticks, Some(UNIX))),
            continued
        );
        assert_eq!(
            RtcEpoch::anchored(9, wall(UNIX, 9), anchor(ticks, None)),
            continued
        );
        // A boot without a wall time keeps the anchor's wall seconds moving
        // with its ticks, so the next boot with one does not count it twice.
        let kept = RtcAnchor::at(0, ticks + 100 * RTC_HZ, None, anchor(ticks, Some(UNIX)));
        assert_eq!(kept.unix_seconds, Some(UNIX + 100));
        assert_eq!(
            RtcEpoch::anchored(0, wall(UNIX + 100, 0), Some(kept)).ticks,
            ticks + 100 * RTC_HZ
        );
    }

    #[test]
    fn the_47_bit_limit_starts_a_lost_power_upcount_instead_of_refusing() {
        let cold = RtcEpoch {
            counter: 5,
            ticks: 0,
        };
        // The last tick still continues and its clock is representable.
        let last = RtcEpoch::anchored(5, None, anchor(LIMIT - 1, Some(UNIX)));
        assert_eq!(last.ticks, LIMIT - 1);
        assert!(RtcClock::new(last).is_ok());
        // An anchor at the limit, or carried to it by the wall time.
        assert_eq!(RtcEpoch::anchored(5, None, anchor(LIMIT, Some(UNIX))), cold);
        assert_eq!(
            RtcEpoch::anchored(5, wall(UNIX + 1, 5), anchor(LIMIT - RTC_HZ, Some(UNIX))),
            cold
        );
        assert_eq!(
            RtcEpoch::anchored(5, wall(u64::MAX, 5), anchor(1, Some(0))),
            cold
        );
        // Past the limit there is no upcount to keep.
        let clock = RtcClock::new(last).unwrap();
        assert_eq!(clock.ticks_at(5 + FREQUENCY_HZ), None);
    }
    #[test]
    fn explicit_epoch_is_shared_and_has_no_implicit_sample() {
        let clock = RtcClock::new(RtcEpoch {
            counter: 240,
            ticks: 1234,
        })
        .unwrap();
        let smc = clock.clone();
        assert!(smc.published_ticks().is_err());
        clock.publish(240 + FREQUENCY_HZ).unwrap();
        assert_eq!(smc.published_ticks(), Ok(1234 + RTC_HZ));
        assert!(clock.publish(239).is_err());
        assert!(clock.publish(240).is_err());
        assert_eq!(clock.regression(), Some((240, 240 + FREQUENCY_HZ)));
        assert_eq!(smc.published_ticks(), Ok(1234 + RTC_HZ));
        smc.publish(240 + FREQUENCY_HZ).unwrap();
        assert_eq!(clock.published_ticks(), Ok(1234 + RTC_HZ));
    }
    #[test]
    fn deadlines_round_up_and_tick_epoch_overflow_refuses_atomically() {
        let clock = RtcClock::new(RtcEpoch {
            counter: 17,
            ticks: LIMIT - 2,
        })
        .unwrap();
        assert_eq!(clock.counter_for(LIMIT - 3), None);
        assert_eq!(clock.counter_for(LIMIT - 2), Some(17));
        let deadline = clock.counter_for(LIMIT - 1).unwrap();
        assert_eq!(clock.check(deadline - 1), Ok(LIMIT - 2));
        assert_eq!(clock.check(deadline), Ok(LIMIT - 1));
        clock.publish(deadline).unwrap();
        assert!(clock.publish(17 + counter_ticks(2)).is_err());
        assert_eq!(clock.published_ticks(), Ok(LIMIT - 1));
        assert_eq!(clock.counter_for(LIMIT), None);
        assert!(
            RtcClock::new(RtcEpoch {
                counter: 0,
                ticks: LIMIT
            })
            .is_err()
        );
        assert!(
            RtcClock::new(RtcEpoch {
                counter: u64::MAX,
                ticks: 0
            })
            .is_err()
        );
    }
}
