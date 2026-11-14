//! The Linux task step watchdog's decisions. The host's Linux environment
//! runs driver work in bounded steps on the boot processor; a step that
//! never returns would leave the PC hanging with no stop record. While that
//! processor is the only one running (the GPU's start, before the other
//! processors start), a core cycle counter armed to overflow after
//! [`LIMIT_SECONDS`] raises an NMI there. Each step that returns, or the
//! dispatcher going idle, puts the counter back once a second at most
//! ([`next_rearm`]): the NMI comes only when nothing returned for the
//! whole limit, and its handler names the stuck step ([`attribute`],
//! [`message`]) with the interrupted instruction pointer.
//!
//! The counter counts unhalted core cycles, whose rate follows the core's
//! clock: [`start_value`] counts [`CLOCK_FACTOR`] times the time-stamp
//! counter's rate, so the NMI never comes before the limit, and comes at
//! most `CLOCK_FACTOR` times later on a core that runs at its base clock.
use alloc::{format, string::String};

/// How long no step may return before the watchdog stops the boot: half of
/// the GPU start's whole budget (60 s), far past nouveau's longest wait
/// inside one step (its 2 s register and firmware polls), and far below
/// the five minutes a reporter waited at a black screen.
pub const LIMIT_SECONDS: u64 = 30;

/// How often at most a returned step puts the counter back.
pub const REARM_SECONDS: u64 = 1;

/// The core clock's bound as a multiple of the time-stamp counter's rate:
/// a turbo clock stays below three times the nominal one.
pub const CLOCK_FACTOR: u64 = 3;

/// The value a `width`-bit up-counter starts from to overflow after the
/// limit's cycles at [`CLOCK_FACTOR`] times `tsc_hz`; `None` when the
/// count does not fit or the rate is unknown.
pub fn start_value(tsc_hz: u64, width: u32) -> Option<u64> {
    if tsc_hz == 0 || !(32..=64).contains(&width) {
        return None;
    }
    let cycles = tsc_hz
        .checked_mul(LIMIT_SECONDS)?
        .checked_mul(CLOCK_FACTOR)?;
    let mask = u64::MAX >> (64 - width);
    if cycles == 0 || cycles > mask {
        return None;
    }
    Some((mask - cycles).wrapping_add(1) & mask)
}

/// When a step returning at `now` next puts the counter back.
pub fn next_rearm(now: u64, tsc_hz: u64) -> u64 {
    now.saturating_add(tsc_hz.saturating_mul(REARM_SECONDS).max(1))
}

/// What the NMI handler reads on the watched processor.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Observation {
    /// The counter is armed on this processor.
    pub armed: bool,
    /// When the counter was last put back.
    pub armed_at: u64,
    /// When the running step's switch began, 0 between steps.
    pub step_began: u64,
    pub task: u64,
    pub sequence: u64,
    pub now: u64,
    pub tsc_hz: u64,
}

/// The step the watchdog found running.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Step {
    pub task: u64,
    pub sequence: u64,
    pub running_ms: u64,
}

/// A watchdog stop: how long nothing returned, and the step running then,
/// if one was.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Stuck {
    pub quiet_ms: u64,
    pub step: Option<Step>,
}

fn millis(ticks: u64, tsc_hz: u64) -> u64 {
    (u128::from(ticks) * 1000 / u128::from(tsc_hz.max(1))) as u64
}

/// Whether an NMI is the watchdog's: the counter was armed and nothing put
/// it back for the whole limit, the soonest it can overflow. Any other NMI
/// (a platform's, or one before the limit) is not, and keeps the plain
/// exception report.
pub fn attribute(observation: &Observation) -> Option<Stuck> {
    let Observation {
        armed,
        armed_at,
        step_began,
        task,
        sequence,
        now,
        tsc_hz,
    } = *observation;
    let quiet = now.checked_sub(armed_at)?;
    if !armed || tsc_hz == 0 || quiet < tsc_hz.checked_mul(LIMIT_SECONDS)? {
        return None;
    }
    let step = (step_began != 0)
        .then(|| now.checked_sub(step_began))
        .flatten()
        .map(|running| Step {
            task,
            sequence,
            running_ms: millis(running, tsc_hz),
        });
    Some(Stuck {
        quiet_ms: millis(quiet, tsc_hz),
        step,
    })
}

/// The stop's reason, before the handler's instruction pointer and stack:
/// what the boot was doing (`stage`) and the step that never returned.
pub fn message(stage: &str, stuck: &Stuck) -> String {
    match stuck.step {
        Some(step) => format!(
            "a Linux task step did not return within {LIMIT_SECONDS} s ({stage}): task {} \
             job {} ran {} ms; the step watchdog stopped it",
            step.task, step.sequence, step.running_ms
        ),
        None => format!(
            "Linux's work did not return to its dispatcher within {LIMIT_SECONDS} s ({stage}), \
             outside any task step, for {} ms; the step watchdog stopped it",
            stuck.quiet_ms
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: u64 = 2_400_000_000;

    #[test]
    fn the_counter_overflows_after_the_limit_at_the_bounded_clock() {
        let start = start_value(HZ, 48).unwrap();
        assert_eq!((1u64 << 48) - start, HZ * LIMIT_SECONDS * CLOCK_FACTOR);
        let start = start_value(HZ, 64).unwrap();
        assert_eq!(start.wrapping_neg(), HZ * LIMIT_SECONDS * CLOCK_FACTOR);
        // A 32-bit counter cannot hold 216 G cycles; no rate, no counter.
        assert_eq!(start_value(HZ, 32), None);
        assert_eq!(start_value(0, 48), None);
        assert_eq!(start_value(HZ, 16), None);
        assert_eq!(start_value(u64::MAX / 2, 64), None);
    }

    #[test]
    fn a_returned_step_puts_the_counter_back_once_a_second_at_most() {
        assert_eq!(next_rearm(10, HZ), 10 + HZ);
        assert_eq!(next_rearm(u64::MAX - 1, HZ), u64::MAX);
        assert_eq!(next_rearm(10, 0), 11);
    }

    fn observed(quiet_seconds: u64, step_seconds: Option<u64>) -> Observation {
        let now = 1000 * HZ;
        Observation {
            armed: true,
            armed_at: now - quiet_seconds * HZ,
            step_began: step_seconds.map_or(0, |seconds| now - seconds * HZ),
            task: 7,
            sequence: 42,
            now,
            tsc_hz: HZ,
        }
    }

    #[test]
    fn only_an_nmi_after_the_whole_limit_on_an_armed_counter_is_the_watchdogs() {
        // Before the limit, the counter cannot have overflowed: another NMI.
        assert_eq!(attribute(&observed(LIMIT_SECONDS - 1, Some(40))), None);
        let unarmed = Observation {
            armed: false,
            ..observed(LIMIT_SECONDS + 5, Some(40))
        };
        assert_eq!(attribute(&unarmed), None);
        let no_rate = Observation {
            tsc_hz: 0,
            ..observed(LIMIT_SECONDS + 5, Some(40))
        };
        assert_eq!(attribute(&no_rate), None);
        // A put-back time after the NMI's own reading is no overflow either.
        let ahead = Observation {
            armed_at: 1001 * HZ,
            ..observed(LIMIT_SECONDS, None)
        };
        assert_eq!(attribute(&ahead), None);
        assert_eq!(
            attribute(&observed(LIMIT_SECONDS, Some(40))),
            Some(Stuck {
                quiet_ms: LIMIT_SECONDS * 1000,
                step: Some(Step {
                    task: 7,
                    sequence: 42,
                    running_ms: 40_000
                }),
            })
        );
        assert_eq!(
            attribute(&observed(LIMIT_SECONDS + 3, None)),
            Some(Stuck {
                quiet_ms: (LIMIT_SECONDS + 3) * 1000,
                step: None,
            })
        );
    }

    #[test]
    fn the_stop_names_the_stage_and_the_step() {
        let stuck = attribute(&observed(LIMIT_SECONDS, Some(31))).unwrap();
        let text = message("the GPU's probe", &stuck);
        assert!(text.contains("(the GPU's probe)"), "{text}");
        assert!(text.contains("task 7 job 42 ran 31000 ms"), "{text}");
        let stuck = attribute(&observed(LIMIT_SECONDS, None)).unwrap();
        assert!(message("Linux's first tasks", &stuck).contains("outside any task step"));
    }
}
