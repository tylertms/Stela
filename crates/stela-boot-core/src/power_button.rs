//! What a press of the host's power button does to the guest.
//!
//! A PC's ACPI power button status records a press, not how long the
//! button was held, so a held button cannot be told from a short press. A
//! short press of a Mac's button asks macOS to sleep or to show its
//! shutdown choice; holding it cuts the power, whatever the guest does. The
//! host gives the guest a press and a release for each press, and stands in
//! for a hold with a second press within [`FORCE_WINDOW_MILLIS`] of the
//! first, while that request is still pending: the guest has not slept
//! since. A press while the guest sleeps wakes it and starts over, as does
//! any press once the window passed, so a press minutes later, or after a
//! sleep and wake, is a new request and never cuts the guest off.

/// How soon after a press a second one stands for a hold of the button.
pub const FORCE_WINDOW_MILLIS: u64 = 3000;

/// What one press does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Press {
    /// A press and a release for the guest.
    Normal,
    /// The button held: the machine turns off, whatever the guest does.
    ForceOff,
}

/// The last press that may still be pending.
#[derive(Clone, Copy, Debug, Default)]
pub struct Presses {
    /// Its counter value and the guest's sleeps then.
    last: Option<(u64, u64)>,
}

impl Presses {
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// A press at counter value `now`, with the window `window` ticks long,
    /// the guest having slept `sleeps` times since the boot and `asleep`
    /// now.
    pub fn press(&mut self, now: u64, window: u64, sleeps: u64, asleep: bool) -> Press {
        if asleep {
            // A wake: the request before it was answered by the sleep.
            self.last = None;
            return Press::Normal;
        }
        let pending = self
            .last
            .is_some_and(|(at, slept)| slept == sleeps && now.wrapping_sub(at) < window);
        if pending {
            self.last = None;
            Press::ForceOff
        } else {
            self.last = Some((now, sleeps));
            Press::Normal
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WINDOW: u64 = 3000;

    #[test]
    fn a_press_after_the_window_is_a_new_request() {
        let mut presses = Presses::new();
        assert_eq!(presses.press(1000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(4000, WINDOW, 0, false), Press::Normal);
        // Minutes later, and again.
        assert_eq!(presses.press(600_000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(900_000, WINDOW, 0, false), Press::Normal);
    }

    #[test]
    fn a_second_press_within_the_window_forces_the_machine_off() {
        let mut presses = Presses::new();
        assert_eq!(presses.press(1000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(3999, WINDOW, 0, false), Press::ForceOff);
        // The forced press starts over.
        assert_eq!(presses.press(4500, WINDOW, 0, false), Press::Normal);
    }

    #[test]
    fn a_sleep_and_wake_start_over() {
        let mut presses = Presses::new();
        // A press, the guest sleeps, a press wakes it, a press soon after
        // asks again.
        assert_eq!(presses.press(1000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(1500, WINDOW, 1, true), Press::Normal);
        assert_eq!(presses.press(2000, WINDOW, 1, false), Press::Normal);
        // A sleep since the last press, the guest awake again (an RTC
        // wake): a new request.
        assert_eq!(presses.press(2500, WINDOW, 2, false), Press::Normal);
        // Two presses with no sleep between still force.
        assert_eq!(presses.press(2600, WINDOW, 2, false), Press::ForceOff);
    }

    #[test]
    fn a_counter_that_wraps_keeps_the_window() {
        let mut presses = Presses::new();
        assert_eq!(
            presses.press(u64::MAX - 10, WINDOW, 0, false),
            Press::Normal
        );
        assert_eq!(presses.press(100, WINDOW, 0, false), Press::ForceOff);
    }
}
