//! A single-writer progress marker, observable without taking the writer's
//! locks. The low eight timestamp bits hold the phase, so one atomic load
//! always reads a coherent phase and timestamp. Phase zero disables it.
use core::sync::atomic::{AtomicU64, Ordering};

pub struct StallWatch(AtomicU64);

impl StallWatch {
    pub const fn new() -> Self {
        Self(AtomicU64::new(0))
    }

    pub fn mark(&self, phase: u8, now: u64) {
        self.0
            .store((now & !255) | u64::from(phase), Ordering::Relaxed);
    }

    /// Phase and elapsed ticks once the threshold is reached. Quantization
    /// adds at most 255 ticks of error; wrapping counters remain valid.
    pub fn stalled(&self, now: u64, threshold: u64) -> Option<(u8, u64)> {
        let marker = self.0.load(Ordering::Relaxed);
        let phase = marker as u8;
        let elapsed = (now & !255).wrapping_sub(marker & !255);
        // The writer may mark after the reader sampled `now`. That is a
        // future marker, not a counter wrap representing a long stall.
        (phase != 0 && elapsed < (1u64 << 63) && elapsed >= threshold).then_some((phase, elapsed))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_and_recent_progress_do_not_report_a_stall() {
        let watch = StallWatch::new();
        assert_eq!(watch.stalled(10_000, 1_000), None);
        watch.mark(2, 2_048);
        assert_eq!(watch.stalled(2_560, 1_024), None);
        assert_eq!(watch.stalled(3_072, 1_024), Some((2, 1_024)));
        watch.mark(3, 4_096);
        assert_eq!(watch.stalled(3_840, 1_024), None);
        watch.mark(3, 3_100);
        assert_eq!(watch.stalled(3_200, 1_024), None);
        watch.mark(0, 3_200);
        assert_eq!(watch.stalled(100_000, 1_024), None);
    }

    #[test]
    fn wrapping_ticks_keep_the_original_phase() {
        let watch = StallWatch::new();
        watch.mark(8, u64::MAX - 127);
        assert_eq!(watch.stalled(768, 1_024), Some((8, 1_024)));
    }
}
