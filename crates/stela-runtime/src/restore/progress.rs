//! Clock-injected, sampled host-send telemetry. No protocol or source I/O.
#[derive(Default)]
pub struct Meter {
    previous: Option<(u64, u64)>,
    changed_at: u64,
}

pub struct Sample {
    pub bytes_per_second: u64,
    pub idle_ms: u64,
    pub eta_seconds: Option<u64>,
}

impl Meter {
    pub fn sample(&mut self, now_ms: u64, sent: u64, total: u64) -> Sample {
        let mut rate = 0;
        if let Some((old_sent, at)) = self.previous {
            let elapsed = now_ms.saturating_sub(at);
            if elapsed != 0 {
                rate = sent.saturating_sub(old_sent).saturating_mul(1000) / elapsed;
            }
            if old_sent != sent {
                self.changed_at = now_ms;
            }
        } else {
            self.changed_at = now_ms;
        }
        self.previous = Some((sent, now_ms));
        Sample {
            bytes_per_second: rate,
            idle_ms: now_ms.saturating_sub(self.changed_at),
            eta_seconds: if sent >= total {
                Some(0)
            } else if rate > 0 {
                Some((total - sent).div_ceil(rate))
            } else {
                None
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rate_uses_sample_delta_not_install_preparation_time() {
        let mut meter = Meter::default();
        assert_eq!(meter.sample(4_103_000, 0, 6_484_393_984).eta_seconds, None);
        let sample = meter.sample(4_113_000, 60_000_000, 6_484_393_984);
        assert_eq!(sample.bytes_per_second, 6_000_000);
        assert_eq!(sample.eta_seconds, Some(1071));
        assert_eq!(sample.idle_ms, 0);
    }

    #[test]
    fn blocked_sends_show_idle_and_unknown_eta_then_recover() {
        let mut meter = Meter::default();
        meter.sample(1000, 100, 1000);
        let sample = meter.sample(61_000, 100, 1000);
        assert_eq!((sample.bytes_per_second, sample.idle_ms), (0, 60_000));
        assert_eq!(sample.eta_seconds, None);
        let sample = meter.sample(62_000, 1000, 1000);
        assert_eq!((sample.bytes_per_second, sample.idle_ms), (900, 0));
        assert_eq!(sample.eta_seconds, Some(0));
    }

    #[test]
    fn zero_length_and_same_timestamp_do_not_divide_by_zero() {
        let mut meter = Meter::default();
        assert_eq!(meter.sample(50, 0, 0).eta_seconds, Some(0));
        assert_eq!(meter.sample(50, 1, 10).eta_seconds, None);
    }
}
