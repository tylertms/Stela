//! Native TLS certificate time: the firmware's real-time clock read at boot,
//! carried forward on the invariant TSC, as Linux keeps time from the RTC
//! without NTP.
use crate::handoff::{Handoff, RtcReading};
use alloc::{boxed::Box, sync::Arc};
use stela_boot_core::firmware_time::interval;
use stela_runtime::net::TlsUnavailable;
use stela_tls::{
    PolicyError,
    time::{TimeError, UtcInterval, UtcSource},
};

pub(super) fn native(handoff: &Handoff) -> Result<Arc<stela_tls::ClientConfig>, TlsUnavailable> {
    let reading = handoff.wall.ok_or(TlsUnavailable::CorrelatedClock)?;
    if !crate::cpu::INVARIANT_TSC.present() || handoff.tsc_hz == 0 {
        return Err(TlsUnavailable::CorrelatedClock);
    }
    let entropy = crate::random::source().map_err(|_| TlsUnavailable::Entropy)?;
    // The provider owner lives for the boot, as the configuration requires.
    let owner = Box::leak(Box::new(stela_runtime::net::tls::provider_owner(entropy)));
    let clock = RtcClock {
        reading,
        tsc_hz: handoff.tsc_hz,
        tolerance_ppm: handoff.tsc_tolerance_ppm,
    };
    stela_tls::time::client_config_with_interval(owner, Arc::new(clock)).map_err(
        |error| match error {
            PolicyError::TrustUnavailable => TlsUnavailable::Trust,
            PolicyError::Utc(_) | PolicyError::TimeUnavailable => TlsUnavailable::CorrelatedClock,
            PolicyError::Verifier(_) | PolicyError::Configuration(_) => TlsUnavailable::Provider,
        },
    )
}

/// The UTC interval the boot's RTC reading admits now, the TSC's rate known
/// within `tolerance_ppm`.
#[derive(Debug)]
struct RtcClock {
    reading: RtcReading,
    tsc_hz: u64,
    tolerance_ppm: u32,
}

impl UtcSource for RtcClock {
    fn interval(&self) -> Result<UtcInterval, TimeError> {
        let now = crate::engine::host::rdtsc();
        let reading = &self.reading;
        let elapsed = interval::elapsed_ns(
            now.checked_sub(reading.tsc_after)
                .ok_or(TimeError::SourceChanged)?,
            now - reading.tsc_before,
            self.tsc_hz,
            self.tolerance_ppm,
        )
        .map_err(|_| TimeError::Unavailable)?;
        let bounds = interval::bounds(reading.utc, reading.resolution, elapsed)
            .map_err(|_| TimeError::Unavailable)?;
        UtcInterval::new(*bounds.start(), *bounds.end())
    }
}
