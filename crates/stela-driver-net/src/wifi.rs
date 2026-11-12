//! Shared native 802.11 frame and security boundaries.
//!
//! Vendor transports supply normalized MPDUs, preserving the Protected
//! bit and removing FCS and vendor alignment padding. Decryption observations
//! never imply association, authentication, replay validation or Ethernet
//! carrier. Station management and scanning require vendor commands
//! and completion/retirement. `connection` decides an association from the
//! configured network and the AP's elements; vendor stations run it through
//! WPA-Personal (PSK or SAE) authentication.

mod allocation;
pub mod connection;
pub mod frame;
pub mod intel;
pub mod mediatek;
pub mod network;
pub mod realtek;
pub mod regulatory;
pub mod sae;
pub mod scan;
pub mod security;
pub mod station;
pub mod stepped;
pub mod supplicant;
pub mod unsupported;

use core::sync::atomic::{AtomicU64, Ordering};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channel {
    pub band: Band,
    pub number: u16,
    pub frequency_mhz: u16,
}

/// On-air band, independent of a driver's raw channel encoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Band {
    Ghz2,
    Ghz5,
    Ghz6,
}

/// Received radio information. This is not regulatory permission to TX.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RadioStatus {
    pub band: Band,
    pub channel: u16,
    pub frequency_mhz: u16,
    /// `None` when the device did not measure a usable value.
    pub signal_dbm: Option<i8>,
    pub chain_signal_dbm: [Option<i8>; 4],
    /// Vendor rate/vector encoding with its negotiated layout version.
    pub rate: RawRate,
    /// The decoded rate, when the driver decodes this device's vector.
    pub rx_rate: Option<RxRate>,
    pub tsf: Option<Timestamp>,
}

/// A received frame's rate as mac80211's `ieee80211_rx_status` holds it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxRate {
    pub encoding: Encoding,
    /// `rate_idx`: the band's bitrate index for legacy rates, else the MCS.
    pub index: u8,
    /// Spatial streams for VHT, HE and EHT; HT carries them in its MCS.
    pub nss: u8,
    pub bandwidth: Bandwidth,
    /// RX_ENC_FLAG_SHORT_GI, for encodings before HE.
    pub short_gi: bool,
    /// Space-time streams, RX_ENC_FLAG_STBC_MASK.
    pub stbc: u8,
    /// HE or EHT guard interval: 0.8, 1.6 or 3.2 us as 0 through 2.
    pub gi: u8,
    pub he_dcm: bool,
}

/// mac80211 `mac80211_rx_encoding`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Encoding {
    #[default]
    Legacy,
    Ht,
    Vht,
    He,
    Eht,
}

/// mac80211 `rate_info_bw`, with an HE 106-tone resource unit as its own width.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Bandwidth {
    #[default]
    Mhz20,
    Mhz40,
    Mhz80,
    Mhz160,
    Mhz320,
    HeRu106,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawRate {
    pub layout: u16,
    pub words: [u32; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Timestamp {
    Low32(u32),
    Full64(u64),
}

/// Aggregation observations. Single-MPDU replay checks refuse them;
/// `security::aggregate` checks aggregates after the backend reorders them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Aggregation {
    pub ampdu: Option<u32>,
    pub reorder: Option<Reorder>,
    pub amsdu: Option<Amsdu>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reorder {
    pub ba_id: u16,
    pub next_sequence: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Amsdu {
    pub index: u16,
    pub first: bool,
    pub last: bool,
}

/// A process-wide source of nonzero identities that never wraps, so a stale
/// owner, epoch or ticket can never match a later one.
pub(crate) struct Identities(AtomicU64);

impl Identities {
    pub(crate) const fn new() -> Self {
        Self(AtomicU64::new(1))
    }

    /// The next identity, or `None` once the 64-bit space is used up.
    pub(crate) fn next(&self) -> Option<u64> {
        self.0
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| {
                id.checked_add(1).filter(|_| id != 0)
            })
            .ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identities_never_wrap_or_return_zero() {
        let ids = Identities(AtomicU64::new(u64::MAX - 1));
        assert_eq!(ids.next(), Some(u64::MAX - 1));
        assert_eq!(ids.next(), None);
        assert_eq!(ids.next(), None);
        assert_eq!(Identities(AtomicU64::new(0)).next(), None);
        assert_eq!(Identities::new().next(), Some(1));
    }
}
