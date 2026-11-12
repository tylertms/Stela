// SPDX-License-Identifier: GPL-2.0-only
// Adapted from pinned Linux net/mac80211/mlme.c. Copyright 2002-2005,
// Instant802 Networks, Inc.; 2005-2006, Devicescape Software, Inc.; 2006-2010
// Johannes Berg; 2018-2026 Intel Corporation. Full source notices in
// NOTICE.
//! What the OS decides for one WPA-Personal station connection, apart from
//! any adapter: the network's [`security`] suites and RSN element, the
//! association's PHY [`mode`] and channel, and the association request's
//! capability field and elements as `ieee80211_send_assoc` builds them.
use super::{Band, station::phy::Mode};
use alloc::vec::Vec;

pub mod keeper;
pub mod mode;
pub mod security;

/// WLAN_CAPABILITY_ESS, _PRIVACY, _SHORT_PREAMBLE, _SPECTRUM_MGMT and
/// _SHORT_SLOT_TIME.
const ESS: u16 = 1 << 0;
pub(crate) const PRIVACY: u16 = 1 << 4;
const SHORT_PREAMBLE: u16 = 1 << 5;
const SPECTRUM_MANAGEMENT: u16 = 1 << 8;
const SHORT_SLOT_TIME: u16 = 1 << 10;

/// The body of the first element `id` among `elements`.
pub(crate) fn find(elements: &[u8], id: u8) -> Option<&[u8]> {
    let mut rest = elements;
    while let [element, length, tail @ ..] = rest {
        let body = tail.get(..usize::from(*length))?;
        if *element == id {
            return Some(body);
        }
        rest = &tail[body.len()..];
    }
    None
}

/// Whether the station manages spectrum with this AP: it advertises
/// spectrum management, which mt76 supports (`SPECTRUM_MGMT`).
pub fn spectrum_management(ap_capabilities: u16) -> bool {
    ap_capabilities & SPECTRUM_MANAGEMENT != 0
}

/// The association request's capability field: privacy for an RSN network,
/// ESS before EHT as mac80211 keeps setting it for old APs, 2.4 GHz short
/// slot time and preamble, and spectrum management with an AP that has it.
pub fn capability(band: Band, mode: Mode, spectrum: bool) -> u16 {
    let mut capability = PRIVACY;
    if mode < Mode::Eht {
        capability |= ESS;
    }
    if band == Band::Ghz2 {
        capability |= SHORT_SLOT_TIME | SHORT_PREAMBLE;
    }
    if spectrum {
        capability |= SPECTRUM_MANAGEMENT;
    }
    capability
}

/// What the station advertises to one AP besides its own capabilities.
pub struct Request<'a> {
    pub ssid: &'a [u8],
    /// The whole RSN element.
    pub rsn: &'a [u8],
    /// The whole RSN Extension element after a hash-to-element SAE exchange.
    pub rsnxe: Option<&'a [u8]>,
    pub mode: Mode,
    /// Whether the AP uses WMM.
    pub wmm: bool,
    /// With spectrum management, the channel's maximum transmit power in dBm
    /// and the channel numbers the band holds.
    pub spectrum: Option<(i8, &'a [u8])>,
}

/// The association request's elements in `ieee80211_send_assoc`'s order:
/// SSID, the station's rates, power capability and supported channels for
/// spectrum management, the RSN element, each capability element of
/// `advertised` the mode uses, the RSN Extension element among the
/// supplicant's remaining non-vendor elements, and WMM last when the AP has
/// it.
pub fn request_elements(request: &Request<'_>, advertised: &[u8]) -> Vec<u8> {
    let mut elements = Vec::new();
    push(&mut elements, 0, request.ssid);
    let parts = advertised_elements(advertised);
    for &(id, body) in parts.iter().filter(|(id, _)| matches!(id, 1 | 50)) {
        push(&mut elements, id, body);
    }
    if let Some((power, channels)) = request.spectrum {
        push(&mut elements, 33, &[0, power as u8]);
        let pairs: Vec<u8> = channels.iter().flat_map(|&number| [number, 1]).collect();
        push(&mut elements, 36, &pairs);
    }
    elements.extend_from_slice(request.rsn);
    for &(id, body) in &parts {
        let used = match (id, body.first()) {
            (45, _) => request.mode >= Mode::Ht,
            (127, _) => true,
            (191, _) => request.mode >= Mode::Vht,
            (255, Some(35 | 59)) => request.mode >= Mode::He,
            (255, Some(108)) => request.mode >= Mode::Eht,
            _ => false,
        };
        if used {
            push(&mut elements, id, body);
        }
    }
    if let Some(rsnxe) = request.rsnxe {
        elements.extend_from_slice(rsnxe);
    }
    for &(_, body) in parts.iter().filter(|(id, _)| *id == 221) {
        if request.wmm {
            push(&mut elements, 221, body);
        }
    }
    elements
}

fn push(elements: &mut Vec<u8>, id: u8, body: &[u8]) {
    elements.extend_from_slice(&[id, body.len() as u8]);
    elements.extend_from_slice(body);
}

fn advertised_elements(bytes: &[u8]) -> Vec<(u8, &[u8])> {
    let mut parts = Vec::new();
    let mut rest = bytes;
    while let [id, length, tail @ ..] = rest {
        let Some(body) = tail.get(..usize::from(*length)) else {
            break;
        };
        parts.push((*id, body));
        rest = &tail[body.len()..];
    }
    parts
}

#[cfg(test)]
mod tests;
