// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! The suites wpa_supplicant selects for a WPA-Personal network against an
//! AP's RSN element (`wpa_supplicant_set_suites`) and the RSN element it
//! then sends (`wpa_write_rsn_ie`). The network is configured as
//! NetworkManager configures one: `key_mgmt=WPA-PSK WPA-PSK-SHA256 SAE`, SAE
//! only for a passphrase, wpa_supplicant's default pairwise (CCMP, TKIP) and
//! group (CCMP, TKIP) ciphers, and `ieee80211w=1`, optional management frame
//! protection.
use crate::wifi::{
    Band,
    sae::Derivation,
    security::{Cipher, management::ManagementCipher},
    supplicant::{Akm, Suites},
};
use alloc::vec::Vec;

const OUI: [u8; 3] = [0x00, 0x0f, 0xac];
const PSK: u8 = 2;
const PSK_SHA256: u8 = 6;
const SAE: u8 = 8;
const CCMP: [u8; 4] = [0, 0x0f, 0xac, 4];
const PSK_SUITE: [u8; 4] = [0, 0x0f, 0xac, PSK];
const MFPR: u16 = 1 << 6;
const MFPC: u16 = 1 << 7;

/// Why the AP's security does not admit the configured network.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// No RSN element: an open, WEP or WPA1-only network.
    NoRsn,
    /// A truncated or malformed RSN element.
    Malformed,
    /// No PSK or SAE AKM the station may use: 802.1X or FT-only networks,
    /// or SAE-only (WPA3-Personal) ones without a passphrase or protection.
    Akm,
    /// No CCMP-128 pairwise cipher.
    Pairwise,
    /// A group cipher other than CCMP-128 or TKIP.
    Group,
    /// Management frame protection the AP requires but does not offer, or
    /// a group management cipher outside BIP.
    ManagementProtection,
    /// A 6 GHz AP without SAE by hash-to-element, the only personal security
    /// wpa_supplicant takes on 6 GHz.
    SixGhz,
}

/// The suites of one association.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Security {
    pub akm: Akm,
    pub group: Cipher,
    /// The group management cipher, when management frames are protected.
    pub management: Option<ManagementCipher>,
}

/// The fields of an AP's RSN element (IEEE 802.11-2020 9.4.2.24), each list
/// as raw suites. Fields past the end take their defaults.
struct Advertised<'a> {
    group: [u8; 4],
    pairwise: &'a [[u8; 4]],
    akms: &'a [[u8; 4]],
    capabilities: u16,
    management: Option<[u8; 4]>,
}

fn take<'a>(bytes: &mut &'a [u8], count: usize) -> Result<&'a [u8], Refusal> {
    let (head, rest) = bytes.split_at_checked(count).ok_or(Refusal::Malformed)?;
    *bytes = rest;
    Ok(head)
}

fn word(bytes: &mut &[u8]) -> Result<u16, Refusal> {
    Ok(u16::from_le_bytes(take(bytes, 2)?.try_into().unwrap()))
}

fn list<'a>(bytes: &mut &'a [u8]) -> Result<&'a [[u8; 4]], Refusal> {
    let count = usize::from(word(bytes)?);
    let (suites, rest) = take(bytes, count * 4)?.as_chunks::<4>();
    debug_assert!(rest.is_empty());
    Ok(suites)
}

impl<'a> Advertised<'a> {
    /// `wpa_parse_wpa_ie_rsn`: the RSN defaults stand for absent fields.
    fn parse(mut bytes: &'a [u8]) -> Result<Self, Refusal> {
        if word(&mut bytes)? != 1 {
            return Err(Refusal::Malformed);
        }
        let mut rsn = Self {
            group: CCMP,
            pairwise: core::slice::from_ref(&CCMP),
            akms: core::slice::from_ref(&PSK_SUITE),
            capabilities: 0,
            management: None,
        };
        if bytes.is_empty() {
            return Ok(rsn);
        }
        rsn.group = take(&mut bytes, 4)?.try_into().unwrap();
        if bytes.is_empty() {
            return Ok(rsn);
        }
        rsn.pairwise = list(&mut bytes)?;
        if bytes.is_empty() {
            return Ok(rsn);
        }
        rsn.akms = list(&mut bytes)?;
        if bytes.is_empty() {
            return Ok(rsn);
        }
        rsn.capabilities = word(&mut bytes)?;
        if bytes.is_empty() {
            return Ok(rsn);
        }
        let pmkids = usize::from(word(&mut bytes)?);
        take(&mut bytes, pmkids * 16)?;
        if bytes.len() >= 4 {
            rsn.management = Some(take(&mut bytes, 4)?.try_into().unwrap());
        }
        Ok(rsn)
    }
}

impl Security {
    /// The suites for the AP whose beacon or probe response carries
    /// `elements`: SAE, for a `passphrase`, over PSK-SHA256 over PSK, as
    /// `wpa_supplicant_set_suites` prefers them, the first two only when
    /// management frames are protected, as the station's association
    /// request admits them.
    /// `select` for an AP on `band`: on 6 GHz only SAE, and only when the
    /// AP's RSNXE offers hash-to-element (`wpa_scan_res_ok`).
    pub fn select_on(band: Band, elements: &[u8], passphrase: bool) -> Result<Self, Refusal> {
        let security = Self::select(elements, passphrase)?;
        if band == Band::Ghz6
            && (security.akm != Akm::Sae
                || Derivation::for_rsnxe(super::find(elements, 244)) != Derivation::HashToElement)
        {
            return Err(Refusal::SixGhz);
        }
        Ok(security)
    }

    pub fn select(elements: &[u8], passphrase: bool) -> Result<Self, Refusal> {
        let body = rsn_element(elements).ok_or(Refusal::NoRsn)?;
        let rsn = Advertised::parse(body)?;
        if !rsn.pairwise.contains(&CCMP) {
            return Err(Refusal::Pairwise);
        }
        let group = match rsn.group {
            [0, 0x0f, 0xac, 4] => Cipher::Ccmp128,
            [0, 0x0f, 0xac, 2] => Cipher::Tkip,
            _ => return Err(Refusal::Group),
        };
        // An AP requiring protection it does not offer is malformed; with
        // ieee80211w=1 the station protects whenever the AP can.
        let protected = rsn.capabilities & MFPC != 0;
        if rsn.capabilities & MFPR != 0 && !protected {
            return Err(Refusal::ManagementProtection);
        }
        let offers = |akm| rsn.akms.contains(&[0, 0x0f, 0xac, akm]);
        let akm = if protected && passphrase && offers(SAE) {
            Akm::Sae
        } else if protected && offers(PSK_SHA256) {
            Akm::PskSha256
        } else if offers(PSK) {
            Akm::Psk
        } else {
            return Err(Refusal::Akm);
        };
        let management = if protected {
            let suite = rsn.management.unwrap_or([0, 0x0f, 0xac, 6]);
            Some(
                (suite[..3] == OUI)
                    .then(|| ManagementCipher::from_suite(suite[3]))
                    .flatten()
                    .ok_or(Refusal::ManagementProtection)?,
            )
        } else {
            None
        };
        Ok(Self {
            akm,
            group,
            management,
        })
    }

    /// The supplicant's suites for this association.
    pub fn suites(&self) -> Suites {
        Suites {
            akm: self.akm,
            pairwise: Cipher::Ccmp128,
            group: self.group,
            management: self.management,
        }
    }

    /// `wpa_write_rsn_ie`: the whole RSN element the station sends, with one
    /// pairwise cipher and AKM, MFPC when it protects management frames, and
    /// the group management cipher when it is not the default BIP-CMAC-128.
    /// It carries no PMKID: wpa_supplicant's `sae_pmkid_in_assoc` defaults
    /// to leaving an SAE exchange's out of the association request.
    pub fn element(&self) -> Vec<u8> {
        self.element_with(None)
    }

    /// The RSN element naming `pmkid`, the cached PMKSA an association
    /// resumes, as `wpa_write_rsn_ie` writes `sm->pmksa`'s.
    pub fn element_with(&self, pmkid: Option<[u8; 16]>) -> Vec<u8> {
        let akm = match self.akm {
            Akm::Psk => PSK,
            Akm::PskSha256 => PSK_SHA256,
            Akm::Sae => SAE,
        };
        let group = match self.group {
            Cipher::Tkip => 2,
            _ => 4,
        };
        let mut body = Vec::from([1, 0]);
        body.extend_from_slice(&[0, 0x0f, 0xac, group]);
        body.extend_from_slice(&[1, 0]);
        body.extend_from_slice(&CCMP);
        body.extend_from_slice(&[1, 0, 0, 0x0f, 0xac, akm]);
        let capabilities = if self.management.is_some() { MFPC } else { 0 };
        body.extend_from_slice(&capabilities.to_le_bytes());
        let management = self
            .management
            .filter(|&cipher| cipher != ManagementCipher::Cmac128);
        if let Some(pmkid) = pmkid {
            body.extend_from_slice(&[1, 0]);
            body.extend_from_slice(&pmkid);
        } else if management.is_some() {
            // An empty PMKID list precedes the group management cipher.
            body.extend_from_slice(&[0, 0]);
        }
        if let Some(cipher) = management {
            body.extend_from_slice(&[0, 0x0f, 0xac, cipher.suite()]);
        }
        let mut element = Vec::from([48, body.len() as u8]);
        element.extend_from_slice(&body);
        element
    }
}

/// The body of the first RSN element among `elements`.
pub(crate) fn rsn_element(elements: &[u8]) -> Option<&[u8]> {
    super::find(elements, 48)
}

#[cfg(test)]
mod tests;
