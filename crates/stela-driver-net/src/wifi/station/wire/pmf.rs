// SPDX-License-Identifier: GPL-2.0-only
// RSN suite/MFP forms follow Linux include/linux/ieee80211.h and
// net/mac80211/mlme.c. See NOTICE for the pinned source.
//! Bounded explicit OS-selected PMF profile. This is parsed policy input,
//! never an authenticated handshake, installed key or publication permission.
use super::{Error, cipher, elements, take, word};
use crate::wifi::security::{Cipher, management::ManagementCipher};
use alloc::vec::Vec;

const MFPR: u16 = 1 << 6;
const MFPC: u16 = 1 << 7;
const EXTENDED_KEY_ID: u16 = 1 << 13;
const SAE: u8 = 8;
const SAE_EXT_KEY: u8 = 24;
const OWE: u8 = 18;

/// Original exact selected RSNE and bounded negotiated policy. Only parsing
/// the explicit opt-in constructor creates it. OS authentication and
/// native hardware permission must be reacquired at every consuming effect.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Selection {
    rsn: Vec<u8>,
    akm: u8,
    required: bool,
    pairwise: Cipher,
    group: Cipher,
    management: ManagementCipher,
    pmkid: bool,
}
impl Selection {
    pub fn rsn(&self) -> &[u8] {
        &self.rsn
    }
    pub const fn akm(&self) -> u8 {
        self.akm
    }
    pub const fn required(&self) -> bool {
        self.required
    }
    pub const fn pairwise(&self) -> Cipher {
        self.pairwise
    }
    pub const fn group(&self) -> Cipher {
        self.group
    }
    pub const fn management(&self) -> ManagementCipher {
        self.management
    }
    /// Whether the AKM authenticates with SAE (`WLAN_AKM_SUITE_SAE` or
    /// `WLAN_AKM_SUITE_SAE_EXT_KEY`), which the OS runs before association.
    pub const fn sae(&self) -> bool {
        matches!(self.akm, SAE | SAE_EXT_KEY)
    }
    /// Whether the RSNE names a PMKID: a cached PMKSA the association
    /// resumes after open-system authentication, as wpa_supplicant's SME
    /// does for SAE.
    pub const fn pmkid(&self) -> bool {
        self.pmkid
    }
    pub(super) fn parse(bytes: &[u8]) -> Result<Self, Error> {
        let profile = Profile::parse(bytes)?;
        if profile.pairwise.len() != 1 || profile.akms.len() != 1 {
            return Err(Error::Unsupported("PMF requires one selected cipher/AKM"));
        }
        let pairwise = cipher(&profile.pairwise[0])?;
        let akm = profile.akms[0];
        // 802.1X and PSK, their SHA-256 forms, Suite B and OWE authenticate
        // open-system, the key derivation being the supplicant's, as neither
        // mac80211 nor mt7925 looks at them; SAE runs in the OS first. FT
        // (3, 4, 9, 13) needs FT authentication and reassociation.
        if akm[..3] != [0, 15, 172]
            || !matches!(akm[3], 1 | 2 | 5 | 6 | 11 | 12 | OWE | SAE | SAE_EXT_KEY)
        {
            return Err(Error::Unsupported("PMF RSN AKM"));
        }
        Ok(Self {
            rsn: bytes.to_vec(),
            akm: akm[3],
            required: profile.capabilities & MFPR != 0,
            pairwise,
            group: profile.group,
            management: profile.management,
            pmkid: profile.pmkids != 0,
        })
    }
    /// Bind to the BSS. A response may omit RSNE, as in Linux;
    /// any supplied RSNE must describe the same selected security profile.
    pub(in crate::wifi) fn validate_peer(
        &self,
        beacon: &[u8],
        response: &[u8],
    ) -> Result<(), Error> {
        let advertised = rsne(beacon)?.ok_or(Error::Invalid("AP PMF RSN absent"))?;
        let ap = Profile::parse(advertised)?;
        if ap.group != self.group
            || ap.management != self.management
            || !ap
                .pairwise
                .iter()
                .any(|s| cipher(s).ok() == Some(self.pairwise))
            || !ap.akms.contains(&[0, 15, 172, self.akm])
        {
            return Err(Error::Invalid("AP does not offer selected PMF suites"));
        }
        if let Some(bytes) = rsne(response)? {
            let selected = Self::parse(bytes)?;
            if selected.akm != self.akm
                || selected.required != self.required
                || selected.pairwise != self.pairwise
                || selected.group != self.group
                || selected.management != self.management
            {
                return Err(Error::Invalid("response/request PMF mismatch"));
            }
        }
        Ok(())
    }
}

fn rsne(bytes: &[u8]) -> Result<Option<&[u8]>, Error> {
    let mut found = None;
    for (id, data) in elements(bytes)? {
        match id {
            48 => {
                if found.replace(data).is_some() {
                    return Err(Error::Invalid("duplicate PMF RSN"));
                }
            }
            221 if data.starts_with(&[0, 0x50, 0xf2, 1]) => {
                return Err(Error::Unsupported("PMF WPA downgrade"));
            }
            _ => (),
        }
    }
    Ok(found)
}
struct Profile {
    group: Cipher,
    pairwise: Vec<[u8; 4]>,
    akms: Vec<[u8; 4]>,
    capabilities: u16,
    management: ManagementCipher,
    pmkids: usize,
}
fn suites(bytes: &mut &[u8]) -> Result<Vec<[u8; 4]>, Error> {
    let count = usize::from(word(bytes)?);
    if count == 0 {
        return Err(Error::Invalid("empty PMF suite list"));
    }
    let bytes = take(bytes, count.checked_mul(4).ok_or(Error::Truncated)?)?;
    Ok(bytes.as_chunks::<4>().0.to_vec())
}
impl Profile {
    fn parse(mut bytes: &[u8]) -> Result<Self, Error> {
        if word(&mut bytes)? != 1 {
            return Err(Error::Unsupported("PMF RSN version"));
        }
        let group = cipher(take(&mut bytes, 4)?)?;
        let pairwise = suites(&mut bytes)?;
        let akms = suites(&mut bytes)?;
        let capabilities = word(&mut bytes)?;
        if capabilities & MFPC == 0 || capabilities & EXTENDED_KEY_ID != 0 {
            return Err(Error::Unsupported("PMF capability/extended key ID"));
        }
        let mut pmkids = 0;
        if !bytes.is_empty() {
            pmkids = usize::from(word(&mut bytes)?);
            take(&mut bytes, pmkids.checked_mul(16).ok_or(Error::Truncated)?)?;
        }
        // IEEE RSN default management-group suite is BIP-CMAC128 when omitted.
        let management = if bytes.is_empty() {
            ManagementCipher::Cmac128
        } else {
            let suite = take(&mut bytes, 4)?;
            (suite[..3] == [0, 15, 172])
                .then(|| ManagementCipher::from_suite(suite[3]))
                .flatten()
                .ok_or(Error::Unsupported("PMF group management cipher"))?
        };
        if !bytes.is_empty() {
            return Err(Error::Unsupported("trailing PMF RSN fields"));
        }
        Ok(Self {
            group,
            pairwise,
            akms,
            capabilities,
            management,
            pmkids,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn selected() -> Vec<u8> {
        vec![
            1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 2, 0xc0, 0, 0, 0, 0, 15,
            172, 6,
        ]
    }
    fn ie(bytes: &[u8]) -> Vec<u8> {
        [vec![48, bytes.len() as u8], bytes.to_vec()].concat()
    }
    #[test]
    fn pmf_selection_carries_gcmp256_and_bip_gmac256() {
        // GCMP-256 group and pairwise, SAE, MFPC|MFPR, no PMKIDs, BIP-GMAC-256.
        let bytes = vec![
            1, 0, 0, 15, 172, 9, 1, 0, 0, 15, 172, 9, 1, 0, 0, 15, 172, 8, 0xc0, 0, 0, 0, 0, 15,
            172, 12,
        ];
        let selection = Selection::parse(&bytes).unwrap();
        assert_eq!(
            (
                selection.pairwise(),
                selection.group(),
                selection.management()
            ),
            (Cipher::Gcmp256, Cipher::Gcmp256, ManagementCipher::Gmac256)
        );
        selection.validate_peer(&ie(&bytes), &ie(&bytes)).unwrap();
        // An AP whose RSNE omits the suite uses BIP-CMAC-128.
        assert!(selection.validate_peer(&ie(&bytes[..22]), &[]).is_err());
        let mut akm = bytes.clone();
        for suite in [5, 6, 11, 12, 18, 3, 4, 9, 13] {
            akm[17] = suite;
            let ft = matches!(suite, 3 | 4 | 9 | 13);
            assert_eq!(
                Selection::parse(&akm).map(|s| s.sae()).ok(),
                (!ft).then_some(false)
            );
        }
    }

    /// An SAE selection naming a PMKID resumes a cached PMKSA; one without
    /// runs SAE first.
    #[test]
    fn an_sae_selection_names_the_pmkid_it_resumes() {
        // CCMP, SAE, MFPC|MFPR, then no PMKID, then one, then BIP-CMAC-128.
        let mut bytes = vec![
            1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 8, 0xc0, 0,
        ];
        let fresh = Selection::parse(&bytes).unwrap();
        assert!(fresh.sae() && !fresh.pmkid());
        bytes.extend_from_slice(&[0, 0]);
        assert!(!Selection::parse(&bytes).unwrap().pmkid());
        bytes[20] = 1;
        bytes.extend_from_slice(&[0x5a; 16]);
        bytes.extend_from_slice(&[0, 15, 172, 6]);
        let resumed = Selection::parse(&bytes).unwrap();
        assert!(resumed.sae() && resumed.pmkid());
        assert_eq!(resumed.management(), ManagementCipher::Cmac128);
    }

    #[test]
    fn exact_pmf_selection_requires_ap_suites_and_preserves_original_rsn() {
        let bytes = selected();
        let selection = Selection::parse(&bytes).unwrap();
        assert_eq!(selection.rsn(), bytes);
        assert_eq!(selection.akm(), 2);
        assert!(selection.required());
        selection.validate_peer(&ie(&bytes), &[]).unwrap();
        selection.validate_peer(&ie(&bytes), &ie(&bytes)).unwrap();
        let mut changed = bytes.clone();
        for offset in [5, 11, 17, 18, 25] {
            changed[offset] = 0;
            assert!(selection.validate_peer(&ie(&changed), &[]).is_err());
            changed.clone_from(&bytes);
        }
        assert!(selection.validate_peer(&[], &[]).is_err());
        assert!(
            selection
                .validate_peer(&[ie(&bytes), ie(&bytes)].concat(), &[])
                .is_err()
        );
        changed[18] = 0x80;
        assert!(selection.validate_peer(&ie(&bytes), &ie(&changed)).is_err());
        let default = Selection::parse(&bytes[..20]).unwrap();
        default.validate_peer(&ie(&bytes[..20]), &[]).unwrap();
        assert!(Selection::parse(&[bytes, vec![0]].concat()).is_err());
    }
}
