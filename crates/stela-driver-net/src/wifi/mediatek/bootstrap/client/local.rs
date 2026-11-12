// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
// Copyright (C) 2020-2025 MediaTek Inc.
//! Source-selected MT7921, MT7922 and MT7925 station advertisements from this device's
//! NIC, image/EEPROM/CLC and source-policy observations. Advertisement
//! bytes are capabilities, never an association or transmission permission.
use super::*;
use crate::wifi::{
    Band,
    mediatek::firmware::auxiliary::Selection,
    station::phy::{self, Capabilities, McsMaps, Role, Width},
};
use alloc::vec;

pub(super) mod scan;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LocalError {
    NoMemory,
    WrongDevice,
    MissingPhy,
    MissingSixGhz,
    UnsupportedBand,
    UnsupportedNss,
    UnsupportedWidth,
    MalformedBeControl,
    WrongAdvertisement,
    UnsupportedProbeElements,
    Selection(crate::wifi::mediatek::firmware::auxiliary::Error),
    Capabilities(phy::Error),
}
/// Preserve the optional NIC TLV facts. The station requests an AP MLD's
/// links only where the NIC reports MLO support, as `mt7925_init_mlo_caps`
/// sets WIPHY_FLAG_SUPPORTS_MLO only under MT792x_CHIP_CAP_MLO_EN.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MloObservations {
    pub flags: Option<u64>,
    pub eml: Option<u16>,
}
impl MloObservations {
    pub fn reported_supported(self) -> Option<bool> {
        self.flags.map(|f| f & (1 << 8) != 0)
    }
    pub fn reported_eml_supported(self) -> Option<bool> {
        self.flags.map(|f| f & (1 << 9) != 0)
    }
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Advertisement {
    elements: Vec<u8>,
    capabilities: Capabilities,
    antenna_mask: u8,
    alpha2: [u8; 2],
    mlo: MloObservations,
    he_source_maps: [McsMaps; 3],
}
/// mac80211's `ext_capa` for `CHANCTX_STA_CSA` and the Multiple BSSID bit of
/// `ieee80211_send_assoc`: extended channel switching, Multiple BSSID and
/// operating mode notifications.
const EXTENDED_CAPABILITIES: [u8; 8] = [4, 0, 0x40, 0, 0, 0, 0, 0x40];

impl Advertisement {
    pub fn elements(&self) -> &[u8] {
        &self.elements
    }
    pub fn capabilities(&self) -> &Capabilities {
        &self.capabilities
    }
    pub const fn antenna_mask(&self) -> u8 {
        self.antenna_mask
    }
    pub const fn source_country(&self) -> [u8; 2] {
        self.alpha2
    }
    pub const fn mlo(&self) -> MloObservations {
        self.mlo
    }
    /// Complete source struct slots (80,160,80+80), independently of which
    /// maps the HE width flags put on the wire. Source initializes160 even on
    /// 2GHz and leaves80+80 zeroed. Zero is an observed source value here,
    /// never an inferred capability or permission for that width.
    pub const fn he_source_maps(&self) -> &[McsMaps; 3] {
        &self.he_source_maps
    }
    /// Check the outgoing station advertisement against this owner's
    /// source-selected capabilities. The frontend may omit an optional PHY
    /// family or supported rate; it cannot add hardware, streams, power-save
    /// behavior or capabilities. This grants no association/radio permission.
    pub fn validate_capabilities(&self, requested: &Capabilities) -> Result<(), LocalError> {
        let actual = &self.capabilities;
        let rates = requested.rates();
        let source_rates = actual.rates();
        let family = |selected: bool, present: bool| !selected || present;
        if requested.band() != actual.band()
            || requested.role() != Role::Station
            || rates.supported & !source_rates.supported != 0
            || rates.basic & !rates.supported != 0
            || (requested.band() != Band::Ghz6 && rates.supported == 0)
            || requested.ht().is_some_and(|h| Some(h) != actual.ht())
            || requested.vht().is_some_and(|v| Some(v) != actual.vht())
            || requested.he().is_some_and(|h| Some(h) != actual.he())
            || requested.eht().is_some_and(|e| Some(e) != actual.eht())
            || requested
                .he_6ghz()
                .is_some_and(|h| Some(h) != actual.he_6ghz())
            // A legacy request to an AP without WMM omits it, as mac80211 does.
            || requested.wmm().is_some_and(|w| Some(w) != actual.wmm())
            || (requested.he().is_some()
                && requested.local_he_source_maps() != Some(&self.he_source_maps))
            || !family(rates.selectors & 1 != 0, requested.ht().is_some())
            || !family(rates.selectors & 2 != 0, requested.vht().is_some())
            || !family(rates.selectors & 4 != 0, requested.he().is_some())
            || !family(rates.selectors & 8 != 0, requested.eht().is_some())
        {
            return Err(LocalError::WrongAdvertisement);
        }
        Ok(())
    }
}
impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// Native single-link, always-awake station capabilities. The OS adds its
    /// SSID/security/operation observations. This does not enable MLO,
    /// power-save, regulatory permissions or any modern native publication.
    pub fn local_advertisement(&self, band: Band) -> Result<Advertisement, LocalError> {
        current(self.state(), band)
    }
}
pub(super) fn current<I: BarIo, Q: QueueIo, C: 'static>(
    s: &Owned<I, Q, C>,
    band: Band,
) -> Result<Advertisement, LocalError> {
    let policy = s
        .controller
        .state()
        .policy
        .as_ref()
        .ok_or(LocalError::MissingPhy)?;
    let observations = policy.capabilities().observed();
    let phy = observations.phy().ok_or(LocalError::MissingPhy)?;
    let chip = observations.chip();
    if chip != s.controller.state().identity.chip {
        return Err(LocalError::WrongDevice);
    }
    let available = match band {
        Band::Ghz2 => phy.has_2ghz(),
        Band::Ghz5 => phy.has_5ghz(),
        Band::Ghz6 => observations.six_ghz().ok_or(LocalError::MissingSixGhz)?,
    };
    if !available {
        return Err(LocalError::UnsupportedBand);
    }
    let antenna = phy.antenna_mask().map_err(|_| LocalError::UnsupportedNss)?;
    // The pinned PHY callback sets chainmask=antenna_mask; it does
    // not infer extra sounding chains from an unrelated EEPROM default.
    let inputs = policy.source_inputs();
    // MT7921 and MT7922 have no EHT, and their CLC no BE control.
    let eht = chip == Chip::Mt7925 && {
        let selection = policy
            .auxiliary()
            .select_clc(policy.eeprom().observed().byte())
            .map_err(LocalError::Selection)?;
        source_eht_enabled(&selection, inputs.alpha2, inputs.mtcl_configuration)?
    };
    make(
        chip,
        band,
        antenna,
        u16::from(antenna),
        Width::Mhz20,
        eht,
        inputs.alpha2,
        MloObservations {
            flags: observations.chip_flags(),
            eml: observations.eml(),
        },
    )
}
/// Pure source BE-control selection. A missing list and MTCL invalid sentinel
/// follow the source's EHT advertisement default; neither grants radio/policy
/// permission. Malformed/no-terminator data is refused instead of overreading.
fn source_eht_enabled(
    selection: &Selection<'_>,
    alpha2: [u8; 2],
    mtcl: u32,
) -> Result<bool, LocalError> {
    if selection.chip() != Chip::Mt7925 {
        return Err(LocalError::WrongDevice);
    }
    be_control(selection.records()[2].map(|r| r.payload()), alpha2, mtcl)
}
fn be_control(data: Option<&[u8]>, alpha2: [u8; 2], mtcl: u32) -> Result<bool, LocalError> {
    if mtcl != u32::MAX && ((mtcl >> 4) & 3) == 0 {
        return Ok(false);
    }
    let Some(mut rest) = data else {
        return Ok(true);
    };
    loop {
        let rule = rest.get(..16).ok_or(LocalError::MalformedBeControl)?;
        if rule[4..6] == alpha2 {
            return Ok(false);
        }
        if u32::from_le_bytes(rule[..4].try_into().unwrap()) & 1 != 0 {
            return Ok(true);
        }
        rest = &rest[16..];
    }
}
fn ie(bytes: &mut Vec<u8>, id: u8, data: &[u8]) {
    bytes.extend_from_slice(&[id, data.len() as u8]);
    bytes.extend_from_slice(data);
}
fn extension(bytes: &mut Vec<u8>, id: u8, data: &[u8]) {
    bytes.extend_from_slice(&[255, (data.len() + 1) as u8, id]);
    bytes.extend_from_slice(data);
}
fn mcs(nss: u8) -> u16 {
    let mut map = u16::MAX;
    for i in 0..nss {
        map = (map & !(3 << (i * 2))) | (2 << (i * 2));
    }
    map
}
// Pinned source station fixed fields: HTC/TF padding/OMI/AMPDU-extension/
// A-MSDU; LDPC/STBC/MU/DCM/beamformee/feedback/PPE/long-SIG-B capabilities.
const HE_STATION_MAC: [u8; 6] = [1, 8, 0, 0x1a, 0x40, 0];
const HE_STATION_PHY: [u8; 11] = [0, 0x70, 0xce, 0x12, 0x6d, 0xc0, 0xb3, 6, 0x4e, 0x3f, 0];
/// mt7921_init_he_caps sets the same station fields except these, which it
/// adds only for MT7922: PHY 0 CHANNEL_WIDTH_SET_160MHZ_IN_5G, PHY 4
/// BEAMFORMEE_MAX_STS_ABOVE_80MHZ_4, PHY 8 20MHZ_IN_160MHZ and
/// 80MHZ_IN_160MHZ_HE_PPDU.
const HE_160: [(usize, u8); 3] = [(0, 0x08), (4, 0x60), (8, 0x0c)];
const HT_BASE: u16 = 0x0973;
const VHT_BASE: u32 = 0x3390_7176;
/// VHT SUPP_CHAN_WIDTH_160MHZ and SHORT_GI_160, which mt7921_register_device
/// adds only for MT7922.
const VHT_160: u32 = 0x44;
#[allow(
    clippy::too_many_arguments,
    reason = "each is one source input of the advertisement"
)]
fn make(
    chip: Chip,
    band: Band,
    antenna: u8,
    chainmask: u16,
    width: Width,
    eht: bool,
    alpha2: [u8; 2],
    mlo: MloObservations,
) -> Result<Advertisement, LocalError> {
    let nss = antenna.count_ones() as u8;
    let sts = chainmask.count_ones() as u8;
    if !(1..=4).contains(&nss) || !(1..=4).contains(&sts) {
        return Err(LocalError::UnsupportedNss);
    }
    if matches!(width, Width::Mhz80P80 | Width::Mhz320)
        || (band == Band::Ghz2 && !matches!(width, Width::Mhz20 | Width::Mhz40))
    {
        return Err(LocalError::UnsupportedWidth);
    }
    let wide = chip != Chip::Mt7921;
    let eht = eht && chip == Chip::Mt7925;
    let mut elements = Vec::new();
    if band == Band::Ghz2 {
        ie(&mut elements, 1, &[2, 4, 11, 22, 12, 18, 24, 36]);
        ie(&mut elements, 50, &[48, 72, 96, 108]);
    } else {
        // mt76_init_sband_6g takes the OFDM rates mt76_init_sband_5g does,
        // and mac80211 puts them in each 6 GHz request.
        ie(&mut elements, 1, &[12, 18, 24, 36, 48, 72, 96, 108]);
    }
    if band != Band::Ghz6 {
        let mut ht = [0; 26];
        let cap = HT_BASE | if nss > 1 { 0x80 } else { 0 };
        ht[..2].copy_from_slice(&cap.to_le_bytes());
        // Preserve the exact pinned init.c assignment:2G density is assigned
        // twice, ending at1us(enum3);5G remains its initialized zero value.
        // mt7921 leaves both at mt76_init_sband's zero.
        let density = if band == Band::Ghz2 && chip == Chip::Mt7925 {
            3
        } else {
            0
        };
        ht[2] = 3 | density << 2;
        ht[3..3 + usize::from(nss)].fill(0xff);
        ht[15] = 1;
        ie(&mut elements, 45, &ht);
        ie(&mut elements, 127, &EXTENDED_CAPABILITIES);
        if band == Band::Ghz5 {
            let mut vht = [0; 12];
            let base = if wide { VHT_BASE } else { VHT_BASE & !VHT_160 };
            let cap = base | if nss > 1 { 0x80 } else { 0 };
            vht[..4].copy_from_slice(&cap.to_le_bytes());
            vht[4..6].copy_from_slice(&mcs(nss).to_le_bytes());
            vht[8..10].copy_from_slice(&mcs(nss).to_le_bytes());
            vht[10..12].copy_from_slice(&0x2000u16.to_le_bytes()); // ExtNSS-capable flag
            ie(&mut elements, 191, &vht);
        }
    }
    let mut he = Vec::from(HE_STATION_MAC);
    let mut phy = HE_STATION_PHY;
    phy[0] = if band == Band::Ghz2 {
        2 | 0x20
    } else {
        4 | 8 | 0x40
    };
    if !wide {
        for (i, bits) in HE_160 {
            phy[i] &= !bits;
        }
    }
    he.extend_from_slice(&phy);
    let map = mcs(nss).to_le_bytes();
    for _ in 0..if band == Band::Ghz2 || !wide { 1 } else { 2 } {
        he.extend_from_slice(&map);
        he.extend_from_slice(&map);
    }
    let ru: u8 = if band == Band::Ghz2 { 3 } else { 15 };
    let bits = 6 * usize::from(nss) * ru.count_ones() as usize;
    let size = bits.div_ceil(8);
    let pattern = [0x1c, 0xc7, 0x71];
    he.push((nss - 1) | (ru << 3));
    for i in 0..size - 1 {
        he.push(pattern[i % 3]);
    }
    he.push(pattern[(size - 1) % 3] & (0xff >> (8 - (bits - 1) % 8)));
    if band == Band::Ghz6 {
        ie(&mut elements, 127, &EXTENDED_CAPABILITIES);
    }
    extension(&mut elements, 35, &he);
    if band == Band::Ghz6 {
        // Source5G density0 + maxAMPDU1024K + MPDU11454 + TX/RX ant pattern.
        extension(&mut elements, 59, &0x30b8u16.to_le_bytes());
    }
    if eht {
        let bfee = (sts - 1).max(3);
        let sd = sts - 1;
        let mcs15 = match width {
            Width::Mhz160 => 7,
            Width::Mhz80 => 3,
            _ => 1,
        };
        let phy = [
            0x68 | (bfee & 1) << 7,
            ((bfee >> 1) & 3) | ((bfee << 2) & 0x1c),
            sd & 7 | ((sd << 3) & 0x38),
            0xfe,
            sd.min(2) << 4,
            0x61,
            4 | (mcs15 << 3),
            0x36,
            0,
        ];
        let mut cap = vec![3, 0];
        cap.extend_from_slice(&phy);
        let nss_map = nss | (nss << 4);
        cap.extend_from_slice(&[nss_map; 3]);
        if band != Band::Ghz2 {
            cap.extend_from_slice(&[nss_map; 3]);
        }
        // MT7925 has no320 bit/map; source admits onlyMT7927 there.
        extension(&mut elements, 108, &cap);
    }
    // Explicit native always-awake selection: no requested UAPSD service
    // period or queue is advertised before a native PM executor exists.
    ie(&mut elements, 221, &[0, 0x50, 0xf2, 2, 0, 1, 0]);
    let he_source_maps = [
        McsMaps {
            rx: mcs(nss),
            tx: mcs(nss),
        },
        // mt7921_init_he_caps fills the 160 MHz maps only for MT7922.
        if wide {
            McsMaps {
                rx: mcs(nss),
                tx: mcs(nss),
            }
        } else {
            McsMaps { rx: 0, tx: 0 }
        },
        McsMaps { rx: 0, tx: 0 },
    ];
    let capabilities = Capabilities::parse(band, Role::Station, &elements)
        .and_then(|cap| cap.with_local_he_source_maps(he_source_maps))
        .map_err(LocalError::Capabilities)?;
    Ok(Advertisement {
        elements,
        capabilities,
        antenna_mask: antenna,
        alpha2,
        mlo,
        he_source_maps,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn source_local_profile_has_real_bands_mcs_ppe_width_and_mlo_observations() {
        if crate::test_process::isolate_model() {
            return;
        }
        for band in [Band::Ghz2, Band::Ghz5, Band::Ghz6] {
            for nss in 1..=4 {
                let a = make(
                    Chip::Mt7925,
                    band,
                    (1 << nss) - 1,
                    (1 << nss) - 1,
                    Width::Mhz20,
                    true,
                    *b"US",
                    MloObservations {
                        flags: Some(3 << 8),
                        eml: Some(0x1234),
                    },
                )
                .unwrap();
                assert_eq!(a.antenna_mask(), (1 << nss) - 1);
                assert_eq!(a.mlo().reported_supported(), Some(true));
                assert_eq!(a.mlo().reported_eml_supported(), Some(true));
                assert_eq!(a.capabilities().he().unwrap().mcs_80.rx_nss(), nss);
                assert!(a.capabilities().eht().unwrap().mcs.bw320.is_none());
                assert_eq!(a.capabilities().he().unwrap().phy[0] & 16, 0);
                if band == Band::Ghz6 {
                    assert!(a.capabilities().ht().is_none());
                    assert_eq!(a.capabilities().he_6ghz(), Some(0x30b8));
                }
                if band != Band::Ghz2 {
                    assert!(
                        a.elements()
                            .starts_with(&[1, 8, 12, 18, 24, 36, 48, 72, 96, 108])
                    );
                }
                if band == Band::Ghz5 {
                    assert_eq!(a.capabilities().vht().unwrap().capability & 12, 4);
                    assert_eq!(a.capabilities().vht().unwrap().tx_highest, 0x2000);
                }
            }
        }
        let m = MloObservations {
            flags: None,
            eml: None,
        };
        assert_eq!(m.reported_supported(), None);
        assert_eq!(
            make(
                Chip::Mt7925,
                Band::Ghz5,
                0,
                0,
                Width::Mhz20,
                true,
                *b"US",
                m
            ),
            Err(LocalError::UnsupportedNss)
        );
        assert_eq!(
            make(
                Chip::Mt7925,
                Band::Ghz6,
                3,
                3,
                Width::Mhz320,
                true,
                *b"US",
                m
            ),
            Err(LocalError::UnsupportedWidth)
        );
        assert_eq!(
            make(
                Chip::Mt7925,
                Band::Ghz5,
                3,
                3,
                Width::Mhz80P80,
                true,
                *b"US",
                m
            ),
            Err(LocalError::UnsupportedWidth)
        );
    }

    /// The element `id` (extension `extension` under 255) of `elements`.
    fn find(elements: &[u8], id: u8, extension: Option<u8>) -> Option<&[u8]> {
        let mut at = 0;
        while at < elements.len() {
            let len = usize::from(elements[at + 1]);
            let body = &elements[at + 2..at + 2 + len];
            if elements[at] == id && (id != 255 || Some(body[0]) == extension) {
                return Some(if id == 255 { &body[1..] } else { body });
            }
            at += 2 + len;
        }
        None
    }

    #[test]
    fn connac2_profiles_follow_mt7921_init_he_caps_and_register_device() {
        if crate::test_process::isolate_model() {
            return;
        }
        let m = MloObservations {
            flags: None,
            eml: None,
        };
        // MT7921 has no 160 MHz; MT7922 has the MT7925's. Neither has EHT,
        // even when asked.
        for (chip, phy0, phy4, phy8, vht, maps) in [
            (Chip::Mt7921, 0x44, 0x0d, 0x42, 0x3390_71b2u32, 1),
            (Chip::Mt7922, 0x4c, 0x6d, 0x4e, 0x3390_71f6, 2),
        ] {
            let a = make(chip, Band::Ghz5, 3, 3, Width::Mhz20, true, *b"US", m).unwrap();
            let e = a.elements();
            assert!(find(e, 255, Some(108)).is_none());
            let ht = find(e, 45, None).unwrap();
            assert_eq!(ht[..5], [0xf3, 0x09, 3, 0xff, 0xff]);
            let v = find(e, 191, None).unwrap();
            assert_eq!(v[..4], vht.to_le_bytes());
            assert_eq!(v[4..12], [0xfa, 0xff, 0, 0, 0xfa, 0xff, 0, 0x20]);
            let he = find(e, 255, Some(35)).unwrap();
            assert_eq!(he[..6], [1, 8, 0, 0x1a, 0x40, 0]);
            assert_eq!(
                he[6..17],
                [phy0, 0x70, 0xce, 0x12, phy4, 0xc0, 0xb3, 6, phy8, 0x3f, 0]
            );
            let ppe = 17 + 4 * maps;
            assert!(he[17..ppe].chunks(2).all(|m| m == [0xfa, 0xff]));
            // mt76_connac_gen_ppe_thresh: NSS 2, RU mask 0xf.
            assert_eq!(he[ppe..], [0x79, 0x1c, 0xc7, 0x71, 0x1c, 0xc7, 0x71]);
            let wide = u16::from(chip == Chip::Mt7922) * 0xfffa;
            assert_eq!(a.he_source_maps()[1], McsMaps { rx: wide, tx: wide });
            assert_eq!(a.validate_capabilities(a.capabilities()), Ok(()));
            // mt76_init_sband leaves the 2.4 GHz density at zero.
            let a = make(chip, Band::Ghz2, 1, 1, Width::Mhz20, false, *b"US", m).unwrap();
            assert_eq!(
                find(a.elements(), 45, None).unwrap()[..4],
                [0x73, 0x09, 3, 0xff]
            );
        }
        // The 6 GHz capability derives from the 5 GHz HT and VHT as for MT7925.
        let a = make(
            Chip::Mt7922,
            Band::Ghz6,
            3,
            3,
            Width::Mhz20,
            false,
            *b"US",
            m,
        )
        .unwrap();
        assert_eq!(a.capabilities().he_6ghz(), Some(0x30b8));
        assert!(find(a.elements(), 45, None).is_none());
    }

    #[test]
    fn outgoing_capabilities_cannot_promote_streams_phy_or_sleep_behavior() {
        if crate::test_process::isolate_model() {
            return;
        }
        let source = make(
            Chip::Mt7925,
            Band::Ghz5,
            3,
            3,
            Width::Mhz20,
            true,
            *b"US",
            MloObservations {
                flags: Some(0),
                eml: None,
            },
        )
        .unwrap();
        assert_eq!(source.validate_capabilities(source.capabilities()), Ok(()));
        let edit = |id, extension_id, field, value| {
            let mut bytes = source.elements().to_vec();
            let mut at = 0;
            while at < bytes.len() {
                let len = usize::from(bytes[at + 1]);
                if bytes[at] == id && (id != 255 || Some(bytes[at + 2]) == extension_id) {
                    bytes[at + 2 + field] = value;
                    break;
                }
                at += 2 + len;
            }
            Capabilities::parse(Band::Ghz5, Role::Station, &bytes).unwrap()
        };
        // Nss2 cannot become nss3 through an OS-copied HT or HE record.
        let ht = edit(45, None, 5, 0xff);
        let he = edit(255, Some(35), 18, 0xea);
        // Source always-awake mode must not advertise a new UAPSD executor.
        let asleep = edit(221, None, 6, 0xf);
        // Nor may copied PHY fields silently add a firmware beamforming effect.
        let eht = edit(255, Some(108), 6, 0xff);
        for requested in [&ht, &he, &asleep, &eht] {
            assert_eq!(
                source.validate_capabilities(requested),
                Err(LocalError::WrongAdvertisement)
            );
        }
        let wrong_band = make(
            Chip::Mt7925,
            Band::Ghz2,
            3,
            3,
            Width::Mhz20,
            true,
            *b"US",
            source.mlo(),
        )
        .unwrap();
        assert_eq!(
            source.validate_capabilities(wrong_band.capabilities()),
            Err(LocalError::WrongAdvertisement)
        );
        // A frontend can omit optional EHT and restrict its supported legacy rates.
        let mut narrowed = Vec::new();
        let mut at = 0;
        while at < source.elements.len() {
            let n = usize::from(source.elements[at + 1]);
            let ie = &source.elements[at..at + 2 + n];
            if ie[0] == 1 {
                narrowed.extend_from_slice(&[1, 2, 0x8c, 24]);
            } else if ie[0] != 255 || ie[2] != 108 {
                narrowed.extend_from_slice(ie);
            }
            at += 2 + n;
        }
        let narrowed = Capabilities::parse(Band::Ghz5, Role::Station, &narrowed)
            .unwrap()
            .with_local_he_source_maps(*source.he_source_maps())
            .unwrap();
        assert_eq!(source.validate_capabilities(&narrowed), Ok(()));
    }
    #[test]
    #[ignore = "oracle: STELA_MT7925_LOCAL_ORACLE"]
    fn unchanged_full_source_local_capabilities_match_all_band_stream_width_profiles() {
        if crate::test_process::isolate_model() {
            return;
        }
        extern crate std;
        let root = std::path::PathBuf::from(std::env::var_os("STELA_MT7925_LOCAL_ORACLE").unwrap());
        let trace = std::fs::read(root.join("client.trace")).unwrap();
        let mut at = 0;
        let mut count = 0;
        for band in [Band::Ghz2, Band::Ghz5, Band::Ghz6] {
            for nss in 1..=4 {
                for (wi, width) in [Width::Mhz20, Width::Mhz40, Width::Mhz80, Width::Mhz160]
                    .into_iter()
                    .enumerate()
                {
                    if band == Band::Ghz2 && wi > 1 {
                        continue;
                    }
                    for enabled in [false, true] {
                        let a = make(
                            Chip::Mt7925,
                            band,
                            (1 << nss) - 1,
                            (1 << nss) - 1,
                            width,
                            enabled,
                            *b"US",
                            MloObservations {
                                flags: None,
                                eml: None,
                            },
                        )
                        .unwrap();
                        assert_eq!(
                            &trace[at..at + 4],
                            &[
                                match band {
                                    Band::Ghz2 => 0,
                                    Band::Ghz5 => 1,
                                    Band::Ghz6 => 3,
                                },
                                nss,
                                wi as u8,
                                u8::from(enabled)
                            ],
                            "case{count}"
                        );
                        at += 4;
                        assert_eq!(
                            a.elements(),
                            &trace[at..at + a.elements().len()],
                            "band{band:?} NSS{nss} width{width:?} EHT{enabled}"
                        );
                        at += a.elements().len();
                        for maps in a.he_source_maps() {
                            for value in [maps.rx, maps.tx] {
                                assert_eq!(&trace[at..at + 2], &value.to_le_bytes());
                                at += 2;
                            }
                        }
                        count += 1;
                    }
                }
            }
        }
        assert_eq!(count, 80);
        for mtcl in [u32::MAX, 0, 0x10, 0x20, 0x30, 0x80] {
            for rows in 0..=4 {
                for matched in 0..=rows {
                    for old in [false, true] {
                        let meta = &trace[at..at + 10];
                        at += 10;
                        assert_eq!(&meta[..4], &mtcl.to_le_bytes());
                        assert_eq!(
                            &meta[4..8],
                            &[
                                rows as u8,
                                matched as u8,
                                u8::from(old),
                                u8::from(rows != 0)
                            ]
                        );
                        let mut data = vec![0; rows * 16];
                        for i in 0..rows {
                            data[i * 16..i * 16 + 4]
                                .copy_from_slice(&u32::from(i == rows - 1).to_le_bytes());
                            data[i * 16 + 4] = if i == matched { b'U' } else { b'A' + i as u8 };
                            data[i * 16 + 5] = if i == matched { b'S' } else { b'a' + i as u8 };
                        }
                        let enabled =
                            be_control(if rows == 0 { None } else { Some(&data) }, *b"US", mtcl)
                                .unwrap();
                        assert_eq!(meta[8], u8::from(enabled));
                        assert_eq!(meta[9], u8::from(old != enabled));
                    }
                }
            }
        }
        assert_eq!(at, trace.len());
    }

    #[test]
    fn be_control_bounds_preserve_exact_match_stop_and_invalid_mtcl_semantics() {
        if crate::test_process::isolate_model() {
            return;
        }
        assert_eq!(be_control(None, *b"US", u32::MAX), Ok(true));
        assert_eq!(be_control(None, *b"US", 0), Ok(false));
        assert_eq!(
            be_control(Some(&[]), *b"US", 0x10),
            Err(LocalError::MalformedBeControl)
        );
        assert_eq!(
            be_control(Some(&[0; 15]), *b"US", 0x10),
            Err(LocalError::MalformedBeControl)
        );
        let mut bytes = [0; 16];
        bytes[4..6].copy_from_slice(b"GB");
        assert_eq!(
            be_control(Some(&bytes), *b"US", 0x10),
            Err(LocalError::MalformedBeControl)
        );
        bytes[0] = 1;
        assert_eq!(be_control(Some(&bytes), *b"US", 0x10), Ok(true));
        bytes[4..6].copy_from_slice(b"US");
        assert_eq!(be_control(Some(&bytes), *b"US", 0x10), Ok(false));
        assert_eq!(be_control(Some(&[]), *b"US", 0), Ok(false));
    }
}
