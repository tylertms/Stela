// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright 2002-2005, Instant802 Networks, Inc.
// Copyright 2005-2006, Devicescape Software, Inc.
// Copyright 2006-2007 Jiri Benc <jbenc@suse.cz>
// Copyright 2007, 2009 Johannes Berg <johannes@sipsolutions.net>
// Copyright 2013-2014 Intel Mobile Communications GmbH
// Copyright (C) 2015-2017 Intel Deutschland GmbH
// Copyright (C) 2018-2026 Intel Corporation
//! Ordinary full source probe IE selection over kept MT7925 capabilities.
use super::{Advertisement, LocalError, extension, ie};
use crate::wifi::{Band, mediatek::mcu::domain};
use alloc::vec::Vec;

const CHAN_DISABLED: u32 = 1;
const CHAN_NO_80: u32 = 1 << 7;
const CHAN_NO_HE: u32 = 1 << 13;
const CHAN_NO_EHT: u32 = 1 << 20;

/// Source any-usable-channel decisions over this owner's completed domain.
/// These only select advertised families; they grant no probe permission.
pub(in crate::wifi::mediatek::bootstrap::client) struct Selection {
    vht: bool,
    he: bool,
    eht: bool,
    he6: Option<u16>,
}
impl Selection {
    pub(in crate::wifi::mediatek::bootstrap::client) fn programmed(
        domain: &domain::Request,
        band: Band,
        he6: u16,
    ) -> Self {
        fn usable(domain: &domain::Request, band: Band, excluded: u32) -> bool {
            let index = match band {
                Band::Ghz2 => 0,
                Band::Ghz5 => 1,
                Band::Ghz6 => 2,
            };
            domain.rows(index).iter().any(|row| {
                u32::from_le_bytes(row[4..8].try_into().unwrap()) & (CHAN_DISABLED | excluded) == 0
            })
        }
        Self {
            vht: usable(domain, band, CHAN_NO_80),
            he: usable(domain, band, CHAN_NO_HE),
            eht: usable(domain, band, CHAN_NO_HE | CHAN_NO_EHT),
            he6: (he6 != 0 && usable(domain, Band::Ghz6, CHAN_NO_HE)).then_some(he6),
        }
    }
}

/// Source mac80211 ordinary full probe contents. Reuse this device's
/// independently source-checked capability issuer, omitting association WMM and
/// placing HE6 after EHT as ieee80211_put_preq_ies_band does. Probe SMPS is OFF.
/// No custom PHY IE may supply capabilities on behalf of this device.
pub(in crate::wifi::mediatek::bootstrap::client) fn elements(
    advertisement: &Advertisement,
    selection: Selection,
) -> Result<Vec<u8>, LocalError> {
    let mut out = Vec::new();
    out.try_reserve_exact(advertisement.elements.len() + 15)
        .map_err(|_| LocalError::NoMemory)?;
    let mut rest = advertisement.elements();
    while !rest.is_empty() {
        let id = rest[0];
        let length = usize::from(*rest.get(1).ok_or(LocalError::WrongAdvertisement)?);
        let bytes = rest
            .get(2..2 + length)
            .ok_or(LocalError::WrongAdvertisement)?;
        match (id, bytes.first()) {
            (221, _) => (),
            (255, Some(59)) => (),
            (191, _) if !selection.vht => (),
            (255, Some(35)) if !selection.he => (),
            (255, Some(108)) if !selection.eht => (),
            _ => ie(&mut out, id, bytes),
        }
        rest = &rest[2 + length..];
    }
    const HE6_SMPS_MASK: u16 = 0x0600;
    const HE6_SMPS_DISABLED: u16 = 0x0600;
    if let Some(he6) = selection.he6 {
        extension(
            &mut out,
            59,
            &((he6 & !HE6_SMPS_MASK) | HE6_SMPS_DISABLED).to_le_bytes(),
        );
    }
    Ok(out)
}

/// Compose OS probe metadata around this device's generated PHY elements.
/// Prefix splitting follows mac80211's contiguous groups, never sorting an IE
/// across an earlier unknown element. MT7925's source MCU helper omits the
/// separate `common_ies` tail; explicitly keep it in EACH native band TLV.
/// The caller checks the firmware's shared budget before scan ownership.
pub(in crate::wifi::mediatek::bootstrap::client) fn with_extra(
    advertisement: &Advertisement,
    selection: Selection,
    extra: &[u8],
) -> Result<Vec<u8>, LocalError> {
    validate_extra(extra)?;
    let generated = elements(advertisement, selection)?;
    let replaced = names_extended_capabilities(extra);
    let mut out = Vec::new();
    out.try_reserve_exact(generated.len() + extra.len())
        .map_err(|_| LocalError::NoMemory)?;
    let mut offset = 0;
    for stage in 0..5 {
        let mut rest = generated.as_slice();
        while !rest.is_empty() {
            let length = 2 + usize::from(rest[1]);
            let family = match rest[0] {
                1 => 0,
                50 => 1,
                45 | 127 => 2,
                191 => 3,
                _ => 4,
            };
            if family == stage && !(rest[0] == 127 && replaced) {
                out.extend_from_slice(&rest[..length]);
            }
            rest = &rest[length..];
        }
        if stage < 4 {
            let start = offset;
            while offset < extra.len() {
                let id = extra[offset];
                let data = &extra[offset + 2..offset + 2 + usize::from(extra[offset + 1])];
                let eligible = match stage {
                    0 => id == 10, // Request information; SSID/rates are owned.
                    1 => id == 59, // Supported regulatory classes; DS is owned.
                    2 => matches!(id, 72 | 127 | 84 | 97 | 107 | 114),
                    3 => id == 239 || (id == 255 && data.first() == Some(&2)),
                    _ => false,
                };
                if !eligible {
                    break;
                }
                let payload_length = usize::from(extra[offset + 1]);
                offset += 2 + payload_length;
                // mac80211 skip_ie keeps fragments attached to their parent.
                if payload_length == 255 {
                    while offset < extra.len() && extra[offset] == 242 {
                        offset += 2 + usize::from(extra[offset + 1]);
                    }
                }
            }
            out.extend_from_slice(&extra[start..offset]);
        }
    }
    out.extend_from_slice(&extra[offset..]);
    Ok(out)
}

/// The OS's Extended Capabilities stand for the device's, as wpa_supplicant's
/// are the only ones in a mac80211 probe.
fn names_extended_capabilities(mut extra: &[u8]) -> bool {
    while let [id, length, tail @ ..] = extra {
        if *id == 127 {
            return true;
        }
        extra = &tail[usize::from(*length)..];
    }
    false
}

fn validate_extra(mut extra: &[u8]) -> Result<(), LocalError> {
    let mut fragments = false;
    while !extra.is_empty() {
        let length = usize::from(*extra.get(1).ok_or(LocalError::WrongAdvertisement)?);
        let data = extra
            .get(2..2 + length)
            .ok_or(LocalError::WrongAdvertisement)?;
        let id = extra[0];
        // Caller metadata cannot replace generated rates, channel or PHY/MLO
        // facts, including unsupported other-band and UHR families. Opaque
        // vendor metadata is not consumed as a local capability/permission.
        // Only FILS request extensions are implemented. Other selectors
        // include PHY/MU power, MLO mapping, identity/regulatory and UHR.
        if matches!(
            id,
            0 | 1 | 3 | 45 | 50 | 61 | 148 | 151 | 158 | 191 | 192 | 217 | 232
        ) || (id == 255 && data.first().is_some_and(|id| *id != 2))
            || (id == 221
                && (data.starts_with(&[0, 0x90, 0x4c]) || data.starts_with(&[0, 0x50, 0xf2, 2])))
            || (id == 242 && !fragments)
        {
            return Err(LocalError::UnsupportedProbeElements);
        }
        if id == 255 && data.is_empty() {
            return Err(LocalError::WrongAdvertisement);
        }
        // cfg80211_defragment_element ends a chain at its first short
        // fragment. A following Fragment would be orphaned even though the
        // ordering-only skip_ie helper assumes an already valid chain.
        fragments = length == 255;
        extra = &extra[2 + length..];
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::super::{MloObservations, make};
    use super::*;
    use crate::{
        Mac,
        wifi::{
            Channel,
            mediatek::{
                identity::Chip,
                mcu::{client as wire, device::Context},
            },
            scan::Request,
            station::phy::Width,
        },
    };

    #[test]
    fn empty_custom_composition_preserves_all_source_owned_probe_bytes() {
        if crate::test_process::isolate_model() {
            return;
        }
        for band in [Band::Ghz2, Band::Ghz5, Band::Ghz6] {
            let actual = make(
                Chip::Mt7925,
                band,
                3,
                3,
                Width::Mhz20,
                true,
                *b"US",
                MloObservations {
                    flags: None,
                    eml: None,
                },
            )
            .unwrap();
            let selected = || Selection {
                vht: true,
                he: true,
                eht: true,
                he6: Some(0x30b8),
            };
            assert_eq!(
                with_extra(&actual, selected(), &[]).unwrap(),
                elements(&actual, selected()).unwrap()
            );
        }
    }

    #[test]
    fn source_custom_split_keeps_fragment_chain_and_unknown_tail_in_input_order() {
        if crate::test_process::isolate_model() {
            return;
        }
        let actual = make(
            Chip::Mt7925,
            Band::Ghz2,
            3,
            3,
            Width::Mhz20,
            true,
            *b"US",
            MloObservations {
                flags: None,
                eml: None,
            },
        )
        .unwrap();
        let selected = || Selection {
            vht: true,
            he: true,
            eht: true,
            he6: None,
        };
        let mut extra = vec![10, 255];
        extra.extend_from_slice(&[0xab; 255]);
        extra.extend_from_slice(&[242, 2, 0xcd, 0xef, 59, 1, 81, 200, 0, 127, 1, 0]);
        let composed = with_extra(&actual, selected(), &extra).unwrap();
        let rates_end = 2 + usize::from(composed[1]);
        assert_eq!(&composed[rates_end..rates_end + 261], &extra[..261]);
        assert_eq!(composed[rates_end + 261], 50);
        // Extcaps after unknown200 must stay in the common tail.
        assert_eq!(&composed[composed.len() - 5..], &[200, 0, 127, 1, 0]);
        // The metadata's Extended Capabilities replace the device's ten bytes.
        let original = elements(&actual, selected()).unwrap();
        assert_eq!(composed.len(), original.len() - 10 + extra.len());
    }

    #[test]
    fn custom_fragment_validity_requires_a_full_immediate_predecessor() {
        if crate::test_process::isolate_model() {
            return;
        }
        let mut valid = vec![10, 255];
        valid.extend_from_slice(&[0xab; 255]);
        valid.extend_from_slice(&[242, 255]);
        valid.extend_from_slice(&[0xcd; 255]);
        valid.extend_from_slice(&[242, 2, 0xef, 0x42]);
        // Pure codec validity only: the generic scan request's separate
        // 384-byte limit cannot admit this full three-part chain.
        assert_eq!(validate_extra(&valid), Ok(()));
        valid.extend_from_slice(&[242, 1, 0x24]);
        assert_eq!(
            validate_extra(&valid),
            Err(LocalError::UnsupportedProbeElements)
        );
        assert_eq!(
            validate_extra(&[10, 1, 0xab, 242, 1, 0xcd]),
            Err(LocalError::UnsupportedProbeElements)
        );
    }

    #[test]
    fn source_usable_band_union_never_turns_flags_into_probe_permission() {
        if crate::test_process::isolate_model() {
            return;
        }
        use crate::wifi::mediatek::identity::Chip;
        let blocked = domain::Channel {
            hardware_value: 1,
            flags: CHAN_NO_80 | CHAN_NO_HE | CHAN_NO_EHT,
        };
        let allowed = domain::Channel {
            hardware_value: 5,
            flags: 0,
        };
        let rows = [blocked, allowed];
        let programmed = domain::Request::build(
            Chip::Mt7925,
            domain::Inputs {
                alpha2: *b"US",
                channels_2ghz: &[],
                channels_5ghz: &rows,
                channels_6ghz: &[blocked],
            },
        )
        .unwrap();
        let selected = Selection::programmed(&programmed, Band::Ghz5, 0x30b8);
        assert!(selected.vht && selected.he && selected.eht);
        assert_eq!(selected.he6, None);
        let selected = Selection::programmed(&programmed, Band::Ghz6, 0x30b8);
        assert!(!selected.vht && !selected.he && !selected.eht);
        assert_eq!(selected.he6, None);
    }

    #[test]
    #[ignore = "oracle: STELA_MT7925_PROBE_ORACLE"]
    fn probe_ies_and_commands_match_source() {
        if crate::test_process::isolate_model() {
            return;
        }
        let root = std::path::PathBuf::from(std::env::var_os("STELA_MT7925_PROBE_ORACLE").unwrap());
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
                        for flags in [
                            0,
                            CHAN_NO_80,
                            CHAN_NO_HE,
                            CHAN_NO_EHT,
                            CHAN_NO_80 | CHAN_NO_HE | CHAN_NO_EHT,
                        ] {
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
                            let rows = [domain::Channel {
                                hardware_value: 1,
                                flags,
                            }];
                            let domain = domain::Request::build(
                                crate::wifi::mediatek::identity::Chip::Mt7925,
                                domain::Inputs {
                                    alpha2: *b"US",
                                    channels_2ghz: &rows,
                                    channels_5ghz: &rows,
                                    channels_6ghz: &rows,
                                },
                            )
                            .unwrap();
                            let selected =
                                elements(&a, Selection::programmed(&domain, band, 0x30b8)).unwrap();
                            let code = match band {
                                Band::Ghz2 => 0,
                                Band::Ghz5 => 1,
                                Band::Ghz6 => 3,
                            };
                            assert_eq!(
                                &trace[at..at + 4],
                                &[code, nss, wi as u8, u8::from(enabled)],
                                "case{count}"
                            );
                            at += 4;
                            let length = usize::from(u16::from_le_bytes(
                                trace[at..at + 2].try_into().unwrap(),
                            ));
                            at += 2;
                            assert_eq!(
                                selected,
                                trace[at..at + length],
                                "source probe case{count}"
                            );
                            at += length;
                            let mut all = [Vec::new(), Vec::new(), Vec::new()];
                            all[match band {
                                Band::Ghz2 => 0,
                                Band::Ghz5 => 1,
                                Band::Ghz6 => 2,
                            }] = selected;
                            let request = Request::new(
                                vec![Channel {
                                    band,
                                    number: if band == Band::Ghz5 { 36 } else { 1 },
                                    frequency_mhz: match band {
                                        Band::Ghz2 => 2412,
                                        Band::Ghz5 => 5180,
                                        Band::Ghz6 => 5955,
                                    },
                                }],
                                vec![],
                                vec![],
                            )
                            .unwrap();
                            let encoded = wire::scan_with_elements(
                                Context {
                                    bss_index: 0,
                                    omac_index: 0,
                                    band_index: 255,
                                    wmm_index: 0,
                                    link_index: 0,
                                    broadcast_wcid: 19,
                                    address: Mac([2; 6]),
                                },
                                1,
                                &request,
                                Some(&all),
                            )
                            .unwrap();
                            assert_eq!(trace[at], 0x16);
                            at += 1;
                            let length = usize::from(u16::from_le_bytes(
                                trace[at..at + 2].try_into().unwrap(),
                            ));
                            at += 2;
                            assert_eq!(
                                encoded.body(),
                                &trace[at..at + length],
                                "source firmware scan case{count}"
                            );
                            at += length;
                            count += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(count, 400);
        assert_eq!(at, trace.len());
    }
}
