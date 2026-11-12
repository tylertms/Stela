// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! The connac2 BSS and peer records, sent as UNI BSS_INFO_UPDATE and
//! STA_REC_UPDATE with connac2 layouts: `mt76_connac_mcu_uni_add_bss`,
//! `mt7921_mcu_sta_update` through `mt76_connac_mcu_sta_cmd` with its
//! embedded WTBL TLVs, and `mt76_connac_mcu_sta_ba`.
use super::super::{Error, Profile, modern::Configuration, put16, put32, tlv};
use crate::wifi::{Band, station::phy::Width};
use alloc::{vec, vec::Vec};

const STA_REC_BASIC: u16 = 0;
const STA_REC_RA: u16 = 1;
const STA_REC_BA: u16 = 6;
const STA_REC_STATE: u16 = 7;
const STA_REC_HT: u16 = 9;
const STA_REC_VHT: u16 = 10;
const STA_REC_WTBL: u16 = 13;
const STA_REC_HE: u16 = 14;
const STA_REC_HW_AMSDU: u16 = 15;
const STA_REC_PHY: u16 = 0x15;
const STA_REC_HE_6G: u16 = 0x17;
const STA_REC_HE_V2: u16 = 0x19;
const WTBL_GENERIC: u16 = 0;
const WTBL_RX: u16 = 1;
const WTBL_HT: u16 = 2;
const WTBL_VHT: u16 = 3;
const WTBL_HDR_TRANS: u16 = 6;
const WTBL_BA: u16 = 8;
const WTBL_SMPS: u16 = 13;
const WTBL_RESET_AND_SET: u8 = 1;
const WTBL_SET: u8 = 2;
const UNI_BSS_INFO_BASIC: u16 = 0;
const UNI_BSS_INFO_RLM: u16 = 2;
const UNI_BSS_INFO_BSS_COLOR: u16 = 4;
const UNI_BSS_INFO_HE_BASIC: u16 = 5;
const UNI_BSS_INFO_11V_MBSSID: u16 = 6;
const UNI_BSS_INFO_QBSS: u16 = 15;
/// `CONNECTION_INFRA_AP` and `CONNECTION_INFRA_STA`.
const CONNECTION_INFRA_AP: u32 = (1 << 1) | (1 << 16);
const CONNECTION_INFRA_STA: u32 = 1 | (1 << 16);
/// `MT76_STA_INFO_STATE_ASSOC`.
const STATE_ASSOC: u8 = 2;
/// `to_rcpi(0)`: mt7921 reports its RSSI average, empty until the AP's
/// frames are measured, as the initial rate-control hint.
const RCPI_UNMEASURED: u8 = 220;

/// The profile's validation for a connac2 record: 6 GHz only through a
/// modern configuration, which must match the profile.
fn validate(p: Profile, modern: Option<&Configuration>) -> Result<(), Error> {
    match modern {
        Some(m) => m.matches(p),
        None => p.validate_inner(false),
    }
}

/// `sta_req_hdr` for `wcid`, with `muar_idx` as
/// `__mt76_connac_mcu_alloc_sta_req` chooses it.
fn sta_header(p: Profile, wcid: u16, muar: u8) -> Vec<u8> {
    let [low, high] = wcid.to_le_bytes();
    vec![p.interface.bss_index, low, 0, 0, 1, muar, high, 0]
}

/// `mt76_connac_mcu_add_tlv` on a STA_REC request: append the TLV and count
/// it in the header's `tlv_num`.
fn record(out: &mut Vec<u8>, tag: u16, len: usize) -> &mut [u8] {
    let count = u16::from_le_bytes([out[2], out[3]]) + 1;
    put16(out, 2, count);
    tlv(out, tag, len)
}

/// `STA_REC_WTBL` and its `wtbl_req_hdr`; the nested TLVs follow through
/// [`nested`]. Returns the record's offset.
fn wtbl(out: &mut Vec<u8>, wcid: u16, operation: u8) -> usize {
    let at = out.len();
    record(out, STA_REC_WTBL, 4 + 8);
    let [low, high] = wcid.to_le_bytes();
    out[at + 4] = low;
    out[at + 5] = operation;
    out[at + 8] = high;
    at
}

/// `mt76_connac_mcu_add_nested_tlv`: one WTBL TLV, counted in the WTBL
/// header and in the length of the enclosing `STA_REC_WTBL`.
fn nested(out: &mut Vec<u8>, wtbl: usize, tag: u16, len: usize) -> &mut [u8] {
    let count = u16::from_le_bytes([out[wtbl + 6], out[wtbl + 7]]) + 1;
    put16(out, wtbl + 6, count);
    let total = u16::from_le_bytes([out[wtbl + 2], out[wtbl + 3]]) + len as u16;
    put16(out, wtbl + 2, total);
    tlv(out, tag, len)
}

/// `mt76_connac_mcu_wtbl_generic_tlv` and `mt76_connac_mcu_wtbl_hdr_trans_tlv`
/// for a station interface: the AP peer, or with `peer` false the
/// interface's own broadcast entry, which names the BSSID on connac2. RX
/// header translation stays off: the driver takes raw 802.11 frames.
fn generic(out: &mut Vec<u8>, at: usize, p: Profile, peer: Option<bool>) {
    let g = nested(out, at, WTBL_GENERIC, 20);
    g[4..10].copy_from_slice(&p.peer.0);
    match peer {
        Some(qos) => {
            g[10] = p.interface.omac_index;
            g[13] = u8::from(qos);
            put16(g, 16, p.aid);
        }
        None => g[10] = 0xe,
    }
    let rx = nested(out, at, WTBL_RX, 12);
    // rca1: the station's peer is never an AP-side client.
    rx[5] = 1;
    rx[6] = 1;
    rx[7] = 1;
    let h = nested(out, at, WTBL_HDR_TRANS, 8);
    h[4] = 1;
    h[6] = 1;
}

/// `mt7921_mcu_sta_update` for the AP peer: `mt76_connac_mcu_sta_cmd` with
/// the connac2 `mt76_connac_mcu_sta_tlv` records and the WTBL generic, RX,
/// header-translation, HT/VHT and SMPS TLVs. `associated` is
/// `MT76_STA_INFO_STATE_ASSOC`; a disabled peer is the removal request.
pub(in crate::wifi::mediatek) fn peer_request(
    p: Profile,
    modern: Option<&Configuration>,
    enable: bool,
    associated: bool,
) -> Result<Vec<u8>, Error> {
    validate(p, modern)?;
    let n = modern.map(Configuration::negotiated);
    let wme = n.is_some_and(|n| n.wmm.is_some());
    let mut out = sta_header(p, p.wcid, p.interface.omac_index);
    let b = record(&mut out, STA_REC_BASIC, 20);
    put32(b, 4, CONNECTION_INFRA_AP);
    b[8] = if enable { 2 } else { 0 };
    b[9] = u8::from(wme);
    put16(b, 10, p.aid);
    b[12..18].copy_from_slice(&p.peer.0);
    // EXTRA_INFO_VER, and EXTRA_INFO_NEW for a newly connected peer.
    put16(b, 18, if enable && !associated { 3 } else { 1 });
    if enable {
        peer_records(&mut out, p, modern, associated);
    }
    let at = wtbl(&mut out, p.wcid, WTBL_RESET_AND_SET);
    if enable {
        generic(&mut out, at, p, Some(wme));
        if let Some(n) = n {
            let ht_ampdu = n.ht.as_ref().map(|h| {
                let ht = n.peer().ht().map_or(h.capability, |raw| raw.capability);
                (ht, h.peer_ampdu_base_exponent, h.peer_mpdu_spacing)
            });
            let six = n.peer().he_6ghz().filter(|&c| c != 0);
            let mut af_at = None;
            if ht_ampdu.is_some() || six.is_some() {
                let h = nested(&mut out, at, WTBL_HT, 12);
                h[4] = 1;
                match (ht_ampdu, six) {
                    (Some((cap, af, mm)), _) => {
                        h[5] = u8::from(cap & 1 != 0);
                        h[6] = af;
                        h[7] = mm;
                    }
                    (None, Some(c)) => {
                        h[6] = ((c >> 3) & 7) as u8;
                        h[7] = (c & 7) as u8;
                    }
                    (None, None) => unreachable!(),
                }
                af_at = Some(out.len() - 12 + 6);
            }
            if n.vht.is_some() || six.is_some() {
                let cap = n.vht.as_ref().map_or(0, |v| v.capability);
                let v = nested(&mut out, at, WTBL_VHT, 12);
                v[4] = u8::from(cap & 0x10 != 0);
                v[6] = 1;
                let af = ((cap >> 23) & 7) as u8;
                if let Some(af_at) = af_at {
                    out[af_at] = out[af_at].max(af);
                }
            }
            // IEEE80211_SMPS_DYNAMIC from the AP's HT SM power save field.
            let smps = n.peer().ht().is_some_and(|h| (h.capability >> 2) & 3 == 1);
            nested(&mut out, at, WTBL_SMPS, 8)[4] = u8::from(smps);
        } else {
            nested(&mut out, at, WTBL_SMPS, 8);
        }
    }
    Ok(out)
}

/// The connac2 records of `mt76_connac_mcu_sta_tlv`, in its order.
fn peer_records(out: &mut Vec<u8>, p: Profile, modern: Option<&Configuration>, associated: bool) {
    let n = modern.map(Configuration::negotiated);
    let two = p.channel.band == Band::Ghz2;
    if let Some(n) = n {
        if let Some(h) = &n.ht {
            put16(record(out, STA_REC_HT, 8), 4, h.capability);
        }
        if let Some(v) = &n.vht {
            let r = record(out, STA_REC_VHT, 16);
            put32(r, 4, v.capability);
            put16(r, 8, v.mcs.rx);
            put16(r, 10, v.mcs.tx);
        }
        if (n.ht.is_some() || n.he.is_some()) && n.max_tx_amsdu != 0 {
            let a = record(out, STA_REC_HW_AMSDU, 8);
            a[4] = 8;
            a[5] = u8::from(n.max_tx_amsdu >= 7991);
            a[6] = 1;
        }
        if let Some(h) = &n.he {
            let bw160 = modern.is_some_and(|m| m.station_bandwidth() == 3);
            let maps = |r: &mut [u8], at: usize| {
                put16(r, at, h.mcs_80.rx);
                if bw160 {
                    if let Some(m) = h.mcs_160 {
                        put16(r, at + 2, m.rx);
                    }
                    if h.phy[0] & 16 != 0
                        && let Some(m) = h.mcs_80p80
                    {
                        put16(r, at + 4, m.rx);
                    }
                }
            };
            let r = record(out, STA_REC_HE, 28);
            put32(r, 4, he_capability(&h.mac, &h.phy));
            // The HE_MAC and HE_PHY fields u8_get_bits extracts.
            r[8] = (h.mac[1] >> 2) & 3;
            r[9] = (h.mac[3] >> 3) & 3;
            r[10] = (h.phy[0] >> 1) & 0x7f;
            r[11] = (h.phy[1] >> 4) & 1;
            r[12] = h.phy[3] & 3;
            r[13] = (h.phy[3] >> 2) & 1;
            r[14] = (h.phy[3] >> 3) & 3;
            r[15] = (h.phy[3] >> 5) & 1;
            r[16] = (h.phy[8] >> 6) & 3;
            r[17] = h.phy[1] & 0xf;
            r[18] = 2;
            maps(r, 20);
            let r = record(out, STA_REC_HE_V2, 28);
            r[4..10].copy_from_slice(&h.mac);
            r[10..21].copy_from_slice(&h.phy);
            r[21] = 2;
            maps(r, 22);
            if p.channel.band == Band::Ghz6
                && associated
                && let Some(c) = n.peer().he_6ghz()
            {
                put16(record(out, STA_REC_HE_6G, 8), 4, c);
            }
        }
    }
    let phy = record(out, STA_REC_PHY, 12);
    put16(phy, 4, p.basic_rates);
    phy[6] = if two { 3 } else { 8 };
    if let Some(n) = n {
        phy[6] |= u8::from(n.ht.is_some()) << 4
            | u8::from(!two && n.vht.is_some()) << 5
            | u8::from(n.he.is_some()) << 6;
        if let Some(h) = &n.ht {
            phy[7] = (h.peer_ampdu_base_exponent & 3) | (h.peer_mpdu_spacing & 7) << 2;
        }
    }
    phy[9] = RCPI_UNMEASURED;
    let ra = record(out, STA_REC_RA, 16);
    put16(
        ra,
        4,
        if two {
            (p.supported_rates >> 4) << 6 | (p.supported_rates & 15)
        } else {
            p.supported_rates << 6
        },
    );
    if let Some(h) = n.and_then(|n| n.ht.as_ref()) {
        ra[6..16].copy_from_slice(&h.peer_rx_mcs);
    }
    let state = record(out, STA_REC_STATE, 12);
    state[8] = if associated { STATE_ASSOC } else { 0 };
    if let Some(m) = modern.filter(|m| m.negotiated().vht.is_some()) {
        state[9] = m.station_bandwidth() | m.negotiated().tx_nss.saturating_sub(1) << 4;
    }
}

/// `mt76_connac_mcu_sta_he_tlv`'s `STA_REC_HE_CAP_*` word from the HE MAC
/// and PHY capability bytes.
fn he_capability(mac: &[u8; 6], phy: &[u8; 11]) -> u32 {
    // Each IEEE80211_HE_{MAC,PHY}_CAPn flag and its STA_REC_HE_CAP bit.
    let bits: [(bool, u32); 20] = [
        (mac[0] & 0x01 != 0, 0),  // MAC_CAP0_HTC_HE
        (mac[4] & 0x04 != 0, 1),  // MAC_CAP4_BQR
        (mac[2] & 0x08 != 0, 2),  // MAC_CAP2_BSR
        (mac[3] & 0x02 != 0, 3),  // MAC_CAP3_OMI_CONTROL
        (mac[4] & 0x40 != 0, 4),  // MAC_CAP4_AMSDU_IN_AMPDU
        (phy[1] & 0x20 != 0, 6),  // PHY_CAP1_LDPC_CODING_IN_PAYLOAD
        (phy[9] & 0x02 != 0, 7),  // PHY_CAP9_NON_TRIGGERED_CQI_FEEDBACK
        (phy[6] & 0x20 != 0, 8),  // PHY_CAP6_PARTIAL_BW_EXT_RANGE
        (phy[2] & 0x04 != 0, 9),  // PHY_CAP2_STBC_TX_UNDER_80MHZ
        (phy[2] & 0x08 != 0, 10), // PHY_CAP2_STBC_RX_UNDER_80MHZ
        (phy[7] & 0x40 != 0, 11), // PHY_CAP7_STBC_TX_ABOVE_80MHZ
        (phy[7] & 0x80 != 0, 12), // PHY_CAP7_STBC_RX_ABOVE_80MHZ
        (phy[1] & 0x40 != 0, 13), // PHY_CAP1_HE_LTF_AND_GI_FOR_HE_PPDUS_0_8US
        (phy[7] & 0x04 != 0, 14), // PHY_CAP7_HE_SU_MU_PPDU_4XLTF_AND_08_US_GI
        (phy[8] & 0x10 != 0, 15), // PHY_CAP8_HE_ER_SU_1XLTF_AND_08_US_GI
        (phy[8] & 0x01 != 0, 16), // PHY_CAP8_HE_ER_SU_PPDU_4XLTF_AND_08_US_GI
        (phy[2] & 0x02 != 0, 17), // PHY_CAP2_NDP_4x_LTF_AND_3_2US
        (phy[0] & 0x60 != 0, 18), // PHY_CAP0_..._RU_MAPPING_IN_2G or _IN_5G
        (phy[9] & 0x04 != 0, 19), // PHY_CAP9_TX_1024_QAM_LESS_THAN_242_TONE_RU
        (phy[9] & 0x08 != 0, 20), // PHY_CAP9_RX_1024_QAM_LESS_THAN_242_TONE_RU
    ];
    bits.iter()
        .filter(|(set, _)| *set)
        .fold(0, |word, (_, bit)| word | 1 << bit)
}

/// `mt7921_mcu_sta_update` with no station: the interface's own broadcast
/// entry, `mvif->sta`. The firmware offload leaves out its BASIC record, so
/// it carries only the WTBL generic, RX and header-translation TLVs.
pub(in crate::wifi::mediatek) fn broadcast_request(
    p: Profile,
    modern: Option<&Configuration>,
) -> Result<Vec<u8>, Error> {
    validate(p, modern)?;
    let wcid = p.interface.broadcast_wcid;
    let mut out = sta_header(p, wcid, 0xe);
    let at = wtbl(&mut out, wcid, WTBL_RESET_AND_SET);
    generic(&mut out, at, p, None);
    Ok(out)
}

/// `mt76_connac_get_phy_mode` and `mt76_connac_get_phy_mode_v2` over the
/// local capabilities: the BSS's PHY modes. A legacy profile advertises
/// only its legacy rates.
fn local_modes(p: Profile, modern: Option<&Configuration>) -> (u8, u8) {
    let local = modern.map(|m| m.negotiated().local());
    let ht = u8::from(local.is_some_and(|l| l.ht().is_some()));
    let vht = u8::from(local.is_some_and(|l| l.vht().is_some()));
    let he = u8::from(local.is_some_and(|l| l.he().is_some()));
    match p.channel.band {
        Band::Ghz2 => (6 | ht << 3 | he << 6, 3 | ht << 4 | he << 6),
        Band::Ghz5 => (
            1 | ht << 4 | vht << 5 | he << 7,
            8 | ht << 4 | vht << 5 | he << 6,
        ),
        Band::Ghz6 => (0xb1, 8 | ht << 4 | vht << 5 | he << 6),
    }
}

/// `mt76_connac_mcu_uni_add_bss` for a station interface: the BASIC and QBSS
/// request, then HE BASIC with BSS color for an HE BSS, 11V MBSSID for an
/// enabled multiple-BSSID member, and last the RLM request of
/// `mt76_connac_mcu_uni_set_chctx`. Each is one BSS_INFO_UPDATE command.
pub(in crate::wifi::mediatek) fn bss_requests(
    p: Profile,
    modern: Option<&Configuration>,
    enable: bool,
) -> Result<Vec<Vec<u8>>, Error> {
    validate(p, modern)?;
    let c = p.interface;
    let mut requests = Vec::new();
    let mut basic = vec![c.bss_index, 0, 0, 0];
    let b = tlv(&mut basic, UNI_BSS_INFO_BASIC, 32);
    // A station's BSS stays active; `conn_state` follows `enable`.
    b[4] = 1;
    b[5] = c.omac_index;
    b[6] = if c.omac_index > 0x10 { 0 } else { c.omac_index };
    b[7] = c.band_index;
    put32(b, 8, CONNECTION_INFRA_STA);
    b[12] = u8::from(!enable);
    b[13] = c.wmm_index;
    b[14..20].copy_from_slice(&p.peer.0);
    put16(b, 20, c.broadcast_wcid);
    put16(b, 22, p.beacon_interval);
    b[24] = p.dtim_period;
    let (mode, basic_phy) = local_modes(p, modern);
    b[25] = mode;
    put16(b, 26, c.broadcast_wcid);
    put16(b, 28, u16::from(basic_phy));
    b[30] = u8::from(p.channel.band == Band::Ghz6);
    let wmm = modern.is_some_and(|m| m.negotiated().wmm.is_some());
    tlv(&mut basic, UNI_BSS_INFO_QBSS, 8)[4] = u8::from(wmm);
    requests.push(basic);
    if let Some(m) = modern
        && let Some(op) = m.operation().he
    {
        let mut he = vec![c.bss_index, 0, 0, 0];
        let h = tlv(&mut he, UNI_BSS_INFO_HE_BASIC, 16);
        put16(
            h,
            4,
            if op.rts_threshold == 0 {
                1023
            } else {
                op.rts_threshold
            },
        );
        h[6] = if op.packet_extension == 0 {
            4
        } else {
            op.packet_extension
        };
        for (index, maps) in m.local_he_maps().iter().enumerate() {
            put16(h, 8 + index * 2, maps.tx);
        }
        let color = tlv(&mut he, UNI_BSS_INFO_BSS_COLOR, 8);
        if enable {
            color[4] = u8::from(op.color_enabled);
            color[5] = op.color;
        }
        requests.push(he);
    }
    if let Some(op) = modern.map(|m| m.operation())
        && enable
        && op.mbssid_indicator != 0
    {
        let mut mbssid = vec![c.bss_index, 0, 0, 0];
        let t = tlv(&mut mbssid, UNI_BSS_INFO_11V_MBSSID, 8);
        t[4] = op.mbssid_indicator;
        t[5] = op.mbssid_index;
        requests.push(mbssid);
    }
    let width = modern.map_or(Width::Mhz20, |m| m.width());
    let (center, center2) = modern.map_or((p.channel.number as u8, 0), |m| {
        (
            m.operation().center_channel,
            m.operation().second_center_channel,
        )
    });
    requests.push(rlm(p, width, center, center2));
    Ok(requests)
}

/// `mt76_connac_mcu_uni_set_chctx`: the RLM request for the BSS at `width`
/// around `center`, on the profile's control channel.
pub(in crate::wifi::mediatek) fn rlm(p: Profile, width: Width, center: u8, center2: u8) -> Vec<u8> {
    let mut out = vec![p.interface.bss_index, 0, 0, 0];
    let r = tlv(&mut out, UNI_BSS_INFO_RLM, 16);
    let control = p.channel.number as u8;
    r[4] = control;
    r[5] = center;
    r[6] = center2;
    r[7] = match width {
        Width::Mhz20 => 0,
        Width::Mhz40 => 1,
        Width::Mhz80 => 2,
        Width::Mhz160 => 3,
        Width::Mhz80P80 => 6,
        Width::Mhz320 => 7,
    };
    r[8] = p.antenna_mask.count_ones() as u8;
    // `phy->chainmask`, the antenna mask itself on connac2.
    r[9] = p.antenna_mask;
    r[10] = 1;
    r[11] = if width == Width::Mhz20 { 0 } else { 4 };
    r[12] = match control.cmp(&center) {
        core::cmp::Ordering::Less => 1,
        core::cmp::Ordering::Greater => 3,
        core::cmp::Ordering::Equal => 0,
    };
    // enum nl80211_band.
    r[13] = match p.channel.band {
        Band::Ghz2 => 0,
        Band::Ghz5 => 1,
        Band::Ghz6 => 3,
    };
    out
}

/// `mt76_connac_mcu_sta_ba` for the AP peer: the WTBL BA request, then the
/// STA_REC_BA request, both STA_REC_UPDATE commands.
#[allow(clippy::too_many_arguments)]
pub(in crate::wifi::mediatek) fn ba_requests(
    p: Profile,
    tid: u8,
    ssn: u16,
    window: u16,
    amsdu: bool,
    enable: bool,
    transmit: bool,
) -> Result<[Vec<u8>; 2], Error> {
    p.validate_inner(true)?;
    if tid >= 8 || ssn >= 4096 || window == 0 || window > 1024 {
        return Err(Error::InvalidAggregation);
    }
    let mut first = sta_header(p, p.wcid, p.interface.omac_index);
    let at = wtbl(&mut first, p.wcid, WTBL_SET);
    let ba = nested(&mut first, at, WTBL_BA, 28);
    ba[4] = tid;
    if transmit {
        ba[5] = 1;
        if enable {
            put16(ba, 8, ssn);
        }
        ba[10] = u8::from(enable);
    } else {
        ba[5] = 2;
        ba[14..20].copy_from_slice(&p.peer.0);
        ba[20] = tid;
        // rst_ba_sel RST_BA_MAC_TID_MATCH, rst_ba_sb 1.
        ba[22] = 1;
    }
    if enable {
        put16(ba, 12, window);
    }
    let mut second = sta_header(p, p.wcid, p.interface.omac_index);
    let r = record(&mut second, STA_REC_BA, 12);
    r[4] = tid;
    r[5] = if transmit { 1 } else { 2 };
    r[6] = u8::from(amsdu);
    r[7] = u8::from(enable) << tid;
    put16(r, 8, ssn);
    put16(r, 10, window);
    Ok([first, second])
}

#[cfg(test)]
mod tests;
