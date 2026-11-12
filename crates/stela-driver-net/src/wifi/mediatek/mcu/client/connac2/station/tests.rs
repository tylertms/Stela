use super::*;
use crate::{
    Mac,
    wifi::{
        Channel,
        mediatek::mcu::{client::modern::tests::fixture, device::Context},
        station::phy::Mode,
    },
};

fn legacy() -> Profile {
    Profile {
        interface: Context {
            bss_index: 1,
            omac_index: 2,
            band_index: 0,
            wmm_index: 1,
            link_index: 0,
            broadcast_wcid: 18,
            address: Mac([2, 1, 2, 3, 4, 5]),
        },
        peer: Mac([2, 6, 7, 8, 9, 10]),
        wcid: 1,
        channel: Channel {
            band: Band::Ghz5,
            number: 36,
            frequency_mhz: 5180,
        },
        aid: 13,
        basic_rates: 0x15,
        supported_rates: 0xff,
        beacon_interval: 100,
        dtim_period: 2,
        slot_time: 9,
        antenna_mask: 3,
        cipher: 4,
    }
}

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}
fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}
/// Each top-level TLV after an `at`-byte header as (offset, tag, length).
fn tlvs(bytes: &[u8], mut at: usize) -> Vec<(usize, u16, usize)> {
    let mut out = Vec::new();
    while at < bytes.len() {
        let len = u16_at(bytes, at + 2) as usize;
        out.push((at, u16_at(bytes, at), len));
        at += len;
    }
    assert_eq!(at, bytes.len());
    out
}
fn find(bytes: &[u8], header: usize, tag: u16) -> &[u8] {
    let (at, _, len) = tlvs(bytes, header)
        .into_iter()
        .find(|t| t.1 == tag)
        .unwrap();
    &bytes[at..at + len]
}

#[test]
fn a_legacy_peer_lays_out_sta_req_hdr_basic_phy_ra_state_and_wtbl() {
    let p = legacy();
    let out = peer_request(p, None, true, false).unwrap();
    // sta_req_hdr: bss_idx, wlan_idx_lo, tlv_num, is_tlv_append, muar_idx,
    // wlan_idx_hi.
    assert_eq!(out[..8], [1, 1, 5, 0, 1, 2, 0, 0]);
    let tags: Vec<_> = tlvs(&out, 8).iter().map(|t| (t.1, t.2)).collect();
    assert_eq!(
        tags,
        [
            (STA_REC_BASIC, 20),
            (STA_REC_PHY, 12),
            (STA_REC_RA, 16),
            (STA_REC_STATE, 12),
            (STA_REC_WTBL, 4 + 8 + 20 + 12 + 8 + 8),
        ]
    );
    // sta_rec_basic: conn_type, conn_state, qos, aid, peer_addr, extra_info.
    let b = find(&out, 8, STA_REC_BASIC);
    assert_eq!(u32_at(b, 4), 0x10002);
    assert_eq!(b[8], 2);
    assert_eq!(b[9], 0);
    assert_eq!(u16_at(b, 10), 13);
    assert_eq!(b[12..18], p.peer.0);
    assert_eq!(u16_at(b, 18), 3);
    // sta_rec_phy: basic_rate, phy_type, ampdu, rts_policy, rcpi.
    let phy = find(&out, 8, STA_REC_PHY);
    assert_eq!(u16_at(phy, 4), 0x15);
    assert_eq!(phy[6..10], [8, 0, 0, 220]);
    // sta_rec_ra_info: legacy OFDM bits 6..13.
    let ra = find(&out, 8, STA_REC_RA);
    assert_eq!(u16_at(ra, 4), 0xff << 6);
    assert_eq!(ra[6..16], [0; 10]);
    // sta_rec_state: flags, state, vht_opmode.
    let state = find(&out, 8, STA_REC_STATE);
    assert_eq!(u32_at(state, 4), 0);
    assert_eq!(state[8..12], [0; 4]);
    // wtbl_req_hdr: wlan_idx_lo, operation, tlv_num, wlan_idx_hi.
    let w = find(&out, 8, STA_REC_WTBL);
    assert_eq!(w[4..12], [1, WTBL_RESET_AND_SET, 4, 0, 0, 0, 0, 0]);
    let nested: Vec<_> = tlvs(w, 12).iter().map(|t| (t.1, t.2)).collect();
    assert_eq!(
        nested,
        [
            (WTBL_GENERIC, 20),
            (WTBL_RX, 12),
            (WTBL_HDR_TRANS, 8),
            (WTBL_SMPS, 8)
        ]
    );
    // wtbl_generic: peer_addr, muar_idx, skip_tx, cf_ack, qos, mesh, adm,
    // partial_aid, baf_en, aad_om.
    let g = &w[12..32];
    assert_eq!(g[4..10], p.peer.0);
    assert_eq!(g[10..16], [2, 0, 0, 0, 0, 0]);
    assert_eq!(u16_at(g, 16), 13);
    assert_eq!(g[18..20], [0, 0]);
    // wtbl_rx: rcid, rca1, rca2, rv.
    assert_eq!(w[32 + 4..32 + 12], [0, 1, 1, 1, 0, 0, 0, 0]);
    // wtbl_hdr_trans: to_ds, from_ds, no_rx_trans.
    assert_eq!(w[44 + 4..44 + 8], [1, 0, 1, 0]);
    assert_eq!(w[52 + 4..52 + 8], [0; 4]);
}

#[test]
fn an_associated_vht_peer_carries_ht_vht_amsdu_and_the_wtbl_aggregation_limits() {
    let (p, c) = fixture(Band::Ghz5, Mode::Vht, Width::Mhz80);
    let n = c.negotiated();
    let out = peer_request(p, Some(&c), true, true).unwrap();
    let tags: Vec<_> = tlvs(&out, 8).iter().map(|t| (t.1, t.2)).collect();
    assert_eq!(
        tags[..7],
        [
            (STA_REC_BASIC, 20),
            (STA_REC_HT, 8),
            (STA_REC_VHT, 16),
            (STA_REC_HW_AMSDU, 8),
            (STA_REC_PHY, 12),
            (STA_REC_RA, 16),
            (STA_REC_STATE, 12),
        ]
    );
    assert_eq!(out[2], 8);
    let b = find(&out, 8, STA_REC_BASIC);
    assert_eq!((b[9], u16_at(b, 18)), (1, 1));
    let ht = n.ht.as_ref().unwrap();
    assert_eq!(u16_at(find(&out, 8, STA_REC_HT), 4), ht.capability);
    let vht = n.vht.as_ref().unwrap();
    let v = find(&out, 8, STA_REC_VHT);
    assert_eq!(u32_at(v, 4), vht.capability);
    assert_eq!((u16_at(v, 8), u16_at(v, 10)), (vht.mcs.rx, vht.mcs.tx));
    assert_eq!(v[12..16], [0; 4]);
    // sta_rec_amsdu: max_amsdu_num, max_mpdu_size, amsdu_en.
    let a = find(&out, 8, STA_REC_HW_AMSDU);
    assert_eq!(a[4..8], [8, u8::from(n.max_tx_amsdu >= 7991), 1, 0]);
    let phy = find(&out, 8, STA_REC_PHY);
    assert_eq!(phy[6], 8 | 0x10 | 0x20);
    assert_eq!(
        phy[7],
        (ht.peer_ampdu_base_exponent & 3) | (ht.peer_mpdu_spacing & 7) << 2
    );
    assert_eq!(find(&out, 8, STA_REC_RA)[6..16], ht.peer_rx_mcs);
    let state = find(&out, 8, STA_REC_STATE);
    assert_eq!(state[8], 2);
    assert_eq!(state[9], 2 | (n.tx_nss - 1) << 4);
    let w = find(&out, 8, STA_REC_WTBL);
    assert_eq!(w[6], 6);
    let nested = tlvs(w, 12);
    let ht_at = nested.iter().find(|t| t.1 == WTBL_HT).unwrap().0;
    let vht_at = nested.iter().find(|t| t.1 == WTBL_VHT).unwrap().0;
    // wtbl_ht: ht, ldpc, af (the larger of the HT and VHT exponents), mm.
    let vht_af = ((vht.capability >> 23) & 7) as u8;
    assert_eq!(
        w[ht_at + 4..ht_at + 8],
        [
            1,
            u8::from(ht.capability & 1 != 0),
            ht.peer_ampdu_base_exponent.max(vht_af),
            ht.peer_mpdu_spacing
        ]
    );
    // wtbl_vht: ldpc, dyn_bw, vht.
    assert_eq!(
        w[vht_at + 4..vht_at + 7],
        [u8::from(vht.capability & 0x10 != 0), 0, 1]
    );
    assert_eq!(nested.last().unwrap().1, WTBL_SMPS);
}

#[test]
fn an_he_peer_carries_both_he_records_with_the_rx_mcs_maps() {
    let (p, c) = fixture(Band::Ghz5, Mode::He, Width::Mhz160);
    let n = c.negotiated();
    let h = n.he.as_ref().unwrap();
    let out = peer_request(p, Some(&c), true, true).unwrap();
    let he = find(&out, 8, STA_REC_HE);
    assert_eq!(he.len(), 28);
    assert_eq!(u32_at(he, 4), he_capability(&h.mac, &h.phy));
    assert_eq!(he[8], (h.mac[1] >> 2) & 3);
    assert_eq!(he[9], (h.mac[3] >> 3) & 3);
    assert_eq!(he[10], h.phy[0] >> 1);
    assert_eq!(he[17], h.phy[1] & 0xf);
    assert_eq!(he[18], 2);
    assert_eq!(u16_at(he, 20), h.mcs_80.rx);
    assert_eq!(u16_at(he, 22), h.mcs_160.unwrap().rx);
    let v2 = find(&out, 8, STA_REC_HE_V2);
    assert_eq!(v2[4..10], h.mac);
    assert_eq!(v2[10..21], h.phy);
    assert_eq!(v2[21], 2);
    assert_eq!(u16_at(v2, 22), h.mcs_80.rx);
    assert_eq!(u16_at(v2, 24), h.mcs_160.unwrap().rx);
    assert_eq!(find(&out, 8, STA_REC_PHY)[6] & 0x40, 0x40);
}

#[test]
fn he_capability_maps_each_ieee80211_flag_to_its_sta_rec_bit() {
    let flags: [(usize, u8, u32); 20] = [
        (0, 0x01, 0),
        (4, 0x04, 1),
        (2, 0x08, 2),
        (3, 0x02, 3),
        (4, 0x40, 4),
        (6 + 1, 0x20, 6),
        (6 + 9, 0x02, 7),
        (6 + 6, 0x20, 8),
        (6 + 2, 0x04, 9),
        (6 + 2, 0x08, 10),
        (6 + 7, 0x40, 11),
        (6 + 7, 0x80, 12),
        (6 + 1, 0x40, 13),
        (6 + 7, 0x04, 14),
        (6 + 8, 0x10, 15),
        (6 + 8, 0x01, 16),
        (6 + 2, 0x02, 17),
        (6, 0x20, 18),
        (6 + 9, 0x04, 19),
        (6 + 9, 0x08, 20),
    ];
    for (byte, mask, bit) in flags {
        let mut caps = [0u8; 17];
        caps[byte] = mask;
        let (mac, phy) = caps.split_at(6);
        assert_eq!(
            he_capability(mac.try_into().unwrap(), phy.try_into().unwrap()),
            1 << bit
        );
    }
    let mut phy = [0; 11];
    phy[0] = 0x40;
    assert_eq!(he_capability(&[0; 6], &phy), 1 << 18);
}

#[test]
fn a_removed_peer_keeps_basic_and_an_empty_wtbl_reset() {
    let p = legacy();
    let out = peer_request(p, None, false, false).unwrap();
    assert_eq!(out.len(), 8 + 20 + 12);
    assert_eq!(out[2], 2);
    let b = find(&out, 8, STA_REC_BASIC);
    assert_eq!((b[8], u16_at(b, 18)), (0, 1));
    let w = find(&out, 8, STA_REC_WTBL);
    assert_eq!(w[..12], [13, 0, 12, 0, 1, 1, 0, 0, 0, 0, 0, 0]);
}

#[test]
fn the_broadcast_entry_has_only_wtbl_records_naming_the_bssid() {
    let p = legacy();
    let out = broadcast_request(p, None).unwrap();
    assert_eq!(out[..8], [1, 18, 1, 0, 1, 0xe, 0, 0]);
    let w = find(&out, 8, STA_REC_WTBL);
    assert_eq!(u16_at(w, 2) as usize, 4 + 8 + 20 + 12 + 8);
    assert_eq!(w[4..8], [18, WTBL_RESET_AND_SET, 3, 0]);
    let g = &w[12..32];
    assert_eq!(g[4..10], p.peer.0);
    assert_eq!(g[10], 0xe);
    assert_eq!(g[13], 0);
    assert_eq!(u16_at(g, 16), 0);
    assert_eq!(w[32 + 5..32 + 8], [1, 1, 1]);
}

#[test]
fn a_legacy_bss_is_basic_with_qbss_then_rlm() {
    let p = legacy();
    let requests = bss_requests(p, None, true).unwrap();
    assert_eq!(requests.len(), 2);
    let basic = &requests[0];
    assert_eq!(basic[..4], [1, 0, 0, 0]);
    assert_eq!(basic.len(), 4 + 32 + 8);
    // mt76_connac_bss_basic_tlv.
    let b = &basic[4..36];
    assert_eq!(u16_at(b, 0), UNI_BSS_INFO_BASIC);
    assert_eq!(u16_at(b, 2), 32);
    assert_eq!(b[4..8], [1, 2, 2, 0]);
    assert_eq!(u32_at(b, 8), 0x10001);
    assert_eq!((b[12], b[13]), (0, 1));
    assert_eq!(b[14..20], p.peer.0);
    assert_eq!(u16_at(b, 20), 18);
    assert_eq!(u16_at(b, 22), 100);
    assert_eq!((b[24], b[25]), (2, 1));
    assert_eq!(u16_at(b, 26), 18);
    assert_eq!(u16_at(b, 28), 8);
    assert_eq!((b[30], b[31]), (0, 0));
    // mt76_connac_bss_qos_tlv.
    assert_eq!(basic[36..44], [15, 0, 8, 0, 0, 0, 0, 0]);
    // The disabled BSS reports conn_state 1.
    assert_eq!(bss_requests(p, None, false).unwrap()[0][4 + 12], 1);
    // rlm_tlv: control, center, center2, bw, tx_streams, rx_streams,
    // short_st, ht_op_info, sco, band.
    let rlm = &requests[1];
    assert_eq!(rlm.len(), 4 + 16);
    assert_eq!(rlm[..8], [1, 0, 0, 0, 2, 0, 16, 0]);
    assert_eq!(rlm[8..20], [36, 36, 0, 0, 2, 3, 1, 0, 0, 1, 0, 0]);
}

#[test]
fn an_he_bss_adds_he_basic_color_and_mbssid_before_rlm() {
    let (p, c) = fixture(Band::Ghz5, Mode::He, Width::Mhz80);
    let requests = bss_requests(p, Some(&c), true).unwrap();
    assert_eq!(requests.len(), 4);
    let b = &requests[0][4..36];
    assert_eq!(b[25], 1 | 0x10 | 0x20 | 0x80);
    assert_eq!(u16_at(b, 28), 8 | 0x10 | 0x20 | 0x40);
    assert_eq!(requests[0][36 + 4], u8::from(c.negotiated().wmm.is_some()));
    // bss_info_uni_he: he_rts_thres, he_pe_duration, su_disable,
    // max_nss_mcs; then bss_info_uni_bss_color.
    let he = &requests[1];
    assert_eq!(he.len(), 4 + 16 + 8);
    assert_eq!(u16_at(he, 4), UNI_BSS_INFO_HE_BASIC);
    assert_eq!(u16_at(he, 8), 1023);
    assert_eq!((he[10], he[11]), (4, 0));
    for (index, maps) in c.local_he_maps().iter().enumerate() {
        assert_eq!(u16_at(he, 12 + index * 2), maps.tx);
    }
    assert_eq!(he[20..28], [4, 0, 8, 0, 1, 37, 0, 0]);
    // bss_info_uni_mbssid: max_indicator, mbss_idx.
    assert_eq!(requests[2][4..12], [6, 0, 8, 0, 2, 1, 0, 0]);
    // RLM at 80 MHz around 42 from control channel 36: SCA.
    let rlm = &requests[3];
    assert_eq!(rlm[8..20], [36, 42, 0, 2, 2, 3, 1, 4, 1, 1, 0, 0]);
    // A disabled HE BSS clears its color and drops the MBSSID record.
    let disabled = bss_requests(p, Some(&c), false).unwrap();
    assert_eq!(disabled.len(), 3);
    assert_eq!(disabled[1][24..26], [0, 0]);
}

#[test]
fn block_ack_is_a_wtbl_ba_request_then_sta_rec_ba() {
    let p = legacy();
    let [wtbl, record] = ba_requests(p, 5, 100, 64, true, true, true).unwrap();
    assert_eq!(wtbl[..8], [1, 1, 1, 0, 1, 2, 0, 0]);
    let w = &wtbl[8..];
    assert_eq!(u16_at(w, 0), STA_REC_WTBL);
    assert_eq!(u16_at(w, 2), 4 + 8 + 28);
    assert_eq!(w[4..8], [1, WTBL_SET, 1, 0]);
    // wtbl_ba: tid, ba_type, sn, ba_en, ba_winsize_idx, ba_winsize,
    // peer_addr, rst_ba_tid, rst_ba_sel, rst_ba_sb, band_idx.
    let ba = &w[12..40];
    assert_eq!(u16_at(ba, 0), WTBL_BA);
    assert_eq!(u16_at(ba, 2), 28);
    assert_eq!(ba[4..8], [5, 1, 0, 0]);
    assert_eq!(u16_at(ba, 8), 100);
    assert_eq!((ba[10], ba[11]), (1, 0));
    assert_eq!(u16_at(ba, 12), 64);
    assert_eq!(ba[14..28], [0; 14]);
    // sta_rec_ba: tid, ba_type, amsdu, ba_en, ssn, winsize.
    assert_eq!(record[..8], [1, 1, 1, 0, 1, 2, 0, 0]);
    assert_eq!(record[8..], [6, 0, 12, 0, 5, 1, 1, 1 << 5, 100, 0, 64, 0]);
    let [wtbl, record] = ba_requests(p, 2, 7, 32, false, true, false).unwrap();
    let ba = &wtbl[8 + 12..8 + 40];
    assert_eq!(ba[4..6], [2, 2]);
    assert_eq!(u16_at(ba, 8), 0);
    assert_eq!(ba[10], 0);
    assert_eq!(u16_at(ba, 12), 32);
    assert_eq!(ba[14..20], p.peer.0);
    assert_eq!(ba[20..24], [2, 0, 1, 0]);
    assert_eq!(record[8 + 4..], [2, 2, 0, 1 << 2, 7, 0, 32, 0]);
    // A torn-down originator clears its sequence and window.
    let [wtbl, _] = ba_requests(p, 5, 100, 64, false, false, true).unwrap();
    assert_eq!(wtbl[8 + 12 + 8..8 + 12 + 14], [0; 6]);
}
