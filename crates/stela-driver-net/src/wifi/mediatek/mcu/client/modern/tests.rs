use super::super::super::device::Context;
use super::*;
use crate::{
    Mac,
    wifi::{
        Channel,
        station::phy::{Capabilities, McsMaps, Mode, Role},
    },
};
use alloc::{vec, vec::Vec};

fn ie(out: &mut Vec<u8>, id: u8, data: &[u8]) {
    out.extend_from_slice(&[id, data.len() as u8]);
    out.extend_from_slice(data);
}
fn extension(out: &mut Vec<u8>, id: u8, data: &[u8]) {
    let mut extended = vec![id];
    extended.extend_from_slice(data);
    ie(out, 255, &extended);
}
fn ht() -> [u8; 26] {
    let mut data = [0; 26];
    data[..2].copy_from_slice(&0x19e3_u16.to_le_bytes());
    data[2] = 3 | (4 << 2);
    data[3..5].fill(0xff);
    data[7] = 1;
    data[15] = 1;
    data
}
fn vht() -> [u8; 12] {
    let mut data = [0; 12];
    data[..4].copy_from_slice(&(1_u32 | 8 | 0x60 | 0x180 | (3 << 23)).to_le_bytes());
    data[4..6].copy_from_slice(&0xfffa_u16.to_le_bytes());
    data[8..10].copy_from_slice(&0xfffa_u16.to_le_bytes());
    data
}
fn he(width_bits: u8) -> Vec<u8> {
    let mut data = vec![0; 17];
    data[6] = width_bits;
    for _ in 0..1 + usize::from(width_bits & 8 != 0) + usize::from(width_bits & 16 != 0) {
        data.extend_from_slice(&[0xfa, 0xff, 0xfa, 0xff]);
    }
    data
}
fn eht(width_bits: u8, role: Role, width320: bool) -> Vec<u8> {
    let mut data = vec![0; 11];
    data[0] = 1 << 6; // 2.4GHz max-MPDU selection; ignored elsewhere.
    data[2] = if width320 { 2 } else { 0 };
    if width_bits & 2 != 0 {
        data.extend_from_slice(&[0x22, 0x22, 0x11]);
    } else {
        for present in [width_bits & 4 != 0, width_bits & 8 != 0, width320] {
            if present {
                data.extend_from_slice(&[0x22, 0x22, 0x11]);
            }
        }
        if data.len() == 11 {
            data.extend_from_slice(if role == Role::Station {
                &[0x22, 0x22, 0x22, 0x11]
            } else {
                &[0x22, 0x22, 0x11]
            });
        }
    }
    data
}
fn wmm(role: Role) -> Vec<u8> {
    if role == Role::Station {
        return vec![0, 0x50, 0xf2, 2, 0, 1, 0x6f];
    }
    vec![
        0, 0x50, 0xf2, 2, 1, 1, 0x89, 0, 3, 0xa4, 0, 0, 0x27, 0xa4, 0, 0, 0x42, 0x43, 94, 0, 0x62,
        0x32, 47, 0,
    ]
}
fn modern_ies(band: Band, role: Role, width_bits: u8, width320: bool) -> Vec<u8> {
    let mut out = Vec::new();
    if band == Band::Ghz2 {
        ie(&mut out, 1, &[0x82, 0x84, 11, 22, 0x8c, 0x98, 0xb0, 108]);
        ie(&mut out, 50, &[18, 36, 72, 96]);
    } else if band != Band::Ghz6 {
        ie(&mut out, 1, &[0x8c, 18, 0x98, 36, 0xb0, 72, 96, 108]);
    }
    if band != Band::Ghz6 {
        ie(&mut out, 45, &ht());
        if band == Band::Ghz5 {
            ie(&mut out, 191, &vht());
        }
    }
    extension(&mut out, 35, &he(width_bits));
    extension(&mut out, 108, &eht(width_bits, role, width320));
    if band == Band::Ghz6 {
        extension(&mut out, 59, &0x004a_u16.to_le_bytes());
    }
    ie(&mut out, 221, &wmm(role));
    out
}
fn pair(band: Band, width_bits: u8, width320: bool) -> (Capabilities, Capabilities) {
    (
        Capabilities::parse(
            band,
            Role::Station,
            &modern_ies(band, Role::Station, width_bits, width320),
        )
        .unwrap(),
        Capabilities::parse(
            band,
            Role::AccessPoint,
            &modern_ies(band, Role::AccessPoint, width_bits, width320),
        )
        .unwrap(),
    )
}

pub(in crate::wifi::mediatek::mcu::client) fn fixture(
    band: Band,
    mode: Mode,
    width: Width,
) -> (Profile, Configuration) {
    let bits = if band == Band::Ghz2 { 2 } else { 4 | 8 | 16 };
    let (local, peer) = pair(band, bits, band == Band::Ghz6);
    let n = Negotiated::between(&local, &peer, mode, width).unwrap();
    let p = Profile {
        interface: Context {
            bss_index: 1,
            omac_index: 2,
            band_index: 255,
            wmm_index: 1,
            link_index: 0,
            broadcast_wcid: 18,
            address: Mac([2, 1, 2, 3, 4, 5]),
        },
        peer: Mac([2, 6, 7, 8, 9, 10]),
        wcid: 1,
        channel: Channel {
            band,
            number: if band == Band::Ghz2 {
                6
            } else if band == Band::Ghz5 {
                36
            } else {
                1
            },
            frequency_mhz: if band == Band::Ghz2 {
                2437
            } else if band == Band::Ghz5 {
                5180
            } else {
                5955
            },
        },
        aid: 13,
        basic_rates: if band == Band::Ghz6 {
            0x15
        } else {
            n.legacy.basic
        },
        supported_rates: if band == Band::Ghz6 {
            0xff
        } else {
            n.legacy.supported
        },
        beacon_interval: 100,
        dtim_period: 2,
        slot_time: 9,
        antenna_mask: 3,
        cipher: 4,
    };
    let center = p.channel.number as u8
        + match width {
            Width::Mhz20 => 0,
            Width::Mhz40 => 2,
            Width::Mhz80 | Width::Mhz80P80 => 6,
            Width::Mhz160 => 14,
            Width::Mhz320 => 30,
        };
    let local_he_maps = n.local().he().map_or([McsMaps { rx: 0, tx: 0 }; 3], |h| {
        [
            h.mcs_80,
            h.mcs_160.unwrap_or(h.mcs_80),
            h.mcs_80p80.unwrap_or(McsMaps { rx: 0, tx: 0 }),
        ]
    });
    let c = Configuration::new(
        p,
        n,
        Operation {
            center_channel: center,
            second_center_channel: if width == Width::Mhz80P80 { 106 } else { 0 },
            punctured: 0,
            he: if matches!(mode, Mode::He | Mode::Eht) {
                Some(HeOperation {
                    packet_extension: 0,
                    rts_threshold: 0,
                    color_enabled: true,
                    color: 37,
                })
            } else {
                None
            },
            mbssid_indicator: 2,
            mbssid_index: 1,
        },
        local_he_maps,
    )
    .unwrap();
    (p, c)
}
fn field(bytes: &[u8], mut at: usize, wanted: u16) -> &[u8] {
    while at < bytes.len() {
        let tag = u16::from_le_bytes(bytes[at..at + 2].try_into().unwrap());
        let len = u16::from_le_bytes(bytes[at + 2..at + 4].try_into().unwrap()) as usize;
        if tag == wanted {
            return &bytes[at..at + len];
        }
        at += len;
    }
    panic!("missing tag{wanted}")
}
#[test]
fn modern_records_keep_direction_and_source_order() {
    for band in [Band::Ghz2, Band::Ghz5, Band::Ghz6] {
        for mode in [Mode::Ht, Mode::Vht, Mode::He, Mode::Eht] {
            if (band == Band::Ghz2 && mode == Mode::Vht)
                || (band == Band::Ghz6 && !matches!(mode, Mode::He | Mode::Eht))
            {
                continue;
            }
            let (p, c) = fixture(band, mode, Width::Mhz20);
            let peer = c.peer_request(p, true).unwrap();
            let bss = c.bss_request(p, true).unwrap();
            assert_eq!(field(&peer, 8, 0)[9], 1);
            let phy = field(&peer, 8, 0x15);
            assert_eq!(phy[10], c.negotiated.peer_ampdu_base_exponent);
            if matches!(mode, Mode::He | Mode::Eht) {
                let he = field(&peer, 8, 0x19);
                assert_eq!(&he[4..10], &c.negotiated.he.as_ref().unwrap().mac);
                assert_eq!(he[21], 2);
                let he_bss = field(&bss, 4, 5);
                assert_eq!(&he_bss[4..8], &[255, 3, 4, 0]);
                assert_eq!(
                    &he_bss[8..10],
                    &c.negotiated.local().he().unwrap().mcs_80.tx.to_le_bytes()
                );
                assert_eq!(&field(&bss, 4, 4)[4..6], &[1, 37]);
            }
            assert_eq!(&field(&bss, 4, 6)[4..6], &[2, 1]);
            if band == Band::Ghz6 {
                assert_eq!(&field(&peer, 8, 10)[4..12], &[0; 8]);
                assert_eq!(field(&bss, 4, 0)[25], 0xb1);
                assert_eq!(field(&bss, 4, 2)[12], 3);
            }
            assert_eq!(&field(&peer, 8, 0x2b)[5..7], &[1, 1]);
        }
    }
}

#[test]
fn bss_he_keeps_source_hidden_maps_without_advertising_their_widths() {
    let (p, c) = fixture(Band::Ghz2, Mode::He, Width::Mhz20);
    let wire = c.negotiated.local().he().unwrap();
    assert_eq!(wire.mcs_160, None);
    assert_eq!(wire.mcs_80p80, None);
    let body = c.bss_request(p, true).unwrap();
    let he = field(&body, 4, 5);
    // Full source initialization writes160 on2GHz, while source
    // zero-allocation leaves80+80 untouched. Wire absence is not either value.
    assert_eq!(&he[8..10], &wire.mcs_80.tx.to_le_bytes());
    assert_eq!(&he[10..12], &wire.mcs_80.tx.to_le_bytes());
    assert_eq!(&he[12..14], &[0, 0]);
    let mut wrong = c.local_he_maps;
    wrong[0].tx ^= 1;
    assert_eq!(
        Configuration::new(p, c.negotiated.clone(), c.operation, wrong),
        Err(Error::InvalidNegotiation)
    );
}
#[test]
fn every_supported_width_encodes_centers_and_distinct_command_bandwidth() {
    for (band, mode) in [
        (Band::Ghz2, Mode::Ht),
        (Band::Ghz5, Mode::He),
        (Band::Ghz6, Mode::Eht),
    ] {
        for width in [
            Width::Mhz20,
            Width::Mhz40,
            Width::Mhz80,
            Width::Mhz160,
            Width::Mhz80P80,
            Width::Mhz320,
        ] {
            if matches!(width, Width::Mhz80P80 | Width::Mhz320) {
                continue;
            }
            if (band == Band::Ghz2 && !matches!(width, Width::Mhz20 | Width::Mhz40))
                || (width == Width::Mhz320 && band != Band::Ghz6)
                || (width == Width::Mhz80P80 && mode == Mode::Eht)
            {
                continue;
            }
            let (p, c) = fixture(band, mode, width);
            let b = c.bss_request(p, true).unwrap();
            let r = field(&b, 4, 2);
            assert_eq!(r[4], p.channel.number as u8);
            assert_eq!(r[5], c.operation.center_channel);
            assert_eq!(r[6], c.operation.second_center_channel);
            assert_eq!(
                r[7],
                match width {
                    Width::Mhz20 => 0,
                    Width::Mhz40 => 1,
                    Width::Mhz80 => 2,
                    Width::Mhz160 => 3,
                    Width::Mhz80P80 => 6,
                    Width::Mhz320 => 7,
                }
            );
        }
    }
}
#[test]
fn edca_wmm_aci_mapping_uses_firmware_queue_order_and_full_parameter_updates() {
    let (p, c) = fixture(Band::Ghz5, Mode::He, Width::Mhz80);
    let b = c.edca_request(p).unwrap();
    assert_eq!(b.len(), 52);
    let expected = [
        (0, 2, 2, 3, 47),
        (1, 2, 3, 4, 94),
        (2, 3, 4, 10, 0),
        (3, 7, 4, 10, 0),
    ];
    for (i, (queue, aifs, min, max, txop)) in expected.into_iter().enumerate() {
        let e = &b[4 + i * 12..4 + (i + 1) * 12];
        assert_eq!((e[4], e[5], e[6], e[7], e[10]), (queue, 15, min, max, aifs));
        assert_eq!(u16::from_le_bytes([e[8], e[9]]), txop);
    }
}
#[test]
fn tampered_negotiation_wrong_antenna_primary_puncture_and_stale_resources_refuse() {
    let (p, c) = fixture(Band::Ghz5, Mode::He, Width::Mhz80);
    let mut n = c.negotiated.clone();
    n.rx_nss = 1;
    assert_eq!(
        Configuration::new(p, n, c.operation, c.local_he_maps),
        Err(Error::InvalidNegotiation)
    );
    let mut p2 = p;
    p2.antenna_mask = 1;
    assert!(c.peer_request(p2, true).is_err());
    let mut op = c.operation;
    op.punctured = 1;
    assert_eq!(
        Configuration::new(p, c.negotiated.clone(), op, c.local_he_maps),
        Err(Error::InvalidGeometry)
    );
    op = c.operation;
    op.second_center_channel = 106;
    assert_eq!(
        Configuration::new(p, c.negotiated.clone(), op, c.local_he_maps),
        Err(Error::InvalidGeometry)
    );
    p2 = p;
    p2.basic_rates = 1;
    assert!(c.bss_request(p2, true).is_err());
}
#[test]
fn ba_encodes_real_recipient_originator_tid_window_and_source_bitmap() {
    let (p, _) = fixture(Band::Ghz5, Mode::He, Width::Mhz80);
    for tid in 0..8 {
        for transmit in [false, true] {
            let b = ba_request(p, tid, 4095, 1024, true, true, transmit).unwrap();
            assert_eq!(
                &b[12..16],
                &[tid, if transmit { 1 } else { 2 }, 1, 1 << tid]
            );
            assert_eq!(&b[16..20], &[255, 15, 0, 4]);
        }
    }
    assert_eq!(
        ba_request(p, 8, 0, 64, false, true, false),
        Err(Error::InvalidAggregation)
    );
    assert_eq!(
        ba_request(p, 0, 4096, 64, false, true, false),
        Err(Error::InvalidAggregation)
    );
}

#[test]
#[ignore = "oracle: STELA_MT7925_MODERN_ORACLE"]
fn full_source_modern_peer_bss_edca_ba_and_puncture_match_exact_requests() {
    extern crate std;
    use std::{
        io::Write,
        process::{Command, Stdio},
    };
    let root = std::path::PathBuf::from(std::env::var_os("STELA_MT7925_MODERN_ORACLE").unwrap());
    let mut input = Vec::new();
    let mut expected = Vec::new();
    let mut count = 0;
    for band in [Band::Ghz2, Band::Ghz5, Band::Ghz6] {
        for mode in [Mode::Ht, Mode::Vht, Mode::He, Mode::Eht] {
            if (band == Band::Ghz2 && mode == Mode::Vht)
                || (band == Band::Ghz6 && !matches!(mode, Mode::He | Mode::Eht))
            {
                continue;
            }
            for width in [
                Width::Mhz20,
                Width::Mhz40,
                Width::Mhz80,
                Width::Mhz160,
                Width::Mhz80P80,
                Width::Mhz320,
            ] {
                if matches!(width, Width::Mhz80P80 | Width::Mhz320) {
                    continue;
                }
                if (band == Band::Ghz2 && !matches!(width, Width::Mhz20 | Width::Mhz40))
                    || (mode == Mode::Ht && !matches!(width, Width::Mhz20 | Width::Mhz40))
                    || (width == Width::Mhz320 && (mode != Mode::Eht || band != Band::Ghz6))
                    || (width == Width::Mhz80P80 && mode == Mode::Eht)
                {
                    continue;
                }
                for variant in 0..8 {
                    let (mut p, c) = fixture(band, mode, width);
                    p.interface.omac_index = [0, 1, 17, 63][variant % 4];
                    p.interface.wmm_index = (variant % 4) as u8;
                    p.antenna_mask = [3, 5, 9, 15][variant % 4];
                    p.cipher = [0, 4, 10, 11, 12, 4, 11, 12][variant];
                    let n = c.negotiated();
                    let op = c.operation();
                    let flags = u8::from(n.ht.is_some())
                        | u8::from(n.vht.is_some()) << 1
                        | u8::from(n.he.is_some()) << 2
                        | u8::from(n.eht.is_some()) << 3;
                    input.extend_from_slice(&[
                        match band {
                            Band::Ghz2 => 0,
                            Band::Ghz5 => 1,
                            Band::Ghz6 => 3,
                        },
                        p.channel.number as u8,
                        op.center_channel,
                        op.second_center_channel,
                        match width {
                            Width::Mhz20 => 0,
                            Width::Mhz40 => 2,
                            Width::Mhz80 => 3,
                            Width::Mhz80P80 => 4,
                            Width::Mhz160 => 5,
                            Width::Mhz320 => 6,
                        },
                        c.station_bandwidth(),
                    ]);
                    input.extend_from_slice(&p.interface.address.0);
                    input.extend_from_slice(&p.peer.0);
                    input.extend_from_slice(&[
                        p.interface.bss_index,
                        p.interface.omac_index,
                        p.interface.band_index,
                        p.interface.wmm_index,
                    ]);
                    for v in [
                        p.wcid,
                        p.interface.broadcast_wcid,
                        p.aid,
                        p.basic_rates,
                        p.supported_rates,
                        p.beacon_interval,
                    ] {
                        input.extend_from_slice(&v.to_le_bytes())
                    }
                    input.push(p.dtim_period);
                    input.extend_from_slice(&p.slot_time.to_le_bytes());
                    input.extend_from_slice(&[p.antenna_mask, p.cipher, flags]);
                    input.extend_from_slice(
                        &n.ht.as_ref().map_or(0, |h| h.capability).to_le_bytes(),
                    );
                    input.extend_from_slice(&[
                        n.ht.as_ref().map_or(0, |h| h.peer_ampdu_base_exponent),
                        n.ht.as_ref().map_or(0, |h| h.peer_mpdu_spacing),
                    ]);
                    input.extend_from_slice(&n.ht.as_ref().map_or([0; 10], |h| h.peer_rx_mcs));
                    input.extend_from_slice(
                        &n.vht.as_ref().map_or(0, |v| v.capability).to_le_bytes(),
                    );
                    for v in [
                        n.vht.as_ref().map_or(0, |v| v.mcs.rx),
                        n.vht.as_ref().map_or(0, |v| v.mcs.tx),
                    ] {
                        input.extend_from_slice(&v.to_le_bytes())
                    }
                    input.extend_from_slice(&n.he.as_ref().map_or([0; 6], |h| h.mac));
                    input.extend_from_slice(&n.he.as_ref().map_or([0; 11], |h| h.phy));
                    let maps =
                        n.he.as_ref()
                            .map(|h| [Some(h.mcs_80), h.mcs_160, h.mcs_80p80])
                            .unwrap_or([None; 3]);
                    for m in maps {
                        for v in [m.map_or(0, |m| m.rx), m.map_or(0, |m| m.tx)] {
                            input.extend_from_slice(&v.to_le_bytes())
                        }
                    }
                    input.extend_from_slice(&n.peer().he_6ghz().unwrap_or(0).to_le_bytes());
                    if let Some(e) = n.peer().eht().filter(|_| n.eht.is_some()) {
                        input.extend_from_slice(&e.mac);
                        input.extend_from_slice(&e.phy);
                        input.extend_from_slice(e.source_mcs_union().bytes());
                    } else {
                        input.extend_from_slice(&[0; 20])
                    }
                    input.push(n.tx_nss);
                    input.extend_from_slice(&n.max_tx_amsdu.to_le_bytes());
                    let local = n.local().he().filter(|_| n.he.is_some());
                    input.extend_from_slice(&local.map_or([0; 6], |h| h.mac));
                    input.extend_from_slice(&local.map_or([0; 11], |h| h.phy));
                    for m in c.local_he_maps {
                        for v in [m.rx, m.tx] {
                            input.extend_from_slice(&v.to_le_bytes())
                        }
                    }
                    input.push(op.he.map_or(0, |h| h.packet_extension));
                    input.extend_from_slice(&op.he.map_or(0, |h| h.rts_threshold).to_le_bytes());
                    input.extend_from_slice(&[
                        u8::from(op.he.is_some_and(|h| h.color_enabled)),
                        op.he.map_or(0, |h| h.color),
                        op.mbssid_indicator,
                        op.mbssid_index,
                    ]);
                    input.extend_from_slice(&op.punctured.to_le_bytes());
                    for aci in [3, 2, 0, 1] {
                        let ac = n
                            .wmm
                            .as_ref()
                            .unwrap()
                            .access_categories
                            .iter()
                            .find(|a| a.aci == aci)
                            .unwrap();
                        input.extend_from_slice(&[ac.aifsn, ac.ecw_min, ac.ecw_max]);
                        input.extend_from_slice(&ac.txop_32us.to_le_bytes())
                    }
                    let tid = variant as u8;
                    let ssn = (count * 31 % 4096) as u16;
                    let window = [1, 64, 256, 1024][variant % 4];
                    let amsdu = variant & 1 != 0;
                    let enable = variant & 2 != 0;
                    let transmit = variant & 4 != 0;
                    input.push(tid);
                    input.extend_from_slice(&ssn.to_le_bytes());
                    input.extend_from_slice(&u16::to_le_bytes(window));
                    input.extend_from_slice(&[
                        u8::from(amsdu),
                        u8::from(enable),
                        u8::from(transmit),
                    ]);
                    expected.extend([
                        (3, c.peer_request(p, false).unwrap()),
                        (3, c.peer_request(p, true).unwrap()),
                        (2, c.bss_request(p, false).unwrap()),
                        (2, c.bss_request(p, true).unwrap()),
                        (4, c.edca_request(p).unwrap()),
                        (
                            3,
                            ba_request(p, tid, ssn, window, amsdu, enable, transmit).unwrap(),
                        ),
                    ]);
                    if n.eht.is_some() {
                        expected.push((2, c.puncture_request(p).unwrap()))
                    }
                    count += 1;
                }
            }
        }
    }
    std::fs::write(root.join("modern.input"), &input).unwrap();
    let mut child = Command::new(root.join("client"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let writer = std::thread::spawn(move || stdin.write_all(&input));
    let result = child.wait_with_output().unwrap();
    writer.join().unwrap().unwrap();
    assert!(result.status.success());
    std::fs::write(root.join("modern.trace"), &result.stdout).unwrap();
    let mut at = 0;
    for (index, (command, body)) in expected.iter().enumerate() {
        let trace = &result.stdout;
        assert_eq!(trace[at], *command, "record{index}");
        let n = u16::from_le_bytes(trace[at + 1..at + 3].try_into().unwrap()) as usize;
        at += 3;
        assert_eq!(body, &trace[at..at + n], "record{index}");
        at += n;
    }
    assert_eq!(at, result.stdout.len());
    use sha2::{Digest, Sha256};
    let input_hash = Sha256::digest(std::fs::read(root.join("modern.input")).unwrap());
    let trace_hash = Sha256::digest(&result.stdout);
    std::fs::write(
            root.join("modern-run.json"),
            std::format!(
                "{{\n  \"contexts\": {count},\n  \"requests\": {},\n  \"input_sha256\": \"{input_hash:x}\",\n  \"trace_sha256\": \"{trace_hash:x}\"\n}}\n",
                expected.len()
            ),
        )
        .unwrap();
    std::println!(
        "{count} actual modern contexts, {} complete unchanged source requests",
        expected.len()
    );
}

/// A station MLD's secondary link on BSS 1 with AP WCID 2, its primary link
/// being BSS 0 with AP WCID 1.
fn link(eml: Option<u16>) -> MldLink {
    MldLink {
        link_id: 1,
        local: Mac([2, 0, 0, 0, 0, 0xd]),
        ap: Mac([2, 0, 0, 0, 0, 0xa]),
        primary_bss: 0,
        primary_wcid: 1,
        eml,
    }
}

/// `mt7925_mcu_sta_mld_tlv` and `mt7925_mcu_sta_eht_mld_tlv` for the
/// secondary link, `mt7925_mcu_bss_mld_tlv` for its BSS.
#[test]
fn a_secondary_link_names_the_primary_and_its_own_records() {
    let (mut p, c) = fixture(Band::Ghz5, Mode::Eht, Width::Mhz20);
    p.wcid = 2;
    p.interface.bss_index = 1;
    let link = link(None);
    let peer = c.clone().with_link(link).peer_request(p, true).unwrap();
    let mld = field(&peer, 8, 0x20);
    assert_eq!(mld[4..10], link.ap.0);
    // Primary 1, secondary 2, WLAN 1; two links: WCID 1 on BSS 0, WCID 2
    // on BSS 1.
    assert_eq!(mld[10..18], [1, 0, 2, 0, 1, 0, 2, 0]);
    assert_eq!(mld[20..28], [1, 0, 0, 0, 2, 0, 1, 0]);
    let eht = field(&peer, 8, 0x21);
    assert_eq!((eht[5], eht[7], &eht[10..12]), (0xff, 2, &[0, 0][..]));
    let eml = c
        .clone()
        .with_link(MldLink {
            eml: Some(0x0081),
            ..link
        })
        .peer_request(p, true)
        .unwrap();
    let eht = field(&eml, 8, 0x21);
    assert_eq!((eht[7], &eht[10..12]), (0, &[0x81, 0][..]));
    let bss = c.clone().with_link(link).bss_request(p, true).unwrap();
    let m = field(&bss, 4, 26);
    assert_eq!((m[4], m[5], m[13], m[14]), (0, 33, 1, 0));
    assert_eq!(m[6..12], link.local.0);
    // The single-link records keep naming the AP and the interface.
    let single = field(&c.peer_request(p, true).unwrap(), 8, 0x20).to_vec();
    assert_eq!((single[16], &single[4..10]), (1, &p.peer.0[..]));
}

/// `mt7925_mcu_set_mlo_roc`: the setup link acquires, the other joins as a
/// sub-link, both MLSR at 20 MHz.
#[test]
fn an_mlo_channel_request_holds_both_links() {
    let (p, _) = fixture(Band::Ghz5, Mode::Eht, Width::Mhz20);
    let mut other = p;
    other.interface.bss_index = p.interface.bss_index + 1;
    other.channel = Channel {
        band: Band::Ghz2,
        number: 6,
        frequency_mhz: 2437,
    };
    let request = p.mlo_roc(other, 7, 5).unwrap();
    let bytes = request.body();
    assert_eq!(bytes.len(), 4 + 2 * 24);
    let first = &bytes[4..28];
    let second = &bytes[28..52];
    let bss = p.interface.bss_index;
    assert_eq!(first[..6], [0, 0, 24, 0, bss, 7]);
    assert_eq!(second[..6], [3, 0, 24, 0, bss + 1, 7]);
    assert_eq!((second[6], second[8], second[10], second[13]), (6, 1, 6, 6));
    // A 2.4 GHz link makes it MT7925_ROC_REQ_MLSR_AG.
    assert_eq!(
        (first[15], second[15], &second[16..21]),
        (10, 10, &[5, 0, 0, 0, 255][..])
    );
    other.channel = p.channel;
    let same = p.mlo_roc(other, 7, 5).unwrap();
    assert_eq!(same.body()[4 + 15], 11);
}
