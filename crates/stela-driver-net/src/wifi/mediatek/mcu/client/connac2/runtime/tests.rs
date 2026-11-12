use super::*;
use crate::{
    Mac,
    wifi::{Channel, mediatek::mcu::device::Context},
};

fn profile(band: Band, number: u16, frequency_mhz: u16) -> Profile {
    Profile {
        interface: Context {
            bss_index: 1,
            omac_index: 2,
            band_index: 0,
            wmm_index: 3,
            link_index: 0,
            broadcast_wcid: 18,
            address: Mac([2, 1, 2, 3, 4, 5]),
        },
        peer: Mac([2, 6, 7, 8, 9, 10]),
        wcid: 1,
        channel: Channel {
            band,
            number,
            frequency_mhz,
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

#[test]
fn roc_and_its_abort_lay_out_the_mt7921_roc_tlvs_unacknowledged() {
    for (chip, band, number, frequency, code) in [
        (Chip::Mt7921, Band::Ghz2, 6, 2437, 1),
        (Chip::Mt7922, Band::Ghz5, 149, 5745, 2),
        (Chip::Mt7922, Band::Ghz6, 37, 6135, 3),
    ] {
        let p = profile(band, number, frequency);
        let r = roc(chip, p, 7, 1000).unwrap();
        assert_eq!(r.chip(), chip);
        assert_eq!(r.command(), (1 << 17) | 0x27);
        assert!(!r.expects_response());
        let b = r.body();
        assert_eq!(b.len(), 4 + 24);
        // hdr rsv[4]; roc_acquire_tlv: tag, len, bss_idx, tokenid,
        // control_channel, sco, band, bw, center_chan, center_chan2,
        // bw_from_ap, center_chan_from_ap, center_chan2_from_ap, reqtype,
        // maxinterval, dbdcband.
        let n = number as u8;
        assert_eq!(b[..8], [0, 0, 0, 0, 0, 0, 24, 0]);
        assert_eq!(b[8..20], [1, 7, n, 0, code, 0, n, 0, 0, n, 0, 0]);
        assert_eq!(b[20..24], 1000u32.to_le_bytes());
        assert_eq!(b[24..28], [0xff, 0, 0, 0]);
        let a = abort_roc(chip, p, 7).unwrap();
        assert!(!a.expects_response());
        // roc_abort_tlv: tag UNI_ROC_ABORT, len, bss_idx, tokenid, dbdcband.
        assert_eq!(
            a.body(),
            [0, 0, 0, 0, 1, 0, 12, 0, 1, 7, 0xff, 0, 0, 0, 0, 0]
        );
    }
    let p = profile(Band::Ghz2, 6, 2437);
    assert!(matches!(
        roc(Chip::Mt7921, p, 1, 0),
        Err(Error::InvalidTiming)
    ));
}

#[test]
fn the_beacon_filter_is_bcnft_then_rx_filter_and_its_end_is_bss_abort() {
    let p = profile(Band::Ghz5, 36, 5180);
    // bcnft_tlv: bcn_interval, dtim_period.
    assert_eq!(
        beacon_filter(p).unwrap(),
        [1, 0, 0, 0, 22, 0, 8, 0, 100, 0, 2, 0]
    );
    for (enable, op) in [(true, 1), (false, 2)] {
        let r = filter(Chip::Mt7921, p, enable).unwrap();
        assert_eq!(r.command(), (1 << 18) | 0x0a);
        assert_eq!(r.kind(), Kind::ClientFilter);
        assert!(!r.expects_response());
        let b = r.body();
        // rsv[4], mode, rsv2[3], fif, bit_map, bit_op, pad[51].
        assert_eq!(b.len(), 68);
        assert_eq!(b[..8], [0, 0, 0, 0, 2, 0, 0, 0]);
        assert_eq!(b[8..12], [0; 4]);
        assert_eq!(b[12..16], (1u32 << 11).to_le_bytes());
        assert_eq!(b[16], op);
        assert!(b[17..].iter().all(|&byte| byte == 0));
    }
    let a = bss_abort(Chip::Mt7922, p).unwrap();
    assert_eq!(a.command(), (1 << 18) | 0x17);
    assert_eq!(a.kind(), Kind::BssAbort);
    assert!(!a.expects_response());
    assert_eq!(a.body(), [1, 0, 0, 0]);
}

#[test]
fn the_rssi_monitor_names_its_thresholds_and_bss() {
    let p = profile(Band::Ghz5, 36, 5180);
    let r = rssi_monitor(Chip::Mt7921, p, true, -60, -70).unwrap();
    assert_eq!(r.command(), (1 << 18) | 0xa1);
    assert_eq!(r.kind(), Kind::RssiMonitor);
    assert!(!r.expects_response());
    // enable, cqm_rssi_high, cqm_rssi_low, bss_idx, duration, rsv2.
    assert_eq!(r.body(), [1, (-60i8) as u8, (-70i8) as u8, 1, 0, 0, 0, 0]);
}

#[test]
fn edca_takes_contention_windows_in_the_to_aci_slots() {
    let p = profile(Band::Ghz5, 36, 5180);
    let ac = |aci, ecw_min, ecw_max, aifsn, txop_32us| AccessCategory {
        aci,
        aifsn,
        admission_control: false,
        ecw_min,
        ecw_max,
        txop_32us,
    };
    // BE, BK, VI, VO in WMM order; BK's zero windows take 5 and 10.
    let effective = [
        ac(0, 4, 10, 3, 0),
        ac(1, 0, 0, 7, 0),
        ac(2, 3, 4, 2, 94),
        ac(3, 2, 3, 2, 47),
    ];
    let r = edca(Chip::Mt7922, p, &effective, true).unwrap();
    assert_eq!(r.command(), (1 << 18) | 0x1d);
    assert_eq!(r.kind(), Kind::Edca);
    assert!(!r.expects_response());
    let b = r.body();
    assert_eq!(b.len(), 44);
    // edca[4] of cw_min, cw_max, txop, aifs, guardtime, acm: VI, VO, BE, BK.
    let slot = |cw_min: u16, cw_max: u16, txop: u16, aifs: u16| {
        [
            cw_min.to_le_bytes(),
            cw_max.to_le_bytes(),
            txop.to_le_bytes(),
            aifs.to_le_bytes(),
            [0, 0],
        ]
        .concat()
    };
    assert_eq!(b[..10], slot(7, 15, 94, 2));
    assert_eq!(b[10..20], slot(3, 7, 47, 2));
    assert_eq!(b[20..30], slot(15, 1023, 0, 3));
    assert_eq!(b[30..40], slot(5, 10, 0, 7));
    // bss_idx, qos, wmm_idx, pad.
    assert_eq!(b[40..44], [1, 1, 3, 0]);
}
