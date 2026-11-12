use super::*;
use crate::{Mac, wifi::Channel};
use alloc::vec;

fn context() -> Context {
    Context {
        bss_index: 0,
        omac_index: 0,
        band_index: 0,
        wmm_index: 0,
        link_index: 0,
        broadcast_wcid: 19,
        address: Mac([2; 6]),
    }
}

/// 13 channels at 2.4 GHz and 21 at 5 GHz: two more than the request's
/// first channel list holds.
fn channels() -> Vec<Channel> {
    let five = [
        36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
        149,
    ];
    (1..=13)
        .map(|number| Channel {
            band: Band::Ghz2,
            number,
            frequency_mhz: 2407 + number * 5,
        })
        .chain(five.into_iter().map(|number| Channel {
            band: Band::Ghz5,
            number,
            frequency_mhz: 5000 + number * 5,
        }))
        .collect()
}

/// struct mt76_connac_hw_scan_req field offsets, from its packed layout.
const SSIDS: usize = 8;
const TIMES: usize = 152;
const CHANNEL_TYPE: usize = 158;
const CHANNELS: usize = 160;
const IES_LEN: usize = 224;
const IES: usize = 226;
const EXT_CHANNELS_NUM: usize = 826;
const EXT_CHANNELS: usize = 830;
const BSSID: usize = 1110;
const SSID_TYPE_EXT: usize = 1185;

#[test]
fn an_active_connac2_scan_lays_out_mt76_connac_hw_scan_req() {
    let request =
        crate::wifi::scan::Request::new(channels(), vec![Vec::new(), b"home".to_vec()], Vec::new())
            .unwrap();
    let elements = [0xdd, 2, 1, 2];
    for chip in [Chip::Mt7921, Chip::Mt7922] {
        let scan = scan(chip, context(), 0x85, &request, &elements).unwrap();
        assert_eq!(scan.chip(), chip);
        assert_eq!(scan.command(), (1 << 18) | 3);
        assert!(!scan.expects_response());
        let body = scan.body();
        assert_eq!(body.len(), 1186);
        // seq_num (7 bits, band 0), bss_idx, active, specified SSID plus
        // wildcard, one SSID, two probes each, split scan, version 1.
        assert_eq!(body[..8], [5, 0, 1, 4, 1, 2, 1 << 5, 1]);
        assert_eq!(body[SSIDS..SSIDS + 8], [4, 0, 0, 0, b'h', b'o', b'm', b'e']);
        assert!(body[SSIDS + 36..TIMES].iter().all(|&b| b == 0));
        // The firmware's dwell times: connac2 passes 0.
        assert_eq!(body[TIMES..CHANNEL_TYPE], [0; 6]);
        assert_eq!(body[CHANNEL_TYPE..CHANNEL_TYPE + 2], [4, 32]);
        assert_eq!(body[CHANNELS..CHANNELS + 4], [1, 1, 1, 2]);
        assert_eq!(body[CHANNELS + 2 * 13..CHANNELS + 2 * 13 + 2], [2, 36]);
        assert_eq!(body[CHANNELS + 2 * 31..CHANNELS + 64], [2, 140]);
        assert_eq!(body[EXT_CHANNELS_NUM], 2);
        assert_eq!(body[EXT_CHANNELS..EXT_CHANNELS + 4], [2, 144, 2, 149]);
        assert_eq!(body[IES_LEN..IES], [4, 0]);
        assert_eq!(body[IES..IES + 4], elements);
        assert_eq!(body[BSSID..BSSID + 6], [0xff; 6]);
        assert_eq!(body[SSID_TYPE_EXT], 1);
    }
}

#[test]
fn a_passive_connac2_scan_takes_the_wildcard_and_no_probes() {
    let request =
        crate::wifi::scan::Request::new(channels()[..2].to_vec(), Vec::new(), Vec::new()).unwrap();
    let scan = scan(Chip::Mt7921, context(), 0x7f, &request, &[]).unwrap();
    let body = scan.body();
    assert_eq!(body[..8], [0x7f, 0, 0, 1, 0, 0, 1 << 5, 1]);
    assert_eq!(body[CHANNEL_TYPE..CHANNEL_TYPE + 2], [4, 2]);
    assert_eq!(body[EXT_CHANNELS_NUM], 0);
    assert_eq!(body[IES_LEN..IES], [0, 0]);
    assert_eq!(body[SSID_TYPE_EXT], 0);
}

#[test]
fn connac2_scan_commands_refuse_the_mt7925_and_overlong_elements() {
    let request =
        crate::wifi::scan::Request::new(channels()[..1].to_vec(), Vec::new(), Vec::new()).unwrap();
    assert_eq!(
        scan(Chip::Mt7925, context(), 1, &request, &[]).err(),
        Some(Error::WrongChip)
    );
    assert_eq!(
        scan(Chip::Mt7922, context(), 1, &request, &[0; SCAN_IE_LEN + 1]).err(),
        Some(Error::InvalidScanElements)
    );
    assert!(scan(Chip::Mt7922, context(), 1, &request, &[0; SCAN_IE_LEN]).is_ok());
    assert_eq!(cancel_scan(Chip::Mt7925, 1).err(), Some(Error::WrongChip));
    let cancel = cancel_scan(Chip::Mt7921, 0x85).unwrap();
    assert_eq!(cancel.command(), (1 << 18) | 0x1b);
    assert_eq!(cancel.body(), [5, 0, 0, 0]);
    assert!(!cancel.expects_response());
    assert!(scan_done(0x0d) && scan_done(0x23) && !scan_done(14));
}

#[test]
fn rssi_notify_reports_the_bss_slot_and_tx_done_carries_one_txs_record() {
    // mt76_connac_rssi_notify_event: rssi[4], by BSS index; 0 is no report.
    let mut payload = [0u8; 16];
    payload[4..8].copy_from_slice(&(-61i32).to_le_bytes());
    assert_eq!(rssi(&payload, 1), Some(-61));
    assert_eq!(rssi(&payload, 0), None);
    assert_eq!(rssi(&payload[..6], 1), None);
    // mt7921_mcu_tx_done_event: txs[28] at offset 28.
    let mut event = [0u8; 88];
    for (i, byte) in event[28..56].iter_mut().enumerate() {
        *byte = i as u8 + 1;
    }
    let packet = tx_done_status(&event).unwrap();
    assert_eq!(packet[..8], [0; 8]);
    assert_eq!(packet[8..36], event[28..56]);
    assert_eq!(packet[36..], [0; 4]);
    assert!(tx_done_status(&event[..55]).is_none());
}
