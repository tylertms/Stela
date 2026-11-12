// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! The connac2 runtime station commands: `mt7921_mcu_set_roc` and
//! `mt7921_mcu_abort_roc`, `mt7921_mcu_set_tx`'s EDCA, the beacon filter of
//! `mt7921_mcu_set_beacon_filter` (`mt7921_mcu_uni_bss_bcnft`,
//! `mt7921_mcu_set_rxfilter`, `mt7921_mcu_set_bss_pm`) and
//! `mt7921_mcu_set_rssimonitor`. None of them is acknowledged.
use super::super::{Error, Profile, put16, tlv};
use crate::wifi::{
    Band,
    mediatek::{
        identity::Chip,
        mcu::init::{Kind, Request},
    },
    station::phy::AccessCategory,
};
use alloc::{vec, vec::Vec};

/// `MCU_UNI_CMD_ROC`.
const ROC: u32 = (1 << 17) | 0x27;
/// `MCU_CE_CMD_SET_RX_FILTER`, `MCU_CE_CMD_SET_BSS_ABORT`,
/// `MCU_CE_CMD_SET_EDCA_PARMS` and `MCU_CE_CMD_RSSI_MONITOR`.
const SET_RX_FILTER: u32 = (1 << 18) | 0x0a;
const SET_BSS_ABORT: u32 = (1 << 18) | 0x17;
const SET_EDCA_PARMS: u32 = (1 << 18) | 0x1d;
const RSSI_MONITOR: u32 = (1 << 18) | 0xa1;
/// `UNI_BSS_INFO_BCNFT`.
const UNI_BSS_INFO_BCNFT: u16 = 22;
/// `MT_WF_RFCR_DROP_OTHER_BEACON`.
const DROP_OTHER_BEACON: u32 = 1 << 11;

/// `mt7921_mcu_set_roc`: a join's 20 MHz channel request (`reqtype`
/// MT7921_ROC_REQ_JOIN) for `duration_ms` under `token`.
pub(in crate::wifi::mediatek) fn roc(
    chip: Chip,
    p: Profile,
    token: u8,
    duration_ms: u32,
) -> Result<Request, Error> {
    p.validate_inner(true)?;
    if duration_ms == 0 {
        return Err(Error::InvalidTiming);
    }
    let channel = p.channel.number as u8;
    let mut body = vec![0; 4];
    let r = tlv(&mut body, 0, 24);
    r[4] = p.interface.bss_index;
    r[5] = token;
    r[6] = channel;
    // The control channel is its own center: sco stays 0.
    r[8] = match p.channel.band {
        Band::Ghz2 => 1,
        Band::Ghz5 => 2,
        Band::Ghz6 => 3,
    };
    r[10] = channel;
    r[13] = channel;
    r[16..20].copy_from_slice(&duration_ms.to_le_bytes());
    r[20] = 0xff;
    Ok(Request::connac2_client(chip, Kind::Roc, ROC, body))
}

/// `mt7921_mcu_abort_roc`: release the channel `token` holds.
pub(in crate::wifi::mediatek) fn abort_roc(
    chip: Chip,
    p: Profile,
    token: u8,
) -> Result<Request, Error> {
    p.validate_inner(true)?;
    let mut body = vec![0; 4];
    let r = tlv(&mut body, 1, 12);
    r[4] = p.interface.bss_index;
    r[5] = token;
    r[6] = 0xff;
    Ok(Request::connac2_client(chip, Kind::Roc, ROC, body))
}

/// `mt7921_mcu_uni_bss_bcnft`: the firmware's beacon filter for the BSS's
/// beacon interval and DTIM period.
pub(in crate::wifi::mediatek) fn beacon_filter(p: Profile) -> Result<Vec<u8>, Error> {
    p.validate_inner(true)?;
    let mut body = vec![p.interface.bss_index, 0, 0, 0];
    let t = tlv(&mut body, UNI_BSS_INFO_BCNFT, 8);
    put16(t, 4, p.beacon_interval);
    t[6] = p.dtim_period;
    Ok(body)
}

/// `mt7921_mcu_set_rxfilter(dev, 0, bit_op, MT_WF_RFCR_DROP_OTHER_BEACON)`:
/// drop other BSSs' beacons while `enable` (MT7921_FIF_BIT_SET), or take
/// them again (MT7921_FIF_BIT_CLR).
pub(in crate::wifi::mediatek) fn filter(
    chip: Chip,
    p: Profile,
    enable: bool,
) -> Result<Request, Error> {
    p.validate_inner(true)?;
    let mut body = vec![0; 68];
    body[4] = 2;
    body[12..16].copy_from_slice(&DROP_OTHER_BEACON.to_le_bytes());
    body[16] = if enable { 1 } else { 2 };
    Ok(Request::connac2_client(
        chip,
        Kind::ClientFilter,
        SET_RX_FILTER,
        body,
    ))
}

/// `mt7921_mcu_set_bss_pm(dev, vif, false)`: SET_BSS_ABORT, which ends the
/// firmware's beacon tracking for the BSS.
pub(in crate::wifi::mediatek) fn bss_abort(chip: Chip, p: Profile) -> Result<Request, Error> {
    p.validate_inner(true)?;
    Ok(Request::connac2_client(
        chip,
        Kind::BssAbort,
        SET_BSS_ABORT,
        vec![p.interface.bss_index, 0, 0, 0],
    ))
}

/// `mt7921_mcu_set_rssimonitor`: firmware RSSI events above `high` or below
/// `low` dBm while `enable`.
pub(in crate::wifi::mediatek) fn rssi_monitor(
    chip: Chip,
    p: Profile,
    enable: bool,
    high: i8,
    low: i8,
) -> Result<Request, Error> {
    p.validate_inner(true)?;
    let body = vec![
        u8::from(enable),
        high as u8,
        low as u8,
        p.interface.bss_index,
        0,
        0,
        0,
        0,
    ];
    Ok(Request::connac2_client(
        chip,
        Kind::RssiMonitor,
        RSSI_MONITOR,
        body,
    ))
}

/// `mt7921_mcu_set_tx`'s SET_EDCA_PARMS: each access category's contention
/// windows (mac80211's `2^ECW - 1`, 5 and 10 when zero), TXOP and AIFSN in
/// the slot `to_aci` gives its mac80211 queue, then the BSS, its QoS and its
/// WMM set. `effective` is validated by the caller.
pub(in crate::wifi::mediatek) fn edca(
    chip: Chip,
    p: Profile,
    effective: &[AccessCategory; 4],
    qos: bool,
) -> Result<Request, Error> {
    let mut body = vec![0; 44];
    // WMM ACI to mac80211 queue (VO 0, VI 1, BE 2, BK 3), then to_aci.
    const TO_ACI: [usize; 4] = [1, 0, 2, 3];
    for ac in effective {
        let queue = match ac.aci {
            3 => 0,
            2 => 1,
            0 => 2,
            1 => 3,
            _ => return Err(Error::InvalidNegotiation),
        };
        let at = TO_ACI[queue] * 10;
        let window = |ecw: u8, zero: u16| match (1u16 << ecw) - 1 {
            0 => zero,
            cw => cw,
        };
        put16(&mut body, at, window(ac.ecw_min, 5));
        put16(&mut body, at + 2, window(ac.ecw_max, 10));
        put16(&mut body, at + 4, ac.txop_32us);
        put16(&mut body, at + 6, u16::from(ac.aifsn));
    }
    body[40] = p.interface.bss_index;
    body[41] = u8::from(qos);
    body[42] = p.interface.wmm_index;
    Ok(Request::connac2_client(
        chip,
        Kind::Edca,
        SET_EDCA_PARMS,
        body,
    ))
}

#[cfg(test)]
mod tests;
