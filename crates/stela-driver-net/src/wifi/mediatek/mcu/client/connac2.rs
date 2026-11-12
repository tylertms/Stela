// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! The connac2 (MT7921, MT7922) station commands whose layout differs from
//! the MT7925's UNI TLVs: the firmware's CE scan requests and events, as
//! `mt76_connac_mcu_hw_scan` and `mt7921_mcu_rx_unsolicited_event` give
//! them.
use super::Error;
use crate::wifi::{
    Band,
    mediatek::{identity::Chip, mcu::device::Context},
};
use alloc::vec::Vec;

/// `MCU_CE_CMD_START_HW_SCAN`.
const START_HW_SCAN: u32 = (1 << 18) | 0x03;
/// `MCU_CE_CMD_CANCEL_HW_SCAN`.
const CANCEL_HW_SCAN: u32 = (1 << 18) | 0x1b;
/// `MCU_EVENT_SCAN_DONE` and `MCU_EVENT_SCHED_SCAN_DONE`.
pub(in crate::wifi::mediatek) const SCAN_DONE_EVENTS: [u8; 2] = [0x0d, 0x23];
/// `MCU_EVENT_BSS_BEACON_LOSS`.
pub(in crate::wifi::mediatek) const BEACON_LOSS_EVENT: u8 = 0x13;
/// `MT76_CONNAC_SCAN_IE_LEN`, the probe request elements one scan carries.
pub(in crate::wifi::mediatek) const SCAN_IE_LEN: usize = 600;
/// `sizeof(struct mt76_connac_hw_scan_req)`.
const SCAN_REQUEST_LEN: usize = 1186;
/// `SCAN_FUNC_SPLIT_SCAN`, which connac2 firmware always takes.
const SCAN_FUNCTION_SPLIT: u8 = 1 << 5;

/// A connac2 chip's command; the MT7925 takes UNI TLVs instead.
fn connac2(chip: Chip) -> Result<(), Error> {
    if chip == Chip::Mt7925 {
        return Err(Error::WrongChip);
    }
    Ok(())
}

/// `mt76_connac_mcu_hw_scan` on a connac2 chip: the scan of `request`'s
/// channels, active for its SSIDs, with `elements` in each probe request.
/// The firmware chooses the dwell times (`duration` 0) and the scan is
/// split, as connac2 firmware requires; the request is not acknowledged,
/// its end being the firmware's scan-done event.
pub(in crate::wifi::mediatek) fn scan(
    chip: Chip,
    context: Context,
    sequence: u8,
    request: &crate::wifi::scan::Request,
    elements: &[u8],
) -> Result<crate::wifi::mediatek::mcu::init::Request, Error> {
    connac2(chip)?;
    if elements.len() > SCAN_IE_LEN {
        return Err(Error::InvalidScanElements);
    }
    let channels = request.channels();
    if channels.len() > 64 {
        return Err(Error::InvalidChannel);
    }
    let mut body = Vec::new();
    body.try_reserve_exact(SCAN_REQUEST_LEN)
        .map_err(|_| Error::NoMemory)?;
    body.resize(SCAN_REQUEST_LEN, 0);
    body[0] = (sequence & 0x7f) | ((context.band_index & 1) << 7);
    body[1] = context.bss_index;
    let active = request.active();
    body[2] = u8::from(active);
    body[5] = if active { 2 } else { 0 };
    body[6] = SCAN_FUNCTION_SPLIT;
    body[7] = 1;
    let mut named = 0;
    for name in request.ssids().iter().filter(|name| !name.is_empty()) {
        // ssids[4]: a 32-bit length and 32 bytes each.
        if named == 4 || name.len() > 32 {
            return Err(Error::InvalidScanElements);
        }
        let at = 8 + named * 36;
        body[at..at + 4].copy_from_slice(&(name.len() as u32).to_le_bytes());
        body[at + 4..at + 4 + name.len()].copy_from_slice(name);
        named += 1;
    }
    body[3] = if named > 0 { 1 << 2 } else { 1 };
    body[1185] = u8::from(named > 0);
    body[4] = named as u8;
    // probe_delay_time, channel_dwell_time and timeout_value stay 0.
    if !channels.is_empty() {
        body[158] = 4;
        body[159] = channels.len().min(32) as u8;
        body[826] = channels.len().saturating_sub(32) as u8;
    }
    for (index, channel) in channels.iter().enumerate() {
        let at = if index < 32 {
            160 + index * 2
        } else {
            830 + (index - 32) * 2
        };
        body[at] = match channel.band {
            Band::Ghz2 => 1,
            Band::Ghz5 => 2,
            Band::Ghz6 => 3,
        };
        body[at + 1] = u8::try_from(channel.number).map_err(|_| Error::InvalidChannel)?;
    }
    body[224..226].copy_from_slice(&(elements.len() as u16).to_le_bytes());
    body[226..226 + elements.len()].copy_from_slice(elements);
    // bssid: the wildcard, as cfg80211 fills an undirected scan's.
    body[1110..1116].copy_from_slice(&[0xff; 6]);
    Ok(crate::wifi::mediatek::mcu::init::Request::connac2_scan(
        chip,
        START_HW_SCAN,
        body,
    ))
}

/// `mt76_connac_mcu_cancel_hw_scan`: stop the scan `sequence` names.
pub(in crate::wifi::mediatek) fn cancel_scan(
    chip: Chip,
    sequence: u8,
) -> Result<crate::wifi::mediatek::mcu::init::Request, Error> {
    connac2(chip)?;
    Ok(crate::wifi::mediatek::mcu::init::Request::connac2_scan(
        chip,
        CANCEL_HW_SCAN,
        alloc::vec![sequence & 0x7f, 0, 0, 0],
    ))
}

/// Whether the unsolicited event `event_id` ends the station's scan, as
/// `mt7921_mcu_scan_event` completes it on either scan-done event.
pub(in crate::wifi::mediatek) fn scan_done(event_id: u8) -> bool {
    SCAN_DONE_EVENTS.contains(&event_id)
}

#[cfg(test)]
mod tests;
