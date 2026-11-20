// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
//! RTL8852CE (PCIe) efuse access, from `rtw89_enable_efuse_pwr_cut_ddv` /
//! `rtw89_disable_efuse_pwr_cut_ddv` (`efuse.c:74`, `:93`),
//! `rtw89_dump_physical_efuse_map_ddv` and its 5-try wrapper
//! `rtw89_dump_physical_efuse_map` (`efuse.c:113-228`),
//! `rtw89_dump_logical_efuse_map` (`efuse.c:238-279`),
//! `rtw89_efuse_recognize_mss_index_v0` / `_info_v1` and
//! `rtw89_efuse_read_fw_secure_ax` (`efuse.c:427-516`), with the chip sizes of
//! `rtw8852c_chip_info` and the MAC offset of `struct rtw8852c_efuse`
//! (`rtw8852c.h:33-103`). Only the DDV (non-DAV) physical map is read; the
//! DAV map, phycap, the efuse version byte and all other logical fields are
//! not ported. This module does not grant firmware download, secure-boot
//! support or any claim that the MAC address is valid.
use super::io::{AxError, AxIo, clear8, clear16, poll32_value_atomic, set8, set16};
use alloc::{vec, vec::Vec};

// reg.h (pinned rtw89).
const R_AX_SYS_ISO_CTRL: u32 = 0x0000;
const R_AX_EFUSE_CTRL: u32 = 0x0030;
const R_AX_PMC_DBG_CTRL2: u32 = 0x00CC;
const B_AX_PWC_EV2EF_B14: u16 = 1 << 14;
const B_AX_PWC_EV2EF_B15: u16 = 1 << 15;
const B_AX_ISO_EB2CORE: u16 = 1 << 8;
const B_AX_SYSON_DIS_PMCR_AX_WRMSK: u8 = 1 << 2;
const B_AX_EF_RDY: u32 = 1 << 29;
const B_AX_EF_ADDR_MASK: u32 = 0x07FF_0000; // GENMASK(26, 16)

// rtw8852c_chip_info (rtw8852c.c:3226-3228).
const SEC_CTRL_EFUSE_SIZE: usize = 4;
pub const PHYSICAL_EFUSE_SIZE: usize = 1216;
const LOGICAL_EFUSE_SIZE: usize = 2048;

// efuse.c:15 and the 5-try (efuse.c:211-228) loop of rtw89_dump_physical_efuse_map.
const EFUSE_EXTERNALPN_ADDR_AX: u16 = 0x5EC;
const DUMP_TRIES: usize = 5;

/// `fsleep(1000)` in the power-cut sequences.
const PWR_CUT_DELAY_US: u64 = 1_000;
/// `read_poll_timeout_atomic(..., 1, 1000000, ...)` of the byte read.
const READ_POLL_INTERVAL_US: u64 = 1;
const READ_POLL_TIMEOUT_US: u64 = 1_000_000;

/// Offset of `e.mac_addr` in the packed `struct rtw8852c_efuse` for PCIe
/// (the union follows `rsvd24[71]`; see `mac_offset_matches_the_layout`).
pub const MAC_ADDR_OFFSET: usize = 0x400;

/// `rtw89_enable_efuse_pwr_cut_ddv` (`efuse.c:74`), RTL8852C: not an 8852A,
/// and the 8852B burst-mode branch does not apply.
fn enable_pwr_cut<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set8(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;
    set16(io, R_AX_SYS_ISO_CTRL, B_AX_PWC_EV2EF_B14)?;
    io.delay_us(PWR_CUT_DELAY_US);
    set16(io, R_AX_SYS_ISO_CTRL, B_AX_PWC_EV2EF_B15)?;
    clear16(io, R_AX_SYS_ISO_CTRL, B_AX_ISO_EB2CORE)
}

/// `rtw89_disable_efuse_pwr_cut_ddv` (`efuse.c:93`), the reverse.
fn disable_pwr_cut<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set16(io, R_AX_SYS_ISO_CTRL, B_AX_ISO_EB2CORE)?;
    clear16(io, R_AX_SYS_ISO_CTRL, B_AX_PWC_EV2EF_B15)?;
    io.delay_us(PWR_CUT_DELAY_US);
    clear16(io, R_AX_SYS_ISO_CTRL, B_AX_PWC_EV2EF_B14)?;
    clear8(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)
}

/// `rtw89_dump_physical_efuse_map_ddv`: one power-cut enable, then per byte a
/// whole-word write of the address with `EF_RDY` clear and a poll of
/// `EF_RDY`, then one power-cut disable. A failed byte returns at once, with
/// the power cut left enabled, as Linux does.
fn dump_once<I: AxIo + ?Sized>(io: &mut I, start: u16, out: &mut [u8]) -> Result<(), AxError> {
    enable_pwr_cut(io)?;
    for (i, byte) in out.iter_mut().enumerate() {
        let addr = u32::from(start) + i as u32;
        let ctl = (addr << 16) & B_AX_EF_ADDR_MASK;
        io.write32(R_AX_EFUSE_CTRL, ctl & !B_AX_EF_RDY)?;
        // `read_poll_timeout_atomic(..., delay_before_read = true)`
        // (efuse.c:126-128): the 1 us wait before the first sample counts
        // against the iteration budget.
        let value = poll32_value_atomic(
            io,
            R_AX_EFUSE_CTRL,
            B_AX_EF_RDY,
            B_AX_EF_RDY,
            READ_POLL_INTERVAL_US,
            READ_POLL_TIMEOUT_US,
            true,
        )?;
        *byte = value as u8;
    }
    disable_pwr_cut(io)
}

/// `rtw89_dump_physical_efuse_map` (DDV): up to five whole dumps, the last
/// error returned. Fills `out` from physical efuse address `start`. An empty
/// `out` does nothing (`dump_size == 0`).
pub fn read_physical<I: AxIo + ?Sized>(
    io: &mut I,
    start: u16,
    out: &mut [u8],
) -> Result<(), AxError> {
    if out.is_empty() {
        return Ok(());
    }
    let mut result = Ok(());
    for _ in 0..DUMP_TRIES {
        result = dump_once(io, start, out);
        if result.is_ok() {
            break;
        }
    }
    result
}

/// What `rtw89_efuse_read_fw_secure_ax` decides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SecureBoot {
    Off,
    /// `sec->secure_boot`, with `sec->mss_idx` (0 unless the v0 key table
    /// matched).
    On {
        mss_index: u8,
    },
}

/// `otp_key_info_{externalPN,customer,serialNum}` (`efuse.c:25-27`).
const OTP_KEY_INFO: [(u8, u8, u8); 2] = [(0x0, 0x0, 0x0), (0x0, 0x1, 0x1)];

/// `rtw89_efuse_recognize_mss_index_v0` (`efuse.c:459`): the matching row.
fn mss_index_v0(b1: u8, b2: u8) -> Option<u8> {
    let external_pn = 0xFF - b1;
    let customer = 0xF - (b2 & 0x0F);
    let serial_num = 0x7 - ((b2 >> 4) & 0x7);
    OTP_KEY_INFO
        .iter()
        .position(|&(pn, cust, serial)| {
            external_pn == pn && customer == cust && serial_num == serial
        })
        .map(|i| i as u8)
}

/// `rtw89_efuse_recognize_mss_info_v1` (`efuse.c:427`) reduced to its verdict:
/// `get_mss_dev_type_idx` accepts only these device types. (The 8852B special
/// case does not apply.)
fn mss_info_v1_valid(b1: u8) -> bool {
    matches!(b1 & 0x0F, 0xC | 0xA | 0x9 | 0x6 | 0xF)
}

/// The decision of `rtw89_efuse_read_fw_secure_ax` (`efuse.c:484-516`) on the two
/// bytes at 0x5EC and 0x5ED. Both 0xFF is off. Otherwise secure boot is on
/// only if the v0 key table or the v1 device type accepts them.
pub fn decode_secure_boot(b1: u8, b2: u8) -> SecureBoot {
    if b1 == 0xFF && b2 == 0xFF {
        return SecureBoot::Off;
    }
    let v0 = mss_index_v0(b1, b2);
    if v0.is_none() && !mss_info_v1_valid(b1) {
        return SecureBoot::Off;
    }
    SecureBoot::On {
        mss_index: v0.unwrap_or(0),
    }
}

/// `rtw89_efuse_read_fw_secure_ax`: read the two physical bytes at 0x5EC and
/// 0x5ED, in one dump.
pub fn secure_boot<I: AxIo + ?Sized>(io: &mut I) -> Result<SecureBoot, AxError> {
    let mut sec = [0u8; 2];
    read_physical(io, EFUSE_EXTERNALPN_ADDR_AX, &mut sec)?;
    Ok(decode_secure_boot(sec[0], sec[1]))
}

/// `rtw89_dump_logical_efuse_map` (`efuse.c:238`) for the 8852C DDV map: a
/// 2048-byte map filled with 0xFF, then each block's words placed, parsing
/// from physical index 4. Linux's `-EINVAL` returns are `AxError::Efuse`.
pub fn logical_map(physical: &[u8]) -> Result<Vec<u8>, AxError> {
    if physical.len() < PHYSICAL_EFUSE_SIZE {
        return Err(AxError::Efuse("physical map is shorter than 1216 bytes"));
    }
    let mut log = vec![0xFFu8; LOGICAL_EFUSE_SIZE];
    let end = PHYSICAL_EFUSE_SIZE - SEC_CTRL_EFUSE_SIZE;
    let mut phy_idx = SEC_CTRL_EFUSE_SIZE;
    while phy_idx < end {
        let hdr1 = physical[phy_idx];
        let hdr2 = physical[phy_idx + 1];
        if hdr1 == 0xFF || hdr2 == 0xFF {
            break;
        }
        let blk_idx = usize::from(((hdr2 & 0xF0) >> 4) | ((hdr1 & 0x0F) << 4));
        let word_en = hdr2 & 0x0F;
        phy_idx += 2;
        for i in 0..4 {
            if word_en & (1 << i) != 0 {
                continue;
            }
            let log_idx = (blk_idx << 3) + (i << 1);
            if phy_idx + 1 > end - 1 || log_idx + 1 > LOGICAL_EFUSE_SIZE {
                return Err(AxError::Efuse("logical map out of range"));
            }
            log[log_idx] = physical[phy_idx];
            log[log_idx + 1] = physical[phy_idx + 1];
            phy_idx += 2;
        }
    }
    Ok(log)
}

/// `ether_addr_copy(efuse->addr, map->e.mac_addr)` (`rtw8852c.c:648`). A map
/// too short to hold the address yields 0xFF for the missing bytes.
pub fn mac_address(logical: &[u8]) -> [u8; 6] {
    let mut mac = [0xFFu8; 6];
    for (i, b) in mac.iter_mut().enumerate() {
        if let Some(&v) = logical.get(MAC_ADDR_OFFSET + i) {
            *b = v;
        }
    }
    mac
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
    use alloc::{boxed::Box, vec, vec::Vec};

    /// A model whose efuse controller answers every read of 0x0030 with
    /// `EF_RDY`, the address last written and that address's physical byte.
    fn efuse_model(phys: Vec<u8>) -> RegisterModel {
        let mut m = RegisterModel::new();
        m.on_read(
            R_AX_EFUSE_CTRL,
            Box::new(move |m: &mut RegisterModel| {
                let ctl = m.get32(R_AX_EFUSE_CTRL);
                let addr = ((ctl >> 16) & 0x7FF) as usize;
                (ctl & 0x07FF_0000) | B_AX_EF_RDY | u32::from(phys[addr])
            }),
        );
        m
    }

    fn phys_with(bytes: &[(usize, u8)]) -> Vec<u8> {
        let mut p = vec![0xFF; 2048];
        for &(i, v) in bytes {
            p[i] = v;
        }
        p
    }

    /// `efuse.c:74-90` on a zeroed model.
    fn enable_ops() -> Vec<Op> {
        vec![
            Op::R8(0x00CC),
            Op::W8(0x00CC, 0x04), // set8 PMC_DBG_CTRL2 bit2
            Op::R16(0x0000),
            Op::W16(0x0000, 0x4000), // set B14
            Op::Delay(1000),         // fsleep(1000)
            Op::R16(0x0000),
            Op::W16(0x0000, 0xC000), // set B15
            Op::R16(0x0000),
            Op::W16(0x0000, 0xC000), // clr bit8 (already clear)
        ]
    }

    /// `efuse.c:93-110`, after `enable_ops` left 0x0000 = 0xC000, 0x00CC = 4.
    fn disable_ops() -> Vec<Op> {
        vec![
            Op::R16(0x0000),
            Op::W16(0x0000, 0xC100), // set ISO_EB2CORE (bit8)
            Op::R16(0x0000),
            Op::W16(0x0000, 0x4100), // clr B15
            Op::Delay(1000),
            Op::R16(0x0000),
            Op::W16(0x0000, 0x0100), // clr B14
            Op::R8(0x00CC),
            Op::W8(0x00CC, 0x00), // clr bit2
        ]
    }

    #[test]
    fn efuse_read_byte_protocol() {
        let mut m = efuse_model(phys_with(&[(0x5EC, 0xA5)]));
        let mut out = [0u8; 1];
        read_physical(&mut m, 0x5EC, &mut out).unwrap();
        assert_eq!(out, [0xA5]);
        let mut want = enable_ops();
        // efuse.c:124-132: whole-word write of addr << 16 with RDY clear,
        // then one read of the poll.
        want.push(Op::W32(0x0030, 0x05EC_0000));
        want.push(Op::Delay(1)); // delay_before_read (efuse.c:126-128)
        want.push(Op::R32(0x0030));
        want.extend(disable_ops());
        assert_eq!(m.trace, want);
    }

    #[test]
    fn read_physical_enables_the_power_cut_once_per_dump() {
        let mut m = efuse_model(phys_with(&[(0x5EC, 1), (0x5ED, 2), (0x5EE, 3)]));
        let mut out = [0u8; 3];
        read_physical(&mut m, 0x5EC, &mut out).unwrap();
        assert_eq!(out, [1, 2, 3]);
        let writes = m.writes();
        assert_eq!(writes.iter().filter(|o| **o == Op::Delay(1000)).count(), 2);
        assert_eq!(
            writes
                .iter()
                .filter(|o| matches!(o, Op::W32(0x0030, _)))
                .cloned()
                .collect::<Vec<_>>(),
            vec![
                Op::W32(0x0030, 0x05EC_0000),
                Op::W32(0x0030, 0x05ED_0000),
                Op::W32(0x0030, 0x05EE_0000)
            ]
        );
    }

    #[test]
    fn read_physical_retries_whole_dump_and_leaves_power_cut_on_failure() {
        // The controller is not ready until past the first poll's 1 s
        // timeout; the second whole dump then succeeds (efuse.c:211-228).
        let phys = phys_with(&[(0x10, 0x42)]);
        let mut m = RegisterModel::new();
        m.on_read(
            R_AX_EFUSE_CTRL,
            Box::new(move |m: &mut RegisterModel| {
                let ctl = m.get32(R_AX_EFUSE_CTRL);
                if m.now < 1_002_500 {
                    return ctl & !B_AX_EF_RDY;
                }
                let addr = ((ctl >> 16) & 0x7FF) as usize;
                B_AX_EF_RDY | (ctl & 0x07FF_0000) | u32::from(phys[addr])
            }),
        );
        let mut out = [0u8; 1];
        read_physical(&mut m, 0x10, &mut out).unwrap();
        assert_eq!(out, [0x42]);
        let w = m.writes();
        // Enable ran twice and disable once: no disable ran between the
        // failed poll and the retry (so the second enable saw B14 and B15
        // already set).
        assert_eq!(w.iter().filter(|o| **o == Op::Delay(1000)).count(), 3);
        assert_eq!(w.iter().filter(|o| **o == Op::W16(0, 0x0100)).count(), 1);
    }

    #[test]
    fn read_physical_gives_up_after_five_dumps() {
        let mut m = RegisterModel::new(); // EF_RDY never sets
        let mut out = [0u8; 1];
        let err = read_physical(&mut m, 0, &mut out).unwrap_err();
        assert!(matches!(err, AxError::Timeout { register: 0x30, .. }));
        let enables = m
            .writes()
            .iter()
            .filter(|o| matches!(o, Op::W8(0x00CC, 0x04)))
            .count();
        assert_eq!(enables, 5);
    }

    #[test]
    fn read_physical_of_nothing_touches_nothing() {
        let mut m = RegisterModel::new();
        read_physical(&mut m, 0, &mut []).unwrap();
        assert!(m.trace.is_empty());
    }

    #[test]
    fn secure_boot_reads_both_bytes_in_one_dump() {
        let mut m = efuse_model(phys_with(&[]));
        assert_eq!(secure_boot(&mut m).unwrap(), SecureBoot::Off);
        let w = m.writes();
        assert_eq!(w.iter().filter(|o| **o == Op::Delay(1000)).count(), 2);
        assert!(w.contains(&Op::W32(0x0030, 0x05EC_0000)));
        assert!(w.contains(&Op::W32(0x0030, 0x05ED_0000)));
    }

    #[test]
    fn secure_boot_off_when_both_ff() {
        assert_eq!(decode_secure_boot(0xFF, 0xFF), SecureBoot::Off);
    }

    #[test]
    fn secure_boot_on_when_a_key_table_row_matches() {
        // efuse.c:459-480: externalPN = 0xFF - b1, customer = 0xF - b2[3:0],
        // serialNum = 0x7 - b2[6:4]; row 0 is {0,0,0}, row 1 is {0,1,1}.
        // b1 = 0xFF, b2 = 0x7F: PN 0, customer 0, serial 0 -> row 0.
        assert_eq!(
            decode_secure_boot(0xFF, 0x7F),
            SecureBoot::On { mss_index: 0 }
        );
        // b2 = 0x6E: customer 0xF - 0xE = 1, serial 7 - 6 = 1 -> row 1.
        assert_eq!(
            decode_secure_boot(0xFF, 0x6E),
            SecureBoot::On { mss_index: 1 }
        );
        let mut m = efuse_model(phys_with(&[(0x5EC, 0xFF), (0x5ED, 0x6E)]));
        assert_eq!(
            secure_boot(&mut m).unwrap(),
            SecureBoot::On { mss_index: 1 }
        );
    }

    #[test]
    fn secure_boot_on_by_v1_device_type_alone_keeps_mss_index_zero() {
        // 0x5EC = 0x0C: device type 0xC (WINLIN_INBOX) is valid for v1 even
        // though no v0 row matches (0xFF - 0x0C != 0).
        assert_eq!(
            decode_secure_boot(0x0C, 0xFF),
            SecureBoot::On { mss_index: 0 }
        );
    }

    #[test]
    fn secure_boot_off_when_neither_decode_accepts() {
        // The brief expected On for 0x5EC = 0x01, 0x5ED = 0xFF. Linux:
        // v0 needs externalPN 0 (b1 == 0xFF); v1 maps device type 0x1 to
        // INV, so `can_mss_v0 || can_mss_v1` is false and secure_boot stays
        // false (efuse.c:508-511).
        assert_eq!(decode_secure_boot(0x01, 0xFF), SecureBoot::Off);
        // Device type 0xE is not valid either.
        assert_eq!(decode_secure_boot(0xFE, 0xFF), SecureBoot::Off);
    }

    #[test]
    fn logical_map_places_words_by_header() {
        // h1 = 0x08, h2 = 0x0E: blk = (h2 >> 4) | (h1 & 0xF) << 4 = 0x80,
        // word_en = 0xE, so only word 0 (a 0 bit) follows, at 0x80 * 8 = 0x400.
        // The next header is 0xFF 0xFF: stop.
        let phys = phys_with(&[(4, 0x08), (5, 0x0E), (6, 0x11), (7, 0x22)]);
        let log = logical_map(&phys).unwrap();
        assert_eq!(log.len(), 2048);
        assert_eq!(&log[0x400..0x402], &[0x11, 0x22]);
        assert!(log[..0x400].iter().all(|&b| b == 0xFF));
        assert!(log[0x402..].iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn logical_map_places_each_enabled_word_at_its_slot() {
        // h2 low nibble 0b0101: words 1 and 3 present, at blk * 8 + 2 / + 6.
        let phys = phys_with(&[
            (4, 0x00),
            (5, 0x15), // blk = 0x01
            (6, 0xA1),
            (7, 0xA2),
            (8, 0xB1),
            (9, 0xB2),
        ]);
        let log = logical_map(&phys).unwrap();
        assert_eq!(&log[10..12], &[0xA1, 0xA2]);
        assert_eq!(&log[14..16], &[0xB1, 0xB2]);
        assert_eq!(log[8], 0xFF);
    }

    #[test]
    fn logical_map_stops_when_either_header_byte_is_ff() {
        let phys = phys_with(&[(4, 0xFF), (5, 0x00), (6, 0), (7, 0), (8, 0x00), (9, 0x00)]);
        assert!(logical_map(&phys).unwrap().iter().all(|&b| b == 0xFF));
        let phys = phys_with(&[(4, 0x00), (5, 0xFF)]);
        assert!(logical_map(&phys).unwrap().iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn logical_map_skips_the_four_control_bytes() {
        let phys = phys_with(&[(0, 0x08), (1, 0x0E), (2, 0x33), (3, 0x44)]);
        assert!(logical_map(&phys).unwrap().iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn logical_map_rejects_a_word_past_the_physical_limit() {
        // Headers and words all sit at even indices from 4. Pad with empty
        // blocks (h2 low nibble 0xF: no words) up to the header under test.
        let padded = |at: usize, rest: &[(usize, u8)]| {
            let mut p = phys_with(rest);
            for i in (4..at).step_by(2) {
                p[i] = 0x00;
                p[i + 1] = 0x0F;
            }
            p
        };
        // Header at 1208 (< 1212), word 0 at 1210..1212: Linux rejects only
        // phy_idx + 1 > 1211, and 1211 is not past it (efuse.c:269).
        let p = padded(
            1208,
            &[(1208, 0x00), (1209, 0x0E), (1210, 0x55), (1211, 0x66)],
        );
        assert_eq!(&logical_map(&p).unwrap()[0..2], &[0x55, 0x66]);
        // A second word would start at 1212: 1213 > 1211.
        let p = padded(
            1208,
            &[(1208, 0x00), (1209, 0x0C), (1210, 0x55), (1211, 0x66)],
        );
        assert!(logical_map(&p).is_err());
        // A header at 1210 is still parsed (1210 < 1212); its word is out.
        let p = padded(1210, &[(1210, 0x00), (1211, 0x0E)]);
        assert!(logical_map(&p).is_err());
        // A header at 1212 is not parsed: the loop ends.
        let p = padded(1212, &[(1212, 0x00), (1213, 0x0E)]);
        assert!(logical_map(&p).is_ok());
    }

    #[test]
    fn logical_map_last_word_of_block_ff_fits() {
        // blk 0xFF word 3 lands at 2046..2048, the last two logical bytes.
        let phys = phys_with(&[(4, 0x0F), (5, 0xF7), (6, 0xAA), (7, 0xBB)]);
        let log = logical_map(&phys).unwrap();
        assert_eq!(&log[2046..2048], &[0xAA, 0xBB]);
    }

    #[test]
    fn logical_map_needs_a_full_physical_map() {
        assert!(logical_map(&[0xFF; 100]).is_err());
    }

    #[test]
    fn mac_from_logical_0x400() {
        let mut log = vec![0xFFu8; 2048];
        log[0x3FF] = 0x99;
        log[0x400..0x406].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        log[0x406] = 0x99;
        assert_eq!(mac_address(&log), [1, 2, 3, 4, 5, 6]);
        assert_eq!(mac_address(&[]), [0xFF; 6]);
    }

    #[test]
    fn mac_offset_matches_the_layout() {
        // rtw8852c.h:33-103, packed. TSSI offset: CCK 6 + MCS_2G 5 + 7 + 5G 14.
        let tssi = 6 + 5 + 7 + 14;
        let mut off = 0x210;
        off += tssi + 10 + tssi + 94; // path A, rsvd1, path B, rsvd2
        off += 1 + 1 + 1 + 1 + 5; // channel_plan, xtal_k, rsvd3, iqk_lck, rsvd4
        off += 1 + 1 + 1 + 1 + 1; // bitfield, rsvd5, bitfield, eeprom_version, customer_id
        off += 1 + 1 + 1 + 1 + 1; // swing 2g/5g, pwr_trk, trx_path, rfe_type
        off += 2 + 3 + 1 + 1 + 2; // country_code, rsvd7, therm a/b, rsvd8
        off += 9 + 35; // rx_gain 2g ofdm..5g high (9 bytes incl. rsvds), rsvd13
        off += 32 + 10 + 32 + 94; // 6 GHz TSSI A, rsvd14, B, rsvd15
        off += 16 + 1 + 71; // rx_gain_6g pairs, channel_plan_6g, rsvd24
        assert_eq!(off, MAC_ADDR_OFFSET);
    }

    #[test]
    fn end_to_end_physical_to_mac() {
        let mut phys = vec![0xFFu8; PHYSICAL_EFUSE_SIZE];
        // Block 0x80 (logical 0x400): words 0, 1, 2 -> six MAC bytes.
        phys[4..6].copy_from_slice(&[0x08, 0x08]);
        phys[6..12].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF0]);
        let log = logical_map(&phys).unwrap();
        assert_eq!(mac_address(&log), [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF0]);
    }
}

#[cfg(test)]
mod budget_tests {
    use super::*;
    use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};

    /// `read_poll_timeout_atomic(.., 1, 1000000, true)` (`efuse.c:126-128`):
    /// the delay_before wait leaves left_ns = 999_999_000, cost 1001 per
    /// failure, so the expired sample is k = 999_999_000 / 1001 + 1 = 999_001
    /// and the poll makes 999_002 samples, where wall time with a 1 us read
    /// ends near 500_000.
    #[test]
    fn efuse_ready_poll_spends_the_linux_iteration_budget() {
        let mut m = RegisterModel::new();
        m.read_cost_us = 1;
        let err = poll32_value_atomic(
            &mut m,
            R_AX_EFUSE_CTRL,
            B_AX_EF_RDY,
            B_AX_EF_RDY,
            READ_POLL_INTERVAL_US,
            READ_POLL_TIMEOUT_US,
            true,
        );
        assert!(err.is_err());
        let reads = m
            .trace
            .iter()
            .filter(|o| **o == Op::R32(R_AX_EFUSE_CTRL))
            .count();
        assert_eq!(reads, 999_002);
        // dump_once itself must be the atomic caller: its first sample of a
        // never-ready byte comes after exactly one 1 us wait.
        let mut d = RegisterModel::new();
        d.read_cost_us = 1;
        let mut out = [0u8; 1];
        assert!(dump_once(&mut d, 0, &mut out).is_err());
        let polls = d
            .trace
            .iter()
            .filter(|o| **o == Op::R32(R_AX_EFUSE_CTRL))
            .count();
        assert_eq!(polls, 999_002);
    }
}
