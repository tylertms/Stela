// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE firmware download: one attempt of `__rtw89_fw_download`
//! (`fw.c:1994`) for an AX chip on PCIe with no BB MCU (`bbmcu_nr` 0,
//! `rtw8852c.c:3146`), with the helpers it reaches:
//! `rtw89_mac_disable_cpu_ax` (`mac.c:4146`) and `rtw89_disable_fw_watchdog`
//! (`:4127`) through `rtw89_mac_mem_write`/`_read` (`:42`, `:52`),
//! `rtw89_mac_enable_cpu_ax` (`:4161`) with `dlfw`,
//! `rtw89_fwdl_secure_idmem_share_mode_ax` (`:7437`),
//! `rtw89_fwdl_check_path_ready_ax` (`:7419`), `rtw89_fw_download_suit`
//! (`fw.c:1957`), `rtw89_fw_download_hdr` / `__rtw89_fw_download_hdr`
//! (`:1785`, `:1729`) with `rtw89_h2c_pkt_set_hdr_fwdl` (`:1657`),
//! `rtw89_fw_download_main` / `__rtw89_fw_download_main` (`:1887`, `:1810`),
//! and `rtw89_fw_check_rdy` (`:106`) with `rtw89_fw_get_rdy_ax`
//! (`mac.c:7411`). The FWCMD descriptor is `rtw89_core_fill_txdesc_fwcmd_v1`
//! (`core.c:1899-1917`), the 8852C `fill_txdesc_fwcmd`. Register addresses
//! are from pinned rtw89 `reg.h` and `mac.h`.
//!
//! Only the non-secure path is ported: the bring-up refuses a secure-boot
//! chip before it gets here, so no MSS key is inserted and the idmem share
//! mode is not recorded. This module does not grant the retry loop of
//! `rtw89_fw_download` (`fw.c:2043`), the failure dump
//! `rtw89_fw_dl_fail_dump` past its first read (the whole
//! `R_AX_WCPU_FW_CTRL`, kept in the error), any H2C after the download, or
//! anything past the firmware's own ready status: no C2H, scan or radio
//! state.
use super::io::{AxError, AxIo, Polled, clear32, field16, poll_sample_atomic, set32};
use super::pci::{Ch12, FWCMD_SLOT_BYTES};
use crate::wifi::realtek::firmware::{AxImage, AxSection};
use alloc::vec::Vec;

/// `FWDL_SECTION_PER_PKT_LEN` (`fw.h:287`): the AX download chunk.
pub const CHUNK: usize = 2020;

/// `h2c_desc_size` = `sizeof(struct rtw89_rxdesc_short)` (`rtw8852c.c:3262`,
/// `core.h:1186`): four dwords, of which only dword0 is filled.
const TXDESC_BYTES: usize = 16;
/// `H2C_HEADER_LEN`, `struct fwcmd_hdr` (`fw.h:4634`, `:4673`).
const FWCMD_HDR_BYTES: usize = 8;
const _: () = assert!(TXDESC_BYTES + CHUNK <= FWCMD_SLOT_BYTES);

/// `AX_RXD_RPKT_LEN_MASK` GENMASK(13, 0) and `AX_RXD_RPKT_TYPE_MASK`
/// GENMASK(27, 24) (`txrx.h:345`, `:350`).
const RPKT_LEN_MASK: u32 = 0x3FFF;
const RPKT_TYPE_SHIFT: u32 = 24;
const RPKT_TYPE_MASK: u32 = 0xF;
/// `RTW89_CORE_RX_TYPE_H2C` and `RTW89_CORE_RX_TYPE_FWDL` (`core.h:239`).
const RX_TYPE_H2C: u32 = 13;
const RX_TYPE_FWDL: u32 = 14;

/// `fw.h:4673-4741`: `H2C_HDR_CAT` [1:0], `H2C_HDR_CLASS` [7:2],
/// `H2C_HDR_FUNC` [15:8], `H2C_HDR_DEL_TYPE` [19:16], `H2C_HDR_H2C_SEQ`
/// [31:24], `H2C_HDR_TOTAL_LEN` [13:0].
const H2C_CAT_MAC: u32 = 0x1;
const H2C_CL_MAC_FWDL: u32 = 0x3;
const H2C_FUNC_MAC_FWHDR_DL: u32 = 0x0;
const FWCMD_TYPE_H2C: u32 = 0;
const H2C_HDR_TOTAL_LEN: u32 = 0x3FFF;
/// `rtwdev->fw.h2c_seq` when the header goes out. The FWDL variant of the
/// header setter does not advance it, and `__rtw89_fw_download` zeroes it
/// after every download that got past the sections (`fw.c:2021`), so a
/// first download, and every retry of it, sends 0.
const H2C_SEQ_AT_DOWNLOAD: u8 = 0;

// reg.h / mac.h (pinned rtw89).
const R_AX_SYS_CLK_CTRL: u32 = 0x0008;
const B_AX_CPU_CLK_EN: u32 = 1 << 14;
const R_AX_PLATFORM_ENABLE: u32 = 0x0088;
const B_AX_WCPU_EN: u32 = 1 << 1;
const B_AX_PLATFORM_EN: u32 = 1 << 0;
const R_AX_HALT_H2C_CTRL: u32 = 0x0160;
const R_AX_HALT_C2H_CTRL: u32 = 0x0164;
const R_AX_HALT_H2C: u32 = 0x0168;
const R_AX_HALT_C2H: u32 = 0x016C;
const R_AX_WCPU_FW_CTRL: u32 = 0x01E0;
const B_AX_WCPU_FWDL_STS_MASK: u32 = 0x00E0; // GENMASK(7, 5)
const B_AX_FWDL_PATH_RDY: u8 = 1 << 2;
const B_AX_H2C_PATH_RDY: u8 = 1 << 1;
const B_AX_WCPU_FWDL_EN: u32 = 1 << 0;
const R_AX_BOOT_REASON: u32 = 0x01E6;
const B_AX_BOOT_REASON_MASK: u16 = 0x7; // GENMASK(2, 0)
const R_AX_UDM1: u32 = 0x01F4;
const R_AX_UDM2: u32 = 0x01F8;
/// `R_AX_FILTER_MODEL_ADDR` and `R_AX_INDIR_ACCESS_ENTRY`, the AX
/// `filter_model_addr` / `indir_access_addr` (`mac.c:7452-7453`).
const R_AX_FILTER_MODEL_ADDR: u32 = 0x0C04;
const R_AX_INDIR_ACCESS_ENTRY: u32 = 0x4_0000;
/// `CPU_LOCAL_BASE_ADDR` (`mac.h:325`), `RTW89_MAC_MEM_CPU_LOCAL` for AX.
const CPU_LOCAL_BASE_ADDR: u32 = 0x1800_3000;
const R_AX_WDT_CTRL: u32 = 0x0040;
const WDT_CTRL_ALL_DIS: u32 = 0;
const R_AX_WDT_STATUS: u32 = 0x0044;
const B_AX_FS_WDT_INT: u32 = 1 << 8;
const B_AX_FS_WDT_INT_MSK: u32 = 1 << 0;

/// `enum rtw89_fw_dl_status` (`fw.h:10-19`).
const RTW89_FWDL_INITIAL_STATE: u32 = 0;
const RTW89_FWDL_CHECKSUM_FAIL: u8 = 2;
const RTW89_FWDL_SECURITY_FAIL: u8 = 3;
const RTW89_FWDL_CV_NOT_MATCH: u8 = 4;
const RTW89_FWDL_WCPU_FW_INIT_RDY: u8 = 7;
/// `read_poll_timeout_atomic(..., 1, FWDL_WAIT_CNT, ...)`: 1 us samples for
/// 400 ms (`fw.h:5358`), in both the path-ready and the ready polls.
const FWDL_POLL_INTERVAL_US: u64 = 1;
const FWDL_WAIT_US: u64 = 400_000;
/// `mdelay(5)` before the ready check (`fw.c:2028`).
const READY_DELAY_US: u64 = 5_000;

/// The 16-byte FWCMD descriptor of `rtw89_core_fill_txdesc_fwcmd_v1`:
/// dword0 = `pkt_size` [13:0] | RX type [27:24], the rest zeroed by
/// `rtw89_pci_fwcmd_submit`'s memset. `pkt_size` is the skb length before
/// the descriptor is pushed (`core.c:1278`).
fn descriptor(pkt_size: usize, rx_type: u32) -> [u8; TXDESC_BYTES] {
    let dword0 = (pkt_size as u32 & RPKT_LEN_MASK) | (rx_type & RPKT_TYPE_MASK) << RPKT_TYPE_SHIFT;
    let mut d = [0; TXDESC_BYTES];
    d[..4].copy_from_slice(&dword0.to_le_bytes());
    d
}

/// The CH12 packet of `__rtw89_fw_download_hdr` for a v0 image: the FWCMD
/// descriptor (type H2C), the FWCMD header of `rtw89_h2c_pkt_set_hdr_fwdl`
/// (MAC / FWDL / FWHDR_DL, H2C sequence `seq`, total length = header +
/// 8), then the image header without its dynamic part and with W7[15:0] =
/// 2020 (`__rtw89_fw_download_tweak_hdr_v0`).
pub fn header_packet(image: &AxImage<'_>, seq: u8) -> Vec<u8> {
    let header = image.header_packet();
    let len = FWCMD_HDR_BYTES + header.len();
    let hdr0 = H2C_CAT_MAC
        | H2C_CL_MAC_FWDL << 2
        | H2C_FUNC_MAC_FWHDR_DL << 8
        | FWCMD_TYPE_H2C << 16
        | u32::from(seq) << 24;
    let hdr1 = len as u32 & H2C_HDR_TOTAL_LEN;
    let mut packet = Vec::with_capacity(TXDESC_BYTES + len);
    packet.extend_from_slice(&descriptor(len, RX_TYPE_H2C));
    packet.extend_from_slice(&hdr0.to_le_bytes());
    packet.extend_from_slice(&hdr1.to_le_bytes());
    packet.extend_from_slice(&header);
    packet
}

/// The CH12 packets of `__rtw89_fw_download_main` for one section: chunks
/// of at most `CHUNK` bytes in order, each behind a FWCMD descriptor of
/// type FWDL and no FWCMD header. An empty section sends nothing.
pub fn chunk_packets<'a>(section: &AxSection<'a>) -> impl Iterator<Item = Vec<u8>> + use<'a> {
    section.bytes.chunks(CHUNK).map(|chunk| {
        let mut packet = Vec::with_capacity(TXDESC_BYTES + chunk.len());
        packet.extend_from_slice(&descriptor(chunk.len(), RX_TYPE_FWDL));
        packet.extend_from_slice(chunk);
        packet
    })
}

/// One attempt of `__rtw89_fw_download(RTW89_FW_NORMAL)` for an RTL8852CE
/// with secure boot off: stop and reset the WCPU, start it in download
/// mode, send `image`'s header and sections on CH12 and wait for the
/// firmware to report `RTW89_FWDL_WCPU_FW_INIT_RDY`.
///
/// Every packet waits until CH12 consumed it, so at most one is in flight.
/// A failure leaves the WCPU as it stands; the next attempt starts with
/// `rtw89_mac_disable_cpu_ax`, as Linux's retry does. After a CH12 submit
/// failure the queue refuses packets until the next `pre_init`.
///
/// # Safety
/// The `Ch12::submit` contract: `io` is the exclusive register window of
/// the same RTL8852C requester that `pre_init` programmed with `ch12`'s
/// rings, its DMA domain isolates that requester, the rings are coherent WB
/// RAM with NoSnoop and ATS disabled, and one worker owns the device and
/// the queue.
pub unsafe fn download<I: AxIo + ?Sized>(
    io: &mut I,
    ch12: &mut Ch12<'_>,
    image: &AxImage<'_>,
) -> Result<(), AxError> {
    // rtw89_fw_hdr_parser ran when the image was parsed; take its sections
    // before touching the chip.
    let sections = image
        .sections()
        .map_err(|_| AxError::Firmware("image sections unreadable"))?;
    let header = header_packet(image, H2C_SEQ_AT_DOWNLOAD);
    if header.len() > FWCMD_SLOT_BYTES {
        return Err(AxError::Firmware("header packet longer than a CH12 slot"));
    }

    disable_cpu(io)?;
    enable_cpu(io)?;
    // rtw89_fw_download_suit: rtw89_fwdl_secure_idmem_share_mode_ax returns
    // at once with secure boot off; the RTL8922A malloc write is not ours.
    path_ready(io, B_AX_H2C_PATH_RDY)?;
    // rtw89_fw_download_hdr.
    // SAFETY: the caller's `Ch12::submit` contract, forwarded unchanged.
    unsafe { ch12.submit(io, &header) }?;
    path_ready(io, B_AX_FWDL_PATH_RDY)?;
    io.write32(R_AX_HALT_H2C_CTRL, 0)?;
    io.write32(R_AX_HALT_C2H_CTRL, 0)?;
    // rtw89_fw_download_main; an AX chip skips the per-suit ready check.
    for section in &sections {
        for packet in chunk_packets(section) {
            // SAFETY: the caller's `Ch12::submit` contract, forwarded.
            unsafe { ch12.submit(io, &packet) }?;
        }
    }
    // The h2c/rec sequence and counter resets are host state only.
    io.delay_us(READY_DELAY_US);
    check_ready(io)
}

/// `rtw89_mac_disable_cpu_ax` (`mac.c:4146`).
fn disable_cpu<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    clear32(io, R_AX_PLATFORM_ENABLE, B_AX_WCPU_EN)?;
    clear32(
        io,
        R_AX_WCPU_FW_CTRL,
        B_AX_WCPU_FWDL_EN | u32::from(B_AX_H2C_PATH_RDY | B_AX_FWDL_PATH_RDY),
    )?;
    clear32(io, R_AX_SYS_CLK_CTRL, B_AX_CPU_CLK_EN)?;
    // rtw89_disable_fw_watchdog (mac.c:4127), the non-885xB branch.
    mem_write(io, R_AX_WDT_CTRL, WDT_CTRL_ALL_DIS)?;
    let status = mem_read(io, R_AX_WDT_STATUS)?;
    mem_write(
        io,
        R_AX_WDT_STATUS,
        (status | B_AX_FS_WDT_INT) & !B_AX_FS_WDT_INT_MSK,
    )?;
    clear32(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)?;
    set32(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)
}

/// `rtw89_mac_mem_write(offset, val, RTW89_MAC_MEM_CPU_LOCAL)` (`mac.c:42`):
/// select the address, then store through the indirect window.
fn mem_write<I: AxIo + ?Sized>(io: &mut I, offset: u32, value: u32) -> Result<(), AxError> {
    io.write32(R_AX_FILTER_MODEL_ADDR, CPU_LOCAL_BASE_ADDR + offset)?;
    io.write32(R_AX_INDIR_ACCESS_ENTRY, value)
}

/// `rtw89_mac_mem_read(offset, RTW89_MAC_MEM_CPU_LOCAL)` (`mac.c:52`).
fn mem_read<I: AxIo + ?Sized>(io: &mut I, offset: u32) -> Result<u32, AxError> {
    io.write32(R_AX_FILTER_MODEL_ADDR, CPU_LOCAL_BASE_ADDR + offset)?;
    io.read32(R_AX_INDIR_ACCESS_ENTRY)
}

/// `rtw89_mac_enable_cpu_ax(rtwdev, 0, dlfw = true, _)` (`mac.c:4161`).
fn enable_cpu<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    if io.read32(R_AX_PLATFORM_ENABLE)? & B_AX_WCPU_EN != 0 {
        // -EFAULT.
        return Err(AxError::Firmware("WCPU already enabled"));
    }
    for reg in [
        R_AX_UDM1,
        R_AX_UDM2,
        R_AX_HALT_H2C_CTRL,
        R_AX_HALT_C2H_CTRL,
        R_AX_HALT_H2C,
        R_AX_HALT_C2H,
    ] {
        io.write32(reg, 0)?;
    }
    set32(io, R_AX_SYS_CLK_CTRL, B_AX_CPU_CLK_EN)?;
    let mut ctrl = io.read32(R_AX_WCPU_FW_CTRL)?;
    ctrl &= !(B_AX_WCPU_FWDL_EN | u32::from(B_AX_H2C_PATH_RDY | B_AX_FWDL_PATH_RDY));
    ctrl = (ctrl & !B_AX_WCPU_FWDL_STS_MASK)
        | (RTW89_FWDL_INITIAL_STATE << B_AX_WCPU_FWDL_STS_MASK.trailing_zeros());
    ctrl |= B_AX_WCPU_FWDL_EN;
    io.write32(R_AX_WCPU_FW_CTRL, ctrl)?;
    // R_AX_SEC_CTRL idmem size: 885xB only.
    field16(io, R_AX_BOOT_REASON, B_AX_BOOT_REASON_MASK, 0)?;
    set32(io, R_AX_PLATFORM_ENABLE, B_AX_WCPU_EN)
    // dlfw: no mdelay(5) and ready check here.
}

/// One `rtw89_read8(R_AX_WCPU_FW_CTRL)` sample. A byte of 0xFF passes every
/// check made on this byte, so it is confirmed against the whole register:
/// all ones there means the device is gone.
fn fw_ctrl8<I: AxIo + ?Sized>(io: &mut I) -> Result<u8, AxError> {
    let value = io.read8(R_AX_WCPU_FW_CTRL)?;
    if value == u8::MAX && io.read32(R_AX_WCPU_FW_CTRL)? == u32::MAX {
        return Err(AxError::AllOnes {
            register: R_AX_WCPU_FW_CTRL,
        });
    }
    Ok(value)
}

/// `rtw89_fwdl_check_path_ready_ax` (`mac.c:7419`) on PCIe; on expiry, the
/// whole register is read once more for the error.
fn path_ready<I: AxIo + ?Sized>(io: &mut I, check: u8) -> Result<(), AxError> {
    match poll_sample_atomic(
        io,
        FWDL_POLL_INTERVAL_US,
        FWDL_WAIT_US,
        false,
        fw_ctrl8,
        |value| value & check != 0,
    )? {
        Polled::Met(_) => Ok(()),
        Polled::Expired(_) => Err(expired(
            io,
            if check == B_AX_H2C_PATH_RDY {
                "H2C path not ready"
            } else {
                "FWDL path not ready"
            },
        )),
    }
}

/// The error of a download wait that expired: `what`, with the whole
/// `R_AX_WCPU_FW_CTRL` read once more, as `rtw89_fw_dl_fail_dump`
/// (`fw.c:1943`) reads it first. All ones there means the device is gone.
fn expired<I: AxIo + ?Sized>(io: &mut I, what: &'static str) -> AxError {
    match io.read32(R_AX_WCPU_FW_CTRL) {
        Ok(u32::MAX) => AxError::AllOnes {
            register: R_AX_WCPU_FW_CTRL,
        },
        Ok(fw_ctrl) => AxError::FirmwareStatus { what, fw_ctrl },
        Err(error) => error,
    }
}

/// `rtw89_fw_get_rdy_ax` (`mac.c:7411`): the download status [7:5].
fn fwdl_status(value: u8) -> u8 {
    ((u32::from(value) & B_AX_WCPU_FWDL_STS_MASK) >> B_AX_WCPU_FWDL_STS_MASK.trailing_zeros()) as u8
}

/// `rtw89_fw_check_rdy(RTW89_FWDL_CHECK_FREERTOS_DONE)` (`fw.c:106`): wait
/// the whole 400 ms for `RTW89_FWDL_WCPU_FW_INIT_RDY`, then name the last
/// status, with the whole register read after the wait.
fn check_ready<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    match poll_sample_atomic(
        io,
        FWDL_POLL_INTERVAL_US,
        FWDL_WAIT_US,
        false,
        fw_ctrl8,
        |value| fwdl_status(value) == RTW89_FWDL_WCPU_FW_INIT_RDY,
    )? {
        Polled::Met(_) => Ok(()),
        Polled::Expired(value) => Err(expired(
            io,
            match fwdl_status(value) {
                RTW89_FWDL_CHECKSUM_FAIL => "checksum failed",
                RTW89_FWDL_SECURITY_FAIL => "security check failed",
                RTW89_FWDL_CV_NOT_MATCH => "cut mismatch",
                _ => "not ready",
            },
        )),
    }
}

#[cfg(test)]
pub(crate) mod tests;
