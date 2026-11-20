// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE MAC pre-init for firmware download, from the part of
//! `rtw89_mac_partial_init` (`mac.c:4307`) that runs before the PCI pre-init
//! and the download: `rtw89_mac_ctrl_hci_dma_trx(true)` (`mac.h:1642`) and
//! `rtw89_mac_dmac_pre_init` (`mac.c:4258`) with its AX helpers
//! `rtw89_mac_hci_func_en_ax` (`:4208`), `rtw89_mac_dmac_func_pre_en_ax`
//! (`:4222`), `rtw89_mac_dle_init(RTW89_QTA_DLFW)` (`:2274`, with
//! `dle_func_en_ax` `:2069`, `dle_clk_en_ax` `:2079`, `dle_mix_cfg_ax`
//! `:2092`, `chk_dle_rdy_ax` `:2149`, `wde_quota_cfg_ax` `:2178`,
//! `ple_quota_cfg_ax` `:2193`) and `rtw89_mac_hfc_init(reset, !en, h2c_en)`
//! (`:1194`) down its H2C-only path (`hfc_func_en_ax` `:1178`,
//! `hfc_h2c_cfg_ax` `:1127`). Register addresses are from pinned rtw89
//! `reg.h`.
//!
//! Only the RTL8852C, PCIe, DLFW branches are ported; `ext_mode` is
//! `RTW89_QTA_SCC` as `rtw89_core_init` leaves it for an AX chip
//! (`core.c:7023`). This module does not grant the BB pre-init, the PCI
//! pre-init, the SCC quota layout (`rtw89_mac_dle_init(SCC)` after the
//! firmware is up), per-channel HFC, or firmware readiness.
use super::io::{AxError, AxIo, clear32, field32, poll32, set32};

// reg.h / mac.h (pinned rtw89).
/// `R_AX_HCI_FUNC_EN_V1` (`reg.h:519`), the 8852C `hci_func_en_addr`
/// (`rtw8852c.c:3261`); `B_AX_HCI_TXDMA_EN`/`B_AX_HCI_RXDMA_EN` (`reg.h:548`).
const R_AX_HCI_FUNC_EN_V1: u32 = 0x7880;
const B_AX_HCI_TXDMA_EN: u32 = 1 << 0;
const B_AX_HCI_RXDMA_EN: u32 = 1 << 1;

const R_AX_DMAC_FUNC_EN: u32 = 0x8400;
const B_AX_DLE_PLE_EN: u32 = 1 << 23;
const B_AX_DLE_WDE_EN: u32 = 1 << 26;
const B_AX_MAC_FUNC_EN: u32 = 1 << 30;
const B_AX_DMAC_FUNC_EN: u32 = 1 << 29;
const B_AX_PKT_BUF_EN: u32 = 1 << 22;
const B_AX_DISPATCHER_EN: u32 = 1 << 18;
const B_AX_H_AXIDMA_EN: u32 = 1 << 14;
const R_AX_DMAC_CLK_EN: u32 = 0x8404;
const B_AX_DLE_PLE_CLK_EN: u32 = 1 << 23;
const B_AX_DLE_WDE_CLK_EN: u32 = 1 << 26;
const B_AX_DISPATCHER_CLK_EN: u32 = 1 << 18;

const R_AX_HAXI_INIT_CFG1: u32 = 0x1000;
/// `B_AX_DMA_MODE_MASK` GENMASK(19, 18); `DMA_MOD_PCIE_1B` is 0 (`reg.h:362`).
const B_AX_DMA_MODE_MASK: u32 = 0x3 << 18;
const B_AX_STOP_AXI_MST: u32 = 1 << 17;
const B_AX_RXHCI_EN_V1: u32 = 1 << 15;
const B_AX_TXHCI_EN_V1: u32 = 1 << 7;
const DMA_MOD_PCIE_1B: u32 = 0;
const R_AX_HAXI_DMA_STOP1: u32 = 0x1010;
/// `B_AX_STOP_ACH0..ACH7` (bits 8-15), `B_AX_STOP_CH8` (16), `CH9` (17) and
/// `CH12` (18). `B_AX_STOP_WPDMA` (19) is not in Linux's list.
const STOP1_BITS: u32 = 0x0007_FF00;
const R_AX_HAXI_DMA_STOP2: u32 = 0x11C0;
/// `B_AX_STOP_CH10 | B_AX_STOP_CH11`.
const STOP2_BITS: u32 = 0x3;
const R_AX_PLATFORM_ENABLE: u32 = 0x0088;
const B_AX_AXIDMA_EN: u32 = 1 << 3;

const R_AX_WDE_PKTBUF_CFG: u32 = 0x8C08;
const R_AX_PLE_PKTBUF_CFG: u32 = 0x9008;
const B_PKTBUF_PAGE_SEL_MASK: u32 = 0x3;
const B_PKTBUF_START_BOUND_MASK: u32 = 0x3F00;
const B_PKTBUF_FREE_PAGE_NUM_MASK: u32 = 0x1FFF_0000;
/// `S_AX_WDE_PAGE_SEL_64` / `S_AX_PLE_PAGE_SEL_128` (`mac.h:605-614`).
const PAGE_SEL_64: u32 = 0;
const PAGE_SEL_128: u32 = 1;
/// `DLE_BOUND_UNIT` (`mac.h:597`).
const DLE_BOUND_UNIT: u32 = 8 * 1024;
const R_AX_WDE_QTA0_CFG: u32 = 0x8C40;
const R_AX_WDE_QTA1_CFG: u32 = 0x8C44;
const R_AX_WDE_QTA3_CFG: u32 = 0x8C4C;
const R_AX_WDE_QTA4_CFG: u32 = 0x8C50;
const R_AX_PLE_QTA0_CFG: u32 = 0x9040;
/// `B_AX_{WDE,PLE}_MIN_SIZE_MASK` GENMASK(11, 0), `..._MAX_SIZE_MASK`
/// GENMASK(27, 16) (`reg.h:1373`, `:1577`).
const QTA_MIN_MASK: u32 = 0x0FFF;
const QTA_MAX_MASK: u32 = 0x0FFF_0000;
const R_AX_WDE_INI_STATUS: u32 = 0x8D00;
const R_AX_PLE_INI_STATUS: u32 = 0x9100;
/// `WDE_MGN_INI_RDY` / `PLE_MGN_INI_RDY`: bits 1 and 0.
const MGN_INI_RDY: u32 = 0x3;

const R_AX_HCI_FC_CTRL_V1: u32 = 0x1700;
const R_AX_CH_PAGE_CTRL_V1: u32 = 0x1704;
const B_AX_HCI_FC_EN: u32 = 1 << 0;
const B_AX_HCI_FC_CH12_EN: u32 = 1 << 3;
/// `B_AX_HCI_FC_CH12_FULL_COND_MASK` GENMASK(11, 10).
const B_AX_HCI_FC_CH12_FULL_COND_MASK: u32 = 0x3 << 10;
/// `B_AX_PREC_PAGE_CH12_MASK` GENMASK(24, 16).
const B_AX_PREC_PAGE_CH12_MASK: u32 = 0x1FF << 16;

// `rtw89_dle_mem` for 8852C, PCIe, `RTW89_QTA_DLFW` (`rtw8852c.c:96-108`,
// `rtw8852c_dle_mem_pcie[RTW89_QTA_DLFW]`): `wde_size18`, `ple_size18`,
// `wde_qt17` (min and max), `ple_qt44` (min), `ple_qt45` (max).

/// `wde_size18 = {RTW89_WDE_PG_64, 0, 2048}` (`mac.c:1747`): page size.
const WDE_PAGE_BYTES: u32 = 64;
/// `wde_size18`: linked pages.
const WDE_LINKED_PAGES: u32 = 0;
/// `wde_size18`: unlinked pages.
const WDE_UNLINKED_PAGES: u32 = 2048;
/// `ple_size18 = {RTW89_PLE_PG_128, 2544, 16}` (`mac.c:1778`): page size.
const PLE_PAGE_BYTES: u32 = 128;
/// `ple_size18`: linked pages.
const PLE_LINKED_PAGES: u32 = 2544;
/// `ple_size18`: unlinked pages.
const PLE_UNLINKED_PAGES: u32 = 16;
/// `rtw8852c.c:3150`: `.fifo_size`, which `dle_used_size` (`mac.c:2041`)
/// must equal for DLFW (`dle_expected_used_size`, `mac.c:2058`;
/// `dle_scc_rsvd_size` is 0 and only applies to SCC).
const FIFO_BYTES: u32 = 458_752;
const _: () = assert!(
    WDE_PAGE_BYTES * (WDE_LINKED_PAGES + WDE_UNLINKED_PAGES)
        + PLE_PAGE_BYTES * (PLE_LINKED_PAGES + PLE_UNLINKED_PAGES)
        == FIFO_BYTES
);

/// `wde_qt17 = {0, 0, 0, 0}` (`mac.c:1811`) for both min and max, as
/// {hif, wcpu, pkt_in, cpu_io}. WDE QTA1's min comes from the extension
/// mode instead (`wde_quota_cfg_ax`, `ext_wde_min_qt_wcpu`): SCC's
/// `wde_qt18 = {3228, 60, 0, 40}` (`mac.c:1813`), whose `wcpu` is 60.
const WDE_QUOTA_ZERO: [u32; 4] = [0; 4];
const WDE_EXT_MIN_WCPU: u32 = 60;
/// `ple_qt44` (`mac.c:1857`, min) and `ple_qt45` (`:1860`, max) as
/// {cma0_tx, cma1_tx, c2h, h2c, wcpu, mpdu_proc, cma0_dma, cma1_dma, bb_rpt,
/// wd_rel, cpu_io, tx_rpt}. `tx_rpt` (QTA11) is written because the chip is
/// an RTL8852C.
const PLE_QUOTA_MIN: [u32; 12] = [0, 0, 16, 256, 0, 0, 0, 0, 0, 0, 0, 0];
const PLE_QUOTA_MAX: [u32; 12] = [0, 0, 32, 256, 0, 0, 0, 0, 0, 0, 0, 0];

/// `rtw89_mac_size.hfc_preccfg_pcie = {2, 40, 0, 0, 1, 0, 0, 0}`
/// (`mac.c:1717`): the H2C pre-cost pages and H2C full condition that
/// `hfc_h2c_cfg_ax` writes. `hfc_reset_param` (`mac.c:895`) reads the
/// parameter entry of `dle_info.qta_mode`, which is SCC here because
/// `rtw89_mac_dle_init` last looked up the SCC extension config; DLFW and
/// SCC share this `prec_cfg` (`rtw8852c.c:46-54`).
const HFC_H2C_PREC_PAGES: u32 = 40;
const HFC_H2C_FULL_COND: u32 = 0;

/// `rtw89_mac_check_mac_en_ax(RTW89_MAC_0, RTW89_DMAC_SEL)` (`mac.c:62`).
fn check_dmac_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let want = B_AX_MAC_FUNC_EN | B_AX_DMAC_FUNC_EN;
    let value = io.read32(R_AX_DMAC_FUNC_EN)?;
    // RTW89_R32_EA and RTW89_R32_DEAD (`core.h:51`).
    if value == 0xEAEA_EAEA || value == 0xDEAD_BEEF || value & want != want {
        return Err(AxError::Dma("DMAC function is not enabled"));
    }
    Ok(())
}

/// `rtw89_mac_hci_func_en_ax`, 8852C branch (`mac.c:4208`).
fn hci_func_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(
        R_AX_DMAC_FUNC_EN,
        B_AX_MAC_FUNC_EN
            | B_AX_DMAC_FUNC_EN
            | B_AX_DISPATCHER_EN
            | B_AX_PKT_BUF_EN
            | B_AX_H_AXIDMA_EN,
    )
}

/// `rtw89_mac_dmac_func_pre_en_ax`, 8852C PCIe branch (`mac.c:4222`).
fn dmac_func_pre_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(R_AX_DMAC_CLK_EN, B_AX_DISPATCHER_CLK_EN)?;

    let mut val = io.read32(R_AX_HAXI_INIT_CFG1)?;
    val &= !(B_AX_DMA_MODE_MASK | B_AX_STOP_AXI_MST);
    val |= B_AX_TXHCI_EN_V1 | B_AX_RXHCI_EN_V1;
    val |= (DMA_MOD_PCIE_1B << B_AX_DMA_MODE_MASK.trailing_zeros()) & B_AX_DMA_MODE_MASK;
    io.write32(R_AX_HAXI_INIT_CFG1, val)?;

    clear32(io, R_AX_HAXI_DMA_STOP1, STOP1_BITS)?;
    clear32(io, R_AX_HAXI_DMA_STOP2, STOP2_BITS)?;
    set32(io, R_AX_PLATFORM_ENABLE, B_AX_AXIDMA_EN)
}

/// `dle_func_en_ax` (`mac.c:2069`).
fn dle_func_en<I: AxIo + ?Sized>(io: &mut I, enable: bool) -> Result<(), AxError> {
    let bits = B_AX_DLE_WDE_EN | B_AX_DLE_PLE_EN;
    if enable {
        set32(io, R_AX_DMAC_FUNC_EN, bits)
    } else {
        clear32(io, R_AX_DMAC_FUNC_EN, bits)
    }
}

/// `dle_clk_en_ax(enable = true)` (`mac.c:2079`); the RTL8851B-only
/// `B_AX_AXIDMA_CLK_EN` is not added.
fn dle_clk_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(
        io,
        R_AX_DMAC_CLK_EN,
        B_AX_DLE_WDE_CLK_EN | B_AX_DLE_PLE_CLK_EN,
    )
}

/// `dle_mix_cfg_ax` (`mac.c:2092`) for the DLFW sizes.
fn dle_mix_cfg<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let mut val = io.read32(R_AX_WDE_PKTBUF_CFG)?;
    val &= !(B_PKTBUF_PAGE_SEL_MASK | B_PKTBUF_START_BOUND_MASK | B_PKTBUF_FREE_PAGE_NUM_MASK);
    // WDE: page select 64 B, start bound 0, free pages = linked pages.
    val |= PAGE_SEL_64;
    val |= (WDE_LINKED_PAGES << B_PKTBUF_FREE_PAGE_NUM_MASK.trailing_zeros())
        & B_PKTBUF_FREE_PAGE_NUM_MASK;
    io.write32(R_AX_WDE_PKTBUF_CFG, val)?;

    // PLE starts where WDE's pages end, in 8 KiB units.
    let bound = (WDE_LINKED_PAGES + WDE_UNLINKED_PAGES) * WDE_PAGE_BYTES / DLE_BOUND_UNIT;
    let mut val = io.read32(R_AX_PLE_PKTBUF_CFG)?;
    val &= !(B_PKTBUF_PAGE_SEL_MASK | B_PKTBUF_START_BOUND_MASK | B_PKTBUF_FREE_PAGE_NUM_MASK);
    val |= PAGE_SEL_128;
    val |= (bound << B_PKTBUF_START_BOUND_MASK.trailing_zeros()) & B_PKTBUF_START_BOUND_MASK;
    val |= (PLE_LINKED_PAGES << B_PKTBUF_FREE_PAGE_NUM_MASK.trailing_zeros())
        & B_PKTBUF_FREE_PAGE_NUM_MASK;
    io.write32(R_AX_PLE_PKTBUF_CFG, val)
}

fn qta_value(min: u32, max: u32) -> u32 {
    (min & QTA_MIN_MASK) | ((max << QTA_MAX_MASK.trailing_zeros()) & QTA_MAX_MASK)
}

/// `wde_quota_cfg_ax` (`mac.c:2178`): QTA0 (hif), QTA1 (wcpu, with the
/// extension minimum), QTA3 (pkt_in), QTA4 (cpu_io). QTA2 is not written.
fn wde_quota_cfg<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let [hif, wcpu, pkt_in, cpu_io] = WDE_QUOTA_ZERO;
    io.write32(R_AX_WDE_QTA0_CFG, qta_value(hif, hif))?;
    io.write32(R_AX_WDE_QTA1_CFG, qta_value(WDE_EXT_MIN_WCPU, wcpu))?;
    io.write32(R_AX_WDE_QTA3_CFG, qta_value(pkt_in, pkt_in))?;
    io.write32(R_AX_WDE_QTA4_CFG, qta_value(cpu_io, cpu_io))
}

/// `ple_quota_cfg_ax` (`mac.c:2193`): QTA0 to QTA11 in order.
fn ple_quota_cfg<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    for (i, (min, max)) in PLE_QUOTA_MIN.iter().zip(PLE_QUOTA_MAX).enumerate() {
        io.write32(R_AX_PLE_QTA0_CFG + 4 * i as u32, qta_value(*min, max))?;
    }
    Ok(())
}

/// `rtw89_mac_dle_init(RTW89_QTA_DLFW, RTW89_QTA_SCC)` (`mac.c:2274`).
fn dle_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    dle_func_en(io, false)?;
    dle_clk_en(io)?;
    dle_mix_cfg(io)?;
    wde_quota_cfg(io)?;
    ple_quota_cfg(io)?;
    dle_func_en(io, true)?;
    // chk_dle_rdy_ax (`mac.c:2149`): read_poll_timeout(.., 1, 2000), WDE
    // first, then PLE.
    poll32(io, R_AX_WDE_INI_STATUS, MGN_INI_RDY, MGN_INI_RDY, 1, 2000)?;
    poll32(io, R_AX_PLE_INI_STATUS, MGN_INI_RDY, MGN_INI_RDY, 1, 2000)
}

/// `hfc_func_en_ax` (`mac.c:1178`) with `en = false`: clear `HCI_FC_EN`, and
/// set or clear `HCI_FC_CH12_EN` by `h2c_en`.
fn hfc_func_en<I: AxIo + ?Sized>(io: &mut I, h2c_en: bool) -> Result<(), AxError> {
    let mut val = io.read32(R_AX_HCI_FC_CTRL_V1)?;
    val &= !B_AX_HCI_FC_EN;
    if h2c_en {
        val |= B_AX_HCI_FC_CH12_EN;
    } else {
        val &= !B_AX_HCI_FC_CH12_EN;
    }
    io.write32(R_AX_HCI_FC_CTRL_V1, val)
}

/// `rtw89_mac_hfc_init(rtwdev, true, false, true)` (`mac.c:1194`): only the
/// `!en && h2c_en` early return, served by `hfc_h2c_cfg_ax` (`mac.c:1127`).
/// `hfc_reset_param` does no register I/O.
fn hfc_init_h2c_only<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    hfc_func_en(io, false)?;
    io.write32(
        R_AX_CH_PAGE_CTRL_V1,
        (HFC_H2C_PREC_PAGES << B_AX_PREC_PAGE_CH12_MASK.trailing_zeros())
            & B_AX_PREC_PAGE_CH12_MASK,
    )?;
    field32(
        io,
        R_AX_HCI_FC_CTRL_V1,
        B_AX_HCI_FC_CH12_FULL_COND_MASK,
        HFC_H2C_FULL_COND,
    )?;
    hfc_func_en(io, true)
}

/// `rtw89_mac_ctrl_hci_dma_trx(true)` (`mac.h:1642`) then
/// `rtw89_mac_dmac_pre_init` (`mac.c:4258`): the MAC side of
/// `rtw89_mac_partial_init`, before the PCI pre-init and firmware download.
pub fn pre_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(
        io,
        R_AX_HCI_FUNC_EN_V1,
        B_AX_HCI_TXDMA_EN | B_AX_HCI_RXDMA_EN,
    )?;
    hci_func_en(io)?;
    dmac_func_pre_en(io)?;
    dle_init(io)?;
    hfc_init_h2c_only(io)
}

#[cfg(test)]
mod tests;
