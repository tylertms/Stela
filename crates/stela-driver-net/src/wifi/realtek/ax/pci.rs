// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2020  Realtek Corporation
//! RTL8852CE PCIe pre-init, DMA rings and the CH12 (FWCMD) TX queue, from
//! `rtw89_pci_ops_mac_pre_init_ax` (`pci.c:3070-3144`) and the helpers it
//! calls that execute for an RTL8852C (`pci.c:47`, `:249-275`,
//! `:2091-2118`, `:2551`, `:2709-2858`, `:2867-2951`, `:2953`),
//! `rtw89_pci_ops_reset` / `rtw89_pci_reset_trx_rings` /
//! `rtw89_pci_init_wp_16sel` (`pci.c:1722`, `:1776-1896`) with the 8852C
//! tables `rtw89_pci_ch_dma_addr_set_v1` (`pci.c:1116`) and
//! `rtw89_bd_ram_table_dual` (`pci.c:1693`), the ring pools of
//! `rtw89_pci_alloc_tx_rings` / `rtw89_pci_alloc_rx_ring` /
//! `rtw89_pci_init_rx_bd` (`pci.c:3512-3800`), and the FWCMD path of
//! `rtw89_pci_fwcmd_submit` (`pci.c:1551`), `rtw89_pci_tx_bd_ring_update` /
//! `__rtw89_pci_tx_kick_off` (`pci.c:1332-1358`) and the reclaim read of
//! `rtw89_pci_txbd_recalc` (`pci.c:87`). Register addresses are from pinned
//! rtw89 `pci.h` and `reg.h`.
//!
//! Only the RTL8852C branches are ported; the DMI-quirk BER fix
//! (`rtw89_pci_ber`, Dell only) and the 8852A/885xB PHY steps are not. This
//! module does not grant interrupts, RX processing, TX on any channel but
//! CH12, LTR/ASPM/L1SS setup (`mac_post_init`) or firmware readiness: a
//! consumed TXBD proves only that the DMA engine read the packet.
use super::io::{AxError, AxIo, clear16, clear32, field32, poll32, poll32_atomic, set32};
use crate::bus::dma::{
    mapping::{Direction, DmaDomain, DmaError},
    memory::{MemoryError, PageSource},
};
use crate::wifi::realtek::beacon::{self, Allocation};
use alloc::{sync::Arc, vec::Vec};
use core::{
    cell::Cell,
    sync::atomic::{Ordering, fence},
};

/// Entries in every TX ring. The donor's ring length is software-chosen and
/// written to NUM, but `rtw89_pci_alloc_tx_rings` pins it to a multiple of 16
/// (`BUILD_BUG_ON(RTW89_PCI_TXBD_NUM_MAX % 16)`, `pci.c:3647`), so the
/// smallest length that keeps that invariant is used.
pub const TX_ENTRIES: usize = 16;
/// RX buffers per RX ring (RXQ and RPQ).
pub const RX_ENTRIES: usize = 8;
/// `RTW89_PCI_RX_BUF_SIZE` (`pci.h:1128`).
pub const RX_BUFFER_BYTES: usize = 11454 + 40 + 4;
/// Bytes of one CH12 packet slot. The largest FWDL packet is a 16-byte TX
/// descriptor plus 2020 section bytes; the header packet is smaller.
pub const FWCMD_SLOT_BYTES: usize = 4096;

/// `struct rtw89_pci_tx_bd_32` and `struct rtw89_pci_rx_bd_32` (`pci.h`).
const BD_BYTES: usize = 8;
const TX_CHANNELS: usize = 13;
const RX_CHANNELS: usize = 2;
const TX_RING_BYTES: usize = TX_ENTRIES * BD_BYTES;
const RX_RING_BYTES: usize = RX_ENTRIES * BD_BYTES;
/// Each RX BD ring starts on its own page of the RX pool, so every DESA is
/// page aligned whatever alignment the HAXI engine needs. (Linux's RPQ sits
/// 256 BDs = 2048 bytes into its pool.)
const RX_RING_STRIDE: usize = 4096;
const _: () = assert!(RX_RING_BYTES <= RX_RING_STRIDE);
/// `RTW89_TXCH_CH12`, the FWCMD channel.
const CH12: usize = 12;

const RTW89_PCI_TXBD_OPT_LS: u16 = 1 << 14;
/// `RTW89_PCI_TXBD_OPT_DMA_HI` / `RTW89_PCI_RXBD_OPT_DMA_HI`, GENMASK(13, 6).
const BD_OPT_DMA_HI_SHIFT: u32 = 6;
/// `TXBD_HW_IDX_MASK` (`pci.h:558`): the hardware read index of a TX ring.
const TXBD_HW_IDX_MASK: u32 = 0x0FFF_0000;

/// `enum rtw89_cv` (`core.h:201`).
const CHIP_CAV: u8 = 0;

// reg.h / pci.h (pinned rtw89).
const R_AX_SYS_SDIO_CTRL: u32 = 0x0070;
const B_AX_PCIE_DIS_L2_CTRL_LDO_HCI: u32 = 1 << 15;
const B_AX_PCIE_FORCE_PWR_NGAT: u32 = 1 << 13;
const R_AX_HCI_OPT_CTRL: u32 = 0x0074;
const BIT_WAKE_CTRL: u32 = 1 << 5;
const R_AX_HCI_BG_CTRL: u32 = 0x0078;
const R_AX_PMC_DBG_CTRL2: u32 = 0x00CC;
const B_AX_SYSON_DIS_PMCR_AX_WRMSK: u32 = 1 << 2;
const R_AX_PCIE_PS_CTRL_V1: u32 = 0x3008;
const B_AX_SEL_REQ_ENTR_L1: u32 = 1 << 2;
const B_AX_DMAC0_EXIT_L1_EN: u32 = 1 << 6;
const R_AX_PCIE_BG_CLR: u32 = 0x303C;
const B_AX_BG_CLR_ASYNC_M3: u32 = 1 << 4;
const R_RAC_DIRECT_OFFSET_G1: u32 = 0x3800;
const R_RAC_DIRECT_OFFSET_G2: u32 = 0x3880;
const RAC_ANA24: u32 = 0x24;
const B_AX_DEGLITCH: u16 = 0x0F00; // GENMASK(11, 8)
const R_AX_PCIE_IO_RCY_M1: u32 = 0x3100;
const R_AX_PCIE_WDT_TIMER_M1: u32 = 0x3104;
const R_AX_PCIE_IO_RCY_M2: u32 = 0x310C;
const R_AX_PCIE_WDT_TIMER_M2: u32 = 0x3110;
const R_AX_PCIE_IO_RCY_E0: u32 = 0x3118;
const R_AX_PCIE_WDT_TIMER_E0: u32 = 0x311C;
const R_AX_PCIE_IO_RCY_S1: u32 = 0x3124;
/// `B_AX_PCIE_IO_RCY_WDT_MODE_{M1,M2,E0,S1}` are all BIT(3).
const B_AX_PCIE_IO_RCY_WDT_MODE: u32 = 1 << 3;
/// `MAC_AX_IO_RCY_ANA_TMR_6MS` (`pci.h:1303`), `rtw8852c_pci_info.io_rcy_tmr`.
const IO_RCY_ANA_TMR_6MS: u32 = 72000;

const R_AX_HAXI_INIT_CFG1: u32 = 0x1000;
const B_AX_WD_ITVL_IDLE_V1_MASK: u32 = 0xF000_0000; // GENMASK(31, 28)
const B_AX_WD_ITVL_ACT_V1_MASK: u32 = 0x0F00_0000; // GENMASK(27, 24)
const B_AX_STOP_AXI_MST: u32 = 1 << 17;
const B_AX_RXHCI_EN_V1: u32 = 1 << 15;
const B_AX_RXBD_MODE_V1: u32 = 1 << 14;
const B_AX_HAXI_MAX_RXDMA_MASK: u32 = 0x0300; // GENMASK(9, 8)
const B_AX_TXHCI_EN_V1: u32 = 1 << 7;
const B_AX_RST_BDRAM: u32 = 1 << 3;
const B_AX_HAXI_MAX_TXDMA_MASK: u32 = 0x0003; // GENMASK(1, 0)
const R_AX_HAXI_DMA_STOP1: u32 = 0x1010;
const B_AX_STOP_WPDMA: u32 = 1 << 19;
const B_AX_STOP_CH12: u32 = 1 << 18;
/// `B_AX_TX_STOP1_MASK`: ACH0-7, CH8, CH9 and CH12 (bits 8-18).
const B_AX_TX_STOP1_MASK: u32 = 0x0007_FF00;
const R_AX_TXBD_RWPTR_CLR1: u32 = 0x1014;
/// ACH0-3, CH8, CH9, CH12 and (8852A/8852C) ACH4-7: bits 0-10.
const B_AX_CLR_TXBD_IDX1: u32 = 0x07FF;
const R_AX_HAXI_DMA_BUSY1: u32 = 0x101C;
/// `DMA_BUSY1_CHECK`: ACH0-7, CH8, CH9 and CH12 busy (bits 8-18).
const DMA_BUSY1_CHECK: u32 = 0x0007_FF00;
const R_AX_HAXI_DMA_STOP2: u32 = 0x11C0;
/// `B_AX_TX_STOP2_ALL`: CH10, CH11.
const B_AX_TX_STOP2_ALL: u32 = 0x3;
const R_AX_TXBD_RWPTR_CLR2_V1: u32 = 0x11C4;
/// `B_AX_CLR_CH10_IDX | B_AX_CLR_CH11_IDX`.
const B_AX_CLR_TXBD_IDX2: u32 = 0x3;
const R_AX_HAXI_DMA_BUSY2: u32 = 0x11C8;
/// `B_AX_CH10_BUSY | B_AX_CH11_BUSY`.
const DMA_BUSY2_CHECK: u32 = 0x3;
const R_AX_RXBD_RWPTR_CLR_V1: u32 = 0x1200;
/// `B_AX_CLR_RXQ_IDX | B_AX_CLR_RPQ_IDX`.
const B_AX_CLR_RXBD_IDX: u32 = 0x3;
const R_AX_HAXI_EXP_CTRL: u32 = 0x1204;
const B_AX_MAX_TAG_NUM_V1_MASK: u32 = 0x7; // GENMASK(2, 0)
const R_AX_HAXI_DMA_BUSY3: u32 = 0x1208;
/// `B_AX_RXQ_BUSY | B_AX_RPQ_BUSY`.
const DMA_BUSY3_CHECK: u32 = 0x3;
const R_AX_WP_ADDR_H_SEL0_3: u32 = 0x1334;
const R_AX_TX_ADDRESS_INFO_MODE_SETTING: u32 = 0x8810;
const B_AX_HOST_ADDR_INFO_8B_SEL: u32 = 1 << 0;
const R_AX_PKTIN_SETTING: u32 = 0x9A00;
const B_AX_WD_ADDR_INFO_LENGTH: u32 = 1 << 1;

// rtw8852c_pci_info (rtw8852ce.c:21-79) mode values (pci.h enums).
/// `MAC_AX_TX_BURST_V1_256B`.
const TX_BURST_V1_256B: u32 = 2;
/// `MAC_AX_RX_BURST_V1_128B`.
const RX_BURST_V1_128B: u32 = 1;
/// `MAC_AX_TAG_NUM_8`.
const TAG_NUM_8: u32 = 7;
/// `MAC_AX_WD_DMA_INTVL_256NS`, both idle and active.
const WD_DMA_INTVL_256NS: u32 = 1;

/// `read_poll_timeout(..., 10, 100, ...)` in the DMA idle polls.
const IDLE_POLL_INTERVAL_US: u64 = 10;
const IDLE_POLL_TIMEOUT_US: u64 = 100;
/// `read_poll_timeout_atomic(..., 1, RTW89_PCI_POLL_BDRAM_RST_CNT, ...)`.
const BDRAM_POLL_TIMEOUT_US: u64 = 100;
/// The CH12 reclaim wait: 1 us samples for up to the 400 ms FWDL budget.
const RECLAIM_INTERVAL_US: u64 = 1;
const RECLAIM_TIMEOUT_US: u64 = 400_000;

/// One TX channel's registers (`rtw89_pci_ch_dma_addr_set_v1`) and BD RAM
/// split (`rtw89_bd_ram_table_dual`).
struct TxChannel {
    num: u32,
    idx: u32,
    bdram: u32,
    desa_l: u32,
    start_idx: u32,
    max_num: u32,
    min_num: u32,
}

const fn tx(num: u32, idx: u32, bdram: u32, desa_l: u32, bd_ram: [u32; 3]) -> TxChannel {
    TxChannel {
        num,
        idx,
        bdram,
        desa_l,
        start_idx: bd_ram[0],
        max_num: bd_ram[1],
        min_num: bd_ram[2],
    }
}

/// `RTW89_TXCH_ACH0` .. `RTW89_TXCH_CH12`, in the donor's loop order.
/// `DEF_TXCHADDRS(info, X, _V1)` keeps the plain NUM/IDX and takes the `_V1`
/// BDRAM/DESA; `DEF_TXCHADDRS_TYPE1` (CH10, CH11) takes `_V1` for all.
const TX_TABLE: [TxChannel; TX_CHANNELS] = [
    tx(0x1024, 0x1058, 0x1300, 0x1230, [0, 5, 2]),  // ACH0
    tx(0x1026, 0x105C, 0x1304, 0x1238, [5, 5, 2]),  // ACH1
    tx(0x1028, 0x1060, 0x1308, 0x1240, [10, 5, 2]), // ACH2
    tx(0x102A, 0x1064, 0x130C, 0x1248, [15, 5, 2]), // ACH3
    tx(0x102C, 0x1068, 0x1310, 0x1250, [20, 5, 2]), // ACH4
    tx(0x102E, 0x106C, 0x1314, 0x1258, [25, 5, 2]), // ACH5
    tx(0x1030, 0x1070, 0x1318, 0x1260, [30, 5, 2]), // ACH6
    tx(0x1032, 0x1074, 0x131C, 0x1268, [35, 5, 2]), // ACH7
    tx(0x1034, 0x1078, 0x1320, 0x1270, [40, 5, 1]), // CH8
    tx(0x1036, 0x107C, 0x1324, 0x1278, [45, 5, 1]), // CH9
    tx(0x1438, 0x11D0, 0x1420, 0x1458, [50, 5, 1]), // CH10
    tx(0x143A, 0x11D4, 0x1424, 0x1460, [55, 5, 1]), // CH11
    tx(0x1038, 0x1080, 0x1328, 0x1280, [60, 4, 1]), // CH12 (FWCMD)
];

/// `DEF_RXCHADDRS(AX, RXQ/RPQ, ..., _V1)`: NUM, IDX, DESA_L.
const RX_TABLE: [(u32, u32, u32); RX_CHANNELS] = [
    (0x1210, 0x1218, 0x1220), // RXQ
    (0x1212, 0x121C, 0x1228), // RPQ
];

/// The 8-byte `struct rtw89_pci_tx_bd_32` of `rtw89_pci_fwcmd_submit`:
/// le16 length, le16 opt = LS | dma[39:32] << 6, le32 dma[31:0].
pub fn encode_txbd(len: u16, dma: u64) -> [u8; 8] {
    encode_bd(len, RTW89_PCI_TXBD_OPT_LS, dma)
}

/// `struct rtw89_pci_rx_bd_32` as `rtw89_pci_init_rx_bd` fills it.
fn encode_rxbd(buf_size: u16, dma: u64) -> [u8; 8] {
    encode_bd(buf_size, 0, dma)
}

fn encode_bd(len: u16, flags: u16, dma: u64) -> [u8; 8] {
    let high = ((dma >> 32) & 0xFF) as u16;
    let opt = flags | high << BD_OPT_DMA_HI_SHIFT;
    let mut bd = [0; BD_BYTES];
    bd[0..2].copy_from_slice(&len.to_le_bytes());
    bd[2..4].copy_from_slice(&opt.to_le_bytes());
    bd[4..8].copy_from_slice(&(dma as u32).to_le_bytes());
    bd
}

impl From<beacon::Error> for AxError {
    fn from(error: beacon::Error) -> Self {
        match error {
            beacon::Error::Memory(MemoryError::OutOfMemory) => Self::Dma("out of ring memory"),
            beacon::Error::Memory(_) => Self::Dma("ring memory unavailable"),
            beacon::Error::Dma(error) => error.into(),
            beacon::Error::Address => Self::Dma("ring address above 4 GiB"),
            _ => Self::Dma("ring allocation failed"),
        }
    }
}

impl From<DmaError> for AxError {
    fn from(error: DmaError) -> Self {
        Self::Dma(match error {
            DmaError::AddressLimit => "ring memory above the device's address limit",
            DmaError::VisibilityFailed => "ring memory visibility failed",
            DmaError::Unmapped => "ring memory is unmapped",
            _ => "ring memory mapping failed",
        })
    }
}

/// The DMA memory behind every rtw89 ring: one pool of 13 TX BD rings, one
/// pool of the RXQ and RPQ BD rings with their buffers, and the CH12 packet
/// slots. All of it lies below 4 GiB.
///
/// Once `pre_init` has written a ring address to the device, the memory is
/// never returned: dropping the rings then leaves every mapping in
/// `DmaMapping`'s quarantine, because a drop cannot prove the DMA stopped.
pub struct Rings {
    tx: Allocation,
    rx: Allocation,
    rx_buffers: Vec<Allocation>,
    fwcmd: Allocation,
    /// A ring address may have reached the device.
    programmed: Cell<bool>,
    /// `bd_ring->wp` of CH12.
    ch12_host: Cell<u16>,
    /// CH12 is started by `pre_init` and has no entry in flight.
    ch12_ready: Cell<bool>,
}

impl Rings {
    /// Allocate and fill every ring as `rtw89_pci_alloc_trx_rings` does:
    /// zeroed TX BDs, and each RX BD pointing at a zeroed buffer of
    /// `RX_BUFFER_BYTES`. Nothing reaches the device until `pre_init`.
    ///
    /// # Safety
    /// `domain` isolates exactly the RTL8852C PCI requester whose register
    /// window these rings will be given to by `pre_init`. `source` yields
    /// real coherent WB DMA RAM for that requester, with NoSnoop and ATS
    /// disabled (WB alone is not coherence proof). One worker owns the
    /// device and these rings; no other engine, IRQ or reset worker touches
    /// them.
    pub unsafe fn install(
        source: &dyn PageSource,
        domain: Arc<dyn DmaDomain>,
    ) -> Result<Self, AxError> {
        let tx = Allocation::new(
            source,
            domain.clone(),
            TX_CHANNELS * TX_RING_BYTES,
            Direction::Bidirectional,
        )?;
        let rx = Allocation::new(
            source,
            domain.clone(),
            RX_CHANNELS * RX_RING_STRIDE,
            Direction::Bidirectional,
        )?;
        let mut rx_buffers = Vec::new();
        rx_buffers
            .try_reserve_exact(RX_CHANNELS * RX_ENTRIES)
            .map_err(|_| AxError::Dma("out of ring memory"))?;
        for _ in 0..RX_CHANNELS * RX_ENTRIES {
            rx_buffers.push(Allocation::new(
                source,
                domain.clone(),
                RX_BUFFER_BYTES,
                Direction::FromDevice,
            )?);
        }
        let fwcmd = Allocation::new(
            source,
            domain,
            TX_ENTRIES * FWCMD_SLOT_BYTES,
            Direction::ToDevice,
        )?;
        let rings = Self {
            tx,
            rx,
            rx_buffers,
            fwcmd,
            programmed: Cell::new(false),
            ch12_host: Cell::new(0),
            ch12_ready: Cell::new(false),
        };
        zero(&rings.tx, TX_CHANNELS * TX_RING_BYTES)?;
        rings
            .tx
            .dma
            .sync_for_device(0..TX_CHANNELS * TX_RING_BYTES)?;
        zero(&rings.rx, RX_CHANNELS * RX_RING_STRIDE)?;
        // RX ring r entry e is buffer r * RX_ENTRIES + e; its BD is entry e
        // of the ring that starts at page r of the pool.
        for (i, buffer) in rings.rx_buffers.iter().enumerate() {
            // `memset(skb->data, 0, buf_sz)` before `dma_map_single`.
            zero(buffer, RX_BUFFER_BYTES)?;
            buffer.dma.sync_for_device(0..RX_BUFFER_BYTES)?;
            let bd = encode_rxbd(RX_BUFFER_BYTES as u16, u64::from(buffer.address()?));
            let at = (i / RX_ENTRIES) * RX_RING_STRIDE + (i % RX_ENTRIES) * BD_BYTES;
            rings.rx.write(at, &bd)?;
        }
        rings
            .rx
            .dma
            .sync_for_device(0..RX_CHANNELS * RX_RING_STRIDE)?;
        Ok(rings)
    }

    /// The firmware command queue. `pre_init` must have succeeded first.
    pub fn ch12(&mut self) -> Ch12<'_> {
        Ch12 { rings: self }
    }

    /// CH12 refuses packets: `pre_init` has not succeeded yet, or a submit
    /// failed after handing its packet over. Only another `pre_init` starts
    /// it again.
    pub fn ch12_stopped(&self) -> bool {
        !self.ch12_ready.get()
    }

    fn tx_ring(&self, channel: usize) -> Result<u32, AxError> {
        Ok(self.tx.address()? + (channel * TX_RING_BYTES) as u32)
    }

    fn rx_ring(&self, channel: usize) -> Result<u32, AxError> {
        Ok(self.rx.address()? + (channel * RX_RING_STRIDE) as u32)
    }
}

impl Drop for Rings {
    fn drop(&mut self) {
        if self.programmed.get() {
            // The device may still reach any ring: keep every mapping.
            self.tx.published = true;
            self.rx.published = true;
            self.fwcmd.published = true;
            for buffer in &mut self.rx_buffers {
                buffer.published = true;
            }
        }
    }
}

fn zero(allocation: &Allocation, bytes: usize) -> Result<(), AxError> {
    const CHUNK: [u8; 512] = [0; 512];
    let mut at = 0;
    while at < bytes {
        let n = CHUNK.len().min(bytes - at);
        allocation.write(at, &CHUNK[..n])?;
        at += n;
    }
    Ok(())
}

/// `rtw89_pci_ops_mac_pre_init_ax` for an RTL8852C of chip cut `cut`
/// (`enum rtw89_cv`, 0 = CAV): PCIe PHY and power settings, stop and drain
/// all DMA, clear every ring index, set the DMA mode, program every ring of
/// `rings`, reset the BD RAM and start DMA with only CH12 enabled.
///
/// Runs after the MAC's DMAC pre-init, as `rtw89_mac_partial_init` orders
/// them; nothing here assumes it runs first. A failure leaves CH12 stopped.
///
/// # Safety
/// `io` is the exclusive register window of exactly the RTL8852C requester
/// that `rings`' DMA domain isolates, and `rings` was installed on real
/// coherent WB RAM for it with NoSnoop and ATS disabled. This publishes
/// every ring address to that device and starts its DMA (RX included).
/// One worker owns the device and the queues: no other engine, IRQ or reset
/// worker touches them.
pub unsafe fn pre_init<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &Rings,
) -> Result<(), AxError> {
    rings.ch12_ready.set(false);
    // rtw89_pci_ber: only on the Dell RTW89_QUIRK_PCI_BER DMI models.
    // rtw89_pci_rxdma_prefth, _l1off_pwroff, _l2_rxen_lat, _aphy_pwrcut,
    // _dphy_delay, _autok_x, _auto_refclk_cal: 8852A/885xB only.
    // rtw89_pci_deglitch_setting (pci.c:2551), RTL8852C branch.
    clear16(io, R_RAC_DIRECT_OFFSET_G1 + RAC_ANA24 * 2, B_AX_DEGLITCH)?;
    clear16(io, R_RAC_DIRECT_OFFSET_G2 + RAC_ANA24 * 2, B_AX_DEGLITCH)?;
    // rtw89_pci_hci_ldo (pci.c:2709), RTL8852C branch.
    clear32(io, R_AX_SYS_SDIO_CTRL, B_AX_PCIE_DIS_L2_CTRL_LDO_HCI)?;
    // rtw89_pci_power_wake_ax(true) (pci.c:2733).
    set32(io, R_AX_HCI_OPT_CTRL, BIT_WAKE_CTRL)?;
    // rtw89_pci_autoload_hang (pci.c:2741).
    set32(io, R_AX_PCIE_BG_CLR, B_AX_BG_CLR_ASYNC_M3)?;
    clear32(io, R_AX_PCIE_BG_CLR, B_AX_BG_CLR_ASYNC_M3)?;
    if cut == CHIP_CAV {
        // rtw89_pci_l12_vmain (pci.c:2750).
        set32(io, R_AX_SYS_SDIO_CTRL, B_AX_PCIE_FORCE_PWR_NGAT)?;
        // rtw89_pci_gen2_force_ib (pci.c:2758).
        set32(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;
        set32(io, R_AX_HCI_BG_CTRL, B_AX_BG_CLR_ASYNC_M3)?;
        clear32(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;
    }
    // rtw89_pci_l1_ent_lat (pci.c:2770) and rtw89_pci_wd_exit_l1 (:2778).
    clear32(io, R_AX_PCIE_PS_CTRL_V1, B_AX_SEL_REQ_ENTR_L1)?;
    set32(io, R_AX_PCIE_PS_CTRL_V1, B_AX_DMAC0_EXIT_L1_EN)?;
    // rtw89_pci_set_sic, _set_lbc, _set_dbg, _set_keep_reg skip 8852C.
    // rtw89_pci_set_io_rcy (pci.c:2814), io_rcy_en enabled.
    for timer in [
        R_AX_PCIE_WDT_TIMER_M1,
        R_AX_PCIE_WDT_TIMER_M2,
        R_AX_PCIE_WDT_TIMER_E0,
    ] {
        io.write32(timer, IO_RCY_ANA_TMR_6MS)?;
    }
    for mode in [
        R_AX_PCIE_IO_RCY_M1,
        R_AX_PCIE_IO_RCY_M2,
        R_AX_PCIE_IO_RCY_E0,
    ] {
        set32(io, mode, B_AX_PCIE_IO_RCY_WDT_MODE)?;
    }
    clear32(io, R_AX_PCIE_IO_RCY_S1, B_AX_PCIE_IO_RCY_WDT_MODE)?;

    // pci.c:3113-3123.
    stop_dma(io)?;
    poll_dma_all_idle(io)?;
    clr_idx_all(io)?;
    mode_op(io)?;
    // rtw89_pci_ops_reset: ring registers and the software indices.
    reset_trx_rings(io, rings)?;
    // rtw89_pci_rst_bdram_ax (pci.c:47).
    rst_bdram(io)?;
    // rtw89_pci_ctrl_txdma_ch_ax(false), then _fw_ch_ax(true) (pci.c:249-275).
    set32(io, R_AX_HAXI_DMA_STOP1, B_AX_TX_STOP1_MASK)?;
    set32(io, R_AX_HAXI_DMA_STOP2, B_AX_TX_STOP2_ALL)?;
    clear32(io, R_AX_HAXI_DMA_STOP1, B_AX_STOP_CH12)?;
    ctrl_dma_all(io, true)?;
    rings.ch12_ready.set(true);
    Ok(())
}

/// Stop the PCIe side's DMA as `rtw89_pci_ops_mac_pre_init_ax` does first
/// (`pci.c:3113-3116`): `B_AX_STOP_WPDMA` in `R_AX_HAXI_DMA_STOP1`, then
/// `rtw89_pci_ctrl_dma_all(false)`: AXI master stopped and the TX/RX HCI
/// DMA disabled. Nothing is waited for.
pub(crate) fn stop_dma<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_HAXI_DMA_STOP1, B_AX_STOP_WPDMA)?;
    ctrl_dma_all(io, false)
}

/// `rtw89_pci_ctrl_dma_all` (`pci.c:2114`): `dma_io_stop`, then the
/// RX/TX HCI enables of `init_cfg_reg`.
fn ctrl_dma_all<I: AxIo + ?Sized>(io: &mut I, enable: bool) -> Result<(), AxError> {
    let hci = B_AX_RXHCI_EN_V1 | B_AX_TXHCI_EN_V1;
    if enable {
        clear32(io, R_AX_HAXI_INIT_CFG1, B_AX_STOP_AXI_MST)?;
        set32(io, R_AX_HAXI_INIT_CFG1, hci)
    } else {
        set32(io, R_AX_HAXI_INIT_CFG1, B_AX_STOP_AXI_MST)?;
        clear32(io, R_AX_HAXI_INIT_CFG1, hci)
    }
}

/// `rtw89_pci_poll_dma_all_idle` (`pci.c:2934`): TX busy1, TX busy2 (CH10,
/// CH11), then RX busy3.
fn poll_dma_all_idle<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    for (reg, mask) in [
        (R_AX_HAXI_DMA_BUSY1, DMA_BUSY1_CHECK),
        (R_AX_HAXI_DMA_BUSY2, DMA_BUSY2_CHECK),
        (R_AX_HAXI_DMA_BUSY3, DMA_BUSY3_CHECK),
    ] {
        poll32(
            io,
            reg,
            mask,
            0,
            IDLE_POLL_INTERVAL_US,
            IDLE_POLL_TIMEOUT_US,
        )?;
    }
    Ok(())
}

/// `rtw89_pci_rst_bdram_ax` (`pci.c:47-54`): set the reset bit, then
/// `read_poll_timeout_atomic(.., 1, 100, false)` for it to clear.
fn rst_bdram<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_HAXI_INIT_CFG1, B_AX_RST_BDRAM)?;
    poll32_atomic(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_RST_BDRAM,
        0,
        1,
        BDRAM_POLL_TIMEOUT_US,
    )
}

/// `rtw89_pci_clr_idx_all_ax` (`pci.c:2867`), RTL8852C channel set.
fn clr_idx_all<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_TXBD_RWPTR_CLR1, B_AX_CLR_TXBD_IDX1)?;
    set32(io, R_AX_TXBD_RWPTR_CLR2_V1, B_AX_CLR_TXBD_IDX2)?;
    set32(io, R_AX_RXBD_RWPTR_CLR_V1, B_AX_CLR_RXBD_IDX)
}

/// `rtw89_pci_mode_op` (`pci.c:2953`) for `rtw8852c_pci_info`: BD trunc
/// mode (no 8852C register), RXBD packet mode, burst sizes, tag count, WD
/// DMA intervals, then the trunc-mode address-info settings.
fn mode_op<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    clear32(io, R_AX_HAXI_INIT_CFG1, B_AX_RXBD_MODE_V1)?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_HAXI_MAX_TXDMA_MASK,
        TX_BURST_V1_256B,
    )?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_HAXI_MAX_RXDMA_MASK,
        RX_BURST_V1_128B,
    )?;
    field32(io, R_AX_HAXI_EXP_CTRL, B_AX_MAX_TAG_NUM_V1_MASK, TAG_NUM_8)?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_WD_ITVL_IDLE_V1_MASK,
        WD_DMA_INTVL_256NS,
    )?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_WD_ITVL_ACT_V1_MASK,
        WD_DMA_INTVL_256NS,
    )?;
    set32(
        io,
        R_AX_TX_ADDRESS_INFO_MODE_SETTING,
        B_AX_HOST_ADDR_INFO_8B_SEL,
    )?;
    clear32(io, R_AX_PKTIN_SETTING, B_AX_WD_ADDR_INFO_LENGTH)
}

/// `rtw89_pci_reset_trx_rings` (`pci.c:1776`) without `group_bd_addr`, with
/// `rx_ring_eq_is_full` false, then `rtw89_pci_init_wp_16sel`.
fn reset_trx_rings<I: AxIo + ?Sized>(io: &mut I, rings: &Rings) -> Result<(), AxError> {
    let mut tx = [0; TX_CHANNELS];
    for (channel, address) in tx.iter_mut().enumerate() {
        *address = rings.tx_ring(channel)?;
    }
    let rx = [rings.rx_ring(0)?, rings.rx_ring(1)?];
    rings.ch12_host.set(0);
    // The first ring address store publishes the rings.
    rings.programmed.set(true);
    for (channel, address) in TX_TABLE.iter().zip(tx) {
        io.write16(channel.num, TX_ENTRIES as u16)?;
        let bd_ram = channel.start_idx | channel.max_num << 8 | channel.min_num << 16;
        io.write32(channel.bdram, bd_ram)?;
        io.write32(channel.desa_l, address)?;
        io.write32(channel.desa_l + 4, 0)?;
    }
    for (&(num, _idx, desa_l), address) in RX_TABLE.iter().zip(rx) {
        io.write16(num, RX_ENTRIES as u16)?;
        io.write32(desa_l, address)?;
        io.write32(desa_l + 4, 0)?;
    }
    for i in (0..16u32).step_by(4) {
        let value = i | (i + 1) << 8 | (i + 2) << 16 | (i + 3) << 24;
        io.write32(R_AX_WP_ADDR_H_SEL0_3 + i, value)?;
    }
    Ok(())
}

/// The CH12 (FWCMD) TX queue: one packet in flight at a time.
pub struct Ch12<'r> {
    rings: &'r mut Rings,
}

impl Ch12<'_> {
    /// Send `packet` (its TX descriptor included) as `rtw89_pci_fwcmd_submit`
    /// and `__rtw89_pci_tx_kick_off` do, then wait until the hardware read
    /// index (`IDX` [27:16]) has passed the entry (1 us samples, 400 ms).
    ///
    /// Any failure after the length check (the slot or TXBD write, a sync,
    /// the kick or the reclaim wait) leaves the queue refusing further
    /// packets until the next successful `pre_init`: it is marked stopped
    /// before the slot is written.
    ///
    /// # Safety
    /// `io` is the exclusive register window of the same RTL8852C requester
    /// that `pre_init` programmed with these rings, and the `pre_init`
    /// contract still holds: its DMA domain isolates that requester, the
    /// rings are coherent WB RAM with NoSnoop and ATS disabled, and one
    /// worker owns the device and this queue.
    pub unsafe fn submit<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        packet: &[u8],
    ) -> Result<(), AxError> {
        let rings = &mut *self.rings;
        if !rings.ch12_ready.get() {
            return Err(AxError::Dma("CH12 is stopped"));
        }
        if packet.is_empty() || packet.len() > FWCMD_SLOT_BYTES {
            return Err(AxError::Dma("CH12 packet length out of range"));
        }
        let slot = usize::from(rings.ch12_host.get());
        let at = slot * FWCMD_SLOT_BYTES;
        let dma = u64::from(rings.fwcmd.address()?) + at as u64;
        let bd = (CH12 * TX_ENTRIES + slot) * BD_BYTES;
        rings.ch12_ready.set(false);
        rings.fwcmd.write(at, packet)?;
        rings.fwcmd.dma.sync_for_device(at..at + packet.len())?;
        rings.tx.write(bd, &encode_txbd(packet.len() as u16, dma))?;
        rings.tx.dma.sync_for_device(bd..bd + BD_BYTES)?;
        fence(Ordering::Release);
        // rtw89_pci_tx_bd_ring_update: advance and wrap the host index.
        let host = ((slot + 1) % TX_ENTRIES) as u16;
        rings.ch12_host.set(host);
        let idx = TX_TABLE[CH12].idx;
        io.write16(idx, host)?;
        poll32(
            io,
            idx,
            TXBD_HW_IDX_MASK,
            u32::from(host) << 16,
            RECLAIM_INTERVAL_US,
            RECLAIM_TIMEOUT_US,
        )?;
        fence(Ordering::Acquire);
        // SAFETY: the hardware read index has passed this entry, so the DMA
        // engine has finished reading the TXBD, and it cannot read the slot
        // again until the host index wraps back to it on a later submit.
        unsafe { rings.tx.dma.sync_for_cpu(bd..bd + BD_BYTES) }?;
        rings.ch12_ready.set(true);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
