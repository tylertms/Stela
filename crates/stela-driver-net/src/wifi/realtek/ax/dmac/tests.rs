extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec::Vec};

/// Both DLE managers report ready, as the hardware's do.
fn ready() -> RegisterModel {
    let mut io = RegisterModel::new();
    io.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 3));
    io.on_read(0x9100, Box::new(|_: &mut RegisterModel| 3));
    io
}

#[test]
fn dmac_pre_init_matches_linux_sequence() {
    let mut m = ready();
    // Non-zero starting values, so every read-modify-write is visible.
    m.set32(0x7880, 0x0000_0004);
    m.set32(0x1000, 0x000E_0001);
    m.set32(0x1010, 0x000F_FFFF);
    m.set32(0x11C0, 0x0000_000F);
    m.set32(0x0088, 0x0000_0002);
    m.set32(0x8C08, 0xFFFF_FFFF);
    m.set32(0x9008, 0x8000_0000);
    m.set32(0x1700, 0x0000_0FFF);

    pre_init(&mut m).unwrap();

    let mut want: Vec<Op> = Vec::new();
    // rtw89_mac_ctrl_hci_dma_trx(true), mac.h:1642: set TXDMA|RXDMA in
    // R_AX_HCI_FUNC_EN_V1 (0x7880).
    want.extend([Op::R32(0x7880), Op::W32(0x7880, 0x0000_0007)]);
    // rtw89_mac_hci_func_en_ax, mac.c:4208 (8852C): a plain write of
    // MAC|DMAC|DISPATCHER|PKT_BUF|H_AXIDMA.
    want.push(Op::W32(0x8400, 0x6044_4000));
    // rtw89_mac_dmac_func_pre_en_ax, mac.c:4222: a plain write of CLK_EN.
    want.push(Op::W32(0x8404, 0x0004_0000));
    // R_AX_HAXI_INIT_CFG1: clear DMA_MODE [19:18] and STOP_AXI_MST (17), set
    // TXHCI_EN_V1 (7), RXHCI_EN_V1 (15), DMA mode PCIE_1B = 0.
    want.extend([Op::R32(0x1000), Op::W32(0x1000, 0x0000_8081)]);
    // R_AX_HAXI_DMA_STOP1: clear ACH0-7, CH8, CH9, CH12 (0x7FF00).
    want.extend([Op::R32(0x1010), Op::W32(0x1010, 0x0008_00FF)]);
    // R_AX_HAXI_DMA_STOP2: clear CH10, CH11.
    want.extend([Op::R32(0x11C0), Op::W32(0x11C0, 0x0000_000C)]);
    // R_AX_PLATFORM_ENABLE: set AXIDMA_EN (bit 3).
    want.extend([Op::R32(0x0088), Op::W32(0x0088, 0x0000_000A)]);
    // rtw89_mac_dle_init(DLFW), mac.c:2274: check_mac_en_ax reads DMAC_FUNC_EN.
    want.push(Op::R32(0x8400));
    // dle_func_en_ax(false), mac.c:2069: clear DLE_WDE_EN (26) | DLE_PLE_EN
    // (23), which the plain write above left clear.
    want.extend([Op::R32(0x8400), Op::W32(0x8400, 0x6044_4000)]);
    // dle_clk_en_ax(true), mac.c:2079: set WDE_CLK_EN (26) | PLE_CLK_EN (23).
    want.extend([Op::R32(0x8404), Op::W32(0x8404, 0x0484_0000)]);
    // dle_mix_cfg_ax, mac.c:2092. WDE: page 64 B (sel 0), start bound 0,
    // free (linked) pages 0, over an all-ones start.
    want.extend([Op::R32(0x8C08), Op::W32(0x8C08, 0xE000_C0FC)]);
    // PLE: page 128 B (sel 1), bound = 2048 * 64 / 8192 = 16, free 2544.
    want.extend([Op::R32(0x9008), Op::W32(0x9008, 0x89F0_1001)]);
    // wde_quota_cfg_ax, mac.c:2178: QTA0 hif, QTA1 wcpu (min = SCC's 60),
    // QTA3 pkt_in, QTA4 cpu_io. QTA2 is skipped.
    want.extend([
        Op::W32(0x8C40, 0),
        Op::W32(0x8C44, 60),
        Op::W32(0x8C4C, 0),
        Op::W32(0x8C50, 0),
    ]);
    // ple_quota_cfg_ax, mac.c:2193: QTA0..QTA11 = min | max << 16 from
    // ple_qt44 (min, mac.c:1857) and ple_qt45 (max, mac.c:1860).
    want.extend([
        Op::W32(0x9040, 0),
        Op::W32(0x9044, 0),
        Op::W32(0x9048, 16 | 32 << 16),   // c2h
        Op::W32(0x904C, 256 | 256 << 16), // h2c
        Op::W32(0x9050, 0),
        Op::W32(0x9054, 0),
        Op::W32(0x9058, 0),
        Op::W32(0x905C, 0),
        Op::W32(0x9060, 0),
        Op::W32(0x9064, 0),
        Op::W32(0x9068, 0),
        Op::W32(0x906C, 0), // tx_rpt, written for 8852C
    ]);
    // dle_func_en_ax(true): set WDE_EN | PLE_EN.
    want.extend([Op::R32(0x8400), Op::W32(0x8400, 0x64C4_4000)]);
    // chk_dle_rdy_ax, mac.c:2149: WDE (0x8D00) then PLE (0x9100), both ready
    // on the first sample.
    want.extend([Op::R32(0x8D00), Op::R32(0x9100)]);
    // rtw89_mac_hfc_init(true, false, true), mac.c:1194: check_mac_en_ax.
    want.push(Op::R32(0x8400));
    // hfc_func_en(false, false): clear FC_EN (0) and FC_CH12_EN (3).
    want.extend([Op::R32(0x1700), Op::W32(0x1700, 0x0000_0FF6)]);
    // hfc_h2c_cfg_ax, mac.c:1127: R_AX_CH_PAGE_CTRL_V1 = h2c_prec 40 << 16 ...
    want.push(Op::W32(0x1704, 40 << 16));
    // ... and CH12_FULL_COND [11:10] = h2c_full_cond 0.
    want.extend([Op::R32(0x1700), Op::W32(0x1700, 0x0000_03F6)]);
    // hfc_func_en(false, true): set CH12_EN (3), FC_EN stays clear.
    want.extend([Op::R32(0x1700), Op::W32(0x1700, 0x0000_03FE)]);

    assert_eq!(m.trace, want);
    // No delay: both polls succeed on their first sample.
    assert!(!m.trace.iter().any(|op| matches!(op, Op::Delay(_))));
}

#[test]
fn dle_ready_timeout() {
    let mut m = RegisterModel::new();
    // Bit 1 of 0x8D00 never rises: only the buffer manager is ready.
    m.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 1));
    m.on_read(0x9100, Box::new(|_: &mut RegisterModel| 3));
    assert_eq!(
        pre_init(&mut m),
        Err(AxError::Timeout {
            register: 0x8D00,
            mask: 0x3
        })
    );
    // Nothing of HFC ran after the failed DLE.
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x1704, _))));
}

#[test]
fn ple_ready_timeout_names_ple_register() {
    let mut m = RegisterModel::new();
    m.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 3));
    m.on_read(0x9100, Box::new(|_: &mut RegisterModel| 2));
    assert_eq!(
        pre_init(&mut m),
        Err(AxError::Timeout {
            register: 0x9100,
            mask: 0x3
        })
    );
}

#[test]
fn dmac_not_enabled_is_refused_before_dle_writes() {
    // `rtw89_mac_check_mac_en_ax` reads MAC_FUNC_EN|DMAC_FUNC_EN back clear.
    let mut m = ready();
    m.on_read(0x8400, Box::new(|_: &mut RegisterModel| 0));
    assert!(matches!(pre_init(&mut m), Err(AxError::Dma(_))));
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x8C08, _))));
}

#[test]
fn dmac_dead_read_is_refused() {
    let mut m = ready();
    m.on_read(0x8400, Box::new(|_: &mut RegisterModel| 0xDEAD_BEEF));
    assert!(matches!(pre_init(&mut m), Err(AxError::Dma(_))));
}
