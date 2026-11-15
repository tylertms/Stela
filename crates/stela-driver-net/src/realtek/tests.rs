// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! Stateful protocol model, with delayed command completion and real DMA rings
//! tested separately. The revision/analogue expectations are from the pinned
//! Linux sources; this is not a physical-device or RTL8139 emulator claim.
use super::*;
use alloc::{collections::BTreeMap, rc::Rc};
use core::cell::{Cell, RefCell};

#[derive(Clone, Default)]
struct Time(Rc<Cell<u64>>);
impl Timer for Time {
    fn now(&self) -> u64 {
        self.0.get()
    }
    /// As the mount's `budget::Bounded` minimum under a start budget of
    /// [`START_BUDGET_US`],
    /// counted from the model's first instant.
    fn delay(&self, us: u64) -> Result<(), BlockError> {
        self.0.set(self.0.get() + us);
        if self.0.get() >= START_BUDGET_US {
            return Err(BlockError::Timeout);
        }
        Ok(())
    }
}
/// The probe's start budget for one PCI function (`probe::DEVICE_START_US`).
const START_BUDGET_US: u64 = 30_000_000;
#[derive(Default)]
struct State {
    bytes: Vec<u8>,
    phy: BTreeMap<(u16, u16), u16>,
    ephy: [u16; 32],
    eri: BTreeMap<u16, u32>,
    oob: BTreeMap<u16, u32>,
    ep_ready_at: Option<u64>,
    cmac_ready_at: Option<u64>,
    mac: BTreeMap<u16, u16>,
    phy_ocp: BTreeMap<u16, u16>,
    dp_ocp: BTreeMap<u16, u32>,
    dp_ready_at: Option<(u64, u16)>,
    dp_commands: usize,
    g_parameter: u16,
    g_parameters: BTreeMap<u16, u16>,
    n_parameter: u16,
    n_parameters: BTreeMap<u16, u16>,
    bmsr_latch_low_once: bool,
    page: u16,
    parameter: u16,
    parameters: BTreeMap<u16, u16>,
    efuse: u8,
    extended_page: u16,
    extended: BTreeMap<(u16, u16), u16>,
    pending: Option<(usize, u32, u64)>,
    reset_phy_at: Option<u64>,
    reset_mac_at: Option<u64>,
    shared_ready_at: Option<u64>,
    writes: Vec<(usize, u32, u8)>,
    mdio_commands: usize,
    phy_ocp_commands: usize,
    eri_commands: usize,
    csi: BTreeMap<(u8, u16), u32>,
    csi_commands: usize,
}
struct Model {
    state: RefCell<State>,
    time: Time,
    fault: Option<usize>,
    failed_read: Option<usize>,
    fail_phy_at: usize,
    fail_eri_at: usize,
    fail_csi_at: usize,
    stuck_reset: bool,
    stuck_phy_reset: bool,
    command_readback: Option<u8>,
    shared_ready_delay: Option<u64>,
    dp_mode: bool,
    dp_ready_delay: Option<u64>,
    fail_dp_at: usize,
    h_adc: Option<(u16, u16)>,
    ep_mode: Option<u8>,
    ep_ready_delay: Option<u64>,
    cmac_ready_delay: Option<u64>,
    fail_mac_address: Option<u16>,
    internal_nbase: bool,
    fail_phy_ocp_at: usize,
}

impl Model {
    fn ready(time: &Time) -> Self {
        let mut state = State {
            bytes: vec![0; 256],
            ..State::default()
        };
        for (reg, value) in [
            (0, 0x1140),
            (1, 0x7909),
            (2, 0x001c),
            (3, 0xc800),
            (9, 0x1b00),
            (15, 0x3000),
        ] {
            state.phy.insert((0, reg), value);
        }
        Self {
            state: RefCell::new(state),
            time: time.clone(),
            fault: None,
            failed_read: None,
            fail_phy_at: usize::MAX,
            fail_eri_at: usize::MAX,
            fail_csi_at: usize::MAX,
            stuck_reset: false,
            stuck_phy_reset: false,
            command_readback: None,
            shared_ready_delay: None,
            dp_mode: false,
            dp_ready_delay: Some(20_000),
            fail_dp_at: usize::MAX,
            h_adc: None,
            ep_mode: None,
            ep_ready_delay: Some(20_000),
            cmac_ready_delay: Some(75_000),
            fail_mac_address: None,
            internal_nbase: false,
            fail_phy_ocp_at: usize::MAX,
        }
    }
    fn dp(time: &Time, version: u8, dash: bool) -> Self {
        let mut model = Self::ready(time);
        model.dp_mode = true;
        {
            let mut s = model.state.borrow_mut();
            Self::store32(&mut s, DP_MDIO_GATE, DP_MDIO_GATED);
            s.dp_ocp.insert(
                if version == 31 { 0xb8 } else { 0x10 },
                if dash { 1 << 15 } else { 0 },
            );
        }
        model
    }
    fn modern(time: &Time) -> Self {
        let mut model = Self::ready(time);
        model.shared_ready_delay = Some(300);
        {
            let mut s = model.state.borrow_mut();
            s.phy_ocp.extend([
                (0xa400, 0x1940),
                (0xa402, 0x7909),
                (0xa404, 0x001c),
                (0xa406, 0xc800),
                (0xa412, 0x1b00),
                (0xa41e, 0x3000),
                (0xa616, 0),
            ]);
            s.bytes[MCU] = 0x32;
            Self::store32(&mut s, TX_CONFIG, 1 << 11);
            s.eri.insert(0xe0, 0x0504_0302);
            s.eri.insert(0xe4, 0x0706);
        }
        model
    }
    fn ep(time: &Time, version: u8, dash: bool) -> Self {
        let mut model = Self::modern(time);
        model.ep_mode = Some(version);
        {
            let mut s = model.state.borrow_mut();
            s.bytes[IBCR0] = 0x85;
            s.bytes[IBCR2] = 0x85;
            s.oob.insert(0x128, u32::from(dash));
        }
        model
    }
    fn nbase(time: &Time) -> Self {
        let mut model = Self::modern(time);
        model.internal_nbase = true;
        {
            let mut s = model.state.borrow_mut();
            s.bytes.resize(0x7000, 0);
            s.bytes[INTERRUPT_MITIGATION..INTERRUPT_MITIGATION + 2]
                .copy_from_slice(&0x0103u16.to_le_bytes());
            s.phy_ocp.insert(0xa616, 0x2070);
            Self::store32(&mut s, NBASE_MAC0_BACKUP, 0x0504_0302);
            Self::store32(&mut s, NBASE_MAC0_BACKUP + 4, 0x0706);
        }
        model
    }
    fn raw32(s: &State, o: usize) -> u32 {
        u32::from_le_bytes(s.bytes[o..o + 4].try_into().unwrap())
    }
    fn store32(s: &mut State, o: usize, v: u32) {
        s.bytes[o..o + 4].copy_from_slice(&v.to_le_bytes());
    }
}

fn revision(version: u8) -> Revision {
    let xid = match version {
        2 => 0x008,
        3 => 0x040,
        4 => 0x100,
        5 => 0x180,
        6 => 0x980,
        17 => 0x380,
        18 => 0x3c8,
        19 => 0x3c0,
        20 => 0x3c2,
        21 => 0x3c3,
        22 => 0x3c4,
        23 => 0x3c9,
        24 => 0x3ca,
        25 => 0x281,
        26 => 0x280,
        28 => 0x28a,
        31 => 0x28b,
        46 => 0x541,
        51 => 0x502,
        52 => 0x54a,
        61 => 0x609,
        63 => 0x641,
        64 => 0x688,
        65 => 0x708,
        66 => 0x681,
        70 => 0x649,
        32 => 0x2c1,
        33 => 0x2c0,
        35 => 0x480,
        36 => 0x481,
        38 => 0x488,
        40 => 0x4c0,
        42 => 0x509,
        44 => 0x5c8,
        34 => 0x2c8,
        _ => panic!("fixture revision"),
    };
    revision::detect(xid << 20).unwrap()
}

#[test]
fn receive_config_follows_linux_per_version_without_multiple_fetch() {
    let fifo = (7 << 13) | (7 << 8);
    let interrupt = (1 << 15) | (7 << 8);
    let early = interrupt | (1 << 11);
    let fetch = (8 << 27) | (7 << 8);
    let expected = [
        (2, fifo),
        (6, fifo),
        (17, fifo),
        (18, interrupt),
        (24, interrupt),
        (25, interrupt),
        (28, interrupt),
        (33, interrupt),
        (34, interrupt),
        (36, interrupt),
        (38, interrupt),
        (40, early),
        (42, early),
        (44, early),
        // RTL8168H (XID 541): Linux's 0x2cf0e less RX_MULTI_EN is 0x8f0e.
        (46, early),
        (51, early),
        (52, early),
        (61, fetch),
        (63, fetch | (1 << 11)),
        (66, fetch | (1 << 11)),
        (70, fetch | (1 << 11)),
    ];
    for (version, value) in expected {
        assert_eq!(receive_config(version), value, "version {version}");
        // On the 8169 family (through 17) bit 14 is part of the FIFO threshold.
        if version >= 18 {
            assert_eq!(receive_config(version) & (1 << 14), 0, "version {version}");
        }
    }
    assert_eq!(receive_config(46) | RX_ACCEPT, 0x8f0e);
}

mod admission;
mod aliases;
mod command_completion;
mod csi;
mod d_revisions;
mod dma_publication;
mod dp_management;
mod e_revisions;
mod ep_management;
mod extended_identity;
mod f_revisions;
mod fp_firmware;
mod g_revisions;
mod h_revisions;
mod legacy;
mod link_state;
mod modern_protocol;
mod nbase_8126;
mod nbase_ab;
mod nbase_admission;
mod nbase_bp;
mod nbase_d_cp;
mod packet_io;
mod registers;
mod stop;
mod stuck_registers;

mod receive_recovery;
