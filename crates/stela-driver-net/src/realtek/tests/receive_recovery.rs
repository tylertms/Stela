// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
use super::*;
use crate::bus::dma::TestRegion;
use crate::realtek::recovery::{Decision, ReceiveWatch, STALL_US, restart};

#[test]
fn watchdog_requires_full_ring_quiet_dma_new_tx_and_limits_failed_restarts() {
    let mut w = ReceiveWatch::default();
    assert_eq!(w.check(0, 255, 10, 0), Decision::Wait);
    assert_eq!(w.check(STALL_US * 2, 255, 11, 0), Decision::Wait);
    assert_eq!(w.check(STALL_US * 2, 256, 11, 0), Decision::Wait);
    assert_eq!(w.check(STALL_US * 3 - 1, 256, 12, 0), Decision::Wait);
    assert_eq!(w.check(STALL_US * 4, 256, 12, 1), Decision::Wait);
    for i in 0..3 {
        let now = STALL_US * (4 + i);
        assert_eq!(w.check(now, 256, 12 + i, 0), Decision::Restart);
        assert_eq!(w.check(now + STALL_US, 256, 12 + i, 0), Decision::Wait);
    }
    assert_eq!(w.check(STALL_US * 7, 256, 15, 0), Decision::Exhausted);
    assert_eq!(w.restarts, 3);
    // Actual receive progress replenishes the consecutive failure budget.
    assert_eq!(w.check(STALL_US * 7, 257, 15, 0), Decision::Wait);
    assert_eq!(w.check(STALL_US * 9, 257, 16, 0), Decision::Restart);
}

#[test]
fn reset_rewinds_drained_ring_preserves_mac_and_republishes_dma_before_irq() {
    for signalled in [false, true] {
        let time = Time::default();
        let model = Model::modern(&time);
        let mut hw = Hardware::new(&model, &time, revision(46), 0);
        let mut dma = TestRegion::new(2_000_000);
        let mut rx = Ring::new(&mut dma, true).unwrap();
        let mut tx = Ring::new(&mut dma, true).unwrap();
        rx.initialize(true);
        tx.initialize(false);
        for _ in 0..DESCRIPTORS + 3 {
            rx.deliver(&[0x5a; 60]);
            assert!(rx.receive().is_some());
        }
        tx.transmit(&[0x11; 60]).unwrap();
        // Model a completed transmitter; the production caller checks all
        // OWN bits before attempting recovery.
        tx.descriptors.write(0, FIRST | LAST | 60);
        let bases = (rx.descriptors.bus_address(), tx.descriptors.bus_address());
        let mac = Mac([2, 3, 4, 5, 6, 7]);
        restart(&mut hw, &mut rx, &mut tx, mac, signalled).unwrap();
        assert_eq!(rx.owned(), DESCRIPTORS);
        assert_eq!(tx.owned(), 0);
        assert_eq!(rx.packets, (DESCRIPTORS + 3) as u64);
        assert_eq!(model.read32(RX_LOW), bases.0 as u32);
        assert_eq!(model.read32(TX_LOW), bases.1 as u32);
        assert_eq!(model.read32(MAC0), 0x05040302);
        assert_eq!(model.read8(COMMAND), ENABLE);
        assert_eq!(
            model.read16(INTERRUPT_MASK),
            if signalled { SIGNALLED_EVENTS } else { 0 }
        );
        // Independent completion at hardware index zero must be the first
        // frame read after reset, even if the old CPU cursor was nonzero.
        rx.descriptors.write(0, FIRST | LAST | 64);
        assert!(rx.receive().is_some());
        tx.transmit(&[0x22; 60]).unwrap();
        assert_ne!(tx.descriptors.read::<u32>(0) & OWN, 0);
        if signalled {
            let writes = &model.state.borrow().writes;
            let base = writes
                .iter()
                .rposition(|(reg, _, _)| *reg == RX_LOW)
                .unwrap();
            let irq = writes
                .iter()
                .rposition(|(reg, value, _)| *reg == INTERRUPT_MASK && *value != 0)
                .unwrap();
            assert!(base < irq);
        }
    }
}

#[test]
fn failed_reset_does_not_recycle_device_owned_memory_or_publish_bases() {
    let time = Time::default();
    let mut model = Model::modern(&time);
    model.stuck_reset = true;
    let mut hw = Hardware::new(&model, &time, revision(46), 0);
    let mut dma = TestRegion::new(2_000_000);
    let mut rx = Ring::new(&mut dma, true).unwrap();
    let mut tx = Ring::new(&mut dma, true).unwrap();
    rx.initialize(true);
    tx.initialize(false);
    tx.transmit(&[0xab; 60]).unwrap();
    let before = (
        rx.descriptors.bytes().to_vec(),
        tx.descriptors.bytes().to_vec(),
    );
    assert!(restart(&mut hw, &mut rx, &mut tx, Mac([2, 0, 0, 0, 0, 1]), true).is_err());
    assert_eq!(rx.descriptors.bytes(), before.0);
    assert_eq!(tx.descriptors.bytes(), before.1);
    assert!(
        !model
            .state
            .borrow()
            .writes
            .iter()
            .any(|(reg, _, _)| matches!(*reg, RX_LOW | RX_HIGH | TX_LOW | TX_HIGH))
    );
}
