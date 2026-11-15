// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! Bounded MAC46 receive-stall recovery; TCP retains retransmission ownership.
use super::{hardware::Hardware, registers::*, ring::Ring, *};

pub(super) const STALL_US: u64 = 2_000_000;
const MAX_ATTEMPTS: u8 = 3;

#[derive(Default)]
pub(super) struct ReceiveWatch {
    received: u64,
    transmitted: u64,
    since: u64,
    attempts: u8,
    pub restarts: u64,
}

#[derive(Debug, PartialEq, Eq)]
pub(super) enum Decision {
    Wait,
    Restart,
    Exhausted,
}

impl ReceiveWatch {
    pub fn check(&mut self, now: u64, received: u64, transmitted: u64, pending: usize) -> Decision {
        if received != self.received {
            self.received = received;
            self.transmitted = transmitted;
            self.since = now;
            self.attempts = 0;
            return Decision::Wait;
        }
        // Idle LANs, startup with no completed ring, and outstanding DMA are
        // not grounds for a reset. A new TX since the last progress/restart is
        // required; unsuccessful recovery cannot loop on every 10ms poll.
        if received < DESCRIPTORS as u64
            || transmitted <= self.transmitted
            || pending != 0
            || now.saturating_sub(self.since) < STALL_US
        {
            return Decision::Wait;
        }
        if self.attempts >= MAX_ATTEMPTS {
            return Decision::Exhausted;
        }
        self.attempts += 1;
        self.restarts = self.restarts.saturating_add(1);
        self.transmitted = transmitted;
        self.since = now;
        Decision::Restart
    }
}

pub(super) fn restart<R: Registers, T: Timer>(
    hw: &mut Hardware<'_, R, T>,
    rx: &mut Ring,
    tx: &mut Ring,
    mac: Mac,
    signalled: bool,
) -> Result<(), Error> {
    // No PHY reset, new allocation or address change. On reset failure leave
    // every descriptor/buffer untouched: the device may still own DMA.
    hw.reset()?;
    hw.configure_mac(None)?;
    rx.initialize(true);
    tx.initialize(false);
    publish(hw, rx, tx, mac)?;
    if signalled {
        hw.interrupts_on(SIGNALLED_EVENTS);
    }
    Ok(())
}
