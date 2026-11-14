// SPDX-License-Identifier: GPL-2.0-only
//! External USB2 reset/debounce through parent-hub EP0 requests.
use super::*;

const POWER: u16 = 1 << 8;
const OVER_CURRENT: u16 = 1 << 3;
const SUSPEND: u16 = 1 << 2;
const RESET_ATTEMPTS: usize = 5;
// hub_port_reset: TRSTRCY plus the donor's ordinary 40-ms device margin.
const RESET_SETTLE_US: u64 = 50_000;

fn powered(status: u16, change: u16) -> Result<(), BlockError> {
    if status & POWER == 0 || (status | change) & OVER_CURRENT != 0 {
        return Err(BlockError::Unsupported(
            "USB2 hub port power absent or over-current",
        ));
    }
    Ok(())
}

impl Controller {
    /// Only a stable, powered, enabled post-reset observation establishes the
    /// child speed and releases an old current subtree. The reset notification
    /// by itself cannot authorize a replacement child.
    #[cfg(test)]
    pub(in super::super) fn hub_port(
        &mut self,
        id: impl DeviceRef,
        port: u8,
    ) -> Result<Option<u8>, BlockError> {
        let slot = id.current_slot(self)?;
        let id = self.device_id(slot)?;
        self.hub_port_for(id, port)
    }

    pub(in super::super) fn hub_port_for(
        &mut self,
        id: SlotId,
        port: u8,
    ) -> Result<Option<u8>, BlockError> {
        block_on(Self::hub_port_for_with(
            &Access::blocking(self, None),
            id,
            port,
        ))
    }

    pub(in super::super) async fn hub_port_for_with(
        a: &Access<'_>,
        id: SlotId,
        port: u8,
    ) -> Result<Option<u8>, BlockError> {
        let slot = a.with(|c| {
            c.check_minimum()?;
            c.validate_topology_id(id)?;
            c.check_minimum()?;
            c.validate_topology_id(id)
        })?;
        Self::observe_current_ancestors_with(a, slot, true).await?;
        let connected = enumerate::debounce_with(a, async || {
            Self::sample_usb2_hub_port_with(a, slot, port).await
        })
        .await;
        if !a.with(|c| c.minimum_status(connected))? {
            return Ok(None);
        }
        Self::reset_debounced_usb2_hub_port_with(a, id, port).await
    }

    pub(in super::super) fn sample_usb2_hub_port(
        &mut self,
        id: SlotId,
        port: u8,
    ) -> Result<(bool, bool), BlockError> {
        let slot = self.validate_topology_id(id)?;
        block_on(Self::sample_usb2_hub_port_with(
            &Access::blocking(self, None),
            slot,
            port,
        ))
    }

    async fn sample_usb2_hub_port_with(
        a: &Access<'_>,
        slot: u8,
        port: u8,
    ) -> Result<(bool, bool), BlockError> {
        let (status, change) = Self::hub_port_status_with(a, slot, port).await?;
        // The changes are acknowledged first, so a port refused below is
        // not reported again by the hub's next status notification.
        for (bit, selector) in [
            (HUB_PORT_CONNECTION, C_PORT_CONNECTION),
            (2, 17),
            (4, 18),
            (16, 20),
            (32, 23),
        ] {
            if change & bit != 0 {
                a.control(
                    slot,
                    setup(0x23, CLEAR_FEATURE, selector, u16::from(port), 0),
                    &[],
                )
                .await?;
            }
        }
        powered(status, change)?;
        Ok((
            status & HUB_PORT_CONNECTION != 0,
            change & HUB_PORT_CONNECTION != 0,
        ))
    }

    /// The caller owns a completed stability interval for this exact port.
    pub(in super::super) async fn reset_debounced_usb2_hub_port_with(
        a: &Access<'_>,
        id: SlotId,
        port: u8,
    ) -> Result<Option<u8>, BlockError> {
        let slot = a.with(|c| {
            c.check_minimum()?;
            c.validate_topology_id(id)
        })?;
        let index = u16::from(port);
        for _ in 0..RESET_ATTEMPTS {
            a.control(slot, setup(0x23, SET_FEATURE, PORT_RESET, index, 0), &[])
                .await?;
            let deadline = a.with(|c| c.clock.deadline(PORT_RESET_US));
            loop {
                let (status, change) = Self::hub_port_status_with(a, slot, port).await?;
                powered(status, change)?;
                if change & HUB_PORT_RESET != 0 && status & HUB_PORT_RESET == 0 {
                    a.control(
                        slot,
                        setup(0x23, CLEAR_FEATURE, C_PORT_RESET, index, 0),
                        &[],
                    )
                    .await?;
                    a.minimum_wait(RESET_SETTLE_US).await?;
                    let (status, change) = Self::hub_port_status_with(a, slot, port).await?;
                    powered(status, change)?;
                    Self::observe_current_ancestors_with(a, slot, true).await?;
                    if status & HUB_PORT_CONNECTION == 0 {
                        return Ok(None);
                    }
                    if change != 0 || status & (SUSPEND | (1 << 5) | (1 << 11)) != 0 {
                        return Err(BlockError::Unsupported(
                            "USB2 hub port changed during reset recovery",
                        ));
                    }
                    if status & (HUB_PORT_ENABLE | HUB_PORT_RESET) == HUB_PORT_ENABLE {
                        a.with(|c| {
                            let root = c.slot(slot)?.attachment.root_port;
                            c.current_hub_port_reset(id, port, root)
                        })?;
                        return Ok(Some(if status & HUB_PORT_LOW_SPEED != 0 {
                            LOW_SPEED
                        } else if status & HUB_PORT_HIGH_SPEED != 0 {
                            HIGH_SPEED
                        } else {
                            FULL_SPEED
                        }));
                    }
                    // A reset which completed disabled may be retried. Keep the
                    // physical current ownership until a later valid ACK.
                    break;
                }
                if deadline.passed() {
                    return Err(BlockError::Timeout);
                }
                a.delay(1000).await;
            }
        }
        Err(BlockError::Unsupported(
            "USB2 hub port failed bounded resets",
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::{
        super::{model::*, tests::controller},
        *,
    };
    use crate::bus::dma::TestRegion;
    use alloc::collections::BTreeMap;

    #[test]
    fn usb2_reset_reobserves_recovery_before_releasing_child_current() {
        for context in [32, 64] {
            for fault in [
                Fault::ResetRecoveryLoss,
                Fault::FirstResetDisabled,
                Fault::NoResetChange,
                Fault::OverCurrentChange,
            ] {
                let nodes =
                    BTreeMap::from([(0, Node::hub(HIGH_SPEED, 1)), (1, Node::leaf(HIGH_SPEED))]);
                let mut dma = TestRegion::new(256 * 1024 * 8);
                let (mut c, model) = controller(&mut dma, context, nodes);
                let mut failures = Vec::new();
                c.enumerate(&mut dma, &mut failures);
                assert!(failures.is_empty());
                let grant = c.power.snapshot(2);
                {
                    let mut m = model.borrow_mut();
                    let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
                    port.fault = fault;
                    port.reset_count = 0;
                }
                let result = c.hub_port(1, 1);
                if fault == Fault::FirstResetDisabled {
                    assert_eq!(result, Ok(Some(HIGH_SPEED)));
                    assert_eq!(
                        model.borrow().nodes[&0].ports.as_ref().unwrap()[0].reset_count,
                        2
                    );
                    assert!(!c.power.retained(2));
                } else {
                    assert!(result != Ok(Some(HIGH_SPEED)), "{fault:?}");
                    assert_eq!(c.power.snapshot(2), grant);
                }
            }
        }
    }
}
