// SPDX-License-Identifier: GPL-2.0-only
//! Linux hub_event/port_event cooldown, power-on and status recovery.
//! An upstream reset publishes a fresh hub owner; old aliases stay fenced.
use super::*;

const HUB_COOLDOWN_US: u64 = 500_000;
const PORT_COOLDOWN_US: u64 = 100_000;
// bPwrOn2PwrGood is u8 in 2-ms units. Waiting its maximum also covers
// Linux's minimum100ms without trusting a descriptor read during recovery.
const MAX_POWER_GOOD_US: u64 = 255 * 2000;

impl Host {
    pub(super) fn start_recovery(
        &mut self,
        index: usize,
        port: Option<u8>,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let id = self.hubs[index].device.id;
        let slot = id.hardware_slot();
        // Repowering any port can affect a ganged supply. Conservatively retire
        // the entire hub, including otherwise healthy siblings, before doing it.
        self.retire_except(id, Some(id), changes)?;
        self.hubs[index].pending = None;
        self.hubs[index].recovery = Some(Recovery {
            slot,
            id: self.hubs[index].device.id,
            port,
        });
        Ok(())
    }

    pub(super) fn recover_hub(
        &mut self,
        index: usize,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let hub = &mut self.hubs[index];
        let owner = hub.recovery.as_mut().unwrap();
        let mut c = self.controller.borrow_mut();
        c.validate_recovery_id(owner.id())?;
        // Reobserve the root and every unaffected physical ancestor
        // before each batch of private EP0 effects, including after cooldown.
        c.observe_current_ancestors_for(owner.id(), false)?;
        let (status, change) = hub_status(&mut c, owner)?;
        if change & 1 != 0 {
            c.recovery_control(owner, setup(0x20, CLEAR_FEATURE, 0, 0, 0))?;
        }
        if change & 2 != 0 {
            c.recovery_control(owner, setup(0x20, CLEAR_FEATURE, 1, 0, 0))?;
        }
        for port in 1..=hub.ports {
            let (_, change) = port_status(&mut c, owner, port)?;
            if change & 8 != 0 {
                c.recovery_control(owner, setup(0x23, CLEAR_FEATURE, 19, u16::from(port), 0))?;
            }
        }
        // A persistent fault is retried only after another complete cooldown;
        // polling frequency must not shorten the source's recovery interval.
        c.minimum_wait(
            if owner.port.is_some() && status & 2 == 0 && change & 2 == 0 {
                PORT_COOLDOWN_US
            } else {
                HUB_COOLDOWN_US
            },
        )?;
        c.observe_current_ancestors_for(owner.id(), false)?;
        // Never substitute bus power for a lost self-powered contract. Preserve
        // the full old current ledger while waiting for restoration.
        let device = c.recovery_control(owner, setup(0x80, GET_STATUS, 0, 0, 2))?;
        if device.len() != 2 || (device[0] & 1 != 0) != c.power.hub_is_self_powered(owner.slot)? {
            return Err(BlockError::Unsupported(
                "USB recovery hub power source changed",
            ));
        }
        let (status, change) = hub_status(&mut c, owner)?;
        if status & 1 != 0 || change != 0 {
            return Err(BlockError::Unsupported(
                "USB recovery awaits stable hub power status",
            ));
        }
        let powered = c.power.powered_ports(owner.slot)?;
        for port in 1..=hub.ports {
            if powered & (1 << port) != 0 {
                c.observe_current_ancestors_for(owner.id(), false)?;
                c.recovery_control(
                    owner,
                    setup(0x23, SET_FEATURE, PORT_POWER, u16::from(port), 0),
                )?;
            }
        }
        c.minimum_wait(MAX_POWER_GOOD_US)?;
        c.observe_current_ancestors_for(owner.id(), false)?;
        if hub_status(&mut c, owner)? != (0, 0) {
            return Err(BlockError::Unsupported(
                "USB recovery hub over-current persists",
            ));
        }
        for port in 1..=hub.ports {
            if powered & (1 << port) != 0 {
                let (status, change) = port_status(&mut c, owner, port)?;
                let power = if hub.attachment.speed >= SUPER_SPEED {
                    1 << 9
                } else {
                    1 << 8
                };
                if status & power == 0 || status & 8 != 0 || change & 8 != 0 {
                    return Err(BlockError::Unsupported(
                        "USB recovery port power absent or over-current persists",
                    ));
                }
            }
        }
        // Private recovery transfers have finished; stop EP0 again before
        // Disable Slot. Any uncertainty keeps the capability and original DMA.
        c.validate_recovery_id(owner.id())?;
        let original_parent = c.original_parent(owner.id())?;
        c.disable_slot(owner.slot, reset::Ownership::Retired)
            .map_err(|(_, error)| error)?;
        let attachment = hub.attachment;
        let location = hub.device.location.clone();
        hub.recovery = None;
        drop(c);
        self.reconnect_hub(attachment, original_parent, location, dma, changes)
    }

    fn reconnect_hub(
        &mut self,
        attachment: Attachment,
        original_parent: Option<SlotId>,
        location: String,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let parent = attachment
            .parent
            .map(|(_, _)| {
                self.hubs
                    .iter()
                    .position(|hub| Some(hub.device.id) == original_parent && !hub.retired)
                    .ok_or(BlockError::Unsupported(
                        "USB recovery physical parent is retired",
                    ))
            })
            .transpose()?;
        let ssp = parent.and_then(|index| self.hubs[index].ssp.clone());
        let superspeed =
            parent.is_some_and(|index| self.hubs[index].attachment.speed >= SUPER_SPEED);
        let mut c = self.controller.borrow_mut();
        let reset = enumerate::Reset::Recover {
            attachment,
            parent: original_parent,
            ssp,
            superspeed,
        };
        c.connect(
            location,
            original_parent,
            reset,
            dma,
            &mut changes.added.devices,
            &mut changes.added.failures,
        );
        Ok(())
    }
}

fn hub_status(c: &mut Controller, owner: &Recovery) -> Result<(u16, u16), BlockError> {
    let bytes = c.recovery_control(owner, setup(0xa0, GET_STATUS, 0, 0, 4))?;
    let (status, change) = status_words(&bytes)?;
    if status & !3 != 0 || change & !3 != 0 {
        return Err(BlockError::Unsupported("reserved USB recovery hub status"));
    }
    Ok((status, change))
}
fn port_status(c: &mut Controller, owner: &Recovery, port: u8) -> Result<(u16, u16), BlockError> {
    let bytes = c.recovery_control(owner, setup(0xa3, GET_STATUS, 0, u16::from(port), 4))?;
    status_words(&bytes)
}
fn status_words(bytes: &[u8]) -> Result<(u16, u16), BlockError> {
    if bytes.len() != 4 {
        return Err(BlockError::Unsupported("short USB recovery status"));
    }
    Ok((
        u16::from_le_bytes([bytes[0], bytes[1]]),
        u16::from_le_bytes([bytes[2], bytes[3]]),
    ))
}
