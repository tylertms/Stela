//! Exclusive leaf reset and fresh identification. Generic resets keep old
//! hardware IDs; the root-leaf service reuses proved-retired storage with a fresh
//! generation while the old raw hardware-slot fence remains permanent.
// SPDX-License-Identifier: GPL-2.0-only
// Linux a804c574 usb/core/hub.c and usb/host/xhci.c/xhci-hub.c;
// Intel xHCI1.2b4.5.3.3,4.6.4,4.19.5,5.4.8. See NOTICE.
use super::*;

pub(super) const DISABLE_SLOT: u32 = 10;
pub(super) const PORT_STATUS_CHANGE: u32 = 34;
const POST_RESET_US: u64 = 50_000;
/// Linux `PORT_RESET_TRIES`: resets one port reset request may take.
const PORT_RESET_TRIES: usize = 5;
pub(super) const EXHAUSTED: &str = "xHCI slot budget exhausted";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Ownership {
    Vacant,
    Enabled,
    /// Disable Slot completed; a later Enable Slot may hand this ID back.
    Retired,
    /// That Enable Slot was acknowledged while the old device's port still
    /// held its current grant, and remains unaddressed.
    Tombstone,
}
impl Controller {
    /// Pure guard before any stale bare-slot API can touch MMIO or DMA.
    pub(super) fn live_slot(&self, slot: u8) -> Result<(), BlockError> {
        let index = usize::from(slot);
        if self.slot_ownership.get(index) != Some(&Ownership::Enabled)
            || self
                .slots
                .get(index)
                .and_then(Option::as_ref)
                .is_none_or(|slot| slot.retiring)
        {
            return Err(BlockError::Unsupported(
                "xHCI slot is absent or permanently retired",
            ));
        }
        Ok(())
    }
    /// Permanently block class reconstruction failure; this is not a claim
    /// that hardware/DMA has stopped. All original allocations remain held.
    #[cfg(test)]
    pub(crate) fn quarantine_slot(&mut self, id: impl DeviceRef) {
        // This notification has no result channel. A retired generation
        // cannot quarantine a later device in the same slot.
        if let Ok(slot) = id.current_slot(self) {
            self.quarantine_slot_owned(slot);
        }
    }

    pub(super) fn quarantine_slot_owned(&mut self, slot: u8) {
        if let Some(Some(slot)) = self.slots.get_mut(usize::from(slot)) {
            slot.retiring = true;
        }
    }

    #[cfg(test)]
    pub(super) fn fresh_slot(&mut self) -> Result<u8, BlockError> {
        block_on(Self::fresh_slot_with(&Access::blocking(self, None)))
    }

    pub(super) async fn fresh_slot_with(a: &Access<'_>) -> Result<u8, BlockError> {
        let max_slots = a.with(|c| c.max_slots);
        if a.with(|c| {
            c.slot_ownership
                .iter()
                .skip(1)
                .all(|state| !matches!(state, Ownership::Vacant | Ownership::Retired))
        }) {
            return Err(BlockError::Unsupported(EXHAUSTED));
        }
        for _ in 0..max_slots {
            let event = a.command(0, 0, ENABLE_SLOT << 10).await?;
            if let Some(id) = a.with(|c| c.enabled_slot(event))? {
                return Ok(id);
            }
        }
        Err(BlockError::Unsupported(EXHAUSTED))
    }

    /// Enable Slot's completion: its slot, or `None` when it named a tombstone.
    fn enabled_slot(&mut self, event: Event) -> Result<Option<u8>, BlockError> {
        {
            let id = usize::from(event.slot());
            if id == 0 || id > self.max_slots || id >= self.slot_ownership.len() {
                self.command_poisoned = true;
                return Err(BlockError::Unsupported("xHCI slot ID out of range"));
            }
            match self.slot_ownership[id] {
                Ownership::Vacant => {
                    // Keep this grant even if any following allocation fails.
                    // Enabled slots do not fetch DCBAA or accept doorbells.
                    self.slot_ownership[id] = Ownership::Enabled;
                    Ok(Some(id as u8))
                }
                // xhci_free_virt_device ends the old device once Disable Slot
                // completes, and Enable Slot may hand its ID to the next one.
                // Every identity of the old generation fails `known_id` once
                // its record goes; its port's current grant must have retired.
                Ownership::Retired if !self.power.retained(id as u8) => {
                    self.slots[id] = None;
                    self.slot_ownership[id] = Ownership::Enabled;
                    Ok(Some(id as u8))
                }
                Ownership::Retired => {
                    self.slot_ownership[id] = Ownership::Tombstone;
                    Ok(None)
                }
                Ownership::Enabled | Ownership::Tombstone => {
                    self.command_poisoned = true;
                    Err(BlockError::Unsupported(
                        "xHCI returned an already-owned slot ID",
                    ))
                }
            }
        }
    }

    /// # Safety
    /// The caller exclusively owns this complete device and has permanently
    /// invalidated all class aliases. Endpoints must be the complete current
    /// configured DCI set. This service supports one physical leaf, including
    /// a downstream hub port; it never authorizes resetting a hub subtree or
    /// another interface's device.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(crate) unsafe fn reset_and_identify(
        &mut self,
        device: &Device,
        endpoints: &[Endpoint],
        dma: &mut dyn DmaAllocator,
    ) -> Result<Device, (String, BlockError)> {
        during(
            "reset exclusive device preflight",
            self.reset_preflight(device, endpoints),
        )?;
        let slot = device.slot;
        let attachment = self.slot(slot).unwrap().attachment;
        let original_parent = during(
            "retain original reset parent",
            self.original_parent(device.id),
        )?;
        let consumer = self.slot(slot).unwrap().consumer;
        // Reobserve root and every ancestor before invalidating the
        // leaf. A failed shared supply cannot authorize private class recovery.
        during(
            "observe reset device physical ancestry",
            // Preflight requires a leaf. Its device status cannot supply a
            // hub power contract; observe the root and every parent.
            if attachment.parent.is_none() {
                self.observe_reset_root(device.id, attachment.root_port)
            } else {
                self.observe_current_ancestors_for(device.id, false)
            },
        )?;
        let parent = attachment
            .parent
            .map(|(_, port)| {
                let parent = original_parent
                    .ok_or(BlockError::Unsupported("USB reset original parent absent"))?;
                let slot = self.validate_topology_id(parent)?;
                let speed = self.slot(slot)?.attachment.speed;
                let ssp = if speed >= SUPER_SPEED {
                    self.hub_ssp_for(parent)?
                } else {
                    None
                };
                Ok::<_, BlockError>((parent, port, speed, ssp))
            })
            .transpose()
            .map_err(|error| ("retain reset parent protocol".into(), error))?;
        self.slot(slot).unwrap().retiring = true;
        // The whole-slot boundary supplies every Stop/Reset/Set Dequeue
        // acknowledgment and ordered event drain before Disable Slot.
        self.disable_slot(slot, Ownership::Retired)
            .map_err(|(operation, error)| (operation.into(), error))?;
        if let Some((parent, port, speed, ssp)) = parent {
            during(
                "observe downstream reset parent",
                self.observe_current_ancestors_for(parent, true),
            )?;
            let actual = during(
                "acknowledge physical downstream-port reset",
                if speed >= SUPER_SPEED {
                    self.superspeed_port_for(parent, port, ssp.as_ref())
                } else {
                    self.hub_port_for(parent, port)
                },
            )?;
            during(
                "observe downstream reset completion ancestry",
                self.observe_current_ancestors_for(parent, true),
            )?;
            if actual != Some(attachment.speed) {
                return Err((
                    "acknowledge downstream reset identity".into(),
                    BlockError::Unsupported(
                        "USB downstream reset lost the original speed or connection",
                    ),
                ));
            }
        } else {
            during(
                "acknowledge physical root-port reset",
                self.reset_root_device(attachment),
            )?;
        }
        let result =
            self.identify_parent(attachment, original_parent, device.location.clone(), dma);
        if let Ok(fresh) = &result {
            self.slot(fresh.slot).unwrap().consumer = consumer;
            // usb_reset_and_verify_device turns hardware LPM back on.
            self.initial_usb2_lpm(fresh, attachment);
        }
        if result.is_err() {
            // A failed fresh enumeration may already have acknowledged Address
            // Device and EP0. Keep that new grant inaccessible as well.
            for slot in self.slots.iter_mut().flatten() {
                if slot.attachment.root_port == attachment.root_port
                    && slot.attachment.route == attachment.route
                {
                    slot.retiring = true;
                }
            }
        }
        result
    }
    /// Whether [`Self::reset_and_reuse_root`] of `id` needs fresh rings and
    /// contexts from its allocator: a device behind a hub gets new ones, as
    /// only a root port's device reuses its own retired storage. It disables
    /// the old slot and resets the port before it allocates, so a caller
    /// with no allocator must not begin it.
    pub(crate) fn reset_needs_allocation(&self, id: SlotId) -> bool {
        self.known_id(id)
            && self.slots[usize::from(id.hardware_slot())]
                .as_ref()
                .is_some_and(|slot| slot.attachment.parent.is_some())
    }

    /// Reset one exclusively owned leaf, reusing complete native storage when
    /// it is directly attached to a root port. Downstream topology keeps the
    /// original nonrecycling service until a parent/subtree reuse proof exists.
    ///
    /// # Safety
    /// Same complete device/interface and alias-retirement contract as
    /// reset_and_identify. Caller-owned payload buffers remain owned by caller;
    /// success retires every old endpoint before it may transfer those buffers.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(crate) unsafe fn reset_and_reuse_root(
        &mut self,
        device: &Device,
        endpoints: &[Endpoint],
        dma: &mut dyn DmaAllocator,
    ) -> Result<Device, (String, BlockError)> {
        during(
            "preflight exclusive reusable device",
            self.reset_preflight(device, endpoints),
        )?;
        let attachment = self.slots[usize::from(device.slot)]
            .as_ref()
            .unwrap()
            .attachment;
        if attachment.parent.is_some() {
            // SAFETY: the identical exclusive contract was just checked.
            return unsafe { self.reset_and_identify(device, endpoints, dma) };
        }
        during(
            "observe reusable root ancestry",
            self.observe_reset_root(device.id, attachment.root_port),
        )?;
        let consumer = self.slots[usize::from(device.slot)]
            .as_ref()
            .unwrap()
            .consumer;
        let resources = self
            .retire_reset_root(device.id)
            .map_err(|(operation, error)| (operation.into(), error))?;
        self.retry_resources.offer(resources);
        let result = self.identify_parent(attachment, None, device.location.clone(), dma);
        match result {
            Ok(fresh) => {
                if fresh.descriptor != device.descriptor
                    || fresh.configurations != device.configurations
                    || fresh.serial != device.serial
                    || fresh.speed != device.speed
                {
                    self.quarantine_slot_owned(fresh.slot);
                    return Err((
                        "verify reusable device descriptors".into(),
                        BlockError::Unsupported("USB device changed across reusable reset"),
                    ));
                }
                self.slots[usize::from(fresh.slot)]
                    .as_mut()
                    .unwrap()
                    .consumer = consumer;
                Ok(fresh)
            }
            Err(error) => {
                for state in self.slots.iter_mut().flatten() {
                    if state.attachment == attachment {
                        state.retiring = true;
                    }
                }
                Err(error)
            }
        }
    }

    /// `usb_queue_reset_device`: the device's native class owner asks the
    /// topology owner to reset it and bind its drivers again, as Linux does for
    /// a driver without pre_reset/post_reset. Every access through `id` fails
    /// once the reset runs.
    pub fn queue_reset(&mut self, id: SlotId) -> Result<(), BlockError> {
        let slot = self.validate_id(id)?;
        if self.slot(slot)?.consumer != identity::Consumer::Native {
            return Err(BlockError::Unsupported(
                "USB reset request without a native class owner",
            ));
        }
        if !self.queued_resets.contains(&id) {
            self.queued_resets.push(id);
        }
        Ok(())
    }
    pub(super) fn take_queued_resets(&mut self) -> Vec<SlotId> {
        core::mem::take(&mut self.queued_resets)
    }
    /// `usb_reset_device` for a queued request: Disable Slot stops every
    /// class transfer, as unbinding the drivers kills their URBs, and the fresh
    /// device chooses its configuration again, unclaimed for normal binding.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn reset_queued(
        &mut self,
        device: &Device,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<Device, (String, BlockError)> {
        let slot = during("validate queued reset", self.validate_id(device.id))?;
        let configured = &self.slot(slot).unwrap().endpoints;
        let mut endpoints: Vec<Endpoint> = Vec::new();
        for endpoint in interfaces(&device.configuration)
            .iter()
            .flat_map(|interface| interface.endpoints.iter())
        {
            let dci = endpoint.dci();
            if configured[usize::from(dci)].is_some()
                && !endpoints.iter().any(|known| known.dci() == dci)
            {
                endpoints.push(*endpoint);
            }
        }
        // SAFETY: the class owner asked for this reset and its old-generation
        // identity fails every access afterwards; Disable Slot retires every
        // configured endpoint before any of its buffers could be reused.
        let mut fresh = unsafe { self.reset_and_identify(device, &endpoints, dma) }?;
        self.choose(&mut fresh, &device.location, failures);
        let slot = self.slot(fresh.slot).unwrap();
        slot.consumer = identity::Consumer::Unclaimed;
        slot.inventory = Some(fresh.clone());
        Ok(fresh)
    }
    fn reset_preflight(&self, device: &Device, endpoints: &[Endpoint]) -> Result<(), BlockError> {
        if self.validate_id(device.id)? != device.slot {
            return Err(BlockError::Unsupported(
                "USB reset descriptor slot identity mismatch",
            ));
        }
        let slot = self.slots[usize::from(device.slot)].as_ref().unwrap();
        let a = slot.attachment;
        let malformed_path = match a.parent {
            None => a.tier != 0 || a.route != 0 || a.translator.is_some(),
            Some((parent, port)) => {
                a.tier == 0
                    || a.tier > MAX_TIERS
                    || a.route == 0
                    || port == 0
                    || self.live_slot(parent).is_err()
                    || self
                        .power
                        .hub_ports(parent)
                        .is_none_or(|ports| port > ports)
            }
        };
        if malformed_path
            || device.descriptor[4] == HUB_CLASS
            || a.root_port == 0
            || a.root_port > self.max_ports
            || a.speed != device.speed
        {
            return Err(BlockError::Unsupported(
                "USB reset requires the original exclusive leaf attachment",
            ));
        }
        if self.slots.iter().enumerate().any(|(id, candidate)| {
            id != usize::from(device.slot)
                && self.slot_ownership[id] == Ownership::Enabled
                // A grant without completed attachment metadata is unknown;
                // do not infer that resetting this port cannot affect it.
                && candidate
                    .as_ref()
                    .is_none_or(|other| match a.parent {
                        None => other.attachment.root_port == a.root_port,
                        Some(parent) => other.attachment.parent == Some(parent)
                            || self.power.belongs_to(id as u8, device.slot),
                    })
        }) {
            return Err(BlockError::Unsupported(
                "USB reset physical port has another unretired slot owner",
            ));
        }
        if self.power.port_owner(a.parent, a.root_port) != Some(device.slot) {
            return Err(BlockError::Unsupported(
                "USB reset leaf lacks retained physical port ownership",
            ));
        }
        if slot.current_configuration != Some(device.configuration_value())
            || device.configuration_value() == 0
        {
            return Err(BlockError::Unsupported(
                "USB reset configuration does not match attachment",
            ));
        }
        let mut expected = 0u32;
        for endpoint in endpoints {
            let dci = endpoint.dci();
            if dci < 2 || expected & (1 << dci) != 0 {
                return Err(BlockError::Unsupported(
                    "USB reset exclusive endpoint inventory",
                ));
            }
            expected |= 1 << dci;
        }
        let actual = slot
            .endpoints
            .iter()
            .enumerate()
            .skip(2)
            .fold(0u32, |bits, (dci, endpoint)| {
                bits | if endpoint.is_some() { 1 << dci } else { 0 }
            });
        if expected == 0 || expected != actual {
            return Err(BlockError::Unsupported(
                "USB reset does not exclusively own every configured endpoint",
            ));
        }
        Ok(())
    }
    /// Linux `hub_port_reset` for a root port's device: a hot reset (PR),
    /// or a warm reset (WPR) where the USB 3 link sits in SS.Inactive or
    /// Compliance Mode or never trained (`hub_port_warm_reset_required`),
    /// as a device whose link failed after it hung needs. Up to
    /// [`PORT_RESET_TRIES`] resets run; one that ends with the link needing a
    /// warm reset is followed by one. The port must come back enabled at the
    /// device's speed, a USB 3 link in U0.
    pub(super) fn reset_root_device(&mut self, attachment: Attachment) -> Result<(), BlockError> {
        self.check_minimum()?;
        let port = attachment.root_port;
        self.validate_current_graph(port)?;
        let at = self.port_offset(port);
        let before = self.registers.read32(at);
        let mut warm = self.warm_reset_required(port, before);
        if before & PP == 0 || before & CCS == 0 && !warm {
            return Err(BlockError::Unsupported(
                "USB reset root device is disconnected or unpowered",
            ));
        }
        // usb_reset_and_verify_device turns hardware LPM off first; a
        // failure is ignored, as in Linux.
        let _ = self.disable_usb2_lpm(port);
        let superspeed = matches!(attachment.speed, SUPER_SPEED | SUPER_SPEED_PLUS);
        for _ in 0..PORT_RESET_TRIES {
            // Clear all prior change bits to establish a fresh PRC/event
            // transition.
            let current = self.registers.read32(at);
            self.write_port(port, current & PORT_CHANGES);
            self.drain();
            if self.registers.read32(at) & PORT_CHANGES != 0 {
                return Err(BlockError::Unsupported(
                    "USB reset could not acknowledge previous port changes",
                ));
            }
            self.write_port(port, if warm { WPR } else { PR });
            let portsc = self.root_reset_done(port)?;
            self.write_port(port, portsc & PORT_CHANGES);
            if portsc & (CCS | PED) == CCS | PED
                && (portsc >> PORT_SPEED_SHIFT & PORT_SPEED_FIELD) as u8 == attachment.speed
                && (!superspeed || portsc & PLS == 0)
            {
                self.minimum_wait(POST_RESET_US)?;
                self.current_port_reset(None, port)?;
                return Ok(());
            }
            let needs_warm = self.warm_reset_required(port, portsc);
            // hub_port_reset: a reset that leaves no connection ends it.
            if portsc & CCS == 0 && !needs_warm {
                return Err(BlockError::Unsupported(
                    "USB root reset left the port without its device",
                ));
            }
            warm |= needs_warm;
        }
        Err(BlockError::Unsupported(
            "USB root reset did not restore the original enabled link",
        ))
    }

    /// The root port's status once the reset written to it ended: its
    /// port status event, Port Reset clear, and Port Reset Change or Warm
    /// Port Reset Change set.
    fn root_reset_done(&mut self, port: u8) -> Result<u32, BlockError> {
        let at = self.port_offset(port);
        let deadline = self.clock.deadline(PORT_RESET_US);
        let mut acknowledged_event = false;
        loop {
            let status = self.registers.read32(self.operational + USBSTS);
            if status & (HSE | HCE) != 0 {
                self.error = Some(status);
                self.command_poisoned = true;
                return Err(BlockError::Device { status });
            }
            while let Some(event) = self.next_event() {
                if event.kind() == PORT_STATUS_CHANGE && event.parameter == u64::from(port) << 24 {
                    if event.code() != SUCCESS {
                        return Err(BlockError::Device {
                            status: event.code(),
                        });
                    }
                    acknowledged_event = true;
                } else if matches!(event.kind(), TRANSFER_EVENT | COMMAND_COMPLETION) {
                    self.pending.push(event);
                }
            }
            self.publish_events();
            let portsc = self.registers.read32(at);
            if acknowledged_event && portsc & (PRC | WRC) != 0 && portsc & PR == 0 {
                return Ok(portsc);
            }
            if deadline.passed() {
                return Err(BlockError::Timeout);
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// The root port a reset device is on still holds it, before its slot
    /// is retired: its power and current as [`Self::observe_current_ancestors_for`]
    /// reads them, or, for a USB 3 link that only a warm reset brings back,
    /// power without over-current, as its connection and speed are unknown
    /// until that reset.
    fn observe_reset_root(&mut self, id: SlotId, port: u8) -> Result<(), BlockError> {
        self.validate_current_graph(port)?;
        let portsc = self.registers.read32(self.port_offset(port));
        if !self.warm_reset_required(port, portsc) {
            return self.observe_current_ancestors_for(id, false);
        }
        self.validate_ancestry(id)?;
        if portsc & PP == 0 || portsc & OVER_CURRENT != 0 {
            return Err(BlockError::Unsupported(
                "USB root port power absent or over-current",
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests;
