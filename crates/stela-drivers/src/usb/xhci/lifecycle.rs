// SPDX-License-Identifier: GPL-2.0-only
//! Explicitly polled topology ownership, adapting pinned Linux hub_irq,
//! port_event, hub_port_connect and usb_disconnect. A disconnected device's
//! slot goes to a later device once its port's current grant retires; every
//! identity of the old generation then fails.
use super::*;

mod after_boot;
mod attach;
mod recovery;
mod suspend;

/// Private proof kept only after complete subtree transfer quiescence.
/// This never restores public access to the old hardware slot.
pub(super) struct Recovery {
    slot: u8,
    id: SlotId,
    port: Option<u8>,
}
impl Recovery {
    pub(super) fn id(&self) -> SlotId {
        self.id
    }
    pub(super) fn slot(&self) -> u8 {
        self.slot
    }
}

/// One poll's topology changes. Removed descriptors name permanently
/// invalidated old owners; they do not assert that failed hardware DMA stopped.
/// Added class owners come from fresh physical enumeration and normal binding.
#[derive(Default)]
pub struct Changes {
    pub added: Found,
    pub removed: Vec<Device>,
}

struct Hub {
    device: Device,
    attachment: Attachment,
    endpoint: Endpoint,
    ports: u8,
    ssp: Option<hub::descriptor::Ssp>,
    bitmap: memory::Buffer,
    pending: Option<u64>,
    retired: bool,
    recovery: Option<Recovery>,
    attachments: Vec<attach::Pending>,
}

/// Keeps the controller even when no leaf class currently owns it. Call
/// `poll` from the native service loop, and deliver its changes to consumers.
/// Automatic reconnection uses fresh generations, reusing a retired slot once
/// its port's current grant has retired. The separate exclusive root-leaf
/// class reset service can reuse storage only after its typed retirement and
/// physical-reset proof; this Host path does not mint that permission. A class
/// owner's `queue_reset` is serviced here instead, without storage reuse.
/// The explicit topology suspend/resume service does not enter platform sleep
/// or authorize a guest-directed controller reset.
pub struct Host {
    controller: Shared,
    devices: Vec<Device>,
    hubs: Vec<Hub>,
    roots: Vec<u32>,
    sleep: suspend::Sleep,
    /// The failures the last poll found, each reported then. One that
    /// stands from poll to poll is reported once, until it changes or ends.
    standing: Vec<(String, BlockError)>,
    /// Bound Bulk-Only interfaces waiting out their settle time, each
    /// discovered by the first poll after it.
    settling: Vec<storage::settle::Settling>,
    /// The connection after boot under way, and those waiting their turn.
    attaching: Option<after_boot::Attaching>,
    queued: alloc::collections::VecDeque<after_boot::Queued>,
}
impl Host {
    /// Whether the controller stopped answering commands (Linux
    /// xhci_hc_died): none of its devices can be reached again without a
    /// controller reset.
    pub fn died(&self) -> bool {
        self.controller
            .try_borrow()
            .is_ok_and(|c| matches!(c.hardware, hardware::State::Died))
    }
    /// What the controller was doing when it stopped answering.
    pub fn death(&self) -> Option<super::Death> {
        self.controller.try_borrow().ok().and_then(|c| c.death)
    }
    /// Transfer this inventory's unbound Address-state device to the
    /// forwarding adapter. A failed constructor keeps the exclusive claim:
    /// neither dropping a handle nor a copied descriptor authorizes rebinding.
    pub(in crate::usb) fn take_forwarding(
        &mut self,
        id: SlotId,
    ) -> Result<(Shared, Device), BlockError> {
        let physical =
            self.devices
                .iter()
                .find(|device| device.id == id)
                .ok_or(BlockError::Unsupported(
                    "USB forwarding device is not in this host inventory",
                ))?;
        if physical.descriptor[4] == HUB_CLASS {
            return Err(BlockError::Unsupported(
                "USB hub forwarding requires topology ownership",
            ));
        }
        self.controller.borrow_mut().claim_forwarding(id)?;
        Ok((self.controller.clone(), physical.clone()))
    }

    pub(super) fn new(
        controller: Shared,
        devices: &[Device],
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Self {
        let roots = {
            let mut c = controller.borrow_mut();
            (1..=c.max_ports)
                .map(|port| {
                    let at = c.port_offset(port);
                    c.registers.read32(at)
                })
                .collect()
        };
        let mut host = Self {
            controller,
            devices: devices.to_vec(),
            hubs: Vec::new(),
            roots,
            sleep: suspend::Sleep::default(),
            standing: Vec::new(),
            settling: Vec::new(),
            attaching: None,
            queued: Default::default(),
        };
        host.register_hubs(devices, dma, failures);
        host
    }

    fn register_hubs(
        &mut self,
        devices: &[Device],
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) {
        for device in devices.iter().filter(|d| d.descriptor[4] == HUB_CLASS) {
            let result = (|| {
                let mut c = self.controller.borrow_mut();
                c.validate_id(device.id)?;
                let ports = c
                    .power
                    .hub_ports(device.slot)
                    .ok_or(BlockError::Unsupported(
                        "USB hub has no acknowledged power contract",
                    ))?;
                // Configure Endpoint committed MTT only after SET_INTERFACE1
                // ACK. Select that setting, whose endpoint may differ.
                const SLOT_MULTI_TT: u32 = 1 << 25;
                let alternate = if c.slot(device.slot)?.context[0] & SLOT_MULTI_TT != 0 {
                    MULTI_TT_ALTERNATE
                } else {
                    0
                };
                let endpoint = enumerate::hub_status_endpoint(&device.configuration, alternate)
                    .ok_or(BlockError::Unsupported("USB hub status endpoint absent"))?;
                if endpoint.kind() != 3
                    || !endpoint.is_in()
                    || usize::from(endpoint.max_packet & 0x7ff) < usize::from(ports + 1).div_ceil(8)
                {
                    return Err(BlockError::Unsupported(
                        "USB hub status bitmap exceeds actual endpoint",
                    ));
                }
                c.device(device.id)?.endpoint_ready(endpoint.dci())?;
                let attachment = c.slot(device.slot)?.attachment;
                let ssp = if matches!(device.speed, SUPER_SPEED | SUPER_SPEED_PLUS) {
                    c.hub_ssp_for(device.id)?
                } else {
                    None
                };
                let bitmap = c.allocate(dma, 64, 64)?;
                Ok(Hub {
                    device: device.clone(),
                    attachment,
                    endpoint,
                    ports,
                    ssp,
                    bitmap,
                    pending: None,
                    retired: false,
                    recovery: None,
                    attachments: Vec::new(),
                })
            })();
            match result {
                Ok(hub) => self.hubs.push(hub),
                Err(error) => failures.push((
                    format!("{} retain hub status owner", device.location),
                    error,
                )),
            }
        }
    }

    /// Whether the controller sends the handoff's completion signal as it
    /// posts an event, so [`Self::due`] can stand in for a timed poll.
    pub fn signals(&self) -> bool {
        self.controller
            .try_borrow()
            .is_ok_and(|controller| controller.signals())
    }

    /// Whether [`Self::poll`] has work, as `xhci_irq` and `hub_irq` find it:
    /// an event on the ring, a root port's status change or a hub's status
    /// transfer that completed, a queued reset, a hub to recover, a dead
    /// controller, a device whose slot changed, an attachment timer due, or
    /// a connection after boot to step.
    /// This does not access controller registers.
    pub fn due(&self) -> bool {
        let Ok(c) = self.controller.try_borrow() else {
            return true;
        };
        if !self.sleep.awake() {
            return false;
        }
        let hub = |slot: u8| {
            self.hubs.iter().any(|hub| {
                hub.pending.is_some() && !hub.retired && hub.device.id.hardware_slot() == slot
            })
        };
        let changed = c.event_posted()
            || (matches!(c.hardware, hardware::State::Died) && !self.devices.is_empty())
            || !c.queued_resets.is_empty()
            || c.pending.iter().any(|event| {
                event.kind() == reset::PORT_STATUS_CHANGE
                    || (event.kind() == super::TRANSFER_EVENT && hub(event.slot()))
            })
            || self.hubs.iter().any(|hub| hub.recovery.is_some())
            || self.attachment_due();
        if changed {
            return true;
        }
        let mut count = 0;
        for device in Self::current(&c) {
            if !self.devices.iter().any(|old| old.id == device.id) {
                return true;
            }
            count += 1;
        }
        count != self.devices.len()
    }

    /// [`Self::poll`] once the boot handed the machine over, from a service
    /// turn: from then on a device that arrives gets only the classes the
    /// running system takes, input devices and the hubs they sit behind, and
    /// the others are reported unused without being started.
    pub fn poll_after_boot(
        &mut self,
        dma: &mut dyn DmaAllocator,
        firmware: &dyn crate::firmware::Provider,
    ) -> Changes {
        self.controller.borrow_mut().after_boot = true;
        self.poll(dma, firmware)
    }

    /// Poll root notifications/PORTSC and queued hub interrupt bitmaps.
    /// No completion or new identity is inferred from elapsed time. Topology
    /// mutation is serialized through this controller's native Rc owner.
    pub fn poll(
        &mut self,
        dma: &mut dyn DmaAllocator,
        firmware: &dyn crate::firmware::Provider,
    ) -> Changes {
        let mut changes = self.poll_changes(dma, firmware);
        // A port or hub whose refusal stands fails the same way on every
        // poll, thousands of times a second: say it once.
        let found = changes.added.failures.clone();
        changes
            .added
            .failures
            .retain(|failure| !self.standing.contains(failure));
        self.standing = found;
        changes
    }

    fn poll_changes(
        &mut self,
        dma: &mut dyn DmaAllocator,
        firmware: &dyn crate::firmware::Provider,
    ) -> Changes {
        let mut changes = Changes::default();
        if matches!(self.controller.borrow().hardware, hardware::State::Died) {
            self.hc_died(&mut changes);
            return changes;
        }
        if !self.sleep.awake() {
            return changes;
        }
        self.reconcile(&mut changes);
        for port in 1..=self.roots.len() as u8 {
            // A connection after boot owns its root port until it ends.
            if self.holds_root(port) {
                continue;
            }
            if let Err(error) = self.root(port, dma, &mut changes) {
                changes
                    .added
                    .failures
                    .push((format!("USB root port {port} lifecycle"), error));
            }
        }
        // Hub status work and queued resets wait for a connection after
        // boot to end: each would use a hub's control endpoint or a port the
        // connection may be resetting.
        let hubs = if self.attaching() { 0 } else { self.hubs.len() };
        for index in 0..hubs {
            if self.hubs[index].recovery.is_some() {
                if let Err(error) = self.recover_hub(index, dma, &mut changes) {
                    changes.added.failures.push((
                        format!("{} hub recovery", self.hubs[index].device.location),
                        error,
                    ));
                }
                continue;
            }
            if self.hubs[index].retired {
                continue;
            }
            if let Err(error) = self.hub(index, dma, &mut changes) {
                let location = self.hubs[index].device.location.clone();
                let current = self
                    .controller
                    .borrow()
                    .known_id(self.hubs[index].device.id);
                if self.hubs[index].recovery.is_none() && current {
                    let _ = self.retire(self.hubs[index].device.id, &mut changes);
                }
                changes
                    .added
                    .failures
                    .push((format!("{location} hub lifecycle"), error));
            }
            self.poll_hub_attachments(index, dma, &mut changes);
        }
        let queued = if self.attaching() {
            Vec::new()
        } else {
            self.controller.borrow_mut().take_queued_resets()
        };
        for id in queued {
            self.queued_reset(id, dma, &mut changes);
        }
        let added = changes.added.devices.clone();
        self.register_hubs(&added, dma, &mut changes.added.failures);
        for device in &added {
            bind_with_provider(
                &self.controller,
                device.clone(),
                dma,
                &mut changes.added,
                firmware,
            );
        }
        self.devices.extend(added);
        self.settle(&mut changes, dma);
        self.step_attaching(dma, &mut changes);
        changes
    }

    /// The first poll once the Bulk-Only interfaces bound before have
    /// settled, on a virtual clock: until then, no poll is due for them.
    #[cfg(test)]
    pub(crate) fn settled(&mut self, dma: &mut dyn DmaAllocator) -> Changes {
        assert!(!self.settling.is_empty() && !self.attachment_due());
        let clock = self.controller.borrow().clock;
        crate::test_clock::advance(clock, storage::settle::SETTLE_US);
        assert!(self.attachment_due());
        let changes = self.poll(dma, &crate::firmware::HandoffProvider::new(None));
        assert!(self.settling.is_empty());
        changes
    }

    /// Keep the interfaces this poll bound until their settle time, and
    /// discover those whose time passed. One whose device left the
    /// inventory, removed or reset, is dropped.
    pub(super) fn settle(&mut self, changes: &mut Changes, dma: &mut dyn DmaAllocator) {
        self.settling.append(&mut changes.added.settling);
        let devices = &self.devices;
        self.settling.retain(|entry| {
            entry
                .xhci_device()
                .is_some_and(|id| devices.iter().any(|device| device.id == id))
        });
        let settled = storage::settle::settle(&mut self.settling, Some(dma), false);
        changes.added.absorb_settled(settled);
    }

    /// Service one class's `usb_queue_reset_device`. A device already gone
    /// needs nothing; otherwise its old owner is reported removed once the
    /// reset retires it, and a fresh device is bound like a new one.
    fn queued_reset(&mut self, id: SlotId, dma: &mut dyn DmaAllocator, changes: &mut Changes) {
        let Some(index) = self.devices.iter().position(|device| device.id == id) else {
            return;
        };
        let old = self.devices[index].clone();
        let result =
            self.controller
                .borrow_mut()
                .reset_queued(&old, dma, &mut changes.added.failures);
        if self.controller.borrow().validate_device_id(id).is_err() {
            self.devices.remove(index);
            changes.removed.push(old.clone());
        }
        match result {
            Ok(fresh) => changes.added.devices.push(fresh),
            Err((operation, error)) => changes
                .added
                .failures
                .push((format!("{} queued reset: {operation}", old.location), error)),
        }
    }

    /// A class-owned exclusive reset may replace a leaf between topology
    /// polls. Follow the controller's fresh descriptor inventory without
    /// attempting a second native bind of that already-owned replacement.
    /// The enabled devices the controller's slots hold now.
    fn current(c: &Controller) -> impl Iterator<Item = &Device> {
        (0..c.slots.len()).filter_map(move |slot| Self::current_slot(c, slot))
    }

    fn current_slot(c: &Controller, slot: usize) -> Option<&Device> {
        let state = c.slots.get(slot)?.as_ref()?;
        if state.retiring || state.suspended || c.slot_ownership[slot] != reset::Ownership::Enabled
        {
            return None;
        }
        state.inventory.as_ref()
    }

    fn reconcile(&mut self, changes: &mut Changes) {
        let c = self.controller.borrow();
        self.devices.retain(|old| {
            if Self::current_slot(&c, usize::from(old.slot))
                .is_some_and(|device| device.id == old.id)
            {
                true
            } else {
                changes.removed.push(old.clone());
                false
            }
        });
        // A connection after boot adds the devices it found when it ends.
        if self.attaching.is_some() {
            return;
        }
        for device in Self::current(&c) {
            if !self.devices.iter().any(|old| old.id == device.id) {
                self.devices.push(device.clone());
            }
        }
    }

    fn root(
        &mut self,
        port: u8,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let (status, owner, notification_error) = {
            let mut c = self.controller.borrow_mut();
            c.validate_current_graph(port)?;
            let owner = c.current_port_owner(None, port)?;
            c.ready()?;
            let notification_error = c
                .pending
                .iter()
                .find(|event| {
                    event.kind() == reset::PORT_STATUS_CHANGE
                        && event.parameter == u64::from(port) << 24
                        && event.code() != SUCCESS
                })
                .map(Event::code);
            c.pending.retain(|event| {
                !(event.kind() == reset::PORT_STATUS_CHANGE
                    && event.parameter == u64::from(port) << 24)
            });
            let at = c.port_offset(port);
            (c.registers.read32(at), owner, notification_error)
        };
        if let Some(status) = notification_error {
            if let Some(owner) = owner {
                self.retire(owner, changes)?;
            }
            return Err(BlockError::Device { status });
        }
        // hub.c port_event: an over-current change is cleared, the ports cool
        // down for 100 ms and hub_power_on repowers every root port, waiting
        // the root hub's bPwrOn2PwrGood (xHCI's 20 ms). The attachment is
        // kept unless the port's status changed.
        let status = if status & OVER_CURRENT_CHANGE != 0 {
            let mut c = self.controller.borrow_mut();
            c.write_port(port, OVER_CURRENT_CHANGE);
            c.minimum_wait(OVER_CURRENT_COOLDOWN_US)?;
            for root in 1..=c.max_ports {
                let at = c.port_offset(root);
                if c.registers.read32(at) & PP == 0 {
                    c.write_port(root, PP);
                }
            }
            c.minimum_wait(PORT_POWER_US)?;
            let at = c.port_offset(port);
            c.registers.read32(at)
        } else {
            status
        };
        let previous = self.roots[usize::from(port - 1)];
        let authority =
            CCS | CAS | PED | PP | OVER_CURRENT | (PORT_SPEED_FIELD << PORT_SPEED_SHIFT);
        let old_link = previous >> PLS_SHIFT & PLS_FIELD;
        let new_link = status >> PLS_SHIFT & PLS_FIELD;
        let link_recovery = old_link != new_link
            && (matches!(old_link, 3 | LINK_INACTIVE | LINK_COMPLIANCE)
                || matches!(new_link, 3 | LINK_INACTIVE | LINK_COMPLIANCE));
        let changed = status & CSC != 0 || (previous ^ status) & authority != 0 || link_recovery;
        self.roots[usize::from(port - 1)] = status;
        if !changed {
            return Ok(());
        }
        if let Some(owner) = owner {
            self.retire(owner, changes)?;
            // hub.c's power budget counts only connected children: the
            // retired owners of an empty port draw no current, so their
            // grants and slots can go to the next device.
            if status & CCS == 0 {
                self.controller
                    .borrow_mut()
                    .current_port_reset(None, port)?;
            }
        } else if status & CCS == 0 {
            // A device that was refused or retired while it stayed
            // connected kept its grant with no owner. The port is empty
            // now, so the grant goes, as hub.c's budget drops a
            // disconnected child.
            self.controller
                .borrow_mut()
                .current_port_reset(None, port)?;
        }
        // A USB 3 device whose link a warm reset must bring up shows no
        // connection until then (`hub_port_warm_reset_required`).
        let warm = self.controller.borrow().warm_reset_required(port, status);
        if status & PP == 0 || (status & CCS == 0 && !warm) || status & OVER_CURRENT != 0 {
            let mut c = self.controller.borrow_mut();
            c.write_port(port, status & PORT_CHANGES);
            // Linux reports an over-current condition that persists.
            return if status & OVER_CURRENT != 0 {
                Err(BlockError::Device { status })
            } else {
                Ok(())
            };
        }
        if new_link == 3 {
            return Err(BlockError::Unsupported(
                "USB root U3 resume requires suspend owner",
            ));
        }
        let mut c = self.controller.borrow_mut();
        // A disabled USB2 port has not completed its speed handshake yet.
        // Its Supported Protocol permits reset; reserve_current still
        // requires the enabled port's speed before Address Device.
        // An untrained USB 3 link has no speed until its warm reset.
        if !warm && (c.port_major[usize::from(port - 1)] != 2 || status & PED != 0) {
            c.root_current(
                port,
                ((status >> PORT_SPEED_SHIFT) & PORT_SPEED_FIELD) as u8,
            )?;
        }
        let location = format!("{} port {port}", c.location);
        if c.after_boot {
            self.queued.push_back(after_boot::Queued {
                location,
                parent: None,
                reset: enumerate::Reset::Changed(port),
                root: Some((port, status)),
            });
            return Ok(());
        }
        let connected = c.connect(
            location,
            None,
            enumerate::Reset::Changed(port),
            dma,
            &mut changes.added.devices,
            &mut changes.added.failures,
        );
        if let Some(speed) = connected.speed {
            // Baseline only effects proved by our reset/training ACK: the
            // connection it found, and CAS a warm reset cleared. Preserve
            // prior power permission so a late physical change is observed
            // on the next poll; late CSC is never consumed here.
            let controlled = CCS | CAS | PED | (PORT_SPEED_FIELD << PORT_SPEED_SHIFT) | PLS;
            self.roots[usize::from(port - 1)] =
                status & !controlled | CCS | PED | u32::from(speed) << PORT_SPEED_SHIFT;
        }
        Ok(())
    }

    fn hub(
        &mut self,
        index: usize,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let bits = {
            let owner = &mut self.hubs[index];
            let mut c = self.controller.borrow_mut();
            c.validate_id(owner.device.id)?;
            let length = usize::from(owner.ports + 1).div_ceil(8);
            if let Some(ticket) = owner.pending {
                let Some(result) =
                    c.device(owner.device.id)?
                        .completed(owner.endpoint.dci(), ticket, length)
                else {
                    return Ok(());
                };
                let received = result?;
                owner.pending = None;
                let mut bits = 0u32;
                for (byte, value) in owner.bitmap.bytes()[..received].iter().enumerate() {
                    bits |= u32::from(*value) << (8 * byte);
                }
                let allowed = u32::MAX >> (31 - owner.ports);
                if bits & !allowed != 0 {
                    return Err(BlockError::Unsupported(
                        "USB hub status bitmap has nonexistent ports",
                    ));
                }
                bits
            } else {
                owner.bitmap.bytes_mut().fill(0);
                owner.pending = Some(c.device(owner.device.id)?.submit(
                    owner.endpoint.dci(),
                    owner.bitmap.device_address(),
                    length,
                )?);
                return Ok(());
            }
        };
        // A notification schedules status reads; it never authorizes
        // recovering or acknowledging a condition that was not observed.
        {
            let id = self.hubs[index].device.id;
            let status = self
                .controller
                .borrow_mut()
                .device(id)?
                .control(setup(0xa0, GET_STATUS, 0, 0, 4), &[])?;
            if status.len() != 4 {
                return Err(BlockError::Unsupported("short USB hub lifecycle status"));
            }
            let actual = u16::from_le_bytes([status[0], status[1]]);
            let change = u16::from_le_bytes([status[2], status[3]]);
            if actual & !3 != 0 || change & !3 != 0 {
                return Err(BlockError::Unsupported("reserved USB hub lifecycle status"));
            }
            if actual != 0 || change & 2 != 0 {
                self.start_recovery(index, None, changes)?;
                return self.recover_hub(index, dma, changes);
            }
        }
        // Bitmap is notification only: status and power permission decide.
        {
            let mut c = self.controller.borrow_mut();
            c.observe_current_ancestors_for(self.hubs[index].device.id, true)?;
        }
        for port in 1..=self.hubs[index].ports {
            if self.hubs[index].retired {
                break;
            }
            if bits & (1 << port) != 0
                && let Err(error) = self.hub_port(index, port, dma, changes)
            {
                changes.added.failures.push((
                    format!("{}.{port} port lifecycle", self.hubs[index].device.location),
                    error,
                ));
            }
        }
        if self.hubs[index].retired {
            return Ok(());
        }
        let owner = &mut self.hubs[index];
        let length = usize::from(owner.ports + 1).div_ceil(8);
        owner.bitmap.bytes_mut().fill(0);
        owner.pending = Some(
            self.controller
                .borrow_mut()
                .device(owner.device.id)?
                .submit(owner.endpoint.dci(), owner.bitmap.device_address(), length)?,
        );
        Ok(())
    }

    fn hub_port(
        &mut self,
        index: usize,
        port: u8,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let id = self.hubs[index].device.id;
        let attachment = self.hubs[index].attachment;
        let ss = attachment.speed >= SUPER_SPEED;
        let (status, change, owner) = {
            let mut c = self.controller.borrow_mut();
            let (status, change) = c.hub_port_status_for(id, port)?;
            (
                status,
                change,
                c.current_port_owner(Some((id, port)), attachment.root_port)?,
            )
        };
        if change & 8 != 0 || status & 8 != 0 {
            self.start_recovery(index, Some(port), changes)?;
            return self.recover_hub(index, dma, changes);
        }
        let reconnect = change & 1 != 0
            || status & 1 == 0
            || owner.is_none()
            || status & 2 == 0
            || status & if ss { 1 << 9 } else { 1 << 8 } == 0
            || ss && (change & (1 << 7) != 0 || matches!(status >> 5 & 15, 3 | 6 | 10));
        if reconnect && let Some(owner) = owner {
            self.retire(owner, changes)?;
        }
        let mut c = self.controller.borrow_mut();
        if owner.is_some() && status & 1 == 0 {
            c.current_hub_port_reset(id, port, attachment.root_port)?;
        }
        let selectors: &[(u16, u16)] = if ss {
            &[(1, 16), (8, 19), (16, 20), (32, 29), (64, 25), (128, 26)]
        } else {
            &[(1, 16), (2, 17), (4, 18), (8, 19), (16, 20)]
        };
        for &(bit, selector) in selectors {
            if change & bit != 0 {
                c.device(id)?.control(
                    setup(0x23, CLEAR_FEATURE, selector, u16::from(port), 0),
                    &[],
                )?;
            }
        }
        if status & if ss { 1 << 9 } else { 1 << 8 } == 0 {
            return Err(BlockError::Unsupported("USB hub port power absent"));
        }
        if ss && status >> 5 & 15 == 3 {
            return Err(BlockError::Unsupported(
                "USB hub U3 resume requires suspend owner",
            ));
        }
        if !reconnect {
            return Ok(());
        }
        if !ss && owner.is_none() {
            if let Some(pending) = self.hubs[index]
                .attachments
                .iter_mut()
                .find(|pending| pending.port == port)
            {
                // The change latch was consumed above. A bounce reported
                // between timer samples must still restart the interval.
                return pending.notice(&mut c, status & 1 != 0, change & 1 != 0);
            }
            if status & 1 != 0
                && let Some(pending) = attach::Pending::start(&mut c, id, port)?
            {
                self.hubs[index].attachments.push(pending);
                return Ok(());
            }
        }
        drop(c);
        self.connect_hub_port(index, port, false, dma, changes);
        Ok(())
    }

    fn connect_hub_port(
        &mut self,
        index: usize,
        port: u8,
        debounced: bool,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) {
        let hub = &self.hubs[index];
        let id = hub.device.id;
        let location = format!("{}.{port}", hub.device.location);
        let reset = enumerate::Reset::HubChanged {
            hub: id,
            port,
            parent: hub.attachment,
            ssp: hub.ssp.clone(),
            debounced,
        };
        if self.controller.borrow().after_boot {
            self.queued.push_back(after_boot::Queued {
                location,
                parent: Some(id),
                reset,
                root: None,
            });
            return;
        }
        self.controller.borrow_mut().connect(
            location,
            Some(id),
            reset,
            dma,
            &mut changes.added.devices,
            &mut changes.added.failures,
        );
    }

    /// Linux usb_hc_died: every device below a dead host controller is
    /// disconnected, suspended ones included. No command can disable their
    /// slots, so each stays quarantined with every original allocation.
    fn hc_died(&mut self, changes: &mut Changes) {
        self.settling.clear();
        self.attaching = None;
        self.queued.clear();
        let mut c = self.controller.borrow_mut();
        for slot in 1..c.slots.len() {
            if let Some(state) = c.slots[slot].as_mut() {
                state.suspended = true;
            }
            c.quarantine_slot_owned(slot as u8);
        }
        for hub in &mut self.hubs {
            hub.retired = true;
            hub.recovery = None;
            hub.attachments.clear();
        }
        changes.removed.append(&mut self.devices);
    }

    fn retire(&mut self, ancestor: SlotId, changes: &mut Changes) -> Result<(), BlockError> {
        self.retire_except(ancestor, None, changes)
    }

    fn retire_except(
        &mut self,
        ancestor: SlotId,
        keep: Option<SlotId>,
        changes: &mut Changes,
    ) -> Result<(), BlockError> {
        let mut c = self.controller.borrow_mut();
        c.validate_ancestry(ancestor)?;
        let root = c.slots[usize::from(ancestor.hardware_slot())]
            .as_ref()
            .unwrap()
            .attachment
            .root_port;
        c.validate_current_graph(root)?;
        let mut slots: Vec<_> = (1..c.slots.len())
            .filter(|slot| c.power.belongs_to(*slot as u8, ancestor.hardware_slot()))
            .map(|slot| slot as u8)
            .collect();
        // Validate the complete original graph before changing any consumer,
        // hub, DMA or current-owner state. No kept raw number is upgraded.
        for &slot in &slots {
            let state = c.slots[usize::from(slot)]
                .as_ref()
                .ok_or(BlockError::Unsupported(
                    "USB retirement original slot absent",
                ))?;
            let inventory = state.inventory.as_ref().ok_or(BlockError::Unsupported(
                "USB retirement original inventory absent",
            ))?;
            if state.retiring && c.slot_ownership[usize::from(slot)] == reset::Ownership::Retired {
                // Disable Slot already acknowledged this exact descendant. Its
                // original parent may also be disabled while physical current
                // remains reserved until port reset. Check the seals,
                // rather than demanding live parent hardware for an idle owner.
                if inventory.id.hardware_slot() != slot || !c.known_id(inventory.id) {
                    return Err(BlockError::Unsupported(
                        "USB retired inventory generation changed",
                    ));
                }
                c.power
                    .validate_generation(inventory.id, state.parent_id, state.attachment)?;
            } else {
                // Uncertain Stop/Disable still owns an Enabled slot and must
                // pass the complete live ancestry check before any more work.
                c.validate_ancestry(inventory.id)?;
            }
        }
        // Fence every generation before the first Stop, but keep physical
        // parents internally live for descendant TT-buffer retirement.
        for &slot in &slots {
            if let Some(state) = c.slots[usize::from(slot)].as_mut() {
                state.suspended = true;
            }
        }
        slots.sort_by_key(|slot| {
            core::cmp::Reverse(
                c.slots[usize::from(*slot)]
                    .as_ref()
                    .map_or(0, |state| state.attachment.tier),
            )
        });
        for hub in &mut self.hubs {
            if slots.contains(&hub.device.slot) && c.known_id(hub.device.id) {
                hub.retired = true;
                hub.attachments.clear();
                if Some(hub.device.id) != keep {
                    hub.recovery = None;
                }
            }
        }
        let mut retained = Vec::new();
        for device in self.devices.drain(..) {
            if slots.contains(&device.slot) && c.known_id(device.id) {
                changes.removed.push(device);
            } else {
                retained.push(device);
            }
        }
        self.devices = retained;
        // Invalidate every alias before the first Stop command. An uncertain
        // contributor blocks all Disable Slot publication and new enumeration.
        let mut failure = None;
        for &slot in &slots {
            if c.slot_ownership[usize::from(slot)] != reset::Ownership::Enabled {
                continue;
            }
            for dci in 1..32 {
                if c.slots[usize::from(slot)]
                    .as_ref()
                    .is_some_and(|state| state.endpoints[dci].is_some())
                    && let Err(error) = c.cancel_inner(slot, dci as u8)
                {
                    failure.get_or_insert(error);
                }
            }
        }
        // From here no private child cancellation needs a live parent. Failed
        // contributors remain quarantined with every original allocation.
        for &slot in &slots {
            c.quarantine_slot_owned(slot);
        }
        if let Some(error) = failure {
            return Err(error);
        }
        for slot in slots {
            if Some(slot) != keep.map(SlotId::hardware_slot)
                && c.slot_ownership[usize::from(slot)] == reset::Ownership::Enabled
            {
                c.disable_slot(slot, reset::Ownership::Retired)
                    .map_err(|(_, error)| error)?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
