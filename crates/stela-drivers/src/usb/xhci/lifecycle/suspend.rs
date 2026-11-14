// SPDX-License-Identifier: GPL-2.0-only
//! Native topology suspend, with fresh consumers after acknowledged resume.
//!
//! Linux hub.c usb_port_suspend/usb_port_resume and xhci-hub.c
//! xhci_bus_suspend/xhci_bus_resume provide the physical ordering. Canceled
//! class TDs are never replayed: resume resets and publishes fresh generations.
use super::*;
mod time;

const LINK_STROBE: u32 = 1 << 16;
const LINK_CHANGE: u32 = 1 << 22;
const OVER_CURRENT: u32 = 1 << 3;
const U3: u32 = 3;
const RESUME: u32 = 15;
const SUSPEND_SETTLE_US: u64 = 10_000;
// xhci_bus_resume uses include/linux/usb.h USB_RESUME_TIMEOUT, 40ms.
const RESUME_SIGNAL_US: u64 = 40_000;
const LINK_TIMEOUT_US: u64 = 100_000;
const RESUME_LINK_TIMEOUT_US: u64 = 10_000;
const RESUME_RECOVERY_US: u64 = 10_000;

#[derive(Default)]
pub(super) enum Sleep {
    #[default]
    Awake,
    Suspended(Snapshot),
    /// Any uncertain transition keeps every consumer fenced and DMA kept.
    Failed,
}
impl Sleep {
    pub(super) fn awake(&self) -> bool {
        matches!(self, Self::Awake)
    }
}
pub(super) struct Snapshot {
    topology: Vec<(SlotId, Attachment)>,
    roots: Vec<(u8, u32)>,
}

impl Host {
    /// Suspend this complete controller topology. Deliver `removed` to native
    /// consumers even when a later physical transition fails. Old handles are
    /// fenced before the first Stop and never become usable again. Remote wake
    /// is disabled by acknowledged device/function requests; this explicit
    /// service does not install a platform wake interrupt or enter system sleep.
    pub fn suspend(&mut self) -> Changes {
        let mut changes = Changes::default();
        if !self.sleep.awake() {
            changes.added.failures.push((
                "USB suspend".into(),
                BlockError::Unsupported("USB topology is already suspended or failed"),
            ));
            return changes;
        }
        if let Err(error) = time::check(self.controller.borrow().clock) {
            changes
                .added
                .failures
                .push(("USB suspend timing".into(), error));
            return changes;
        }
        self.reconcile(&mut changes);
        let result = self.prepare_suspend(&mut changes);
        match result {
            Ok(snapshot) => self.sleep = Sleep::Suspended(snapshot),
            Err(error) => {
                if matches!(self.sleep, Sleep::Failed) {
                    self.controller.borrow_mut().quarantine_suspended_slots();
                }
                changes
                    .added
                    .failures
                    .push(("USB topology suspend".into(), error));
            }
        }
        changes
    }

    fn prepare_suspend(&mut self, changes: &mut Changes) -> Result<Snapshot, BlockError> {
        let mut c = self.controller.borrow_mut();
        if c.quirks.reset_on_resume {
            return Err(BlockError::Unsupported(
                "USB suspend requires controller reset/republication profile support",
            ));
        }
        let mut topology = Vec::new();
        for (slot, ownership) in c.slot_ownership.iter().enumerate().skip(1) {
            if *ownership != reset::Ownership::Enabled {
                continue;
            }
            let state = c.slots[slot].as_ref().ok_or(BlockError::Unsupported(
                "USB suspend has an uninitialized hardware slot grant",
            ))?;
            if state.retiring || state.suspended || state.inventory.is_none() {
                return Err(BlockError::Unsupported(
                    "USB suspend has unresolved device ownership",
                ));
            }
            let id = state.inventory.as_ref().unwrap().id;
            c.validate_topology_id(id)?;
            topology.push((id, state.attachment));
        }
        for port in 1..=c.max_ports {
            c.validate_current_graph(port)?;
        }
        c.ready()?;
        let mut roots = Vec::new();
        for port in 1..=c.max_ports {
            let at = c.port_offset(port);
            let value = c.registers.read32(at);
            if value == u32::MAX || value & (OVER_CURRENT | CSC) != 0 {
                return Err(BlockError::Unsupported("USB root changed before suspend"));
            }
            if value & CCS != 0 {
                if value & (PP | PED | PLS) != PP | PED {
                    return Err(BlockError::Unsupported(
                        "USB suspend requires stable enabled U0 roots",
                    ));
                }
                roots.push((port, value));
            }
        }
        // No hardware or consumer mutation preceded the complete preflight.
        // A failed transition from here cannot make the old handles usable.
        self.sleep = Sleep::Failed;
        for &(id, _) in &topology {
            c.slot(id.hardware_slot())?.suspended = true;
        }
        changes.removed.append(&mut self.devices);
        for &(id, _) in &topology {
            c.disable_consumer_wake(id)?;
        }
        topology.sort_by_key(|(_, attachment)| core::cmp::Reverse(attachment.tier));
        for &(id, attachment) in &topology {
            // An earlier descendant request re-rings upstream hub EP0. Recancel
            // this exact subtree immediately before suspending its uplink.
            c.stop_suspend_subtree(id)?;
            if let Some((_, port)) = attachment.parent {
                let parent = c.original_parent(id)?.ok_or(BlockError::Unsupported(
                    "USB suspend original parent absent",
                ))?;
                c.suspend_hub_port_for(parent, port)?;
            }
        }
        for &(port, before) in &roots {
            let actual = c.read_suspend_port(port);
            root_session(before, actual)?;
            // Neutral writable bits, with all host wake enables disabled.
            c.root_link_write(port, actual, U3, LINK_CHANGE);
            c.wait_root_link(port, before, U3, false)?;
        }
        time::delay(&mut c, SUSPEND_SETTLE_US)?;
        c.suspend_controller()?;
        for hub in &mut self.hubs {
            hub.pending = None;
            hub.attachments.clear();
        }
        Ok(Snapshot { topology, roots })
    }

    /// Restore controller/link state, retire the canceled old generations,
    /// then reset and enumerate complete fresh devices/class owners. Consumers
    /// must apply the returned additions; a failed restoration keeps every
    /// old allocation and leaves all old handles fenced.
    pub fn resume(
        &mut self,
        dma: &mut dyn DmaAllocator,
        firmware: &dyn crate::firmware::Provider,
    ) -> Changes {
        let mut changes = Changes::default();
        if matches!(&self.sleep, Sleep::Suspended(_))
            && let Err(error) = time::check(self.controller.borrow().clock)
        {
            changes
                .added
                .failures
                .push(("USB resume timing".into(), error));
            return changes;
        }
        let previous = core::mem::replace(&mut self.sleep, Sleep::Failed);
        let Sleep::Suspended(snapshot) = previous else {
            self.sleep = previous;
            changes.added.failures.push((
                "USB resume".into(),
                BlockError::Unsupported("USB topology has no acknowledged suspend snapshot"),
            ));
            return changes;
        };
        if let Err(error) = self.restore_suspend(&snapshot) {
            self.controller
                .borrow_mut()
                .quarantine_suspend_snapshot(&snapshot);
            changes
                .added
                .failures
                .push(("USB topology resume".into(), error));
            return changes;
        }
        // All physical ancestors are awake. Private descendant cancellation
        // can now clear TT buffers, then Disable Slot retires old DMA.
        let roots: Vec<_> = snapshot
            .topology
            .iter()
            .filter(|(_, attachment)| attachment.parent.is_none())
            .map(|(id, _)| *id)
            .collect();
        for slot in roots {
            if let Err(error) = self.retire(slot, &mut changes) {
                self.controller
                    .borrow_mut()
                    .quarantine_suspend_snapshot(&snapshot);
                changes
                    .added
                    .failures
                    .push(("USB resumed device retirement".into(), error));
                return changes;
            }
        }
        {
            let mut c = self.controller.borrow_mut();
            for &(port, _) in &snapshot.roots {
                // A real reset is required even for a trained USB3 U0 link:
                // it retires old physical addresses/current before rebind.
                let value = c.read_suspend_port(port);
                let speed = ((value >> PORT_SPEED_SHIFT) & PORT_SPEED_FIELD) as u8;
                let attachment = Attachment {
                    root_port: port,
                    route: 0,
                    tier: 0,
                    speed,
                    parent: None,
                    translator: None,
                };
                if let Err(error) = c.reset_root_device(attachment) {
                    c.quarantine_suspend_snapshot(&snapshot);
                    changes
                        .added
                        .failures
                        .push((format!("USB resumed root {port} reset"), error));
                    return changes;
                }
            }
            changes.added.devices = c.enumerate(dma, &mut changes.added.failures);
            for port in 1..=c.max_ports {
                self.roots[usize::from(port - 1)] = c.read_suspend_port(port);
            }
        }
        // The old generation's interfaces never settle.
        self.settling.clear();
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
        self.sleep = Sleep::Awake;
        self.settle(&mut changes, dma);
        changes
    }

    fn restore_suspend(&mut self, snapshot: &Snapshot) -> Result<(), BlockError> {
        let mut c = self.controller.borrow_mut();
        time::check(c.clock)?;
        c.validate_suspend_snapshot(snapshot)?;
        c.resume_controller()?;
        for &(port, before) in &snapshot.roots {
            c.validate_suspend_snapshot(snapshot)?;
            let actual = c.read_suspend_port(port);
            root_session(before, actual)?;
            if actual >> PLS_SHIFT & PLS_FIELD != U3 {
                return Err(BlockError::Unsupported(
                    "USB suspended root lost its owned U3 session",
                ));
            }
            let usb2 = c.port_major[usize::from(port - 1)] == 2;
            c.root_link_write(port, actual, if usb2 { RESUME } else { 0 }, LINK_CHANGE);
            if usb2 {
                time::delay(&mut c, RESUME_SIGNAL_US)?;
                let actual = c.read_suspend_port(port);
                root_session(before, actual)?;
                c.root_link_write(port, actual, 0, LINK_CHANGE);
            }
            c.wait_root_link(port, before, 0, true)?;
            let actual = c.read_suspend_port(port);
            c.root_link_write(port, actual, 0, LINK_CHANGE);
        }
        time::delay(&mut c, RESUME_RECOVERY_US)?;
        // Restore parents before sending controls through their EP0 to children.
        for &(id, attachment) in snapshot.topology.iter().rev() {
            if !c.known_id(id)
                || c.slot_ownership[usize::from(id.hardware_slot())] != reset::Ownership::Enabled
            {
                return Err(BlockError::Unsupported(
                    "USB suspended device generation changed",
                ));
            }
            c.validate_suspend_snapshot(snapshot)?;
            if let Some((_, port)) = attachment.parent {
                let parent = c
                    .original_parent(id)?
                    .ok_or(BlockError::Unsupported("USB resume original parent absent"))?;
                c.resume_hub_port_for(parent, port)?;
            }
        }
        Ok(())
    }
}

fn root_session(before: u32, actual: u32) -> Result<(), BlockError> {
    let authority = PP | CCS | PED | OVER_CURRENT | (PORT_SPEED_FIELD << PORT_SPEED_SHIFT);
    if actual == u32::MAX
        || (before ^ actual) & authority != 0
        || actual & (CSC | OVER_CURRENT | PR | WPR) != 0
    {
        return Err(BlockError::Unsupported(
            "USB suspend root connection/power session changed",
        ));
    }
    Ok(())
}

impl Controller {
    /// Terminal transition failures have no remaining private port operations.
    /// Close the raw-slot consumer boundary as well as the SlotId boundary.
    /// This is a pure quarantine: ownership, pending TDs, poison, contexts and
    /// DMA allocations remain unchanged, with no inferred Stop/Disable ACK.
    /// Successful suspend/resume keeps its private parent EP0 path until all
    /// link operations finish; only a failed path takes this terminal fence.
    fn quarantine_suspended_slots(&mut self) {
        for state in self.slots.iter_mut().flatten() {
            if state.suspended {
                state.retiring = true;
            }
        }
    }

    fn quarantine_suspend_snapshot(&mut self, snapshot: &Snapshot) {
        for &(id, _) in &snapshot.topology {
            if self.known_id(id)
                && let Some(state) = self.slots[usize::from(id.hardware_slot())].as_mut()
                && state.suspended
            {
                state.retiring = true;
            }
        }
    }

    fn validate_suspend_snapshot(&self, snapshot: &Snapshot) -> Result<(), BlockError> {
        if self.slot_ownership.iter().enumerate().any(|(slot, owner)| {
            *owner == reset::Ownership::Enabled
                && !snapshot
                    .topology
                    .iter()
                    .any(|(id, _)| usize::from(id.hardware_slot()) == slot && self.known_id(*id))
        }) {
            return Err(BlockError::Unsupported(
                "USB suspended topology gained a foreign generation",
            ));
        }
        for &(id, attachment) in &snapshot.topology {
            self.validate_topology_id(id)?;
            if self.slots[usize::from(id.hardware_slot())]
                .as_ref()
                .unwrap()
                .attachment
                != attachment
            {
                return Err(BlockError::Unsupported(
                    "USB suspended original attachment changed",
                ));
            }
            self.validate_current_graph(attachment.root_port)?;
        }
        Ok(())
    }

    fn read_suspend_port(&mut self, port: u8) -> u32 {
        let at = self.port_offset(port);
        self.registers.read32(at)
    }

    fn root_link_write(&mut self, port: u8, actual: u32, link: u32, acknowledge: u32) {
        let at = self.port_offset(port);
        self.registers.write32(
            at,
            actual & PP | LINK_STROBE | link << PLS_SHIFT | acknowledge,
        );
    }

    fn wait_root_link(
        &mut self,
        port: u8,
        before: u32,
        link: u32,
        change: bool,
    ) -> Result<(), BlockError> {
        let deadline = time::deadline(
            self.clock,
            if change {
                RESUME_LINK_TIMEOUT_US
            } else {
                LINK_TIMEOUT_US
            },
        )?;
        loop {
            self.ready()?;
            let actual = self.read_suspend_port(port);
            root_session(before, actual)?;
            if actual >> PLS_SHIFT & PLS_FIELD == link && (!change || actual & LINK_CHANGE != 0) {
                return Ok(());
            }
            if deadline.passed() {
                return Err(BlockError::Timeout);
            }
            time::poll_delay(self.clock, 100)?;
        }
    }

    fn stop_suspend_subtree(&mut self, ancestor: SlotId) -> Result<(), BlockError> {
        let slot = self.validate_topology_id(ancestor)?;
        let root = self.slots[usize::from(slot)]
            .as_ref()
            .unwrap()
            .attachment
            .root_port;
        self.validate_current_graph(root)?;
        let mut slots: Vec<_> = self
            .slots
            .iter()
            .enumerate()
            .filter_map(|(id, state)| {
                state
                    .as_ref()
                    .filter(|_| {
                        self.slot_ownership[id] == reset::Ownership::Enabled
                            && self.power.belongs_to(id as u8, ancestor.hardware_slot())
                    })
                    .map(|state| (id as u8, state.attachment.tier))
            })
            .collect();
        slots.sort_by_key(|(_, tier)| core::cmp::Reverse(*tier));
        for (slot, _) in slots {
            for dci in 1..32 {
                if self.slot(slot)?.endpoints[dci].is_some() {
                    self.cancel_inner(slot, dci as u8)?;
                }
            }
        }
        Ok(())
    }

    fn disable_consumer_wake(&mut self, id: SlotId) -> Result<(), BlockError> {
        let slot = self.validate_topology_id(id)?;
        let state = self.slot(slot)?;
        let value = state.current_configuration.ok_or(BlockError::Unsupported(
            "USB suspend configuration unresolved",
        ))?;
        if value == 0 {
            return Ok(());
        }
        let inventory = state.inventory.as_ref().ok_or(BlockError::Unsupported(
            "USB suspend descriptor inventory absent",
        ))?;
        let configuration = inventory
            .configurations
            .iter()
            .find(|bytes| bytes.get(5) == Some(&value))
            .ok_or(BlockError::Unsupported(
                "USB suspend configuration inventory changed",
            ))?;
        let active: Vec<_> = interfaces(configuration)
            .into_iter()
            .filter(|interface| interface.alternate == 0)
            .map(|interface| interface.number)
            .collect();
        let superspeed = state.speed >= SUPER_SPEED;
        let status = self.control_owned(slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])?;
        if status.len() != 2 {
            return Err(BlockError::Unsupported("short USB suspend device status"));
        }
        let status = u16::from_le_bytes([status[0], status[1]]);
        if !superspeed {
            if status & !3 != 0 {
                return Err(BlockError::Unsupported(
                    "reserved USB2 suspend device status",
                ));
            }
            if status & 2 != 0 {
                self.control_owned(slot, setup(0, CLEAR_FEATURE, 1, 0, 0), &[])?;
            }
        } else {
            if status & !0x1f != 0 {
                return Err(BlockError::Unsupported(
                    "reserved USB3 suspend device status",
                ));
            }
            // Disable every interface's function remote-wakeup/LP flags,
            // including composite devices, rather than assuming interface zero.
            for interface in active {
                self.control_owned(slot, setup(1, SET_FEATURE, 0, u16::from(interface), 0), &[])?;
            }
            for (bit, selector) in [(2, 48), (3, 49), (4, 50)] {
                if status & (1 << bit) != 0 {
                    self.control_owned(slot, setup(0, CLEAR_FEATURE, selector, 0, 0), &[])?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
