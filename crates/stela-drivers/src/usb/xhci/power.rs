// SPDX-License-Identifier: GPL-2.0-only
//! Owned USB current accounting, following pinned hub.c hub_configure,
//! hub_power_remaining and the USB configuration 2-mA/8-mA units.
//! Budgets are USB-standard protocol contracts, not measured current.
use super::*;

const USB2_UNIT_MA: u16 = 100;
const USB3_UNIT_MA: u16 = 150;
const USB2_PORT_MA: u16 = 500;
const USB3_PORT_MA: u16 = 900;
const ROOT_OVER_CURRENT: u32 = 1 << 3;

fn unit(speed: u8) -> u16 {
    if matches!(speed, SUPER_SPEED | SUPER_SPEED_PLUS) {
        USB3_UNIT_MA
    } else {
        USB2_UNIT_MA
    }
}
fn full(speed: u8) -> u16 {
    if matches!(speed, SUPER_SPEED | SUPER_SPEED_PLUS) {
        USB3_PORT_MA
    } else {
        USB2_PORT_MA
    }
}

#[derive(Clone)]
struct Hub {
    ports: u8,
    controller_ma: u16,
    self_powered: bool,
    /// A bit is reserved BEFORE publishing PORT_POWER, and survives failed or
    /// uncertain ACK. Empty powered ports still reserve an initial unit load.
    powered: u32,
}
#[derive(Clone)]
struct Grant {
    /// Reserved before Address Device, kept until physical reset proof.
    generation: Option<u64>,
    parent_id: Option<SlotId>,
    attachment: Attachment,
    maximum_ma: u16,
    reserved_ma: u16,
    configurations: Vec<(u8, u16)>,
    hub: Option<Hub>,
}

pub(super) struct Ledger {
    entries: Vec<Option<Grant>>,
}
impl Ledger {
    pub fn new(slots: usize) -> Self {
        Self {
            entries: vec![None; slots],
        }
    }
    #[cfg(test)]
    pub(super) fn snapshot(&self, slot: u8) -> (u16, u16, u16) {
        let grant = self.grant(slot).unwrap();
        (
            grant.maximum_ma,
            grant.reserved_ma,
            self.draw(slot, 0).unwrap(),
        )
    }
    #[cfg(test)]
    pub(super) fn inject_parent_identity(
        &mut self,
        slot: u8,
        parent: Option<SlotId>,
    ) -> Option<SlotId> {
        core::mem::replace(&mut self.grant_mut(slot).unwrap().parent_id, parent)
    }

    pub(super) fn retained(&self, slot: u8) -> bool {
        self.grant(slot).is_ok()
    }
    fn grant(&self, slot: u8) -> Result<&Grant, BlockError> {
        self.entries
            .get(usize::from(slot))
            .and_then(Option::as_ref)
            .ok_or(BlockError::Unsupported(
                "USB device has no retained current grant",
            ))
    }
    pub(super) fn validate_generation(
        &self,
        id: SlotId,
        parent: Option<SlotId>,
        attachment: Attachment,
    ) -> Result<(), BlockError> {
        if let Ok(grant) = self.grant(id.hardware_slot()) {
            if grant.generation == Some(id.generation())
                && grant.parent_id == parent
                && grant.attachment == attachment
            {
                return Ok(());
            }
            #[cfg(test)]
            if id.generation() == 0 && grant.generation.is_none() && parent.is_none() {
                return Ok(());
            }
            return Err(BlockError::Unsupported(
                "USB physical current belongs to another generation",
            ));
        }
        // Some endpoint-only fixtures have no physical power model. Production
        // addressed generations always reserve their current before publication.
        #[cfg(test)]
        if id.generation() == 0 {
            return Ok(());
        }
        Err(BlockError::Unsupported(
            "USB original physical current grant absent",
        ))
    }
    fn original_parent(&self, slot: u8) -> Result<Option<u8>, BlockError> {
        let grant = self.grant(slot)?;
        let Some((parent, _)) = grant.attachment.parent else {
            if grant.parent_id.is_some() {
                return Err(BlockError::Unsupported(
                    "USB current root has a parent identity",
                ));
            }
            return Ok(None);
        };
        let ancestor = self.grant(parent)?;
        if let Some(id) = grant.parent_id {
            if id.hardware_slot() != parent || ancestor.generation != Some(id.generation()) {
                return Err(BlockError::Unsupported(
                    "USB current original parent generation retired",
                ));
            }
        } else if grant.generation.is_some() {
            return Err(BlockError::Unsupported(
                "USB current original parent identity absent",
            ));
        }
        Ok(Some(parent))
    }

    fn grant_mut(&mut self, slot: u8) -> Result<&mut Grant, BlockError> {
        self.entries
            .get_mut(usize::from(slot))
            .and_then(Option::as_mut)
            .ok_or(BlockError::Unsupported(
                "USB device has no retained current grant",
            ))
    }
    fn draw(&self, slot: u8, depth: u8) -> Result<u16, BlockError> {
        if depth > MAX_TIERS {
            return Err(BlockError::Unsupported("cyclic USB current ownership"));
        }
        let owner = self.grant(slot)?;
        let Some(hub) = owner.hub.as_ref().filter(|hub| !hub.self_powered) else {
            return Ok(owner.reserved_ma);
        };
        let mut total = hub.controller_ma;
        for port in 1..=hub.ports {
            if hub.powered & (1 << port) == 0 {
                continue;
            }
            let mut downstream = unit(owner.attachment.speed);
            for (id, child) in self.entries.iter().enumerate() {
                if child
                    .as_ref()
                    .is_some_and(|child| child.attachment.parent == Some((slot, port)))
                {
                    downstream = downstream.max(self.draw(id as u8, depth + 1)?);
                }
            }
            total = total
                .checked_add(downstream)
                .ok_or(BlockError::Unsupported("USB current accounting overflow"))?;
        }
        Ok(total.max(owner.reserved_ma))
    }
    fn check(&self) -> Result<(), BlockError> {
        for (slot, owner) in self
            .entries
            .iter()
            .enumerate()
            .filter_map(|(slot, owner)| owner.as_ref().map(|owner| (slot, owner)))
        {
            if self.draw(slot as u8, 0)? > owner.maximum_ma {
                return Err(BlockError::Unsupported(
                    "USB upstream current budget exhausted",
                ));
            }
        }
        Ok(())
    }
    fn reserve(
        &mut self,
        slot: u8,
        attachment: Attachment,
        root_ma: u16,
    ) -> Result<(), BlockError> {
        if self.entries.iter().flatten().any(|owner| {
            owner.attachment.root_port == attachment.root_port
                && owner.attachment.parent == attachment.parent
        }) {
            return Err(BlockError::Unsupported(
                "USB port still has an unretired current owner",
            ));
        }
        let maximum_ma = if let Some((parent, port)) = attachment.parent {
            let ancestor = self.grant(parent)?;
            if ancestor.attachment.root_port != attachment.root_port
                || ancestor.attachment.tier + 1 != attachment.tier
                || attachment.route
                    != ancestor.attachment.route
                        | u32::from(port.min(MAX_ROUTE_PORT)) << (4 * ancestor.attachment.tier)
            {
                return Err(BlockError::Unsupported(
                    "USB child current grant has a foreign physical parent",
                ));
            }
            let hub = ancestor.hub.as_ref().ok_or(BlockError::Unsupported(
                "USB parent has no configured hub power contract",
            ))?;
            if port == 0 || port > hub.ports || hub.powered & (1 << port) == 0 {
                return Err(BlockError::Unsupported(
                    "USB child port power was not reserved",
                ));
            }
            if hub.self_powered {
                full(attachment.speed)
            } else {
                unit(attachment.speed)
            }
        } else {
            if attachment.tier != 0 || attachment.route != 0 || full(attachment.speed) != root_ma {
                return Err(BlockError::Unsupported(
                    "USB root current grant has a foreign protocol/route",
                ));
            }
            root_ma
        };
        let previous = self
            .entries
            .get_mut(usize::from(slot))
            .ok_or(BlockError::Unsupported("USB current slot out of range"))?;
        if previous.is_some() {
            return Err(BlockError::Unsupported(
                "USB current grant slot already owned",
            ));
        }
        *previous = Some(Grant {
            generation: None,
            parent_id: None,
            attachment,
            maximum_ma,
            reserved_ma: unit(attachment.speed),
            configurations: Vec::new(),
            hub: None,
        });
        if let Err(error) = self.check() {
            self.entries[usize::from(slot)] = None;
            return Err(error);
        }
        Ok(())
    }
    fn descriptions(&mut self, slot: u8, configurations: &[Vec<u8>]) -> Result<(), BlockError> {
        let owner = self.grant_mut(slot)?;
        if !owner.configurations.is_empty() {
            return Err(BlockError::Unsupported(
                "USB current descriptors already retained",
            ));
        }
        for config in configurations {
            if config.len() < 9
                || config[0] != 9
                || config[1] != CONFIGURATION_DESCRIPTOR
                || config[5] == 0
            {
                return Err(BlockError::Unsupported(
                    "USB current configuration descriptor",
                ));
            }
            let scale = if owner.attachment.speed >= SUPER_SPEED {
                8
            } else {
                2
            };
            owner
                .configurations
                .push((config[5], u16::from(config[8]) * scale));
        }
        Ok(())
    }
    fn demand(&self, slot: u8, value: u8) -> Result<u16, BlockError> {
        let owner = self.grant(slot)?;
        if value == 0 {
            return Ok(unit(owner.attachment.speed));
        }
        owner
            .configurations
            .iter()
            .find(|(id, _)| *id == value)
            .map(|(_, ma)| *ma)
            .ok_or(BlockError::Unsupported(
                "SET_CONFIGURATION has no retained current descriptor",
            ))
    }
    fn configuration_prepare(&mut self, slot: u8, value: u8) -> Result<u16, BlockError> {
        let demand = self.demand(slot, value)?;
        let old = self.grant(slot)?.reserved_ma;
        self.grant_mut(slot)?.reserved_ma = old.max(demand);
        if let Err(error) = self.check() {
            self.grant_mut(slot)?.reserved_ma = old;
            return Err(error);
        }
        Ok(demand)
    }
    fn configuration_ack(&mut self, slot: u8, demand: u16) -> Result<(), BlockError> {
        self.grant_mut(slot)?.reserved_ma = demand;
        Ok(())
    }
    fn hub(
        &mut self,
        slot: u8,
        ports: u8,
        controller_ma: u16,
        self_powered: bool,
    ) -> Result<(), BlockError> {
        let owner = self.grant_mut(slot)?;
        if ports == 0 || ports > MAX_HUB_PORTS || owner.hub.is_some() {
            return Err(BlockError::Unsupported(
                "USB hub current contract already owned or invalid",
            ));
        }
        if !self_powered && owner.maximum_ma <= unit(owner.attachment.speed) {
            return Err(BlockError::Unsupported(
                "consecutive bus-powered hubs exceed one-unit upstream grant",
            ));
        }
        owner.hub = Some(Hub {
            ports,
            controller_ma,
            self_powered,
            powered: 0,
        });
        if let Err(error) = self.check() {
            self.grant_mut(slot)?.hub = None;
            return Err(error);
        }
        Ok(())
    }
    fn powered_port(&mut self, slot: u8, port: u8) -> Result<(), BlockError> {
        let hub = self
            .grant_mut(slot)?
            .hub
            .as_mut()
            .ok_or(BlockError::Unsupported("USB hub power contract absent"))?;
        if port == 0 || port > hub.ports {
            return Err(BlockError::Unsupported("USB hub power port out of range"));
        }
        let previous = hub.powered;
        hub.powered |= 1 << port;
        if let Err(error) = self.check() {
            self.grant_mut(slot)?.hub.as_mut().unwrap().powered = previous;
            return Err(error);
        }
        Ok(())
    }
    pub(super) fn recovery_power(&self, slot: u8, port: u8) -> Result<(), BlockError> {
        if self.powered_ports(slot)? & (1 << port) == 0 {
            return Err(BlockError::Unsupported(
                "USB recovery port has no current reservation",
            ));
        }
        self.check()
    }
    pub(super) fn powered_ports(&self, slot: u8) -> Result<u32, BlockError> {
        self.grant(slot)?
            .hub
            .as_ref()
            .map(|hub| hub.powered)
            .ok_or(BlockError::Unsupported("USB hub power contract absent"))
    }
    pub(super) fn hub_is_self_powered(&self, slot: u8) -> Result<bool, BlockError> {
        self.grant(slot)?
            .hub
            .as_ref()
            .map(|hub| hub.self_powered)
            .ok_or(BlockError::Unsupported("USB hub power contract absent"))
    }
    pub(super) fn belongs_to(&self, mut slot: u8, ancestor: u8) -> bool {
        for _ in 0..=MAX_TIERS {
            if slot == ancestor {
                return true;
            }
            let Ok(Some(parent)) = self.original_parent(slot) else {
                return false;
            };
            slot = parent;
        }
        false
    }
    pub(super) fn port_owner(&self, parent: Option<(u8, u8)>, root: u8) -> Option<u8> {
        self.entries.iter().enumerate().find_map(|(slot, grant)| {
            grant
                .as_ref()
                .filter(|grant| {
                    grant.attachment.parent == parent && grant.attachment.root_port == root
                })
                .map(|_| slot as u8)
        })
    }
    pub(super) fn hub_ports(&self, slot: u8) -> Option<u8> {
        self.grant(slot).ok()?.hub.as_ref().map(|hub| hub.ports)
    }
    fn reset_port(&mut self, parent: Option<(u8, u8)>, root: u8) {
        let roots: Vec<u8> = self
            .entries
            .iter()
            .enumerate()
            .filter_map(|(slot, grant)| {
                grant
                    .as_ref()
                    .filter(|grant| {
                        grant.attachment.parent == parent && grant.attachment.root_port == root
                    })
                    .map(|_| slot as u8)
            })
            .collect();
        let retired: Vec<usize> = self
            .entries
            .iter()
            .enumerate()
            .filter_map(|(slot, _)| {
                roots
                    .iter()
                    .any(|ancestor| self.belongs_to(slot as u8, *ancestor))
                    .then_some(slot)
            })
            .collect();
        for slot in retired {
            self.entries[slot] = None;
        }
    }
}

impl Controller {
    pub(super) fn current_port_owner(
        &self,
        parent: Option<(SlotId, u8)>,
        root: u8,
    ) -> Result<Option<SlotId>, BlockError> {
        if let Some((id, _)) = parent {
            self.validate_topology_id(id)?;
        }
        let raw = parent.map(|(id, port)| (id.hardware_slot(), port));
        let Some(slot) = self.power.port_owner(raw, root) else {
            return Ok(None);
        };
        // Permission is the old grant, never a fresh lookup from a
        // numerical slot. A changed generation refuses the operation.
        let grant = self.power.grant(slot)?;
        let state = self.slots.get(usize::from(slot)).and_then(Option::as_ref);
        let Some(state) = state else {
            // A root port's device that was refused or retired while it
            // stayed connected keeps its grant until the port is reset or
            // empties, and has no slot left: the port has no owner to
            // retire, and its status is still read, so a disconnection is
            // seen. Linux leaves such a port the same way after "unable to
            // enumerate USB device". Under a hub the owner's record is
            // required.
            return if parent.is_none() {
                Ok(None)
            } else {
                Err(BlockError::Unsupported(
                    "USB current owner has no original slot metadata",
                ))
            };
        };
        if grant.generation != Some(state.generation) || grant.parent_id != parent.map(|(id, _)| id)
        {
            return Err(BlockError::Unsupported(
                "USB current owner generation changed",
            ));
        }
        let id =
            state
                .inventory
                .as_ref()
                .map(|device| device.id)
                .ok_or(BlockError::Unsupported(
                    "USB current owner inventory unresolved",
                ))?;
        self.validate_ancestry(id)?;
        Ok(Some(id))
    }

    pub(super) fn observe_current_ancestors_for(
        &mut self,
        id: SlotId,
        include_hub: bool,
    ) -> Result<(), BlockError> {
        block_on(Self::observe_current_ancestors_for_with(
            &Access::blocking(self, None),
            id,
            include_hub,
        ))
    }

    pub(super) async fn observe_current_ancestors_for_with(
        a: &Access<'_>,
        id: SlotId,
        include_hub: bool,
    ) -> Result<(), BlockError> {
        a.with(|c| c.validate_ancestry(id))?;
        Self::observe_current_ancestors_with(a, id.hardware_slot(), include_hub).await
    }

    pub(super) fn root_current(&mut self, port: u8, speed: u8) -> Result<u16, BlockError> {
        let major = self
            .port_major
            .get(usize::from(port.wrapping_sub(1)))
            .copied();
        let ma = match (major, speed) {
            (Some(2), LOW_SPEED | FULL_SPEED | HIGH_SPEED) => USB2_PORT_MA,
            (Some(3), SUPER_SPEED | SUPER_SPEED_PLUS) => USB3_PORT_MA,
            _ => {
                self.quarantine_root_current(port);
                return Err(BlockError::Unsupported(
                    "USB root current needs its actual supported protocol",
                ));
            }
        };
        let status = self.registers.read32(self.port_offset(port));
        if status & (PP | CCS) != PP | CCS || status & ROOT_OVER_CURRENT != 0 {
            self.quarantine_root_current(port);
            return Err(BlockError::Unsupported(
                "USB root port power absent or over-current",
            ));
        }
        let wire_speed = ((status >> PORT_SPEED_SHIFT) & PORT_SPEED_FIELD) as u8;
        if !matches!(
            (major, wire_speed),
            (Some(2), LOW_SPEED | FULL_SPEED | HIGH_SPEED)
                | (Some(3), SUPER_SPEED | SUPER_SPEED_PLUS)
        ) {
            self.quarantine_root_current(port);
            return Err(BlockError::Unsupported(
                "USB root current protocol does not match actual port speed",
            ));
        }
        Ok(ma)
    }
    fn quarantine_root_current(&mut self, port: u8) {
        for state in self.slots.iter_mut().flatten() {
            if state.attachment.root_port == port {
                state.retiring = true;
            }
        }
    }
    pub(super) fn reserve_current(
        &mut self,
        slot: u8,
        generation: u64,
        attachment: Attachment,
        parent_id: Option<SlotId>,
    ) -> Result<(), BlockError> {
        self.validate_attachment_parent(attachment, parent_id)?;
        self.validate_current_graph(attachment.root_port)?;
        let root_speed = if self
            .port_major
            .get(usize::from(attachment.root_port.wrapping_sub(1)))
            == Some(&3)
        {
            SUPER_SPEED
        } else {
            HIGH_SPEED
        };
        let root_ma = self.root_current(attachment.root_port, root_speed)?;
        if let Some((parent, _)) = attachment.parent {
            self.live_slot(parent)?;
        }
        self.power.reserve(slot, attachment, root_ma)?;
        let grant = self.power.grant_mut(slot)?;
        grant.generation = Some(generation);
        grant.parent_id = parent_id;
        Ok(())
    }
    pub(super) fn current_descriptions(
        &mut self,
        slot: u8,
        configurations: &[Vec<u8>],
    ) -> Result<(), BlockError> {
        self.power.descriptions(slot, configurations)
    }
    pub(super) async fn prepare_current_configuration_with(
        a: &Access<'_>,
        slot: u8,
        value: u8,
    ) -> Result<u16, BlockError> {
        a.with(|c| {
            // A configured hub owns its powered ports and descendant
            // contracts. Endpoint teardown alone cannot authorize replacing
            // that permission. Reconfiguration needs explicit subtree
            // retirement and a fresh hub owner; the cached same-value helper
            // remains a harmless no-op.
            if c.power.grant(slot)?.hub.is_some() {
                return Err(BlockError::Unsupported(
                    "USB hub configuration requires complete subtree retirement",
                ));
            }
            c.power.demand(slot, value)?;
            Ok(())
        })?;
        Self::observe_current_ancestors_with(a, slot, false).await?;
        a.with(|c| c.power.configuration_prepare(slot, value))
    }
    pub(super) fn observe_current_ancestors(
        &mut self,
        slot: u8,
        include_hub: bool,
    ) -> Result<(), BlockError> {
        block_on(Self::observe_current_ancestors_with(
            &Access::blocking(self, None),
            slot,
            include_hub,
        ))
    }

    pub(super) async fn observe_current_ancestors_with(
        a: &Access<'_>,
        slot: u8,
        include_hub: bool,
    ) -> Result<(), BlockError> {
        let attachment = a.with(|c| {
            c.validate_owned_slot_ancestry(slot)?;
            let attachment = c.power.grant(slot)?.attachment;
            let root_speed = if c
                .port_major
                .get(usize::from(attachment.root_port.wrapping_sub(1)))
                == Some(&3)
            {
                SUPER_SPEED
            } else {
                HIGH_SPEED
            };
            c.root_current(attachment.root_port, root_speed)?;
            Ok::<_, BlockError>(attachment)
        })?;
        if include_hub {
            Self::observe_hub_current_with(a, slot).await?;
        }
        let mut parent = attachment.parent;
        while let Some((slot, _)) = parent {
            let ancestor = a.with(|c| Ok::<_, BlockError>(c.power.grant(slot)?.attachment))?;
            // Raw GET_STATUS does not recurse into SET_CONFIGURATION accounting.
            Self::observe_hub_current_with(a, slot).await?;
            parent = ancestor.parent;
        }
        Ok(())
    }
    pub(super) async fn prepare_port_current_with(
        a: &Access<'_>,
        slot: u8,
        port: u8,
    ) -> Result<(), BlockError> {
        a.with(|c| {
            c.validate_owned_slot_ancestry(slot)?;
            let root = c.slots[usize::from(slot)]
                .as_ref()
                .unwrap()
                .attachment
                .root_port;
            c.validate_current_graph(root)?;
            // Reject pure budget/ownership failures before touching MMIO or
            // DMA. Reservation is conservative across any later uncertain
            // health/ACK.
            c.power.powered_port(slot, port)
        })?;
        Self::observe_current_ancestors_with(a, slot, true).await
    }
    pub(super) fn ack_current_configuration(
        &mut self,
        slot: u8,
        demand: u16,
    ) -> Result<(), BlockError> {
        self.power.configuration_ack(slot, demand)
    }
    pub(super) fn install_hub_current(
        &mut self,
        slot: u8,
        ports: u8,
        controller_ma: u16,
        self_powered: bool,
    ) -> Result<(), BlockError> {
        self.validate_owned_slot_ancestry(slot)?;
        self.power.hub(slot, ports, controller_ma, self_powered)
    }
    pub(super) fn reserve_port_current(&mut self, slot: u8, port: u8) -> Result<(), BlockError> {
        self.validate_owned_slot_ancestry(slot)?;
        self.power.powered_port(slot, port)
    }
    pub(super) async fn observe_hub_current_with(
        a: &Access<'_>,
        slot: u8,
    ) -> Result<(), BlockError> {
        a.with(|c| c.validate_owned_slot_ancestry(slot))?;
        let result = async {
            let bytes = a
                .control(slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                .await?;
            if bytes.len() != 2
                || (bytes[0] & 1 != 0) != a.with(|c| c.power.hub_is_self_powered(slot))?
            {
                return Err(BlockError::Unsupported("USB hub power source changed"));
            }
            for _ in 0..2 {
                let bytes = a
                    .control(slot, setup(0xa0, GET_STATUS, 0, 0, 4), &[])
                    .await?;
                if bytes.len() != 4 {
                    return Err(BlockError::Unsupported("short USB hub power status"));
                }
                let status = u16::from_le_bytes([bytes[0], bytes[1]]);
                let change = u16::from_le_bytes([bytes[2], bytes[3]]);
                if status != 0 || change & !1 != 0 {
                    return Err(BlockError::Unsupported(
                        "USB hub power/over-current authority lost",
                    ));
                }
                if change == 0 {
                    return Ok(());
                }
                a.control(slot, setup(0x20, CLEAR_FEATURE, 0, 0, 0), &[])
                    .await?;
            }
            Err(BlockError::Unsupported(
                "USB hub local power change did not clear",
            ))
        }
        .await;
        if result.is_err() {
            a.with(|c| c.quarantine_current_subtree(slot));
        }
        result
    }
    pub(super) fn quarantine_current_subtree(&mut self, ancestor: u8) {
        for slot in 1..self.slots.len() {
            if self.power.belongs_to(slot as u8, ancestor)
                && let Some(state) = self.slots[slot].as_mut()
            {
                state.retiring = true;
            }
        }
    }
    pub(super) fn current_hub_port_reset(
        &mut self,
        id: SlotId,
        port: u8,
        root: u8,
    ) -> Result<(), BlockError> {
        self.validate_topology_id(id)?;
        let state = self.slots[usize::from(id.hardware_slot())]
            .as_ref()
            .unwrap();
        if state.attachment.root_port != root {
            return Err(BlockError::Unsupported("USB reset current root changed"));
        }
        self.validate_current_graph(root)?;
        self.current_port_reset(Some((id.hardware_slot(), port)), root)?;
        Ok(())
    }

    pub(super) fn validate_current_graph(&self, root: u8) -> Result<(), BlockError> {
        // Validate all grants on this physical root, not only members accepted
        // by a boolean descendant predicate: malformed ancestry must refuse,
        // rather than disappear from validation and be released later.
        for (slot, entry) in self.power.entries.iter().enumerate() {
            let Some(grant) = entry.as_ref().filter(|g| g.attachment.root_port == root) else {
                continue;
            };
            if let Some(state) = self.slots.get(slot).and_then(Option::as_ref)
                && (grant.generation != Some(state.generation)
                    || grant.parent_id != state.parent_id
                    || grant.attachment != state.attachment)
            {
                return Err(BlockError::Unsupported(
                    "USB physical current generation changed",
                ));
            }
            let mut current = slot as u8;
            let mut complete = false;
            for _ in 0..=MAX_TIERS {
                let Some(parent) = self.power.original_parent(current)? else {
                    complete = true;
                    break;
                };
                let child = self.power.grant(current)?;
                let ancestor = self.power.grant(parent)?;
                if ancestor.attachment.root_port != root
                    || ancestor.attachment.tier.checked_add(1) != Some(child.attachment.tier)
                {
                    return Err(BlockError::Unsupported(
                        "USB physical current original ancestry changed",
                    ));
                }
                current = parent;
            }
            if !complete {
                return Err(BlockError::Unsupported(
                    "USB physical current ancestry is cyclic",
                ));
            }
        }
        Ok(())
    }

    pub(super) fn current_port_reset(
        &mut self,
        parent: Option<(u8, u8)>,
        root: u8,
    ) -> Result<(), BlockError> {
        self.validate_current_graph(root)?;
        let owners: Vec<u8> = self
            .power
            .entries
            .iter()
            .enumerate()
            .filter_map(|(slot, grant)| {
                grant
                    .as_ref()
                    .filter(|grant| {
                        grant.attachment.parent == parent && grant.attachment.root_port == root
                    })
                    .map(|_| slot as u8)
            })
            .collect();
        for slot in owners {
            self.quarantine_current_subtree(slot);
        }
        self.power.reset_port(parent, root);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saturated_route_ports_have_separate_current_owners_and_physical_reset_subtrees() {
        let root = Attachment {
            root_port: 1,
            route: 0,
            tier: 0,
            speed: HIGH_SPEED,
            translator: None,
            parent: None,
        };
        let mut ledger = Ledger::new(8);
        ledger.reserve(1, root, 500).unwrap();
        ledger.hub(1, 31, 0, true).unwrap();
        for (slot, port) in [(2, 15), (3, 16)] {
            ledger.powered_port(1, port).unwrap();
            ledger
                .reserve(
                    slot,
                    Attachment {
                        route: 15,
                        tier: 1,
                        parent: Some((1, port)),
                        ..root
                    },
                    500,
                )
                .unwrap();
        }
        assert!(
            ledger
                .reserve(
                    4,
                    Attachment {
                        route: 15,
                        tier: 1,
                        parent: Some((1, 16)),
                        ..root
                    },
                    500
                )
                .is_err()
        );
        ledger.hub(2, 1, 0, true).unwrap();
        ledger.powered_port(2, 1).unwrap();
        ledger
            .reserve(
                4,
                Attachment {
                    route: 0x1f,
                    tier: 2,
                    parent: Some((2, 1)),
                    ..root
                },
                500,
            )
            .unwrap();
        ledger.reset_port(Some((1, 15)), 1);
        assert!(!ledger.retained(2));
        assert!(!ledger.retained(4));
        assert!(ledger.retained(1));
        assert!(ledger.retained(3));
        ledger
            .reserve(
                5,
                Attachment {
                    route: 15,
                    tier: 1,
                    parent: Some((1, 15)),
                    ..root
                },
                500,
            )
            .unwrap();
    }
}
