// SPDX-License-Identifier: GPL-2.0-only
//! Exact software ownership of an acknowledged Address Device generation.
//!
//! Linux usb_disconnect detaches class consumers before xhci_free_dev disables
//! the hardware slot. A slot number alone cannot express that lifetime: the
//! next Enable Slot may return the same number. This boundary preserves old
//! consumers' permanent refusal across that hardware reuse. Linux's pinned
//! xhci_check_args also compares virt_dev->udev with the usb_device;
//! a bare numerical slot is insufficient here after retirement.
//!
//! Public raw operations check one monotonic retirement fence. Exact access
//! validates its SlotId before entering private implementations; no
//! public bypass flag or raw-to-current identity conversion is exposed.
//! This identity boundary alone does not authorize recycling published slots.
//! Exclusive root-leaf reset also requires retirement's physical-reset
//! witness; a new generation never clears the permanent raw-slot fence.
use super::*;
use core::sync::atomic::AtomicU64;

static NEXT_GENERATION: AtomicU64 = AtomicU64::new(1);
/// Check again at the first poll a USB microframe after a status sample,
/// even when the caller polls too sparsely to reach the quiet drain limit.
const TICKET_STATUS_US: u64 = 125;
/// A control transfer its owner stopped before it ended.
pub(crate) const STOPPED: &str = "USB control transfer stopped by its owner";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Consumer {
    Unclaimed,
    Native,
    Forwarded,
}

pub(super) fn next_generation() -> Result<u64, BlockError> {
    NEXT_GENERATION
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
            value.checked_add(1)
        })
        .map_err(|_| BlockError::Unsupported("xHCI device generation space exhausted"))
}

/// Unforgeable identity of one addressed device. Cloning descriptors preserves
/// the same identity; a physical reset or re-enumeration creates a fresh one.
/// Identities are unique across controllers, not just across slots in one host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotId {
    slot: u8,
    generation: u64,
}

impl SlotId {
    /// The hardware slot number is diagnostic data, not an access credential.
    pub fn hardware_slot(self) -> u8 {
        self.slot
    }

    pub(super) fn generation(self) -> u64 {
        self.generation
    }

    #[cfg(test)]
    pub(crate) fn fixture(slot: u8) -> Self {
        Self {
            slot,
            generation: 0,
        }
    }
}

/// The device a controller method acts on: its [`SlotId`], which names one
/// device generation and fails once that device's slot is disabled and handed
/// to another, as a freed `usb_device` would. Tests may also name a slot's
/// current device by number.
pub trait DeviceRef: Copy {
    #[doc(hidden)]
    fn current_slot(self, controller: &Controller) -> Result<u8, BlockError>;
}
impl DeviceRef for SlotId {
    fn current_slot(self, controller: &Controller) -> Result<u8, BlockError> {
        controller.current_slot(self)
    }
}
/// A test's bare number names the slot's first device only: once disabled
/// it stays refused, so a stale number cannot reach the slot's next device.
#[cfg(test)]
impl DeviceRef for u8 {
    fn current_slot(self, controller: &Controller) -> Result<u8, BlockError> {
        if controller.test_retired_numbers.get(usize::from(self)) != Some(&false) {
            return Err(BlockError::Unsupported(
                "raw xHCI slot authority is permanently retired",
            ));
        }
        controller.live_slot(self)?;
        Ok(self)
    }
}

impl Controller {
    /// The slot of `id` while that exact device generation holds it. Pure:
    /// no readiness read, event drain, descriptor access, endpoint index or
    /// DMA mutation precedes it, so an identity whose slot was disabled and
    /// handed to a later device fails here.
    pub(super) fn current_slot(&self, id: SlotId) -> Result<u8, BlockError> {
        if !self.known_id(id) {
            return Err(BlockError::Unsupported(
                "xHCI device generation is permanently retired",
            ));
        }
        self.live_slot(id.slot)?;
        Ok(id.slot)
    }

    #[cfg(test)]
    pub(crate) fn test_device_id(&self, slot: u8) -> Result<SlotId, BlockError> {
        self.device_id(slot)
    }

    pub(super) fn device_id(&self, slot: u8) -> Result<SlotId, BlockError> {
        self.live_slot(slot)?;
        let state = self.slots[usize::from(slot)].as_ref().unwrap();
        Ok(SlotId {
            slot,
            generation: state.generation,
        })
    }

    pub(super) fn validate_id(&self, id: SlotId) -> Result<u8, BlockError> {
        // Reject the consumer's own retired identity before walking the current
        // replacement's topology. This pure, permanent refusal must also win
        // over a suspended replacement and any absent or foreign parent grant.
        if !self.known_id(id) {
            return Err(BlockError::Unsupported(
                "xHCI device generation is permanently retired",
            ));
        }
        self.validate_ancestry(id)?;
        self.live_slot(id.slot)?;
        if self.slots[usize::from(id.slot)].as_ref().unwrap().suspended {
            return Err(BlockError::Unsupported(
                "USB consumer is fenced for device lifecycle",
            ));
        }
        Ok(id.slot)
    }

    pub(super) fn known_id(&self, id: SlotId) -> bool {
        self.slots
            .get(usize::from(id.slot))
            .and_then(Option::as_ref)
            .is_some_and(|state| state.generation == id.generation)
    }

    /// Validate the identities stored at enumeration, including the
    /// original parent objects. This pure check also works for a disabled child
    /// whose physical current grant must outlive Disable Slot. Suspension is
    /// allowed for private topology sequencing, never for class access.
    pub(super) fn validate_ancestry(&self, id: SlotId) -> Result<(), BlockError> {
        let mut current = id;
        for depth in 0..=MAX_TIERS {
            if !self.known_id(current) {
                return Err(BlockError::Unsupported(
                    "USB original topology generation retired",
                ));
            }
            let state = self.slots[usize::from(current.slot)].as_ref().unwrap();
            self.power
                .validate_generation(current, state.parent_id, state.attachment)?;
            match (state.attachment.parent, state.parent_id) {
                (None, None) if state.attachment.tier == 0 && state.attachment.route == 0 => {
                    return Ok(());
                }
                (Some((slot, port)), Some(parent))
                    if slot == parent.slot && port != 0 && depth < MAX_TIERS =>
                {
                    if !self.known_id(parent)
                        || self.slot_ownership.get(usize::from(slot))
                            != Some(&reset::Ownership::Enabled)
                    {
                        return Err(BlockError::Unsupported(
                            "USB original parent generation retired",
                        ));
                    }
                    let ancestor = self.slots[usize::from(slot)].as_ref().unwrap();
                    let translator = match (state.attachment.speed, ancestor.attachment.speed) {
                        (LOW_SPEED | FULL_SPEED, HIGH_SPEED) => Some((slot, port)),
                        _ => ancestor.attachment.translator,
                    };
                    if ancestor.retiring
                        || state.attachment.translator != translator
                        || ancestor.attachment.root_port != state.attachment.root_port
                        || ancestor.attachment.tier.checked_add(1) != Some(state.attachment.tier)
                        || enumerate::child_route(
                            ancestor.attachment.route,
                            ancestor.attachment.tier,
                            port,
                        ) != state.attachment.route
                    {
                        return Err(BlockError::Unsupported(
                            "USB original parent attachment changed",
                        ));
                    }
                    current = parent;
                }
                _ => {
                    return Err(BlockError::Unsupported(
                        "USB original parent identity absent",
                    ));
                }
            }
        }
        Err(BlockError::Unsupported(
            "USB original topology exceeds depth",
        ))
    }

    /// Private owned command paths may act before descriptor inventory exists,
    /// e.g. failed EP0 enumeration retirement. Check their stored graph without
    /// issuing a new credential to any raw-slot caller.
    pub(super) fn validate_owned_slot_ancestry(&self, slot: u8) -> Result<(), BlockError> {
        let state = self
            .slots
            .get(usize::from(slot))
            .and_then(Option::as_ref)
            .ok_or(BlockError::Unsupported("USB owned topology slot absent"))?;
        self.validate_ancestry(SlotId {
            slot,
            generation: state.generation,
        })
    }

    pub(super) fn validate_topology_id(&self, id: SlotId) -> Result<u8, BlockError> {
        self.validate_ancestry(id)?;
        self.live_slot(id.slot)?;
        Ok(id.slot)
    }

    pub(super) fn original_parent(&self, id: SlotId) -> Result<Option<SlotId>, BlockError> {
        self.validate_ancestry(id)?;
        Ok(self.slots[usize::from(id.slot)].as_ref().unwrap().parent_id)
    }

    pub(super) fn validate_attachment_parent(
        &self,
        attachment: Attachment,
        parent: Option<SlotId>,
    ) -> Result<(), BlockError> {
        match (attachment.parent, parent) {
            (None, None)
                if attachment.tier == 0
                    && attachment.route == 0
                    && attachment.translator.is_none() =>
            {
                Ok(())
            }
            (Some((slot, port)), Some(id)) if slot == id.slot && port != 0 => {
                self.validate_topology_id(id)?;
                let ancestor = self.slots[usize::from(slot)].as_ref().unwrap();
                let translator = match (attachment.speed, ancestor.attachment.speed) {
                    (LOW_SPEED | FULL_SPEED, HIGH_SPEED) => Some((slot, port)),
                    _ => ancestor.attachment.translator,
                };
                if attachment.translator != translator
                    || ancestor.attachment.root_port != attachment.root_port
                    || ancestor.attachment.tier.checked_add(1) != Some(attachment.tier)
                    || enumerate::child_route(
                        ancestor.attachment.route,
                        ancestor.attachment.tier,
                        port,
                    ) != attachment.route
                    || self.power.hub_ports(slot).is_none_or(|ports| port > ports)
                {
                    return Err(BlockError::Unsupported(
                        "USB new attachment has a foreign original parent",
                    ));
                }
                Ok(())
            }
            _ => Err(BlockError::Unsupported(
                "USB enumeration requires its original typed parent",
            )),
        }
    }

    /// The private hub recovery capability keeps a quarantined
    /// generation. It never grants ordinary class access or matches a reuse.
    pub(super) fn validate_recovery_id(&self, id: SlotId) -> Result<(), BlockError> {
        self.validate_ancestry(id)?;
        if self.slot_ownership.get(usize::from(id.slot)) != Some(&reset::Ownership::Enabled)
            || self
                .slots
                .get(usize::from(id.slot))
                .and_then(Option::as_ref)
                .is_none_or(|state| state.generation != id.generation || !state.retiring)
        {
            return Err(BlockError::Unsupported(
                "USB recovery generation is permanently retired",
            ));
        }
        Ok(())
    }

    /// Pure admission for a native consumer, including its original
    /// ancestor generations. No register, completion or DMA access occurs.
    pub(crate) fn validate_device_id(&self, id: SlotId) -> Result<(), BlockError> {
        self.validate_id(id).map(|_| ())
    }

    /// Borrow operations for this exact device. Validation is pure and occurs
    /// before any MMIO, DMA, completion drain or endpoint mutation. The mutable
    /// controller borrow prevents topology mutation while this access is live.
    pub fn device(&mut self, id: SlotId) -> Result<DeviceAccess<'_>, BlockError> {
        self.validate_id(id)?;
        Ok(DeviceAccess {
            controller: self,
            id,
        })
    }

    /// Bind dispatch takes the whole composite device before the first class
    /// SET_CONFIGURATION or vendor initialization. A failed bind keeps its claim.
    pub(super) fn claim_native(&mut self, id: SlotId) -> Result<(), BlockError> {
        let slot = self.validate_id(id)?;
        let state = self.slot(slot)?;
        if state.consumer != Consumer::Unclaimed {
            return Err(BlockError::Unsupported("USB device already has a consumer"));
        }
        state.consumer = Consumer::Native;
        Ok(())
    }

    pub(super) fn claim_forwarding(&mut self, id: SlotId) -> Result<(), BlockError> {
        let slot = self.validate_id(id)?;
        let state = self.slot(slot)?;
        if state.consumer != Consumer::Unclaimed
            || state.current_configuration != Some(0)
            || state.endpoints.iter().skip(2).any(Option::is_some)
        {
            return Err(BlockError::Unsupported(
                "USB forwarding requires an unclaimed Address-state device",
            ));
        }
        state.consumer = Consumer::Forwarded;
        Ok(())
    }
}

/// Device operations whose slot permission comes from physical enumeration.
/// There is no Deref to the raw controller or slot setter.
pub struct DeviceAccess<'a> {
    controller: &'a mut Controller,
    id: SlotId,
}

impl DeviceAccess<'_> {
    fn slot(&self) -> Result<u8, BlockError> {
        self.controller.validate_id(self.id)
    }

    pub fn control(&mut self, request: [u8; 8], out: &[u8]) -> Result<Vec<u8>, BlockError> {
        let slot = self.slot()?;
        self.controller.control_owned(slot, request, out)
    }

    /// Read a standard string descriptor for this exact addressed generation.
    pub(crate) fn string(&mut self, language: u16, index: u8) -> Option<String> {
        let slot = self.slot().ok()?;
        self.controller.string_owned(slot, language, index)
    }

    pub fn connected(&mut self) -> Result<bool, BlockError> {
        let slot = self.slot()?;
        self.controller.connected_owned(slot)
    }

    pub fn endpoint_ready(&mut self, dci: u8) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.endpoint_ready_owned(slot, dci)
    }

    /// Check all the endpoints a class operation may use before changing its
    /// buffers, with one controller readiness check.
    pub(crate) fn endpoints_ready(&mut self, dcis: &[u8]) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.endpoints_ready_owned(slot, dcis)
    }

    pub fn current_configuration(&mut self) -> Result<Option<u8>, BlockError> {
        let slot = self.slot()?;
        self.controller.current_configuration_owned(slot)
    }

    pub fn set_configuration(&mut self, value: u8) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.set_configuration_owned(slot, value)
    }

    pub fn configure(
        &mut self,
        endpoints: &[Endpoint],
        hub: Option<(u8, u8, bool)>,
        dma: &mut dyn DmaAllocator,
    ) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.configure_owned(slot, endpoints, hub, dma)
    }

    pub fn preflight_endpoints(&mut self, endpoints: &[Endpoint]) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.preflight_endpoints_owned(slot, endpoints)
    }

    pub fn deconfigure(&mut self, endpoints: &[Endpoint]) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.deconfigure_owned(slot, endpoints)
    }

    pub fn configure_streams(
        &mut self,
        endpoints: &[Endpoint],
        stream_endpoints: &[u8],
        requested: u16,
        dma: &mut dyn DmaAllocator,
    ) -> Result<u16, BlockError> {
        let slot = self.slot()?;
        self.controller
            .configure_streams_owned(slot, endpoints, stream_endpoints, requested, dma)
    }

    pub(crate) fn stream_capacity(
        &self,
        endpoints: &[Endpoint],
        selected: &[u8],
        requested: u16,
    ) -> Result<u16, BlockError> {
        self.controller
            .stream_capacity_for_device(self.id, endpoints, selected, requested)
    }

    pub fn submit(&mut self, dci: u8, buffer: u64, length: usize) -> Result<u64, BlockError> {
        let slot = self.slot()?;
        self.controller.submit_owned(slot, dci, buffer, length)
    }

    pub fn submit_ticket(
        &mut self,
        dci: u8,
        buffer: u64,
        length: usize,
    ) -> Result<TransferTicket, BlockError> {
        let slot = self.slot()?;
        self.controller
            .submit_ticket_owned(slot, dci, buffer, length)
    }

    pub fn completed_ticket(
        &mut self,
        ticket: &TransferTicket,
    ) -> Option<Result<usize, BlockError>> {
        if let Err(error) = self.slot() {
            return Some(Err(error));
        }
        if ticket.device_id() != self.id {
            return Some(Err(BlockError::Unsupported("foreign USB transfer ticket")));
        }
        self.controller.completed_ticket(ticket)
    }

    /// Poll a class transfer and report expiry only after checking for a
    /// completion or controller failure.
    pub(crate) fn poll_ticket_until(
        &mut self,
        ticket: &TransferTicket,
        deadline: &Deadline,
        status_due: &mut Option<Deadline>,
    ) -> Option<Result<usize, BlockError>> {
        if self.id == ticket.device_id()
            && let Some(sample) = status_due.as_ref()
            && self.controller.ticket_quiet(ticket)
        {
            let now = Clock::ticks();
            if !sample.passed_at(now) && !deadline.passed_at(now) {
                return None;
            }
        }
        // Start the interval before checking USBSTS so a slow register read
        // cannot extend it. An invalid clock keeps the mandatory check.
        let next_status = self.controller.clock.try_deadline(TICKET_STATUS_US).ok();
        let result = self
            .completed_ticket(ticket)
            .or_else(|| deadline.passed().then_some(Err(BlockError::Timeout)));
        if result.is_none() {
            *status_due = next_status;
        }
        result
    }

    pub fn cancel_ticket(
        &mut self,
        ticket: &TransferTicket,
    ) -> Result<TransferCancellation, BlockError> {
        self.slot()?;
        if ticket.device_id() != self.id {
            return Err(BlockError::Unsupported(
                "foreign USB transfer cancellation ticket",
            ));
        }
        self.controller.cancel_ticket(ticket)
    }

    pub fn completed(
        &mut self,
        dci: u8,
        trb: u64,
        length: usize,
    ) -> Option<Result<usize, BlockError>> {
        let slot = match self.slot() {
            Ok(slot) => slot,
            Err(error) => return Some(Err(error)),
        };
        self.controller.completed_owned(slot, dci, trb, length)
    }

    pub fn transfer(
        &mut self,
        dci: u8,
        buffer: u64,
        length: usize,
        microseconds: u64,
    ) -> Result<usize, BlockError> {
        let slot = self.slot()?;
        self.controller
            .transfer_owned(slot, dci, buffer, length, microseconds)
    }

    pub fn submit_stream(
        &mut self,
        dci: u8,
        stream: u16,
        buffer: u64,
        length: usize,
    ) -> Result<StreamTransfer, BlockError> {
        let slot = self.slot()?;
        self.controller
            .submit_stream_owned(slot, dci, stream, buffer, length)
    }

    /// Returns the two-TRB read limit for this configured data-in endpoint.
    /// A capacity query reserves no ring space or DMA.
    pub fn chained_stream_read_capacity(&self, dci: u8) -> Result<Option<usize>, BlockError> {
        let slot = self.slot()?;
        self.controller
            .chained_stream_read_capacity_owned(slot, dci)
    }

    /// Queues a read that requires two TRBs on a qualified boot DMA endpoint.
    /// The caller keeps the complete buffer out of CPU use until this ticket
    /// completes or endpoint cancellation is acknowledged.
    pub fn submit_chained_stream_read(
        &mut self,
        dci: u8,
        stream: u16,
        buffer: &DmaBuffer,
        length: usize,
    ) -> Result<StreamTransfer, BlockError> {
        let slot = self.slot()?;
        self.controller
            .submit_chained_stream_read_owned(slot, dci, stream, buffer, length)
    }

    pub fn completed_stream(
        &mut self,
        ticket: &StreamTransfer,
    ) -> Option<Result<usize, BlockError>> {
        if let Err(error) = self.slot() {
            return Some(Err(error));
        }
        if ticket.device_id() != self.id {
            return Some(Err(BlockError::Unsupported("foreign USB stream ticket")));
        }
        self.controller.completed_stream(ticket)
    }

    pub fn submit_iso(
        &mut self,
        dci: u8,
        buffer: u64,
        length: usize,
    ) -> Result<IsoTransfer, BlockError> {
        let slot = self.slot()?;
        self.controller.submit_iso_owned(slot, dci, buffer, length)
    }

    /// Preflight scheduling requests for this exact addressed generation.
    /// Returned values reserve nothing and every submission revalidates native
    /// identity, endpoint state, controller time and scheduling constraints.
    pub fn iso_schedule_list(
        &mut self,
        dci: u8,
        schedule: IsoSchedule,
        packet_count: usize,
    ) -> Result<Vec<IsoSchedule>, BlockError> {
        let slot = self.slot()?;
        self.controller
            .iso_schedule_list_owned(slot, dci, schedule, packet_count)
    }

    /// Submit a scheduling request for this exact addressed generation. The
    /// request carries no hardware permission; controller admission follows
    /// identity validation and precedes DMA publication.
    pub fn submit_iso_scheduled(
        &mut self,
        dci: u8,
        buffer: u64,
        length: usize,
        schedule: IsoSchedule,
    ) -> Result<IsoTransfer, BlockError> {
        let slot = self.slot()?;
        self.controller
            .submit_iso_scheduled_owned(slot, dci, buffer, length, schedule)
    }

    pub fn completed_iso(
        &mut self,
        ticket: &IsoTransfer,
    ) -> Option<Result<IsoCompletion, BlockError>> {
        if let Err(error) = self.slot() {
            return Some(Err(error));
        }
        if ticket.device_id() != self.id {
            return Some(Err(BlockError::Unsupported(
                "foreign USB isochronous ticket",
            )));
        }
        self.controller.completed_iso(ticket)
    }

    pub fn cancel(&mut self, dci: u8) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.cancel_owned(slot, dci)
    }

    pub fn reset_endpoint(&mut self, endpoint: &Endpoint) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.reset_endpoint_owned(slot, endpoint)
    }

    pub fn clear(&mut self, endpoint: &Endpoint) -> Result<(), BlockError> {
        let slot = self.slot()?;
        self.controller.clear_owned(slot, endpoint)
    }

    /// Start [`Self::control`]; [`Self::poll`] gives its result.
    pub fn start_control(&mut self, request: [u8; 8], out: &[u8]) -> Result<Operation, BlockError> {
        let slot = self.slot()?;
        let control = self.controller.start_control_owned(slot, request, out)?;
        Ok(self.operation(transfer::Kind::Control(control)))
    }

    /// Start [`Self::cancel`]; [`Self::poll`] gives its result.
    pub fn start_cancel(&mut self, dci: u8) -> Result<Operation, BlockError> {
        let slot = self.slot()?;
        self.controller.live_slot(slot)?;
        let cancel = self.controller.start_cancel(slot, dci)?;
        Ok(self.operation(transfer::Kind::Cancel(cancel)))
    }

    /// Start [`Self::clear`]; [`Self::poll`] gives its result.
    pub fn start_clear(&mut self, endpoint: &Endpoint) -> Result<Operation, BlockError> {
        let slot = self.slot()?;
        let (cancel, request) = self.controller.start_clear_owned(slot, endpoint)?;
        Ok(self.operation(transfer::Kind::Clear(cancel, request)))
    }

    fn operation(&self, kind: transfer::Kind) -> Operation {
        Operation { id: self.id, kind }
    }

    /// A started operation's result once it ended, a control transfer's IN
    /// data or else empty; never waits.
    pub fn poll(&mut self, operation: &mut Operation) -> Option<Result<Vec<u8>, BlockError>> {
        if operation.id != self.id {
            return Some(Err(BlockError::Unsupported("foreign USB device operation")));
        }
        if let Err(error) = self.slot() {
            return Some(Err(error));
        }
        self.controller.poll_operation(operation)
    }

    /// [`Self::poll`] for an owner that stops: a control transfer under way
    /// is cancelled on the host side and ends as stopped once that is
    /// acknowledged, never waiting out its own deadline.
    pub fn abort(&mut self, operation: &mut Operation) -> Option<Result<Vec<u8>, BlockError>> {
        if operation.id != self.id {
            return Some(Err(BlockError::Unsupported("foreign USB device operation")));
        }
        if let Err(error) = self.slot() {
            return Some(Err(error));
        }
        self.controller
            .abort_operation(operation, BlockError::Unsupported(STOPPED))
    }

    /// Quarantine this generation without asserting that DMA has stopped.
    pub(crate) fn quarantine(&mut self) {
        if let Ok(slot) = self.slot() {
            self.controller.quarantine_slot_owned(slot);
        }
    }
}

#[cfg(test)]
pub(super) mod tests;
