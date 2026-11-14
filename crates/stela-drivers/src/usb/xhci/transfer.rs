//! Control, bulk and interrupt transfers on kept rings, and the
//! cancellation that must be acknowledged before a buffer is reused.
use super::*;
mod operation;
mod ordinary;
mod queue;
pub use operation::Operation;
pub(super) use operation::{Cancel, Control, Kind};
pub use ordinary::{TransferCancellation, TransferTicket};
const BABBLE_DETECTED: u32 = 3;
pub(super) const USB_TRANSACTION_ERROR: u32 = 4;
/// Reset Endpoint's Transfer State Preserve flag.
pub(super) const TSP: u32 = 1 << 9;
/// Slot context tt_info's TT Hub Slot ID.
const TT_SLOT: u32 = 0xff;
/// Linux MAX_SOFT_RETRY.
pub(super) const MAX_SOFT_RETRY: u32 = 3;
const SPLIT_TRANSACTION_ERROR: u32 = 36;
const STOPPED_TRANSFER: u32 = 26;
const STOPPED_SHORT_PACKET: u32 = 28;
const INCOMPATIBLE_DEVICE: u32 = 22;
const EVENT_DATA: u32 = 1 << 2;
pub(super) use queue::{Failed, Queue, Rings};

/// Linux process_bulk_intr_td's test for a transaction error: never retried
/// with XHCI_NO_SOFT_RETRY (err_count untouched); otherwise counted, and
/// retried in place while within MAX_SOFT_RETRY on a slot not behind a TT.
pub(super) fn retry_in_place(errors: &mut u32, no_soft_retry: bool, tt_info: u32) -> bool {
    if no_soft_retry {
        return false;
    }
    let previous = *errors;
    *errors = previous.saturating_add(1);
    previous <= MAX_SOFT_RETRY && tt_info & TT_SLOT == 0
}

impl Controller {
    /// Linux xhci_handle_halted_endpoint: xhci_reset_halted_ep queues Reset
    /// Endpoint, with TSP for EP_SOFT_RESET to keep the halted TD in place,
    /// and its completion rings the pending rings again
    /// (ring_doorbell_for_active_rings). As Linux queues the command and
    /// handles its completion apart, it is started here and stepped by the
    /// owner's polls ([`Self::step_recovery_reset`]), under the command's
    /// own deadline and timeout handling, never waited for in one call.
    /// `None` while it runs.
    pub(super) fn begin_recovery_reset(
        &mut self,
        slot: u8,
        dci: u8,
        flags: u32,
        rings: Rings,
        failed: Failed,
    ) -> Option<Result<(), BlockError>> {
        let reset = queue::Reset {
            command: None,
            flags,
            rings,
            failed,
        };
        match self.recovery_queue(slot, dci) {
            Some(queue) => queue.begin_reset(reset),
            None => return Some(Err(BlockError::Unsupported("xHCI endpoint not configured"))),
        }
        self.step_recovery_reset(slot, dci)
    }

    /// One step of the endpoint's recovery Reset Endpoint: `Some(Ok)` once
    /// none runs, `None` while it does. Like xhci_handle_cmd_reset_ep, the
    /// completion code is ignored; only a command the controller never
    /// completed is an error, left where the reset's `failed` says.
    pub(super) fn step_recovery_reset(
        &mut self,
        slot: u8,
        dci: u8,
    ) -> Option<Result<(), BlockError>> {
        let Some(&mut queue::Reset {
            mut command, flags, ..
        }) = self.recovery_queue(slot, dci).and_then(Queue::reset)
        else {
            return Some(Ok(()));
        };
        let target = u32::from(slot) << 24 | u32::from(dci) << 16;
        let result = self.step_command(&mut command, 0, 0, RESET_ENDPOINT << 10 | flags | target);
        // An endpoint that went away meanwhile has no reset left to run.
        let Some(queue) = self.recovery_queue(slot, dci) else {
            return Some(Ok(()));
        };
        let Some(result) = result else {
            if let Some(reset) = queue.reset() {
                reset.command = command;
            }
            return None;
        };
        let Some(reset) = queue.take_reset() else {
            return Some(Ok(()));
        };
        match result {
            Ok(_) => {}
            Err(BlockError::Device { .. }) if !self.command_poisoned => {}
            Err(error) => {
                self.recovery_failed(slot, dci, reset.failed, &error);
                return Some(Err(error));
            }
        }
        let doorbells = match reset.rings {
            Rings::Endpoint => vec![u32::from(dci)],
            Rings::Streams => self
                .slot(slot)
                .ok()
                .and_then(|state| state.endpoints[usize::from(dci)].as_ref())
                .and_then(|endpoint| endpoint.streams.as_ref())
                .map_or_else(Vec::new, |streams| streams.active_doorbells(dci)),
            Rings::None => Vec::new(),
        };
        // The doorbells below may restart the endpoint.
        self.rang(slot, dci);
        fence(Ordering::SeqCst);
        for value in doorbells {
            self.registers
                .write32(self.doorbells + 4 * usize::from(slot), value);
        }
        Some(Ok(()))
    }

    fn recovery_queue(&mut self, slot: u8, dci: u8) -> Option<&mut Queue> {
        let endpoint = self.slot(slot).ok()?.endpoints.get_mut(usize::from(dci))?;
        Some(&mut endpoint.as_mut()?.ring.normal)
    }

    fn recovery_failed(&mut self, slot: u8, dci: u8, failed: Failed, error: &BlockError) {
        let Some(endpoint) = self
            .slot(slot)
            .ok()
            .and_then(|state| state.endpoints[usize::from(dci)].as_mut())
        else {
            return;
        };
        match failed {
            Failed::Td(event) => {
                let _ = endpoint.ring.normal.event(event);
            }
            Failed::Stream(stream) => {
                if let Some(streams) = &mut endpoint.streams {
                    streams.reset_failed(stream, error);
                }
            }
            Failed::Endpoint => {}
        }
    }

    /// Recovers a halted endpoint: Reset Endpoint, then its dequeue pointer
    /// moved past the failed TD (4.6.8, 4.6.10).
    pub(super) fn recover(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.cancel_owned(slot, dci)
    }

    /// Check controller/endpoint poison before modifying a buffer used by a
    /// previous failed transfer. This does not claim completion of a currently
    /// outstanding TD; its normal completion/cancellation must still be awaited.
    pub fn endpoint_ready(&mut self, id: impl DeviceRef, dci: u8) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.endpoint_ready_owned(slot, dci)
    }

    pub(super) fn endpoint_ready_owned(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.endpoints_ready_owned(slot, &[dci])
    }

    pub(super) fn endpoints_ready_owned(
        &mut self,
        slot: u8,
        dcis: &[u8],
    ) -> Result<(), BlockError> {
        self.live_slot(slot)?;
        self.ready()?;
        for &dci in dcis {
            self.endpoint_enabled(slot, dci)?;
            self.endpoint_can_submit(slot, dci)?;
        }
        Ok(())
    }

    fn endpoint_can_submit(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        // Incompatible Device may be transferless; its pointer need not name
        // any ticket. It requires slot retirement before new DMA publication.
        let incompatible = self.pending.iter().any(|event| {
            event.kind() == TRANSFER_EVENT
                && event.slot() == slot
                && event.endpoint() == dci
                && event.code() == INCOMPATIBLE_DEVICE
        });
        let endpoint = self.slot(slot)?.endpoints[usize::from(dci)]
            .as_mut()
            .unwrap();
        if incompatible {
            endpoint.poisoned = true;
            return Err(BlockError::Device {
                status: INCOMPATIBLE_DEVICE,
            });
        }
        endpoint.ring.normal.readiness()?;
        if endpoint
            .streams
            .as_ref()
            .is_some_and(streams::Streams::requires_cancellation)
        {
            return Err(BlockError::Unsupported(
                "stream endpoint requires cancellation",
            ));
        }
        if endpoint
            .iso
            .as_ref()
            .is_some_and(iso::Queue::requires_cancellation)
        {
            return Err(BlockError::Unsupported(
                "isochronous endpoint requires cancellation",
            ));
        }
        Ok(())
    }

    /// Completion polling may inspect an errored TD while its buffer remains
    /// fenced against resubmission. Cancellation bypasses this
    /// submission fence and establishes retirement through commands.
    pub(super) fn completion_ready(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.live_slot(slot)?;
        self.endpoint_ready_inner(slot, dci)
    }

    /// [`Self::completion_ready`] for an owner that polls its endpoint
    /// between other work, as the input service polls a HID endpoint at
    /// every turn: fatal status comes from the event drain alone
    /// ([`Controller::polled_ready`]).
    pub(super) fn polled_completion_ready(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.live_slot(slot)?;
        self.polled_ready()?;
        self.endpoint_enabled(slot, dci)
    }

    fn endpoint_ready_inner(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.ready()?;
        self.endpoint_enabled(slot, dci)
    }

    fn endpoint_enabled(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        let state = self.slot(slot)?;
        if dci != 1 && !state.current_configuration.is_some_and(|value| value != 0) {
            return Err(BlockError::Unsupported(
                "USB device configuration is not acknowledged",
            ));
        }
        let endpoint = state
            .endpoints
            .get(usize::from(dci))
            .and_then(Option::as_ref)
            .ok_or(BlockError::Unsupported("xHCI endpoint not configured"))?;
        if endpoint.poisoned {
            return Err(BlockError::Unsupported("xHCI endpoint cancellation failed"));
        }
        if !endpoint.enabled {
            return Err(BlockError::Unsupported(
                "xHCI endpoint configuration is not acknowledged",
            ));
        }
        Ok(())
    }

    /// One control transfer on EP0; IN data is returned, OUT data sent.
    pub fn control(
        &mut self,
        id: impl DeviceRef,
        request: [u8; 8],
        out: &[u8],
    ) -> Result<Vec<u8>, BlockError> {
        let slot = id.current_slot(self)?;
        self.control_owned(slot, request, out)
    }

    pub(super) fn control_owned(
        &mut self,
        slot: u8,
        request: [u8; 8],
        out: &[u8],
    ) -> Result<Vec<u8>, BlockError> {
        let control = self.start_control_owned(slot, request, out)?;
        self.run_control(control)
    }

    pub(super) fn start_control_owned(
        &mut self,
        slot: u8,
        request: [u8; 8],
        out: &[u8],
    ) -> Result<Control, BlockError> {
        block_on(Self::start_control_owned_with(
            &Access::blocking(self, None),
            slot,
            request,
            out,
        ))
    }

    /// [`Self::start_control_owned`], its hub status reads for the current
    /// a PORT_POWER or SET_CONFIGURATION request reserves made through `a`.
    pub(super) async fn start_control_owned_with(
        a: &Access<'_>,
        slot: u8,
        request: [u8; 8],
        out: &[u8],
    ) -> Result<Control, BlockError> {
        a.with(|c| c.live_slot(slot))?;
        // Any raw class requester must acquire the same physical-port current
        // reservation as enumeration before publishing a PORT_POWER request.
        if request[0] == 0x23 && request[2..4] == PORT_POWER.to_le_bytes() {
            if request[1] == CLEAR_FEATURE {
                return Err(BlockError::Unsupported(
                    "USB hub port power removal requires subtree retirement",
                ));
            }
            if request[1] == SET_FEATURE {
                if request[5..] != [0; 3] || !out.is_empty() {
                    return Err(BlockError::Unsupported(
                        "invalid USB hub PORT_POWER request",
                    ));
                }
                // Boxed: its status reads are controls of their own.
                Box::pin(Self::prepare_port_current_with(a, slot, request[4])).await?;
            }
        }
        let configuration = request[0] == 0 && request[1] == SET_CONFIGURATION;
        if configuration {
            if request[3..] != [0; 5] || !out.is_empty() {
                return Err(BlockError::Unsupported("invalid SET_CONFIGURATION request"));
            }
            if a.with(|c| {
                Ok::<_, BlockError>(c.slot(slot)?.endpoints[2..].iter().any(Option::is_some))
            })? {
                return Err(BlockError::Unsupported(
                    "USB configuration change requires endpoint teardown",
                ));
            }
        }
        // A timeout may occur after the device accepted the request; do not
        // preserve the old value or infer success from cancellation of EP0.
        a.with(|c| c.endpoint_ready_owned(slot, 1))?;
        let configuration = if configuration {
            let demand = Box::pin(Self::prepare_current_configuration_with(
                a, slot, request[2],
            ))
            .await?;
            a.with(|c| {
                c.slot(slot)?.current_configuration = None;
                Ok::<_, BlockError>(())
            })?;
            Some((request[2], demand))
        } else {
            None
        };
        a.with(|c| c.start_control(slot, request, out, configuration))
    }

    /// Only the topology owner can mint this capability, after every old alias
    /// is fenced and every descendant/status TD is retired. No public
    /// slot API can obtain access by naming this still-enabled EP0.
    pub(super) fn recovery_control(
        &mut self,
        owner: &lifecycle::Recovery,
        request: [u8; 8],
    ) -> Result<Vec<u8>, BlockError> {
        self.validate_recovery_id(owner.id())?;
        let slot = owner.slot();
        let status = request == setup(0x80, GET_STATUS, 0, 0, 2)
            || request == setup(0xa0, GET_STATUS, 0, 0, 4);
        let hub_ack = request == setup(0x20, CLEAR_FEATURE, 0, 0, 0)
            || request == setup(0x20, CLEAR_FEATURE, 1, 0, 0);
        let port = request[4];
        let port_request = port != 0
            && self
                .power
                .hub_ports(slot)
                .is_some_and(|ports| port <= ports)
            && (request == setup(0xa3, GET_STATUS, 0, u16::from(port), 4)
                || request == setup(0x23, CLEAR_FEATURE, 19, u16::from(port), 0)
                || request == setup(0x23, SET_FEATURE, PORT_POWER, u16::from(port), 0));
        if !(status || hub_ack || port_request)
            || self.slot_ownership.get(usize::from(slot)) != Some(&reset::Ownership::Enabled)
            || !self.slot(slot)?.retiring
            || (1..self.slots.len()).any(|id| {
                id != usize::from(slot)
                    && self.power.belongs_to(id as u8, slot)
                    && self.slot_ownership[id] == reset::Ownership::Enabled
            })
        {
            return Err(BlockError::Unsupported(
                "invalid quiesced USB hub control authority",
            ));
        }
        if request == setup(0x23, SET_FEATURE, PORT_POWER, u16::from(port), 0) {
            // Preserve all old downstream grants. Only an already reserved
            // physical port may be repowered before the upstream reset ACK.
            self.power.recovery_power(slot, port)?;
        }
        let control = self.start_control(slot, request, &[], None)?;
        self.run_control(control)
    }

    pub(super) async fn descriptor_with(
        a: &Access<'_>,
        slot: u8,
        kind: u8,
        index: u8,
        length: u16,
    ) -> Result<Vec<u8>, BlockError> {
        let value = u16::from(kind) << 8 | u16::from(index);
        a.control(
            slot,
            setup(DEVICE_TO_HOST, GET_DESCRIPTOR, value, 0, length),
            &[],
        )
        .await
    }

    /// Linux `usb_get_langid`: the first language string descriptor 0 lists,
    /// English when it lists none, and no strings when it cannot be read.
    #[cfg(test)]
    pub(super) fn language(&mut self, slot: u8) -> Option<u16> {
        block_on(Self::language_with(&Access::blocking(self, None), slot))
    }

    pub(super) async fn language_with(a: &Access<'_>, slot: u8) -> Option<u16> {
        let bytes = Self::descriptor_with(a, slot, STRING_DESCRIPTOR, 0, 255)
            .await
            .ok()?;
        if bytes.get(1).is_some_and(|kind| *kind != STRING_DESCRIPTOR) {
            return Some(LANGUAGE_EN_US);
        }
        match usize::from(*bytes.first()?).min(bytes.len()) & !1 {
            0 => None,
            2 => Some(LANGUAGE_EN_US),
            _ => Some(u16::from_le_bytes([bytes[2], bytes[3]])),
        }
    }

    #[cfg(test)]
    pub(crate) fn string(
        &mut self,
        id: impl DeviceRef,
        language: u16,
        index: u8,
    ) -> Option<String> {
        let slot = id.current_slot(self).ok()?;
        self.string_owned(slot, language, index)
    }

    pub(super) fn string_owned(&mut self, slot: u8, language: u16, index: u8) -> Option<String> {
        block_on(Self::string_with(
            &Access::blocking(self, None),
            slot,
            language,
            index,
        ))
    }

    pub(super) async fn string_with(
        a: &Access<'_>,
        slot: u8,
        language: u16,
        index: u8,
    ) -> Option<String> {
        if index == 0 {
            return None;
        }
        let value = u16::from(STRING_DESCRIPTOR) << 8 | u16::from(index);
        let request = setup(DEVICE_TO_HOST, GET_DESCRIPTOR, value, language, 255);
        let bytes = a.control(slot, request, &[]).await.ok()?;
        if bytes.len() < 2 || bytes[1] != STRING_DESCRIPTOR {
            return None;
        }
        let units: Vec<u16> = bytes[2..usize::from(bytes[0]).min(bytes.len())]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|pair| u16::from_le_bytes(*pair))
            .collect();
        let text: String = char::decode_utf16(units)
            .map(|c| c.unwrap_or(char::REPLACEMENT_CHARACTER))
            .collect();
        Some(String::from(text.trim()))
    }

    /// Current connection of the slot's root port. Downstream hub-port loss
    /// is reported by transfer errors; hotplug enumeration is not performed.
    pub fn connected(&mut self, id: impl DeviceRef) -> Result<bool, BlockError> {
        let slot = id.current_slot(self)?;
        self.connected_owned(slot)
    }

    pub(super) fn connected_owned(&mut self, slot: u8) -> Result<bool, BlockError> {
        self.live_slot(slot)?;
        self.ready()?;
        let port = (self.slot(slot)?.context[1] >> 16 & 0xff) as u8;
        if port == 0 || port > self.max_ports {
            return Err(BlockError::Unsupported("xHCI invalid slot root port"));
        }
        let at = self.port_offset(port);
        Ok(self.registers.read32(at) & CCS != 0)
    }

    /// Abandons every outstanding TD on one exclusively owned
    /// endpoint. Running endpoints are stopped, halted endpoints reset, then
    /// Set TR Dequeue moves past the abandoned TDs (xHCI 1.2 4.6.9/4.6.10).
    /// Success acknowledges that their buffers are no longer hardware-owned.
    /// Failure permanently poisons submission to this endpoint: callers must
    /// keep the buffers and must not read, rewrite or recycle them. Device
    /// disconnect alone is not proof that controller DMA has stopped.
    pub fn cancel(&mut self, id: impl DeviceRef, dci: u8) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.cancel_owned(slot, dci)
    }

    pub(super) fn cancel_owned(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.live_slot(slot)?;
        self.cancel_inner(slot, dci)
    }

    pub(super) fn cancel_inner(&mut self, slot: u8, dci: u8) -> Result<(), BlockError> {
        self.cancel_observed(slot, dci).map(|_| ())
    }

    fn cancel_observed(&mut self, slot: u8, dci: u8) -> Result<Vec<Event>, BlockError> {
        let cancel = self.start_cancel(slot, dci)?;
        self.run_cancel(cancel)
    }

    /// Resets the host side of an endpoint a transaction error halted; the
    /// device's endpoint is not halted, so no CLEAR_FEATURE follows.
    pub fn reset_endpoint(
        &mut self,
        id: impl DeviceRef,
        endpoint: &Endpoint,
    ) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.reset_endpoint_owned(slot, endpoint)
    }

    pub(super) fn reset_endpoint_owned(
        &mut self,
        slot: u8,
        endpoint: &Endpoint,
    ) -> Result<(), BlockError> {
        self.recover(slot, endpoint.dci())
    }

    /// Clears a halted bulk or interrupt endpoint on both sides.
    pub fn clear(&mut self, id: impl DeviceRef, endpoint: &Endpoint) -> Result<(), BlockError> {
        let slot = id.current_slot(self)?;
        self.clear_owned(slot, endpoint)
    }

    pub(super) fn clear_owned(&mut self, slot: u8, endpoint: &Endpoint) -> Result<(), BlockError> {
        let (cancel, request) = self.start_clear_owned(slot, endpoint)?;
        self.run_cancel(cancel)?;
        self.control_owned(slot, request, &[])?;
        Ok(())
    }

    /// The host side's cancellation, begun, and the CLEAR_FEATURE that
    /// follows it.
    pub(super) fn start_clear_owned(
        &mut self,
        slot: u8,
        endpoint: &Endpoint,
    ) -> Result<(Cancel, [u8; 8]), BlockError> {
        if endpoint.kind() == 1 {
            return Err(BlockError::Unsupported(
                "isochronous endpoints have no USB halt feature",
            ));
        }
        self.live_slot(slot)?;
        let cancel = self.start_cancel(slot, endpoint.dci())?;
        let request = setup(
            RECIPIENT_ENDPOINT,
            CLEAR_FEATURE,
            0,
            u16::from(endpoint.address),
            0,
        );
        Ok((cancel, request))
    }
}

#[cfg(test)]
mod tests;
