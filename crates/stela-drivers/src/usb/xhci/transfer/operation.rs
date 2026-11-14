//! EP0 control transfers and endpoint cancellation, started and then looked
//! at without waiting, as Linux submits an URB and the command ring and
//! completions run apart from its submitter. The waiting forms run these to
//! their end.
use super::*;

/// A started EP0 control transfer.
pub(in super::super) struct Control {
    slot: u8,
    length: usize,
    is_in: bool,
    setup_address: u64,
    data_address: Option<u64>,
    status_address: u64,
    deadline: Deadline,
    received: usize,
    short_seen: bool,
    /// SET_CONFIGURATION's value and current demand, acknowledged with it.
    configuration: Option<(u8, u16)>,
    /// EP0's cancellation after a failure, which it then reports.
    failing: Option<(Box<Cancel>, BlockError)>,
}

/// A started endpoint cancellation: Stop or Reset Endpoint, then Set TR
/// Dequeue past every abandoned TD (xHCI 1.2 4.6.9/4.6.10), then any TT
/// buffer clear.
pub(in super::super) struct Cancel {
    slot: u8,
    dci: u8,
    step: Cancelling,
    pointers: Vec<(u16, u64)>,
    clear_tt: bool,
    command: Option<u64>,
    observed: Vec<Event>,
    /// Until when a Stop Endpoint that failed with a Context State Error on
    /// an endpoint still Running, or still showing Stopped, is sent again:
    /// [`STOP_RETRY_US`] from the first one.
    retry_until: Option<Deadline>,
}

/// Linux `xhci_handle_cmd_stop_ep` sends a Stop Endpoint that failed with a
/// Context State Error again for 100 ms from the first while the endpoint
/// shows Running or Stopped, then goes on to Set TR Dequeue. Stopped past
/// it is taken as stopped here too; Running past it did not stop.
const STOP_RETRY_US: u64 = 100_000;

enum Cancelling {
    Stop,
    Reset,
    /// Set TR Dequeue for the pointer at this index.
    Dequeue(usize),
    /// The TT hub's CLEAR_TT_BUFFER requests still to send, last first, and
    /// the one running.
    Tt {
        hub: u8,
        requests: Vec<[u8; 8]>,
        running: Option<Box<Control>>,
    },
}

/// A control transfer, endpoint cancellation or halt clear started on a
/// device, looked at with [`DeviceAccess::poll`] until it ends.
pub struct Operation {
    pub(in super::super) id: SlotId,
    pub(in super::super) kind: Kind,
}

pub(in super::super) enum Kind {
    Control(Control),
    Cancel(Cancel),
    /// The host side's cancellation, then CLEAR_FEATURE(ENDPOINT_HALT).
    Clear(Cancel, [u8; 8]),
}

impl Controller {
    /// Queue `request` on EP0 and ring its doorbell. EP0 takes one control
    /// transfer at a time.
    pub(in super::super) fn start_control(
        &mut self,
        slot: u8,
        request: [u8; 8],
        out: &[u8],
        configuration: Option<(u8, u16)>,
    ) -> Result<Control, BlockError> {
        self.endpoint_ready_inner(slot, 1)?;
        let length = usize::from(u16::from_le_bytes([request[6], request[7]]));
        let is_in = request[0] & 0x80 != 0;
        if length > CONTROL_BYTES || (!is_in && out.len() != length) {
            return Err(BlockError::Unsupported("control transfer length"));
        }
        let deadline = self.event_deadline(CONTROL_US)?;
        let buffer = {
            let state = self.slot(slot)?;
            let endpoint = state.endpoints[1]
                .as_ref()
                .ok_or(BlockError::Unsupported("xHCI control endpoint"))?;
            if endpoint.poisoned {
                return Err(BlockError::Unsupported("xHCI endpoint cancellation failed"));
            }
            if endpoint.in_flight != 0 {
                return Err(BlockError::Protocol(
                    "an xHCI control transfer is outstanding",
                ));
            }
            if !is_in {
                state.control.bytes_mut()[..length].copy_from_slice(out);
            }
            state.control.sync_dirty()?;
            state.control.device_address()
        };
        let transfer_type = match (length, is_in) {
            (0, _) => 0,
            (_, false) => 2,
            (_, true) => 3,
        };
        let setup = (
            u64::from_le_bytes(request),
            8,
            SETUP << 10 | IDT | transfer_type << 16,
        );
        let status_direction = if length > 0 && is_in { 0 } else { DIRECTION_IN };
        let status = (0, 0, STATUS << 10 | IOC | status_direction);
        let (setup_address, data_address, status_address) = {
            let endpoint = self.slot(slot)?.endpoints[1].as_mut().unwrap();
            let addresses = if length > 0 {
                let direction = if is_in { DIRECTION_IN } else { 0 };
                let data = (buffer, length as u32, DATA << 10 | ISP | direction);
                let [setup, data, status] = endpoint.ring.push(&[setup, data, status])?;
                (setup, Some(data), status)
            } else {
                let [setup, status] = endpoint.ring.push(&[setup, status])?;
                (setup, None, status)
            };
            endpoint.in_flight = 1;
            addresses
        };
        self.doorbell(slot, 1);
        Ok(Control {
            slot,
            length,
            is_in,
            setup_address,
            data_address,
            status_address,
            deadline,
            received: length,
            short_seen: false,
            configuration,
            failing: None,
        })
    }

    /// The control transfer's IN data, or an empty OUT result, once it
    /// ended; `None` while it runs. A failure first cancels EP0.
    pub(in super::super) fn poll_control(
        &mut self,
        control: &mut Control,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        if let Some((cancel, error)) = &mut control.failing {
            let result = self.poll_cancel(cancel)?;
            return Some(result.and(Err(error.clone())));
        }
        let slot = control.slot;
        let status_address = control.status_address;
        loop {
            let event = match self.poll_event(control.deadline, |e| {
                e.kind() == TRANSFER_EVENT && e.slot() == slot && e.endpoint() == 1
            }) {
                None => return None,
                Some(Ok(event)) => event,
                // A missing transfer completion leaves EP0 Running, not
                // Halted. Only acknowledged Stop/Set Dequeue permits reuse;
                // failed cancellation is reported and keeps poison.
                Some(Err(error)) => return self.fail_control(control, error),
            };
            // An event must name this TD's Setup/Data/Status TRB.
            // Event Data is never submitted by this control path. Only Data
            // short packets may precede the final zero-length Status ACK.
            let valid = event.control & EVENT_DATA == 0
                && (event.parameter == control.setup_address
                    || event.parameter == status_address
                    || Some(event.parameter) == control.data_address)
                && match event.code() {
                    SUCCESS => event.parameter == status_address && event.residual() == 0,
                    SHORT_PACKET => {
                        control.data_address == Some(event.parameter)
                            && event.residual() <= control.length
                            && !control.short_seen
                    }
                    _ => true,
                };
            if !valid {
                return self.fail_control(
                    control,
                    BlockError::Unsupported("invalid xHCI control transfer event"),
                );
            }
            match event.code() {
                SUCCESS => return Some(self.conclude_control(control)),
                SHORT_PACKET => {
                    control.short_seen = true;
                    control.received = control.length - event.residual();
                }
                status => {
                    let endpoint = match self.slot(slot) {
                        Ok(state) => state.endpoints[1].as_mut().unwrap(),
                        Err(error) => return Some(Err(error)),
                    };
                    if status == INCOMPATIBLE_DEVICE {
                        endpoint.poisoned = true;
                        return Some(Err(BlockError::Device { status }));
                    }
                    endpoint.ring.normal.control_failure(status);
                    return self.fail_control(control, BlockError::Device { status });
                }
            }
        }
    }

    fn fail_control(
        &mut self,
        control: &mut Control,
        error: BlockError,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        match self.start_cancel(control.slot, 1) {
            Ok(cancel) => {
                control.failing = Some((Box::new(cancel), error));
                self.poll_control(control)
            }
            Err(cancel) => Some(Err(cancel)),
        }
    }

    fn conclude_control(&mut self, control: &Control) -> Result<Vec<u8>, BlockError> {
        let state = self.slot(control.slot)?;
        state.endpoints[1].as_mut().unwrap().in_flight = 0;
        let bytes = if control.is_in {
            // SAFETY: the Status completion retired this EP0 TD's
            // Data stage.
            unsafe {
                state.control.sync_cpu(0..control.received)?;
            }
            state.control.bytes()[..control.received].to_vec()
        } else if control.received != control.length {
            return Err(BlockError::Unsupported("short USB control OUT transfer"));
        } else {
            Vec::new()
        };
        if let Some((value, demand)) = control.configuration {
            self.slot(control.slot)?.current_configuration = Some(value);
            self.ack_current_configuration(control.slot, demand)?;
        }
        Ok(bytes)
    }

    /// Run a control transfer to its end.
    pub(in super::super) fn run_control(
        &mut self,
        mut control: Control,
    ) -> Result<Vec<u8>, BlockError> {
        loop {
            if let Some(result) = self.poll_control(&mut control) {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// Begin abandoning every outstanding TD on one exclusively owned
    /// endpoint. The endpoint stays poisoned until [`Self::poll_cancel`]
    /// reports success.
    pub(in super::super) fn start_cancel(
        &mut self,
        slot: u8,
        dci: u8,
    ) -> Result<Cancel, BlockError> {
        if !(1..32).contains(&dci) {
            return Err(BlockError::Unsupported("cancel requires a valid endpoint"));
        }
        let endpoint = self.slot(slot)?.endpoints[usize::from(dci)]
            .as_mut()
            .ok_or(BlockError::Unsupported("xHCI endpoint not configured"))?;
        if endpoint.poisoned {
            return Err(BlockError::Unsupported("xHCI endpoint cancellation failed"));
        }
        // Set before issuing any command, so every error path keeps poison.
        endpoint.poisoned = true;
        let pointers = endpoint.streams.as_ref().map_or_else(
            || vec![(0u16, endpoint.ring.dequeue_pointer())],
            |streams| streams.dequeue_pointers(),
        );
        let clear_tt = endpoint.ring.normal.needs_tt_clear();
        let mut cancel = Cancel {
            slot,
            dci,
            step: Cancelling::Stop,
            pointers,
            clear_tt,
            command: None,
            observed: Vec::new(),
            retry_until: None,
        };
        let stopped = endpoint.stopped;
        // Running, including a ring whose TD just completed, is stopped, as
        // is Stopped since a doorbell: as Linux `xhci_urb_dequeue`, a Stop
        // Endpoint unless software knows it stopped. Halted only Reset
        // Endpoint leaves.
        match self.endpoint_state(slot, dci)? {
            ENDPOINT_HALTED => {
                cancel.clear_tt |= dci != 1;
                cancel.step = Cancelling::Reset;
            }
            ENDPOINT_STOPPED if stopped => cancel.step = Cancelling::Dequeue(0),
            ENDPOINT_RUNNING | ENDPOINT_STOPPED => {}
            _ => return Err(DID_NOT_STOP),
        }
        Ok(cancel)
    }

    /// Whether a Stop Endpoint may be sent again: within [`STOP_RETRY_US`]
    /// of the first, which starts the window.
    fn may_retry_stop(&self, cancel: &mut Cancel) -> Result<bool, BlockError> {
        match &cancel.retry_until {
            Some(until) => Ok(!until.passed()),
            None => {
                cancel.retry_until = Some(self.event_deadline(STOP_RETRY_US)?);
                Ok(true)
            }
        }
    }

    fn endpoint_state(&mut self, slot: u8, dci: u8) -> Result<u32, BlockError> {
        let context = self.context();
        fence(Ordering::Acquire);
        Ok(self
            .slot(slot)?
            ._output
            .coherent_u32(usize::from(dci) * context)?
            & 7)
    }

    /// The events the cancelled TDs left once every command and TT clear
    /// acknowledged their abandonment; `None` while one runs.
    pub(in super::super) fn poll_cancel(
        &mut self,
        cancel: &mut Cancel,
    ) -> Option<Result<Vec<Event>, BlockError>> {
        let (slot, dci) = (cancel.slot, cancel.dci);
        let target = u32::from(slot) << 24 | u32::from(dci) << 16;
        loop {
            match &mut cancel.step {
                Cancelling::Stop => {
                    if cancel.command.is_none()
                        && let Err(error) = self.may_retry_stop(cancel)
                    {
                        return Some(Err(error));
                    }
                    let refused = match self.step_command(
                        &mut cancel.command,
                        0,
                        0,
                        STOP_ENDPOINT << 10 | target,
                    )? {
                        Ok(_) => false,
                        // A STALL can race Stop Endpoint. Recheck the hardware
                        // state; never treat an arbitrary Context State Error
                        // as acknowledgment.
                        Err(BlockError::Device {
                            status: CONTEXT_STATE_ERROR,
                        }) => true,
                        Err(error) => return Some(Err(error)),
                    };
                    match (self.endpoint_state(slot, dci), refused) {
                        (Ok(ENDPOINT_HALTED), _) => {
                            cancel.clear_tt |= dci != 1;
                            cancel.step = Cancelling::Reset;
                        }
                        (Ok(ENDPOINT_STOPPED), false) => cancel.step = Cancelling::Dequeue(0),
                        // The controller handled the Stop before the
                        // endpoint ran (Running), or before its context
                        // caught up with a doorbell (Stopped): send it
                        // again, from the next look, within the window.
                        // Stopped past it is taken as stopped.
                        (Ok(state @ (ENDPOINT_RUNNING | ENDPOINT_STOPPED)), true) => {
                            match self.may_retry_stop(cancel) {
                                Ok(true) => return None,
                                Ok(false) if state == ENDPOINT_STOPPED => {
                                    cancel.step = Cancelling::Dequeue(0);
                                }
                                Ok(false) => return Some(Err(DID_NOT_STOP)),
                                Err(error) => return Some(Err(error)),
                            }
                        }
                        (Ok(_), _) => return Some(Err(DID_NOT_STOP)),
                        (Err(error), _) => return Some(Err(error)),
                    }
                }
                Cancelling::Reset => {
                    let reset = RESET_ENDPOINT << 10 | target;
                    if let Err(error) = self.step_command(&mut cancel.command, 0, 0, reset)? {
                        return Some(Err(error));
                    }
                    // Stopped is the only state in which dequeue is legal.
                    match self.endpoint_state(slot, dci) {
                        Ok(ENDPOINT_STOPPED) => cancel.step = Cancelling::Dequeue(0),
                        Ok(_) => return Some(Err(DID_NOT_STOP)),
                        Err(error) => return Some(Err(error)),
                    }
                }
                Cancelling::Dequeue(index) => {
                    let index = *index;
                    let Some(&(stream_id, pointer)) = cancel.pointers.get(index) else {
                        match self.dequeued(cancel) {
                            Ok(Some((hub, mut requests))) => {
                                requests.reverse();
                                cancel.step = Cancelling::Tt {
                                    hub,
                                    requests,
                                    running: None,
                                };
                                continue;
                            }
                            Ok(None) => return Some(self.cancelled(cancel)),
                            Err(error) => return Some(Err(error)),
                        }
                    };
                    let dequeue = SET_DEQUEUE << 10 | target;
                    let status = streams::dequeue_status(stream_id);
                    match self.step_command(&mut cancel.command, pointer, status, dequeue)? {
                        Ok(_) => {}
                        // The endpoint ran again after all: stop it again
                        // within the window, from the next look.
                        Err(
                            error @ BlockError::Device {
                                status: CONTEXT_STATE_ERROR,
                            },
                        ) => {
                            let running = self.endpoint_state(slot, dci) == Ok(ENDPOINT_RUNNING);
                            match self.may_retry_stop(cancel) {
                                Ok(true) if running => {
                                    cancel.step = Cancelling::Stop;
                                    return None;
                                }
                                Err(error) => return Some(Err(error)),
                                _ => return Some(Err(error)),
                            }
                        }
                        Err(error) => return Some(Err(error)),
                    }
                    match self.slot(slot) {
                        Ok(state) => {
                            if let Some(streams) =
                                &mut state.endpoints[usize::from(dci)].as_mut().unwrap().streams
                            {
                                streams.dequeue_acknowledged(stream_id);
                            }
                        }
                        Err(error) => return Some(Err(error)),
                    }
                    cancel.step = Cancelling::Dequeue(index + 1);
                }
                Cancelling::Tt {
                    hub,
                    requests,
                    running,
                } => {
                    if let Some(control) = running {
                        if let Err(error) = self.poll_control(control)? {
                            return Some(Err(error));
                        }
                        *running = None;
                    }
                    let Some(request) = requests.pop() else {
                        return Some(self.cancelled(cancel));
                    };
                    match self.start_control_owned(*hub, request, &[]) {
                        Ok(control) => *running = Some(Box::new(control)),
                        Err(error) => return Some(Err(error)),
                    }
                }
            }
        }
    }

    /// After the last Set TR Dequeue: the cancelled TDs' events, and the TT
    /// clear a split failure needs.
    fn dequeued(&mut self, cancel: &mut Cancel) -> Result<TtRequests, BlockError> {
        let (slot, dci) = (cancel.slot, cancel.dci);
        // Stop/Set Dequeue completion acknowledges abandonment. Drain also takes
        // stale transfer events posted while those commands were executing.
        self.drain();
        if self.pending.iter().any(|event| {
            event.kind() == TRANSFER_EVENT
                && event.slot() == slot
                && event.endpoint() == dci
                && event.code() == INCOMPATIBLE_DEVICE
        }) {
            // Stop/SetDequeue is not the required Disable Slot recovery.
            // Preserve poison and all records even if those commands ACKed.
            return Err(BlockError::Device {
                status: INCOMPATIBLE_DEVICE,
            });
        }
        let observed = &mut cancel.observed;
        self.pending.retain(|event| {
            if event.kind() == TRANSFER_EVENT && event.slot() == slot && event.endpoint() == dci {
                observed.push(*event);
                false
            } else {
                true
            }
        });
        cancel.clear_tt |= observed.iter().any(|event| {
            matches!(
                event.code(),
                BABBLE_DETECTED | USB_TRANSACTION_ERROR | SPLIT_TRANSACTION_ERROR
            ) || event.code() == STALL && dci != 1
        });
        // A failed split control/bulk transaction may leave the external
        // high-speed hub's TT occupied even after xHC DMA has retired. Only
        // the hub request ACK permits this endpoint to resume.
        if cancel.clear_tt {
            self.hub_tt_requests(slot, dci)
        } else {
            Ok(None)
        }
    }

    fn cancelled(&mut self, cancel: &mut Cancel) -> Result<Vec<Event>, BlockError> {
        let endpoint = self.slot(cancel.slot)?.endpoints[usize::from(cancel.dci)]
            .as_mut()
            .unwrap();
        if let Some(streams) = &mut endpoint.streams {
            streams.cancelled();
        }
        if let Some(iso) = &mut endpoint.iso {
            iso.cancelled();
        }
        endpoint.ring.normal.cancelled();
        endpoint.poisoned = false;
        endpoint.in_flight = 0;
        endpoint.stopped = true;
        Ok(core::mem::take(&mut cancel.observed))
    }

    /// Run a cancellation to its end.
    pub(in super::super) fn run_cancel(
        &mut self,
        mut cancel: Cancel,
    ) -> Result<Vec<Event>, BlockError> {
        loop {
            if let Some(result) = self.poll_cancel(&mut cancel) {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// An operation's result once it ended; `None` while it runs.
    pub(in super::super) fn poll_operation(
        &mut self,
        operation: &mut Operation,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        loop {
            match &mut operation.kind {
                Kind::Control(control) => return self.poll_control(control),
                Kind::Cancel(cancel) => return Some(self.poll_cancel(cancel)?.map(|_| Vec::new())),
                Kind::Clear(cancel, request) => {
                    let request = *request;
                    if let Err(error) = self.poll_cancel(cancel)? {
                        return Some(Err(error));
                    }
                    match self.start_control_owned(cancel.slot, request, &[]) {
                        Ok(control) => operation.kind = Kind::Control(control),
                        Err(error) => return Some(Err(error)),
                    }
                }
            }
        }
    }

    /// [`Self::poll_operation`] for an owner that stops: a control transfer
    /// under way has its EP0 TDs cancelled on the host side, as
    /// usb_kill_urb unlinks an URB, and ends as `error` once the
    /// cancellation is acknowledged; a halt clear sends no CLEAR_FEATURE
    /// after its cancellation. A cancellation runs to its end.
    pub(in super::super) fn abort_operation(
        &mut self,
        operation: &mut Operation,
        error: BlockError,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        match &mut operation.kind {
            Kind::Control(control) if control.failing.is_none() => {
                self.fail_control(control, error)
            }
            Kind::Clear(cancel, _) => Some(self.poll_cancel(cancel)?.and(Err(error))),
            _ => self.poll_operation(operation),
        }
    }
}

const DID_NOT_STOP: BlockError = BlockError::Unsupported("xHCI endpoint did not stop");

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::TestRegion;
    use crate::usb::xhci::model::*;

    #[test]
    fn operations_never_wait_and_ep0_takes_one_control_transfer_at_a_time() {
        let mut dma = TestRegion::new(64 * 1024 * 8);
        let (mut controller, model) = cancellation_controller(&mut dma);
        let id = controller.test_device_id(1).unwrap();
        model.borrow_mut().silence_transfer = Some(1);
        let mut device = controller.device(id).unwrap();
        let request = setup(0x20, 0, 0, 0, 3);
        let mut control = device.start_control(request, &[3, 0x0c, 0]).unwrap();
        assert!(device.poll(&mut control).is_none());
        assert!(matches!(
            device.start_control(request, &[3, 0x0c, 0]),
            Err(BlockError::Protocol(
                "an xHCI control transfer is outstanding"
            ))
        ));
        let result = loop {
            if let Some(result) = device.poll(&mut control) {
                break result;
            }
        };
        assert_eq!(result, Err(BlockError::Timeout));
        assert_eq!(model.borrow().seen, [STOP_ENDPOINT, SET_DEQUEUE]);
        model.borrow_mut().silence_transfer = None;
        let mut control = device.start_control(request, &[3, 0x0c, 0]).unwrap();
        assert_eq!(device.poll(&mut control), Some(Ok(Vec::new())));
        // A halt clear: the host side's cancellation, then CLEAR_FEATURE.
        let mut clear = device.start_clear(&fixture_endpoint(3)).unwrap();
        assert_eq!(device.poll(&mut clear), Some(Ok(Vec::new())));
        let last = model.borrow().setup.last().unwrap().0;
        assert_eq!(last, setup(RECIPIENT_ENDPOINT, CLEAR_FEATURE, 0, 0x81, 0));
    }
}
