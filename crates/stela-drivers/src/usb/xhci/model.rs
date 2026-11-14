//! xHCI controller and device models shared by tests across the USB stack.
pub(crate) use super::bind::{bind, bind_with_provider};
pub(crate) use super::configuration::tests::{addressed_device, device, ecm};
pub(crate) use super::configuration::{sanitize, select};
pub(crate) use super::iso::tests::class_fixture as iso_class_fixture;
pub(crate) use super::reset::tests::abort_peer::Fault as UasAbortFault;
pub(crate) use super::reset::tests::{
    Fault as ResetFault, Hardware as ResetHardware, fixture as reset_fixture,
    fixture_bot as reset_bot_fixture, fixture_forwarding as reset_forwarding_fixture,
    fixture_hid as reset_hid_fixture,
};
use super::*;
pub(crate) use crate::bus::dma::{RegionAllocator, TestRegion};

impl Controller {
    /// Install a hostile register surface after fixture construction. Identity
    /// admission must fail before even a readiness read can reach this surface.
    pub(crate) fn forbid_mmio(&mut self) {
        struct NoAccess;
        impl Registers for NoAccess {
            fn read32(&mut self, _: usize) -> u32 {
                panic!("retired/foreign consumer reached MMIO read")
            }
            fn write32(&mut self, _: usize, _: u32) {
                panic!("retired/foreign consumer reached MMIO write")
            }
        }
        self.registers = Box::new(NoAccess);
    }
    /// Read kept native allocations after checking the caller's
    /// attachment identity. Includes controller-wide and retry owners;
    /// allocation lengths include full ring padding, never just active TRBs.
    pub(crate) fn reset_allocation_geometry(
        &self,
        id: SlotId,
    ) -> Result<Vec<(u64, usize)>, BlockError> {
        let slot = self.validate_id(id)?;
        let state = self.slots[usize::from(slot)].as_ref().unwrap();
        let mut all: Vec<_> = [
            &self.dcbaa,
            &self.commands.buffer,
            &self.events,
            &self._event_table,
            &state.input,
            &state._output,
            &state.control,
        ]
        .into_iter()
        .chain(self._scratchpads.iter())
        .map(|buffer| (buffer.physical(), buffer.len()))
        .collect();
        for endpoint in state.endpoints.iter().flatten() {
            all.push((endpoint.ring.buffer.physical(), endpoint.ring.buffer.len()));
            if let Some(streams) = &endpoint.streams {
                all.extend(streams.allocation_geometry());
            }
        }
        for ring in state.retired_rings.iter().flatten() {
            all.push((ring.buffer.physical(), ring.buffer.len()));
        }
        all.extend(self.retry_resources.allocation_geometry());
        all.sort_unstable();
        Ok(all)
    }

    /// Record the speed a model device in `slot` was addressed at.
    pub(crate) fn set_slot_speed(&mut self, slot: u8, speed: u8) {
        self.slots[usize::from(slot)].as_mut().unwrap().speed = speed;
    }
    /// Whether failed cancellation poisoned an endpoint.
    pub(crate) fn endpoint_poisoned(&self, slot: u8, dci: usize) -> bool {
        self.slots[usize::from(slot)].as_ref().unwrap().endpoints[dci]
            .as_ref()
            .unwrap()
            .poisoned
    }
    /// The contents of a slot's EP0 data buffer.
    pub(crate) fn control_bytes(&self, slot: u8) -> Vec<u8> {
        self.slots[usize::from(slot)]
            .as_ref()
            .unwrap()
            .control
            .bytes()
            .to_vec()
    }
}

/// A command/event DMA model with a device that needs the full
/// SET_ADDRESS recovery interval before accepting its first Setup.
pub(crate) struct AddressModel {
    pub(crate) port_speed: u8,
    pub(crate) context_bytes: usize,
    pub(crate) commands: u64,
    pub(crate) command_index: usize,
    pub(crate) events: u64,
    pub(crate) event_index: usize,
    pub(crate) control_ring: u64,
    pub(crate) dcbaa: u64,
    pub(crate) output: u64,
    pub(crate) ready_at: u64,
    pub(crate) recovery_ticks: u64,
    pub(crate) address_code: u32,
    pub(crate) commands_seen: Vec<u32>,
    pub(crate) setup_seen: Vec<[u8; 8]>,
    pub(crate) early_setup: bool,
}

impl AddressModel {
    pub(crate) fn event(&mut self, parameter: u64, code: u32, kind: u32, endpoint: u8) {
        let address = self.events + (self.event_index * TRB) as u64;
        // SAFETY: these are aligned DMA words in the test's backing
        // allocation, which outlives the controller and this model.
        unsafe {
            (address as *mut u64).write_volatile(parameter);
            ((address + 8) as *mut u32).write_volatile(code << 24);
            ((address + 12) as *mut u32)
                .write_volatile(1 << 24 | u32::from(endpoint) << 16 | kind << 10 | CYCLE);
        }
        self.event_index += 1;
    }
}

pub(crate) struct AddressRegisters(Rc<RefCell<AddressModel>>);

impl Registers for AddressRegisters {
    fn read32(&mut self, offset: usize) -> u32 {
        if offset == 0x440 {
            PP | CCS | PED | u32::from(self.0.borrow().port_speed) << 10
        } else {
            0
        }
    }

    fn write32(&mut self, offset: usize, _value: u32) {
        let mut model = self.0.borrow_mut();
        if offset == 0x1000 {
            let address = model.commands + (model.command_index * TRB) as u64;
            // SAFETY: the command ring and the input context named by
            // Address Device belong to the live test DMA allocation.
            let (parameter, kind) = unsafe {
                (
                    (address as *const u64).read_volatile(),
                    ((address + 12) as *const u32).read_volatile() >> 10 & 0x3f,
                )
            };
            model.command_index += 1;
            model.commands_seen.push(kind);
            let code = if kind == ADDRESS_DEVICE {
                // SAFETY: the live DCBAA slot points to the output context
                // allocated by address() before publishing this command.
                model.output = unsafe { ((model.dcbaa + 8) as *const u64).read_volatile() };
                // SAFETY: EP0's dequeue pointer in the input context
                // points to this test controller's control ring.
                model.control_ring = unsafe {
                    ((parameter + 2 * model.context_bytes as u64 + 8) as *const u64).read_volatile()
                        & !0xf
                };
                model.ready_at = Clock::ticks() + model.recovery_ticks;
                model.address_code
            } else {
                if matches!(kind, RESET_ENDPOINT | STOP_ENDPOINT) {
                    // SAFETY: EP0 state in the output context.
                    unsafe {
                        ((model.output + model.context_bytes as u64) as *mut u32)
                            .write_volatile(ENDPOINT_STOPPED);
                    }
                }
                if kind == SET_DEQUEUE {
                    model.control_ring = parameter & !0xf;
                }
                SUCCESS
            };
            model.event(address, code, COMMAND_COMPLETION, 0);
        } else if offset == 0x1004 {
            // This model runs one GET_DESCRIPTOR with Setup/Data/Status.
            let ring = model.control_ring;
            // SAFETY: the ring is initialized by control() before its
            // doorbell, in the test's still-live DMA allocation.
            let request = unsafe { (ring as *const u64).read_volatile().to_le_bytes() };
            model.setup_seen.push(request);
            model.early_setup = Clock::ticks() < model.ready_at;
            let (parameter, code) = if model.early_setup {
                // SAFETY: failed transfer halts EP0 in the live context.
                unsafe {
                    ((model.output + model.context_bytes as u64) as *mut u32)
                        .write_volatile(ENDPOINT_HALTED);
                }
                (ring, 4)
            } else {
                (ring + 2 * TRB as u64, SUCCESS)
            };
            model.event(parameter, code, TRANSFER_EVENT, 1);
        }
    }
}

pub(crate) fn address_controller(
    dma: &mut dyn DmaAllocator,
    address_code: u32,
) -> (Controller, Rc<RefCell<AddressModel>>) {
    address_controller_with_capabilities(dma, address_code, None)
}

/// Independent capability-register image installed before the model Controller
/// captures ISO capabilities. The default fixture does not install this peer.
struct CapabilityRegisters {
    original: Box<dyn Registers>,
    words: [u32; 3],
}
impl Registers for CapabilityRegisters {
    fn read32(&mut self, offset: usize) -> u32 {
        match offset {
            0x00 => self.words[0],
            0x08 => self.words[1],
            0x10 => self.words[2],
            _ => self.original.read32(offset),
        }
    }
    fn write32(&mut self, offset: usize, value: u32) {
        self.original.write32(offset, value);
    }
}

fn address_controller_with_capabilities(
    dma: &mut dyn DmaAllocator,
    address_code: u32,
    capability_words: Option<[u32; 3]>,
) -> (Controller, Rc<RefCell<AddressModel>>) {
    let commands = Ring::new(dma).unwrap();
    let events = dma.allocate(RING_TRBS * TRB, 64).unwrap();
    let clock = Clock {
        ticks_per_microsecond: 1_000,
    };
    let model = Rc::new(RefCell::new(AddressModel {
        port_speed: HIGH_SPEED,
        context_bytes: 32,
        commands: commands.buffer.physical(),
        command_index: 0,
        events: events.physical(),
        event_index: 0,
        control_ring: 0,
        dcbaa: 0,
        output: 0,
        ready_at: 0,
        recovery_ticks: ADDRESS_RECOVERY_US * clock.ticks_per_microsecond,
        address_code,
        commands_seen: Vec::new(),
        setup_seen: Vec::new(),
        early_setup: false,
    }));
    let mut registers: Box<dyn Registers> = Box::new(AddressRegisters(model.clone()));
    let (version, iso_caps) = if let Some(words) = capability_words {
        registers = Box::new(CapabilityRegisters {
            original: registers,
            words,
        });
        // Reads of the installed independent wire image precede capture;
        // caller integers never replace an already-constructed Controller field.
        let version = (registers.read32(0x00) >> 16) as u16;
        let hcs2 = registers.read32(0x08);
        let hcc1 = registers.read32(0x10);
        (version, iso::Capabilities::new(version, hcs2, hcc1))
    } else {
        (0x100, iso::Capabilities::new(0x100, 0, 0))
    };
    let controller = Controller {
        minimum: crate::usb::xhci::minimum::fixture(clock),
        native_memory: None,
        registers,
        clock,
        operational: 0x40,
        runtime: 0x800,
        doorbells: 0x1000,
        context_bytes: 32,
        version,
        dma_profile: dma::Profile::SPEC,
        hardware: hardware::State::Running,
        quirks: quirks::Profile::default(),
        max_primary_streams: 4,
        stream_policy: streams::Policy::Unknown,
        iso_caps,
        max_slots: 1,
        max_ports: 1,
        port_major: vec![2],
        lpm: lpm::Host::default(),
        dcbaa: dma.allocate(16, 64).unwrap().into(),
        _scratchpads: Vec::new(),
        commands,
        events: events.into(),
        _event_table: dma.allocate(16, 64).unwrap().into(),
        event_index: 0,
        event_cycle: true,
        slots: vec![None, None],
        retry_resources: slot_resources::Cache::new(2),
        queued_resets: Vec::new(),
        slot_ownership: vec![reset::Ownership::Vacant; 2],
        test_retired_numbers: vec![false; 2],
        power: power::Ledger::new(2),
        pending: Vec::new(),
        error: None,
        settle_from: None,
        command_poisoned: false,
        death: None,
        command_ring_stopped: false,
        unpublished: 0,
        signalled: false,
        quiet_drains: 0,
        outstanding: None,
        finished: Vec::new(),
        location: String::from("test xhci"),
    };
    model.borrow_mut().dcbaa = controller.dcbaa.physical();
    (controller, model)
}

/// Hardware model enforces the endpoint state restrictions absent from the
/// old address model: Stop accepts Running, Reset accepts Halted, Set Dequeue
/// accepts Stopped. It can leave DMA active on command failure.
pub(crate) type ControlScript =
    alloc::collections::VecDeque<(Vec<u8>, Vec<(crate::usb::bluetooth::Pipe, Vec<u8>)>)>;

/// How the model answers a command abort (xHCI 4.6.1.2).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum AbortReply {
    /// Command Aborted, then Command Ring Stopped.
    Aborted,
    /// The command completed first; then Command Ring Stopped.
    Completed,
    /// The command was never fetched; only Command Ring Stopped.
    Unfetched,
    /// CRR clears but no event follows.
    Unstopped,
    /// CRR never clears.
    Ignored,
}

pub(crate) struct CancelModel {
    pub(crate) port_speed: u8,
    pub(crate) context_bytes: usize,
    pub(crate) commands: u64,
    pub(crate) command_index: usize,
    pub(crate) events: u64,
    pub(crate) event_index: usize,
    pub(crate) output: u64,
    pub(crate) rings: [u64; 32],
    pub(crate) indices: [usize; 32],
    pub(crate) active: [Option<(u64, u64, usize)>; 32],
    pub(crate) connected: bool,
    pub(crate) fail: Option<(u32, u32)>,
    pub(crate) silence: Option<u32>,
    pub(crate) race_halt: bool,
    pub(crate) stale: Option<u64>,
    pub(crate) seen: Vec<u32>,
    pub(crate) dequeues: Vec<u64>,
    pub(crate) setup: Vec<([u8; 8], Vec<u8>)>,
    pub(crate) incoming_events: alloc::collections::VecDeque<Vec<u8>>,
    pub(crate) incoming_acl: alloc::collections::VecDeque<Vec<u8>>,
    /// Opt-in independent test peripherals, keyed by configured DCI.
    pub(crate) incoming_bulk:
        alloc::collections::BTreeMap<usize, alloc::collections::VecDeque<Vec<u8>>>,
    pub(crate) outgoing_bulk: alloc::collections::BTreeMap<usize, Vec<Vec<u8>>>,
    pub(crate) outgoing: Vec<Vec<u8>>,
    pub(crate) fatal_status: u32,
    pub(crate) status_reads: usize,
    pub(crate) fatal_on_command: Option<u32>,
    pub(crate) silence_transfer: Option<usize>,
    pub(crate) halt_transfer: Option<usize>,
    /// A silenced command, keeping CRCR.CRR set until it is aborted.
    pub(crate) stuck: Option<u64>,
    pub(crate) abort_reply: AbortReply,
    /// Transaction errors each endpoint reports before transferring.
    pub(crate) transaction_errors: [usize; 32],
    /// Endpoints reset with Transfer State Preserve, in order.
    pub(crate) soft_resets: Vec<usize>,
    pub(crate) script: Option<crate::usb::bluetooth::intel::tests::Script>,
    pub(crate) control_script: ControlScript,
    pub(crate) descriptors: Vec<(u8, u8, Vec<u8>)>,
    pub(crate) configuration_value: u8,
    pub(crate) configuration_seen_before_endpoints: Vec<u8>,
    pub(crate) short_control_out: bool,
    pub(crate) control_replies: alloc::collections::VecDeque<([u8; 8], Vec<u8>)>,
    pub(crate) stall_control: Option<u8>,
    pub(crate) silence_control: Option<u8>,
    /// The device stops answering EP0 from the control transfer at this
    /// index of [`Self::setup`] on, though it still takes each request.
    pub(crate) silence_control_from: Option<usize>,
    pub(crate) short_bulk_out: bool,
    pub(crate) ethernet: Option<ethernet::rtl815x::tests::Model>,
    pub(crate) ethernet_upload: Option<ethernet::rtl815x::FirmwareModel>,
    pub(crate) asix: Option<ethernet::AsixModel>,
    pub(crate) asix_legacy: Option<ethernet::LegacyModel>,
    pub(crate) asix_ticks: u64,
    /// Command completions and EP0 transfer events wait in [`Self::held`]
    /// until [`Self::release`]: the device answers late, after the call that
    /// asked has returned.
    pub(crate) late: bool,
    pub(crate) held: Vec<(u64, u32, u32, u8, usize)>,
    /// Runs the controller's one-tick-per-microsecond deadlines on virtual time.
    _clock: crate::test_clock::Guard,
}

impl CancelModel {
    pub(crate) fn state(&self, dci: usize) -> u32 {
        // SAFETY: output context is live aligned test DMA, exclusively
        // accessed by this model between controller operations.
        unsafe {
            ((self.output + dci as u64 * self.context_bytes as u64) as *const u32).read_volatile()
                & 7
        }
    }
    pub(crate) fn set_state(&mut self, dci: usize, state: u32) {
        // EP State occupies bits 2:0; hardware transitions keep the configured
        // Interval, Mult and stream-array fields in the rest of this word.
        // SAFETY: same live context as state().
        unsafe {
            let info = (self.output + dci as u64 * self.context_bytes as u64) as *mut u32;
            info.write_volatile(info.read_volatile() & !7 | state & 7);
        }
    }
    pub(crate) fn event(&mut self, parameter: u64, code: u32, kind: u32, dci: u8, residual: usize) {
        if self.late && (kind == COMMAND_COMPLETION || (kind == TRANSFER_EVENT && dci == 1)) {
            self.held.push((parameter, code, kind, dci, residual));
            return;
        }
        let cycle = u32::from((self.event_index / RING_TRBS).is_multiple_of(2));
        let address = self.events + ((self.event_index % RING_TRBS) * TRB) as u64;
        // SAFETY: all three fields are in the still-live test event ring.
        unsafe {
            (address as *mut u64).write_volatile(parameter);
            ((address + 8) as *mut u32).write_volatile(code << 24 | residual as u32);
            ((address + 12) as *mut u32)
                .write_volatile(1 << 24 | u32::from(dci) << 16 | kind << 10 | cycle);
        }
        self.event_index += 1;
    }
    /// The controller reports a Host System Error from here on.
    pub(crate) fn host_system_error(&mut self) {
        self.fatal_status = HSE;
    }
    /// Post the events held while [`Self::late`].
    pub(crate) fn release(&mut self) {
        let late = core::mem::replace(&mut self.late, false);
        for (parameter, code, kind, dci, residual) in core::mem::take(&mut self.held) {
            self.event(parameter, code, kind, dci, residual);
        }
        self.late = late;
    }
    /// The device sends interrupt-IN (DCI 3) data, bulk-IN (DCI 5) data or
    /// an independent peripheral's bulk-IN data, as soon as a TD waits.
    pub(crate) fn push_event(&mut self, bytes: Vec<u8>) {
        self.incoming_events.push_back(bytes);
        self.deliver();
    }
    pub(crate) fn extend_events(&mut self, packets: impl IntoIterator<Item = Vec<u8>>) {
        self.incoming_events.extend(packets);
        self.deliver();
    }
    pub(crate) fn push_acl(&mut self, bytes: Vec<u8>) {
        self.incoming_acl.push_back(bytes);
        self.deliver();
    }
    pub(crate) fn extend_bulk(&mut self, dci: usize, packets: impl IntoIterator<Item = Vec<u8>>) {
        self.incoming_bulk.entry(dci).or_default().extend(packets);
        self.deliver();
    }
    /// Moves queued IN data into every endpoint with a TD waiting.
    pub(crate) fn deliver(&mut self) {
        let registered: Vec<_> = self.incoming_bulk.keys().copied().collect();
        for dci in registered {
            self.receive(dci);
        }
        if self.asix.is_some() || self.asix_legacy.is_some() {
            for dci in (3..32).step_by(2) {
                self.receive(dci);
            }
        } else {
            self.receive(3);
            self.receive(5);
        }
    }
    pub(crate) fn receive(&mut self, dci: usize) {
        let queue = if let Some(queue) = self.incoming_bulk.get_mut(&dci) {
            assert!((3..32).contains(&dci) && !dci.is_multiple_of(2));
            queue
        } else if (self.asix.is_some() || self.asix_legacy.is_some())
            && dci >= 3
            && !dci.is_multiple_of(2)
        {
            &mut self.incoming_events
        } else {
            match dci {
                3 => &mut self.incoming_events,
                5 => &mut self.incoming_acl,
                _ => return,
            }
        };
        if self.active[dci].is_none() || queue.is_empty() {
            return;
        }
        let bytes = queue.pop_front().unwrap();
        let (trb, buffer, length) = self.active[dci].take().unwrap();
        assert!(bytes.len() <= length);
        // SAFETY: submitted TD names exactly this live DMA allocation;
        // only the model owns it until the completion it posts below.
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), buffer as *mut u8, bytes.len());
        }
        self.event(
            trb,
            SHORT_PACKET,
            TRANSFER_EVENT,
            dci as u8,
            length - bytes.len(),
        );
    }
    pub(crate) fn command(&mut self) {
        let at = self.commands + (self.command_index * TRB) as u64;
        // SAFETY: the controller has published this command before ringing.
        let (parameter, control) = unsafe {
            (
                (at as *const u64).read_volatile(),
                ((at + 12) as *const u32).read_volatile(),
            )
        };
        self.command_index += 1;
        let kind = control >> 10 & 0x3f;
        let dci = (control >> 16 & 0x1f) as usize;
        self.seen.push(kind);
        if self.silence == Some(kind) {
            self.stuck = Some(at);
            return;
        }
        let mut code = SUCCESS;
        if let Some((wanted, status)) = self.fail
            && kind == wanted
        {
            code = status;
        }
        if code == SUCCESS {
            match kind {
                CONFIGURE_ENDPOINT => {
                    let value = self.configuration_value;
                    self.configuration_seen_before_endpoints.push(value);
                    // SAFETY: Configure Endpoint points to the live input
                    // context, with enabled endpoint dequeue pointers.
                    let flags = unsafe { ((parameter + 4) as *const u32).read_volatile() };
                    for index in 2..32 {
                        if flags & (1 << index) != 0 {
                            // SAFETY: the model device, its pages and its domain belong to this
                            // test alone; no other driver, worker or DMA touches them.
                            self.rings[index] = unsafe {
                                ((parameter + (index as u64 + 1) * self.context_bytes as u64 + 8)
                                    as *const u64)
                                    .read_volatile()
                                    & !0xf
                            };
                            self.indices[index] = 0;
                            self.set_state(index, 1);
                        }
                    }
                }
                STOP_ENDPOINT => {
                    if self.race_halt {
                        self.race_halt = false;
                        self.set_state(dci, 2);
                        code = 19;
                    } else if self.state(dci) == 1 {
                        self.set_state(dci, 3);
                        self.active[dci] = None;
                    } else {
                        code = 19;
                    }
                }
                RESET_ENDPOINT => {
                    if self.state(dci) == 2 {
                        if control & transfer::TSP != 0 {
                            self.soft_resets.push(dci);
                        }
                        self.set_state(dci, 3);
                        self.active[dci] = None;
                    } else {
                        code = 19;
                    }
                }
                SET_DEQUEUE => {
                    if self.state(dci) != 3 {
                        code = 19;
                    } else {
                        self.dequeues.push(parameter);
                        self.indices[dci] = ((parameter & !0xf) - self.rings[dci]) as usize / TRB;
                        if let Some(stale) = self.stale.take() {
                            self.event(stale, 26, TRANSFER_EVENT, dci as u8, 0);
                        }
                    }
                }
                _ => {}
            }
        }
        if self.fatal_on_command == Some(kind) {
            self.fatal_status = HSE;
        }
        self.event(at, code, COMMAND_COMPLETION, 0, 0);
    }
    /// A command abort: the stuck command completes aborted, then the ring stops.
    pub(crate) fn abort(&mut self) {
        if self.abort_reply == AbortReply::Ignored {
            return;
        }
        let Some(at) = self.stuck.take() else {
            return;
        };
        match self.abort_reply {
            AbortReply::Aborted => self.event(at, COMMAND_ABORTED, COMMAND_COMPLETION, 0, 0),
            AbortReply::Completed => self.event(at, SUCCESS, COMMAND_COMPLETION, 0, 0),
            // The next doorbell fetches it again.
            AbortReply::Unfetched => self.command_index -= 1,
            AbortReply::Unstopped | AbortReply::Ignored => return,
        }
        self.event(0, COMMAND_RING_STOPPED, COMMAND_COMPLETION, 0, 0);
    }
    pub(crate) fn scripted(&mut self, bulk: bool, packet: &[u8]) {
        use crate::usb::bluetooth::Pipe;
        // Opt-in command-correlated peripheral: future replies must not be
        // visible during the mandatory settling poll of an earlier HCI
        // command. Standard zero-data USB requests do not carry HCI commands.
        if !bulk && !packet.is_empty() && !self.control_script.is_empty() {
            assert!(self.script.is_none(), "two HCI control peripherals");
            let (expected, replies) = self.control_script.pop_front().unwrap();
            assert_eq!(packet, expected, "actual EP0 command bytes");
            for (pipe, event) in replies {
                match pipe {
                    Pipe::Event => &mut self.incoming_events,
                    Pipe::Acl => &mut self.incoming_acl,
                }
                .push_back(event);
            }
            return;
        }
        if let Some(script) = &mut self.script {
            for (pipe, event) in script.command(bulk, packet) {
                match pipe {
                    Pipe::Event => &mut self.incoming_events,
                    Pipe::Acl => &mut self.incoming_acl,
                }
                .push_back(event);
            }
        }
    }
    pub(crate) fn legacy_request(&mut self, request: [u8; 8], output: &[u8]) -> Vec<u8> {
        // Controller Clock uses one test tick per microsecond.
        let ticks = Clock::ticks();
        let legacy = self.asix_legacy.as_mut().unwrap();
        if self.asix_ticks != 0 {
            legacy.delay(ticks - self.asix_ticks);
        }
        self.asix_ticks = ticks;
        legacy.request(request, output).unwrap()
    }
    pub(crate) fn transfer(&mut self, dci: usize) {
        self.indices[dci] %= RING_TRBS - 1;
        let at = self.rings[dci] + (self.indices[dci] * TRB) as u64;
        // SAFETY: a doorbell publishes a live transfer ring entry and any
        // data buffer encoded by that entry.
        let (parameter, length) = unsafe {
            (
                (at as *const u64).read_volatile(),
                ((at + 8) as *const u32).read_volatile() as usize & 0x1ffff,
            )
        };
        self.set_state(dci, 1);
        if self.transaction_errors[dci] > 0 {
            // The TD stays at the dequeue pointer of the halted endpoint.
            self.transaction_errors[dci] -= 1;
            self.set_state(dci, ENDPOINT_HALTED);
            self.active[dci] = Some((at, parameter, length));
            let code = transfer::USB_TRANSACTION_ERROR;
            self.event(at, code, TRANSFER_EVENT, dci as u8, length);
            return;
        }
        if self.halt_transfer == Some(dci) {
            // A STALL leaves the failed TD at the hardware dequeue pointer.
            // Reset stops it, but only Set Dequeue abandons it; another
            // doorbell could otherwise resume this same TD.
            self.set_state(dci, ENDPOINT_HALTED);
            self.active[dci] = Some((at, parameter, length));
            self.event(at, STALL, TRANSFER_EVENT, dci as u8, 0);
            return;
        }
        if dci == 1 {
            let request = parameter.to_le_bytes();
            let length = usize::from(u16::from_le_bytes([request[6], request[7]]));
            let mut received = length;
            let data = if length == 0 {
                if request[0] == 0 && request[1] == SET_CONFIGURATION {
                    self.configuration_value = request[2];
                }
                if request[0] == 0x40 && self.asix_legacy.is_some() {
                    self.legacy_request(request, &[]);
                }
                Vec::new()
            } else {
                // SAFETY: the next Data TRB's buffer is live through this
                // synchronous modeled control completion.
                let next =
                    self.rings[dci] + (((self.indices[dci] + 1) % (RING_TRBS - 1)) * TRB) as u64;
                // SAFETY: the pointer addresses this test's own live pages, and the stopped model
                // writes nothing concurrently.
                let buffer = unsafe { (next as *const u64).read_volatile() };
                if matches!(request[0], 0x80 | 0x81)
                    && request[1] == GET_DESCRIPTOR
                    && let Some((_, _, bytes)) = self
                        .descriptors
                        .iter()
                        .find(|(kind, index, _)| (*kind, *index) == (request[3], request[2]))
                {
                    received = bytes.len().min(length);
                    // SAFETY: buffer has the requested length and the
                    // device writes only that bounded response.
                    unsafe {
                        core::ptr::copy_nonoverlapping(bytes.as_ptr(), buffer as *mut u8, received);
                    }
                }
                if matches!(request[0], 0xa1 | 0x82) && !self.control_replies.is_empty() {
                    let (expected, reply) = self.control_replies.pop_front().unwrap();
                    assert_eq!(request, expected);
                    received = reply.len();
                    assert!(received <= length);
                    // SAFETY: this synchronous device reply fits the live
                    // Data TRB allocation, kept through completion.
                    unsafe {
                        core::ptr::copy_nonoverlapping(reply.as_ptr(), buffer as *mut u8, received);
                    }
                }
                if matches!(request[0], 0xc0 | 0x40)
                    && (self.asix.is_some()
                        || self.asix_legacy.is_some()
                        || (request[1] == 5
                            && (self.ethernet.is_some() || self.ethernet_upload.is_some())))
                {
                    let output = if request[0] == 0x40 {
                        // SAFETY: Data TRB refers to the live control buffer.
                        unsafe { core::slice::from_raw_parts(buffer as *const u8, length) }
                    } else {
                        &[]
                    };
                    let reply = if self.asix_legacy.is_some() {
                        self.legacy_request(request, output)
                    } else if let Some(asix) = &mut self.asix {
                        asix.request(request, output).unwrap()
                    } else if let Some(ethernet) = &mut self.ethernet {
                        ethernet.request(request, output).unwrap()
                    } else {
                        self.ethernet_upload
                            .as_mut()
                            .unwrap()
                            .request(request, output)
                            .unwrap()
                    };
                    if request[0] == 0xc0 {
                        received = reply.len();
                        assert!(received <= length);
                        // SAFETY: reply is bounded by this TD's requested buffer.
                        unsafe {
                            core::ptr::copy_nonoverlapping(
                                reply.as_ptr(),
                                buffer as *mut u8,
                                received,
                            );
                        }
                    }
                }
                if request[0] & 0x80 == 0 && self.short_control_out {
                    received = length - 1;
                }
                // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
                // nothing accesses it concurrently.
                unsafe { core::slice::from_raw_parts(buffer as *const u8, length).to_vec() }
            };
            if request[0] & 0x80 == 0 && request[1] != SET_CONFIGURATION {
                self.scripted(false, &data);
            }
            self.setup.push((request, data));
            let trbs = if length == 0 { 2 } else { 3 };
            if self.stall_control == Some(request[1]) {
                self.set_state(dci, ENDPOINT_HALTED);
                self.active[dci] = Some((at, 0, length));
                self.event(at, STALL, TRANSFER_EVENT, dci as u8, 0);
                return;
            }
            let silent_from = self
                .silence_control_from
                .is_some_and(|from| self.setup.len() > from);
            if self.silence_transfer == Some(dci)
                || self.silence_control == Some(request[1])
                || silent_from
            {
                self.active[dci] = Some((at, 0, length));
            } else {
                if received < length {
                    self.event(
                        self.rings[dci]
                            + (((self.indices[dci] + 1) % (RING_TRBS - 1)) * TRB) as u64,
                        SHORT_PACKET,
                        TRANSFER_EVENT,
                        1,
                        length - received,
                    );
                }
                self.event(
                    self.rings[dci]
                        + (((self.indices[dci] + trbs as usize - 1) % (RING_TRBS - 1)) * TRB)
                            as u64,
                    SUCCESS,
                    TRANSFER_EVENT,
                    1,
                    0,
                );
            }
            self.indices[dci] += trbs as usize;
        } else {
            self.indices[dci] += 1;
            if dci == 4
                || self.outgoing_bulk.contains_key(&dci)
                || ((self.asix.is_some() || self.asix_legacy.is_some())
                    && dci >= 2
                    && dci.is_multiple_of(2))
            {
                // SAFETY: the submitted bulk OUT buffer is live until the
                // completion below acknowledges its bytes.
                let packet =
                    unsafe { core::slice::from_raw_parts(parameter as *const u8, length).to_vec() };
                if let Some(queue) = self.outgoing_bulk.get_mut(&dci) {
                    assert!((2..32).contains(&dci) && dci.is_multiple_of(2));
                    queue.push(packet);
                } else {
                    self.scripted(true, &packet);
                    self.outgoing.push(packet);
                }
                if self.silence_transfer == Some(dci) {
                    self.active[dci] = Some((at, parameter, length));
                } else {
                    let short = self.short_bulk_out && length != 0;
                    self.event(
                        at,
                        if short { SHORT_PACKET } else { SUCCESS },
                        TRANSFER_EVENT,
                        dci as u8,
                        usize::from(short),
                    );
                }
            } else {
                self.active[dci] = Some((at, parameter, length));
            }
        }
    }
}

pub(crate) struct CancelRegisters(Rc<RefCell<CancelModel>>);

impl Registers for CancelRegisters {
    fn read32(&mut self, offset: usize) -> u32 {
        let mut model = self.0.borrow_mut();
        if offset == 0x40 + CRCR {
            return if model.stuck.is_some() {
                CRCR_RUNNING
            } else {
                0
            };
        }
        if offset == 0x40 + PORTSC {
            return PP
                | u32::from(model.port_speed) << PORT_SPEED_SHIFT
                | if model.connected { CCS | PED } else { 0 };
        }
        if offset == 0x40 + USBSTS {
            model.status_reads += 1;
            model.deliver();
            return model.fatal_status;
        }
        0
    }
    fn write32(&mut self, offset: usize, value: u32) {
        if offset == 0x40 + CRCR && value & CRCR_ABORT != 0 {
            self.0.borrow_mut().abort();
        } else if offset == 0x1000 {
            self.0.borrow_mut().command();
        } else if offset == 0x1004 {
            let mut model = self.0.borrow_mut();
            model.transfer(value as usize);
            // Waiting IN data meets the TD the doorbell announced.
            model.deliver();
        }
    }
}

pub(crate) fn unconfigured_cancellation_controller(
    dma: &mut dyn DmaAllocator,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    unconfigured_cancellation_controller_with_configurations(
        dma,
        &[vec![9, 2, 9, 0, 0, 1, 0, 0x80, 50]],
    )
}

pub(crate) fn unconfigured_cancellation_controller_with_configurations(
    dma: &mut dyn DmaAllocator,
    configurations: &[Vec<u8>],
) -> (Controller, Rc<RefCell<CancelModel>>) {
    cancellation_controller_at_speed(dma, configurations, FULL_SPEED)
}

fn cancellation_controller_at_speed(
    dma: &mut dyn DmaAllocator,
    configurations: &[Vec<u8>],
    speed: u8,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    cancellation_controller_with_capabilities(dma, configurations, speed, None)
}

fn cancellation_controller_with_capabilities(
    dma: &mut dyn DmaAllocator,
    configurations: &[Vec<u8>],
    speed: u8,
    capability_words: Option<[u32; 3]>,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    cancellation_controller_with_context(dma, configurations, speed, capability_words, 32)
}

/// Address Device and SET_CONFIGURATION acknowledgements for endpoint
/// peers that exercise 32- and 64-byte contexts. No generation, parent
/// identity or current grant is synthesized by the later endpoint-only peer.
pub(crate) fn configured_endpoint_controller(
    dma: &mut dyn DmaAllocator,
    speed: u8,
    context_bytes: usize,
) -> Controller {
    let (mut controller, peer) = cancellation_controller_with_context(
        dma,
        &[vec![9, 2, 9, 0, 0, 1, 0, 0x80, 0]],
        speed,
        None,
        context_bytes,
    );
    controller.set_configuration(1, 1).unwrap();
    assert_eq!(peer.borrow().configuration_value, 1);
    assert_eq!(
        peer.borrow().setup[0].0,
        setup(0, SET_CONFIGURATION, 1, 0, 0)
    );
    controller
}

fn cancellation_controller_with_context(
    dma: &mut dyn DmaAllocator,
    configurations: &[Vec<u8>],
    speed: u8,
    capability_words: Option<[u32; 3]>,
    context_bytes: usize,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    assert!(matches!(context_bytes, 32 | 64));
    let (mut controller, address_peer) =
        address_controller_with_capabilities(dma, SUCCESS, capability_words);
    controller.context_bytes = context_bytes;
    {
        let mut peer = address_peer.borrow_mut();
        peer.context_bytes = context_bytes;
        peer.port_speed = speed;
    }
    if speed >= SUPER_SPEED {
        controller.port_major[0] = 3;
    }
    let slot = controller
        .address(
            Attachment {
                root_port: 1,
                speed,
                route: 0,
                tier: 0,
                translator: None,
                parent: None,
            },
            dma,
        )
        .unwrap();
    assert_eq!(slot, 1);
    let state = controller.slots[1].as_ref().unwrap();
    let mut rings = [0; 32];
    rings[1] = state.endpoints[1].as_ref().unwrap().ring.buffer.physical();
    let model = Rc::new(RefCell::new(CancelModel {
        port_speed: speed,
        context_bytes,
        commands: controller.commands.buffer.physical(),
        command_index: controller.commands.index,
        events: controller.events.physical(),
        event_index: controller.event_index,
        output: state._output.physical(),
        rings,
        indices: [0; 32],
        active: [None; 32],
        connected: true,
        fail: None,
        silence: None,
        race_halt: false,
        stale: None,
        seen: Vec::new(),
        dequeues: Vec::new(),
        setup: Vec::new(),
        incoming_events: Default::default(),
        incoming_acl: Default::default(),
        incoming_bulk: Default::default(),
        outgoing_bulk: Default::default(),
        outgoing: Vec::new(),
        fatal_status: 0,
        status_reads: 0,
        fatal_on_command: None,
        silence_transfer: None,
        halt_transfer: None,
        stuck: None,
        abort_reply: AbortReply::Aborted,
        transaction_errors: [0; 32],
        soft_resets: Vec::new(),
        script: None,
        control_script: Default::default(),
        descriptors: Vec::new(),
        configuration_value: 0,
        configuration_seen_before_endpoints: Vec::new(),
        short_control_out: false,
        control_replies: Default::default(),
        stall_control: None,
        silence_control: None,
        silence_control_from: None,
        short_bulk_out: false,
        ethernet: None,
        ethernet_upload: None,
        asix: None,
        asix_legacy: None,
        asix_ticks: 0,
        late: false,
        held: Vec::new(),
        _clock: crate::test_clock::virtualize(),
    }));
    model.borrow_mut().set_state(1, 1);
    controller.registers = Box::new(CancelRegisters(model.clone()));
    // A short test deadline, on the model's virtual counter so preemption of
    // the test thread cannot expire it, without pretending any completion.
    controller.clock.ticks_per_microsecond = 1;
    if !configurations.is_empty() {
        // These are the fixture's original wire descriptor/current profiles;
        // production discovery keeps the descriptors before class binding.
        controller.current_descriptions(1, configurations).unwrap();
    }
    (controller, model)
}

/// Put the controller's minimum provider on the controller clock's own
/// calibration, so minimum intervals pass on the virtual time its deadlines
/// and a link's runtime calls count.
pub(crate) fn minimum_on_controller_clock(controller: &mut Controller) {
    controller.minimum = super::minimum::fixture(controller.clock);
}

pub(crate) fn cancellation_controller(
    dma: &mut dyn DmaAllocator,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    let (mut controller, model) = unconfigured_cancellation_controller(dma);
    controller.set_configuration(1, 1).unwrap();
    let endpoints = [
        Endpoint {
            address: 0x81,
            attributes: 3,
            max_packet: 16,
            interval: 1,
            companion: None,
        },
        Endpoint {
            address: 0x82,
            attributes: 2,
            max_packet: 64,
            interval: 0,
            companion: None,
        },
        Endpoint {
            address: 0x02,
            attributes: 2,
            max_packet: 64,
            interval: 0,
            companion: None,
        },
    ];
    controller.configure(1, &endpoints, None, dma).unwrap();
    model.borrow_mut().seen.clear();
    (controller, model)
}

/// Shared class-driver regression fixture: a real submitted TD left
/// unresolved by failed Running Stop or Halted Set Dequeue recovery.
pub(crate) fn controller_with_unresolved_td(
    dma: &mut dyn DmaAllocator,
    buffer: u64,
    length: usize,
    dci: u8,
    halted: bool,
) -> Shared {
    if halted {
        let (controller, _) = failed_stall_recovery(dma, buffer, length, dci);
        return Rc::new(RefCell::new(controller));
    }
    let (mut controller, model) = cancellation_controller(dma);
    model.borrow_mut().silence_transfer = Some(usize::from(dci));
    model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
    if dci == 1 {
        assert!(
            controller
                .control(1, setup(0x20, 0, 0, 0, 3), &[3, 0x0c, 0])
                .is_err()
        );
    } else {
        controller.submit(1, dci, buffer, length).unwrap();
        assert!(controller.cancel(1, dci).is_err());
    }
    assert!(model.borrow().active[usize::from(dci)].is_some());
    Rc::new(RefCell::new(controller))
}

pub(crate) fn fixture_endpoint(dci: u8) -> Endpoint {
    Endpoint {
        address: (dci / 2) | if dci & 1 != 0 { 0x80 } else { 0 },
        attributes: if dci == 3 { 3 } else { 2 },
        max_packet: if dci == 3 { 16 } else { 64 },
        interval: u8::from(dci == 3),
        companion: None,
    }
}

pub(crate) fn failed_stall_recovery(
    dma: &mut dyn DmaAllocator,
    buffer: u64,
    length: usize,
    dci: u8,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    let (mut controller, model) = cancellation_controller(dma);
    model.borrow_mut().halt_transfer = Some(usize::from(dci));
    model.borrow_mut().fail = Some((SET_DEQUEUE, CONTEXT_STATE_ERROR));
    let result = if dci == 1 {
        controller
            .control(1, setup(0x20, 0, 0, 0, 3), &[3, 0x0c, 0])
            .map(|_| ())
    } else {
        let trb = controller.submit(1, dci, buffer, length).unwrap();
        assert_eq!(
            controller.completed(1, dci, trb, length),
            Some(Err(BlockError::Device { status: STALL }))
        );
        controller.clear(1, &fixture_endpoint(dci))
    };
    assert_eq!(
        result,
        Err(BlockError::Device {
            status: CONTEXT_STATE_ERROR
        })
    );
    assert_eq!(model.borrow().seen, vec![RESET_ENDPOINT, SET_DEQUEUE]);
    assert_eq!(model.borrow().state(usize::from(dci)), ENDPOINT_STOPPED);
    // EP0 already completed the configuration request in this fixture.
    assert_eq!(
        model.borrow().indices[usize::from(dci)],
        if dci == 1 { 2 } else { 0 }
    );
    assert!(model.borrow().dequeues.is_empty());
    assert!(!controller.command_poisoned);
    (controller, model)
}

/// Compose a forwarding peer around the existing real EP0/command DMA model.
/// The caller supplies original descriptor bytes and a separately modeled PCI
/// function; stream admission uses the same observed ConfigSpace policy as a
/// native attachment. The wrapper observes the original rings, not fake tickets.
/// Existing fixture construction and defaults remain unchanged.
pub(crate) fn forwarding_fixture<C: crate::bus::pci::ConfigSpace>(
    dma: &mut dyn DmaAllocator,
    configurations: &[Vec<u8>],
    speed: u8,
    function: &mut Function<C>,
    wrap: impl FnOnce(Box<dyn Registers>, Rc<RefCell<CancelModel>>) -> Box<dyn Registers>,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    let (mut controller, model) = cancellation_controller_at_speed(dma, configurations, speed);
    controller.stream_policy = streams::pci_policy(function).unwrap();
    controller.registers = wrap(controller.registers, model.clone());
    (controller, model)
}

/// The interface version, HCSPARAMS2 and HCCPARAMS1 a fixture's
/// capability registers report.
pub(crate) struct CapabilityImage {
    pub(crate) version: u16,
    pub(crate) hcs2: u32,
    pub(crate) hcc1: u32,
}

/// A forwarding peer whose capability-register image is installed and read
/// before the original Controller captures its immutable scheduling policy.
/// The existing fixture above keeps its no-CFC/default capability path.
pub(crate) fn forwarding_scheduled_fixture<C: crate::bus::pci::ConfigSpace>(
    dma: &mut dyn DmaAllocator,
    configurations: &[Vec<u8>],
    speed: u8,
    function: &mut Function<C>,
    capabilities: CapabilityImage,
    wrap: impl FnOnce(Box<dyn Registers>, Rc<RefCell<CancelModel>>) -> Box<dyn Registers>,
) -> (Controller, Rc<RefCell<CancelModel>>) {
    let CapabilityImage {
        version,
        hcs2,
        hcc1,
    } = capabilities;
    let (mut controller, model) = cancellation_controller_with_capabilities(
        dma,
        configurations,
        speed,
        Some([u32::from(version) << 16 | 0x40, hcs2, hcc1]),
    );
    controller.stream_policy = streams::pci_policy(function).unwrap();
    controller.registers = wrap(controller.registers, model.clone());
    (controller, model)
}

/// Same failed ordinary-TD fixture with its unique submitted ticket
/// by asynchronous class owners instead of an invented ring address.
pub(crate) fn controller_with_unresolved_ticket(
    dma: &mut dyn DmaAllocator,
    buffer: u64,
    length: usize,
    dci: u8,
    halted: bool,
) -> (Shared, TransferTicket) {
    let (mut controller, model) = cancellation_controller(dma);
    if halted {
        model.borrow_mut().halt_transfer = Some(usize::from(dci));
        model.borrow_mut().fail = Some((SET_DEQUEUE, CONTEXT_STATE_ERROR));
    } else {
        model.borrow_mut().silence_transfer = Some(usize::from(dci));
        model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
    }
    let id = controller.device_id(1).unwrap();
    let ticket = controller
        .device(id)
        .unwrap()
        .submit_ticket(dci, buffer, length)
        .unwrap();
    if halted {
        assert_eq!(
            controller.completed_ticket(&ticket),
            Some(Err(BlockError::Device { status: STALL }))
        );
        assert_eq!(
            controller.device(id).unwrap().clear(&fixture_endpoint(dci)),
            Err(BlockError::Device {
                status: CONTEXT_STATE_ERROR
            })
        );
        assert_eq!(model.borrow().seen, vec![RESET_ENDPOINT, SET_DEQUEUE]);
        assert_eq!(model.borrow().state(usize::from(dci)), ENDPOINT_STOPPED);
    } else {
        assert!(controller.cancel_ticket(&ticket).is_err());
        assert!(model.borrow().active[usize::from(dci)].is_some());
    }
    assert!(controller.endpoint_poisoned(1, usize::from(dci)));
    (Rc::new(RefCell::new(controller)), ticket)
}

/// High-speed class profile, with the original configuration.
pub(crate) fn high_speed_media_controller(
    dma: &mut dyn DmaAllocator,
    configuration: &[u8],
) -> (Controller, Rc<RefCell<CancelModel>>) {
    cancellation_controller_at_speed(dma, &[configuration.to_vec()], HIGH_SPEED)
}

/// Status fixture: a device-side endpoint halt persists until the EP0
/// CLEAR_FEATURE request is accepted. The underlying peer still consumes real
/// contexts/TRBs and posts the control transfer completion.
pub(crate) fn status_interrupt_controller(
    dma: &mut dyn DmaAllocator,
    configuration: &[u8],
) -> (Controller, Rc<RefCell<CancelModel>>) {
    struct StatusRegisters {
        inner: Box<dyn Registers>,
        peer: Rc<RefCell<CancelModel>>,
    }
    impl Registers for StatusRegisters {
        fn read32(&mut self, offset: usize) -> u32 {
            self.inner.read32(offset)
        }
        fn write32(&mut self, offset: usize, value: u32) {
            self.inner.write32(offset, value);
            if offset == 0x1004 && value == 1 {
                let mut peer = self.peer.borrow_mut();
                if peer
                    .setup
                    .last()
                    .is_some_and(|(request, _)| *request == setup(2, CLEAR_FEATURE, 0, 0x81, 0))
                    && peer.stall_control != Some(CLEAR_FEATURE)
                    && peer.silence_control != Some(CLEAR_FEATURE)
                    && peer.silence_transfer != Some(1)
                {
                    peer.halt_transfer = None;
                }
            }
        }
    }
    let (mut controller, peer) = high_speed_media_controller(dma, configuration);
    controller.registers = Box::new(StatusRegisters {
        inner: controller.registers,
        peer: peer.clone(),
    });
    (controller, peer)
}
