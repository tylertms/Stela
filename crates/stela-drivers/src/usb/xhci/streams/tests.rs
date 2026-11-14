extern crate std;
use super::*;
use crate::bus::dma::{RegionAllocator, TestRegion};
use alloc::collections::BTreeMap;

struct Cursor {
    base: u64,
    index: usize,
    cycle: u32,
}
struct DeviceTd {
    trb: u64,
    buffer: u64,
    bytes: usize,
}
struct Hardware {
    command_base: u64,
    command_index: usize,
    event_base: u64,
    event_index: usize,
    output: u64,
    context_bytes: usize,
    cursors: BTreeMap<(u8, u16), Cursor>,
    active: BTreeMap<(u8, u16), DeviceTd>,
    doorbells: Vec<(u8, u16)>,
    commands: Vec<(u32, u16, u64)>,
    fail_command: Option<(u32, u16)>,
    silent_command: Option<u32>,
    /// A silenced command, keeping CRCR.CRR set until it is aborted.
    stuck: Option<u64>,
    fatal: bool,
    cadence: bool,
    /// Reset Endpoint commands with Transfer State Preserve.
    soft_resets: usize,
}
impl Hardware {
    fn word(address: u64) -> u32 {
        // SAFETY: the model only accesses kept, aligned allocator
        // ranges published in the real controller's contexts/TRBs.
        unsafe { (address as *const u32).read_volatile() }
    }
    fn long(address: u64) -> u64 {
        // SAFETY: same published live test allocation as word().
        unsafe { (address as *const u64).read_volatile() }
    }
    fn state(&self, dci: u8) -> u32 {
        Self::word(self.output + u64::from(dci) * self.context_bytes as u64) & 7
    }
    fn set_state(&mut self, dci: u8, state: u32) {
        let address = self.output + u64::from(dci) * self.context_bytes as u64;
        // SAFETY: the complete output context is live through this test.
        unsafe { (address as *mut u32).write_volatile(Self::word(address) & !7 | state) };
    }
    fn event(&mut self, trb: u64, code: u32, kind: u32, dci: u8, residual: usize) {
        let address = self.event_base + ((self.event_index % RING_TRBS) * TRB) as u64;
        let cycle = u32::from((self.event_index / RING_TRBS).is_multiple_of(2));
        // SAFETY: the test event ring remains allocated; ownership is
        // published in control last, exactly as a real controller does.
        unsafe {
            (address as *mut u64).write_volatile(trb);
            ((address + 8) as *mut u32).write_volatile(code << 24 | residual as u32);
            ((address + 12) as *mut u32)
                .write_volatile(1 << 24 | u32::from(dci) << 16 | kind << 10 | cycle);
        }
        self.event_index += 1;
    }
    /// A command abort: the stuck command completes aborted, then the ring stops.
    fn abort(&mut self) {
        if let Some(at) = self.stuck.take() {
            self.event(at, COMMAND_ABORTED, COMMAND_COMPLETION, 0, 0);
            self.event(0, COMMAND_RING_STOPPED, COMMAND_COMPLETION, 0, 0);
        }
    }
    fn command(&mut self) {
        let address = self.command_base + (self.command_index * TRB) as u64;
        let parameter = Self::long(address);
        let status = Self::word(address + 8);
        let control = Self::word(address + 12);
        let kind = control >> 10 & 0x3f;
        let dci = (control >> 16 & 0x1f) as u8;
        let stream = (status >> 16) as u16;
        self.commands.push((kind, stream, parameter));
        self.command_index = (self.command_index + 1) % (RING_TRBS - 1);
        if self.silent_command == Some(kind) {
            self.stuck = Some(address);
            return;
        }
        let mut code = if self.fail_command == Some((kind, stream)) {
            17
        } else {
            SUCCESS
        };
        if code == SUCCESS {
            match kind {
                CONFIGURE_ENDPOINT => {
                    let added = Self::word(parameter + 4);
                    for ep in 2..32u8 {
                        if added & (1 << ep) == 0 {
                            continue;
                        }
                        let from = parameter + (u64::from(ep) + 1) * self.context_bytes as u64;
                        let to = self.output + u64::from(ep) * self.context_bytes as u64;
                        // SAFETY: separate kept input/output contexts,
                        // validated by the real Configure Endpoint path.
                        unsafe {
                            core::ptr::copy_nonoverlapping(
                                from as *const u8,
                                to as *mut u8,
                                self.context_bytes,
                            );
                        }
                        self.set_state(ep, ENDPOINT_RUNNING);
                    }
                }
                STOP_ENDPOINT => {
                    if self.state(dci) != ENDPOINT_RUNNING {
                        code = CONTEXT_STATE_ERROR;
                    } else {
                        self.set_state(dci, ENDPOINT_STOPPED);
                    }
                }
                RESET_ENDPOINT => {
                    if self.state(dci) != ENDPOINT_HALTED {
                        code = CONTEXT_STATE_ERROR;
                    } else {
                        self.soft_resets += usize::from(control & transfer::TSP != 0);
                        self.set_state(dci, ENDPOINT_STOPPED);
                    }
                }
                SET_DEQUEUE => {
                    if self.state(dci) != ENDPOINT_STOPPED {
                        code = CONTEXT_STATE_ERROR;
                    } else {
                        assert_ne!(stream, 0);
                        assert_eq!(parameter & 0xe, PRIMARY_TRANSFER_RING);
                        let cursor = self.cursors.get_mut(&(dci, stream));
                        if let Some(cursor) = cursor {
                            cursor.index = ((parameter & !0xf) - cursor.base) as usize / TRB;
                            cursor.cycle = parameter as u32 & 1;
                        } else {
                            let at = self.output + u64::from(dci) * self.context_bytes as u64;
                            let array = Self::long(at + 8);
                            let base = Self::long(array + u64::from(stream) * 16) & !0xf;
                            self.cursors.insert(
                                (dci, stream),
                                Cursor {
                                    base,
                                    index: ((parameter & !0xf) - base) as usize / TRB,
                                    cycle: parameter as u32 & 1,
                                },
                            );
                        }
                        if self.cadence {
                            let at = self.output + u64::from(dci) * self.context_bytes as u64;
                            let array = Self::long(at + 8);
                            // SAFETY: stream array in the
                            // output endpoint context; emulate its quirk.
                            unsafe {
                                ((array + u64::from(stream) * 16 + 8) as *mut u32)
                                    .write_volatile(0xcafef00d);
                                ((array + u64::from(stream) * 16 + 12) as *mut u32)
                                    .write_volatile(0xdeadbeef);
                            }
                        }
                        // A stopped-TD event precedes the final command ACK;
                        // the real cancellation must drain it before reuse.
                        if let Some(td) = self.active.remove(&(dci, stream)) {
                            self.event(td.trb, 26, TRANSFER_EVENT, dci, td.bytes);
                        }
                    }
                }
                _ => panic!("unexpected command {kind}"),
            }
        }
        self.event(address, code, COMMAND_COMPLETION, 0, 0);
    }
    fn transfer(&mut self, dci: u8, stream: u16) {
        if self.active.contains_key(&(dci, stream)) {
            // A preserved TD resumes where it stopped.
            self.set_state(dci, ENDPOINT_RUNNING);
            self.doorbells.push((dci, stream));
            return;
        }
        let context = self.output + u64::from(dci) * self.context_bytes as u64;
        let info = Self::word(context);
        let initial = if info & LINEAR_STREAM_ARRAY != 0 {
            let contexts = 1u32 << ((info >> 10 & 0x1f) + 1);
            assert!(stream != 0 && u32::from(stream) < contexts);
            let array = Self::long(context + 8);
            assert!(array.is_multiple_of(16));
            assert_eq!(Self::long(array), 0);
            let initial = Self::long(array + u64::from(stream) * 16);
            assert_eq!(initial & 0xe, PRIMARY_TRANSFER_RING);
            if self.cadence {
                assert_eq!(
                    Self::long(array + u64::from(stream) * 16 + 8),
                    0,
                    "Cadence state was not cleared after acknowledged dequeue"
                );
            }
            initial
        } else {
            assert_eq!(stream, 0);
            Self::long(context + 8)
        };
        let cursor = self.cursors.entry((dci, stream)).or_insert(Cursor {
            base: initial & !0xf,
            index: 0,
            cycle: initial as u32 & 1,
        });
        let buffer = Self::long(cursor.base + (cursor.index * TRB) as u64);
        let mut bytes = 0;
        let trb = loop {
            let trb = cursor.base + (cursor.index * TRB) as u64;
            let control = Self::word(trb + 12);
            assert_eq!(control & CYCLE, cursor.cycle, "unpublished/stale TD");
            assert_eq!(control >> 10 & 0x3f, NORMAL);
            let chained = control & CHAIN != 0;
            assert_eq!(control & IOC != 0, !chained);
            let segment = Self::long(trb);
            let length = (Self::word(trb + 8) & TRB_LENGTH_MASK) as usize;
            assert_eq!(segment, buffer + bytes as u64);
            assert!(length <= MAX_TD && segment % MAX_TD as u64 + length as u64 <= MAX_TD as u64);
            bytes += length;
            cursor.index += 1;
            if cursor.index == RING_TRBS - 1 {
                let link = cursor.base + ((RING_TRBS - 1) * TRB) as u64;
                let link_control = Self::word(link + 12);
                assert_eq!(Self::long(link), cursor.base);
                assert_eq!(link_control >> 10 & 0x3f, LINK);
                assert_eq!(link_control & CYCLE, cursor.cycle);
                assert_ne!(link_control & TOGGLE, 0);
                assert_eq!(link_control & CHAIN != 0, chained);
                cursor.index = 0;
                cursor.cycle ^= 1;
            }
            if !chained {
                break trb;
            }
            assert_eq!(bytes, MAX_TD);
        };
        assert!(
            self.active
                .insert((dci, stream), DeviceTd { trb, buffer, bytes })
                .is_none()
        );
        self.set_state(dci, ENDPOINT_RUNNING);
        self.doorbells.push((dci, stream));
    }
    /// Halts on a transaction error, keeping the TD at the dequeue pointer.
    fn transaction_error(&mut self, dci: u8, stream: u16) {
        let (trb, bytes) = {
            let td = &self.active[&(dci, stream)];
            (td.trb, td.bytes)
        };
        self.set_state(dci, ENDPOINT_HALTED);
        self.event(
            trb,
            transfer::USB_TRANSACTION_ERROR,
            TRANSFER_EVENT,
            dci,
            bytes,
        );
    }
    fn complete(&mut self, dci: u8, stream: u16, code: u32, residual: usize) {
        let td = self.active.remove(&(dci, stream)).unwrap();
        if matches!(code, SUCCESS | SHORT_PACKET) && residual <= td.bytes {
            let n = td.bytes - residual;
            if dci & 1 != 0 {
                // SAFETY: the real submitted DMA range is held; only device model
                // accesses it before the event below.
                unsafe { core::ptr::write_bytes(td.buffer as *mut u8, stream as u8, n) };
            }
        }
        if code == STALL {
            self.set_state(dci, ENDPOINT_HALTED);
        }
        self.event(td.trb, code, TRANSFER_EVENT, dci, residual);
    }
}
struct Model(Rc<RefCell<Hardware>>);
impl Registers for Model {
    fn read32(&mut self, offset: usize) -> u32 {
        if offset == 0x40 + USBSTS && self.0.borrow().fatal {
            HSE
        } else if offset == 0x40 + CRCR && self.0.borrow().stuck.is_some() {
            CRCR_RUNNING
        } else {
            0
        }
    }
    fn write32(&mut self, offset: usize, value: u32) {
        match offset {
            0x58 if value & CRCR_ABORT != 0 => self.0.borrow_mut().abort(),
            0x1000 => self.0.borrow_mut().command(),
            0x1004 => self
                .0
                .borrow_mut()
                .transfer(value as u8, (value >> 16) as u16),
            _ => {}
        }
    }
}
fn endpoint(address: u8, exponent: u8) -> Endpoint {
    Endpoint {
        address,
        attributes: 2,
        max_packet: 1024,
        interval: 0,
        companion: Some(SuperSpeedCompanion {
            max_burst: 7,
            attributes: exponent,
            bytes_per_interval: 0,
        }),
    }
}
fn controller(
    dma: &mut dyn DmaAllocator,
    context_bytes: usize,
) -> (Controller, Rc<RefCell<Hardware>>) {
    let commands = Ring::new(dma).unwrap();
    let events = dma.allocate(RING_TRBS * TRB, RING_TRBS * TRB).unwrap();
    let output = dma.allocate(32 * context_bytes, 64).unwrap();
    let model = Rc::new(RefCell::new(Hardware {
        command_base: commands.buffer.physical(),
        command_index: 0,
        event_base: events.physical(),
        event_index: 0,
        output: output.physical(),
        context_bytes,
        cursors: BTreeMap::new(),
        active: BTreeMap::new(),
        doorbells: Vec::new(),
        commands: Vec::new(),
        fail_command: None,
        silent_command: None,
        stuck: None,
        fatal: false,
        cadence: false,
        soft_resets: 0,
    }));
    let mut slots = vec![None];
    slots.push(Some(Slot {
        generation: 0,
        parent_id: None,
        inventory: None,
        consumer: identity::Consumer::Unclaimed,
        suspended: false,
        attachment: Attachment {
            root_port: 1,
            route: 0,
            tier: 0,
            speed: SUPER_SPEED,
            translator: None,
            parent: None,
        },
        retiring: false,
        input: dma.allocate(33 * context_bytes, 64).unwrap().into(),
        _output: output.into(),
        context: [u32::from(SUPER_SPEED) << 20 | 1 << 27, 1 << 16, 0, 0],
        endpoints: Default::default(),
        retired_rings: Default::default(),
        control: dma.allocate(CONTROL_BYTES, CONTROL_BYTES).unwrap().into(),
        speed: SUPER_SPEED,
        current_configuration: Some(1),
    }));
    (
        Controller {
            minimum: crate::usb::xhci::minimum::fixture(Clock {
                ticks_per_microsecond: 1,
            }),
            native_memory: None,
            registers: Box::new(Model(model.clone())),
            clock: Clock {
                ticks_per_microsecond: 1,
            },
            operational: 0x40,
            runtime: 0x800,
            doorbells: 0x1000,
            context_bytes,
            version: 0x100,
            dma_profile: dma::Profile::SPEC,
            hardware: hardware::State::Running,
            quirks: quirks::Profile::default(),
            max_primary_streams: 16,
            stream_policy: Policy::Standard,
            iso_caps: iso::Capabilities::new(0x100, 0, 0),
            max_slots: 1,
            max_ports: 1,
            port_major: vec![3],
            lpm: lpm::Host::default(),
            dcbaa: dma.allocate(16, 64).unwrap().into(),
            _scratchpads: Vec::new(),
            commands,
            events: events.into(),
            _event_table: dma.allocate(16, 64).unwrap().into(),
            event_index: 0,
            event_cycle: true,
            retry_resources: slot_resources::Cache::new(slots.len()),
            queued_resets: Vec::new(),
            after_boot: false,
            slot_ownership: vec![reset::Ownership::Vacant, reset::Ownership::Enabled],
            test_retired_numbers: vec![false; 2],
            power: power::Ledger::new(2),
            slots,
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
            location: String::from("stream model"),
        },
        model,
    )
}

fn addressed_controller(
    dma: &mut dyn DmaAllocator,
    context_bytes: usize,
) -> (Controller, Rc<RefCell<Hardware>>) {
    let mut controller = model::configured_endpoint_controller(dma, SUPER_SPEED, context_bytes);
    let id = controller.device_id(1).unwrap();
    controller.validate_id(id).unwrap();
    let model = Rc::new(RefCell::new(Hardware {
        command_base: controller.commands.buffer.physical(),
        command_index: controller.commands.index,
        event_base: controller.events.physical(),
        event_index: controller.event_index + if controller.event_cycle { 0 } else { RING_TRBS },
        output: controller.slots[1].as_ref().unwrap()._output.physical(),
        context_bytes,
        cursors: BTreeMap::new(),
        active: BTreeMap::new(),
        doorbells: Vec::new(),
        commands: Vec::new(),
        fail_command: None,
        silent_command: None,
        stuck: None,
        fatal: false,
        cadence: false,
        soft_resets: 0,
    }));
    controller.registers = Box::new(Model(model.clone()));
    controller.max_primary_streams = 16;
    controller.stream_policy = Policy::Standard;
    (controller, model)
}

#[test]
fn retired_device_stream_ticket_is_pure_before_readiness_and_preserves_current_dma() {
    struct NoAccess;
    impl Registers for NoAccess {
        fn read32(&mut self, _: usize) -> u32 {
            panic!("retired stream ticket reached MMIO read")
        }
        fn write32(&mut self, _: usize, _: u32) {
            panic!("retired stream ticket reached MMIO write")
        }
    }
    for (context_bytes, already_pending) in [(32, false), (64, true)] {
        let mut backing = vec![0u64; 256 * 1024];
        let start = backing.as_mut_ptr() as u64;
        // SAFETY: all modeled DMA remains in this live, disjoint allocation.
        let mut dma = unsafe { RegionAllocator::new(start..start + backing.len() as u64 * 8) };
        let (mut previous, _) = addressed_controller(&mut dma, context_bytes);
        let (mut current, model) = addressed_controller(&mut dma, context_bytes);
        for c in [&mut previous, &mut current] {
            c.configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma)
                .unwrap();
        }
        let data = dma.allocate(128, 64).unwrap();
        let mut old = previous
            .submit_stream(1, 3, 1, data.physical(), 64)
            .unwrap();
        let fresh = current
            .submit_stream(1, 3, 1, data.physical() + 64, 64)
            .unwrap();
        let current_id = current.device_id(1).unwrap();
        // Distinct modeled generations share hardware slot 1. This tests
        // ticket admission, not the separate architectural slot-reuse proof.
        assert_eq!(old.hardware_slot(), fresh.hardware_slot());
        assert_ne!(old.device_id(), current_id);
        assert_eq!(fresh.device_id(), current_id);
        model.borrow_mut().complete(3, 1, SUCCESS, 0);
        if already_pending {
            current.drain();
        }
        let pending = |c: &Controller| {
            c.pending
                .iter()
                .map(|event| (event.parameter, event.status, event.control))
                .collect::<Vec<_>>()
        };
        let pending_before = pending(&current);
        assert_eq!(pending_before.len(), usize::from(already_pending));
        let memory_before = backing.clone();
        let cursor_before = (current.event_index, current.event_cycle);
        let endpoint = current.slots[1].as_mut().unwrap().endpoints[3]
            .as_mut()
            .unwrap();
        let streams = endpoint.streams.as_mut().unwrap();
        let active = streams.stream(1).unwrap().active.as_ref().unwrap();
        let active_before = (active.id, active.td, active.error.clone());
        let registers = core::mem::replace(&mut current.registers, Box::new(NoAccess));
        for (dci, unhealthy) in [(3, false), (u8::MAX, false), (3, true), (u8::MAX, true)] {
            old.dci = dci;
            current.error = unhealthy.then_some(HSE);
            current.command_poisoned = unhealthy;
            let retired = BlockError::Unsupported("xHCI device generation is permanently retired");
            assert_eq!(current.completed_stream(&old), Some(Err(retired)));
            assert_eq!(
                current.device(current_id).unwrap().completed_stream(&old),
                Some(Err(BlockError::Unsupported("foreign USB stream ticket")))
            );
            assert_eq!(current.error, unhealthy.then_some(HSE));
            assert_eq!(current.command_poisoned, unhealthy);
        }
        current.slots[1].as_mut().unwrap().suspended = true;
        assert_eq!(
            current.completed_stream(&old),
            Some(Err(BlockError::Unsupported(
                "xHCI device generation is permanently retired"
            )))
        );
        assert!(matches!(
            current.submit_stream(1, 3, 1, data.physical(), 64),
            Err(BlockError::Unsupported(
                "USB consumer is fenced for device lifecycle"
            ))
        ));
        current.slots[1].as_mut().unwrap().suspended = false;
        assert_eq!(pending(&current), pending_before);
        assert_eq!((current.event_index, current.event_cycle), cursor_before);
        assert!(backing == memory_before);
        let endpoint = current.slots[1].as_mut().unwrap().endpoints[3]
            .as_mut()
            .unwrap();
        let streams = endpoint.streams.as_mut().unwrap();
        assert!(!streams.recovery);
        let active = streams.stream(1).unwrap().active.as_ref().unwrap();
        assert_eq!((active.id, active.td, active.error.clone()), active_before);
        current.registers = registers;
        current.error = None;
        current.command_poisoned = false;
        assert_eq!(current.completed_stream(&fresh), Some(Ok(64)));
    }
}

#[test]
fn primary_arrays_match_limits_and_keep_zero_unused_contexts() {
    let endpoints = [endpoint(0x81, 4), endpoint(2, 3), endpoint(0x83, 0)];
    for maximum in [4, 8, 16, 65536] {
        for requested in [1, 2, 3, 4, 7, 8, 9, 65535] {
            let layout =
                Layout::new(maximum, SUPER_SPEED, &endpoints, &[0x81, 2], requested).unwrap();
            assert_eq!(
                u32::from(layout.usable),
                u32::from(requested).min(8).min(maximum - 1)
            );
            assert_eq!(1 << ((layout.info() >> 10 & 0x1f) + 1), layout.contexts);
            let mut dma = TestRegion::new(64 * 1024 * 8);
            let streams = Streams::new(&mut dma, layout, dma::Profile::SPEC, false).unwrap();
            for index in 0..layout.contexts as usize {
                let value = streams.array.read::<u64>(index * 16);
                if index == 0 || index > usize::from(layout.usable) {
                    assert_eq!(value, 0);
                } else {
                    assert_eq!(
                        value,
                        streams.rings[index - 1].ring.dequeue_pointer() | PRIMARY_TRANSFER_RING
                    );
                    assert_eq!(streams.array.read::<u64>(index * 16 + 8), 0);
                }
            }
        }
    }
}

#[test]
fn real_contexts_independent_dma_completions_and_stream_rings_wrap() {
    for (context_bytes, overfetch) in [(32, false), (64, false), (32, true), (64, true)] {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, context_bytes);
        if overfetch {
            controller.dma_profile = dma::Profile::pci(0x1d17, 0x9202);
        }
        let endpoints = [endpoint(0x81, 4), endpoint(2, 4)];
        assert_eq!(
            controller.configure_streams(1, &endpoints, &[0x81, 2], 2, &mut dma),
            Ok(2)
        );
        for endpoint in controller.slots[1]
            .as_ref()
            .unwrap()
            .endpoints
            .iter()
            .flatten()
        {
            assert_eq!(
                endpoint.ring.buffer.len(),
                controller.dma_profile.segment_bytes()
            );
            for stream in &endpoint.streams.as_ref().unwrap().rings {
                assert_eq!(
                    stream.ring.buffer.len(),
                    controller.dma_profile.segment_bytes()
                );
                assert!(stream.ring.buffer.bytes()[4096..].iter().all(|b| *b == 0));
            }
        }
        let a = dma.allocate(64, 64).unwrap();
        let b = dma.allocate(64, 64).unwrap();
        assert!(controller.submit(1, 3, a.physical(), 64).is_err());
        assert!(controller.submit_stream(1, 3, 0, a.physical(), 64).is_err());
        assert!(controller.submit_stream(1, 3, 3, a.physical(), 64).is_err());
        let mut old = None;
        for index in 0..600 {
            let first = controller.submit_stream(1, 3, 1, a.physical(), 64).unwrap();
            let second = controller.submit_stream(1, 3, 2, b.physical(), 64).unwrap();
            assert!(controller.submit_stream(1, 3, 1, a.physical(), 64).is_err());
            assert_eq!(controller.completed_stream(&first), None);
            if let Some(ticket) = &old {
                assert!(matches!(controller.completed_stream(ticket), Some(Err(_))));
            }
            model.borrow_mut().complete(3, 2, SHORT_PACKET, 7);
            assert_eq!(controller.completed_stream(&first), None);
            assert_eq!(controller.completed_stream(&second), Some(Ok(57)));
            assert_eq!(&b.bytes()[..57], &[2; 57]);
            model.borrow_mut().complete(3, 1, SUCCESS, 0);
            if let Some(ticket) = &old {
                assert!(matches!(controller.completed_stream(ticket), Some(Err(_))));
            }
            assert_eq!(controller.completed_stream(&first), Some(Ok(64)));
            assert_eq!(a.bytes(), &[1; 64]);
            if index == 0 {
                old = Some(first);
            }
        }
        assert!(
            model
                .borrow()
                .doorbells
                .iter()
                .all(|&(dci, stream)| dci == 3 && (1..=2).contains(&stream))
        );
        let out = controller.submit_stream(1, 4, 2, a.physical(), 64).unwrap();
        model.borrow_mut().complete(4, 2, SUCCESS, 0);
        assert_eq!(controller.completed_stream(&out), Some(Ok(64)));
    }
}

#[test]
fn ordinary_and_stream_submissions_reject_high_payloads_before_ring_mutation() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller
        .configure_streams(
            1,
            &[endpoint(0x81, 4), endpoint(2, 0)],
            &[0x81],
            2,
            &mut dma,
        )
        .unwrap();
    // These test epochs model invalid caller payloads; production profiles
    // are immutable and applied during controller construction.
    for (profile, address) in [
        (dma::Profile::pci(0x1b21, 0x3242), 1u64 << 32),
        (dma::Profile::pci(0x1106, 0x3483), 1u64 << 36),
    ] {
        controller.dma_profile = profile;
        let stream = controller.slots[1].as_ref().unwrap().endpoints[3]
            .as_ref()
            .unwrap()
            .streams
            .as_ref()
            .unwrap()
            .rings[0]
            .ring
            .buffer
            .bytes()
            .to_vec();
        let ordinary = controller.slots[1].as_ref().unwrap().endpoints[4]
            .as_ref()
            .unwrap()
            .ring
            .buffer
            .bytes()
            .to_vec();
        let doorbells = model.borrow().doorbells.len();
        assert!(controller.submit_stream(1, 3, 1, address, 64).is_err());
        assert!(controller.submit(1, 4, address, 64).is_err());
        let slot = controller.slots[1].as_ref().unwrap();
        assert_eq!(
            slot.endpoints[3]
                .as_ref()
                .unwrap()
                .streams
                .as_ref()
                .unwrap()
                .rings[0]
                .ring
                .buffer
                .bytes(),
            stream
        );
        assert_eq!(
            slot.endpoints[4].as_ref().unwrap().ring.buffer.bytes(),
            ordinary
        );
        assert_eq!(model.borrow().doorbells.len(), doorbells);
    }
}

#[test]
fn cancel_retires_every_stream_then_rejects_stale_tickets_and_events() {
    for halted in [false, true] {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32);
        controller
            .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 3, &mut dma)
            .unwrap();
        let data = dma.allocate(256, 64).unwrap();
        let mut tickets = Vec::new();
        for id in 1..=3 {
            tickets.push(
                controller
                    .submit_stream(1, 3, id, data.physical() + u64::from(id - 1) * 64, 64)
                    .unwrap(),
            );
        }
        if halted {
            model.borrow_mut().complete(3, 2, STALL, 64);
            assert_eq!(
                controller.completed_stream(&tickets[1]),
                Some(Err(BlockError::Device { status: STALL }))
            );
            assert!(
                controller
                    .submit_stream(1, 3, 2, data.physical(), 64)
                    .is_err()
            );
        }
        controller.cancel(1, 3).unwrap();
        assert!(model.borrow().active.is_empty());
        assert!(controller.pending.iter().all(|event| event.endpoint() != 3));
        let commands = model.borrow().commands.clone();
        let set: Vec<_> = commands
            .iter()
            .filter(|c| c.0 == SET_DEQUEUE)
            .map(|c| c.1)
            .collect();
        assert_eq!(set, vec![1, 2, 3]);
        assert_eq!(
            commands
                .iter()
                .filter(|c| c.0
                    == if halted {
                        RESET_ENDPOINT
                    } else {
                        STOP_ENDPOINT
                    })
                .count(),
            1
        );
        for ticket in &tickets {
            assert!(matches!(controller.completed_stream(ticket), Some(Err(_))));
        }
        let fresh = controller
            .submit_stream(1, 3, 1, data.physical(), 64)
            .unwrap();
        assert_eq!(controller.completed_stream(&fresh), None);
        model.borrow_mut().complete(3, 1, SUCCESS, 0);
        assert_eq!(controller.completed_stream(&fresh), Some(Ok(64)));
    }
}

fn submit_wrapped_chain(
    controller: &mut Controller,
    hardware: &RefCell<Hardware>,
    buffer: &DmaBuffer,
) -> StreamTransfer {
    let dci = 3;
    let stream = 1;
    let device = controller.device_id(1).unwrap();
    controller.endpoint_ready_owned(1, dci).unwrap();
    let streams = controller.slot(1).unwrap().endpoints[usize::from(dci)]
        .as_mut()
        .unwrap()
        .streams
        .as_mut()
        .unwrap();
    let ring = &mut streams.stream(stream).unwrap().ring;
    ring.index = RING_TRBS - 2;
    let base = ring.buffer.device_address();
    hardware.borrow_mut().cursors.insert(
        (dci, stream),
        Cursor {
            base,
            index: ring.index,
            cycle: u32::from(ring.cycle),
        },
    );
    let mut access = controller.device(device).unwrap();
    let ticket = access
        .submit_chained_stream_read(dci, stream, buffer, CHAINED_READ_BYTES)
        .unwrap();
    assert_eq!(ticket.td.last, base);
    assert_eq!(ticket.td.first, base + ((RING_TRBS - 2) * TRB) as u64);
    assert_eq!(
        Hardware::word(ticket.td.first + 8),
        MAX_TD as u32 | 31 << TD_SIZE_SHIFT,
    );
    ticket
}

#[test]
fn chained_stream_admission_requires_newer_input_stream_and_bounded_dma() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    let mut bad = endpoint(0x81, 4);
    bad.max_packet = 512;
    assert!(
        controller
            .configure_streams(1, &[bad], &[0x81], 2, &mut dma)
            .is_err()
    );
    controller
        .configure_streams(
            1,
            &[endpoint(0x81, 4), endpoint(0x02, 4), endpoint(0x83, 4)],
            &[0x81, 0x02],
            2,
            &mut dma,
        )
        .unwrap();
    let device = controller.device_id(1).unwrap();
    let full = dma.allocate(CHAINED_READ_BYTES, MAX_TD).unwrap();
    let short = dma.allocate(512, 64).unwrap();
    let misaligned = dma.allocate(MAX_TD + 1, 64).unwrap();
    assert!(!misaligned.bus_address().is_multiple_of(MAX_TD as u64));
    controller.version = 0x96;
    let mut access = controller.device(device).unwrap();
    assert_eq!(access.chained_stream_read_capacity(3), Ok(None));
    assert!(
        access
            .submit_chained_stream_read(3, 1, &full, MAX_TD + 1)
            .is_err()
    );
    controller.version = 0x100;
    let mut access = controller.device(device).unwrap();
    assert_eq!(
        access.chained_stream_read_capacity(3),
        Ok(Some(CHAINED_READ_BYTES))
    );
    for dci in [0, 4, 7, 32] {
        assert_eq!(access.chained_stream_read_capacity(dci), Ok(None));
        assert!(
            access
                .submit_chained_stream_read(dci, 1, &full, MAX_TD + 1)
                .is_err()
        );
    }
    for (stream, buffer, length) in [
        (0, &full, MAX_TD + 1),
        (1, &full, MAX_TD),
        (1, &full, CHAINED_READ_BYTES + 1),
        (1, &short, MAX_TD + 1),
        (1, &misaligned, MAX_TD + 1),
    ] {
        assert!(
            access
                .submit_chained_stream_read(3, stream, buffer, length)
                .is_err()
        );
    }
    assert!(model.borrow().doorbells.is_empty());
    let ticket = access
        .submit_chained_stream_read(3, 1, &full, MAX_TD + 1)
        .unwrap();
    assert_eq!(model.borrow().doorbells, [(3, 1)]);
    assert_eq!(
        Hardware::word(ticket.td.first + 8),
        MAX_TD as u32 | 1 << TD_SIZE_SHIFT
    );
    assert_eq!(Hardware::word(ticket.td.last + 8), 1);
    model.borrow_mut().complete(3, 1, SUCCESS, 0);
    assert_eq!(access.completed_stream(&ticket), Some(Ok(MAX_TD + 1)));
}

#[test]
fn chained_stream_tail_is_exact_and_reports_total_residual() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller
        .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma)
        .unwrap();
    let data = dma.allocate(2 * MAX_TD, MAX_TD).unwrap();
    let sibling_data = dma.allocate(64, 64).unwrap();
    assert!(
        controller
            .submit_stream(1, 3, 1, data.physical(), 2 * MAX_TD)
            .is_err()
    );
    let ticket = submit_wrapped_chain(&mut controller, &model, &data);
    let sibling = controller
        .submit_stream(1, 3, 2, sibling_data.physical(), 32)
        .unwrap();
    model
        .borrow_mut()
        .event(ticket.td.first + TRB as u64, SUCCESS, TRANSFER_EVENT, 3, 0);
    model.borrow_mut().complete(3, 2, SUCCESS, 0);
    assert_eq!(controller.completed_stream(&ticket), None);
    assert_eq!(controller.completed_stream(&sibling), Some(Ok(32)));
    model.borrow_mut().complete(3, 1, SHORT_PACKET, 511);
    assert_eq!(
        controller.completed_stream(&ticket),
        Some(Ok(2 * MAX_TD - 511))
    );
    assert!(matches!(controller.completed_stream(&ticket), Some(Err(_))));
}

#[test]
fn chained_stream_uncertain_events_require_acknowledged_retirement() {
    for (code, residual, transferless, failed_ack) in [
        (SHORT_PACKET, 37, false, false),
        (SUCCESS, MAX_TD + 1, false, true),
        (transfer::USB_TRANSACTION_ERROR, 37, false, false),
        (INVALID_STREAM_ID, 0, true, false),
    ] {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32);
        controller
            .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma)
            .unwrap();
        let data = dma.allocate(2 * MAX_TD, MAX_TD).unwrap();
        let ticket = submit_wrapped_chain(&mut controller, &model, &data);
        {
            let mut hardware = model.borrow_mut();
            let address = if transferless { 0 } else { ticket.td.first };
            hardware.event(address, code, TRANSFER_EVENT, 3, residual);
            hardware.event(ticket.td.last, SUCCESS, TRANSFER_EVENT, 3, 0);
        }
        let first = controller.completed_stream(&ticket);
        assert!(matches!(first, Some(Err(_))));
        assert_eq!(controller.completed_stream(&ticket), first);
        assert!(
            controller
                .submit_stream(1, 3, 2, data.physical(), 32)
                .is_err()
        );
        assert_eq!(model.borrow().doorbells.len(), 1);
        assert_eq!(model.borrow().soft_resets, 0);
        if failed_ack {
            model.borrow_mut().fail_command = Some((SET_DEQUEUE, 1));
        }
        assert_eq!(
            controller.cancel_native_stream(&ticket).is_err(),
            failed_ack
        );
        let state = controller.slot(1).unwrap().endpoints[3].as_ref().unwrap();
        assert_eq!(
            state.streams.as_ref().unwrap().rings[0].active.is_some(),
            failed_ack
        );
        if !failed_ack {
            assert!(controller.pending.iter().all(|event| event.endpoint() != 3));
            let fresh = controller
                .submit_stream(1, 3, 1, data.physical(), 32)
                .unwrap();
            assert_eq!(fresh.td.first, ticket.td.last + TRB as u64);
            assert!(matches!(controller.completed_stream(&ticket), Some(Err(_))));
            model.borrow_mut().complete(3, 1, SUCCESS, 0);
            assert_eq!(controller.completed_stream(&fresh), Some(Ok(32)));
        }
    }
}

#[test]
fn stream_transaction_errors_soft_retry_every_pending_stream_like_linux() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller
        .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 3, &mut dma)
        .unwrap();
    let data = dma.allocate(256, 64).unwrap();
    let one = controller
        .submit_stream(1, 3, 1, data.physical(), 64)
        .unwrap();
    let two = controller
        .submit_stream(1, 3, 2, data.physical() + 64, 64)
        .unwrap();
    model.borrow_mut().doorbells.clear();
    // Each retry resets with TSP and rings both pending streams again.
    for _ in 0..4 {
        model.borrow_mut().transaction_error(3, 2);
        assert_eq!(controller.completed_stream(&two), None);
    }
    assert_eq!(model.borrow().soft_resets, 4);
    assert_eq!(model.borrow().doorbells, [(3, 1), (3, 2)].repeat(4));
    model.borrow_mut().complete(3, 2, SUCCESS, 0);
    assert_eq!(controller.completed_stream(&two), Some(Ok(64)));
    // The success cleared err_count; the fifth error then fails.
    for _ in 0..4 {
        model.borrow_mut().transaction_error(3, 1);
        assert_eq!(controller.completed_stream(&one), None);
    }
    model.borrow_mut().transaction_error(3, 1);
    let error = BlockError::Device {
        status: transfer::USB_TRANSACTION_ERROR,
    };
    assert_eq!(controller.completed_stream(&one), Some(Err(error)));
    assert!(
        controller
            .submit_stream(1, 3, 2, data.physical(), 64)
            .is_err()
    );
    // The failed TD is retired by a hard reset, without TSP.
    controller.cancel(1, 3).unwrap();
    assert_eq!(model.borrow().soft_resets, 8);
    assert!(model.borrow().active.is_empty());
}

/// A stream soft reset the controller answers late is stepped by any
/// stream's poll, each returning at once; once it completes, every stream
/// with a TD then active is rung, one submitted meanwhile included, and the
/// retried TD completes.
#[test]
fn a_late_stream_soft_reset_is_stepped_by_every_stream_poll() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller
        .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 3, &mut dma)
        .unwrap();
    let data = dma.allocate(256, 64).unwrap();
    let one = controller
        .submit_stream(1, 3, 1, data.physical(), 64)
        .unwrap();
    model.borrow_mut().silent_command = Some(RESET_ENDPOINT);
    model.borrow_mut().transaction_error(3, 1);
    for _ in 0..3 {
        let before = Clock::ticks();
        assert_eq!(controller.completed_stream(&one), None);
        assert!(Clock::ticks() - before < 10_000, "a poll waited");
    }
    let two = controller
        .submit_stream(1, 3, 2, data.physical() + 64, 64)
        .unwrap();
    assert_eq!(controller.completed_stream(&two), None);
    model.borrow_mut().doorbells.clear();
    {
        let mut model = model.borrow_mut();
        model.silent_command = None;
        let at = model.stuck.take().unwrap();
        model.soft_resets += 1;
        model.set_state(3, ENDPOINT_STOPPED);
        model.event(at, SUCCESS, COMMAND_COMPLETION, 0, 0);
    }
    assert_eq!(controller.completed_stream(&two), None);
    assert_eq!(model.borrow().doorbells, [(3, 1), (3, 2)]);
    let resets = model
        .borrow()
        .commands
        .iter()
        .filter(|c| c.0 == RESET_ENDPOINT)
        .count();
    assert_eq!((resets, model.borrow().soft_resets), (1, 1));
    model.borrow_mut().complete(3, 1, SUCCESS, 0);
    assert_eq!(controller.completed_stream(&one), Some(Ok(64)));
    assert!(controller.finished.is_empty());
}

#[test]
fn transferless_stream_errors_reset_the_endpoint_like_linux() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller
        .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 3, &mut dma)
        .unwrap();
    let data = dma.allocate(256, 64).unwrap();
    let one = controller
        .submit_stream(1, 3, 1, data.physical(), 64)
        .unwrap();
    model.borrow_mut().doorbells.clear();
    let resets = |model: &Rc<RefCell<Hardware>>| {
        let model = model.borrow();
        let all = model
            .commands
            .iter()
            .filter(|c| c.0 == RESET_ENDPOINT)
            .count();
        (all - model.soft_resets, model.soft_resets)
    };
    // Errors naming no stream ring: four soft resets, each ringing the
    // pending stream, then hard resets until a success.
    for (code, hard, soft) in [
        (transfer::USB_TRANSACTION_ERROR, 0, 1),
        (STALL, 0, 2),
        (INVALID_STREAM_ID, 0, 3),
        (INVALID_STREAM_TYPE, 0, 4),
        (transfer::USB_TRANSACTION_ERROR, 1, 4),
        (transfer::USB_TRANSACTION_ERROR, 2, 4),
    ] {
        model.borrow_mut().set_state(3, ENDPOINT_HALTED);
        model.borrow_mut().event(0, code, TRANSFER_EVENT, 3, 0);
        assert_eq!(controller.completed_stream(&one), None);
        assert_eq!(resets(&model), (hard, soft));
    }
    assert_eq!(model.borrow().doorbells, [(3, 1); 4]);
    // Underruns and unknown events naming no ring are dropped.
    model
        .borrow_mut()
        .event(0, RING_UNDERRUN, TRANSFER_EVENT, 3, 0);
    model.borrow_mut().event(0, SUCCESS, TRANSFER_EVENT, 5, 0);
    model.borrow_mut().event(0, SUCCESS, TRANSFER_EVENT, 3, 0);
    assert_eq!(controller.completed_stream(&one), None);
    assert_eq!(resets(&model), (2, 4));
    assert!(controller.pending.iter().all(|e| e.endpoint() != 3));
    assert!(controller.pending.iter().any(|e| e.endpoint() == 5));
    // A success clears err_count: the next error is soft again.
    model.borrow_mut().set_state(3, ENDPOINT_RUNNING);
    model.borrow_mut().complete(3, 1, SUCCESS, 0);
    assert_eq!(controller.completed_stream(&one), Some(Ok(64)));
    let two = controller
        .submit_stream(1, 3, 2, data.physical() + 64, 64)
        .unwrap();
    model.borrow_mut().set_state(3, ENDPOINT_HALTED);
    model.borrow_mut().event(0, STALL, TRANSFER_EVENT, 3, 0);
    assert_eq!(controller.completed_stream(&two), None);
    assert_eq!(resets(&model), (2, 5));
}

#[test]
fn partial_dequeue_failure_keeps_every_stream_and_blocks_all_mutation() {
    for fail_stream in 1..=3 {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32);
        controller
            .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 3, &mut dma)
            .unwrap();
        let data = dma.allocate(256, 64).unwrap();
        let mut tickets = Vec::new();
        for id in 1..=3 {
            tickets.push(
                controller
                    .submit_stream(1, 3, id, data.physical() + u64::from(id - 1) * 64, 64)
                    .unwrap(),
            );
        }
        // An idle stream must also remain closed after another stream's
        // uncertain cancellation; active-record busy checks are insufficient.
        model.borrow_mut().complete(3, 3, SUCCESS, 0);
        assert_eq!(controller.completed_stream(&tickets[2]), Some(Ok(64)));
        model.borrow_mut().fail_command = Some((SET_DEQUEUE, fail_stream));
        assert!(controller.cancel(1, 3).is_err());
        let endpoint = controller.slots[1].as_ref().unwrap().endpoints[3]
            .as_ref()
            .unwrap();
        assert!(endpoint.poisoned);
        let streams = endpoint.streams.as_ref().unwrap();
        assert!(streams.rings[..2].iter().all(|s| s.active.is_some()));
        assert!(streams.rings[2].active.is_none());
        let bytes: Vec<_> = streams
            .rings
            .iter()
            .map(|s| s.ring.buffer.bytes().to_vec())
            .collect();
        let doorbells = model.borrow().doorbells.len();
        for id in 1..=3 {
            assert!(
                controller
                    .submit_stream(1, 3, id, data.physical(), 64)
                    .is_err()
            );
        }
        assert!(controller.cancel(1, 3).is_err());
        assert_eq!(model.borrow().doorbells.len(), doorbells);
        let endpoint = controller.slots[1].as_ref().unwrap().endpoints[3]
            .as_ref()
            .unwrap();
        for (stream, before) in endpoint.streams.as_ref().unwrap().rings.iter().zip(bytes) {
            assert_eq!(stream.ring.buffer.bytes(), before);
        }
    }
}

#[test]
fn invalid_limits_and_advertisements_refuse_before_context_publication() {
    let mut dma = TestRegion::new(128 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    for (maximum, speed, exponent, selected, requested) in [
        (2, SUPER_SPEED, 4, vec![0x81], 2),
        (3, SUPER_SPEED, 4, vec![0x81], 2),
        (16, HIGH_SPEED, 4, vec![0x81], 2),
        (16, SUPER_SPEED, 0, vec![0x81], 2),
        (16, SUPER_SPEED, 17, vec![0x81], 2),
        (16, SUPER_SPEED, 4, vec![0x81, 0x81], 2),
        (16, SUPER_SPEED, 4, vec![0x82], 2),
        (16, SUPER_SPEED, 4, vec![0x81], 0),
    ] {
        controller.max_primary_streams = maximum;
        controller.slots[1].as_mut().unwrap().speed = speed;
        assert!(
            controller
                .configure_streams(
                    1,
                    &[endpoint(0x81, exponent)],
                    &selected,
                    requested,
                    &mut dma
                )
                .is_err()
        );
        assert!(model.borrow().commands.is_empty());
        assert!(controller.slots[1].as_ref().unwrap().endpoints[3].is_none());
    }
}

#[test]
fn uncertain_configuration_and_invalid_residual_never_release_streams() {
    for configure_error in [false, true] {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32);
        if configure_error {
            model.borrow_mut().fail_command = Some((CONFIGURE_ENDPOINT, 0));
        }
        let result = controller.configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma);
        let data = dma.allocate(64, 64).unwrap();
        if configure_error {
            assert!(result.is_err());
            assert!(
                controller
                    .submit_stream(1, 3, 1, data.physical(), 64)
                    .is_err()
            );
            let endpoint = controller.slots[1].as_ref().unwrap().endpoints[3]
                .as_ref()
                .unwrap();
            assert!(!endpoint.enabled && endpoint.poisoned && endpoint.streams.is_some());
        } else {
            result.unwrap();
            let ticket = controller
                .submit_stream(1, 3, 1, data.physical(), 64)
                .unwrap();
            model.borrow_mut().complete(3, 1, SUCCESS, 65);
            assert!(matches!(
                controller.completed_stream(&ticket),
                Some(Err(BlockError::Unsupported(_)))
            ));
            assert!(
                controller.slots[1].as_ref().unwrap().endpoints[3]
                    .as_ref()
                    .unwrap()
                    .poisoned
            );
            assert!(
                controller
                    .submit_stream(1, 3, 2, data.physical(), 64)
                    .is_err()
            );
        }
    }
}

#[test]
#[ignore = "oracle: STELA_XHCI_STREAMS_ORACLE"]
fn unchanged_linux_primary_size_context_and_wire_encodings() {
    use sha2::{Digest, Sha256};
    let path =
        std::env::var("STELA_XHCI_STREAMS_ORACLE").expect("path to immutable source-helper trace");
    let trace = std::fs::read_to_string(path).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
        "204f854062e00199edf3c744ccc6f282876c8051f87282372710d6af1289f36f"
    );
    let mut count = 0;
    for line in trace.lines() {
        let mut parts = line.split_whitespace();
        let kind = parts.next().unwrap();
        let n: Vec<u64> = parts.map(|s| s.parse().unwrap()).collect();
        match kind {
            "S" => {
                assert_eq!(n.len(), 8);
                let endpoints = [
                    endpoint(0x81, n[1] as u8),
                    endpoint(2, (n[1] + 4).min(16) as u8),
                ];
                let layout = Layout::new(
                    1 << (n[0] + 1),
                    SUPER_SPEED,
                    &endpoints,
                    &[0x81, 2],
                    n[2] as u16,
                )
                .unwrap();
                assert_eq!(u64::from(layout.usable), n[3], "{line}");
                assert_eq!(u64::from(layout.contexts), n[4], "{line}");
                assert_eq!(u64::from(0x12800000 | layout.info()), n[5], "{line}");
                assert_eq!(n[6], 0x123456780);
                assert_eq!(n[7], 1 << endpoints[0].dci() | 1 << endpoints[1].dci());
            }
            "E" => {
                assert_eq!(n.len(), 5);
                assert_eq!(u64::from(doorbell_value(n[0] as u8, n[1] as u16)), n[2]);
                assert_eq!(u64::from(dequeue_status(n[1] as u16)), n[3]);
                assert_eq!(PRIMARY_TRANSFER_RING, n[4]);
            }
            _ => panic!("unknown oracle record"),
        }
        count += 1;
    }
    assert_eq!(count, 4560);
}

#[test]
fn status_and_data_streams_are_posted_before_ordinary_uas_command() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 64);
    let endpoints = [
        endpoint(0x81, 4),
        endpoint(0x82, 4),
        endpoint(3, 4),
        endpoint(4, 0),
    ];
    controller
        .configure_streams(1, &endpoints, &[0x81, 0x82, 3], 2, &mut dma)
        .unwrap();
    let data = dma.allocate(256, 64).unwrap();
    let status = controller
        .submit_stream(1, 3, 1, data.physical(), 16)
        .unwrap();
    let receive = controller
        .submit_stream(1, 5, 1, data.physical() + 64, 64)
        .unwrap();
    let send = controller
        .submit_stream(1, 6, 1, data.physical() + 128, 64)
        .unwrap();
    let command = controller.submit(1, 8, data.physical() + 192, 32).unwrap();
    assert_eq!(
        model.borrow().doorbells,
        vec![(3, 1), (5, 1), (6, 1), (8, 0)]
    );
    // Completion order is independent of command/doorbell order.
    model.borrow_mut().complete(3, 1, SHORT_PACKET, 0);
    assert_eq!(controller.completed_stream(&status), Some(Ok(16)));
    assert_eq!(controller.completed_stream(&receive), None);
    model.borrow_mut().complete(8, 0, SUCCESS, 0);
    assert_eq!(controller.completed(1, 8, command, 32), Some(Ok(32)));
    model.borrow_mut().complete(6, 1, SUCCESS, 0);
    assert_eq!(controller.completed_stream(&send), Some(Ok(64)));
    model.borrow_mut().complete(5, 1, SHORT_PACKET, 5);
    assert_eq!(controller.completed_stream(&receive), Some(Ok(59)));
}

#[test]
fn foreign_ticket_cannot_consume_another_controllers_pending_completion() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut first, one) = controller(&mut dma, 32);
    let (mut second, two) = controller(&mut dma, 32);
    for controller in [&mut first, &mut second] {
        controller
            .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma)
            .unwrap();
    }
    let data = dma.allocate(128, 64).unwrap();
    let a = first.submit_stream(1, 3, 1, data.physical(), 64).unwrap();
    let b = second
        .submit_stream(1, 3, 1, data.physical() + 64, 64)
        .unwrap();
    two.borrow_mut().complete(3, 1, SUCCESS, 0);
    assert!(matches!(second.completed_stream(&a), Some(Err(_))));
    assert_eq!(second.completed_stream(&b), Some(Ok(64)));
    one.borrow_mut().complete(3, 1, SUCCESS, 0);
    assert_eq!(first.completed_stream(&a), Some(Ok(64)));
}

#[test]
fn each_stream_allocation_failure_precedes_every_hardware_context_change() {
    struct Fail<'a> {
        inner: &'a mut dyn DmaAllocator,
        at: usize,
        count: usize,
    }
    impl DmaAllocator for Fail<'_> {
        fn allocate(&mut self, bytes: usize, align: usize) -> Result<DmaBuffer, BlockError> {
            let at = self.count;
            self.count += 1;
            if at == self.at {
                Err(BlockError::NoDmaMemory)
            } else {
                self.inner.allocate(bytes, align)
            }
        }
    }
    for fail_at in 0..13 {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32);
        let input = controller.slots[1].as_ref().unwrap().input.bytes().to_vec();
        let endpoints = [
            endpoint(0x81, 4),
            endpoint(0x82, 4),
            endpoint(3, 4),
            endpoint(4, 0),
        ];
        let mut fail = Fail {
            inner: &mut dma,
            at: fail_at,
            count: 0,
        };
        assert!(matches!(
            controller.configure_streams(1, &endpoints, &[0x81, 0x82, 3], 2, &mut fail),
            Err(BlockError::NoDmaMemory)
        ));
        assert_eq!(fail.count, fail_at + 1);
        assert!(model.borrow().commands.is_empty());
        let slot = controller.slots[1].as_ref().unwrap();
        assert!(slot.endpoints.iter().all(Option::is_none));
        assert_eq!(slot.input.bytes(), input);
    }
}

#[test]
fn command_timeout_and_32bit_dma_refusal_keep_unacknowledged_streams_inaccessible() {
    let mut backing = vec![0u64; 256 * 1024];
    let start = backing.as_mut_ptr() as u64;
    // SAFETY: all allocations kept even after uncertain command completion.
    let mut dma = unsafe { RegionAllocator::new(start..start + backing.len() as u64 * 8) };
    let (mut controller, model) = controller(&mut dma, 32);
    model.borrow_mut().silent_command = Some(CONFIGURE_ENDPOINT);
    assert_eq!(
        controller.configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma),
        Err(BlockError::Timeout)
    );
    let transfer = controller.slots[1].as_ref().unwrap().endpoints[3]
        .as_ref()
        .unwrap();
    assert!(!transfer.enabled && transfer.poisoned && transfer.streams.is_some());
    // The aborted command leaves the command ring usable.
    assert!(!controller.command_poisoned);
    let data = dma.allocate(64, 64).unwrap();
    assert!(
        controller
            .submit_stream(1, 3, 1, data.physical(), 64)
            .is_err()
    );
    // The hosted allocation's pointer is above the narrow hardware
    // aperture; it must never be truncated into an apparently valid address.
    if start > u64::from(u32::MAX) {
        let layout = Layout::new(16, SUPER_SPEED, &[endpoint(0x81, 4)], &[0x81], 2).unwrap();
        assert!(
            Streams::new(
                &mut dma,
                layout,
                dma::Profile::SPEC.capabilities(false),
                false
            )
            .is_err()
        );
    }
}

#[test]
fn pci_identity_binds_exact_stream_quirks_without_writes() {
    use crate::bus::pci::Address;
    struct Config {
        id: u32,
        class: u32,
    }
    impl ConfigSpace for Config {
        fn read32(&self, at: u16) -> core::result::Result<u32, crate::bus::pci::ConfigError> {
            Ok({
                match at {
                    0 => self.id,
                    8 => self.class,
                    _ => 0,
                }
            })
        }
        fn write32(
            &mut self,
            _: u16,
            _: u32,
        ) -> core::result::Result<(), crate::bus::pci::ConfigError> {
            panic!("preflight must never mutate PCI");
        }

        fn read8(&self, offset: u16) -> core::result::Result<u8, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
        }
        fn write8(
            &mut self,
            _: u16,
            _: u8,
        ) -> core::result::Result<(), crate::bus::pci::ConfigError> {
            panic!("unexpected narrow configuration write")
        }
        fn read16(&self, offset: u16) -> core::result::Result<u16, crate::bus::pci::ConfigError> {
            Ok((self.read32(offset & !3)? >> ((offset & 2) * 8)) as u16)
        }
        fn write16(
            &mut self,
            _: u16,
            _: u16,
        ) -> core::result::Result<(), crate::bus::pci::ConfigError> {
            panic!("unexpected narrow configuration write")
        }
    }
    for (vendor, device, expected) in [
        (0x8086u16, 0x1e31u16, Some(Policy::Standard)),
        (0x1b36, 0x000d, Some(Policy::Standard)),
        (0x1b73, 0x1000, Some(Policy::Broken)),
        (0x1b73, 0x1009, Some(Policy::Broken)),
        (0x1b6f, 0x7023, Some(Policy::Broken)),
        (0x1b6f, 0x7052, Some(Policy::Broken)),
        (0x1106, 0x3432, Some(Policy::Broken)),
        (0x1b21, 0x1042, Some(Policy::Broken)),
        (0x17cd, 0x0200, Some(Policy::Cadence)),
        (0x1b21, 0x1142, Some(Policy::Standard)),
        (0x1b21, 0x1242, Some(Policy::Standard)),
        (0x1b21, 0x2142, Some(Policy::Standard)),
        (0x1b21, 0x3242, Some(Policy::Standard)),
        (0x1106, 0x3483, Some(Policy::Standard)),
        (0x1d17, 0x9202, Some(Policy::Standard)),
        (0x1d17, 0x9203, Some(Policy::Standard)),
    ] {
        let address = Address {
            segment: 0,
            bus: 0,
            device: 1,
            function: 0,
        };
        let id = u32::from(vendor) | u32::from(device) << 16;
        let mut function = Function::new(
            address,
            Config {
                id,
                class: 0x0c033001,
            },
        )
        .unwrap();
        let result = pci_policy(&mut function);
        match expected {
            Some(policy) => assert_eq!(result, Ok(policy)),
            None => assert!(result.is_err()),
        }
        function.config().id ^= 1 << 16;
        assert!(pci_policy(&mut function).is_err());
        function.config().id = id;
        function.config().class ^= 1 << 8;
        assert!(pci_policy(&mut function).is_err());
        function.config().class = 0x0c033001;
        function.vendor ^= 1;
        assert!(pci_policy(&mut function).is_err());
    }
}

#[test]
fn pure_capacity_query_never_publishes_and_unknown_or_broken_identity_refuses() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller.slots[1].as_mut().unwrap().current_configuration = Some(0);
    let endpoints = [endpoint(0x81, 4), endpoint(2, 3)];
    assert_eq!(
        controller.stream_capacity(1, &endpoints, &[0x81, 2], 32),
        Ok(8)
    );
    assert!(
        controller.slots[1]
            .as_ref()
            .unwrap()
            .endpoints
            .iter()
            .all(Option::is_none)
    );
    assert!(model.borrow().commands.is_empty() && model.borrow().doorbells.is_empty());
    for policy in [Policy::Unknown, Policy::Broken] {
        controller.stream_policy = policy;
        assert!(
            controller
                .stream_capacity(1, &endpoints, &[0x81, 2], 32)
                .is_err()
        );
        assert!(
            controller
                .configure_streams(1, &endpoints, &[0x81, 2], 32, &mut dma)
                .is_err()
        );
    }
    assert!(model.borrow().commands.is_empty());
}

#[test]
fn cadence_reserved_state_is_cleared_after_each_acknowledged_stream_dequeue() {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32);
    controller.stream_policy = Policy::pci(0x17cd, 0x0200);
    model.borrow_mut().cadence = true;
    controller
        .configure_streams(1, &[endpoint(0x81, 4)], &[0x81], 2, &mut dma)
        .unwrap();
    let data = dma.allocate(128, 64).unwrap();
    for id in 1..=2 {
        controller
            .submit_stream(1, 3, id, data.physical() + u64::from(id - 1) * 64, 64)
            .unwrap();
    }
    controller.cancel(1, 3).unwrap();
    let streams = controller.slots[1].as_ref().unwrap().endpoints[3]
        .as_ref()
        .unwrap()
        .streams
        .as_ref()
        .unwrap();
    for id in 1..=2 {
        assert_eq!(streams.array.read::<u64>(id * 16 + 8), 0);
    }
    let ticket = controller
        .submit_stream(1, 3, 1, data.physical(), 64)
        .unwrap();
    model.borrow_mut().complete(3, 1, SUCCESS, 0);
    assert_eq!(controller.completed_stream(&ticket), Some(Ok(64)));
}
