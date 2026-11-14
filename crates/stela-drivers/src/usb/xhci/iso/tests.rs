// SPDX-License-Identifier: GPL-2.0-only
//! xHCI isochronous DMA model and shared class lifecycle fixture.
use super::*;
use crate::bus::dma::{RegionAllocator, TestRegion};
use alloc::collections::BTreeMap;
extern crate std;

struct Td {
    trb: u64,
    buffer: u64,
    bytes: usize,
    control: u32,
}
struct Hardware {
    slot: u8,
    speed: u8,
    commands: u64,
    command_index: usize,
    events: u64,
    event_index: usize,
    output: u64,
    context_bytes: usize,
    cursors: BTreeMap<u8, (u64, usize, u32)>,
    active: Vec<(u8, Td)>,
    seen: Vec<(u32, u8)>,
    mfindex: u32,
    fail: Option<u32>,
    silent: Option<u32>,
    /// A silenced command, keeping CRCR.CRR set until it is aborted.
    stuck: Option<u64>,
    fatal: bool,
    output_bytes: Vec<u8>,
    serviced: BTreeMap<u8, u32>,
}
impl Hardware {
    fn word(address: u64) -> u32 {
        assert!(address > 65536 && address.is_multiple_of(4));
        // SAFETY: Private callers derive this aligned word address from kept
        // command/event/context allocations. Their identity-mapped test backing stays
        // live throughout the synchronous model access.
        unsafe { (address as *const u32).read_volatile() }
    }
    fn long(address: u64) -> u64 {
        assert!(address > 65536 && address.is_multiple_of(8));
        // SAFETY: Private callers use an eight-byte-aligned address within kept
        // descriptor/context RAM. The identity-mapped allocation remains live and no
        // hardware or concurrent writer accesses it.
        unsafe { (address as *const u64).read_volatile() }
    }
    fn state(&mut self, dci: u8, state: u32) {
        let at = self.output + u64::from(dci) * self.context_bytes as u64;
        // SAFETY: output names the 32-context output allocation; callers
        // supply a valid endpoint DCI and 32/64-byte context stride. The synchronous
        // model exclusively updates its state word.
        unsafe {
            (at as *mut u32).write_volatile(Self::word(at) & !7 | state);
        }
    }
    fn event(&mut self, parameter: u64, code: u32, dci: u8, residual: usize, kind: u32) {
        let at = self.events + (self.event_index % RING_TRBS * TRB) as u64;
        // SAFETY: events names the event-ring allocation; modulo RING_TRBS
        // selects a complete aligned TRB. Model callbacks serialize writes and the
        // controller keeps backing while consuming events.
        unsafe {
            (at as *mut u64).write_volatile(parameter);
            ((at + 8) as *mut u32).write_volatile(code << 24 | residual as u32);
            ((at + 12) as *mut u32).write_volatile(
                u32::from(self.slot) << 24
                    | u32::from(dci) << 16
                    | kind << 10
                    | u32::from((self.event_index / RING_TRBS).is_multiple_of(2)),
            );
        }
        self.event_index += 1;
    }
    /// A command abort: the stuck command completes aborted, then the ring stops.
    fn abort(&mut self) {
        if let Some(at) = self.stuck.take() {
            self.event(at, COMMAND_ABORTED, 0, 0, COMMAND_COMPLETION);
            self.event(0, COMMAND_RING_STOPPED, 0, 0, COMMAND_COMPLETION);
        }
    }
    fn command(&mut self) {
        let at = self.commands + (self.command_index % (RING_TRBS - 1) * TRB) as u64;
        let parameter = Self::long(at);
        let control = Self::word(at + 12);
        assert_eq!((control >> 24) as u8, self.slot);
        assert_eq!(
            control & CYCLE,
            u32::from((self.command_index / (RING_TRBS - 1)).is_multiple_of(2))
        );
        let kind = control >> 10 & 63;
        let dci = (control >> 16 & 31) as u8;
        self.command_index += 1;
        self.seen.push((kind, dci));
        if self.silent == Some(kind) {
            self.stuck = Some(at);
            return;
        }
        let code = if self.fail == Some(kind) { 17 } else { SUCCESS };
        if code == SUCCESS {
            match kind {
                CONFIGURE_ENDPOINT => {
                    let add = Self::word(parameter + 4);
                    let drop = Self::word(parameter);
                    for ep in 2..32u8 {
                        if drop & (1 << ep) != 0 {
                            self.state(ep, 0);
                            self.cursors.remove(&ep);
                        }
                        if add & (1 << ep) == 0 {
                            continue;
                        }
                        let from = parameter + (u64::from(ep) + 1) * self.context_bytes as u64;
                        let to = self.output + u64::from(ep) * self.context_bytes as u64;
                        // SAFETY: The published input and kept output contexts
                        // are distinct allocations. ep is in 2..32, and both
                        // allocations contain the complete selected 32/64-byte
                        // context; no model callback accesses them concurrently.
                        unsafe {
                            core::ptr::copy_nonoverlapping(
                                from as *const u8,
                                to as *mut u8,
                                self.context_bytes,
                            );
                        }
                        let info2 = Self::word(to + 4);
                        assert!(matches!(info2 >> 3 & 7, 1 | 5));
                        assert_eq!(info2 & 6, 0);
                        assert_eq!(Self::word(to + 16) & 0xffff, Self::word(to + 16) >> 16);
                        self.state(ep, ENDPOINT_RUNNING);
                    }
                }
                STOP_ENDPOINT => self.state(dci, ENDPOINT_STOPPED),
                RESET_ENDPOINT => self.state(dci, ENDPOINT_STOPPED),
                SET_DEQUEUE => {
                    let endpoint = self.output + u64::from(dci) * self.context_bytes as u64;
                    assert_eq!(Self::word(endpoint) & 7, ENDPOINT_STOPPED);
                    let base = Self::long(endpoint + 8) & !((RING_TRBS * TRB) as u64 - 1);
                    self.cursors.insert(
                        dci,
                        (
                            base,
                            ((parameter & !15) - base) as usize / TRB,
                            parameter as u32 & 1,
                        ),
                    );
                    let old = core::mem::take(&mut self.active);
                    for (ep, td) in old {
                        if ep == dci {
                            self.event(td.trb, 26, dci, td.bytes, TRANSFER_EVENT);
                        } else {
                            self.active.push((ep, td));
                        }
                    }
                }
                _ => panic!("unexpected command {kind}"),
            }
        }
        self.event(at, code, 0, 0, COMMAND_COMPLETION);
    }
    fn transfer(&mut self, dci: u8) {
        let ep = self.output + u64::from(dci) * self.context_bytes as u64;
        let initial = Self::long(ep + 8);
        let base = initial & !((RING_TRBS * TRB) as u64 - 1);
        let cursor = self.cursors.entry(dci).or_insert((
            base,
            ((initial & !15) - base) as usize / TRB,
            initial as u32 & 1,
        ));
        let trb = cursor.0 + (cursor.1 * TRB) as u64;
        let control = Self::word(trb + 12);
        let bytes = (Self::word(trb + 8) & 0x1ffff) as usize;
        assert_eq!(control & CYCLE, cursor.2);
        assert_eq!(control >> 10 & 63, ISOCH);
        assert_ne!(control & IOC, 0);
        let packet = Self::word(ep + 4) >> 16;
        let packets = (bytes as u32).div_ceil(packet).max(1);
        if self.speed >= SUPER_SPEED {
            let maximum = (Self::word(ep + 4) >> 8 & 0xff) + 1;
            assert_eq!(control >> 7 & 3, (packets - 1) / maximum);
            assert_eq!(control >> 16 & 15, (packets - 1) % maximum);
        } else {
            assert_eq!(control >> 7 & 3, 0);
            assert_eq!(control >> 16 & 15, packets - 1);
        }
        let buffer = Self::long(trb);
        assert!(buffer % 65536 + bytes as u64 <= 65536);
        assert!(
            !self
                .active
                .iter()
                .any(|(ep, td)| *ep == dci && td.trb == trb)
        );
        self.active.push((
            dci,
            Td {
                trb,
                buffer,
                bytes,
                control,
            },
        ));
        cursor.1 += 1;
        if cursor.1 == RING_TRBS - 1 {
            let link = cursor.0 + ((RING_TRBS - 1) * TRB) as u64;
            assert_eq!(Self::long(link), cursor.0);
            assert_eq!(Self::word(link + 12) & 1, cursor.2);
            assert_eq!(Self::word(link + 12) >> 10 & 63, LINK);
            assert_ne!(Self::word(link + 12) & TOGGLE, 0);
            cursor.1 = 0;
            cursor.2 ^= 1;
        }
        self.state(dci, ENDPOINT_RUNNING);
    }
    fn complete(&mut self, dci: u8, index: usize, code: u32, residual: usize) {
        let at = self
            .active
            .iter()
            .enumerate()
            .filter(|(_, (ep, _))| *ep == dci)
            .nth(index)
            .unwrap()
            .0;
        let (_, td) = self.active.remove(at);
        if code == 6 {
            self.state(dci, ENDPOINT_HALTED);
        }
        if matches!(code, SUCCESS | SHORT_PACKET) && residual <= td.bytes {
            let n = td.bytes - residual;
            if dci & 1 != 0 {
                // SAFETY: The active TD came from a published payload and n
                // is bounded by td.bytes. The caller keeps the whole payload
                // allocation live; this synchronous IN completion is its sole writer.
                unsafe {
                    core::ptr::write_bytes(td.buffer as *mut u8, 0x5a, n);
                }
            } else {
                self.output_bytes.extend_from_slice(
                    // SAFETY: This OUT TD names a live payload and n is bounded
                    // by td.bytes. The slice is read only for this immediate copy, with
                    // no concurrent device writer or escaping borrow.
                    unsafe { core::slice::from_raw_parts(td.buffer as *const u8, n) },
                );
            }
        }
        self.event(td.trb, code, dci, residual, TRANSFER_EVENT);
    }

    /// Independent device peer: service a Frame-ID TD only when
    /// the modeled controller clock reaches its encoded frame and the
    /// endpoint context's interval selects this microframe. No
    /// production scheduling helper or software queue metadata is used.
    fn service_at(&mut self, dci: u8, microframe: u32, code: u32) {
        self.mfindex = microframe;
        let td = &self.active.iter().find(|(ep, _)| *ep == dci).unwrap().1;
        assert_eq!(td.control & SIA, 0);
        assert_eq!((td.control >> 20) & 0x7ff, microframe / 8);
        let ep = self.output + u64::from(dci) * self.context_bytes as u64;
        let interval = 1 << (Self::word(ep) >> 16 & 0xff);
        let expected = self
            .serviced
            .get(&dci)
            .map_or(microframe / 8 * 8, |last| (last + interval) & 0x3fff);
        assert_eq!(microframe, expected);
        self.serviced.insert(dci, microframe);
        self.complete(dci, 0, code, 0);
    }
}
struct Model(Rc<RefCell<Hardware>>);
impl Registers for Model {
    fn read32(&mut self, offset: usize) -> u32 {
        let model = self.0.borrow();
        if offset == 0x800 {
            model.mfindex
        } else if offset == 0x40 + CRCR && model.stuck.is_some() {
            CRCR_RUNNING
        } else if offset == 0x40 + USBSTS && model.fatal {
            HSE
        } else {
            0
        }
    }
    fn write32(&mut self, offset: usize, value: u32) {
        let doorbell = 0x1000 + 4 * usize::from(self.0.borrow().slot);
        match offset {
            0x58 if value & CRCR_ABORT != 0 => self.0.borrow_mut().abort(),
            0x1000 => self.0.borrow_mut().command(),
            at if at == doorbell => self.0.borrow_mut().transfer(value as u8),
            _ => {}
        }
    }
}
fn endpoint(address: u8, speed: u8) -> Endpoint {
    Endpoint {
        address,
        attributes: 1,
        max_packet: if speed == FULL_SPEED {
            192
        } else {
            1024 | 2 << 11
        },
        interval: 1,
        companion: None,
    }
}
fn controller(
    dma: &mut dyn DmaAllocator,
    speed: u8,
    context_bytes: usize,
    cfc: bool,
) -> (Controller, Rc<RefCell<Hardware>>) {
    let commands = Ring::new(dma).unwrap();
    let events = dma.allocate(RING_TRBS * TRB, RING_TRBS * TRB).unwrap();
    let output = dma.allocate(32 * context_bytes, 64).unwrap();
    let model = Rc::new(RefCell::new(Hardware {
        slot: 1,
        speed,
        commands: commands.buffer.physical(),
        command_index: 0,
        events: events.physical(),
        event_index: 0,
        output: output.physical(),
        context_bytes,
        cursors: BTreeMap::new(),
        active: Vec::new(),
        seen: Vec::new(),
        mfindex: 0x3ff0,
        fail: None,
        silent: None,
        stuck: None,
        fatal: false,
        output_bytes: Vec::new(),
        serviced: BTreeMap::new(),
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
            speed,
            translator: None,
            parent: None,
        },
        retiring: false,
        input: dma.allocate(33 * context_bytes, 64).unwrap().into(),
        _output: output.into(),
        context: [u32::from(speed) << 20 | 1 << 27, 1 << 16, 0, 0],
        endpoints: Default::default(),
        retired_rings: Default::default(),
        control: dma.allocate(CONTROL_BYTES, CONTROL_BYTES).unwrap().into(),
        speed,
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
            dma_profile: dma::Profile::SPEC,
            hardware: hardware::State::Running,
            quirks: quirks::Profile::default(),
            max_primary_streams: 4,
            stream_policy: streams::Policy::Unknown,
            version: 0x110,
            iso_caps: Capabilities::new(0x110, 0xb, if cfc { 1 << 11 } else { 0 }),
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
            retry_resources: slot_resources::Cache::new(slots.len()),
            queued_resets: Vec::new(),
            after_boot: false,
            slots,
            slot_ownership: vec![reset::Ownership::Vacant, reset::Ownership::Enabled],
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
            location: String::from("native iso model"),
        },
        model,
    )
}

/// Install the same independent ISO peer on the exact controller resources
/// after real address/configuration (or full hub enumeration).
fn attach_addressed_peer(
    mut controller: Controller,
    slot: u8,
    speed: u8,
    cfc: bool,
) -> (Controller, Rc<RefCell<Hardware>>) {
    let id = controller.device_id(slot).unwrap();
    controller.validate_id(id).unwrap();
    let model = Rc::new(RefCell::new(Hardware {
        slot,
        speed,
        commands: controller.commands.buffer.physical(),
        command_index: controller.commands.index,
        events: controller.events.physical(),
        event_index: controller.event_index + if controller.event_cycle { 0 } else { RING_TRBS },
        output: controller.slots[usize::from(slot)]
            .as_ref()
            .unwrap()
            ._output
            .physical(),
        context_bytes: controller.context_bytes,
        cursors: BTreeMap::new(),
        active: Vec::new(),
        seen: Vec::new(),
        mfindex: 0x3ff0,
        fail: None,
        silent: None,
        stuck: None,
        fatal: false,
        output_bytes: Vec::new(),
        serviced: BTreeMap::new(),
    }));
    controller.registers = Box::new(Model(model.clone()));
    controller.clock.ticks_per_microsecond = 1;
    controller.iso_caps = Capabilities::new(0x110, 0xb, if cfc { 1 << 11 } else { 0 });
    (controller, model)
}

fn addressed_controller(
    dma: &mut dyn DmaAllocator,
    speed: u8,
    context_bytes: usize,
    cfc: bool,
) -> (Controller, Rc<RefCell<Hardware>>) {
    let controller = model::configured_endpoint_controller(dma, speed, context_bytes);
    attach_addressed_peer(controller, 1, speed, cfc)
}

fn assert_schedule_refused(
    controller: &mut Controller,
    model: &Rc<RefCell<Hardware>>,
    buffer: u64,
    requested: u16,
) {
    let transfer = controller.slot(1).unwrap().endpoints[3].as_ref().unwrap();
    let bytes = transfer.ring.buffer.bytes().to_vec();
    let cursor = transfer.ring.address();
    let queue = transfer.iso.as_ref().unwrap();
    let next = queue.next;
    let active: Vec<_> = queue
        .active
        .iter()
        .map(|td| (td.id, td.trb, td.bytes, td.done))
        .collect();
    let device_count = model.borrow().active.len();
    let commands = model.borrow().seen.clone();
    let device_cursors = model.borrow().cursors.clone();
    assert!(
        controller
            .submit_iso_scheduled(1, 3, buffer, 32, IsoSchedule::AtMicroframe(requested))
            .is_err()
    );
    let transfer = controller.slot(1).unwrap().endpoints[3].as_ref().unwrap();
    assert_eq!(transfer.ring.buffer.bytes(), bytes);
    assert_eq!(transfer.ring.address(), cursor);
    let queue = transfer.iso.as_ref().unwrap();
    assert_eq!(queue.next, next);
    assert_eq!(
        queue
            .active
            .iter()
            .map(|td| (td.id, td.trb, td.bytes, td.done))
            .collect::<Vec<_>>(),
        active
    );
    assert_eq!(model.borrow().active.len(), device_count);
    assert_eq!(model.borrow().seen, commands);
    assert_eq!(model.borrow().cursors, device_cursors);
}

fn assert_list_refused(
    controller: &mut Controller,
    model: &Rc<RefCell<Hardware>>,
    schedule: IsoSchedule,
    packets: usize,
) {
    let transfer = controller.slot(1).unwrap().endpoints[3].as_ref().unwrap();
    let bytes = transfer.ring.buffer.bytes().to_vec();
    let cursor = (transfer.ring.index, transfer.ring.cycle);
    let queue = transfer.iso.as_ref().unwrap();
    let queue_state = (queue.next, queue.skip, queue.recovery);
    let active: Vec<_> = queue
        .active
        .iter()
        .map(|td| (td.id, td.trb, td.bytes, td.done))
        .collect();
    let device_count = model.borrow().active.len();
    let commands = model.borrow().seen.clone();
    let device_cursors = model.borrow().cursors.clone();
    assert!(
        controller
            .iso_schedule_list(1, 3, schedule, packets)
            .is_err()
    );
    let transfer = controller.slot(1).unwrap().endpoints[3].as_ref().unwrap();
    assert_eq!(transfer.ring.buffer.bytes(), bytes);
    assert_eq!((transfer.ring.index, transfer.ring.cycle), cursor);
    let queue = transfer.iso.as_ref().unwrap();
    assert_eq!((queue.next, queue.skip, queue.recovery), queue_state);
    assert_eq!(
        queue
            .active
            .iter()
            .map(|td| (td.id, td.trb, td.bytes, td.done))
            .collect::<Vec<_>>(),
        active
    );
    assert_eq!(model.borrow().active.len(), device_count);
    assert_eq!(model.borrow().seen, commands);
    assert_eq!(model.borrow().cursors, device_cursors);
}

/// Opaque reuse of the real-ring ISO model by class lifecycle regressions. This
/// is not USB enumeration, normal control/HCI I/O or a hardware validation.
pub(crate) struct ClassIsoFixture {
    controller: Shared,
    hardware: Rc<RefCell<Hardware>>,
}
pub(crate) fn class_fixture(dma: &mut dyn DmaAllocator, speed: u8) -> ClassIsoFixture {
    let (controller, hardware) = controller(dma, speed, 32, true);
    ClassIsoFixture {
        controller: Rc::new(RefCell::new(controller)),
        hardware,
    }
}
impl ClassIsoFixture {
    pub(crate) fn controller(&self) -> Shared {
        self.controller.clone()
    }
    pub(crate) fn set_mfindex(&self, value: u32) {
        self.hardware.borrow_mut().mfindex = value;
    }
    /// Service queued FrameID/interval words at this model tick.
    pub(crate) fn service_at(&self, dci: u8, microframe: u32, code: u32) {
        self.hardware.borrow_mut().service_at(dci, microframe, code);
    }
    /// Complete the oldest queued service on this endpoint.
    pub(crate) fn complete(&self, dci: u8, code: u32, residual: usize) {
        self.hardware.borrow_mut().complete(dci, 0, code, residual);
    }
    pub(crate) fn commands(&self) -> Vec<(u32, u8)> {
        self.hardware.borrow().seen.clone()
    }
}

mod admission;
mod cancellation;
mod completion;
mod epochs;
mod intervals;
mod oracles;
mod request_lists;
mod retirement;
mod reuse;
mod scheduling;
mod superspeed;
mod transfers;
