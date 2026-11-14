// SPDX-License-Identifier: GPL-2.0-only
//! Unpublished Address/Disable/physical-reset/Address lifetime peer.
use super::*;

fn attachment(speed: u8) -> Attachment {
    Attachment {
        root_port: 1,
        route: 0,
        tier: 0,
        speed,
        translator: None,
        parent: None,
    }
}

fn unpublished(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
) -> (Controller, SlotId, Rc<RefCell<Hardware>>, RegionAllocator) {
    unpublished_profile(memory, speed, context, max_slots, dma::Profile::SPEC)
}

fn unpublished_profile(
    memory: &mut [u64],
    speed: u8,
    context: usize,
    max_slots: usize,
    profile: dma::Profile,
) -> (Controller, SlotId, Rc<RefCell<Hardware>>, RegionAllocator) {
    let range = memory.as_mut_ptr() as u64
        ..memory.as_mut_ptr() as u64 + core::mem::size_of_val(memory) as u64;
    // SAFETY: memory is the live, aligned backing allocation for every modeled
    // DMA owner; callers keep it until both controller and peer are dropped.
    let mut dma = unsafe { RegionAllocator::new(range) };
    let commands = Ring::with_profile(&mut dma, profile).unwrap();
    let events = dma.allocate(RING_TRBS * 16, 64).unwrap();
    let dcbaa = dma.allocate((max_slots + 1) * 8, 64).unwrap();
    let hardware = Rc::new(RefCell::new(Hardware {
        commands: Cursor {
            next: commands.buffer.physical(),
            cycle: 1,
        },
        events: events.physical(),
        event_count: 0,
        consumed: 0,
        context,
        dcbaa: dcbaa.physical(),
        outputs: BTreeMap::new(),
        cursors: BTreeMap::new(),
        pending: BTreeMap::new(),
        deferred: Vec::new(),
        enabled: vec![false; max_slots + 1],
        enables: Vec::new(),
        commands_seen: Vec::new(),
        port: PP | CCS | PED | PRC | u32::from(speed) << 10,
        speed,
        resets: 0,
        reset_ready: 0,
        reset_writes: 0,
        accesses: 0,
        trap_mmio: false,
        stuck: None,
        deaf: false,
        fault: Fault::None,
        hang_next: false,
        device_task: false,
        empty_media: false,
        sense_once: None,
        task: None,
        scsi: Vec::new(),
        writes: Vec::new(),
        bot: None,
        preferred_slot: None,
        iso: false,
        force_idle_stop: false,
        forced_idle_events: 0,
        class_peer: None,
        uas_abort: abort_peer::State::default(),
        _clock: crate::test_clock::virtualize(),
    }));
    let mut controller = Controller {
        minimum: crate::usb::xhci::minimum::fixture(Clock {
            ticks_per_microsecond: 1,
        }),
        native_memory: None,
        registers: Box::new(Model(hardware.clone())),
        clock: Clock {
            ticks_per_microsecond: 1,
        },
        operational: 0x40,
        runtime: 0x800,
        doorbells: 0x1000,
        context_bytes: context,
        dma_profile: profile,
        hardware: hardware::State::Running,
        quirks: quirks::Profile::default(),
        max_primary_streams: 4,
        stream_policy: streams::Policy::Standard,
        version: 0x100,
        iso_caps: iso::Capabilities::new(0x100, 0, 0),
        max_slots,
        max_ports: 1,
        port_major: vec![if speed >= SUPER_SPEED { 3 } else { 2 }],
        lpm: lpm::Host::default(),
        dcbaa: dcbaa.into(),
        _scratchpads: Vec::new(),
        commands,
        events: events.into(),
        _event_table: dma.allocate(16, 64).unwrap().into(),
        event_index: 0,
        event_cycle: true,
        slots: core::iter::repeat_with(|| None)
            .take(max_slots + 1)
            .collect(),
        retry_resources: slot_resources::Cache::new(max_slots + 1),
        queued_resets: Vec::new(),
        after_boot: false,
        slot_ownership: vec![Ownership::Vacant; max_slots + 1],
        test_retired_numbers: vec![false; max_slots + 1],
        power: power::Ledger::new(max_slots + 1),
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
        location: "reset model".into(),
    };
    let slot = controller.address(attachment(speed), &mut dma).unwrap();
    let id = controller.device_id(slot).unwrap();
    (controller, id, hardware, dma)
}

struct IsoDoorbell {
    inner: Box<dyn Registers>,
    hardware: Rc<RefCell<Hardware>>,
}
impl Registers for IsoDoorbell {
    fn read32(&mut self, offset: usize) -> u32 {
        self.inner.read32(offset)
    }
    fn write32(&mut self, offset: usize, value: u32) {
        if offset == 0x1004 && value == 3 {
            self.hardware.borrow_mut().queue_iso_service(1, 3);
        } else {
            self.inner.write32(offset, value);
        }
    }
    fn write64(&mut self, offset: usize, value: u64) {
        self.inner.write64(offset, value);
    }
}

#[derive(Clone, Copy)]
enum Kind {
    Ordinary,
    Stream,
    Iso,
}

fn generations(kind: Kind) {
    for context in [32, 64] {
        for cached in [false, true] {
            let speed = if matches!(kind, Kind::Stream) {
                SUPER_SPEED
            } else {
                HIGH_SPEED
            };
            let mut backing = vec![0u64; 512 * 1024];
            let (mut c, previous, hardware, mut dma) = unpublished(&mut backing, speed, context, 1);
            let slot = previous.hardware_slot();
            // The first address answers through its own identity.
            assert_eq!(c.current_configuration(previous), Ok(Some(0)));
            assert_eq!(
                c.control(previous, setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 18), &[])
                    .unwrap()[0],
                18
            );
            assert_eq!(c.current_slot(previous), Ok(slot));
            // This is an unpublished generation: the existing production owner
            // may retire it to Vacant. Published generations still tombstone.
            c.disable_slot(slot, Ownership::Vacant).unwrap();
            assert!(c.current_slot(previous).is_err());
            assert_eq!(
                hardware.borrow().commands_seen.last(),
                Some(&(DISABLE_SLOT, slot, 0))
            );
            assert_eq!(c.slot_ownership[usize::from(slot)], Ownership::Vacant);
            assert!(c.device(previous).is_err());
            assert!(c.power.retained(slot));
            // Physical reset acknowledgment retires the old current
            // grant. No ownership/SlotId/endpoint-state receipt is injected.
            c.reset_root_device(attachment(speed)).unwrap();
            assert_eq!(hardware.borrow().reset_writes, 1);
            assert!(!c.power.retained(slot));
            let current_slot = c.address(attachment(speed), &mut dma).unwrap();
            let current = c.device_id(current_slot).unwrap();
            assert_eq!(current_slot, slot);
            assert_ne!(current, previous);
            assert!(c.current_slot(previous).is_err());
            assert!(c.device(previous).is_err());
            let header = c
                .device(current)
                .unwrap()
                .control(setup(0x80, GET_DESCRIPTOR, 2 << 8, 0, 9), &[])
                .unwrap();
            let total = u16::from_le_bytes([header[2], header[3]]);
            let description = c
                .device(current)
                .unwrap()
                .control(setup(0x80, GET_DESCRIPTOR, 2 << 8, 0, total), &[])
                .unwrap();
            c.current_descriptions(slot, core::slice::from_ref(&description))
                .unwrap();
            c.claim_native(current).unwrap();
            let before_claim_check = hardware.borrow().accesses;
            assert!(c.disable_slot(slot, Ownership::Vacant).is_err());
            assert_eq!(hardware.borrow().accesses, before_claim_check);
            assert!(!c.slot(slot).unwrap().retiring);
            c.device(current).unwrap().set_configuration(1).unwrap();
            assert_eq!(
                c.device(current).unwrap().current_configuration(),
                Ok(Some(1))
            );
            let endpoints = interfaces(&description)[0].endpoints.clone();
            let mut data = dma.allocate(128, 64).unwrap();
            data.bytes_mut().fill(0xa5);
            let mut command = dma.allocate(64, 64).unwrap();
            command.bytes_mut().fill(0);
            command.bytes_mut()[0] = 1;
            command.bytes_mut()[3] = 1;
            match kind {
                Kind::Ordinary | Kind::Stream => {
                    if matches!(kind, Kind::Stream) {
                        assert_eq!(
                            c.device(current).unwrap().configure_streams(
                                &endpoints,
                                &[0x82, 0x83, 4],
                                2,
                                &mut dma
                            ),
                            Ok(2)
                        );
                    } else {
                        c.device(current)
                            .unwrap()
                            .configure(&endpoints, None, &mut dma)
                            .unwrap();
                    }
                    let trb = c.slot(slot).unwrap().endpoints[5]
                        .as_ref()
                        .unwrap()
                        .ring
                        .address();
                    let ordinary = matches!(kind, Kind::Ordinary).then(|| {
                        c.device(current)
                            .unwrap()
                            .submit_ticket(5, data.physical(), 64)
                            .unwrap()
                    });
                    let stream = matches!(kind, Kind::Stream).then(|| {
                        c.device(current)
                            .unwrap()
                            .submit_stream(5, 1, data.physical(), 64)
                            .unwrap()
                    });
                    let sent = c
                        .device(current)
                        .unwrap()
                        .submit_ticket(2, command.physical(), 32)
                        .unwrap();
                    if cached {
                        c.drain();
                    }
                    let before = backing.clone();
                    identity::tests::raw::refuse(&mut c, slot, data.physical(), trb);
                    assert_eq!(backing, before);
                    if let Some(ticket) = ordinary {
                        assert_eq!(
                            c.device(current).unwrap().completed_ticket(&ticket),
                            Some(Ok(16))
                        );
                    }
                    if let Some(ticket) = stream {
                        assert_eq!(c.completed_stream(&ticket), Some(Ok(16)));
                    }
                    assert_eq!(c.completed_ticket(&sent), Some(Ok(32)));
                    assert_eq!(&data.bytes()[..4], &[3, 0, 0, 1]);
                    assert_eq!(&data.bytes()[16..], &[0xa5; 112]);
                    // A fresh exact-generation cancellation still executes the
                    // real Stop/Set Dequeue boundary despite the raw fence.
                    if matches!(kind, Kind::Stream) {
                        c.device(current)
                            .unwrap()
                            .submit_stream(5, 1, data.physical(), 64)
                            .unwrap();
                    } else {
                        c.device(current)
                            .unwrap()
                            .submit_ticket(5, data.physical(), 64)
                            .unwrap();
                    }
                    let before_commands = hardware.borrow().commands_seen.len();
                    c.device(current).unwrap().cancel(5).unwrap();
                    let seen = hardware.borrow().commands_seen[before_commands..].to_vec();
                    assert!(seen.contains(&(STOP_ENDPOINT, slot, 5)));
                    assert!(seen.contains(&(SET_DEQUEUE, slot, 5)));
                    let raw_address = c
                        .device(current)
                        .unwrap()
                        .submit(2, command.physical(), 32)
                        .unwrap();
                    assert_eq!(
                        c.device(current).unwrap().completed(2, raw_address, 32),
                        Some(Ok(32))
                    );
                    assert_eq!(
                        c.device(current)
                            .unwrap()
                            .transfer(2, command.physical(), 32, 50_000_000),
                        Ok(32)
                    );
                    c.device(current)
                        .unwrap()
                        .reset_endpoint(&endpoints[0])
                        .unwrap();
                }
                Kind::Iso => {
                    let endpoint = Endpoint {
                        address: 0x81,
                        attributes: 1,
                        max_packet: 1024,
                        interval: 1,
                        companion: None,
                    };
                    c.device(current)
                        .unwrap()
                        .configure(core::slice::from_ref(&endpoint), None, &mut dma)
                        .unwrap();
                    // The original peer's bulk parser rejects ISO;
                    // this peer consumes its published Isoch TRB instead.
                    struct Unused;
                    impl Registers for Unused {
                        fn read32(&mut self, _: usize) -> u32 {
                            unreachable!()
                        }
                        fn write32(&mut self, _: usize, _: u32) {
                            unreachable!()
                        }
                    }
                    let inner = core::mem::replace(&mut c.registers, Box::new(Unused));
                    c.registers = Box::new(IsoDoorbell {
                        inner,
                        hardware: hardware.clone(),
                    });
                    let trb = c.slot(slot).unwrap().endpoints[3]
                        .as_ref()
                        .unwrap()
                        .ring
                        .address();
                    let ticket = c
                        .device(current)
                        .unwrap()
                        .submit_iso(3, data.physical(), 64)
                        .unwrap();
                    {
                        let mut peer = hardware.borrow_mut();
                        let td = peer.pending.remove(&(slot, 3, 0)).unwrap();
                        assert_eq!(td.length, 64);
                        copy(td.buffer, &[0x6a; 64]);
                        peer.event(td.trb, SUCCESS << 24, TRANSFER_EVENT, slot, 3);
                    }
                    if cached {
                        c.drain();
                    }
                    let before = backing.clone();
                    identity::tests::raw::refuse(&mut c, slot, data.physical(), trb);
                    assert_eq!(backing, before);
                    assert_eq!(
                        c.completed_iso(&ticket),
                        Some(Ok(IsoCompletion {
                            bytes: 64,
                            status: IsoStatus::Success
                        }))
                    );
                    assert_eq!(&data.bytes()[..64], &[0x6a; 64]);
                    assert_eq!(&data.bytes()[64..], &[0xa5; 64]);
                    c.device(current)
                        .unwrap()
                        .submit_iso(3, data.physical(), 64)
                        .unwrap();
                    c.device(current).unwrap().cancel(3).unwrap();
                    assert!(
                        hardware
                            .borrow()
                            .pending
                            .keys()
                            .all(|(s, dci, _)| *s != slot || *dci != 3)
                    );
                }
            }
            assert!(c.current_slot(previous).is_err());
            // A published slot whose current grant retired goes to the next
            // device, as xhci_free_virt_device frees it; the old identity
            // then fails before any MMIO.
            c.disable_slot(slot, Ownership::Retired).unwrap();
            assert!(c.power.retained(slot));
            c.reset_root_device(attachment(speed)).unwrap();
            assert_eq!(c.fresh_slot(), Ok(slot));
            assert_eq!(c.slot_ownership[usize::from(slot)], Ownership::Enabled);
            let before = hardware.borrow().accesses;
            assert!(c.device(current).is_err());
            assert!(c.current_configuration(current).is_err());
            assert_eq!(hardware.borrow().accesses, before);
        }
    }
}

#[test]
fn retired_raw_slot_refuses_before_mmio_and_preserves_real_current_ordinary_completion() {
    generations(Kind::Ordinary);
}

#[test]
fn retired_raw_slot_refuses_before_mmio_and_preserves_real_current_stream_completion() {
    generations(Kind::Stream);
}

#[test]
fn retired_raw_slot_refuses_before_mmio_and_preserves_real_current_iso_completion() {
    generations(Kind::Iso);
}

struct FatalAfterDisable {
    inner: Box<dyn Registers>,
    hardware: Rc<RefCell<Hardware>>,
    status_reads: Option<usize>,
}
impl Registers for FatalAfterDisable {
    fn read32(&mut self, offset: usize) -> u32 {
        let status = self.inner.read32(offset);
        if offset == 0x40 + USBSTS
            && let Some(reads) = &mut self.status_reads
        {
            *reads += 1;
            // The first read receives the peer's matching Disable ACK;
            // the independent following health read reports a fatal controller.
            if *reads > 1 {
                return status | HCE;
            }
        }
        status
    }
    fn write32(&mut self, offset: usize, value: u32) {
        let disabling =
            offset == 0x1000 && self.hardware.borrow().pending_command_kind() == DISABLE_SLOT;
        self.inner.write32(offset, value);
        if disabling {
            self.status_reads = Some(0);
        }
    }
    fn write64(&mut self, offset: usize, value: u64) {
        self.inner.write64(offset, value);
    }
}

#[test]
fn matching_disable_ack_permanently_fences_raw_access_before_fatal_health_failure() {
    for ownership in [Ownership::Vacant, Ownership::Retired] {
        let mut backing = vec![0u64; 512 * 1024];
        let (mut c, id, hardware, mut dma) = unpublished(&mut backing, HIGH_SPEED, 32, 1);
        let slot = id.hardware_slot();
        struct Unused;
        impl Registers for Unused {
            fn read32(&mut self, _: usize) -> u32 {
                unreachable!()
            }
            fn write32(&mut self, _: usize, _: u32) {
                unreachable!()
            }
        }
        let inner = core::mem::replace(&mut c.registers, Box::new(Unused));
        c.registers = Box::new(FatalAfterDisable {
            inner,
            hardware: hardware.clone(),
            status_reads: None,
        });
        let error = c.disable_slot(slot, ownership).unwrap_err();
        assert_eq!(error.0, "post-Disable Slot controller status");
        assert!(matches!(error.1, BlockError::Device { status } if status & HCE != 0));
        assert_eq!(
            hardware.borrow().commands_seen.last(),
            Some(&(DISABLE_SLOT, slot, 0))
        );
        assert!(c.current_slot(id).is_err());
        assert_eq!(c.slot_ownership[usize::from(slot)], Ownership::Enabled);
        assert!(c.slot(slot).unwrap().retiring);
        assert_ne!(c.dcbaa.read::<u64>(usize::from(slot) * 8), 0);
        assert!(c.power.retained(slot));
        let data = dma.allocate(64, 64).unwrap();
        let before = backing.clone();
        identity::tests::raw::refuse(&mut c, slot, data.physical(), 0);
        assert_eq!(backing, before);
    }
}

mod slot_resource_tests;

#[test]
fn immutable_uas_planner_keeps_current_generation_after_slot_reuse() {
    for context in [32, 64] {
        let mut backing = vec![0u64; 512 * 1024];
        let (mut c, previous, hardware, mut dma) =
            unpublished(&mut backing, SUPER_SPEED, context, 1);
        let mut foreign_backing = vec![0u64; 512 * 1024];
        let (_foreign_controller, foreign, foreign_hardware, _foreign_dma) =
            unpublished(&mut foreign_backing, SUPER_SPEED, context, 1);
        assert_eq!(previous.hardware_slot(), foreign.hardware_slot());
        assert_ne!(previous, foreign);
        let slot = previous.hardware_slot();
        c.disable_slot(slot, Ownership::Vacant).unwrap();
        assert!(c.power.retained(slot));
        c.reset_root_device(attachment(SUPER_SPEED)).unwrap();
        assert_eq!(hardware.borrow().reset_writes, 1);
        assert!(!c.power.retained(slot));
        // Identify reads the real descriptor/configuration bytes through EP0
        // after a fresh Enable/Address; no SlotId is manufactured.
        let mut device = c
            .identify(attachment(SUPER_SPEED), "reused UAS peer".into(), &mut dma)
            .unwrap();
        assert_eq!(device.slot, slot);
        assert_ne!(device.id, previous);
        assert!(c.current_slot(previous).is_err());
        device.configuration = device.configurations[0].clone();
        let all = interfaces(&device.configuration);
        let interface = &all[0];
        let endpoints = &interface.endpoints;
        let selected = [0x82, 0x83, 4];
        struct NoAccess;
        impl Registers for NoAccess {
            fn read32(&mut self, _: usize) -> u32 {
                panic!("pure stream planner read MMIO")
            }
            fn write32(&mut self, _: usize, _: u32) {
                panic!("pure stream planner wrote MMIO")
            }
        }
        let registers = core::mem::replace(&mut c.registers, Box::new(NoAccess));
        let before = backing.clone();
        let foreign_before = foreign_backing.clone();
        let remaining = dma.remaining();
        let accesses = hardware.borrow().accesses;
        let foreign_accesses = foreign_hardware.borrow().accesses;
        assert!(c.stream_capacity(slot, endpoints, &selected, 32).is_err());
        assert_eq!(
            c.stream_capacity_for_device(device.id, endpoints, &selected, 32),
            Ok(2)
        );
        assert_eq!(
            c.device(device.id)
                .unwrap()
                .stream_capacity(endpoints, &selected, 32),
            Ok(2)
        );
        assert!(matches!(
            crate::usb::storage::plan(&device, interface, &all, Some(&c)),
            Ok(Plan::Storage(crate::usb::storage::Binding::Uas(_)))
        ));
        for stale in [previous, foreign] {
            assert!(
                c.stream_capacity_for_device(stale, endpoints, &selected, 32)
                    .is_err()
            );
        }
        c.slots[usize::from(slot)].as_mut().unwrap().suspended = true;
        assert!(
            c.stream_capacity_for_device(device.id, endpoints, &selected, 32)
                .is_err()
        );
        assert!(crate::usb::storage::plan(&device, interface, &all, Some(&c)).is_err());
        c.slots[usize::from(slot)].as_mut().unwrap().suspended = false;
        assert_eq!(hardware.borrow().accesses, accesses);
        assert_eq!(foreign_hardware.borrow().accesses, foreign_accesses);
        assert_eq!(backing, before);
        assert_eq!(foreign_backing, foreign_before);
        assert_eq!(dma.remaining(), remaining);
        assert_eq!(
            c.slots[usize::from(slot)]
                .as_ref()
                .unwrap()
                .current_configuration,
            Some(0)
        );
        assert!(
            c.slots[usize::from(slot)]
                .as_ref()
                .unwrap()
                .endpoints
                .iter()
                .skip(2)
                .all(Option::is_none)
        );
        c.registers = registers;
    }
}
