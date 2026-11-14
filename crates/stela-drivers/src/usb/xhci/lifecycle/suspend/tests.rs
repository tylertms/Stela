// SPDX-License-Identifier: GPL-2.0-only
//! Authored physical register/TRB model checks; UNRUN in the source-only cut.
use super::super::super::Registers as RegisterIo;
use super::*;
use crate::block::BlockDevice;
use crate::bus::dma::RegionAllocator;
use alloc::collections::BTreeMap;
use hub::model::{Hardware, Node, RegistersModel};

struct Registers {
    bus: RegistersModel,
    state: Rc<RefCell<State>>,
}
#[derive(Default)]
struct State {
    save: usize,
    restore: usize,
    refuse_suspend: bool,
    refuse_resume: bool,
    resume_started: Option<u64>,
    resume_durations: Vec<u64>,
    minimum_resume_ticks: u64,
}
impl RegisterIo for Registers {
    fn read32(&mut self, at: usize) -> u32 {
        self.bus.read32(at)
    }
    fn write32(&mut self, at: usize, value: u32) {
        self.bus.write32(at, value);
        if at == 0x440 && value & LINK_STROBE != 0 {
            let mut bus = self.bus.0.borrow_mut();
            let port = bus.root_status.as_mut().unwrap();
            let requested = value & PLS;
            let previous = *port & PLS;
            let refused = requested == U3 << PLS_SHIFT && self.state.borrow().refuse_suspend
                || previous == RESUME << PLS_SHIFT
                    && requested == 0
                    && self.state.borrow().refuse_resume;
            if !refused {
                if requested == RESUME << PLS_SHIFT && previous != requested {
                    self.state.borrow_mut().resume_started = Some(Clock::ticks());
                }
                if previous == RESUME << PLS_SHIFT && requested == 0 {
                    let mut state = self.state.borrow_mut();
                    let elapsed = Clock::ticks().wrapping_sub(state.resume_started.take().unwrap());
                    assert!(
                        elapsed >= state.minimum_resume_ticks,
                        "actual USB2 root left Resume before source 40ms"
                    );
                    state.resume_durations.push(elapsed);
                }
                *port = *port & !PLS | requested;
                if previous != requested {
                    *port |= LINK_CHANGE;
                }
            }
        }
        if at == 0x40 {
            if value & (1 << 8) != 0 {
                self.state.borrow_mut().save += 1;
            }
            if value & (1 << 9) != 0 {
                let mut state = self.state.borrow_mut();
                assert!(
                    state.save > state.restore,
                    "restore without an acknowledged save request"
                );
                state.restore += 1;
            }
        }
    }
    fn size(&self) -> Option<usize> {
        Some(0x2000)
    }
}

#[test]
fn usb2_root_resume_preserves_source_forty_millisecond_signal_before_u0() {
    let _clock = crate::test_clock::virtualize();
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let mut memory = vec![0u64; 512 * 1024];
        let (mut host, devices, hardware, state, mut dma) = fixture_with_leaf_at_clock(
            &mut memory,
            speed,
            false,
            Node::bot_disk(speed),
            Clock {
                ticks_per_microsecond: 1_000,
            },
        );
        let mut original = bind_root_disk(&host, &devices, &mut dma);
        let payload = [0x6c; 512];
        original.disks[0].write(2, &payload).unwrap();
        // Independent test TSC scale, not a measured host-clock calibration.
        // The model's physical PORTSC
        // assertion uses the original source macro, not the production constant.
        state.borrow_mut().minimum_resume_ticks = 40_000 * 1_000;
        let suspended = host.suspend();
        assert!(
            suspended.added.failures.is_empty(),
            "{:?}",
            suspended.added.failures
        );
        let resumed = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
        assert!(
            resumed.added.failures.is_empty(),
            "{:?}",
            resumed.added.failures
        );
        assert!(host.sleep.awake());
        assert_eq!(hardware.borrow().root_status.unwrap() & PLS, 0);
        assert_eq!(
            state.borrow().resume_durations.len(),
            usize::from(speed == HIGH_SPEED)
        );
        assert!(resumed.added.disks.is_empty());
        let mut resumed = host.settled(&mut dma);
        assert_eq!(resumed.added.disks.len(), 1);
        let mut actual = [0; 512];
        resumed.added.disks[0].read(2, &mut actual).unwrap();
        assert_eq!(actual, payload);
        assert!(original.disks[0].read(2, &mut actual).is_err());
    }
}

#[test]
fn unavailable_root_timing_preserves_live_consumers_before_any_suspend_effect() {
    for rate in [0, u64::MAX] {
        let mut memory = vec![0u64; 512 * 1024];
        let (mut host, devices, hardware, state, mut dma) = fixture(&mut memory, HIGH_SPEED, false);
        let mut consumer = RawConsumer::retain(&host, &devices[0], &mut dma);
        host.controller.borrow_mut().clock.ticks_per_microsecond = rate;
        let before = {
            let bus = hardware.borrow();
            (
                bus.mmio_reads,
                bus.mmio_writes,
                bus.commands.clone(),
                bus.requests.clone(),
                bus.root_status,
            )
        };
        let refused = host.suspend();
        assert_eq!(refused.added.failures.len(), 1);
        assert!(refused.removed.is_empty());
        assert!(host.sleep.awake());
        assert_eq!(state.borrow().save, 0);
        let after = {
            let bus = hardware.borrow();
            (
                bus.mmio_reads,
                bus.mmio_writes,
                bus.commands.clone(),
                bus.requests.clone(),
                bus.root_status,
            )
        };
        assert_eq!(before, after);
        host.controller.borrow_mut().clock.ticks_per_microsecond = 1;
        assert!(consumer.submit().is_ok());
    }
}

#[test]
fn missing_root_u0_acknowledgement_keeps_fenced_original_consumers() {
    let mut memory = vec![0u64; 512 * 1024];
    let (mut host, devices, hardware, state, mut dma) = fixture(&mut memory, HIGH_SPEED, false);
    let mut consumer = RawConsumer::retain(&host, &devices[0], &mut dma);
    let suspended = host.suspend();
    assert!(suspended.added.failures.is_empty());
    state.borrow_mut().refuse_resume = true;
    let before = hardware.borrow().commands.len();
    let refused = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
    assert!(
        refused
            .added
            .failures
            .iter()
            .any(|(_, error)| *error == BlockError::Timeout)
    );
    assert!(refused.added.devices.is_empty());
    assert!(matches!(host.sleep, Sleep::Failed));
    assert_eq!(
        hardware.borrow().root_status.unwrap() & PLS,
        RESUME << PLS_SHIFT
    );
    assert!(
        hardware.borrow().commands[before..]
            .iter()
            .all(|(kind, _)| *kind != reset::DISABLE_SLOT)
    );
    consumer.assert_fenced(&hardware);
}

#[test]
fn unavailable_resume_timing_keeps_private_snapshot_without_an_extra_consumer_fence() {
    let _clock = crate::test_clock::virtualize();
    for rate in [0, u64::MAX] {
        let mut memory = vec![0u64; 512 * 1024];
        let (mut host, devices, hardware, state, mut dma) =
            fixture_with_leaf(&mut memory, HIGH_SPEED, false, Node::bot_disk(HIGH_SPEED));
        let mut original = bind_root_disk(&host, &devices, &mut dma);
        let payload = [0x7d; 512];
        original.disks[0].write(3, &payload).unwrap();
        assert!(host.suspend().added.failures.is_empty());
        let before = {
            let bus = hardware.borrow();
            (
                bus.mmio_reads,
                bus.mmio_writes,
                bus.commands.clone(),
                bus.requests.clone(),
                bus.root_status,
            )
        };
        let prior_roots = match &host.sleep {
            Sleep::Suspended(snapshot) => snapshot.roots.clone(),
            _ => panic!("actual suspend snapshot required"),
        };
        let prior_slots: Vec<_> = host
            .controller
            .borrow()
            .slots
            .iter()
            .map(|slot| slot.as_ref().map(|slot| (slot.retiring, slot.suspended)))
            .collect();
        host.controller.borrow_mut().clock.ticks_per_microsecond = rate;
        let refused = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
        assert_eq!(refused.added.failures.len(), 1);
        assert!(refused.added.devices.is_empty() && refused.removed.is_empty());
        match &host.sleep {
            Sleep::Suspended(snapshot) => assert_eq!(snapshot.roots, prior_roots),
            _ => panic!("pure timing refusal lost original private snapshot"),
        }
        let after = {
            let bus = hardware.borrow();
            (
                bus.mmio_reads,
                bus.mmio_writes,
                bus.commands.clone(),
                bus.requests.clone(),
                bus.root_status,
            )
        };
        assert_eq!(before, after);
        let current_slots: Vec<_> = host
            .controller
            .borrow()
            .slots
            .iter()
            .map(|slot| slot.as_ref().map(|slot| (slot.retiring, slot.suspended)))
            .collect();
        assert_eq!(current_slots, prior_slots);
        assert_eq!(state.borrow().restore, 0);
        host.controller.borrow_mut().clock.ticks_per_microsecond = 1;
        let resumed = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
        assert!(
            resumed.added.failures.is_empty(),
            "{:?}",
            resumed.added.failures
        );
        assert!(host.sleep.awake());
        assert_eq!(state.borrow().restore, 1);
        assert!(resumed.added.disks.is_empty());
        let mut resumed = host.settled(&mut dma);
        assert_eq!(resumed.added.disks.len(), 1);
        let mut actual = [0; 512];
        resumed.added.disks[0].read(3, &mut actual).unwrap();
        assert_eq!(actual, payload);
        assert!(original.disks[0].read(3, &mut actual).is_err());
    }
}

fn bind_root_disk(host: &Host, devices: &[Device], dma: &mut RegionAllocator) -> Found {
    assert_eq!(devices.len(), 1);
    let mut found = Found::default();
    crate::usb::xhci::bind::bind(&host.controller, devices[0].clone(), dma, &mut found);
    assert!(found.failures.is_empty(), "{:?}", found.failures);
    assert_eq!(found.disks.len(), 1);
    found
}

type Fixture = (
    Host,
    Vec<Device>,
    Rc<RefCell<Hardware>>,
    Rc<RefCell<State>>,
    RegionAllocator,
);

fn fixture(memory: &mut [u64], speed: u8, hub: bool) -> Fixture {
    fixture_with_leaf(memory, speed, hub, Node::leaf(speed))
}

fn fixture_with_leaf(memory: &mut [u64], speed: u8, hub: bool, leaf: Node) -> Fixture {
    fixture_with_leaf_at_clock(
        memory,
        speed,
        hub,
        leaf,
        Clock {
            ticks_per_microsecond: 1,
        },
    )
}

fn fixture_with_leaf_at_clock(
    memory: &mut [u64],
    speed: u8,
    hub: bool,
    leaf: Node,
    clock: Clock,
) -> Fixture {
    let start = memory.as_mut_ptr() as u64;
    // SAFETY: the caller keeps this exclusive backing through every owner.
    let mut dma =
        unsafe { RegionAllocator::new(start..start + core::mem::size_of_val(memory) as u64) };
    let nodes = if hub {
        BTreeMap::from([(0, Node::hub(speed, 1)), (1, leaf)])
    } else {
        BTreeMap::from([(0, leaf)])
    };
    let hardware = Rc::new(RefCell::new(Hardware::new(64, nodes)));
    hardware.borrow_mut().root_status = Some(PP | CCS | PED | u32::from(speed) << PORT_SPEED_SHIFT);
    let state = Rc::new(RefCell::new(State::default()));
    let registers = Registers {
        bus: RegistersModel(hardware.clone()),
        state: state.clone(),
    };
    let mut c = Controller::new(
        Box::new(registers),
        clock,
        &mut dma,
        "suspend model".into(),
        crate::minimum_model::owned(clock),
    )
    .unwrap();
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    let shared = Rc::new(RefCell::new(c));
    let host = Host::new(shared, &devices, &mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    (host, devices, hardware, state, dma)
}

/// Bare-slot consumer shaped like the legacy HID/CDC/storage owners.
/// Configuration and initial I/O use the native command/TRB model.
struct RawConsumer {
    controller: Shared,
    slot: u8,
    endpoint: Endpoint,
    buffer: DmaBuffer,
    pending: u64,
    output: u64,
    ring: u64,
}
impl RawConsumer {
    fn retain(host: &Host, device: &Device, dma: &mut dyn DmaAllocator) -> Self {
        let configuration = &device.configurations[0];
        let endpoints = interfaces(configuration)[0].endpoints.clone();
        let endpoint = *endpoints.iter().find(|endpoint| endpoint.is_in()).unwrap();
        let buffer = dma.allocate(64, 64).unwrap();
        let mut c = host.controller.borrow_mut();
        c.set_configuration(device.slot, configuration[5]).unwrap();
        c.configure(device.slot, &endpoints, None, dma).unwrap();
        let pending = c
            .submit(device.slot, endpoint.dci(), buffer.physical(), buffer.len())
            .unwrap();
        let state = c.slot(device.slot).unwrap();
        Self {
            controller: host.controller.clone(),
            slot: device.slot,
            endpoint,
            buffer,
            pending,
            output: state._output.physical(),
            ring: state.endpoints[usize::from(endpoint.dci())]
                .as_ref()
                .unwrap()
                .ring
                .buffer
                .physical(),
        }
    }

    fn submit(&mut self) -> Result<u64, BlockError> {
        self.controller.borrow_mut().submit(
            self.slot,
            self.endpoint.dci(),
            self.buffer.physical(),
            self.buffer.len(),
        )
    }

    fn assert_fenced(&mut self, hardware: &Rc<RefCell<Hardware>>) {
        let before = (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes);
        let (commands, input, control, output, ring, count, poisoned) = {
            let c = self.controller.borrow();
            let state = c.slots[usize::from(self.slot)].as_ref().unwrap();
            let transfer = state.endpoints[usize::from(self.endpoint.dci())]
                .as_ref()
                .unwrap();
            // Quarantine is not a claim that hardware stopped or freed the slot.
            assert_eq!(
                c.slot_ownership[usize::from(self.slot)],
                reset::Ownership::Enabled
            );
            assert_eq!(state._output.physical(), self.output);
            assert_eq!(transfer.ring.buffer.physical(), self.ring);
            (
                c.commands.buffer.bytes().to_vec(),
                state.input.bytes().to_vec(),
                state.control.bytes().to_vec(),
                state._output.bytes().to_vec(),
                transfer.ring.buffer.bytes().to_vec(),
                transfer.in_flight,
                transfer.poisoned,
            )
        };
        let payload = self.buffer.bytes().to_vec();
        // Exercise the legacy owner itself, not only SlotId access.
        assert!(self.submit().is_err());
        let mut c = self.controller.borrow_mut();
        assert!(c.endpoint_ready(self.slot, self.endpoint.dci()).is_err());
        assert!(matches!(
            c.completed(
                self.slot,
                self.endpoint.dci(),
                self.pending,
                self.buffer.len()
            ),
            Some(Err(_))
        ));
        assert!(
            c.control(self.slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                .is_err()
        );
        assert!(c.current_configuration(self.slot).is_err());
        assert!(c.connected(self.slot).is_err());
        assert!(c.cancel(self.slot, self.endpoint.dci()).is_err());
        assert!(c.set_configuration(self.slot, 1).is_err());
        let state = c.slots[usize::from(self.slot)].as_ref().unwrap();
        assert!(state.retiring && state.suspended);
        let transfer = state.endpoints[usize::from(self.endpoint.dci())]
            .as_ref()
            .unwrap();
        assert_eq!(c.commands.buffer.bytes(), commands);
        assert_eq!(state.input.bytes(), input);
        assert_eq!(state.control.bytes(), control);
        assert_eq!(state._output.bytes(), output);
        assert_eq!(transfer.ring.buffer.bytes(), ring);
        assert_eq!(transfer.in_flight, count);
        assert_eq!(transfer.poisoned, poisoned);
        assert_eq!(self.buffer.bytes(), payload);
        assert_eq!(
            (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes),
            before
        );
    }
}

#[test]
fn topology_suspend_uses_real_hub_links_and_controller_save_then_fresh_owners() {
    let _clock = crate::test_clock::virtualize();
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let mut memory = vec![0u64; 1024 * 1024];
        let (mut host, old, hardware, state, mut dma) =
            fixture_with_leaf(&mut memory, speed, true, Node::bot_disk(speed));
        // Host::new keeps topology/hubs; normal class binding separately
        // configures the addressed BOT leaf and publishes its disk.
        let mut original = Found::default();
        for device in old
            .iter()
            .filter(|device| device.descriptor[4] != HUB_CLASS)
        {
            crate::usb::xhci::bind::bind(&host.controller, device.clone(), &mut dma, &mut original);
        }
        assert!(original.failures.is_empty(), "{:?}", original.failures);
        assert_eq!(original.storage.len(), 1);
        assert_eq!(original.disks.len(), 1);
        let retained_payload = [0x5b; 512];
        original.disks[0].write(2, &retained_payload).unwrap();
        let mut original_read = [0xee; 512];
        original.disks[0].read(2, &mut original_read).unwrap();
        assert_eq!(original_read, retained_payload);
        let original_commands = hardware.borrow().nodes[&1].bot_commands().len();
        if speed >= SUPER_SPEED {
            // Seed acknowledged function options in both configured
            // devices. Suspend must clear them through each native EP0 ring.
            for device in &old {
                host.controller
                    .borrow_mut()
                    .control(device.slot, setup(1, SET_FEATURE, 0, 0x0300, 0), &[])
                    .unwrap();
            }
            assert!(hardware.borrow().nodes.values().all(|node| {
                node.functions.len() == 1
                    && node.functions[&0].low_power
                    && node.functions[&0].remote_wake
            }));
        }
        let removed = host.suspend();
        assert!(
            removed.added.failures.is_empty(),
            "{:?}",
            removed.added.failures
        );
        assert_eq!(removed.removed.len(), old.len());
        assert_eq!(state.borrow().save, 1);
        if speed >= SUPER_SPEED {
            assert!(hardware.borrow().nodes.values().all(|node| {
                node.functions.len() == 1
                    && !node.functions[&0].low_power
                    && !node.functions[&0].remote_wake
            }));
        }
        assert_eq!(
            hardware.borrow().root_status.unwrap() & PLS,
            U3 << PLS_SHIFT
        );
        for old in &old {
            assert!(host.controller.borrow_mut().device(old.id).is_err());
        }
        let before = (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes);
        assert!(original.disks[0].read(2, &mut original_read).is_err());
        assert_eq!(
            (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes),
            before
        );
        assert_eq!(
            hardware.borrow().nodes[&1].bot_commands().len(),
            original_commands
        );
        assert!(
            host.poll(&mut dma, &crate::firmware::HandoffProvider::new(None))
                .added
                .devices
                .is_empty()
        );
        let added = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
        assert!(
            added.added.failures.is_empty(),
            "{:?}",
            added.added.failures
        );
        assert_eq!(state.borrow().restore, 1);
        assert_eq!(added.added.devices.len(), old.len());
        for old in &old {
            assert!(host.controller.borrow_mut().device(old.id).is_err());
        }
        for fresh in &added.added.devices {
            assert!(host.controller.borrow_mut().device(fresh.id).is_ok());
        }
        assert!(added.added.storage.is_empty());
        let mut added = host.settled(&mut dma);
        assert_eq!(added.added.storage.len(), 1);
        assert_eq!(added.added.disks.len(), 1);
        let disk = &mut added.added.disks[0];
        assert_eq!(
            (disk.identity().blocks, disk.identity().block_size),
            (16, 512)
        );
        let mut retained_read = [0xee; 512];
        disk.read(2, &mut retained_read).unwrap();
        assert_eq!(retained_read, retained_payload);
        let payload = [0xa7; 512];
        disk.write(3, &payload).unwrap();
        let mut received = [0xee; 512];
        disk.read(3, &mut received).unwrap();
        assert_eq!(received, payload);
        let bus = hardware.borrow();
        assert!(
            bus.requests
                .iter()
                .any(|(_, route, request)| *route == 1 && *request == setup(0xa1, 0xfe, 0, 0, 1))
        );
        assert!(
            !bus.requests
                .iter()
                .any(|(_, _, request)| *request == setup(0x21, 0xff, 0, 0, 0))
        );
        let commands = bus.nodes[&1].bot_commands();
        for opcode in [0x12, 0x00, 0x25, 0x28, 0x2a] {
            assert!(commands.iter().any(|(_, op)| *op == opcode));
        }
        // Each real class binding owns a new BOT tag sequence. Keep both
        // command histories; resetting a physical address does not erase it.
        for binding_commands in [
            &commands[..original_commands],
            &commands[original_commands..],
        ] {
            assert_eq!(binding_commands[0].0, 1);
            assert!(
                binding_commands
                    .windows(2)
                    .all(|pair| pair[1].0 == pair[0].0 + 1)
            );
        }
        drop(bus);
        let before = (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes);
        assert!(original.disks[0].read(2, &mut original_read).is_err());
        assert_eq!(
            (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes),
            before
        );
    }
}

#[test]
fn unacknowledged_root_suspend_never_saves_or_revives_consumers() {
    let mut memory = vec![0u64; 512 * 1024];
    let (mut host, old, hardware, state, mut dma) = fixture(&mut memory, HIGH_SPEED, false);
    let mut consumer = RawConsumer::retain(&host, &old[0], &mut dma);
    state.borrow_mut().refuse_suspend = true;
    let removed = host.suspend();
    assert_eq!(removed.removed.len(), old.len());
    assert!(!removed.added.failures.is_empty());
    assert_eq!(state.borrow().save, 0);
    assert!(matches!(
        host.controller.borrow().hardware,
        hardware::State::Running
    ));
    consumer.assert_fenced(&hardware);
    let before = (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes);
    for device in old {
        assert!(host.controller.borrow_mut().device(device.id).is_err());
    }
    assert_eq!(
        (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes),
        before
    );
    let refused = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
    assert!(!refused.added.failures.is_empty());
    assert_eq!(state.borrow().restore, 0);
}

#[test]
fn post_fence_suspend_errors_quarantine_raw_consumers_without_fake_retirement() {
    for failure in 0..4 {
        let mut memory = vec![0u64; 512 * 1024];
        let (mut host, devices, hardware, state, mut dma) = fixture(&mut memory, HIGH_SPEED, false);
        let mut consumer = RawConsumer::retain(&host, &devices[0], &mut dma);
        let status = setup(0x80, GET_STATUS, 0, 0, 2);
        match failure {
            0 => hardware.borrow_mut().effect_fail = Some((consumer.slot, status)),
            1 => hardware.borrow_mut().effect_silence = Some((consumer.slot, status)),
            2 => hardware.borrow_mut().fail = Some((STOP_ENDPOINT, consumer.slot)),
            3 => hardware.borrow_mut().silent = Some((STOP_ENDPOINT, consumer.slot)),
            _ => unreachable!(),
        }
        let before = hardware.borrow().commands.len();
        let removed = host.suspend();
        assert_eq!(removed.removed.len(), devices.len());
        assert!(!removed.added.failures.is_empty());
        assert_eq!(state.borrow().save, 0);
        assert!(matches!(host.sleep, Sleep::Failed));
        // An unanswered Stop Endpoint kills the host, as xhci_hc_died does.
        assert_eq!(
            matches!(host.controller.borrow().hardware, hardware::State::Died),
            failure == 3
        );
        // These failures precede bulk cancellation. Keep its TD and
        // original buffers even when EP0/command ownership is unresolved.
        assert_eq!(
            host.controller.borrow().slots[usize::from(consumer.slot)]
                .as_ref()
                .unwrap()
                .endpoints[usize::from(consumer.endpoint.dci())]
            .as_ref()
            .unwrap()
            .in_flight,
            1
        );
        assert!(
            hardware.borrow().commands[before..]
                .iter()
                .all(|(kind, _)| *kind != reset::DISABLE_SLOT)
        );
        if failure == 3 {
            assert!(host.controller.borrow().command_poisoned);
        }
        consumer.assert_fenced(&hardware);
    }
}

#[test]
fn failed_external_port_suspend_quarantines_both_parent_and_raw_leaf() {
    let mut memory = vec![0u64; 1024 * 1024];
    let (mut host, devices, hardware, state, mut dma) = fixture(&mut memory, HIGH_SPEED, true);
    let leaf = devices
        .iter()
        .find(|device| device.descriptor[4] != HUB_CLASS)
        .unwrap();
    let parent = devices
        .iter()
        .find(|device| device.descriptor[4] == HUB_CLASS)
        .unwrap();
    let mut consumer = RawConsumer::retain(&host, leaf, &mut dma);
    let suspend = setup(0x23, SET_FEATURE, 2, 1, 0);
    hardware.borrow_mut().effect_fail = Some((parent.slot, suspend));
    let removed = host.suspend();
    assert_eq!(removed.removed.len(), devices.len());
    assert!(!removed.added.failures.is_empty());
    assert!(
        hardware
            .borrow()
            .requests
            .iter()
            .any(|(slot, _, request)| *slot == parent.slot && *request == suspend)
    );
    assert_eq!(state.borrow().save, 0);
    consumer.assert_fenced(&hardware);
    let before = (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes);
    assert!(
        host.controller
            .borrow_mut()
            .control(parent.slot, setup(0xa0, GET_STATUS, 0, 0, 4), &[])
            .is_err()
    );
    assert_eq!(
        (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes),
        before
    );
}

#[test]
fn failed_resume_after_controller_restart_does_not_revive_raw_consumers() {
    let mut memory = vec![0u64; 512 * 1024];
    let (mut host, devices, hardware, state, mut dma) = fixture(&mut memory, HIGH_SPEED, false);
    let mut consumer = RawConsumer::retain(&host, &devices[0], &mut dma);
    let removed = host.suspend();
    assert!(
        removed.added.failures.is_empty(),
        "{:?}",
        removed.added.failures
    );
    assert_eq!(removed.removed.len(), devices.len());
    // A connection-change latch invalidates the restored session even
    // if CCS and speed happen to match the original values.
    *hardware.borrow_mut().root_status.as_mut().unwrap() |= CSC;
    let refused = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
    assert!(!refused.added.failures.is_empty());
    assert!(refused.added.devices.is_empty());
    assert_eq!(state.borrow().restore, 1);
    assert!(matches!(
        host.controller.borrow().hardware,
        hardware::State::Running
    ));
    consumer.assert_fenced(&hardware);
}

#[test]
fn suspend_preflight_failure_preserves_live_raw_consumer() {
    let mut memory = vec![0u64; 512 * 1024];
    let (mut host, devices, hardware, _, mut dma) = fixture(&mut memory, HIGH_SPEED, false);
    let mut consumer = RawConsumer::retain(&host, &devices[0], &mut dma);
    *hardware.borrow_mut().root_status.as_mut().unwrap() |= CSC;
    let unchanged = host.suspend();
    assert!(!unchanged.added.failures.is_empty());
    assert!(unchanged.removed.is_empty());
    assert!(host.sleep.awake());
    assert!(consumer.submit().is_ok());
}

#[test]
fn superspeed_function_suspend_stall_is_reported_and_quarantines_original_consumers() {
    let mut memory = vec![0u64; 512 * 1024];
    let (mut host, devices, hardware, state, mut dma) = fixture(&mut memory, SUPER_SPEED, false);
    let mut consumer = RawConsumer::retain(&host, &devices[0], &mut dma);
    host.controller
        .borrow_mut()
        .control(consumer.slot, setup(1, SET_FEATURE, 0, 0x0300, 0), &[])
        .unwrap();
    let disable = setup(1, SET_FEATURE, 0, 0, 0);
    hardware.borrow_mut().effect_fail = Some((consumer.slot, disable));
    let before = hardware.borrow().commands.len();
    let removed = host.suspend();
    assert_eq!(removed.removed.len(), devices.len());
    assert!(
        removed
            .added
            .failures
            .iter()
            .any(|(_, error)| matches!(error, BlockError::Device { status: 6 }))
    );
    assert!(
        hardware
            .borrow()
            .requests
            .iter()
            .any(|(slot, _, request)| *slot == consumer.slot && *request == disable)
    );
    assert_eq!(state.borrow().save, 0);
    assert!(matches!(host.sleep, Sleep::Failed));
    assert!(matches!(
        host.controller.borrow().hardware,
        hardware::State::Running
    ));
    assert!(
        hardware.borrow().commands[before..]
            .iter()
            .all(|(kind, _)| *kind != reset::DISABLE_SLOT)
    );
    assert_eq!(
        host.controller.borrow().slots[usize::from(consumer.slot)]
            .as_ref()
            .unwrap()
            .endpoints[usize::from(consumer.endpoint.dci())]
        .as_ref()
        .unwrap()
        .in_flight,
        1
    );
    consumer.assert_fenced(&hardware);
    let refused = host.resume(&mut dma, &crate::firmware::HandoffProvider::new(None));
    assert!(!refused.added.failures.is_empty());
    assert!(refused.added.devices.is_empty());
    assert_eq!(state.borrow().restore, 0);
}
