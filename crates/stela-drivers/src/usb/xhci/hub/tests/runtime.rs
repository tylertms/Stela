//! Ring/context/interrupt model coverage of runtime topology ownership.
use super::super::{
    model::*,
    tests::{controller, controller_with_minimum},
    *,
};
use crate::bus::dma::TestRegion;
use alloc::collections::BTreeMap;

fn unbound_leaf(speed: u8) -> Node {
    let mut node = Node::leaf(speed);
    node.interface_class = Some(0xff);
    node
}

fn runtime_poll(host: &mut Host, dma: &mut dyn DmaAllocator) -> Changes {
    host.poll(dma, &crate::firmware::HandoffProvider::new(None))
}

/// A device that a root port finds connected after the start, polled as
/// the service turn does after boot when `after_boot`: the model, the
/// changes and the requests the device saw from its connection on.
fn connect_late(node: Node, after_boot: bool) -> (Rc<RefCell<Hardware>>, Changes, Vec<[u8; 8]>) {
    let _clock = crate::test_clock::virtualize();
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let nodes = BTreeMap::from([(0, node)]);
    let (mut c, model) = controller(&mut dma, 32, nodes);
    model.borrow_mut().root_status = Some(PP);
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert!(devices.is_empty() && failures.is_empty(), "{failures:?}");
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared, &devices, &mut dma, &mut failures);
    let firmware = crate::firmware::HandoffProvider::new(None);
    let begin = model.borrow().requests.len();
    model.borrow_mut().root_status = Some(PP | CCS | CSC | u32::from(HIGH_SPEED) << 10);
    let changes = match after_boot {
        // A connection after boot runs over the polls that follow.
        true => {
            let mut changes = Changes::default();
            for _ in 0..1000 {
                let mut poll = host.poll_after_boot(&mut dma, &firmware);
                changes.added.devices.append(&mut poll.added.devices);
                changes.added.unused.append(&mut poll.added.unused);
                changes.added.failures.append(&mut poll.added.failures);
                changes.added.storage.append(&mut poll.added.storage);
                changes.added.disks.append(&mut poll.added.disks);
                if !host.attachment_due() {
                    break;
                }
            }
            changes
        }
        false => host.poll(&mut dma, &firmware),
    };
    let requests = model.borrow().requests[begin..]
        .iter()
        .map(|(_, _, request)| *request)
        .collect();
    (model, changes, requests)
}

/// A disk that arrives after boot is enumerated and reported unused: no
/// configuration, Bulk-Only command or wait of its start runs in the
/// service turn's poll. At boot the same disk is configured.
#[test]
fn a_disk_that_arrives_after_boot_is_reported_unused_and_not_started() {
    let (model, changes, requests) = connect_late(Node::bot_disk(HIGH_SPEED), true);
    assert_eq!(
        changes.added.devices.len(),
        1,
        "{:?}",
        changes.added.failures
    );
    assert!(changes.added.storage.is_empty() && changes.added.disks.is_empty());
    assert!(
        changes
            .added
            .unused
            .iter()
            .any(|(_, why)| *why == crate::usb::xhci::AFTER_BOOT),
        "{:?}",
        changes.added.unused
    );
    assert!(!model.borrow().nodes[&0].configured);
    assert!(model.borrow().nodes[&0].bot_commands().is_empty());
    // Only enumeration's descriptor reads reached the device.
    assert!(
        requests.iter().all(|request| request[1] == GET_DESCRIPTOR),
        "{requests:x?}"
    );
    let (model, changes, _) = connect_late(Node::bot_disk(HIGH_SPEED), false);
    assert!(
        changes
            .added
            .unused
            .iter()
            .all(|(_, why)| *why != crate::usb::xhci::AFTER_BOOT)
    );
    assert!(model.borrow().nodes[&0].configured);
}

#[test]
fn runtime_hub_bitmap_disconnect_reconnect_retires_old_alias_before_fresh_binding() {
    let _clock = crate::test_clock::virtualize();
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for context in [32, 64] {
            let nodes = BTreeMap::from([(0, Node::hub(speed, 1)), (1, unbound_leaf(speed))]);
            let mut dma = TestRegion::new(512 * 1024 * 8);
            let (mut c, model) = controller(&mut dma, context, nodes);
            let mut failures = Vec::new();
            let devices = c.enumerate(&mut dma, &mut failures);
            assert!(failures.is_empty());
            let old = devices
                .iter()
                .find(|d| d.location.ends_with(".1"))
                .unwrap()
                .slot;
            let shared = Rc::new(RefCell::new(c));
            let old_output = shared.borrow().slots[usize::from(old)]
                .as_ref()
                .unwrap()
                ._output
                .physical();
            let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
            assert!(runtime_poll(&mut host, &mut dma).added.devices.is_empty());
            {
                let mut m = model.borrow_mut();
                let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
                port.status &= !3;
                port.change |= 1;
            }
            let removed = runtime_poll(&mut host, &mut dma);
            assert_eq!(
                removed.removed.iter().map(|d| d.slot).collect::<Vec<_>>(),
                [old]
            );
            assert!(removed.added.devices.is_empty());
            assert!(
                removed.added.failures.is_empty(),
                "{:?}",
                removed.added.failures
            );
            assert_eq!(
                shared.borrow().slot_ownership[usize::from(old)],
                reset::Ownership::Retired
            );
            // The empty port's retired owner draws no current, as hub.c's
            // budget counts only connected children.
            assert!(!shared.borrow().power.retained(old));
            {
                let mut m = model.borrow_mut();
                let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
                port.status |= 3;
                port.change |= 1;
            }
            let mut added = runtime_poll(&mut host, &mut dma);
            if speed == HIGH_SPEED {
                assert!(added.added.devices.is_empty());
                for _ in 0..4 {
                    crate::test_clock::advance(shared.borrow().clock, DEBOUNCE_STEP_US);
                    added = runtime_poll(&mut host, &mut dma);
                    assert!(added.added.failures.is_empty());
                }
            }
            assert!(added.removed.is_empty());
            assert!(
                added.added.failures.is_empty(),
                "{:?}",
                added.added.failures
            );
            assert_eq!(added.added.devices.len(), 1);
            assert_eq!(added.added.devices[0].slot, old + 1);
            assert!(!shared.borrow().power.retained(old));
            assert_eq!(
                shared.borrow().slots[usize::from(old)]
                    .as_ref()
                    .unwrap()
                    ._output
                    .physical(),
                old_output
            );
            let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
            assert!(
                shared
                    .borrow_mut()
                    .control(old, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                    .is_err()
            );
            assert_eq!(
                (model.borrow().mmio_reads, model.borrow().mmio_writes),
                before
            );
            assert!(runtime_poll(&mut host, &mut dma).added.devices.is_empty());
        }
    }
}

#[test]
fn runtime_usb2_hub_debounce_yields_between_samples_and_restarts_on_a_bounce() {
    let _clock = crate::test_clock::virtualize();
    let speed = HIGH_SPEED;
    let mut hub = Node::hub(speed, 1);
    hub.ports.as_mut().unwrap()[0].status &= !3;
    let nodes = BTreeMap::from([(0, hub), (1, unbound_leaf(speed))]);
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let physical = Clock {
        ticks_per_microsecond: 2,
    };
    let (mut c, model) = controller_with_minimum(&mut dma, 64, nodes, physical);
    let clock = c.clock;
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty());
    assert_eq!(devices.len(), 1);
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    runtime_poll(&mut host, &mut dma);

    let arm = || {
        let mut m = model.borrow_mut();
        let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
        port.status |= 1;
        port.change |= 1;
    };
    let reset = setup(0x23, SET_FEATURE, PORT_RESET, 1, 0);
    let start_requests = model.borrow().requests.len();
    let assert_no_reset = || {
        assert!(
            !model.borrow().requests[start_requests..]
                .iter()
                .any(|&(_, _, setup)| setup == reset)
        );
    };
    arm();
    let mut service_start = Clock::ticks();
    let first = runtime_poll(&mut host, &mut dma);
    assert!(first.added.devices.is_empty() && first.added.failures.is_empty());
    assert!(Clock::ticks().wrapping_sub(service_start) < DEBOUNCE_STEP_US);
    assert!(!host.attachment_due());
    assert_no_reset();
    // The nominal clock schedules the callback halfway through the physical
    // minimum. This callback must neither sample the port nor credit time.
    let requests = model.borrow().requests.len();
    crate::test_clock::advance(clock, DEBOUNCE_STEP_US);
    assert!(host.attachment_due());
    let early = runtime_poll(&mut host, &mut dma);
    assert!(early.added.devices.is_empty() && early.added.failures.is_empty());
    assert_eq!(model.borrow().requests.len(), requests);
    for _ in 0..3 {
        crate::test_clock::advance(physical, DEBOUNCE_STEP_US);
        assert!(host.attachment_due());
        service_start = Clock::ticks();
        let changes = runtime_poll(&mut host, &mut dma);
        assert!(changes.added.devices.is_empty() && changes.added.failures.is_empty());
        assert!(Clock::ticks().wrapping_sub(service_start) < DEBOUNCE_STEP_US);
        assert_no_reset();
    }
    // A reconnect with the same final level still invalidates all three
    // observed steps, including when it arrives before the next timer poll.
    arm();
    let bounced = runtime_poll(&mut host, &mut dma);
    assert!(bounced.added.devices.is_empty() && bounced.added.failures.is_empty());
    let bounced_at = Clock::ticks();
    assert_no_reset();
    for _ in 0..3 {
        crate::test_clock::advance(physical, DEBOUNCE_STEP_US);
        assert!(runtime_poll(&mut host, &mut dma).added.devices.is_empty());
        assert_no_reset();
    }
    crate::test_clock::advance(physical, DEBOUNCE_STEP_US);
    let completed = runtime_poll(&mut host, &mut dma);
    assert!(
        completed.added.failures.is_empty(),
        "{:?}",
        completed.added.failures
    );
    assert_eq!(completed.added.devices.len(), 1);
    let m = model.borrow();
    let reset_at = (start_requests..m.requests.len())
        .find(|&i| m.requests[i].2 == reset)
        .unwrap();
    assert!(
        physical
            .elapsed_at(bounced_at, m.request_times[reset_at])
            .unwrap()
            >= DEBOUNCE_STABLE_US
    );
}

#[test]
fn runtime_hub_chain_disconnect_stops_queued_bitmaps_and_children_before_parent_disable() {
    let nodes = BTreeMap::from([
        (0, Node::hub(SUPER_SPEED, 2)),
        (1, Node::hub(SUPER_SPEED, 1)),
        (0x11, unbound_leaf(SUPER_SPEED)),
        (2, unbound_leaf(SUPER_SPEED)),
    ]);
    let mut dma = TestRegion::new(1024 * 1024 * 8);
    let (mut c, model) = controller(&mut dma, 64, nodes);
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty());
    let middle = devices
        .iter()
        .find(|d| d.location.ends_with("port 1.1"))
        .unwrap()
        .slot;
    let leaf = devices
        .iter()
        .find(|d| d.location.ends_with(".1.1"))
        .unwrap()
        .slot;
    let sibling = devices
        .iter()
        .find(|d| d.location.ends_with(".2"))
        .unwrap()
        .slot;
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    runtime_poll(&mut host, &mut dma);
    {
        let mut m = model.borrow_mut();
        let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
        port.status &= !3;
        port.change |= 1;
    }
    let changes = runtime_poll(&mut host, &mut dma);
    assert!(
        changes.added.failures.is_empty(),
        "{:?}",
        changes.added.failures
    );
    assert_eq!(changes.removed.len(), 2);
    let disabled: Vec<_> = model
        .borrow()
        .commands
        .iter()
        .filter(|(kind, _)| *kind == reset::DISABLE_SLOT)
        .map(|(_, slot)| *slot)
        .collect();
    assert_eq!(disabled, [leaf, middle]);
    assert!(
        shared
            .borrow_mut()
            .control(sibling, setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 8), &[])
            .is_ok()
    );
    {
        let mut m = model.borrow_mut();
        let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
        port.status |= 3;
        port.change |= 1;
    }
    let fresh = runtime_poll(&mut host, &mut dma);
    assert!(
        fresh.added.failures.is_empty(),
        "{:?}",
        fresh.added.failures
    );
    assert_eq!(fresh.added.devices.len(), 2);
    assert!(
        fresh
            .added
            .devices
            .iter()
            .all(|device| device.slot != middle && device.slot != leaf)
    );
    assert!(runtime_poll(&mut host, &mut dma).added.devices.is_empty());
}

#[test]
fn runtime_failed_retirement_never_replaces_that_owner_and_sibling_survives() {
    for failed_kind in [STOP_ENDPOINT, reset::DISABLE_SLOT] {
        let nodes = BTreeMap::from([
            (0, Node::hub(SUPER_SPEED, 2)),
            (1, unbound_leaf(SUPER_SPEED)),
            (2, unbound_leaf(SUPER_SPEED)),
        ]);
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty());
        let old = devices
            .iter()
            .find(|d| d.location.ends_with(".1"))
            .unwrap()
            .slot;
        let sibling = devices
            .iter()
            .find(|d| d.location.ends_with(".2"))
            .unwrap()
            .slot;
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        runtime_poll(&mut host, &mut dma);
        {
            let mut m = model.borrow_mut();
            m.fail = Some((failed_kind, old));
            let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
            port.status &= !3;
            port.change |= 1;
        }
        let changes = runtime_poll(&mut host, &mut dma);
        assert_eq!(changes.removed.len(), 1);
        assert!(!changes.added.failures.is_empty());
        assert!(changes.added.devices.is_empty());
        if failed_kind == STOP_ENDPOINT {
            assert!(
                !model
                    .borrow()
                    .commands
                    .iter()
                    .any(|(kind, _)| *kind == reset::DISABLE_SLOT)
            );
        }
        assert_eq!(
            model
                .borrow()
                .commands
                .iter()
                .filter(|(kind, _)| *kind == ADDRESS_DEVICE)
                .count(),
            3
        );
        assert_eq!(
            shared.borrow().slot_ownership[usize::from(old)],
            reset::Ownership::Enabled
        );
        assert!(
            shared.borrow().slots[usize::from(old)]
                .as_ref()
                .unwrap()
                .retiring
        );
        assert!(shared.borrow().power.retained(old));
        assert!(
            shared
                .borrow_mut()
                .control(sibling, setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 8), &[])
                .is_ok()
        );
    }
}

#[test]
fn runtime_empty_controller_keeps_root_events_and_reconnect_never_reuses_old_identity() {
    let nodes = BTreeMap::from([(0, unbound_leaf(SUPER_SPEED))]);
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let (mut c, model) = controller(&mut dma, 64, nodes);
    model.borrow_mut().root_status = Some(PP);
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert!(devices.is_empty());
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    model.borrow_mut().root_status = Some(PP | CCS | PED | CSC | u32::from(SUPER_SPEED) << 10);
    model.borrow_mut().notify_root();
    shared.borrow_mut().drain();
    assert!(
        shared
            .borrow()
            .pending
            .iter()
            .any(|event| event.kind() == reset::PORT_STATUS_CHANGE)
    );
    let attached = runtime_poll(&mut host, &mut dma);
    assert!(
        attached.added.failures.is_empty(),
        "{:?}",
        attached.added.failures
    );
    assert_eq!(attached.added.devices.len(), 1);
    let old = attached.added.devices[0].slot;
    model.borrow_mut().root_status = Some(PP | CSC);
    model.borrow_mut().notify_root();
    let disconnected = runtime_poll(&mut host, &mut dma);
    assert_eq!(disconnected.removed.len(), 1);
    assert_eq!(disconnected.removed[0].slot, old);
    model.borrow_mut().root_status = Some(PP | CCS | PED | CSC | u32::from(SUPER_SPEED) << 10);
    model.borrow_mut().notify_root();
    let reconnected = runtime_poll(&mut host, &mut dma);
    assert!(
        reconnected.added.failures.is_empty(),
        "{:?}",
        reconnected.added.failures
    );
    assert_eq!(reconnected.added.devices.len(), 1);
    assert_eq!(reconnected.added.devices[0].slot, old + 1);
    assert!(shared.borrow_mut().live_slot(old).is_err());
}

#[test]
fn runtime_malformed_bitmap_or_global_power_loss_quarantines_the_hub_tree() {
    for malformed in [false, true] {
        let nodes = BTreeMap::from([
            (0, Node::hub(SUPER_SPEED, 1)),
            (1, unbound_leaf(SUPER_SPEED)),
        ]);
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 64, nodes);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        runtime_poll(&mut host, &mut dma);
        if malformed {
            model.borrow_mut().notification_override = Some(1 << 7);
        } else {
            let mut m = model.borrow_mut();
            m.nodes.get_mut(&0).unwrap().hub_status = 1;
            m.nodes.get_mut(&0).unwrap().hub_change = 1;
        }
        let changed = runtime_poll(&mut host, &mut dma);
        assert_eq!(changed.removed.len(), 2);
        assert!(!changed.added.failures.is_empty());
        assert!(changed.added.devices.is_empty());
        model.borrow_mut().notification_override = None;
        model.borrow_mut().nodes.get_mut(&0).unwrap().hub_status = 0;
        model.borrow_mut().nodes.get_mut(&0).unwrap().hub_change = 0;
        for device in &devices {
            let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
            assert!(
                shared
                    .borrow_mut()
                    .control(device.slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                    .is_err()
            );
            assert_eq!(
                (model.borrow().mmio_reads, model.borrow().mmio_writes),
                before
            );
            assert!(shared.borrow().power.retained(device.slot));
        }
        assert!(runtime_poll(&mut host, &mut dma).added.devices.is_empty());
    }
}

#[test]
fn runtime_repeated_root_attachment_reuses_slots_without_reusing_aliases() {
    let nodes = BTreeMap::from([(0, unbound_leaf(SUPER_SPEED))]);
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let (mut c, model) = controller(&mut dma, 32, nodes);
    // Select a lower software budget than the real fixture's32;
    // no hardware capability or successful allocation is invented.
    c.max_slots = 2;
    c.slots.truncate(3);
    c.slot_ownership.truncate(3);
    model.borrow_mut().slot_limit = 2;
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert_eq!(devices.len(), 1);
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    // Each unplug frees the slot for the next device, as xhci_free_dev
    // does, well past the two-slot budget; no old identity reaches it.
    let mut previous = devices[0].id;
    for _ in 0..3 {
        model.borrow_mut().root_status = Some(PP | CSC);
        let detached = runtime_poll(&mut host, &mut dma);
        assert_eq!(detached.removed.len(), 1);
        let gone = previous.hardware_slot();
        assert!(!shared.borrow().power.retained(gone));
        assert_eq!(
            shared.borrow().slot_ownership[usize::from(gone)],
            reset::Ownership::Retired
        );
        model.borrow_mut().root_status = Some(PP | CCS | PED | CSC | u32::from(SUPER_SPEED) << 10);
        let attached = runtime_poll(&mut host, &mut dma);
        assert_eq!(
            attached.added.devices.len(),
            1,
            "{:?}",
            attached.added.failures
        );
        let fresh = attached.added.devices[0].id;
        assert_ne!(fresh, previous);
        assert!(shared.borrow_mut().device(previous).is_err());
        previous = fresh;
    }
    assert_eq!(
        model
            .borrow()
            .commands
            .iter()
            .filter(|(kind, _)| *kind == ADDRESS_DEVICE)
            .count(),
        4
    );
    assert!(shared.borrow_mut().device(previous).is_ok());
}

#[test]
fn runtime_unknown_root_protocol_or_failed_notification_retires_before_any_new_address() {
    for bad_notification in [false, true] {
        let nodes = BTreeMap::from([(0, unbound_leaf(SUPER_SPEED))]);
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 64, nodes);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        let old = devices[0].slot;
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        if bad_notification {
            model.borrow_mut().notify_root();
            model.borrow_mut().notify_root_code(5);
        } else {
            model.borrow_mut().root_status = Some(PP | CCS | PED);
            model.borrow_mut().notify_root();
        }
        let lost = runtime_poll(&mut host, &mut dma);
        assert_eq!(lost.removed.len(), 1);
        assert!(!lost.added.failures.is_empty());
        assert!(lost.added.devices.is_empty());
        assert_eq!(
            model
                .borrow()
                .commands
                .iter()
                .filter(|(kind, _)| *kind == ADDRESS_DEVICE)
                .count(),
            1
        );
        let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
        assert!(
            shared
                .borrow_mut()
                .control(old, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                .is_err()
        );
        assert_eq!(
            (model.borrow().mmio_reads, model.borrow().mmio_writes),
            before
        );
        model.borrow_mut().root_status = Some(PP | CCS | PED | CSC | u32::from(SUPER_SPEED) << 10);
        model.borrow_mut().notify_root();
        let fresh = runtime_poll(&mut host, &mut dma);
        assert!(
            fresh.added.failures.is_empty(),
            "{:?}",
            fresh.added.failures
        );
        assert_eq!(fresh.added.devices.len(), 1);
        assert_ne!(fresh.added.devices[0].slot, old);
        assert!(shared.borrow_mut().live_slot(old).is_err());
    }
}

#[test]
fn runtime_usb2_multi_tt_bitmap_uses_acknowledged_alternate_and_physical_port31() {
    let mut hub = Node::hub(HIGH_SPEED, 31);
    hub.multi_tt = true;
    for port in &mut hub.ports.as_mut().unwrap()[..30] {
        port.status = 0;
        port.change = 0;
    }
    hub.ports.as_mut().unwrap()[30].child_speed = FULL_SPEED;
    let nodes = BTreeMap::from([(0, hub), (15, unbound_leaf(FULL_SPEED))]);
    let mut dma = TestRegion::new(1024 * 1024 * 8);
    let (mut c, model) = controller(&mut dma, 64, nodes);
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 2);
    let old = devices
        .iter()
        .find(|d| d.location.ends_with(".31"))
        .unwrap()
        .slot;
    assert_eq!(
        c.slots[usize::from(old)]
            .as_ref()
            .unwrap()
            .attachment
            .translator,
        Some((1, 31))
    );
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    runtime_poll(&mut host, &mut dma);
    {
        let mut m = model.borrow_mut();
        let port = &mut m.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[30];
        port.status &= !3;
        port.change = 1;
    }
    let removed = runtime_poll(&mut host, &mut dma);
    assert!(
        removed.added.failures.is_empty(),
        "{:?}",
        removed.added.failures
    );
    assert_eq!(removed.removed.len(), 1);
    assert_eq!(removed.removed[0].slot, old);
}

#[test]
fn runtime_usb2_initial_ped_zero_is_baselined_from_reset_ack_without_churning_identity() {
    for context in [32, 64] {
        let nodes = BTreeMap::from([(0, unbound_leaf(HIGH_SPEED))]);
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, context, nodes);
        model.borrow_mut().root_status = Some(PP);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        assert!(devices.is_empty());
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        model.borrow_mut().root_status = Some(PP | CCS | CSC); // PED0/speed0 until reset.
        model.borrow_mut().notify_root();
        let first = runtime_poll(&mut host, &mut dma);
        assert!(
            first.added.failures.is_empty(),
            "{:?}",
            first.added.failures
        );
        assert_eq!(first.added.devices.len(), 1);
        let slot = first.added.devices[0].slot;
        let commands = model.borrow().commands.clone();
        for _ in 0..4 {
            let stable = runtime_poll(&mut host, &mut dma);
            assert!(stable.removed.is_empty());
            assert!(stable.added.devices.is_empty());
            assert!(stable.added.failures.is_empty());
        }
        assert_eq!(model.borrow().commands, commands);
        assert!(
            shared
                .borrow_mut()
                .control(slot, setup(0x80, GET_DESCRIPTOR, 1 << 8, 0, 8), &[])
                .is_ok()
        );
    }
}

#[test]
fn runtime_hub_recovery_acknowledges_cools_rechecks_and_publishes_fresh_subtree() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for port_oc in [false, true] {
            for context in [32, 64] {
                let nodes = BTreeMap::from([
                    (0, Node::hub(speed, 2)),
                    (1, unbound_leaf(speed)),
                    (2, unbound_leaf(speed)),
                ]);
                let mut dma = TestRegion::new(1024 * 1024 * 8);
                let (mut c, model) = controller(&mut dma, context, nodes);
                model.borrow_mut().root_status = Some(PP | CCS | PED | u32::from(speed) << 10);
                let mut failures = Vec::new();
                let devices = c.enumerate(&mut dma, &mut failures);
                assert!(failures.is_empty(), "{failures:?}");
                let root = devices
                    .iter()
                    .find(|d| d.descriptor[4] == HUB_CLASS)
                    .unwrap()
                    .slot;
                let shared = Rc::new(RefCell::new(c));
                let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
                runtime_poll(&mut host, &mut dma);
                let begin = model.borrow().requests.len();
                {
                    let mut m = model.borrow_mut();
                    let hub = m.nodes.get_mut(&0).unwrap();
                    if port_oc {
                        hub.ports.as_mut().unwrap()[0].change = 8;
                    } else {
                        hub.hub_change = 2;
                    }
                }
                let changes = runtime_poll(&mut host, &mut dma);
                assert!(
                    changes.added.failures.is_empty(),
                    "{:?}",
                    changes.added.failures
                );
                assert_eq!(changes.removed.len(), 3);
                assert_eq!(changes.added.devices.len(), 3);
                for old in &devices {
                    assert!(
                        changes
                            .added
                            .devices
                            .iter()
                            .all(|fresh| fresh.slot != old.slot)
                    );
                    let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
                    assert!(
                        shared
                            .borrow_mut()
                            .control(old.slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                            .is_err()
                    );
                    assert_eq!(
                        (model.borrow().mmio_reads, model.borrow().mmio_writes),
                        before
                    );
                    assert!(!shared.borrow().power.retained(old.slot));
                }
                let m = model.borrow();
                let ack_request = if port_oc {
                    setup(0x23, CLEAR_FEATURE, 19, 1, 0)
                } else {
                    setup(0x20, CLEAR_FEATURE, 1, 0, 0)
                };
                let ack = (begin..m.requests.len())
                    .find(|&i| m.requests[i].0 == root && m.requests[i].2 == ack_request)
                    .unwrap();
                let power = (ack + 1..m.requests.len())
                    .find(|&i| {
                        m.requests[i].0 == root
                            && m.requests[i].2 == setup(0x23, SET_FEATURE, PORT_POWER, 1, 0)
                    })
                    .unwrap();
                assert!(
                    m.request_times[power] - m.request_times[ack]
                        >= if port_oc { 100_000 } else { 500_000 }
                );
                let last_power = (power..m.requests.len())
                    .rfind(|&i| m.requests[i].0 == root && m.requests[i].2[1] == SET_FEATURE)
                    .unwrap();
                let recheck = (last_power + 1..m.requests.len())
                    .find(|&i| {
                        m.requests[i].0 == root
                            && m.requests[i].2 == setup(0xa0, GET_STATUS, 0, 0, 4)
                    })
                    .unwrap();
                assert!(m.request_times[recheck] - m.request_times[last_power] >= 510_000);
                drop(m);
                let stable = runtime_poll(&mut host, &mut dma);
                assert!(stable.removed.is_empty());
                assert!(stable.added.devices.is_empty());
                assert!(
                    stable.added.failures.is_empty(),
                    "{:?}",
                    stable.added.failures
                );
            }
        }
    }
}

/// A root port's device whose slot went while it stayed connected (a
/// refused enumeration) keeps its grant with no owner. The port's polls
/// then find no owner and no failure, where each once refused the port,
/// and its disconnection is seen and releases the grant.
#[test]
fn runtime_root_port_with_an_ownerless_grant_is_polled_and_released_on_disconnect() {
    let speed = HIGH_SPEED;
    let nodes = BTreeMap::from([(0, unbound_leaf(speed))]);
    let mut dma = TestRegion::new(1024 * 1024 * 8);
    let (mut c, model) = controller(&mut dma, 64, nodes);
    let status = PP | CCS | PED | u32::from(speed) << 10;
    model.borrow_mut().root_status = Some(status);
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert_eq!(devices.len(), 1);
    let slot = devices[0].slot;
    let shared = Rc::new(RefCell::new(c));
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    runtime_poll(&mut host, &mut dma);
    shared.borrow_mut().slots[usize::from(slot)] = None;
    assert_eq!(shared.borrow().current_port_owner(None, 1), Ok(None));
    for _ in 0..3 {
        let changed = runtime_poll(&mut host, &mut dma);
        assert!(
            changed
                .added
                .failures
                .iter()
                .all(|(location, _)| !location.contains("root port")),
            "{:?}",
            changed.added.failures
        );
        assert!(shared.borrow().power.retained(slot));
    }
    model.borrow_mut().root_status = Some(PP | CSC);
    let changed = runtime_poll(&mut host, &mut dma);
    assert!(
        changed
            .added
            .failures
            .iter()
            .all(|(location, _)| !location.contains("root port")),
        "{:?}",
        changed.added.failures
    );
    assert!(!shared.borrow().power.retained(slot));
}

#[test]
fn runtime_persistent_hub_power_fault_keeps_public_aliases_fenced_until_restoration() {
    for local_loss in [false, true] {
        let speed = SUPER_SPEED;
        let nodes = BTreeMap::from([(0, Node::hub(speed, 1)), (1, unbound_leaf(speed))]);
        let mut dma = TestRegion::new(1024 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 64, nodes);
        model.borrow_mut().root_status = Some(PP | CCS | PED | u32::from(speed) << 10);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        runtime_poll(&mut host, &mut dma);
        {
            let mut m = model.borrow_mut();
            let hub = m.nodes.get_mut(&0).unwrap();
            hub.hub_status = if local_loss { 1 } else { 2 };
            hub.hub_change = hub.hub_status;
        }
        for iteration in 0..3 {
            let changed = runtime_poll(&mut host, &mut dma);
            assert_eq!(changed.removed.len(), if iteration == 0 { 2 } else { 0 });
            assert!(changed.added.devices.is_empty());
            // The fault is reported as it begins and as recovery first
            // finds it standing; the same refusal is not repeated after.
            assert_eq!(changed.added.failures.is_empty(), iteration == 2);
            for old in &devices {
                assert!(shared.borrow_mut().live_slot(old.slot).is_err());
                assert!(shared.borrow().power.retained(old.slot));
            }
        }
        {
            let mut m = model.borrow_mut();
            let hub = m.nodes.get_mut(&0).unwrap();
            hub.hub_status = 0;
            hub.hub_change = if local_loss { 1 } else { 2 };
        }
        let restored = runtime_poll(&mut host, &mut dma);
        assert!(
            restored.added.failures.is_empty(),
            "{:?}",
            restored.added.failures
        );
        assert_eq!(restored.added.devices.len(), 2);
        assert!(
            devices
                .iter()
                .all(|old| shared.borrow_mut().live_slot(old.slot).is_err())
        );
    }
}

#[test]
fn runtime_hub_recovery_uncertain_ack_stop_or_disable_never_authorizes_fresh_address() {
    for fault in 0..9 {
        let speed = SUPER_SPEED;
        let nodes = BTreeMap::from([(0, Node::hub(speed, 1)), (1, unbound_leaf(speed))]);
        let mut dma = TestRegion::new(1024 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 64, nodes);
        model.borrow_mut().root_status = Some(PP | CCS | PED | u32::from(speed) << 10);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        let hub = devices
            .iter()
            .find(|d| d.descriptor[4] == HUB_CLASS)
            .unwrap()
            .slot;
        let child = devices.iter().find(|d| d.slot != hub).unwrap().slot;
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        runtime_poll(&mut host, &mut dma);
        let first = model.borrow().requests.len();
        {
            let mut m = model.borrow_mut();
            m.nodes.get_mut(&0).unwrap().hub_change = 2;
            match fault {
                0 => m.fail = Some((STOP_ENDPOINT, child)),
                1 => m.fail = Some((reset::DISABLE_SLOT, child)),
                2 => m.effect_fail = Some((hub, setup(0x20, CLEAR_FEATURE, 1, 0, 0))),
                3 => m.effect_fail = Some((hub, setup(0x23, SET_FEATURE, PORT_POWER, 1, 0))),
                4 => m.root_loss_after = Some(setup(0x20, CLEAR_FEATURE, 1, 0, 0)),
                5 => m.root_loss_after = Some(setup(0x23, SET_FEATURE, PORT_POWER, 1, 0)),
                6 => m.fail = Some((reset::DISABLE_SLOT, hub)),
                7 => m.effect_silence = Some((hub, setup(0x20, CLEAR_FEATURE, 1, 0, 0))),
                8 => m.effect_silence = Some((hub, setup(0x23, SET_FEATURE, PORT_POWER, 1, 0))),
                _ => unreachable!(),
            }
        }
        let stopped = runtime_poll(&mut host, &mut dma);
        assert_eq!(stopped.removed.len(), 2);
        assert!(stopped.added.devices.is_empty());
        assert!(!stopped.added.failures.is_empty());
        assert!(
            devices
                .iter()
                .all(|old| shared.borrow().power.retained(old.slot))
        );
        assert_eq!(
            model
                .borrow()
                .commands
                .iter()
                .filter(|(kind, _)| *kind == ADDRESS_DEVICE)
                .count(),
            2
        );
        if fault <= 2 || fault == 4 || fault == 7 {
            assert!(
                model.borrow().requests[first..]
                    .iter()
                    .all(|(_, _, request)| *request != setup(0x23, SET_FEATURE, PORT_POWER, 1, 0))
            );
        }
        for old in &devices {
            let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
            assert!(
                shared
                    .borrow_mut()
                    .control(old.slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                    .is_err()
            );
            assert_eq!(
                (model.borrow().mmio_reads, model.borrow().mmio_writes),
                before
            );
        }
        // An acknowledged request error after its physical effect still does
        // not count as success. Only a later complete observed recovery does.
        if matches!(fault, 2 | 3 | 7 | 8) {
            let retried = runtime_poll(&mut host, &mut dma);
            assert!(
                retried.added.failures.is_empty(),
                "{:?}",
                retried.added.failures
            );
            assert_eq!(retried.added.devices.len(), 2);
        }
    }
}

#[test]
fn runtime_nested_hub_recovery_preserves_unaffected_upstream_sibling() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let nodes = BTreeMap::from([
            (0, Node::hub(speed, 2)),
            (1, Node::hub(speed, 1)),
            (0x11, unbound_leaf(speed)),
            (2, unbound_leaf(speed)),
        ]);
        let mut dma = TestRegion::new(1024 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 64, nodes);
        model.borrow_mut().root_status = Some(PP | CCS | PED | u32::from(speed) << 10);
        let mut failures = Vec::new();
        let devices = c.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        let sibling = devices
            .iter()
            .find(|d| d.location.ends_with(".2"))
            .unwrap()
            .slot;
        let shared = Rc::new(RefCell::new(c));
        let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
        runtime_poll(&mut host, &mut dma);
        model.borrow_mut().nodes.get_mut(&1).unwrap().hub_change = 2;
        let changed = runtime_poll(&mut host, &mut dma);
        assert!(
            changed.added.failures.is_empty(),
            "{:?}",
            changed.added.failures
        );
        assert_eq!(changed.removed.len(), 2);
        assert_eq!(changed.added.devices.len(), 2);
        assert!(changed.removed.iter().all(|old| old.slot != sibling));
        assert!(
            shared
                .borrow_mut()
                .control(sibling, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                .is_ok()
        );
        assert!(runtime_poll(&mut host, &mut dma).added.devices.is_empty());
    }
}

#[test]
fn runtime_hub_recovery_repower_never_expands_the_reserved_current_contract() {
    let mut hub = Node::hub(SUPER_SPEED, 7);
    hub.self_powered = false;
    hub.max_power = 112;
    hub.controller_current = 20;
    let mut nodes = BTreeMap::from([(0, hub)]);
    for port in 1..=7 {
        nodes.insert(port, unbound_leaf(SUPER_SPEED));
    }
    let mut dma = TestRegion::new(1024 * 1024 * 8);
    let (mut c, model) = controller(&mut dma, 64, nodes);
    model.borrow_mut().root_status = Some(PP | CCS | PED | u32::from(SUPER_SPEED) << 10);
    let mut failures = Vec::new();
    let devices = c.enumerate(&mut dma, &mut failures);
    assert_eq!(devices.len(), 6);
    assert_eq!(failures.len(), 2);
    let shared = Rc::new(RefCell::new(c));
    let original = shared.borrow().power.snapshot(1);
    let mut host = Host::new(shared.clone(), &devices, &mut dma, &mut failures);
    runtime_poll(&mut host, &mut dma);
    let begin = model.borrow().requests.len();
    {
        let mut m = model.borrow_mut();
        m.nodes.get_mut(&0).unwrap().hub_status = 2;
        m.nodes.get_mut(&0).unwrap().hub_change = 2;
    }
    for _ in 0..2 {
        let changed = runtime_poll(&mut host, &mut dma);
        assert!(changed.added.devices.is_empty());
        assert!(!changed.added.failures.is_empty());
        assert_eq!(shared.borrow().power.snapshot(1), original);
    }
    let m = model.borrow();
    let repowered: Vec<_> = m.requests[begin..]
        .iter()
        .filter(|(slot, _, q)| {
            *slot == 1 && q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == PORT_POWER as u8
        })
        .map(|(_, _, q)| q[4])
        .collect();
    assert_eq!(repowered, [1, 2, 3, 4, 5, 1, 2, 3, 4, 5]);
}
