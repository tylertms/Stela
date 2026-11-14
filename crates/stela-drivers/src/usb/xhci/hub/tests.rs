extern crate std;
use super::{model::*, *};
use crate::bus::dma::{RegionAllocator, TestRegion};
use alloc::collections::BTreeMap;

pub(in super::super) fn controller(
    dma: &mut dyn DmaAllocator,
    context: usize,
    nodes: BTreeMap<u32, Node>,
) -> (Controller, Rc<RefCell<Hardware>>) {
    controller_with_minimum(
        dma,
        context,
        nodes,
        Clock {
            ticks_per_microsecond: 1,
        },
    )
}

fn controller_with_minimum(
    dma: &mut dyn DmaAllocator,
    context: usize,
    nodes: BTreeMap<u32, Node>,
    physical: Clock,
) -> (Controller, Rc<RefCell<Hardware>>) {
    let model = Rc::new(RefCell::new(Hardware::new(context, nodes)));
    let controller = Controller::new(
        Box::new(RegistersModel(model.clone())),
        Clock {
            ticks_per_microsecond: 1,
        },
        dma,
        "hub model".into(),
        crate::minimum_model::owned(physical),
    )
    .unwrap();
    (controller, model)
}
fn nodes() -> BTreeMap<u32, Node> {
    BTreeMap::from([(0, Node::hub(SUPER_SPEED, 1)), (1, Node::leaf(SUPER_SPEED))])
}
fn bos() -> Vec<u8> {
    let mut bytes = vec![5, 15, 25, 0, 1, 20, 16, 10, 0, 1, 0, 0, 0, 1, 0, 0, 0];
    for ty in [0u32, 2] {
        bytes.extend_from_slice(&(1 | 3 << 4 | ty << 6 | 1 << 14 | 10 << 16).to_le_bytes());
    }
    bytes
}
fn fault_node(fault: Fault) -> Node {
    let mut node = Node::hub(SUPER_SPEED, 1);
    let port = &mut node.ports.as_mut().unwrap()[0];
    port.fault = fault;
    match fault {
        Fault::Suspended => port.status |= 3 << 5,
        Fault::Inactive => port.status |= 6 << 5,
        Fault::WarmForever => port.status |= 10 << 5,
        Fault::UnknownSpeed => {
            node.bos = Some(bos());
            node.speed = SUPER_SPEED_PLUS;
            node.ports.as_mut().unwrap()[0].extended = 0x22;
        }
        _ => {}
    }
    node
}

#[test]
fn superspeed_cold_hub_uses_contexts_depth_power_routes_and_bulk_io() {
    for context in [32, 64] {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, context, nodes());
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        assert_eq!(devices.len(), 2);
        let child = devices
            .iter()
            .find(|d| d.descriptor[4] != HUB_CLASS)
            .unwrap();
        assert_eq!(child.speed, SUPER_SPEED);
        assert_eq!(child.location, "hub model port 1.1");
        let actual = controller.slots[usize::from(child.slot)].as_ref().unwrap();
        assert_eq!(actual.attachment.route, 1);
        assert_eq!(actual.attachment.translator, None);
        assert!(!actual.retiring);
        let endpoints = interfaces(&child.configuration)[0].endpoints.clone();
        controller.set_configuration(child.slot, 1).unwrap();
        controller
            .configure(child.slot, &endpoints, None, &mut dma)
            .unwrap();
        let mut payload = dma.allocate(64, 64).unwrap();
        payload.bytes_mut().fill(0xee);
        assert_eq!(
            controller.transfer(child.slot, 3, payload.physical(), 64, 50_000_000),
            Ok(64)
        );
        assert_eq!(payload.bytes(), &[child.slot; 64]);
        assert_eq!(
            controller.transfer(child.slot, 4, payload.physical(), 64, 50_000_000),
            Ok(64)
        );
        assert_eq!(model.borrow().outgoing, [vec![child.slot; 64]]);
        let state = model.borrow();
        let hub = state
            .contexts
            .iter()
            .find(|(slot, kind, _)| *slot == 1 && *kind == CONFIGURE_ENDPOINT)
            .unwrap()
            .2;
        assert_ne!(hub[0] & 1 << 26, 0);
        assert_eq!(hub[0] & 1 << 25, 0);
        assert_eq!(hub[1] >> 24, 1);
        assert_eq!(hub[2], 0);
        let requests = &state.requests;
        let config = requests
            .iter()
            .position(|(_, _, q)| q[1] == SET_CONFIGURATION)
            .unwrap();
        let depth = requests
            .iter()
            .position(|(_, _, q)| q[0] == 0x20 && q[1] == 12)
            .unwrap();
        let power = requests
            .iter()
            .position(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == 8)
            .unwrap();
        let reset = requests
            .iter()
            .position(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == 4)
            .unwrap();
        assert!(config < depth && depth < power && power < reset);
        assert!(
            requests
                .iter()
                .any(|(_, _, q)| q[0] == 0xa0 && q[2..4] == [0, 0x2a] && q[6] == 12)
        );
        assert!(requests.iter().all(|(_, _, q)| !(q[0] == 0x23
            && q[1] == CLEAR_FEATURE
            && [17, 18, 23].contains(&q[2]))));
    }
}

#[test]
fn nested_superspeed_hubs_preserve_five_route_nibbles_and_no_usb2_tt() {
    let mut nodes = BTreeMap::new();
    for tier in 0..5 {
        nodes.insert(
            (0..tier).fold(0, |r, t| r | 15 << (4 * t)),
            Node::hub(SUPER_SPEED, 15),
        );
    }
    nodes.insert(0xfffff, Node::leaf(SUPER_SPEED));
    for node in nodes.values_mut().filter(|node| node.ports.is_some()) {
        for port in &mut node.ports.as_mut().unwrap()[..14] {
            port.status = 0;
            port.change = 0;
        }
    }
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 64, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 6);
    let leaf = devices
        .iter()
        .find(|d| d.descriptor[4] != HUB_CLASS)
        .unwrap();
    assert_eq!(
        controller.slots[usize::from(leaf.slot)]
            .as_ref()
            .unwrap()
            .attachment
            .route,
        0xfffff
    );
    let state = model.borrow();
    for tier in 0..5 {
        let route = (0..tier).fold(0, |r, t| r | 15 << (4 * t));
        assert_eq!(state.nodes[&route].depth, Some(tier));
    }
    assert!(state.contexts.iter().all(|(_, _, ctx)| ctx[2] == 0));
}

#[test]
fn superspeedplus_uses_bos_speed_ids_and_extended_status() {
    for lanes in [1, 2] {
        let mut nodes = nodes();
        let root = nodes.get_mut(&0).unwrap();
        root.speed = SUPER_SPEED_PLUS;
        root.bos = Some(bos());
        root.ports.as_mut().unwrap()[0].extended = 0x11 | (lanes - 1) << 8 | (lanes - 1) << 12;
        nodes.get_mut(&1).unwrap().speed = SUPER_SPEED_PLUS;
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        assert_eq!(devices.len(), 2);
        let child = devices
            .iter()
            .find(|d| d.descriptor[4] != HUB_CLASS)
            .unwrap();
        assert_eq!(child.speed, SUPER_SPEED_PLUS);
        assert!(
            model
                .borrow()
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0xa3 && q[2] == 2 && q[6] == 8)
        );
    }
}

#[test]
fn asymmetric_or_unknown_ssp_port_status_never_inherits_upstream_speed() {
    for extended in [0x21, 0x111, 0x10011] {
        let mut nodes = nodes();
        let root = nodes.get_mut(&0).unwrap();
        root.speed = SUPER_SPEED_PLUS;
        root.bos = Some(bos());
        root.ports.as_mut().unwrap()[0].extended = extended;
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert_eq!(devices.len(), 1);
        assert_eq!(failures.len(), 1);
        assert!(
            model
                .borrow()
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0xa3 && q[2] == 2 && q[6] == 8)
        );
        assert_eq!(
            model
                .borrow()
                .commands
                .iter()
                .filter(|(k, _)| *k == ENABLE_SLOT)
                .count(),
            1
        );
    }
}

#[test]
fn hub_power_and_overcurrent_health_gate_all_downstream_admission() {
    for (self_powered, status, change, after) in [
        (true, 1, 0, None),
        (true, 2, 0, None),
        (true, 4, 0, None),
        (true, 0, 2, None),
        (true, 0, 4, None),
        (true, 0, 0, Some(2)),
        (true, 0, 0, Some(1)),
    ] {
        let mut nodes = nodes();
        let root = nodes.get_mut(&0).unwrap();
        root.self_powered = self_powered;
        root.hub_status = status;
        root.hub_change = change;
        root.after_power_status = after;
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert_eq!(devices.len(), 1);
        assert_eq!(failures.len(), 1);
        assert_eq!(
            model
                .borrow()
                .commands
                .iter()
                .filter(|(kind, _)| *kind == ENABLE_SLOT)
                .count(),
            1
        );
        assert!(
            !model
                .borrow()
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == 4)
        );
    }
}

#[test]
fn historical_local_power_change_is_acknowledged_and_health_rechecked() {
    let mut nodes = nodes();
    nodes.get_mut(&0).unwrap().hub_change = 1;
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 2);
    assert!(
        model
            .borrow()
            .requests
            .iter()
            .any(|(_, _, q)| q[0] == 0x20 && q[1] == CLEAR_FEATURE && q[2..6] == [0; 4])
    );
}

#[test]
fn port_admission_refuses_power_lifecycle_reset_and_status_failures() {
    for fault in [
        Fault::Power,
        Fault::PowerAbsent,
        Fault::Reset,
        Fault::ResetForever,
        Fault::NoEnable,
        Fault::Suspended,
        Fault::UnknownSpeed,
        Fault::WarmForever,
        Fault::ShortStatus,
        Fault::ClearChange,
        Fault::LinkNotU0,
        Fault::ConfigError,
        Fault::OverCurrentChange,
    ] {
        let mut nodes = nodes();
        nodes.insert(0, fault_node(fault));
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert_eq!(devices.len(), 1, "{fault:?}: {failures:?}");
        assert_eq!(failures.len(), 1, "{fault:?}");
        assert_eq!(
            model
                .borrow()
                .commands
                .iter()
                .filter(|(kind, _)| *kind == ENABLE_SLOT)
                .count(),
            1,
            "{fault:?}"
        );
        if fault == Fault::NoEnable {
            assert_eq!(
                model.borrow().nodes[&0].ports.as_ref().unwrap()[0].reset_count,
                5
            );
        }
        let hub = controller.slots[1].as_ref().unwrap();
        assert!(hub.endpoints[1].is_some());
        assert_eq!(controller.slot_ownership[1], reset::Ownership::Enabled);
    }
}

/// Linux `hub_port_warm_reset_required` warm-resets a SuperSpeed hub port
/// whose link sits in SS.Inactive even when it shows no connection; the
/// device behind it is then found.
#[test]
fn an_inactive_hub_port_without_a_connection_is_warm_reset_and_enumerated() {
    let mut nodes = nodes();
    let mut hub = fault_node(Fault::Inactive);
    let port = &mut hub.ports.as_mut().unwrap()[0];
    port.status &= !1;
    port.change = 0;
    nodes.insert(0, hub);
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 2);
    assert!(
        model
            .borrow()
            .requests
            .iter()
            .any(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == 28)
    );
}

#[test]
fn inactive_and_failed_hot_reset_require_real_bounded_warm_reset() {
    for fault in [Fault::Inactive, Fault::HotThenWarm] {
        let mut nodes = nodes();
        nodes.insert(0, fault_node(fault));
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{fault:?}: {failures:?}");
        assert_eq!(devices.len(), 2);
        let state = model.borrow();
        assert!(
            state
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == 28)
        );
        assert!(
            state
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0x23 && q[1] == CLEAR_FEATURE && q[2] == 29)
        );
    }
}

#[test]
fn disconnect_during_reset_does_not_publish_a_child_or_recycle_hub_owners() {
    let mut nodes = nodes();
    nodes.insert(0, fault_node(Fault::Disconnect));
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty());
    assert_eq!(devices.len(), 1);
    assert_eq!(
        model
            .borrow()
            .commands
            .iter()
            .filter(|(k, _)| *k == ENABLE_SLOT)
            .count(),
        1
    );
    assert!(
        controller.slots[1].as_ref().unwrap().endpoints[3]
            .as_ref()
            .unwrap()
            .enabled
    );
}

#[test]
fn failed_or_uncertain_hub_configuration_never_sends_depth_or_power() {
    for silent in [false, true] {
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes());
        if silent {
            model.borrow_mut().silent = Some((CONFIGURE_ENDPOINT, 1));
        } else {
            model.borrow_mut().fail = Some((CONFIGURE_ENDPOINT, 1));
        }
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert_eq!(devices.len(), 1);
        assert_eq!(failures.len(), 1);
        assert!(
            !model
                .borrow()
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0x20 || q[0] == 0x23)
        );
        let endpoint = controller.slots[1].as_ref().unwrap().endpoints[3]
            .as_ref()
            .unwrap();
        assert!(!endpoint.enabled && endpoint.poisoned);
        // A Configure Endpoint that never completes is aborted; the command
        // ring carries on.
        assert!(!controller.command_poisoned);
    }
}

#[test]
fn usb2_hub_refactor_preserves_full_speed_child_transaction_translator() {
    let mut nodes = BTreeMap::from([(0, Node::hub(HIGH_SPEED, 1)), (1, Node::leaf(FULL_SPEED))]);
    nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0].child_speed = FULL_SPEED;
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 2);
    let child = devices
        .iter()
        .find(|d| d.descriptor[4] != HUB_CLASS)
        .unwrap();
    assert_eq!(
        controller.slots[usize::from(child.slot)]
            .as_ref()
            .unwrap()
            .attachment
            .translator,
        Some((1, 1))
    );
    let state = model.borrow();
    let hub = state
        .contexts
        .iter()
        .find(|(s, k, _)| *s == 1 && *k == CONFIGURE_ENDPOINT)
        .unwrap()
        .2;
    assert_eq!(hub[2] >> 16 & 3, 3);
    let child = state
        .contexts
        .iter()
        .find(|(s, k, _)| *s == 2 && *k == ADDRESS_DEVICE)
        .unwrap()
        .2;
    assert_eq!(child[0] & 0xfffff, 1);
    assert_eq!(child[2], 1 | 1 << 8);
}

#[test]
fn malformed_hub_descriptors_and_bos_never_reach_configuration_or_child_allocation() {
    let valid = Node::hub(SUPER_SPEED, 1)
        .descriptor
        .unwrap_or_else(|| vec![12, 0x2a, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0]);
    let mut variants = Vec::new();
    for (at, value) in [
        (0, 11),
        (1, 0x29),
        (2, 0),
        (2, 16),
        (3, 3),
        (3, 0x18),
        (3, 0x20),
    ] {
        let mut bytes = valid.clone();
        bytes[at] = value;
        variants.push((Some(bytes), None));
    }
    let mut short = valid.clone();
    short.pop();
    variants.push((Some(short), None));
    for (at, value) in [
        (0, 4),
        (2, 24),
        (4, 2),
        (5, 19),
        (6, 15),
        (8, 1),
        (9, 2),
        (16, 0x41),
        (17, 0x71),
    ] {
        let mut bytes = bos();
        bytes[at] = value;
        variants.push((None, Some(bytes)));
    }
    for (descriptor, bos) in variants {
        let mut nodes = nodes();
        let root = nodes.get_mut(&0).unwrap();
        root.descriptor = descriptor;
        root.bos = bos;
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert_eq!(devices.len(), 1);
        assert_eq!(
            failures.len(),
            1,
            "wire case {:?}",
            model.borrow().nodes[&0].bos
        );
        assert!(
            !model
                .borrow()
                .requests
                .iter()
                .any(|(_, _, q)| q[1] == SET_CONFIGURATION)
        );
        assert!(controller.slots[1].as_ref().unwrap().endpoints[3].is_none());
    }
}

#[test]
fn failed_later_port_preserves_prior_child_dma_and_does_not_authorize_behind_hub_reset() {
    let mut nodes = nodes();
    nodes
        .get_mut(&0)
        .unwrap()
        .ports
        .as_mut()
        .unwrap()
        .push(PortModel::connected());
    nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[1].fault = Fault::Reset;
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert_eq!(devices.len(), 2);
    assert_eq!(failures.len(), 1);
    let child = devices
        .iter()
        .find(|d| d.descriptor[4] != HUB_CLASS)
        .unwrap();
    let endpoints = interfaces(&child.configuration)[0].endpoints.clone();
    controller.set_configuration(child.slot, 1).unwrap();
    controller
        .configure(child.slot, &endpoints, None, &mut dma)
        .unwrap();
    let payload = dma.allocate(64, 64).unwrap();
    assert_eq!(
        controller.transfer(child.slot, 3, payload.physical(), 64, 50_000_000),
        Ok(64)
    );
    let commands = model.borrow().commands.len();
    let failed_port_resets = model.borrow().nodes[&0].ports.as_ref().unwrap()[1].reset_count;
    // A failed sibling port does not authorize resetting this already active
    // child with only part of its endpoint inventory. Downstream leaf reset
    // itself is supported, but requires exclusive ownership of every DCI.
    // SAFETY: this test keeps the full class endpoint set and DMA.
    let error =
        unsafe { controller.reset_and_identify(child, &endpoints[..1], &mut dma) }.unwrap_err();
    assert!(matches!(
        error.1,
        BlockError::Unsupported("USB reset does not exclusively own every configured endpoint")
    ));
    assert_eq!(model.borrow().commands.len(), commands);
    assert!(controller.live_slot(child.slot).is_ok());
    assert_eq!(
        controller.transfer(child.slot, 3, payload.physical(), 64, 50_000_000),
        Ok(64)
    );
    let old_output = controller.slot(child.slot).unwrap()._output.physical();
    // The complete inventory permits resetting the exclusive healthy leaf;
    // the unrelated failed port neither grants nor prevents that permission.
    // SAFETY: all configured DCIs belong exclusively to this test.
    let fresh = unsafe { controller.reset_and_identify(child, &endpoints, &mut dma) }.unwrap();
    assert_ne!(fresh.id, child.id);
    assert!(controller.device(child.id).is_err());
    assert!(controller.device(fresh.id).is_ok());
    assert_eq!(
        controller.slot(child.slot).unwrap()._output.physical(),
        old_output
    );
    assert_eq!(
        model.borrow().nodes[&0].ports.as_ref().unwrap()[1].reset_count,
        failed_port_resets
    );
}

#[test]
fn invalid_route_and_retired_or_foreign_hub_owner_refuse_before_transport_effects() {
    use descriptor::{child, topology};
    let root = Attachment {
        root_port: 1,
        route: 0,
        tier: 0,
        speed: SUPER_SPEED,
        translator: None,
        parent: None,
    };
    assert!(topology(root).is_ok());
    for invalid in [
        Attachment { route: 1, ..root },
        Attachment { tier: 5, ..root },
        Attachment { tier: 1, ..root },
        Attachment {
            translator: Some((1, 1)),
            ..root
        },
    ] {
        assert!(topology(invalid).is_err());
    }
    assert!(child(1, root, 0, SUPER_SPEED).is_err());
    assert!(child(1, root, 16, SUPER_SPEED).is_err());
    assert!(child(1, root, 1, HIGH_SPEED).is_err());
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes());
    let mut device = controller
        .identify(root, "hub model port 1".into(), &mut dma)
        .unwrap();
    configuration::select_with_controller(&mut device, Some(&controller)).unwrap();
    let before = model.borrow().requests.len();
    let mut devices = Vec::new();
    let mut failures = Vec::new();
    assert!(
        controller
            .hub(
                &device,
                Attachment { route: 1, ..root },
                &mut dma,
                &mut devices,
                &mut failures
            )
            .is_err()
    );
    controller.quarantine_slot(device.slot);
    assert!(
        controller
            .hub(&device, root, &mut dma, &mut devices, &mut failures)
            .is_err()
    );
    assert_eq!(model.borrow().requests.len(), before);
}

#[test]
#[ignore = "oracle: STELA_XHCI_HUB_ORACLE"]
fn unchanged_linux_hub_requests_power_warm_reset_and_sublink_rates() {
    use descriptor::{Descriptor, Ssp};
    use sha2::{Digest, Sha256};
    use superspeed::{Port, depth_request, status_request};
    let raw =
        std::fs::read(std::env::var("STELA_XHCI_HUB_ORACLE").expect("set STELA_XHCI_HUB_ORACLE"))
            .unwrap();
    assert_eq!(raw.len(), 5400);
    assert_eq!(
        format!("{:x}", Sha256::digest(&raw)),
        "6c311188681264c2f7decb6072e1802fc18635f23bdd29afc7f970b28d4281dd"
    );
    let node = Node::hub(SUPER_SPEED, 1);
    let config = node.configuration();
    let endpoint = interfaces(&config)[0].endpoints[0];
    let mut counts = [0usize; 6];
    for line in core::str::from_utf8(&raw).unwrap().lines() {
        let mut fields = line.split_whitespace();
        let tag = fields.next().unwrap();
        let n: Vec<u32> = fields.map(|f| f.parse().unwrap()).collect();
        match tag {
            "D" => {
                let desc = [12, 0x2a, 1, 1, 0, n[0] as u8, 0, 0, 0, 0, 0, 0];
                assert_eq!(
                    Descriptor::new(&desc, &endpoint).unwrap().power_good_us,
                    u64::from(n[1]) * 1000
                );
                counts[0] += 1;
            }
            "W" => {
                let status = 0x203u16 | (n[0] as u16) << 5;
                let bytes = [status as u8, (status >> 8) as u8, 0, 0];
                assert_eq!(Port::decode(&bytes, false).unwrap().warm(), n[1] != 0);
                counts[1] += 1;
            }
            "G" | "P" | "H" => {
                let expected = setup(
                    n[0] as u8,
                    n[1] as u8,
                    n[2] as u16,
                    n[3] as u16,
                    n[4] as u16,
                );
                match tag {
                    "G" => {
                        assert_eq!(expected, setup(0xa0, GET_DESCRIPTOR, 0x2a00, 0, 12));
                        counts[2] += 1;
                    }
                    "P" => {
                        assert_eq!(expected, status_request(n[3] as u8, n[2] == 2));
                        counts[3] += 1;
                    }
                    _ => {
                        assert_eq!(expected, depth_request(n[2] as u8));
                        counts[4] += 1;
                    }
                }
            }
            "S" => {
                let mut capability = bos();
                for (at, ty) in [(17usize, n[3]), (21, n[3] + 2)] {
                    let attr = 1 | n[1] << 4 | ty << 6 | 1 << 14 | n[0] << 16;
                    capability[at..at + 4].copy_from_slice(&attr.to_le_bytes());
                }
                let status = 0x11 | (n[2] - 1) << 8 | (n[2] - 1) << 12;
                let qualified = Ssp::bos(&capability)
                    .ok()
                    .flatten()
                    .and_then(|s| s.speed(status).ok());
                assert_eq!(qualified == Some(SUPER_SPEED_PLUS), n[4] != 0, "{line}");
                counts[5] += 1;
            }
            _ => panic!("unexpected donor row {tag}"),
        }
    }
    assert_eq!(counts, [256, 11, 1, 4, 5, 216]);
}

#[test]
fn superspeed_port_connection_is_debounced_before_any_reset_or_child_publication() {
    let mut nodes = nodes();
    let port = &mut nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
    port.connection_samples = [
        (true, true),
        (false, true),
        (true, true),
        (true, false),
        (true, false),
        (true, false),
        (true, false),
    ]
    .into();
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 64, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 2);
    let state = model.borrow();
    let reset = state
        .requests
        .iter()
        .position(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == PORT_RESET as u8)
        .unwrap();
    assert!(
        state.requests[..reset]
            .iter()
            .filter(|(_, _, q)| q[0] == 0xa3 && q[1] == GET_STATUS)
            .count()
            >= 7
    );
}

#[test]
fn superspeed_unsettled_connection_or_repeated_csc_never_resets_or_addresses_child() {
    for toggles in [false, true] {
        let mut nodes = nodes();
        let port = &mut nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
        port.connection_samples = (0..200).map(|i| (!toggles || i % 2 == 0, true)).collect();
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert_eq!(devices.len(), 1);
        assert_eq!(failures.len(), 1);
        let state = model.borrow();
        assert_eq!(
            state
                .commands
                .iter()
                .filter(|(kind, _)| *kind == ADDRESS_DEVICE)
                .count(),
            1
        );
        assert!(
            !state
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && [4, 28].contains(&q[2]))
        );
    }
}

#[test]
fn superspeed_reset_waits_for_reconnection_and_ignores_reset_connection_change() {
    for fault in [
        Fault::ResetConnectionBounce,
        Fault::ConnectionChange,
        Fault::NoResetChange,
    ] {
        let mut nodes = nodes();
        nodes.insert(0, fault_node(fault));
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 64, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{fault:?}: {failures:?}");
        assert_eq!(devices.len(), 2, "{fault:?}");
        let state = model.borrow();
        let resets: Vec<_> = state
            .requests
            .iter()
            .filter(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == PORT_RESET as u8)
            .collect();
        assert_eq!(resets.len(), 1);
        assert_eq!(state.nodes[&0].ports.as_ref().unwrap()[0].change, 0);
    }
}

#[test]
fn observed_hub_power_loss_quarantines_all_previous_children_without_releasing_dma() {
    let nodes = BTreeMap::from([
        (0, Node::hub(SUPER_SPEED, 2)),
        (1, Node::leaf(SUPER_SPEED)),
        (2, Node::leaf(SUPER_SPEED)),
    ]);
    let mut backing = vec![0u64; 256 * 1024];
    let start = backing.as_mut_ptr() as u64;
    // SAFETY: all fixture DMA owners remain held until the controller drops.
    let mut dma = unsafe { RegionAllocator::new(start..start + backing.len() as u64 * 8) };
    let (mut controller, model) = controller(&mut dma, 64, nodes);
    model.borrow_mut().loss_after_child_route = Some(1);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert_eq!(devices.len(), 2);
    assert_eq!(failures.len(), 1);
    assert!(controller.slots[1].as_ref().unwrap().retiring);
    assert!(controller.slots[2].as_ref().unwrap().retiring);
    let pointers: Vec<_> = controller
        .slots
        .iter()
        .flatten()
        .map(|state| (state.input.physical(), state._output.physical()))
        .collect();
    let before = (
        model.borrow().requests.len(),
        model.borrow().commands.len(),
        model.borrow().mmio_reads,
        model.borrow().mmio_writes,
    );
    let memory = backing.clone();
    // Restoring observed power cannot revive either kept old handle.
    model.borrow_mut().nodes.get_mut(&0).unwrap().hub_status = 0;
    for device in &devices {
        assert!(controller.endpoint_ready(device.slot, 3).is_err());
        assert!(controller.submit(device.slot, 3, 0, 1).is_err());
        assert!(
            controller
                .control(device.slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                .is_err()
        );
    }
    assert_eq!(
        (
            model.borrow().requests.len(),
            model.borrow().commands.len(),
            model.borrow().mmio_reads,
            model.borrow().mmio_writes
        ),
        before
    );
    assert_eq!(backing, memory);
    assert_eq!(
        controller
            .slots
            .iter()
            .flatten()
            .map(|state| (state.input.physical(), state._output.physical()))
            .collect::<Vec<_>>(),
        pointers
    );
}

#[test]
fn superspeed_retries_ordinary_disabled_or_not_ready_hot_resets() {
    for fault in [Fault::FirstResetDisabled, Fault::FirstResetNotU0] {
        let mut nodes = nodes();
        nodes.insert(0, fault_node(fault));
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{fault:?}: {failures:?}");
        assert_eq!(devices.len(), 2);
        let state = model.borrow();
        assert_eq!(state.nodes[&0].ports.as_ref().unwrap()[0].reset_count, 2);
        assert_eq!(
            state
                .requests
                .iter()
                .filter(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == PORT_RESET as u8)
                .count(),
            2
        );
        assert!(
            !state
                .requests
                .iter()
                .any(|(_, _, q)| q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == 28)
        );
    }
}

fn bus_powered_hub(ports: usize) -> Node {
    let mut node = Node::hub(SUPER_SPEED, ports);
    node.self_powered = false;
    node.max_power = 112; // 896mA descriptor demand, inside the standard900mA contract.
    node.controller_current = 20;
    node
}

#[test]
fn bus_powered_hub_charges_each_budget_once() {
    let mut nodes = BTreeMap::from([(0, bus_powered_hub(4))]);
    for port in 1..=4 {
        let mut leaf = Node::leaf(SUPER_SPEED);
        leaf.max_power = 12;
        nodes.insert(port, leaf);
    }
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 64, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 5);
    let child = devices.iter().find(|d| d.location.ends_with(".1")).unwrap();
    assert_eq!(controller.power.snapshot(child.slot), (150, 150, 150));
    controller
        .control(child.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
        .unwrap();
    assert_eq!(controller.power.snapshot(child.slot), (150, 96, 96));
    assert_eq!(controller.power.snapshot(1), (900, 896, 896));
    let requests = model.borrow().requests.len();
    controller.set_configuration(child.slot, 1).unwrap();
    assert_eq!(model.borrow().requests.len(), requests);
    assert_eq!(controller.power.snapshot(1), (900, 896, 896));
    controller
        .control(child.slot, setup(0, SET_CONFIGURATION, 0, 0, 0), &[])
        .unwrap();
    assert_eq!(controller.power.snapshot(child.slot), (150, 150, 150));
    let before = model.borrow().requests.len();
    assert!(
        controller
            .control(child.slot, setup(0, SET_CONFIGURATION, 2, 0, 0), &[])
            .is_err()
    );
    assert_eq!(model.borrow().requests.len(), before);
}

#[test]
fn bus_powered_hub_never_powers_ports_without_a_current_reservation() {
    let mut nodes = BTreeMap::from([(0, bus_powered_hub(7))]);
    for port in 1..=7 {
        nodes.insert(port, Node::leaf(SUPER_SPEED));
    }
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert_eq!(devices.len(), 6);
    assert_eq!(failures.len(), 2);
    let state = model.borrow();
    let powered: Vec<_> = state
        .requests
        .iter()
        .filter(|(_, route, q)| {
            *route == 0 && q[0] == 0x23 && q[1] == SET_FEATURE && q[2] == PORT_POWER as u8
        })
        .map(|(_, _, q)| q[4])
        .collect();
    assert_eq!(powered, [1, 2, 3, 4, 5]);
    assert!(
        state.nodes[&0].ports.as_ref().unwrap()[5..]
            .iter()
            .all(|p| p.status & (1 << 9) == 0)
    );
    assert_eq!(controller.power.snapshot(1), (900, 896, 896));
    drop(state);
    let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
    let requests = model.borrow().requests.len();
    let control = controller.slots[1]
        .as_ref()
        .unwrap()
        .control
        .bytes()
        .to_vec();
    let input = controller.slots[1].as_ref().unwrap().input.bytes().to_vec();
    for port in [6, 7] {
        assert!(
            controller
                .control(1, setup(0x23, SET_FEATURE, PORT_POWER, port, 0), &[])
                .is_err()
        );
        assert_eq!(controller.power.snapshot(1), (900, 896, 896));
    }
    assert!(
        controller
            .control(1, setup(0x23, CLEAR_FEATURE, PORT_POWER, 1, 0), &[])
            .is_err()
    );
    assert_eq!(
        (model.borrow().mmio_reads, model.borrow().mmio_writes),
        before
    );
    assert_eq!(model.borrow().requests.len(), requests);
    assert_eq!(
        controller.slots[1].as_ref().unwrap().control.bytes(),
        control
    );
    assert_eq!(controller.slots[1].as_ref().unwrap().input.bytes(), input);
    assert!(
        model.borrow().nodes[&0].ports.as_ref().unwrap()[5..]
            .iter()
            .all(|p| p.status & (1 << 9) == 0)
    );
}

#[test]
fn bus_powered_child_configuration_cannot_bypass_the_one_unit_limit() {
    let mut leaf = Node::leaf(SUPER_SPEED);
    leaf.max_power = 20; //160mA >150mA parent contract.
    let nodes = BTreeMap::from([(0, bus_powered_hub(1)), (1, leaf)]);
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 64, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty());
    let child = devices.iter().find(|d| d.location.ends_with(".1")).unwrap();
    let input = controller.slots[usize::from(child.slot)]
        .as_ref()
        .unwrap()
        .input
        .bytes()
        .to_vec();
    assert!(
        controller
            .control(child.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
            .is_err()
    );
    assert!(
        !model
            .borrow()
            .requests
            .iter()
            .any(|(slot, _, q)| *slot == child.slot && q[0] == 0 && q[1] == SET_CONFIGURATION)
    );
    assert_eq!(controller.current_configuration(child.slot), Ok(Some(0)));
    assert_eq!(controller.power.snapshot(child.slot), (150, 150, 150));
    assert_eq!(
        controller.slots[usize::from(child.slot)]
            .as_ref()
            .unwrap()
            .input
            .bytes(),
        input
    );
}

#[test]
fn consecutive_bus_hubs_refuse_but_self_powered_downstream_hubs_keep_local_capacity() {
    for self_powered in [false, true] {
        let mut middle = Node::hub(SUPER_SPEED, 1);
        middle.self_powered = self_powered;
        middle.max_power = 12;
        let mut leaf = Node::leaf(SUPER_SPEED);
        leaf.max_power = 100;
        let nodes = BTreeMap::from([(0, bus_powered_hub(1)), (1, middle), (0x11, leaf)]);
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 64, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        if self_powered {
            assert!(failures.is_empty(), "{failures:?}");
            assert_eq!(devices.len(), 3);
            let child = devices
                .iter()
                .find(|d| d.location.ends_with(".1.1"))
                .unwrap();
            controller
                .control(child.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
                .unwrap();
            assert_eq!(controller.power.snapshot(child.slot), (900, 800, 800));
            assert_eq!(controller.power.snapshot(1), (900, 896, 896));
        } else {
            assert_eq!(devices.len(), 2);
            assert_eq!(failures.len(), 1);
            assert!(
                !model
                    .borrow()
                    .requests
                    .iter()
                    .any(|(_, route, q)| *route == 1
                        && q[0] == 0x23
                        && q[1] == SET_FEATURE
                        && q[2] == PORT_POWER as u8)
            );
        }
    }
}

#[test]
fn failed_or_uncertain_raw_configuration_keeps_maximum_draw_until_ack() {
    for silent in [false, true] {
        let mut leaf = Node::leaf(SUPER_SPEED);
        leaf.max_power = 100;
        let nodes = BTreeMap::from([(0, leaf)]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty());
        let slot = devices[0].slot;
        if silent {
            model.borrow_mut().silent_configuration = Some(slot);
        } else {
            model.borrow_mut().failed_configuration = Some(slot);
        }
        assert!(
            controller
                .control(slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
                .is_err()
        );
        assert_eq!(controller.current_configuration(slot), Ok(None));
        assert_eq!(controller.power.snapshot(slot), (900, 800, 800));
        model.borrow_mut().silent_configuration = None;
        model.borrow_mut().failed_configuration = None;
        controller
            .control(slot, setup(0, SET_CONFIGURATION, 0, 0, 0), &[])
            .unwrap();
        assert_eq!(controller.power.snapshot(slot), (900, 150, 150));
    }
}

#[test]
fn usb2_bus_hub_uses_two_ma_configuration_units_and_hundred_ma_child_limits() {
    let mut hub = Node::hub(HIGH_SPEED, 2);
    for port in hub.ports.as_mut().unwrap() {
        port.child_speed = FULL_SPEED;
    }
    hub.self_powered = false;
    hub.max_power = 250;
    hub.controller_current = 20;
    let mut first = Node::leaf(FULL_SPEED);
    first.max_power = 50;
    let mut second = Node::leaf(FULL_SPEED);
    second.max_power = 51;
    let nodes = BTreeMap::from([(0, hub), (1, first), (2, second)]);
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    assert_eq!(devices.len(), 3);
    let first = devices.iter().find(|d| d.location.ends_with(".1")).unwrap();
    let second = devices.iter().find(|d| d.location.ends_with(".2")).unwrap();
    controller
        .control(first.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
        .unwrap();
    assert_eq!(controller.power.snapshot(first.slot), (100, 100, 100));
    assert!(
        controller
            .control(second.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
            .is_err()
    );
    assert!(
        !model
            .borrow()
            .requests
            .iter()
            .any(|(slot, _, q)| *slot == second.slot && q[0] == 0 && q[1] == SET_CONFIGURATION)
    );
    assert_eq!(controller.power.snapshot(1), (500, 500, 500));
}

#[test]
fn root_power_loss_quarantines_old_handles_and_unknown_power_never_grants_current() {
    for override_status in [
        PP | CCS | PED | u32::from(SUPER_SPEED) << 10 | (1 << 3),
        CCS | PED | u32::from(SUPER_SPEED) << 10,
        PP,                        // Physical disconnect also removes the speed observation.
        PP | CCS | PED,            // Unknown protocol state cannot preserve permission.
        PP | CCS | PED | (1 << 3), // Over-current with unknown wire speed.
    ] {
        let nodes = BTreeMap::from([(0, Node::leaf(SUPER_SPEED))]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty());
        let slot = devices[0].slot;
        model.borrow_mut().root_status = Some(override_status);
        let requests = model.borrow().requests.len();
        assert!(
            controller
                .control(slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
                .is_err()
        );
        assert_eq!(model.borrow().requests.len(), requests);
        assert!(
            controller.slots[usize::from(slot)]
                .as_ref()
                .unwrap()
                .retiring
        );
        model.borrow_mut().root_status = None;
        let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
        assert!(
            controller
                .control(slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
                .is_err()
        );
        assert_eq!(
            (model.borrow().mmio_reads, model.borrow().mmio_writes),
            before
        );
    }
    let nodes = BTreeMap::from([(0, Node::leaf(SUPER_SPEED))]);
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let (mut controller, model) = controller(&mut dma, 32, nodes);
    controller.port_major[0] = 0;
    let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
    assert!(
        controller
            .reserve_current(
                1,
                identity::next_generation().unwrap(),
                Attachment {
                    root_port: 1,
                    route: 0,
                    tier: 0,
                    speed: SUPER_SPEED,
                    translator: None,
                    parent: None,
                },
                None,
            )
            .is_err()
    );
    assert_eq!(
        (model.borrow().mmio_reads, model.borrow().mmio_writes),
        before
    );
    assert!(controller.slots[1].is_none());
}

#[test]
fn hub_power_contract_uses_status_after_configuration_ack() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for self_powered in [false, true] {
            let mut hub = Node::hub(speed, 1);
            hub.self_powered = self_powered;
            hub.unconfigured_self_powered = Some(!self_powered);
            hub.max_power = if speed == HIGH_SPEED { 250 } else { 112 };
            hub.controller_current = 20;
            let mut leaf = Node::leaf(speed);
            leaf.max_power = 100; // 200/800mA needs a self-powered parent.
            let nodes = BTreeMap::from([(0, hub), (1, leaf)]);
            let mut dma = TestRegion::new(256 * 1024 * 8);
            let (mut controller, model) = controller(&mut dma, 64, nodes);
            let mut failures = Vec::new();
            let devices = controller.enumerate(&mut dma, &mut failures);
            assert!(failures.is_empty(), "{speed}/{self_powered}: {failures:?}");
            assert_eq!(devices.len(), 2);
            let state = model.borrow();
            let configured = state
                .requests
                .iter()
                .position(|(_, route, q)| *route == 0 && q[0] == 0 && q[1] == SET_CONFIGURATION)
                .unwrap();
            let observed = state
                .requests
                .iter()
                .position(|(_, route, q)| *route == 0 && q[0] == 0x80 && q[1] == GET_STATUS)
                .unwrap();
            assert!(configured < observed);
            drop(state);
            let child = devices.iter().find(|d| d.location.ends_with(".1")).unwrap();
            assert_eq!(
                controller
                    .control(child.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
                    .is_ok(),
                self_powered
            );
        }
    }
}

#[test]
fn hub_endpoint_teardown_does_not_authorize_reconfiguration_of_live_children() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let mut hub = Node::hub(speed, 1);
        hub.max_power = 10;
        let nodes = BTreeMap::from([(0, hub), (1, Node::leaf(speed))]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty());
        let hub = devices
            .iter()
            .find(|d| d.location.ends_with("port 1"))
            .unwrap();
        let child = devices.iter().find(|d| d.location.ends_with(".1")).unwrap();
        let endpoint = interfaces(&hub.configuration)[0].endpoints[0];
        controller.deconfigure(hub.slot, &[endpoint]).unwrap();
        assert!(
            controller.slots[usize::from(hub.slot)]
                .as_ref()
                .unwrap()
                .endpoints[usize::from(endpoint.dci())]
            .is_none()
        );
        let grant = controller.power.snapshot(hub.slot);
        for value in [0, 1, 2] {
            let requests = model.borrow().requests.len();
            assert_eq!(
                controller.control(hub.slot, setup(0, SET_CONFIGURATION, value, 0, 0), &[]),
                Err(BlockError::Unsupported(
                    "USB hub configuration requires complete subtree retirement"
                ))
            );
            assert_eq!(model.borrow().requests.len(), requests);
            assert_eq!(controller.power.snapshot(hub.slot), grant);
            assert_eq!(controller.current_configuration(hub.slot), Ok(Some(1)));
        }
        let requests = model.borrow().requests.len();
        controller.set_configuration(hub.slot, 1).unwrap();
        assert_eq!(model.borrow().requests.len(), requests);
        controller
            .control(child.slot, setup(0, SET_CONFIGURATION, 1, 0, 0), &[])
            .unwrap();
        assert!(
            !controller.slots[usize::from(child.slot)]
                .as_ref()
                .unwrap()
                .retiring
        );
    }
}

#[test]
fn raw_port_power_revalidates_root_and_hub_health_before_authorizing_current() {
    for root_lost in [false, true] {
        let nodes = BTreeMap::from([(0, Node::hub(SUPER_SPEED, 1)), (1, Node::leaf(SUPER_SPEED))]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut controller, model) = controller(&mut dma, 64, nodes);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty());
        if root_lost {
            model.borrow_mut().root_status = Some(PP | CCS | PED | (1 << 3));
        } else {
            model.borrow_mut().nodes.get_mut(&0).unwrap().self_powered = false;
        }
        let powers = model
            .borrow()
            .requests
            .iter()
            .filter(|(_, _, q)| *q == setup(0x23, SET_FEATURE, PORT_POWER, 1, 0))
            .count();
        assert!(
            controller
                .control(1, setup(0x23, SET_FEATURE, PORT_POWER, 1, 0), &[])
                .is_err()
        );
        assert_eq!(
            model
                .borrow()
                .requests
                .iter()
                .filter(|(_, _, q)| *q == setup(0x23, SET_FEATURE, PORT_POWER, 1, 0))
                .count(),
            powers
        );
        model.borrow_mut().root_status = None;
        model.borrow_mut().nodes.get_mut(&0).unwrap().self_powered = true;
        let before = (model.borrow().mmio_reads, model.borrow().mmio_writes);
        for device in &devices {
            assert!(
                controller
                    .control(device.slot, setup(0x80, GET_STATUS, 0, 0, 2), &[])
                    .is_err()
            );
            assert!(
                controller.slots[usize::from(device.slot)]
                    .as_ref()
                    .unwrap()
                    .retiring
            );
            assert!(controller.power.retained(device.slot));
        }
        assert_eq!(
            (model.borrow().mmio_reads, model.borrow().mmio_writes),
            before
        );
    }
}

#[test]
fn usb2_hub_descriptors_need_only_device_removable_like_get_hub_descriptor() {
    for (ports, qemu) in [1usize, 7, 8, 15, 31]
        .into_iter()
        .flat_map(|ports| [(ports, false), (ports, true)])
    {
        let mut hub = Node::hub(HIGH_SPEED, ports);
        hub.multi_tt = true;
        if qemu {
            // QEMU's hub sizes PortPwrCtrlMask without bit zero, one byte
            // shorter than DeviceRemovable at eight ports.
            let removable = (ports + 1).div_ceil(8);
            let mut bytes = vec![0, 0x29, ports as u8, 0x0a, 0, 1, 0];
            bytes.resize(7 + removable, 0);
            bytes.resize(7 + removable + ports.div_ceil(8), 0xff);
            bytes[0] = bytes.len() as u8;
            hub.descriptor = Some(bytes);
        }
        // Empty ports exercise descriptor/current/status ownership without a
        // separate fake leaf at each route.
        for port in hub.ports.as_mut().unwrap() {
            port.status = 0;
            port.change = 0;
        }
        let nodes = BTreeMap::from([(0, hub)]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        assert_eq!(c.enumerate(&mut dma, &mut failures).len(), 1);
        assert!(failures.is_empty(), "{ports} {qemu}: {failures:?}");
        // sizeof(struct usb_hub_descriptor), whatever the hub reports.
        assert!(model.borrow().requests.iter().any(|(_, _, q)| q[0] == 0xa0
            && q[1] == GET_DESCRIPTOR
            && q[3] == HUB_DESCRIPTOR
            && u16::from_le_bytes([q[6], q[7]]) == 15));
    }
    for bytes in [
        vec![9, 0x29, 0, 1, 0, 1, 0, 0, 255],
        // Sixteen ports need DeviceRemovable's third byte.
        vec![9, 0x29, 16, 1, 0, 1, 0, 0, 255],
        vec![8, 0x29, 1, 1, 0, 1, 0, 0],
        vec![9, 0x29, 1, 3, 0, 1, 0, 0, 255],
        vec![9, 0x29, 1, 1, 1, 1, 0, 0, 255],
    ] {
        let mut hub = Node::hub(HIGH_SPEED, 1);
        hub.descriptor = Some(bytes);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 64, BTreeMap::from([(0, hub)]));
        let mut failures = Vec::new();
        assert_eq!(c.enumerate(&mut dma, &mut failures).len(), 1);
        assert_eq!(failures.len(), 1);
        assert!(!model.borrow().requests.iter().any(|(_, _, q)| q[0] == 0x23));
    }
}

#[test]
fn kept_hub_status_uses_same_reserved_field_contract_as_cold_enumeration() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let nodes = BTreeMap::from([(0, Node::hub(speed, 1)), (1, Node::leaf(speed))]);
        let mut dma = TestRegion::new(256 * 1024 * 8);
        let (mut c, model) = controller(&mut dma, 32, nodes);
        let mut failures = Vec::new();
        c.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty());
        let original = model.borrow().nodes[&0].ports.as_ref().unwrap()[0].status;
        for (extra_status, change) in if speed == SUPER_SPEED {
            [(1 << 2, 0), (1 << 10, 0), (11 << 5, 0), (0, 1 << 1)]
        } else {
            [(1 << 6, 0), (1 << 7, 0), (3 << 9, 0), (0, 1 << 6)]
        } {
            {
                let mut hardware = model.borrow_mut();
                let port = &mut hardware.nodes.get_mut(&0).unwrap().ports.as_mut().unwrap()[0];
                port.status = original | extra_status;
                port.change = change;
            }
            assert!(
                c.hub_port_status(1, 1).is_err(),
                "speed={speed}, status={extra_status}, change={change}"
            );
        }
    }
}

#[test]
fn usb2_multi_tt_validates_the_acknowledged_original_alternate_before_powering_children() {
    let setting = |endpoints: &[[u8; 7]]| {
        let mut bytes = vec![9, 4, 0, 1, endpoints.len() as u8, HUB_CLASS, 0, 2, 0];
        for endpoint in endpoints {
            bytes.extend_from_slice(endpoint);
        }
        bytes
    };
    let valid = [7, 5, 0x82, 3, 2, 0, 8];
    let mut duplicate_interface = setting(&[valid]);
    duplicate_interface.extend_from_slice(&setting(&[valid]));
    let mut wrong_count = setting(&[valid]);
    wrong_count[4] = 2;
    let variants = [
        ("missing", Vec::new(), false),
        ("empty", setting(&[]), false),
        ("wrong declared count", wrong_count, false),
        ("undersized", setting(&[[7, 5, 0x82, 3, 1, 0, 8]]), false),
        ("bulk", setting(&[[7, 5, 0x82, 2, 0, 2, 0]]), false),
        ("OUT", setting(&[[7, 5, 2, 3, 2, 0, 8]]), false),
        ("endpoint zero", setting(&[[7, 5, 0x80, 3, 2, 0, 8]]), false),
        (
            "invalid interval",
            setting(&[[7, 5, 0x82, 3, 2, 0, 0]]),
            false,
        ),
        ("duplicate endpoint", setting(&[valid, valid]), false),
        ("duplicate interface", duplicate_interface, false),
        ("valid distinct endpoint", setting(&[valid]), true),
    ];
    for context in [32, 64] {
        for (case, alternate, usable) in &variants {
            // Alt0 is valid in every case. The model acknowledges interface-zero
            // alt1 even when the device's own original descriptors are bad.
            let mut configuration = vec![
                9, 2, 0, 0, 1, 1, 0, 0xc0, 0, 9, 4, 0, 0, 1, HUB_CLASS, 0, 1, 0, 7, 5, 0x81, 3, 2,
                0, 8,
            ];
            configuration.extend_from_slice(alternate);
            let length = configuration.len() as u16;
            configuration[2..4].copy_from_slice(&length.to_le_bytes());
            let mut hub = Node::hub(HIGH_SPEED, 8);
            hub.multi_tt = true;
            hub.configuration_override = Some(configuration);
            for port in &mut hub.ports.as_mut().unwrap()[1..] {
                port.status = 0;
                port.change = 0;
            }
            let nodes = BTreeMap::from([(0, hub), (1, Node::leaf(HIGH_SPEED))]);
            let mut dma = TestRegion::new(256 * 1024 * 8);
            let (mut c, model) = controller(&mut dma, context, nodes);
            let mut failures = Vec::new();
            let devices = c.enumerate(&mut dma, &mut failures);
            let hardware = model.borrow();
            let selected = setup(RECIPIENT_INTERFACE, SET_INTERFACE, 1, 0, 0);
            assert!(
                hardware
                    .requests
                    .iter()
                    .any(|(_, _, request)| *request == selected),
                "{case}: selection was not reached"
            );
            if *usable {
                assert!(failures.is_empty(), "{case}: {failures:?}");
                assert_eq!(devices.len(), 2);
                let state = c.slots[1].as_ref().unwrap();
                assert!(
                    state.endpoints[5]
                        .as_ref()
                        .is_some_and(|endpoint| endpoint.enabled)
                );
                assert!(
                    state.endpoints[3].is_none(),
                    "alt0 endpoint must stay inactive"
                );
                assert_ne!(state.context[0] & (1 << 25), 0);
            } else {
                assert_eq!(failures.len(), 1, "{case}: {failures:?}");
                assert_eq!(devices.len(), 1, "{case}: a child was published");
                assert!(
                    !hardware
                        .requests
                        .iter()
                        .any(|(_, _, request)| request[0] == 0x23
                            && request[1] == SET_FEATURE
                            && request[2] == PORT_POWER as u8),
                    "{case}: downstream power published"
                );
                assert!(
                    !hardware
                        .commands
                        .iter()
                        .any(|(kind, _)| *kind == CONFIGURE_ENDPOINT),
                    "{case}: malformed setting configured"
                );
                assert_eq!(
                    c.slots[1].as_ref().unwrap().context[0] & (1 << 25),
                    0,
                    "unconfigured MTT must not be committed"
                );
            }
        }
    }
}

mod after_boot;
mod runtime;
