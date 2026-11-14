// SPDX-License-Identifier: GPL-2.0-only
//! Real command/context/port model checks; source-only cut, UNRUN.
use super::super::*;
use crate::bus::dma::TestRegion;
use alloc::collections::BTreeMap;
use hub::model::Node;

#[test]
fn exclusive_downstream_leaf_reset_preserves_its_parent_and_sibling() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        for context in [32, 64] {
            let mut dma = TestRegion::new(512 * 1024 * 8);
            let nodes = BTreeMap::from([
                (0, Node::hub(speed, 2)),
                (1, Node::leaf(speed)),
                (2, Node::leaf(speed)),
            ]);
            let (mut controller, hardware) = hub::tests::controller(&mut dma, context, nodes);
            let mut failures = Vec::new();
            let devices = controller.enumerate(&mut dma, &mut failures);
            assert!(failures.is_empty(), "{failures:?}");
            let old = devices.iter().find(|d| d.location.ends_with(".1")).unwrap();
            let sibling = devices.iter().find(|d| d.location.ends_with(".2")).unwrap();
            let parent = devices
                .iter()
                .find(|d| d.descriptor[4] == HUB_CLASS)
                .unwrap();
            let endpoints = interfaces(&old.configuration)[0].endpoints.clone();
            controller
                .set_configuration(old.slot, old.configuration_value())
                .unwrap();
            controller
                .configure(old.slot, &endpoints, None, &mut dma)
                .unwrap();
            let sibling_output = controller.slot(sibling.slot).unwrap()._output.physical();
            let before = hardware.borrow().nodes[&0].ports.as_ref().unwrap()[1].reset_count;
            let requests = hardware.borrow().requests.len();
            // SAFETY: no class owner was bound; endpoints are the full
            // configured DCI inventory of this one downstream leaf.
            let fresh =
                unsafe { controller.reset_and_identify(old, &endpoints, &mut dma) }.unwrap();
            assert_ne!(fresh.id, old.id);
            assert_eq!(fresh.location, old.location);
            assert_eq!(fresh.speed, old.speed);
            let health: Vec<_> = hardware.borrow().requests[requests..]
                .iter()
                .filter(|(_, _, request)| request[1] == GET_STATUS)
                .map(|(slot, _, request)| (*slot, request[0]))
                .collect();
            assert!(health.contains(&(parent.slot, 0x80)));
            assert!(health.contains(&(parent.slot, 0xa0)));
            assert!(health.iter().all(|(slot, _)| *slot != old.slot));
            assert!(controller.device(parent.id).is_ok());
            assert!(controller.device(sibling.id).is_ok());
            assert_eq!(
                controller.slot(sibling.slot).unwrap()._output.physical(),
                sibling_output
            );
            assert_eq!(
                hardware.borrow().nodes[&0].ports.as_ref().unwrap()[1].reset_count,
                before
            );
            let before = (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes);
            assert!(controller.device(old.id).is_err());
            assert_eq!(
                (hardware.borrow().mmio_reads, hardware.borrow().mmio_writes),
                before
            );
        }
    }
}

#[test]
fn downstream_reset_failure_never_addresses_a_replacement() {
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut controller, hardware) = hub::tests::controller(
            &mut dma,
            64,
            BTreeMap::from([(0, Node::hub(speed, 1)), (1, Node::leaf(speed))]),
        );
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        let old = devices
            .iter()
            .find(|d| d.descriptor[4] != HUB_CLASS)
            .unwrap();
        let endpoints = interfaces(&old.configuration)[0].endpoints.clone();
        controller
            .set_configuration(old.slot, old.configuration_value())
            .unwrap();
        controller
            .configure(old.slot, &endpoints, None, &mut dma)
            .unwrap();
        hardware.borrow_mut().fail = Some((STOP_ENDPOINT, old.slot));
        let before = hardware.borrow().commands.len();
        // SAFETY: no class owner exists and the complete endpoint set is held.
        let error =
            unsafe { controller.reset_and_identify(old, &endpoints, &mut dma) }.unwrap_err();
        assert_eq!(error.0, "retire every endpoint before Disable Slot");
        assert!(controller.device(old.id).is_err());
        assert!(
            hardware.borrow().commands[before..]
                .iter()
                .all(|(kind, _)| !matches!(*kind, ADDRESS_DEVICE | reset::DISABLE_SLOT))
        );
    }
}

#[test]
fn downstream_reset_reobserves_root_and_hub_power_before_endpoint_retirement() {
    for root_lost in [false, true] {
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut controller, hardware) = hub::tests::controller(
            &mut dma,
            64,
            BTreeMap::from([(0, Node::hub(HIGH_SPEED, 1)), (1, Node::leaf(HIGH_SPEED))]),
        );
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        let old = devices
            .iter()
            .find(|d| d.descriptor[4] != HUB_CLASS)
            .unwrap();
        let endpoints = interfaces(&old.configuration)[0].endpoints.clone();
        controller
            .set_configuration(old.slot, old.configuration_value())
            .unwrap();
        controller
            .configure(old.slot, &endpoints, None, &mut dma)
            .unwrap();
        let output = controller.slot(old.slot).unwrap()._output.physical();
        if root_lost {
            hardware.borrow_mut().root_status = Some(CCS | PED | u32::from(HIGH_SPEED) << 10);
        } else {
            // USB hub class status bit0 reports loss of the local supply.
            hardware.borrow_mut().nodes.get_mut(&0).unwrap().hub_status = 1;
        }
        let commands = hardware.borrow().commands.len();
        // SAFETY: this is the exclusive complete leaf endpoint inventory.
        let error =
            unsafe { controller.reset_and_identify(old, &endpoints, &mut dma) }.unwrap_err();
        assert_eq!(error.0, "observe reset device physical ancestry");
        assert_eq!(hardware.borrow().commands.len(), commands);
        assert!(controller.device(old.id).is_err());
        assert_eq!(
            controller.slot(old.slot).unwrap()._output.physical(),
            output
        );
        assert_eq!(
            controller.slot_ownership[usize::from(old.slot)],
            reset::Ownership::Enabled
        );
        assert!(controller.power.retained(old.slot));
    }
}

/// A stick behind a hub that hangs after its configuration: its first
/// command never completes and its class reset stalls. The storage owner
/// must leave the port reset to the storage service, which has the
/// allocator a device behind a hub needs for its new rings; begun without
/// one, the reset disabled the slot and reset the port before it failed,
/// and the service's own reset then found the device gone for good.
#[test]
fn a_hung_stick_behind_a_hub_is_reset_by_the_storage_service_and_found() {
    use crate::block::BlockDevice;
    let _clock = crate::test_clock::virtualize();
    for speed in [HIGH_SPEED, SUPER_SPEED] {
        let mut dma = TestRegion::new(1024 * 1024 * 8);
        let (mut controller, hardware) = hub::tests::controller(
            &mut dma,
            64,
            BTreeMap::from([(0, Node::hub(speed, 1)), (1, Node::bot_disk(speed))]),
        );
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        let stick = devices
            .iter()
            .find(|d| d.descriptor[4] != HUB_CLASS)
            .unwrap()
            .clone();
        let shared = Rc::new(RefCell::new(controller));
        hardware.borrow_mut().nodes.get_mut(&1).unwrap().wedge_bot();
        let resets = hardware.borrow().nodes[&0].ports.as_ref().unwrap()[0].reset_count;
        let mut found = Found::default();
        bind::bind(&shared, stick.clone(), &mut dma, &mut found);
        // The first INQUIRY timed out; the stick came back after one port
        // reset by the service and was found on the same port.
        assert!(!hardware.borrow().nodes[&1].bot_wedged());
        assert_eq!(
            hardware.borrow().nodes[&0].ports.as_ref().unwrap()[0].reset_count,
            resets + 1
        );
        assert!(
            found.failures.iter().all(|(_, error)| *error
                != BlockError::Unsupported("xHCI device generation is permanently retired")),
            "{:?}",
            found.failures
        );
        assert_eq!(found.disks.len(), 1, "{:?}", found.failures);
        assert_eq!(found.storage.len(), 1);
        assert!(shared.borrow_mut().device(stick.id).is_err());
        // The timed-out command's line names the hub's root port as the
        // transfer failed.
        let (timed_out, _) = found
            .failures
            .iter()
            .find(|(_, error)| *error == BlockError::Timeout)
            .unwrap();
        assert!(
            timed_out.contains("LUN 0 root port 1 PORTSC=") && timed_out.ends_with("route=0x1"),
            "{timed_out}"
        );
        let payload = [0x3c; 512];
        found.disks[0].write(4, &payload).unwrap();
        let mut read = [0; 512];
        found.disks[0].read(4, &mut read).unwrap();
        assert_eq!(read, payload);
    }
}

/// The same hung stick on a root port: its port reset reuses the old
/// binding's storage and runs at once from the storage owner. A device
/// whose firmware hung may drop its USB 3 link to SS.Inactive, showing a
/// connection or not, where a hot reset cannot retrain it: as Linux's
/// `hub_port_reset`, the reset is then a warm one, where the in-place reset
/// refused the port and the stick was lost for the boot.
#[test]
fn a_hung_stick_on_a_root_port_is_reset_in_place_and_found() {
    use crate::block::BlockDevice;
    let _clock = crate::test_clock::virtualize();
    for (speed, link) in [
        (HIGH_SPEED, None),
        (SUPER_SPEED, None),
        (SUPER_SPEED, Some(true)),
        (SUPER_SPEED, Some(false)),
    ] {
        let mut dma = TestRegion::new(1024 * 1024 * 8);
        let (mut controller, hardware) =
            hub::tests::controller(&mut dma, 64, BTreeMap::from([(0, Node::bot_disk(speed))]));
        hardware.borrow_mut().root_status =
            Some(PP | CCS | PED | u32::from(speed) << PORT_SPEED_SHIFT);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        let stick = devices[0].clone();
        let shared = Rc::new(RefCell::new(controller));
        hardware.borrow_mut().nodes.get_mut(&0).unwrap().wedge_bot();
        hardware.borrow_mut().wedge_drops_root_link = link;
        let mut found = Found::default();
        bind::bind(&shared, stick.clone(), &mut dma, &mut found);
        assert!(!hardware.borrow().nodes[&0].bot_wedged(), "{link:?}");
        assert_eq!(found.disks.len(), 1, "{link:?} {:?}", found.failures);
        assert_eq!(hardware.borrow().root_status.unwrap() & PLS, 0);
        // The timed-out command's line keeps the link state it failed in.
        let (timed_out, _) = found
            .failures
            .iter()
            .find(|(_, error)| *error == BlockError::Timeout)
            .unwrap();
        let link_state = if link.is_some() { "SS.Inactive" } else { "U0" };
        assert!(
            timed_out.ends_with(&format!("link={link_state}")),
            "{timed_out}"
        );
        let payload = [0x5d; 512];
        found.disks[0].write(4, &payload).unwrap();
        let mut read = [0; 512];
        found.disks[0].read(4, &mut read).unwrap();
        assert_eq!(read, payload);
    }
}

/// A port reset that cannot bring the hung stick back is reported with the
/// step that failed and the port's status after it, where its failure was
/// dropped and only the later refusal of the retired device was logged.
#[test]
fn a_failed_port_reset_of_a_hung_stick_is_reported_with_its_port_status() {
    let _clock = crate::test_clock::virtualize();
    let mut dma = TestRegion::new(1024 * 1024 * 8);
    let (mut controller, hardware) = hub::tests::controller(
        &mut dma,
        64,
        BTreeMap::from([(0, Node::bot_disk(SUPER_SPEED))]),
    );
    hardware.borrow_mut().root_status =
        Some(PP | CCS | PED | u32::from(SUPER_SPEED) << PORT_SPEED_SHIFT);
    let mut failures = Vec::new();
    let devices = controller.enumerate(&mut dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    let shared = Rc::new(RefCell::new(controller));
    hardware.borrow_mut().nodes.get_mut(&0).unwrap().wedge_bot();
    hardware.borrow_mut().wedge_drops_root_link = Some(true);
    hardware.borrow_mut().root_untrainable = true;
    let mut found = Found::default();
    bind::bind(&shared, devices[0].clone(), &mut dma, &mut found);
    assert!(found.disks.is_empty());
    let (reset, error) = found
        .failures
        .iter()
        .find(|(location, _)| location.contains("port reset after a failed command"))
        .unwrap_or_else(|| panic!("{:?}", found.failures));
    assert_eq!(
        *error,
        BlockError::Unsupported("USB root reset left the port without its device")
    );
    assert!(
        reset.contains("acknowledge reusable root reset (root port 1 PORTSC="),
        "{reset}"
    );
}
