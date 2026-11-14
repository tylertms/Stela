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
        let payload = [0x3c; 512];
        found.disks[0].write(4, &payload).unwrap();
        let mut read = [0; 512];
        found.disks[0].read(4, &mut read).unwrap();
        assert_eq!(read, payload);
    }
}
