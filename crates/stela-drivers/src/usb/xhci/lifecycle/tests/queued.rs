// SPDX-License-Identifier: GPL-2.0-only
//! A class's queued reset serviced by the topology owner, on the
//! command/context/port model.
use super::super::*;
use crate::block::BlockDevice;
use crate::bus::dma::TestRegion;
use alloc::collections::BTreeMap;
use hub::model::{Hardware, Node, RegistersModel};

/// A Bulk-Only disk on a root port, enumerated, under its topology owner.
fn bot_disk(dma: &mut TestRegion) -> (Host, Shared, Rc<RefCell<Hardware>>, Vec<Device>) {
    let nodes = BTreeMap::from([(0, Node::bot_disk(HIGH_SPEED))]);
    let hardware = Rc::new(RefCell::new(Hardware::new(64, nodes)));
    hardware.borrow_mut().root_status =
        Some(PP | CCS | PED | u32::from(HIGH_SPEED) << PORT_SPEED_SHIFT);
    let mut c = Controller::new(
        Box::new(RegistersModel(hardware.clone())),
        Clock {
            ticks_per_microsecond: 1,
        },
        dma,
        "queued reset model".into(),
        crate::minimum_model::owned(Clock {
            ticks_per_microsecond: 1,
        }),
    )
    .unwrap();
    let mut failures = Vec::new();
    let devices = c.enumerate(dma, &mut failures);
    let shared = Rc::new(RefCell::new(c));
    let host = Host::new(shared.clone(), &devices, dma, &mut failures);
    assert!(failures.is_empty(), "{failures:?}");
    (host, shared, hardware, devices)
}

#[test]
fn a_queued_reset_rebinds_the_class_on_a_fresh_generation_like_usb_queue_reset_device() {
    let _clock = crate::test_clock::virtualize();
    let mut dma = TestRegion::new(512 * 1024 * 8);
    let (mut host, shared, _, devices) = bot_disk(&mut dma);
    let old = devices[0].id;
    // Only the device's native class owner may ask.
    assert!(shared.borrow_mut().queue_reset(old).is_err());
    let mut original = Found::default();
    bind::bind(&shared, devices[0].clone(), &mut dma, &mut original);
    assert_eq!(original.disks.len(), 1);
    let firmware = crate::firmware::HandoffProvider::new(None);
    // The first look takes what enumeration left; then nothing is due.
    let first = host.poll(&mut dma, &firmware);
    assert!(first.removed.is_empty() && first.added.devices.is_empty());
    assert!(!host.due());
    shared.borrow_mut().queue_reset(old).unwrap();
    shared.borrow_mut().queue_reset(old).unwrap();
    assert!(host.due());

    let changes = host.poll(&mut dma, &firmware);
    assert!(
        changes.added.failures.is_empty(),
        "{:?}",
        changes.added.failures
    );
    assert_eq!(
        changes.removed.iter().map(|d| d.id).collect::<Vec<_>>(),
        [old]
    );
    assert_eq!(changes.added.devices.len(), 1);
    assert_ne!(changes.added.devices[0].id, old);
    assert!(changes.added.disks.is_empty());
    let settled = host.settled(&mut dma);
    assert_eq!(settled.added.disks.len(), 1);
    let mut sector = [0; 512];
    assert!(original.disks[0].read(0, &mut sector).is_err());
    let mut fresh = settled.added;
    fresh.disks[0].read(0, &mut sector).unwrap();
    assert!(shared.borrow_mut().queue_reset(old).is_err());

    assert!(!host.due());
    let quiet = host.poll(&mut dma, &firmware);
    assert!(quiet.removed.is_empty() && quiet.added.devices.is_empty());
    assert!(
        quiet.added.failures.is_empty(),
        "{:?}",
        quiet.added.failures
    );
}

/// usb-storage's `delay_use`: a freshly configured Bulk-Only disk gets its
/// first class request (GET_MAX_LUN) only from the first poll a second
/// later, and polls before return without waiting; one unplugged in that
/// second is dropped without a request or a failure.
#[test]
fn a_bulk_only_disk_gets_its_first_class_request_a_second_after_its_configuration() {
    let _clock = crate::test_clock::virtualize();
    for unplug in [false, true] {
        let mut dma = TestRegion::new(512 * 1024 * 8);
        let (mut host, shared, hardware, devices) = bot_disk(&mut dma);
        let mut original = Found::default();
        bind::bind(&shared, devices[0].clone(), &mut dma, &mut original);
        assert_eq!(original.disks.len(), 1);
        let firmware = crate::firmware::HandoffProvider::new(None);
        host.poll(&mut dma, &firmware);
        let max_lun = || {
            let bus = hardware.borrow();
            let get_max_lun = setup(0xa1, 0xfe, 0, 0, 1);
            bus.requests
                .iter()
                .filter(|(_, _, request)| *request == get_max_lun)
                .count()
        };
        let before = max_lun();
        shared.borrow_mut().queue_reset(devices[0].id).unwrap();
        let configured = host.poll(&mut dma, &firmware);
        assert_eq!(configured.added.devices.len(), 1);
        assert!(configured.added.storage.is_empty() && configured.added.disks.is_empty());
        assert_eq!(max_lun(), before);
        let clock = shared.borrow().clock;
        crate::test_clock::advance(clock, bind::STORAGE_SETTLE_US / 2);
        assert!(!host.attachment_due());
        let start = Clock::ticks();
        let early = host.poll(&mut dma, &firmware);
        assert!(Clock::ticks() - start < 5_000, "a poll waited");
        assert!(early.added.disks.is_empty() && early.added.failures.is_empty());
        assert_eq!(max_lun(), before);
        if unplug {
            hardware.borrow_mut().root_status = Some(PP | CSC);
            hardware.borrow_mut().notify_root();
            let gone = host.poll(&mut dma, &firmware);
            assert_eq!(gone.removed.len(), 1);
            assert!(gone.added.failures.is_empty(), "{:?}", gone.added.failures);
            crate::test_clock::advance(clock, bind::STORAGE_SETTLE_US);
            assert!(!host.attachment_due());
            let late = host.poll(&mut dma, &firmware);
            assert!(late.added.storage.is_empty() && late.added.disks.is_empty());
            assert!(late.added.failures.is_empty(), "{:?}", late.added.failures);
            assert_eq!(max_lun(), before);
        } else {
            let settled = host.settled(&mut dma);
            assert!(
                settled.added.failures.is_empty(),
                "{:?}",
                settled.added.failures
            );
            assert_eq!(settled.added.disks.len(), 1);
            assert_eq!(max_lun(), before + 1);
        }
    }
}
