// SPDX-License-Identifier: GPL-2.0-only
//! Real vendor EP0/bulk peer, keeping the exact addressed generation.
use super::super::*;
use crate::usb::descriptor::*;
use crate::usb::xhci::{model, model::bind, *};

pub(crate) fn peer(controller: Shared, dma: &mut dyn DmaAllocator) -> UsbEthernet {
    let mut host = controller.borrow_mut();
    host.set_slot_speed(1, HIGH_SPEED);
    let configuration = vec![
        9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0, 7, 5, 0x81, 2, 0, 2, 0, 7, 5,
        0x02, 2, 0, 2, 0,
    ];
    let mut device = model::addressed_device(&host, vec![configuration]);
    device.speed = HIGH_SPEED;
    device.descriptor[8..12].copy_from_slice(&[0xda, 0x0b, 0x52, 0x81]);
    host.preflight_vendor_ethernet(&mut device).unwrap();
    assert_eq!(device.vendor_ethernet_revision, Some(0x4c00));
    model::select(&mut device).unwrap();
    drop(host);
    // Real class bind executes configuration/endpoint commands and
    // vendor control TDs against the independent RTL8152B register model.
    let mut found = Found::default();
    bind(&controller, device, dma, &mut found);
    assert!(found.failures.is_empty(), "{:?}", found.failures);
    assert!(
        found.vendor_failures.is_empty(),
        "{:?}",
        found.vendor_failures
    );
    assert_eq!(found.vendor_links.len(), 1);
    found.vendor_links.pop().unwrap()
}

pub(crate) fn queue(peer: &mut UsbEthernet) {
    assert!(peer.carrier());
    assert_eq!(peer.receive().unwrap(), None);
    assert!(peer.queued.is_some());
    peer.pending.push_back(vec![0x53; 60]);
    peer.send.bytes_mut().fill(0xa5);
}

pub(crate) fn refuse(peer: &mut UsbEthernet, target: &Shared) {
    peer.controller = target.clone();
    let Some(Backend::Realtek(chip)) = &mut peer.chip else {
        unreachable!()
    };
    let io = &mut chip.io;
    io.source_mut().controller = target.clone();
    let before_id = peer.id;
    let before_pending = peer.pending.clone();
    let before_send = peer.send.bytes().to_vec();
    let before_receive = peer.receive.bytes().to_vec();
    assert!(block_on(io.control(setup(0xc0, 5, 0, 0, 4), &[])).is_err());
    assert!(!peer.carrier());
    assert!(peer.transmit(&[0x88; 60]).is_err());
    assert!(peer.receive().is_err());
    let endpoints = [peer.binding.input.dci(), peer.binding.output.dci()];
    assert!(block_on(quiesce(&peer.controller, peer.id, endpoints)).is_err());
    assert!(peer.stop().is_err());
    assert_eq!(peer.id, before_id);
    assert_eq!(peer.pending, before_pending);
    assert_eq!(peer.send.bytes(), before_send);
    assert_eq!(peer.receive.bytes(), before_receive);
    assert!(peer.queued.is_some());
}
