// SPDX-License-Identifier: GPL-2.0-or-later
//! Authored current-owner USB peers; no observed physical keyboard is implied.
use super::super::*;
use crate::{
    bus::pci::{Address, ConfigSpace, Function},
    usb::xhci::{self, Registers, model},
};
use alloc::{boxed::Box, rc::Rc, vec};
use core::cell::{Cell, RefCell};

const RESET_ENDPOINT_COMMAND: u32 = 14;
const SET_DEQUEUE_COMMAND: u32 = 16;
// The only keyboard usage is in the middle of an otherwise Consumer array.
// This is a boot-interface usability/admission control: canonical
// REPORT decoding already handles the selected data; never force legacy mode.
const MIXED: &[u8] = &[
    5, 1, 9, 6, 0xa1, 1, 0x85, 1, 0x15, 0, 0x25, 4, 0x75, 8, 0x95, 2, 0x0b, 1, 0, 0x0c, 0, 0x0b, 4,
    0, 7, 0, 0x0b, 1, 0, 7, 0, 0x0b, 3, 0, 0x0c, 0, 0x0b, 2, 0, 0x0c, 0, 0x81, 0, 0xc0,
];
struct Config;
impl ConfigSpace for Config {
    fn read32(&self, at: u16) -> core::result::Result<u32, crate::bus::pci::ConfigError> {
        Ok({
            match at {
                0 => 0x0014_1b36,
                8 => 0x0c03_3001,
                _ => 0,
            }
        })
    }
    fn write32(
        &mut self,
        _: u16,
        _: u32,
    ) -> core::result::Result<(), crate::bus::pci::ConfigError> {
        panic!("read-only software fixture")
    }

    fn read8(&self, offset: u16) -> core::result::Result<u8, crate::bus::pci::ConfigError> {
        Ok((self.read32(offset & !3)? >> ((offset & 3) * 8)) as u8)
    }
    fn write8(&mut self, _: u16, _: u8) -> core::result::Result<(), crate::bus::pci::ConfigError> {
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
struct Trap {
    original: Box<dyn Registers>,
    active: Rc<Cell<bool>>,
}
impl Registers for Trap {
    fn read32(&mut self, at: usize) -> u32 {
        assert!(!self.active.get(), "stale sparse HID MMIO read");
        self.original.read32(at)
    }
    fn write32(&mut self, at: usize, value: u32) {
        assert!(!self.active.get(), "stale sparse HID MMIO write");
        self.original.write32(at, value);
    }
}
fn fixture(
    dma: &mut dyn DmaAllocator,
    bytes: &[u8],
) -> (UsbInput, Rc<RefCell<model::CancelModel>>, Rc<Cell<bool>>) {
    let mut function = Function::new(
        Address {
            segment: 0,
            bus: 0,
            device: 1,
            function: 0,
        },
        Config,
    )
    .unwrap();
    let trap = Rc::new(Cell::new(false));
    let (controller, peer) = model::forwarding_fixture(
        dma,
        &[vec![9, 2, 9, 0, 0, 1, 0, 0x80, 50]],
        xhci::FULL_SPEED,
        &mut function,
        |original, _| {
            Box::new(Trap {
                original,
                active: trap.clone(),
            })
        },
    );
    let shared: Shared = Rc::new(RefCell::new(controller));
    let mut physical = model::device(Vec::new());
    physical.id = shared.borrow().test_device_id(1).unwrap();
    let endpoint = Endpoint {
        address: 0x81,
        attributes: 3,
        max_packet: 16,
        interval: 1,
        companion: None,
    };
    {
        let mut c = shared.borrow_mut();
        let mut access = c.device(physical.id).unwrap();
        access.set_configuration(1).unwrap();
        access.configure(&[endpoint], None, dma).unwrap();
    }
    peer.borrow_mut()
        .descriptors
        .push((REPORT_DESCRIPTOR, 0, bytes.to_vec()));
    let length = bytes.len() as u16;
    let interface = Interface {
        number: 4,
        alternate: 0,
        class: CLASS,
        subclass: BOOT,
        protocol: KEYBOARD,
        endpoints: vec![endpoint],
        extra: vec![vec![
            9,
            HID_DESCRIPTOR,
            0x11,
            1,
            0,
            1,
            REPORT_DESCRIPTOR,
            length as u8,
            (length >> 8) as u8,
        ]],
    };
    (
        UsbInput::new(shared, &physical, &interface, endpoint, dma)
            .unwrap()
            .unwrap(),
        peer,
        trap,
    )
}
fn feed(input: &mut UsbInput, peer: &Rc<RefCell<model::CancelModel>>, raw: &[u8]) -> Poll {
    peer.borrow_mut().push_event(raw.to_vec());
    input.poll_hid()
}
fn key(usage: u8, pressed: bool) -> Input {
    Input::Key { usage, pressed }
}

#[test]
fn mixed_page_report_binding_and_dma_poll_parity_keep_canonical_decoder() {
    let mut dma = model::TestRegion::new(1024 * 1024 * 8);
    let mut descriptor = MIXED.to_vec();
    descriptor.extend_from_slice(&[
        5, 1, 9, 6, 0xa1, 1, 0x85, 2, 5, 7, 9, 0, 9, 5, 0x15, 0, 0x25, 1, 0x75, 8, 0x95, 1, 0x81,
        0, 0xc0,
    ]);
    let (mut rich, peer, _) = fixture(&mut dma, &descriptor);
    let (mut legacy, other, _) = fixture(&mut dma, &descriptor);
    for (input, hardware) in [(&rich, &peer), (&legacy, &other)] {
        assert_eq!(input.report_descriptor(), descriptor);
        assert!(
            input.descriptor().is_some(),
            "valid full descriptor remains canonical"
        );
        assert_eq!(
            input.translator.reports.keys.as_ref().unwrap().report,
            Some(1)
        );
        assert_eq!(
            input
                .translator
                .reports
                .keys
                .as_ref()
                .unwrap()
                .pressed(&[1, 0]),
            Some(vec![4])
        );
        assert!(
            hardware
                .borrow()
                .setup
                .iter()
                .any(|(r, _)| *r == [0x21, SET_PROTOCOL, 1, 0, 4, 0, 0, 0])
        );
        assert!(
            !hardware
                .borrow()
                .setup
                .iter()
                .any(|(r, _)| *r == [0x21, SET_PROTOCOL, 0, 0, 4, 0, 0, 0])
        );
        assert!(hardware.borrow().setup.iter().any(|(r, _)| *r
            == [
                0x81,
                GET_DESCRIPTOR,
                0,
                REPORT_DESCRIPTOR,
                4,
                0,
                descriptor.len() as u8,
                0
            ]));
    }
    assert!(rich.poll_hid().reports.is_empty());
    let ticket = rich.queued.unwrap();
    let sequence = [
        (vec![1, 1, 0], vec![key(4, true)]),
        (vec![2, 1], vec![key(5, true)]),
        (vec![1, 2, 0], vec![]),
        (vec![1, 3, 4], vec![key(4, false)]),
        (vec![2, 0], vec![key(5, false)]),
    ];
    for (index, (raw, expected)) in sequence.iter().enumerate() {
        let polled = feed(&mut rich, &peer, raw);
        other.borrow_mut().push_event(raw.clone());
        assert_eq!(polled.inputs, *expected);
        assert_eq!(
            legacy.poll(),
            *expected,
            "actual poll/poll_hid share canonical delivery"
        );
        assert_eq!(rich.last_report(), raw);
        assert_eq!(legacy.last_report(), raw);
        if index == 2 {
            assert!(
                matches!(&polled.reports[0],ReceivedReport::Rejected {bytes,error:DecodeError::KeyboardError(0x0007_0001)} if bytes==raw)
            );
            assert_eq!(rich.translator.held, [4, 5]);
            assert!(polled.events.is_empty());
        } else {
            let ReceivedReport::Data(frame) = &polled.reports[0] else {
                panic!("actual descriptor DMA frame")
            };
            assert_eq!(&frame.bytes, raw);
        }
        if index == 0 {
            assert_ne!(rich.queued, Some(ticket));
            assert_eq!(&rich.buffer.bytes()[..raw.len()], raw);
        }
    }
    assert!(rich.translator.held.is_empty() && legacy.translator.held.is_empty());
    rich.stop_hid().unwrap();
    legacy.stop_hid().unwrap();
}

#[test]
fn unsigned32_selector_keeps_canonical_report_protocol_and_complete_values() {
    // Preserved native canonical parity: this all-keyboard table already kept
    // REPORT protocol. The legacy selector bounds are the corrected behavior.
    let bytes = [
        5, 1, 9, 6, 0xa1, 1, 0x85, 1, 5, 7, 9, 4, 9, 5, 9, 6, 0x17, 0xfe, 0xff, 0xff, 0x7f, 0x27,
        0xff, 0xff, 0xff, 0xff, 0x75, 32, 0x95, 1, 0x81, 0, 0xc0,
    ];
    let mut dma = model::TestRegion::new(512 * 1024 * 8);
    let (mut input, peer, _) = fixture(&mut dma, &bytes);
    assert!(input.descriptor().is_some());
    assert!(
        peer.borrow()
            .setup
            .iter()
            .any(|(r, _)| *r == [0x21, SET_PROTOCOL, 1, 0, 4, 0, 0, 0])
    );
    let raw = [1, 0, 0, 0, 0x80];
    assert_eq!(
        input
            .translator
            .reports
            .keys
            .as_ref()
            .unwrap()
            .pressed(&raw[1..]),
        Some(vec![6])
    );
    let actual = feed(&mut input, &peer, &raw);
    assert_eq!(actual.inputs, [key(6, true)]);
    let ReceivedReport::Data(frame) = &actual.reports[0] else {
        panic!("actual wide report")
    };
    assert_eq!(frame.bytes, raw);
    assert_eq!(
        frame.samples[0].value,
        descriptor::Value::Scalar(0x8000_0000)
    );
    assert_eq!(frame.samples[0].usage, Some(0x0007_0006));
    input.stop_hid().unwrap();
}

#[test]
fn stall_retirement_keeps_sparse_binding_and_original_dma_until_ack() {
    let mut dma = model::TestRegion::new(512 * 1024 * 8);
    let (mut input, peer, _) = fixture(&mut dma, MIXED);
    assert!(input.poll_hid().reports.is_empty());
    let physical = input.buffer.physical();
    peer.borrow_mut().halt_transfer = Some(3);
    let first = feed(&mut input, &peer, &[1, 1, 0]);
    assert_eq!(first.inputs, [key(4, true)]);
    assert!(input.retry.is_some());
    assert_eq!(input.buffer.physical(), physical);
    assert!(peer.borrow().seen.contains(&RESET_ENDPOINT_COMMAND));
    assert!(peer.borrow().seen.contains(&SET_DEQUEUE_COMMAND));
    assert_eq!(input.translator.held, [4]);
    peer.borrow_mut().halt_transfer = None;
    assert_eq!(input.stop_hid().unwrap().inputs, [key(4, false)]);
    input.restart().unwrap();
    assert_eq!(feed(&mut input, &peer, &[1, 1, 0]).inputs, [key(4, true)]);
    assert_eq!(
        input
            .translator
            .reports
            .keys
            .as_ref()
            .unwrap()
            .pressed(&[1, 0]),
        Some(vec![4])
    );
    assert_eq!(input.buffer.physical(), physical);
    input.stop_hid().unwrap();
}

#[test]
fn stale_sparse_binding_cannot_touch_current_native_dma_or_consume_its_completion() {
    let mut ram = vec![0u64; 1024 * 1024];
    let start = ram.as_mut_ptr() as u64;
    // SAFETY: both controller owners keep this exclusive live RAM.
    let mut dma = unsafe { model::RegionAllocator::new(start..start + ram.len() as u64 * 8) };
    let (mut old, old_peer, _) = fixture(&mut dma, MIXED);
    let (mut current, peer, trap) = fixture(&mut dma, MIXED);
    assert_ne!(old.id, current.id);
    assert_eq!(feed(&mut old, &old_peer, &[1, 1, 0]).inputs, [key(4, true)]);
    assert!(current.poll_hid().reports.is_empty());
    peer.borrow_mut().push_event(vec![1, 1, 0]);
    peer.borrow_mut().receive(3);
    let original = old.controller.clone();
    old.controller = current.controller.clone();
    let before = ram.clone();
    let commands = peer.borrow().seen.clone();
    let last = old.last_report().to_vec();
    trap.set(true);
    // The owner finds its device gone and releases the key it held, once,
    // without hardware access.
    assert_eq!(old.poll_hid().inputs, [key(4, false)]);
    assert!(old.poll_hid().inputs.is_empty());
    assert!(old.stop_hid().is_err());
    assert!(old.restart().is_err());
    assert_eq!(ram, before);
    assert_eq!(peer.borrow().seen, commands);
    assert_eq!(old.last_report(), last);
    trap.set(false);
    assert_eq!(current.poll_hid().inputs, [key(4, true)]);
    old.controller = original;
    assert!(old.stop_hid().unwrap().inputs.is_empty());
    current.stop_hid().unwrap();
}

#[test]
fn rejected_boot_descriptor_keeps_original_bytes_and_standard_boot_table() {
    let bytes = [5, 7, 0x19, 9, 0x29, 4, 0x75, 8, 0x95, 1, 0x81, 0];
    let mut dma = model::TestRegion::new(512 * 1024 * 8);
    let (mut input, peer, _) = fixture(&mut dma, &bytes);
    assert_eq!(input.report_descriptor(), bytes);
    assert!(
        input.descriptor().is_none(),
        "real rejected descriptor selects standard boot fallback"
    );
    assert!(
        peer.borrow()
            .setup
            .iter()
            .any(|(r, _)| *r == [0x21, SET_PROTOCOL, 0, 0, 4, 0, 0, 0])
    );
    let raw = [2, 0, 4, 0, 0, 0, 0, 0];
    let actual = feed(&mut input, &peer, &raw);
    assert_eq!(actual.inputs, [key(0xe1, true), key(4, true)]);
    assert!(matches!(&actual.reports[0],ReceivedReport::Boot(bytes) if bytes==&raw));
    assert_eq!(
        input
            .translator
            .reports
            .keys
            .as_ref()
            .unwrap()
            .pressed(&raw),
        Some(vec![0xe1, 4])
    );
    input.stop_hid().unwrap();
}
