// SPDX-License-Identifier: GPL-2.0-only
//! Authored composite USB peers; no physical receiver descriptor is implied.
use super::super::*;
use crate::{
    bus::pci::{Address, ConfigSpace, Function},
    usb::xhci::{self, Registers, model},
};
use alloc::{boxed::Box, vec};
use core::cell::Cell;

const TRB_BYTES: usize = 16;
const RING_ENTRIES: usize = 256;
const TRANSFER_EVENT_TRB: u32 = 32;
const SHORT_PACKET_COMPLETION: u32 = 13;
const RESET_ENDPOINT_COMMAND: u32 = 14;
const KEYBOARD_PROFILE: &[u8] = &[
    5, 1, 9, 6, 0xa1, 1, 5, 7, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0, 0x25, 1, 0x75, 1, 0x95, 8, 0x81, 2,
    0x75, 8, 0x95, 1, 0x81, 1, 0x19, 0, 0x29, 0xff, 0x26, 0xff, 0, 0x95, 6, 0x81, 0, 0xc0,
];
const MOUSE_PROFILE: &[u8] = &[
    5, 1, 9, 2, 0xa1, 1, 5, 9, 0x19, 1, 0x29, 3, 0x15, 0, 0x25, 1, 0x75, 1, 0x95, 3, 0x81, 2, 0x75,
    5, 0x95, 1, 0x81, 1, 5, 1, 9, 0x30, 9, 0x31, 0x15, 0x81, 0x25, 0x7f, 0x75, 8, 0x95, 2, 0x81, 6,
    0xc0,
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
        panic!("read-only authored PCI function")
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
struct Hardware {
    original: Box<dyn Registers>,
    peer: Rc<RefCell<model::CancelModel>>,
    descriptors: Vec<(u16, Vec<u8>)>,
    trap: Rc<Cell<bool>>,
}
impl Registers for Hardware {
    fn read32(&mut self, at: usize) -> u32 {
        assert!(!self.trap.get(), "stale receiver MMIO read");
        self.original.read32(at)
    }
    fn write32(&mut self, at: usize, value: u32) {
        assert!(!self.trap.get(), "stale receiver MMIO write");
        if at == 0x1004 && value == 1 {
            let mut peer = self.peer.borrow_mut();
            let address =
                peer.rings[1] + ((peer.indices[1] % (RING_ENTRIES - 1)) * TRB_BYTES) as u64;
            // SAFETY: the real owned EP0 ring has just been published.
            let request = unsafe { (address as *const u64).read_volatile().to_le_bytes() };
            if request[0] == 0x81 && request[1] == GET_DESCRIPTOR && request[3] == REPORT_DESCRIPTOR
            {
                let index = u16::from_le_bytes([request[4], request[5]]);
                let bytes = self
                    .descriptors
                    .iter()
                    .find(|(number, _)| *number == index)
                    .unwrap()
                    .1
                    .clone();
                peer.descriptors
                    .retain(|(kind, _, _)| *kind != REPORT_DESCRIPTOR);
                peer.descriptors.push((REPORT_DESCRIPTOR, 0, bytes));
            }
        }
        self.original.write32(at, value);
    }
}
fn configuration(profiles: &[(u8, u8, u8, &[u8])]) -> Vec<u8> {
    let mut bytes = vec![9, 2, 0, 0, profiles.len() as u8, 1, 0, 0x80, 50];
    for (number, protocol, endpoint, profile) in profiles {
        bytes.extend_from_slice(&[
            9,
            4,
            *number,
            0,
            1,
            CLASS,
            u8::from(*protocol != 0),
            *protocol,
            0,
            9,
            HID_DESCRIPTOR,
            0x11,
            1,
            0,
            1,
            REPORT_DESCRIPTOR,
            profile.len() as u8,
            (profile.len() >> 8) as u8,
            7,
            5,
            *endpoint,
            3,
            32,
            0,
            1,
        ]);
    }
    let length = bytes.len() as u16;
    bytes[2..4].copy_from_slice(&length.to_le_bytes());
    bytes
}
fn fixture(
    dma: &mut dyn DmaAllocator,
    profile: &[u8],
    product: u16,
    extra: bool,
) -> (xhci::Found, Rc<RefCell<model::CancelModel>>, Rc<Cell<bool>>) {
    let mut profiles = vec![
        (0, KEYBOARD, 0x82, KEYBOARD_PROFILE),
        (1, MOUSE, 0x83, MOUSE_PROFILE),
        (2, 0, 0x81, profile),
    ];
    if extra {
        profiles.push((3, 0, 0x84, profile));
    }
    fixture_profiles(dma, &profiles, product)
}
fn fixture_profiles(
    dma: &mut dyn DmaAllocator,
    profiles: &[(u8, u8, u8, &[u8])],
    product: u16,
) -> (xhci::Found, Rc<RefCell<model::CancelModel>>, Rc<Cell<bool>>) {
    fixture_profiles_at(dma, profiles, product, false)
}
/// [`fixture_profiles`], bound after boot when `after_boot`.
fn fixture_profiles_at(
    dma: &mut dyn DmaAllocator,
    profiles: &[(u8, u8, u8, &[u8])],
    product: u16,
    after_boot: bool,
) -> (xhci::Found, Rc<RefCell<model::CancelModel>>, Rc<Cell<bool>>) {
    let config = configuration(profiles);
    let descriptors = profiles
        .iter()
        .map(|(n, _, _, b)| (u16::from(*n), b.to_vec()))
        .collect();
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
    let (mut controller, peer) = model::forwarding_fixture(
        dma,
        core::slice::from_ref(&config),
        xhci::FULL_SPEED,
        &mut function,
        |original, peer| {
            Box::new(Hardware {
                original,
                peer,
                descriptors,
                trap: trap.clone(),
            })
        },
    );
    controller.clock.ticks_per_microsecond = 1000;
    let shared: Shared = Rc::new(RefCell::new(controller));
    let mut physical = model::addressed_device(&shared.borrow(), vec![config.clone()]);
    physical.configuration = config;
    let device = [
        18,
        1,
        0,
        2,
        0,
        0,
        0,
        64,
        0x6d,
        4,
        product as u8,
        (product >> 8) as u8,
        0,
        1,
        0,
        0,
        0,
        1,
    ];
    peer.borrow_mut().descriptors.push((1, 0, device.to_vec()));
    let wire = shared
        .borrow_mut()
        .device(physical.id)
        .unwrap()
        .control([0x80, GET_DESCRIPTOR, 0, 1, 0, 0, 18, 0], &[])
        .unwrap();
    physical.descriptor = wire.try_into().unwrap();
    if after_boot {
        model::boot_ended(&mut shared.borrow_mut());
    }
    let mut found = xhci::Found::default();
    model::bind(&shared, physical, dma, &mut found);
    (found, peer, trap)
}

/// Input devices are what the running system takes: a keyboard, mouse and
/// receiver that arrive after boot are started as at boot.
#[test]
fn input_interfaces_that_arrive_after_boot_are_started() {
    let mut dma = model::TestRegion::new(1024 * 1024 * 8);
    let profiles = [
        (0, KEYBOARD, 0x82, KEYBOARD_PROFILE),
        (1, MOUSE, 0x83, MOUSE_PROFILE),
        (2, 0, 0x81, tests::PROFILE),
    ];
    let (mut found, _, _) = fixture_profiles_at(&mut dma, &profiles, 0xc539, true);
    assert!(found.failures.is_empty(), "{:?}", found.failures);
    assert!(found.unused.is_empty(), "{:?}", found.unused);
    assert_eq!(found.inputs.len(), 3);
    stop(&mut found);
}
// Avoid colliding with the parent HID protocol constants.
fn deliver(peer: &Rc<RefCell<model::CancelModel>>, dci: u8, bytes: &[u8]) {
    let mut peer = peer.borrow_mut();
    let (ticket, buffer, length) = peer.active[usize::from(dci)]
        .take()
        .expect("actual native IN TD armed");
    assert!(bytes.len() <= length);
    // SAFETY: only the peer owns this submitted DMA until this completion.
    unsafe {
        core::ptr::copy_nonoverlapping(bytes.as_ptr(), buffer as *mut u8, bytes.len());
    }
    peer.event(
        ticket,
        SHORT_PACKET_COMPLETION,
        TRANSFER_EVENT_TRB,
        dci,
        length - bytes.len(),
    );
}
fn feed(input: &mut UsbInput, peer: &Rc<RefCell<model::CancelModel>>, bytes: &[u8]) -> Poll {
    if input.queued.is_none() {
        input.poll_hid();
    }
    deliver(peer, input.endpoint.dci(), bytes);
    input.poll_hid()
}
fn vendor(found: &mut xhci::Found) -> &mut UsbInput {
    found
        .inputs
        .iter_mut()
        .find(|input| input.interface == 2)
        .unwrap()
}
fn snapshot(input: &mut UsbInput) -> Snapshot {
    input.receiver_snapshot().unwrap().unwrap()
}
fn initialize(input: &mut UsbInput) {
    assert!(input.poll_hid().inputs.is_empty());
    assert_eq!(
        snapshot(input).initialization,
        Initialization::EnableAcknowledged
    );
    assert!(input.poll_hid().inputs.is_empty());
    assert_eq!(
        snapshot(input).initialization,
        Initialization::QueryAcknowledged
    );
}
fn note(slot: u8, id: u8) -> [u8; 7] {
    [SHORT, slot, CONNECTION, 0x0c, 2, id, 0]
}
fn ping_answer(input: &mut UsbInput, major: u8) -> Vec<u8> {
    let p = snapshot(input).pending.unwrap();
    vec![
        SHORT,
        p.slot,
        ROOT,
        VERSION_FUNCTION | p.software_id,
        major,
        0,
        p.nonce,
    ]
}
fn feature_answer(input: &mut UsbInput, index: u8) -> Vec<u8> {
    let p = snapshot(input).pending.unwrap();
    let mut b = vec![0; LONG_BYTES];
    b[..5].copy_from_slice(&[LONG, p.slot, ROOT, p.software_id, index]);
    b
}
fn stop(found: &mut xhci::Found) {
    for input in &mut found.inputs {
        input.stop_hid().unwrap();
    }
}

#[test]
fn composite_ep0_dma_initialization_and_discovery_preserve_standard_input_once() {
    let mut dma = model::TestRegion::new(1024 * 1024 * 8);
    let (mut found, peer, _) = fixture(&mut dma, tests::PROFILE, 0xc539, true);
    assert!(found.failures.is_empty());
    assert_eq!(found.inputs.len(), 4);
    assert!(
        found.inputs[3].receiver_snapshot().unwrap().is_none(),
        "interface>=3 remains generic"
    );
    let input = vendor(&mut found);
    assert_eq!(snapshot(input).member_interfaces, [0, 1, 2]);
    assert_eq!(input.report_descriptor(), tests::PROFILE);
    initialize(input);
    assert!(
        snapshot(input).routes.is_empty(),
        "two actual USB ACKs grant no protocol"
    );
    let report = feed(input, &peer, &note(1, 9));
    assert_eq!(report.reports.len(), 1);
    assert!(report.inputs.is_empty());
    assert_eq!(input.last_report(), note(1, 9));
    let mut answer = ping_answer(input, 2);
    answer[0] = LONG;
    answer.resize(LONG_BYTES, 0); // Source-supported LONG positive version form.
    let actual = feed(input, &peer, &answer);
    assert!(
        matches!(&actual.reports[0], ReceivedReport::Data(frame) if frame.bytes == answer && frame.bytes.len() == LONG_BYTES)
    );
    assert_eq!(
        snapshot(input).routes[0].version,
        Some(Version { major: 2, minor: 0 })
    );
    assert!(snapshot(input).routes[0].feature.is_none());
    let answer = feature_answer(input, 5);
    let report = feed(input, &peer, &answer);
    assert!(matches!(&report.reports[0],ReceivedReport::Data(frame) if frame.bytes==answer));
    assert_eq!(
        snapshot(input).routes[0].feature,
        Some(Feature {
            id: WIRELESS_STATUS,
            index: 5
        })
    );
    let setup = peer.borrow().setup.clone();
    assert!(
        setup
            .iter()
            .any(|(r, b)| *r == [0x21, SET_REPORT, SHORT, 2, 2, 0, 7, 0]
                && *b == [SHORT, RECEIVER, 0x80, 0, 0, 9, 0])
    );
    assert!(
        setup
            .iter()
            .any(|(r, b)| *r == [0x21, SET_REPORT, SHORT, 2, 2, 0, 7, 0]
                && *b == [SHORT, RECEIVER, 0x80, 2, 2, 0, 0])
    );
    assert!(
        !setup.iter().any(|(_, b)| b.first() == Some(&0x20)),
        "c539 never switches DJ mode"
    );
    for number in 0..=3 {
        assert!(setup.iter().any(|(r, _)| r[0] == 0x81
            && r[1] == GET_DESCRIPTOR
            && r[3] == REPORT_DESCRIPTOR
            && r[4] == number));
    }
    let keyboard = found.inputs.iter_mut().find(|i| i.interface == 0).unwrap();
    let input = feed(keyboard, &peer, &[0, 0, 4, 0, 0, 0, 0, 0]);
    assert_eq!(
        input.inputs,
        [Input::Key {
            usage: 4,
            pressed: true
        }]
    );
    assert!(
        feed(keyboard, &peer, &[0, 0, 4, 0, 0, 0, 0, 0])
            .inputs
            .is_empty()
    );
    let mouse = found.inputs.iter_mut().find(|i| i.interface == 1).unwrap();
    let first = feed(mouse, &peer, &[0, 3, 0xfd]);
    assert_eq!(first.inputs, [Input::Motion { dx: 3, dy: -3 }]);
    deliver(&peer, mouse.endpoint.dci(), &[0, 3, 0xfd]);
    assert_eq!(mouse.poll(), first.inputs, "same native poll owner/parity");
    assert_eq!(
        snapshot(vendor(&mut found)).routes[0].version,
        Some(Version { major: 2, minor: 0 })
    );
    stop(&mut found);
}

#[test]
fn wrong_slot_nonce_and_old_success_or_error_replies_cannot_requalify_reconnect() {
    let mut dma = model::TestRegion::new(1024 * 1024 * 8);
    let (mut found, peer, _) = fixture(&mut dma, tests::PROFILE, 0xc539, false);
    let input = vendor(&mut found);
    initialize(input);
    feed(input, &peer, &note(1, 1));
    let old = ping_answer(input, 2);
    let mut wrong = old.clone();
    wrong[1] = 2;
    feed(input, &peer, &wrong);
    assert!(snapshot(input).pending.is_some());
    feed(input, &peer, &old);
    let feature = feature_answer(input, 8);
    feed(input, &peer, &feature);
    feed(input, &peer, &note(1, 2));
    let current = ping_answer(input, 2);
    assert_ne!(current[3], old[3]);
    feed(input, &peer, &old);
    feed(
        input,
        &peer,
        &[SHORT, 1, RAP_ERROR, ROOT, old[3], INVALID_SUBID, 0],
    );
    assert!(snapshot(input).routes[0].version.is_none());
    feed(input, &peer, &current);
    feed(input, &peer, &feature);
    assert!(snapshot(input).pending.is_some());
    let current_feature = feature_answer(input, 0);
    feed(input, &peer, &current_feature);
    assert_eq!(
        snapshot(input).routes[0].feature,
        Some(Feature {
            id: WIRELESS_STATUS,
            index: 0
        })
    );
    feed(input, &peer, &note(1, 3));
    let mut bad = ping_answer(input, 2);
    bad[6] ^= 0xff;
    feed(input, &peer, &bad);
    assert_eq!(snapshot(input).routes[0].error, Some(Error::InvalidNonce));
    feed(input, &peer, &note(1, 4));
    let pending = snapshot(input).pending.unwrap();
    feed(
        input,
        &peer,
        &[
            SHORT,
            1,
            RAP_ERROR,
            ROOT,
            VERSION_FUNCTION | pending.software_id,
            INVALID_SUBID,
            0,
        ],
    );
    assert_eq!(
        snapshot(input).routes[0].version,
        Some(Version { major: 1, minor: 0 })
    );
    assert!(snapshot(input).pending.is_none());
    let mut unsupported = note(1, 4);
    unsupported[3] = 0x05;
    let commands = peer.borrow().setup.len();
    let raw = feed(input, &peer, &unsupported);
    assert!(matches!(&raw.reports[0],ReceivedReport::Data(frame) if frame.bytes==unsupported));
    assert!(snapshot(input).routes[0].version.is_none());
    assert_eq!(
        snapshot(input).routes[0].error,
        Some(Error::UnsupportedNotification)
    );
    assert_eq!(peer.borrow().setup.len(), commands);
    feed(input, &peer, &note(1, 5));
    let late = ping_answer(input, 2);
    let mut unsupported = note(1, 5);
    unsupported[4] = 9; // Changed unknown device type.
    let commands = peer.borrow().setup.len();
    feed(input, &peer, &unsupported);
    assert!(snapshot(input).pending.is_none());
    feed(input, &peer, &late);
    assert!(snapshot(input).routes[0].version.is_none());
    assert_eq!(
        peer.borrow().setup.len(),
        commands,
        "unsupported replacement never schedules another USB query"
    );
    stop(&mut found);
}

#[test]
fn stop_restart_stall_and_external_output_revoke_protocol_without_reusing_history() {
    let mut dma = model::TestRegion::new(1024 * 1024 * 8);
    let (mut found, peer, _) = fixture(&mut dma, tests::PROFILE, 0xc539, false);
    let input = vendor(&mut found);
    initialize(input);
    feed(input, &peer, &note(1, 1));
    let old = ping_answer(input, 2);
    let physical = input.buffer.physical();
    input.stop_hid().unwrap();
    assert!(snapshot(input).pending.is_none());
    input.restart().unwrap();
    initialize(input);
    feed(input, &peer, &note(1, 1));
    assert_ne!(ping_answer(input, 2)[3], old[3]);
    feed(input, &peer, &old);
    assert!(snapshot(input).routes[0].version.is_none());
    let answer = ping_answer(input, 2);
    feed(input, &peer, &answer);
    let answer = feature_answer(input, 4);
    feed(input, &peer, &answer);
    input
        .set_report(
            ReportType::Output,
            SHORT,
            &[SHORT, RECEIVER, 0x80, 2, 2, 0, 0],
        )
        .unwrap();
    assert!(snapshot(input).routes[0].version.is_none());
    input.stop_hid().unwrap();
    input.restart().unwrap();
    initialize(input);
    let rejected = feed(input, &peer, &[SHORT, 1]);
    assert!(
        matches!(&rejected.reports[0], ReceivedReport::Rejected { bytes, .. } if bytes == &[SHORT,1])
    );
    assert!(snapshot(input).pending.is_none());
    assert!(
        snapshot(input)
            .routes
            .iter()
            .all(|route| route.version.is_none())
    );
    input.stop_hid().unwrap();
    input.restart().unwrap();
    initialize(input);
    peer.borrow_mut().halt_transfer = Some(3);
    feed(input, &peer, &note(1, 2));
    assert!(input.retry.is_some());
    assert_eq!(input.buffer.physical(), physical);
    assert!(peer.borrow().seen.contains(&RESET_ENDPOINT_COMMAND));
    assert!(snapshot(input).pending.is_none());
    peer.borrow_mut().halt_transfer = None;
    stop(&mut found);
}

#[test]
fn stale_member_cannot_mutate_shared_current_group_or_consume_current_dma() {
    let mut ram = vec![0u64; 2 * 1024 * 1024];
    let start = ram.as_mut_ptr() as u64;
    // SAFETY: both separately addressed native controllers keep this backing.
    let mut dma = unsafe { model::RegionAllocator::new(start..start + ram.len() as u64 * 8) };
    let (mut old, _, _) = fixture(&mut dma, tests::PROFILE, 0xc539, false);
    let (mut current, peer, trap) = fixture(&mut dma, tests::PROFILE, 0xc539, false);
    let input = vendor(&mut current);
    initialize(input);
    feed(input, &peer, &note(1, 1));
    let before = snapshot(input);
    let answer = ping_answer(input, 2);
    deliver(&peer, input.endpoint.dci(), &answer);
    let current_controller = input.controller.clone();
    let current_link = input.receiver.clone();
    let current_commands = peer.borrow().seen.clone();
    let bytes = ram.clone();
    // A current standard owner with the vendor member's Link keeps its
    // identity; the mismatched member seal refuses before MMIO/state.
    let standard = current
        .inputs
        .iter_mut()
        .find(|input| input.interface == 0)
        .unwrap();
    let own_standard_link = standard.receiver.clone();
    standard.receiver = current_link.clone();
    trap.set(true);
    assert!(standard.receiver_snapshot().is_err());
    standard.invalidate_receiver(Error::Transport);
    standard.receiver_observe(&note(2, 99));
    trap.set(false);
    standard.receiver = own_standard_link;
    assert_eq!(snapshot(vendor(&mut current)), before);
    let stale = vendor(&mut old);
    let own_controller = stale.controller.clone();
    let own_id = stale.id;
    let own_link = stale.receiver.clone();
    stale.controller = current_controller;
    stale.receiver = current_link;
    // Keep the old SlotId while injecting a mismatched current group.
    // No current numeric credential is installed in this old owner.
    trap.set(true);
    assert!(stale.receiver_snapshot().is_err());
    assert!(stale.poll_hid().inputs.is_empty());
    assert!(stale.stop_hid().is_err());
    assert_eq!(ram, bytes);
    assert_eq!(peer.borrow().seen, current_commands);
    trap.set(false);
    assert_eq!(snapshot(vendor(&mut current)), before);
    assert_eq!(vendor(&mut current).poll_hid().reports.len(), 1);
    assert_eq!(
        snapshot(vendor(&mut current)).routes[0].version,
        Some(Version { major: 2, minor: 0 })
    );
    // Advance only the existing authored clock seam; expire a sent
    // Feature request through the current-member snapshot path without sleep.
    let live = vendor(&mut current);
    let deadline = snapshot(live).pending.unwrap().deadline_us;
    live.controller.borrow_mut().clock.ticks_per_microsecond = 1;
    assert!(live.controller.borrow().clock.now_us() >= deadline);
    assert!(snapshot(live).pending.is_none());
    assert_eq!(snapshot(live).routes[0].error, Some(Error::Timeout));
    live.controller.borrow_mut().clock.ticks_per_microsecond = 1000;
    stale.controller = own_controller;
    stale.id = own_id;
    stale.receiver = own_link;
    stale.interface = 2;
    stop(&mut old);
    let id = vendor(&mut current).id;
    let controller = vendor(&mut current).controller.clone();
    let endpoints: Vec<_> = current.inputs.iter().map(|input| input.endpoint).collect();
    stop(&mut current);
    {
        let mut host = controller.borrow_mut();
        let mut access = host.device(id).unwrap();
        access.deconfigure(&endpoints).unwrap();
        access.set_configuration(0).unwrap();
    }
    assert!(
        vendor(&mut current).receiver_snapshot().is_err(),
        "actual acknowledged configuration0 revokes sealed configuration1"
    );
    assert_eq!(peer.borrow().configuration_value, 0);
}

#[test]
fn candidate_or_profile_refusal_keeps_generic_hid_owners_and_raw_reports() {
    let mut dma = model::TestRegion::new(2 * 1024 * 1024 * 8);
    let mut malformed = tests::PROFILE.to_vec();
    malformed[21] = 4;
    let mut split = vec![
        6, 0, 0xff, 9, 1, 0xa1, 1, 0x85, 0x10, 0x75, 8, 0x95, 3, 0x15, 0, 0x26, 0xff, 0, 9, 1,
        0x81, 0, 0xc0, 9, 1, 0xa1, 1, 9, 1, 0x81, 0, 0x95, 6, 9, 1, 0x91, 0, 0xc0,
    ];
    split.extend_from_slice(&tests::PROFILE[27..]);
    for (product, profile, refused) in [
        (0xc539, malformed.as_slice(), true),
        (0xc539, split.as_slice(), true),
        (0xc53a, tests::PROFILE, false),
    ] {
        let (mut found, peer, _) = fixture(&mut dma, profile, product, false);
        assert_eq!(found.inputs.len(), 3);
        assert_eq!(!found.failures.is_empty(), refused);
        let input = vendor(&mut found);
        assert!(input.receiver_snapshot().unwrap().is_none());
        assert_eq!(input.report_descriptor(), profile);
        let report = feed(input, &peer, &note(1, 1));
        assert!(matches!(&report.reports[0],ReceivedReport::Data(frame) if frame.bytes==note(1,1)));
        assert!(report.inputs.is_empty());
        assert!(
            !peer
                .borrow()
                .setup
                .iter()
                .any(|(r, _)| r[0] == 0x21 && r[1] == SET_REPORT)
        );
        stop(&mut found);
    }
    let profiles = [
        (0, 0, 0x82, tests::PROFILE),
        (1, MOUSE, 0x83, MOUSE_PROFILE),
        (2, 0, 0x81, tests::PROFILE),
    ];
    let (mut found, peer, _) = fixture_profiles(&mut dma, &profiles, 0xc539);
    assert_eq!(found.inputs.len(), 3);
    assert!(
        found.failures.iter().any(
            |(_, error)| *error == BlockError::Unsupported("ambiguous native HID++ interfaces")
        )
    );
    for input in found.inputs.iter_mut().filter(|input| input.interface != 1) {
        assert!(input.receiver_snapshot().unwrap().is_none());
        let raw = note(1, 4);
        let report = feed(input, &peer, &raw);
        assert!(matches!(&report.reports[0],ReceivedReport::Data(frame) if frame.bytes==raw));
    }
    assert!(
        !peer
            .borrow()
            .setup
            .iter()
            .any(|(r, _)| r[0] == 0x21 && r[1] == SET_REPORT)
    );
    stop(&mut found);
}

#[test]
fn output_status_failure_is_not_a_protocol_ack_or_an_automatic_retry() {
    let mut dma = model::TestRegion::new(1024 * 1024 * 8);
    let (mut found, peer, _) = fixture(&mut dma, tests::PROFILE, 0xc539, false);
    peer.borrow_mut().stall_control = Some(SET_REPORT);
    let input = vendor(&mut found);
    assert!(input.poll_hid().inputs.is_empty());
    assert_eq!(
        snapshot(input).initialization,
        Initialization::Unavailable(Error::Transport)
    );
    assert!(snapshot(input).routes.is_empty());
    let commands = peer.borrow().setup.len();
    input.poll_hid();
    assert_eq!(
        peer.borrow().setup.len(),
        commands,
        "no automatic state-changing retry"
    );
    assert_eq!(
        input.first_error(),
        Some(BlockError::Device { status: STALL })
    );
    assert_eq!(input.state(), State::Running);
    assert!(!input.failed());
    let physical = input.buffer.physical();
    let ticket = input.queued;
    assert!(ticket.is_some());
    peer.borrow_mut().stall_control = None;
    input.stop_hid().unwrap();
    input.restart().unwrap();
    peer.borrow_mut().short_control_out = true;
    input.poll_hid();
    assert_eq!(
        snapshot(input).initialization,
        Initialization::Unavailable(Error::Transport)
    );
    assert_eq!(
        input.first_error(),
        Some(BlockError::Device { status: STALL }),
        "earlier actual STALL wins over later short-OUT error"
    );
    assert_eq!(input.buffer.physical(), physical);
    assert!(input.queued.is_some());
    peer.borrow_mut().short_control_out = false;
    input.stop_hid().unwrap();
    input.restart().unwrap();
    initialize(input);
    stop(&mut found);
}
