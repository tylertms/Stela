use super::{
    model::{Hardware, Io},
    *,
};
use crate::usb::descriptor::*;
use crate::usb::ethernet;
use crate::usb::xhci::{model, model::*, *};
use alloc::rc::Rc;
use core::cell::RefCell;
use stela_driver_base::stepped::block_on;

#[test]
fn ax179_real_power_eeprom_efuse_old_led_phy_and_link_states() {
    for bus in [0, 2, 4] {
        for ua2 in [false, true] {
            for source in 0..3 {
                let hw = Rc::new(RefCell::new(Hardware::new(bus, ua2)));
                {
                    let mut h = hw.borrow_mut();
                    if source == 0 {
                        h.eeprom.fill(0);
                        h.eeprom[5] = 0xff;
                        h.eeprom[0x42] = 0x8c23;
                    } else if source == 1 {
                        h.efuse.fill(0);
                        h.efuse[51] = 0x8c;
                        h.efuse[52] = 0x23;
                        h.efuse[63] = 255 - 0x8c - 0x23;
                    } else {
                        h.eeprom[0x3c] = 0xfc00;
                    }
                    h.eeprom[0x43] = 0x100;
                }
                let (mut chip, mac) = block_on(Chip::attach(Io(hw.clone()))).unwrap();
                assert_eq!(mac.0, [2, 0x34, 0x56, 0x78, 0x9a, 0xbc]);
                assert_eq!(chip.general_status, if ua2 { 4 } else { 0 });
                assert_eq!(chip.phy_id, 0x0007_c001);
                let h = hw.borrow();
                assert_eq!(&h.delays[..2], &[500_000, 200_000]);
                assert_eq!(h.mac[0x33], 0x0b);
                assert_eq!(h.word(0x26), 0x1020);
                assert_eq!(h.mac[0x34], 0);
                assert_eq!(h.mac[0x35], 0);
                assert_eq!(h.mmd[&(7, 0x3c)], 0);
                assert_eq!(h.phy[&(3, 0, 0x19)], 0x3246);
                assert_eq!(h.phy[&(0, 0, 0)] & 0x1200, 0x1200);
                assert_eq!(h.mac[0x25], if ua2 { 0 } else { 0xe0 });
                let led = if source == 2 {
                    led::old_led(0xfc)
                } else {
                    0x8c23
                };
                let (a, l, d) = led::led_masks(led, 0xa55a, 0x5aa5);
                assert_eq!(h.phy[&(7, 0x2c, 0x1a)], a);
                assert_eq!(h.phy[&(7, 0x2c, 0x1c)], l);
                assert_eq!(h.mac[0x73], d);
                drop(h);
                for (speed, expected_queue, medium) in [
                    (
                        0x8000,
                        if bus == 4 {
                            0
                        } else if bus == 2 {
                            1
                        } else {
                            3
                        },
                        0x139,
                    ),
                    (0x4000, if bus != 0 { 2 } else { 3 }, 0x330),
                    (0, 3, 0x130),
                ] {
                    for duplex in [0, 0x2000] {
                        hw.borrow_mut()
                            .phy
                            .insert((0, 0, 0x11), speed | duplex | 0x400);
                        assert!(block_on(chip.carrier()).unwrap());
                        assert_eq!(&hw.borrow().mac[0x2e..0x33], &BULKIN[expected_queue]);
                        assert_eq!(
                            hw.borrow().word(0x22),
                            medium | if duplex != 0 { 2 } else { 0 }
                        );
                        block_on(chip.stop()).unwrap();
                        assert_eq!(hw.borrow().word(0x22) & 0x100, 0);
                        assert_eq!(hw.borrow().word(0x0b), 0);
                    }
                }
                hw.borrow_mut().phy.insert((0, 0, 0x11), 0);
                assert!(!block_on(chip.carrier()).unwrap());
            }
        }
    }
}

#[test]
fn ax179_all_startup_transfer_faults_and_short_reads_poison_access() {
    fn model(source: u8, ua2: bool) -> Hardware {
        let mut h = Hardware::new(4, ua2);
        if source == 0 {
            h.eeprom.fill(0);
            h.eeprom[5] = 0xff;
            h.eeprom[0x42] = 0x8c23;
        }
        if source == 1 {
            h.efuse.fill(0);
            h.efuse[51] = 0x8c;
            h.efuse[52] = 0x23;
            h.efuse[63] = 0x50;
        }
        h.eeprom[0x43] = 0x100;
        h
    }
    for source in 0..3 {
        for ua2 in [false, true] {
            let hw = Rc::new(RefCell::new(model(source, ua2)));
            block_on(Chip::attach(Io(hw.clone()))).unwrap();
            let count = hw.borrow().trace.len();
            for at in 0..count {
                let read = hw.borrow().trace[at].0[0] == 0xc0;
                for short in [false, true] {
                    if short && !read {
                        continue;
                    }
                    let hw = Rc::new(RefCell::new(model(source, ua2)));
                    if short {
                        hw.borrow_mut().short_at = Some(at);
                    } else {
                        hw.borrow_mut().fail_at = Some(at);
                    }
                    assert!(
                        block_on(Chip::attach(Io(hw.clone()))).is_err(),
                        "{source}/{ua2}/{at}/{short}"
                    );
                    assert_eq!(hw.borrow().trace.len(), at + 1);
                }
            }
        }
    }
    let hw = Rc::new(RefCell::new(Hardware::new(4, true)));
    let (mut chip, _) = block_on(Chip::attach(Io(hw.clone()))).unwrap();
    let at = hw.borrow().trace.len();
    hw.borrow_mut().fail_at = Some(at);
    assert!(block_on(chip.carrier()).is_err());
    assert!(block_on(chip.carrier()).is_err());
    assert_eq!(hw.borrow().trace.len(), at + 1);
}

#[test]
fn ax179_hardware_deadlines_and_invalid_identity_refuse_success() {
    for case in 0..5 {
        let hw = Rc::new(RefCell::new(Hardware::new(4, true)));
        match case {
            0 => hw.borrow_mut().stuck_srom = true,
            1 => {
                hw.borrow_mut().phy.insert((0, 0, 2), 0);
                hw.borrow_mut().phy.insert((0, 0, 3), 0);
            }
            2 => hw.borrow_mut().mac[0x10] = 1,
            3 => {
                hw.borrow_mut().phy.insert((0, 0, 0), 0);
            }
            _ => hw.borrow_mut().mac[0x10..0x16].fill(0),
        }
        assert!(block_on(Chip::attach(Io(hw))).is_err(), "{case}");
    }
    let hw = Rc::new(RefCell::new(Hardware::new(4, true)));
    let (mut chip, _) = block_on(Chip::attach(Io(hw.clone()))).unwrap();
    hw.borrow_mut().stuck_fifo = true;
    assert!(matches!(block_on(chip.carrier()), Err(BlockError::Timeout)));
    let at = hw.borrow().trace.len();
    assert!(block_on(chip.carrier()).is_err());
    assert_eq!(hw.borrow().trace.len(), at);
}

#[test]
#[ignore = "oracle: STELA_ASIX_LED_ORACLE"]
fn unchanged_linux_asix_led_eeprom_mmd_fields() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let fields = std::fs::read_to_string(std::env::var("STELA_ASIX_LED_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(fields.as_bytes())),
        "6767be649469c4a73d2341c5e9f62d1f74a2fc9e40f3a2f37e10d2e49d3ea035"
    );
    let mut count = 0;
    for line in fields.lines() {
        let v: Vec<u32> = line
            .split_whitespace()
            .map(|v| v.parse().unwrap())
            .collect();
        assert_eq!(v.len(), 13);
        let (kind, raw, ua2, detach) = (v[0], v[1] as u16, v[2] != 0, v[3] != 0);
        let selected = if kind != 2 && raw != 0xffff && raw & 0x8000 != 0 {
            raw
        } else {
            led::old_led(if kind == 0 { 0xff } else { raw as u8 })
        };
        let (a, l, d) = led::led_masks(selected, 0xa55a, 0x5aa5);
        assert_eq!(
            (u32::from(a), u32::from(l), u32::from(d)),
            (v[4], v[5], v[6]),
            "{line}"
        );
        assert_eq!(
            &v[7..],
            &[
                if ua2 { 0 } else { 0xe0 },
                if detach { 0x0b } else { 3 },
                if detach { 0x1020 } else { 0x20 },
                0x3246,
                0,
                0
            ],
            "{line}"
        );
        if kind != 0 || raw % 4096 == 0 {
            let hw = Rc::new(RefCell::new(Hardware::new(4, ua2)));
            {
                let mut h = hw.borrow_mut();
                h.eeprom[0x3c] = raw.wrapping_shl(8);
                if kind == 0 {
                    h.eeprom.fill(0);
                    h.eeprom[5] = 0xff;
                    h.eeprom[0x42] = raw;
                    h.eeprom[0x3c] = 0xff00;
                }
                if kind == 1 {
                    h.efuse.fill(0);
                    h.efuse[51] = (raw >> 8) as u8;
                    h.efuse[52] = raw as u8;
                    h.efuse[63] =
                        (255 - (u16::from(h.efuse[51]) + u16::from(h.efuse[52])) % 255) as u8;
                }
                h.eeprom[0x43] = if detach { 0x100 } else { 0xffff };
            }
            block_on(Chip::attach(Io(hw.clone()))).unwrap();
            let h = hw.borrow();
            assert_eq!(
                (
                    u32::from(h.phy[&(7, 0x2c, 0x1a)]),
                    u32::from(h.phy[&(7, 0x2c, 0x1c)]),
                    u32::from(h.mac[0x73])
                ),
                (v[4], v[5], v[6]),
                "{line}"
            );
            assert_eq!(u32::from(h.mac[0x25]), v[7]);
            assert_eq!(u32::from(h.mac[0x33]), v[8]);
            assert_eq!(u32::from(h.word(0x26)), v[9]);
            assert_eq!(u32::from(h.phy[&(3, 0, 0x19)]), v[10]);
            assert_eq!(u32::from(h.mmd[&(7, 0x3c)]), v[11]);
        }
        count += 1;
    }
    assert_eq!(count, 66576);
}

#[test]
#[ignore = "oracle: STELA_ASIX_LINK_ORACLE"]
fn unchanged_linux_asix_negotiated_queue_and_medium_fields() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let fields = std::fs::read_to_string(std::env::var("STELA_ASIX_LINK_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(fields.as_bytes())),
        "8b437dc92942d630ae98fc666154363f92ee8087c91cc57b74c4dc258c0f5467"
    );
    let mut count = 0;
    for line in fields.lines() {
        let v: Vec<u32> = line
            .split_whitespace()
            .map(|v| v.parse().unwrap())
            .collect();
        assert_eq!(v.len(), 9);
        let hw = Rc::new(RefCell::new(Hardware::new(v[0] as u8, true)));
        let (mut chip, _) = block_on(Chip::attach(Io(hw.clone()))).unwrap();
        hw.borrow_mut().phy.insert((0, 0, 0x11), v[1] as u16);
        assert!(block_on(chip.carrier()).unwrap());
        assert_eq!(u32::from(hw.borrow().word(0x22)), v[2], "{line}");
        for (actual, expected) in hw.borrow().mac[0x2e..0x33].iter().zip(&v[3..8]) {
            assert_eq!(u32::from(*actual), *expected, "{line}");
        }
        assert_eq!(1024 * (u32::from(hw.borrow().mac[0x31]) + 2), v[8]);
        assert!(v[8] as usize <= chip.receive_bytes());
        count += 1;
    }
    assert_eq!(count, 24);
}

#[test]
#[ignore = "oracle: STELA_ASIX_CHECKSUM_ORACLE"]
fn unchanged_linux_asix_eeprom_and_efuse_checksum_mutations() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let fields =
        std::fs::read_to_string(std::env::var("STELA_ASIX_CHECKSUM_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(fields.as_bytes())),
        "703da1345df07e177c81cdfd5a3c1f56b9d463ac5ea3d4b99460edb4ab3bf3c2"
    );
    let mut count = 0;
    for line in fields.lines() {
        let v: Vec<_> = line.split_whitespace().collect();
        assert_eq!(v.len(), 3);
        let bytes: Vec<u8> = (0..v[1].len())
            .step_by(2)
            .map(|at| u8::from_str_radix(&v[1][at..at + 2], 16).unwrap())
            .collect();
        let hw = Rc::new(RefCell::new(Hardware::new(4, true)));
        let (mut chip, _) = block_on(Chip::attach(Io(hw.clone()))).unwrap();
        let valid = if v[0] == "0" {
            assert_eq!(bytes.len(), 12);
            for (i, word) in bytes.as_chunks::<2>().0.iter().enumerate() {
                hw.borrow_mut().eeprom[i] = u16::from_le_bytes(*word);
            }
            block_on(chip.eeprom_valid()).unwrap()
        } else {
            assert_eq!(bytes.len(), 64);
            hw.borrow_mut().efuse.copy_from_slice(&bytes);
            block_on(chip.efuse_led()).unwrap().is_some()
        };
        assert_eq!(valid, v[2] == "1", "{line}");
        count += 1;
    }
    assert_eq!(count, 686);
}

#[test]
#[ignore = "oracle: STELA_ASIX_IDS_ORACLE"]
fn unchanged_linux_asix_exact_ids_and_shared_callback_flags() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let text = std::fs::read_to_string(std::env::var("STELA_ASIX_IDS_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(text.as_bytes())),
        "685ea8ff81f174d8a8a2ffc27c7ad5c046b118119ead252220ddb60762bd37d9"
    );
    let mut count = 0;
    for (line, id) in text.lines().zip(identity::IDS) {
        let v: Vec<_> = line.split_whitespace().collect();
        assert_eq!(v.len(), 14);
        let pair = (
            u16::from_str_radix(v[0], 16).unwrap(),
            u16::from_str_radix(v[1], 16).unwrap(),
        );
        assert_eq!(pair, (id.0, id.1));
        let fields: Vec<u32> = v[2..].iter().map(|v| v.parse().unwrap()).collect();
        assert_eq!(fields, [255, 255, 0, 0x60, 1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(
            id.2,
            if pair == (0x0b95, 0x178a) {
                Identity::Ax88178a
            } else {
                Identity::Ax88179
            }
        );
        count += 1;
    }
    assert_eq!(count, 13);
    assert_eq!(identity::IDS.len(), 13);
}

#[test]
fn ax88772_native_all_candidates_revisions_split_rx_and_quarantine() {
    use crate::net::{Link, LinkError};
    let mut cases = 0;
    for (vid, pid) in ethernet::asix_legacy_ids() {
        for code in [0, 0x10, 0x20] {
            for (speed, packet) in [
                (FULL_SPEED, 8u16),
                (FULL_SPEED, 16),
                (FULL_SPEED, 32),
                (FULL_SPEED, 64),
                (HIGH_SPEED, 512),
            ] {
                for scenario in 0..6 {
                    cases += 1;
                    let mut dma = TestRegion::new(64 * 1024 * 8);
                    let (mut controller, model) =
                        unconfigured_cancellation_controller_with_configurations(
                            &mut dma,
                            &[vec![9, 2, 9, 0, 0, 9, 0, 0x80, 50]],
                        );
                    controller.set_slot_speed(1, speed);
                    model::minimum_on_controller_clock(&mut controller);
                    model.borrow_mut().asix_legacy = Some(ethernet::LegacyModel::new(
                        code,
                        if code == 0x10 { 0x003b1861 } else { 0x003b1881 },
                    ));
                    let (input, output) = if scenario % 2 == 0 {
                        (0x82u8, 3u8)
                    } else {
                        (0x81, 2)
                    };
                    let in_dci = (2 * (input & 15) + 1) as usize;
                    let out_dci = (2 * output) as usize;
                    let mut config =
                        vec![9, 2, 32, 0, 1, 9, 0, 0x80, 50, 9, 4, 0, 0, 2, 0xff, 0, 0, 0];
                    for address in [input, output] {
                        config.extend_from_slice(&[
                            7,
                            5,
                            address,
                            2,
                            packet as u8,
                            (packet >> 8) as u8,
                            0,
                        ]);
                    }
                    let mut dev = model::addressed_device(&controller, vec![config]);
                    dev.speed = speed;
                    dev.descriptor[8..10].copy_from_slice(&vid.to_le_bytes());
                    dev.descriptor[10..12].copy_from_slice(&pid.to_le_bytes());
                    model::select(&mut dev).unwrap();
                    if scenario == 4 {
                        model.borrow_mut().short_control_out = true;
                    }
                    if scenario == 5 {
                        model.borrow_mut().asix_legacy.as_mut().unwrap().phy_address = 3;
                    }
                    let shared = Rc::new(RefCell::new(controller));
                    let mut found = Found::default();
                    bind(&shared, dev, &mut dma, &mut found);
                    assert!(found.failures.is_empty(), "{:?}", found.failures);
                    assert_eq!(model.borrow().configuration_seen_before_endpoints, [9]);
                    if scenario >= 4 {
                        assert!(found.vendor_links.is_empty());
                        assert_eq!(found.vendor_failures.len(), 1);
                        let ethernet::Error::Device(error) = &found.vendor_failures[0].1 else {
                            panic!("{:?}", found.vendor_failures);
                        };
                        assert_eq!(
                            *error,
                            BlockError::Unsupported(if scenario == 4 {
                                "short USB control OUT transfer"
                            } else {
                                "AX88772 external/unknown PHY requires its own initialization"
                            })
                        );
                        let hardware = model.borrow();
                        let trace = &hardware.asix_legacy.as_ref().unwrap().trace;
                        assert!(!trace.is_empty());
                        if scenario == 5 {
                            assert!(
                                trace
                                    .iter()
                                    .any(|(request, _)| *request == setup(0xc0, 0x19, 0, 0, 2))
                            );
                        }
                        assert!(hardware.active[in_dci].is_none());
                        continue;
                    }
                    assert!(
                        found.vendor_failures.is_empty(),
                        "{:?} identity {vid:04x}:{pid:04x} code {code} scenario {scenario}; legacy trace {:?}",
                        found.vendor_failures,
                        model.borrow().asix_legacy.as_ref().unwrap().trace
                    );
                    let mut link = found.vendor_links.pop().unwrap();
                    assert_eq!(link.driver(), "usb-asix");
                    assert_eq!(link.mac().0, [2, 0x27, 0x72, 0x43, 0x54, 0x65]);
                    // Each check's waits advance the autonegotiation clock.
                    for _ in 0..4 {
                        if link.checked_carrier() {
                            break;
                        }
                    }
                    if !link.settled_carrier() {
                        let result = link.receive();
                        panic!(
                            "code {code} scenario {scenario} phy {:?} RX {:?}",
                            model.borrow().asix_legacy.as_ref().unwrap().phy,
                            result
                        );
                    }
                    let length = 64usize.next_multiple_of(packet as usize) - 4;
                    for _ in 0..if vid == 0x0b95 && pid == 0x7720 && scenario == 0 {
                        300
                    } else {
                        1
                    } {
                        // A carrier check that comes due holds the chip
                        // for a few calls; the frame is busy until it ends.
                        let frame = vec![0x5a; length];
                        while let Err(error) = link.transmit(&frame) {
                            assert_eq!(error, LinkError::Busy);
                        }
                    }
                    let sent = model.borrow().outgoing.last().unwrap().clone();
                    let n = length.max(60) as u32;
                    assert_eq!(&sent[..4], &(n | ((!n & 0xffff) << 16)).to_le_bytes());
                    assert_eq!(&sent[4..4 + length], &vec![0x5a; length]);
                    assert_eq!(&sent[sent.len() - 4..], &0xffff0000u32.to_le_bytes());
                    assert_eq!(link.receive().unwrap(), None);
                    let mut bytes = (1514u32 | ((!1514u32 & 0xffff) << 16))
                        .to_le_bytes()
                        .to_vec();
                    bytes.extend_from_slice(&vec![0x73; 1514]);
                    model.borrow_mut().push_event(bytes[..62].to_vec());
                    assert_eq!(link.receive().unwrap(), None);
                    model.borrow_mut().push_event(bytes[62..].to_vec());
                    assert_eq!(link.receive().unwrap(), Some(vec![0x73; 1514]));
                    assert_eq!(link.receive().unwrap(), None);
                    if scenario == 0 {
                        // Three distinct bulk-IN DMA completions: a
                        // low header half must survive a second
                        // two-byte transfer before the payload arrives.
                        let header = (60u32 | (0xffc3 << 16)).to_le_bytes();
                        for chunk in [&header[..2], &header[2..], &[0x53; 60][..]] {
                            model.borrow_mut().push_event(chunk.to_vec());
                            let frame = link.receive().unwrap();
                            if chunk.len() == 60 {
                                assert_eq!(frame, Some(vec![0x53; 60]));
                            } else {
                                assert_eq!(frame, None);
                            }
                        }
                        assert_eq!(link.receive().unwrap(), None);
                    }
                    match scenario {
                        0 => {
                            model.borrow_mut().asix_legacy.as_mut().unwrap().link = false;
                            assert!(!link.checked_carrier());
                            assert!(model.borrow().active[in_dci].is_none());
                            model.borrow_mut().asix_legacy.as_mut().unwrap().link = true;
                            for _ in 0..4 {
                                if link.checked_carrier() {
                                    break;
                                }
                            }
                            if !link.settled_carrier() {
                                let result = link.receive();
                                panic!(
                                    "code {code} scenario {scenario} phy {:?} RX {:?}",
                                    model.borrow().asix_legacy.as_ref().unwrap().phy,
                                    result
                                );
                            }
                            link.stop().unwrap();
                            assert_eq!(model.borrow().asix_legacy.as_ref().unwrap().medium, 0);
                            assert!(link.transmit(&[0; 60]).is_err());
                        }
                        1 => {
                            model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
                            assert!(link.stop().is_err());
                            assert!(model.borrow().active[in_dci].is_some());
                            assert!(link.receive().is_err());
                        }
                        2 => {
                            model.borrow_mut().connected = false;
                            assert!(!link.carrier());
                            assert!(link.receive().is_err());
                            assert!(model.borrow().active[in_dci].is_some());
                        }
                        _ => {
                            model.borrow_mut().halt_transfer = Some(out_dci);
                            model.borrow_mut().fail = Some((SET_DEQUEUE, CONTEXT_STATE_ERROR));
                            assert!(link.transmit(&[0x77; 60]).is_err());
                            let hw = model.borrow();
                            let at = hw.rings[out_dci] + (hw.indices[out_dci] * TRB) as u64;
                            // SAFETY: halted TD and DMA buffer remain live.
                            let (buffer, n) = unsafe {
                                (
                                    (at as *const u64).read_volatile(),
                                    ((at + 8) as *const u32).read_volatile() as usize & 0x1ffff,
                                )
                            };
                            drop(hw);
                            // SAFETY: model performs no DMA; failed cancellation keeps ownership.
                            let before = unsafe {
                                core::slice::from_raw_parts(buffer as *const u8, n).to_vec()
                            };
                            assert!(link.transmit(&[0x88; 60]).is_err());
                            assert_eq!(
                                // SAFETY: the pointer addresses this test's own live pages, and the
                                // stopped model writes nothing concurrently.
                                unsafe { core::slice::from_raw_parts(buffer as *const u8, n) },
                                before
                            );
                        }
                    }
                }
            }
        }
    }
    assert_eq!(cases, 1260);
}

#[test]
fn ax179_aliases_use_shared_dma_and_quarantine() {
    use crate::net::Link;
    for (vid, pid) in ethernet::asix_ids() {
        for (speed, packet) in [
            (FULL_SPEED, 8u16),
            (FULL_SPEED, 16),
            (FULL_SPEED, 32),
            (FULL_SPEED, 64),
            (HIGH_SPEED, 512),
            (SUPER_SPEED, 1024),
            (SUPER_SPEED_PLUS, 1024),
        ] {
            for scenario in 0..6 {
                let mut dma = TestRegion::new(128 * 1024 * 8);
                let (mut controller, model) =
                    unconfigured_cancellation_controller_with_configurations(
                        &mut dma,
                        &[vec![9, 2, 9, 0, 0, 7, 0, 0x80, 50]],
                    );
                controller.set_slot_speed(1, speed);
                model.borrow_mut().asix = Some(ethernet::AsixModel::new(
                    if speed >= SUPER_SPEED {
                        4
                    } else if speed == HIGH_SPEED {
                        2
                    } else {
                        0
                    },
                    pid & 1 != 0,
                ));
                let (input, output) = match scenario % 3 {
                    0 => (0x82u8, 3u8),
                    1 => (0x81, 2),
                    _ => (0x84, 5),
                };
                let in_dci = (2 * (input & 15) + 1) as usize;
                let out_dci = (2 * output) as usize;
                let irq = if input == 0x81 { 0x83 } else { 0x81 };
                let alternate = if scenario == 0 || scenario == 5 { 2 } else { 0 };
                let mut config = vec![9, 2, 0, 0, 1, 7, 0, 0x80, 50];
                if alternate != 0 {
                    config.extend_from_slice(&[9, 4, 0, 0, 0, 0xff, 0xff, 0, 0]);
                }
                config.extend_from_slice(&[9, 4, 0, alternate, 3, 0xff, 0xff, 0, 0]);
                for address in [input, output] {
                    config.extend_from_slice(&[
                        7,
                        5,
                        address,
                        2,
                        packet as u8,
                        (packet >> 8) as u8,
                        0,
                    ]);
                    if speed >= SUPER_SPEED {
                        config.extend_from_slice(&[6, SS_ENDPOINT_COMPANION, 3, 0, 0, 0]);
                    }
                }
                config.extend_from_slice(&[7, 5, irq, 3, 8, 0, 4]);
                if speed >= SUPER_SPEED {
                    config.extend_from_slice(&[6, SS_ENDPOINT_COMPANION, 0, 0, 8, 0]);
                }
                let total = config.len() as u16;
                config[2..4].copy_from_slice(&total.to_le_bytes());
                let mut dev = model::addressed_device(&controller, vec![config]);
                dev.speed = speed;
                dev.descriptor[8..10].copy_from_slice(&vid.to_le_bytes());
                dev.descriptor[10..12].copy_from_slice(&pid.to_le_bytes());
                model::select(&mut dev).unwrap();
                if scenario == 5 {
                    controller.set_configuration(1, 7).unwrap();
                    model.borrow_mut().halt_transfer = Some(1);
                }
                let shared = Rc::new(RefCell::new(controller));
                if scenario == 4 {
                    model.borrow_mut().short_control_out = true;
                }
                let mut found = Found::default();
                bind(&shared, dev, &mut dma, &mut found);
                assert!(found.failures.is_empty(), "{:?}", found.failures);
                assert_eq!(model.borrow().configuration_seen_before_endpoints, [7]);
                if scenario == 4 || scenario == 5 {
                    assert!(found.vendor_links.is_empty());
                    assert_eq!(found.vendor_failures.len(), 1);
                    let ethernet::Error::Device(error) = &found.vendor_failures[0].1 else {
                        panic!("{:?}", found.vendor_failures);
                    };
                    assert_eq!(
                        *error,
                        if scenario == 5 {
                            BlockError::Device { status: STALL }
                        } else {
                            BlockError::Unsupported("short USB control OUT transfer")
                        }
                    );
                    assert_eq!(
                        model.borrow().asix.as_ref().unwrap().trace.len(),
                        if scenario == 5 { 0 } else { 1 }
                    );
                    assert!(model.borrow().active[in_dci].is_none());
                    continue;
                }
                assert!(
                    found.vendor_failures.is_empty(),
                    "{:?}",
                    found.vendor_failures
                );
                assert_eq!(found.vendor_links.len(), 1);
                assert_eq!(model.borrow().rings[(2 * (irq & 15) + 1) as usize], 0);
                let mut link = found.vendor_links.pop().unwrap();
                assert_eq!(link.driver(), "usb-asix");
                assert_eq!(link.mac().0, [2, 0x34, 0x56, 0x78, 0x9a, 0xbc]);
                assert!(link.firmware.is_none());
                if alternate != 0 {
                    let hw = model.borrow();
                    let setting = hw
                        .setup
                        .iter()
                        .position(|(request, _)| request[0] == 1 && request[1] == SET_INTERFACE)
                        .unwrap();
                    let first_vendor = hw
                        .setup
                        .iter()
                        .position(|(request, _)| request[0] == 0x40)
                        .unwrap();
                    assert!(setting < first_vendor);
                    assert_eq!(hw.setup[setting].0, setup(1, SET_INTERFACE, 2, 0, 0));
                }
                assert!(link.settled_carrier());
                let length = 68usize.next_multiple_of(packet as usize) - 8;
                let frames = if vid == 0x0b95 && pid == 0x1790 && scenario == 0 {
                    300
                } else {
                    1
                };
                for _ in 0..frames {
                    link.transmit(&vec![0x5a; length]).unwrap();
                }
                let sent = model.borrow().outgoing.last().unwrap().clone();
                assert_eq!(
                    u32::from_le_bytes(sent[..4].try_into().unwrap()),
                    length as u32
                );
                assert_eq!(
                    u32::from_le_bytes(sent[4..8].try_into().unwrap()),
                    0x80008000
                );
                assert_eq!(&sent[8..], vec![0x5a; length]);
                assert_eq!(link.receive().unwrap(), None);
                let mut aggregate = Vec::new();
                let mut headers = Vec::new();
                for value in 0..12 {
                    aggregate.extend_from_slice(&[0; 2]);
                    aggregate.extend_from_slice(&vec![value; 1514]);
                    aggregate.resize(aggregate.len().next_multiple_of(8), 0);
                    headers.extend_from_slice(&(1516u32 << 16).to_le_bytes());
                    headers.extend_from_slice(&0x80000000u32.to_le_bytes());
                }
                let offset = aggregate.len();
                aggregate.extend_from_slice(&headers);
                aggregate.extend_from_slice(&[0; 4]);
                aggregate.extend_from_slice(&((offset as u32) << 16 | 24).to_le_bytes());
                model.borrow_mut().push_event(aggregate);
                for value in 0..12 {
                    assert_eq!(link.receive().unwrap(), Some(vec![value; 1514]));
                }
                assert_eq!(link.receive().unwrap(), None);
                match scenario {
                    0 => {
                        model
                            .borrow_mut()
                            .asix
                            .as_mut()
                            .unwrap()
                            .phy
                            .insert((0, 0, 0x11), 0);
                        assert!(!link.checked_carrier());
                        assert!(model.borrow().active[in_dci].is_none());
                        model
                            .borrow_mut()
                            .asix
                            .as_mut()
                            .unwrap()
                            .phy
                            .insert((0, 0, 0x11), 0xa400);
                        assert!(link.checked_carrier());
                        link.stop().unwrap();
                        assert!(link.transmit(&[0; 60]).is_err());
                    }
                    1 => {
                        model.borrow_mut().fail = Some((STOP_ENDPOINT, CONTEXT_STATE_ERROR));
                        assert!(link.stop().is_err());
                        assert!(model.borrow().active[in_dci].is_some());
                        assert!(link.receive().is_err() && link.transmit(&[0; 60]).is_err());
                    }
                    2 => {
                        model.borrow_mut().connected = false;
                        assert!(!link.carrier());
                        assert!(link.receive().is_err() && link.transmit(&[0; 60]).is_err());
                        assert!(model.borrow().active[in_dci].is_some());
                    }
                    _ => {
                        model.borrow_mut().halt_transfer = Some(out_dci);
                        model.borrow_mut().fail = Some((SET_DEQUEUE, CONTEXT_STATE_ERROR));
                        assert!(link.transmit(&[0x77; 60]).is_err());
                        let hw = model.borrow();
                        let at = hw.rings[out_dci] + (hw.indices[out_dci] * TRB) as u64;
                        // SAFETY: halted TD and its DMA buffer remain live.
                        let (buffer, length) = unsafe {
                            (
                                (at as *const u64).read_volatile(),
                                ((at + 8) as *const u32).read_volatile() as usize & 0x1ffff,
                            )
                        };
                        drop(hw);
                        // SAFETY: the hardware still owns this test sentinel.
                        let before = unsafe {
                            core::slice::from_raw_parts(buffer as *const u8, length).to_vec()
                        };
                        assert!(link.transmit(&[0x88; 60]).is_err());
                        assert_eq!(
                            // SAFETY: the pointer addresses this test's own live pages, and the
                            // stopped model writes nothing concurrently.
                            unsafe { core::slice::from_raw_parts(buffer as *const u8, length) },
                            before
                        );
                    }
                }
            }
        }
    }
}

/// An ASIX link through carrier, a frame each way, a lost link and its
/// return, its calls bounded by [`ethernet::tests::stepped::Calls`]; what
/// the device saw, apart from the control writes `ignored` keeps out.
fn asix_link_through_loss(
    late: bool,
    legacy: bool,
    ignored: impl Fn(&[u8; 8]) -> bool,
) -> ethernet::tests::stepped::Effects {
    use crate::net::{Link, LinkError};
    use ethernet::tests::stepped::{Calls, effects};
    let mut dma = TestRegion::new(128 * 1024 * 8);
    let value = if legacy { 9 } else { 7 };
    let (mut controller, model) = unconfigured_cancellation_controller_with_configurations(
        &mut dma,
        &[vec![9, 2, 9, 0, 0, value, 0, 0x80, 50]],
    );
    controller.set_slot_speed(1, HIGH_SPEED);
    model::minimum_on_controller_clock(&mut controller);
    let mut config = vec![9, 2, 0, 0, 1, value, 0, 0x80, 50];
    if legacy {
        model.borrow_mut().asix_legacy = Some(ethernet::LegacyModel::new(0x10, 0x003b1861));
        config.extend_from_slice(&[9, 4, 0, 0, 2, 0xff, 0, 0, 0]);
    } else {
        model.borrow_mut().asix = Some(ethernet::AsixModel::new(2, false));
        config.extend_from_slice(&[9, 4, 0, 0, 3, 0xff, 0xff, 0, 0]);
    }
    for address in [0x81, 0x02] {
        config.extend_from_slice(&[7, 5, address, 2, 0, 2, 0]);
    }
    if !legacy {
        config.extend_from_slice(&[7, 5, 0x83, 3, 8, 0, 4]);
    }
    let total = config.len() as u16;
    config[2..4].copy_from_slice(&total.to_le_bytes());
    let mut device = model::addressed_device(&controller, vec![config]);
    device.speed = HIGH_SPEED;
    let product: u16 = if legacy { 0x7720 } else { 0x1790 };
    device.descriptor[8..10].copy_from_slice(&0x0b95u16.to_le_bytes());
    device.descriptor[10..12].copy_from_slice(&product.to_le_bytes());
    model::select(&mut device).unwrap();
    let shared = Rc::new(RefCell::new(controller));
    let mut found = Found::default();
    bind(&shared, device, &mut dma, &mut found);
    assert!(
        found.vendor_failures.is_empty(),
        "{:?}",
        found.vendor_failures
    );
    let mut link = found.vendor_links.pop().unwrap();
    let clock = shared.borrow().clock;
    let mut calls = Calls::new(model.clone(), clock, late);
    let not_busy = |result: &Result<(), LinkError>| result != &Err(LinkError::Busy);
    let set_link = |up: bool| {
        let mut model = model.borrow_mut();
        match &mut model.asix_legacy {
            Some(legacy) => legacy.link = up,
            None => {
                let status = if up { 0xa400 } else { 0 };
                model
                    .asix
                    .as_mut()
                    .unwrap()
                    .phy
                    .insert((0, 0, 0x11), status);
            }
        }
    };

    calls.until(|| link.carrier(), |up| *up);
    calls
        .until(|| link.transmit(&[0x5a; 60]), not_busy)
        .unwrap();
    calls
        .until(|| link.transmit(&[0x6a; 60]), not_busy)
        .unwrap();
    calls.until(
        || link.receive().unwrap(),
        |_| model.borrow().active[3].is_some(),
    );
    let bytes = if legacy {
        let mut bytes = (60u32 | (0xffc3 << 16)).to_le_bytes().to_vec();
        bytes.extend_from_slice(&[0x7a; 60]);
        bytes
    } else {
        let mut bytes = vec![0; 2];
        bytes.extend_from_slice(&[0x7a; 60]);
        bytes.resize(64, 0);
        bytes.extend_from_slice(&(62u32 << 16).to_le_bytes());
        bytes.extend_from_slice(&0x80000000u32.to_le_bytes());
        bytes.extend_from_slice(&[0; 4]);
        bytes.extend_from_slice(&(64u32 << 16 | 2).to_le_bytes());
        bytes
    };
    model.borrow_mut().push_event(bytes);
    let frame = calls.until(|| link.receive().unwrap(), Option::is_some);
    assert_eq!(frame, Some(vec![0x7a; 60]));

    // The lost link: both bulk endpoints cancelled, then the MAC stopped;
    // the AX88772A's PHY reset with its twelve 50 ms waits runs in steps.
    set_link(false);
    calls.wait(ethernet::LINK_POLL_US);
    calls.until(
        || (link.carrier(), link.running.is_some()),
        |(up, busy)| !up && !busy,
    );
    assert!(model.borrow().active[3].is_none());
    set_link(true);
    calls.wait(ethernet::LINK_POLL_US);
    calls.until(|| link.carrier(), |up| *up);
    calls
        .until(|| link.transmit(&[0x8a; 60]), not_busy)
        .unwrap();
    calls
        .until(|| link.transmit(&[0x9a; 60]), not_busy)
        .unwrap();
    assert_eq!(calls.answered_late > 0, late);
    let seen = effects(&model.borrow(), ignored);
    calls.answer_at_once();
    seen
}

/// A device that answers each control transfer and command only after the
/// link call that asked returned: the AX88179's carrier check and medium
/// set up with its FIFO wait, transmit, receive and the lost link's quiesce
/// and MAC stop run in steps, no call holding the processor beyond its
/// bound, and the device sees what one that answers at once sees.
#[test]
fn ax88179_link_calls_run_in_steps_when_the_device_answers_late() {
    let everything = |_: &[u8; 8]| false;
    assert_eq!(
        asix_link_through_loss(true, false, everything),
        asix_link_through_loss(false, false, everything)
    );
}

/// The AX88772A alike, its lost link also resetting the PHY. Each carrier
/// check takes and returns MII ownership, and a device that answers late
/// is checked more often, so those writes are left out.
#[test]
fn ax88772_link_calls_run_in_steps_when_the_device_answers_late() {
    let ownership = |request: &[u8; 8]| request[0] == 0x40 && matches!(request[1], 0x06 | 0x0a);
    assert_eq!(
        asix_link_through_loss(true, true, ownership),
        asix_link_through_loss(false, true, ownership)
    );
}
