use super::*;
use crate::usb::ethernet::{
    firmware::tests::synthetic_upload,
    rtl815x::{FirmwareIo, FirmwareModel},
};
use alloc::rc::Rc;
use core::cell::RefCell;
use stela_driver_base::stepped::block_on;

fn seed(model: &mut FirmwareModel, mdi: bool) {
    model
        .core
        .set(USB, USB_UPS_CFG, 2, if mdi { MID_REVERSE } else { 0 });
    model.core.set(PLA, 0xd068, 2, if mdi { 2 } else { 0 });
    model.mdi = [0x8111, 0x9222, 0xa333, 0xc444];
    model.core.set(PLA, 0xbd5a, 2, 0x1122);
    model.core.set(PLA, 0xbd5c, 2, 0x3344);
    model.core.set(PLA, 0xbc18, 2, 0x5566);
    model.core.set(PLA, 0xbc1a, 2, 0x7788);
    model.core.set(PLA, PLA_RCR1, 2, 0xa5ff);
    model.core.set(PLA, PLA_CPCR, 2, CPCR_RX_VLAN);
    model.core.set(PLA, OCP_EEE_ADV2, 2, 0xffff);
    model.core.set(PLA, OCP_10GBT_CTRL, 2, 0xffff);
    model.core.set(PLA, PLA_RXFIFO_CTRL0, 4, 0xaaaa_5555);
    model.core.set(PLA, PLA_TXFIFO_CTRL + 4, 4, 0x1234_abcd);
}

#[test]
fn rtl8156a_exact_revision_phy_sram2_mdi_and_48k_receive() {
    for revision in [0x7020, 0x7030] {
        let bytes = if revision == 0x7030 {
            synthetic_upload(revision)
        } else {
            Vec::new()
        };
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        for speed in [1, 3, 4, 5] {
            for mdi in [false, true] {
                let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
                seed(&mut model.borrow_mut(), mdi);
                let (mut chip, _) = block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    Revision(revision),
                    speed,
                    firmware.as_ref(),
                ))
                .unwrap();
                assert!(chip.initialized);
                assert_eq!(chip.receive_bytes(), 49152);
                {
                    let hw = model.borrow();
                    assert_eq!(
                        hw.core.get(PLA, OCP_10GBT_CTRL, 2) & MDIO_ADV_MULTIGIG_MASK,
                        if speed >= SUPER_SPEED {
                            MDIO_ADV_2500
                        } else {
                            0
                        }
                    );
                    assert_eq!(hw.core.get(PLA, OCP_EEE_ADV2, 2), 0xfffc);
                    assert_eq!(hw.core.get(PLA, PLA_RCR1, 2), 0xa53f);
                    assert_eq!(hw.core.get(PLA, PLA_CPCR, 2) & CPCR_RX_VLAN, CPCR_RX_VLAN);
                    assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL0, 4), 0xa008_5555);
                    assert_eq!(hw.core.get(PLA, PLA_TXFIFO_CTRL, 4), 0x0080_0008);
                    assert_eq!(hw.core.get(PLA, PLA_TXFIFO_CTRL + 4, 4), 0x1234_abcd);
                    assert_eq!(hw.core.get(PLA, PLA_RX_FIFO_FULL, 2), 512);
                    assert_eq!(hw.core.get(PLA, PLA_RX_FIFO_EMPTY, 2), 1024);
                    assert_eq!(hw.core.get(USB, USB_RX_BUF_TH, 4), 0x0060_0400);
                    if revision == 0x7020 {
                        assert_eq!(hw.sram[&0x819f], 0xddb6);
                        assert_eq!(hw.core.get(PLA, 0xad16, 2) & 0x3ff, 6);
                    } else {
                        assert_eq!(hw.sram2[&0x80a2], 0x153);
                        assert_eq!(hw.sram2[&0x809c], 0x153);
                        assert_eq!(hw.sram2[&0x8157] & 0xff00, 0x500);
                        assert_eq!(hw.core.get(PLA, OCP_SYSCLK_CFG, 2), 5 << 8);
                        if mdi {
                            assert_eq!(hw.mdi, [0x8444, 0x9333, 0xa222, 0xc111]);
                            assert_eq!(hw.core.get(PLA, 0xbd5a, 2), 0x0433);
                            assert_eq!(hw.core.get(PLA, 0xbd5c, 2), 0x2251);
                            assert_eq!(hw.core.get(PLA, 0xbc18, 2), 0x4877);
                            assert_eq!(hw.core.get(PLA, 0xbc1a, 2), 0x6695);
                        }
                    }
                }
                for status in [0x06, 0x07, 0x0a, 0x0b, 0x13, 0x113, 0x403, 0x473] {
                    model.borrow_mut().core.phy_status = status;
                    let before = model.borrow().core.trace.len();
                    assert!(block_on(chip.carrier()).unwrap());
                    {
                        let hw = model.borrow();
                        let writes = &hw.core.trace[before..];
                        let flow = writes
                            .iter()
                            .position(|(request, _)| {
                                request[0] == 0x40
                                    && u16::from_le_bytes([request[2], request[3]])
                                        == (PLA_RX_FIFO_FULL as u16 & !3)
                            })
                            .unwrap();
                        let eee = writes
                            .iter()
                            .position(|(request, _)| {
                                request[0] == 0x40
                                    && u16::from_le_bytes([request[2], request[3]])
                                        == (PLA_EEEP_CR as u16 & !3)
                            })
                            .unwrap();
                        assert!(flow < eee); // rtl8156_enable requires pause setup before EEE+.
                        assert_eq!(hw.core.get(USB, USB_RX_EARLY_TIMEOUT, 2), 80);
                        assert_eq!(hw.core.get(USB, USB_RX_EXTRA_AGGR_TMR, 2), 1875);
                        assert_eq!(hw.core.get(USB, USB_RX_EARLY_SIZE, 2), 5949);
                        assert_eq!(
                            hw.core.get(PLA, PLA_MAC_PWR_CTRL4, 2) & IDLE_SPDWN_EN,
                            if status & PHYSTATUS_2500 != 0 {
                                0
                            } else {
                                IDLE_SPDWN_EN
                            }
                        );
                        if speed == 3 {
                            assert_eq!(
                                hw.core.get(USB, USB_L1_CTRL, 2) & 15,
                                if status & PHYSTATUS_FLOW == PHYSTATUS_FLOW {
                                    15
                                } else {
                                    1
                                }
                            );
                        }
                    }
                    block_on(chip.stop()).unwrap();
                    assert_eq!(model.borrow().core.get(PLA, PLA_RX_FIFO_FULL, 2), 0);
                    assert_eq!(model.borrow().core.get(PLA, PLA_RX_FIFO_EMPTY, 2), 0);
                }
            }
        }
    }
}

#[test]
fn rtl8156a_observed_phy_upload_modes_and_ups_reload_are_distinct() {
    let bytes = synthetic_upload(0x7030);
    let firmware = Firmware::parse(&bytes, Revision(0x7030)).unwrap();
    for state in [PHY_STAT_LAN_ON, PHY_STAT_PWRDN, PHY_STAT_EXT_INIT] {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x7030, &bytes)));
        model.borrow_mut().phy_states.extend([
            PHY_STAT_LAN_ON,
            PHY_STAT_LAN_ON,
            state,
            PHY_STAT_LAN_ON,
        ]);
        let (chip, _) = block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(0x7030),
            4,
            Some(&firmware),
        ))
        .unwrap();
        assert!(chip.initialized);
        assert!(model.borrow().phy_states.is_empty());
        assert_eq!(
            model.borrow().core.get(PLA, 0xa468, 2) & ((1 << 3) | (1 << 1)),
            0
        );
    }
    let model = Rc::new(RefCell::new(FirmwareModel::new(0x7020, &[])));
    model.borrow_mut().ups_exit_power_cut = true;
    let (chip, _) = block_on(Chip::attach_with_firmware(
        FirmwareIo(model.clone()),
        Revision(0x7020),
        4,
        None,
    ))
    .unwrap();
    assert!(chip.initialized);
    assert!(!model.borrow().ups_exit_power_cut);
    let resets = model
        .borrow()
        .core
        .trace
        .iter()
        .filter(|(request, bytes)| {
            request[0] == 0x40
                && request[2..4] == [0, 0xb4]
                && bytes.len() == 4
                && u16::from_le_bytes([bytes[0], bytes[1]]) & BMCR_RESET as u16 != 0
        })
        .count();
    assert_eq!(resets, 2);
}

#[test]
fn rtl8156a_every_mdi_startup_transfer_failure_and_deadline_refuses_ready() {
    let bytes = synthetic_upload(0x7030);
    let firmware = Firmware::parse(&bytes, Revision(0x7030)).unwrap();
    let model = Rc::new(RefCell::new(FirmwareModel::new(0x7030, &bytes)));
    seed(&mut model.borrow_mut(), true);
    block_on(Chip::attach_with_firmware(
        FirmwareIo(model.clone()),
        Revision(0x7030),
        4,
        Some(&firmware),
    ))
    .unwrap();
    let count = model.borrow().core.trace.len();
    for at in 0..count {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x7030, &bytes)));
        seed(&mut model.borrow_mut(), true);
        model.borrow_mut().core.fail_at = Some(at);
        assert!(
            block_on(Chip::attach_with_firmware(
                FirmwareIo(model.clone()),
                Revision(0x7030),
                4,
                Some(&firmware)
            ))
            .is_err()
        );
        assert_eq!(model.borrow().core.trace.len(), at + 1);
    }
    for case in 0..6 {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x7030, &bytes)));
        {
            let mut hw = model.borrow_mut();
            match case {
                0 => hw.core.set(PLA, PLA_BOOT_CTRL, 2, 0),
                1 => hw.core.set(PLA, OCP_PHY_STATUS, 2, 1),
                2 => hw.core.set(PLA, 0xe000, 2, 0),
                3 => hw.core.stuck_phy = true,
                4 => hw.core.stuck_reset = true,
                _ => hw.stuck_patch = true,
            }
        }
        assert!(matches!(
            block_on(Chip::attach_with_firmware(
                FirmwareIo(model),
                Revision(0x7030),
                4,
                Some(&firmware)
            )),
            Err(BlockError::Timeout)
        ));
    }
}
#[test]
#[ignore = "oracle: STELA_RTL8156A_ENABLE_ORACLE"]
fn unchanged_linux_rtl8156a_enable_register_effects() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let trace =
        std::fs::read_to_string(std::env::var("STELA_RTL8156A_ENABLE_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
        "28c79c6b9b29e32a608a02816ce818bded7acfc95746039c9421ca89eb9489a9"
    );
    let fields = [
        (USB, USB_RX_EARLY_TIMEOUT),
        (USB, USB_RX_EXTRA_AGGR_TMR),
        (USB, USB_RX_EARLY_SIZE),
        (PLA, PLA_TCR1),
        (PLA, PLA_MAC_PWR_CTRL4),
        (USB, USB_L1_CTRL),
        (PLA, PLA_EEE_TXTWSYS),
        (PLA, PLA_RX_FIFO_FULL),
        (PLA, PLA_RX_FIFO_EMPTY),
        (USB, USB_FW_TASK),
        (PLA, PLA_EEEP_CR),
    ];
    let mut count = 0;
    for line in trace.lines() {
        let v: Vec<_> = line.split_whitespace().collect();
        assert_eq!(v.len(), 14);
        let revision = u16::from_str_radix(v[0], 16).unwrap();
        let speed = v[1].parse::<u8>().unwrap();
        let status = v[2].parse::<u32>().unwrap();
        let bytes = if revision == 0x7030 {
            synthetic_upload(revision)
        } else {
            Vec::new()
        };
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
        let (mut chip, _) = block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(revision),
            speed,
            firmware.as_ref(),
        ))
        .unwrap();
        for (bank, address) in fields {
            model.borrow_mut().core.set(bank, address, 2, 0xa55a);
        }
        model.borrow_mut().core.phy_status = status;
        assert!(block_on(chip.carrier()).unwrap());
        for ((bank, address), expected) in fields.into_iter().zip(&v[3..]) {
            assert_eq!(
                model.borrow().core.get(bank, address, 2),
                expected.parse::<u32>().unwrap(),
                "{line}"
            );
        }
        count += 1;
    }
    assert_eq!(count, 64);
}

#[test]
#[ignore = "oracle: STELA_RTL815X_FIRMWARE"]
fn real_firmware_8156a_full_startup_link_and_stop() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let (name, revision, _, hash) = crate::usb::ethernet::firmware::tests::FIXTURES
        .iter()
        .copied()
        .find(|(_, revision, _, _)| *revision == 0x7030)
        .unwrap();
    let directory = std::env::var("STELA_RTL815X_FIRMWARE").unwrap();
    let bytes = std::fs::read(std::path::Path::new(&directory).join(name)).unwrap();
    assert_eq!(alloc::format!("{:x}", Sha256::digest(&bytes)), hash);
    let firmware = Firmware::parse(&bytes, Revision(revision)).unwrap();
    for speed in [1, 3, 4, 5] {
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
        seed(&mut model.borrow_mut(), true);
        let (mut chip, _) = block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(revision),
            speed,
            Some(&firmware),
        ))
        .unwrap();
        assert!(chip.initialized && block_on(chip.carrier()).unwrap());
        block_on(chip.stop()).unwrap();
        assert_eq!(model.borrow().core.get(PLA, PLA_CR, 1), 0);
    }
}
