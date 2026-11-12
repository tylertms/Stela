use super::*;
use crate::usb::ethernet::{
    firmware::tests::synthetic_upload,
    rtl815x::{FirmwareIo, FirmwareModel},
};
use alloc::rc::Rc;
use core::cell::RefCell;
use stela_driver_base::stepped::block_on;

#[test]
fn rtl8153b_calibration_power_phy_and_receive_paths() {
    for revision in [0x6000, 0x6010] {
        let bytes = if revision == 0x6010 {
            synthetic_upload(revision)
        } else {
            Vec::new()
        };
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        for speed in [1, 3, 4, 5] {
            for (fuse, counter, test_io) in
                [(0x1ffff, 0, false), (0x12345, 2000, true), (0, 1, false)]
            {
                let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
                {
                    let mut hw = model.borrow_mut();
                    hw.efuse = fuse;
                    hw.core.set(PLA, OCP_ADC_IOFFSET, 2, 0xa55a);
                    hw.core.set(PLA, 0xc426, 2, counter);
                    hw.core.set(
                        USB,
                        USB_POWER_CUT,
                        2,
                        PWR_EN | PHASE2_EN | UPS_EN | USP_PREWAKE,
                    );
                    hw.core.set(USB, USB_UPS_CFG, 2, 0xa123);
                    hw.core
                        .set(PLA, 0xdc00, 1, if test_io { 1 << 5 } else { 0 });
                }
                let (mut chip, _) = block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    Revision(revision),
                    speed,
                    firmware.as_ref(),
                ))
                .unwrap();
                assert!(chip.initialized);
                assert_eq!(chip.receive_bytes(), 32768);
                {
                    let hw = model.borrow();
                    assert_eq!(hw.core.get(USB, USB_POWER_CUT, 2), PHASE2_EN);
                    assert_eq!(
                        hw.core.get(USB, USB_LPM_CONFIG, 2) & LPM_U1U2_EN,
                        if speed >= SUPER_SPEED { LPM_U1U2_EN } else { 0 }
                    );
                    assert_eq!(
                        hw.core.get(PLA, OCP_ADC_IOFFSET, 2),
                        if fuse == 0x1ffff {
                            0xa55a
                        } else {
                            ((fuse & 0x1fff0) >> 1) | (fuse & 7)
                        }
                    );
                    assert_eq!(
                        hw.core.get(USB, USB_UPS_CFG, 2),
                        match counter {
                            0 => 0xa123,
                            2000 => 0xaf40,
                            1 => 0xa400,
                            _ => unreachable!(),
                        }
                    );
                    assert_eq!(
                        hw.core.get(PLA, PLA_PHY_PWR, 2) & TEST_IO_OFF,
                        if revision == 0x6010 && test_io {
                            TEST_IO_OFF
                        } else {
                            0
                        }
                    );
                    assert_eq!(hw.sram[&0x8045], 0);
                    assert_eq!(hw.sram[&0x804d], 0x1222);
                    assert_eq!(hw.sram[&0x805d], 0x22);
                    assert_eq!(
                        hw.sram[&(SRAM_GREEN_CFG as u16)] & (GREEN_ETH_EN | R_TUNE_EN) as u16,
                        (GREEN_ETH_EN | R_TUNE_EN) as u16
                    );
                    assert_eq!(
                        hw.core.get(PLA, OCP_NCTL_CFG, 2) & PGA_RETURN_EN,
                        PGA_RETURN_EN
                    );
                    assert_eq!(hw.core.get(PLA, OCP_SYSCLK_CFG, 2), 5 << 4);
                    assert_eq!(hw.core.get(USB, USB_RX_BUF_TH, 4), RX_THR_B);
                    assert_eq!(hw.core.get(PLA, OCP_PHY_PATCH_CMD, 2) & PATCH_REQUEST, 0);
                }
                assert!(block_on(chip.carrier()).unwrap());
                {
                    let hw = model.borrow();
                    assert_eq!(hw.core.get(USB, USB_RX_EARLY_TIMEOUT, 2), 16);
                    assert_eq!(hw.core.get(USB, USB_RX_EXTRA_AGGR_TMR, 2), 1875);
                    assert_eq!(hw.core.get(USB, USB_RX_EARLY_SIZE, 2), (32768 - 1554) / 8);
                    assert_eq!(
                        hw.core.get(USB, USB_UPT_RXDMA_OWN, 1),
                        OWN_CLEAR | OWN_UPDATE
                    );
                    assert_eq!(
                        hw.core.get(USB, USB_FW_TASK, 2) & FC_PATCH_TASK,
                        if revision == 0x6010 { FC_PATCH_TASK } else { 0 }
                    );
                }
                block_on(chip.stop()).unwrap();
                assert_eq!(model.borrow().core.get(PLA, PLA_CR, 1), 0);
            }
        }
    }
}

#[test]
fn rtl8153b_clears_w1c_teredo_events_with_one_byte_and_preserves_neighbors() {
    for revision in [0x6000, 0x6010] {
        let bytes = if revision == 0x6010 {
            synthetic_upload(revision)
        } else {
            Vec::new()
        };
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
        model
            .borrow_mut()
            .core
            .set(PLA, PLA_TEREDO_CFG, 4, 0x1234_abff);
        block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(revision),
            4,
            firmware.as_ref(),
        ))
        .unwrap();
        assert_eq!(model.borrow().core.get(PLA, PLA_TEREDO_CFG, 4), 0x1234_ab00);
        let hw = model.borrow();
        let writes: Vec<_> = hw
            .core
            .trace
            .iter()
            .filter(|(request, _)| request[0] == 0x40 && request[2..4] == [0xbc, 0xc0])
            .collect();
        assert_eq!(writes.len(), 1);
        assert_eq!(writes[0].0[4..6], [0x11, 1]);
        assert_eq!(writes[0].1[0], 0xff);
    }
}

#[test]
fn rtl8153b_power_cut_during_ups_exit_requires_an_extra_phy_reload() {
    let model = Rc::new(RefCell::new(FirmwareModel::new(0x6000, &[])));
    model.borrow_mut().ups_exit_power_cut = true;
    let (chip, _) = block_on(Chip::attach_with_firmware(
        FirmwareIo(model.clone()),
        Revision(0x6000),
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
    assert_eq!(model.borrow().core.get(USB, USB_MISC_0, 2) & PCUT_STATUS, 0);
}

#[test]
fn rtl8153b_reloads_observed_power_cut_phy_and_refuses_all_deadlines() {
    let bytes = synthetic_upload(0x6010);
    let firmware = Firmware::parse(&bytes, Revision(0x6010)).unwrap();
    for state in [PHY_STAT_LAN_ON, PHY_STAT_PWRDN, PHY_STAT_EXT_INIT] {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x6010, &bytes)));
        model
            .borrow_mut()
            .phy_states
            .extend([PHY_STAT_LAN_ON, PHY_STAT_LAN_ON, state]);
        let (chip, _) = block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(0x6010),
            4,
            Some(&firmware),
        ))
        .unwrap();
        assert!(chip.initialized);
        assert!(model.borrow().phy_states.is_empty());
        assert_eq!(model.borrow().core.get(PLA, MII_BMCR, 2) & BMCR_PDOWN, 0);
    }
    for case in 0..6 {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x6000, &[])));
        {
            let mut hw = model.borrow_mut();
            match case {
                0 => hw.core.set(PLA, PLA_BOOT_CTRL, 2, 0),
                1 => hw.core.set(PLA, OCP_PHY_STATUS, 2, 1),
                2 => hw.core.set(PLA, 0xe000, 2, 0),
                3 => hw.core.stuck_phy = true,
                4 => hw.core.stuck_list = true,
                _ => hw.stuck_patch = true,
            }
        }
        assert!(matches!(
            block_on(Chip::attach_with_firmware(
                FirmwareIo(model),
                Revision(0x6000),
                4,
                None
            )),
            Err(BlockError::Timeout)
        ));
    }
}

#[test]
#[ignore = "oracle: STELA_RTL8153B_LINK_ORACLE"]
fn unchanged_linux_8153b_link_register_effects() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let trace =
        std::fs::read_to_string(std::env::var("STELA_RTL8153B_LINK_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
        "8f3a8d6cc91aa85fe0408b578af853e2b8c5277a32a08c829cf4772db610f835"
    );
    let mut count = 0;
    for line in trace.lines() {
        let v: Vec<_> = line.split_whitespace().collect();
        assert_eq!(v.len(), 7);
        let revision = u16::from_str_radix(v[0], 16).unwrap();
        let speed = v[1].parse::<u8>().unwrap();
        let status = v[2].parse::<u32>().unwrap();
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &[])));
        model
            .borrow_mut()
            .core
            .set(PLA, PLA_MAC_PWR_CTRL4, 2, 0xa55a);
        let mut chip =
            block_on(Chip::probe(FirmwareIo(model.clone()), Revision(revision))).unwrap();
        model.borrow_mut().core.set(PLA, PLA_TCR1, 2, 0xa55a);
        chip.usb_speed = speed;
        block_on(chip.enable_8153b(status)).unwrap();
        for ((bank, address), expected) in [
            (USB, USB_RX_EARLY_TIMEOUT),
            (USB, USB_RX_EARLY_SIZE),
            (PLA, PLA_TCR1),
            (PLA, PLA_MAC_PWR_CTRL4),
        ]
        .into_iter()
        .zip(&v[3..])
        {
            assert_eq!(
                model.borrow().core.get(bank, address, 2),
                expected.parse::<u32>().unwrap(),
                "{line}"
            );
        }
        count += 1;
    }
    assert_eq!(count, 40);
}

#[test]
#[ignore = "oracle: STELA_RTL815X_FIRMWARE"]
fn real_firmware_8153b_full_startup_and_link() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let (name, revision, _, hash) = crate::usb::ethernet::firmware::tests::FIXTURES
        .iter()
        .copied()
        .find(|(_, revision, _, _)| *revision == 0x6010)
        .unwrap();
    let directory = std::env::var("STELA_RTL815X_FIRMWARE").unwrap();
    let bytes = std::fs::read(std::path::Path::new(&directory).join(name)).unwrap();
    assert_eq!(alloc::format!("{:x}", Sha256::digest(&bytes)), hash);
    let firmware = Firmware::parse(&bytes, Revision(revision)).unwrap();
    for speed in [1, 3, 4, 5] {
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
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

#[test]
fn every_rtl8153b_startup_transfer_failure_prevents_ready_state() {
    let bytes = synthetic_upload(0x6010);
    let firmware = Firmware::parse(&bytes, Revision(0x6010)).unwrap();
    let model = Rc::new(RefCell::new(FirmwareModel::new(0x6010, &bytes)));
    block_on(Chip::attach_with_firmware(
        FirmwareIo(model.clone()),
        Revision(0x6010),
        4,
        Some(&firmware),
    ))
    .unwrap();
    let count = model.borrow().core.trace.len();
    for at in 0..count {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x6010, &bytes)));
        model.borrow_mut().core.fail_at = Some(at);
        assert!(
            block_on(Chip::attach_with_firmware(
                FirmwareIo(model.clone()),
                Revision(0x6010),
                4,
                Some(&firmware)
            ))
            .is_err()
        );
        assert_eq!(model.borrow().core.trace.len(), at + 1);
    }
}
