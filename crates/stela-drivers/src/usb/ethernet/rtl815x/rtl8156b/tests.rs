use super::*;
use crate::usb::ethernet::{
    firmware::tests::synthetic_upload,
    rtl815x::{FirmwareIo, FirmwareModel},
};
use alloc::rc::Rc;
use core::cell::RefCell;
use stela_driver_base::stepped::block_on;

fn image(revision: u16) -> Vec<u8> {
    if revision == 0x7400 {
        Vec::new()
    } else {
        synthetic_upload(revision)
    }
}
fn seed(hw: &mut FirmwareModel, dack: bool) {
    hw.core.set(PLA, PLA_RCR, 4, 0xffff_ffff);
    hw.core.set(PLA, PLA_CPCR, 2, 0xaa00 | CPCR_RX_VLAN);
    hw.core.set(USB, USB_FW_CTRL, 2, 0xa45f);
    hw.core.set(
        PLA,
        PLA_POL_GPIO_CTRL,
        2,
        if dack { DACK_DET_EN } else { 0 },
    );
    hw.core.set(USB, USB_UPHY_XTAL, 2, 0xff00);
    hw.core.set(USB, USB_BMU_CONFIG, 2, 0xab00);
    hw.core.set(USB, USB_RX_AGGR_NUM, 2, 0xffff);
    hw.core.set(PLA, OCP_10GBT_CTRL, 2, 0x1020);
    hw.core.set(PLA, 0xac46, 2, 0x1234);
    hw.core.set(PLA, 0xad30, 2, 0x1234);
}

#[test]
fn rtl8156b_12_13_15_flash_phy_dsp_tuning_and_capabilities() {
    for revision in [0x7400, 0x7410, 0x7420] {
        let bytes = image(revision);
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        for speed in [1, 3, 4, 5] {
            for dack in [false, true] {
                let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
                seed(&mut model.borrow_mut(), dack);
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
                    assert_eq!(hw.core.get(PLA, PLA_RCR, 2) & SLOT_EN, 0);
                    assert_eq!(hw.core.get(PLA, PLA_RCR1, 2), 0xff3f);
                    assert_eq!(
                        hw.core.get(PLA, PLA_CPCR, 2),
                        0xaa00 | CPCR_RX_VLAN | FLOW_CTRL_EN
                    );
                    assert_eq!(hw.core.get(USB, USB_FC_TIMER, 2), CTRL_TIMER_EN | 75);
                    assert_eq!(
                        hw.core.get(USB, USB_FW_CTRL, 2),
                        if dack { 0xa457 } else { 0xa557 }
                    );
                    assert_eq!(
                        hw.core.get(PLA, PLA_POL_GPIO_CTRL, 2) & DACK_DET_EN,
                        if dack { DACK_DET_EN } else { 0 }
                    );
                    assert_eq!(
                        hw.core.get(USB, USB_UPHY_XTAL, 2),
                        if revision == 0x7400 { 0xff00 } else { 0xfe00 }
                    );
                    assert_eq!(hw.core.get(USB, USB_BMU_CONFIG, 2), 0xab00);
                    assert_eq!(hw.core.get(USB, USB_U2P3_CTRL, 2) & RX_DETECT8, RX_DETECT8);
                    assert_eq!(
                        hw.core.get(PLA, OCP_10GBT_CTRL, 2),
                        if revision == 0x7420 {
                            0x1020
                        } else if speed >= SUPER_SPEED {
                            0xa0
                        } else {
                            0x20
                        }
                    );
                    if revision == 0x7400 {
                        assert_eq!(hw.core.get(PLA, 0xbf86, 2), 0x9000);
                        assert_eq!(hw.sram2[&0x8560], 0x19cc);
                        assert_eq!(hw.sram2[&0x8fd8], 0xf600);
                        for (address, value) in [
                            (0xc23e, 0),
                            (0xc240, 0x103),
                            (0xc242, 0x507),
                            (0xc244, 0x90b),
                            (0xc246, 0xc0e),
                            (0xc248, 0x1012),
                            (0xc24a, 0x1416),
                        ] {
                            assert_eq!(hw.dsp[&address], value);
                        }
                        assert_eq!(hw.core.get(PLA, 0xb896, 2) & 1, 1);
                    } else {
                        assert_eq!(hw.sram2[&0x80f5], 0x760e);
                        assert_eq!(hw.sram2[&0x8107], 0x360e);
                        assert_eq!(hw.sram[&0x8044], 0x2417);
                        assert_eq!(hw.sram[&0x807a], 0x2417);
                        assert_eq!(
                            hw.core.get(PLA, 0xac46, 2),
                            if revision == 0x7410 { 0x1294 } else { 0x1234 }
                        );
                        assert_eq!(
                            hw.core.get(PLA, 0xad30, 2),
                            if revision == 0x7410 { 0x1235 } else { 0x1234 }
                        );
                    }
                }
                for status in [0x06, 0x07, 0x0a, 0x0b, 0x13, 0x103, 0x403, 0x473] {
                    model.borrow_mut().core.phy_status = status;
                    assert!(block_on(chip.carrier()).unwrap());
                    assert_eq!(model.borrow().core.get(USB, USB_RX_AGGR_NUM, 2), 0xfe00);
                    assert_eq!(model.borrow().core.get(USB, USB_RX_EARLY_TIMEOUT, 2), 80);
                    assert_eq!(model.borrow().core.get(USB, USB_RX_EARLY_SIZE, 2), 5949);
                    block_on(chip.stop()).unwrap();
                    // B keeps the pause thresholds; only the A disable callback clears them.
                    assert_eq!(model.borrow().core.get(PLA, PLA_RX_FIFO_FULL, 2), 512);
                    assert_eq!(model.borrow().core.get(PLA, PLA_RX_FIFO_EMPTY, 2), 1024);
                }
            }
        }
    }
}

#[test]
fn rtl8156b_flash_delay_bypass_and_timeout_are_observed() {
    let bytes = image(0x7410);
    let firmware = Firmware::parse(&bytes, Revision(0x7410)).unwrap();
    for mode in 0..4 {
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x7410, &bytes)));
        {
            let mut hw = model.borrow_mut();
            hw.core.set(PLA, PLA_GPHY_CTRL, 2, GPHY_FLASH);
            match mode {
                0 => hw.flash_pending = Some(3),
                1 => hw.core.set(USB, USB_GPHY_CTRL, 2, BYPASS_FLASH),
                2 => hw.core.set(USB, USB_GPHY_CTRL, 2, GPHY_PATCH_DONE),
                _ => {
                    hw.flash_pending = Some(3);
                    hw.stuck_flash = true;
                }
            }
        }
        let result = block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(0x7410),
            4,
            Some(&firmware),
        ));
        if mode == 3 {
            assert!(matches!(result, Err(BlockError::Timeout)));
            assert!(
                model
                    .borrow()
                    .core
                    .trace
                    .iter()
                    .all(|(_, data)| data.len() <= 4)
            );
        } else {
            assert!(result.unwrap().0.initialized);
            if mode == 0 {
                assert!(model.borrow().flash_pending.is_none());
            }
        }
    }
}

#[test]
fn rtl8156b_every_transfer_failure_prevents_ready_on_large_12_tuning_and_speedup13() {
    for revision in [0x7400, 0x7410] {
        let bytes = image(revision);
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
        block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(revision),
            4,
            firmware.as_ref(),
        ))
        .unwrap();
        let count = model.borrow().core.trace.len();
        for at in 0..count {
            let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
            model.borrow_mut().core.fail_at = Some(at);
            assert!(
                block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    Revision(revision),
                    4,
                    firmware.as_ref()
                ))
                .is_err()
            );
            assert_eq!(model.borrow().core.trace.len(), at + 1);
        }
    }
}
#[test]
fn rtl8156b_phy_upload_modes_ups_reload_and_deadlines() {
    for revision in [0x7400, 0x7410, 0x7420] {
        let bytes = image(revision);
        let firmware =
            (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
        for state in [PHY_STAT_LAN_ON, PHY_STAT_PWRDN, PHY_STAT_EXT_INIT] {
            let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
            model.borrow_mut().core.set(PLA, 0xa466, 2, 0xab01);
            model.borrow_mut().core.set(PLA, 0xa468, 2, 0xaa0a);
            model.borrow_mut().phy_states.extend([
                PHY_STAT_LAN_ON,
                PHY_STAT_LAN_ON,
                state,
                PHY_STAT_LAN_ON,
            ]);
            let (chip, _) = block_on(Chip::attach_with_firmware(
                FirmwareIo(model.clone()),
                Revision(revision),
                4,
                firmware.as_ref(),
            ))
            .unwrap();
            assert!(chip.initialized && model.borrow().phy_states.is_empty());
            assert_eq!(
                model.borrow().core.get(PLA, 0xa466, 2),
                if state == PHY_STAT_EXT_INIT {
                    0xab00
                } else {
                    0xab01
                }
            );
            assert_eq!(
                model.borrow().core.get(PLA, 0xa468, 2),
                if state == PHY_STAT_EXT_INIT {
                    0xaa00
                } else {
                    0xaa0a
                }
            );
        }
        let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
        model.borrow_mut().ups_exit_power_cut = true;
        let (chip, _) = block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(revision),
            4,
            firmware.as_ref(),
        ))
        .unwrap();
        assert!(chip.initialized && !model.borrow().ups_exit_power_cut);
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
        for case in 0..if revision == 0x7400 { 6 } else { 7 } {
            let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
            {
                let mut hw = model.borrow_mut();
                match case {
                    0 => hw.core.set(PLA, PLA_BOOT_CTRL, 2, 0),
                    1 => hw.core.set(PLA, OCP_PHY_STATUS, 2, 1),
                    2 => hw.core.set(PLA, 0xe000, 2, 0),
                    3 => hw.core.stuck_phy = true,
                    4 => hw.core.stuck_reset = true,
                    5 => hw.stuck_patch = true,
                    _ => hw.stuck_speed = true,
                }
            }
            assert!(
                matches!(
                    block_on(Chip::attach_with_firmware(
                        FirmwareIo(model),
                        Revision(revision),
                        4,
                        firmware.as_ref()
                    )),
                    Err(BlockError::Timeout)
                ),
                "{revision:04x}/{case}"
            );
        }
    }
}

#[test]
#[ignore = "oracle: STELA_RTL8156B_ENABLE_ORACLE"]
fn unchanged_linux_rtl8156b_enable_register_effects() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let trace =
        std::fs::read_to_string(std::env::var("STELA_RTL8156B_ENABLE_ORACLE").unwrap()).unwrap();
    assert_eq!(
        alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
        "6033cef7fdad8f3918be859c4021e9c2bb40780bd6812b6ffc22754c500635a6"
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
        (USB, USB_RX_AGGR_NUM),
    ];
    let mut count = 0;
    for line in trace.lines() {
        let v: Vec<_> = line.split_whitespace().collect();
        assert_eq!(v.len(), 15);
        let revision = u16::from_str_radix(v[0], 16).unwrap();
        let speed = v[1].parse::<u8>().unwrap();
        let status = v[2].parse::<u32>().unwrap();
        let bytes = image(revision);
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
    assert_eq!(count, 96);
}

#[test]
#[ignore = "oracle: STELA_RTL815X_FIRMWARE"]
fn real_firmware_8156b_full_startup_link_and_stop() {
    extern crate std;
    use sha2::{Digest, Sha256};
    let (name, _, _, hash) = crate::usb::ethernet::firmware::tests::FIXTURES
        .iter()
        .copied()
        .find(|(_, revision, _, _)| *revision == 0x7410)
        .unwrap();
    let directory = std::env::var("STELA_RTL815X_FIRMWARE").unwrap();
    let bytes = std::fs::read(std::path::Path::new(&directory).join(name)).unwrap();
    assert_eq!(alloc::format!("{:x}", Sha256::digest(&bytes)), hash);
    for revision in [0x7410, 0x7420] {
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
}
