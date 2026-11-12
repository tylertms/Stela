// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! RTL8153C revision14 has its own SPI, UPS and FIFO/MTU sequence. Linux
//! explicitly shares its PHY configuration with B, not its init/up path.
use super::super::firmware::Firmware;
use super::*;

impl<C: Control> Chip<C> {
    async fn config34_8153c(&mut self) -> Result<(), BlockError> {
        self.write(PLA, PLA_CRWECR, 1, 0xc0).await?;
        self.modify(PLA, PLA_CONFIG34, 2, 0, 1 << 8).await?;
        self.write(PLA, PLA_CRWECR, 1, 0).await
    }

    pub(super) async fn initialize_8153c(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        self.u1u2_8153b(false).await?;
        self.write(PLA, PLA_CRWECR, 1, 0xc0).await?;
        self.modify(PLA, PLA_CONFIG5, 2, 1 << 3, 0).await?;
        self.modify(USB, 0xcbf0, 2, 0, 1 << 1).await?;
        self.poll(
            PLA,
            PLA_BOOT_CTRL,
            2,
            AUTOLOAD_DONE,
            AUTOLOAD_DONE,
            (500, 20_000),
        )
        .await?;
        self.phy_state(None).await?;
        let mapped = self.phy_address(MII_BMCR).await?;
        let bmcr = self.read(PLA, mapped, 2).await?;
        if bmcr & BMCR_PDOWN != 0 {
            self.write(PLA, mapped, 2, bmcr & !BMCR_PDOWN).await?;
        }
        self.phy_state(Some(PHY_STAT_LAN_ON)).await?;
        self.u2p3(false).await?;
        self.write(USB, USB_MSC_TIMER, 2, 0x0fff).await?;
        self.modify(USB, USB_POWER_CUT, 2, PWR_EN, 0).await?;
        self.modify(USB, USB_MISC_0, 2, PCUT_STATUS, 0).await?;
        self.modify(USB, USB_POWER_CUT, 1, UPS_EN | USP_PREWAKE, 0)
            .await?;
        self.modify(USB, USB_MISC_2, 1, UPS_FORCE_PWR_DOWN, 0)
            .await?;
        if self.read(USB, USB_MISC_0, 2).await? & PCUT_STATUS != 0 {
            self.poll(
                PLA,
                PLA_BOOT_CTRL,
                2,
                AUTOLOAD_DONE,
                AUTOLOAD_DONE,
                (500, 20_000),
            )
            .await?;
            self.hardware_phy_8153b(firmware).await?;
        }
        self.config34_8153c().await?;
        self.queue_wake_off().await?;
        self.write(PLA, PLA_CRWECR, 1, 0xc0).await?;
        self.modify(PLA, PLA_CONFIG34, 2, LINK_OFF_WAKE_EN, 0)
            .await?;
        self.write(PLA, PLA_CRWECR, 1, 0).await?;
        let linked = self.read(PLA, PLA_PHYSTATUS, 2).await? & PHYSTATUS_LINK != 0;
        self.modify(
            PLA,
            PLA_EXTRA_STATUS,
            2,
            CUR_LINK_OK,
            POLL_LINK_CHG | if linked { CUR_LINK_OK } else { 0 },
        )
        .await?;
        self.u1u2_8153b(true).await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL2, 2, 0, MAC_CLK_SPDWN_EN)
            .await?;
        self.modify(USB, USB_MISC_2, 1, 1 << 7, 0).await?;
        self.modify(USB, USB_USB_CTRL, 2, RX_AGG_DISABLE | RX_ZERO_EN, 0)
            .await?;
        self.modify(PLA, PLA_RSTTALLY, 2, 0, TALLY_RESET).await?;
        // r8153c_hw_phy_cfg calls B's full routine. Its extra r_tune field
        // only describes future UPS flags; host runtime suspend is unsupported.
        self.hardware_phy_8153b(firmware).await?;
        self.up_8153c().await
    }

    async fn up_8153c(&mut self) -> Result<(), BlockError> {
        self.u1u2_8153b(false).await?;
        self.u2p3(false).await?;
        self.aldps(false).await?;
        self.modify(PLA, PLA_MISC_1, 2, 0, RXDY_GATED_EN).await?;
        // C uses the newer W1C Teredo event register, not A/B's select mask.
        self.write(PLA, PLA_TEREDO_CFG, 1, 0xff).await?;
        self.write(PLA, PLA_WDT6_CTRL, 2, WDT6_SET_MODE).await?;
        self.write(PLA, PLA_REALWOW_TIMER, 2, 0).await?;
        self.write(PLA, PLA_TEREDO_TIMER, 4, 0).await?;
        self.modify(PLA, PLA_RCR, 4, RCR_ACPT_ALL, 0).await?;
        self.nic_reset().await?;
        self.reset_bmu().await?;
        self.modify(PLA, PLA_OOB_CTRL, 1, NOW_IS_OOB, 0).await?;
        self.modify(PLA, PLA_SFF_STS_7, 2, MCU_BORW_EN, 0).await?;
        self.poll(
            PLA,
            PLA_OOB_CTRL,
            1,
            LINK_LIST_READY,
            LINK_LIST_READY,
            (1000, 1000),
        )
        .await?;
        self.modify(PLA, PLA_SFF_STS_7, 2, 0, RE_INIT_LL).await?;
        self.poll(
            PLA,
            PLA_OOB_CTRL,
            1,
            LINK_LIST_READY,
            LINK_LIST_READY,
            (1000, 1000),
        )
        .await?;
        self.modify(PLA, PLA_CPCR, 2, CPCR_RX_VLAN, 0).await?;
        // rtl8153c_change_mtu (MTU1500). In particular the TX FIFO credit
        // writes are words and must not overwrite the neighbor at +2.
        self.write(PLA, PLA_RMS, 2, 1522).await?;
        self.write(PLA, PLA_MTPS, 1, 10 * 1024 / 64).await?;
        self.write(PLA, PLA_TXFIFO_CTRL, 2, 512 / 64).await?;
        self.write(PLA, PLA_TXFIFO_FULL, 2, 2048 / 8).await?;
        self.nic_reset().await?;
        self.write(PLA, PLA_RXFIFO_CTRL0, 1, 2).await?;
        self.write(PLA, PLA_RXFIFO_FULL, 1, 8).await?;
        self.write(PLA, PLA_RXFIFO_CTRL1, 2, RXFIFO_THR2_NORMAL)
            .await?;
        self.write(PLA, PLA_RXFIFO_CTRL2, 2, RXFIFO_THR3_NORMAL)
            .await?;
        self.write(USB, USB_RX_BUF_TH, 4, RX_THR_B).await?;
        self.config34_8153c().await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL3, 2, PLA_MCU_SPDWN_EN, 0)
            .await?;
        self.aldps(true).await?;
        self.u1u2_8153b(true).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::ethernet::{
        firmware::tests::synthetic_upload,
        rtl815x::{FirmwareIo, FirmwareModel},
    };
    use alloc::rc::Rc;
    use core::cell::RefCell;
    use stela_driver_base::stepped::block_on;

    #[test]
    fn rtl8153c_spi_ups_fifo_widths_and_w1c_events_follow_revision14() {
        let bytes = synthetic_upload(0x6400);
        let firmware = Firmware::parse(&bytes, Revision(0x6400)).unwrap();
        for speed in [1, 3, 4, 5] {
            for power_cut in [false, true] {
                let model = Rc::new(RefCell::new(FirmwareModel::new(0x6400, &bytes)));
                {
                    let mut hw = model.borrow_mut();
                    hw.ups_exit_power_cut = power_cut;
                    hw.core.set(PLA, PLA_CONFIG5, 2, 0xf8ff);
                    hw.core.set(PLA, PLA_CONFIG34, 2, LINK_OFF_WAKE_EN);
                    hw.core.set(USB, 0xcbf0, 4, 0x1234_a5f1);
                    hw.core.set(USB, USB_MISC_2, 1, 0xff);
                    hw.core.set(PLA, PLA_RXFIFO_CTRL0, 4, 0xbbcc_aaee);
                    hw.core.set(PLA, PLA_RXFIFO_CTRL1, 4, 0xabcd_1111);
                    hw.core.set(PLA, PLA_RXFIFO_CTRL2, 4, 0x1234_1111);
                    hw.core.set(PLA, PLA_TXFIFO_CTRL, 4, 0xfeed_beef);
                    hw.core.set(PLA, PLA_TXFIFO_CTRL + 4, 4, 0x5678_1234);
                    hw.core.set(PLA, PLA_TEREDO_CFG, 4, 0x55aa_33ff);
                }
                let (mut chip, _) = block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    Revision(0x6400),
                    speed,
                    Some(&firmware),
                ))
                .unwrap();
                assert!(chip.initialized);
                assert_eq!(chip.receive_bytes(), 32768);
                {
                    let hw = model.borrow();
                    assert!(!hw.ups_exit_power_cut);
                    assert_eq!(hw.core.get(PLA, PLA_CONFIG5, 2), 0xf8f7);
                    assert_eq!(hw.core.get(PLA, PLA_CONFIG34, 2), 1 << 8);
                    assert_eq!(hw.core.get(USB, 0xcbf0, 4), 0x1234_a5f3);
                    assert_eq!(hw.core.get(USB, USB_MISC_2, 1), 0x7e);
                    assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL0, 4), 0xbb08_aa02);
                    assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL1, 4), 0xabcd_00a0);
                    assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL2, 4), 0x1234_0110);
                    assert_eq!(hw.core.get(PLA, PLA_TXFIFO_CTRL, 4), 0x0100_0008);
                    assert_eq!(hw.core.get(PLA, PLA_TXFIFO_CTRL + 4, 4), 0x5678_1234);
                    assert_eq!(hw.core.get(PLA, PLA_TEREDO_CFG, 4), 0x55aa_3300);
                    assert_eq!(hw.core.get(PLA, PLA_MTPS, 1), 160);
                    assert_eq!(
                        hw.core.get(USB, USB_LPM_CONFIG, 2) & LPM_U1U2_EN,
                        LPM_U1U2_EN
                    );
                    assert_eq!(hw.core.get(PLA, PLA_TCR0, 2) & TCR0_AUTO_FIFO, 0);
                }
                assert!(block_on(chip.carrier()).unwrap());
                assert_eq!(
                    model.borrow().core.get(USB, USB_FW_TASK, 2) & FC_PATCH_TASK,
                    FC_PATCH_TASK
                );
                assert_eq!(
                    model.borrow().core.get(USB, USB_UPT_RXDMA_OWN, 1),
                    OWN_CLEAR | OWN_UPDATE
                );
                block_on(chip.stop()).unwrap();
                assert_eq!(model.borrow().core.get(PLA, PLA_CR, 1), 0);
            }
        }
    }

    #[test]
    fn every_rtl8153c_transfer_failure_and_all_deadlines_prevent_ready_state() {
        let bytes = synthetic_upload(0x6400);
        let firmware = Firmware::parse(&bytes, Revision(0x6400)).unwrap();
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x6400, &bytes)));
        block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(0x6400),
            4,
            Some(&firmware),
        ))
        .unwrap();
        let count = model.borrow().core.trace.len();
        for at in 0..count {
            let model = Rc::new(RefCell::new(FirmwareModel::new(0x6400, &bytes)));
            model.borrow_mut().core.fail_at = Some(at);
            assert!(
                block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    Revision(0x6400),
                    4,
                    Some(&firmware)
                ))
                .is_err()
            );
            assert_eq!(model.borrow().core.trace.len(), at + 1);
        }
        for case in 0..6 {
            let model = Rc::new(RefCell::new(FirmwareModel::new(0x6400, &bytes)));
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
                    Revision(0x6400),
                    4,
                    Some(&firmware)
                )),
                Err(BlockError::Timeout)
            ));
        }
    }
    #[test]
    #[ignore = "oracle: STELA_RTL8153C_LINK_ORACLE"]
    fn unchanged_linux_8153c_link_register_effects() {
        extern crate std;
        use sha2::{Digest, Sha256};
        let trace =
            std::fs::read_to_string(std::env::var("STELA_RTL8153C_LINK_ORACLE").unwrap()).unwrap();
        assert_eq!(
            alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
            "38dd9fc3060215342d901ce2e6a157f7f63708f2879ceb53d543bd5b1ffbe427"
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
        assert_eq!(count, 20);
    }

    #[test]
    #[ignore = "oracle: STELA_RTL815X_FIRMWARE"]
    fn real_firmware_8153c_full_startup_and_link() {
        extern crate std;
        use sha2::{Digest, Sha256};
        let (name, revision, _, hash) = crate::usb::ethernet::firmware::tests::FIXTURES
            .iter()
            .copied()
            .find(|(_, revision, _, _)| *revision == 0x6400)
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
}
