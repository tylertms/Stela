// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! RTL8153A revisions 03/04/05/06: r8153_init, r8153_hw_phy_cfg,
//! rtl8153_up/enable/disable and their exact per-revision power/PHY branches.
use super::super::firmware::Firmware;
use super::*;

impl<C: Control> Chip<C> {
    async fn tolerance(&mut self, enable: bool) -> Result<(), BlockError> {
        let value = if enable { u32::MAX } else { 0 };
        self.write(USB, USB_TOLERANCE, 4, value).await?;
        self.write(USB, USB_TOLERANCE + 4, 2, value & 0xffff).await
    }

    pub(super) async fn u2p3(&mut self, enable: bool) -> Result<(), BlockError> {
        self.modify(
            USB,
            USB_U2P3_CTRL,
            2,
            U2P3_ENABLE,
            if enable { U2P3_ENABLE } else { 0 },
        )
        .await
    }

    pub(super) async fn aldps_8153(&mut self, enable: bool) -> Result<(), BlockError> {
        self.phy_modify(OCP_POWER_CFG, EN_ALDPS, if enable { EN_ALDPS } else { 0 })
            .await?;
        if !enable {
            self.minimum_wait(1000).await?;
            self.poll(PLA, 0xe000, 2, 0x0100, 0x0100, (20, 1000))
                .await?;
        }
        Ok(())
    }

    pub(super) async fn phy_state(&mut self, desired: Option<u32>) -> Result<u32, BlockError> {
        let mapped = self.phy_address(OCP_PHY_STATUS).await?;
        for _ in 0..500 {
            let state = self.read(PLA, mapped, 2).await? & PHY_STAT_MASK;
            if desired.map_or(
                matches!(state, PHY_STAT_LAN_ON | PHY_STAT_PWRDN | PHY_STAT_EXT_INIT),
                |expected| state == expected,
            ) {
                return Ok(state);
            }
            self.io.delay_us(20_000).await;
        }
        Err(BlockError::Timeout)
    }

    pub(super) async fn queue_wake_off(&mut self) -> Result<(), BlockError> {
        self.modify(PLA, PLA_INDICATE_FALG, 1, UPCOMING_RUNTIME_D3, 0)
            .await?;
        self.modify(PLA, PLA_SUSPEND_FLAG, 1, LINK_CHG_EVENT, 0)
            .await?;
        self.modify(PLA, PLA_EXTRA_STATUS, 2, LINK_CHANGE_FLAG, 0)
            .await
    }

    pub(super) async fn initialize_8153a(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        self.tolerance(false).await?;
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
        if matches!(self.revision.0, 0x5c00 | 0x5c10 | 0x5c20) {
            self.phy_write(OCP_ADC_CFG, CKADSEL_L | ADC_EN | EN_EMI_L)
                .await?;
        }
        let mapped = self.phy_address(MII_BMCR).await?;
        let bmcr = self.read(PLA, mapped, 2).await?;
        if bmcr & BMCR_PDOWN != 0 {
            self.write(PLA, mapped, 2, bmcr & !BMCR_PDOWN).await?;
        }
        self.phy_state(Some(PHY_STAT_LAN_ON)).await?;
        self.u2p3(false).await?;
        match self.revision.0 {
            0x5c10 => {
                self.modify(USB, USB_SSPHYLINK2, 2, PWD_DN_SCALE_MASK, 96 << 1)
                    .await?;
                self.modify(USB, USB_USB2PHY, 1, 0, USB2PHY_L1 | USB2PHY_SUSPEND)
                    .await?;
            }
            0x5c20 | 0x5c30 => {
                if self.revision.0 == 0x5c20 {
                    self.modify(PLA, PLA_DMY_REG0, 1, ECM_ALDPS, 0).await?;
                }
                let dynamic = self.read(USB, USB_BURST_SIZE, 2).await? != 0;
                self.modify(
                    USB,
                    USB_CSR_DUMMY1,
                    1,
                    DYNAMIC_BURST,
                    if dynamic { DYNAMIC_BURST } else { 0 },
                )
                .await?;
                if self.revision.0 == 0x5c30 {
                    self.queue_wake_off().await?;
                    let linked = self.read(PLA, PLA_PHYSTATUS, 2).await? & PHYSTATUS_LINK != 0;
                    self.modify(
                        PLA,
                        PLA_EXTRA_STATUS,
                        2,
                        CUR_LINK_OK,
                        POLL_LINK_CHG | if linked { CUR_LINK_OK } else { 0 },
                    )
                    .await?;
                }
            }
            _ => {}
        }
        self.modify(USB, USB_CSR_DUMMY2, 1, 0, EP4_FULL_FC).await?;
        self.modify(USB, USB_WDT11_CTRL, 2, TIMER11_EN, 0).await?;
        self.modify(PLA, PLA_LED_FEATURE, 2, LED_MODE_MASK, 0)
            .await?;
        let timer = if self.revision.0 == 0x5c10 && self.usb_speed < SUPER_SPEED {
            LPM_TIMER_500MS
        } else {
            LPM_TIMER_500US
        };
        self.write(USB, USB_LPM_CTRL, 1, FIFO_EMPTY_1FB | ROK_EXIT_LPM | timer)
            .await?;
        self.modify(
            USB,
            USB_AFE_CTRL2,
            2,
            SEN_VAL_MASK,
            SEN_VAL_NORMAL | SEL_RXIDLE,
        )
        .await?;
        self.write(USB, USB_CONNECT_TIMER, 2, 1).await?;
        self.modify(USB, USB_POWER_CUT, 2, PWR_EN | PHASE2_EN, 0)
            .await?;
        self.modify(USB, USB_MISC_0, 2, PCUT_STATUS, 0).await?;
        // Keep the original wake filters (saved_wolopts), while disabling the
        // runtime-only link-off wake bit. Host USB runtime suspend is not armed.
        self.write(PLA, PLA_CRWECR, 1, 0xc0).await?;
        self.modify(PLA, PLA_CONFIG34, 2, LINK_OFF_WAKE_EN, 0)
            .await?;
        self.write(PLA, PLA_CRWECR, 1, 0).await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL2, 2, MAC_CLK_SPDWN_EN, 0)
            .await?;
        self.tolerance(true).await?;
        // Linux's usb_enable_lpm is host power policy; this polled backend
        // keeps host U1/U2/suspend disabled and makes no successful LPM claim.
        self.modify(PLA, PLA_CONFIG6, 1, 0, LANWAKE_CLR_EN).await?;
        self.modify(PLA, PLA_LWAKE_CTRL_REG, 1, LANWAKE_PIN, 0)
            .await?;
        self.modify(
            USB,
            USB_USB_CTRL,
            2,
            RX_AGG_DISABLE | RX_ZERO_EN,
            if self.policy.disable_receive_aggregation {
                RX_AGG_DISABLE
            } else {
                0
            },
        )
        .await?;
        self.modify(PLA, PLA_RSTTALLY, 2, 0, TALLY_RESET).await?;

        // r8153_hw_phy_cfg, with EEE advertisement disabled.
        self.aldps(false).await?;
        self.modify(PLA, PLA_EEE_CR, 2, EEE_RX_EN | EEE_TX_EN, 0)
            .await?;
        self.phy_modify(OCP_EEE_CFG, EEE10_EN, 0).await?;
        self.phy_write(OCP_EEE_ADV, 0).await?;
        if let Some(firmware) = firmware {
            self.upload_firmware(firmware, false).await?;
        }
        if self.revision.0 == 0x5c00 {
            self.phy_modify(OCP_EEE_CFG, CTAP_SHORT_EN, 0).await?;
        }
        self.phy_modify(OCP_POWER_CFG, 0, EEE_CLKDIV_EN).await?;
        self.phy_modify(OCP_DOWN_SPEED, 0, EN_10M_BGOFF).await?;
        self.phy_modify(OCP_POWER_CFG, 0, EN_10M_PLLOFF).await?;
        self.sram_write(SRAM_IMPEDANCE as u16, 0x0b13).await?;
        self.modify(PLA, PLA_PHY_PWR, 2, 0, PFM_PWM_SWITCH).await?;
        self.sram_write(SRAM_LPF_CFG as u16, 0xf70f).await?;
        self.sram_write(SRAM_10M_AMP1 as u16, 0x00af).await?;
        self.sram_write(SRAM_10M_AMP2 as u16, 0x0208).await?;
        self.aldps(true).await?;
        self.phy_modify(MII_ADVERTISE, 0x01e0, ADVERTISE_10_100_PAUSE)
            .await?;
        if matches!(self.revision.0, 0x5c20 | 0x5c30) {
            self.u2p3(true).await?;
        }
        // Advertise copper gigabit full duplex, never 1000-half, then reset PHY.
        self.phy_modify(OCP_BASE_MII + 18, 0x0300, 0x0200).await?;
        self.phy_write(MII_BMCR, BMCR_ANENABLE_RESTART | BMCR_RESET)
            .await?;
        let mapped = self.phy_address(MII_BMCR).await?;
        self.minimum_wait(20_000).await?;
        self.poll(PLA, mapped, 2, BMCR_RESET, 0, (50, 20_000))
            .await?;
        self.up_8153a().await
    }

    pub(super) async fn reset_bmu(&mut self) -> Result<(), BlockError> {
        self.modify(USB, USB_BMU_RESET, 1, BMU_RESET_EP_IN | BMU_RESET_EP_OUT, 0)
            .await?;
        self.modify(USB, USB_BMU_RESET, 1, 0, BMU_RESET_EP_IN | BMU_RESET_EP_OUT)
            .await
    }

    async fn up_8153a(&mut self) -> Result<(), BlockError> {
        self.tolerance(false).await?;
        self.u2p3(false).await?;
        self.aldps(false).await?;
        self.first_init_8153().await?;
        self.modify(PLA, PLA_CONFIG6, 1, 0, LANWAKE_CLR_EN).await?;
        self.modify(PLA, PLA_LWAKE_CTRL_REG, 1, LANWAKE_PIN, 0)
            .await?;
        self.modify(USB, USB_SSPHYLINK1, 2, DELAY_PHY_PWR_CHG, 0)
            .await?;
        self.aldps(true).await?;
        if matches!(self.revision.0, 0x5c20 | 0x5c30) {
            self.u2p3(true).await?;
        }
        self.tolerance(true).await
    }

    /// r8153_first_init is shared by A and B in the donor. C/8156 have
    /// different FIFO/MTU paths and must never enter this helper.
    pub(super) async fn first_init_8153(&mut self) -> Result<(), BlockError> {
        self.modify(PLA, PLA_MISC_1, 2, 0, RXDY_GATED_EN).await?;
        if self.revision.is_8153b() {
            // r8153_teredo_off: 08/09 use W1C event bits; clearing a
            // select-mask word would leave those latched wake events pending.
            self.write(PLA, PLA_TEREDO_CFG, 1, 0xff).await?;
        } else {
            self.modify(
                PLA,
                PLA_TEREDO_CFG,
                2,
                TEREDO_SEL | TEREDO_RS_EVENT_MASK | OOB_TEREDO_EN,
                0,
            )
            .await?;
        }
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
        self.write(PLA, PLA_RMS, 2, 1522).await?;
        self.write(PLA, PLA_MTPS, 1, MTPS_JUMBO).await?;
        self.modify(PLA, PLA_TCR0, 2, 0, TCR0_AUTO_FIFO).await?;
        self.nic_reset().await?;
        self.write(PLA, PLA_RXFIFO_CTRL0, 4, RXFIFO_THR1_NORMAL)
            .await?;
        self.write(PLA, PLA_RXFIFO_CTRL1, 2, RXFIFO_THR2_NORMAL)
            .await?;
        self.write(PLA, PLA_RXFIFO_CTRL2, 2, RXFIFO_THR3_NORMAL)
            .await?;
        self.write(PLA, PLA_TXFIFO_CTRL, 4, TXFIFO_THR_NORMAL2)
            .await
    }

    pub(super) async fn enable_8153a(&mut self, status: u32) -> Result<(), BlockError> {
        let timeout = match self.usb_speed {
            4 | 5 => 85000,
            3 => 250000,
            _ => 524280,
        };
        self.write(USB, USB_RX_EARLY_TIMEOUT, 2, timeout / 8)
            .await?;
        self.write(
            USB,
            USB_RX_EARLY_SIZE,
            2,
            ((self.receive_bytes() - (1522 + 24 + 8)) / 4) as u32,
        )
        .await?;
        let half = status & (0x04 | 0x08) != 0 && status & 1 == 0;
        self.modify(
            PLA,
            PLA_TCR1,
            2,
            IFG_MASK,
            if half { IFG_144NS } else { IFG_96NS },
        )
        .await?;
        self.modify(
            PLA,
            PLA_MAC_PWR_CTRL4,
            2,
            TX10MIDLE_EN,
            if half { 0 } else { TX10MIDLE_EN },
        )
        .await
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
    fn exact_device_receive_aggregation_policy_preserves_other_usb_fields() {
        for revision in [0x5c00, 0x5c10, 0x5c20, 0x5c30] {
            let bytes = if revision == 0x5c00 {
                Vec::new()
            } else {
                synthetic_upload(revision)
            };
            let firmware =
                (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
            for disabled in [false, true] {
                let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
                model.borrow_mut().core.set(USB, USB_USB_CTRL, 2, 0xffff);
                let (mut chip, _) = block_on(Chip::attach_with_policy(
                    FirmwareIo(model.clone()),
                    Revision(revision),
                    3,
                    firmware.as_ref(),
                    Policy {
                        disable_receive_aggregation: disabled,
                    },
                ))
                .unwrap();
                assert_eq!(
                    model.borrow().core.get(USB, USB_USB_CTRL, 2),
                    if disabled { 0xff7f } else { 0xff6f }
                );
                assert!(block_on(chip.carrier()).unwrap());
                block_on(chip.stop()).unwrap();
                assert_eq!(
                    model.borrow().core.get(USB, USB_USB_CTRL, 2),
                    if disabled { 0xff7f } else { 0xff6f }
                );
            }
        }
    }

    #[test]
    fn all_8153a_revisions_apply_their_phy_power_fifo_and_link_branches() {
        for revision in [0x5c00, 0x5c10, 0x5c20, 0x5c30] {
            let bytes = if revision == 0x5c00 {
                Vec::new()
            } else {
                synthetic_upload(revision)
            };
            let firmware =
                (!bytes.is_empty()).then(|| Firmware::parse(&bytes, Revision(revision)).unwrap());
            for speed in [1, 3, 4, 5] {
                for dynamic in [false, true] {
                    let model = Rc::new(RefCell::new(FirmwareModel::new(revision, &bytes)));
                    {
                        let mut hw = model.borrow_mut();
                        hw.core.set(USB, USB_TOLERANCE + 4, 4, 0x1234_0000);
                        hw.core.set(USB, USB_BURST_SIZE, 2, u32::from(dynamic));
                        hw.core.set(PLA, PLA_RXFIFO_CTRL1, 4, 0xa5a5_ffff);
                        hw.core.set(PLA, PLA_RXFIFO_CTRL2, 4, 0x5a5a_ffff);
                        hw.core.set(PLA, OCP_EEE_CFG, 2, 0xffff);
                    }
                    let (mut chip, mac) = block_on(Chip::attach_with_firmware(
                        FirmwareIo(model.clone()),
                        Revision(revision),
                        speed,
                        firmware.as_ref(),
                    ))
                    .unwrap();
                    assert_eq!(mac.0, [2, 0x11, 0x22, 0x33, 0x44, 0x55]);
                    assert!(chip.initialized);
                    assert_eq!(
                        chip.receive_bytes(),
                        if speed >= SUPER_SPEED { 32768 } else { 16384 }
                    );
                    {
                        let hw = model.borrow();
                        assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL0, 4), 0x0008_0002);
                        assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL1, 4), 0xa5a5_00a0);
                        assert_eq!(hw.core.get(PLA, PLA_RXFIFO_CTRL2, 4), 0x5a5a_0110);
                        assert_eq!(hw.core.get(PLA, OCP_BASE_MII + 18, 2) & 0x300, 0x200);
                        assert_eq!(hw.core.get(PLA, OCP_EEE_ADV, 2), 0);
                        assert_eq!(hw.core.get(PLA, OCP_EEE_CFG, 2) & EEE10_EN, 0);
                        assert_eq!(
                            hw.core.get(PLA, OCP_POWER_CFG, 2),
                            EEE_CLKDIV_EN | EN_10M_PLLOFF | EN_ALDPS
                        );
                        assert_eq!(hw.core.get(USB, USB_TOLERANCE + 4, 4), 0x1234_ffff);
                        assert_eq!(
                            hw.core.get(USB, USB_U2P3_CTRL, 2) & U2P3_ENABLE,
                            u32::from(revision >= 0x5c20)
                        );
                        assert_eq!(hw.core.get(PLA, PLA_CR, 1), 0);
                        assert_eq!(hw.sram[&(SRAM_IMPEDANCE as u16)], 0x0b13);
                        assert_eq!(hw.sram[&(SRAM_LPF_CFG as u16)], 0xf70f);
                        assert_eq!(hw.sram[&(SRAM_10M_AMP1 as u16)], 0x00af);
                        assert_eq!(hw.sram[&(SRAM_10M_AMP2 as u16)], 0x0208);
                        assert_eq!(
                            hw.core.get(USB, USB_LPM_CTRL, 1),
                            FIFO_EMPTY_1FB
                                | ROK_EXIT_LPM
                                | if revision == 0x5c10 && speed < SUPER_SPEED {
                                    LPM_TIMER_500MS
                                } else {
                                    LPM_TIMER_500US
                                }
                        );
                        if matches!(revision, 0x5c20 | 0x5c30) {
                            assert_eq!(
                                hw.core.get(USB, USB_CSR_DUMMY1, 1) & DYNAMIC_BURST,
                                if dynamic { DYNAMIC_BURST } else { 0 }
                            );
                        }
                    }
                    for status in [0x0a, 0x0b, 0x06, 0x07, 0x13] {
                        model.borrow_mut().core.phy_status = status;
                        assert!(block_on(chip.carrier()).unwrap());
                        let half = matches!(status, 0x0a | 0x06);
                        {
                            let hw = model.borrow();
                            assert_eq!(
                                hw.core.get(PLA, PLA_TCR1, 2) & IFG_MASK,
                                if half { IFG_144NS } else { IFG_96NS }
                            );
                            assert_eq!(
                                hw.core.get(PLA, PLA_MAC_PWR_CTRL4, 2) & TX10MIDLE_EN,
                                if half { 0 } else { TX10MIDLE_EN }
                            );
                            assert_eq!(
                                hw.core.get(USB, USB_RX_EARLY_TIMEOUT, 2),
                                match speed {
                                    4 | 5 => 85000 / 8,
                                    3 => 250000 / 8,
                                    _ => 524280 / 8,
                                }
                            );
                            assert_eq!(
                                hw.core.get(USB, USB_RX_EARLY_SIZE, 2),
                                ((chip.receive_bytes() - 1554) / 4) as u32
                            );
                        }
                        model.borrow_mut().core.link = false;
                        assert!(!block_on(chip.carrier()).unwrap());
                        block_on(chip.stop()).unwrap();
                        model.borrow_mut().core.link = true;
                    }
                }
            }
        }
    }

    #[test]
    #[ignore = "oracle: STELA_RTL8153A_LINK_ORACLE"]
    fn unchanged_linux_8153a_link_register_effects() {
        extern crate std;
        use sha2::{Digest, Sha256};
        let trace =
            std::fs::read_to_string(std::env::var("STELA_RTL8153A_LINK_ORACLE").unwrap()).unwrap();
        assert_eq!(
            alloc::format!("{:x}", Sha256::digest(trace.as_bytes())),
            "2875157e5c95d235dec6726231dc30d0dd86c81a977ab2a46216c83b67a5f70b"
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
            block_on(chip.enable_8153a(status)).unwrap();
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
        assert_eq!(count, 80);
    }

    #[test]
    #[ignore = "oracle: STELA_RTL815X_FIRMWARE"]
    fn real_firmware_8153a_full_startup_and_link() {
        extern crate std;
        use sha2::{Digest, Sha256};
        let directory = std::env::var("STELA_RTL815X_FIRMWARE").unwrap();
        for (name, revision, _, hash) in crate::usb::ethernet::firmware::tests::FIXTURES {
            let revision = Revision(revision);
            if !revision.is_8153a() {
                continue;
            }
            let bytes = std::fs::read(std::path::Path::new(&directory).join(name)).unwrap();
            assert_eq!(alloc::format!("{:x}", Sha256::digest(&bytes)), hash);
            let firmware = Firmware::parse(&bytes, revision).unwrap();
            for speed in [3, 4, 5] {
                let model = Rc::new(RefCell::new(FirmwareModel::new(revision.0, &bytes)));
                let (mut chip, _) = block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    revision,
                    speed,
                    Some(&firmware),
                ))
                .unwrap();
                assert!(chip.initialized && block_on(chip.carrier()).unwrap());
                assert_eq!(
                    model.borrow().core.get(PLA, PLA_CR, 1) & (CR_RE | CR_TE),
                    CR_RE | CR_TE
                );
                block_on(chip.stop()).unwrap();
                assert_eq!(model.borrow().core.get(PLA, PLA_CR, 1), 0);
            }
        }
    }

    #[test]
    fn every_initialization_transfer_failure_prevents_a_ready_8153a() {
        let bytes = synthetic_upload(0x5c10);
        let firmware = Firmware::parse(&bytes, Revision(0x5c10)).unwrap();
        let model = Rc::new(RefCell::new(FirmwareModel::new(0x5c10, &bytes)));
        block_on(Chip::attach_with_firmware(
            FirmwareIo(model.clone()),
            Revision(0x5c10),
            4,
            Some(&firmware),
        ))
        .unwrap();
        let count = model.borrow().core.trace.len();
        for at in 0..count {
            let model = Rc::new(RefCell::new(FirmwareModel::new(0x5c10, &bytes)));
            model.borrow_mut().core.fail_at = Some(at);
            assert!(
                block_on(Chip::attach_with_firmware(
                    FirmwareIo(model.clone()),
                    Revision(0x5c10),
                    4,
                    Some(&firmware)
                ))
                .is_err()
            );
            assert_eq!(model.borrow().core.trace.len(), at + 1);
        }
    }

    #[test]
    fn autoload_phy_aldps_reset_and_link_list_deadlines_are_explicit() {
        for case in 0..5 {
            let model = Rc::new(RefCell::new(FirmwareModel::new(0x5c00, &[])));
            {
                let mut hw = model.borrow_mut();
                match case {
                    0 => hw.core.set(PLA, PLA_BOOT_CTRL, 2, 0),
                    1 => hw.core.set(PLA, OCP_PHY_STATUS, 2, 1),
                    2 => hw.core.set(PLA, 0xe000, 2, 0),
                    3 => hw.core.stuck_phy = true,
                    _ => hw.core.stuck_list = true,
                }
            }
            assert!(matches!(
                block_on(Chip::attach_with_firmware(
                    FirmwareIo(model),
                    Revision(0x5c00),
                    4,
                    None
                )),
                Err(BlockError::Timeout)
            ));
        }
    }
}
