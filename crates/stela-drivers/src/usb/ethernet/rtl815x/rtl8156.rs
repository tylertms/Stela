// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! Donor-shared RTL8156 power/up/enable with distinct A/B PHY callbacks.
use super::super::firmware::Firmware;
use super::*;

impl<C: Control> Chip<C> {
    pub(super) async fn sram_modify(
        &mut self,
        address: u16,
        clear: u16,
        set: u16,
    ) -> Result<(), BlockError> {
        let value = self.sram_read(address).await?;
        self.phy_write(OCP_SRAM_DATA, u32::from((value & !clear) | set))
            .await
    }
    async fn sram2_read(&mut self, address: u16) -> Result<u16, BlockError> {
        self.phy_write(OCP_SRAM2_ADDR, u32::from(address)).await?;
        let mapped = self.phy_address(OCP_SRAM2_DATA).await?;
        Ok(self.read(PLA, mapped, 2).await? as u16)
    }
    pub(super) async fn sram2_write(&mut self, address: u16, value: u16) -> Result<(), BlockError> {
        self.phy_write(OCP_SRAM2_ADDR, u32::from(address)).await?;
        self.phy_write(OCP_SRAM2_DATA, u32::from(value)).await
    }
    pub(super) async fn sram2_modify(
        &mut self,
        address: u16,
        clear: u16,
        set: u16,
    ) -> Result<(), BlockError> {
        let value = self.sram2_read(address).await?;
        self.phy_write(OCP_SRAM2_DATA, u32::from((value & !clear) | set))
            .await
    }
    pub(super) async fn phy_read(&mut self, address: u32) -> Result<u32, BlockError> {
        let mapped = self.phy_address(address).await?;
        self.read(PLA, mapped, 2).await
    }

    async fn mdi_swap_8156a11(&mut self) -> Result<(), BlockError> {
        if self.read(USB, USB_UPS_CFG, 2).await? & MID_REVERSE == 0
            || self.phy_read(0xd068).await? & (1 << 1) == 0
        {
            return Ok(());
        }
        let mut data = (self.phy_read(0xd068).await? & !0x1f) | 1;
        self.phy_write(0xd068, data).await?;
        let mut swap_a = self.phy_read(0xd06a).await?;
        data = (data & !0x18) | 0x18;
        self.phy_write(0xd068, data).await?;
        let mut swap_b = self.phy_read(0xd06a).await?;
        data &= !0x18;
        self.phy_write(0xd068, data).await?;
        self.phy_write(0xd06a, (swap_a & !0x7ff) | (swap_b & 0x7ff))
            .await?;
        data |= 0x18;
        self.phy_write(0xd068, data).await?;
        self.phy_write(0xd06a, (swap_b & !0x7ff) | (swap_a & 0x7ff))
            .await?;
        data = (data & !0x18) | 8;
        self.phy_write(0xd068, data).await?;
        swap_a = self.phy_read(0xd06a).await?;
        data = (data & !0x18) | 0x10;
        self.phy_write(0xd068, data).await?;
        swap_b = self.phy_read(0xd06a).await?;
        data = (data & !0x18) | 8;
        self.phy_write(0xd068, data).await?;
        self.phy_write(0xd06a, (swap_a & !0x7ff) | (swap_b & 0x7ff))
            .await?;
        data = (data & !0x18) | 0x10;
        self.phy_write(0xd068, data).await?;
        self.phy_write(0xd06a, (swap_b & !0x7ff) | (swap_a & 0x7ff))
            .await?;
        for (a, b) in [(0xbd5a, 0xbd5c), (0xbc18, 0xbc1a)] {
            swap_a = self.phy_read(a).await?;
            swap_b = self.phy_read(b).await?;
            self.phy_write(
                a,
                (swap_a & !0x1f1f) | ((swap_b & 0x1f) << 8) | ((swap_b >> 8) & 0x1f),
            )
            .await?;
            self.phy_write(
                b,
                (swap_b & !0x1f1f) | ((swap_a & 0x1f) << 8) | ((swap_a >> 8) & 0x1f),
            )
            .await?;
        }
        Ok(())
    }

    async fn hardware_phy_8156a(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        let misc = self.read(USB, USB_MISC_0, 2).await?;
        if misc & PCUT_STATUS != 0 {
            self.write(USB, USB_MISC_0, 2, misc & !PCUT_STATUS).await?;
        }
        let state = self.phy_state(None).await?;
        if let Some(firmware) = firmware {
            self.upload_firmware(firmware, state == PHY_STAT_EXT_INIT)
                .await?;
        }
        if state == PHY_STAT_EXT_INIT {
            self.phy_modify(0xa468, (1 << 3) | (1 << 1), 0).await?;
        }
        self.aldps(false).await?;
        self.modify(PLA, PLA_EEE_CR, 2, EEE_RX_EN | EEE_TX_EN, 0)
            .await?;
        self.phy_modify(OCP_EEE_CFG, EEE10_EN, 0).await?;
        self.phy_write(OCP_EEE_ADV, 0).await?;
        self.phy_modify(OCP_EEE_ADV2, MDIO_EEE_2500_5000, 0).await?;
        self.phy_state(Some(PHY_STAT_LAN_ON)).await?;
        self.modify(PLA, PLA_PHY_PWR, 2, 0, PFM_PWM_SWITCH).await?;
        match self.revision.0 {
            0x7020 => self.tune_8156a10().await?,
            0x7030 => {
                self.tune_8156a11().await?;
                self.mdi_swap_8156a11().await?;
                self.phy_modify(OCP_INTR_EN, 0, INTR_SPEED_FORCE).await?;
            }
            _ => return Err(BlockError::Unsupported("RTL8156A exact analogue revision")),
        }
        self.sram_modify(SRAM_GREEN_CFG as u16, 0, GREEN_ETH_EN as u16)
            .await?;
        self.phy_modify(0xa428, 1 << 9, 0).await?;
        self.phy_modify(0xa5ea, 1, 0).await?;
        self.aldps(true).await?;
        self.phy_modify(MII_ADVERTISE, 0, 0x0c00).await?;
        self.u2p3(true).await
    }

    async fn hardware_phy_8156(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        if self.revision.is_8156a() {
            self.hardware_phy_8156a(firmware).await
        } else if self.revision.is_8156b() {
            self.hardware_phy_8156b(firmware).await
        } else {
            Err(BlockError::Unsupported("RTL8156 exact PHY callback"))
        }
    }

    async fn autoneg_8156(&mut self) -> Result<(), BlockError> {
        self.phy_modify(MII_ADVERTISE, 0x01e0, ADVERTISE_10_100_PAUSE)
            .await?;
        self.phy_modify(OCP_BASE_MII + 18, 0x0300, 0x0200).await?;
        // rtl8152_probe advertises 2.5G only at SuperSpeed/Plus. USB2 remains
        // gigabit-advertising even though the same PHY could negotiate 2.5G.
        if self.revision.supports_2500() {
            self.phy_modify(
                OCP_10GBT_CTRL,
                MDIO_ADV_MULTIGIG_MASK,
                if self.usb_speed >= SUPER_SPEED {
                    MDIO_ADV_2500
                } else {
                    0
                },
            )
            .await?;
        }
        self.phy_write(MII_BMCR, BMCR_ANENABLE_RESTART | BMCR_RESET)
            .await?;
        let mapped = self.phy_address(MII_BMCR).await?;
        self.minimum_wait(20_000).await?;
        self.poll(PLA, mapped, 2, BMCR_RESET, 0, (50, 20_000)).await
    }

    pub(super) async fn initialize_8156(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        self.modify(USB, USB_ECM_OP, 1, EN_ALL_SPEED, 0).await?;
        self.write(USB, USB_SPEED_OPTION, 2, 0).await?;
        self.modify(USB, USB_ECM_OPTION, 2, 0, BYPASS_MAC_RESET)
            .await?;
        if self.revision.is_8156b() {
            self.modify(USB, USB_U2P3_CTRL, 2, 0, RX_DETECT8).await?;
        }
        self.u1u2_8153b(false).await?;
        if matches!(self.revision.0, 0x7410 | 0x7420) {
            self.wait_flash_8156b().await?;
        }
        self.poll(
            PLA,
            PLA_BOOT_CTRL,
            2,
            AUTOLOAD_DONE,
            AUTOLOAD_DONE,
            (500, 20_000),
        )
        .await?;
        if self.phy_state(None).await? == PHY_STAT_EXT_INIT {
            self.phy_modify(0xa468, (1 << 3) | (1 << 1), 0).await?;
            if self.revision.is_8156b() {
                self.phy_modify(0xa466, 1, 0).await?;
            }
        }
        let mapped = self.phy_address(MII_BMCR).await?;
        let bmcr = self.read(PLA, mapped, 2).await?;
        if bmcr & BMCR_PDOWN != 0 {
            self.write(PLA, mapped, 2, bmcr & !BMCR_PDOWN).await?;
        }
        self.phy_state(Some(PHY_STAT_LAN_ON)).await?;
        self.u2p3(false).await?;
        self.write(USB, USB_MSC_TIMER, 2, 0x0fff).await?;
        self.write(USB, USB_U1U2_TIMER, 2, 500).await?;
        self.modify(USB, USB_POWER_CUT, 2, PWR_EN, 0).await?;
        self.modify(USB, USB_MISC_0, 2, PCUT_STATUS, 0).await?;
        self.modify(USB, USB_POWER_CUT, 1, UPS_EN | USP_PREWAKE, 0)
            .await?;
        self.modify(USB, USB_MISC_2, 1, UPS_FORCE_PWR_DOWN, 0)
            .await?;
        if matches!(self.revision.0, 0x7410 | 0x7420) {
            self.modify(USB, USB_UPHY_XTAL, 2, OOBS_POLLING, 0).await?;
        }
        if self.read(USB, USB_MISC_0, 2).await? & PCUT_STATUS != 0 {
            self.hardware_phy_8156(firmware).await?;
            self.autoneg_8156().await?;
        }
        self.queue_wake_off().await?;
        self.write(PLA, PLA_CRWECR, 1, 0xc0).await?;
        self.modify(PLA, PLA_CONFIG34, 2, LINK_OFF_WAKE_EN, 0)
            .await?;
        self.write(PLA, PLA_CRWECR, 1, 0).await?;
        if self.usb_speed >= SUPER_SPEED {
            self.u1u2_8153b(true).await?;
        }
        if self.revision.is_8156b() {
            self.modify(PLA, PLA_RCR, 2, SLOT_EN, 0).await?;
            self.modify(PLA, PLA_CPCR, 2, 0, FLOW_CTRL_EN).await?;
            self.write(USB, USB_FC_TIMER, 2, CTRL_TIMER_EN | (600 / 8))
                .await?;
            let mut control = self.read(USB, USB_FW_CTRL, 2).await?;
            if self.read(PLA, PLA_POL_GPIO_CTRL, 2).await? & DACK_DET_EN == 0 {
                control |= FLOW_CTRL_PATCH_2;
            }
            self.write(USB, USB_FW_CTRL, 2, control & !AUTO_SPEEDUP)
                .await?;
            self.modify(USB, USB_FW_TASK, 2, 0, FC_PATCH_TASK).await?;
        }
        self.write(PLA, PLA_MAC_PWR_CTRL, 2, 0x0403).await?;
        self.modify(
            PLA,
            PLA_MAC_PWR_CTRL2,
            2,
            EEE_SPDWN_RATIO_MASK,
            MAC_CLK_SPDWN_EN | 3,
        )
        .await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL3, 2, PLA_MCU_SPDWN_EN, 0)
            .await?;
        let status = self.read(PLA, PLA_PHYSTATUS, 2).await?;
        self.modify(
            PLA,
            PLA_EXTRA_STATUS,
            2,
            CUR_LINK_OK,
            POLL_LINK_CHG
                | if status & PHYSTATUS_LINK != 0 {
                    CUR_LINK_OK
                } else {
                    0
                },
        )
        .await?;
        self.modify(USB, USB_USB_CTRL, 2, RX_AGG_DISABLE | RX_ZERO_EN, 0)
            .await?;
        if self.revision.is_8156a() {
            self.modify(USB, USB_BMU_CONFIG, 1, 0, ACT_ODMA).await?;
        }
        let force = self.phy_read(0xa5b4).await?;
        if force & (1 << 15) != 0 {
            self.phy_write(0xa5b4, force & !(1 << 15)).await?;
        }
        self.modify(PLA, PLA_RSTTALLY, 2, 0, TALLY_RESET).await?;
        self.hardware_phy_8156(firmware).await?;
        self.autoneg_8156().await?;
        self.up_8156().await
    }

    pub(super) async fn flow_control_8156(&mut self) -> Result<(), BlockError> {
        self.write(PLA, PLA_RX_FIFO_FULL, 2, 512).await?;
        self.write(PLA, PLA_RX_FIFO_EMPTY, 2, 1024).await
    }

    async fn up_8156(&mut self) -> Result<(), BlockError> {
        self.u1u2_8153b(false).await?;
        self.u2p3(false).await?;
        self.aldps(false).await?;
        self.modify(PLA, PLA_MISC_1, 2, 0, RXDY_GATED_EN).await?;
        self.write(PLA, PLA_TEREDO_CFG, 1, 0xff).await?;
        self.write(PLA, PLA_WDT6_CTRL, 2, WDT6_SET_MODE).await?;
        self.write(PLA, PLA_REALWOW_TIMER, 2, 0).await?;
        self.write(PLA, PLA_TEREDO_TIMER, 4, 0).await?;
        self.modify(PLA, PLA_RCR, 4, RCR_ACPT_ALL, 0).await?;
        self.nic_reset().await?;
        self.reset_bmu().await?;
        self.modify(PLA, PLA_OOB_CTRL, 1, NOW_IS_OOB, 0).await?;
        self.modify(PLA, PLA_SFF_STS_7, 2, MCU_BORW_EN, 0).await?;
        // Unlike8153 first_init,8156_up has no link-list reinitialization.
        self.modify(PLA, PLA_RCR1, 2, OUTER_VLAN | INNER_VLAN, 0)
            .await?;
        self.write(PLA, PLA_RMS, 2, 1522).await?;
        self.write(PLA, PLA_MTPS, 1, MTPS_JUMBO).await?;
        self.flow_control_8156().await?;
        self.write(PLA, PLA_TXFIFO_CTRL, 2, 512 / 64).await?;
        self.write(PLA, PLA_TXFIFO_FULL, 2, 2048 / 16).await?;
        if self.revision.is_8156a() {
            self.modify(USB, USB_BMU_CONFIG, 2, 0, ACT_ODMA).await?;
        }
        self.modify(PLA, PLA_RXFIFO_FULL, 2, RXFIFO_FULL_MASK, 8)
            .await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL3, 2, PLA_MCU_SPDWN_EN, 0)
            .await?;
        self.modify(USB, USB_SPEED_OPTION, 2, RG_PWRDN_EN | ALL_SPEED_OFF, 0)
            .await?;
        self.write(USB, USB_RX_BUF_TH, 4, 0x0060_0400).await?;
        // Wake filters were preserved throughout this non-suspending path.
        self.aldps(true).await?;
        self.u2p3(true).await?;
        if self.usb_speed >= SUPER_SPEED {
            self.u1u2_8153b(true).await?;
        }
        Ok(())
    }

    pub(super) async fn enable_8156(&mut self, status: u32) -> Result<(), BlockError> {
        if self.revision.is_8156b() {
            self.modify(USB, USB_RX_AGGR_NUM, 2, RX_AGGR_NUM_MASK, 0)
                .await?;
        }
        self.write(USB, USB_RX_EARLY_TIMEOUT, 2, 640 / 8).await?;
        self.write(USB, USB_RX_EXTRA_AGGR_TMR, 2, 15000 / 8).await?;
        self.write(
            USB,
            USB_RX_EARLY_SIZE,
            2,
            ((self.receive_bytes() - 1554) / 8) as u32,
        )
        .await?;
        let half = status & (PHYSTATUS_10 | 8) != 0 && status & 1 == 0;
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
        .await?;
        self.modify(
            PLA,
            PLA_MAC_PWR_CTRL4,
            2,
            IDLE_SPDWN_EN,
            if status & PHYSTATUS_2500 != 0 {
                0
            } else {
                IDLE_SPDWN_EN
            },
        )
        .await?;
        if self.revision.is_8156a() {
            if status & PHYSTATUS_1000 != 0 {
                self.write(PLA, PLA_EEE_TXTWSYS, 2, 0x11).await?;
            } else if status & PHYSTATUS_500 != 0 {
                self.write(PLA, PLA_EEE_TXTWSYS, 2, 0x3d).await?;
            }
        }
        if self.usb_speed == super::super::super::xhci::HIGH_SPEED {
            self.modify(
                USB,
                USB_L1_CTRL,
                2,
                0xf,
                if status & PHYSTATUS_FLOW == PHYSTATUS_FLOW {
                    0xf
                } else {
                    1
                },
            )
            .await?;
        }
        self.modify(USB, USB_FW_TASK, 2, FC_PATCH_TASK, 0).await?;
        self.minimum_wait(1000).await?;
        self.modify(USB, USB_FW_TASK, 2, 0, FC_PATCH_TASK).await
    }
}

#[cfg(test)]
mod tests;
