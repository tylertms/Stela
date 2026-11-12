// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! RTL8153B 08/09: independent r8153b_init/hw_phy_cfg/up, efuse calibration
//! and bank-specific upload/PHY power-cut handling. C explicitly shares the
//! PHY routine; its initialization/up remains separate. 8156 never enters here.
use super::super::firmware::Firmware;
use super::*;

impl<C: Control> Chip<C> {
    pub(super) async fn u1u2_8153b(&mut self, enable: bool) -> Result<(), BlockError> {
        self.modify(
            USB,
            USB_LPM_CONFIG,
            2,
            LPM_U1U2_EN,
            if enable { LPM_U1U2_EN } else { 0 },
        )
        .await
    }

    pub(super) async fn hardware_phy_8153b(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        let misc = self.read(USB, USB_MISC_0, 2).await?;
        if misc & PCUT_STATUS != 0 {
            self.write(USB, USB_MISC_0, 2, misc & !PCUT_STATUS).await?;
        }
        self.aldps(false).await?;
        self.modify(PLA, PLA_EEE_CR, 2, EEE_RX_EN | EEE_TX_EN, 0)
            .await?;
        self.phy_modify(OCP_EEE_CFG, EEE10_EN, 0).await?;
        self.phy_write(OCP_EEE_ADV, 0).await?;
        self.write(USB, USB_U1U2_TIMER, 2, 500).await?;
        let state = self.phy_state(None).await?;
        let power_cut = matches!(state, PHY_STAT_PWRDN | PHY_STAT_EXT_INIT);
        if let Some(firmware) = firmware {
            self.upload_firmware(firmware, power_cut).await?;
        }
        if power_cut {
            self.phy_modify(MII_BMCR, BMCR_PDOWN, 0).await?;
        }
        // Linux init selects GREEN_ETHERNET for 08/09; these are its
        // per-speed analogue settings, not the older A impedance sequence.
        self.sram_write(0x8045, 0).await?;
        self.sram_write(0x804d, 0x1222).await?;
        self.sram_write(0x805d, 0x0022).await?;
        let green = self.sram_read(SRAM_GREEN_CFG as u16).await?;
        self.sram_write(SRAM_GREEN_CFG as u16, green | GREEN_ETH_EN as u16)
            .await?;
        let green = self.sram_read(SRAM_GREEN_CFG as u16).await?;
        self.sram_write(SRAM_GREEN_CFG as u16, green | R_TUNE_EN as u16)
            .await?;
        self.phy_modify(OCP_NCTL_CFG, 0, PGA_RETURN_EN).await?;
        // Preserve the adapter's efuse calibration. Bit3 in the 17-bit
        // efuse word is dummy; all-one calibration means no programmed value.
        self.write(PLA, PLA_EFUSE_CMD, 2, EFUSE_READ_CMD | 0x7d)
            .await?;
        let high = (self.read(PLA, PLA_EFUSE_CMD, 2).await? & EFUSE_DATA_BIT16) << 9;
        let fuse = high | self.read(PLA, PLA_EFUSE_DATA, 2).await?;
        let offset = ((fuse & 0x1fff0) >> 1) | (fuse & 7);
        if offset != 0xffff {
            self.phy_write(OCP_ADC_IOFFSET, offset).await?;
        }
        let mapped = self.phy_address(0xc426).await?;
        let counter = self.read(PLA, mapped, 2).await? & 0x3fff;
        if let Some(timer) = 16_000_000u32.checked_div(counter) {
            self.modify(
                USB,
                USB_UPS_CFG,
                2,
                SAW_CNT_1MS_MASK,
                timer & SAW_CNT_1MS_MASK,
            )
            .await?;
        }
        self.modify(PLA, PLA_PHY_PWR, 2, 0, PFM_PWM_SWITCH).await?;
        self.patch_request(true, true).await?;
        self.phy_modify(OCP_POWER_CFG, 0, EEE_CLKDIV_EN).await?;
        self.phy_modify(
            OCP_DOWN_SPEED,
            0,
            EN_EEE_CMODE | EN_EEE_1000 | EN_10M_CLKDIV,
        )
        .await?;
        self.phy_write(OCP_SYSCLK_CFG, 0).await?;
        self.phy_write(OCP_SYSCLK_CFG, 5 << 4).await?;
        self.patch_request(false, true).await?;
        self.aldps(true).await?;
        self.phy_modify(MII_ADVERTISE, 0x01e0, ADVERTISE_10_100_PAUSE)
            .await?;
        self.phy_modify(OCP_BASE_MII + 18, 0x0300, 0x0200).await?;
        self.phy_write(MII_BMCR, BMCR_ANENABLE_RESTART | BMCR_RESET)
            .await?;
        let mapped = self.phy_address(MII_BMCR).await?;
        self.minimum_wait(20_000).await?;
        self.poll(PLA, mapped, 2, BMCR_RESET, 0, (50, 20_000)).await
    }

    pub(super) async fn initialize_8153b(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        self.u1u2_8153b(false).await?;
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
        // B keeps PHASE2_EN when disabling power-cut (A clears it).
        self.modify(USB, USB_POWER_CUT, 2, PWR_EN, 0).await?;
        self.modify(USB, USB_MISC_0, 2, PCUT_STATUS, 0).await?;
        self.modify(USB, USB_POWER_CUT, 1, UPS_EN | USP_PREWAKE, 0)
            .await?;
        self.modify(USB, USB_MISC_2, 1, UPS_FORCE_PWR_DOWN, 0)
            .await?;
        // A newly observed power-cut requires the donor's PHY reload before
        // continuing normal init; no optimistic reuse of the prior PHY state.
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
        self.queue_wake_off().await?;
        // Preserve wake filters; this host never arms runtime USB suspend.
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
        if self.usb_speed >= SUPER_SPEED {
            self.u1u2_8153b(true).await?;
        }
        self.modify(PLA, PLA_MAC_PWR_CTRL2, 2, 0, MAC_CLK_SPDWN_EN)
            .await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL3, 2, PLA_MCU_SPDWN_EN, 0)
            .await?;
        if self.revision.0 == 0x6010 && self.read(PLA, 0xdc00, 1).await? & (1 << 5) != 0 {
            self.modify(PLA, PLA_PHY_PWR, 2, 0, TEST_IO_OFF).await?;
        }
        self.modify(USB, USB_USB_CTRL, 2, RX_AGG_DISABLE | RX_ZERO_EN, 0)
            .await?;
        self.modify(PLA, PLA_RSTTALLY, 2, 0, TALLY_RESET).await?;
        self.hardware_phy_8153b(firmware).await?;
        // rtl8153b_up uses exactly r8153_first_init, while C has different FIFO.
        self.u1u2_8153b(false).await?;
        self.u2p3(false).await?;
        self.aldps(false).await?;
        self.first_init_8153().await?;
        self.write(USB, USB_RX_BUF_TH, 4, RX_THR_B).await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL3, 2, PLA_MCU_SPDWN_EN, 0)
            .await?;
        self.aldps(true).await?;
        if self.usb_speed >= SUPER_SPEED {
            self.u1u2_8153b(true).await?;
        }
        Ok(())
    }

    pub(super) async fn enable_8153b(&mut self, status: u32) -> Result<(), BlockError> {
        self.write(USB, USB_RX_EARLY_TIMEOUT, 2, 128 / 8).await?;
        self.write(USB, USB_RX_EXTRA_AGGR_TMR, 2, 15000 / 8).await?;
        self.write(
            USB,
            USB_RX_EARLY_SIZE,
            2,
            ((self.receive_bytes() - 1554) / 8) as u32,
        )
        .await?;
        let half = status & (PHYSTATUS_10 | 0x08) != 0 && status & 1 == 0;
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
        if matches!(self.revision.0, 0x6010 | 0x6400) {
            self.modify(USB, USB_FW_TASK, 2, FC_PATCH_TASK, 0).await?;
            self.minimum_wait(1000).await?;
            self.modify(USB, USB_FW_TASK, 2, 0, FC_PATCH_TASK).await?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
