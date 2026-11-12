// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! RTL8156B12/13/15 PHY callback, flash-loading and selected analogue tuning.
use super::super::firmware::Firmware;
use super::*;
impl<C: Control> Chip<C> {
    pub(super) async fn wait_flash_8156b(&mut self) -> Result<(), BlockError> {
        if self.read(PLA, PLA_GPHY_CTRL, 2).await? & GPHY_FLASH != 0
            && self.read(USB, USB_GPHY_CTRL, 2).await? & BYPASS_FLASH == 0
        {
            self.poll(
                USB,
                USB_GPHY_CTRL,
                2,
                GPHY_PATCH_DONE,
                GPHY_PATCH_DONE,
                (100, 1000),
            )
            .await?;
        }
        Ok(())
    }
    pub(super) async fn hardware_phy_8156b(
        &mut self,
        firmware: Option<&Firmware<'_>>,
    ) -> Result<(), BlockError> {
        match self.revision.0 {
            0x7400 => self.pre_8156b12().await?,
            0x7410 | 0x7420 => self.wait_flash_8156b().await?,
            _ => return Err(BlockError::Unsupported("RTL8156B exact PHY callback")),
        }
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
            self.phy_modify(0xa466, 1, 0).await?;
            self.phy_modify(0xa468, (1 << 3) | (1 << 1), 0).await?;
        }
        let bmcr = self.phy_read(MII_BMCR).await?;
        if bmcr & BMCR_PDOWN != 0 {
            self.phy_write(MII_BMCR, bmcr & !BMCR_PDOWN).await?;
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
            0x7400 => self.tune_8156b12().await?,
            0x7410 => {
                self.tune_8156b13().await?;
                self.tune_8156b13_15().await?;
            }
            0x7420 => self.tune_8156b13_15().await?,
            _ => unreachable!("validated exact RTL8156B revision"),
        }
        self.phy_modify(OCP_INTR_EN, 0, INTR_SPEED_FORCE).await?;
        self.patch_request(true, true).await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL4, 2, 0, EEE_SPDWN_EN)
            .await?;
        self.phy_modify(OCP_DOWN_SPEED, EN_EEE_100 | EN_EEE_1000, EN_10M_CLKDIV)
            .await?;
        self.phy_modify(OCP_POWER_CFG, EEE_CLKDIV_EN, 0).await?;
        self.patch_request(false, true).await?;
        self.sram_modify(SRAM_GREEN_CFG as u16, 0, GREEN_ETH_EN as u16)
            .await?;
        self.phy_modify(0xa428, 1 << 9, 0).await?;
        self.phy_modify(0xa5ea, 1, 0).await?;
        self.aldps(true).await?;
        self.phy_modify(MII_ADVERTISE, 0, 0x0c00).await?;
        self.u2p3(true).await
    }
}

#[cfg(test)]
mod tests;
