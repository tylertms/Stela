// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2014 Realtek Semiconductor Corp. All rights reserved.
//! Distinct RTL8156A10/11 analogue tuning from pinned r8156_hw_phy_cfg.
use super::*;
impl<C: Control> Chip<C> {
    pub(super) async fn tune_8156a10(&mut self) -> Result<(), BlockError> {
        self.phy_modify(0xad40, 0x3ff, 1 << 7 | 1 << 2).await?;
        self.phy_modify(0xad4e, 0, 1 << 4).await?;
        self.phy_modify(0xad16, 0x3ff, 0x6).await?;
        self.phy_modify(0xad32, 0x3f, 0x6).await?;
        self.phy_modify(0xac08, 1 << 12 | 1 << 8, 0).await?;
        self.phy_modify(0xac8a, 1 << 15, 1 << 12 | 1 << 13 | 1 << 14)
            .await?;
        self.phy_modify(0xad18, 0, 1 << 10).await?;
        self.phy_modify(0xad1a, 0, 0x3ff).await?;
        self.phy_modify(0xad1c, 0, 0x3ff).await?;
        self.sram_modify(0x80ea, 0xff00, 0xc400).await?;
        self.sram_modify(0x80eb, 0x0700, 0x0300).await?;
        self.sram_modify(0x80f8, 0xff00, 0x1c00).await?;
        self.sram_modify(0x80f1, 0xff00, 0x3000).await?;
        self.sram_modify(0x80fe, 0xff00, 0xa500).await?;
        self.sram_modify(0x8102, 0xff00, 0x5000).await?;
        self.sram_modify(0x8015, 0xff00, 0x3300).await?;
        self.sram_modify(0x8100, 0xff00, 0x7000).await?;
        self.sram_modify(0x8014, 0xff00, 0xf000).await?;
        self.sram_modify(0x8016, 0xff00, 0x6500).await?;
        self.sram_modify(0x80dc, 0xff00, 0xed00).await?;
        self.sram_modify(0x80df, 0, 1 << 8).await?;
        self.sram_modify(0x80e1, 1 << 8, 0).await?;
        self.phy_modify(0xbf06, 0x003f, 0x0038).await?;
        self.sram_write(0x819f, 0xddb6).await?;
        self.phy_write(0xbc34, 0x5555).await?;
        self.phy_modify(0xbf0a, 0x0e00, 0x0a00).await?;
        self.phy_modify(0xbd2c, 1 << 13, 0).await?;
        Ok(())
    }
    pub(super) async fn tune_8156a11(&mut self) -> Result<(), BlockError> {
        self.phy_modify(0xad16, 0, 0x3ff).await?;
        self.phy_modify(0xad32, 0x3f, 0x6).await?;
        self.phy_modify(0xac08, 1 << 12 | 1 << 8, 0).await?;
        self.phy_modify(0xacc0, 0x3, 1 << 1).await?;
        self.phy_modify(0xad40, 0xe7, 1 << 6 | 1 << 2).await?;
        self.phy_modify(0xac14, 1 << 7, 0).await?;
        self.phy_modify(0xac80, 1 << 8 | 1 << 9, 0).await?;
        self.phy_modify(0xac5e, 0x7, 1 << 1).await?;
        self.phy_write(0xad4c, 0x00a8).await?;
        self.phy_write(0xac5c, 0x01ff).await?;
        self.phy_modify(0xac8a, 0xf0, 1 << 4 | 1 << 5).await?;
        self.sram2_modify(0x8157, 0xff00, 0x0500).await?;
        self.sram2_modify(0x8159, 0xff00, 0x0700).await?;
        self.sram2_write(0x80a2, 0x0153).await?;
        self.sram2_write(0x809c, 0x0153).await?;
        self.write(PLA, PLA_EEE_TXTWSYS_2P5G, 2, 0x0056).await?;
        self.modify(PLA, PLA_USB_CFG, 2, 0, EN_XG_LIP | EN_G_LIP)
            .await?;
        self.sram_write(0x8257, 0x020f).await?;
        self.sram_write(0x80ea, 0x7843).await?;
        self.patch_request(true, true).await?;
        self.modify(PLA, PLA_MAC_PWR_CTRL4, 2, 0, EEE_SPDWN_EN)
            .await?;
        self.phy_modify(OCP_DOWN_SPEED, EN_EEE_100 | EN_EEE_1000, EN_10M_CLKDIV)
            .await?;
        self.phy_modify(OCP_POWER_CFG, EEE_CLKDIV_EN, 0).await?;
        self.phy_write(OCP_SYSCLK_CFG, 0).await?;
        self.phy_write(OCP_SYSCLK_CFG, 5 << 8).await?;
        self.patch_request(false, true).await?;
        self.phy_modify(0xd068, 0, 1 << 13).await?;
        self.sram_modify(0x81a2, 1 << 8, 0).await?;
        self.phy_modify(0xb54c, 0xff00, 0xdb00).await?;
        self.phy_modify(0xa454, 1 << 0, 0).await?;
        self.phy_modify(OCP_10GBT_CTRL, 0, RTL_ADV2_5G_F_R).await?;
        self.phy_modify(0xad4e, 1 << 4, 0).await?;
        self.phy_modify(0xa86a, 1 << 0, 0).await?;
        Ok(())
    }
}
