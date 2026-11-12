// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2011-2013 ASIX
//! EEPROM/efuse validity and UA1/UA2 LED/PHY page programming.
use super::*;
impl<C: Control> Chip<C> {
    async fn srom(&mut self, word: u8) -> Result<u16, BlockError> {
        self.mac_write(AX_SROM_ADDR, 1, u16::from(word)).await?;
        self.mac_write(AX_SROM_CMD, 1, EEP_RD).await?;
        for _ in 0..100 {
            if self.mac_read(AX_SROM_CMD, 1).await? & EEP_BUSY == 0 {
                return self.mac_read(AX_SROM_DATA_LOW, 2).await;
            }
            self.io.delay_us(1000).await;
        }
        Err(self.timeout())
    }
    pub(super) async fn eeprom_valid(&mut self) -> Result<bool, BlockError> {
        let mut bytes = [0u8; 12];
        for i in 0..6 {
            bytes[i * 2..i * 2 + 2].copy_from_slice(&self.srom(i as u8).await?.to_le_bytes());
            if i == 0 && bytes[0] == 0xff {
                return Ok(false);
            }
        }
        let sum = bytes[6..10]
            .iter()
            .map(|value| u16::from(*value))
            .sum::<u16>();
        Ok((sum >> 8) + (sum & 0xff) + u16::from(bytes[10]) == 0xff)
    }
    pub(super) async fn efuse_led(&mut self) -> Result<Option<u16>, BlockError> {
        let bytes = self.read(AX_ACCESS_EFUS, 0, 64, 64).await?;
        if bytes[0] == 0xff {
            return Ok(None);
        }
        let mut sum = bytes.iter().map(|value| u16::from(*value)).sum::<u16>();
        while sum > 255 {
            sum = (sum & 0xff) + (sum >> 8);
        }
        Ok((sum == 0xff).then(|| (u16::from(bytes[51]) << 8) | u16::from(bytes[52])))
    }
    pub(super) async fn led_setting(&mut self) -> Result<(), BlockError> {
        self.general_status = self.mac_read(GENERAL_STATUS, 1).await? as u8;
        if u16::from(self.general_status) & AX_SECLD == 0 {
            self.mac_write(
                AX_GPIO_CTRL,
                1,
                AX_GPIO_CTRL_GPIO3EN | AX_GPIO_CTRL_GPIO2EN | AX_GPIO_CTRL_GPIO1EN,
            )
            .await?;
        }
        let value = if self.eeprom_valid().await? {
            Some(self.srom(EEPROM_LED_MODE).await?)
        } else {
            self.efuse_led().await?
        };
        let led = match value {
            Some(value) if value != 0xffff && value & LED_VALID != 0 => value,
            _ => old_led((self.eeprom(EEPROM_LEGACY_LED).await? >> 8) as u8),
        };
        self.phy_write(GMII_PHY_PAGE_SELECT, GMII_PHY_PGSEL_EXT)
            .await?;
        self.phy_write(GMII_PHYPAGE, 0x2c).await?;
        let active = self.phy_read(GMII_LED_ACT).await?;
        let link = self.phy_read(GMII_LED_LINK).await?;
        let (active, link, duplex) = led_masks(led, active, link);
        self.phy_write(GMII_LED_ACT, active).await?;
        self.phy_write(GMII_LED_LINK, link).await?;
        self.phy_write(GMII_PHY_PAGE_SELECT, GMII_PHY_PGSEL_PAGE0)
            .await?;
        self.mac_write(AX_LEDCTRL, 1, u16::from(duplex)).await
    }
}

pub(super) fn old_led(mode: u8) -> u16 {
    match mode {
        0xfe => LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED_VALID,
        0xfd => LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED2_LINK_10 | LED_VALID,
        0xfc => {
            LED0_ACTIVE
                | LED1_ACTIVE
                | LED1_LINK_1000
                | LED2_ACTIVE
                | LED2_LINK_100
                | LED2_LINK_10
                | LED_VALID
        }
        _ => {
            LED0_ACTIVE
                | LED1_LINK_10
                | LED1_LINK_100
                | LED1_LINK_1000
                | LED2_ACTIVE
                | LED2_LINK_10
                | LED2_LINK_100
                | LED2_LINK_1000
                | LED_VALID
        }
    }
}

pub(super) fn led_masks(value: u16, active: u16, link: u16) -> (u16, u16, u8) {
    let mut active = active & GMII_LED_ACTIVE_MASK;
    let mut link = link & GMII_LED_LINK_MASK;
    for (enabled, bit) in [
        (LED0_ACTIVE, GMII_LED0_ACTIVE),
        (LED1_ACTIVE, GMII_LED1_ACTIVE),
        (LED2_ACTIVE, GMII_LED2_ACTIVE),
    ] {
        if value & enabled != 0 {
            active |= bit;
        }
    }
    for (enabled, bit) in [
        (LED0_LINK_10, GMII_LED0_LINK_10),
        (LED1_LINK_10, GMII_LED1_LINK_10),
        (LED2_LINK_10, GMII_LED2_LINK_10),
        (LED0_LINK_100, GMII_LED0_LINK_100),
        (LED1_LINK_100, GMII_LED1_LINK_100),
        (LED2_LINK_100, GMII_LED2_LINK_100),
        (LED0_LINK_1000, GMII_LED0_LINK_1000),
        (LED1_LINK_1000, GMII_LED1_LINK_1000),
        (LED2_LINK_1000, GMII_LED2_LINK_1000),
    ] {
        if value & enabled != 0 {
            link |= bit;
        }
    }
    let mut duplex = 0;
    for (fd, usb, a, b) in [
        (LED0_FD, LED0_USB3_MASK, 1, 2),
        (LED1_FD, LED1_USB3_MASK, 4, 8),
        (LED2_FD, LED2_USB3_MASK, 16, 32),
    ] {
        duplex |= if value & fd != 0 {
            a
        } else if value & usb == 0 {
            b
        } else {
            0
        };
    }
    (active, link, duplex)
}
