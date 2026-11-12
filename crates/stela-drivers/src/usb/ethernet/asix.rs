// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2011-2013 ASIX
//! Native AX88179/178A register/PHY backend derived from pinned Linux usbnet.
//! DMA, configuration acknowledgement and endpoint quarantine stay in the
//! parent USB Ethernet transport; this module never owns controller rings.
use super::Control;
use crate::{block::BlockError, net::Mac};
use alloc::vec::Vec;
mod framing;
pub(super) use framing::{decode, encode};
mod identity;
mod led;
mod legacy;
#[cfg(test)]
pub(crate) mod model;
#[cfg(test)]
pub(crate) use legacy::model::Hardware as LegacyModel;
mod registers;
pub(super) use identity::{Identity, identify, listed};
use registers::*;

/// One register operation's setup fields and its endpoint-zero data stage.
struct RegisterRequest<'a> {
    command: u8,
    value: u16,
    index: u16,
    data: RegisterData<'a>,
}

enum RegisterData<'a> {
    Read(usize),
    Write(&'a [u8]),
}

/// Exact endpoint-zero transfers shared by both ASIX register protocols.
async fn request<C: Control>(
    io: &mut super::MinimumControl<C>,
    accessible: &mut bool,
    operation: RegisterRequest<'_>,
) -> Result<Vec<u8>, BlockError> {
    let RegisterRequest {
        command,
        value,
        index,
        data,
    } = operation;
    let (data, length, read) = match data {
        RegisterData::Read(length) => (&[][..], length, true),
        RegisterData::Write(data) => (data, data.len(), false),
    };
    if let Some(error) = io.error() {
        return Err(error.clone());
    }
    if !*accessible {
        return Err(BlockError::Unsupported("ASIX register access poisoned"));
    }
    let request = crate::usb::descriptor::setup(
        if read { 0xc0 } else { 0x40 },
        command,
        value,
        index,
        length as u16,
    );
    let result = io.control(request, data).await;
    match result {
        Ok(bytes) if bytes.len() == if read { length } else { 0 } => Ok(bytes),
        Ok(_) => {
            *accessible = false;
            Err(BlockError::Unsupported("ASIX short register transfer"))
        }
        Err(error) => {
            *accessible = false;
            Err(error)
        }
    }
}
pub(super) struct Chip<C> {
    io: super::MinimumControl<C>,
    accessible: bool,
    initialized: bool,
    status: Option<u16>,
    pub(super) general_status: u8,
    pub(super) phy_id: u32,
}

impl<C: Control> Chip<C> {
    async fn request(&mut self, operation: RegisterRequest<'_>) -> Result<Vec<u8>, BlockError> {
        request(&mut self.io, &mut self.accessible, operation).await
    }

    async fn read(
        &mut self,
        command: u8,
        value: u16,
        index: u16,
        length: usize,
    ) -> Result<Vec<u8>, BlockError> {
        self.request(RegisterRequest {
            command,
            value,
            index,
            data: RegisterData::Read(length),
        })
        .await
    }
    async fn write(
        &mut self,
        command: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<(), BlockError> {
        self.request(RegisterRequest {
            command,
            value,
            index,
            data: RegisterData::Write(data),
        })
        .await
        .map(|_| ())
    }
    async fn mac_read(&mut self, address: u16, width: usize) -> Result<u16, BlockError> {
        let bytes = self
            .read(AX_ACCESS_MAC, address, width as u16, width)
            .await?;
        Ok(if width == 1 {
            u16::from(bytes[0])
        } else {
            u16::from_le_bytes(bytes.try_into().unwrap())
        })
    }
    async fn mac_write(
        &mut self,
        address: u16,
        width: usize,
        value: u16,
    ) -> Result<(), BlockError> {
        self.write(
            AX_ACCESS_MAC,
            address,
            width as u16,
            &value.to_le_bytes()[..width],
        )
        .await
    }
    async fn phy_read(&mut self, address: u16) -> Result<u16, BlockError> {
        let bytes = self.read(AX_ACCESS_PHY, AX88179_PHY_ID, address, 2).await?;
        Ok(u16::from_le_bytes(bytes.try_into().unwrap()))
    }
    async fn phy_write(&mut self, address: u16, value: u16) -> Result<(), BlockError> {
        self.write(AX_ACCESS_PHY, AX88179_PHY_ID, address, &value.to_le_bytes())
            .await
    }
    async fn eeprom(&mut self, word: u16) -> Result<u16, BlockError> {
        let bytes = self.read(AX_ACCESS_EEPROM, word, 1, 2).await?;
        Ok(u16::from_le_bytes(bytes.try_into().unwrap()))
    }
    async fn minimum_wait(&mut self, us: u64) -> Result<(), BlockError> {
        let result = self.io.minimum_wait(us).await;
        if result.is_err() {
            self.initialized = false;
        }
        result
    }
    fn timeout(&mut self) -> BlockError {
        self.accessible = false;
        self.initialized = false;
        BlockError::Timeout
    }
    pub(super) async fn attach(io: C) -> Result<(Self, Mac), BlockError> {
        let mut chip = Self {
            io: super::MinimumControl::new(io),
            accessible: true,
            initialized: false,
            status: None,
            general_status: 0,
            phy_id: 0,
        };
        chip.mac_write(AX_PHYPWR_RSTCTL, 2, 0).await?;
        chip.mac_write(AX_PHYPWR_RSTCTL, 2, AX_PHYPWR_RSTCTL_IPRL)
            .await?;
        chip.minimum_wait(500_000).await?;
        chip.mac_write(AX_CLK_SELECT, 1, AX_CLK_SELECT_ACS | AX_CLK_SELECT_BCS)
            .await?;
        chip.minimum_wait(200_000).await?;
        let detach = chip.eeprom(EEPROM_AUTODETACH).await?;
        if detach != 0xffff && detach & EEPROM_AUTODETACH_ENABLE != 0 {
            let clock = chip.mac_read(AX_CLK_SELECT, 1).await?;
            chip.mac_write(AX_CLK_SELECT, 1, clock | AX_CLK_SELECT_ULR)
                .await?;
            let power = chip.mac_read(AX_PHYPWR_RSTCTL, 2).await?;
            chip.mac_write(AX_PHYPWR_RSTCTL, 2, power | AX_PHYPWR_RSTCTL_AT)
                .await?;
        }
        // The internal PHY's standard identification words must be readable.
        // Their complete value is held rather than replaced with a rebrand
        // assumption. UA1/UA2 handling comes from GENERAL_STATUS below.
        chip.phy_id = (u32::from(chip.phy_read(MII_PHYSID1).await?) << 16)
            | u32::from(chip.phy_read(MII_PHYSID2).await?);
        if matches!(chip.phy_id, 0 | 0xffff_ffff) {
            return Err(BlockError::Unsupported("ASIX absent internal PHY identity"));
        }
        let address = chip.read(AX_ACCESS_MAC, AX_NODE_ID, 6, 6).await?;
        let mac = Mac(address.as_slice().try_into().unwrap());
        if mac.0[0] & 1 != 0 || mac.0.iter().all(|b| *b == 0) {
            return Err(BlockError::Unsupported(
                "ASIX invalid permanent station address",
            ));
        }
        chip.write(AX_ACCESS_MAC, AX_NODE_ID, 6, &mac.0).await?;
        chip.write(AX_ACCESS_MAC, AX_RX_BULKIN_QCTRL, 5, &BULKIN[0])
            .await?;
        chip.mac_write(AX_PAUSE_WATERLVL_LOW, 1, 0x34).await?;
        chip.mac_write(AX_PAUSE_WATERLVL_HIGH, 1, 0x52).await?;
        // Native packets carry no checksum/TSO metadata. Hardware offload must
        // therefore be disabled in both directions rather than claimed complete.
        chip.mac_write(AX_RXCOE_CTL, 1, 0).await?;
        chip.mac_write(AX_TXCOE_CTL, 1, 0).await?;
        chip.mac_write(AX_RX_CTL, 2, RX_CONTROL).await?;
        chip.mac_write(
            AX_MONITOR_MOD,
            1,
            AX_MONITOR_MODE_PMETYPE | AX_MONITOR_MODE_PMEPOL | AX_MONITOR_MODE_RWMP,
        )
        .await?;
        chip.mac_write(
            AX_MEDIUM_STATUS_MODE,
            2,
            MEDIUM_BASE | AX_MEDIUM_FULL_DUPLEX | AX_MEDIUM_GIGAMODE,
        )
        .await?;
        chip.led_setting().await?;
        chip.phy_write(GMII_PHY_PAGE_SELECT, GMII_PHY_PGSEL_PAGE3)
            .await?;
        chip.phy_write(MII_PHYADDR, 0x3246).await?; // MII_PHYADDR, ax88179_disable_eee.
        chip.phy_write(GMII_PHY_PAGE_SELECT, GMII_PHY_PGSEL_PAGE0)
            .await?;
        // MMD AN EEE advertisement (ax88179_phy_write_mmd_indirect).
        chip.phy_write(MII_MMD_CTRL, MDIO_MMD_AN).await?;
        chip.phy_write(MII_MMD_DATA, MDIO_AN_EEE_ADV).await?;
        chip.phy_write(MII_MMD_CTRL, MDIO_MMD_AN | MII_MMD_CTRL_NOINCR)
            .await?;
        chip.phy_write(MII_MMD_DATA, 0).await?;
        let bmcr = chip.phy_read(MII_BMCR).await?;
        if bmcr & BMCR_ANENABLE == 0 {
            return Err(BlockError::Unsupported(
                "ASIX PHY autonegotiation disabled after reset",
            ));
        }
        chip.phy_write(MII_BMCR, bmcr | BMCR_ANRESTART).await?; // mii_nway_restart preserves other PHY controls.
        chip.initialized = true;
        Ok((chip, mac))
    }
    pub(super) fn receive_bytes(&self) -> usize {
        32 * 1024
    }
    pub(super) async fn carrier(&mut self) -> Result<bool, BlockError> {
        if let Some(error) = self.io.error() {
            return Err(error.clone());
        }
        if !self.initialized {
            return Err(BlockError::Unsupported("ASIX chip is not initialized"));
        }
        let phy = self.phy_read(GMII_PHY_PHYSR).await?;
        if phy & GMII_PHY_PHYSR_LINK == 0 {
            return Ok(false);
        }
        let selected = phy & (GMII_PHY_PHYSR_SMASK | GMII_PHY_PHYSR_FULL | GMII_PHY_PHYSR_LINK);
        if self.status == Some(selected) {
            return Ok(true);
        }
        let mut ready = false;
        for _ in 0..100 {
            self.mac_write(AX_RX_CTL, 2, 0).await?;
            self.mac_write(AX_RX_CTL, 2, RX_CONTROL).await?;
            let bytes = self
                .read(AX_FIFO_STATUS, AX_FIFO_STATUS_VALUE, 0, 4)
                .await?;
            if u32::from_le_bytes(bytes.try_into().unwrap()) & AX_TXFIFO_FULL == 0 {
                ready = true;
                break;
            }
            self.io.delay_us(1000).await;
        }
        if !ready {
            return Err(self.timeout());
        }
        let bus = self.mac_read(PHYSICAL_LINK_STATUS, 1).await?;
        let mut mode = MEDIUM_BASE;
        let queue = match phy & GMII_PHY_PHYSR_SMASK {
            GMII_PHY_PHYSR_GIGA => {
                mode |= AX_MEDIUM_GIGAMODE | AX_MEDIUM_EN_125MHZ;
                if bus & AX_USB_SS != 0 {
                    0
                } else if bus & AX_USB_HS != 0 {
                    1
                } else {
                    3
                }
            }
            GMII_PHY_PHYSR_100 => {
                mode |= AX_MEDIUM_PS;
                if bus & (AX_USB_SS | AX_USB_HS) != 0 {
                    2
                } else {
                    3
                }
            }
            0 => 3,
            _ => {
                return Err(BlockError::Unsupported(
                    "ASIX reserved negotiated PHY speed",
                ));
            }
        };
        self.write(AX_ACCESS_MAC, AX_RX_BULKIN_QCTRL, 5, &BULKIN[queue])
            .await?;
        if phy & GMII_PHY_PHYSR_FULL != 0 {
            mode |= AX_MEDIUM_FULL_DUPLEX;
        }
        self.mac_write(AX_MEDIUM_STATUS_MODE, 2, mode).await?;
        self.status = Some(selected);
        Ok(true)
    }
    pub(super) async fn stop(&mut self) -> Result<(), BlockError> {
        self.status = None;
        let mode = self.mac_read(AX_MEDIUM_STATUS_MODE, 2).await?;
        self.mac_write(AX_MEDIUM_STATUS_MODE, 2, mode & !AX_MEDIUM_RECEIVE_EN)
            .await?;
        self.mac_write(AX_RX_CTL, 2, 0).await
    }
}

const BULKIN: [[u8; 5]; 4] = [
    [7, 0x4f, 0, 0x12, 0xff],
    [7, 0x20, 3, 0x16, 0xff],
    [7, 0xae, 7, 0x18, 0xff],
    [7, 0xcc, 0x4c, 0x18, 8],
];
const RX_CONTROL: u16 = AX_RX_CTL_DROPCRCERR
    | AX_RX_CTL_IPE
    | AX_RX_CTL_START
    | AX_RX_CTL_AP
    | AX_RX_CTL_AMALL
    | AX_RX_CTL_AB;
const MEDIUM_BASE: u16 = AX_MEDIUM_RECEIVE_EN | AX_MEDIUM_TXFLOW_CTRLEN | AX_MEDIUM_RXFLOW_CTRLEN;

#[cfg(test)]
pub(super) fn legacy_ids() -> &'static [(u16, u16, Identity)] {
    identity::LEGACY_IDS
}
#[cfg(test)]
pub(super) fn ids() -> &'static [(u16, u16, Identity)] {
    identity::IDS
}

/// Protocol dispatch only; the parent owns every USB endpoint and DMA lease.
pub(super) enum Device<C> {
    Modern(Chip<C>),
    Legacy(legacy::Chip<C>),
}
impl<C: Control> Device<C> {
    pub(super) async fn attach(io: C, identity: Identity) -> Result<(Self, Mac), BlockError> {
        if let Identity::Ax88772 { eeprom_mac } = identity {
            legacy::Chip::attach(io, eeprom_mac)
                .await
                .map(|(chip, mac)| (Self::Legacy(chip), mac))
        } else {
            Chip::attach(io)
                .await
                .map(|(chip, mac)| (Self::Modern(chip), mac))
        }
    }
    pub(super) fn identity(&self) -> (u8, u32) {
        match self {
            Self::Modern(c) => (c.general_status, c.phy_id),
            Self::Legacy(c) => (c.raw_status, c.phy_id),
        }
    }
    pub(super) fn receive_bytes(&self) -> usize {
        match self {
            Self::Modern(c) => c.receive_bytes(),
            Self::Legacy(_) => 2048,
        }
    }
    pub(super) async fn carrier(&mut self) -> Result<bool, BlockError> {
        match self {
            Self::Modern(c) => c.carrier().await,
            Self::Legacy(c) => c.carrier().await,
        }
    }
    pub(super) async fn link_down(&mut self) -> Result<(), BlockError> {
        match self {
            Self::Modern(c) => c.stop().await,
            Self::Legacy(c) => c.link_down().await,
        }
    }
    pub(super) async fn stop(&mut self) -> Result<(), BlockError> {
        match self {
            Self::Modern(c) => c.stop().await,
            Self::Legacy(c) => c.stop().await,
        }
    }
    pub(super) fn encode(
        &self,
        frame: &[u8],
        output: &mut [u8],
        packet: u16,
    ) -> Result<usize, &'static str> {
        match self {
            Self::Modern(_) => encode(frame, output, packet),
            Self::Legacy(_) => legacy::framing::encode(frame, output, packet),
        }
    }
    pub(super) fn decode(&mut self, bytes: &[u8]) -> Result<Vec<Vec<u8>>, &'static str> {
        match self {
            Self::Modern(_) => decode(bytes),
            Self::Legacy(c) => c.decode(bytes),
        }
    }
}

#[cfg(test)]
mod tests;
