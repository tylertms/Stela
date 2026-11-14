// SPDX-License-Identifier: GPL-2.0-only
//! Checked SuperSpeed hub descriptors, capabilities and route construction.
use super::*;

pub(super) const SS_HUB_DESCRIPTOR: u8 = 0x2a;
pub(super) const SS_HUB_BYTES: usize = 12;
pub(super) const BOS_DESCRIPTOR: u8 = 15;
const DEVICE_CAPABILITY: u8 = 16;
const SSP_CAPABILITY: u8 = 10;

pub(super) struct Descriptor {
    pub ports: u8,
    pub power_good_us: u64,
    pub controller_ma: u16,
}
impl Descriptor {
    pub fn new(bytes: &[u8], endpoint: &Endpoint) -> Result<Self, BlockError> {
        if bytes.len() != SS_HUB_BYTES
            || bytes[0] != SS_HUB_BYTES as u8
            || bytes[1] != SS_HUB_DESCRIPTOR
            || !(1..=15).contains(&bytes[2])
        {
            return Err(BlockError::Unsupported("invalid SuperSpeed hub descriptor"));
        }
        let characteristics = u16::from_le_bytes([bytes[3], bytes[4]]);
        if characteristics & !0x1f != 0
            || characteristics & 3 == 3
            || characteristics & 0x18 == 0x18
        {
            return Err(BlockError::Unsupported(
                "reserved SuperSpeed hub characteristics",
            ));
        }
        let notification = usize::from(bytes[2] + 1).div_ceil(8);
        if usize::from(endpoint.max_packet & 0x7ff) < notification
            || endpoint
                .companion
                .is_none_or(|companion| usize::from(companion.bytes_per_interval) < notification)
        {
            return Err(BlockError::Unsupported(
                "SuperSpeed hub status bitmap exceeds endpoint",
            ));
        }
        Ok(Self {
            ports: bytes[2],
            controller_ma: u16::from(bytes[6]),
            // Pinned hub_power_on_good_delay: every non-root hub gets >=100ms.
            power_good_us: (u64::from(bytes[5]) * 2000).max(100_000),
        })
    }
}

/// `get_hub_descriptor` for USB2: the reply needs only DeviceRemovable,
/// whose bit zero is the hub's. PortPwrCtrlMask and bLength are not checked.
pub(in crate::usb::xhci) struct Usb2Descriptor {
    pub ports: u8,
    pub power_good_us: u64,
    pub controller_ma: u16,
    pub tt_think: u8,
}
impl Usb2Descriptor {
    /// sizeof(struct usb_hub_descriptor), the length Linux requests.
    pub const REQUEST: u16 = 15;
    /// USB_DT_HUB_NONVAR_SIZE + 2: a shorter reply is read again.
    pub const MINIMUM: usize = 9;
    pub fn new(bytes: &[u8], endpoint: &Endpoint) -> Result<Self, BlockError> {
        if bytes.len() < Self::MINIMUM
            || bytes[1] != HUB_DESCRIPTOR
            || !(1..=MAX_HUB_PORTS).contains(&bytes[2])
        {
            return Err(BlockError::Unsupported("invalid USB2 hub descriptor"));
        }
        if bytes.len() < 7 + usize::from(bytes[2]) / 8 + 1 {
            return Err(BlockError::Unsupported(
                "USB2 hub descriptor lacks DeviceRemovable",
            ));
        }
        let characteristics = u16::from_le_bytes([bytes[3], bytes[4]]);
        if characteristics & !0xff != 0
            || characteristics & 3 == 3
            || characteristics & 0x18 == 0x18
        {
            return Err(BlockError::Unsupported("reserved USB2 hub characteristics"));
        }
        if usize::from(endpoint.max_packet & 0x7ff) < usize::from(bytes[2] + 1).div_ceil(8) {
            return Err(BlockError::Unsupported(
                "USB2 hub status bitmap exceeds endpoint",
            ));
        }
        Ok(Self {
            ports: bytes[2],
            power_good_us: (u64::from(bytes[5]) * 2000).max(100_000),
            controller_ma: u16::from(bytes[6]),
            tt_think: (characteristics >> 5 & 3) as u8,
        })
    }
}

pub(super) fn topology(attachment: Attachment) -> Result<(), BlockError> {
    if attachment.root_port == 0
        || attachment.tier >= MAX_TIERS
        || attachment.translator.is_some()
        || !matches!(attachment.speed, SUPER_SPEED | SUPER_SPEED_PLUS)
        || attachment.route >> (4 * attachment.tier) != 0
        || (0..attachment.tier).any(|tier| attachment.route >> (tier * 4) & 15 == 0)
    {
        return Err(BlockError::Unsupported(
            "unsupported SuperSpeed hub route or translator",
        ));
    }
    Ok(())
}

pub(in super::super) fn child(
    parent_slot: u8,
    parent: Attachment,
    port: u8,
    speed: u8,
) -> Result<Attachment, BlockError> {
    topology(parent)?;
    if !(1..=15).contains(&port) || !matches!(speed, SUPER_SPEED | SUPER_SPEED_PLUS) {
        return Err(BlockError::Unsupported(
            "invalid SuperSpeed child port/speed",
        ));
    }
    Ok(Attachment {
        root_port: parent.root_port,
        route: parent.route | u32::from(port) << (4 * parent.tier),
        tier: parent.tier + 1,
        speed,
        translator: None,
        parent: Some((parent_slot, port)),
    })
}

/// BOS sublink attributes, kept for the port-status speed IDs.
#[derive(Clone)]
pub(in super::super) struct Ssp {
    attributes: Vec<u32>,
}
impl Ssp {
    pub fn bos(bytes: &[u8]) -> Result<Option<Self>, BlockError> {
        let invalid = || BlockError::Unsupported("invalid SuperSpeed hub BOS capabilities");
        if bytes.len() < 5
            || bytes[0] != 5
            || bytes[1] != BOS_DESCRIPTOR
            || usize::from(u16::from_le_bytes([bytes[2], bytes[3]])) != bytes.len()
        {
            return Err(invalid());
        }
        let mut at = 5;
        let mut count = 0;
        let mut ssp = None;
        while at < bytes.len() {
            let length = usize::from(bytes[at]);
            if length < 3 || at + length > bytes.len() || bytes[at + 1] != DEVICE_CAPABILITY {
                return Err(invalid());
            }
            if bytes[at + 2] == SSP_CAPABILITY {
                if ssp.is_some() || length < 12 || bytes[at + 3] != 0 {
                    return Err(invalid());
                }
                let flags = u32::from_le_bytes(bytes[at + 4..at + 8].try_into().unwrap());
                let entries = (flags & 31) as usize + 1;
                if flags & !0x1ff != 0
                    || length != 12 + 4 * entries
                    || bytes[at + 10..at + 12] != [0, 0]
                {
                    return Err(invalid());
                }
                let mut attributes = Vec::<u32>::new();
                for word in bytes[at + 12..at + length].as_chunks::<4>().0 {
                    let attr = u32::from_le_bytes(*word);
                    if attr & 0xff00_3f00 != 0 || attr >> 14 & 3 > 1 || attr >> 6 & 1 != 0 {
                        return Err(invalid());
                    }
                    if attributes.iter().any(|old| *old & 0xcf == attr & 0xcf) {
                        return Err(invalid());
                    }
                    attributes.push(attr);
                }
                let ids = (0u16..16)
                    .filter(|id| attributes.iter().any(|attr| attr & 15 == u32::from(*id)))
                    .count();
                if ids != ((flags >> 5 & 15) + 1) as usize {
                    return Err(invalid());
                }
                ssp = Some(Self { attributes });
            }
            count += 1;
            at += length;
        }
        if count != usize::from(bytes[4]) {
            return Err(invalid());
        }
        Ok(ssp)
    }

    fn lane(&self, id: u8, tx: bool) -> Result<(u32, u32), BlockError> {
        let attr = self
            .attributes
            .iter()
            .find(|attr| {
                **attr & 15 == u32::from(id) && (**attr >> 6 & 3) == if tx { 2 } else { 0 }
            })
            .ok_or(BlockError::Unsupported(
                "hub port has unknown/asymmetric sublink speed",
            ))?;
        let mut mantissa = attr >> 16 & 255;
        for _ in (attr >> 4 & 3)..3 {
            mantissa /= 1000;
        }
        Ok((attr >> 14 & 3, mantissa))
    }

    pub fn speed(&self, status: u32) -> Result<u8, BlockError> {
        if status & !0xffff != 0 {
            return Err(BlockError::Unsupported("reserved hub extended port status"));
        }
        let receive = self.lane((status & 15) as u8, false)?;
        let transmit = self.lane((status >> 4 & 15) as u8, true)?;
        let rx = (status >> 8 & 15) + 1;
        let tx = (status >> 12 & 15) + 1;
        if receive != transmit || rx != tx {
            return Err(BlockError::Unsupported("asymmetric hub port link"));
        }
        match (receive, rx) {
            ((0, 5), 1) => Ok(SUPER_SPEED),
            ((1, rate), 1 | 2) if rate >= 10 => Ok(SUPER_SPEED_PLUS),
            ((1, rate), 2) if rate >= 5 => Ok(SUPER_SPEED_PLUS),
            _ => Err(BlockError::Unsupported(
                "unsupported hub port sublink rate/lanes",
            )),
        }
    }
}
