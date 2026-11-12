// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Ordinary raw-802.11 PCIe TXD construction, adapted from Connac2/MT7925.
//! Input metadata must come from the shared validated MPDU and installed
//! firmware station/rate context. Encoding does not install keys, allocate a
//! token, publish DMA or claim the hardware transmitted an indicated sequence.
use super::identity::Chip;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    InvalidLength,
    InvalidHeader,
    InvalidStation,
    InvalidVif,
    InvalidTid,
    InvalidSequence,
    InvalidRate,
    MissingRate,
    UnsupportedFragment,
    UnsupportedFrame,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek TXWI: {self:?}")
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Kind {
    Management,
    Control,
    Data,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Sequence {
    /// Source ordinary path: hardware assigns the air sequence. Host MPDU
    /// bytes alone must not be reported as the transmitted sequence.
    Hardware,
    /// Source injected path: SN_VALID and the explicit 12-bit sequence are set.
    Explicit(u16),
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Fragment {
    None,
    First,
    Middle,
    Last,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Queue {
    Data { access_category: u8 },
    Management,
    Beacon,
    Discovery,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Vif {
    pub omac: u8,
    pub wmm: u8,
    /// Firmware hardware-band index, not the radio frequency band.
    pub band: u8,
    pub mld: bool,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FixedRate {
    /// Source PHY-encoded legacy rate word, selected from negotiated
    /// basic rates. This helper never guesses a channel/rate or substitutes zero.
    Connac2(u16),
    /// Configured firmware basic/multicast/beacon rate table index.
    Connac3(u8),
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Frame {
    pub bytes: usize,
    pub header_bytes: u8,
    pub kind: Kind,
    pub subtype: u8,
    pub tid: u8,
    pub multicast: bool,
    pub fragment: Fragment,
    pub sequence: Sequence,
    /// Ordinary management or data only. Action frames require body
    /// validation below; BIP/MMIE/translated 802.3 and monitor radiotap
    /// policy remain outside this explicitly bounded profile.
    pub minimum_rate: bool,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    pub chip: Chip,
    pub wcid: u16,
    pub vif: Option<Vif>,
    pub phy_band_one: bool,
    pub queue: Queue,
    pub pid: u8,
    pub frame: Frame,
    /// A current acknowledged hardware-key context must exist at
    /// publication. This wire flag neither proves encryption nor owns a PN.
    pub protect: bool,
    pub no_ack: bool,
    pub hardware_amsdu: bool,
    /// Used only when frame type/multicast/minimum_rate selects the source
    /// fixed-rate path; providing this context alone never forces a rate.
    pub fixed_rate: Option<FixedRate>,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum QualifiedAction {
    /// A Block Ack action and its code; an ADDBA request also names the
    /// TID its capability field requests.
    BlockAck(u8, u8),
    SaQuery,
    /// Any other unprotected action, ordinary management to
    /// `mt7925_mac_write_txwi_80211`.
    Other,
}

pub fn encode(c: Config) -> Result<[u8; 32], Error> {
    encode_inner(c, None)
}

/// Body-checked Block Ack, protected unicast SA Query and other action
/// path; only a robust category may be protected.
/// Metadata-only Action encoding remains refused: the real category/action bytes select the donor
/// ADDBA-request TID independently of ordinary response/DELBA management.
/// This constructs no agreement, radio permission, protection or air receipt.
pub(in crate::wifi::mediatek) fn encode_mpdu(c: Config, bytes: &[u8]) -> Result<[u8; 32], Error> {
    if c.frame.kind != Kind::Management || c.frame.subtype != 13 {
        return encode(c);
    }
    let h = crate::wifi::frame::Header::parse(bytes).map_err(|_| Error::InvalidHeader)?;
    if h.fragmented()
        || h.control & 0x00fc != 0x00d0
        || h.length != 24
        || usize::from(c.frame.header_bytes) != h.length
        || c.frame.bytes != bytes.len()
    {
        return Err(Error::UnsupportedFrame);
    }
    let body = bytes.get(h.length..).ok_or(Error::InvalidHeader)?;
    if c.protect != h.protected() {
        return Err(Error::UnsupportedFrame);
    }
    if c.protect
        && (h.receiver.is_multicast()
            || h.control & 0x0300 != 0
            || c.frame.multicast
            || c.queue != Queue::Management)
    {
        return Err(Error::UnsupportedFrame);
    }
    if c.protect && body.first() == Some(&8) {
        if body.len() != 4 || body[1] > 1 {
            return Err(Error::UnsupportedFrame);
        }
        return encode_inner(c, Some(QualifiedAction::SaQuery));
    }
    match body.first() {
        Some(3) => {}
        // Only a robust category may be protected (`_ieee80211_is_robust_mgmt_frame`).
        Some(category)
            if !c.protect || !matches!(category, 4 | 7 | 11 | 15 | 20 | 21 | 22 | 127) =>
        {
            return encode_inner(c, Some(QualifiedAction::Other));
        }
        _ => return Err(Error::UnsupportedFrame),
    }
    let action = *body.get(1).ok_or(Error::InvalidHeader)?;
    let minimum = match action {
        0 | 1 => 9,
        2 => 6,
        _ => return Err(Error::UnsupportedFrame),
    };
    if body.len() < minimum {
        return Err(Error::InvalidHeader);
    }
    // The ADDBA request's TID, from its Block Ack Parameter Set.
    let tid = if action == 0 {
        ((u16::from_le_bytes([body[3], body[4]]) >> 2) & 15) as u8
    } else {
        0
    };
    encode_inner(c, Some(QualifiedAction::BlockAck(action, tid)))
}

fn encode_inner(c: Config, action: Option<QualifiedAction>) -> Result<[u8; 32], Error> {
    let f = c.frame;
    let connac3 = c.chip == Chip::Mt7925;
    // BAR derives TID/start sequence from its body; Action can select the
    // ADDBA-specific TID/flag path. This bounded metadata has neither body.
    if (f.kind == Kind::Control && f.subtype == 8)
        || (f.kind == Kind::Management && f.subtype == 13 && action.is_none())
    {
        return Err(Error::UnsupportedFrame);
    }
    if c.wcid >= 20 {
        return Err(Error::InvalidStation);
    }
    if f.bytes
        .checked_add(32)
        .is_none_or(|n| n > u16::MAX as usize)
    {
        return Err(Error::InvalidLength);
    }
    if f.header_bytes < 10
        || f.header_bytes > 62
        || !f.header_bytes.is_multiple_of(2)
        || usize::from(f.header_bytes) > f.bytes
        || f.subtype > 15
    {
        return Err(Error::InvalidHeader);
    }
    if f.tid > 15 || (!connac3 && f.tid > 7) {
        return Err(Error::InvalidTid);
    }
    if let Sequence::Explicit(n) = f.sequence
        && n > 4095
    {
        return Err(Error::InvalidSequence);
    }
    if connac3 && f.fragment != Fragment::None {
        return Err(Error::UnsupportedFragment);
    }
    if let Some(v) = c.vif
        && (v.omac > 63 || v.wmm > 3 || (v.band > 3 && v.band != 255))
    {
        return Err(Error::InvalidVif);
    }
    let vif = c.vif.unwrap_or(Vif {
        omac: 0,
        wmm: 0,
        band: 0,
        mld: false,
    });
    if (connac3 && c.phy_band_one) || (!connac3 && vif.mld) {
        return Err(Error::InvalidVif);
    }
    if !connac3 && vif.band > 1 {
        return Err(Error::InvalidVif);
    }
    let (format, queue) = match c.queue {
        Queue::Data { access_category } if access_category < 4 => {
            (0, u32::from(vif.wmm) * 4 + 3 - u32::from(access_category))
        }
        Queue::Data { .. } => return Err(Error::InvalidVif),
        Queue::Management => (0, 16),
        Queue::Beacon => (3, 18),
        Queue::Discovery => (3, 16),
    };
    let kind = match f.kind {
        Kind::Management => 0,
        Kind::Control => 1,
        Kind::Data => 2,
    };
    let fixed = f.kind != Kind::Data || f.multicast || f.minimum_rate;
    let mut words = [0u32; 8];
    words[0] = (f.bytes + 32) as u32 | (format << 23) | (queue << 25);
    words[1] = u32::from(c.wcid);
    words[2] = (kind << 4) | u32::from(f.subtype);
    words[3] = 15 << 11 | u32::from(c.protect) << 1 | u32::from(c.no_ack);
    words[5] = u32::from(c.pid) | if c.pid >= 3 { 1 << 10 } else { 0 };
    let mut amsdu = c.hardware_amsdu && c.pid < 3;
    if connac3 {
        // pinned connac3 tx_mgnt_type::{MT_TX_NORMAL=0,MT_TX_ADDBA=2}.
        let tid = if matches!(action, Some(QualifiedAction::BlockAck(0, _))) {
            2
        } else if f.kind == Kind::Management {
            0
        } else {
            f.tid
        };
        words[1] |= u32::from(vif.omac) << 25
            | u32::from(tid) << 21
            | u32::from(f.header_bytes / 2) << 16
            | 2 << 14;
        if vif.band != 0 && vif.band != 255 {
            words[1] |= u32::from(vif.band) << 12;
        }
        words[3] |= u32::from(f.multicast) << 4;
        if c.pid >= 3 {
            words[3] |= 1 << 28;
        }
        words[6] = (1 << 2) | (1 << 4);
        if c.vif.is_some() && (!vif.mld || (16..=18).contains(&queue)) {
            words[6] |= 1 << 3;
        }
        if fixed {
            words[1] |= 1 << 31;
        }
    } else {
        // mt76_connac2_mac_write_txwi_80211: an ADDBA request asks for the
        // session (MT_TXD5_ADD_BA) and carries its TID.
        let tid = match action {
            Some(QualifiedAction::BlockAck(0, tid)) => {
                words[5] |= 1 << 14;
                tid
            }
            _ => f.tid,
        };
        if tid > 7 {
            return Err(Error::InvalidTid);
        }
        words[1] |= 1 << 31
            | u32::from(vif.omac) << 24
            | u32::from(tid) << 20
            | 2 << 16
            | u32::from(f.header_bytes / 2) << 11;
        if c.phy_band_one || vif.band != 0 {
            words[1] |= 1 << 30;
        }
        words[2] |= u32::from(f.multicast) << 10;
        let frag = match f.fragment {
            Fragment::None => 0,
            Fragment::First => 1,
            Fragment::Middle => 2,
            Fragment::Last => 3,
        };
        words[2] |= frag << 14;
        words[7] = kind << 20 | u32::from(f.subtype) << 16;
        if fixed {
            words[2] |= 1 << 31;
        }
    }
    if f.kind == Kind::Management && f.subtype == 8 {
        words[3] |= 31 << 11;
    }
    if let Sequence::Explicit(seq) = f.sequence {
        words[3] |= 1 << 31 | u32::from(seq) << 16;
        amsdu = false;
    }
    if fixed {
        match (connac3, c.fixed_rate) {
            (false, Some(FixedRate::Connac2(rate))) if rate < 0x4000 => {
                words[2] |= 1 << 13;
                words[6] = (1 << 2) | (u32::from(rate) << 16);
            }
            (true, Some(FixedRate::Connac3(index))) if index < 64 => {
                words[6] |= u32::from(index) << 16;
            }
            (_, None) => return Err(Error::MissingRate),
            _ => return Err(Error::InvalidRate),
        }
        words[3] |= 1 << 28;
        // Connac2 clears HW_AMSDU on fixed-rate frames. Normal MT7925's
        // source keeps its bit unless PID/status or explicit sequence clears it.
        if !connac3 {
            amsdu = false;
        }
    }
    if amsdu {
        if connac3 {
            words[3] |= 1 << 5;
        } else {
            words[7] |= 1 << 10;
        }
    }
    let mut bytes = [0u8; 32];
    for (i, word) in words.into_iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests;
