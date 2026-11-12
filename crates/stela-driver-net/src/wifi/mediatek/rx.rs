// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Bounded ordinary raw-802.11 receive descriptors. These are observations,
//! not association/key-generation, authenticated integrity or replay receipts.
use super::identity::Chip;
use crate::{
    bytes::get_le32,
    wifi::{Bandwidth, Encoding, RxRate},
};
use alloc::vec::Vec;
use core::ops::Range;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Truncated,
    TooLarge,
    WrongPacketType,
    UnsupportedBandIndex,
    UnsupportedTranslation,
    InvalidGroupLayout,
    AmsduError,
    MaximumLengthError,
    InvalidHeaderLength,
    UnsupportedFragment,
    /// A P-RXV rate `mt7925_mac_fill_rx_rate` refuses; Linux drops the frame.
    InvalidRate,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek RX descriptor: {self:?}")
    }
}
impl core::error::Error for Error {}

/// Raw hardware security observations. Even `decrypted` plus no ICV
/// error does not establish a current installed key or authenticated integrity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Security {
    pub mode: u8,
    pub key_index: u8,
    pub cipher_mismatch: bool,
    pub cipher_length_mismatch: bool,
    pub icv_error: bool,
    pub tkip_mic_error: bool,
    pub security_done: bool,
    pub decrypted: bool,
    pub iv_stripped: bool,
    pub mic_stripped: bool,
    /// Source status.iv order: most significant PN byte first. Only supplied
    /// when the decrypted cipher uses PN and group 1 is present.
    pub packet_number_be: Option<[u8; 6]>,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Band {
    Ghz2,
    Ghz5,
    Ghz6,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Channel {
    pub encoded: u8,
    pub band: Band,
    /// Source encoding decoded without its u8 wrap. Regulatory/channel policy
    /// must validate this observed channel before admitting a packet.
    pub number: u16,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Amsdu {
    pub raw_format: u8,
    pub first: bool,
    pub last: bool,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Signal {
    /// Raw RCPI is held even when source dBm conversion is nonnegative.
    pub rcpi: [u8; 4],
    pub dbm: [i8; 4],
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Groups {
    pub four: Option<Range<usize>>,
    pub one: Option<Range<usize>>,
    pub two: Option<Range<usize>>,
    pub three: Option<Range<usize>>,
    pub five: Option<Range<usize>>,
}

pub struct Decoded<'a> {
    pub chip: Chip,
    pub raw: &'a [u8],
    pub base: [u32; 8],
    pub base_words: u8,
    pub declared_length: u16,
    pub groups: Groups,
    pub wcid: u16,
    pub unicast_to_me: bool,
    pub channel: Channel,
    pub security: Security,
    pub fcs_error: bool,
    pub only_monitor: bool,
    pub timestamp_low: Option<u32>,
    /// Raw NON_AMPDU observation; source aggregate details additionally require
    /// the group-2 timestamp. No reorder/aggregate identifier is invented.
    pub ampdu: bool,
    pub fragment: bool,
    pub amsdu: Option<Amsdu>,
    pub signal: Option<Signal>,
    /// Full source group-3/5 bytes remain available through groups/raw; these
    /// first raw P-RXV words fit the canonical compact rate envelope.
    pub rxv: Option<[u32; 4]>,
    /// The MT7925 P-RXV rate. MT7921/MT7922 vectors are left undecoded: their
    /// layout is `mt76_connac2_mac_fill_rx_rate`'s, which is not implemented.
    pub rate: Option<RxRate>,
    pub header_gap: usize,
    /// Ordinary MPDU bytes after RXD groups and hardware header offset;
    /// A-MSDU still contains the device's two-byte pad after its MAC header.
    pub payload: &'a [u8],
}
fn word(bytes: &[u8], at: usize) -> Result<u32, Error> {
    get_le32(bytes, at).ok_or(Error::Truncated)
}
fn group(at: &mut usize, bytes: usize, total: usize) -> Result<Range<usize>, Error> {
    let end = at
        .checked_add(bytes)
        .filter(|&end| end < total)
        .ok_or(Error::Truncated)?;
    let range = *at..end;
    *at = end;
    Ok(range)
}

/// Decode only the source ordinary raw-802.11 profile. Header translation
/// requires a reverse codec and is explicitly refused. Bounds are
/// checked before every optional group read, including malformed combinations.
/// Caller keeps completed DMA/drop/association provenance separately.
pub fn decode(chip: Chip, bytes: &[u8]) -> Result<Decoded<'_>, Error> {
    if bytes.len() > 16 * 1024 {
        return Err(Error::TooLarge);
    }
    let connac3 = chip == Chip::Mt7925;
    let base_words = if connac3 { 8 } else { 6 };
    let mut base = [0; 8];
    for (i, value) in base.iter_mut().enumerate().take(base_words) {
        *value = word(bytes, i * 4)?;
    }
    let [r0, r1, r2, r3, r4, ..] = base;
    let kind = (r0 >> 27) & 31;
    let normal_mcu = kind == 7 && (r0 >> 16) & 15 == 1;
    let normal_sw = connac3 && ((r0 >> 16) & 0x380f) == 0x3801;
    if kind != 2 && !normal_mcu && !normal_sw {
        return Err(Error::WrongPacketType);
    }
    if !connac3 && r1 & (1 << 28) != 0 {
        return Err(Error::UnsupportedBandIndex);
    }
    if r2 & (1 << 23) != 0 {
        return Err(Error::AmsduError);
    }
    if r2 & (1 << 24) != 0 {
        return Err(Error::MaximumLengthError);
    }
    if r2 & (1 << if connac3 { 7 } else { 13 }) != 0 {
        return Err(Error::UnsupportedTranslation);
    }
    let group_bits = (r1 >> if connac3 { 16 } else { 11 }) & 31;
    if group_bits & 16 != 0 && group_bits & 4 == 0 {
        return Err(Error::InvalidGroupLayout);
    }
    let mut at = base_words * 4;
    let four = if group_bits & 8 != 0 {
        Some(group(&mut at, 16, bytes.len())?)
    } else {
        None
    };
    let one = if group_bits & 1 != 0 {
        Some(group(&mut at, 16, bytes.len())?)
    } else {
        None
    };
    let two = if group_bits & 2 != 0 {
        Some(group(&mut at, if connac3 { 16 } else { 8 }, bytes.len())?)
    } else {
        None
    };
    let three = if group_bits & 4 != 0 {
        Some(group(&mut at, if connac3 { 16 } else { 8 }, bytes.len())?)
    } else {
        None
    };
    let five = if group_bits & 16 != 0 {
        Some(group(&mut at, if connac3 { 96 } else { 72 }, bytes.len())?)
    } else {
        None
    };
    let padding = if connac3 {
        (r2 >> 13) & 7
    } else {
        (r2 >> 14) & 3
    } as usize
        * 2;
    let header_gap = at
        .checked_add(padding)
        .filter(|&gap| gap < bytes.len())
        .ok_or(Error::Truncated)?;
    let mode = if connac3 {
        (r2 >> 16) & 31
    } else {
        (r1 >> 16) & 31
    } as u8;
    let mismatch = r1 & (1 << 23) != 0;
    let length_mismatch = r1 & (1 << 24) != 0;
    let decrypted = mode != 0 && !mismatch && !length_mismatch;
    let pn_cipher = matches!(mode, 2 | 3 | 4 | 9 | 10 | 11 | 12);
    let packet_number_be = if decrypted && pn_cipher {
        one.as_ref()
            .map(|range| core::array::from_fn(|i| bytes[range.start + 5 - i]))
    } else {
        None
    };
    let security = Security {
        mode,
        key_index: ((r1 >> 21) & 3) as u8,
        cipher_mismatch: mismatch,
        cipher_length_mismatch: length_mismatch,
        icv_error: r1 & (1 << 25) != 0,
        tkip_mic_error: r1 & (1 << 26) != 0,
        security_done: r1 & (1 << 31) != 0,
        decrypted,
        iv_stripped: decrypted,
        mic_stripped: decrypted,
        packet_number_be,
    };
    let timestamp_low = two
        .as_ref()
        .map(|range| word(bytes, range.start))
        .transpose()?;
    let rxv = three
        .as_ref()
        .map(|range| {
            let mut values = [0; 4];
            for (i, value) in values
                .iter_mut()
                .enumerate()
                .take(if connac3 { 4 } else { 2 })
            {
                *value = word(bytes, range.start + i * 4)?;
            }
            Ok(values)
        })
        .transpose()?;
    let signal = three
        .as_ref()
        .map(|range| {
            let at = if connac3 {
                range.start + 12
            } else {
                five.as_ref().map_or(range.start + 4, |r| r.start + 24)
            };
            let raw = word(bytes, at)?;
            let rcpi = raw.to_le_bytes();
            // Source unsigned FIELD_GET/subtract/divide then s8 narrowing:
            // odd negative values round down, so signed (rcpi-220)/2 is incorrect.
            let dbm = rcpi.map(|value| (i16::from(value / 2) - 110) as i8);
            Ok(Signal { rcpi, dbm })
        })
        .transpose()?;
    let encoded = ((r3 >> 8) & 255) as u8;
    let channel = if encoded > 180 {
        Channel {
            encoded,
            band: Band::Ghz6,
            number: u16::from(encoded - 181) * 4 + 1,
        }
    } else if encoded > 14 {
        Channel {
            encoded,
            band: Band::Ghz5,
            number: u16::from(encoded),
        }
    } else {
        Channel {
            encoded,
            band: Band::Ghz2,
            number: u16::from(encoded),
        }
    };
    let rate = match rxv {
        Some(words) if connac3 => Some(rate(&words, channel.band)?),
        _ => None,
    };
    let amsdu = match (r4 & 3) as u8 {
        0 => None,
        raw_format => Some(Amsdu {
            raw_format,
            first: raw_format == 3,
            last: raw_format == 1,
        }),
    };
    Ok(Decoded {
        chip,
        raw: bytes,
        base,
        base_words: base_words as u8,
        declared_length: r0 as u16,
        groups: Groups {
            four,
            one,
            two,
            three,
            five,
        },
        wcid: (r1 & if connac3 { 0xfff } else { 0x3ff }) as u16,
        unicast_to_me: (r3 >> 16) & 3 == 1,
        channel,
        security,
        fcs_error: if connac3 {
            r3 & (1 << 24) != 0
        } else {
            r1 & (1 << 27) != 0
        },
        only_monitor: security.icv_error,
        timestamp_low,
        ampdu: r2 & (1 << 30) == 0,
        fragment: r2 & (1 << 27) != 0,
        amsdu,
        signal,
        rxv,
        rate,
        header_gap,
        payload: &bytes[header_gap..],
    })
}

/// `mt76_rates` hardware values, CCK then OFDM, as `mt76_get_rate` searches
/// them. The 5 and 6 GHz bands hold only the OFDM ones.
pub(crate) const LEGACY_RATES: [u8; 12] = [0, 1, 2, 3, 11, 15, 10, 14, 9, 13, 8, 12];

/// Linux `mt76_get_rate`: the band's bitrate index, or 0 when none matches.
fn legacy_index(band: Band, index: u8, cck: bool) -> u8 {
    let (rates, start, index) = match (band, cck) {
        // Bit 2 selects the short preamble.
        (Band::Ghz2, true) => (&LEGACY_RATES[..], 0, index & !4),
        (Band::Ghz2, false) => (&LEGACY_RATES[..], 4, index),
        (_, true) => return 0,
        (_, false) => (&LEGACY_RATES[4..], 0, index),
    };
    rates
        .iter()
        .skip(start)
        .position(|&rate| rate == index)
        .map_or(0, |at| (start + at) as u8)
}

/// `legacy_index` for a band as the shared radio status names it.
pub(crate) fn legacy_rate_index(band: crate::wifi::Band, index: u8, cck: bool) -> u8 {
    use crate::wifi::Band as Radio;
    let band = match band {
        Radio::Ghz2 => Band::Ghz2,
        Radio::Ghz5 => Band::Ghz5,
        Radio::Ghz6 => Band::Ghz6,
    };
    legacy_index(band, index, cck)
}

/// Linux `mt7925_mac_fill_rx_rate` over P-RXV words 0 and 2.
fn rate(rxv: &[u32; 4], band: Band) -> Result<RxRate, Error> {
    let (v0, v2) = (rxv[0], rxv[2]);
    let index = (v0 & 0x7f) as u8;
    let nss = ((v0 >> 7) & 15) as u8 + 1;
    let gi = ((v2 >> 3) & 3) as u8;
    let mode = ((v2 >> 11) & 15) as u8;
    let mut rate = RxRate {
        index,
        stbc: ((v2 >> 9) & 3) as u8,
        ..RxRate::default()
    };
    match mode {
        // CCK and OFDM.
        0 | 1 => rate.index = legacy_index(band, index, mode == 0),
        // HT and HT greenfield.
        2 | 3 if index <= 31 => rate.encoding = Encoding::Ht,
        4 if index <= 11 => {
            rate.encoding = Encoding::Vht;
            rate.nss = nss;
        }
        // HE SU, extended-range SU, trigger-based and MU.
        8..=11 => {
            rate.encoding = Encoding::He;
            rate.nss = nss;
            rate.index = index & 15;
            rate.gi = if gi <= 2 { gi } else { 0 };
            rate.he_dcm = v2 & (1 << 5) != 0;
        }
        // EHT SU, trigger-based and MU.
        13..=15 => {
            rate.encoding = Encoding::Eht;
            rate.nss = nss;
            rate.index = index & 15;
            rate.gi = if gi <= 2 { gi } else { 0 };
        }
        _ => return Err(Error::InvalidRate),
    }
    rate.bandwidth = match v2 & 7 {
        0 => Bandwidth::Mhz20,
        // Linux tests the mode against HE_EXT_SU bitwise, and the raw rate
        // field's ER_SU_106T bit.
        1 if mode & 9 != 0 && index & (1 << 5) != 0 => Bandwidth::HeRu106,
        1 => Bandwidth::Mhz40,
        2 => Bandwidth::Mhz80,
        3 => Bandwidth::Mhz160,
        // The vector reports 320 MHz in two positions.
        4 | 5 => Bandwidth::Mhz320,
        _ => return Err(Error::InvalidRate),
    };
    rate.short_gi = mode < 8 && gi != 0;
    Ok(rate)
}
impl Decoded<'_> {
    /// A fragment mac80211 can put back together: in the clear, or under the
    /// CCMP whose header `mt7925_mac_fill_rx` restores for it.
    pub fn rejoinable_fragment(&self) -> bool {
        self.fragment
            && self.amsdu.is_none()
            && (!self.security.decrypted || matches!(self.security.mode, 4 | 10))
    }

    /// `mt76_insert_ccmp_hdr` on a [`Self::rejoinable_fragment`]: a fragment's
    /// packet number is checked against the last, so its header comes back.
    pub fn normalize_fragment(&self, header_bytes: usize) -> Result<Vec<u8>, Error> {
        if !self.rejoinable_fragment() {
            return Err(Error::UnsupportedFragment);
        }
        if !(10..=62).contains(&header_bytes)
            || !header_bytes.is_multiple_of(2)
            || header_bytes > self.payload.len()
        {
            return Err(Error::InvalidHeaderLength);
        }
        let mut frame = self.payload.to_vec();
        if let Some([pn5, pn4, pn3, pn2, pn1, pn0]) = self.security.packet_number_be {
            let key = 0x20 | self.security.key_index << 6;
            let header = [pn0, pn1, 0, key, pn2, pn3, pn4, pn5];
            frame.splice(header_bytes..header_bytes, header);
        }
        Ok(frame)
    }

    /// `header_bytes` must come from the shared 802.11 header parser
    /// over this payload, not RXD's translated-header hint. It is independently
    /// bounded here before removing exactly the source A-MSDU two-byte pad.
    /// This does not reorder A-MSDU/AMPDU, reinsert fragment crypto headers
    /// (`normalize_fragment` does), authenticate security metadata or
    /// associate it with an installed key.
    pub fn normalize(&self, header_bytes: usize) -> Result<Vec<u8>, Error> {
        if self.fragment {
            return Err(Error::UnsupportedFragment);
        }
        if !(10..=62).contains(&header_bytes)
            || !header_bytes.is_multiple_of(2)
            || header_bytes > self.payload.len()
        {
            return Err(Error::InvalidHeaderLength);
        }
        if self.amsdu.is_none() {
            return Ok(self.payload.to_vec());
        }
        if header_bytes
            .checked_add(2)
            .is_none_or(|n| n > self.payload.len())
        {
            return Err(Error::Truncated);
        }
        let mut normalized = Vec::with_capacity(self.payload.len() - 2);
        normalized.extend_from_slice(&self.payload[..header_bytes]);
        normalized.extend_from_slice(&self.payload[header_bytes + 2..]);
        Ok(normalized)
    }
}

#[cfg(test)]
mod tests;
