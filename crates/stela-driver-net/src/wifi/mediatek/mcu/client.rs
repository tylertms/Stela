// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! MT7925 ordinary non-MLO legacy20 station/BSS commands. These are the full
//! source-selected TLV sets for that explicitly negotiated profile. Encoding
//! does not establish a firmware context or grant regulatory permission.
use super::device::{Context, Requests};
use crate::{
    Mac,
    wifi::{Band, Channel},
};
use alloc::vec::Vec;

pub(in crate::wifi::mediatek) mod connac2;
pub mod modern;
pub(in crate::wifi::mediatek) mod six_ghz;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    NoMemory,
    Context(super::device::Error),
    InvalidPeer,
    InvalidWcid,
    InvalidChannel,
    InvalidScanTargets,
    InvalidScanElements,
    InvalidRates,
    InvalidTiming,
    UnsupportedCipher,
    UnsupportedWidth,
    InvalidNegotiation,
    InvalidGeometry,
    InvalidAggregation,
    /// A command of the other connac generation's layout.
    WrongChip,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MT7925 client codec: {self:?}")
    }
}
impl core::error::Error for Error {}
/// Source resource and observed legacy BSS inputs. No inferred EEPROM,
/// beacon/DTIM, antenna count, channel, rate bitmap, or policy default.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Profile {
    pub interface: Context,
    pub peer: Mac,
    pub wcid: u16,
    pub channel: Channel,
    pub aid: u16,
    pub basic_rates: u16,
    pub supported_rates: u16,
    pub beacon_interval: u16,
    pub dtim_period: u8,
    pub slot_time: u16,
    pub antenna_mask: u8,
    /// Source Connac3 cipher number: open0, CCMP1284/CCMP25610/GCMP12811/GCMP25612.
    pub cipher: u8,
}
/// One link of a station MLD's association as the MT7925's MLD records
/// name it: `mt7925_mcu_sta_mld_tlv`, `mt7925_mcu_sta_eht_mld_tlv` and
/// `mt7925_mcu_bss_mld_tlv`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MldLink {
    pub link_id: u8,
    /// The station MLD's address.
    pub local: Mac,
    /// The AP MLD's address.
    pub ap: Mac,
    /// The setup link's BSS index and AP WCID, the MLD's primary link.
    pub primary_bss: u8,
    pub primary_wcid: u16,
    /// The EML capabilities `STA_REC_EHT_MLD` takes when the AP and the
    /// station both support EMLSR, which `eml_enable` then switches on.
    pub eml: Option<u16>,
}

fn put16(bytes: &mut [u8], at: usize, v: u16) {
    bytes[at..at + 2].copy_from_slice(&v.to_le_bytes());
}
fn put32(bytes: &mut [u8], at: usize, v: u32) {
    bytes[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
fn tlv(bytes: &mut Vec<u8>, tag: u16, len: usize) -> &mut [u8] {
    let start = bytes.len();
    bytes.resize(start + len, 0);
    put16(bytes, start, tag);
    put16(bytes, start + 2, len as u16);
    &mut bytes[start..]
}
/// Store the number of TLVs that [`tlv`] appended after `start`.
fn recount(bytes: &mut [u8], start: usize) {
    let mut at = start;
    let mut count = 0;
    while at < bytes.len() {
        let len = u16::from_le_bytes([bytes[at + 2], bytes[at + 3]]) as usize;
        // Every TLV holds its own 4-byte header, so the walk always advances.
        assert!(len >= 4, "MCU TLV shorter than its header");
        at += len;
        count += 1;
    }
    assert_eq!(at, bytes.len(), "MCU TLV overruns its command");
    put16(bytes, 2, count);
}
impl Profile {
    /// Source mt792x_rx_get_wcid lookup for this single-peer namespace.
    /// This is only slot selection, never key/association or DMA permission.
    pub(in crate::wifi::mediatek) fn receive_slot(self, raw: u16, unicast: bool) -> Option<u16> {
        if raw == self.wcid {
            Some(if unicast {
                self.wcid
            } else {
                self.interface.broadcast_wcid
            })
        } else if raw == self.interface.broadcast_wcid && unicast {
            Some(raw)
        } else {
            None
        }
    }
    /// A legacy join's profile: 6 GHz associates only through the modern
    /// path. The per-BSS TLV builders take every band, as mt7925 builds them.
    pub fn validate(self) -> Result<(), Error> {
        self.validate_inner(false)
    }
    fn validate_inner(self, modern: bool) -> Result<(), Error> {
        Requests::station(self.interface, true).map_err(Error::Context)?;
        if self.peer.is_multicast() || self.peer.0 == [0; 6] {
            return Err(Error::InvalidPeer);
        }
        if self.wcid >= 14 || self.wcid == self.interface.broadcast_wcid {
            return Err(Error::InvalidWcid);
        }
        let ch = u8::try_from(self.channel.number).map_err(|_| Error::InvalidChannel)?;
        super::init::validate_channel(self.channel.band, ch, self.channel.frequency_mhz)
            .map_err(|_| Error::InvalidChannel)?;
        if self.channel.band == Band::Ghz6 && !modern {
            return Err(Error::InvalidChannel);
        }
        let mask = if self.channel.band == Band::Ghz2 {
            0xfff
        } else {
            0xff
        };
        if self.supported_rates == 0
            || self.supported_rates & !mask != 0
            || self.basic_rates == 0
            || self.basic_rates & !self.supported_rates != 0
        {
            return Err(Error::InvalidRates);
        }
        if self.aid > 2007
            || self.beacon_interval == 0
            || self.dtim_period == 0
            || !matches!(self.slot_time, 9 | 20)
            || self.antenna_mask == 0
            || self.antenna_mask & !0xf != 0
        {
            return Err(Error::InvalidTiming);
        }
        if !matches!(self.cipher, 0 | 1 | 2 | 4 | 5 | 10 | 11 | 12) {
            return Err(Error::UnsupportedCipher);
        }
        Ok(())
    }
    /// Full mt7925_mcu_sta_cmd. NONE=0 creates an unassociated peer;
    /// ASSOC=2 includes the source single-link MLD record. PORT_SECURE is the
    /// source wire state only; the shared OS controlled port stays independent.
    pub fn peer_request(self, enable: bool, associated: bool) -> Result<Vec<u8>, Error> {
        self.peer_request_inner(enable, associated, None, None)
    }
    fn peer_request_inner(
        self,
        enable: bool,
        associated: bool,
        modern: Option<&modern::Configuration>,
        link: Option<&MldLink>,
    ) -> Result<Vec<u8>, Error> {
        self.peer_request_validated(enable, associated, modern, link, modern.is_some())
    }
    fn peer_request_validated(
        self,
        enable: bool,
        associated: bool,
        modern: Option<&modern::Configuration>,
        link: Option<&MldLink>,
        allow_six: bool,
    ) -> Result<Vec<u8>, Error> {
        self.validate_inner(allow_six)?;
        let mut out = vec![0; 8];
        out[0] = self.interface.bss_index;
        out[1] = self.wcid as u8;
        out[4] = 1;
        out[5] = self.interface.omac_index;
        if !enable {
            tlv(&mut out, 0x25, 8);
            tlv(&mut out, 0x23, 4);
            put16(&mut out, 2, 2);
            return Ok(out);
        }
        let basic = tlv(&mut out, 0, 20);
        put32(basic, 4, 0x10002);
        basic[8] = 2;
        put16(basic, 10, self.aid);
        basic[12..18].copy_from_slice(&self.peer.0);
        put16(basic, 18, if associated { 1 } else { 3 });
        let phy = tlv(&mut out, 0x15, 12);
        put16(phy, 4, self.basic_rates);
        phy[6] = if self.channel.band == Band::Ghz2 {
            3
        } else {
            8
        };
        if let Some(m) = modern {
            m.peer_capabilities(self, &mut out)?;
        }
        let ra = tlv(&mut out, 1, 16);
        put16(
            ra,
            4,
            if self.channel.band == Band::Ghz2 {
                (self.supported_rates >> 4) << 6 | (self.supported_rates & 15)
            } else {
                self.supported_rates << 6
            },
        );
        if let Some(h) = modern.and_then(|m| m.negotiated().ht.as_ref()) {
            ra[6..16].copy_from_slice(&h.peer_rx_mcs);
        }
        let state = tlv(&mut out, 7, 16);
        state[4] = if associated { 2 } else { 0 };
        if let Some(m) = modern.filter(|m| m.negotiated().vht.is_some()) {
            state[12] = m.station_bandwidth() | m.negotiated().tx_nss << 4;
        }
        if associated {
            // STA_REC_MLD: the primary link first, then a secondary link's
            // own WCID and BSS.
            let (address, primary, bss) = link
                .map_or((self.peer, self.wcid, self.interface.bss_index), |link| {
                    (link.ap, link.primary_wcid, link.primary_bss)
                });
            let mld = tlv(&mut out, 0x20, 28);
            mld[4..10].copy_from_slice(&address.0);
            put16(mld, 10, primary);
            put16(mld, 14, primary);
            mld[16] = 1;
            put16(mld, 20, primary);
            mld[22] = bss;
            if self.wcid != primary {
                put16(mld, 12, self.wcid);
                mld[16] = 2;
                put16(mld, 24, self.wcid);
                mld[26] = self.interface.bss_index;
            }
            if let Some(link) = link {
                // STA_REC_EHT_MLD: EMLSR's capabilities, or STR otherwise.
                let eht = tlv(&mut out, 0x21, 16);
                eht[5] = 0xff;
                match link.eml {
                    Some(eml) => eht[10..12].copy_from_slice(&eml.to_le_bytes()),
                    None => eht[7] = 1 << 1,
                }
            }
        }
        let hdr = tlv(&mut out, 0x2b, 8);
        hdr[5] = 1;
        hdr[6] = 1;
        recount(&mut out, 8);
        Ok(out)
    }
    /// Broadcast slot uses the source no-link_sta path: only its own header
    /// translation TLV, distinct from a full peer and from WMWA raw updates.
    pub fn broadcast_request(self) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut out = vec![0; 8];
        out[0] = self.interface.bss_index;
        out[1] = self.interface.broadcast_wcid as u8;
        out[2] = 1;
        out[4] = 1;
        out[5] = 0xe;
        let h = tlv(&mut out, 0x2b, 8);
        h[5] = 1;
        h[6] = 1;
        Ok(out)
    }
    pub fn beacon_filter(self) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        let t = tlv(&mut b, 22, 12);
        put16(t, 4, self.beacon_interval);
        t[6] = self.dtim_period;
        Ok(b)
    }
    /// Linux `mt7925_mcu_uni_bss_ps`: the BSS in dynamic power save, or
    /// awake.
    pub fn power_save_request(self, enable: bool) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        let t = tlv(&mut b, 21, 8);
        t[4] = if enable { 2 } else { 0 };
        Ok(b)
    }
    /// Linux `mt7925_mcu_set_rssimonitor`: firmware RSSI events above
    /// `high` or below `low` dBm while `enable`.
    pub fn rssi_monitor_request(self, enable: bool, high: i8, low: i8) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        let t = tlv(&mut b, 0, 8);
        t[4] = u8::from(enable);
        t[5] = high as u8;
        t[6] = low as u8;
        Ok(b)
    }
    /// Linux `mt7925_mcu_set_chctx`: the RLM TLV for `definition`.
    pub fn channel_request(
        self,
        definition: &crate::wifi::station::operations::ChannelDefinition,
    ) -> Result<Vec<u8>, Error> {
        use crate::wifi::station::phy::Width;
        self.validate_inner(true)?;
        // ieee80211_frequency_to_channel
        let channel = |mhz: u32| -> u8 {
            (match self.channel.band {
                Band::Ghz2 if mhz == 2484 => 14,
                Band::Ghz2 => (mhz - 2407) / 5,
                Band::Ghz5 if mhz < 5000 => (mhz - 4000) / 5,
                Band::Ghz5 => (mhz - 5000) / 5,
                Band::Ghz6 if mhz == 5935 => 2,
                Band::Ghz6 => (mhz - 5950) / 5,
            }) as u8
        };
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        let r = tlv(&mut b, 2, 16);
        r[4] = self.channel.number as u8;
        r[5] = channel(definition.center1_mhz);
        r[7] = match definition.width {
            Width::Mhz20 => 0,
            Width::Mhz40 => 1,
            Width::Mhz80 => 2,
            Width::Mhz160 => 3,
            Width::Mhz80P80 => 6,
            Width::Mhz320 => 7,
        };
        if matches!(definition.width, Width::Mhz80P80 | Width::Mhz320) {
            r[6] = definition.center2_mhz.map_or(0, channel);
        }
        r[8] = self.antenna_mask.count_ones() as u8;
        r[9] = r[8];
        r[10] = if definition.width == Width::Mhz20 {
            0
        } else {
            4
        };
        r[11] = match r[4].cmp(&r[5]) {
            core::cmp::Ordering::Less => 1,
            core::cmp::Ordering::Greater => 3,
            core::cmp::Ordering::Equal => 0,
        };
        r[12] = match self.channel.band {
            Band::Ghz2 => 1,
            Band::Ghz5 => 2,
            Band::Ghz6 => 3,
        };
        Ok(b)
    }
    /// Linux `mt7925_mcu_set_timing`: the IFS TLV with only the slot time.
    pub fn timing(self) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        let ifs = tlv(&mut b, 23, 20);
        ifs[4] = 1;
        put16(ifs, 8, self.slot_time);
        Ok(b)
    }
    /// Source explicit always-awake native profile, without runtime/deep sleep.
    pub fn awake(self) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        tlv(&mut b, 21, 8);
        Ok(b)
    }
    pub fn disable_beacon_pm(self) -> Result<Vec<u8>, Error> {
        self.validate_inner(true)?;
        let mut b = vec![self.interface.bss_index, 0, 0, 0];
        tlv(&mut b, 27, 4);
        Ok(b)
    }
    pub fn filter_request(self, enable: bool) -> Result<super::init::Request, Error> {
        self.validate_inner(true)?;
        let mut b = vec![0; 72];
        // Primary PHY band_idx is 0 for this single-PHY profile,
        // distinct from the non-MLO interface's band selector ff.
        put16(&mut b, 4, 12);
        put16(&mut b, 6, 68);
        b[8] = 1;
        put32(&mut b, 16, 1 << 11);
        b[20] = if enable { 1 } else { 2 };
        Ok(super::init::Request::client_filter(b))
    }
    /// The profile without what the AP changes later, the timing from its
    /// beacons and the channel within the band through a switch: the fields
    /// naming one association.
    pub fn association_identity(self) -> Self {
        Self {
            slot_time: 0,
            channel: Channel {
                number: 0,
                frequency_mhz: 0,
                ..self.channel
            },
            ..self
        }
    }
    pub fn roc(self, token: u8, duration_ms: u32) -> Result<super::init::Request, Error> {
        self.roc_validated(token, duration_ms, false)
    }
    fn roc_validated(
        self,
        token: u8,
        duration_ms: u32,
        allow_six: bool,
    ) -> Result<super::init::Request, Error> {
        self.validate_inner(allow_six)?;
        if duration_ms == 0 {
            return Err(Error::InvalidTiming);
        }
        let mut b = vec![0; 28];
        put16(&mut b, 6, 24);
        b[8] = self.interface.bss_index;
        b[9] = token;
        b[10] = self.channel.number as u8;
        b[12] = match self.channel.band {
            Band::Ghz2 => 1,
            Band::Ghz5 => 2,
            Band::Ghz6 => 3,
        };
        b[14] = self.channel.number as u8;
        b[17] = self.channel.number as u8;
        put32(&mut b, 20, duration_ms);
        b[24] = 255;
        Ok(super::init::Request::roc(b))
    }
    /// `mt7925_mcu_set_mlo_roc`: one channel request for this setup link's
    /// channel and `other`'s, each at 20 MHz on its control channel, as
    /// single-radio multi-link operation with a 2.4 GHz link among them
    /// (MT7925_ROC_REQ_MLSR_AG) or not (MT7925_ROC_REQ_MLSR_AA).
    pub fn mlo_roc(
        self,
        other: Self,
        token: u8,
        duration_ms: u32,
    ) -> Result<super::init::Request, Error> {
        self.validate_inner(true)?;
        other.validate_inner(true)?;
        if duration_ms == 0 {
            return Err(Error::InvalidTiming);
        }
        let two = [self, other]
            .iter()
            .any(|link| link.channel.band == Band::Ghz2);
        let mut b = vec![0; 4];
        // UNI_ROC_ACQUIRE for the setup link, UNI_ROC_SUB_LINK for the other.
        for (link, tag) in [(self, 0), (other, 3)] {
            let roc = tlv(&mut b, tag, 24);
            roc[4] = link.interface.bss_index;
            roc[5] = token;
            roc[6] = link.channel.number as u8;
            roc[8] = match link.channel.band {
                Band::Ghz2 => 1,
                Band::Ghz5 => 2,
                Band::Ghz6 => 3,
            };
            roc[10] = link.channel.number as u8;
            roc[13] = link.channel.number as u8;
            roc[15] = if two { 10 } else { 11 };
            put32(roc, 16, duration_ms);
            roc[20] = 255;
        }
        Ok(super::init::Request::roc(b))
    }
    pub fn abort_roc(self, token: u8) -> Result<super::init::Request, Error> {
        self.abort_roc_validated(token, false)
    }
    fn abort_roc_validated(
        self,
        token: u8,
        allow_six: bool,
    ) -> Result<super::init::Request, Error> {
        self.validate_inner(allow_six)?;
        let mut b = vec![0; 16];
        put16(&mut b, 4, 1);
        put16(&mut b, 6, 12);
        b[8] = self.interface.bss_index;
        b[9] = token;
        b[10] = 255;
        Ok(super::init::Request::roc(b))
    }
    /// Full associated BSS helper for the admitted local/peer legacy profile.
    pub fn bss_request(self, enable: bool) -> Result<Vec<u8>, Error> {
        self.bss_request_inner(enable, None, None)
    }
    /// The BSS of one link of a station MLD before its association.
    pub fn link_bss_request(self, enable: bool, link: &MldLink) -> Result<Vec<u8>, Error> {
        self.bss_request_inner(enable, None, Some(link))
    }
    fn bss_request_inner(
        self,
        enable: bool,
        modern: Option<&modern::Configuration>,
        link: Option<&MldLink>,
    ) -> Result<Vec<u8>, Error> {
        self.bss_request_validated(enable, modern, link, modern.is_some())
    }
    fn bss_request_validated(
        self,
        enable: bool,
        modern: Option<&modern::Configuration>,
        link: Option<&MldLink>,
        allow_six: bool,
    ) -> Result<Vec<u8>, Error> {
        self.validate_inner(allow_six)?;
        let c = self.interface;
        let two = self.channel.band == Band::Ghz2;
        let mut out = vec![c.bss_index, 0, 0, 0];
        let b = tlv(&mut out, 0, 32);
        b[4] = 1;
        b[5] = c.omac_index;
        b[6] = if c.omac_index > 0x10 { 0 } else { c.omac_index };
        b[7] = c.band_index;
        put32(b, 8, 0x10001);
        b[12] = u8::from(!enable);
        b[13] = c.wmm_index;
        b[14..20].copy_from_slice(&self.peer.0);
        put16(b, 20, c.broadcast_wcid);
        put16(b, 22, self.beacon_interval);
        b[24] = self.dtim_period;
        b[25] = if two { 6 } else { 1 };
        put16(b, 26, self.wcid);
        put16(b, 28, if two { 1 } else { 3 });
        b[31] = c.bss_index;
        let sec = tlv(&mut out, 16, 8);
        // Source switch, including its default for CCMP256. These are
        // firmware BSS fields, never evidence of an OS SAE/RSN authentication.
        let (mode, status) = match self.cipher {
            1 | 5 => (1, 0),
            2 => (7, 4),
            4 => (7, 6),
            11 | 12 => (11, 8),
            _ => (0, 1),
        };
        sec[4] = mode;
        sec[5] = status;
        sec[6] = self.cipher;
        let rate = tlv(&mut out, 11, 16);
        put16(rate, 6, if two { 15 } else { 0x540 });
        rate[12] = u8::from(two);
        rate[13] = (if two { 14 } else { 18 }) + self.basic_rates.trailing_zeros() as u8;
        rate[14] = rate[13];
        tlv(&mut out, 15, 8);
        // UNI_BSS_INFO_MLD: an MLD link names its link, the primary link's
        // BSS and the station MLD.
        let m = tlv(&mut out, 26, 20);
        m[4] = link.map_or(255, |link| link.primary_bss);
        m[5] = c.bss_index + 32;
        m[6..12].copy_from_slice(&link.map_or(c.address, |link| link.local).0);
        m[12] = 255;
        m[13] = link.map_or(255, |link| link.link_id);
        m[14] = u8::from(link.is_some_and(|link| link.eml.is_some()));
        let ifs = tlv(&mut out, 23, 20);
        ifs[4] = 1;
        put16(ifs, 8, self.slot_time);
        if let Some(m) = modern {
            m.bss_capabilities(self, enable, &mut out)?;
        }
        if enable {
            let r = tlv(&mut out, 2, 16);
            r[4] = self.channel.number as u8;
            r[5] = r[4];
            r[8] = self.antenna_mask.count_ones() as u8;
            r[9] = r[8];
            r[12] = if two { 1 } else { 2 };
            if let Some(m) = modern {
                m.rlm(self, r);
            }
            let mbssid = tlv(&mut out, 6, 8);
            if let Some(m) = modern {
                mbssid[4] = m.operation().mbssid_indicator;
                mbssid[5] = m.operation().mbssid_index;
            }
        }
        recount(&mut out, 4);
        Ok(out)
    }
}

/// Full non-random MT7925 scan TLVs, including source RNR BSSID targeting.
/// The caller supplies already prepared probe IEs for this codec-only entry.
pub fn scan(
    context: Context,
    sequence: u8,
    request: &crate::wifi::scan::Request,
) -> Result<super::init::Request, Error> {
    scan_with_elements(context, sequence, request, None)
}
/// Native prepared per-band probe IEs. Capacity refusal precedes scan ownership;
/// the native caller must never silently lose a later band's capabilities.
pub(in crate::wifi::mediatek) fn scan_with_elements(
    context: Context,
    sequence: u8,
    request: &crate::wifi::scan::Request,
    elements: Option<&[Vec<u8>; 3]>,
) -> Result<super::init::Request, Error> {
    const SCAN_IE_BUDGET: usize = 608;
    const RNR_MAX_BSSIDS: usize = 10;
    const SCAN_FUNCTION_SPLIT: u8 = 1 << 5;
    const SCAN_FUNCTION_RNR: u8 = 1 << 3;
    if let Some(elements) = elements {
        let required: usize = elements
            .iter()
            .filter(|ie| !ie.is_empty())
            .map(|ie| 8 + ie.len())
            .sum();
        if required > SCAN_IE_BUDGET {
            return Err(Error::InvalidScanElements);
        }
    }
    Requests::station(context, true).map_err(Error::Context)?;
    // mt76_connac_mcu_build_rnr_scan_param deduplicates by BSSID, keeping the
    // first channel, and mt7925_mcu_hw_scan sends the first ten.
    let mut targets = [None; RNR_MAX_BSSIDS];
    let mut target_count = 0;
    for target in request.six_ghz() {
        if targets[..target_count]
            .iter()
            .flatten()
            .any(|&(bssid, _)| bssid == target.bssid)
        {
            continue;
        }
        if target_count == targets.len() {
            break;
        }
        let channel = request
            .channels()
            .get(target.channel_index)
            .filter(|ch| ch.band == Band::Ghz6)
            .ok_or(Error::InvalidScanTargets)?;
        let number = u8::try_from(channel.number).map_err(|_| Error::InvalidChannel)?;
        targets[target_count] = Some((target.bssid, number));
        target_count += 1;
    }
    let mut out = Vec::new();
    out.try_reserve_exact(1308).map_err(|_| Error::NoMemory)?;
    out.extend_from_slice(&[
        sequence | (context.band_index << 7),
        context.bss_index,
        0,
        0,
    ]);
    let req = tlv(&mut out, 1, 20);
    req[4] = u8::from(request.active());
    req[5] = if request.active() { 2 } else { 0 };
    req[6] = SCAN_FUNCTION_SPLIT
        | if request.six_ghz().is_empty() {
            0
        } else {
            SCAN_FUNCTION_RNR
        };
    let ssid = tlv(&mut out, 10, 368);
    let mut count = 0;
    for name in request.ssids() {
        if name.is_empty() {
            continue;
        }
        let at = 8 + count * 36;
        put32(ssid, at, name.len() as u32);
        ssid[at + 4..at + 4 + name.len()].copy_from_slice(name);
        count += 1;
    }
    ssid[4] = if count > 0 { 4 } else { 1 };
    ssid[5] = count as u8;
    if target_count == 0 {
        tlv(&mut out, 11, 16)[4..10].copy_from_slice(&[255; 6]);
    } else {
        for &(bssid, channel) in targets[..target_count].iter().flatten() {
            let bssid_tlv = tlv(&mut out, 11, 16);
            bssid_tlv[4..10].copy_from_slice(&bssid.0);
            bssid_tlv[10] = channel;
            bssid_tlv[11] = RNR_MAX_BSSIDS as u8;
            bssid_tlv[13] = RNR_MAX_BSSIDS as u8;
        }
    }
    let channels = tlv(&mut out, 12, 136);
    channels[4] = 4;
    channels[5] = request.channels().len() as u8;
    for (i, ch) in request.channels().iter().enumerate() {
        if ch.number > 255 {
            return Err(Error::InvalidChannel);
        }
        channels[8 + i * 2] = match ch.band {
            Band::Ghz2 => 1,
            Band::Ghz5 => 2,
            Band::Ghz6 => 3,
        };
        channels[9 + i * 2] = ch.number as u8;
    }
    tlv(&mut out, 14, 12);
    let mut ie_count = 0;
    if elements.is_some() || !request.extra_elements().is_empty() {
        // Full source helper shares this budget between all selected bands;
        // it stops, rather than emitting an over-budget later band.
        let mut remaining = SCAN_IE_BUDGET;
        for (index, band) in [Band::Ghz2, Band::Ghz5, Band::Ghz6].into_iter().enumerate() {
            if request.channels().iter().any(|ch| ch.band == band) {
                let bytes = elements.map_or(request.extra_elements(), |ies| ies[index].as_slice());
                if bytes.is_empty() {
                    continue;
                }
                let length = 8 + bytes.len();
                if length > remaining {
                    break;
                }
                let ie = tlv(&mut out, 13, length);
                put16(ie, 4, bytes.len() as u16);
                ie[6] = match band {
                    Band::Ghz2 => 1,
                    Band::Ghz5 => 2,
                    Band::Ghz6 => 3,
                };
                ie[8..].copy_from_slice(bytes);
                ie_count += 1;
                remaining -= length;
            }
        }
    }
    let count = 4 + target_count.max(1) + ie_count;
    put16(&mut out, 2, count as u16);
    Ok(super::init::Request::scan(out, false))
}
pub fn cancel_scan(context: Context, sequence: u8) -> super::init::Request {
    let mut b = vec![sequence, context.bss_index, 0, 0];
    tlv(&mut b, 2, 8);
    super::init::Request::scan(b, true)
}
/// UNI_SCAN_DONE's four-byte fixed header followed by source TLVs. Unknown
/// country/channel facts stay raw; they never overwrite trusted policy.
pub fn scan_done(payload: &[u8]) -> Result<bool, Error> {
    let mut rest = payload.get(4..).ok_or(Error::InvalidTiming)?;
    let mut done = false;
    while !rest.is_empty() {
        if rest.len() < 4 {
            return Err(Error::InvalidTiming);
        }
        let tag = u16::from_le_bytes([rest[0], rest[1]]);
        let len = u16::from_le_bytes([rest[2], rest[3]]) as usize;
        if len < 4 || len > rest.len() {
            return Err(Error::InvalidTiming);
        }
        done |= tag == 0;
        rest = &rest[len..];
    }
    Ok(done)
}

#[cfg(test)]
mod tests;

/// UNI_ROC event observations. Unlike the source's global boolean, the
/// native owner additionally matches exact token/BSS/channel/profile. The raw
/// status remains visible; no copied grant may authorize a different owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RocGrant {
    pub bss: u8,
    pub token: u8,
    pub status: u8,
    pub channel: u8,
    pub sco: u8,
    pub band: u8,
    pub width: u8,
    pub center: u8,
    pub center2: u8,
    pub request_type: u8,
    pub hardware_band: u8,
    pub duration_ms: u32,
}
pub fn roc_grants(payload: &[u8]) -> Result<Vec<RocGrant>, Error> {
    let mut rest = payload.get(4..).ok_or(Error::InvalidTiming)?;
    let mut out = Vec::new();
    while !rest.is_empty() {
        if rest.len() < 4 {
            return Err(Error::InvalidTiming);
        }
        let tag = u16::from_le_bytes([rest[0], rest[1]]);
        let len = u16::from_le_bytes([rest[2], rest[3]]) as usize;
        if len < 4 || len > rest.len() {
            return Err(Error::InvalidTiming);
        }
        if tag == 0 {
            if len < 20 {
                return Err(Error::InvalidTiming);
            }
            out.push(RocGrant {
                bss: rest[4],
                token: rest[5],
                status: rest[6],
                channel: rest[7],
                sco: rest[8],
                band: rest[9],
                width: rest[10],
                center: rest[11],
                center2: rest[12],
                request_type: rest[13],
                hardware_band: rest[14],
                duration_ms: u32::from_le_bytes(rest[16..20].try_into().unwrap()),
            });
        }
        rest = &rest[len..];
    }
    Ok(out)
}
