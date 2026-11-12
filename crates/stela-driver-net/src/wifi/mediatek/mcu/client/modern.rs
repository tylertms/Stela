// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Full MT7925 modern station/BSS/EDCA and BA requests, on one link of a
//! station MLD too.
//! Negotiated metadata encodes capabilities; only the current shared
//! association owner and matching native completions can install these bytes.
use super::{Error, MldLink, Profile, put16, put32, tlv};
use crate::wifi::{
    Band,
    station::phy::{AccessCategory, McsMaps, Negotiated, Width},
};
use alloc::vec::Vec;

/// Selected operating channel, including the second center used by
/// 80+80/320 and the disabled-subchannel bitmap. These are source channel
/// numbers, not MHz or inferred regulatory permissions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Operation {
    pub center_channel: u8,
    pub second_center_channel: u8,
    pub punctured: u16,
    pub he: Option<HeOperation>,
    pub mbssid_indicator: u8,
    pub mbssid_index: u8,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HeOperation {
    pub packet_extension: u8,
    pub rts_threshold: u16,
    pub color_enabled: bool,
    pub color: u8,
}
/// Keeps the exact validated local/peer provenance and operation. This is
/// not an association, key, radio or applied-policy receipt.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Configuration {
    negotiated: Negotiated,
    operation: Operation,
    local_he_maps: [McsMaps; 3],
    /// The BSS's operating width: the negotiated one until a channel switch.
    width: Width,
    /// The station MLD link the BSS and its AP are, whose MLD records every
    /// BSS and peer request then carries.
    link: Option<MldLink>,
}
impl Configuration {
    pub fn new(
        profile: Profile,
        negotiated: Negotiated,
        operation: Operation,
        local_he_maps: [McsMaps; 3],
    ) -> Result<Self, Error> {
        let width = negotiated.width;
        Self::checked(profile, negotiated, operation, local_he_maps, width)
    }
    /// The configuration once a channel switch moved the BSS to `operation`
    /// at `width`, no wider than negotiated, on `profile`'s channel.
    pub fn switched(
        &self,
        profile: Profile,
        operation: Operation,
        width: Width,
    ) -> Result<Self, Error> {
        let mhz = |width| match width {
            Width::Mhz20 => 20,
            Width::Mhz40 => 40,
            Width::Mhz80 => 80,
            Width::Mhz160 | Width::Mhz80P80 => 160,
            Width::Mhz320 => 320,
        };
        if mhz(width) > mhz(self.negotiated.width)
            || operation.he != self.operation.he
            || (operation.mbssid_indicator, operation.mbssid_index)
                != (self.operation.mbssid_indicator, self.operation.mbssid_index)
        {
            return Err(Error::InvalidGeometry);
        }
        Self::checked(
            profile,
            self.negotiated.clone(),
            operation,
            self.local_he_maps,
            width,
        )
        .map(|switched| Self {
            link: self.link,
            ..switched
        })
    }
    /// The configuration of one link of a station MLD.
    pub fn with_link(self, link: MldLink) -> Self {
        Self {
            link: Some(link),
            ..self
        }
    }
    pub const fn link(&self) -> Option<&MldLink> {
        self.link.as_ref()
    }
    fn checked(
        profile: Profile,
        negotiated: Negotiated,
        operation: Operation,
        local_he_maps: [McsMaps; 3],
        width: Width,
    ) -> Result<Self, Error> {
        profile.validate_inner(true)?;
        // The pinned MT7925 local advertisement includes160MHz, not80+80;
        // is_320mhz_supported() explicitly admits onlyMT7927. Generic shared
        // metadata can describe those widths without checking this device.
        if matches!(negotiated.width, Width::Mhz80P80 | Width::Mhz320) {
            return Err(Error::UnsupportedWidth);
        }
        // Negotiated's outputs are public for consumers; reject edited outputs
        // rather than accepting capabilities disconnected from their inputs.
        let checked = Negotiated::between(
            negotiated.local(),
            negotiated.peer(),
            negotiated.mode,
            negotiated.width,
        )
        .map_err(|_| Error::InvalidNegotiation)?;
        if checked != negotiated
            || negotiated.local().band() != profile.channel.band
            || negotiated.rx_nss > profile.antenna_mask.count_ones() as u8
            || negotiated.tx_nss > profile.antenna_mask.count_ones() as u8
            || (profile.channel.band != Band::Ghz6
                && (profile.basic_rates != negotiated.legacy.basic
                    || profile.supported_rates != negotiated.legacy.supported))
        {
            return Err(Error::InvalidNegotiation);
        }
        validate_geometry(profile, width, operation)?;
        if negotiated
            .local()
            .local_he_source_maps()
            .is_some_and(|maps| *maps != local_he_maps)
        {
            return Err(Error::InvalidNegotiation);
        }
        if let Some(local) = negotiated.local().he()
            && [Some(local.mcs_80), local.mcs_160, local.mcs_80p80]
                .into_iter()
                .zip(local_he_maps)
                .any(|(wire, source)| wire.is_some_and(|wire| wire != source))
        {
            return Err(Error::InvalidNegotiation);
        }
        if negotiated.he.is_some() != operation.he.is_some()
            || operation
                .he
                .is_some_and(|h| h.color > 63 || h.packet_extension > 7)
            || operation.mbssid_indicator > 8
            || u16::from(operation.mbssid_index) >= 1u16 << operation.mbssid_indicator
        {
            return Err(Error::InvalidNegotiation);
        }
        Ok(Self {
            negotiated,
            operation,
            local_he_maps,
            width,
            link: None,
        })
    }
    pub fn negotiated(&self) -> &Negotiated {
        &self.negotiated
    }
    pub const fn operation(&self) -> Operation {
        self.operation
    }
    pub fn peer_request(&self, profile: Profile, associated: bool) -> Result<Vec<u8>, Error> {
        self.matches(profile)?;
        profile.peer_request_inner(true, associated, Some(self), self.link.as_ref())
    }
    pub fn bss_request(&self, profile: Profile, enable: bool) -> Result<Vec<u8>, Error> {
        self.matches(profile)?;
        profile.bss_request_inner(enable, Some(self), self.link.as_ref())
    }
    /// The local HE MCS maps for 80, 160 and 80+80 MHz the BSS's HE record
    /// carries.
    pub(super) const fn local_he_maps(&self) -> &[McsMaps; 3] {
        &self.local_he_maps
    }
    /// The BSS's operating width.
    pub(super) const fn width(&self) -> Width {
        self.width
    }
    pub(super) fn matches(&self, profile: Profile) -> Result<(), Error> {
        // Recheck supplied resources/rates/antenna against this kept
        // metadata on every request, before any caller can publish it.
        Self::checked(
            profile,
            self.negotiated.clone(),
            self.operation,
            self.local_he_maps,
            self.width,
        )
        .map(|_| ())
    }
    pub(super) fn station_bandwidth(&self) -> u8 {
        match self.negotiated.width {
            Width::Mhz20 => 0,
            Width::Mhz40 => 1,
            Width::Mhz80 => 2,
            Width::Mhz160 | Width::Mhz80P80 => 3,
            Width::Mhz320 => 4,
        }
    }
    fn command_bandwidth(&self) -> u8 {
        match self.width {
            Width::Mhz20 => 0,
            Width::Mhz40 => 1,
            Width::Mhz80 => 2,
            Width::Mhz160 => 3,
            Width::Mhz80P80 => 6,
            Width::Mhz320 => 7,
        }
    }
    pub(super) fn peer_capabilities(&self, p: Profile, out: &mut Vec<u8>) -> Result<(), Error> {
        let n = &self.negotiated;
        // Existing full BASIC20 + PHY12 prefix. Source BASIC.qos reflects WME;
        // source sta_uapsd emits AP-only records, never station STA_APPS here.
        out[8 + 9] = u8::from(n.wmm.is_some());
        let phy = &mut out[28..40];
        phy[6] = phy_type(p.channel.band, n);
        phy[7] = (n.peer_ampdu_base_exponent & 3) | (n.peer_mpdu_spacing & 7) << 2;
        phy[10] = n.peer_ampdu_base_exponent;
        if let Some(h) = &n.ht {
            put16(tlv(out, 9, 8), 4, h.capability);
        }
        // Source demands this record on 6GHz even without a VHT capability.
        if n.vht.is_some() || n.peer().he_6ghz().is_some_and(|v| v != 0) {
            let v = tlv(out, 10, 16);
            if let Some(c) = &n.vht {
                put32(v, 4, c.capability);
                put16(v, 8, c.mcs.rx);
                put16(v, 10, c.mcs.tx);
            }
        }
        if n.max_tx_amsdu != 0 {
            let a = tlv(out, 15, 8);
            a[4] = 8;
            a[5] = match n.max_tx_amsdu {
                11454 => 2,
                7935 | 7991 => 1,
                _ => 0,
            };
            a[6] = 1;
        }
        if let Some(h) = &n.he {
            let t = tlv(out, 0x19, 28);
            t[4..10].copy_from_slice(&h.mac);
            t[10..21].copy_from_slice(&h.phy);
            t[21] = 2; // source nominal-packet-padding16us enum
            put16(t, 22, h.mcs_80.rx);
            if self.station_bandwidth() == 3 {
                if let Some(m) = h.mcs_160 {
                    put16(t, 24, m.rx);
                }
                if h.phy[0] & 16 != 0
                    && let Some(m) = h.mcs_80p80
                {
                    put16(t, 26, m.rx);
                }
            }
        }
        if let Some(c) = n.peer().he_6ghz().filter(|&v| v != 0) {
            put16(tlv(out, 0x17, 8), 4, c);
        }
        if let Some(e) = n.peer().eht().filter(|_| n.eht.is_some()) {
            let t = tlv(out, 0x22, 40);
            t[4] = 0xff;
            t[6..8].copy_from_slice(&e.mac);
            t[8..16].copy_from_slice(&e.phy[..8]);
            t[16] = e.phy[8];
            // Source's link_sta EHT map is the observed zero-filled nine-byte
            // peer union, independently of directional common-map admission.
            let union = e.source_mcs_union().bytes();
            if self.station_bandwidth() == 0 {
                t[24..28].copy_from_slice(&union[..4]);
            }
            t[28..31].copy_from_slice(&union[..3]);
            t[31..34].copy_from_slice(&union[3..6]);
            t[34..37].copy_from_slice(&union[6..9]);
        }
        Ok(())
    }
    pub(super) fn bss_capabilities(
        &self,
        p: Profile,
        enable: bool,
        out: &mut Vec<u8>,
    ) -> Result<(), Error> {
        let n = &self.negotiated;
        // Source BSS_BASIC uses the older family bitmap and separate extension.
        let (mode, ext) = bss_phy_mode(p.channel.band, n);
        out[4 + 25] = mode;
        out[4 + 30] = ext;
        // QBSS WMM flag, sourced from the negotiated station WME state.
        let qos = find_tlv_mut(out, 4, 15).ok_or(Error::InvalidNegotiation)?;
        qos[4] = u8::from(n.wmm.is_some());
        if let Some(op) = self.operation.he {
            let h = tlv(out, 5, 16);
            // Source defaults on parsed zero-valued operation
            // fields; these are firmware packet-extension/RTS values, no policy.
            put16(
                h,
                4,
                if op.rts_threshold == 0 {
                    1023
                } else {
                    op.rts_threshold
                },
            );
            h[6] = if op.packet_extension == 0 {
                4
            } else {
                op.packet_extension
            };
            for (index, maps) in self.local_he_maps.iter().enumerate() {
                put16(h, 8 + index * 2, maps.tx);
            }
            let c = tlv(out, 4, 8);
            c[4] = u8::from(enable && op.color_enabled);
            c[5] = if enable { op.color } else { 0 };
        }
        Ok(())
    }
    pub(super) fn rlm(&self, p: Profile, r: &mut [u8]) {
        r[5] = self.operation.center_channel;
        r[6] = self.operation.second_center_channel;
        r[7] = self.command_bandwidth();
        r[10] = if self.width == Width::Mhz20 { 0 } else { 4 };
        r[11] = match (p.channel.number as u8).cmp(&r[5]) {
            core::cmp::Ordering::Less => 1,
            core::cmp::Ordering::Greater => 3,
            core::cmp::Ordering::Equal => 0,
        };
        r[12] = match p.channel.band {
            Band::Ghz2 => 1,
            Band::Ghz5 => 2,
            Band::Ghz6 => 3,
        };
    }
    /// Source set_tx full four-queue update. WMM ACI order BE/BK/VI/VO differs
    /// from ieee80211_ac_numbers VO/VI/BE/BK consumed by mt7925_mcu_set_tx.
    pub fn edca_request(&self, p: Profile) -> Result<Vec<u8>, Error> {
        self.matches(p)?;
        self.edca_restricted_request(p, &self.negotiated.edca())
    }
    /// Private native queue restriction; never change negotiated AP metadata.
    pub(in crate::wifi::mediatek) fn edca_restricted_request(
        &self,
        p: Profile,
        effective: &[AccessCategory; 4],
    ) -> Result<Vec<u8>, Error> {
        self.matches(p)?;
        Self::observed_edca_request(p, &self.negotiated.edca(), effective)
    }
    /// Current native RX supplies the new AP baseline; it is not a replacement
    /// association or a mutation of the immutable negotiated capability object.
    pub(in crate::wifi::mediatek) fn observed_edca_request(
        p: Profile,
        original: &[AccessCategory; 4],
        effective: &[AccessCategory; 4],
    ) -> Result<Vec<u8>, Error> {
        p.validate_inner(true)?;
        let mut seen = 0u8;
        for ac in effective {
            let old = original
                .iter()
                .find(|old| old.aci == ac.aci)
                .ok_or(Error::InvalidNegotiation)?;
            if ac.aci >= 4
                || seen & (1 << ac.aci) != 0
                || ac.ecw_min < old.ecw_min
                || ac.ecw_max < old.ecw_max
                || ac.ecw_min > ac.ecw_max
                || ac.ecw_max > 15
                || ac.aifsn < old.aifsn
                || ac.txop_32us > old.txop_32us
            {
                return Err(Error::InvalidNegotiation);
            }
            seen |= 1 << ac.aci;
        }
        let mut out = Vec::new();
        out.try_reserve_exact(52).map_err(|_| Error::NoMemory)?;
        out.extend_from_slice(&[p.interface.bss_index, 0, 0, 0]);
        for (queue, aci) in [3, 2, 0, 1].into_iter().enumerate() {
            let ac = effective
                .iter()
                .find(|c| c.aci == aci)
                .ok_or(Error::InvalidNegotiation)?;
            let e = tlv(&mut out, 0, 12);
            e[4] = queue as u8;
            e[5] = 15;
            // fls((1<<ECW)-1)==ECW; source substitutes5/10 for zeroCW.
            e[6] = if ac.ecw_min == 0 { 5 } else { ac.ecw_min };
            e[7] = if ac.ecw_max == 0 { 10 } else { ac.ecw_max };
            put16(e, 8, ac.txop_32us);
            e[10] = ac.aifsn;
        }
        super::recount(&mut out, 4);
        Ok(out)
    }
    /// Separate source EHT puncture update, including a zero bitmap.
    pub fn puncture_request(&self, p: Profile) -> Result<Vec<u8>, Error> {
        self.matches(p)?;
        if self.negotiated.eht.is_none() {
            return Err(Error::InvalidNegotiation);
        }
        let mut out = vec![p.interface.bss_index, 0, 0, 0];
        let t = tlv(&mut out, 30, 16);
        t[5] = u8::from(self.operation.punctured != 0);
        put16(t, 10, self.operation.punctured);
        super::recount(&mut out, 4);
        Ok(out)
    }
}
fn find_tlv_mut(bytes: &mut [u8], mut at: usize, tag: u16) -> Option<&mut [u8]> {
    while at < bytes.len() {
        let id = u16::from_le_bytes(bytes[at..at + 2].try_into().ok()?);
        let len = u16::from_le_bytes(bytes[at + 2..at + 4].try_into().ok()?) as usize;
        if id == tag {
            return bytes.get_mut(at..at + len);
        }
        at += len;
    }
    None
}
fn phy_type(band: Band, n: &Negotiated) -> u8 {
    let base = if band == Band::Ghz2 { 3 } else { 8 };
    base | u8::from(n.ht.is_some()) << 4
        | u8::from(band != Band::Ghz2 && n.vht.is_some()) << 5
        | u8::from(n.he.is_some()) << 6
        | u8::from(n.eht.is_some()) << 7
}
fn bss_phy_mode(band: Band, n: &Negotiated) -> (u8, u8) {
    let ht = u8::from(n.ht.is_some());
    let vht = u8::from(n.vht.is_some());
    let he = u8::from(n.he.is_some());
    let eht = u8::from(n.eht.is_some());
    match band {
        Band::Ghz2 => (6 | ht << 3 | he << 6, eht << 1),
        Band::Ghz5 => (1 | ht << 4 | vht << 5 | he << 7, eht << 2),
        // Source advertises A|AN|AC|AX5 on6GHz plus extension,
        // independent of the invalid/absent HT/VHT IEs on that band.
        Band::Ghz6 => (
            0xb1,
            u8::from(n.peer().he_6ghz().is_some_and(|v| v != 0)) | eht << 3,
        ),
    }
}
fn validate_geometry(p: Profile, width: Width, op: Operation) -> Result<(), Error> {
    let delta = p.channel.number.abs_diff(u16::from(op.center_channel));
    let max = match width {
        Width::Mhz20 => 0,
        Width::Mhz40 => 2,
        Width::Mhz80 | Width::Mhz80P80 => 6,
        Width::Mhz160 => 14,
        Width::Mhz320 => 30,
    };
    if op.center_channel == 0
        || delta > max
        || (delta != 0 && delta % 4 != 2)
        || (width == Width::Mhz20 && delta != 0)
        || (width != Width::Mhz20 && delta == 0)
        || (!matches!(width, Width::Mhz80P80 | Width::Mhz320) && op.second_center_channel != 0)
        || (width == Width::Mhz80P80
            && (op.second_center_channel == 0
                || op.second_center_channel.abs_diff(op.center_channel) <= 16))
        || (op.punctured != 0 && !matches!(width, Width::Mhz80 | Width::Mhz160 | Width::Mhz320))
    {
        return Err(Error::InvalidGeometry);
    }
    let subchannels = match width {
        Width::Mhz80 => 4,
        Width::Mhz160 => 8,
        Width::Mhz320 => 16,
        _ => 0,
    };
    if subchannels != 0 {
        let mask = (1u32 << subchannels) - 1;
        let first = i32::from(op.center_channel) - 2 * (subchannels - 1);
        let primary = (i32::from(p.channel.number) - first) / 4;
        if primary < 0
            || primary >= subchannels
            || u32::from(op.punctured) & !mask != 0
            || u32::from(op.punctured) & (1 << primary) != 0
        {
            return Err(Error::InvalidGeometry);
        }
    }
    Ok(())
}

/// Source STA_REC_BA (recipient/originator), not an ADDBA exchange or reorder
/// receipt. Caller must bind peer/TID/generation and keep the
/// reorder/token/key owners through the matching command and later drain.
pub fn ba_request(
    p: Profile,
    tid: u8,
    ssn: u16,
    window: u16,
    amsdu: bool,
    enable: bool,
    transmit: bool,
) -> Result<Vec<u8>, Error> {
    p.validate_inner(true)?;
    if tid >= 8 || ssn >= 4096 || window == 0 || window > 1024 {
        return Err(Error::InvalidAggregation);
    }
    let mut out = vec![0; 8];
    out[0] = p.interface.bss_index;
    out[1] = p.wcid as u8;
    out[2] = 1;
    out[4] = 1;
    out[5] = p.interface.omac_index;
    let t = tlv(&mut out, 6, 16);
    t[4] = tid;
    t[5] = if transmit { 1 } else { 2 };
    t[6] = u8::from(amsdu);
    t[7] = u8::from(enable) << tid;
    put16(t, 8, ssn);
    put16(t, 10, window);
    Ok(out)
}

#[cfg(test)]
pub(super) mod tests;
