// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Source initial STATE_NONE peer/BSS and ROC20 codecs, in the chip's
//! connac generation. This has no associated Configuration and cannot encode
//! association/key/port success.
use super::{Codec, Error, Outgoing, Profile, connac2};
use crate::wifi::{
    Band,
    scan::result::Bss,
    station::{
        operations::Operations,
        phy::{Capabilities, McsMaps, Role},
    },
};
use alloc::vec::Vec;

#[derive(Clone, Debug)]
pub(in crate::wifi::mediatek) struct Preauthentication {
    codec: Codec,
    profile: Profile,
    power_type: u8,
}
impl Preauthentication {
    pub fn new(
        codec: Codec,
        profile: Profile,
        local: &Capabilities,
        bss: &Bss,
    ) -> Result<Self, Error> {
        profile.validate_inner(true)?;
        if profile.channel.band != Band::Ghz6
            || profile.aid != 0
            || profile.cipher != 0
            || profile.channel != bss.channel
            || !bss.answers_for(profile.peer)
            || profile.beacon_interval != bss.beacon_interval
            || bss.capabilities & 0x10 == 0
            || bss.observed.band != Band::Ghz6
            || bss.observed.channel != bss.channel.number
            || bss.observed.frequency_mhz != bss.channel.frequency_mhz
            || local.band() != Band::Ghz6
            || local.role() != Role::Station
        {
            return Err(Error::InvalidNegotiation);
        }
        let peer = Capabilities::parse(Band::Ghz6, Role::AccessPoint, &bss.elements)
            .map_err(|_| Error::InvalidNegotiation)?;
        let operations = Operations::parse(&bss.elements).map_err(|_| Error::InvalidGeometry)?;
        let he = operations.he.as_ref().ok_or(Error::InvalidGeometry)?;
        let six = he.six_ghz.ok_or(Error::InvalidGeometry)?;
        // The SAE frames go at 20 MHz on the primary whatever the AP's width
        // or EHT; whether its power type is usable is the board's decision.
        let power_type = (six[1] & 0x78) >> 3;
        if six[0] != profile.channel.number as u8
            || operations.ht.is_some()
            || operations.vht.is_some()
        {
            return Err(Error::InvalidGeometry);
        }
        let own_he = local.he().ok_or(Error::InvalidNegotiation)?;
        let peer_he = peer.he().ok_or(Error::InvalidNegotiation)?;
        let common = McsMaps::intersect(own_he.mcs_80, peer_he.mcs_80);
        if local.he_6ghz().is_none_or(|v| v == 0)
            || peer.he_6ghz().is_none_or(|v| v == 0)
            || local.wmm().is_none()
            || peer.wmm().is_none()
            || common.rx & 3 == 3
            || common.tx & 3 == 3
            || !peer.meets_peer_he_basic(he.basic_mcs)
            || !local
                .meets_local_he_basic(he.basic_mcs)
                .map_err(|_| Error::InvalidNegotiation)?
            || profile.basic_rates != peer.rates().basic
            || profile.supported_rates != peer.rates().supported
        {
            return Err(Error::InvalidNegotiation);
        }
        // TIM is required by this finite initial peer profile. Neither
        // caller timing nor an invented DTIM default substitutes for it.
        let mut rest = bss.elements.as_slice();
        let mut dtim = None;
        while !rest.is_empty() {
            let h = rest.get(..2).ok_or(Error::InvalidTiming)?;
            let bytes = rest
                .get(2..2 + usize::from(h[1]))
                .ok_or(Error::InvalidTiming)?;
            if h[0] == 5 && (bytes.len() < 4 || dtim.replace(bytes[1]).is_some()) {
                return Err(Error::InvalidTiming);
            }
            rest = &rest[2 + bytes.len()..];
        }
        if dtim != Some(profile.dtim_period) {
            return Err(Error::InvalidTiming);
        }
        Ok(Self {
            codec,
            profile,
            power_type,
        })
    }
    pub const fn profile(&self) -> Profile {
        self.profile
    }
    pub const fn power_type(&self) -> u8 {
        self.power_type
    }
    /// The hardware band a ROC grant assigned.
    pub fn set_band_index(&mut self, band: u8) {
        self.profile.interface.band_index = band;
    }
    /// The disabled BSS's BSS_INFO_UPDATE bodies.
    pub fn bss_requests(&self) -> Result<Vec<Vec<u8>>, Error> {
        if let Codec::Connac2(_) = self.codec {
            return connac2::station::six_ghz_bss(self.profile);
        }
        let mut bytes = self
            .profile
            .bss_request_validated(false, None, None, true)?;
        // Real nonNULL initial zero-capability peer: source6GHz older-family
        // bitmap is unconditional A|AN|AC|AX5, extension remains zero.
        bytes[4 + 25] = 0xb1;
        Ok(alloc::vec![bytes])
    }
    pub fn peer_request(&self, enable: bool) -> Result<Vec<u8>, Error> {
        match self.codec {
            Codec::Connac3 => self
                .profile
                .peer_request_validated(enable, false, None, None, true),
            Codec::Connac2(_) => connac2::station::six_ghz_peer(self.profile, enable),
        }
    }
    pub fn broadcast_request(&self) -> Result<Vec<u8>, Error> {
        match self.codec {
            Codec::Connac3 => self.profile.broadcast_request(),
            Codec::Connac2(_) => connac2::station::six_ghz_broadcast(self.profile),
        }
    }
    pub fn disable_beacon_pm(&self) -> Result<Outgoing, Error> {
        self.codec.disable_beacon_pm(self.profile)
    }
    pub fn filter_request(&self, enable: bool) -> Result<super::super::init::Request, Error> {
        self.codec.filter(self.profile, enable)
    }
    pub fn roc(&self, token: u8, duration: u32) -> Result<super::super::init::Request, Error> {
        match self.codec {
            Codec::Connac3 => self.profile.roc_validated(token, duration, true),
            Codec::Connac2(_) => self.codec.roc(self.profile, token, duration),
        }
    }
    pub fn abort_roc(&self, token: u8) -> Result<super::super::init::Request, Error> {
        match self.codec {
            Codec::Connac3 => self.profile.abort_roc_validated(token, true),
            Codec::Connac2(_) => self.codec.abort_roc(self.profile, token),
        }
    }
}
