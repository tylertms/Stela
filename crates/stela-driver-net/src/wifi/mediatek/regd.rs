// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
// Copyright (C) 2020-2025 MediaTek Inc.
//! MT7925's programming after its CLC, as `mt7925_mcu_regd_update` makes it:
//! each band's channels as cfg80211 regulated them for the country
//! (`mt76_channels_*`), restricted by `mt7925_regd_be_ctrl` and
//! `mt7925_regd_channel_update`, for `mt7925_mcu_set_channel_domain`; then
//! each band's rate power (`mt7925_mcu_rate_txpower_band`) under the
//! platform's ACPI SAR limits (`mt7925_set_tx_sar_pwr`).

use super::{
    acpi::Sar,
    identity::Chip,
    mailbox::clc::remaining,
    mcu::{domain, power},
};
use crate::{
    bytes::le32,
    wifi::{
        Band, Channel,
        regulatory::{channel, database::Country},
    },
};
use alloc::vec::Vec;

const BANDS: [Band; 3] = [Band::Ghz2, Band::Ghz5, Band::Ghz6];
/// `mt76_channels_2ghz`, `_5ghz` and `_6ghz`.
const CHANNELS_2GHZ: [u8; 14] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
const CHANNELS_5GHZ: [u8; 28] = [
    36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165, 169, 173, 177,
];
const CHANNELS_6GHZ: [u8; 59] = {
    let mut channels = [0; 59];
    let mut i = 0;
    while i < channels.len() {
        channels[i] = 1 + 4 * i as u8;
        i += 1;
    }
    channels
};
/// `mt76_sar_freq_ranges`, in MHz.
const SAR_RANGES: [(u32, u32); 11] = [
    (2402, 2494),
    (5150, 5350),
    (5350, 5470),
    (5470, 5725),
    (5725, 5950),
    (5945, 6165),
    (6165, 6405),
    (6405, 6525),
    (6525, 6705),
    (6705, 6865),
    (6865, 7125),
];
/// `mt7925_regd_channel_update`'s UNII-4 to UNII-8, in MHz inclusive, each
/// with the CLC channel configuration bit enabling it.
const UNII: [(u16, u16, u8); 5] = [
    (5845, 5925, 1 << 0),
    (5925, 6425, 1 << 1),
    (6425, 6525, 1 << 2),
    (6525, 6875, 1 << 3),
    (6875, 7125, 1 << 4),
];

/// The channel numbers mt76 registers on `band`.
pub fn channels(band: Band) -> &'static [u8] {
    match band {
        Band::Ghz2 => &CHANNELS_2GHZ,
        Band::Ghz5 => &CHANNELS_5GHZ,
        Band::Ghz6 => &CHANNELS_6GHZ,
    }
}

/// `ieee80211_channel_to_frequency`.
fn frequency(band: Band, channel: u8) -> u16 {
    let n = u16::from(channel);
    match band {
        Band::Ghz2 if n == 14 => 2484,
        Band::Ghz2 => 2407 + 5 * n,
        Band::Ghz5 => 5000 + 5 * n,
        Band::Ghz6 if n == 2 => 5935,
        Band::Ghz6 => 5950 + 5 * n,
    }
}

/// What the plan reads from the completed CLC.
pub struct Inputs<'a> {
    /// MT7925 follows `mt7925_mcu_regd_update`; MT7921 and MT7922 follow
    /// `__mt7921_mcu_regd_update`, which leaves EHT and the MTCL to the
    /// firmware's CLC and sends every band's rate power.
    pub chip: Chip,
    pub country: &'a Country,
    /// The bands the PHY registers: `cap.has_2ghz`, `has_5ghz`, `has_6ghz`.
    pub bands: [bool; 3],
    /// `clc_chan_conf`: the UNII flags of the last CLC rule sent.
    pub channel_configuration: u8,
    /// The rules of the firmware's BE_CTRL CLC record, when it has one.
    pub eht_control: Option<&'a [u8]>,
    pub sar: &'a Sar,
}

/// One country's channel domain and rate power for MT7925.
pub struct Plan {
    alpha2: [u8; 2],
    channel_configuration: u8,
    domain: [Vec<domain::Channel>; 3],
    power: [Option<Vec<power::Channel>>; 3],
}

/// `mt7925_regd_be_ctrl`: whether the country may use EHT.
fn eht(inputs: &Inputs<'_>, mtcl: Option<u32>) -> bool {
    if mtcl.is_some_and(|configuration| (configuration >> 4) & 3 == 0) {
        return false;
    }
    let alpha2 = inputs.country.alpha2();
    // Rules of a 32-bit flag, bit 0 marking the last, then the country.
    for rule in inputs.eht_control.unwrap_or_default().as_chunks::<16>().0 {
        if rule[4..6] == alpha2 {
            return false;
        }
        if le32(rule, 0) & 1 != 0 {
            break;
        }
    }
    true
}

impl Plan {
    /// `None` when the plan cannot be allocated.
    pub fn new(inputs: Inputs<'_>) -> Option<Self> {
        let country = inputs.country;
        let connac3 = inputs.chip == Chip::Mt7925;
        // mt7921 hands the MTCL to the firmware in SET_CLC and has no EHT.
        let mtcl = connac3
            .then(|| inputs.sar.mtcl_configuration(country.alpha2()))
            .flatten();
        let eht = !connac3 || eht(&inputs, mtcl);
        let mut configuration = inputs.channel_configuration;
        if let Some(mtcl) = mtcl {
            if mtcl & 3 == 0 {
                configuration &= !1;
            }
            if (mtcl >> 2) & 3 == 0 {
                configuration &= !0x1e;
            }
        }
        let mut sar = [None; SAR_RANGES.len()];
        for (i, (limit, &(start, _))) in sar.iter_mut().zip(&SAR_RANGES).enumerate() {
            *limit = inputs.sar.range_power(i, start, country.dfs_region());
        }
        let mut plan = Self {
            alpha2: country.alpha2(),
            channel_configuration: configuration,
            domain: Default::default(),
            power: Default::default(),
        };
        for (i, band) in BANDS.into_iter().enumerate() {
            if !inputs.bands[i] {
                continue;
            }
            let values: &[u8] = match band {
                Band::Ghz2 => &CHANNELS_2GHZ,
                Band::Ghz5 => &CHANNELS_5GHZ,
                Band::Ghz6 => &CHANNELS_6GHZ,
            };
            let mut centers = [0; CHANNELS_6GHZ.len()];
            let centers = &mut centers[..values.len()];
            for (center, &value) in centers.iter_mut().zip(values) {
                *center = frequency(band, value);
            }
            let mut regulated = country.band(centers)?;
            // mt7925_regd_channel_update stops at the first band the PHY
            // lacks; mt7921_regd_channel_update starts at 5 GHz.
            let first = if connac3 { 0 } else { 1 };
            let updated = inputs.bands[first.min(i)..=i].iter().all(|&b| b);
            let unii = match band {
                Band::Ghz2 => &UNII[..0],
                Band::Ghz5 => &UNII[..1],
                Band::Ghz6 => &UNII[1..],
            };
            for (r, &center) in regulated.iter_mut().zip(centers.iter()) {
                if updated && !eht {
                    r.flags |= channel::NO_EHT;
                }
                if updated
                    && unii.iter().any(|&(start, end, bit)| {
                        configuration & bit == 0 && (start..=end).contains(&center)
                    })
                {
                    r.flags |= channel::DISABLED;
                }
            }
            let domain = &mut plan.domain[i];
            domain.try_reserve_exact(values.len()).ok()?;
            domain.extend(
                values
                    .iter()
                    .zip(&regulated)
                    .filter(|(_, r)| r.flags & channel::DISABLED == 0)
                    .map(|(&value, r)| domain::Channel {
                        hardware_value: u16::from(value),
                        flags: r.flags,
                    }),
            );
            // mt7925_mcu_set_rate_txpower skips 6 GHz without a CLC
            // channel configuration; mt76_connac_mcu_set_rate_txpower
            // sends every band the PHY has.
            if connac3 && band == Band::Ghz6 && configuration == 0 {
                continue;
            }
            let rows = power::channels(inputs.chip, band);
            let mut power = Vec::new();
            power.try_reserve_exact(rows.len()).ok()?;
            for &value in rows {
                // Before any interface, conf.power_level is zero and the
                // source starts every row at 127.
                let mut half_dbm: i8 = 127;
                // mt76_connac_get_ch_power: an enabled wiphy channel's
                // regulatory limit, narrowed to s8 as there.
                if let Some(j) = values.iter().position(|&v| v == value)
                    && regulated[j].flags & channel::DISABLED == 0
                {
                    half_dbm = half_dbm.min((2 * regulated[j].max_reg_dbm) as i8);
                }
                // mt76_get_sar_power: the first range holding the channel.
                if half_dbm == i8::MIN {
                    half_dbm = 127;
                }
                let mhz = u32::from(frequency(band, value));
                if let Some(limit) = SAR_RANGES
                    .iter()
                    .zip(sar)
                    .find(|&(&(start, end), _)| (start..end).contains(&mhz))
                    .and_then(|(_, limit)| limit)
                {
                    half_dbm = half_dbm.min(limit);
                }
                power.push(power::Channel {
                    hardware_value: value,
                    limits: power::Limits::uniform(half_dbm),
                });
            }
            plan.power[i] = Some(power);
        }
        Some(plan)
    }

    /// The channel `ieee80211_register_hw` starts the radio on: the first
    /// enabled one of the first band that has one.
    pub fn default_channel(&self) -> Option<Channel> {
        BANDS
            .into_iter()
            .zip(&self.domain)
            .find_map(|(band, channels)| {
                let number = channels.first()?.hardware_value;
                Some(Channel {
                    band,
                    number,
                    frequency_mhz: frequency(band, number as u8),
                })
            })
    }

    pub fn inputs(&self) -> remaining::Inputs<'_> {
        remaining::Inputs {
            channel_configuration: self.channel_configuration,
            domain: domain::Inputs {
                alpha2: self.alpha2,
                channels_2ghz: &self.domain[0],
                channels_5ghz: &self.domain[1],
                channels_6ghz: &self.domain[2],
            },
            power: power::Inputs {
                alpha2: self.alpha2,
                ghz2: self.power[0].as_deref(),
                ghz5: self.power[1].as_deref(),
                ghz6: self.power[2].as_deref(),
            },
        }
    }
}

#[cfg(test)]
mod tests;
