// SPDX-License-Identifier: GPL-2.0-only
//! Realtek family controls from realtek.c::{alc_fill_eapd_coef,alc_auto_init_amp},
//! alc882.c::alc882_probe and alc662.c::{alc662_probe,alc668_coefs} at
//! Linux a804c5742bff224d4cb4d25dd383554d598be32b (GPL-2.0-or-later).
use super::CoefficientChange;
use crate::{
    Error, Timer,
    hda::{Controller, Registers},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealtekFamily {
    Alc882,
    Alc662,
    Alc269,
    Alc245,
}

pub(super) fn family(id: u32, revision: u32) -> Result<RealtekFamily, Error> {
    match id {
        0x10ec_0245 => Ok(RealtekFamily::Alc245),
        0x10ec_0882 | 0x10ec_0883 | 0x10ec_0885 | 0x10ec_0887 | 0x10ec_0888 | 0x10ec_0889
        | 0x10ec_0899 | 0x10ec_0900 | 0x10ec_0b00 | 0x10ec_1168 | 0x10ec_1220 => {
            Ok(RealtekFamily::Alc882)
        }
        0x10ec_0662 if matches!(revision, 0x100101 | 0x100300) => Ok(RealtekFamily::Alc662),
        0x10ec_0272 | 0x10ec_0663 | 0x10ec_0665 | 0x10ec_0667 | 0x10ec_0668 | 0x10ec_0670
        | 0x10ec_0671 | 0x10ec_0867 | 0x10ec_0892 | 0x10ec_0897 => Ok(RealtekFamily::Alc662),
        _ => Err(Error::UnsupportedTopology),
    }
}

// Preserve the donor's literal writes and order, including high vendor indices.
const ALC668_DEFAULTS: &[(u16, u16)] = &[
    (0x01, 0xbebe),
    (0x02, 0xaaaa),
    (0x03, 0x0000),
    (0x04, 0x0180),
    (0x06, 0x0000),
    (0x07, 0x0f80),
    (0x08, 0x0031),
    (0x0a, 0x0060),
    (0x0b, 0x0000),
    (0x0c, 0x7cf7),
    (0x0d, 0x1080),
    (0x0e, 0x7f7f),
    (0x0f, 0xcccc),
    (0x10, 0xddcc),
    (0x11, 0x0001),
    (0x13, 0x0000),
    (0x14, 0x2aa0),
    (0x17, 0xa940),
    (0x19, 0x0000),
    (0x1a, 0x0000),
    (0x1b, 0x0000),
    (0x1c, 0x0000),
    (0x1d, 0x0000),
    (0x1e, 0x7418),
    (0x1f, 0x0804),
    (0x20, 0x4200),
    (0x21, 0x0468),
    (0x22, 0x8ccc),
    (0x23, 0x0250),
    (0x24, 0x7418),
    (0x27, 0x0000),
    (0x28, 0x8ccc),
    (0x2a, 0xff00),
    (0x2b, 0x8000),
    (0xa7, 0xff00),
    (0xa8, 0x8000),
    (0xaa, 0x2e17),
    (0xab, 0xa0c0),
    (0xac, 0x0000),
    (0xad, 0x0000),
    (0xae, 0x2ac6),
    (0xaf, 0xa480),
    (0xb0, 0x0000),
    (0xb1, 0x0000),
    (0xb2, 0x0000),
    (0xb3, 0x0000),
    (0xb4, 0x0000),
    (0xb5, 0x1040),
    (0xb6, 0xd697),
    (0xb7, 0x902b),
    (0xb8, 0xd697),
    (0xb9, 0x902b),
    (0xba, 0xb8ba),
    (0xbb, 0xaaab),
    (0xbc, 0xaaaf),
    (0xbd, 0x6aaa),
    (0xbe, 0x1c02),
    (0xc0, 0x00ff),
    (0xc1, 0x0fa6),
];

impl<R: Registers, T: Timer> Controller<R, T> {
    pub(super) fn realtek_family_coefficients(
        &mut self,
        codec: u8,
        id: u32,
        selected: RealtekFamily,
        zero: u16,
    ) -> Result<alloc::vec::Vec<CoefficientChange>, Error> {
        self.minimum_guard()?;
        let mut changes = alloc::vec::Vec::new();
        let eapd = match id {
            0x10ec_0662 if zero & 0xf0 == 0x30 => Some((4, 1 << 10, 0)),
            0x10ec_0272 | 0x10ec_0663 | 0x10ec_0665 | 0x10ec_0670 | 0x10ec_0671 => {
                Some((0x0d, 0, 1 << 14))
            }
            0x10ec_0668 => Some((7, 3 << 13, 0)),
            0x10ec_0867 => Some((4, 1 << 10, 0)),
            0x10ec_0888 if matches!(zero & 0xf0, 0x20 | 0x30) => Some((7, 1 << 5, 0)),
            0x10ec_0892 | 0x10ec_0897 => Some((7, 1 << 5, 0)),
            0x10ec_0899 | 0x10ec_0900 | 0x10ec_0b00 | 0x10ec_1168 | 0x10ec_1220 => {
                Some((7, 1 << 1, 0))
            }
            _ => None,
        };
        if let Some((index, mask, bits)) = eapd {
            changes.push(self.update_coefficient(codec, index, mask, bits)?);
        }
        // alc_pre_init EAPD controls precede alc668_init's literal restore.
        if id == 0x10ec_0668 {
            for &(index, value) in ALC668_DEFAULTS {
                let previous = self.read_coefficient(codec, index)?;
                changes.push(self.write_coefficient_observed(codec, index, previous, value)?);
            }
        }
        let pll = match selected {
            RealtekFamily::Alc662 => Some((4, 1 << 15)),
            RealtekFamily::Alc882
                if !matches!(
                    id,
                    0x10ec_0882 | 0x10ec_0885 | 0x10ec_0900 | 0x10ec_0b00 | 0x10ec_1220
                ) =>
            {
                Some((0x0a, 1 << 10))
            }
            _ => None,
        };
        if let Some((index, mask)) = pll {
            changes.push(self.clear_coefficient_bits(codec, index, mask)?);
        }
        Ok(changes)
    }

    // alc_auto_init_amp enables its EAPD pins before these default
    // amplifier coefficients. Keep this separate from pre-init/PLL controls
    // so an unacknowledged EAPD enable cannot reach the amplifier write.
    pub(super) fn realtek_default_amplifier_coefficient(
        &mut self,
        codec: u8,
        id: u32,
        zero: u16,
    ) -> Result<Option<CoefficientChange>, Error> {
        self.minimum_guard()?;
        if matches!(id, 0x10ec_0882 | 0x10ec_0883 | 0x10ec_0885)
            || (id == 0x10ec_0888 && matches!(zero & 0xf0, 0 | 0x10))
        {
            return self.update_coefficient(codec, 7, 0, 0x2030).map(Some);
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_family_revisions_and_complete_668_defaults() {
        assert_eq!(family(0x10ec_0245, 0x100001), Ok(RealtekFamily::Alc245));
        assert_eq!(family(0x10ec_0662, 0x100101), Ok(RealtekFamily::Alc662));
        assert_eq!(family(0x10ec_0662, 0x100300), Ok(RealtekFamily::Alc662));
        assert_eq!(
            family(0x10ec_0662, 0x100102),
            Err(Error::UnsupportedTopology)
        );
        assert_eq!(
            family(0x10ec_0269, 0x100101),
            Err(Error::UnsupportedTopology)
        );
        assert_eq!(ALC668_DEFAULTS.len(), 59);
        assert_eq!(ALC668_DEFAULTS.last(), Some(&(0xc1, 0x0fa6)));
    }
}
