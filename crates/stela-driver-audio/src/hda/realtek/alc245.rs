// SPDX-License-Identifier: GPL-2.0-only
//! Dedicated Realtek ALC245 codec support.

pub const ALC245_VENDOR_ID: u32 = 0x10ec_0245;
pub const ALC245_VARIANT_BIT: u16 = 0x0010;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alc245SiliconVariant {
    Alc215Compatible,
    Alc245,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Alc245Identity {
    pub vendor_id: u32,
    pub revision_id: u32,
}

impl Alc245Identity {
    pub const fn detect(vendor_id: u32, revision_id: u32) -> Option<Self> {
        if vendor_id == ALC245_VENDOR_ID {
            Some(Self {
                vendor_id,
                revision_id,
            })
        } else {
            None
        }
    }

    pub const fn silicon_variant(&self, coefficient_zero: u16) -> Alc245SiliconVariant {
        if coefficient_zero & ALC245_VARIANT_BIT != 0 {
            Alc245SiliconVariant::Alc245
        } else {
            Alc245SiliconVariant::Alc215Compatible
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_alc245() {
        assert!(Alc245Identity::detect(ALC245_VENDOR_ID, 0x100001).is_some());
    }

    #[test]
    fn rejects_other_codecs() {
        assert!(Alc245Identity::detect(0x10ec_0269, 0x100001).is_none());
    }

    #[test]
    fn identifies_silicon_variant_from_coefficient_zero() {
        let codec = Alc245Identity::detect(ALC245_VENDOR_ID, 0x100001).unwrap();

        assert_eq!(codec.silicon_variant(0x0010), Alc245SiliconVariant::Alc245);
        assert_eq!(
            codec.silicon_variant(0x0000),
            Alc245SiliconVariant::Alc215Compatible
        );
    }
}

/// Acer Nitro ANV15-51 subsystem identifier.
pub const ACER_NITRO_ANV15_51_SUBSYSTEM_ID: u32 = 0x1025_171e;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alc245Board {
    AcerNitroAnv15_51,
}

pub const fn board_profile(subsystem_id: u32) -> Option<Alc245Board> {
    if subsystem_id == ACER_NITRO_ANV15_51_SUBSYSTEM_ID {
        Some(Alc245Board::AcerNitroAnv15_51)
    } else {
        None
    }
}

#[cfg(test)]
mod board_tests {
    use super::*;

    #[test]
    fn detects_acer_nitro_anv15_51() {
        assert_eq!(
            board_profile(0x1025_171e),
            Some(Alc245Board::AcerNitroAnv15_51)
        );
    }

    #[test]
    fn rejects_unknown_acer_board() {
        assert_eq!(board_profile(0x1025_0000), None);
    }
}
