//! Discrete NVIDIA chipset identity shared by boot and the NVK device path.
//! The table follows nouveau's `nvkm_device_ctor` in
//! `drivers/gpu/drm/nouveau/nvkm/engine/device/base.c` (Linux v7.2.12)
//! for the GSP-RM chipsets starting at Turing.

/// The GPU generation, which fixes NVK's 3D class and NAK's shader model.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Architecture {
    Pascal,
    Turing,
    Ampere,
    Ada,
    Hopper,
    Blackwell,
}

/// One chipset from nouveau's table.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Chip {
    /// `PMC_BOOT_0` bits 20-28, as nouveau's `device->chipset`.
    pub chipset: u16,
    /// nouveau's chipset name, which is also NVK's `chipset_name`.
    pub name: &'static str,
    pub architecture: Architecture,
    /// nouveau instantiates a display engine for this chipset.
    pub display: bool,
}

const fn chip(chipset: u16, name: &'static str, architecture: Architecture, display: bool) -> Chip {
    Chip {
        chipset,
        name,
        architecture,
        display,
    }
}

use Architecture::{Ada, Ampere, Blackwell, Hopper, Pascal, Turing};

/// nouveau's Pascal, Turing and later chipsets; GA100, GH100, GB100 and GB102 are
/// compute parts without a display engine.
pub static CHIPS: [Chip; 29] = [
    chip(0x132, "GP102", Pascal, true),
    chip(0x134, "GP104", Pascal, true),
    chip(0x136, "GP106", Pascal, true),
    chip(0x137, "GP107", Pascal, true),
    chip(0x138, "GP108", Pascal, true),
    chip(0x162, "TU102", Turing, true),
    chip(0x164, "TU104", Turing, true),
    chip(0x166, "TU106", Turing, true),
    chip(0x167, "TU117", Turing, true),
    chip(0x168, "TU116", Turing, true),
    chip(0x170, "GA100", Ampere, false),
    chip(0x172, "GA102", Ampere, true),
    chip(0x173, "GA103", Ampere, true),
    chip(0x174, "GA104", Ampere, true),
    chip(0x176, "GA106", Ampere, true),
    chip(0x177, "GA107", Ampere, true),
    chip(0x180, "GH100", Hopper, false),
    chip(0x192, "AD102", Ada, true),
    chip(0x193, "AD103", Ada, true),
    chip(0x194, "AD104", Ada, true),
    chip(0x196, "AD106", Ada, true),
    chip(0x197, "AD107", Ada, true),
    chip(0x1a0, "GB100", Blackwell, false),
    chip(0x1a2, "GB102", Blackwell, false),
    chip(0x1b2, "GB202", Blackwell, true),
    chip(0x1b3, "GB203", Blackwell, true),
    chip(0x1b5, "GB205", Blackwell, true),
    chip(0x1b6, "GB206", Blackwell, true),
    chip(0x1b7, "GB207", Blackwell, true),
];

impl Chip {
    pub fn find(chipset: u16) -> Option<&'static Chip> {
        CHIPS.iter().find(|chip| chip.chipset == chipset)
    }

    /// Decode the little-endian PMC_BOOT_0 of the actual register BAR.
    pub fn from_boot0(boot0: u32) -> Option<&'static Chip> {
        Self::find(((boot0 >> 20) & 0x1ff) as u16)
    }

    /// nouveau gives this chipset a devinit subdev whose post waits for the
    /// GPU's own firmware boot (`tu102_devinit_post`, also `ga100_devinit`):
    /// Turing, Ampere and Ada. Their VBIOS is read (`nvkm_bios_new`) before
    /// that wait. Hopper and Blackwell have neither subdev; FSP boots them.
    pub fn waits_for_firmware_boot(&self) -> bool {
        matches!(self.architecture, Turing | Ampere | Ada)
    }
}

/// The firmware boot ("GFW boot") a Turing-to-Ada GPU's own falcon runs
/// after every reset: devinit from the VBIOS, which also places the VBIOS
/// image the PRAMIN shadow reads. nouveau waits for it in
/// `nvkm/subdev/devinit/tu102.c` `tu102_devinit_wait`, after the VBIOS read,
/// which is safe on Linux only because nothing reset the GPU since power-on.
/// Stela resets the function first, so it waits here, before nouveau runs.
pub mod firmware_boot {
    /// `NV_PGC6_AON_SECURE_SCRATCH_GROUP_05_PRIV_LEVEL_MASK`: bit 0 set once
    /// the progress register below may be read.
    pub const PRIV_LEVEL_MASK: usize = 0x11_8128;
    /// `NV_PGC6_AON_SECURE_SCRATCH_GROUP_05_0_GFW_BOOT`: bits 7:0 are the
    /// progress, [`COMPLETED`] when devinit has finished.
    pub const PROGRESS: usize = 0x11_8234;
    pub const COMPLETED: u32 = 0xff;
    /// nouveau's bound: `50 + 2000` polls, each after `usleep_range(1000,
    /// 2000)`, so at least 2.05 s.
    pub const POLLS: u32 = 2051;
    pub const POLL_MICROS: u64 = 1000;

    /// What the two registers last held.
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct Reading {
        pub priv_level_mask: u32,
        pub progress: u32,
        /// Polls taken, at most [`POLLS`].
        pub polls: u32,
    }

    impl Reading {
        pub fn completed(&self) -> bool {
            self.priv_level_mask & 1 != 0 && self.progress & 0xff == COMPLETED
        }
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub enum Error<E> {
        /// A register read or the timer failed.
        Access(E),
        /// The firmware boot did not complete within nouveau's bound.
        Timeout(Reading),
    }

    /// Poll `read` (a BAR0 register read) until the firmware boot completes,
    /// sleeping `sleep` microseconds between polls, at most [`POLLS`] times.
    pub fn wait<E>(
        mut read: impl FnMut(usize) -> Result<u32, E>,
        mut sleep: impl FnMut(u64) -> Result<(), E>,
    ) -> Result<Reading, Error<E>> {
        let mut reading = Reading {
            priv_level_mask: 0,
            progress: 0,
            polls: 0,
        };
        loop {
            reading.polls += 1;
            reading.priv_level_mask = read(PRIV_LEVEL_MASK).map_err(Error::Access)?;
            // As nouveau: the progress is only meaningful once readable.
            if reading.priv_level_mask & 1 != 0 {
                reading.progress = read(PROGRESS).map_err(Error::Access)?;
            }
            if reading.completed() {
                return Ok(reading);
            }
            if reading.polls >= POLLS {
                return Err(Error::Timeout(reading));
            }
            sleep(POLL_MICROS).map_err(Error::Access)?;
        }
    }
}

/// What nouveau's device file answers about the bound chip
/// (`DRM_IOCTL_NOUVEAU_GETPARAM`, `include/uapi/drm/nouveau_drm.h`).
pub mod query {
    use crate::stack::Control;

    /// `struct drm_nouveau_getparam`.
    #[repr(C)]
    struct GetParam {
        parameter: u64,
        value: u64,
    }

    /// `DRM_IOCTL_NOUVEAU_GETPARAM`.
    const GET_PARAM: u32 = 0xc010_6440;
    /// `NOUVEAU_GETPARAM_FB_SIZE`: the VRAM nouveau makes available.
    const FB_SIZE: u64 = 8;
    /// `NOUVEAU_GETPARAM_CHIPSET_ID`.
    const CHIPSET_ID: u64 = 11;

    fn get(control: Control, parameter: u64) -> Option<u64> {
        let mut record = GetParam {
            parameter,
            value: 0,
        };
        (control(GET_PARAM, (&raw mut record).cast()) == 0).then_some(record.value)
    }

    /// nouveau's `device->chipset` ([`super::Chip::chipset`]).
    pub fn chipset(control: Control) -> Option<u32> {
        get(control, CHIPSET_ID).map(|chipset| u32::from(chipset as u16))
    }

    pub fn memory_bytes(control: Control) -> Option<u64> {
        get(control, FB_SIZE).filter(|bytes| *bytes != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot0_selects_the_silicon_chipset_independently_of_revision() {
        for chip in CHIPS {
            let field = u32::from(chip.chipset) << 20;
            assert_eq!(Chip::from_boot0(field | 0x91), Some(&chip));
            assert_eq!(Chip::from_boot0(field | 0xa1), Some(&chip));
        }
        assert_eq!(Chip::from_boot0(u32::MAX), None);
        assert_eq!(Chip::from_boot0(0), None);
    }

    #[test]
    fn only_the_chips_nouveau_posts_wait_for_their_firmware_boot() {
        let waits = |chipset| Chip::find(chipset).unwrap().waits_for_firmware_boot();
        // TU117 (GTX 16), GA104 (RTX 30), AD107 (RTX 4060).
        assert!(waits(0x167) && waits(0x174) && waits(0x197));
        // GH100 and GB203 (RTX 5080) have no devinit subdev. GP106 (GTX 1060) boots via VBIOS.
        assert!(!waits(0x180) && !waits(0x1b3) && !waits(0x136));
    }

    #[test]
    fn the_firmware_boot_wait_reads_progress_only_once_readable() {
        use firmware_boot::*;
        let mut reads = alloc::vec::Vec::new();
        let poll = core::cell::Cell::new(0);
        let mut slept = 0;
        let reading = wait::<()>(
            |offset| {
                reads.push(offset);
                Ok(match offset {
                    PRIV_LEVEL_MASK if poll.get() < 2 => 0,
                    PRIV_LEVEL_MASK => 1,
                    PROGRESS if poll.get() < 4 => 0x7f,
                    _ => 0x1ff,
                })
            },
            |micros| {
                assert_eq!(micros, POLL_MICROS);
                poll.set(poll.get() + 1);
                slept += 1;
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(slept, 4);
        assert_eq!(
            reading,
            Reading {
                priv_level_mask: 1,
                progress: 0x1ff,
                polls: 5
            }
        );
        // Two locked polls read the mask alone.
        assert_eq!(
            reads[..3],
            [PRIV_LEVEL_MASK, PRIV_LEVEL_MASK, PRIV_LEVEL_MASK]
        );
        assert_eq!(reads.len(), 2 + 3 * 2);
    }

    #[test]
    fn the_firmware_boot_wait_ends_at_nouveaus_bound_with_what_it_read() {
        use firmware_boot::*;
        let mut slept = 0u32;
        let error = wait::<()>(
            |offset| Ok(if offset == PRIV_LEVEL_MASK { 1 } else { 0x42 }),
            |_| {
                slept += 1;
                Ok(())
            },
        )
        .unwrap_err();
        assert_eq!(
            error,
            Error::Timeout(Reading {
                priv_level_mask: 1,
                progress: 0x42,
                polls: POLLS
            })
        );
        assert_eq!(slept, POLLS - 1);
        // A failed read ends it at once.
        let mut sleeps = 0;
        assert_eq!(
            wait(
                |_| Err::<u32, _>("bar"),
                |_| {
                    sleeps += 1;
                    Ok(())
                }
            ),
            Err(Error::Access("bar"))
        );
        assert_eq!(sleeps, 0);
    }
}
