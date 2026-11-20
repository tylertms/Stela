// SPDX-License-Identifier: GPL-2.0-only
//! Bounded Intel HDA capability/link setup adapted from Linux
//! sound/hda/core/controller.c, core/ext/controller.c and controllers/intel.c
//! at a804c5742bff224d4cb4d25dd383554d598be32b. See NOTICE.
//! Copyright (C) 2014-2015 Intel Corp; Author Jeeja KP <jeeja.kp@intel.com>
//! Copyright(c) 2004 Intel Corporation
//! Copyright (c) 2004 Takashi Iwai <tiwai@suse.de>
//!                     PeiSen Hou <pshou@realtek.com.tw>
use super::{Controller, Registers, ResetPolicy};
use crate::{Error, Phase, Timer};
use alloc::vec::Vec;

const LLCH: usize = 0x14;
const MAX_CAPS: usize = 32;
const MAX_LINKS: usize = 8;
const CAP_GTS: u16 = 1;
const CAP_ML: u16 = 2;
const CAP_PP: u16 = 3;
const CAP_SPB: u16 = 4;
const CAP_DRSM: u16 = 5;
const LINK_BASE: usize = 0x40;
const LINK_INTERVAL: usize = 0x40;
const LCTL: usize = 4;
const ALT: u32 = 1 << 28;
const SPA: u32 = 1 << 16;
const CPA: u32 = 1 << 23;
const SCF: u32 = 15;
const FREQUENCIES: u32 = 0x3f;
const POWER_TIMEOUT_US: u64 = 500;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkClock {
    pub encoding: u8,
    pub mhz: u16,
}
impl LinkClock {
    fn from_encoding(encoding: u8) -> Self {
        Self {
            encoding,
            mhz: 6u16 << encoding,
        }
    }
}
#[derive(Clone, Copy)]
struct Extent {
    start: usize,
    end: usize,
}
#[derive(Clone, Copy)]
struct Link {
    base: usize,
    frequencies: u32,
}
#[derive(Default)]
pub(super) struct ExtendedCapabilities {
    links: Vec<Link>,
    pub(super) clock: Option<LinkClock>,
}

fn extent<R: Registers>(registers: &R, start: usize, bytes: usize) -> Result<Extent, Error> {
    let end = start
        .checked_add(bytes)
        .ok_or(Error::InvalidExtendedCapability { offset: start })?;
    if end > registers.region_bytes() {
        return Err(Error::InvalidExtendedCapability { offset: start });
    }
    Ok(Extent { start, end })
}
fn control<R: Registers>(registers: &mut R, offset: usize) -> Result<u32, Error> {
    let value = registers.read32(offset);
    if value == u32::MAX {
        return Err(Error::RegisterUnavailable { offset });
    }
    Ok(value)
}

/// Read-only ownership/layout preflight, also used before any native modern
/// PCI preparation. Unknown/active processing paths are never disabled here.
pub(super) fn inspect<R: Registers>(
    registers: &mut R,
    streams: u8,
) -> Result<ExtendedCapabilities, Error> {
    let fixed_end = 0x80 + usize::from(streams) * 0x20;
    let mut ranges = Vec::<Extent>::new();
    let mut seen = [false; 6];
    let mut result = ExtendedCapabilities::default();
    let mut offset = usize::from(registers.read16(LLCH));
    while offset != 0 {
        if ranges.len() == MAX_CAPS
            || offset & 3 != 0
            || offset < fixed_end
            || ranges
                .iter()
                .any(|range| range.start <= offset && offset < range.end)
        {
            return Err(Error::InvalidExtendedCapability { offset });
        }
        extent(registers, offset, 4)?;
        let header = control(registers, offset)?;
        let id = ((header >> 16) & 0xfff) as u16;
        let version = (header >> 28) as u8;
        // Raptor Lake PCH (8086:7a50) reports its multi-link header as
        // version 0; Linux's capability walk never checks the version.
        if version > 1 || !(CAP_GTS..=CAP_DRSM).contains(&id) {
            return Err(Error::UnsupportedExtendedCapability { id, version });
        }
        if seen[usize::from(id)] {
            return Err(Error::InvalidExtendedCapability { offset });
        }
        seen[usize::from(id)] = true;
        let bytes = match id {
            // Only the source-used first global unit is described here;
            // nonzero GTS description layouts remain explicitly unsupported.
            CAP_GTS => 0x40,
            CAP_PP => 0x10 + usize::from(streams) * 0x20,
            CAP_SPB | CAP_DRSM => 8 + usize::from(streams) * 8,
            CAP_ML => {
                extent(registers, offset, 8)?;
                let count = registers
                    .read32(offset + 4)
                    .checked_add(1)
                    .filter(|count| *count <= MAX_LINKS as u32)
                    .ok_or(Error::InvalidExtendedCapability { offset })?
                    as usize;
                LINK_BASE + count * LINK_INTERVAL
            }
            _ => unreachable!(),
        };
        let current = extent(registers, offset, bytes)?;
        if ranges
            .iter()
            .any(|other| other.start < current.end && current.start < other.end)
        {
            return Err(Error::InvalidExtendedCapability { offset });
        }
        ranges.push(current);
        match id {
            // A nonzero descriptor count lengthens GTS past the 0x40 bytes
            // measured above, so a later capability could sit in that tail
            // unseen by the overlap check; such layouts stay refused.
            CAP_GTS => {
                if registers.read32(offset + 4) != 0 {
                    return Err(Error::UnsupportedExtendedCapability { id, version });
                }
            }
            CAP_PP | CAP_SPB | CAP_DRSM => {
                if control(registers, offset + 4)? != 0 {
                    return Err(Error::ProcessingOwned { offset: offset + 4 });
                }
            }
            CAP_ML => {
                let count = (bytes - LINK_BASE) / LINK_INTERVAL;
                for index in 0..count {
                    let base = offset + LINK_BASE + index * LINK_INTERVAL;
                    let capabilities = registers.read32(base);
                    if capabilities & ALT != 0 || capabilities & FREQUENCIES == 0 {
                        // Only the first (conventional) link is ever clocked;
                        // later alternate links are left to firmware.
                        if index == 0 {
                            return Err(Error::UnsupportedLink { offset: base });
                        }
                        continue;
                    }
                    result.links.push(Link {
                        base,
                        frequencies: capabilities & FREQUENCIES,
                    });
                }
            }
            _ => unreachable!(),
        }
        offset = (header & 0xffff) as usize;
    }
    Ok(result)
}

fn select_clock(link: Link, value: u32) -> Result<u8, Error> {
    let offset = link.base + LCTL;
    if (value & SPA != 0) != (value & CPA != 0) {
        return Err(Error::LinkPowerUncertain { offset });
    }
    let current = (value & SCF) as u8;
    let selected = if current == 0 {
        [2u8, 3, 1, 4, 5, 0]
            .into_iter()
            .find(|bit| link.frequencies & (1u32 << bit) != 0)
            .unwrap()
    } else {
        current
    };
    if selected > 5 || link.frequencies & (1u32 << selected) == 0 {
        return Err(Error::UnsupportedLink { offset });
    }
    Ok(selected)
}
impl ExtendedCapabilities {
    /// Validate the exact first link's observable power/clock state without
    /// changing it. Native PCI preparation calls this before any config write.
    /// Reset repeats selection; this observation is not a power grant.
    pub(super) fn preflight_clock(&self, registers: &mut impl Registers) -> Result<(), Error> {
        if let Some(&link) = self.links.first() {
            select_clock(link, control(registers, link.base + LCTL)?)?;
        }
        Ok(())
    }
}

impl<R: Registers, T: Timer> Controller<R, T> {
    pub(super) fn refresh_extended(&mut self) -> Result<(), Error> {
        self.minimum_guard()?;
        if self.config.reset_policy == ResetPolicy::IntelClockGate {
            self.extended = Some(inspect(&mut *self.registers, self.streams.len() as u8)?);
        }
        Ok(())
    }
    /// Last successfully configured conventional external-link clock. This is
    /// controller metadata, not a PCM/transport timestamp or CPU timer source.
    pub fn link_clock(&self) -> Result<Option<LinkClock>, Error> {
        self.minimum_guard()?;
        if self.lifecycle_failed {
            return Err(Error::NeedsReset);
        }
        if self.stopped {
            return Err(Error::Shutdown);
        }
        Ok(self.extended.as_ref().and_then(|cap| cap.clock))
    }
    fn link_power(&mut self, link: Link, on: bool) -> Result<(), Error> {
        self.minimum_guard()?;
        let offset = link.base + LCTL;
        let value = control(&mut *self.registers, offset)?;
        let wanted = if on { SPA | CPA } else { 0 };
        if value & (SPA | CPA) == wanted {
            return Ok(());
        }
        self.registers
            .write32(offset, if on { value | SPA } else { value & !SPA });
        let start = self.timer.now_us();
        loop {
            let value = control(&mut *self.registers, offset)?;
            if value & (SPA | CPA) == wanted {
                return Ok(());
            }
            if self.timer.now_us().wrapping_sub(start)
                >= POWER_TIMEOUT_US.min(u64::from(self.config.transition_timeout_us))
            {
                return Err(Error::Timeout(if on {
                    Phase::LinkPowerUp
                } else {
                    Phase::LinkPowerDown
                }));
            }
            self.timer.relax();
        }
    }
    pub(super) fn configure_extended_link(&mut self) -> Result<(), Error> {
        self.minimum_guard()?;
        let Some(link) = self
            .extended
            .as_ref()
            .and_then(|cap| cap.links.first())
            .copied()
        else {
            return Ok(());
        };
        let offset = link.base + LCTL;
        let value = control(&mut *self.registers, offset)?;
        let selected = select_clock(link, value)?;
        let current = (value & SCF) as u8;
        let result = (|| {
            if current == 0 {
                self.link_power(link, false)?;
                self.delay(100)?;
                let stopped = control(&mut *self.registers, offset)?;
                if stopped & (SPA | CPA) != 0 {
                    return Err(Error::LinkPowerUncertain { offset });
                }
                self.registers
                    .write32(offset, (stopped & !SCF) | u32::from(selected));
                let readback = control(&mut *self.registers, offset)?;
                if readback & SCF != u32::from(selected) || readback & (SPA | CPA) != 0 {
                    return Err(Error::LinkClockRejected { offset });
                }
            }
            self.link_power(link, true)?;
            self.delay(100)?;
            let final_control = control(&mut *self.registers, offset)?;
            if final_control & SCF != u32::from(selected)
                || final_control & (SPA | CPA) != (SPA | CPA)
            {
                return Err(Error::LinkClockRejected { offset });
            }
            Ok(())
        })();
        if let Err(error) = result {
            // A failed clock/power transition never becomes a usable controller;
            // still attempt to restore requested link power while DMA stays pinned.
            let _ = self.link_power(link, true);
            self.delay(100)?;
            return Err(error);
        }
        self.extended.as_mut().unwrap().clock = Some(LinkClock::from_encoding(selected));
        Ok(())
    }
}
