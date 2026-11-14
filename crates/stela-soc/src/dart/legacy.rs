//! Legacy Apple DART t8020/t6000 register and page-entry formats.
//!
//! Linux v6.17 apple-dart.c and io-pgtable-dart.c define the separate old
//! register layout and APPLE_DART/APPLE_DART2 entries. m1n1's dart8020.py
//! independently identifies the stream remap bytes and CONFIG lock. Original
//! 26A434 T8103 SPTM compares the remap words with the DT routes, or with
//! identity when there are none (VA 0xfffffff0270dc51c..0xfffffff0270dc594),
//! and gates CONFIG lock15 on PARAMS2 bit1 (0xd8454); the original mac13g kernel
//! checks page shift and region/bypass capabilities at VA9dc20b4, and decodes
//! a fault's SID from bits27:24 at VA9dc09e8 (full VAs start fffffe000).
//!
//! Original T6000 SPTM, like T8103's, programs source->context bytes at80..8c before
//! enabling streams (VA270bce64..bcecc); mapper entry270be1e0..be22c
//! rejects remapped sources. DMA therefore remaps the incoming SID once,
//! before context enable/TCR/TTBR. Remapped-fault SID provenance remains
//! unchecked and refuses instead of inventing captured metadata.
//! A separate T6000 UMMS owner exposes the native absent capture store and
//! disabled performance state. Unsupported SMMU context selection
//! (TCR23:16), active performance, APF filtering, auxiliary fault/capture
//! fields and the different 64-SID USB4 interface refuse explicitly.
//! This module does not check those board configurations by constructing a
//! successful-looking superset. Translation is cacheless and DMA synchronous.
use super::{Profile, SMMU_BYTES, SMMU_OFFSET, descriptor::Format, legacy_umms};
use crate::Result;
use alloc::{rc::Rc, vec, vec::Vec};
use core::cell::RefCell;
use stela_contracts::Refusal;
use stela_contracts::refuse;
use {
    stela_contracts::bus::Access, stela_contracts::bus::Direction,
    stela_contracts::bus::MmioDevice, stela_contracts::bus::MmioError, stela_contracts::bus::Width,
    stela_contracts::bus::refuse,
};
use {stela_contracts::dma::DmaError, stela_contracts::dma::Iommu, stela_contracts::dma::Memory};
use {stela_contracts::interrupt::AicInput, stela_contracts::interrupt::Wire};

const PARAMS1: u64 = 0;
const PARAMS2: u64 = 4;
const T6000_PARAMS3: u64 = 8;
const REGION: u32 = 1 << 31;
const SUPPORT_BYPASS: u32 = 1;
const SUPPORT_LOCK: u32 = 1 << 1;
const COMMAND: u64 = 0x20;
const INVALIDATE: u32 = 1 << 20;
const SELECT: u64 = 0x34;
const ERROR: u64 = 0x40;
const ERROR_ADDRESS: [u64; 2] = [0x50, 0x54];
const CONFIG: u64 = 0x60;
const LOCK: u32 = 1 << 15;
const REMAP: u64 = 0x80;
const ENABLED: u64 = 0xfc;
const TCR: u64 = 0x100;
const TTBR: u64 = 0x200;
const TRANSLATE: u32 = 1 << 7;
const BYPASS: u32 = 1 << 8;
const BYPASS_APF: u32 = 1 << 12;
const SMMU_CONTEXT: u32 = 0xff << 16;
const TCR_KNOWN: u32 = TRANSLATE | BYPASS | BYPASS_APF;
const TTBR_VALID: u32 = 1 << 31;
const TTBR_ADDRESS: u32 = !TTBR_VALID;
const TTBR_SHIFT: u32 = 12;
const TABLES: usize = 4;
const IOVA_BITS: u32 = 32;
const T8020_PA_BITS: u32 = 36;
const T6000_PA_BITS: u32 = 42;
const VALID: u64 = 1;
const SUBPAGE_DISABLE: u64 = 1 << 1;
const SUBPAGE_UNIT: u64 = 4;
const FLAG: u32 = 1 << 31;
const WRITE_ACCESS: u32 = 1 << 10;
const SID_SHIFT: u32 = 24;
const NO_TTBR: u32 = 1;
const NO_PMD: u32 = 1 << 1;
const NO_PTE: u32 = 1 << 2;
const WRITE_FAULT: u32 = 1 << 3;
const READ_FAULT: u32 = 1 << 4;
const REGION_FAULT: u32 = 1 << 7;
const T6000_ERROR_ACK_CODES: u32 = 0xb7f;
const PERF_CONTROL: u64 = 0x1000;
const PERF_STATUS: u64 = 0x100c;
const PERF_ACK_CODES: u32 = 0x2d;

pub(super) struct Dart {
    profile: Profile,
    streams: usize,
    page_shift: u32,
    powered: bool,
    clock_enabled: bool,
    quiesced: bool,
    reset: bool,
    apf: bool,
    select: u32,
    enabled: u32,
    locked: bool,
    remap: [u8; 16],
    tcr: Vec<u32>,
    ttbr: Vec<[u32; TABLES]>,
    error: u32,
    error_address: u64,
    irq: Option<AicInput>,
    level: bool,
    shared_fault: Option<(Rc<RefCell<Vec<bool>>>, usize)>,
    smmu: Option<legacy_umms::Umms>,
    performance: legacy_umms::Performance,
}

impl Dart {
    pub(super) fn bind_shared_irq(&mut self, irq: &super::SharedIrq) -> Result<()> {
        if self.irq != Some(irq.input) {
            return Err(refuse!("DART shared IRQ does not match its physical input"));
        }
        if self.level
            || self
                .shared_fault
                .as_ref()
                .is_some_and(|(levels, _)| levels.borrow().iter().any(|&level| level))
        {
            return Err(refuse!("an asserted DART IRQ source cannot be rebound"));
        }
        self.shared_fault = Some(irq.source());
        Ok(())
    }

    pub(super) const fn stream_capacity(&self) -> usize {
        self.streams
    }

    pub(super) fn new(profile: Profile, streams: usize, irq: Option<AicInput>) -> Self {
        Self {
            profile,
            streams,
            page_shift: 14,
            powered: true,
            clock_enabled: true,
            quiesced: false,
            reset: false,
            apf: false,
            select: 0,
            enabled: 0,
            locked: false,
            remap: core::array::from_fn(|sid| sid as u8),
            tcr: vec![0; streams],
            ttbr: vec![[0; TABLES]; streams],
            error: 0,
            error_address: 0,
            irq,
            level: false,
            shared_fault: None,
            smmu: None,
            performance: legacy_umms::Performance::default(),
        }
    }

    pub(super) fn group(
        profile: Profile,
        streams: usize,
        irq: Option<AicInput>,
        count: usize,
    ) -> Vec<Self> {
        let levels = Rc::new(RefCell::new(vec![false; count]));
        (0..count)
            .map(|index| {
                let mut dart = Self::new(profile, streams, irq);
                dart.shared_fault = Some((levels.clone(), index));
                dart
            })
            .collect()
    }

    pub(super) fn with_apf(mut self, slices: usize) -> Self {
        self.apf = slices != 0;
        self
    }

    pub(super) fn supports_smmu(&self) -> bool {
        self.profile == Profile::T6000
    }

    pub(super) fn enable_smmu(&mut self) -> Result<()> {
        if !self.supports_smmu() {
            return Err(refuse!(
                "legacy DART paired SMMU capture format is unsupported"
            ));
        }
        if self.smmu.is_some() {
            return Err(refuse!("DART already has a paired SMMU instance"));
        }
        self.smmu = Some(legacy_umms::Umms::default());
        Ok(())
    }

    pub(super) fn has_smmu(&self) -> bool {
        self.smmu.is_some()
    }

    pub(super) fn set_power(&mut self, powered: bool, retention: bool, access: &mut Access<'_>) {
        if self.powered && !powered && !retention {
            self.clear(access);
        }
        self.powered = powered;
    }

    pub(super) fn set_quiesced(&mut self, quiesced: bool) {
        self.quiesced = quiesced;
    }

    pub(super) fn set_clock(&mut self, enabled: bool) {
        self.clock_enabled = enabled;
    }

    pub(super) fn set_reset(&mut self, asserted: bool, access: &mut Access<'_>) {
        if asserted && !self.reset {
            self.clear(access);
        }
        self.reset = asserted;
    }

    fn clear(&mut self, access: &mut Access<'_>) {
        self.select = 0;
        self.enabled = 0;
        self.locked = false;
        self.remap = core::array::from_fn(|sid| sid as u8);
        self.tcr.fill(0);
        self.ttbr.fill([0; TABLES]);
        self.error = 0;
        self.error_address = 0;
        if let Some(smmu) = &mut self.smmu {
            smmu.clear();
        }
        self.performance = legacy_umms::Performance::default();
        self.signal(access);
    }

    fn unavailable(&self) -> Option<&'static str> {
        if !self.powered {
            Some("DART power domain is off")
        } else if !self.clock_enabled {
            Some("DART functional clock is gated")
        } else if self.reset {
            Some("DART reset is asserted")
        } else {
            None
        }
    }

    fn mask(&self) -> u32 {
        (1 << self.streams) - 1
    }

    fn sid_register(&self, offset: u64, base: u64, stride: u64) -> Option<usize> {
        let offset = offset.checked_sub(base)?;
        let sid = usize::try_from(offset / stride).ok()?;
        (offset.is_multiple_of(4) && sid < self.streams).then_some(sid)
    }

    fn target(&self, entry: u64, leaf: bool) -> Result<u64, DmaError> {
        let (format, bits) = match self.profile {
            Profile::T8020 => (Format::Dart1, T8020_PA_BITS),
            Profile::T6000 => (Format::Dart2, T6000_PA_BITS),
            Profile::T8110 | Profile::T8110Rev20 | Profile::T8110Rev22 => {
                unreachable!("legacy format")
            }
        };
        format.target(entry, bits, self.page(), leaf)
    }

    fn entry(&self, memory: &mut dyn Memory, table: u64, index: u64) -> Result<u64, DmaError> {
        let mut bytes = [0; 8];
        let bits = self.page_shift - 3;
        if !memory.read(table + (index & ((1 << bits) - 1)) * 8, &mut bytes) {
            return Err(DmaError::Memory);
        }
        Ok(u64::from_le_bytes(bytes))
    }

    pub(super) fn walk(
        &self,
        memory: &mut dyn Memory,
        input_sid: usize,
        iova: u64,
        last: u64,
        write: bool,
    ) -> Result<u64, DmaError> {
        if let Some(reason) = self.unavailable() {
            return Err(DmaError::Unsupported(reason.into()));
        }
        if self.quiesced {
            return Err(DmaError::Unsupported(refuse!("DART DMA is quiesced")));
        }
        if input_sid >= self.streams {
            return Err(DmaError::Unsupported(refuse!(
                "DART incoming stream is outside the native profile"
            )));
        }
        let sid = usize::from(self.remap[input_sid]);
        if sid >= self.streams {
            return Err(DmaError::Unsupported(refuse!(
                "DART remap target is outside the native profile"
            )));
        }
        if self.enabled & (1 << sid) == 0 {
            return Err(DmaError::Fault(NO_TTBR));
        }
        let tcr = self.tcr[sid];
        if tcr & (TRANSLATE | BYPASS) == TRANSLATE | BYPASS {
            return Err(DmaError::Unsupported(refuse!(
                "DART simultaneous translation and bypass"
            )));
        }
        if self.apf && tcr & BYPASS_APF == 0 {
            return Err(DmaError::Unsupported(refuse!(
                "legacy DART address permission filtering"
            )));
        }
        if tcr & BYPASS != 0 {
            return Ok(iova);
        }
        if tcr & TRANSLATE == 0 || iova >> IOVA_BITS != 0 {
            return Err(DmaError::Fault(NO_TTBR));
        }
        let bits = self.page_shift - 3;
        let ttbr_index = (iova >> (self.page_shift + 2 * bits)) as usize;
        let ttbr = self.ttbr[sid][ttbr_index];
        if ttbr & TTBR_VALID == 0 {
            return Err(DmaError::Fault(NO_TTBR));
        }
        let root = u64::from(ttbr & TTBR_ADDRESS) << TTBR_SHIFT;
        let pa_bits = if self.profile == Profile::T8020 {
            T8020_PA_BITS
        } else {
            T6000_PA_BITS
        };
        if root >> pa_bits != 0 {
            return Err(DmaError::Unsupported(refuse!(
                "DART root address exceeds its native profile"
            )));
        }
        if root & (self.page() - 1) != 0 {
            return Err(DmaError::Unsupported(refuse!(
                "unaligned legacy DART root table"
            )));
        }
        let first = self.entry(memory, root, iova >> (self.page_shift + bits))?;
        if first & VALID == 0 {
            return Err(DmaError::Fault(NO_PMD));
        }
        let page = self.entry(memory, self.target(first, false)?, iova >> self.page_shift)?;
        if page & VALID == 0 {
            return Err(DmaError::Fault(NO_PTE));
        }
        let target = self.target(page, true)?;
        let (no_read, no_write) = if self.profile == Profile::T8020 {
            (1 << 8, 1 << 7)
        } else {
            (1 << 3, 1 << 2)
        };
        if write && page & no_write != 0 {
            return Err(DmaError::Fault(WRITE_FAULT));
        }
        if !write && page & no_read != 0 {
            return Err(DmaError::Fault(READ_FAULT));
        }
        let offset = iova & (self.page() - 1);
        let subpage_disabled = self.profile == Profile::T8020 && page & SUBPAGE_DISABLE != 0;
        if !subpage_disabled {
            let first = page >> 52 & 0xfff;
            let end = page >> 40 & 0xfff;
            if offset / SUBPAGE_UNIT < first || (last & (self.page() - 1)) / SUBPAGE_UNIT > end {
                return Err(DmaError::Fault(REGION_FAULT));
            }
        }
        Ok(target + offset)
    }

    fn signal(&mut self, access: &mut Access<'_>) {
        let level = self.error & FLAG != 0;
        if self.level == level {
            return;
        }
        self.level = level;
        let level = if let Some((levels, index)) = &self.shared_fault {
            let mut levels = levels.borrow_mut();
            levels[*index] = level;
            levels.iter().any(|&level| level)
        } else {
            level
        };
        if let Some(irq) = self.irq {
            access.interrupts.set_level(Wire::Aic(irq), level);
        }
    }

    fn acknowledge_error(&mut self, value: u32) -> Result<()> {
        if self.profile != Profile::T6000 {
            if value != 0 && value & FLAG == 0 {
                return Err(refuse!(
                    "legacy DART code-only fault acknowledgement is not specified"
                ));
            }
            if value & FLAG != 0 {
                self.error = 0;
            }
            return Ok(());
        }
        if value & FLAG != 0 {
            // The original clear-all path reads ERROR and writes that exact
            // captured word (SPTM270bb83c..bb840). It does not establish
            // FLAG-only or stale/modified metadata acknowledgement effects.
            if value != self.error {
                return Err(refuse!(
                    "T6000 DART acknowledgement is not its captured fault snapshot"
                ));
            }
            self.error = 0;
        } else if value != 0 {
            // SPTM270bbc64..bbc9c writes one selected code and reads back.
            // Validate the entire write before changing the first capture.
            if value & !T6000_ERROR_ACK_CODES != 0 || value.count_ones() != 1 {
                return Err(refuse!("unknown T6000 DART code acknowledgement"));
            }
            if self.error & value != 0 {
                self.error &= !value;
                if self.error & T6000_ERROR_ACK_CODES == 0 {
                    self.error = 0;
                }
            }
        }
        Ok(())
    }
}

impl Iommu for Dart {
    fn page(&self) -> u64 {
        1 << self.page_shift
    }

    fn translate(
        &mut self,
        memory: &mut dyn Memory,
        stream: usize,
        iova: u64,
        last: u64,
        write: bool,
        access: &mut Access<'_>,
    ) -> Result<u64, DmaError> {
        debug_assert!(last >= iova && last / self.page() == iova / self.page());
        let result = self.walk(memory, stream, iova, last, write);
        if let Err(DmaError::Fault(code)) = result {
            if stream < self.streams && usize::from(self.remap[stream]) != stream {
                return Err(DmaError::Unsupported(refuse!(
                    "legacy DART remapped fault source-versus-context SID provenance"
                )));
            }
            if self.error & FLAG == 0 {
                self.error = FLAG
                    | (stream as u32 & 0xf) << SID_SHIFT
                    | if write { WRITE_ACCESS } else { 0 }
                    | code;
                self.error_address = iova;
            }
            self.signal(access);
        }
        result
    }
}

impl MmioDevice for Dart {
    fn name(&self) -> &'static str {
        if self.profile == Profile::T8020 {
            "dart-t8020"
        } else {
            "dart-t6000"
        }
    }

    fn read(&mut self, offset: u64, width: Width, _: &mut Access<'_>) -> Result<u64, MmioError> {
        let fail = |reason: Refusal| refuse(self.name(), offset, width, Direction::Read, reason);
        if width != Width::Word || !offset.is_multiple_of(4) {
            return Err(fail(refuse!(
                "legacy DART registers are aligned 32-bit words"
            )));
        }
        if let Some(reason) = self.unavailable() {
            return Err(fail(reason.into()));
        }
        if (SMMU_OFFSET..SMMU_OFFSET + SMMU_BYTES).contains(&offset) {
            return self
                .smmu
                .as_ref()
                .ok_or_else(|| fail(refuse!("legacy DART has no paired UMMS instance")))?
                .read(offset - SMMU_OFFSET)
                .map(u64::from)
                .map_err(fail);
        }
        let value = match offset {
            PARAMS1 => REGION | self.page_shift << 24,
            PARAMS2 => SUPPORT_BYPASS | SUPPORT_LOCK,
            // Original T6000 SPTM270bc554..bc5b8 derives the native PA
            // aperture from bits29:24 and requires at most42 address bits.
            T6000_PARAMS3 if self.profile == Profile::T6000 => T6000_PA_BITS << 24,
            COMMAND => 0,
            SELECT => self.select,
            ERROR => self.error,
            CONFIG => {
                if self.locked {
                    LOCK
                } else {
                    0
                }
            }
            ENABLED => self.enabled,
            PERF_CONTROL if self.profile == Profile::T6000 => self.performance.control(),
            PERF_STATUS if self.profile == Profile::T6000 => self.performance.status(),
            _ if ERROR_ADDRESS.contains(&offset) => {
                (self.error_address >> (32 * usize::from(offset == ERROR_ADDRESS[1]))) as u32
            }
            _ if (REMAP..REMAP + 16).contains(&offset) => {
                let sid = (offset - REMAP) as usize;
                u32::from_le_bytes(
                    self.remap[sid..sid + 4]
                        .try_into()
                        .expect("four remap bytes"),
                )
            }
            _ if (TCR..TCR + 16 * 4).contains(&offset) => {
                let sid = self.sid_register(offset, TCR, 4).ok_or_else(|| {
                    fail(refuse!("legacy DART stream is outside the native profile"))
                })?;
                self.tcr[sid]
            }
            _ if (TTBR..TTBR + 16 * 16).contains(&offset) => {
                let sid = self.sid_register(offset, TTBR, 16).ok_or_else(|| {
                    fail(refuse!("legacy DART stream is outside the native profile"))
                })?;
                self.ttbr[sid][((offset - TTBR) % 16 / 4) as usize]
            }
            _ => {
                return Err(fail(refuse!(
                    "unknown legacy DART register or unsupported capture/filter view"
                )));
            }
        };
        Ok(u64::from(value))
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        let name = self.name();
        let fail = |reason: Refusal| refuse(name, offset, width, Direction::Write, reason);
        if width != Width::Word || !offset.is_multiple_of(4) {
            return Err(fail(refuse!(
                "legacy DART registers are aligned 32-bit words"
            )));
        }
        if let Some(reason) = self.unavailable() {
            return Err(fail(reason.into()));
        }
        let value = value as u32;
        if (SMMU_OFFSET..SMMU_OFFSET + SMMU_BYTES).contains(&offset) {
            return self
                .smmu
                .as_mut()
                .ok_or_else(|| fail(refuse!("legacy DART has no paired UMMS instance")))?
                .write(offset - SMMU_OFFSET, value)
                .map_err(fail);
        }
        match offset {
            COMMAND if value == INVALIDATE => {}
            // T6000 powerup270bcfcc and powerdown270bdf60 initialize
            // an idle command word before polling. There is no queued work
            // or translation cache to cancel in this native profile.
            COMMAND if self.profile == Profile::T6000 && value == 0 => {}
            COMMAND => return Err(fail(refuse!("unknown legacy DART stream command"))),
            SELECT => self.select = value & self.mask(),
            ERROR => self.acknowledge_error(value).map_err(fail)?,
            CONFIG => {
                if value & !LOCK != 0 {
                    return Err(fail(refuse!("unknown legacy DART CONFIG bits")));
                }
                self.locked |= value & LOCK != 0;
            }
            ENABLED => {
                if self.locked {
                    return Err(fail(refuse!(
                        "legacy DART enabled-stream lock coverage is not specified"
                    )));
                }
                self.enabled = value & self.mask();
            }
            PERF_CONTROL if self.profile == Profile::T6000 => {
                self.performance.disable(value).map_err(fail)?;
            }
            PERF_STATUS if self.profile == Profile::T6000 => {
                self.performance
                    .acknowledge(value, PERF_ACK_CODES)
                    .map_err(fail)?;
            }
            _ if (REMAP..REMAP + 16).contains(&offset) => {
                if self.locked {
                    return Err(fail(refuse!(
                        "legacy DART remap lock coverage is not specified"
                    )));
                }
                let sid = (offset - REMAP) as usize;
                if value
                    .to_le_bytes()
                    .iter()
                    .any(|&sid| usize::from(sid) >= self.streams)
                {
                    return Err(fail(refuse!(
                        "legacy DART remap target is outside the native profile"
                    )));
                }
                self.remap[sid..sid + 4].copy_from_slice(&value.to_le_bytes());
            }
            _ if (TCR..TCR + 16 * 4).contains(&offset) => {
                let sid = self.sid_register(offset, TCR, 4).ok_or_else(|| {
                    fail(refuse!("legacy DART stream is outside the native profile"))
                })?;
                if value & SMMU_CONTEXT != 0 {
                    return Err(fail(refuse!("legacy DART SMMU stream context selection")));
                }
                if value & !TCR_KNOWN != 0 {
                    return Err(fail(refuse!("unknown legacy DART TCR bits")));
                }
                if !self.locked {
                    self.tcr[sid] = value;
                }
            }
            _ if (TTBR..TTBR + 16 * 16).contains(&offset) => {
                let sid = self.sid_register(offset, TTBR, 16).ok_or_else(|| {
                    fail(refuse!("legacy DART stream is outside the native profile"))
                })?;
                if !self.locked {
                    self.ttbr[sid][((offset - TTBR) % 16 / 4) as usize] = value;
                }
            }
            _ => {
                return Err(fail(refuse!(
                    "unknown legacy DART register or unsupported capture/filter view"
                )));
            }
        }
        self.signal(access);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
