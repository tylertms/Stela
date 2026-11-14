// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! Native NVMe transport and requester-domain memory. No physical
//! address is interpreted as a CPU pointer or a device IOVA.
use super::*;
use crate::bus::{
    dma::{
        DmaBuffer,
        mapping::{Direction, DmaDomain, DmaError, DmaMapping},
        memory::{CacheAttribute, CpuMapping, PageSource},
    },
    mmio::{
        Mmio,
        owned::{Mapping, Protection},
    },
};
use alloc::{boxed::Box, string::String, sync::Arc};
use core::{mem, ptr};

/// Exclusive, fallible controller registers, kept through final retirement.
///
/// # Safety
/// Implementors keep the controller BAR mapping, CPU-root/resource
/// reservation and its mapping epoch. The paired Lifecycle keeps PCI/reset
/// hierarchy ownership through register retirement. Every access checks its
/// complete range and alignment, uses the device's UC mapping and
/// preserves device ordering. Failures
/// must remain errors, never synthetic all-ones reads or acknowledged writes.
/// No other controller or register accessor may drive this controller.
pub unsafe trait Registers {
    fn read32(&self, offset: usize) -> Result<u32, BlockError>;
    fn write32(&self, offset: usize, value: u32) -> Result<(), BlockError>;
    fn read64(&self, offset: usize) -> Result<u64, BlockError> {
        let high = offset.checked_add(4).ok_or_else(register_error)?;
        Ok(u64::from(self.read32(offset)?) | u64::from(self.read32(high)?) << 32)
    }
    fn write64(&self, offset: usize, value: u64) -> Result<(), BlockError> {
        let high = offset.checked_add(4).ok_or_else(register_error)?;
        self.write32(offset, value as u32)?;
        self.write32(high, (value >> 32) as u32)
    }
    /// End CPU views/mapping while the configuration/resource owner is held.
    ///
    /// # Safety
    /// Controller/submission paths are stopped, requester BME is contained,
    /// domain payload writes drained, and all RAM mappings unmapped. No access
    /// to these registers may follow success. Failure keeps the owner.
    unsafe fn retire(&mut self) -> Result<(), BlockError>;
}

/// The kept PCI/child hierarchy; a BDF is not this permission.
///
/// # Safety
/// Implementors keep exclusive controller/configuration/reset/resource and
/// parent root owners until release. On entry, the entire hierarchy
/// must have acknowledged BME-clear, with no uncontrolled DMA engine. `prepare`
/// establishes D0 and memory decode while preserving containment.
/// `enable_dma` establishes memory decode/BME for this exact domain-bound requester; errors after any
/// possible effect keep that permission. `disable_dma` acknowledges
/// BME-clear and excludes new requester DMA. It is not a payload-write drain.
/// Failed `prepare` does not authorize any NVMe MMIO: the caller quarantines
/// the complete owner because no generic recovery can establish D0.
/// `release` returns owners only after its caller's completed teardown; failed
/// operations keep the same hardware epoch and are retryable or quarantined.
pub unsafe trait Lifecycle {
    fn prepare(&mut self) -> Result<(), BlockError>;
    fn enable_dma(&mut self) -> Result<(), BlockError>;
    fn disable_dma(&mut self) -> Result<(), BlockError>;
    fn release(&mut self) -> Result<(), BlockError>;
}

/// An exclusively reserved, owned UC BAR mapping.
pub struct MappingRegisters {
    mapping: Mapping,
}
impl MappingRegisters {
    pub fn new(mapping: Mapping) -> Result<Self, (BlockError, Mapping)> {
        if mapping.attribute() != Ok(CacheAttribute::Uncached)
            || mapping.protection() != Ok(Protection::ReadWrite)
            || !mapping
                .address(0, 0x2000)
                .is_ok_and(|p| (p.as_ptr() as usize).is_multiple_of(align_of::<u32>()))
        {
            return Err((register_error(), mapping));
        }
        Ok(Self { mapping })
    }
    fn word(&self, offset: usize) -> Result<*mut u32, BlockError> {
        if !offset.is_multiple_of(4) {
            return Err(register_error());
        }
        let pointer = self
            .mapping
            .address(offset, size_of::<u32>())
            .map_err(|_| register_error())?;
        if !(pointer.as_ptr() as usize).is_multiple_of(align_of::<u32>()) {
            return Err(register_error());
        }
        Ok(pointer.as_ptr().cast())
    }
}
// SAFETY: the owned Mapping keeps the exclusive resource, UC policy,
// CPU root and span; checked volatile accesses preserve their exact effects.
unsafe impl Registers for MappingRegisters {
    fn read32(&self, offset: usize) -> Result<u32, BlockError> {
        // SAFETY: word validated the complete live aligned UC register.
        Ok(unsafe { self.word(offset)?.read_volatile() })
    }
    fn write32(&self, offset: usize, value: u32) -> Result<(), BlockError> {
        // SAFETY: same live exclusive register boundary.
        unsafe { self.word(offset)?.write_volatile(value) };
        Ok(())
    }
    unsafe fn retire(&mut self) -> Result<(), BlockError> {
        // SAFETY: the caller establishes stopped/drained exclusive access.
        unsafe { self.mapping.retire() }.map_err(|_| register_error())
    }
}
fn register_error() -> BlockError {
    BlockError::Protocol("NVMe owned register mapping")
}
fn dma_error() -> BlockError {
    BlockError::Protocol("NVMe requester-domain DMA")
}
pub(super) fn map_dma_error(error: DmaError) -> BlockError {
    match error {
        DmaError::IommuUnavailable => BlockError::Unsupported("NVMe requester IOMMU domain"),
        DmaError::CoherenceUnavailable => BlockError::Unsupported("NVMe coherent CQ observation"),
        DmaError::DrainUnavailable => BlockError::Unsupported("NVMe original-domain payload drain"),
        DmaError::VisibilityFailed => BlockError::Protocol("NVMe DMA cache visibility"),
        DmaError::InvalidationFailed => BlockError::Protocol("NVMe DMA invalidation"),
        _ => dma_error(),
    }
}

pub(super) enum RegisterIo {
    Legacy(Mmio),
    Owned(Option<Box<dyn Registers>>),
}
impl RegisterIo {
    pub(super) fn read32(&self, offset: usize) -> Result<u32, BlockError> {
        match self {
            Self::Legacy(m) => Ok(m.read32(offset)),
            Self::Owned(Some(m)) => m.read32(offset),
            _ => Err(register_error()),
        }
    }
    pub(super) fn write32(&self, offset: usize, value: u32) -> Result<(), BlockError> {
        match self {
            Self::Legacy(m) => {
                m.write32(offset, value);
                Ok(())
            }
            Self::Owned(Some(m)) => m.write32(offset, value),
            _ => Err(register_error()),
        }
    }
    pub(super) fn read64(&self, offset: usize) -> Result<u64, BlockError> {
        match self {
            Self::Legacy(m) => Ok(m.read64(offset)),
            Self::Owned(Some(m)) => m.read64(offset),
            _ => Err(register_error()),
        }
    }
    pub(super) fn write64(&self, offset: usize, value: u64) -> Result<(), BlockError> {
        match self {
            Self::Legacy(m) => {
                m.write64(offset, value);
                Ok(())
            }
            Self::Owned(Some(m)) => m.write64(offset, value),
            _ => Err(register_error()),
        }
    }
    unsafe fn retire(&mut self) -> Result<(), BlockError> {
        if let Self::Owned(Some(r)) = self {
            // SAFETY: forwarded stopped/drained contract from Controller.
            unsafe { r.retire()? };
        }
        Ok(())
    }
    fn quarantine(&mut self) {
        if let Self::Owned(owner) = self
            && let Some(r) = owner.take()
        {
            mem::forget(r);
        }
    }
}

pub(super) struct Memory {
    cpu: CpuMapping,
    dma: DmaMapping,
    cpu_owned: bool,
    published: bool,
}
pub(super) enum Buffer {
    Legacy(DmaBuffer),
    Owned(Memory),
    Empty,
}
impl Buffer {
    fn allocate(
        source: &dyn PageSource,
        domain: Arc<dyn DmaDomain>,
        bytes: usize,
        direction: Direction,
    ) -> Result<Self, BlockError> {
        let pages = source
            .allocate(bytes, PAGE, CacheAttribute::WriteBack)
            .map_err(|_| BlockError::NoDmaMemory)?;
        if pages.logical_bytes() < bytes || pages.page_shift() != 12 {
            return Err(BlockError::Unsupported("NVMe requires 4 KiB DMA pages"));
        }
        let cpu = pages
            .cpu_mapping(CacheAttribute::WriteBack)
            .map_err(|_| BlockError::Protocol("NVMe retained CPU mapping"))?;
        let dma = DmaMapping::map(domain, pages, direction, u64::MAX).map_err(map_dma_error)?;
        Ok(Self::Owned(Memory {
            cpu,
            dma,
            cpu_owned: true,
            published: false,
        }))
    }
    pub(super) fn address(&self) -> Result<u64, BlockError> {
        match self {
            Self::Legacy(b) => Ok(b.bus_address()),
            Self::Owned(b) => b.dma.address().map_err(map_dma_error),
            Self::Empty => Err(dma_error()),
        }
    }
    pub(super) fn bytes(&self) -> &[u8] {
        match self {
            Self::Legacy(b) => b.bytes(),
            Self::Owned(b) => {
                assert!(b.cpu_owned, "CPU view while NVMe owns DMA");
                // SAFETY: allocation is held and CPU ownership established
                // by completion or before any publication; no device writer.
                unsafe {
                    core::slice::from_raw_parts(
                        b.cpu
                            .address(0, b.cpu.bytes())
                            .expect("validated CPU span")
                            .as_ptr(),
                        b.cpu.bytes(),
                    )
                }
            }
            Self::Empty => &[],
        }
    }
    pub(super) fn bytes_mut(&mut self) -> &mut [u8] {
        match self {
            Self::Legacy(b) => b.bytes_mut(),
            Self::Owned(b) => {
                assert!(b.cpu_owned, "CPU write while NVMe owns DMA");
                // SAFETY: exclusive CPU ownership and kept mapped span.
                unsafe {
                    core::slice::from_raw_parts_mut(
                        b.cpu
                            .address(0, b.cpu.bytes())
                            .expect("validated CPU span")
                            .as_ptr(),
                        b.cpu.bytes(),
                    )
                }
            }
            Self::Empty => &mut [],
        }
    }
    pub(super) fn write<T: crate::bus::dma::DmaValue>(&mut self, offset: usize, value: T) {
        let bytes = self.bytes_mut();
        assert!(
            offset
                .checked_add(size_of::<T>())
                .is_some_and(|end| end <= bytes.len())
        );
        // SAFETY: bytes_mut established a unique CPU-owned mapped slice, and
        // the checked complete T span above keeps offset within that allocation.
        // Pointer arithmetic alone does not dereference the cast T pointer;
        // its alignment is checked below before the volatile write.
        let p = unsafe { bytes.as_mut_ptr().add(offset) }.cast::<T>();
        assert!((p as usize).is_multiple_of(align_of::<T>()));
        // SAFETY: complete aligned span in uniquely CPU-owned memory.
        unsafe { ptr::write_volatile(p, value) };
    }
    /// Write a whole submission entry through one checked CPU view. The
    /// caller publishes it only after all words are written.
    pub(super) fn write_submission(&mut self, offset: usize, words: [u32; 16]) {
        let entry = &mut self.bytes_mut()[offset..][..size_of_val(&words)];
        let pointer = entry.as_mut_ptr().cast::<[u32; 16]>();
        assert!(pointer.is_aligned());
        // SAFETY: bounds and alignment were checked for the complete array.
        // The queue granted CPU ownership of this entry until publication.
        unsafe { pointer.write_volatile(words) };
    }
    pub(super) fn coherent(&self, offset: usize) -> Result<u32, BlockError> {
        match self {
            Self::Legacy(b) => Ok(b.read(offset)),
            Self::Owned(b) => b.dma.read_coherent_u32(offset).map_err(map_dma_error),
            Self::Empty => Err(dma_error()),
        }
    }
    pub(super) fn publish(&mut self) {
        if let Self::Owned(b) = self {
            b.published = true;
        }
    }
    /// CPU writes to a submission ring the controller may still read.
    ///
    /// # Safety
    /// The buffer is device-read-only, and no entry the controller has yet
    /// to fetch is rewritten.
    pub(super) unsafe fn resume_writes(&mut self) {
        if let Self::Owned(b) = self {
            b.cpu_owned = true;
        }
    }
    pub(super) fn give(&mut self) -> Result<(), BlockError> {
        if let Self::Owned(b) = self {
            if b.cpu_owned {
                b.dma
                    .sync_for_device(0..b.dma.bytes())
                    .map_err(map_dma_error)?;
                b.cpu_owned = false;
            }
            b.published = true;
        }
        Ok(())
    }
    /// Only after command completion or acknowledged stopped/drained DMA.
    pub(super) unsafe fn take(&mut self) -> Result<(), BlockError> {
        if let Self::Owned(b) = self
            && !b.cpu_owned
        {
            // SAFETY: caller establishes hardware completion.
            unsafe { b.dma.sync_for_cpu(0..b.dma.bytes()) }.map_err(map_dma_error)?;
            b.cpu_owned = true;
        }
        Ok(())
    }
    unsafe fn unmap(&mut self) -> Result<(), BlockError> {
        if let Self::Owned(b) = self {
            // SAFETY: stopped requester and completed original-domain drain.
            unsafe { b.dma.unmap() }.map_err(map_dma_error)?;
            b.published = false;
        }
        *self = Self::Empty;
        Ok(())
    }
    fn quarantine(&mut self) {
        let old = mem::replace(self, Self::Empty);
        mem::forget(old);
    }
}
impl Drop for Memory {
    fn drop(&mut self) {
        if !self.published {
            // SAFETY: IOVA was never exposed, or unmap already succeeded.
            let _ = unsafe { self.dma.unmap() };
        }
        // A published lease automatically keeps pins/domain on Drop. The
        // controller additionally keeps the CPU/config/root owners on failure.
    }
}
/// Monotonic observations, not inferred hardware state. A failed step leaves
/// its phase unchanged; completed hardware steps are never replayed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Teardown {
    Unadmitted,
    Unprepared,
    Preparing,
    Active,
    Stopped,
    Contained,
    Drained,
    RamUnmapped,
    RegistersRetired,
    Released,
}

pub(super) struct Owner {
    pub(super) lifecycle: Box<dyn Lifecycle>,
    pub(super) domain: Arc<dyn DmaDomain>,
}

/// Initialize real admin/I/O queues and discover ordinary NVM namespaces using
/// the same command/namespace engine as the boot NVMe API, with the same
/// queue depth. Completions are polled; it does not claim interrupts or
/// hotplug. A reset after a
/// timeout clears BME and drains the domain before the CPU reuses any buffer.
/// # Safety
/// The supplied register/lifecycle owners and domain must denote the same
/// exclusive controller, its requester and every DMA alias/root/reset
/// epoch. Bind that real domain before entry, and keep the complete hierarchy
/// BME-contained. This identity cannot be established from numeric BDFs or by
/// combining independently obtained type-erased owners. `quirks` are the
/// controller's by its PCI ID and board ([`super::Quirks`]).
pub unsafe fn attach(
    registers: Box<dyn Registers>,
    lifecycle: Box<dyn Lifecycle>,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    clock: Clock,
    location: String,
    quirks: super::Quirks,
) -> Result<Attached, BlockError> {
    // SAFETY: this is the same exclusive original-controller contract; the
    // production command budget is not exposed as an attachment capability.
    unsafe {
        attach_with_timeout(
            registers,
            lifecycle,
            source,
            domain,
            clock,
            location,
            COMMAND_TIMEOUT_US,
            quirks,
        )
    }
}

#[expect(
    clippy::too_many_arguments,
    reason = "Keep the independently supplied owners, clock, label, command budget and hardware quirks explicit."
)]
unsafe fn attach_with_timeout(
    registers: Box<dyn Registers>,
    lifecycle: Box<dyn Lifecycle>,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    clock: Clock,
    location: String,
    command_timeout_us: u64,
    quirks: super::Quirks,
) -> Result<Attached, BlockError> {
    // Install the complete owner before fallible allocation or hardware effects.
    let mut controller = Controller {
        registers: RegisterIo::Owned(Some(registers)),
        doorbell_stride: 0,
        admin: Queue {
            submissions: Buffer::Empty,
            completions: Buffer::Empty,
            id: 0,
            entries: ADMIN_ENTRIES,
            tail: 0,
            head: 0,
            phase: true,
            consumed: 0,
        },
        io: Queue {
            submissions: Buffer::Empty,
            completions: Buffer::Empty,
            id: 1,
            entries: IO_ENTRIES,
            tail: 0,
            head: 0,
            phase: true,
            consumed: 0,
        },
        admin_data: Buffer::Empty,
        pool: Pool::new(alloc::vec::Vec::new()),
        held: (0..DEPTH).map(|_| alloc::vec::Vec::new()).collect(),
        tags: (0..DEPTH).map(|_| Tag::Free).collect(),
        recovery: None,
        signalled: false,
        owed: false,
        max_transfer: BOUNCE_BYTES,
        max_command_bytes: MAX_COMMAND_BYTES,
        shutdown_timeout_us: SHUTDOWN_MINIMUM_US,
        shut_down: false,
        command_id: 0,
        clock,
        command_timeout_us,
        // Linux's admin timeout is twice its I/O timeout.
        admin_timeout_us: command_timeout_us.saturating_mul(2),
        fault: None,
        owned: Some(Owner {
            lifecycle,
            domain: domain.clone(),
        }),
        teardown: Teardown::Unadmitted,
        ready_timeout_us: 500_000,
        configuration: CONFIGURATION_VALUE & !ENABLE,
        independent_identify: false,
        fatal_standing: false,
        version: 0,
        commands: 0,
        scans: 0,
        retry_delays: [0; 3],
        host_memory: None,
        host_memory_enabled: false,
        events: super::Events::default(),
        aborts: alloc::vec::Vec::new(),
        limits: super::Limits::default(),
        quirks,
    };
    validate_clock(clock)?;
    controller.teardown = Teardown::Unprepared;
    controller.admin.submissions =
        Buffer::allocate(source, domain.clone(), PAGE, Direction::ToDevice)?;
    controller.admin.completions =
        Buffer::allocate(source, domain.clone(), PAGE, Direction::FromDevice)?;
    controller.io.submissions =
        Buffer::allocate(source, domain.clone(), PAGE, Direction::ToDevice)?;
    controller.io.completions =
        Buffer::allocate(source, domain.clone(), PAGE, Direction::FromDevice)?;
    controller.admin_data =
        Buffer::allocate(source, domain.clone(), PAGE, Direction::Bidirectional)?;
    // Each pool page is mapped on its own, so it changes hands between the
    // CPU and the controller apart from the pages other commands hold.
    let pages = (0..POOL_PAGES)
        .map(|_| Buffer::allocate(source, domain.clone(), PAGE, Direction::Bidirectional))
        .collect::<Result<_, _>>()?;
    controller.pool = Pool::new(pages);
    // Polling live CQ memory requires the domain's checked
    // coherent observation; refuse absent support before enabling the engine.
    controller.admin.completions.coherent(12)?;
    controller.io.completions.coherent(12)?;
    controller.initialize()?;
    discover(controller, location, &mut Pages { source, domain })
}

/// Host memory buffer pages from the controller's own page source, mapped
/// into its domain.
struct Pages<'a> {
    source: &'a dyn PageSource,
    domain: Arc<dyn DmaDomain>,
}

impl super::HostMemorySource for Pages<'_> {
    fn allocate(&mut self, bytes: usize, chunk: bool) -> Result<Buffer, BlockError> {
        let direction = if chunk {
            Direction::Bidirectional
        } else {
            Direction::ToDevice
        };
        Buffer::allocate(self.source, self.domain.clone(), bytes, direction)
    }
}

impl Controller {
    /// Stop once for all sibling namespaces. A failed step keeps every
    /// remaining owner, and a retry continues against the same exact epoch.
    fn retire_owned(&mut self) -> Result<(), BlockError> {
        if self.teardown == Teardown::Released || self.owned.is_none() {
            return Ok(());
        }
        self.shut_down = true;
        match self.teardown {
            Teardown::Unadmitted => {
                // Clock refusal authorizes neither configuration nor MMIO.
                // Keep every original owner while the old error-only API
                // cannot return this complete still-contained permission.
                return Err(BlockError::Unsupported("NVMe clock admission quarantined"));
            }
            Teardown::Preparing => {
                // The hierarchy may still be in D3. Neither reset nor any
                // register read is authorized. Drop keeps the complete epoch.
                return Err(BlockError::Unsupported(
                    "NVMe uncertain preparation quarantined",
                ));
            }
            Teardown::Unprepared => {
                // No prepare/engine/register effect was attempted; attachment
                // required an already contained hierarchy with no DMA engine.
                self.teardown = Teardown::Stopped;
            }
            Teardown::Active => {
                self.registers.write32(CONFIGURATION, 0)?;
                self.wait_status(self.ready_timeout_us, |status| status & READY == 0)?;
                self.teardown = Teardown::Stopped;
            }
            _ => {}
        }
        let owner = self.owned.as_mut().expect("owned");
        if self.teardown == Teardown::Stopped {
            owner.lifecycle.disable_dma()?;
            self.teardown = Teardown::Contained;
        }
        if self.teardown == Teardown::Contained {
            // SAFETY: acknowledged stopped engine and BME containment.
            unsafe { owner.domain.drain_posted_writes() }.map_err(map_dma_error)?;
            self.teardown = Teardown::Drained;
        }
        if self.teardown == Teardown::Drained {
            let host_memory = self.host_memory.iter_mut().flat_map(|memory| {
                core::iter::once(&mut memory.descriptors).chain(&mut memory.chunks)
            });
            for buffer in [
                &mut self.admin.submissions,
                &mut self.admin.completions,
                &mut self.io.submissions,
                &mut self.io.completions,
                &mut self.admin_data,
            ]
            .into_iter()
            .chain(&mut self.pool.pages)
            .chain(host_memory)
            {
                // SAFETY: stop/containment/drain remain acknowledged;
                // each successful unmap becomes Empty before the next step.
                // Retry therefore only visits still RAM mappings.
                unsafe { buffer.unmap()? };
            }
            self.teardown = Teardown::RamUnmapped;
        }
        if self.teardown == Teardown::RamUnmapped {
            // SAFETY: all RAM removed, no CPU/device access survives.
            unsafe { self.registers.retire()? };
            self.teardown = Teardown::RegistersRetired;
        }
        if self.teardown == Teardown::RegistersRetired {
            owner.lifecycle.release()?;
            self.teardown = Teardown::Released;
        }
        Ok(())
    }
}
impl Drop for Controller {
    fn drop(&mut self) {
        if self.owned.is_some() && self.retire_owned().is_err() {
            self.registers.quarantine();
            let host_memory = self.host_memory.iter_mut().flat_map(|memory| {
                core::iter::once(&mut memory.descriptors).chain(&mut memory.chunks)
            });
            for buffer in [
                &mut self.admin.submissions,
                &mut self.admin.completions,
                &mut self.io.submissions,
                &mut self.io.completions,
                &mut self.admin_data,
            ]
            .into_iter()
            .chain(&mut self.pool.pages)
            .chain(host_memory)
            {
                buffer.quarantine();
            }
            if let Some(owner) = self.owned.take() {
                mem::forget(owner);
            }
        }
    }
}

impl Namespace {
    /// Disable the entire shared owned controller and retire its real resources.
    /// Siblings refuse commands thereafter. Failure preserves ownership for retry.
    pub fn retire_owned(&mut self) -> Result<(), BlockError> {
        let mut controller = self.controller.borrow_mut();
        if controller.owned.is_none() {
            return Err(BlockError::Unsupported("legacy NVMe resource retirement"));
        }
        controller.retire_owned()
    }
}

#[cfg(test)]
pub(crate) use tests::protocol;

#[cfg(test)]
mod tests;
