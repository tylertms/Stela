//! The mount's native MMIO source: owned uncached mappings in the
//! loader-built aperture, valid on every host processor and in every root
//! that shares the loader's window, which is every root the engine builds.
//!
//! The controls the leaves need are checked once on the boot processor
//! (`stela_boot_core::aperture::qualify`); the other host processors run
//! with the same PAT, which their bootstrap programs from the boot
//! processor's. An access checks only that the current root reaches the
//! aperture and that the only handler running is a device interrupt's
//! ([`crate::native_irq`]); mapping and unmapping refuse in every handler.
//! Installations are
//! serialized by the platform's lock and need no remote flush; once the other
//! processors start, a retirement shoots the translation down on each of them
//! ([`super::shootdown`]).
use super::*;
use super::{
    aliases::{aliases, owned_table},
    capture::LEAVES,
};
use crate::{
    engine::host::{HostOwner, MmioRoots, RootIdentity},
    native_smp::memory_types::Image,
};
mod config_space;
pub(crate) use config_space::{ContainedPci, ContainmentError, PciScope};
mod hpet;
use alloc::{boxed::Box, rc::Rc, string::String, sync::Arc, vec::Vec};
use core::{
    arch::asm,
    cell::UnsafeCell,
    ptr::NonNull,
    sync::atomic::{AtomicBool, Ordering},
};
pub(crate) use hpet::{ExclusiveHpet as NativeExclusiveHpet, HpetMapping as NativeHpetMapping};
use stela_boot_core::aperture::{self, Controls, Flush, Processors};
use stela_drivers::bus::{
    irq::{self, Platform as _, posted_msi::NotificationDomain},
    mmio::owned::{
        AddressSpace, Authority, CacheAttribute, InstalledMapping, MapFailure, MapRequest,
        MappingPlatform, PhysicalSpan, Protection, Resource, ResourceBacking, ResourceId,
        ResourceKind,
    },
    pci::{
        EcamWindow,
        assignment::{
            self, AdmissionClose, Assignment, OpaqueUse, Parent, PhysicalAssignment, Retention,
            UseKind,
        },
    },
};

/// Which host processors may hold aperture translations, for the boot.
pub(crate) static PROCESSORS: Processors = Processors::new();

/// A span this source reserved outside RAM: a captured ECAM bus, a BAR inside
/// a root bridge window, or the separately admitted whole HPET controller page.
struct Backing {
    platform: Arc<Platform>,
    span: PhysicalSpan,
    kind: ResourceKind,
    retention: Retention<Reservation>,
}
// SAFETY: Source validates every immutable span against the captured MCFG,
// root bridge windows or HPET descriptor and final non-RAM memory inventory.
// The HPET constructor additionally requires its real whole-controller handoff;
// this backing only keeps that original capture and checked physical extent.
unsafe impl ResourceBacking for Backing {
    fn physical_span(&self) -> PhysicalSpan {
        self.span
    }
    fn kind(&self) -> ResourceKind {
        self.kind
    }
    fn begin_mapping(&self) -> Result<OpaqueUse, Error> {
        self.platform.check_mapping_control()?;
        self.retention
            .begin_use(UseKind::Mapping)
            .map(|usage| usage.erase())
            .map_err(Error::Assignment)
    }
}

struct Captured {
    _origin: super::capture::Origin,
    wire: &'static Wire,
    _roots: Arc<MmioRoots>,
    hpet_unregistered: AtomicBool,
}

/// The actual native aperture ancestry, kept independently of its registry.
/// A later PCI fabric handoff also has to own and retire device-side routing.
struct ApertureProduct {
    _capture: Arc<Captured>,
    _code: crate::native_image::CodePin,
    identity: Arc<RootIdentity>,
}

// SAFETY: Source constructs this only from its unique post-EBS capture and
// original BSP identity. These retentions contain no mapping, Authority,
// Source or driver backedge. They cannot release the aperture or a device.
unsafe impl PhysicalAssignment for ApertureProduct {
    fn parent(&self) -> Option<&Parent> {
        None
    }

    fn check_control(&self) -> Result<(), assignment::Error> {
        self.identity
            .check_irq_cpu()
            .map_err(|_| assignment::Error::Context)?;
        if self.identity.in_interrupt() {
            return Err(assignment::Error::Context);
        }
        Ok(())
    }
}

struct Reservation {
    platform: Arc<Platform>,
    parent: Parent,
    _span: PhysicalSpan,
}

// SAFETY: These spans come from Source's checked, exclusive device handoffs.
// The boot reservation pins the captured tables, root allocations and decoded
// span. Platform contains aperture allocation metadata and no mapping or
// authority backedge. The parent keeps any upstream assignment. A reservation
// alone conveys no PCI writer or driver authority.
unsafe impl PhysicalAssignment for Reservation {
    fn parent(&self) -> Option<&Parent> {
        Some(&self.parent)
    }

    fn check_control(&self) -> Result<(), assignment::Error> {
        self.platform
            .check_mapping_control()
            .map_err(|_| assignment::Error::Context)
    }
}

/// The aperture's two mapping sizes: 4 KiB pages in its leaf tables and
/// 2 MiB pages in its directory's other entries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Size {
    Page,
    Large,
}

impl Size {
    fn bytes(self) -> u64 {
        match self {
            Size::Page => PAGE,
            Size::Large => PAGE << 9,
        }
    }
    /// The slots of this size, numbered from the aperture's base in units
    /// of this size.
    fn slots(self) -> core::ops::Range<usize> {
        match self {
            Size::Page => 0..LEAVES * ENTRIES,
            Size::Large => LEAVES..ENTRIES,
        }
    }
    /// The size a request maps with: 2 MiB pages when it is 2 MiB aligned at
    /// both ends.
    fn of(physical: PhysicalSpan) -> Self {
        let large = Size::Large.bytes();
        if physical.start().is_multiple_of(large) && physical.bytes().is_multiple_of(large) {
            Size::Large
        } else {
            Size::Page
        }
    }
}

struct Allocation {
    request: MapRequest,
    size: Size,
    first: usize,
    count: usize,
}

struct Platform {
    captured: Arc<Captured>,
    space: AddressSpace,
    flush: Flush,
    /// The TSC rate the shootdown's wait is measured in.
    tsc_hz: u64,
    /// The loader root's top-level entry for the aperture, and its slot.
    loader_entry: u64,
    slot: usize,
    held: AtomicBool,
    mappings: UnsafeCell<Vec<Allocation>>,
}
// SAFETY: the metadata is immutable; `mappings` is reached only under `held`.
unsafe impl Send for Platform {}
// SAFETY: shared references rely on the same argument as `Send` above.
unsafe impl Sync for Platform {}

/// Present, NX, and PWT and PCD for PAT entry 3, uncached.
const RIGHTS: u64 = PRESENT | NX | (1 << 3) | (1 << 4);
/// The processor's accessed and dirty bits.
const ACCESSED_DIRTY: u64 = 0x60;

fn current_root() -> u64 {
    let root: u64;
    // SAFETY: CPL0 observation.
    unsafe { asm!("mov {}, cr3", out(reg) root, options(nomem, nostack, preserves_flags)) };
    root & ADDRESS
}

impl Platform {
    fn wire(&self) -> &'static Wire {
        self.captured.wire
    }
    fn check_mapping_control(&self) -> Result<(), Error> {
        if crate::engine::host::in_interrupt() {
            return Err(Error::Unavailable);
        }
        self.check_context(self.space)
    }
    /// The entry of slot `index` of `size`.
    fn entry(&self, size: Size, index: usize) -> Result<*mut u64, Error> {
        if !size.slots().contains(&index) {
            return Err(Error::InvalidRange);
        }
        let wire = self.wire();
        Ok(match size {
            Size::Page => {
                owned_table(wire, wire.leaves[index / ENTRIES])?.wrapping_add(index % ENTRIES)
            }
            Size::Large => owned_table(wire, wire.directory)?.wrapping_add(index),
        })
    }
    fn address(&self, size: Size, index: usize) -> u64 {
        self.wire().virtual_base + index as u64 * size.bytes()
    }
    fn value(size: Size, physical: u64, protection: Protection) -> u64 {
        physical
            | RIGHTS
            | if protection == Protection::ReadWrite {
                WRITABLE
            } else {
                0
            }
            | if size == Size::Large { HUGE } else { 0 }
    }
    fn lock(&self) -> Guard<'_> {
        while self
            .held
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            // The holder may be retiring a mapping, which waits for this
            // processor's acknowledgement.
            shootdown::service();
            core::hint::spin_loop();
        }
        Guard(self)
    }
    fn invalidate(&self, size: Size, first: usize, count: usize) {
        // SAFETY: TLB maintenance only; CR4 is restored to its value.
        unsafe {
            asm!("mfence", options(nostack, preserves_flags));
            match self.flush {
                Flush::Pages => {
                    for index in first..first + count {
                        let address = self.address(size, index);
                        asm!("invlpg [{}]", in(reg) address, options(nostack, preserves_flags));
                    }
                }
                Flush::AllContexts => {
                    let saved: u64;
                    asm!("pushfq", "pop {}", "cli", out(reg) saved);
                    let cr4: u64;
                    asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack, preserves_flags));
                    asm!("mov cr4, {}", in(reg) cr4 ^ (1 << 7), options(nostack, preserves_flags));
                    asm!("mov cr4, {}", in(reg) cr4, options(nostack, preserves_flags));
                    asm!("push {}", "popfq", in(reg) saved);
                }
            }
            asm!("mfence", options(nostack, preserves_flags));
        }
    }
    fn resource(&self, resource: &Resource) -> bool {
        if let Some(backing) = resource.backing().downcast_ref::<Backing>() {
            return core::ptr::eq(backing.platform.as_ref(), self)
                && backing.span == resource.physical_span();
        }
        stela_drivers::platform::acpi::resources::reservation_parent(resource)
            .is_some_and(|parent| self.resource(parent))
    }
}

struct Guard<'a>(&'a Platform);
impl Guard<'_> {
    fn mappings(&mut self) -> &mut Vec<Allocation> {
        // SAFETY: the held lock; no callback runs while borrowed.
        unsafe { &mut *self.0.mappings.get() }
    }
}
impl Drop for Guard<'_> {
    fn drop(&mut self) {
        self.0.held.store(false, Ordering::Release);
    }
}

// SAFETY: map publishes only the aperture's own leaves, uncached through PAT
// entry 3, which `qualify` checked; every processor runs the same PAT and
// every engine root shares the leaf table. Installation needs no remote
// flush, and retirement refuses once a remote TLB may hold the translation.
unsafe impl MappingPlatform for Platform {
    fn check_context(&self, space: AddressSpace) -> Result<(), Error> {
        if space != self.space
            || (crate::engine::host::in_interrupt() && !crate::native_irq::in_device_dispatch())
        {
            return Err(Error::Unavailable);
        }
        // SAFETY: the current root page is RAM the physmap reaches.
        let entry = unsafe {
            (crate::layout::physical(current_root()) as *const u64)
                .add(self.slot)
                .read_volatile()
        };
        if !aperture::shares_aperture(entry, self.loader_entry) {
            return Err(Error::Unavailable);
        }
        Ok(())
    }
    unsafe fn map(
        &self,
        resource: &Resource,
        request: MapRequest,
    ) -> Result<InstalledMapping, MapFailure> {
        let unchanged = MapFailure::Unchanged;
        if crate::engine::host::in_interrupt() {
            return Err(unchanged(Error::Unavailable));
        }
        self.check_context(request.address_space)
            .map_err(unchanged)?;
        if request.attribute != CacheAttribute::Uncached {
            return Err(unchanged(Error::UnsupportedCache));
        }
        if !self.resource(resource) {
            return Err(unchanged(Error::OutsideResource));
        }
        let size = Size::of(request.physical);
        let count = usize::try_from(request.physical.bytes() / size.bytes())
            .map_err(|_| unchanged(Error::InvalidRange))?;
        let slots = size.slots();
        if count == 0 || count > slots.len() {
            return Err(unchanged(Error::Exhausted));
        }
        let mut guard = self.lock();
        let mappings = guard.mappings();
        mappings
            .try_reserve(1)
            .map_err(|_| unchanged(Error::Exhausted))?;
        let first = (slots.start..=slots.end - count)
            .find(|first| {
                mappings
                    .iter()
                    .filter(|m| m.size == size)
                    .all(|m| first + count <= m.first || m.first + m.count <= *first)
            })
            .ok_or(unchanged(Error::Exhausted))?;
        for index in first..first + count {
            // SAFETY: an owned aperture table, a bounded slot.
            if unsafe { self.entry(size, index).map_err(unchanged)?.read_volatile() } != 0 {
                return Err(unchanged(Error::MappingFailed));
            }
        }
        mappings.push(Allocation {
            request,
            size,
            first,
            count,
        });
        for offset in 0..count {
            let physical = request.physical.start() + offset as u64 * size.bytes();
            // SAFETY: every slot was checked empty above.
            unsafe {
                self.entry(size, first + offset)
                    .expect("checked")
                    .write_volatile(Self::value(size, physical, request.protection))
            };
        }
        self.invalidate(size, first, count);
        Ok(InstalledMapping {
            token: request.id,
            cpu_base: NonNull::new(self.address(size, first) as *mut u8).expect("nonzero aperture"),
            physical: request.physical,
            attribute: request.attribute,
            protection: request.protection,
        })
    }
    unsafe fn unmap(&self, request: MapRequest, mapping: InstalledMapping) -> Result<(), Error> {
        if crate::engine::host::in_interrupt() {
            return Err(Error::Unavailable);
        }
        self.check_context(request.address_space)?;
        let mut guard = self.lock();
        let mappings = guard.mappings();
        let at = mappings
            .iter()
            .position(|m| m.request == request)
            .ok_or(Error::UnmapFailed)?;
        let allocation = &mappings[at];
        let (size, first, count) = (allocation.size, allocation.first, allocation.count);
        if mapping.token != request.id
            || mapping.physical != request.physical
            || mapping.cpu_base.as_ptr() as u64 != self.address(size, first)
        {
            return Err(Error::UnmapFailed);
        }
        for offset in 0..count {
            let physical = request.physical.start() + offset as u64 * size.bytes();
            let expected = Self::value(size, physical, request.protection);
            // SAFETY: `entry` checks the slot against the owned table, and the held guard
            // serializes it.
            if unsafe { self.entry(size, first + offset)?.read_volatile() } & !ACCESSED_DIRTY
                != expected
            {
                return Err(Error::UnmapFailed);
            }
        }
        for offset in 0..count {
            // SAFETY: `entry` checks the slot against the owned table, and these entries belong to
            // the retiring request.
            unsafe { self.entry(size, first + offset)?.write_volatile(0) };
        }
        self.invalidate(size, first, count);
        // A processor that did not acknowledge may still translate through
        // the slots, so they stay allocated and the mapping is refused.
        // Peers flush everything: the slots may be large pages.
        if !super::shootdown::retire(self.tsc_hz, None) {
            return Err(Error::Busy);
        }
        mappings.swap_remove(at);
        Ok(())
    }
}

/// The owned MMIO permission over the captured firmware description.
pub(crate) struct Source {
    /// The original BSP owner, independent of finite RAM absence scopes.
    /// Keep this affine pin on Source; shared MappingPlatform instances do not
    /// gain a foreign-CPU root ownership or destruction permission.
    identity: Arc<RootIdentity>,
    aperture: Assignment<ApertureProduct>,
    platform: Arc<Platform>,
    authority: Authority,
    window: Option<super::Window>,
    reservations: core::cell::RefCell<Vec<Assignment<Reservation>>>,
    hpet_assignment: core::cell::RefCell<Option<Assignment<Reservation>>>,
    pci_slots: config_space::PciSlots,
    pci_clock: core::cell::OnceCell<crate::native_clock::Prepared>,
    /// The firmware framebuffer's PCI function, checked before and after
    /// firmware exit, or unknown, or why it could not be checked.
    framebuffer: core::cell::OnceCell<Result<crate::pci_io::ScreenOwner, String>>,
    ioapics: core::cell::RefCell<MountedIoapics>,
}

impl Source {
    /// Activate the permission on the boot processor, after `host::install`
    /// and before the other host processors start; the TSC runs at `tsc_hz`.
    ///
    /// # Safety
    /// `handoff` is this boot's original transfer; `owner` is the boot
    /// processor's installed host. No firmware, SMM or other driver maps
    /// the aperture.
    pub(crate) unsafe fn activate(
        handoff: &'static crate::handoff::Handoff,
        owner: &mut HostOwner,
    ) -> Result<Self, Error> {
        if !PROCESSORS.alone() {
            return Err(Error::Unavailable);
        }
        // SAFETY: the caller supplies the original boot handoff and physmap.
        let (wire, origin) = unsafe { super::claim_capture(handoff) }?;
        let tsc_hz = handoff.tsc_hz;
        let (catalog, snapshot, identity) = {
            let execution = owner.enter_root().map_err(failed)?;
            let context = execution.context();
            (
                context.roots().clone(),
                context.memory_snapshot().map_err(failed)?,
                context.identity().clone(),
            )
        };
        let flush = aperture::qualify(&Controls {
            cr0: snapshot.cr0,
            cr3: snapshot.cr3,
            cr4: snapshot.cr4,
            efer: snapshot.efer,
            pat: snapshot.pat,
            levels: wire.levels,
        })
        .map_err(|refusal| {
            crate::output::arguments(format_args!(
                "NATIVE MMIO unavailable controls={refusal:?}\n"
            ));
            Error::UnsupportedCache
        })?;
        if !snapshot.pat_valid || !catalog.roots().contains(&wire.root) {
            return Err(Error::Unavailable);
        }
        let image = Image::from_snapshot(&snapshot).map_err(failed)?;
        let space = AddressSpace {
            identity: wire.root,
            physical_bits: image.physical_bits,
            virtual_bits: if wire.levels == 5 { 57 } else { 48 },
        };
        let shift = 12 + 9 * (wire.levels - 1);
        let slot = ((wire.virtual_base >> shift) & 511) as usize;
        // SAFETY: the loader root is an owned table page.
        let loader_entry = unsafe { owned_table(wire, wire.root)?.add(slot).read_volatile() };
        let platform = Arc::new(Platform {
            captured: Arc::new(Captured {
                _origin: origin,
                wire,
                _roots: catalog.clone(),
                hpet_unregistered: AtomicBool::new(false),
            }),
            space,
            flush,
            tsc_hz,
            loader_entry,
            slot,
            held: AtomicBool::new(false),
            mappings: UnsafeCell::new(Vec::new()),
        });
        // SAFETY: the original installed BSP and retained loader root above
        // own the relocated immutable engine image and its final handoff.
        let code = unsafe { handoff.native_image.pin(handoff) }.map_err(failed)?;
        let aperture = Assignment::from_owned(ApertureProduct {
            _capture: platform.captured.clone(),
            _code: code,
            identity: identity.clone(),
        });
        let mut written = Vec::new();
        written.try_reserve_exact(LEAVES + 1).map_err(failed)?;
        written.extend_from_slice(&wire.leaves);
        written.push(wire.directory);
        for &table in &written {
            crate::native_memory::cache::verify_writeback(
                crate::layout::physical(table),
                table,
                PAGE as usize,
                crate::layout::physical(0),
            )
            .map_err(failed)?;
        }
        let aliases = aliases(wire, &catalog, &snapshot, space, &written)?;
        // How many mappings the permission imports, which its activation
        // time and every map's conflict check scale with.
        crate::output::arguments(format_args!("NATIVE MMIO aliases={}\n", aliases.len()));
        platform.check_context(space)?;
        for root in catalog.roots() {
            let mut table = *root;
            for level in (2..wire.levels).rev() {
                let slot = ((wire.virtual_base >> (12 + 9 * level)) & 511) as usize;
                // SAFETY: an owned table page.
                let entry = unsafe { owned_table(wire, table)?.add(slot).read_volatile() };
                if entry & (PRESENT | HUGE) != PRESENT {
                    return Err(Error::Unavailable);
                }
                table = entry & ADDRESS;
            }
            if table != wire.directory {
                return Err(Error::Unavailable);
            }
        }
        let directory = owned_table(wire, wire.directory)?;
        for (index, &leaf) in wire.leaves.iter().enumerate() {
            // SAFETY: the owned directory.
            let entry = unsafe { directory.add(index).read_volatile() };
            if entry & (PRESENT | WRITABLE | HUGE) != (PRESENT | WRITABLE)
                || entry & ADDRESS != leaf
            {
                return Err(Error::Unavailable);
            }
        }
        for size in [Size::Page, Size::Large] {
            for index in size.slots() {
                // SAFETY: the owned aperture tables.
                if unsafe { platform.entry(size, index)?.read_volatile() } != 0 {
                    return Err(Error::Unavailable);
                }
            }
        }
        // SAFETY: the inventory holds every root's and the guest RAM's
        // aliases; the leaf table is owned and shared by every root.
        let authority = unsafe { Authority::activate(platform.clone(), space, aliases) }?;
        // SAFETY: this boot's capture, checked above.
        let window = unsafe { super::Window::new(wire, snapshot.pat, tsc_hz) };
        Ok(Self {
            identity,
            aperture,
            platform,
            authority,
            window,
            reservations: core::cell::RefCell::new(Vec::new()),
            hpet_assignment: core::cell::RefCell::new(None),
            pci_slots: config_space::PciSlots::new(),
            pci_clock: core::cell::OnceCell::new(),
            framebuffer: core::cell::OnceCell::new(),
            ioapics: core::cell::RefCell::new(MountedIoapics::Unclaimed),
        })
    }

    /// The checked owner of the firmware screen, or a sentence saying why it
    /// could not be checked.
    pub(crate) fn verified_framebuffer(&self) -> Result<crate::pci_io::ScreenOwner, String> {
        match self.framebuffer.get() {
            Some(result) => result.clone(),
            None => Err(String::from(
                "the firmware framebuffer's PCI function was never checked",
            )),
        }
    }

    /// Keep why the framebuffer's PCI function could not be checked. The
    /// first outcome stays.
    pub(crate) fn refuse_framebuffer(&self, why: String) {
        let _ = self.framebuffer.set(Err(why));
    }

    /// Keep that no PCI function was found for the firmware screen. The
    /// first outcome stays.
    pub(crate) fn framebuffer_owner_unknown(&self) {
        let _ = self
            .framebuffer
            .set(Ok(crate::pci_io::ScreenOwner::Unknown));
    }

    pub(crate) fn authority(&self) -> &Authority {
        &self.authority
    }

    /// The page window, on a processor whose PAT offers write-combining.
    pub(crate) fn window(&self) -> Option<&super::Window> {
        self.window.as_ref()
    }

    fn reserve(&self, span: PhysicalSpan, kind: ResourceKind) -> Result<Resource, Error> {
        let mut reservations = self
            .reservations
            .try_borrow_mut()
            .map_err(|_| Error::Busy)?;
        reservations.try_reserve(1).map_err(|_| Error::Exhausted)?;
        let (resource, _, assignment) = self.register_reservation(span, kind)?;
        reservations.push(assignment);
        Ok(resource)
    }

    fn reservation(
        &self,
        span: PhysicalSpan,
        kind: ResourceKind,
    ) -> Result<(Resource, Assignment<Reservation>), Error> {
        self.reservation_under(span, kind, self.aperture.parent())
    }

    fn reservation_under(
        &self,
        span: PhysicalSpan,
        kind: ResourceKind,
        parent: Parent,
    ) -> Result<(Resource, Assignment<Reservation>), Error> {
        let reservation = Reservation {
            platform: self.platform.clone(),
            parent,
            _span: span,
        };
        reservation.check_control().map_err(Error::Assignment)?;
        let assignment = Assignment::from_owned(reservation);
        let resource = Resource::new(Box::new(Backing {
            platform: self.platform.clone(),
            span,
            kind,
            retention: assignment.retain(),
        }));
        Ok((resource, assignment))
    }

    fn register_reservation(
        &self,
        span: PhysicalSpan,
        kind: ResourceKind,
    ) -> Result<(Resource, ResourceId, Assignment<Reservation>), Error> {
        let (resource, mut assignment) = self.reservation(span, kind)?;
        match self.authority.register_resource(resource.clone()) {
            Ok(id) => Ok((resource, id, assignment)),
            Err(refusal) => {
                let error = refusal.error;
                drop(refusal.into_resource());
                drop(resource);
                if assignment.close().is_ok() {
                    let _ = assignment.into_original();
                }
                Err(error)
            }
        }
    }

    /// Reserve the memory BAR at bus address `address` for `bytes` on
    /// `segment`, when a root bridge window holds it and it is outside RAM.
    ///
    /// # Safety
    /// The caller owns the function that decodes it.
    pub(crate) unsafe fn claim_bar(
        &self,
        segment: u16,
        address: u64,
        bytes: u64,
        prefetchable: bool,
    ) -> Result<Resource, Error> {
        let wire = self.platform.wire();
        let windows = wire
            .roots()
            .iter()
            .filter(|root| root.space == 0)
            .map(|root| aperture::BusWindow {
                segment: root.segment,
                minimum: root.minimum,
                maximum: root.maximum,
                translation: root.translation,
            });
        let physical = aperture::bar_physical(windows, u32::from(segment), address, bytes)
            .ok_or(Error::OutsideResource)?;
        let span = PhysicalSpan::new(physical, bytes)?;
        if !span.start().is_multiple_of(PAGE) || !span.bytes().is_multiple_of(PAGE) {
            return Err(Error::InvalidRange);
        }
        if !wire.excludes_memory(span.start(), span.end()) {
            return Err(Error::OutsideResource);
        }
        self.reserve(
            span,
            if prefetchable {
                ResourceKind::PrefetchableMemory
            } else {
                ResourceKind::Registers
            },
        )
    }
}

impl Source {
    /// Whether the final memory map leaves `first..end` to the firmware,
    /// overlapping no RAM the host allocates.
    pub(crate) fn firmware_owned(&self, first: u64, end: u64) -> bool {
        self.platform.wire().firmware_owned(first, end)
    }

    /// Reserve the registers at `span`, which an ACPI device's `_CRS` names,
    /// when the final memory map leaves them outside RAM and they avoid the
    /// local APIC. The permission refuses a span that overlaps another
    /// reservation.
    ///
    /// # Safety
    /// The caller owns the device that decodes the span.
    pub(crate) unsafe fn claim_platform(&self, span: PhysicalSpan) -> Result<Resource, Error> {
        if !span.start().is_multiple_of(PAGE) || !span.bytes().is_multiple_of(PAGE) {
            return Err(Error::InvalidRange);
        }
        let apic = crate::apic::base_msr() & crate::apic::BASE_ADDRESS;
        if !self
            .platform
            .wire()
            .excludes_memory(span.start(), span.end())
            || (span.start() < apic + PAGE && apic < span.end())
        {
            return Err(Error::OutsideResource);
        }
        self.reserve(span, ResourceKind::Registers)
    }
}

fn in8(port: u16) -> u8 {
    let value: u8;
    // SAFETY: only the MADT-qualified PIC interrupt-mask ports are read.
    // Reading the mask does not change the controller or acknowledge an IRQ.
    unsafe {
        asm!("in al, dx", out("al") value, in("dx") port, options(nomem, nostack, preserves_flags))
    };
    value
}

/// Log an interrupt-path refusal's own cause at the step that refused,
/// before it becomes the MMIO service's `Unavailable`.
fn refused<E: core::fmt::Debug>(step: &'static str) -> impl FnOnce(E) -> Error {
    move |error| {
        crate::output::arguments(format_args!("HOST {step} refused: {error:?}\n"));
        Error::Unavailable
    }
}

/// A mapping kept by every pin using the controller. Unit serializes
/// selector/window pairs with local interrupt exclusion.
pub(crate) struct IoapicRegisters {
    mapping: core::mem::ManuallyDrop<stela_drivers::bus::mmio::owned::Mapping>,
    mmio: crate::native_irq::ioapic::Mmio,
}
// SAFETY: the all-root UC mapping is held for every access; IOAPIC Unit
// serializes selector/window operations on the owning boot processor.
unsafe impl crate::native_irq::ioapic::Registers for IoapicRegisters {
    fn read(&self, index: u8) -> u32 {
        crate::native_irq::ioapic::Registers::read(&self.mmio, index)
    }
    fn write(&self, index: u8, value: u32) {
        crate::native_irq::ioapic::Registers::write(&self.mmio, index, value);
    }
    fn write_eoi(&self, vector: u8) {
        crate::native_irq::ioapic::Registers::write_eoi(&self.mmio, vector);
    }
}
impl Drop for IoapicRegisters {
    fn drop(&mut self) {
        if crate::native_irq::in_device_dispatch()
            || crate::native_irq::NativePlatform::current().is_err()
        {
            return;
        }
        // SAFETY: the last Unit owner has gone, every pin retired or kept
        // the Unit in quarantine, and this is the owning process CPU.
        let _ = unsafe { self.mapping.retire() };
        // SAFETY: same final process context. Failed retirement is quarantined
        // by Mapping's destructor rather than authorizing reuse.
        unsafe { core::mem::ManuallyDrop::drop(&mut self.mapping) };
    }
}
pub(crate) type IoapicUnit =
    crate::native_irq::ioapic::Unit<IoapicRegisters, crate::native_irq::NativePlatform>;

type IoapicEntry = (crate::native_irq::madt::Unit, Arc<IoapicUnit>);
pub(crate) enum ModePin {
    Intel(crate::native_dma::vtd::interrupt::DirectDelivery<crate::native_dma::vtd::MmioRegisters>),
    Amd(stela_drivers::bus::dma::amd::unit::native::DirectDelivery),
    /// The platform describes no remapping hardware (neither DMAR nor IVRS),
    /// so every interrupt message is delivered directly.
    Unremapped,
}
impl ModePin {
    unsafe fn check(&self) -> Result<(), irq::posted_msi::Error> {
        let alive = match self {
            // SAFETY: both projections use the same checked IRQ CPU/root.
            Self::Intel(pin) => unsafe { pin.check() }.is_ok(),
            // SAFETY: forwarded original CPU/root and kept AMD register view.
            Self::Amd(pin) => unsafe { pin.check() }.is_ok(),
            Self::Unremapped => true,
        };
        if alive {
            Ok(())
        } else {
            Err(irq::posted_msi::Error::RootFailed)
        }
    }
}

enum MountedIoapics {
    Unclaimed,
    Unavailable,
    Containing(Rc<MountedControllers>),
    Ready(Rc<MountedControllers>),
}

struct MountedControllers {
    units: Vec<IoapicEntry>,
    spans: Vec<(crate::native_irq::madt::Unit, u16)>,
    failure: core::cell::Cell<Option<irq::IrqError>>,
    routes: Vec<
        crate::native_irq::ioapic::FirmwareRoutes<
            IoapicRegisters,
            crate::native_irq::NativePlatform,
        >,
    >,
}

/// The original IOAPIC cohort and its retained notification mode, joined
/// with a check of the independent legacy PIC delivery path.
#[derive(Clone)]
pub(crate) struct InitialPciLines {
    source: &'static Source,
    controllers: Rc<MountedControllers>,
    legacy_pic: bool,
}

impl InitialPciLines {
    pub(crate) fn check(&self, source: &Source) -> Result<(), Error> {
        if !core::ptr::eq(self.source, source) {
            return Err(Error::OutsideResource);
        }
        source
            .identity
            .check_irq_cpu()
            .map_err(refused("PCI interrupt paths: CPU and root check"))?;
        for routes in &self.controllers.routes {
            routes
                .verify_contained()
                .map_err(refused("PCI interrupt paths: IOAPIC recheck"))?;
        }
        // The IOAPIC's programming permission does not cover the PIC.
        // Observe its two original interrupt masks without acknowledging or
        // reprogramming either controller.
        if self.legacy_pic {
            let masks = (in8(0x21), in8(0xa1));
            if masks != (u8::MAX, u8::MAX) {
                return Err(refused("PCI interrupt paths: legacy PIC not fully masked")(
                    masks,
                ));
            }
        }
        Ok(())
    }
}

impl MountedControllers {
    fn check_line(
        &self,
        routing: &crate::native_irq::madt::Routing,
        line: irq::LineRoute,
    ) -> Result<&Arc<IoapicUnit>, irq::IrqError> {
        let index =
            crate::native_irq::ioapic::verify_owned_line(routing, &self.spans, &self.routes, line)?;
        Ok(&self.units[index].1)
    }
}

struct MountedNotification {
    identity: Arc<RootIdentity>,
    modes: Vec<ModePin>,
}

// SAFETY: the mounted source supplies its actual retained RootIdentity and
// the original mode pin for every remapper before any controller is published.
// The immutable CPU/root check precedes lock-free reads of kept UC registers.
unsafe impl NotificationDomain for MountedNotification {
    fn check(&self) -> Result<(), irq::posted_msi::Error> {
        self.identity
            .check_irq_cpu()
            .map_err(|_| irq::posted_msi::Error::Context)?;
        for mode in &self.modes {
            // SAFETY: checked original IRQ CPU/root; the actual register maps
            // stay held by the pins across every admitted root.
            unsafe { mode.check() }?;
        }
        Ok(())
    }
}

impl Source {
    /// Publish the complete original IOAPIC cohort once.
    ///
    /// # Safety
    /// This boot has exclusive whole-controller ownership, including SMM
    /// exclusion. Prior destinations and firmware execution are stopped and
    /// their contributors/backing remain kept. `modes` contains the same
    /// source's exact complete pinned remapper inventory.
    pub(crate) unsafe fn install_ioapics(
        &'static self,
        modes: Vec<ModePin>,
    ) -> Result<usize, Error> {
        self.identity
            .check_current()
            .map_err(refused("IOAPIC: source context"))?;
        let platform = crate::native_irq::NativePlatform::current()
            .map_err(refused("IOAPIC: native interrupt platform"))?;
        platform
            .check_process()
            .map_err(refused("IOAPIC: process context"))?;
        {
            let mut state = self.ioapics.borrow_mut();
            match &*state {
                MountedIoapics::Unclaimed => *state = MountedIoapics::Unavailable,
                MountedIoapics::Containing(controllers) => {
                    return Err(controllers.failure.get().map_or(Error::Busy, failed));
                }
                _ => return Err(Error::Busy),
            }
        }
        let routing = crate::native_irq::madt::parse(
            self.platform.wire().madt(),
            self.platform.space.physical_bits,
        )
        .map_err(refused("IOAPIC: MADT interrupt routing"))?;
        // An NMI source pin is contained like any other: masked when
        // firmware left it masked, refused when left delivering NMIs
        // (`maskable_firmware_mode`), and never routed (`Routing::owner`).
        if modes.is_empty() || routing.units.is_empty() {
            return Err(refused("IOAPIC: inventory")(format_args!(
                "{} remapper modes, {} MADT controllers",
                modes.len(),
                routing.units.len()
            )));
        }
        let domain: Arc<dyn NotificationDomain> = Arc::new(MountedNotification {
            identity: self.identity.clone(),
            modes,
        });
        domain
            .check()
            .map_err(refused("IOAPIC: remapper notification mode"))?;
        let mut units = Vec::new();
        let mut spans = Vec::new();
        units
            .try_reserve_exact(routing.units.len())
            .map_err(|_| Error::Exhausted)?;
        spans
            .try_reserve_exact(routing.units.len())
            .map_err(|_| Error::Exhausted)?;
        for &descriptor in &routing.units {
            let span = PhysicalSpan::new(descriptor.base, PAGE)?;
            // SAFETY: explicit exclusive handoff and exact original MADT unit.
            let resource = unsafe { self.claim_platform(span) }?;
            let mapping = self.authority.map(
                resource,
                0,
                PAGE as usize,
                CacheAttribute::Uncached,
                Protection::ReadWrite,
            )?;
            let address = mapping.address(0, 68)?.as_ptr() as usize;
            // SAFETY: the exact full controller's exclusive all-root UC mapping.
            let mmio = unsafe { crate::native_irq::ioapic::Mmio::new(address) }
                .map_err(refused("IOAPIC: register window"))?;
            let registers = IoapicRegisters {
                mapping: core::mem::ManuallyDrop::new(mapping),
                mmio,
            };
            // SAFETY: same original controller and complete direct-mode owner.
            let unit = unsafe {
                IoapicUnit::attach(
                    registers,
                    crate::native_irq::NativePlatform::current()
                        .map_err(refused("IOAPIC: native interrupt platform"))?,
                    domain.clone(),
                    crate::native_irq::CONTROLLER.identity(),
                    descriptor.id,
                    descriptor.gsi_base,
                )
            }
            .map_err(refused("IOAPIC: controller attach"))?;
            spans.push((descriptor, unit.pins()));
            units.push((descriptor, unit));
        }
        routing
            .validate_units(&spans)
            .map_err(refused("IOAPIC: MADT controller spans"))?;
        // SAFETY: exact complete inventory with NMI source pins unrouted, original
        // mode pins, and the caller's exclusive controller handoff.
        let routes = unsafe { crate::native_irq::plan_firmware_lines(&units) }
            .map_err(refused("IOAPIC: firmware route snapshot"))?;
        let count = units.len();
        let controllers = Rc::new(MountedControllers {
            units,
            spans,
            routes,
            failure: core::cell::Cell::new(None),
        });
        *self.ioapics.borrow_mut() = MountedIoapics::Containing(controllers.clone());
        // SAFETY: same exclusive original roster. The Source now retains every
        // controller, plan and mode pin even if the hardware refuses.
        let contained = unsafe { crate::native_irq::contain_firmware_lines(&controllers.routes) };
        if let Err(error) = contained {
            controllers.failure.set(Some(error));
            return Err(refused("IOAPIC: firmware route containment")(error));
        }
        // Containment withholds an SMI pin as Unsupported, a pin whose
        // Remote IRR, Delivery Status or old vector stayed pending as Drain,
        // and a pin that would not read back cleared as Route.
        for (gsi, error) in controllers.routes.iter().flat_map(|plan| plan.withheld()) {
            match error {
                irq::IrqError::Unsupported => crate::output::arguments(format_args!(
                    "HOST IOAPIC GSI {gsi} left to firmware: SMI pin\n"
                )),
                error => crate::output::arguments(format_args!(
                    "HOST IOAPIC GSI {gsi} withheld from devices: {error}\n"
                )),
            }
        }
        *self.ioapics.borrow_mut() = MountedIoapics::Ready(controllers);
        Ok(count)
    }

    pub(crate) fn routing(&self) -> Result<crate::native_irq::madt::Routing, Error> {
        crate::native_irq::madt::parse(
            self.platform.wire().madt(),
            self.platform.space.physical_bits,
        )
        .map_err(failed)
    }
    pub(crate) fn fadt(&self) -> &'static [u8] {
        self.platform.wire().fadt()
    }

    /// Join the original controller cohort with the independent legacy line
    /// mask state. This supplies no ownership of SCI or PCI line contributors.
    pub(crate) fn initial_pci_lines(&'static self) -> Result<InitialPciLines, Error> {
        self.identity.check_current().map_err(failed)?;
        let legacy_pic = self.routing()?.has_legacy_pic();
        let state = self.ioapics.borrow();
        let MountedIoapics::Ready(controllers) = &*state else {
            return Err(Error::Unavailable);
        };
        let lines = InitialPciLines {
            source: self,
            controllers: controllers.clone(),
            legacy_pic,
        };
        drop(state);
        lines.check(self)?;
        Ok(lines)
    }

    /// Reuse the original controller after checking this unclaimed route
    /// against the masked entry its containment left.
    pub(crate) fn original_line(
        &self,
        line: irq::LineRoute,
    ) -> Result<Arc<IoapicUnit>, irq::IrqError> {
        if self.identity.in_interrupt() {
            return Err(irq::IrqError::WrongCpu);
        }
        let routing = self.routing().map_err(|_| irq::IrqError::Route)?;
        let state = self.ioapics.borrow();
        let MountedIoapics::Ready(controllers) = &*state else {
            return Err(irq::IrqError::Unavailable);
        };
        controllers.check_line(&routing, line).cloned()
    }

    /// SCI_INT is the firmware source number. Its MADT GSI override
    /// applies to the IOAPIC and does not move the legacy PIC input.
    pub(crate) fn require_sci_pic_masked(&self, sci_int: u32) -> Result<(), Error> {
        self.identity.check_irq_cpu().map_err(failed)?;
        if self.identity.in_interrupt() {
            return Err(Error::Busy);
        }
        if sci_int < 16 && self.routing()?.has_legacy_pic() {
            require_pic_masked(sci_int as u8)?;
        }
        Ok(())
    }

    /// Verify the IOAPIC routes and any PC/AT PIC masks for these ISA lines.
    /// Whole IOAPIC handoff grants no PIC programming permission.
    pub(crate) fn require_isa_masked(&self, irqs: &[u8]) -> Result<(), Error> {
        if self.identity.in_interrupt() {
            return Err(Error::Busy);
        }
        let routing = self.routing()?;
        let state = self.ioapics.borrow();
        let MountedIoapics::Ready(controllers) = &*state else {
            return Err(Error::Unavailable);
        };
        for &irq in irqs {
            let line = routing.isa(irq).map_err(failed)?;
            controllers.check_line(&routing, line).map_err(failed)?;
        }
        if routing.has_legacy_pic() {
            for &irq in irqs {
                require_pic_masked(irq)?;
            }
        }
        Ok(())
    }
}

fn require_pic_masked(irq: u8) -> Result<(), Error> {
    let port = if irq < 8 { 0x21 } else { 0xa1 };
    let mask = 1 << (irq & 7);
    if in8(port) & mask == 0 {
        return Err(Error::Unavailable);
    }
    Ok(())
}
