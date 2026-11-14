//! The DMA domain a requester gets when no IOMMU unit translates for it: the
//! device addresses the host's physical pages directly. It confines no DMA;
//! it maps a request only when the pages are one physically contiguous run
//! inside a region handed to the drivers and below the device's address
//! limit, and gives that run's physical address as its IOVA. The host's
//! caches snoop device DMA to write-back memory, so a sync is the full fence
//! that drains write-combining buffers before the device reads, a word the
//! device writes can be observed in place while it runs, and no remapper
//! holds posted writes to drain.
use alloc::{boxed::Box, collections::BTreeMap, vec::Vec};
use core::{
    any::Any,
    cell::UnsafeCell,
    ops::Range,
    sync::atomic::{AtomicBool, AtomicU64, Ordering, fence},
};
use stela_drivers::bus::dma::{
    mapping::{Direction, DmaDomain, DmaError, DomainKind, DomainMapping},
    memory::CacheAttribute,
    pages::DmaPages,
};

pub struct Passthrough {
    regions: Vec<Range<u64>>,
    next: AtomicU64,
    /// The pages of each live mapping the device may write, by mapping id,
    /// so a coherent observation reads those exact pins.
    written: Locked<BTreeMap<u64, (DomainMapping, DmaPages)>>,
    /// What the domain keeps for its requester, such as its binding.
    _retained: Option<Box<dyn Any + Send + Sync>>,
}

/// A value behind a spin lock no holder keeps across anything but a short
/// map update.
struct Locked<T> {
    held: AtomicBool,
    value: UnsafeCell<T>,
}

// SAFETY: `with` hands out the value to one holder at a time.
unsafe impl<T: Send> Sync for Locked<T> {}

impl<T> Locked<T> {
    fn new(value: T) -> Self {
        Self {
            held: AtomicBool::new(false),
            value: UnsafeCell::new(value),
        }
    }

    fn with<U>(&self, f: impl FnOnce(&mut T) -> U) -> U {
        while self
            .held
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }
        // SAFETY: the acquire above makes this the only holder until the
        // release below; the borrow does not outlive `f`.
        let result = f(unsafe { &mut *self.value.get() });
        self.held.store(false, Ordering::Release);
        result
    }
}

impl Passthrough {
    /// Device DMA to the pages of `regions`.
    pub fn new(regions: Vec<Range<u64>>) -> Self {
        Self {
            regions,
            next: AtomicU64::new(1),
            written: Locked::new(BTreeMap::new()),
            _retained: None,
        }
    }

    /// One requester's domain over `regions`, keeping `retained` for as long
    /// as the domain lives.
    pub fn retaining(regions: Vec<Range<u64>>, retained: impl Any + Send + Sync) -> Self {
        Self {
            _retained: Some(Box::new(retained)),
            ..Self::new(regions)
        }
    }

    /// The one contiguous physical run `physical` (pages of `1 << shift`
    /// bytes) covers, when it lies in a region and below `limit`.
    fn span(&self, physical: &[u64], shift: u8, limit: u64) -> Result<Range<u64>, DmaError> {
        let page = 1u64
            .checked_shl(u32::from(shift))
            .ok_or(DmaError::InvalidRange)?;
        let (&first, rest) = physical.split_first().ok_or(DmaError::InvalidRange)?;
        let mut end = first.checked_add(page).ok_or(DmaError::InvalidRange)?;
        for &address in rest {
            if address != end {
                return Err(DmaError::MappingFailed);
            }
            end = end.checked_add(page).ok_or(DmaError::InvalidRange)?;
        }
        let inside = |region: &Range<u64>| first >= region.start && end <= region.end;
        if !self.regions.iter().any(inside) {
            return Err(DmaError::InvalidRange);
        }
        if end - 1 > limit {
            return Err(DmaError::AddressLimit);
        }
        Ok(first..end)
    }
}

// SAFETY: there is no translation to install or invalidate: the IOVA is the
// pages' physical address, which the caller's pin keeps for the mapping's
// life. Identifiers are unique by a monotonic counter. Syncs are full
// fences, which order CPU stores (including write-combining ones) before the
// device's reads and the device's completed writes before CPU reads on x86.
unsafe impl DmaDomain for Passthrough {
    fn kind(&self) -> DomainKind {
        DomainKind::Passthrough
    }

    unsafe fn map(
        &self,
        pages: &DmaPages,
        direction: Direction,
        address_limit: u64,
    ) -> Result<DomainMapping, DmaError> {
        let span = self.span(pages.physical_pages(), pages.page_shift(), address_limit)?;
        let bytes = usize::try_from(span.end - span.start).map_err(|_| DmaError::InvalidRange)?;
        let mapping = DomainMapping {
            id: self.next.fetch_add(1, Ordering::Relaxed),
            address: span.start,
            bytes,
        };
        // Only one allocation the device writes has a CPU view to observe
        // in place; a scattered page list, such as Linux's, is never kept.
        if direction != Direction::ToDevice && pages.as_single().is_some() {
            self.written.with(|written| {
                written.insert(mapping.id, (mapping, pages.clone()));
            });
        }
        Ok(mapping)
    }

    unsafe fn unmap(&self, mapping: DomainMapping) -> Result<(), DmaError> {
        self.written.with(|written| {
            if written
                .get(&mapping.id)
                .is_some_and(|(live, _)| *live == mapping)
            {
                written.remove(&mapping.id);
            }
        });
        Ok(())
    }

    unsafe fn sync_for_device(
        &self,
        mapping: DomainMapping,
        range: Range<usize>,
        _direction: Direction,
    ) -> Result<(), DmaError> {
        check(mapping, &range)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    unsafe fn sync_for_cpu(
        &self,
        mapping: DomainMapping,
        range: Range<usize>,
        _direction: Direction,
    ) -> Result<(), DmaError> {
        check(mapping, &range)?;
        fence(Ordering::SeqCst);
        Ok(())
    }

    unsafe fn read_coherent_u32(
        &self,
        mapping: DomainMapping,
        offset: usize,
        direction: Direction,
    ) -> Result<u32, DmaError> {
        let end = offset.checked_add(4).ok_or(DmaError::InvalidRange)?;
        if !offset.is_multiple_of(4) || direction == Direction::ToDevice {
            return Err(DmaError::InvalidRange);
        }
        check(mapping, &(offset..end))?;
        // A scattered allocation, which has no contiguous CPU view, was not
        // kept, so it has no in-place observation.
        let pages = self
            .written
            .with(|written| {
                written
                    .get(&mapping.id)
                    .filter(|(live, _)| *live == mapping)
                    .map(|(_, pages)| pages.clone())
            })
            .ok_or(DmaError::CoherenceUnavailable)?;
        // The device writes these pinned pages at their physical address
        // and the host's caches snoop it, but only for write-back memory.
        let pinned = pages.as_single().ok_or(DmaError::CoherenceUnavailable)?;
        let cpu = pinned
            .cpu_mapping(CacheAttribute::WriteBack)
            .map_err(|_| DmaError::CoherenceUnavailable)?;
        let pointer = cpu.address(offset, 4).map_err(|_| DmaError::InvalidRange)?;
        if !(pointer.as_ptr() as usize).is_multiple_of(4) {
            return Err(DmaError::InvalidRange);
        }
        fence(Ordering::SeqCst);
        // SAFETY: `cpu` keeps this live mapping's aligned write-back pages;
        // an aligned word read is one observation of what the device wrote,
        // not a snapshot of a whole record.
        let value = unsafe { pointer.as_ptr().cast::<u32>().read_volatile() };
        fence(Ordering::SeqCst);
        Ok(u32::from_le(value))
    }

    unsafe fn drain_posted_writes(&self) -> Result<(), DmaError> {
        // No remapper stands between the device and memory, so none holds
        // translated writes. The caller stopped the engine and cleared its
        // Bus Master; its writes already issued complete in PCIe order
        // ahead of the completion it read. The fence orders this CPU's
        // later accesses after that observation.
        fence(Ordering::SeqCst);
        Ok(())
    }
}

fn check(mapping: DomainMapping, range: &Range<usize>) -> Result<(), DmaError> {
    if range.start > range.end || range.end > mapping.bytes {
        return Err(DmaError::InvalidRange);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const PAGE: u64 = 4096;

    fn domain() -> Passthrough {
        Passthrough::new(alloc::vec![0x10_0000..0x20_0000, 0x40_0000..0x41_0000])
    }

    #[test]
    fn the_domain_names_itself_passthrough() {
        assert_eq!(domain().kind(), DomainKind::Passthrough);
        assert_eq!(
            Passthrough::retaining(alloc::vec![0..PAGE], 7u32).kind(),
            DomainKind::Passthrough
        );
    }

    #[test]
    fn a_contiguous_run_in_the_region_maps_at_its_physical_address() {
        let pages = [0x10_0000, 0x10_1000, 0x10_2000];
        assert_eq!(
            domain().span(&pages, 12, u64::MAX),
            Ok(0x10_0000..0x10_3000)
        );
    }

    #[test]
    fn every_region_admits_its_pages() {
        assert_eq!(
            domain().span(&[0x40_0000], 12, u64::MAX),
            Ok(0x40_0000..0x40_1000)
        );
    }

    #[test]
    fn scattered_pages_outside_pages_and_high_runs_refuse() {
        let domain = domain();
        assert_eq!(
            domain.span(&[0x10_0000, 0x10_2000], 12, u64::MAX),
            Err(DmaError::MappingFailed)
        );
        assert_eq!(
            domain.span(&[0x1f_f000, 0x20_0000], 12, u64::MAX),
            Err(DmaError::InvalidRange)
        );
        assert_eq!(
            domain.span(&[0xf_f000], 12, u64::MAX),
            Err(DmaError::InvalidRange)
        );
        assert_eq!(
            domain.span(&[0x1f_f000], 12, 0x1f_ffff - PAGE),
            Err(DmaError::AddressLimit)
        );
        assert_eq!(domain.span(&[], 12, u64::MAX), Err(DmaError::InvalidRange));
    }

    #[test]
    fn syncs_stay_inside_the_mapping() {
        let mapping = DomainMapping {
            id: 1,
            address: 0x10_0000,
            bytes: 0x2000,
        };
        assert_eq!(check(mapping, &(0..0x2000)), Ok(()));
        assert_eq!(
            check(mapping, &(0x1000..0x2001)),
            Err(DmaError::InvalidRange)
        );
    }

    /// Model RAM standing for pages at a named physical address.
    struct Backing {
        pointer: core::ptr::NonNull<u8>,
        layout: core::alloc::Layout,
        pages: Vec<u64>,
        released: alloc::sync::Arc<AtomicU64>,
    }
    // SAFETY: test-owned allocation; shared access exposes metadata and raw
    // pointers only.
    unsafe impl Send for Backing {}
    // SAFETY: as above.
    unsafe impl Sync for Backing {}
    // SAFETY: the CPU RAM exists and is held until drop; the physical
    // addresses are modeled.
    unsafe impl stela_drivers::bus::dma::memory::PageBacking for Backing {
        fn cpu_base(&self) -> core::ptr::NonNull<u8> {
            self.pointer
        }
        fn physical_pages(&self) -> &[u64] {
            &self.pages
        }
        fn page_shift(&self) -> u8 {
            12
        }
        fn cache_attribute(&self) -> CacheAttribute {
            CacheAttribute::WriteBack
        }
    }
    impl Drop for Backing {
        fn drop(&mut self) {
            self.released.fetch_add(1, Ordering::Relaxed);
            // SAFETY: the exact allocation and layout made below.
            unsafe { alloc::alloc::dealloc(self.pointer.as_ptr(), self.layout) };
        }
    }
    fn pinned(
        physical: u64,
        released: &alloc::sync::Arc<AtomicU64>,
    ) -> stela_drivers::bus::dma::memory::PinnedPages {
        let layout =
            core::alloc::Layout::from_size_align(2 * PAGE as usize, PAGE as usize).unwrap();
        // SAFETY: a fresh zeroed allocation, released by Backing.
        let pointer =
            core::ptr::NonNull::new(unsafe { alloc::alloc::alloc_zeroed(layout) }).unwrap();
        let backing = Backing {
            pointer,
            layout,
            pages: alloc::vec![physical, physical + PAGE],
            released: released.clone(),
        };
        stela_drivers::bus::dma::memory::PinnedPages::new(
            Box::new(backing),
            2 * PAGE as usize,
            PAGE as usize,
        )
        .unwrap()
    }

    /// An NVMe completion queue polled while the controller writes it: the
    /// word is read in place from the mapping's own pages, until unmap.
    #[test]
    fn a_word_the_device_writes_is_observed_from_the_mappings_pins() {
        use stela_drivers::bus::dma::mapping::DmaMapping;
        let domain: alloc::sync::Arc<dyn DmaDomain> = alloc::sync::Arc::new(domain());
        let released = alloc::sync::Arc::new(AtomicU64::new(0));
        let pages = pinned(0x10_0000, &released);
        let cpu = pages.cpu_mapping(CacheAttribute::WriteBack).unwrap();
        let word = cpu
            .address(PAGE as usize + 12, 4)
            .unwrap()
            .as_ptr()
            .cast::<u32>();
        let mut queue =
            DmaMapping::map(domain.clone(), pages, Direction::FromDevice, u64::MAX).unwrap();
        let mut other = DmaMapping::map(
            domain.clone(),
            pinned(0x10_4000, &released),
            Direction::Bidirectional,
            u64::MAX,
        )
        .unwrap();
        for value in [0u32, 1, 0x0001_0007, u32::MAX] {
            // SAFETY: the modeled device writes its own queue page.
            unsafe { word.write_volatile(value.to_le()) };
            assert_eq!(queue.read_coherent_u32(PAGE as usize + 12), Ok(value));
            assert_eq!(other.read_coherent_u32(PAGE as usize + 12), Ok(0));
        }
        for offset in [1, 2 * PAGE as usize - 3, 2 * PAGE as usize, usize::MAX] {
            assert_eq!(queue.read_coherent_u32(offset), Err(DmaError::InvalidRange));
        }
        // A buffer only the device reads is never observed.
        let mut submissions = DmaMapping::map(
            domain.clone(),
            pinned(0x10_8000, &released),
            Direction::ToDevice,
            u64::MAX,
        )
        .unwrap();
        assert_eq!(
            submissions.read_coherent_u32(0),
            Err(DmaError::InvalidRange)
        );
        // No remapper holds posted writes.
        // SAFETY: the modeled device is stopped.
        assert_eq!(unsafe { domain.drain_posted_writes() }, Ok(()));
        // Unmapping lets the pins go: the domain keeps none of them.
        for mapping in [&mut queue, &mut other, &mut submissions] {
            // SAFETY: the modeled device is done with every mapping.
            unsafe { mapping.unmap() }.unwrap();
        }
        drop((queue, other, submissions, cpu));
        assert_eq!(released.load(Ordering::Relaxed), 3);
    }

    /// A scattered page list, as Linux's DMA mappings are, has no CPU view
    /// and is not kept.
    #[test]
    fn a_scattered_mapping_has_no_in_place_observation() {
        use stela_drivers::bus::dma::mapping::DmaMapping;
        let domain: alloc::sync::Arc<dyn DmaDomain> = alloc::sync::Arc::new(domain());
        // SAFETY: the modeled pages are never touched through this list.
        let pages = unsafe {
            DmaPages::from_physical(
                alloc::vec![0x10_0000, 0x10_1000],
                12,
                CacheAttribute::WriteBack,
            )
        }
        .unwrap();
        let mut mapping =
            DmaMapping::map(domain.clone(), pages, Direction::FromDevice, u64::MAX).unwrap();
        assert_eq!(
            mapping.read_coherent_u32(0),
            Err(DmaError::CoherenceUnavailable)
        );
        // SAFETY: the modeled device is done with the mapping.
        unsafe { mapping.unmap() }.unwrap();
    }
}
