//! Common native DMA backing construction for Wi-Fi queue owners.
//!
//! Publication, completion and uncertain retirement stay with the vendor
//! queue. A WB mapping alone does not establish hardware coherence: each
//! installer must check the requester and its NoSnoop/ATS policy.
use crate::bus::dma::{
    mapping::{Direction, DmaDomain, DmaError, DmaMapping, DomainKind},
    memory::{CacheAttribute, CpuMapping, MemoryError, PageSource},
};
use alloc::sync::Arc;

/// The first byte a 32-bit DMA mask (`DMA_BIT_MASK(32)`) cannot reach.
const BELOW_4_GIB: u64 = 1 << 32;

/// Preserve the native allocation, CPU mapping and device mapping order.
/// Each vendor keeps its existing typed error conversions and queue lifetime.
///
/// A passthrough domain maps each page at its physical address, so, as the
/// DMA API bounces for a 32-bit mask, its pages come from below 4 GiB; an
/// IOMMU domain places the IOVA below 4 GiB whatever the page.
pub(super) fn allocate_32bit<E>(
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    bytes: usize,
    direction: Direction,
) -> Result<(CpuMapping, DmaMapping), E>
where
    E: From<MemoryError> + From<DmaError>,
{
    let pages = if domain.kind() == DomainKind::Passthrough {
        source.allocate_below(bytes, 4096, CacheAttribute::WriteBack, BELOW_4_GIB)?
    } else {
        source.allocate(bytes, 4096, CacheAttribute::WriteBack)?
    };
    let cpu = pages.cpu_mapping(CacheAttribute::WriteBack)?;
    let dma = DmaMapping::map(domain, pages, direction, u32::MAX.into())?;
    Ok((cpu, dma))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::{mapping::DomainMapping, memory::PinnedPages, pages::DmaPages};
    use core::{cell::Cell, ops::Range};

    /// A source that records which allocation each request took and gives
    /// no memory, so the domain is never reached.
    #[derive(Default)]
    struct Source {
        ordinary: Cell<usize>,
        below: Cell<Option<u64>>,
    }
    impl PageSource for Source {
        fn allocate(
            &self,
            _: usize,
            _: usize,
            _: CacheAttribute,
        ) -> Result<PinnedPages, MemoryError> {
            self.ordinary.set(self.ordinary.get() + 1);
            Err(MemoryError::OutOfMemory)
        }
        fn allocate_below(
            &self,
            _: usize,
            _: usize,
            _: CacheAttribute,
            physical_end_exclusive: u64,
        ) -> Result<PinnedPages, MemoryError> {
            self.below.set(Some(physical_end_exclusive));
            Err(MemoryError::OutOfMemory)
        }
    }
    struct Domain(DomainKind);
    // SAFETY: never maps: every test allocation fails before the domain.
    unsafe impl DmaDomain for Domain {
        fn kind(&self) -> DomainKind {
            self.0
        }
        unsafe fn map(
            &self,
            _: &DmaPages,
            _: Direction,
            _: u64,
        ) -> Result<DomainMapping, DmaError> {
            unreachable!("no page reaches the domain")
        }
        unsafe fn unmap(&self, _: DomainMapping) -> Result<(), DmaError> {
            unreachable!()
        }
        unsafe fn sync_for_device(
            &self,
            _: DomainMapping,
            _: Range<usize>,
            _: Direction,
        ) -> Result<(), DmaError> {
            unreachable!()
        }
        unsafe fn sync_for_cpu(
            &self,
            _: DomainMapping,
            _: Range<usize>,
            _: Direction,
        ) -> Result<(), DmaError> {
            unreachable!()
        }
    }
    #[derive(Debug, PartialEq)]
    enum Failure {
        Memory(MemoryError),
        Dma,
    }
    impl From<MemoryError> for Failure {
        fn from(error: MemoryError) -> Self {
            Self::Memory(error)
        }
    }
    impl From<DmaError> for Failure {
        fn from(_: DmaError) -> Self {
            Self::Dma
        }
    }

    #[test]
    fn a_passthrough_domain_takes_pages_below_4_gib_as_a_32_bit_mask_needs() {
        let source = Source::default();
        let result = allocate_32bit::<Failure>(
            &source,
            Arc::new(Domain(DomainKind::Passthrough)),
            4096,
            Direction::ToDevice,
        );
        assert_eq!(
            result.err(),
            Some(Failure::Memory(MemoryError::OutOfMemory))
        );
        assert_eq!(source.below.get(), Some(1 << 32));
        assert_eq!(source.ordinary.get(), 0);
        // An IOMMU domain places the IOVA itself: any page will do.
        let source = Source::default();
        let result = allocate_32bit::<Failure>(
            &source,
            Arc::new(Domain(DomainKind::Isolated)),
            4096,
            Direction::ToDevice,
        );
        assert!(result.is_err());
        assert_eq!(source.below.get(), None);
        assert_eq!(source.ordinary.get(), 1);
    }
}
