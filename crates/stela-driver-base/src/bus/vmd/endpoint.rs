// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2015, Intel Corporation.
//! One firmware-assigned endpoint below an owned VMD hierarchy: its BAR claim
//! and the hierarchy lifecycle an endpoint driver runs it through.
//!
//! This consumes the host, not an MCFG alias for a synthetic segment.
//! The platform supplies the domain every child requester's DMA goes through,
//! as VMD forwards it under the parent's requester ID: an IOMMU domain or the
//! named passthrough domain. There is no hotplug or BAR reassignment.
use super::{Address, Child, ChildConfig, Error, Host, NativeRegisters, Registers, Window};
use crate::bus::{
    dma::mapping::DmaDomain,
    mmio::owned::{Authority, CacheAttribute, Mapping, Protection, Resource},
};
use alloc::{rc::Rc, sync::Arc, vec::Vec};
use core::cell::RefCell;

mod capabilities;
mod hierarchy;
use hierarchy::Hierarchy;

struct Returned<R>(Option<R>);
impl<R> Drop for Returned<R> {
    fn drop(&mut self) {
        if let Some(owner) = self.0.take() {
            core::mem::forget(owner);
        }
    }
}
/// Receives the original stopped parent Function, resource reservations
/// and configuration mappings, only after the endpoint driver retired every
/// RAM mapping and its register mapping and then successfully stopped this
/// Host. This handoff does not detach the IOMMU domain or unmap parent
/// configuration.
pub struct RetiredHierarchy<R = NativeRegisters>(Rc<RefCell<Returned<R>>>);

impl<R> RetiredHierarchy<R> {
    /// The endpoint teardown and Host stop published original owners.
    /// This does not certify domain detach or final parent mapping retirement.
    pub fn is_ready(&self) -> bool {
        self.0.borrow().0.is_some()
    }

    /// None means retirement has not completed, or an uncertain effect kept
    /// the hierarchy. Success returns original owners, never an active Host.
    /// The platform must finish its real domain/alias retirement and final
    /// mapping unmaps under those original owners. Retired endpoint driver
    /// handles can still hold the domain Arc; drop them before attempting a
    /// final config unmap whose domain binding pins that mapping.
    ///
    /// # Safety
    /// The old requester/alias domain attachment has retired under its
    /// real platform owner, and all stale endpoint driver handles have been
    /// dropped. A returned owner or unmapped RAM alone authorizes no reset,
    /// reassignment or new DMA attachment. Final mapping unmaps still require
    /// the original platform's CPU/root/context acknowledgements.
    pub unsafe fn take_after_retirement(&mut self) -> Option<R> {
        self.0.borrow_mut().0.take()
    }
}

/// The firmware's complete sized BAR0 assignment, pinned to this original host.
/// A number obtained by probing a running BAR is not an ownership grant.
pub struct FirmwareBar {
    domain: u32,
    child: Address,
    low: u32,
    high: u32,
    bus_start: u64,
    bytes: u64,
    window: Window,
    resource: Resource,
}
impl FirmwareBar {
    /// # Safety
    /// `bytes` is the complete firmware-sized BAR0 reservation for
    /// `child` at this host's assignment/reset epoch. It is not an
    /// estimate, a new allocation or a size probe with decode enabled. The
    /// original VMD resource owner reserves the whole routed interval, including
    /// rounded mapping pages, exclusively from RAM and other child resources.
    /// The original takeover grant excludes reset/removal/reassignment and all
    /// firmware/foreign child access until the eventual owned attachment retires.
    pub unsafe fn claim(
        host: &mut Host<NativeRegisters>,
        child: Address,
        bytes: u64,
    ) -> Result<Self, Error> {
        host.check_live()?;
        hierarchy::parent_d0(host)?;
        if child.domain != host.domain
            || bytes < 0x2000
            || !bytes.is_power_of_two()
            || !bytes.is_multiple_of(4096)
        {
            return Err(Error::InvalidResource);
        }
        let (low, high) = {
            let mut config = host.config(child.bus, child.device, child.function)?;
            let low = config.read(0x10, 4)?;
            let high = if low & 6 == 4 {
                config.read(0x14, 4)?
            } else {
                0
            };
            (low, high)
        };
        let (bus_start, prefetchable) = bar_address(low, high, bytes)?;
        let (index, window) = host
            .windows
            .iter()
            .copied()
            .enumerate()
            .find(|(_, window)| {
                window.prefetchable == prefetchable && window.translate(bus_start, bytes).is_ok()
            })
            .ok_or(Error::InvalidResource)?;
        // This is the original parent reservation, not a new overlapping
        // child ResourceBacking manufactured from physical address metadata.
        let resource = host.io().resource(index + 1);
        let physical = window.translate(bus_start, bytes)?;
        let span = resource.physical_span();
        if !physical.is_multiple_of(4096)
            || physical < span.start()
            || physical
                .checked_add(bytes)
                .is_none_or(|end| end > span.end())
        {
            return Err(Error::InvalidResource);
        }
        Ok(Self {
            domain: host.domain,
            child,
            low,
            high,
            bus_start,
            bytes,
            window,
            resource,
        })
    }

    /// Map the whole BAR uncached from the original parent reservation.
    pub fn map(&self, authority: &Authority) -> Result<Mapping, Error> {
        let physical = self.window.translate(self.bus_start, self.bytes)?;
        let offset = usize::try_from(physical - self.resource.physical_span().start())
            .map_err(|_| Error::Address)?;
        let bytes = usize::try_from(self.bytes).map_err(|_| Error::Address)?;
        authority
            .map(
                self.resource.clone(),
                offset,
                bytes,
                CacheAttribute::Uncached,
                Protection::ReadWrite,
            )
            .map_err(|_| Error::Mapping)
    }
}

fn bar_address(low: u32, high: u32, bytes: u64) -> Result<(u64, bool), Error> {
    if bytes < 0x2000
        || !bytes.is_power_of_two()
        || low == u32::MAX
        || low & 1 != 0
        || !matches!(low & 6, 0 | 4)
    {
        return Err(Error::InvalidResource);
    }
    let base = (u64::from(high) << 32) | u64::from(low & !15);
    if base == 0
        || !base.is_multiple_of(bytes)
        || base.checked_add(bytes).is_none()
        || (low & 6 == 0 && base + bytes > 1u64 << 32)
    {
        return Err(Error::InvalidResource);
    }
    Ok((base, low & 8 != 0))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    Claimed,
    Prepared,
    Enabled,
    Disabled,
    Released,
    Quarantined,
}

/// The consumed host and the selected endpoint's hierarchy, run through the
/// endpoint driver's prepare, DMA enable/disable and release. Any failed
/// effect quarantines it: the host and domain are then kept, never dropped.
pub struct Endpoint<R: Registers = NativeRegisters> {
    host: Option<Host<R>>,
    returned: Rc<RefCell<Returned<R>>>,
    hierarchy: Option<Hierarchy>,
    domain: Option<Arc<dyn DmaDomain>>,
    minimum: Option<Rc<dyn crate::bus::pci::power::Timer>>,
    phase: Phase,
}
impl<R: Registers> Endpoint<R> {
    /// Consume `host`, with the handle that later receives its original
    /// owners. Nothing is discovered or changed yet. `minimum` retains the
    /// actual platform clock/CPU/root epoch; a nominal frequency is insufficient.
    pub fn new(
        host: Host<R>,
        domain: Arc<dyn DmaDomain>,
        minimum: Rc<dyn crate::bus::pci::power::Timer>,
    ) -> (Self, RetiredHierarchy<R>) {
        let returned = Rc::new(RefCell::new(Returned(None)));
        let handle = RetiredHierarchy(Rc::clone(&returned));
        let endpoint = Self {
            host: Some(host),
            returned,
            hierarchy: None,
            domain: Some(domain),
            minimum: Some(minimum),
            phase: Phase::Claimed,
        };
        (endpoint, handle)
    }

    /// Capture the hierarchy above `bar`'s function, which must be of
    /// `class`, refusing any foreign or active sibling. Every other function
    /// below this host, of any class, stays stopped.
    pub fn discover(&mut self, bar: &FirmwareBar, class: [u8; 3]) -> Result<(), Error> {
        if self.phase != Phase::Claimed || self.hierarchy.is_some() {
            return Err(Error::ActiveChild);
        }
        let host = self.host.as_mut().ok_or(Error::Quarantined)?;
        self.hierarchy = Some(Hierarchy::discover(host, bar, class)?);
        Ok(())
    }

    fn transition(
        &mut self,
        effect: impl FnOnce(
            &mut Host<R>,
            &mut Hierarchy,
            &dyn crate::bus::pci::power::Timer,
        ) -> Result<(), Error>,
    ) -> Result<(), Error> {
        if self.phase == Phase::Quarantined || self.phase == Phase::Released {
            return Err(Error::Quarantined);
        }
        let result = effect(
            self.host.as_mut().ok_or(Error::Quarantined)?,
            self.hierarchy.as_mut().ok_or(Error::InvalidResource)?,
            self.minimum.as_deref().ok_or(Error::Quarantined)?,
        );
        if result.is_err() {
            self.phase = Phase::Quarantined;
        }
        result
    }

    /// Bring the path to D0 with memory decode and quiesce the siblings.
    pub fn prepare(&mut self) -> Result<(), Error> {
        if self.phase != Phase::Claimed {
            return Err(Error::ActiveChild);
        }
        self.transition(|host, hierarchy, minimum| hierarchy.prepare(host, minimum))?;
        self.phase = Phase::Prepared;
        Ok(())
    }

    /// Enable bus mastering along the path, also after a disable: a
    /// controller reset re-enables it, as `nvme_pci_enable` does after
    /// `nvme_dev_disable`.
    pub fn enable_dma(&mut self) -> Result<(), Error> {
        if !matches!(self.phase, Phase::Prepared | Phase::Disabled) {
            return Err(Error::ActiveChild);
        }
        self.transition(|host, hierarchy, _| hierarchy.enable(host))?;
        self.phase = Phase::Enabled;
        Ok(())
    }

    /// Clear bus mastering along the path and read it back.
    pub fn disable_dma(&mut self) -> Result<(), Error> {
        if !matches!(
            self.phase,
            Phase::Claimed | Phase::Prepared | Phase::Enabled | Phase::Disabled
        ) {
            return Err(Error::Quarantined);
        }
        self.transition(|host, hierarchy, _| hierarchy.disable(host))?;
        self.phase = Phase::Disabled;
        Ok(())
    }

    /// Recheck the stopped hierarchy, stop the host and publish its original
    /// owners to the [`RetiredHierarchy`].
    ///
    /// # Safety
    /// The endpoint driver stopped the device, drained this same domain and
    /// retired every RAM mapping and its register mapping; it has no IRQ
    /// registrations.
    pub unsafe fn release(&mut self) -> Result<(), Error> {
        if self.phase != Phase::Disabled {
            return Err(Error::ActiveChild);
        }
        self.transition(|host, hierarchy, _| hierarchy.recheck(host, false))?;
        let host = self.host.take().ok_or(Error::Quarantined)?;
        self.phase = Phase::Quarantined;
        // SAFETY: the caller stopped and drained the endpoint and retired its
        // mappings; this polled profile has no IRQ registrations.
        let registers = unsafe { host.retire() }?;
        self.returned.borrow_mut().0 = Some(registers);
        self.phase = Phase::Released;
        self.domain = None;
        Ok(())
    }
}
impl<R: Registers> Drop for Endpoint<R> {
    fn drop(&mut self) {
        if self.phase != Phase::Released {
            if let Some(host) = self.host.take() {
                core::mem::forget(host);
            }
            if let Some(domain) = self.domain.take() {
                core::mem::forget(domain);
            }
            if let Some(minimum) = self.minimum.take() {
                core::mem::forget(minimum);
            }
        }
    }
}

#[cfg(any(test, feature = "test-support"))]
pub mod test_support;
#[cfg(test)]
mod tests;
