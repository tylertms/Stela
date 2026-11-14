// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2015, Intel Corporation.
//! Firmware-assigned, polled NVMe attachment below an owned VMD hierarchy:
//! the shared owned NVMe engine run over a [`vmd::endpoint`](crate::bus::vmd::endpoint)
//! lifecycle.
use super::{Attached, CLASS, owned};
use crate::{
    Clock,
    block::BlockError,
    bus::{
        dma::{mapping::DmaDomain, memory::PageSource},
        mmio::owned::Authority,
        vmd::{
            Error, Host, NativeRegisters, Registers,
            endpoint::{Endpoint, FirmwareBar, RetiredHierarchy},
        },
    },
};
use alloc::{boxed::Box, rc::Rc, string::String, sync::Arc, vec::Vec};

/// One NVMe controller below the host: its firmware BAR0, the location the
/// boot log names it by and the quirks its identity or platform selects.
pub struct Controller {
    pub bar: FirmwareBar,
    pub label: String,
    pub quirks: super::Quirks,
}

/// Each controller's own attachment by its label, in the order given, with
/// the handle that returns the hierarchy once every attached controller has
/// retired; or the refusal of the whole hierarchy, with that same handle.
pub type Attachment<R = NativeRegisters> = Result<
    (
        Vec<(String, Result<Attached, BlockError>)>,
        RetiredHierarchy<R>,
    ),
    (BlockError, RetiredHierarchy<R>),
>;

/// Each hierarchy refusal keeps its own cause in the boot log.
pub fn block(error: Error) -> BlockError {
    use BlockError::{Protocol, Unsupported};
    match error {
        Error::Config(error) => BlockError::Config(error),
        Error::PciCapability(error) => BlockError::PciCapability(error),
        Error::PciPower(error) => BlockError::PciPower(error),
        Error::ReadFailed => Protocol("VMD hierarchy configuration read failed"),
        Error::Readback => Protocol("VMD hierarchy register did not read back as written"),
        Error::Mapping => Protocol("VMD hierarchy register window not mapped"),
        Error::Quarantined => Protocol("VMD hierarchy quarantined after a fault"),
        Error::UnstoppedChild(..) => {
            Protocol("a child behind VMD kept DMA or interrupts on after a stop")
        }
        Error::UnsupportedDevice => {
            Unsupported("VMD endpoint or a function on its path is not one this driver takes")
        }
        Error::InvalidResource => {
            Unsupported("VMD endpoint BAR, bridge window or bus range does not route")
        }
        Error::Address => Unsupported("VMD child address outside the hierarchy's buses"),
        Error::Capability => Unsupported("VMD hierarchy capability list malformed"),
        Error::Exhausted => Unsupported("no PCI domain number left for VMD"),
        Error::Suspended => Unsupported("VMD hierarchy suspended"),
        Error::ActiveChild => Unsupported("VMD hierarchy not in the stopped state this step needs"),
    }
}

/// Attach every firmware NVMe controller in `controllers` through VMD child
/// configuration, original parent resource reservations and the shared
/// domain-owned engine, each with its own BAR and engine, as Linux's VMD
/// driver enumerates every controller below one host. The chosen disk is
/// then found among their namespaces by its identity, like any other disk.
/// The hierarchy is discovered with the first controller whose BAR it takes;
/// a controller it refuses, or whose engine fails, stays stopped with its own
/// cause while the others attach. The whole attach refuses only when no
/// controller's BAR is taken. Both success and error return a retirement
/// handle. It receives original owners only once every attached controller's
/// teardown completed, including teardown on failed NVMe initialization.
/// Uncertain effects keep the original hierarchy/domain.
///
/// # Safety
/// `domain` meets the [`DmaDomain`] contract for its kind for the parent
/// requester and every alias/child requester in this hierarchy: an isolated
/// domain is installed on all of them, while the passthrough domain covers
/// them by leaving their DMA untranslated. Either way it drains translated
/// payload writes and observes coherent completions, and keeps the IOMMU,
/// alias and hierarchy reset/assignment owners it needs. Every controller's
/// requests reach the root under the parent's requester ID, so the one
/// domain is each controller's.
/// All other children and parent engines, IRQs and work are stopped/drained;
/// no other native source registration or child driver exists besides the
/// engines this call attaches. Firmware/foreign
/// access, reset, removal, sleep and resource reassignment remain excluded for
/// this consumed host's complete lifetime. `authority` is the original real
/// native mapping permission for these resources and native CPU roots.
/// Every BAR was claimed from this same host and complete firmware assignment.
/// All calls/final releases obey the providers' process/deferred context rules.
pub unsafe fn attach(
    host: Host<NativeRegisters>,
    controllers: Vec<Controller>,
    domain: Arc<dyn DmaDomain>,
    authority: Authority,
    pages: &dyn PageSource,
    clock: Clock,
    minimum: Rc<dyn crate::bus::pci::power::Timer>,
) -> Attachment {
    let registers = |bar: &FirmwareBar| {
        let mapping = bar.map(&authority).map_err(block)?;
        let registers = owned::MappingRegisters::new(mapping).map_err(|(error, mapping)| {
            // Mapping Drop itself quarantines its original reservation and roots.
            drop(mapping);
            error
        })?;
        Ok(Box::new(registers) as Box<dyn owned::Registers>)
    };
    // SAFETY: the caller grants the original hierarchy, domain and mapping
    // permission; the native path always uses the unchanged owned policy.
    unsafe {
        attach_with(
            host,
            controllers,
            domain,
            pages,
            clock,
            minimum,
            (registers, owned::attach),
        )
    }
}

type AttachPolicy = unsafe fn(
    Box<dyn owned::Registers>,
    Box<dyn owned::Lifecycle>,
    &dyn PageSource,
    Arc<dyn DmaDomain>,
    Clock,
    String,
    super::Quirks,
) -> Result<Attached, BlockError>;

// The native sequence, shared with the composed model test's register peer
// and owned policy. This never exposes a public model attachment route.
// SAFETY: the caller keeps the matching original hierarchy/domain permission,
// and the factory keeps registers for exactly the controller it is given.
unsafe fn attach_with<R: Registers + 'static>(
    host: Host<R>,
    controllers: Vec<Controller>,
    domain: Arc<dyn DmaDomain>,
    pages: &dyn PageSource,
    clock: Clock,
    minimum: Rc<dyn crate::bus::pci::power::Timer>,
    factories: (
        impl FnMut(&FirmwareBar) -> Result<Box<dyn owned::Registers>, BlockError>,
        AttachPolicy,
    ),
) -> Attachment<R> {
    let (mut registers, attach_policy) = factories;
    let timing = minimum.check().map_err(BlockError::PciPower);
    let (first, handle) = Endpoint::new(host, Arc::clone(&domain), minimum);
    // Admit the shared NVMe controller's complete operation budget before
    // discovery performs any CFGBAR/config or mapping effects.
    if let Err(error) = timing.and_then(|()| super::validate_clock(clock)) {
        return Err((error, handle));
    }
    // The first controller whose BAR the hierarchy takes discovers it; each
    // later one is admitted as an endpoint of its own.
    let mut first = Some(first);
    let mut anchor: Option<usize> = None;
    let mut endpoints: Vec<Result<Option<Endpoint<R>>, BlockError>> = Vec::new();
    for controller in &controllers {
        let endpoint = match (anchor, first.as_mut()) {
            (None, Some(endpoint)) => match endpoint.discover(&controller.bar, CLASS) {
                Ok(()) => {
                    anchor = Some(endpoints.len());
                    // Moved into this slot below, once every other is admitted.
                    Ok(None)
                }
                Err(error) => Err(block(error)),
            },
            (Some(_), Some(endpoint)) => endpoint
                .admit(&controller.bar, CLASS)
                .map(Some)
                .map_err(block),
            (_, None) => Err(BlockError::Protocol("VMD hierarchy endpoint missing")),
        };
        endpoints.push(endpoint);
    }
    let Some(anchor) = anchor else {
        let error = endpoints
            .into_iter()
            .find_map(Result::err)
            .unwrap_or(BlockError::Unsupported("no NVMe controller behind VMD"));
        drop(first);
        return Err((error, handle));
    };
    endpoints[anchor] = Ok(first.take());
    let mut attached = Vec::with_capacity(controllers.len());
    for (controller, endpoint) in controllers.into_iter().zip(endpoints) {
        let Controller { bar, label, quirks } = controller;
        let result = endpoint
            .and_then(|endpoint| {
                endpoint.ok_or(BlockError::Protocol("VMD hierarchy endpoint missing"))
            })
            .and_then(|endpoint| {
                // A refused register mapping drops the endpoint before any
                // effect: it stays a stopped function.
                let registers = registers(&bar)?;
                // SAFETY: caller grants the original hierarchy/domain identity;
                // the reservation-backed mapping and consumed lifecycle refer to
                // this same controller. No independent numeric-domain assertion
                // is used as proof.
                unsafe {
                    attach_policy(
                        registers,
                        Box::new(Lifecycle(endpoint)),
                        pages,
                        Arc::clone(&domain),
                        clock,
                        label.clone(),
                        quirks,
                    )
                }
            });
        attached.push((label, result));
    }
    Ok((attached, handle))
}
/// The owned NVMe engine's view of the VMD endpoint lifecycle.
struct Lifecycle<R: Registers>(Endpoint<R>);

// SAFETY: the consumed original host serializes every child config effect.
// Real register readback proves Command changes; the owned engine independently
// stops NVMe, drains this same domain and unmaps before release.
unsafe impl<R: Registers> owned::Lifecycle for Lifecycle<R> {
    fn prepare(&mut self) -> Result<(), BlockError> {
        self.0.prepare().map_err(block)
    }
    fn enable_dma(&mut self) -> Result<(), BlockError> {
        self.0.enable_dma().map_err(block)
    }
    fn disable_dma(&mut self) -> Result<(), BlockError> {
        self.0.disable_dma().map_err(block)
    }
    fn release(&mut self) -> Result<(), BlockError> {
        // SAFETY: the owned engine invokes release only after CC.EN0/RDY0,
        // hierarchy BM-clear, original-domain drain, all RAM unmaps and MMIO
        // retirement. This polled profile has no IRQ registrations.
        unsafe { self.0.release() }.map_err(block)
    }
}

#[cfg(test)]
mod tests;
