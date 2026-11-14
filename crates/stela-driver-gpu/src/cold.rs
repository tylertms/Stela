//! GPU reset while the original configuration and DMA root stay owned: a
//! function-local reset, else a Secondary Bus Reset of the GPU's own bus.
use alloc::{boxed::Box, rc::Rc};
use core::mem::ManuallyDrop;
use stela_driver_base::{
    block::BlockError,
    bus::pci::{
        CapabilityError, ConfigSpace, Function, ResetMethod, capability,
        power::{self, Timer},
        requester::{
            self, ClaimError,
            preparation::{self, BoundRequester, PreparingRequester},
        },
    },
};

const NVIDIA: u16 = 0x10de;
const DISPLAY: u8 = 3;
const EXPRESS: u8 = 0x10;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ViewError {
    Context,
    Busy,
    Root,
    Cache,
    Retired,
    RegisterReadback,
    ForeignSource,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Error {
    Clock(power::Error),
    Identity,
    Claim(ClaimError),
    Capability(CapabilityError),
    /// No function-local reset, no bus reset, and DMA is not translated.
    ResetUnavailable,
    Views(ViewError),
    Reset(BlockError),
    Preparation(preparation::Error),
}

/// The reset a cold start performed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Reset {
    /// FLR or a D3hot soft reset ([`Function::reset`]).
    Function(ResetMethod),
    /// Secondary Bus Reset on the bridge above the GPU's own bus, as Linux
    /// `pci_reset_function` falls back to `pci_parent_bus_reset`.
    SecondaryBus,
}

/// The original CPU framebuffer/register writers and their translation owners.
///
/// # Safety
/// This is a real retained root/view owner for the same native bootstrap. It
/// confirms the supplied function against its retained original configuration
/// owner before acknowledging any seal; a BDF/label comparison is insufficient.
/// It excludes firmware, other CPUs and source/work users that could restore an
/// old GPU view. `seal` joins executing CPU writers and makes old framebuffer
/// status/fatal/reclaim/fallback paths unavailable until a new owned framebuffer
/// epoch is installed. It drains their CPU WC stores and completes a safe
/// uncached read from this exact GPU after the fence, so CPU posted writes to
/// this function precede its reset. A CPU fence alone is not that PCI ordering.
/// All original mappings, backing, code and root owners remain held through
/// reset/bind/failure; Drop never reopens old views or reclaims uncertain data.
/// This supplies no DMA drain, no private domain and no new framebuffer grant.
/// Timing and operations are admitted only on the original process/CPU context.
pub unsafe trait Views<C: ConfigSpace>: 'static {
    fn seal(&mut self, function: &mut Function<C>) -> Result<(), ViewError>;
}

/// An original requester/root, or an explicitly unsafe diagnostic peer.
///
/// # Safety
/// The function is the original exclusive physical configuration/reset epoch,
/// already gated for BME/MSI/INTx, with old hardware backing kept. The platform
/// excludes competing firmware/configuration/reset and software producers.
/// `finalize` consumes the same current-epoch stop receipt, drains the ORIGINAL
/// remapper payload path and only then attaches a private domain once. Failure,
/// panic and abandoned Drop retain all uncertain roots/configuration/backing.
/// No new allocation, IRQ route or DMA submission occurs before finalization.
pub unsafe trait Preparation: 'static {
    type Config: ConfigSpace;
    type Bound: 'static;
    type Failure;
    fn function_mut(&mut self) -> &mut Function<Self::Config>;
    /// Whether a Secondary Bus Reset of the function's own bus is offered
    /// and would currently find a bus it may reset. Reads only.
    fn bus_reset_available(&self) -> bool;
    /// Reset the function's own bus, the function's state saved and
    /// restored around it.
    fn reset_bus(&mut self, timer: &dyn Timer) -> Result<(), BlockError>;
    /// # Safety
    /// A function-local reset (FLR or D3hot soft reset) or a Secondary Bus
    /// Reset, and readiness/containment revalidation, completed with CPU
    /// views sealed and all old owners still kept, or the function has no
    /// reset and the private domain translates its every request, so
    /// engines firmware left running reach only that domain. Nothing can
    /// rearm the function before the original root drain and private bind.
    unsafe fn finalize(self) -> Result<Self::Bound, (preparation::Error, Self::Failure)>;
}

// SAFETY: PreparingRequester keeps the original config seal and original Root;
// its consumed receipt validates and drains that same path before binding once.
unsafe impl Preparation for PreparingRequester {
    type Config = stela_driver_base::bus::pci::owned::OwnedConfig;
    type Bound = BoundRequester;
    type Failure = preparation::Failure;
    fn function_mut(&mut self) -> &mut Function<Self::Config> {
        self.function_mut()
    }
    fn bus_reset_available(&self) -> bool {
        self.bus_reset_available()
    }
    fn reset_bus(&mut self, timer: &dyn Timer) -> Result<(), BlockError> {
        self.reset_bus(timer)
    }
    unsafe fn finalize(self) -> Result<Self::Bound, (preparation::Error, Self::Failure)> {
        // SAFETY: Cold reset the function or its bus, or found it has no
        // reset and its DMA translated, and sealed the original CPU views.
        unsafe { self.acknowledge_quiescence() }
            .finalize()
            .map_err(|failure| (failure.error, failure))
    }
}

pub struct Inputs<P: Preparation, V: Views<P::Config>> {
    pub requester: P,
    pub views: V,
    pub timer: Rc<dyn Timer>,
}

struct Pending<P, V> {
    requester: Option<P>,
    views: V,
    timer: Rc<dyn Timer>,
    translated: bool,
}

/// A single consumed cold attempt. Abandonment retains the actual inputs.
pub struct Cold<P: Preparation, V: Views<P::Config>> {
    kept: ManuallyDrop<Box<Pending<P, V>>>,
}

#[must_use]
pub struct Failure<P: Preparation, V: Views<P::Config>> {
    pub error: Error,
    _kept: Cold<P, V>,
    _binding: Option<ManuallyDrop<P::Failure>>,
}

/// Private-domain attachment and the still-retained original view/root owners.
/// Unfinished Drop retains both, even after a successful binding.
pub struct Ready<B, V> {
    bound: ManuallyDrop<B>,
    views: ManuallyDrop<V>,
    reset: Option<Reset>,
}
impl<B, V> Ready<B, V> {
    /// The reset the function had, or None when it started in the state
    /// firmware left, as Linux starts it.
    pub fn reset(&self) -> Option<Reset> {
        self.reset
    }
    /// # Safety
    /// Transfer both owners into the same native function lifetime before any
    /// probe/DMA/IRQ effect. Keep the original view/root guard through endpoint
    /// retirement. This is not authorization to reopen firmware framebuffer
    /// views or release a bound requester without its independent DMA/IRQ proof.
    pub unsafe fn into_parts(self) -> (B, V) {
        (
            ManuallyDrop::into_inner(self.bound),
            ManuallyDrop::into_inner(self.views),
        )
    }
}

fn preflight<C: ConfigSpace>(
    function: &mut Function<C>,
    timer: &dyn Timer,
    translated: bool,
    bus_reset: bool,
) -> Result<Option<Reset>, Error> {
    timer.check().map_err(Error::Clock)?;
    if function.vendor != NVIDIA || function.class[0] != DISPLAY {
        return Err(Error::Identity);
    }
    requester::validate_stopped(function).map_err(Error::Claim)?;
    let mut express = None;
    for entry in capability::capabilities(function.config()) {
        let entry = entry.map_err(Error::Capability)?;
        if entry.id == EXPRESS && express.replace(entry.offset).is_some() {
            return Err(Error::Identity);
        }
    }
    // Linux `pci_reset_function` falls back from FLR to a D3hot soft reset,
    // then to a reset of the bus the function sits on alone. nouveau itself
    // never resets: it starts what firmware left. A GPU an earlier run left
    // with its secure region (WPR2) standing fails nouveau's FWSEC-FRTS
    // until it is reset, and translation does not clear that, so every
    // reset that is available comes first. Only a GPU with none starts as
    // firmware left it, and only with translation: otherwise an engine
    // firmware left running could write recycled RAM once bus mastering is
    // on.
    match function.reset_method().map_err(Error::Reset)? {
        Some(method) => Ok(Some(Reset::Function(method))),
        None if bus_reset => Ok(Some(Reset::SecondaryBus)),
        None if translated => Ok(None),
        None => Err(Error::ResetUnavailable),
    }
}

/// Read-only early qualification before native BAR probing/mapping. Success
/// grants no stop, root drain, view seal or private-domain authority; Cold
/// repeats the same checks immediately before its consumed reset attempt.
/// `translated`: whether an active remapper confines the function's DMA.
/// `bus_reset`: whether a Secondary Bus Reset of its bus is available.
pub fn check_candidate<C: ConfigSpace>(
    function: &mut Function<C>,
    timer: &dyn Timer,
    translated: bool,
    bus_reset: bool,
) -> Result<Option<Reset>, Error> {
    preflight(function, timer, translated, bus_reset)
}
impl<P: Preparation, V: Views<P::Config>> Cold<P, V> {
    pub fn new(inputs: Inputs<P, V>) -> Self {
        Self {
            kept: ManuallyDrop::new(Box::new(Pending {
                requester: Some(inputs.requester),
                views: inputs.views,
                timer: inputs.timer,
                translated: false,
            })),
        }
    }

    /// Let a function with no function-local reset start in the state
    /// firmware left, without one.
    ///
    /// # Safety
    /// The private domain the requester binds translates its every request,
    /// so DMA from engines firmware left running reaches only that domain.
    pub unsafe fn translated(mut self) -> Self {
        self.kept.translated = true;
        self
    }

    pub fn quiesce_and_bind(mut self) -> Result<Ready<P::Bound, V>, Failure<P, V>> {
        let attempt = &mut **self.kept;
        let requester = attempt.requester.as_mut().expect("original cold requester");
        let result = (|| {
            let bus_reset = requester.bus_reset_available();
            let reset = preflight(
                requester.function_mut(),
                attempt.timer.as_ref(),
                attempt.translated,
                bus_reset,
            )?;
            attempt
                .views
                .seal(requester.function_mut())
                .map_err(Error::Views)?;
            let timer = attempt.timer.as_ref();
            match reset {
                Some(Reset::Function(method)) => {
                    if requester
                        .function_mut()
                        .reset(timer)
                        .map_err(Error::Reset)?
                        != Some(method)
                    {
                        return Err(Error::ResetUnavailable);
                    }
                }
                Some(Reset::SecondaryBus) => requester.reset_bus(timer).map_err(Error::Reset)?,
                None => {}
            }
            requester::validate_stopped(requester.function_mut()).map_err(Error::Claim)?;
            Ok(reset)
        })();
        let reset = match result {
            Ok(reset) => reset,
            Err(error) => {
                return Err(Failure {
                    error,
                    _kept: self,
                    _binding: None,
                });
            }
        };

        let requester = attempt.requester.take().expect("original cold requester");
        // SAFETY: this exact function completed its reset or its bus's, or has
        // none and its DMA is translated, and stopped revalidation.
        // The original CPU views remain parked through the root drain and bind.
        match unsafe { requester.finalize() } {
            Ok(bound) => {
                let bound = ManuallyDrop::new(bound);
                // SAFETY: finalization acknowledged the original binding; take
                // this consumed envelope once, and transfer its exact views.
                let inputs = unsafe { ManuallyDrop::take(&mut self.kept) };
                Ok(Ready {
                    bound,
                    views: ManuallyDrop::new(inputs.views),
                    reset,
                })
            }
            Err((error, failure)) => Err(Failure {
                error: Error::Preparation(error),
                _kept: self,
                _binding: Some(ManuallyDrop::new(failure)),
            }),
        }
    }
}

#[cfg(test)]
mod tests;
