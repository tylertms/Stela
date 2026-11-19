//! Conventional HDA through one original-root/config/BAR/DMA lifetime.
use super::{Failure, NativeServices, ServiceError, map};
use crate::{
    Clock,
    audio::{
        self,
        hda::{self, CodecFailure, CodecGraph, CodecSetup},
    },
    bus::{
        mmio::{Mmio, owned::Mapping},
        pci::Address,
    },
    probe::Error,
};
use alloc::{boxed::Box, string::String, vec::Vec};
mod lifetime;
pub use lifetime::Error as RetirementError;
use lifetime::{Inputs, Native, OriginalUsers};

pub const PLAYBACK_FORMAT: audio::PcmFormat = audio::PcmFormat {
    rate: 48_000,
    bits: 16,
    channels: 2,
};
pub const PLAYBACK_PERIOD_BYTES: usize = 4096;
pub const PLAYBACK_PERIODS: usize = 8;
pub const PLAYBACK_RING_BYTES: usize = PLAYBACK_PERIOD_BYTES * PLAYBACK_PERIODS;

/// Active analog playback route bound on a native HDA controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlaybackInfo {
    pub codec: u8,
    pub pin: u8,
    pub converter: u8,
    pub device: hda::PinDevice,
    pub jack_detected: bool,
    pub ring_bytes: usize,
    pub period_bytes: usize,
    pub transport_only: bool,
}

/// HD Audio BAR0 through its actual owned uncached mapping/native widths.
pub struct Bar0 {
    mmio: Mmio,
    bytes: usize,
    mapping: Mapping,
}
impl hda::Registers for Bar0 {
    fn region_bytes(&self) -> usize {
        self.bytes
    }
    fn read8(&mut self, o: usize) -> u8 {
        self.mmio.read8(o)
    }
    fn read16(&mut self, o: usize) -> u16 {
        self.mmio.read16(o)
    }
    fn read32(&mut self, o: usize) -> u32 {
        self.mmio.read32(o)
    }
    fn write8(&mut self, o: usize, v: u8) {
        self.mmio.write8(o, v);
    }
    fn write16(&mut self, o: usize, v: u16) {
        self.mmio.write16(o, v);
    }
    fn write32(&mut self, o: usize, v: u32) {
        self.mmio.write32(o, v);
    }
}
pub struct Codec {
    pub graph: CodecGraph,
    pub setup: Result<CodecSetup, audio::Error>,
}
/// The real register provider/core/DMA allocator never escape independently.
/// Discovery is metadata; a retained success alone is not guest audio readiness.
pub struct Host {
    pub location: String,
    owner: Native<Bar0, audio::Timing<Clock>>,
    pub codecs: Vec<Codec>,
    pub codec_failures: Vec<CodecFailure>,
    pub discovery_error: Option<audio::Error>,
}
impl Host {
    /// Allocate and start a 48 kHz 16-bit stereo cyclic playback stream on the
    /// highest-priority analog output pin discovered on this controller.
    pub fn start_playback(&mut self) -> Result<PlaybackInfo, audio::Error> {
        self.owner.start_playback(&self.codecs)
    }

    /// Poll the running playback stream's hardware link position and status.
    pub fn playback_status(&mut self) -> Result<hda::StreamStatus, audio::Error> {
        self.owner.playback_status()
    }

    /// Write 4-byte-aligned 16-bit stereo PCM directly into the live DMA ring.
    pub fn write_playback(&mut self, offset: usize, bytes: &[u8]) -> Result<(), audio::Error> {
        self.owner.write_playback(offset, bytes)
    }

    /// Stop active playback DMA without retiring the controller.
    pub fn stop_playback(&mut self) -> Result<(), audio::Error> {
        self.owner.stop_playback()
    }

    /// Explicit one-shot hardware/root/lease/Attachment/mapping retirement.
    /// Failure and unwind keep the complete graph and permanently close entry.
    pub fn shutdown(&mut self) -> Result<(), RetirementError> {
        self.owner.close()
    }
}
struct ColdOwner {
    requester: Option<crate::bus::pci::requester::preparation::PreparingRequester>,
    mapping: Option<Mapping>,
    minimum: Option<alloc::rc::Rc<dyn crate::bus::pci::power::Timer>>,
}
impl Drop for ColdOwner {
    fn drop(&mut self) {
        core::mem::forget(self.requester.take());
        core::mem::forget(self.mapping.take());
        core::mem::forget(self.minimum.take());
    }
}
/// # Safety
/// NativeServices ownership, calibrated clock and original D0/decode/power
/// hold. Conventional identity alone cannot exclude firmware/AML/SMM/DSP users.
/// Their original root/backing and competing-owner exclusions remain required.
/// The separate original IRQ/work retirement producer is currently unavailable:
/// this factory performs source containment, then retains/refuses before bind.
#[expect(
    clippy::result_large_err,
    reason = "A failed HDA attach keeps its typed cause and all original resource owners"
)]
pub(in crate::probe) unsafe fn attach(
    services: &dyn NativeServices,
    address: Address,
    clock: Clock,
    minimum: alloc::rc::Rc<dyn crate::bus::pci::power::Timer>,
) -> Result<Host, Failure> {
    minimum.check().map_err(|error| {
        (
            Error::HdaNative(RetirementError::Controller(audio::Error::MinimumDelay(
                error,
            ))),
            None,
        )
    })?;
    // Check the largest selected conventional-core wait before any config or
    // vendor effect. Generic clock metadata alone does not qualify a timer.
    clock
        .check_duration(1_000_000)
        .map_err(|error| (Error::HdaNative(RetirementError::Clock(error)), None))?;
    let requester = services
        .prepare_requester(address)
        .map_err(|e| (Error::Native(e), None))?;
    let mut cold = ColdOwner {
        requester: Some(requester),
        mapping: None,
        minimum: Some(minimum),
    };
    let bars = match cold.requester.as_mut().unwrap().function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err((error.into(), Some(Box::new(cold)))),
    };
    let mapping = match map(services, address, &bars, 0) {
        Ok(m) => m,
        Err(e) => return Err((Error::Native(e), Some(Box::new(cold)))),
    };
    cold.mapping = Some(mapping);
    let bar = bars[0].expect("map checked BAR0");
    let window = (|| {
        let mapping = cold.mapping.as_ref().unwrap();
        let bytes = mapping.bytes()?;
        Ok((mapping.address(0, bytes)?, bytes))
    })();
    let (window, bytes) = match window {
        Ok(v) => v,
        Err(e) => {
            return Err((
                Error::Native(ServiceError::Mapping(e)),
                Some(Box::new(cold)),
            ));
        }
    };
    let registers = Bar0 {
        // SAFETY: exactly this original owned UC BAR0; Native outer owner
        // keeps actual mapping across constructor error and callback unwind.
        mmio: unsafe { Mmio::new(window.as_ptr() as u64, bytes) },
        bytes,
        mapping: cold.mapping.take().unwrap(),
    };
    let retained = match cold
        .requester
        .as_mut()
        .unwrap()
        .function_mut()
        .config()
        .retain_epoch()
    {
        Ok(retained) => retained,
        Err(error) => {
            return Err((
                Error::HdaNative(RetirementError::Mapping(error)),
                Some(Box::new(cold)),
            ));
        }
    };
    let inputs = Inputs {
        requester: cold.requester.take().unwrap(),
        registers,
        clock: audio::Timing::new(clock, cold.minimum.take().unwrap()),
        pages: services.pages(),
        bar,
        // SAFETY: native::hda::attach runs post-ExitBootServices during device
        // probe after start_platform parked inherited PCI functions, masked
        // legacy IRQs, installed our IDT/APIC, and initialized DMA remapping.
        original_users: Some(unsafe { OriginalUsers::post_ebs(retained) }),
    };
    let mut owner = match Native::new(inputs) {
        Ok(owner) => owner,
        Err(failure) => {
            let error = failure
                .construction_error()
                .cloned()
                .map_or(Error::HdaNative(failure.error), Error::Device);
            return Err((error, Some(Box::new(failure))));
        }
    };
    let discovery = match owner.discover_and_setup() {
        Ok(d) => d,
        Err(error) => return Err((Error::HdaNative(error), Some(Box::new(owner)))),
    };
    Ok(Host {
        location: alloc::format!("{address}"),
        owner,
        codecs: discovery.codecs,
        codec_failures: discovery.codec_failures,
        discovery_error: discovery.discovery_error,
    })
}
