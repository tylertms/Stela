// SPDX-License-Identifier: GPL-2.0-or-later
//! USB Audio Class 1/2/3 PCM and UAC3 BADD, on native isochronous transport.
//! Mixer adaptation keeps Copyright (c) 2002 Takashi Iwai; original audio.c
//! contributions by Alan Cox and Thomas Sailer. UAC2 layout reference keeps
//! Copyright (c) 2010 Daniel Mack. See NOTICE.
//! Control/rate/feedback sequencing adapts Linux sound/usb/{clock,format,
//! endpoint,pcm,mixer}.c at a804c5742bff224d4cb4d25dd383554d598be32b.
//! Like snd-usb-audio, standard UAC1/UAC2 control interfaces bind by class; their
//! descriptors alone decide clocks, alternates, channels and sample rates. UAC3
//! BADD uses its owning standardized profile and endpoint/power controls.
//! Full UAC3 resolves high-capability cluster replies before publication.
use super::descriptor::{Endpoint, Interface};
use crate::{audio::Direction, block::BlockError};

mod badd;
mod plan;
mod uac3;
pub use plan::Plan;
pub use uac3::{ChannelInformation, Cluster};
mod uac1;
pub use badd::BasicProfile;
mod power;
pub use power::PowerDomain;
pub use uac1::EndpointRates;

mod connector;
mod control;
pub use connector::ConnectorStatus;
mod descriptors;
mod feature;
pub use feature::{FeatureControl, FeatureValue};
mod mixer;
mod native;
mod packet;
pub use native::{Native, UsbAudio};
mod status;
pub use status::{
    StatusAttribute, StatusEvent, StatusNotice, StatusNotifications, Uac1StatusNotice,
};
pub mod sink;
mod stream;
pub use control::{Control, MixerRange, RateRange, Rates, ResolvedClock};
pub use descriptors::{Binding, Entity, Format, Graph, Sync};

pub const CLASS: u8 = 1;
pub const CONTROL_SUBCLASS: u8 = 1;
pub const STREAMING_SUBCLASS: u8 = 2;
pub const UAC1: u8 = 0;
pub const UAC2: u8 = 0x20;
pub const UAC3: u8 = 0x30;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Descriptor(&'static str),
    Unsupported(&'static str),
    Transport(BlockError),
    ControlLength,
    ControlReadback,
    ClockInvalid,
    PowerState,
    RateUnsupported,
    Format,
    Busy,
    NotRunning,
    Feedback,
    Underrun,
    Overrun,
    Packet(&'static str),
    Poisoned,
}
impl From<BlockError> for Error {
    fn from(error: BlockError) -> Self {
        Self::Transport(error)
    }
}

/// A terminal isochronous packet completion releases its buffer even when the
/// packet failed. Transport errors do not grant ownership back to the caller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketStatus {
    Success,
    Short,
    MissedService,
    Transaction,
    Babble,
    Bandwidth,
    BufferOverrun,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Completion {
    pub bytes: usize,
    pub status: PacketStatus,
}

/// Control/isochronous ownership boundary, also used by stateful tests.
/// `deconfigure` must acknowledge endpoint stop, drain and context removal
/// before returning success. Implementations keep all DMA on any uncertainty.
pub trait Transport: Control {
    type Ticket;
    /// Validate kept device identity without MMIO or DMA mutation. Native
    /// transports must reject retired generations before caller-owned PCM writes.
    fn validate(&self) -> Result<(), BlockError>;
    /// Observe that every proposed DCI is unowned, before SET_INTERFACE or any
    /// cleanup plan is staged. Callers serialize this observation with mutation.
    fn preflight(&mut self, endpoints: &[Endpoint]) -> Result<(), BlockError>;
    fn configure(&mut self, owner: Direction, endpoints: &[Endpoint]) -> Result<(), BlockError>;
    fn deconfigure(&mut self, owner: Direction, endpoints: &[Endpoint]) -> Result<(), BlockError>;
    fn submit(
        &mut self,
        endpoint: &Endpoint,
        physical: u64,
        bytes: usize,
    ) -> Result<Self::Ticket, BlockError>;
    fn completed(&mut self, ticket: &Self::Ticket) -> Option<Result<Completion, BlockError>>;
}

pub(super) fn plan(
    interface: &Interface,
    all: &[Interface],
    configuration: &[u8],
    speed: u8,
) -> Result<Plan, Error> {
    Plan::parse(interface, all, configuration, speed)
}

pub use stream::{AudioDevice, MixerState, Progress};

#[cfg(test)]
pub(crate) use descriptors::tests::board_codec as board_codec_configuration;
#[cfg(test)]
pub(crate) use stream::tests::configuration as test_configuration;

#[cfg(test)]
pub(crate) use native::tests::generation as generation_tests;
