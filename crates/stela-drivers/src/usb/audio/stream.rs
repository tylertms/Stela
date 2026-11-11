// SPDX-License-Identifier: GPL-2.0-or-later
//! Polled PCM packet ownership. Buffers return to the CPU only after a terminal
//! isochronous completion or acknowledged endpoint removal. No live refill is
//! inferred from a clock, ring position, or failed cancellation.
use super::packet::Packetizer;
use super::{Binding, Completion, Error, Format, PacketStatus, ResolvedClock, Sync, Transport};
use crate::{
    audio::Direction,
    bus::dma::{DmaAllocator, DmaBuffer},
    usb::descriptor::Endpoint,
};
use alloc::{
    collections::{BTreeMap, VecDeque},
    vec::Vec,
};

/// Packets a lane keeps in flight at most. The controller's transfer ring
/// holds 254 TDs and its event ring 256 events shared by every device, so
/// a stream leaves room for both.
const MAX_PACKETS: usize = 192;
/// Packets a lane keeps in flight at least, as far as its pool allows.
const MIN_PACKETS: usize = 32;
/// The service a lane's queue covers: a polled owner may be served this
/// late without starving the device.
const DEPTH_US: u64 = 64_000;
/// One lane's packet buffers, carved into slots of the stream's packet size.
const POOL_BYTES: usize = 128 * 1024;
/// A feedback lane's pool: packets of at most 4 bytes.
const FEEDBACK_POOL_BYTES: usize = 4096;
const SLOT_ALIGN: usize = 64;
/// A TD buffer must not cross a 64 KiB boundary.
const TD_BOUNDARY: u64 = 64 * 1024;
const PCM_LIMIT: usize = 1024 * 1024;
const FEEDBACK_TIMEOUT_US: u64 = 100_000;

struct Packet<K> {
    /// The slot's offset in the lane's pool.
    offset: usize,
    ticket: Option<K>,
    bytes: usize,
}
struct Lane<K> {
    endpoint: Option<Endpoint>,
    pool: DmaBuffer,
    /// The pool's slot size while armed.
    slot: usize,
    packets: Vec<Packet<K>>,
    order: VecDeque<usize>,
}
impl<K> Lane<K> {
    fn new(dma: &mut dyn DmaAllocator, bytes: usize) -> Result<Self, Error> {
        Ok(Self {
            endpoint: None,
            pool: dma.allocate(bytes, 4096)?,
            slot: 0,
            packets: Vec::new(),
            order: VecDeque::new(),
        })
    }
    /// Carve the idle pool into slots of `bytes` for a stream served every
    /// `microframes`: enough packets to cover [`DEPTH_US`], within
    /// [`MAX_PACKETS`] and the pool, no slot crossing a 64 KiB boundary.
    fn arm(&mut self, bytes: usize, microframes: u32) -> Result<(), Error> {
        debug_assert!(self.packets.iter().all(|p| p.ticket.is_none()));
        let slot = bytes.max(4).next_multiple_of(SLOT_ALIGN);
        let service_us = u64::from(microframes.max(1)) * 125;
        let wanted = (DEPTH_US.div_ceil(service_us) as usize).clamp(MIN_PACKETS, MAX_PACKETS);
        let base = self.pool.physical();
        let mut packets = Vec::new();
        let mut offset = 0usize;
        while packets.len() < wanted && offset + slot <= self.pool.len() {
            let start = base + offset as u64;
            if start / TD_BOUNDARY != (start + slot as u64 - 1) / TD_BOUNDARY {
                offset = ((start / TD_BOUNDARY + 1) * TD_BOUNDARY - base) as usize;
                continue;
            }
            packets.push(Packet {
                offset,
                ticket: None,
                bytes: 0,
            });
            offset += slot;
        }
        if packets.is_empty() {
            return Err(Error::Format);
        }
        self.slot = slot;
        self.packets = packets;
        self.order.clear();
        Ok(())
    }
    fn data(&self, index: usize, bytes: usize) -> &[u8] {
        let offset = self.packets[index].offset;
        &self.pool.bytes()[offset..offset + bytes]
    }
    fn queued(&self) -> usize {
        self.packets.iter().filter(|p| p.ticket.is_some()).count()
    }
    fn endpoints(&self) -> Vec<Endpoint> {
        self.endpoint.into_iter().collect()
    }
    fn released(&mut self) {
        for p in &mut self.packets {
            p.ticket = None;
            p.bytes = 0;
        }
        self.endpoint = None;
        self.order.clear();
    }
}
#[derive(Clone, Copy)]
enum StreamClock {
    Entity(ResolvedClock),
    Endpoint,
    Fixed,
}

struct Playback {
    format: Format,
    rate: u32,
    clock: StreamClock,
    packetizer: Packetizer,
    planned: Option<usize>,
    finite: bool,
    last_feedback: u64,
    last_progress: u64,
}
struct Capture {
    format: Format,
    rate: u32,
    clock: StreamClock,
    requested: bool,
    implicit: bool,
    last_progress: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    pub playback_frames: u64,
    pub capture_frames: u64,
    pub feedback_updates: u64,
    /// Packets whose data a USB error lost; the stream keeps running.
    pub packet_errors: u64,
    pub queued_playback: usize,
    pub queued_capture: usize,
    pub playback_drained: bool,
}

pub struct AudioDevice<T: Transport> {
    transport: T,
    binding: Binding,
    output: Lane<T::Ticket>,
    input: Lane<T::Ticket>,
    feedback: Lane<T::Ticket>,
    playback: Option<Playback>,
    capture: Option<Capture>,
    output_pcm: VecDeque<u8>,
    input_pcm: VecDeque<u8>,
    implicit_frames: VecDeque<usize>,
    progress: Progress,
    poisoned: bool,
    power_owners: Vec<super::PowerDomain>,
    /// First failure is held even if cleanup itself fails.
    pub last_fault: Option<Error>,
    /// The last rate change's (requested, running) rates when the clock
    /// reported another; 0 running means it could not be read.
    pub clock_mismatch: Option<(u32, u32)>,
    /// Linux `chip->sample_rate_read_error`: failed UAC1 rate reads.
    rate_read_errors: u8,
    mixer: MixerState,
}

/// What Linux keeps in each `usb_mixer_elem_info` across a reset-resume:
/// probed ranges and the values last read or written, by entity, selector
/// or input, and channel or output. Resume publishes a fresh owner; hand it
/// the old owner's state with `restore_mixer`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MixerState {
    ranges: BTreeMap<(u8, u8, u8), super::MixerRange>,
    values: BTreeMap<(u8, u8, u8), Cached>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cached {
    Mute(bool),
    Volume(i16),
    Feature(super::FeatureControl, super::FeatureValue),
    MixerCell(i16),
    Input(u8),
    Clock(u8),
}

impl AudioDevice<super::Native> {
    /// Query an advertised UAC2 terminal connector through native EP0.
    /// The returned channel observation grants no routing or PCM permission.
    pub fn connector_status(&mut self, terminal: u8) -> Result<super::ConnectorStatus, Error> {
        self.ready()?;
        self.transport.connector_status(terminal)
    }
    /// Create an idle owner for the optional UAC1/UAC2 AC status endpoint.
    /// Call `start`, poll and drain notices explicitly, then acknowledge `stop`.
    pub fn status_notifications(
        &self,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Option<super::StatusNotifications>, Error> {
        self.transport.status_notifications(dma)
    }
}

impl<T: Transport> AudioDevice<T> {
    pub fn new(transport: T, binding: Binding, dma: &mut dyn DmaAllocator) -> Result<Self, Error> {
        transport.validate()?;
        Ok(Self {
            transport,
            binding,
            output: Lane::new(dma, POOL_BYTES)?,
            input: Lane::new(dma, POOL_BYTES)?,
            feedback: Lane::new(dma, FEEDBACK_POOL_BYTES)?,
            playback: None,
            capture: None,
            output_pcm: VecDeque::new(),
            input_pcm: VecDeque::new(),
            implicit_frames: VecDeque::new(),
            progress: Progress::default(),
            poisoned: false,
            power_owners: Vec::new(),
            last_fault: None,
            clock_mismatch: None,
            rate_read_errors: 0,
            mixer: MixerState::default(),
        })
    }
    #[cfg(test)]
    pub(crate) fn generation_transport(&mut self) -> &mut T {
        &mut self.transport
    }

    #[cfg(test)]
    pub(crate) fn generation_snapshot(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend(self.output_pcm.iter().copied());
        bytes.extend(self.input_pcm.iter().copied());
        for count in [
            self.output_pcm.len(),
            self.input_pcm.len(),
            self.implicit_frames.len(),
        ] {
            bytes.extend_from_slice(&count.to_le_bytes());
        }
        bytes.extend([
            u8::from(self.playback.is_some()),
            u8::from(self.capture.is_some()),
            u8::from(self.poisoned),
            u8::from(self.last_fault.is_some()),
        ]);
        for lane in [&self.output, &self.input, &self.feedback] {
            bytes.extend_from_slice(&lane.order.len().to_le_bytes());
            bytes.push(u8::from(lane.endpoint.is_some()));
            bytes.extend_from_slice(lane.pool.bytes());
            for packet in &lane.packets {
                bytes.extend_from_slice(&packet.bytes.to_le_bytes());
                bytes.push(u8::from(packet.ticket.is_some()));
            }
        }
        bytes
    }

    /// Immutable descriptor snapshot; this does not establish current attachment.
    pub fn binding(&self) -> &Binding {
        &self.binding
    }
    /// Historical software accounting; poll validates the native owner.
    pub fn progress(&self) -> Progress {
        let mut p = self.progress;
        p.queued_playback = self.output.queued();
        p.queued_capture = self.input.queued();
        p
    }
    fn ready(&self) -> Result<(), Error> {
        self.transport.validate()?;
        if self.poisoned {
            Err(Error::Poisoned)
        } else {
            Ok(())
        }
    }
    fn prepare_power(&mut self) -> Result<(), Error> {
        if self.playback.is_some() || self.capture.is_some() {
            // This is a polled transport. Sleeping for a cold sibling's 300ms
            // recovery while an existing 32-packet queue runs would starve it.
            // Both BADD domains stay awake for the complete duplex session.
            for domain in &self.power_owners {
                domain.ensure_awake(&mut self.transport, self.binding.graph.control_interface)?;
            }
            return Ok(());
        }
        for format in &self.binding.formats {
            if let Some(domain) = format.power_domain
                && !self.power_owners.iter().any(|d| d.id == domain.id)
            {
                // Initially selected by class attachment; acknowledge zero again
                // before keeping a power owner, including an unused sibling.
                self.transport.alternate(format.interface, 0)?;
                self.power_owners.push(domain);
                domain.awake(&mut self.transport, self.binding.graph.control_interface)?;
            }
        }
        Ok(())
    }
    fn idle_power(&mut self) -> Result<(), Error> {
        if self.playback.is_none() && self.capture.is_none() {
            while let Some(domain) = self.power_owners.last().copied() {
                domain.idle(&mut self.transport, self.binding.graph.control_interface)?;
                self.power_owners.pop();
            }
        }
        Ok(())
    }
    fn powered_clock(&mut self, format: &Format, rate: u32) -> Result<StreamClock, Error> {
        if format.power_domain.is_none() {
            return self.clock(format, rate);
        }
        if self.binding.basic_profile.is_some() && rate != super::badd::RATE {
            return Err(Error::RateUnsupported);
        }
        let active = self.playback.is_some() || self.capture.is_some();
        // Full UAC3 clock entities may share the stream's power domain. Their
        // controls are usable only after D0 recovery, before any DMA.
        if let Err(error) = self.prepare_power() {
            return self.fail(error);
        }
        match self.clock(format, rate) {
            Ok(clock) => Ok(clock),
            // These are request/clock-sharing refusals before any rate write.
            // An existing healthy session must survive an invalid new request.
            Err(error @ (Error::Busy | Error::RateUnsupported)) if active => Err(error),
            Err(error) => self.fail(error),
        }
    }
    fn clock(&mut self, format: &Format, rate: u32) -> Result<StreamClock, Error> {
        if self.binding.basic_profile.is_some() {
            if rate != super::badd::RATE {
                return Err(Error::RateUnsupported);
            }
            return Ok(StreamClock::Fixed);
        }
        if let Some(rates) = &format.endpoint_rates {
            rates.validate_rate(rate)?;
            // UAC1 provides no clock-sharing graph. Do not retune one side of
            // an active duplex device to a conflicting nominal frequency.
            if self
                .playback
                .iter()
                .map(|p| p.rate)
                .chain(self.capture.iter().map(|c| c.rate))
                .any(|active| active != rate)
            {
                return Err(Error::Busy);
            }
            return Ok(StreamClock::Endpoint);
        }
        let clock = self
            .binding
            .graph
            .resolve_clock(&mut self.transport, format.clock)?;
        for (other_clock, other_rate) in self
            .playback
            .iter()
            .map(|p| (p.clock, p.rate))
            .chain(self.capture.iter().map(|c| (c.clock, c.rate)))
        {
            // Linux shares one clock reference per terminal clock ID.
            if let StreamClock::Entity(other_clock) = other_clock
                && other_clock.terminal == clock.terminal
                && other_rate != rate
            {
                return Err(Error::Busy);
            }
        }
        // ALSA constrains hw_params to the format's announced rates.
        if !self
            .binding
            .graph
            .rates(&mut self.transport, clock)?
            .contains(rate)
        {
            return Err(Error::RateUnsupported);
        }
        let running = self
            .binding
            .graph
            .set_rate(&mut self.transport, clock, rate)?;
        // Linux notes a different running rate and streams on.
        self.clock_mismatch = (running != rate).then_some((rate, running));
        Ok(StreamClock::Entity(clock))
    }
    fn format(&self, index: usize, direction: Direction) -> Result<Format, Error> {
        let f = self
            .binding
            .formats
            .get(index)
            .filter(|f| f.direction == direction)
            .ok_or(Error::Format)?;
        Ok(f.clone())
    }
    /// Start a finite or continuous playback queue with interleaved PCM.
    /// A continuous producer must replenish it before all submitted packets
    /// complete; starvation is an explicit underrun, never implicit silence.
    pub fn start_playback(
        &mut self,
        index: usize,
        rate: u32,
        pcm: &[u8],
        finite: bool,
    ) -> Result<(), Error> {
        self.ready()?;
        if self.playback.is_some() {
            return Err(Error::Busy);
        }
        let format = self.format(index, Direction::Playback)?;
        if format.frame_bytes() == 0
            || pcm.is_empty()
            || pcm.len() > PCM_LIMIT
            || !pcm.len().is_multiple_of(format.frame_bytes())
        {
            return Err(Error::Format);
        }
        let packetizer = Packetizer::new(&format, rate)?;
        if finite && format.fill_max && !pcm.len().is_multiple_of(format.max_bytes) {
            return Err(Error::Format);
        }
        let endpoints = format.endpoints();
        if self.capture.as_ref().is_some_and(|c| {
            c.format.interface == format.interface
                || endpoints.iter().any(|ep| ep.dci() == c.format.data.dci())
        }) {
            return Err(Error::Busy);
        }
        if let Sync::Implicit {
            interface,
            alternate,
            ..
        } = format.sync
            && self.capture.as_ref().is_some_and(|c| {
                c.format.interface != interface || c.format.alternate != alternate || c.rate != rate
            })
        {
            return Err(Error::Busy);
        }
        if let Sync::Implicit {
            interface,
            alternate,
            ..
        } = format.sync
        {
            let source = self
                .binding
                .formats
                .iter()
                .find(|f| {
                    f.interface == interface
                        && f.alternate == alternate
                        && f.direction == Direction::Capture
                })
                .ok_or(Error::Descriptor(
                    "implicit feedback source format is missing",
                ))?;
            if endpoints.iter().any(|ep| ep.dci() == source.data.dci()) {
                return Err(Error::Busy);
            }
        }
        self.transport.preflight(&endpoints)?;
        let clock = self.powered_clock(&format, rate)?;
        let result = (|| {
            if let Sync::Implicit {
                interface,
                alternate,
                ..
            } = format.sync
            {
                let capture_index = self
                    .binding
                    .formats
                    .iter()
                    .position(|f| {
                        f.interface == interface
                            && f.alternate == alternate
                            && f.direction == Direction::Capture
                    })
                    .ok_or(Error::Descriptor(
                        "implicit feedback source format is missing",
                    ))?;
                self.start_capture_inner(capture_index, rate, false, true, &mut false)?;
            }
            self.prepare_power()?;
            self.output
                .arm(format.max_bytes, format.service_microframes)?;
            self.output.endpoint = Some(format.data);
            if let Sync::Explicit(endpoint) = format.sync {
                let interval = super::descriptors::service_interval(endpoint, self.binding.speed)
                    .unwrap_or(format.service_microframes);
                self.feedback.arm(4, interval)?;
                self.feedback.endpoint = Some(endpoint);
            }
            self.output_pcm.extend(pcm.iter().copied());
            self.progress.playback_drained = false;
            self.playback = Some(Playback {
                format,
                rate,
                clock,
                packetizer,
                planned: None,
                finite,
                last_feedback: self.transport.now_us(),
                last_progress: self.transport.now_us(),
            });
            self.transport.alternate(
                self.playback.as_ref().unwrap().format.interface,
                self.playback.as_ref().unwrap().format.alternate,
            )?;
            let format = &self.playback.as_ref().unwrap().format;
            if let Some(rates) = &format.endpoint_rates {
                let running = rates.apply(
                    &mut self.transport,
                    format.data.address,
                    rate,
                    &mut self.rate_read_errors,
                )?;
                // Linux warns of a different rate and streams on.
                self.clock_mismatch = (running != rate).then_some((rate, running));
            }
            format.init_pitch(&mut self.transport)?;
            self.transport.configure(Direction::Playback, &endpoints)?;
            self.queue_inputs()?;
            self.queue_output()?;
            Ok(())
        })();
        if let Err(error) = result {
            return self.fail(error);
        }
        Ok(())
    }
    pub fn write_playback(&mut self, pcm: &[u8]) -> Result<(), Error> {
        self.ready()?;
        let p = self.playback.as_ref().ok_or(Error::NotRunning)?;
        if p.finite {
            return Err(Error::Busy);
        }
        if !pcm.len().is_multiple_of(p.format.frame_bytes()) {
            return Err(Error::Format);
        }
        if self.output_pcm.len().saturating_add(pcm.len()) > PCM_LIMIT {
            return Err(Error::Overrun);
        }
        self.output_pcm.extend(pcm.iter().copied());
        Ok(())
    }
    pub fn finish_playback(&mut self) -> Result<(), Error> {
        self.ready()?;
        let p = self.playback.as_mut().ok_or(Error::NotRunning)?;
        if p.format.fill_max && !self.output_pcm.len().is_multiple_of(p.format.max_bytes) {
            return Err(Error::Format);
        }
        p.finite = true;
        Ok(())
    }
    pub fn start_capture(&mut self, index: usize, rate: u32) -> Result<(), Error> {
        self.ready()?;
        let format = self.format(index, Direction::Capture)?;
        if self
            .capture
            .as_ref()
            .is_some_and(|c| c.format != format || c.rate != rate)
        {
            return Err(Error::Busy);
        }
        let mut effects_attempted = false;
        let result = self.start_capture_inner(index, rate, true, false, &mut effects_attempted);
        if let Err(error) = result {
            // Existing duplex-session power does not belong to this request.
            // A rate/alias/preflight refusal cannot tear down a healthy sibling.
            if self.input.endpoint.is_none() && !effects_attempted {
                return Err(error);
            }
            return self.fail(error);
        }
        Ok(())
    }
    fn start_capture_inner(
        &mut self,
        index: usize,
        rate: u32,
        requested: bool,
        implicit: bool,
        effects_attempted: &mut bool,
    ) -> Result<(), Error> {
        let format = self.format(index, Direction::Capture)?;
        Packetizer::new(&format, rate)?;
        if let Some(c) = self.capture.as_mut() {
            if c.format != format || c.rate != rate {
                return Err(Error::Busy);
            }
            c.requested |= requested;
            c.implicit |= implicit;
            return Ok(());
        }
        if self.playback.as_ref().is_some_and(|p| {
            p.format.interface == format.interface
                || p.format
                    .endpoints()
                    .iter()
                    .any(|ep| ep.dci() == format.data.dci())
        }) {
            return Err(Error::Busy);
        }
        self.transport.preflight(&[format.data])?;
        let clock = self.powered_clock(&format, rate)?;
        // Beyond preflight, an unsuccessful live power observation invalidates
        // the session's permission even if it does not acquire a new domain.
        *effects_attempted = true;
        self.prepare_power()?;
        self.input
            .arm(format.max_bytes, format.service_microframes)?;
        self.input.endpoint = Some(format.data);
        self.capture = Some(Capture {
            format,
            rate,
            clock,
            requested,
            implicit,
            last_progress: self.transport.now_us(),
        });
        let format = &self.capture.as_ref().unwrap().format;
        self.transport
            .alternate(format.interface, format.alternate)?;
        if let Some(rates) = &format.endpoint_rates {
            let running = rates.apply(
                &mut self.transport,
                format.data.address,
                rate,
                &mut self.rate_read_errors,
            )?;
            self.clock_mismatch = (running != rate).then_some((rate, running));
        }
        format.init_pitch(&mut self.transport)?;
        self.transport
            .configure(Direction::Capture, &self.input.endpoints())?;
        self.queue_inputs()
    }
    pub fn read_capture(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.ready()?;
        let c = self
            .capture
            .as_ref()
            .filter(|c| c.requested)
            .ok_or(Error::NotRunning)?;
        if !out.len().is_multiple_of(c.format.frame_bytes()) {
            return Err(Error::Format);
        }
        let bytes = self.input_pcm.len().min(out.len());
        for byte in &mut out[..bytes] {
            *byte = self.input_pcm.pop_front().unwrap();
        }
        Ok(bytes)
    }
    /// Poll the standardized headset-adapter insertion bitmap. Status delivery
    /// is explicit polling; no interrupt notification is inferred from this read.
    pub fn connector_present(&mut self, terminal: u8) -> Result<bool, Error> {
        self.ready()?;
        if self.binding.basic_profile != Some(super::BasicProfile::HeadsetAdapter)
            || !matches!(terminal, 3 | 4)
        {
            return Err(Error::Unsupported(
                "no BADD insertion control for this terminal",
            ));
        }
        let reply = self.transport.control(
            crate::usb::descriptor::setup(
                0xa1,
                1,
                1 << 8,
                u16::from(terminal) << 8 | u16::from(self.binding.graph.control_interface),
                2,
            ),
            &[],
        )?;
        if reply.len() != 2 {
            return Err(Error::ControlLength);
        }
        if reply[0] != 1 || reply[1] & !1 != 0 {
            return Err(Error::ControlReadback);
        }
        Ok(reply[1] != 0)
    }
    pub fn set_mute(&mut self, entity: u8, channel: u8, muted: bool) -> Result<(), Error> {
        self.ready()?;
        self.binding
            .graph
            .set_mute(&mut self.transport, entity, channel, muted)?;
        self.mixer
            .values
            .insert((entity, 1, channel), Cached::Mute(muted));
        Ok(())
    }
    /// The volume's range, probed before its first use.
    pub fn volume_range(&mut self, entity: u8, channel: u8) -> Result<super::MixerRange, Error> {
        self.ready()?;
        if let Some(range) = self.mixer.ranges.get(&(entity, 2, channel)) {
            return Ok(*range);
        }
        let range = self
            .binding
            .graph
            .volume_range(&mut self.transport, entity, channel)?;
        self.mixer.ranges.insert((entity, 2, channel), range);
        Ok(range)
    }
    /// Set the step at or below `volume_db256`, clamped to the range.
    pub fn set_volume(&mut self, entity: u8, channel: u8, volume_db256: i16) -> Result<(), Error> {
        let range = self.volume_range(entity, channel)?;
        let written = range.snap(i64::from(volume_db256)) as i16;
        self.binding
            .graph
            .set_volume(&mut self.transport, entity, channel, range, written)?;
        self.mixer
            .values
            .insert((entity, 2, channel), Cached::Volume(written));
        Ok(())
    }
    pub fn feature_value(
        &mut self,
        entity: u8,
        channel: u8,
        feature: super::FeatureControl,
    ) -> Result<super::FeatureValue, Error> {
        self.ready()?;
        // Linux probes a control when it creates it, before any read.
        if feature.has_range() {
            self.feature_range(entity, channel, feature)?;
        }
        let value =
            self.binding
                .graph
                .feature_value(&mut self.transport, entity, channel, feature)?;
        self.mixer.values.insert(
            (entity, feature.selector(), channel),
            Cached::Feature(feature, value),
        );
        Ok(value)
    }
    /// A numeric feature control's range, probed before its first use.
    pub fn feature_range(
        &mut self,
        entity: u8,
        channel: u8,
        feature: super::FeatureControl,
    ) -> Result<super::MixerRange, Error> {
        self.ready()?;
        let key = (entity, feature.selector(), channel);
        if let Some(range) = self.mixer.ranges.get(&key) {
            return Ok(*range);
        }
        let range =
            self.binding
                .graph
                .feature_range(&mut self.transport, entity, channel, feature)?;
        self.mixer.ranges.insert(key, range);
        Ok(range)
    }
    pub fn set_feature_value(
        &mut self,
        entity: u8,
        channel: u8,
        feature: super::FeatureControl,
        value: super::FeatureValue,
    ) -> Result<(), Error> {
        self.ready()?;
        let range = if feature.has_range() {
            Some(self.feature_range(entity, channel, feature)?)
        } else {
            None
        };
        let written = self.binding.graph.set_feature_value(
            &mut self.transport,
            entity,
            channel,
            feature,
            range,
            value,
        )?;
        self.mixer.values.insert(
            (entity, feature.selector(), channel),
            Cached::Feature(feature, written),
        );
        Ok(())
    }
    pub fn mixer_volume(&mut self, entity: u8, input: u8, output: u8) -> Result<i16, Error> {
        self.mixer_volume_range(entity, input, output)?;
        let value = self
            .binding
            .graph
            .mixer_volume(&mut self.transport, entity, input, output)?;
        self.mixer
            .values
            .insert((entity, input, output), Cached::MixerCell(value));
        Ok(value)
    }
    /// The mixer cell's range, probed before its first use.
    pub fn mixer_volume_range(
        &mut self,
        entity: u8,
        input: u8,
        output: u8,
    ) -> Result<super::MixerRange, Error> {
        self.ready()?;
        if let Some(range) = self.mixer.ranges.get(&(entity, input, output)) {
            return Ok(*range);
        }
        let range =
            self.binding
                .graph
                .mixer_volume_range(&mut self.transport, entity, input, output)?;
        self.mixer.ranges.insert((entity, input, output), range);
        Ok(range)
    }
    /// Set the step at or below `value`, clamped to the range.
    pub fn set_mixer_volume(
        &mut self,
        entity: u8,
        input: u8,
        output: u8,
        value: i16,
    ) -> Result<(), Error> {
        let range = self.mixer_volume_range(entity, input, output)?;
        let written = range.snap(i64::from(value)) as i16;
        self.binding.graph.set_mixer_volume(
            &mut self.transport,
            entity,
            input,
            output,
            range,
            written,
        )?;
        self.mixer
            .values
            .insert((entity, input, output), Cached::MixerCell(written));
        Ok(())
    }
    /// The mixer state to hand a fresh owner after resume.
    pub fn mixer_state(&self) -> MixerState {
        self.mixer.clone()
    }
    /// Linux `snd_usb_mixer_resume` after a reset-resume: adopt the old
    /// owner's ranges and write every cached value back. Cached values are
    /// steps of their ranges, so they are written unchanged. As
    /// `restore_mixer_value`, a failed write is skipped, not reported.
    pub fn restore_mixer(&mut self, state: MixerState) -> Result<(), Error> {
        self.ready()?;
        let graph = &self.binding.graph;
        let io = &mut self.transport;
        for (&(entity, selector, channel), &value) in &state.values {
            let range = state.ranges.get(&(entity, selector, channel)).copied();
            let _ = match (value, range) {
                (Cached::Mute(muted), _) => graph.set_mute(io, entity, channel, muted),
                (Cached::Volume(volume), Some(range)) => {
                    graph.set_volume(io, entity, channel, range, volume)
                }
                (Cached::Feature(feature, value), range) => graph
                    .set_feature_value(io, entity, channel, feature, range, value)
                    .map(|_| ()),
                (Cached::MixerCell(value), Some(range)) => {
                    graph.set_mixer_volume(io, entity, selector, channel, range, value)
                }
                (Cached::Input(pin), _) => graph.signal_selection(io, entity, pin),
                (Cached::Clock(pin), _) => graph.clock_selection(io, entity, pin),
                // Numeric controls are probed before they are cached.
                (Cached::Volume(_) | Cached::MixerCell(_), None) => Ok(()),
            };
        }
        self.mixer = state;
        Ok(())
    }
    pub fn select_clock(&mut self, entity: u8, pin: u8) -> Result<(), Error> {
        self.ready()?;
        if self.playback.is_some() || self.capture.is_some() {
            return Err(Error::Busy);
        }
        self.binding
            .graph
            .clock_selection(&mut self.transport, entity, pin)?;
        self.mixer.values.insert((entity, 1, 0), Cached::Clock(pin));
        Ok(())
    }
    pub fn select_input(&mut self, entity: u8, pin: u8) -> Result<(), Error> {
        self.ready()?;
        if self.playback.is_some() || self.capture.is_some() {
            return Err(Error::Busy);
        }
        self.binding
            .graph
            .signal_selection(&mut self.transport, entity, pin)?;
        self.mixer.values.insert((entity, 1, 0), Cached::Input(pin));
        Ok(())
    }
    fn queue_inputs(&mut self) -> Result<(), Error> {
        self.transport.validate()?;
        for lane in [&mut self.input, &mut self.feedback] {
            let Some(endpoint) = lane.endpoint else {
                continue;
            };
            let bytes = if endpoint.attributes & 0x30 == 0x10 {
                if self.binding.speed == 3 { 4 } else { 3 }
            } else {
                self.capture
                    .as_ref()
                    .ok_or(Error::NotRunning)?
                    .format
                    .max_bytes
            };
            if bytes > lane.slot {
                return Err(Error::Format);
            }
            let Lane {
                pool,
                packets,
                order,
                ..
            } = lane;
            for (index, packet) in packets.iter_mut().enumerate() {
                if packet.ticket.is_none() {
                    pool.bytes_mut()[packet.offset..packet.offset + bytes].fill(0xa5);
                    packet.bytes = bytes;
                    packet.ticket = Some(self.transport.submit(
                        &endpoint,
                        pool.physical() + packet.offset as u64,
                        bytes,
                    )?);
                    order.push_back(index);
                }
            }
        }
        Ok(())
    }
    fn queue_output(&mut self) -> Result<(), Error> {
        self.transport.validate()?;
        let Some(p) = self.playback.as_mut() else {
            return Ok(());
        };
        let endpoint = self.output.endpoint.ok_or(Error::NotRunning)?;
        let Lane {
            pool,
            slot,
            packets,
            order,
            ..
        } = &mut self.output;
        for (index, packet) in packets.iter_mut().enumerate() {
            if packet.ticket.is_some() {
                continue;
            }
            let bytes = match p.planned.take() {
                Some(bytes) => bytes,
                None => {
                    if matches!(p.format.sync, Sync::Implicit { .. }) {
                        let Some(frames) = self.implicit_frames.pop_front() else {
                            break;
                        };
                        frames
                            .checked_mul(p.format.frame_bytes())
                            .ok_or(Error::Feedback)?
                    } else {
                        p.packetizer.next_bytes()?
                    }
                }
            };
            if bytes > p.format.max_bytes || bytes > *slot {
                return Err(Error::Feedback);
            }
            let bytes = if self.output_pcm.len() < bytes {
                if p.finite {
                    self.output_pcm.len()
                } else {
                    p.planned = Some(bytes);
                    break;
                }
            } else {
                bytes
            };
            if bytes == 0 && p.finite && self.output_pcm.is_empty() {
                break;
            }
            // A fractional zero-frame interval still consumes USB service time.
            // Submit its ZLP TD rather than compressing sample cadence.
            for b in &mut pool.bytes_mut()[packet.offset..packet.offset + bytes] {
                *b = self.output_pcm.pop_front().unwrap();
            }
            packet.bytes = bytes;
            packet.ticket = Some(self.transport.submit(
                &endpoint,
                pool.physical() + packet.offset as u64,
                bytes,
            )?);
            order.push_back(index);
        }
        Ok(())
    }
    /// Like snd-usb-audio, a packet error loses only that packet: playback
    /// counts it as sent, while capture and feedback take no bytes from it.
    fn packet(
        progress: &mut Progress,
        completion: Completion,
        expected: usize,
        input: bool,
    ) -> Result<usize, Error> {
        if !matches!(
            completion.status,
            PacketStatus::Success | PacketStatus::Short
        ) {
            progress.packet_errors += 1;
            return Ok(if input { 0 } else { expected });
        }
        if completion.bytes > expected || (!input && completion.bytes != expected) {
            return Err(Error::Packet("USB audio packet length"));
        }
        Ok(completion.bytes)
    }
    fn poll_inner(&mut self) -> Result<(), Error> {
        while let Some(index) = self.feedback.order.front().copied() {
            let packet = &mut self.feedback.packets[index];
            let ticket = packet.ticket.as_ref().unwrap();
            let Some(result) = self.transport.completed(ticket) else {
                break;
            };
            let bytes = Self::packet(&mut self.progress, result?, packet.bytes, true)?;
            packet.ticket = None;
            self.feedback.order.pop_front();
            let data = self.feedback.data(index, bytes);
            if bytes == 0 || data.iter().all(|byte| *byte == 0) {
                continue;
            }
            let p = self.playback.as_mut().ok_or(Error::NotRunning)?;
            p.packetizer.feedback(data, self.binding.speed);
            p.last_feedback = self.transport.now_us();
            self.progress.feedback_updates += 1;
        }
        if let Some(c) = &mut self.capture {
            while let Some(index) = self.input.order.front().copied() {
                let packet = &mut self.input.packets[index];
                let ticket = packet.ticket.as_ref().unwrap();
                let Some(result) = self.transport.completed(ticket) else {
                    break;
                };
                let bytes = Self::packet(&mut self.progress, result?, packet.bytes, true)?;
                packet.ticket = None;
                self.input.order.pop_front();
                if !bytes.is_multiple_of(c.format.frame_bytes()) {
                    return Err(Error::Packet("capture packet splits a PCM frame"));
                }
                let frames = bytes / c.format.frame_bytes();
                if frames > 0 {
                    c.last_progress = self.transport.now_us();
                }
                if c.requested {
                    if self.input_pcm.len().saturating_add(bytes) > PCM_LIMIT {
                        return Err(Error::Overrun);
                    }
                    self.input_pcm
                        .extend(self.input.data(index, bytes).iter().copied());
                    self.progress.capture_frames += frames as u64;
                }
                if c.implicit && frames > 0 {
                    if self.implicit_frames.len() >= MAX_PACKETS * 2 {
                        return Err(Error::Overrun);
                    }
                    self.implicit_frames.push_back(frames);
                    self.progress.feedback_updates += 1;
                    if let Some(p) = self.playback.as_mut() {
                        p.last_feedback = self.transport.now_us();
                    }
                }
            }
        }
        if let Some(p) = self.playback.as_mut() {
            while let Some(index) = self.output.order.front().copied() {
                let packet = &mut self.output.packets[index];
                let ticket = packet.ticket.as_ref().unwrap();
                let Some(result) = self.transport.completed(ticket) else {
                    break;
                };
                let bytes = Self::packet(&mut self.progress, result?, packet.bytes, false)?;
                packet.ticket = None;
                self.output.order.pop_front();
                self.progress.playback_frames += (bytes / p.format.frame_bytes()) as u64;
                p.last_progress = self.transport.now_us();
            }
            let feedback_deadline = if let Sync::Explicit(endpoint) = p.format.sync {
                FEEDBACK_TIMEOUT_US.max(
                    u64::from(super::descriptors::service_interval(
                        endpoint,
                        self.binding.speed,
                    )?) * 125
                        * 4,
                )
            } else {
                FEEDBACK_TIMEOUT_US
            };
            if matches!(p.format.sync, Sync::Explicit(_) | Sync::Implicit { .. })
                && self.transport.now_us().saturating_sub(p.last_feedback) > feedback_deadline
            {
                return Err(Error::Feedback);
            }
        }
        let now = self.transport.now_us();
        // No completion (or indefinitely empty input) is not healthy streaming.
        // Include four service intervals for unusual long-bInterval alts.
        let deadline = |f: &Format| 1_000_000u64.max(u64::from(f.service_microframes) * 125 * 4);
        if self
            .playback
            .as_ref()
            .is_some_and(|p| now.saturating_sub(p.last_progress) > deadline(&p.format))
        {
            return Err(Error::Packet("USB audio playback made no progress"));
        }
        if self
            .capture
            .as_ref()
            .is_some_and(|c| now.saturating_sub(c.last_progress) > deadline(&c.format))
        {
            return Err(Error::Packet("USB audio capture made no progress"));
        }
        self.queue_inputs()?;
        self.queue_output()?;
        if let Some(p) = &self.playback
            && self.output.queued() == 0
            && (self.output_pcm.is_empty() || (!p.finite && p.planned.is_some()))
        {
            if p.finite {
                self.stop_playback()?;
                self.progress.playback_drained = true;
            } else {
                return Err(Error::Underrun);
            }
        }
        Ok(())
    }
    pub fn poll(&mut self) -> Result<Progress, Error> {
        self.ready()?;
        if let Err(error) = self.poll_inner() {
            return self.fail(error);
        }
        Ok(self.progress())
    }
    fn stop_output(&mut self) -> Result<(), Error> {
        let mut endpoints = self.output.endpoints();
        endpoints.extend(self.feedback.endpoints());
        if !endpoints.is_empty() {
            self.transport
                .deconfigure(Direction::Playback, &endpoints)?;
        }
        if let Some(p) = &self.playback {
            self.transport.alternate(p.format.interface, 0)?;
        }
        self.output.released();
        self.feedback.released();
        self.playback = None;
        self.output_pcm.clear();
        self.implicit_frames.clear();
        self.idle_power()
    }
    fn stop_input(&mut self) -> Result<(), Error> {
        let endpoints = self.input.endpoints();
        if !endpoints.is_empty() {
            self.transport.deconfigure(Direction::Capture, &endpoints)?;
        }
        if let Some(c) = &self.capture {
            self.transport.alternate(c.format.interface, 0)?;
        }
        self.input.released();
        self.capture = None;
        self.input_pcm.clear();
        self.idle_power()
    }
    pub fn stop_playback(&mut self) -> Result<(), Error> {
        self.ready()?;
        if let Err(error) = self.stop_output() {
            self.last_fault = Some(error);
            self.poisoned = true;
            return Err(Error::Poisoned);
        }
        if let Some(c) = self.capture.as_mut() {
            c.implicit = false;
            if !c.requested
                && let Err(error) = self.stop_input()
            {
                self.last_fault = Some(error);
                self.poisoned = true;
                return Err(Error::Poisoned);
            }
        }
        Ok(())
    }
    pub fn stop_capture(&mut self) -> Result<(), Error> {
        self.ready()?;
        if let Some(c) = self.capture.as_mut()
            && c.implicit
        {
            c.requested = false;
            self.input_pcm.clear();
            return Ok(());
        }
        if let Err(error) = self.stop_input() {
            self.last_fault = Some(error);
            self.poisoned = true;
            return Err(Error::Poisoned);
        }
        Ok(())
    }
    pub fn shutdown(&mut self) -> Result<(), Error> {
        self.transport.validate()?;
        let output = self.stop_output();
        let input = self.stop_input();
        if output.is_err() || input.is_err() {
            self.poisoned = true;
            return Err(Error::Poisoned);
        }
        self.poisoned = false;
        Ok(())
    }
    fn fail<R>(&mut self, error: Error) -> Result<R, Error> {
        self.last_fault = Some(error.clone());
        if self.shutdown().is_err() {
            Err(Error::Poisoned)
        } else {
            Err(error)
        }
    }
}

impl<T: Transport> Drop for AudioDevice<T> {
    fn drop(&mut self) {
        // DMA storage lives for the boot. An uncertain stop is held and
        // quarantined by Transport; it never licenses reuse by this owner.
        let _ = self.shutdown();
    }
}

#[cfg(test)]
pub(crate) mod tests;
