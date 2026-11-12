// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
// Copyright (C) 2019 Lorenzo Bianconi <lorenzo.bianconi83@gmail.com>
//! Serialized polling boot/runtime mailbox over WFDMA TX/RX queues. Firmware
//! scatter needs real TX-DMA completion; acknowledged commands need both DMA
//! completion and a correctly sequenced MCU event. Timeout poisons the
//! complete mailbox epoch and keeps owned queues/pins until controller reset.
use super::{
    download::{FailureKind, PortState, Transport, TransportFailure},
    firmware::{Prepared, Requests},
    identity::Identity,
    mcu::{self, Command, Encoder, Event, Queue},
    ring::{
        self, QueueIo, TxRing,
        rx::{Assembly, Frame, RxRing},
    },
};
use alloc::vec::Vec;

pub mod clc;
pub mod device;
pub mod eeprom;
pub mod firmware;
pub mod init;
pub mod key;

static NEXT_OWNER: crate::wifi::Identities = crate::wifi::Identities::new();

// PCI send callbacks override the generic fill helper's 20*HZ with
// 3*HZ before the common MCU owner computes its response expiry.
const PCI_TIMEOUT_US: u64 = 3_000_000;
const MAX_EVENTS: usize = 64;
const MAX_EVENT_BYTES: usize = 64 * 1024;
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Cause {
    PendingTransmit,
    OwnerExhausted,
    ClockUnavailable,
    Ring(ring::Error),
    Message(mcu::Error),
    Timeout,
    EventLimit,
    CoreDump,
    CookieMismatch,
    Poisoned,
    FirmwareTarget,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    Boot(Command),
    Unified(mcu::runtime::Command),
    Capabilities,
    Encapsulation,
    Clc,
    Domain,
    Power(mcu::power::Kind),
    Initial(mcu::init::Kind),
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Failure {
    pub command: Operation,
    pub sequence: u8,
    pub dma_released: bool,
    pub cause: Cause,
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Reply {
    pub command: Command,
    pub sequence: u8,
    pub result: u32,
    pub dma_released: bool,
    pub frame: Frame,
}
/// UNI result plus real TX DMA completion. A nonzero status is an
/// explicit firmware rejection; it must never install a caller's context.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct UnifiedReply {
    pub command: mcu::runtime::Command,
    pub sequence: u8,
    pub status: u32,
    pub dma_released: bool,
    pub frame: Frame,
}
/// Matched NIC-capability response, not a station-style status ACK.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityReply {
    pub sequence: u8,
    pub dma_released: bool,
    pub frame: Frame,
}
/// Owned observations bound to the native command owner. This
/// receipt does not advertise radio modes or satisfy board/regulatory setup.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CapabilityReport {
    identity: Identity,
    owner: u64,
    reply: CapabilityReply,
    observed: mcu::capabilities::Capabilities,
}
impl CapabilityReport {
    pub fn identity(&self) -> Identity {
        self.identity
    }
    pub fn reply(&self) -> &CapabilityReply {
        &self.reply
    }
    pub fn observed(&self) -> &mcu::capabilities::Capabilities {
        &self.observed
    }
    /// Historical native owner binding only. The controller still
    /// validates live power/firmware/reset/port state and policy before use.
    pub fn belongs_to<W: QueueIo, F: QueueIo, R: QueueIo>(
        &self,
        mailbox: &Mailbox<W, F, R>,
    ) -> bool {
        self.owner == mailbox.owner
    }
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CapabilityQueryFailure {
    Transport(TransportFailure),
    /// Command/response and real TX DMA completed. Preserve the owned
    /// raw response even though its capability body cannot be accepted.
    Malformed {
        error: mcu::capabilities::Error,
        reply: CapabilityReply,
    },
}
impl CapabilityQueryFailure {
    pub fn port(&self) -> PortState {
        match self {
            Self::Transport(e) => e.port,
            Self::Malformed { .. } => PortState::Usable,
        }
    }
}
impl core::fmt::Display for CapabilityQueryFailure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek capability query: {self:?}")
    }
}
impl core::error::Error for CapabilityQueryFailure {}
enum Packet {
    Boot(mcu::Packet),
    Unified(mcu::runtime::Packet),
    Capabilities(mcu::capabilities::Packet),
    Encapsulation(mcu::eeprom::Packet),
    Clc(mcu::clc::Packet),
    Domain(mcu::domain::Packet),
    Power(mcu::power::Packet),
    Initial(mcu::init::Packet),
}
impl Packet {
    fn operation(&self) -> Operation {
        match self {
            Self::Boot(p) => Operation::Boot(p.command),
            Self::Unified(p) => Operation::Unified(p.command()),
            Self::Capabilities(_) => Operation::Capabilities,
            Self::Encapsulation(_) => Operation::Encapsulation,
            Self::Clc(_) => Operation::Clc,
            Self::Domain(_) => Operation::Domain,
            Self::Power(p) => Operation::Power(p.kind()),
            Self::Initial(p) => Operation::Initial(p.kind()),
        }
    }
    fn sequence(&self) -> u8 {
        match self {
            Self::Boot(p) => p.sequence,
            Self::Unified(p) => p.sequence(),
            Self::Capabilities(p) => p.sequence(),
            Self::Encapsulation(p) => p.sequence(),
            Self::Clc(p) => p.sequence(),
            Self::Domain(p) => p.sequence(),
            Self::Power(p) => p.sequence(),
            Self::Initial(p) => p.sequence(),
        }
    }
    fn bytes(&self) -> &[u8] {
        match self {
            Self::Boot(p) => &p.bytes,
            Self::Unified(p) => p.bytes(),
            Self::Capabilities(p) => p.bytes(),
            Self::Encapsulation(p) => p.bytes(),
            Self::Clc(p) => p.bytes(),
            Self::Domain(p) => p.bytes(),
            Self::Power(p) => p.bytes(),
            Self::Initial(p) => p.bytes(),
        }
    }
    fn queue(&self) -> Queue {
        match self {
            Self::Boot(p) => p.queue,
            Self::Domain(_) | Self::Power(_) | Self::Initial(_) => Queue::Wm,
            Self::Unified(_) | Self::Capabilities(_) | Self::Encapsulation(_) | Self::Clc(_) => {
                Queue::Wm
            }
        }
    }
    fn expects_response(&self) -> bool {
        match self {
            Self::Boot(p) => p.expects_response(),
            Self::Unified(_) | Self::Capabilities(_) | Self::Encapsulation(_) => true,
            Self::Clc(p) => p.completion() != super::firmware::clc::Completion::OneWay,
            Self::Domain(p) => p.expects_response(),
            Self::Power(p) => p.expects_response(),
            Self::Initial(p) => p.expects_response(),
        }
    }
    fn result(&self, event: &Event<'_>) -> Option<u32> {
        match self {
            Self::Boot(p) => event.boot_result(p).ok(),
            Self::Unified(p) => event.uni_result(p).ok().map(|r| r.status),
            Self::Capabilities(p) => event.capability_payload(p).ok().map(|_| 0),
            Self::Encapsulation(p) => event.encapsulation_payload(p).ok().map(|_| 0),
            Self::Clc(p) => event.clc_payload(p).ok().map(|_| 0),
            Self::Domain(p) => event.domain_payload(p).ok().map(|_| 0),
            Self::Power(p) => event.power_payload(p).ok().map(|_| 0),
            Self::Initial(p) => event.bootstrap_result(p).ok().map(|s| s.unwrap_or(0)),
        }
    }
}
#[derive(Clone, Copy)]
enum ReplySlot {
    Boot(usize),
    Unified,
    Capabilities,
    Encapsulation,
    Clc,
    Regulatory,
    Initial,
}

pub struct Mailbox<W: QueueIo, F: QueueIo, R: QueueIo> {
    identity: Identity,
    owner: u64,
    wm: TxRing<W>,
    fwdl: TxRing<F>,
    rx: RxRing<R>,
    runtime_rx: Option<RxRing<R>>,
    assembly: Assembly,
    runtime_assembly: Assembly,
    runtime_next: bool,
    encoder: Encoder,
    clock: crate::Clock,
    cookie: u64,
    poisoned: bool,
    last_failure: Option<Failure>,
    events: Vec<Frame>,
    event_bytes: usize,
    /// Frames dropped as mt76 drops them: marked dropped by the hardware,
    /// or arriving while the held events were full.
    dropped_events: u64,
    replies: Vec<Reply>,
    interrupted_unified_reply: Option<UnifiedReply>,
    interrupted_capability_reply: Option<CapabilityReply>,
    interrupted_encapsulation_reply: Option<eeprom::Reply>,
    interrupted_clc_reply: Option<clc::Reply>,
    interrupted_regulatory_reply: Option<clc::Reply>,
    download_binding: Option<firmware::Binding>,
    source_restart_cookie: Option<u64>,
    interrupted_initial_reply: Option<init::Reply>,
}
impl<W: QueueIo, F: QueueIo, R: QueueIo> Mailbox<W, F, R> {
    /// # Safety
    /// All three queues must belong exclusively to this one
    /// observed identity/requester and the same coherent native DMA attachment.
    /// They must be the chip's WM/FWDL TX and WM event RX queues, configured by
    /// the owning controller with engines enabled, device IRQ sources masked,
    /// no competing command/NAPI/reset owners, and the calibrated Clock.
    /// Successful construction also requires a reset/fresh firmware
    /// command-sequencing epoch: every old command-processing path, old RX
    /// response, IRQ and deferred worker has drained, and no late response from
    /// a previous epoch can arrive. Empty TX queues alone cannot prove this.
    /// This constructor neither admits PCI ownership nor starts/resets hardware.
    pub unsafe fn from_rings(
        identity: Identity,
        wm: TxRing<W>,
        fwdl: TxRing<F>,
        rx: RxRing<R>,
        clock: crate::Clock,
    ) -> Result<Self, Cause> {
        if wm.pending() != 0 || fwdl.pending() != 0 {
            return Err(Cause::PendingTransmit);
        }
        if clock.ticks_per_microsecond == 0 {
            return Err(Cause::ClockUnavailable);
        }
        let owner = NEXT_OWNER.next().ok_or(Cause::OwnerExhausted)?;
        Ok(Self {
            identity,
            owner,
            wm,
            fwdl,
            rx,
            runtime_rx: None,
            assembly: Assembly::default(),
            runtime_assembly: Assembly::default(),
            runtime_next: false,
            encoder: Encoder::default(),
            clock,
            cookie: 0,
            poisoned: false,
            last_failure: None,
            events: Vec::new(),
            event_bytes: 0,
            dropped_events: 0,
            replies: Vec::new(),
            interrupted_unified_reply: None,
            interrupted_capability_reply: None,
            interrupted_encapsulation_reply: None,
            interrupted_clc_reply: None,
            interrupted_regulatory_reply: None,
            download_binding: None,
            source_restart_cookie: None,
            interrupted_initial_reply: None,
        })
    }
    /// # Safety
    /// All `from_rings` real requester/coherence/fresh sequencing conditions
    /// apply to every queue. `runtime_rx` is the same C2 requester's
    /// source post-download MCU/WA queue (WFDMA0+0x540), independently owned
    /// and posted before firmware can select it. No other consumer may poll,
    /// reset, release or assemble either event stream. This does not create a
    /// firmware epoch, install missing data queues or check a PCI claim.
    pub(super) unsafe fn from_rings_with_runtime_rx(
        identity: Identity,
        wm: TxRing<W>,
        fwdl: TxRing<F>,
        rx: RxRing<R>,
        runtime_rx: Option<RxRing<R>>,
        clock: crate::Clock,
    ) -> Result<Self, Cause> {
        if identity.chip == super::identity::Chip::Mt7925 && runtime_rx.is_some() {
            return Err(Cause::FirmwareTarget);
        }
        // SAFETY: covered by this function's own safety contract.
        let mut mailbox = unsafe { Self::from_rings(identity, wm, fwdl, rx, clock) }?;
        mailbox.runtime_rx = runtime_rx;
        Ok(mailbox)
    }
    pub(in crate::wifi::mediatek) fn receive_idle(&self) -> bool {
        self.assembly.is_idle() && self.runtime_assembly.is_idle() && self.events.is_empty()
    }
    pub fn last_failure(&self) -> Option<Failure> {
        self.last_failure
    }
    /// Unexpected/unsolicited frames are held under explicit resource bounds.
    /// The caller dispatches them before radio/MLME continuation; none is silently
    /// consumed as an acknowledgement or dropped from a successful boot result.
    pub fn events(&self) -> &[Frame] {
        &self.events
    }
    /// Transfer kept asynchronous/unmatched frames to the owning
    /// controller for bounded dispatch. Their descriptor/drop evidence remains
    /// attached; removing them does not repair or reopen a poisoned epoch.
    /// How many frames were dropped instead of held.
    pub fn dropped_events(&self) -> u64 {
        self.dropped_events
    }
    pub fn take_events(&mut self) -> Vec<Frame> {
        self.event_bytes = 0;
        core::mem::take(&mut self.events)
    }
    /// Vendor lifecycle polling while no command is active. Complete
    /// frames keep per-stream assembly and bounded asynchronous ownership;
    /// they are not automatically classified as command ACKs or data packets.
    pub(in crate::wifi::mediatek) fn poll_events(&mut self) -> Result<Vec<Frame>, Cause> {
        if self.poisoned {
            return Err(Cause::Poisoned);
        }
        let result = (|| {
            for _ in 0..MAX_EVENTS {
                let Some(frame) = self.next_frame().map_err(Cause::Ring)? else {
                    break;
                };
                let core = !frame.drop
                    && Event::parse(self.identity.chip, &frame.bytes).is_ok_and(|e| {
                        e.event_id == 0xf0 || (e.option & 2 != 0 && e.event_id == 0x0a)
                    });
                self.retain(frame);
                if core {
                    return Err(Cause::CoreDump);
                }
            }
            Ok(())
        })();
        if let Err(cause) = result {
            self.poisoned = true;
            self.download_binding = None;
            self.source_restart_cookie = None;
            return Err(cause);
        }
        Ok(self.take_events())
    }
    pub fn replies(&self) -> &[Reply] {
        &self.replies
    }
    /// If a UNI ACK arrived before a later uncertain DMA/error, keep
    /// it for diagnosis. It does not authorize retry or context installation.
    pub fn interrupted_unified_reply(&self) -> Option<&UnifiedReply> {
        self.interrupted_unified_reply.as_ref()
    }
    pub fn interrupted_capability_reply(&self) -> Option<&CapabilityReply> {
        self.interrupted_capability_reply.as_ref()
    }
    /// Capability response plus real TX completion, with owned complete
    /// observations. This is not a usable radio/board/regulatory receipt.
    pub fn query_capabilities(&mut self) -> Result<CapabilityReport, CapabilityQueryFailure> {
        self.query_capabilities_timed(PCI_TIMEOUT_US)
    }
    fn query_capabilities_timed(
        &mut self,
        timeout_us: u64,
    ) -> Result<CapabilityReport, CapabilityQueryFailure> {
        if self.poisoned {
            return Err(CapabilityQueryFailure::Transport(self.failure(
                Operation::Capabilities,
                0,
                false,
                Cause::Poisoned,
            )));
        }
        let packet = self.encoder.encode_capabilities(self.identity.chip);
        self.execute(&Packet::Capabilities(packet), timeout_us)
            .map_err(CapabilityQueryFailure::Transport)?;
        let reply = self
            .interrupted_capability_reply
            .take()
            .expect("actual completed capability reply");
        // execute matched this exact chip/sequence event and kept its real
        // frame; parsing creates owned facts before this temporary view ends.
        let event =
            Event::parse(self.identity.chip, &reply.frame.bytes).expect("matched capability event");
        let observed =
            match mcu::capabilities::Capabilities::parse(self.identity.chip, event.payload) {
                Ok(observed) => observed,
                Err(error) => return Err(CapabilityQueryFailure::Malformed { error, reply }),
            };
        Ok(CapabilityReport {
            identity: self.identity,
            owner: self.owner,
            reply,
            observed,
        })
    }
    pub fn is_poisoned(&self) -> bool {
        self.poisoned
    }
    fn failure(
        &mut self,
        command: Operation,
        sequence: u8,
        dma_released: bool,
        cause: Cause,
    ) -> TransportFailure {
        self.poisoned = true;
        self.download_binding = None;
        self.source_restart_cookie = None;
        if self.last_failure.is_none() {
            self.last_failure = Some(Failure {
                command,
                sequence,
                dma_released,
                cause,
            });
        }
        TransportFailure {
            kind: if cause == Cause::Timeout {
                FailureKind::Timeout
            } else {
                FailureKind::InvalidResponse
            },
            port: PortState::Uncertain,
        }
    }
    /// Hold an asynchronous frame for the controller. As mt76's receive path
    /// frees a frame it cannot take and goes on, a frame the hardware marked
    /// dropped, or one beyond the held bounds, is counted and dropped; the
    /// mailbox stays usable.
    fn retain(&mut self, frame: Frame) {
        if frame.drop
            || self.events.len() == MAX_EVENTS
            || self.event_bytes + frame.bytes.len() > MAX_EVENT_BYTES
        {
            self.dropped_events = self.dropped_events.saturating_add(1);
            return;
        }
        self.event_bytes += frame.bytes.len();
        self.events.push(frame);
    }
    fn next_frame(&mut self) -> Result<Option<Frame>, ring::Error> {
        if let Some(runtime) = self.runtime_rx.as_mut() {
            // Never concatenate fragments from two hardware queues. Alternate
            // complete-stream polls so a busy runtime queue cannot starve ROM
            // responses or vice versa across the firmware transition.
            for _ in 0..2 {
                let use_runtime = self.runtime_next;
                self.runtime_next = !self.runtime_next;
                let (queue, assembly) = if use_runtime {
                    (&mut *runtime, &mut self.runtime_assembly)
                } else {
                    (&mut self.rx, &mut self.assembly)
                };
                if let Some(fragment) = queue.poll()?
                    && let Some(frame) = assembly.push(fragment)?
                {
                    return Ok(Some(frame));
                }
            }
            return Ok(None);
        }
        let Some(fragment) = self.rx.poll()? else {
            return Ok(None);
        };
        self.assembly.push(fragment)
    }
    fn transfer(&mut self, command: Command, payload: &[u8]) -> Result<u32, TransportFailure> {
        self.transfer_timed(command, payload, PCI_TIMEOUT_US)
    }
    fn transfer_timed(
        &mut self,
        command: Command,
        payload: &[u8],
        timeout_us: u64,
    ) -> Result<u32, TransportFailure> {
        // A raw boot/download command may replace the firmware input.
        // Only the complete native wrapper may establish a new binding.
        self.download_binding = None;
        if self.poisoned {
            return Err(self.failure(Operation::Boot(command), 0, false, Cause::Poisoned));
        }
        let packet = self
            .encoder
            .encode(self.identity.chip, command, payload)
            .map_err(|_| TransportFailure {
                kind: FailureKind::InvalidResponse,
                port: PortState::Usable,
            })?;
        self.execute(&Packet::Boot(packet), timeout_us)
    }
    /// Execute a device/BSS/station UNI command using this existing
    /// owned WM/RX mailbox and sequence epoch. The body must be source-correct
    /// for the chip and kept caller generation. This does not infer
    /// a running radio, create a station or install a key from enqueue alone.
    /// Only a returned zero-status reply after RX and TX DMA completion
    /// can satisfy the command's firmware-acknowledgement dependency.
    pub fn unified(
        &mut self,
        command: mcu::runtime::Command,
        payload: &[u8],
    ) -> Result<UnifiedReply, TransportFailure> {
        self.unified_timed(command, payload, PCI_TIMEOUT_US)
    }
    fn unified_timed(
        &mut self,
        command: mcu::runtime::Command,
        payload: &[u8],
        timeout_us: u64,
    ) -> Result<UnifiedReply, TransportFailure> {
        if self.poisoned {
            return Err(self.failure(Operation::Unified(command), 0, false, Cause::Poisoned));
        }
        let packet = self
            .encoder
            .encode_uni(self.identity.chip, command, payload)
            .map_err(|_| TransportFailure {
                kind: FailureKind::InvalidResponse,
                port: PortState::Usable,
            })?;
        self.execute(&Packet::Unified(packet), timeout_us)?;
        Ok(self
            .interrupted_unified_reply
            .take()
            .expect("actual completed UNI reply"))
    }
    fn mark_dma_released(&mut self, slot: ReplySlot) {
        match slot {
            ReplySlot::Boot(i) => self.replies[i].dma_released = true,
            ReplySlot::Unified => {
                self.interrupted_unified_reply
                    .as_mut()
                    .expect("retained actual UNI reply")
                    .dma_released = true
            }
            ReplySlot::Capabilities => {
                self.interrupted_capability_reply
                    .as_mut()
                    .expect("retained actual capability reply")
                    .dma_released = true
            }
            ReplySlot::Encapsulation => {
                self.interrupted_encapsulation_reply
                    .as_mut()
                    .expect("retained actual EEPROM reply")
                    .dma_released = true
            }
            ReplySlot::Clc => {
                self.interrupted_clc_reply
                    .as_mut()
                    .expect("retained actual CLC reply")
                    .dma_released = true
            }
            ReplySlot::Initial => {
                self.interrupted_initial_reply
                    .as_mut()
                    .expect("retained actual initial reply")
                    .dma_released = true;
            }
            ReplySlot::Regulatory => {
                self.interrupted_regulatory_reply
                    .as_mut()
                    .expect("retained actual regulatory reply")
                    .dma_released = true;
            }
        }
    }
    fn execute(&mut self, packet: &Packet, timeout_us: u64) -> Result<u32, TransportFailure> {
        self.source_restart_cookie = None;
        let command = packet.operation();
        let sequence = packet.sequence();
        let Some(cookie) = self.cookie.checked_add(1) else {
            return Err(self.failure(command, sequence, false, Cause::CookieMismatch));
        };
        self.cookie = cookie;
        let submit = match packet.queue() {
            Queue::Wm => self.wm.submit(cookie, packet.bytes()),
            Queue::FirmwareDownload => self.fwdl.submit(cookie, packet.bytes()),
        };
        submit.map_err(|e| self.failure(command, sequence, false, Cause::Ring(e)))?;
        let deadline = self.clock.deadline(timeout_us);
        let mut dma_released = false;
        let mut reply: Option<(u32, ReplySlot)> = None;
        loop {
            if !dma_released {
                let completion = match packet.queue() {
                    Queue::Wm => self.wm.poll(),
                    Queue::FirmwareDownload => self.fwdl.poll(),
                }
                .map_err(|e| self.failure(command, sequence, false, Cause::Ring(e)))?;
                if let Some(completion) = completion {
                    if completion.cookie != cookie || completion.bytes != packet.bytes().len() {
                        return Err(self.failure(command, sequence, false, Cause::CookieMismatch));
                    }
                    dma_released = true;
                    if let Some((_, index)) = reply {
                        self.mark_dma_released(index);
                    }
                }
            }
            if let Some(frame) = self
                .next_frame()
                .map_err(|e| self.failure(command, sequence, dma_released, Cause::Ring(e)))?
            {
                if frame.drop {
                    // mt76_dma_rx_process frees it and waits on: the reply
                    // may still come.
                    self.retain(frame);
                    if deadline.passed() {
                        return Err(self.failure(command, sequence, dma_released, Cause::Timeout));
                    }
                    continue;
                }
                let event = Event::parse(self.identity.chip, &frame.bytes);
                match event {
                    Ok(event)
                        if event.event_id == 0xf0
                            || (event.option & 2 != 0 && event.event_id == 0x0a) =>
                    {
                        self.retain(frame);
                        return Err(self.failure(command, sequence, dma_released, Cause::CoreDump));
                    }
                    Ok(event) => match packet.result(&event) {
                        Some(result) if reply.is_none() => {
                            if matches!(command, Operation::Boot(_)) && self.replies.len() == 1024 {
                                return Err(self.failure(
                                    command,
                                    sequence,
                                    dma_released,
                                    Cause::EventLimit,
                                ));
                            }
                            let index = match command {
                                Operation::Boot(command) => {
                                    let index = self.replies.len();
                                    self.replies.push(Reply {
                                        command,
                                        sequence,
                                        result,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Boot(index)
                                }
                                Operation::Unified(command) => {
                                    self.interrupted_unified_reply = Some(UnifiedReply {
                                        command,
                                        sequence,
                                        status: result,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Unified
                                }
                                Operation::Capabilities => {
                                    self.interrupted_capability_reply = Some(CapabilityReply {
                                        sequence,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Capabilities
                                }
                                Operation::Encapsulation => {
                                    self.interrupted_encapsulation_reply = Some(eeprom::Reply {
                                        sequence,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Encapsulation
                                }
                                Operation::Clc => {
                                    self.interrupted_clc_reply = Some(clc::Reply {
                                        sequence,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Clc
                                }
                                Operation::Initial(kind) => {
                                    self.interrupted_initial_reply = Some(init::Reply {
                                        kind,
                                        sequence,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Initial
                                }
                                Operation::Domain | Operation::Power(_) => {
                                    self.interrupted_regulatory_reply = Some(clc::Reply {
                                        sequence,
                                        dma_released,
                                        frame,
                                    });
                                    ReplySlot::Regulatory
                                }
                            };
                            reply = Some((result, index));
                        }
                        _ => self.retain(frame),
                    },
                    Err(_) => self.retain(frame),
                }
            }
            if dma_released && (!packet.expects_response() || reply.is_some()) {
                if let Some((result, index)) = reply {
                    self.mark_dma_released(index);
                    return Ok(result);
                }
                return Ok(0);
            }
            if deadline.passed() {
                return Err(self.failure(command, sequence, dma_released, Cause::Timeout));
            }
            core::hint::spin_loop();
        }
    }
    /// # Safety
    /// The controller has acknowledged WFDMA idle/reset and drained every
    /// submission/IRQ/worker, including timed-out command processing, before any
    /// queue mapping is removed. A software timeout is never this proof.
    pub unsafe fn release_after_quiesce(&mut self) -> Result<(), ring::Error> {
        self.poisoned = true;
        self.download_binding = None;
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            self.wm.release_after_quiesce()?;
            self.fwdl.release_after_quiesce()?;
            self.rx.release_after_quiesce()?
        };
        if let Some(runtime) = self.runtime_rx.as_mut() {
            // SAFETY: covered by this function's own safety contract.
            unsafe { runtime.release_after_quiesce()? };
        }
        Ok(())
    }
}
impl<W: QueueIo, F: QueueIo, R: QueueIo> Transport for Mailbox<W, F, R> {
    fn validate_firmware(&self, prepared: &Prepared<'_>) -> Result<(), TransportFailure> {
        if *prepared.requests() != Requests::new(&self.identity) {
            return Err(TransportFailure {
                kind: FailureKind::FirmwareTarget,
                port: PortState::Usable,
            });
        }
        Ok(())
    }
    fn command(&mut self, command: u8, payload: &[u8]) -> Result<u32, TransportFailure> {
        let command = Command::try_from(command).map_err(|_| TransportFailure {
            kind: FailureKind::InvalidResponse,
            port: PortState::Usable,
        })?;
        if command == Command::Scatter {
            return Err(TransportFailure {
                kind: FailureKind::InvalidResponse,
                port: PortState::Usable,
            });
        }
        self.transfer(command, payload)
    }
    fn scatter(&mut self, payload: &[u8]) -> Result<(), TransportFailure> {
        self.transfer(Command::Scatter, payload).map(|_| ())
    }
}

#[cfg(test)]
pub(crate) mod tests;
