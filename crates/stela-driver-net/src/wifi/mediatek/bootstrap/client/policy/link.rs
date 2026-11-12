// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2020-2023 MediaTek Inc.
//! An owning Ethernet link over the already constructed native policy client.
//! Discovery uses the policy client's programmed bands, including
//! active/passive 6GHz scans. The data profile remains ordinary single-link
//! 2.4/5GHz WPA2 with CCMP or GCMP, with or without protected management
//! frames. This creates no PCI/requester/coherency, supplicant,
//! regulatory or physical-radio grant.
//! The platform still supplies the controller and independent drain fences.

use super::*;
use crate::{
    Link, LinkError, Mac,
    wifi::mediatek::{
        bootstrap,
        control::{self, Control},
        mailbox::key,
    },
};
use alloc::{collections::VecDeque, format, vec::IntoIter};

const DATA_FRAMES: usize = 64;
/// mac80211 STA_MAX_TX_BUFFER: frames a TID holds while its block-ack
/// session starts.
const HELD_FRAMES: usize = 64;
mod external;
mod management;

const NOTICES: usize = 64;

#[derive(Debug)]
pub enum LinkControlError {
    Policy(PolicyError),
    WrongPhase,
    UnsupportedProfile,
    NoMemory,
    Backpressure,
}
impl From<PolicyError> for LinkControlError {
    fn from(error: PolicyError) -> Self {
        Self::Policy(error)
    }
}
impl core::fmt::Display for LinkControlError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MT7925 managed link: {self:?}")
    }
}
impl core::error::Error for LinkControlError {}

/// Construction failure returns both owners without changing hardware.
pub struct LinkAttachError<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> {
    pub error: LinkControlError,
    pub client: ManagedClient<I, Q, C>,
    pub source: S,
}
impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> core::fmt::Debug
    for LinkAttachError<I, Q, C, S>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(&self.error, f)
    }
}

pub enum LinkEvent<C: 'static> {
    /// Original owned controller/control-port event. Ingress::ControlPort must
    /// reach the real supplicant; classification is no handshake/MIC proof.
    Controller(Event),
    /// Both real descriptor consumption and exact-once TX-free completed.
    /// This is resource retirement, never a physical air ACK.
    Released(Released<PacketOwner<C>>),
}
pub struct LinkNotice<C: 'static> {
    /// Original client association when this native poll began. This is useful
    /// for routing; no copy authorizes a delayed association/key operation.
    pub association: Option<u64>,
    pub event: LinkEvent<C>,
}
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LinkProgress {
    pub data_frames: usize,
    pub notices: usize,
    /// A destination is full, or a returned native batch awaits room.
    pub backpressured: bool,
}
struct QueuedData {
    association: u64,
    bytes: Vec<u8>,
}
struct HeldData {
    tid: u8,
    bytes: Vec<u8>,
}
struct PendingEvents {
    association: Option<u64>,
    events: IntoIter<Event>,
}
struct State<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> {
    client: ManagedClient<I, Q, C>,
    source: S,
    mac: Mac,
    data: VecDeque<QueuedData>,
    /// Outgoing frames held while their TID's block-ack session starts.
    held: VecDeque<HeldData>,
    notices: VecDeque<LinkNotice<C>>,
    pending: Option<PendingEvents>,
    released: Option<IntoIter<Released<PacketOwner<C>>>>,
    epoch: Option<u64>,
    /// Receive frames refused and dropped, as mac80211 counts what
    /// `ieee80211_rx_handlers` drops.
    rejected_frames: u64,
    /// The function raises a vector the caller has routed.
    vector: bool,
    /// The device may raise it now.
    armed: bool,
}
/// How long a link that signals sleeps at most: Linux's timers are the
/// jiffies behind each of these passes.
const TIMER_TICK_US: u64 = 100_000;

/// Owns the exact existing native client and allocation provider. It has no raw
/// mutable-client escape: controls, timers, packets and revocation serialize on
/// this owner. Call service regularly even while there is no Ethernet traffic.
/// Live/uncertain Drop keeps this whole object, including pending frontend
/// events and provider, until platform quiescence can be established.
pub struct ManagedLink<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> {
    owned: Option<Box<State<I, Q, C, S>>>,
}
impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> ManagedLink<I, Q, C, S> {
    // Returning the owner/provider also on allocation failure avoids a
    // second allocation solely to report failure while preserving ownership.
    #[allow(
        clippy::result_large_err,
        reason = "Link admission failure returns the managed client and its original page source without dropping uncertain live native owners"
    )]
    pub fn new(
        client: ManagedClient<I, Q, C>,
        source: S,
    ) -> Result<Self, LinkAttachError<I, Q, C, S>> {
        // Begin before any timed scan/join, so all frontend deadlines use the
        // already calibrated controller Clock's single monotonic time domain.
        if client.phase() != Phase::Idle
            || client.revoked
            || client.client.state().controller.phase() != ControllerPhase::StartFacts
        {
            return Err(LinkAttachError {
                error: LinkControlError::WrongPhase,
                client,
                source,
            });
        }
        let Some(interface) = client.client.state().interface.registered_context() else {
            return Err(LinkAttachError {
                error: LinkControlError::WrongPhase,
                client,
                source,
            });
        };
        let mac = interface.address;
        let mut data = VecDeque::new();
        let mut notices = VecDeque::new();
        if data.try_reserve_exact(DATA_FRAMES).is_err()
            || notices.try_reserve_exact(NOTICES).is_err()
        {
            return Err(LinkAttachError {
                error: LinkControlError::NoMemory,
                client,
                source,
            });
        }
        Ok(Self {
            owned: Some(Box::new(State {
                client,
                source,
                mac,
                data,
                held: VecDeque::new(),
                notices,
                pending: None,
                released: None,
                epoch: None,
                rejected_frames: 0,
                vector: false,
                armed: false,
            })),
        })
    }
    fn state(&self) -> &State<I, Q, C, S> {
        self.owned.as_deref().expect("owned managed link")
    }
    fn state_mut(&mut self) -> &mut State<I, Q, C, S> {
        self.owned.as_deref_mut().expect("owned managed link")
    }
    fn now(&self) -> u64 {
        self.state()
            .client
            .client
            .state()
            .controller
            .state()
            .control
            .clock
            .now_us()
    }
    fn active(s: &State<I, Q, C, S>) -> bool {
        let c = s.client.client.state();
        !s.client.revoked
            && c.phase == Phase::Associated
            && c.controller.phase() == ControllerPhase::StartFacts
            && c.controller
                .state()
                .data_tx
                .as_ref()
                .is_some_and(|queue| !queue.is_poisoned())
            && c.profile.is_some()
    }
    /// An association whose management state may change: a PMF one once
    /// the OS has installed its handshake's pairwise key.
    fn ready(s: &State<I, Q, C, S>) -> bool {
        Self::active(s) && s.client.management_ready()
    }
    fn up(s: &State<I, Q, C, S>) -> bool {
        let c = s.client.client.state();
        Self::active(s) && c.station.port_open(&Client::key_slots(c))
    }
    fn reconcile(s: &mut State<I, Q, C, S>) {
        let epoch = s.client.station().association_epoch();
        if epoch != s.epoch || !Self::up(s) {
            s.data.clear();
            s.held.clear();
        }
        s.epoch = epoch;
    }
    fn progress(s: &State<I, Q, C, S>) -> LinkProgress {
        LinkProgress {
            data_frames: s.data.len(),
            notices: s.notices.len(),
            backpressured: s.pending.is_some()
                || s.released.is_some()
                || s.data.len() == DATA_FRAMES
                || s.notices.len() == NOTICES,
        }
    }
    fn drain(s: &mut State<I, Q, C, S>) {
        while let Some(pending) = s.pending.as_mut() {
            let Some(next) = pending.events.as_slice().first() else {
                s.pending = None;
                continue;
            };
            let data = matches!(next, Event::Ingress(Ingress::Data(_)));
            if (data && s.data.len() == DATA_FRAMES) || (!data && s.notices.len() == NOTICES) {
                break;
            }
            let epoch = pending.association;
            let event = pending.events.next().unwrap();
            if let Event::Ingress(Ingress::Data(bytes)) = event {
                // Linux Link semantics discard held data after carrier loss.
                // Control-port/controller events always keep their own bytes.
                if Self::up(s)
                    && epoch == s.client.station().association_epoch()
                    && let Some(association) = epoch
                {
                    s.data.push_back(QueuedData { association, bytes });
                }
            } else {
                s.notices.push_back(LinkNotice {
                    association: epoch,
                    event: LinkEvent::Controller(event),
                });
            }
        }
        while s.notices.len() < NOTICES {
            let Some(released) = s.released.as_mut() else {
                break;
            };
            let Some(packet) = released.next() else {
                s.released = None;
                break;
            };
            s.notices.push_back(LinkNotice {
                association: Some(packet.context.association_epoch()),
                event: LinkEvent::Released(packet),
            });
        }
    }
    pub fn phase(&self) -> Phase {
        self.state().client.phase()
    }
    /// Whether the adapter is gone for good: the client or its controller
    /// quarantined the hardware, or it was stopped.
    pub fn lost(&self) -> bool {
        let client = &self.state().client;
        matches!(client.phase(), Phase::Quarantined | Phase::Stopped)
            || client.client.state().controller.phase() == ControllerPhase::Quarantined
    }
    pub fn station(&self) -> &Station {
        self.state().client.station()
    }
    pub fn country(&self) -> [u8; 2] {
        self.state().client.country()
    }
    /// The verified domain's channels on `band` that may transmit, with
    /// their cfg80211 flags.
    pub fn channels(&self, band: Band) -> Vec<(wifi::Channel, u32)> {
        self.state().client.channels(band)
    }
    /// Whether a scan may probe on `channel`; elsewhere it listens.
    pub fn probes(&self, channel: wifi::Channel) -> bool {
        self.state().client.probes(channel)
    }
    pub fn local_advertisement(&self, band: Band) -> Result<Advertisement, LinkControlError> {
        self.state()
            .client
            .local_advertisement(band)
            .map_err(|e| LinkControlError::Policy(PolicyError::Client(Error::Local(e))))
    }
    pub fn bss(&self) -> impl Iterator<Item = (BssHandle, &scan::result::Bss)> {
        self.state().client.bss()
    }
    /// The client's own record of the beacon of the BSS `handle` names, as
    /// another link of an AP MLD association takes it.
    pub fn observation(
        &self,
        handle: BssHandle,
    ) -> Result<Arc<super::PmfBssObservation>, LinkControlError> {
        Ok(self.state().client.observation(handle)?)
    }
    /// The 6 GHz part of a split scan (WIPHY_FLAG_SPLIT_SCAN_6GHZ), planned
    /// from the BSSes this link knows as `cfg80211_scan_6ghz` plans from the
    /// BSS list; `scan::six_ghz::first_part` is the scan before it.
    pub fn six_ghz_scan(
        &self,
        request: &scan::Request,
        options: scan::six_ghz::Options,
    ) -> Result<Option<scan::Request>, LinkControlError> {
        scan::six_ghz::second_part(request, options, self.bss().map(|(_, bss)| bss))
            .map_err(|e| LinkControlError::Policy(PolicyError::Client(Error::Scan(e))))
    }
    pub fn pending_edca(&self) -> bool {
        self.state().client.pending_edca()
    }
    pub fn applied_edca(&self) -> Option<&AppliedEdca> {
        self.state().client.applied_edca()
    }
    /// Apply only a native observation. Frontend notice batches remain
    /// owned and normal service backpressure is unchanged.
    pub fn update_edca(&mut self) -> Result<(), LinkControlError> {
        let s = self.state_mut();
        let result = s.client.update_edca();
        Self::reconcile(s);
        Ok(result?)
    }
    /// Sleep on the function's vector once the port is open.
    pub fn with_vector(mut self) -> Self {
        self.state_mut().vector = true;
        self
    }
    /// The OS's power save setting, on by default as Linux's
    /// CONFIG_CFG80211_DEFAULT_PS; it applies while the port is open.
    pub fn set_power_save(&mut self, enabled: bool) {
        self.state_mut().client.set_power_save(enabled);
    }
    /// Offer the OS action frames whose body starts with `prefix`, as
    /// `NL80211_CMD_REGISTER_FRAME` does; they arrive as controller notices,
    /// raw or authenticated, instead of going back to their sender.
    pub fn register_action(&mut self, prefix: &[u8]) -> Result<(), LinkControlError> {
        let result = self.state_mut().client.client.register_action(prefix);
        Ok(result.map_err(PolicyError::Client)?)
    }
    /// Withdraw a `register_action` registration.
    pub fn unregister_action(&mut self, prefix: &[u8]) {
        self.state_mut().client.client.unregister_action(prefix);
    }
    /// Set or clear the OS's interworking QoS Map, which classifies data
    /// frames' DSCP as `cfg80211_classify8021d` does.
    pub fn set_qos_map(&mut self, map: Option<&[u8]>) -> Result<(), LinkControlError> {
        Ok(self.state_mut().client.set_qos_map(map)?)
    }
    /// Set or stop the firmware RSSI monitor; crossings arrive as
    /// `Event::Rssi` notices.
    pub fn set_rssi_monitor(
        &mut self,
        monitor: Option<(i32, u32)>,
    ) -> Result<(), LinkControlError> {
        let s = self.state_mut();
        let result = s.client.set_rssi_monitor(monitor);
        Self::reconcile(s);
        Ok(result?)
    }
    pub fn take_notice(&mut self) -> Option<LinkNotice<C>> {
        self.state_mut().notices.pop_front()
    }
    pub fn queues(&self) -> LinkProgress {
        Self::progress(self.state())
    }
    /// Process one native RX batch, real TX retirement and station timers. Full
    /// frontend queues keep their batch and apply backpressure without making
    /// an invented completion or discarding control-port/event bytes.
    pub fn service(&mut self) -> Result<LinkProgress, LinkControlError> {
        Self::capture(self.state_mut())?;
        let result = self.service_inner();
        let rearmed = Self::rearm(self.state_mut());
        result.and_then(|progress| rearmed.map(|()| progress))
    }
    /// The vector stays quiet through a pass that follows it.
    fn capture(s: &mut State<I, Q, C, S>) -> Result<(), LinkControlError> {
        if !s.armed {
            return Ok(());
        }
        let captured = Self::control(s).capture_interrupts();
        s.armed = captured.is_ok();
        captured.map_err(Self::control_failed)
    }
    /// A pass over, the vector ends the next sleep; with the port shut the
    /// link polls again.
    fn rearm(s: &mut State<I, Q, C, S>) -> Result<(), LinkControlError> {
        let open = s.vector && Self::up(s);
        let armed = core::mem::replace(&mut s.armed, open);
        let control = Self::control(s);
        match (armed, open) {
            (false, false) => Ok(()),
            (false, true) => control.open_interrupts(),
            (true, true) => control.unmask_interrupts(),
            (true, false) => control.mask_interrupts(),
        }
        .map_err(Self::control_failed)
    }
    fn control(s: &mut State<I, Q, C, S>) -> &mut Control<I> {
        &mut s.client.client.state_mut().controller.state_mut().control
    }
    fn control_failed(error: control::Error) -> LinkControlError {
        PolicyError::Client(Error::Bootstrap(bootstrap::Error::Control(error))).into()
    }
    fn service_inner(&mut self) -> Result<LinkProgress, LinkControlError> {
        let now = self.now();
        let s = self.state_mut();
        Self::reconcile(s);
        Self::drain(s);
        let mut failure = None;
        if s.pending.is_none()
            && s.notices.len() < NOTICES
            && matches!(
                s.client.phase(),
                Phase::Scanning
                    | Phase::ScanCompleted
                    | Phase::Joining
                    | Phase::AwaitingModernAdmission
                    | Phase::Associated
                    | Phase::Removing
                    | Phase::NeedsDrain
            )
        {
            let epoch = s.client.station().association_epoch();
            match s
                .client
                .poll(now, &s.source)
                .and_then(|events| Self::protected_block_ack(s, events, now))
            {
                Ok(events) if !events.is_empty() => {
                    s.pending = Some(PendingEvents {
                        association: epoch,
                        events: events.into_iter(),
                    });
                }
                Ok(_) => {}
                // ieee80211_rx_handlers drops a frame it refuses (a replay,
                // another key or BSS, a malformed or unexpected frame) and
                // the link goes on; the frames before it were kept.
                Err(PolicyError::Client(
                    Error::WrongProfile
                    | Error::Rx(_)
                    | Error::Frame(_)
                    | Error::Data(_)
                    | Error::Aggregate(_)
                    | Error::IncompleteParent
                    | Error::Key(key::Error::Bip(key::bip::Error::Authentication(_))),
                )) => s.rejected_frames = s.rejected_frames.saturating_add(1),
                Err(error) => failure = Some(error),
            }
        }
        // A rejected receive frame must not starve timeouts or completed
        // packet retirement. Preserve the first error; any native stop/failure
        // still updates its original owner before carrier/queues are rechecked.
        let timer_now = s
            .client
            .client
            .state()
            .controller
            .state()
            .control
            .clock
            .now_us();
        if matches!(s.client.phase(), Phase::Joining | Phase::Associated)
            && let Err(error) = s.client.poll_station(timer_now, &s.source)
        {
            failure.get_or_insert(error);
        }
        if s.released.is_none()
            && !matches!(
                s.client.phase(),
                Phase::Stopped | Phase::ControllerDrain | Phase::Quarantined
            )
        {
            match s.client.poll_released() {
                Ok(released) if !released.is_empty() => {
                    Self::check_aggr(s, &released, timer_now);
                    s.released = Some(released.into_iter());
                }
                Ok(_) => {}
                Err(error) => {
                    failure.get_or_insert(error);
                }
            }
        }
        // mac80211 hands a beacon's new WMM parameters and slot time
        // straight to the driver.
        if Self::ready(s) && s.client.pending_edca() {
            match s.client.update_edca() {
                Ok(()) | Err(PolicyError::Client(Error::NeedsDrain)) => {}
                Err(error) => {
                    failure.get_or_insert(error);
                }
            }
        }
        if Self::ready(s) {
            match s.client.channel_switch(timer_now, &s.source) {
                Ok(()) | Err(PolicyError::Client(Error::NeedsDrain)) => {}
                Err(error) => {
                    failure.get_or_insert(error);
                }
            }
        }
        if Self::ready(s) && s.client.client.pending_channel().is_some() {
            match s.client.update_channel(&s.source) {
                Ok(()) | Err(PolicyError::Client(Error::NeedsDrain)) => {}
                Err(error) => {
                    failure.get_or_insert(error);
                }
            }
        }
        if Self::ready(s) && s.client.client.pending_slot_time() {
            match s.client.update_slot_time() {
                Ok(()) | Err(PolicyError::Client(Error::NeedsDrain)) => {}
                Err(error) => {
                    failure.get_or_insert(error);
                }
            }
        }
        // ieee80211_recalc_ps_vif: power save while the authorized port is
        // up and the OS allows it.
        if Self::ready(s) {
            let enable = Self::up(s) && s.client.power_save_allowed();
            if enable != s.client.client.power_save() {
                match s.client.update_power_save(enable) {
                    Ok(()) | Err(PolicyError::Client(Error::NeedsDrain)) => {}
                    Err(error) => {
                        failure.get_or_insert(error);
                    }
                }
            }
        }
        if let Err(error) = Self::block_ack(s, timer_now) {
            failure.get_or_insert(error);
        }
        Self::reconcile(s);
        Self::drain(s);
        failure.map_or_else(|| Ok(Self::progress(s)), |error| Err(error.into()))
    }
    /// mac80211 runs a PMF association's decrypted block-ack actions itself
    /// (`ieee80211_process_addba_request` and the rest), dropping one it
    /// cannot take as it drops a malformed or excess frame.
    fn protected_block_ack(
        s: &mut State<I, Q, C, S>,
        events: Vec<Event>,
        now: u64,
    ) -> Result<Vec<Event>, PolicyError> {
        if !s.client.protected() {
            return Ok(events);
        }
        let mut out = Vec::new();
        out.try_reserve_exact(events.len())
            .map_err(|_| PolicyError::NoMemory)?;
        for event in events {
            match event {
                Event::ProtectedManagement(rx) if !rx.group() && rx.body().first() == Some(&3) => {
                    let mut received = Some(rx);
                    // SAFETY: the link is the association's management
                    // policy, as mac80211 is for its station.
                    match unsafe { s.client.receive_management_block_ack(&mut received, now) } {
                        Ok(())
                        | Err(PolicyError::Client(
                            Error::Busy | Error::BlockAck(_) | Error::Key(_),
                        )) => {}
                        Err(error) => return Err(error),
                    }
                }
                event => out.push(event),
            }
        }
        Ok(out)
    }

    /// Linux `mt7925_tx_check_aggr`, run as TX-free retires each packet: a
    /// QoS data frame to an HT or HE peer starts its TID's TX block-ack
    /// session unless one exists. A failed start, like a failed
    /// `ieee80211_start_tx_ba_session`, leaves the TID to retry on a later
    /// TX-free; the station limits retries as mac80211 does. A PMF
    /// association's sessions start with protected actions; an OS management
    /// session without PMF starts its own.
    fn check_aggr(s: &mut State<I, Q, C, S>, released: &[Released<PacketOwner<C>>], now: u64) {
        let epoch = s.client.station().association_epoch();
        let Some(context) = s.client.client.modern_context() else {
            return;
        };
        let negotiated = context.negotiated();
        if negotiated.ht.is_none() && negotiated.he.is_none() {
            return;
        }
        for packet in released {
            let owner = &packet.context;
            if let Some(tid) = owner.tid()
                && Some(owner.association_epoch()) == epoch
                && !owner.control_port()
            {
                if !s.client.protected() {
                    let _ = s.client.client.begin_tx_block_ack(tid, 0, now);
                } else if s.client.management_ready()
                    && let Some(session) = s.client.client.state().management.clone()
                {
                    // SAFETY: as for protected_block_ack.
                    let _ = unsafe { s.client.begin_management_block_ack(&session, tid, 0, now) };
                }
            }
        }
    }

    /// mac80211's block-ack work: send queued ADDBA/DELBA/BAR operations and
    /// expired session timers, then release frames held for a TID whose
    /// session has started or fallen back (`ieee80211_agg_splice_finish`).
    fn block_ack(s: &mut State<I, Q, C, S>, now: u64) -> Result<(), PolicyError> {
        if !Self::ready(s) {
            return Ok(());
        }
        if s.client
            .next_transmit_deadline()
            .is_some_and(|deadline| deadline <= now)
        {
            s.client.poll_block_ack_timers(now)?;
        }
        // Transmission waits while the AP asked for silence.
        if s.client.client.channel_switch_blocks_tx() {
            return Ok(());
        }
        if s.client.client.pending_block_ack() && !Self::progress(s).backpressured {
            match s.client.progress_block_ack(&s.source, now) {
                Ok(()) | Err(PolicyError::Client(Error::NeedsDrain | Error::Busy)) => {}
                Err(error) => return Err(error),
            }
        }
        if s.client.client.pending_action_reply() && !Self::progress(s).backpressured {
            match s.client.progress_action_reply(&s.source) {
                Ok(()) | Err(PolicyError::Client(Error::NeedsDrain | Error::Busy)) => {}
                Err(error) => return Err(error),
            }
        }
        let Some(epoch) = s.client.station().association_epoch() else {
            return Ok(());
        };
        let mut at = 0;
        while at < s.held.len() {
            let tid = s.held[at].tid;
            if Client::tx_ba_blocks_data(s.client.client.state(), tid) {
                at += 1;
                continue;
            }
            let frame = s.held.remove(at).unwrap();
            s.client
                .transmit(epoch, &frame.bytes, TxClass::Data, Some(tid), &s.source)?;
        }
        Ok(())
    }

    /// `ieee80211_tx_prep_agg`: holds a frame for a TID whose block-ack
    /// session is starting, dropping that TID's oldest beyond the limit.
    fn hold(s: &mut State<I, Q, C, S>, tid: u8, frame: &[u8]) -> Result<(), LinkError> {
        let mut bytes = Vec::new();
        if bytes.try_reserve_exact(frame.len()).is_err() || s.held.try_reserve(1).is_err() {
            return Err(LinkError::Adapter("MT7925 managed link: no memory".into()));
        }
        bytes.extend_from_slice(frame);
        if s.held.iter().filter(|held| held.tid == tid).count() == HELD_FRAMES {
            let oldest = s.held.iter().position(|held| held.tid == tid).unwrap();
            s.held.remove(oldest);
        }
        s.held.push_back(HeldData { tid, bytes });
        Ok(())
    }

    /// The receive frames refused and dropped so far.
    pub fn rejected_frames(&self) -> u64 {
        self.state().rejected_frames
    }
    pub fn begin_scan(
        &mut self,
        request: scan::Request,
        budget_us: u64,
    ) -> Result<scan::Ticket, LinkControlError> {
        let now = self.now();
        Ok(self
            .state_mut()
            .client
            .begin_scan(request, now, budget_us)?)
    }
    pub fn cancel_scan(&mut self) -> Result<(), LinkControlError> {
        Ok(self.state_mut().client.cancel_scan()?)
    }
    /// # Safety
    /// Radio restoration and all DMA/IRQ/late users satisfy the original
    /// native client's independent scan drain contract.
    pub unsafe fn retire_scan_after_drain(&mut self) -> Result<scan::Outcome, LinkControlError> {
        // SAFETY: inherited unchanged real native scan drain contract.
        Ok(unsafe { self.state_mut().client.retire_scan_after_drain() }?)
    }
    pub fn join(
        &mut self,
        handle: BssHandle,
        join: Join,
        profile: Profile,
    ) -> Result<(), LinkControlError> {
        // Every pairwise and group cipher an RSN request parses to is one
        // the client installs, as mt7925_set_key takes CCMP and GCMP. An open
        // network carries data once associated, as mac80211 authorizes it.
        if !matches!(join.channel.band, Band::Ghz2 | Band::Ghz5)
            || (join.policy == station::PortPolicy::Controlled
                && join.request.key_ciphers().is_none())
        {
            return Err(LinkControlError::UnsupportedProfile);
        }
        let now = self.now();
        let s = self.state_mut();
        let result = s.client.join_modern(handle, join, profile, now, &s.source);
        Self::reconcile(s);
        Ok(result?)
    }
    pub fn configure(&mut self) -> Result<(), LinkControlError> {
        let now = self.now();
        Ok(self.state_mut().client.configure_modern(now)?)
    }
    /// The real OS supplicant supplies the authenticated material and RSC.
    pub fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, LinkControlError> {
        let s = self.state_mut();
        let result = s.client.install_key(pairwise, index, material, seed);
        Self::reconcile(s);
        Ok(result?)
    }
    /// The GTK of the AP of the association's link `link_id`, which the real
    /// OS supplicant takes from the handshake's MLO GTK KDE.
    pub fn install_link_key(
        &mut self,
        link_id: u8,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, LinkControlError> {
        let s = self.state_mut();
        let result = s.client.install_link_key(link_id, index, material, seed);
        Self::reconcile(s);
        Ok(result?)
    }
    /// The OS supplies this only after its authenticated handshake.
    pub fn authorize_port(&mut self, epoch: u64) -> Result<(), LinkControlError> {
        Ok(self.state_mut().client.authorize_port(epoch)?)
    }
    pub fn send_control(
        &mut self,
        ethernet: &[u8],
        key: Option<KeyIdentity>,
    ) -> Result<Receipt, LinkControlError> {
        self.service_inner()?;
        let s = self.state_mut();
        // EAPOL may precede controlled-port/data carrier, but it still needs
        // the current association and a live native controller.
        if !Self::active(s) {
            return Err(LinkControlError::WrongPhase);
        }
        if Self::progress(s).backpressured {
            return Err(LinkControlError::Backpressure);
        }
        let epoch = s
            .client
            .station()
            .association_epoch()
            .ok_or(LinkControlError::WrongPhase)?;
        let result = s.client.transmit(
            epoch,
            ethernet,
            key.map_or(TxClass::ControlPortPlaintext, TxClass::ControlPortProtected),
            None,
            &s.source,
        );
        Self::reconcile(s);
        Ok(result?)
    }
    /// The OS's deauthentication with WLAN_REASON_DEAUTH_LEAVING.
    pub fn disconnect(&mut self) -> Result<(), LinkControlError> {
        self.deauthenticate(3)
    }
    /// The OS's deauthentication with its reason, as `ieee80211_mgd_deauth`
    /// sends it.
    pub fn deauthenticate(&mut self, reason: u16) -> Result<(), LinkControlError> {
        let s = self.state_mut();
        let result = s.client.deauthenticate(reason, &s.source);
        Self::reconcile(s);
        Ok(result?)
    }
    pub fn revoke(&mut self) -> Result<(), LinkControlError> {
        let s = self.state_mut();
        let result = s.client.revoke();
        Self::reconcile(s);
        Ok(result?)
    }
    /// # Safety
    /// Native peer/key removal and all old radio/DMA/IRQ/work users meet
    /// the original independent peer fence; local queue emptiness is no fence.
    pub unsafe fn retire_peer_after_drain(&mut self) -> Result<(), LinkControlError> {
        let s = self.state_mut();
        // SAFETY: inherited unchanged real native peer drain contract.
        let result = unsafe { s.client.retire_peer_after_drain() };
        Self::reconcile(s);
        Ok(result?)
    }
    pub fn stop(&mut self) -> Result<(), LinkControlError> {
        let s = self.state_mut();
        let result = s.client.stop();
        Self::reconcile(s);
        Ok(result?)
    }
    /// # Safety
    /// Exact whole-controller/radio/DMA/IOTLB/IRQ/work quiescence meets the
    /// original native platform fence, independently of stop/idle observations.
    pub unsafe fn stop_after_quiesce(&mut self) -> Result<(), LinkControlError> {
        // SAFETY: inherited unchanged native controller fence.
        Ok(unsafe { self.state_mut().client.stop_after_quiesce() }?)
    }
}
impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> Link for ManagedLink<I, Q, C, S> {
    fn mac(&self) -> Mac {
        self.state().mac
    }
    fn carrier(&mut self) -> bool {
        Self::up(self.state())
    }
    /// Once its port is open; the handshake polls, as on the other radios.
    fn signals(&self) -> bool {
        let s = self.state();
        s.vector && Self::up(s)
    }
    /// The timers no signal ends, or at once while a pass found work, as NAPI
    /// polls until a poll comes back short.
    fn due_us(&self) -> Option<u64> {
        let s = self.state();
        self.signals().then(|| {
            let idle = s.client.client.rings_empty()
                && s.pending.is_none()
                && s.released.is_none()
                && s.notices.is_empty();
            let due = self
                .now()
                .saturating_add(if idle { TIMER_TICK_US } else { 0 });
            s.client
                .client
                .state()
                .link_removal
                .map_or(due, |deadline| due.min(deadline))
        })
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        crate::check_frame(frame)?;
        self.service_inner()
            .map_err(|e| LinkError::Adapter(format!("{e}")))?;
        let s = self.state_mut();
        if !Self::up(s) {
            return Err(LinkError::NoCarrier);
        }
        // ieee80211_vif_block_queues_csa: the AP asked for silence.
        if Self::progress(s).backpressured || s.client.client.channel_switch_blocks_tx() {
            return Err(LinkError::Adapter(format!(
                "{}",
                LinkControlError::Backpressure
            )));
        }
        let epoch = s
            .client
            .station()
            .association_epoch()
            .ok_or(LinkError::NoCarrier)?;
        let tid = s.client.data_tid(frame);
        // Frames already held for the TID keep their order.
        if let Some(tid) = tid
            && (Client::tx_ba_blocks_data(s.client.client.state(), tid)
                || s.held.iter().any(|held| held.tid == tid))
        {
            return Self::hold(s, tid, frame);
        }
        let result = s
            .client
            .transmit(epoch, frame, TxClass::Data, tid, &s.source);
        Self::reconcile(s);
        result
            .map(|_| ())
            .map_err(|e| LinkError::Adapter(format!("MT7925 managed link: {e:?}")))
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.service_inner()
            .map_err(|e| LinkError::Adapter(format!("{e}")))?;
        let s = self.state_mut();
        if Self::up(s) {
            while let Some(packet) = s.data.pop_front() {
                if Some(packet.association) == s.client.station().association_epoch() {
                    return Ok(Some(packet.bytes));
                }
            }
        }
        Ok(None)
    }
}
impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> Drop for ManagedLink<I, Q, C, S> {
    fn drop(&mut self) {
        if let Some(state) = self.owned.take()
            && state.client.phase() != Phase::Stopped
        {
            core::mem::forget(state);
        }
    }
}

#[cfg(test)]
mod tests;
