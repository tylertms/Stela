// SPDX-License-Identifier: GPL-2.0-only
// Management state rules adapted from Linux net/mac80211/mlme.c and key.c.
// Copyright 2003-2008 Jouni Malinen; 2004 Instant802 Networks, Inc.;
// 2005 Devicescape Software, Inc.; 2006-2007 Jiri Benc; 2007 Michael Wu;
// 2013-2014 Intel Mobile Communications GmbH;
// 2015-2017 Intel Deutschland GmbH; 2018-2026 Intel Corporation.
//! Open-system authentication and association control, on one link or, after
//! SAE with an AP MLD, on the links a multi-link setup requests.
//!
//! The OS/frontend chooses the BSS and complete negotiated association IEs.
//! WPA/802.1X authentication remains with that OS service. This layer cannot
//! synthesize authorization: controlled-port opening requires an explicit
//! association-scoped frontend command and acknowledged PTK/GTK keys.
//! Vendor code owns DMA/radio resources and command acknowledgements.

use super::{
    Channel,
    frame::{DEAUTHENTICATION, DISASSOCIATION, Mpdu, QOS_TIDS, TYPE_SUBTYPE, next_sequence},
    security::{KeyIdentity, KeyScope, KeySlot},
};
use crate::Mac;
use alloc::{sync::Arc, vec::Vec};

mod aggregate;
mod context;
pub mod csa;
pub mod external;
pub mod monitor;
pub mod multi_link;
mod sequence;
pub use context::{AssociationContext, NativeBssObservation};
use multi_link::Affiliation;
pub mod ba;
pub mod data;
pub mod fragments;
pub mod operations;
pub mod phy;
pub mod power_envelope;
pub mod wire;

static NEXT_ASSOCIATION: super::Identities = super::Identities::new();
// Linux no-ACK-status auth/assoc response timing and retry count.
const MANAGEMENT_RESPONSE_TIMEOUT_US: u64 = 500_000;
const MANAGEMENT_ATTEMPTS: u8 = 3;
/// IEEE80211_AUTH_TIMEOUT_SHORT and IEEE80211_ASSOC_TIMEOUT_SHORT.
const MANAGEMENT_ACKED_TIMEOUT_US: u64 = 100_000;
const CONFIGURATION_TIMEOUT_US: u64 = 500_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortPolicy {
    Open,
    Controlled,
}

/// Regulatory authorization must come from the real OS/radio policy service.
/// The frame engine supplies no permissive default or country/DFS guesses.
pub trait RegulatoryPolicy {
    fn allow_management_tx(&mut self, channel: Channel) -> bool;
    fn allow_data_tx(&mut self, channel: Channel) -> bool;
}

#[derive(Clone, Debug)]
pub struct Join {
    pub bssid: Mac,
    pub channel: Channel,
    pub policy: PortPolicy,
    pub request: wire::AssociationRequest,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ticket {
    epoch: u64,
    serial: u32,
}

impl Ticket {
    pub const fn epoch(self) -> u64 {
        self.epoch
    }
    pub const fn serial(self) -> u32 {
        self.serial
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Idle,
    Authenticating,
    Associating,
    Configuring,
    Associated,
    Disconnecting,
    Quarantined,
    ExternalAuthenticating,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Completion {
    Accepted,
    RejectedAndIdle,
    Uncertain,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    Busy,
    InvalidAddress,
    Regulatory,
    ClockRegressed,
    Overflow,
    Stale,
    NotQueued,
    WrongChannel,
    Wire(wire::Error),
    Phy(phy::Error),
    Timeout,
    NotAssociated,
    WrongKey,
    Unsupported(&'static str),
}

impl From<wire::Error> for Error {
    fn from(error: wire::Error) -> Self {
        Self::Wire(error)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    PeerRejected(u16),
    PeerDeparture(u16),
    Timeout,
    Transmit,
    /// The AP MLD's answer broke the multi-link setup, which
    /// `ieee80211_rx_mgmt_assoc_resp` abandons.
    MultiLink,
}

#[derive(Clone, Debug)]
pub enum Action {
    /// Queue these real bytes on the specified permitted channel. Before
    /// calling `submitted`, the vendor must establish successful ownership
    /// transfer into its TX path. DMA completion is not an air ACK.
    Transmit {
        ticket: Ticket,
        channel: Channel,
        bytes: Vec<u8>,
    },
    /// Program firmware association context and match its real status.
    Configure {
        ticket: Ticket,
        bssid: Mac,
        channel: Channel,
        request: wire::AssociationRequest,
        response: wire::AssociationResponse,
        /// Kept modern configuration. Legacy actions carry None.
        context: Option<Arc<AssociationContext>>,
        /// The other links an AP MLD accepted, each its own configuration.
        links: Vec<Arc<AssociationContext>>,
    },
    /// Remove association/keys, mask/drain associated work, then acknowledge.
    Remove {
        ticket: Ticket,
        failure: Option<Failure>,
    },
}

#[derive(Clone, Copy, Debug)]
struct Pending {
    ticket: Ticket,
    queued: bool,
    deadline: u64,
    attempts: u8,
}

#[derive(Debug)]
pub struct Station {
    local: Mac,
    phase: Phase,
    join: Option<Join>,
    external: Option<Arc<external::Session>>,
    /// The external SAE exchange's progress.
    sae: external::Progress,
    selected_beacon: Option<super::scan::result::Bss>,
    context: Option<Arc<AssociationContext>>,
    /// The other links of a multi-link association.
    links: Vec<Arc<AssociationContext>>,
    /// Where a channel switch moved the association of this context ticket.
    switched: Option<(Ticket, operations::ChannelDefinition)>,
    pmf_observation: Option<Arc<NativeBssObservation>>,
    epoch: u64,
    serial: u32,
    sequence: u16,
    qos_sequence: [u16; QOS_TIDS],
    now: u64,
    pending: Option<Pending>,
    /// The authentication transaction `ieee80211_rx_mgmt_auth` expects next.
    auth_transaction: u16,
    /// The last WEP IV `ieee80211_wep_get_iv` handed out.
    wep_iv: u32,
    /// The keys an open controlled port admits: each link's PTK and GTK,
    /// in `on_air` order.
    authorization: Option<Vec<(KeyIdentity, KeyIdentity)>>,
    rx_ba: ba::Sessions,
    tx_ba: ba::tx::Sessions,
}

impl Station {
    pub fn new(local: Mac) -> Result<Self, Error> {
        if local.is_multicast() || local.0 == [0; 6] {
            return Err(Error::InvalidAddress);
        }
        Ok(Self {
            local,
            phase: Phase::Idle,
            join: None,
            external: None,
            sae: external::Progress::default(),
            selected_beacon: None,
            context: None,
            links: Vec::new(),
            switched: None,
            pmf_observation: None,
            epoch: 0,
            serial: 0,
            sequence: 0,
            qos_sequence: [0; 16],
            now: 0,
            pending: None,
            auth_transaction: 2,
            wep_iv: 0,
            authorization: None,
            rx_ba: ba::Sessions::new(),
            tx_ba: ba::tx::Sessions::new(),
        })
    }

    /// `ieee80211_send_smps_action`: an HT action frame telling `context`'s AP
    /// of `control`.
    pub fn spatial_power_save(
        &mut self,
        context: &AssociationContext,
        control: u8,
    ) -> Result<Vec<u8>, Error> {
        self.validate_association_context(context)?;
        if self.phase != Phase::Associated {
            return Err(Error::NotAssociated);
        }
        let ap = context.bssid();
        let mut frame = alloc::vec![0xd0, 0, 0, 0];
        frame.extend_from_slice(&ap.0);
        frame.extend_from_slice(&self.local.0);
        frame.extend_from_slice(&ap.0);
        frame.extend_from_slice(&(self.sequence << 4).to_le_bytes());
        frame.extend_from_slice(&[7, 1, control]);
        self.sequence = next_sequence(self.sequence);
        Ok(frame)
    }

    pub const fn phase(&self) -> Phase {
        self.phase
    }
    pub const fn association_epoch(&self) -> Option<u64> {
        if matches!(self.phase, Phase::Idle) {
            None
        } else {
            Some(self.epoch)
        }
    }

    /// The station's address on the link its management frames use: the
    /// setup link of a multi-link request, or the interface's own.
    fn link_address(&self) -> Mac {
        self.join
            .as_ref()
            .and_then(|join| join.request.multi_link())
            .map_or(self.local, |setup| setup.association.local)
    }

    /// The other links of the current association.
    pub fn link_contexts(&self) -> &[Arc<AssociationContext>] {
        if matches!(self.phase, Phase::Configuring | Phase::Associated) {
            &self.links
        } else {
            &[]
        }
    }

    fn clock(&mut self, now_us: u64) -> Result<(), Error> {
        if now_us < self.now {
            return Err(Error::ClockRegressed);
        }
        self.now = now_us;
        Ok(())
    }

    fn ticket(&mut self) -> Result<Ticket, Error> {
        self.serial = self.serial.checked_add(1).ok_or(Error::Overflow)?;
        Ok(Ticket {
            epoch: self.epoch,
            serial: self.serial,
        })
    }

    fn transmit(
        &mut self,
        phase: Phase,
        attempts: u8,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        let local = self.link_address();
        let join = self.join.as_ref().ok_or(Error::NotAssociated)?;
        let channel = join.channel;
        if !policy.allow_management_tx(channel) {
            return Err(Error::Regulatory);
        }
        let bytes = match phase {
            // ieee80211_auth always restarts at transaction 1.
            Phase::Authenticating => wire::authentication_with(
                self.local,
                join.bssid,
                self.sequence,
                join.request.algorithm(),
            ),
            Phase::Associating => join.request.encode(local, join.bssid, self.sequence),
            _ => return Err(Error::Busy),
        }?;
        let deadline = self
            .now
            .checked_add(MANAGEMENT_RESPONSE_TIMEOUT_US)
            .ok_or(Error::Overflow)?;
        if phase == Phase::Authenticating {
            self.auth_transaction = 2;
        }
        self.queue(phase, attempts, deadline, channel, bytes)
    }

    fn queue(
        &mut self,
        phase: Phase,
        attempts: u8,
        deadline: u64,
        channel: Channel,
        bytes: Vec<u8>,
    ) -> Result<Action, Error> {
        let ticket = self.ticket()?;
        self.sequence = next_sequence(self.sequence);
        self.phase = phase;
        self.pending = Some(Pending {
            ticket,
            queued: false,
            deadline,
            attempts,
        });
        Ok(Action::Transmit {
            ticket,
            channel,
            bytes,
        })
    }

    pub fn begin(
        &mut self,
        join: Join,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        if join.request.modern().is_some() {
            return Err(Error::Unsupported(
                "modern join requires selected actual beacon",
            ));
        }
        self.begin_inner(join, None, now_us, policy)
    }

    /// Start an explicit modern association using the selected beacon.
    /// This supplies no SAE/PMF, OS authentication or native radio permission;
    /// 6GHz remains refused until those boundaries are operational.
    pub fn begin_modern(
        &mut self,
        join: Join,
        beacon: super::scan::result::Bss,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        if join.request.pmf().is_some() {
            return Err(Error::Unsupported(
                "PMF requires original native observation/OS authority",
            ));
        }
        self.begin_modern_inner(join, beacon, now_us, policy)
    }
    pub(in crate::wifi) fn begin_modern_pmf(
        &mut self,
        join: Join,
        observed: Arc<NativeBssObservation>,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        match join.request.pmf() {
            None => return Err(Error::Unsupported("selected PMF request required")),
            // The OS runs SAE before associate_external_pmf; a cached
            // PMKSA resumes by open-system authentication and its PMKID.
            Some(pmf) if pmf.sae() && !pmf.pmkid() => {
                return Err(Error::Unsupported("SAE authenticates through the OS"));
            }
            Some(_) => {}
        }
        let action = self.begin_modern_inner(join, observed.bss.clone(), now_us, policy)?;
        self.pmf_observation = Some(observed);
        Ok(action)
    }
    fn begin_modern_inner(
        &mut self,
        join: Join,
        beacon: super::scan::result::Bss,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        let selection = join
            .request
            .modern()
            .ok_or(Error::Unsupported("modern request required"))?;
        if selection.capabilities().band() != join.channel.band
            || beacon.channel != join.channel
            || !beacon.answers_for(join.bssid)
        {
            return Err(Error::Unsupported("selected beacon/local PHY mismatch"));
        }
        self.begin_inner(join, Some(beacon), now_us, policy)
    }

    pub(in crate::wifi) fn pmf_authentication(
        &self,
    ) -> Result<(Ticket, &Join, &Arc<NativeBssObservation>), Error> {
        if !matches!(self.phase, Phase::Authenticating | Phase::Associating) {
            return Err(Error::Stale);
        }
        let join = self.join.as_ref().ok_or(Error::Stale)?;
        if join.request.pmf().is_none() {
            return Err(Error::Stale);
        }
        Ok((
            self.pending.ok_or(Error::Stale)?.ticket,
            join,
            self.pmf_observation.as_ref().ok_or(Error::Stale)?,
        ))
    }

    /// Withdraw only this first never-submitted PMF request. Native
    /// code calls this before its first peer/ROC/DMA mutation; it releases no
    /// hardware owner and supplies no retirement proof. Epochs are not reused.
    pub(in crate::wifi) fn withdraw_unsubmitted_pmf(
        &mut self,
        ticket: Ticket,
    ) -> Result<(), Error> {
        // An SAE association goes back to its authenticated exchange.
        let external = self.phase == Phase::Associating && self.external.is_some();
        if (self.phase != Phase::Authenticating && !external)
            || self.pending.is_none_or(|pending| {
                pending.ticket != ticket || pending.queued || pending.attempts != 1
            })
            || !self.requires_protected_management()
        {
            return Err(Error::Stale);
        }
        self.pending = None;
        self.join = None;
        self.selected_beacon = None;
        self.context = None;
        self.links.clear();
        self.pmf_observation = None;
        self.phase = if external {
            Phase::ExternalAuthenticating
        } else {
            Phase::Idle
        };
        Ok(())
    }

    /// `ieee80211_mgd_assoc` once the OS's SAE exchange authenticated the
    /// station (it is -EBUSY before): associate with the OS's SAE request
    /// and the native beacon observation, without another authentication.
    /// The exchange stays until the association ends either way.
    pub(in crate::wifi) fn associate_external_pmf(
        &mut self,
        session: &external::Session,
        join: Join,
        observed: Arc<NativeBssObservation>,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        self.validate_external_authentication(session)?;
        if !self.external_authenticated() {
            return Err(Error::Busy);
        }
        self.clock(now_us)?;
        if self.now >= self.sae.deadline() {
            return Err(Error::Timeout);
        }
        if join.request.pmf().is_none_or(|pmf| !pmf.sae()) {
            return Err(Error::Unsupported("SAE PMF request required"));
        }
        if join.bssid != session.peer() || join.channel != session.channel() {
            return Err(Error::InvalidAddress);
        }
        // A multi-link setup runs on the link and between the MLDs the
        // exchange authenticated.
        if join.request.multi_link().map(|setup| setup.association) != session.multi_link().copied()
        {
            return Err(Error::Unsupported(
                "multi-link association outside its exchange",
            ));
        }
        // A 6 GHz exchange associates with the beacon and RSN it authenticated.
        if let Some(six) = session.six_ghz_observation()
            && (!Arc::ptr_eq(six, &observed)
                || session
                    .six_ghz_selection()
                    .zip(join.request.pmf())
                    .is_none_or(|(selected, pmf)| selected.rsn() != pmf.rsn()))
        {
            return Err(Error::Unsupported("6GHz association outside its exchange"));
        }
        let selection = join
            .request
            .modern()
            .ok_or(Error::Unsupported("modern request required"))?;
        let beacon = observed.bss.clone();
        if selection.capabilities().band() != join.channel.band
            || beacon.channel != join.channel
            || !beacon.answers_for(join.bssid)
        {
            return Err(Error::Unsupported("selected beacon/local PHY mismatch"));
        }
        join.request.validate_policy(join.policy)?;
        self.join = Some(join);
        self.selected_beacon = Some(beacon);
        self.context = None;
        self.links.clear();
        self.pmf_observation = Some(observed);
        self.phase = Phase::Associating;
        match self.transmit(Phase::Associating, 1, policy) {
            Ok(action) => Ok(action),
            Err(error) => {
                self.join = None;
                self.selected_beacon = None;
                self.pmf_observation = None;
                self.phase = Phase::ExternalAuthenticating;
                Err(error)
            }
        }
    }

    /// Identity preflight, not a publication lease. Native submission must also
    /// hold current association/radio/regulatory guards through submit.
    pub fn validate_association_context(&self, context: &AssociationContext) -> Result<(), Error> {
        if !matches!(self.phase, Phase::Configuring | Phase::Associated)
            || self
                .context
                .as_ref()
                .is_none_or(|current| !core::ptr::eq(current.as_ref(), context))
        {
            return Err(Error::Stale);
        }
        Ok(())
    }

    /// The channel frames of the current join use: the association's
    /// operating channel once one exists.
    pub(crate) fn current_channel(&self) -> Option<super::Channel> {
        self.operating_definition()
            .map(|definition| definition.control)
            .or(self.join.as_ref().map(|join| join.channel))
    }

    /// The channel the current association operates: its own, or where a
    /// channel switch moved it.
    pub fn operating_definition(&self) -> Option<operations::ChannelDefinition> {
        let context = self.context.as_ref()?;
        Some(match self.switched {
            Some((ticket, definition)) if ticket == context.ticket() => definition,
            _ => context.definition(),
        })
    }

    /// mac80211 `ieee80211_chswitch_done`: the associated `context` now
    /// operates `definition`, on the same band.
    pub fn switch_channel(
        &mut self,
        context: &AssociationContext,
        definition: operations::ChannelDefinition,
    ) -> Result<(), Error> {
        self.validate_association_context(context)?;
        if self.phase != Phase::Associated {
            return Err(Error::NotAssociated);
        }
        definition.validate().map_err(Error::Phy)?;
        if definition.control.band != context.channel().band {
            return Err(Error::Unsupported("channel switch to another band"));
        }
        self.switched = Some((context.ticket(), definition));
        Ok(())
    }

    pub fn association_context(&self) -> Option<&AssociationContext> {
        self.context
            .as_deref()
            .filter(|context| self.validate_association_context(context).is_ok())
    }

    fn begin_inner(
        &mut self,
        join: Join,
        beacon: Option<super::scan::result::Bss>,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        if self.phase != Phase::Idle {
            return Err(Error::Busy);
        }
        self.clock(now_us)?;
        if join.bssid.is_multicast() || join.bssid.0 == [0; 6] || join.bssid == self.local {
            return Err(Error::InvalidAddress);
        }
        if join.channel.band == super::Band::Ghz6 {
            return Err(Error::Unsupported("6GHz HE/SAE/PMF association"));
        }
        if join.request.multi_link().is_some() {
            return Err(Error::Unsupported("multi-link setup outside SAE"));
        }
        join.request.validate_policy(join.policy)?;
        self.epoch = NEXT_ASSOCIATION.next().ok_or(Error::Overflow)?;
        self.serial = 0;
        self.join = Some(join);
        self.selected_beacon = beacon;
        self.context = None;
        self.links.clear();
        self.pmf_observation = None;
        self.wep_iv = self
            .join
            .as_ref()
            .and_then(|join| join.request.wep())
            .map_or(0, wire::Wep::initial_iv);
        self.phase = Phase::Authenticating;
        match self.transmit(Phase::Authenticating, 1, policy) {
            Ok(action) => Ok(action),
            Err(error) => {
                self.phase = Phase::Idle;
                self.join = None;
                self.selected_beacon = None;
                self.context = None;
                Err(error)
            }
        }
    }

    /// `ieee80211_auth_challenge`: without a challenge element the frame is
    /// ignored; otherwise transaction 3 goes out under the running
    /// authentication timeout, which its TX status re-arms.
    fn answer_challenge(
        &mut self,
        reply: &wire::Authentication<'_>,
        key: super::security::wep::Key,
        pending: Pending,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        let challenge = reply
            .challenge()
            .ok_or(Error::Wire(wire::Error::Invalid("shared-key challenge")))?;
        let join = self.join.as_ref().ok_or(Error::NotAssociated)?;
        let (bssid, channel) = (join.bssid, join.channel);
        if !policy.allow_management_tx(channel) {
            return Err(Error::Regulatory);
        }
        let iv = super::security::wep::next_iv(&mut self.wep_iv, key.material().len());
        let bytes =
            wire::shared_key_response(self.local, bssid, self.sequence, challenge, &key, iv)?;
        self.auth_transaction = 4;
        self.queue(
            Phase::Authenticating,
            pending.attempts,
            pending.deadline,
            channel,
            bytes,
        )
    }

    pub fn submitted(&mut self, ticket: Ticket) -> Result<(), Error> {
        if !matches!(self.phase, Phase::Authenticating | Phase::Associating) {
            return Err(Error::Stale);
        }
        let pending = self.pending.as_mut().ok_or(Error::Stale)?;
        if pending.ticket != ticket {
            return Err(Error::Stale);
        }
        pending.queued = true;
        Ok(())
    }

    /// Call after a failed queue/air result. The current association is
    /// retired explicitly; callers may begin again after real removal/drain.
    pub fn transmit_failed(&mut self, ticket: Ticket) -> Result<Action, Error> {
        if !matches!(self.phase, Phase::Authenticating | Phase::Associating) {
            return Err(Error::Stale);
        }
        if self.pending.is_none_or(|pending| pending.ticket != ticket) {
            return Err(Error::Stale);
        }
        self.remove(Some(Failure::Transmit))
    }

    /// The AP's departure from a PMF association, `body` being the frame's
    /// reason code onward: authenticated by its keys, or in the clear before
    /// the 4-way handshake installs them, which `ieee80211_drop_unencrypted_mgmt`
    /// permits.
    pub fn receive_protected_departure(
        &mut self,
        epoch: u64,
        frame: &Mpdu,
        body: &[u8],
    ) -> Result<Action, Error> {
        if epoch != self.epoch {
            return Err(Error::Stale);
        }
        let join = self.join.as_ref().ok_or(Error::NotAssociated)?;
        let (frame, link) = self.mld_view(frame)?;
        if frame.fcs_error || frame.monitor_only {
            return Err(Error::Wire(wire::Error::Integrity));
        }
        if join.request.pmf().is_none()
            || !matches!(
                frame.header().control & TYPE_SUBTYPE,
                DISASSOCIATION | DEAUTHENTICATION
            )
        {
            return Err(Error::Unsupported("protected departure"));
        }
        // check_peer's addressing, for a frame that may be protected.
        let header = frame.header();
        if header.control & 0x800c != 0 || header.length != 24 || header.fragmented() {
            return Err(Error::Unsupported("nonordinary management frame"));
        }
        let (local, peer) = self.addresses()?;
        let peer = if header.receiver.is_multicast() {
            link.ap
        } else {
            peer
        };
        if (header.receiver != local && header.receiver != Mac::BROADCAST)
            || header.transmitter != Some(peer)
            || frame.bytes()[16..22] != peer.0
        {
            return Err(Error::Wire(wire::Error::WrongPeer));
        }
        let reason = body
            .first_chunk()
            .map(|reason| u16::from_le_bytes(*reason))
            .ok_or(Error::Wire(wire::Error::Truncated))?;
        self.remove(Some(Failure::PeerDeparture(reason)))
    }

    /// `epoch` must be held from the firmware/RX association context;
    /// never retag old deferred RX by looking up the current epoch at delivery.
    pub fn receive(
        &mut self,
        epoch: u64,
        frame: &Mpdu,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Action, Error> {
        self.clock(now_us)?;
        if epoch != self.epoch {
            return Err(Error::Stale);
        }
        let join = self.join.as_ref().ok_or(Error::NotAssociated)?;
        let (view, link) = self.mld_view(frame)?;
        let frame = view.as_ref();
        if frame.fcs_error || frame.monitor_only {
            return Err(Error::Wire(wire::Error::Integrity));
        }
        let (local, peer) = self.addresses()?;
        let peer = if frame.header().receiver.is_multicast() {
            link.ap
        } else {
            peer
        };
        wire::check_peer(frame, local, peer)?;
        if join.request.pmf().is_some() && super::security::management::robust(frame) {
            return Err(Error::Unsupported(
                "PMF requires authenticated management consumer",
            ));
        }
        if matches!(
            frame.header().control & TYPE_SUBTYPE,
            DISASSOCIATION | DEAUTHENTICATION
        ) {
            let reason = wire::departure(frame)?;
            return self.remove(Some(Failure::PeerDeparture(reason)));
        }
        let pending = self.pending.ok_or(Error::Stale)?;
        if !pending.queued {
            return Err(Error::NotQueued);
        }
        if matches!(self.phase, Phase::Authenticating | Phase::Associating)
            && self.now >= pending.deadline
        {
            if pending.attempts < MANAGEMENT_ATTEMPTS {
                return self.transmit(self.phase, pending.attempts + 1, policy);
            }
            return self.remove(Some(Failure::Timeout));
        }
        match self.phase {
            Phase::Authenticating => {
                // ieee80211_rx_mgmt_auth ignores another algorithm or
                // transaction.
                let reply = wire::authentication_frame(frame)?;
                if reply.algorithm != join.request.algorithm()
                    || reply.transaction != self.auth_transaction
                {
                    return Err(Error::Wire(wire::Error::Invalid(
                        "unexpected authentication state",
                    )));
                }
                if reply.status != 0 {
                    return self.remove(Some(Failure::PeerRejected(reply.status)));
                }
                let shared = join
                    .request
                    .wep()
                    .filter(|wep| wep.algorithm() == wire::SHARED_KEY)
                    .filter(|_| self.auth_transaction == 2)
                    .map(|wep| wep.key().clone());
                if let Some(key) = shared {
                    return self.answer_challenge(&reply, key, pending, policy);
                }
                self.transmit(Phase::Associating, 1, policy)
            }
            Phase::Associating => {
                let response = wire::association_response(frame)?;
                if response.status != 0 {
                    return self.remove(Some(Failure::PeerRejected(response.status)));
                }
                let answers = match join.request.multi_link() {
                    Some(setup) => {
                        let requested: Vec<u8> =
                            setup.links.iter().map(|link| link.link_id).collect();
                        match wire::multi_link::association_response(
                            setup.association.mld.ap,
                            &response.elements,
                            &requested,
                        ) {
                            Ok((_, answers)) => answers,
                            Err(_) => return self.remove(Some(Failure::MultiLink)),
                        }
                    }
                    None => Vec::new(),
                };
                let bssid = join.bssid;
                let channel = join.channel;
                let request = join.request.clone();
                let deadline = self
                    .now
                    .checked_add(CONFIGURATION_TIMEOUT_US)
                    .ok_or(Error::Overflow)?;
                let ticket = self.ticket()?;
                let context = if request.modern().is_some() {
                    let join = self.join.as_ref().ok_or(Error::NotAssociated)?;
                    let beacon = self
                        .selected_beacon
                        .as_ref()
                        .ok_or(Error::Unsupported("selected beacon required"))?;
                    let context = if request.pmf().is_some() {
                        AssociationContext::build_with_native(
                            ticket,
                            join,
                            beacon,
                            &response,
                            self.pmf_observation.as_ref(),
                        )?
                    } else {
                        AssociationContext::build(ticket, join, beacon, &response)?
                    };
                    Some(Arc::new(context))
                } else {
                    None
                };
                let links = self.accepted_links(ticket, &response, &answers)?;
                self.context = context.clone();
                self.links = links.clone();
                self.external = None;
                self.phase = Phase::Configuring;
                self.pending = Some(Pending {
                    ticket,
                    queued: true,
                    deadline,
                    attempts: 0,
                });
                Ok(Action::Configure {
                    ticket,
                    bssid,
                    channel,
                    request,
                    response,
                    context,
                    links,
                })
            }
            _ => Err(Error::Stale),
        }
    }

    /// `ieee80211_assoc_success` for the other links of a multi-link
    /// setup: each the AP MLD accepted is configured from its affiliated
    /// BSS and its answer under the association's AID, and one it refused is
    /// left out.
    fn accepted_links(
        &self,
        ticket: Ticket,
        response: &wire::AssociationResponse,
        answers: &[wire::multi_link::Answer],
    ) -> Result<Vec<Arc<AssociationContext>>, Error> {
        let join = self.join.as_ref().ok_or(Error::NotAssociated)?;
        let Some(setup) = join.request.multi_link() else {
            return Ok(Vec::new());
        };
        let mut links = Vec::new();
        for answer in answers.iter().filter(|answer| answer.status == 0) {
            let link = setup.link(answer.link_id).ok_or(Error::Stale)?;
            let bss = link.observation.bss();
            let link_join = Join {
                bssid: bss.bssid,
                channel: bss.channel,
                policy: join.policy,
                request: link.request.clone(),
            };
            let link_response = wire::AssociationResponse {
                capabilities: answer.capability,
                status: 0,
                elements: answer.elements.clone(),
                ..response.clone()
            };
            let context = AssociationContext::build_with_native(
                ticket,
                &link_join,
                bss,
                &link_response,
                Some(&link.observation),
            )?
            .affiliated(Affiliation {
                link_id: link.link_id,
                local: link.local,
                ..setup.association
            });
            links.push(Arc::new(context));
        }
        Ok(links)
    }

    pub fn poll(
        &mut self,
        now_us: u64,
        policy: &mut impl RegulatoryPolicy,
    ) -> Result<Option<Action>, Error> {
        self.clock(now_us)?;
        if self.phase == Phase::ExternalAuthenticating && self.external.is_some() {
            // ieee80211_rx_mgmt_auth destroys a denied exchange; one past its
            // timeout, or authenticated but never associated, ends too.
            if let Some(status) = self.sae.denied() {
                return self.remove(Some(Failure::PeerRejected(status))).map(Some);
            }
            if self.now >= self.sae.deadline() || self.sae.exhausted(self.now) {
                return self.remove(Some(Failure::Timeout)).map(Some);
            }
        }
        let Some(pending) = self.pending else {
            return Ok(None);
        };
        if self.phase == Phase::Configuring && self.now >= pending.deadline {
            self.phase = Phase::Quarantined;
            self.pending = None;
            return Err(Error::Timeout);
        }
        if !matches!(self.phase, Phase::Authenticating | Phase::Associating)
            || self.now < pending.deadline
        {
            return Ok(None);
        }
        if !pending.queued || pending.attempts >= MANAGEMENT_ATTEMPTS {
            return self.remove(Some(Failure::Timeout)).map(Some);
        }
        self.transmit(self.phase, pending.attempts + 1, policy)
            .map(Some)
    }

    /// `ieee80211_sta_work` on the TX status of the latest authentication or
    /// association request: an acknowledged one waits
    /// IEEE80211_AUTH_TIMEOUT_SHORT or IEEE80211_ASSOC_TIMEOUT_SHORT for the
    /// response, and a lost one is retried at once.
    pub fn management_status(&mut self, acked: bool, now_us: u64) -> Result<(), Error> {
        self.clock(now_us)?;
        match self.phase {
            Phase::Authenticating | Phase::Associating => {
                if let Some(pending) = self.pending.as_mut().filter(|pending| pending.queued) {
                    pending.deadline = if acked {
                        self.now
                            .checked_add(MANAGEMENT_ACKED_TIMEOUT_US)
                            .ok_or(Error::Overflow)?
                    } else {
                        self.now
                    };
                }
                Ok(())
            }
            Phase::ExternalAuthenticating => self.sae.transmit_status(acked, self.now),
            _ => Ok(()),
        }
    }

    pub fn complete_configuration(
        &mut self,
        ticket: Ticket,
        result: Completion,
        now_us: u64,
    ) -> Result<(), Error> {
        self.clock(now_us)?;
        if self.phase != Phase::Configuring
            || self.pending.is_none_or(|pending| pending.ticket != ticket)
        {
            return Err(Error::Stale);
        }
        if self
            .pending
            .is_some_and(|pending| self.now >= pending.deadline)
        {
            self.phase = Phase::Quarantined;
            self.pending = None;
            return Err(Error::Timeout);
        }
        self.pending = None;
        self.phase = match result {
            Completion::Accepted => Phase::Associated,
            Completion::RejectedAndIdle => {
                self.join = None;
                self.selected_beacon = None;
                self.context = None;
                self.links.clear();
                self.pmf_observation = None;
                Phase::Idle
            }
            Completion::Uncertain => Phase::Quarantined,
        };
        Ok(())
    }

    pub(in crate::wifi) fn protected_port_authorized(&self) -> bool {
        self.requires_protected_management() && self.authorization.is_some()
    }

    pub fn requires_protected_management(&self) -> bool {
        self.join
            .as_ref()
            .is_some_and(|join| join.request.pmf().is_some())
    }

    /// Generic OS controlled-port command, not proof generated by this layer.
    /// The trusted frontend must issue it only after authenticating the real
    /// handshake and peer. Each link's PTK and GTK, in `on_air` order, must
    /// already have firmware ACKs.
    pub fn authorize_port(
        &mut self,
        epoch: u64,
        links: &[(&KeySlot, &KeySlot)],
    ) -> Result<(), Error> {
        if self.requires_protected_management() {
            return Err(Error::Unsupported(
                "PMF requires current OS/key session authority",
            ));
        }
        self.authorize_port_keys(epoch, links)
    }

    /// Internal native consumer: caller has validated the original current
    /// management Session and holds the OS protected-port permission.
    pub(in crate::wifi) fn authorize_protected_port(
        &mut self,
        context: &AssociationContext,
        epoch: u64,
        links: &[(&KeySlot, &KeySlot)],
    ) -> Result<(), Error> {
        self.validate_association_context(context)?;
        if context.pmf().is_none() {
            return Err(Error::Unsupported("no selected PMF policy"));
        }
        self.authorize_port_keys(epoch, links)
    }

    fn authorize_port_keys(
        &mut self,
        epoch: u64,
        links: &[(&KeySlot, &KeySlot)],
    ) -> Result<(), Error> {
        if self.phase != Phase::Associated || epoch != self.epoch {
            return Err(Error::NotAssociated);
        }
        // A multi-link association's PTK is the AP MLD's, kept on each link;
        // each link keeps its AP's GTK.
        let peer = self.pairwise_peer().ok_or(Error::NotAssociated)?;
        let ciphers = self
            .join
            .as_ref()
            .and_then(|join| join.request.key_ciphers());
        let mut keys = Vec::new();
        let mut on_air = self.on_air();
        for (pairwise, group) in links {
            let link = on_air.next().ok_or(Error::WrongKey)?;
            let ptk = pairwise
                .installed()
                .filter(|key| key.scope() == KeyScope::Pairwise(peer))
                .ok_or(Error::WrongKey)?;
            let gtk = group
                .installed()
                .filter(|key| key.scope() == KeyScope::Group(link.ap))
                .ok_or(Error::WrongKey)?;
            if ptk.nonce_owner() == super::security::NonceOwner::ReceiveOnly {
                return Err(Error::WrongKey);
            }
            // A static WEP transmit key may take any default key index.
            if ptk.index() != 0 && !ptk.cipher().is_wep() {
                return Err(Error::Unsupported("extended pairwise key ID"));
            }
            if ciphers != Some((ptk.cipher(), gtk.cipher())) {
                return Err(Error::WrongKey);
            }
            keys.push((ptk, gtk));
        }
        if on_air.next().is_some() {
            return Err(Error::WrongKey);
        }
        drop(on_air);
        self.authorization = Some(keys);
        Ok(())
    }

    pub fn close_port(&mut self) {
        self.authorization = None;
    }

    /// Recheck current acknowledged identities, so external key invalidation or
    /// rekey cannot leave a cached successful-looking carrier value: every
    /// authorized key must still be installed in one of `keys`.
    pub fn port_open(&self, keys: &[&KeySlot]) -> bool {
        if self.phase != Phase::Associated {
            return false;
        }
        if self
            .join
            .as_ref()
            .is_some_and(|join| join.policy == PortPolicy::Open)
        {
            return true;
        }
        self.authorization.as_ref().is_some_and(|links| {
            links
                .iter()
                .flat_map(|&(ptk, gtk)| [ptk, gtk])
                .all(|key| keys.iter().any(|slot| slot.installed() == Some(key)))
        })
    }

    pub fn disconnect(&mut self) -> Result<Action, Error> {
        self.remove(None)
    }

    fn remove(&mut self, failure: Option<Failure>) -> Result<Action, Error> {
        if matches!(self.phase, Phase::Idle | Phase::Disconnecting) {
            return Err(Error::NotAssociated);
        }
        self.authorization = None;
        let ticket = self.ticket()?;
        self.phase = Phase::Disconnecting;
        self.pending = Some(Pending {
            ticket,
            queued: true,
            deadline: u64::MAX,
            attempts: 0,
        });
        Ok(Action::Remove { ticket, failure })
    }

    /// Accepted removal includes radio/key/RX/deferred-work quiescence.
    /// Uncertainty quarantines the session; it never permits another join.
    pub fn complete_removal(&mut self, ticket: Ticket, result: Completion) -> Result<(), Error> {
        if self.phase != Phase::Disconnecting
            || self.pending.is_none_or(|pending| pending.ticket != ticket)
        {
            return Err(Error::Stale);
        }
        self.pending = None;
        self.authorization = None;
        self.phase = if result == Completion::Accepted {
            // Accepted removal includes every BA/reorder/deferred user.
            self.rx_ba = ba::Sessions::new();
            self.tx_ba = ba::tx::Sessions::new();
            self.qos_sequence = [0; 16];
            self.join = None;
            self.external = None;
            self.selected_beacon = None;
            self.context = None;
            self.links.clear();
            self.pmf_observation = None;
            Phase::Idle
        } else {
            Phase::Quarantined
        };
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests;
