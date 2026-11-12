// SPDX-License-Identifier: GPL-2.0-only
// Adapted from pinned Linux net/mac80211/mlme.c; full source notices in
// NOTICE. The connection policy follows wpa_supplicant's behavior.
//! One station kept connected to the configured network, as wpa_supplicant
//! and mac80211 keep one, over any vendor's [`Radio`]: a scan of every band,
//! 6 GHz last as cfg80211 splits it, the strongest BSS of the network whose
//! security the station supports, open-system or SAE authentication and
//! association on mac80211's channel and mode, with an AP MLD's setup link and
//! one more link when the NIC supports MLO, the 4-way and group key handshakes,
//! their keys and the controlled port. A lost or failed connection scans again
//! after wpa_supplicant's delays.
use crate::{
    Clock, Link, LinkError, Mac,
    wifi::{
        Band, Channel,
        connection::{
            self, Request,
            mode::{self, Channels},
            security::{self, Security},
        },
        network::{Network, Secret},
        regulatory::PowerType,
        sae::{self, Derivation, Exchange},
        scan::{self, result::Bss},
        security::{KeyIdentity, PacketNumber, ReplaySeed},
        station::{
            Join, NativeBssObservation, PortPolicy, Station,
            external::{self, Received},
            multi_link::{self as mlo, Affiliation, Mld, Setup},
            operations::Operations,
            phy::{Capabilities, McsMaps, Mode, Role},
            wire::{
                AssociationRequest,
                multi_link::{Basic, affiliated},
                six_ghz::Selection,
            },
        },
        supplicant::{self, Akm, EAPOL, IntegrityKey, MultiLink, Step, Supplicant},
    },
};
use alloc::{boxed::Box, collections::VecDeque, format, string::String, sync::Arc, vec, vec::Vec};
use core::fmt::Display;
use stela_driver_base::entropy::Entropy;
use zeroize::Zeroizing;

/// Where the radio's own association and scan machinery stands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// No scan or peer: a scan or a join may begin.
    Idle,
    /// A scan finished; it waits to be retired.
    Scanned,
    /// The AP accepted the association: it waits to be configured.
    Admitted,
    /// A departure finished; its peer waits to be retired.
    NeedsDrain,
    /// Anything between those.
    Busy,
    /// The radio gave up on its hardware: only [`Radio::recover`] can bring
    /// the link back, and nothing does on a radio that cannot recover.
    Lost,
}

/// What the radio reported since the keeper last looked.
pub enum Event {
    /// An EAPOL frame from the AP, as an Ethernet frame.
    Eapol(Vec<u8>),
    /// An SAE frame from the AP.
    Sae(Box<Received>),
    /// The SAE exchange ended without success.
    SaeEnded(String),
    /// An operation the radio declines, which the link reports once.
    Unsupported(&'static str),
    /// The AP or the firmware ended the association.
    ConnectionLost,
    /// The firmware went with all it held, and the radio may be booting it
    /// again.
    FirmwareLost,
    /// A TKIP frame failed its Michael MIC, `cfg80211_michael_mic_failure`.
    MichaelFailure { pairwise: bool },
}

/// What the NIC offers on a band.
pub struct Advertisement {
    pub capabilities: Capabilities,
    /// The capability elements an association request carries.
    pub elements: Vec<u8>,
    /// The NIC's HE MCS maps at 80, 160 and 80+80 MHz.
    pub he_source_maps: [McsMaps; 3],
    /// Whether the NIC reports MLO, when it does report it.
    pub multi_link: Option<bool>,
}

/// The chosen AP as a station profile describes it.
#[derive(Clone, Copy, Debug)]
pub struct Peer {
    pub bssid: Mac,
    pub channel: Channel,
    pub basic_rates: u16,
    pub supported_rates: u16,
    pub beacon_interval: u16,
    pub dtim_period: u8,
    pub slot_time: u16,
}

/// One vendor's station: its firmware's scans, joins, keys and frames.
pub trait Radio: Link {
    /// A BSS the radio observed.
    type Handle: Copy;
    /// What the vendor's join needs beside the [`Join`].
    type Profile;
    /// The protected-management transport of an installed IGTK.
    type Management;
    type Error: Display;

    /// The name the keeper's reports carry.
    const NAME: &'static str;
    /// mac80211's `hw.conf.listen_interval` for this driver.
    const LISTEN_INTERVAL: u16;

    /// Whether `error` only asks to try again once queued frames drain.
    fn deferred(error: &Self::Error) -> bool;

    /// Service the radio.
    fn service(&mut self) -> Result<(), Self::Error>;

    /// Whether [`Self::recover`] can bring the radio back once it is
    /// [`Stage::Lost`].
    fn recoverable(&self) -> bool {
        false
    }

    /// One step of resetting the lost radio's hardware and bringing it back
    /// to [`Stage::Idle`] with the same station address, as Linux's chip
    /// reset work does (`mt7925_mac_reset_work`, iwlwifi's firmware restart,
    /// rtw88's firmware crash recovery). Everything the radio held before is
    /// gone; the keeper joins again once a step returns
    /// [`Recovery::Done`].
    ///
    /// One call returns within [`RECOVERY_STEP_US`]: it does one bounded
    /// slice of the bring-up, such as one reset wait, one batch of firmware
    /// chunks or one command, and returns [`Recovery::Pending`] until the
    /// bring-up is complete. The keeper calls it again on its next poll. The
    /// radio bounds the whole attempt as Linux bounds each of its waits and
    /// returns an error once it fails, saying why; the next attempt starts
    /// over from the reset.
    fn recover(&mut self) -> Result<Recovery, String> {
        Err(format!("{}: cannot recover", Self::NAME))
    }
    fn take_event(&mut self) -> Option<Event>;
    fn stage(&self) -> Stage;
    fn station(&self) -> &Station;

    /// The channels of `band` that may transmit, with their rule flags.
    fn channels(&self, band: Band) -> Vec<(Channel, u32)>;
    fn probes(&self, channel: Channel) -> bool;
    /// The Supported Channels of `band`, which a spectrum-managed request names.
    fn supported_channels(&self, band: Band) -> &'static [u8];
    /// Whether a multi-link association may use a 6 GHz link.
    fn six_ghz_links(&self) -> bool {
        false
    }
    /// `eml_capabilities` of `struct wiphy_iftype_ext_capab`.
    fn eml_capabilities(&self) -> u16 {
        0
    }
    /// The country's power limit at `frequency_mhz`.
    fn regulated_power_dbm(&self, frequency_mhz: u16) -> i32;
    fn advertisement(&self, band: Band) -> Result<Advertisement, Self::Error>;
    fn profile(&self, peer: Peer) -> Result<Self::Profile, Self::Error>;

    fn begin_scan(&mut self, request: scan::Request, budget_us: u64) -> Result<(), Self::Error>;
    /// # Safety
    /// The scan's completion was its last user, and the radio is polled with
    /// its interrupts masked.
    unsafe fn retire_scan(&mut self) -> Result<(), Self::Error>;
    fn six_ghz_scan(
        &self,
        request: &scan::Request,
        options: scan::six_ghz::Options,
    ) -> Result<Option<scan::Request>, Self::Error>;
    fn bss(&self) -> Vec<(Self::Handle, &Bss)>;
    fn observation(&self, handle: Self::Handle) -> Result<Arc<NativeBssObservation>, Self::Error>;

    fn join(
        &mut self,
        handle: Self::Handle,
        join: Join,
        profile: Self::Profile,
    ) -> Result<(), Self::Error>;
    fn begin_external_authentication(
        &mut self,
        handle: Self::Handle,
        profile: Self::Profile,
        multi_link: Option<Affiliation>,
        budget_us: u64,
    ) -> Result<Arc<external::Session>, Self::Error>;
    fn begin_six_ghz_external_authentication(
        &mut self,
        handle: Self::Handle,
        selection: Selection,
        profile: Self::Profile,
        multi_link: Option<Affiliation>,
        budget_us: u64,
    ) -> Result<Arc<external::Session>, Self::Error>;
    fn transmit_external_authentication(
        &mut self,
        session: &external::Session,
        frame: external::Frame<'_>,
    ) -> Result<(), Self::Error>;
    fn associate_external(
        &mut self,
        session: &external::Session,
        handle: Self::Handle,
        join: Join,
    ) -> Result<(), Self::Error>;
    fn configure(&mut self) -> Result<(), Self::Error>;

    fn send_control(
        &mut self,
        ethernet: &[u8],
        key: Option<KeyIdentity>,
    ) -> Result<(), Self::Error>;
    fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, Self::Error>;
    fn install_link_key(
        &mut self,
        link_id: u8,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<(), Self::Error>;
    /// Keep link `link_id`'s IGTK: whether the radio can.
    ///
    /// # Safety
    /// As install_management_key, including peer retirement before replacing a
    /// live index and independent old hardware/RX/IRQ/work/key-user drainage.
    unsafe fn install_link_management_key(
        &mut self,
        _link_id: u8,
        _index: u8,
        _material: &[u8],
        _receive: PacketNumber,
    ) -> Result<bool, Self::Error> {
        Ok(false)
    }
    /// # Safety
    /// The association's authenticated handshake gave the key, and the radio
    /// is polled with its interrupts masked, so no earlier key user remains.
    unsafe fn install_management_key(
        &mut self,
        epoch: u64,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Self::Management, Self::Error>;
    /// # Safety
    /// As for [`Radio::install_management_key`].
    unsafe fn rotate_management_key(
        &mut self,
        session: &Self::Management,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Self::Management, Self::Error>;
    fn authorize_port(&mut self, epoch: u64) -> Result<(), Self::Error>;
    fn deauthenticate(&mut self, reason: u16) -> Result<(), Self::Error>;
    /// # Safety
    /// The departure's removal event was its last user, and the radio is
    /// polled with its interrupts masked.
    unsafe fn retire_peer(&mut self) -> Result<(), Self::Error>;
}

/// Where one [`Radio::recover`] step left the recovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Recovery {
    /// More steps follow, on the next polls.
    Pending,
    /// The radio is back at [`Stage::Idle`].
    Done,
}

/// The longest one [`Radio::recover`] step may hold a poll: one runtime
/// command at its bound, with room for the register accesses around it.
pub const RECOVERY_STEP_US: u64 = RUNTIME_COMMAND_TIMEOUT_US + 10_000;

/// `mt7925_mac_reset_work`'s ten chip reset attempts.
pub const RECOVERY_ATTEMPTS: u32 = 10;

/// The pause after a failed chip reset attempt, so that a radio that keeps
/// failing still lets its caller run.
const RECOVERY_RETRY_US: u64 = 1_000_000;

/// The longest a scan may run.
const SCAN_BUDGET_US: u64 = 30_000_000;
/// The longest one firmware command may hold a station poll. Firmware
/// answers in a few milliseconds; polls run from guest exits, which must end
/// well inside the platform's exit bound (about 0.5 s), so a hung firmware
/// costs the radio, never the machine. Every driver's runtime commands use
/// this bound and report a command past it as a timeout, which loses the
/// radio through `Stage::Lost`. Boot-time attach keeps its own longer waits.
pub const RUNTIME_COMMAND_TIMEOUT_US: u64 = 50_000;
/// wpa_supplicant's default `scan_interval` after a scan finds nothing.
const SCAN_INTERVAL_US: u64 = 5_000_000;
/// The delay before reconnecting once the AP drops the association.
const RECONNECT_US: u64 = 100_000;
/// wpa_supplicant's 10 s authentication timeout around the handshake.
const HANDSHAKE_TIMEOUT_US: u64 = 10_000_000;
/// How long a station may take from its start to carrier: one whole scan,
/// then authentication and association, then the 4-way handshake within
/// its timeout. A wired PHY's wait would end before the scan does.
pub const CARRIER_WAIT_US: u64 = SCAN_BUDGET_US + 5_000_000 + HANDSHAKE_TIMEOUT_US;
/// The longest an SAE exchange may take, its commits, anti-clogging retry
/// and confirms included: the same 10 s wpa_supplicant allows.
const SAE_BUDGET_US: u64 = 10_000_000;
/// WLAN_REASON_DEAUTH_LEAVING, which `wpa_supplicant_timeout` sends.
const LEAVING: u16 = 3;

/// wpa_supplicant's `wpas_auth_failed` delay after `failures` consecutive
/// failed connections.
fn failure_delay_us(failures: u32) -> u64 {
    let seconds = match failures {
        51.. => 300,
        11.. => 120,
        6.. => 90,
        4.. => 60,
        3 => 30,
        2 => 20,
        _ => 10,
    };
    seconds * 1_000_000
}

/// `cfg80211_6ghz_power_type_valid`: whether a client may use the 6 GHz
/// `bss` under the flags its channel has among `channels`.
fn power_type_usable(bss: &Bss, channels: &[(Channel, u32)]) -> bool {
    let info = Operations::parse(&bss.elements)
        .ok()
        .and_then(|operations| operations.he?.six_ghz)
        .map(|six| (six[1] & 0x78) >> 3);
    let flags = channels
        .iter()
        .find(|&&(channel, _)| channel == bss.channel)
        .map(|&(_, flags)| flags);
    info.zip(flags)
        .is_some_and(|(info, flags)| PowerType::usable(info, flags))
}

/// The AP MLD `bss` is a link of, as its Basic Multi-Link element names it:
/// the AP MLD's address and the link's ID.
fn ap_mld(bss: &Bss) -> Option<(Mac, u8)> {
    let basic = Basic::find(&bss.elements).ok()??;
    Some((basic.address, basic.link_id?))
}

/// `mt76_select_links` beside the setup link `bss` of the AP MLD `ap`: the
/// first link by ID that the RNR of `bss` reports on another band, whose AP
/// `observed` holds as that link of the same AP MLD and `usable` takes.
fn other_link<'a, H: Copy>(
    bss: &Bss,
    ap: Mac,
    observed: &[(H, &'a Bss)],
    usable: impl Fn(&Bss) -> bool,
) -> Option<(u8, H, &'a Bss)> {
    let mut reported = affiliated(&bss.elements).ok()?;
    reported.sort_by_key(|link| link.link_id);
    reported.into_iter().find_map(|link| {
        observed
            .iter()
            .find(|(_, other)| {
                other.bssid == link.bssid
                    && other.channel.number == u16::from(link.channel)
                    && other.channel.band != bss.channel.band
                    && ap_mld(other) == Some((ap, link.link_id))
                    && usable(other)
            })
            .map(|&(handle, other)| (link.link_id, handle, other))
    })
}

/// The whole element `id` with `body`.
fn element(id: u8, body: &[u8]) -> Vec<u8> {
    let mut element = vec![id, body.len() as u8];
    element.extend_from_slice(body);
    element
}

/// Link `link_id` of an AP MLD association's handshakes, the station at
/// `local` and the AP of `bss`, as wpa_supplicant keeps it in `sm->mlo`.
fn keyed_link(link_id: u8, local: Mac, bss: &Bss) -> supplicant::Link {
    supplicant::Link {
        link_id,
        local,
        ap: bss.bssid,
        rsn: element(48, security::rsn_element(&bss.elements).unwrap_or_default()),
        rsnxe: connection::find(&bss.elements, 244).map(|body| element(244, body)),
        accepted: false,
    }
}

/// The handshake of an association with `bss` under `security` and `pmk`.
fn candidate(bss: &Bss, security: Security, pmk: Zeroizing<[u8; 32]>) -> Candidate {
    Candidate {
        bssid: bss.bssid,
        security,
        pmk,
        own_rsn: security.element(),
        peer_rsn: element(48, security::rsn_element(&bss.elements).unwrap_or_default()),
        links: None,
    }
}

/// The chosen BSS of the network.
struct Candidate {
    /// The authenticator: the BSS, or the AP MLD of a multi-link
    /// association.
    bssid: Mac,
    security: Security,
    /// The PMK the 4-way handshake proves: the passphrase's or the PSK, or
    /// the SAE exchange's.
    pmk: Zeroizing<[u8; 32]>,
    /// The station's whole RSN element.
    own_rsn: Vec<u8>,
    /// The AP's whole RSN element.
    peer_rsn: Vec<u8>,
    /// The links of a multi-link association.
    links: Option<MultiLink>,
}

/// The links of an AP MLD association, as `ieee80211_mgd_assoc` requests
/// them.
struct Links<H> {
    /// The setup link, which the SAE exchange and association run on.
    affiliation: Affiliation,
    /// The other link requested, on another band.
    other: Option<OtherLink<H>>,
}

/// Another link of an AP MLD association.
struct OtherLink<H> {
    link_id: u8,
    /// The station's random address on the link.
    local: Mac,
    handle: H,
    bss: Bss,
}

/// An SAE exchange with the chosen BSS, before its association.
struct Authenticating<H> {
    handle: H,
    bss: Bss,
    security: Security,
    session: Arc<external::Session>,
    /// The password element's derivation, a bounded share per poll, until
    /// the exchange's first commit goes out.
    deriving: Option<sae::Deriving>,
    exchange: Option<Exchange>,
    /// Whether the exchange derives its element by hash-to-element.
    h2e: bool,
    /// The AP MLD links the association requests.
    links: Option<Links<H>>,
}

enum State<H, M> {
    /// Scan once `at` passes.
    Waiting {
        at: u64,
    },
    /// A scan part runs; the parts after it follow in order.
    Scanning(Vec<scan::Request>),
    Authenticating(Box<Authenticating<H>>),
    Joining(Box<Candidate>),
    Handshake {
        supplicant: Box<Supplicant>,
        bssid: Mac,
        deadline: u64,
        /// Message 3 failed its MIC: the passphrase may be wrong.
        mic_failed: bool,
        /// The pairwise key message 3 installed.
        ptk: Option<KeyIdentity>,
        /// Message 3's IGTK until the link has drained the frames before it.
        integrity: Option<IntegrityKey>,
        /// Secondary-link IGTKs waiting for earlier frames and key users.
        link_integrity: Vec<(u8, IntegrityKey)>,
        /// The protected-management transport of the IGTK message 3 gave.
        management: Option<M>,
    },
    Connected {
        supplicant: Box<Supplicant>,
        bssid: Mac,
        ptk: KeyIdentity,
        management: Option<M>,
        /// A rekeyed IGTK the link had no room to rotate in yet.
        integrity: Option<IntegrityKey>,
        /// Secondary-link IGTKs waiting for earlier frames and key users.
        link_integrity: Vec<(u8, IntegrityKey)>,
    },
    /// Leave the association, then scan once `at` passes.
    Leaving {
        at: u64,
    },
}

/// How many diagnostics wait at most, and how many are remembered so
/// that each is queued once.
const MAX_NOTICES: usize = 32;

/// `WLAN_REASON_MICHAEL_MIC_FAILURE`.
const MICHAEL_FAILURE: u16 = 14;
/// How near two Michael failures start countermeasures, and how long they last.
const COUNTERMEASURES_US: u64 = 60_000_000;

/// A radio kept connected to one network.
pub struct Keeper<R: Radio> {
    link: R,
    network: Network,
    pmk: Zeroizing<[u8; 32]>,
    entropy: Arc<dyn Entropy>,
    clock: Clock,
    state: State<R::Handle, R::Management>,
    /// wpa_supplicant's `auth_failures` for the network.
    failures: u32,
    /// Diagnostics not taken yet, each message once.
    notices: VecDeque<String>,
    /// Every message queued so far, so that each is queued once.
    noticed: Vec<String>,
    /// The radio's latest error, the reason if it is lost.
    last_error: Option<String>,
    /// Why the radio stopped for good; the keeper no longer retries.
    lost: Option<String>,
    /// Recovery attempts that failed since the radio was lost.
    recovery_failures: u32,
    /// When the next recovery attempt may start.
    recover_at: u64,
    /// wpa_supplicant's `last_michael_mic_error`.
    michael_failed: Option<u64>,
    /// The last scan missed the network, so the next asks for it by name.
    directed: bool,
}

impl<R: Radio> Keeper<R> {
    /// Keep `link` connected to `network`; `entropy` gives each handshake's
    /// SNonce.
    pub fn keep(link: R, network: Network, entropy: Arc<dyn Entropy>, clock: Clock) -> Self {
        let pmk = network.secret.pmk(&network.ssid);
        Self {
            link,
            network,
            pmk,
            entropy,
            clock,
            state: State::Waiting { at: 0 },
            failures: 0,
            notices: VecDeque::new(),
            noticed: Vec::new(),
            last_error: None,
            lost: None,
            recovery_failures: 0,
            recover_at: 0,
            michael_failed: None,
            directed: false,
        }
    }

    /// The radio, for device models.
    #[cfg(test)]
    pub(crate) fn radio(&self) -> &R {
        &self.link
    }

    /// The radio, for device models.
    #[cfg(test)]
    pub(crate) fn radio_mut(&mut self) -> &mut R {
        &mut self.link
    }

    /// A diagnostic, queued for [`Link::take_diagnostic`] once per message, as
    /// wpa_supplicant logs an event: it never fails the link.
    fn fail(&mut self, message: String) {
        if self.noticed.contains(&message) {
            return;
        }
        if self.noticed.len() == MAX_NOTICES {
            self.noticed.remove(0);
        }
        self.noticed.push(message.clone());
        if self.notices.len() == MAX_NOTICES {
            self.notices.pop_front();
        }
        self.notices.push_back(message);
    }

    /// Why the radio stopped for good, once it has.
    pub fn lost(&self) -> Option<&str> {
        self.lost.as_deref()
    }

    /// The radio gave up on its hardware. A radio that can recover is reset
    /// step by step and joins again, as `mt7925_mac_reset_work` resets the
    /// chip and mac80211 reconnects. Only a failed attempt waits before the
    /// next; after [`RECOVERY_ATTEMPTS`] failures in a row the radio is lost
    /// for good, and so is one that cannot recover.
    fn recover(&mut self, now: u64) {
        if !self.link.recoverable() {
            self.lose();
            return;
        }
        if now < self.recover_at {
            return;
        }
        if let Some(reason) = self.last_error.take() {
            self.fail(reason);
        }
        match self.link.recover() {
            Ok(Recovery::Pending) => {}
            Ok(Recovery::Done) => {
                self.recovery_failures = 0;
                self.fail(format!(
                    "{}: recovered by a chip reset; joining again",
                    R::NAME
                ));
                self.state = State::Waiting { at: now };
            }
            Err(error) => {
                self.recovery_failures += 1;
                let message = format!(
                    "{}: chip reset {} of {RECOVERY_ATTEMPTS} failed: {error}",
                    R::NAME,
                    self.recovery_failures
                );
                if self.recovery_failures >= RECOVERY_ATTEMPTS {
                    self.last_error = Some(message);
                    self.lose();
                } else {
                    self.fail(message);
                    self.recover_at = now + RECOVERY_RETRY_US;
                }
            }
        }
    }

    /// Whether the radio is lost and waits for its recovery.
    fn recovering(&self) -> bool {
        self.lost.is_none() && self.link.recoverable() && self.link.stage() == Stage::Lost
    }

    /// The radio gave up on its hardware for good: report why once and stop
    /// retrying against it, as wpa_supplicant stops on an interface that
    /// is removed.
    fn lose(&mut self) {
        let reason = self
            .last_error
            .take()
            .unwrap_or_else(|| format!("{}: the radio stopped for good", R::NAME));
        self.fail(reason.clone());
        self.lost = Some(reason);
    }

    /// A connection attempt failed: leave it and wait as wpa_supplicant
    /// waits after `failures` attempts.
    fn failed(&mut self, now: u64, message: String) {
        self.failures = self.failures.saturating_add(1);
        self.fail(message);
        if self.link.stage() != Stage::Idle {
            let _ = self.link.deauthenticate(LEAVING);
        }
        self.state = State::Leaving {
            at: now + failure_delay_us(self.failures),
        };
    }

    /// Service the link, take its notices and advance the connection.
    fn poll(&mut self) {
        if self.lost.is_some() {
            return;
        }
        let now = self.clock.now_us();
        // A rotation waits for the frames queued before it to be taken.
        if let State::Connected {
            integrity: integrity @ Some(_),
            ..
        } = &mut self.state
        {
            let key = integrity.take().unwrap();
            if let Err(message) = self.rotate(key) {
                self.failed(now, message);
            }
        }
        if let Err(error) = self.link.service() {
            let message = format!("{error}");
            self.fail(message.clone());
            self.last_error = Some(message);
        }
        if self.link.stage() == Stage::Lost {
            self.recover(now);
            return;
        }
        while let Some(event) = self.link.take_event() {
            match event {
                Event::Eapol(ethernet) => self.eapol(&ethernet, now),
                Event::Sae(received) => self.sae(&received, now),
                Event::SaeEnded(failure) if matches!(self.state, State::Authenticating(_)) => {
                    self.failed(
                        now,
                        format!("{}: SAE authentication ended: {failure}", R::NAME),
                    );
                }
                Event::SaeEnded(_) => {}
                Event::Unsupported(operation) => {
                    self.fail(format!("{} unsupported: {operation}", R::NAME));
                }
                Event::ConnectionLost => {
                    self.fail(format!("{}: the AP ended the association", R::NAME));
                    self.state = State::Leaving {
                        at: now + RECONNECT_US,
                    };
                }
                Event::MichaelFailure { pairwise } => self.michael_failure(pairwise, now),
                // `service` reported why.
                Event::FirmwareLost => {
                    self.state = State::Leaving {
                        at: now + RECONNECT_US,
                    };
                }
            }
        }
        if let Err(message) = self.complete() {
            self.failed(now, message);
        }
        self.advance(now);
    }

    fn advance(&mut self, now: u64) {
        let stage = self.link.stage();
        if stage == Stage::Lost {
            self.recover(now);
            return;
        }
        match &self.state {
            State::Waiting { at } if now >= *at && stage == Stage::Idle => self.scan(now),
            State::Scanning(_) if stage == Stage::Scanned => self.scanned(now),
            State::Authenticating(authenticating) if authenticating.deriving.is_some() => {
                self.derive(now)
            }
            State::Authenticating(authenticating)
                if authenticating
                    .exchange
                    .as_ref()
                    .is_some_and(Exchange::accepted)
                    && self.link.station().external_authenticated() =>
            {
                self.associate(now)
            }
            State::Joining(_) if stage == Stage::Admitted => self.configure(now),
            State::Authenticating(_)
            | State::Joining(_)
            | State::Handshake { .. }
            | State::Connected { .. }
                if matches!(stage, Stage::Idle | Stage::NeedsDrain) =>
            {
                self.failed(
                    now,
                    format!("{}: authentication or association failed", R::NAME),
                );
            }
            State::Handshake {
                deadline,
                mic_failed,
                ..
            } if now >= *deadline => {
                let message = if *mic_failed {
                    "the 4-way handshake failed: the passphrase may be wrong"
                } else {
                    "the 4-way handshake timed out"
                };
                self.failed(now, format!("{}: {message}", R::NAME));
            }
            State::Leaving { at } => match stage {
                Stage::NeedsDrain => {
                    // SAFETY: as for the scan above, the departure's removal
                    // event was its last user, and the retirement refuses
                    // while any peer work is queued.
                    if let Err(error) = unsafe { self.link.retire_peer() } {
                        self.fail(format!("{error}"));
                    }
                }
                Stage::Idle => self.state = State::Waiting { at: *at },
                _ => {}
            },
            _ => {}
        }
    }

    /// `wpa_supplicant_req_scan`: a scan with the wildcard SSID on every
    /// channel that may transmit, 2.4 and 5 GHz first and 6 GHz after them as
    /// cfg80211 splits a scan. It probes where the channel permits and listens
    /// elsewhere, as mac80211 scans NO_IR channels passively within one active
    /// scan, so each runs in those two parts. `scan_ssid` has wpa_supplicant
    /// name a hidden network; with nobody to set that here, a scan names the
    /// network once the last one missed it, so that a visible network's name
    /// stays off the air.
    fn scan(&mut self, now: u64) {
        let mut ssids = vec![Vec::new()];
        if self.directed {
            ssids.push(self.network.ssid.clone());
        }
        let parts = [&[Band::Ghz2, Band::Ghz5][..], &[Band::Ghz6]]
            .into_iter()
            .flat_map(|bands| {
                let (active, passive): (Vec<_>, Vec<_>) = bands
                    .iter()
                    .flat_map(|&band| self.link.channels(band))
                    .map(|(channel, _)| channel)
                    .partition(|&channel| self.link.probes(channel));
                [(active, ssids.clone()), (passive, Vec::new())]
            })
            .filter(|(channels, _)| !channels.is_empty())
            .map(|(channels, ssids)| scan::Request::new(channels, ssids, Vec::new()))
            .collect::<Result<Vec<_>, _>>();
        match parts {
            Ok(parts) => self.next_scan(now, parts),
            Err(error) => {
                self.fail(format!("{} scan: {error:?}", R::NAME));
                self.state = State::Waiting {
                    at: now + SCAN_INTERVAL_US,
                };
            }
        }
    }

    /// A scan part completed: retire it, then run the next.
    fn scanned(&mut self, now: u64) {
        // SAFETY: the link is polled with the device's interrupts masked, so
        // the scan's completion event was its last user; the retirement
        // itself refuses while receive work is queued.
        if let Err(error) = unsafe { self.link.retire_scan() } {
            return self.fail(format!("{error}"));
        }
        if let State::Scanning(parts) = &mut self.state {
            let parts = core::mem::take(parts);
            self.next_scan(now, parts);
        }
    }

    /// Start the first of `parts`, or choose a BSS once none remains. A
    /// 6 GHz part scans what `cfg80211_scan_6ghz` plans from the BSSes the
    /// parts before it found, with wpa_supplicant's colocated 6 GHz scan.
    fn next_scan(&mut self, now: u64, mut parts: Vec<scan::Request>) {
        let part = loop {
            if parts.is_empty() {
                return self.choose(now);
            }
            let part = parts.remove(0);
            if part.channels()[0].band != Band::Ghz6 {
                break part;
            }
            let options = scan::six_ghz::Options {
                colocated: true,
                bssid: None,
            };
            match self.link.six_ghz_scan(&part, options) {
                Ok(Some(planned)) => break planned,
                Ok(None) => {}
                Err(error) => self.fail(format!("{error}")),
            }
        };
        self.state = match self.link.begin_scan(part, SCAN_BUDGET_US) {
            Ok(_) => State::Scanning(parts),
            Err(error) => {
                self.fail(format!("{error}"));
                State::Waiting {
                    at: now + SCAN_INTERVAL_US,
                }
            }
        };
    }

    /// Join the strongest BSS of the network whose security the station
    /// supports, after an SAE exchange when that is the security. A 6 GHz
    /// BSS whose power type its channel bars is not one cfg80211 connects to.
    fn choose(&mut self, now: u64) {
        let mut best: Option<(R::Handle, &Bss, Security)> = None;
        let mut refusal = None;
        let passphrase = matches!(self.network.secret, Secret::Passphrase(_));
        let six = self.link.channels(Band::Ghz6);
        let observed = self.link.bss();
        for &(handle, bss) in &observed {
            if bss.ssid.as_deref() != Some(&self.network.ssid[..]) {
                continue;
            }
            if bss.channel.band == Band::Ghz6 && !power_type_usable(bss, &six) {
                refusal = Some(format!(
                    "is on 6 GHz channel {} only as an AP whose power type the channel bars",
                    bss.channel.number
                ));
                continue;
            }
            match Security::select_on(bss.channel.band, &bss.elements, passphrase) {
                Ok(security) => {
                    let signal = |bss: &Bss| bss.signal_dbm.unwrap_or(i8::MIN);
                    if best
                        .as_ref()
                        .is_none_or(|(_, b, _)| signal(bss) > signal(b))
                    {
                        best = Some((handle, bss, security));
                    }
                }
                Err(reason) => {
                    refusal = Some(format!("offers no supported security: {reason:?}"));
                }
            }
        }
        let ssid = String::from_utf8_lossy(&self.network.ssid).into_owned();
        self.directed = best.is_none() && refusal.is_none();
        let Some((handle, bss, security)) = best else {
            self.fail(match refusal {
                Some(reason) => format!("{}: {ssid} {reason}", R::NAME),
                None => format!("{}: {ssid} not found", R::NAME),
            });
            self.state = State::Waiting {
                at: now + SCAN_INTERVAL_US,
            };
            return;
        };
        let bss = bss.clone();
        drop(observed);
        let started = if security.akm == Akm::Sae {
            self.authenticate(handle, bss, security)
                .map(State::Authenticating)
        } else {
            self.association(&bss, security, None)
                .and_then(|(join, profile)| {
                    self.link
                        .join(handle, join, profile)
                        .map_err(|error| format!("{error}"))?;
                    Ok(State::Joining(Box::new(candidate(
                        &bss,
                        security,
                        self.pmk.clone(),
                    ))))
                })
        };
        match started {
            Ok(state) => self.state = state,
            Err(message) => self.failed(now, message),
        }
    }

    /// `sme_send_authentication` for SAE: claim the BSS's peer and send the
    /// exchange's commit, its element by hash-to-element when the AP's RSNXE
    /// offers it.
    fn authenticate(
        &mut self,
        handle: R::Handle,
        bss: Bss,
        security: Security,
    ) -> Result<Box<Authenticating<R::Handle>>, String> {
        let Secret::Passphrase(passphrase) = &self.network.secret else {
            return Err(format!("{}: SAE needs the network's passphrase", R::NAME));
        };
        let (join, profile) = self.association(&bss, security, None)?;
        let links = self.links(&bss, &join.request)?;
        let session = if bss.channel.band == Band::Ghz6 {
            // The exchange binds the RSN element and RSNXE the association
            // will carry.
            let mut elements = security.element();
            elements.extend_from_slice(&sae::H2E_RSNXE);
            let selection = Selection::new(elements)
                .map_err(|error| format!("{} 6 GHz SAE selection: {error:?}", R::NAME))?;
            self.link.begin_six_ghz_external_authentication(
                handle,
                selection,
                profile,
                links.as_ref().map(|links| links.affiliation),
                SAE_BUDGET_US,
            )
        } else {
            self.link.begin_external_authentication(
                handle,
                profile,
                links.as_ref().map(|links| links.affiliation),
                SAE_BUDGET_US,
            )
        }
        .map_err(|error| format!("{error}"))?;
        let derivation = Derivation::for_rsnxe(connection::find(&bss.elements, 244));
        let deriving =
            sae::Deriving::for_session(&session, derivation, passphrase.as_bytes(), None)
                .map_err(|error| format!("{} SAE: {error:?}", R::NAME))?;
        Ok(Box::new(Authenticating {
            handle,
            bss,
            security,
            session,
            deriving: Some(deriving),
            exchange: None,
            h2e: derivation == Derivation::HashToElement,
            links,
        }))
    }

    /// One bounded share of the password element's derivation; the step
    /// that finds it sends the exchange's first commit.
    fn derive(&mut self, now: u64) {
        let State::Authenticating(authenticating) = &mut self.state else {
            return;
        };
        let Some(deriving) = authenticating.deriving.as_mut() else {
            return;
        };
        let entropy = &self.entropy;
        let mut unavailable = None;
        let mut random = || {
            let mut bytes = [0; 32];
            if let Err(error) = entropy.fill(&mut bytes) {
                unavailable = Some(error);
            }
            bytes
        };
        let stepped = deriving.step(&mut random);
        if let Some(error) = unavailable {
            return self.failed(now, format!("{} SAE: {error}", R::NAME));
        }
        let exchange = match stepped {
            Ok(Some(exchange)) => exchange,
            Ok(None) => return,
            Err(error) => return self.failed(now, format!("{} SAE: {error:?}", R::NAME)),
        };
        let commit = exchange.commit();
        authenticating.deriving = None;
        authenticating.exchange = Some(exchange);
        let session = authenticating.session.clone();
        if let Err(message) = self.send_sae(&session, commit) {
            self.failed(now, message);
        }
    }

    /// `ieee80211_mgd_assoc` for an AP MLD: when the NIC reports MLO and
    /// `bss` is a link of an AP MLD that `request` associates with as EHT
    /// with PMF, on 6 GHz only for a radio with `six_ghz_links`: the station
    /// takes a random address on it and requests one more link as
    /// `mt76_select_links` picks it.
    fn links(
        &self,
        bss: &Bss,
        request: &AssociationRequest,
    ) -> Result<Option<Links<R::Handle>>, String> {
        let Some((ap, link_id)) = ap_mld(bss) else {
            return Ok(None);
        };
        let multi_link = self
            .link
            .advertisement(bss.channel.band)
            .map_err(|error| format!("{error}"))?
            .multi_link;
        let six_ghz = self.link.six_ghz_links();
        if (bss.channel.band == Band::Ghz6 && !six_ghz)
            || multi_link != Some(true)
            || request.pmf().is_none()
            || request.modern().map(|modern| modern.mode()) != Some(Mode::Eht)
        {
            return Ok(None);
        }
        // The links run on channels that may transmit, and on 6 GHz under a
        // power type the channel allows.
        let usable = |other: &Bss| {
            let channels = self.link.channels(other.channel.band);
            match other.channel.band {
                Band::Ghz6 => six_ghz && power_type_usable(other, &channels),
                _ => channels
                    .iter()
                    .any(|&(channel, _)| channel == other.channel),
            }
        };
        let observed = self.link.bss();
        let other = other_link(bss, ap, &observed, usable)
            .map(|(link_id, handle, other)| (link_id, handle, other.clone()));
        let affiliation = Affiliation {
            mld: Mld {
                local: self.link.mac(),
                ap,
            },
            link_id,
            local: self.random_address()?,
        };
        let other = match other {
            Some((link_id, handle, bss)) => Some(OtherLink {
                link_id,
                local: self.random_address()?,
                handle,
                bss,
            }),
            None => None,
        };
        Ok(Some(Links { affiliation, other }))
    }

    /// `eth_random_addr`: a random locally administered unicast address.
    fn random_address(&self) -> Result<Mac, String> {
        let mut address = [0; 6];
        self.entropy
            .fill(&mut address)
            .map_err(|error| format!("{} link address: {error}", R::NAME))?;
        address[0] = (address[0] & 0xfe) | 0x02;
        Ok(Mac(address))
    }

    /// The multi-link setup of `links` with the AP MLD whose setup link is
    /// `bss`, each other link requested as it would associate alone, and the
    /// links the handshakes key. The station MLD reports mt7925's MLD
    /// capabilities, one simultaneous link, and the radio's EML capabilities.
    fn multi_link(
        &self,
        links: &Links<R::Handle>,
        bss: &Bss,
        security: Security,
        rsnxe: Option<&[u8]>,
    ) -> Result<(Setup, MultiLink), String> {
        let affiliation = links.affiliation;
        let mut requested = Vec::new();
        let mut keyed = vec![keyed_link(affiliation.link_id, affiliation.local, bss)];
        if let Some(other) = &links.other {
            let (join, _) = self.association(&other.bss, security, rsnxe)?;
            let observation = self
                .link
                .observation(other.handle)
                .map_err(|error| format!("{error}"))?;
            requested.push(mlo::Link {
                link_id: other.link_id,
                local: other.local,
                observation,
                request: join.request,
            });
            keyed.push(keyed_link(other.link_id, other.local, &other.bss));
        }
        Ok((
            Setup {
                association: affiliation,
                mld_capabilities: 0,
                eml_capabilities: self.link.eml_capabilities(),
                links: requested,
            },
            MultiLink {
                association: affiliation.link_id,
                links: keyed,
            },
        ))
    }

    /// Send one SAE frame of `session`.
    fn send_sae(&mut self, session: &external::Session, frame: sae::Frame) -> Result<(), String> {
        self.link
            .transmit_external_authentication(
                session,
                external::Frame {
                    transaction: frame.transaction,
                    status: frame.status,
                    data: &frame.data,
                },
            )
            .map(|_| ())
            .map_err(|error| format!("{error}"))
    }

    /// One SAE frame from the AP: the exchange's answer goes back to it.
    fn sae(&mut self, received: &Received, now: u64) {
        let State::Authenticating(authenticating) = &mut self.state else {
            return;
        };
        if !Arc::ptr_eq(received.session(), &authenticating.session) {
            return;
        }
        // Nothing the AP sends precedes the station's first commit.
        let Some(exchange) = authenticating.exchange.as_mut() else {
            return;
        };
        let reply = match exchange.receive_from(received) {
            Ok(reply) => reply,
            Err(sae::Error::Confirm) => {
                return self.failed(
                    now,
                    format!(
                        "{}: SAE confirm failed: the passphrase may be wrong",
                        R::NAME
                    ),
                );
            }
            Err(error) => return self.failed(now, format!("{} SAE: {error:?}", R::NAME)),
        };
        let session = authenticating.session.clone();
        if let Some(frame) = reply
            && let Err(message) = self.send_sae(&session, frame)
        {
            self.failed(now, message);
        }
    }

    /// `sme_associate` once SAE authenticated the station: its PMK keys the
    /// handshake, and the request carries no PMKID (`sae_pmkid_in_assoc`).
    fn associate(&mut self, now: u64) {
        let State::Authenticating(authenticating) =
            core::mem::replace(&mut self.state, State::Scanning(Vec::new()))
        else {
            return;
        };
        let Authenticating {
            handle,
            bss,
            security,
            session,
            exchange,
            h2e,
            links,
            ..
        } = *authenticating;
        let Some(pmk) = exchange.as_ref().and_then(Exchange::pmk) else {
            return self.failed(now, format!("{}: SAE gave no PMK", R::NAME));
        };
        let rsnxe = h2e.then_some(&sae::H2E_RSNXE[..]);
        let started =
            self.association(&bss, security, rsnxe)
                .and_then(|(mut join, _)| {
                    let mut candidate = candidate(&bss, security, pmk);
                    if let Some(links) = &links {
                        let (setup, keyed) = self.multi_link(links, &bss, security, rsnxe)?;
                        join.request = join.request.with_multi_link(setup).map_err(|error| {
                            format!("{} multi-link request: {error:?}", R::NAME)
                        })?;
                        candidate.bssid = links.affiliation.mld.ap;
                        candidate.links = Some(keyed);
                    }
                    self.link
                        .associate_external(&session, handle, join)
                        .map_err(|error| format!("{error}"))?;
                    Ok(candidate)
                });
        match started {
            Ok(candidate) => self.state = State::Joining(Box::new(candidate)),
            Err(message) => self.failed(now, message),
        }
    }

    /// `ieee80211_mgd_assoc`: the association request and station profile
    /// for `bss` on mac80211's mode and channel, its RSN element with an SAE
    /// exchange's `rsnxe`.
    fn association(
        &self,
        bss: &Bss,
        security: Security,
        rsnxe: Option<&[u8]>,
    ) -> Result<(Join, R::Profile), String> {
        let band = bss.channel.band;
        let advertisement = self
            .link
            .advertisement(band)
            .map_err(|error| format!("{error}"))?;
        let peer = Capabilities::parse(band, Role::AccessPoint, &bss.elements)
            .map_err(|error| format!("{} AP capabilities: {error:?}", R::NAME))?;
        let operations = Operations::parse(&bss.elements)
            .map_err(|error| format!("{} AP operation: {error:?}", R::NAME))?;
        let channels = self.link.channels(band);
        let chosen = mode::determine(
            &advertisement.capabilities,
            &peer,
            &operations,
            bss.channel,
            Channels(&channels),
        )
        .map_err(|refusal| format!("{}: {refusal:?}", R::NAME))?;
        let spectrum = connection::spectrum_management(bss.capabilities).then(|| {
            let power = self.link.regulated_power_dbm(bss.channel.frequency_mhz);
            (
                power.clamp(0, 127) as i8,
                self.link.supported_channels(band),
            )
        });
        let own_rsn = security.element();
        let elements = connection::request_elements(
            &Request {
                ssid: &self.network.ssid,
                rsn: &own_rsn,
                rsnxe,
                mode: chosen.mode,
                wmm: peer.wmm().is_some(),
                spectrum,
            },
            &advertisement.elements,
        );
        let mut local = Capabilities::parse(band, Role::Station, &elements)
            .map_err(|error| format!("{} request capabilities: {error:?}", R::NAME))?;
        if local.he().is_some() {
            local = local
                .with_local_he_source_maps(advertisement.he_source_maps)
                .map_err(|error| format!("{} request capabilities: {error:?}", R::NAME))?;
        }
        let rates = local.rates().supported & peer.rates().supported;
        let capability = connection::capability(band, chosen.mode, spectrum.is_some());
        let width = chosen.definition.width;
        let request = if security.management.is_some() {
            AssociationRequest::new_modern_with_local_pmf(
                capability,
                R::LISTEN_INTERVAL,
                elements,
                local,
                chosen.mode,
                width,
            )
        } else {
            AssociationRequest::new_modern_with_local(
                capability,
                R::LISTEN_INTERVAL,
                elements,
                local,
                chosen.mode,
                width,
            )
        }
        .map_err(|error| format!("{} association request: {error:?}", R::NAME))?;
        let dtim_period = connection::find(&bss.elements, 5)
            .and_then(|tim| tim.get(1).copied())
            .filter(|&period| period != 0)
            .unwrap_or(1);
        // mac80211 uses the short slot time on 5 GHz and wherever the AP does.
        let slot_time = if band != Band::Ghz2 || bss.capabilities & (1 << 10) != 0 {
            9
        } else {
            20
        };
        let profile = self
            .link
            .profile(Peer {
                bssid: bss.bssid,
                channel: bss.channel,
                basic_rates: peer.rates().basic,
                supported_rates: rates,
                beacon_interval: bss.beacon_interval,
                dtim_period,
                slot_time,
            })
            .map_err(|error| format!("{error}"))?;
        let join = Join {
            bssid: bss.bssid,
            channel: bss.channel,
            policy: PortPolicy::Controlled,
            request,
        };
        Ok((join, profile))
    }

    /// Configure the association the AP accepted, then await message 1.
    fn configure(&mut self, now: u64) {
        let State::Joining(candidate) =
            core::mem::replace(&mut self.state, State::Scanning(Vec::new()))
        else {
            return;
        };
        if let Err(error) = self.link.configure() {
            return self.failed(now, format!("{error}"));
        }
        let mut links = candidate.links;
        if let Some(links) = &mut links {
            // The links the AP MLD accepted, the setup link among them.
            let accepted: Vec<u8> = self
                .link
                .station()
                .link_contexts()
                .iter()
                .filter_map(|context| context.affiliation())
                .map(|affiliation| affiliation.link_id)
                .chain([links.association])
                .collect();
            for link in &mut links.links {
                link.accepted = accepted.contains(&link.link_id);
            }
        }
        match Supplicant::new(
            candidate.security.suites(),
            candidate.pmk,
            self.link.mac(),
            candidate.bssid,
            candidate.own_rsn,
            candidate.peer_rsn,
        )
        .and_then(|supplicant| match links {
            Some(links) => supplicant.with_multi_link(links),
            None => Ok(supplicant),
        }) {
            Ok(supplicant) => {
                self.state = State::Handshake {
                    supplicant: Box::new(supplicant),
                    bssid: candidate.bssid,
                    deadline: now + HANDSHAKE_TIMEOUT_US,
                    mic_failed: false,
                    ptk: None,
                    integrity: None,
                    link_integrity: Vec::new(),
                    management: None,
                }
            }
            Err(error) => self.failed(now, format!("{} supplicant: {error:?}", R::NAME)),
        }
    }

    /// One EAPOL frame from the AP: answer it, install the keys it carries
    /// and, once message 3 has, open the port.
    fn eapol(&mut self, ethernet: &[u8], now: u64) {
        if ethernet.get(12..14) != Some(&EAPOL.to_be_bytes()[..]) {
            return;
        }
        let mut nonce = [0; 32];
        if let Err(error) = self.entropy.fill(&mut nonce) {
            return self.failed(now, format!("{} SNonce: {error}", R::NAME));
        }
        let (supplicant, bssid, ptk) = match &mut self.state {
            State::Handshake {
                supplicant, bssid, ..
            } => (supplicant, *bssid, None),
            State::Connected {
                supplicant,
                bssid,
                ptk,
                ..
            } => (supplicant, *bssid, Some(*ptk)),
            _ => return,
        };
        let step = match supplicant.receive(&ethernet[14..], &mut || nonce) {
            Ok(step) => step,
            // wpa_supplicant drops a frame that does not verify.
            Err(error) => {
                if let (crate::wifi::supplicant::Error::Mic, State::Handshake { mic_failed, .. }) =
                    (error, &mut self.state)
                {
                    *mic_failed = true;
                }
                return;
            }
        };
        if let Err(message) = self.apply(step, bssid, ptk) {
            self.failed(now, message);
        }
    }

    /// `wpa_supplicant_event_michael_mic_failure`: the AP hears of each
    /// failure, and a second within a minute leaves it for a minute.
    fn michael_failure(&mut self, pairwise: bool, now: u64) {
        let State::Connected {
            supplicant,
            bssid,
            ptk,
            ..
        } = &mut self.state
        else {
            return;
        };
        let (bssid, ptk) = (*bssid, *ptk);
        let step = Step {
            reply: supplicant.michael_report(pairwise),
            ..Step::default()
        };
        if let Err(message) = self.apply(step, bssid, Some(ptk)) {
            return self.failed(now, message);
        }
        let last = self.michael_failed.replace(now);
        if last.is_some_and(|last| now < last + COUNTERMEASURES_US) {
            self.fail(format!("{}: TKIP countermeasures started", R::NAME));
            let _ = self.link.deauthenticate(MICHAEL_FAILURE);
            self.state = State::Leaving {
                at: now + COUNTERMEASURES_US,
            };
        }
    }

    /// Send a handshake step's reply, then install its keys as
    /// wpa_supplicant does after message 4.
    fn apply(&mut self, step: Step, bssid: Mac, ptk: Option<KeyIdentity>) -> Result<(), String> {
        if let Some(reply) = step.reply {
            let mut frame = Vec::with_capacity(14 + reply.len());
            frame.extend_from_slice(&bssid.0);
            frame.extend_from_slice(&self.link.mac().0);
            frame.extend_from_slice(&EAPOL.to_be_bytes());
            frame.extend_from_slice(&reply);
            self.link
                .send_control(&frame, ptk)
                .map_err(|error| format!("{error}"))?;
        }
        let pairwise = step
            .pairwise
            .map(|key| {
                self.link
                    .install_key(true, key.index, &key.material, key.receive)
            })
            .transpose()
            .map_err(|error| format!("{error}"))?;
        if let Some(key) = step.group {
            self.link
                .install_key(false, key.index, &key.material, key.receive)
                .map_err(|error| format!("{error}"))?;
        }
        // Each other link's GTK goes to its AP's broadcast WCID.
        for link in step.links {
            let key = link.group;
            self.link
                .install_link_key(link.link_id, key.index, &key.material, key.receive)
                .map_err(|error| format!("{error}"))?;
            if let Some(key) = link.integrity {
                let pending = match &mut self.state {
                    State::Handshake { link_integrity, .. }
                    | State::Connected { link_integrity, .. } => link_integrity,
                    _ => return Err(format!("{}: link keys outside the handshake", R::NAME)),
                };
                // A retransmitted authenticated KDE replaces only a pending,
                // unsubmitted update for the same link and management index.
                if let Some(position) = pending
                    .iter()
                    .position(|(id, old)| *id == link.link_id && old.index == key.index)
                {
                    pending.remove(position);
                }
                pending.push((link.link_id, key));
            }
        }
        match &mut self.state {
            State::Handshake { ptk, integrity, .. } => {
                *ptk = pairwise.or(*ptk);
                *integrity = step.integrity.or(integrity.take());
            }
            State::Connected { ptk, .. } => {
                *ptk = pairwise.unwrap_or(*ptk);
                if let Some(key) = step.integrity {
                    self.rotate(key)?;
                }
            }
            _ => {}
        }
        self.complete()
    }

    /// Keep secondary IGTKs pending across native TX/notices and key commands,
    /// just as the setup link's IGTK waits for message 4 to drain.
    fn complete_link_management_keys(&mut self) -> Result<bool, String> {
        loop {
            let pending = match &mut self.state {
                State::Handshake { link_integrity, .. }
                | State::Connected { link_integrity, .. } => link_integrity,
                _ => return Ok(true),
            };
            let Some((link_id, key)) = pending.first() else {
                return Ok(true);
            };
            // SAFETY: the authenticated handshake supplied this link's KDE.
            // The sole polled radio owner serializes key publication with all
            // prior hardware/RX/IRQ/work and deferred users; a busy native
            // queue preserves the key here until the driver reports retirement.
            match unsafe {
                self.link.install_link_management_key(
                    *link_id,
                    key.index,
                    &key.material,
                    key.receive,
                )
            } {
                Ok(kept) => {
                    pending.remove(0);
                    if !kept {
                        self.fail(format!("{} unsupported: MLO link IGTKs; another link's group-addressed robust management frames reach the OS unverified", R::NAME));
                    }
                }
                Err(error) if R::deferred(&error) => return Ok(false),
                Err(error) => return Err(format!("{error}")),
            }
        }
    }

    /// Install message 3's IGTK once the link drained the frames before it,
    /// message 4 among them, then open the port.
    fn complete(&mut self) -> Result<(), String> {
        if !self.complete_link_management_keys()? {
            return Ok(());
        }
        let State::Handshake {
            ptk: Some(ptk),
            integrity,
            management,
            ..
        } = &mut self.state
        else {
            return Ok(());
        };
        let (ptk, pending, protected) = (*ptk, integrity.take(), management.is_some());
        let epoch = self
            .link
            .station()
            .association_epoch()
            .ok_or_else(|| format!("{}: the association ended", R::NAME))?;
        let installed = match pending {
            // SAFETY: this association's authenticated handshake gave the
            // IGTK, and the link is polled with the device's interrupts
            // masked, so no earlier key user remains.
            Some(key) => match unsafe {
                self.link
                    .install_management_key(epoch, key.index, &key.material, key.receive)
            } {
                Ok(session) => {
                    // The port may yet wait for the key's firmware answer.
                    if let State::Handshake { management, .. } = &mut self.state {
                        *management = Some(session);
                    }
                    true
                }
                Err(error) if R::deferred(&error) => {
                    if let State::Handshake { integrity, .. } = &mut self.state {
                        *integrity = Some(key);
                    }
                    return Ok(());
                }
                Err(error) => return Err(format!("{error}")),
            },
            None => false,
        };
        if !installed && !protected && self.link.station().requires_protected_management() {
            return Ok(());
        }
        match self.link.authorize_port(epoch) {
            Ok(()) => {}
            Err(error) if R::deferred(&error) => return Ok(()),
            Err(error) => return Err(format!("{error}")),
        }
        let State::Handshake {
            supplicant,
            bssid,
            management,
            ..
        } = core::mem::replace(&mut self.state, State::Scanning(Vec::new()))
        else {
            unreachable!("matched above")
        };
        self.state = State::Connected {
            supplicant,
            bssid,
            ptk,
            management,
            integrity: None,
            link_integrity: Vec::new(),
        };
        self.failures = 0;
        Ok(())
    }

    /// Rotate the protected-management transport to a rekeyed IGTK, once the
    /// link holds no frame received before it.
    fn rotate(&mut self, key: IntegrityKey) -> Result<(), String> {
        let State::Connected {
            management,
            integrity,
            ..
        } = &mut self.state
        else {
            return Ok(());
        };
        let session = management
            .as_ref()
            .ok_or_else(|| format!("{}: an IGTK without management protection", R::NAME))?;
        // SAFETY: as for the initial IGTK.
        match unsafe {
            self.link
                .rotate_management_key(session, key.index, &key.material, key.receive)
        } {
            Ok(session) => *management = Some(session),
            Err(error) if R::deferred(&error) => *integrity = Some(key),
            Err(error) => return Err(format!("{error}")),
        }
        Ok(())
    }
}

impl<R: Radio> Link for Keeper<R> {
    fn mac(&self) -> Mac {
        self.link.mac()
    }
    fn carrier(&mut self) -> bool {
        self.poll();
        self.lost.is_none() && self.link.carrier()
    }
    fn carrier_wait_us(&self) -> u64 {
        CARRIER_WAIT_US
    }
    /// Each distinct message once.
    fn take_diagnostic(&mut self) -> Option<String> {
        self.notices.pop_front()
    }
    /// A recovering radio is polled for its steps.
    fn signals(&self) -> bool {
        self.lost.is_none() && !self.recovering() && self.link.signals()
    }
    fn due_us(&self) -> Option<u64> {
        self.lost.is_none().then(|| self.link.due_us()).flatten()
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        if let Some(lost) = &self.lost {
            return Err(LinkError::Adapter(lost.clone()));
        }
        if self.recovering() {
            return Err(LinkError::NoCarrier);
        }
        match self.link.transmit(frame) {
            // The next poll recovers the radio.
            Err(_) if self.recovering() => Err(LinkError::NoCarrier),
            result => result,
        }
    }
    /// A notice is never an error here: only a radio lost for good, or the
    /// radio's own receive error, is.
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.poll();
        if let Some(lost) = &self.lost {
            return Err(LinkError::Adapter(lost.clone()));
        }
        if self.recovering() {
            return Ok(None);
        }
        match self.link.receive() {
            // The next poll recovers the radio.
            Err(_) if self.recovering() => Ok(None),
            result => result,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests;
