// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2020-2023 MediaTek Inc.
// Channel geometry/policy follows Linux cfg80211/mac80211; see NOTICE.
//! Concrete policy owner for the MT7925 controller. The normal
//! OS/setup input is a pinned database and explicit country/environment. This
//! wrapper owns the only client and exposes no raw mutable escape. Every method
//! serializes policy validation with publication; revocation performs
//! real cancel/remove/disable work and keeps all resources through drain.

use super::*;
use core::sync::atomic::{AtomicBool, Ordering};
use reg::{channel, database::Country};
use station::phy::{Mode, Width};
use triomphe::Arc as Shared;
use wifi::{Band, Channel, regulatory as reg};

mod external;
mod link;
mod management;
pub use link::{
    LinkAttachError, LinkControlError, LinkEvent, LinkNotice, LinkProgress, ManagedLink,
};

static NEXT_OWNER: wifi::Identities = wifi::Identities::new();
static NEXT_BSS: wifi::Identities = wifi::Identities::new();
/// IEEE80211_SCAN_RESULT_EXPIRE: how long a scan's BSS stays known.
const BSS_EXPIRE_US: u64 = 30_000_000;

/// Where the station operates, as cfg80211's `reg_is_indoor` knows it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Environment {
    /// The user says it is indoors: indoor-only channels may initiate.
    Indoor,
    /// `ENVIRON_ANY`: indoor-only channels never initiate.
    Any,
}
#[derive(Debug)]
pub enum PolicyError {
    Client(Error),
    Regulatory(reg::Error),
    NoMemory,
    IncompleteProgramming,
    CountryMismatch,
    RestrictionMismatch,
    PowerMismatch,
    Revoked,
    Bss,
}
/// Failed attachment preserves the original native owner for correction/drain.
pub struct AttachError<I: BarIo, Q: QueueIo, C: 'static> {
    pub error: PolicyError,
    pub client: Client<I, Q, C>,
}
impl<I: BarIo, Q: QueueIo, C: 'static> core::fmt::Debug for AttachError<I, Q, C> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AttachError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BssHandle(u64);
struct Observed {
    handle: BssHandle,
    seen: u64,
    bss: scan::result::Bss,
}
#[derive(Clone, Copy)]
struct Programmed {
    channel: Channel,
    flags: u32,
    /// The highest rate power programmed for it; `None` when the source
    /// rate-power channel list has no row for it, so nothing transmits there.
    max_half_dbm: Option<i8>,
}
struct Snapshot {
    context: reg::NativeContext,
    channels: Vec<Programmed>,
    // All source power rows include wide/center-channel indices not
    // present in the domain's enabled20MHz channel list.
    powers: Vec<(Channel, i8)>,
}
struct Hardware(Shared<Snapshot>, Environment);
/// The OS's protection of the current association: it joined a PMF network
/// (`selected`), then installed the pairwise key its 4-way handshake
/// derived (`authenticated`), as wpa_supplicant does through NL80211_CMD_NEW_KEY.
#[derive(Default)]
struct Protection {
    selected: AtomicBool,
    authenticated: AtomicBool,
}
impl Protection {
    fn set(&self, selected: bool, authenticated: bool) {
        self.selected.store(selected, Ordering::Release);
        self.authenticated.store(authenticated, Ordering::Release);
    }
    fn selected(&self) -> bool {
        self.selected.load(Ordering::Acquire)
    }
    fn authenticated(&self) -> bool {
        self.selected() && self.authenticated.load(Ordering::Acquire)
    }
}
struct Os {
    country: Shared<Country>,
    snapshot: Shared<Snapshot>,
    environment: Environment,
    context: reg::NativeContext,
    protection: Shared<Protection>,
}
struct HardwareGuard<'a>(&'a Snapshot, Environment);
struct OsGuard<'a>(&'a Os);
struct Check<'a> {
    snapshot: &'a Snapshot,
    country: &'a Country,
    environment: Environment,
    revoked: bool,
    peer: Option<Channel>,
}

/// Fully owning, source-programmed single-interface client. The underlying
/// Controller/Execution keeps board/SAR/CLC inputs under their existing native
/// bootstrap contract. The selected country must match its programmed
/// domain; a new country requires stop/drain and a new programmed attachment.
pub struct ManagedClient<I: BarIo, Q: QueueIo, C: 'static> {
    client: Client<I, Q, C>,
    authority: reg::Owner<Hardware, Os>,
    snapshot: Shared<Snapshot>,
    country: Shared<Country>,
    environment: Environment,
    revoked: bool,
    peer: Option<Channel>,
    observed: Vec<Observed>,
    /// The OS's interworking QoS Map, `sdata->qos_map`.
    qos_map: Option<station::data::QosMap>,
    /// The OS's power save setting, `ifmgd->powersave`.
    power_save: bool,
    protection: Shared<Protection>,
}
fn copy_bss(b: &scan::result::Bss) -> Result<scan::result::Bss, PolicyError> {
    fn copy(v: &[u8]) -> Result<Vec<u8>, PolicyError> {
        let mut out = Vec::new();
        out.try_reserve_exact(v.len())
            .map_err(|_| PolicyError::NoMemory)?;
        out.extend_from_slice(v);
        Ok(out)
    }
    Ok(scan::result::Bss {
        ticket: b.ticket,
        kind: b.kind,
        bssid: b.bssid,
        transmitter: b.transmitter,
        timestamp: b.timestamp,
        beacon_interval: b.beacon_interval,
        capabilities: b.capabilities,
        channel: b.channel,
        observed: b.observed.clone(),
        signal_dbm: b.signal_dbm,
        ssid: b.ssid.as_deref().map(copy).transpose()?,
        elements: copy(&b.elements)?,
        profile: b.profile,
    })
}
fn channel(band: Band, n: u16) -> Option<Channel> {
    let mhz = match band {
        Band::Ghz2 if (1..=13).contains(&n) => 2407 + 5 * n,
        Band::Ghz2 if n == 14 => 2484,
        Band::Ghz5 if (182..=196).contains(&n) => 4000 + 5 * n,
        Band::Ghz5 if n > 0 && n <= 177 => 5000 + 5 * n,
        Band::Ghz6 if n == 2 => 5935,
        Band::Ghz6 if (1..=233).contains(&n) => 5950 + 5 * n,
        _ => return None,
    };
    Some(Channel {
        band,
        number: n,
        frequency_mhz: mhz,
    })
}
fn sku_max(sku: &[u8], band: Band) -> Option<i8> {
    if sku.len() != 449 {
        return None;
    }
    let mut result = i8::MIN;
    for (i, &raw) in sku.iter().enumerate() {
        let used = (band == Band::Ghz2 && i < 4)
            || (4..12).contains(&i)
            || (44..61).contains(&i)
            || (61..109).contains(&i) && (i - 61) % 12 < 10
            || (109..449).contains(&i);
        if used {
            result = result.max(raw as i8);
        }
    }
    Some(result)
}
impl Snapshot {
    fn read<I: BarIo, Q: QueueIo, C: 'static>(
        client: &Client<I, Q, C>,
        country: &Country,
    ) -> Result<Self, PolicyError> {
        let c = client.state().controller.state();
        if client.phase() != Phase::Idle
            || client.state().controller.phase() != ControllerPhase::StartFacts
        {
            return Err(PolicyError::IncompleteProgramming);
        }
        let (domain, power) = c
            .policy
            .as_ref()
            .and_then(|p| p.programmed_mt7925())
            .ok_or(PolicyError::IncompleteProgramming)?;
        if domain.alpha2() != country.alpha2() || power.alpha2() != country.alpha2() {
            return Err(PolicyError::CountryMismatch);
        }
        let mut powers = Vec::new();
        let n = power
            .requests()
            .iter()
            .map(|r| r.body().len().saturating_sub(52) / 450)
            .sum();
        powers
            .try_reserve_exact(n)
            .map_err(|_| PolicyError::NoMemory)?;
        for request in power.requests() {
            let body = request.body();
            if request.kind() != wifi::mediatek::mcu::power::Kind::RatePower {
                return Err(PolicyError::IncompleteProgramming);
            }
            let band = match body.get(13) {
                Some(1) => Band::Ghz2,
                Some(2) => Band::Ghz5,
                Some(3) => Band::Ghz6,
                _ => return Err(PolicyError::IncompleteProgramming),
            };
            for raw in body
                .get(52..)
                .ok_or(PolicyError::IncompleteProgramming)?
                .as_chunks::<450>()
                .0
            {
                let c = channel(band, u16::from(raw[0])).ok_or(PolicyError::PowerMismatch)?;
                let max = sku_max(&raw[1..], band).ok_or(PolicyError::PowerMismatch)?;
                if powers.iter().any(|&(p, _)| p == c) {
                    return Err(PolicyError::PowerMismatch);
                }
                powers.push((c, max));
            }
        }
        let counts = domain.counts();
        let mut channels = Vec::new();
        channels
            .try_reserve_exact(counts.iter().map(|&n| usize::from(n)).sum())
            .map_err(|_| PolicyError::NoMemory)?;
        let mut entries = domain.body()[16..].as_chunks::<8>().0.iter();
        for (band, count) in [Band::Ghz2, Band::Ghz5, Band::Ghz6].into_iter().zip(counts) {
            for _ in 0..count {
                let e = entries.next().ok_or(PolicyError::IncompleteProgramming)?;
                let channel = channel(band, u16::from_le_bytes([e[0], e[1]]))
                    .ok_or(PolicyError::RestrictionMismatch)?;
                let flags = u32::from_le_bytes(e[4..8].try_into().unwrap());
                // The programmed channel keeps every restriction cfg80211
                // gives it under the country, and its power stays within the
                // regulatory limit mt76_connac_get_ch_power applies.
                let required = country.channel(channel.frequency_mhz);
                if (flags | required.flags) & reg::channel::DISABLED != 0
                    || flags & required.flags != required.flags
                    || channels.iter().any(|c: &Programmed| c.channel == channel)
                {
                    return Err(PolicyError::RestrictionMismatch);
                }
                let max_half_dbm = powers
                    .iter()
                    .find(|&&(c, _)| c == channel)
                    .map(|&(_, max)| max);
                if max_half_dbm.is_some_and(|max| i32::from(max) > 2 * required.max_reg_dbm) {
                    return Err(PolicyError::PowerMismatch);
                }
                channels.push(Programmed {
                    channel,
                    flags,
                    max_half_dbm,
                });
            }
        }
        let id = NEXT_OWNER
            .next()
            .ok_or(PolicyError::Regulatory(reg::Error::OwnerExhausted))?;
        Ok(Self {
            context: reg::NativeContext {
                device: id,
                reset_epoch: id,
                firmware_epoch: id,
                policy_revision: id,
            },
            channels,
            powers,
        })
    }
    fn footprint_power(&self, def: station::operations::ChannelDefinition) -> Option<i8> {
        let bw = match def.width {
            Width::Mhz20 => 20,
            Width::Mhz40 => 40,
            Width::Mhz80 => 80,
            Width::Mhz160 => 160,
            _ => return None,
        };
        let start = def.center1_mhz.checked_sub(bw / 2)?;
        let end = def.center1_mhz.checked_add(bw / 2)?;
        // Include every programmed primary/secondary/center SKU that
        // firmware can select within this footprint. No guessed mapping of a
        // firmware wide-rate slot can bypass the most restrictive rule. The
        // 10MHz center grid excludes overlapping neighboring 2GHz 20MHz channels
        // that are not part of the negotiated footprint.
        self.powers
            .iter()
            .filter(|&&(ch, _)| {
                ch.band == def.control.band
                    && u32::from(ch.frequency_mhz) > start
                    && u32::from(ch.frequency_mhz) < end
                    && ch
                        .frequency_mhz
                        .abs_diff(def.control.frequency_mhz)
                        .is_multiple_of(10)
            })
            .map(|&(_, max)| max)
            .max()
    }
    fn get(&self, ch: Channel) -> Option<&Programmed> {
        self.channels.iter().find(|c| c.channel == ch)
    }
    fn association(&self, op: &reg::AssociationOperation<'_>, environment: Environment) -> bool {
        let ctx = op.context();
        let def = op.definition();
        let n = ctx.negotiated();
        if def.validate().is_err() || matches!(def.width, Width::Mhz80P80 | Width::Mhz320) {
            return false;
        }
        let bw = match def.width {
            Width::Mhz20 => 20,
            Width::Mhz40 => 40,
            Width::Mhz80 => 80,
            Width::Mhz160 => 160,
            _ => return false,
        };
        let start = def.center1_mhz - bw / 2;
        for mhz in (start + 10..start + bw).step_by(20) {
            // Punctured channels remain checked, matching cfg80211 chandef
            // usability; puncturing cannot erase a disabled/no-width rule.
            let Some(c) = self.channels.iter().find(|c| {
                c.channel.band == def.control.band && u32::from(c.channel.frequency_mhz) == mhz
            }) else {
                return false;
            };
            let f = c.flags;
            if f & ((1 << 11) | 1) != 0
                || f & (1 << 9) != 0 && environment != Environment::Indoor
                || f & (1 << 6) != 0
                    && (n.mode != Mode::Legacy
                        || def.control.band != Band::Ghz2
                        || n.legacy.supported & !15 != 0)
                || matches!(n.mode, Mode::He | Mode::Eht) && f & (1 << 13) != 0
                || n.mode == Mode::Eht && f & (1 << 20) != 0
                || bw >= 80 && f & (1 << 7) != 0
                || bw >= 160 && f & (1 << 8) != 0
            {
                return false;
            }
        }
        if bw >= 40 {
            let Some(control) = self.get(def.control) else {
                return false;
            };
            let secondary_above =
                ((u32::from(def.control.frequency_mhz) - start) / 20).is_multiple_of(2);
            if control.flags & (if secondary_above { 1 << 4 } else { 1 << 5 }) != 0 {
                return false;
            }
        }
        true
    }
}
impl reg::HardwareLease for HardwareGuard<'_> {
    fn context(&self) -> reg::NativeContext {
        self.0.context
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
        self.0
            .channels
            .iter()
            .find(|c| c.channel.band == band && c.channel.number == number)
            .map(|c| c.channel)
    }
    fn allows(&mut self, op: reg::Operation) -> bool {
        let Some(c) = self.0.get(op.channel()) else {
            return false;
        };
        match op {
            reg::Operation::Receive(_) => true,
            reg::Operation::Transmit {
                purpose, profile, ..
            } => {
                c.flags & ((1 << 11) | 1) == 0
                    && (c.flags & (1 << 9) == 0 || self.1 == Environment::Indoor)
                    && (purpose != reg::Purpose::Probe || c.flags & (2 | 8) == 0)
                    && c.max_half_dbm
                        .is_some_and(|max| i32::from(max) * 50 <= profile.max_eirp_mbm)
            }
        }
    }
    fn allows_pmf_authentication(&mut self, op: &reg::PmfAuthenticationOperation<'_>) -> bool {
        self.0.get(op.join().channel).is_some_and(|c| {
            c.flags & ((1 << 11) | 1) == 0
                && (c.flags & (1 << 9) == 0 || self.1 == Environment::Indoor)
        })
    }
    /// The SAE frames go at 20 MHz OFDM on the primary, which takes HE on
    /// 6 GHz, to an AP whose power type a client may use there
    /// (`cfg80211_6ghz_power_type_valid`).
    fn allows_six_ghz_authentication(
        &mut self,
        op: &reg::SixGhzAuthenticationOperation<'_>,
    ) -> bool {
        self.0.get(op.session().channel()).is_some_and(|c| {
            c.flags & (channel::DISABLED | channel::NO_20MHZ | channel::NO_HE) == 0
                && (c.flags & channel::INDOOR_ONLY == 0 || self.1 == Environment::Indoor)
                && c.max_half_dbm.is_some()
                && reg::PowerType::usable(op.profile().raw_ap_power_type, c.flags)
        })
    }
    fn allows_association(&mut self, op: &reg::AssociationOperation<'_>) -> bool {
        self.0.association(op, self.1)
    }
    fn allows_edca(&mut self, op: &reg::EdcaOperation<'_>) -> bool {
        self.0.association(op.association(), self.1)
            && op
                .observed()
                .iter()
                .all(|ac| ac.aifsn >= 2 && ac.ecw_min != 0)
            && reg::wmm::station(op.observed(), None).is_ok()
            && reg::wmm::station(op.effective(), None).is_ok()
    }
}
// SAFETY: private provider exists only inside the exclusive ManagedClient. Its
// immutable snapshot comes from that exact completed Execution. No raw
// controller/policy mutation escapes. Native operations/revocation take &mut
// that same owner, and its client keeps all physical resources through drain.
unsafe impl reg::HardwarePolicy for Hardware {
    type Lease<'a> = HardwareGuard<'a>;
    fn acquire(&mut self) -> Option<Self::Lease<'_>> {
        Some(HardwareGuard(&self.0, self.1))
    }
}
impl Os {
    fn receive(&self, ch: Channel) -> bool {
        self.country
            .rule(u32::from(ch.frequency_mhz) * 1000, 20_000)
            .is_some()
    }
    fn tx(&self, ch: Channel, probe: bool) -> bool {
        self.country
            .rule(u32::from(ch.frequency_mhz) * 1000, 20_000)
            .is_some_and(|r| {
                (!r.indoor_only || self.environment == Environment::Indoor)
                    && (!probe || !r.no_ir && !r.dfs)
            })
    }
    fn association(&self, op: &reg::AssociationOperation<'_>) -> bool {
        let ctx = op.context();
        let def = op.definition();
        let n = ctx.negotiated();
        let bw = match def.width {
            Width::Mhz20 => 20,
            Width::Mhz40 => 40,
            Width::Mhz80 => 80,
            Width::Mhz160 => 160,
            _ => return false,
        };
        let Some(max_power) = self.snapshot.footprint_power(def) else {
            return false;
        };
        let start = def.center1_mhz - bw / 2;
        for mhz in (start + 10..start + bw).step_by(20) {
            let Some(r) = self.country.rule(mhz * 1000, 20_000) else {
                return false;
            };
            if i32::from(max_power) * 50 > i32::from(r.max_eirp_mbm)
                || (r.indoor_only && self.environment != Environment::Indoor)
                || self.country.bandwidth(r).is_none_or(|max| max < bw * 1000)
                || (r.no_ofdm
                    && (n.mode != Mode::Legacy
                        || def.control.band != Band::Ghz2
                        || n.legacy.supported & !15 != 0))
            {
                return false;
            }
        }
        let parameters = n.edca();
        let Some(rule) = self
            .country
            .rule(u32::from(def.control.frequency_mhz) * 1000, 20_000)
        else {
            return false;
        };
        // ManagedClient installs the exact derived local queues before its
        // private configuration publication. AP observations stay unchanged.
        reg::wmm::station(&parameters, rule.wmm.as_ref()).is_ok()
    }
}
impl reg::OsLease for OsGuard<'_> {
    fn revision(&self) -> u64 {
        self.0.context.policy_revision
    }
    fn allows(&mut self, op: reg::Operation) -> bool {
        match op {
            reg::Operation::Receive(ch) => self.0.receive(ch),
            reg::Operation::Transmit {
                channel,
                purpose,
                profile,
            } => {
                self.0.tx(channel, purpose == reg::Purpose::Probe)
                    && self
                        .0
                        .country
                        .rule(u32::from(channel.frequency_mhz) * 1000, 20_000)
                        .is_some_and(|r| profile.max_eirp_mbm <= i32::from(r.max_eirp_mbm))
            }
        }
    }
    fn allows_pmf_authentication(&mut self, op: &reg::PmfAuthenticationOperation<'_>) -> bool {
        self.0.protection.selected()
            && op.join().request.pmf().is_some()
            && self.0.tx(op.join().channel, false)
    }
    /// The OS selected the SAE network the exchange authenticates against;
    /// the country decides reception and transmission on its channel.
    fn allows_six_ghz_authentication(
        &mut self,
        op: &reg::SixGhzAuthenticationOperation<'_>,
    ) -> bool {
        let channel = op.session().channel();
        self.0.protection.selected()
            && match op.purpose() {
                reg::SixGhzAuthenticationUse::Receive(_) => self.0.receive(channel),
                _ => self.0.tx(channel, false),
            }
    }
    fn allows_association(&mut self, op: &reg::AssociationOperation<'_>) -> bool {
        self.0.association(op)
    }
    /// Until the OS installs the handshake's pairwise key, a PMF association
    /// carries only its configuration, reception and EAPOL.
    fn allows_protected_management(&mut self, op: &reg::AssociationOperation<'_>) -> bool {
        match op.purpose() {
            reg::AssociationUse::Configure
            | reg::AssociationUse::Receive
            | reg::AssociationUse::Authentication => self.0.protection.selected(),
            reg::AssociationUse::Management
            | reg::AssociationUse::Data
            | reg::AssociationUse::ChannelSwitch => self.0.protection.authenticated(),
        }
    }
    fn allows_protected_port(&mut self, _: &reg::AssociationOperation<'_>) -> bool {
        self.0.protection.authenticated()
    }
    fn allows_edca(&mut self, op: &reg::EdcaOperation<'_>) -> bool {
        let channel = op.association().channel();
        self.0.association(op.association())
            && self
                .0
                .country
                .rule(u32::from(channel.frequency_mhz) * 1000, 20_000)
                .is_some_and(|rule| {
                    reg::wmm::station(op.observed(), rule.wmm.as_ref())
                        .is_ok_and(|effective| &effective == op.effective())
                })
    }
}
// SAFETY: selected source bytes/country/environment are immutable and kept
// privately by the same exclusive live owner. All native publications and the
// real stop/drain path serialize on its &mut borrow; country cannot be replaced
// underneath autonomous firmware work. Context is minted only for this owner.
unsafe impl reg::OsPolicy for Os {
    type Lease<'a> = OsGuard<'a>;
    fn acquire(&mut self, native: reg::NativeContext) -> Option<Self::Lease<'_>> {
        (native == self.context).then_some(OsGuard(self))
    }
}
impl Check<'_> {
    fn receive(&self, ch: Channel) -> bool {
        self.snapshot.get(ch).is_some()
            && self
                .country
                .rule(u32::from(ch.frequency_mhz) * 1000, 20_000)
                .is_some()
    }
    fn tx(&self, ch: Channel, probe: bool) -> bool {
        if self.revoked {
            return false;
        }
        let Some(c) = self.snapshot.get(ch) else {
            return false;
        };
        let Some(r) = self
            .country
            .rule(u32::from(ch.frequency_mhz) * 1000, 20_000)
        else {
            return false;
        };
        // NO_IR/DFS disallow initiation/probes; an observed AP may serve a
        // managed station on that channel. This is no AP/IBSS/radar exception.
        c.flags & (1 | 1 << 11) == 0
            && (!r.indoor_only || self.environment == Environment::Indoor)
            && (c.flags & (1 << 9) == 0 || self.environment == Environment::Indoor)
            && if probe {
                (ch.band == Band::Ghz2 || c.flags & (1 << 6) == 0)
                    && c.flags & (2 | 8) == 0
                    && !r.no_ir
                    && !r.dfs
            } else {
                self.peer == Some(ch)
            }
    }
}
impl scan::Policy for Check<'_> {
    fn allow_receive(&mut self, ch: Channel) -> bool {
        self.receive(ch)
    }
    fn allow_probe(&mut self, ch: Channel) -> bool {
        self.tx(ch, true)
    }
    fn lookup_channel(&mut self, b: Band, n: u16) -> Option<Channel> {
        self.snapshot
            .channels
            .iter()
            .find(|c| c.channel.band == b && c.channel.number == n && self.receive(c.channel))
            .map(|c| c.channel)
    }
}
impl station::RegulatoryPolicy for Check<'_> {
    fn allow_management_tx(&mut self, ch: Channel) -> bool {
        self.tx(ch, false)
    }
    fn allow_data_tx(&mut self, ch: Channel) -> bool {
        self.tx(ch, false)
    }
}
impl<I: BarIo, Q: QueueIo, C: 'static> ManagedClient<I, Q, C> {
    /// Consume the exclusive native client; verify selected source
    /// policy against its completed source programming before accepting work.
    // Preserve the original owner even for allocation failure: boxing this
    // error would itself require an allocation while reporting NoMemory.
    #[allow(
        clippy::result_large_err,
        reason = "Policy admission failure returns the original live client and diagnostics so uncertain radio/DMA ownership remains retained"
    )]
    pub fn new(
        client: Client<I, Q, C>,
        country: Country,
        environment: Environment,
    ) -> Result<Self, AttachError<I, Q, C>> {
        let snapshot = match Snapshot::read(&client, &country)
            .and_then(|s| Shared::try_new(s).map_err(|_| PolicyError::NoMemory))
        {
            Ok(snapshot) => snapshot,
            Err(error) => return Err(AttachError { error, client }),
        };
        let (Ok(country), Ok(protection)) = (
            Shared::try_new(country),
            Shared::try_new(Protection::default()),
        ) else {
            return Err(AttachError {
                error: PolicyError::NoMemory,
                client,
            });
        };
        let authority = match reg::Owner::new(
            Hardware(snapshot.clone(), environment),
            Os {
                country: country.clone(),
                snapshot: snapshot.clone(),
                environment,
                context: snapshot.context,
                protection: protection.clone(),
            },
        ) {
            Ok(authority) => authority,
            Err(error) => {
                return Err(AttachError {
                    error: PolicyError::Regulatory(error),
                    client,
                });
            }
        };
        Ok(Self {
            client,
            authority,
            snapshot,
            country,
            environment,
            revoked: false,
            peer: None,
            observed: Vec::new(),
            qos_map: None,
            // CONFIG_CFG80211_DEFAULT_PS
            power_save: true,
            protection,
        })
    }
    fn live(&self) -> Result<(), PolicyError> {
        if self.revoked {
            Err(PolicyError::Revoked)
        } else {
            Ok(())
        }
    }
    pub fn phase(&self) -> Phase {
        self.client.phase()
    }
    pub fn local_advertisement(&self, band: Band) -> Result<Advertisement, LocalError> {
        self.client.local_advertisement(band)
    }
    pub fn station(&self) -> &Station {
        self.client.station()
    }
    pub fn country(&self) -> [u8; 2] {
        self.country.alpha2()
    }
    /// The verified domain's channels on `band` that may transmit, with
    /// their cfg80211 flags.
    pub fn channels(&self, band: Band) -> Vec<(Channel, u32)> {
        self.snapshot
            .channels
            .iter()
            .filter(|c| c.channel.band == band && c.max_half_dbm.is_some())
            .map(|c| (c.channel, c.flags))
            .collect()
    }
    pub fn begin_scan(
        &mut self,
        request: scan::Request,
        now: u64,
        budget: u64,
    ) -> Result<scan::Ticket, PolicyError> {
        self.live()?;
        let mut check = Check {
            snapshot: &self.snapshot,
            country: &self.country,
            environment: self.environment,
            revoked: false,
            peer: self.peer,
        };
        let ticket = self
            .client
            .begin_scan(request, now, budget, &mut check)
            .map_err(PolicyError::Client)?;
        // cfg80211_bss_expire: the BSSes earlier scans found stay until old.
        self.observed.retain(|b| {
            now.checked_sub(b.seen)
                .is_some_and(|age| age <= BSS_EXPIRE_US)
        });
        Ok(ticket)
    }
    /// Whether a scan may probe on `channel`, as its admission decides.
    pub fn probes(&self, channel: Channel) -> bool {
        Check {
            snapshot: &self.snapshot,
            country: &self.country,
            environment: self.environment,
            revoked: self.revoked,
            peer: self.peer,
        }
        .tx(channel, true)
    }
    /// Whether the current association protects its management frames.
    pub fn protected(&self) -> bool {
        self.client.station().requires_protected_management()
    }
    /// Whether the association may change its management state: always
    /// without PMF, and with it once the OS has installed its pairwise key.
    pub fn management_ready(&self) -> bool {
        !self.protected() || self.protection.authenticated()
    }
    pub fn poll(&mut self, now: u64, source: &dyn PageSource) -> Result<Vec<Event>, PolicyError> {
        let mut check = Check {
            snapshot: &self.snapshot,
            country: &self.country,
            environment: self.environment,
            revoked: self.revoked,
            peer: self.peer,
        };
        let six = self
            .client
            .six_ghz_session()
            .filter(|_| self.client.phase() == Phase::Joining)
            .cloned();
        let events = match self.client.phase() {
            // A 6 GHz exchange is serviced only under its held transport.
            Phase::Joining if let Some(session) = six => {
                // SAFETY: permission and client are private disjoint fields of
                // this same exclusive owning attachment.
                unsafe {
                    self.client.poll_six_ghz_external_authentication(
                        &mut self.authority,
                        &session,
                        now,
                        source,
                        &mut check,
                    )
                }
            }
            Phase::Joining if self.protected() => {
                // SAFETY: permission and client are private disjoint fields of
                // this same exclusive owning attachment.
                unsafe {
                    self.client.poll_pmf_authentication(
                        &mut self.authority,
                        now,
                        source,
                        &mut check,
                    )
                }
            }
            // The OS configures the association before it is polled.
            Phase::AwaitingModernAdmission if self.protected() => Ok(Vec::new()),
            Phase::Associated if self.protected() => {
                let context = self
                    .client
                    .modern_context()
                    .ok_or(PolicyError::Bss)?
                    .clone();
                let admission = self
                    .authority
                    .admit_association(self.client.station(), context, reg::AssociationUse::Receive)
                    .map_err(PolicyError::Regulatory)?;
                // SAFETY: as for poll_pmf_authentication.
                unsafe {
                    self.client
                        .poll_pmf(&mut self.authority, admission, now, source, &mut check)
                }
            }
            _ => self.client.poll(now, source, &mut check),
        }
        .map_err(PolicyError::Client)?;
        self.observed
            .try_reserve(events.len())
            .map_err(|_| PolicyError::NoMemory)?;
        if !self.revoked {
            for event in &events {
                if let Event::Bss(bss) = event {
                    let name = self
                        .observed
                        .iter()
                        .find(|b| b.bss.bssid == bss.bssid && b.bss.channel == bss.channel)
                        .and_then(|b| {
                            (bss.hides_ssid() && !b.bss.hides_ssid()).then(|| b.bss.ssid.clone())
                        });
                    self.observed.retain(|b| {
                        now.checked_sub(b.seen)
                            .is_some_and(|age| age <= BSS_EXPIRE_US)
                            && (b.bss.bssid != bss.bssid || b.bss.channel != bss.channel)
                    });
                    if self.observed.len() >= 1024 {
                        self.observed.remove(0);
                    }
                    let handle = BssHandle(NEXT_BSS.next().ok_or(PolicyError::Bss)?);
                    let mut bss = copy_bss(bss)?;
                    if let Some(name) = name {
                        bss.ssid = name;
                    }
                    self.observed.push(Observed {
                        handle,
                        seen: now,
                        bss,
                    });
                }
            }
        }
        Ok(events)
    }
    pub fn bss(&self) -> impl Iterator<Item = (BssHandle, &scan::result::Bss)> {
        self.observed.iter().map(|b| (b.handle, &b.bss))
    }
    pub fn join_modern(
        &mut self,
        handle: BssHandle,
        join: Join,
        profile: Profile,
        now: u64,
        source: &dyn PageSource,
    ) -> Result<(), PolicyError> {
        self.live()?;
        let observed = self
            .observed
            .iter()
            .find(|b| b.handle == handle)
            .ok_or(PolicyError::Bss)?;
        if now
            .checked_sub(observed.seen)
            .is_none_or(|age| age > BSS_EXPIRE_US)
            || observed.bss.channel != join.channel
            || observed.bss.bssid != join.bssid
        {
            return Err(PolicyError::Bss);
        }
        let mut check = Check {
            snapshot: &self.snapshot,
            country: &self.country,
            environment: self.environment,
            revoked: false,
            peer: Some(join.channel),
        };
        let flags = self
            .snapshot
            .get(profile.channel)
            .ok_or(PolicyError::RestrictionMismatch)?
            .flags;
        if flags & (1 << 6) != 0
            && (profile.channel.band != Band::Ghz2 || profile.basic_rates & 15 == 0)
        {
            return Err(PolicyError::RestrictionMismatch);
        }
        if join.request.pmf().is_none() {
            let beacon = copy_bss(&observed.bss)?;
            self.peer = Some(join.channel);
            self.protection.set(false, false);
            return self
                .client
                .join_modern(join, beacon, profile, now, source, &mut check)
                .map_err(PolicyError::Client);
        }
        let beacon = self.observation(handle)?;
        self.peer = Some(join.channel);
        self.protection.set(true, false);
        // SAFETY: permission and client are private disjoint fields of this
        // same exclusive owning attachment, and the OS selected this network.
        unsafe {
            self.client.join_modern_pmf(
                &mut self.authority,
                PmfJoin {
                    join,
                    observed: &beacon,
                },
                profile,
                now,
                source,
                &mut check,
            )
        }
        .map_err(PolicyError::Client)
    }
    /// The client's own record of the beacon of the BSS `handle` names.
    pub fn observation(&self, handle: BssHandle) -> Result<Arc<PmfBssObservation>, PolicyError> {
        let observed = self
            .observed
            .iter()
            .find(|b| b.handle == handle)
            .ok_or(PolicyError::Bss)?;
        Ok(self
            .client
            .pmf_bss_observations()
            .filter(|o| {
                let bss = o.bss();
                bss.bssid == observed.bss.bssid
                    && bss.channel == observed.bss.channel
                    && bss.elements == observed.bss.elements
            })
            .last()
            .ok_or(PolicyError::Bss)?
            .clone())
    }
    pub fn configure_modern(&mut self, now: u64) -> Result<(), PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let rule = self
            .country
            .rule(u32::from(context.channel().frequency_mhz) * 1000, 20_000)
            .ok_or(PolicyError::RestrictionMismatch)?;
        // ieee80211_set_wmm_default limits its defaults as it does WMM's.
        let parameters = context.negotiated().edca();
        let effective = reg::wmm::station(&parameters, rule.wmm.as_ref())
            .map_err(|_| PolicyError::RestrictionMismatch)?;
        self.client
            .restrict_prepared_edca(&effective)
            .map_err(PolicyError::Client)?;
        for link in self.client.station().link_contexts().to_vec() {
            let rule = self
                .country
                .rule(u32::from(link.channel().frequency_mhz) * 1000, 20_000)
                .ok_or(PolicyError::RestrictionMismatch)?;
            let effective = reg::wmm::station(&link.negotiated().edca(), rule.wmm.as_ref())
                .map_err(|_| PolicyError::RestrictionMismatch)?;
            self.client
                .restrict_link_edca(
                    link.affiliation().ok_or(PolicyError::Bss)?.link_id,
                    &effective,
                )
                .map_err(PolicyError::Client)?;
        }
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Configure,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: permission and client are private disjoint fields of this same
        // exclusive owning attachment. No controller/policy mutator escapes.
        unsafe {
            self.client
                .configure_modern(&mut self.authority, admission, now)
        }
        .map_err(PolicyError::Client)
    }
    /// Follow the current AP's announced channel switch: admit the target as
    /// `mt7925_pre_channel_switch` checks it is usable, switch when due, and
    /// disconnect as mac80211 does when the target is refused.
    pub fn channel_switch(&mut self, now: u64, source: &dyn PageSource) -> Result<(), PolicyError> {
        self.live()?;
        // A switch already due runs as soon as its target is admitted.
        while let Some(step) = self.client.channel_switch_step(now) {
            let definition = match step {
                ChannelSwitchStep::Disconnect => return self.drop_connection(source),
                ChannelSwitchStep::Prepare(definition) | ChannelSwitchStep::Switch(definition) => {
                    definition
                }
            };
            let context = self
                .client
                .modern_context()
                .ok_or(PolicyError::Bss)?
                .clone();
            let Ok(admission) =
                self.authority
                    .admit_channel_switch(self.client.station(), context, definition)
            else {
                return self.drop_connection(source);
            };
            if matches!(step, ChannelSwitchStep::Prepare(_)) {
                self.client.channel_switch_prepared();
                continue;
            }
            // SAFETY: as for update_edca, with the target's own admission.
            unsafe {
                self.client
                    .switch_channel_modern(&mut self.authority, admission)
            }
            .map_err(PolicyError::Client)?;
            self.peer = Some(definition.control);
        }
        Ok(())
    }
    /// Apply a channel the current AP's beacon changed, or disconnect when
    /// the change leaves the associated channel, as mac80211 does.
    pub fn update_channel(&mut self, source: &dyn PageSource) -> Result<(), PolicyError> {
        self.live()?;
        if self.client.pending_channel() == Some(false) {
            return self.drop_connection(source);
        }
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: as for update_edca.
        unsafe {
            self.client
                .update_channel_modern(&mut self.authority, admission)
        }
        .map_err(PolicyError::Client)
    }
    /// The OS's connection-quality monitor (`cqm_rssi_thold` and
    /// `cqm_rssi_hyst`), or `None` to stop it.
    pub fn set_rssi_monitor(&mut self, monitor: Option<(i32, u32)>) -> Result<(), PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: as for update_edca.
        unsafe {
            self.client
                .set_rssi_monitor_modern(&mut self.authority, admission, monitor)
        }
        .map_err(PolicyError::Client)
    }
    /// The OS's power save setting, as `iw set power_save` changes it.
    pub fn set_power_save(&mut self, enabled: bool) {
        self.power_save = enabled;
    }
    /// `ieee80211_powersave_allowed` once the port is authorized: the OS
    /// enabled power save and the AP gave a valid AID.
    pub fn power_save_allowed(&self) -> bool {
        self.power_save
            && self
                .client
                .modern_context()
                .is_some_and(|context| context.response().power_save_allowed)
    }
    /// `ieee80211_recalc_ps_vif`: tell the firmware whether to save power.
    pub fn update_power_save(&mut self, enable: bool) -> Result<(), PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: as for update_edca.
        unsafe {
            self.client
                .set_power_save_modern(&mut self.authority, admission, enable)
        }
        .map_err(PolicyError::Client)
    }
    /// Send a slot time the current AP's beacon changed.
    pub fn update_slot_time(&mut self) -> Result<(), PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: as for update_edca.
        unsafe {
            self.client
                .update_slot_time_modern(&mut self.authority, admission)
        }
        .map_err(PolicyError::Client)
    }
    /// Consume the latest native current-peer observation using this owner's
    /// immutable country rule and same association publication permission.
    pub fn update_edca(&mut self) -> Result<(), PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let channel = self.client.pending_edca_channel().ok_or(PolicyError::Bss)?;
        let rule = self
            .country
            .rule(u32::from(channel.frequency_mhz) * 1000, 20_000)
            .ok_or(PolicyError::RestrictionMismatch)?;
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: private client, programmed country and permission belong to
        // this same exclusive attachment and serialize all native publication.
        unsafe {
            self.client
                .update_edca_modern(&mut self.authority, admission, rule.wmm.as_ref())
        }
        .map_err(PolicyError::Client)
    }
    pub fn pending_edca(&self) -> bool {
        self.client.pending_edca()
    }
    pub fn applied_edca(&self) -> Option<&AppliedEdca> {
        self.client.applied_edca()
    }
    pub fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, PolicyError> {
        self.live()?;
        if pairwise && self.protected() {
            self.protection.set(true, true);
        }
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: same exclusive privately kept controller and permission.
        unsafe {
            self.client.install_key_modern(
                &mut self.authority,
                admission,
                pairwise,
                index,
                material,
                seed,
            )
        }
        .map_err(PolicyError::Client)
    }
    /// The GTK of the AP of the association's link `link_id`, from the
    /// handshake's MLO GTK KDE.
    pub fn install_link_key(
        &mut self,
        link_id: u8,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: same exclusive privately kept controller and permission.
        unsafe {
            self.client.install_link_key_modern(
                &mut self.authority,
                admission,
                link_id,
                index,
                material,
                seed,
            )
        }
        .map_err(PolicyError::Client)
    }
    pub fn authorize_port(&mut self, epoch: u64) -> Result<(), PolicyError> {
        self.live()?;
        if !self.protected() {
            return self
                .client
                .authorize_port(epoch)
                .map_err(PolicyError::Client);
        }
        if self.client.station().association_epoch() != Some(epoch) {
            return Err(PolicyError::Client(Error::Station(station::Error::Stale)));
        }
        // A PMF port also needs the IGTK's management session.
        let session = self
            .client
            .state()
            .management
            .as_ref()
            .ok_or(PolicyError::Client(Error::NoKey))?
            .clone();
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        let admission = self
            .authority
            .admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            )
            .map_err(PolicyError::Regulatory)?;
        // SAFETY: same exclusive privately kept controller and permission;
        // the OS installed this association's keys after its handshake.
        unsafe {
            self.client
                .authorize_port_pmf(&mut self.authority, admission, &session)
        }
        .map_err(PolicyError::Client)
    }
    /// The QoS TID of a data frame on the current association, as
    /// `ieee80211_select_queue` chooses it; `None` for non-QoS data.
    pub fn data_tid(&self, ethernet: &[u8]) -> Option<u8> {
        let current = self.client.applied_edca().map(|e| &e.access_categories);
        station::data::data_tid(
            self.client.modern_context()?,
            current,
            self.qos_map.as_ref(),
            ethernet,
        )
    }
    /// `ieee80211_set_qos_map`: the OS's QoS Map Set field for the current
    /// association, or `None` to stop mapping. As `nl80211_key_allowed`
    /// requires, the station must be associated.
    pub fn set_qos_map(&mut self, map: Option<&[u8]>) -> Result<(), PolicyError> {
        self.live()?;
        if self.client.station().phase() != station::Phase::Associated {
            return Err(PolicyError::Client(Error::WrongPhase));
        }
        self.qos_map = map
            .map(station::data::QosMap::parse)
            .transpose()
            .map_err(|e| PolicyError::Client(Error::Data(e)))?;
        Ok(())
    }
    pub fn transmit(
        &mut self,
        epoch: u64,
        ethernet: &[u8],
        class: TxClass,
        tid: Option<u8>,
        source: &dyn PageSource,
    ) -> Result<Receipt, PolicyError> {
        self.live()?;
        let context = self
            .client
            .modern_context()
            .ok_or(PolicyError::Bss)?
            .clone();
        // A PMF association's EAPOL, keyed or not, uses its Authentication grant.
        let purpose = if self.protected() && !matches!(class, TxClass::Data) {
            reg::AssociationUse::Authentication
        } else {
            reg::AssociationUse::Data
        };
        let admission = self
            .authority
            .admit_association(self.client.station(), context, purpose)
            .map_err(PolicyError::Regulatory)?;
        let mut check = Check {
            snapshot: &self.snapshot,
            country: &self.country,
            environment: self.environment,
            revoked: false,
            peer: self.peer,
        };
        // SAFETY: same exclusive privately kept controller and permission.
        unsafe {
            self.client.transmit_modern(
                &mut self.authority,
                admission,
                epoch,
                Msdu {
                    ethernet,
                    class,
                    tid,
                },
                source,
                &mut check,
            )
        }
        .map_err(PolicyError::Client)
    }
    pub fn poll_station(&mut self, now: u64, source: &dyn PageSource) -> Result<(), PolicyError> {
        if self.protected() {
            return Ok(());
        }
        let mut check = Check {
            snapshot: &self.snapshot,
            country: &self.country,
            environment: self.environment,
            revoked: self.revoked,
            peer: self.peer,
        };
        self.client
            .poll_station(now, source, &mut check)
            .map_err(PolicyError::Client)
    }
    pub fn poll_released(&mut self) -> Result<Vec<Released<PacketOwner<C>>>, PolicyError> {
        self.client.poll_released().map_err(PolicyError::Client)
    }
    pub fn cancel_scan(&mut self) -> Result<(), PolicyError> {
        self.client.cancel_scan().map_err(PolicyError::Client)
    }
    /// Remove the station without telling the AP, as a
    /// `local_state_change` request does.
    pub fn disconnect(&mut self) -> Result<(), PolicyError> {
        self.protection
            .authenticated
            .store(false, Ordering::Release);
        self.client.disconnect().map_err(PolicyError::Client)
    }
    /// `ieee80211_mgd_deauth` and `ieee80211_set_disassoc`: leave power
    /// save, send the AP a deauthentication with this reason unless it asked
    /// for silence, then remove the station.
    pub fn deauthenticate(
        &mut self,
        reason: u16,
        source: &dyn PageSource,
    ) -> Result<(), PolicyError> {
        if !self.revoked && self.client.phase() == Phase::Joining {
            let mut check = Check {
                snapshot: &self.snapshot,
                country: &self.country,
                environment: self.environment,
                revoked: false,
                peer: self.peer,
            };
            // As below, a frame that cannot be sent does not hold up the
            // departure.
            let _ = if self.protected()
                && self.client.station().phase() != station::Phase::ExternalAuthenticating
            {
                // SAFETY: permission and client are private disjoint fields of
                // this same exclusive owning attachment.
                unsafe {
                    self.client.transmit_join_deauthentication_pmf(
                        &mut self.authority,
                        reason,
                        source,
                        &mut check,
                    )
                }
            } else {
                self.client
                    .transmit_join_deauthentication(reason, source, &mut check)
            };
        }
        if !self.revoked
            && self.client.phase() == Phase::Associated
            && !self.client.channel_switch_blocks_tx()
            && self.management_ready()
        {
            if self.client.power_save() {
                // ieee80211_recalc_ps_vif once the station is no longer
                // associated; the departure goes ahead regardless.
                let _ = self.update_power_save(false);
            }
            let context = self
                .client
                .modern_context()
                .ok_or(PolicyError::Bss)?
                .clone();
            if let Ok(admission) = self.authority.admit_association(
                self.client.station(),
                context,
                reg::AssociationUse::Management,
            ) {
                // A frame that cannot be built or sent, like a failed
                // `ieee80211_tx_skb`, does not hold up the departure; an
                // uncertain submission quarantines the client.
                // SAFETY: same exclusive privately kept controller and
                // permission.
                let _ = unsafe {
                    self.client.transmit_deauthentication(
                        &mut self.authority,
                        admission,
                        reason,
                        source,
                    )
                };
            }
        }
        self.disconnect()
    }
    /// `__ieee80211_disconnect` for a connection the station drops itself.
    fn drop_connection(&mut self, source: &dyn PageSource) -> Result<(), PolicyError> {
        // WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY
        self.deauthenticate(4, source)
    }
    /// Deny new work before initiating cancel/remove/disable commands.
    /// This return never claims completed retirement. Continue polling/draining
    /// the owner; no country replacement or raw mutator is exposed.
    pub fn revoke(&mut self) -> Result<(), PolicyError> {
        self.revoked = true;
        self.observed.clear();
        match self.client.phase() {
            Phase::Scanning => self.client.cancel_scan(),
            Phase::Joining | Phase::AwaitingModernAdmission | Phase::Associated => {
                self.client.disconnect()
            }
            Phase::Idle => self.client.stop(),
            Phase::ScanCompleted | Phase::NeedsDrain | Phase::ControllerDrain | Phase::Stopped => {
                Ok(())
            }
            _ => Err(Error::NeedsDrain),
        }
        .map_err(PolicyError::Client)
    }
    /// # Safety
    /// The scan radio restoration and DMA/IRQ/late notifications meet
    /// Client::retire_scan_after_drain, independently of its terminal event.
    pub unsafe fn retire_scan_after_drain(&mut self) -> Result<scan::Outcome, PolicyError> {
        // SAFETY: caller establishes the unchanged native scan fence contract.
        let result =
            unsafe { self.client.retire_scan_after_drain() }.map_err(PolicyError::Client)?;
        if self.revoked {
            self.client.stop().map_err(PolicyError::Client)?;
        }
        Ok(result)
    }
    /// # Safety
    /// All peer/key/radio/DMA/IRQ/late users meet the native peer fence.
    pub unsafe fn retire_peer_after_drain(&mut self) -> Result<(), PolicyError> {
        // SAFETY: caller establishes the unchanged native peer fence contract.
        unsafe { self.client.retire_peer_after_drain() }.map_err(PolicyError::Client)?;
        self.peer = None;
        if self.revoked {
            self.client.stop().map_err(PolicyError::Client)?;
        }
        Ok(())
    }
    pub fn stop(&mut self) -> Result<(), PolicyError> {
        self.revoked = true;
        self.client.stop().map_err(PolicyError::Client)
    }
    /// # Safety
    /// As for [`Client::release_after_reset`].
    pub unsafe fn release_after_reset(&mut self) -> Result<(), PolicyError> {
        self.revoked = true;
        // SAFETY: forwarded from the caller.
        unsafe { self.client.release_after_reset() }.map_err(PolicyError::Client)
    }
    /// # Safety
    /// The independent whole-controller platform fence meets
    /// Client::stop_after_quiesce; local idle alone is insufficient.
    pub unsafe fn stop_after_quiesce(&mut self) -> Result<(), PolicyError> {
        // SAFETY: caller establishes the unchanged native controller fence.
        unsafe { self.client.stop_after_quiesce() }.map_err(PolicyError::Client)
    }
}

#[cfg(test)]
mod tests;
