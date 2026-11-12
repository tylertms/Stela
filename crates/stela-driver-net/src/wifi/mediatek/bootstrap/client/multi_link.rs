// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Link lifecycle follows pinned Linux mt7925/main.c (mt7925_change_vif_links,
// mt7925_mac_link_bss_add, mt7925_mac_link_sta_add, mt7925_set_mlo_roc);
// source notices in NOTICE.
//! A station MLD association on the MT7925, as mt7925 runs one: the
//! interface rebound to the setup link's own address for authentication,
//! each other link on an interface of its own once the setup link is
//! configured, and the interface back on its own address once the
//! association's peers retire.
use super::modern::{associated, cipher_code, operation};
use super::*;
use crate::wifi::{
    mediatek::{
        mailbox::device::Deactivated,
        mcu::{client::MldLink, key::Context as KeyContext},
    },
    security::KeySlot,
    station::{
        AssociationContext,
        wire::link_changes::{mappings, removals},
    },
};
use alloc::borrow::Cow;
use station::multi_link::Affiliation;

/// A link's profile, PTK and GTK.
pub(super) type LinkKeys<'a, C> = (
    Profile,
    Option<&'a HardwareKey<Arc<C>>>,
    Option<&'a HardwareKey<Arc<C>>>,
);
/// A link's PTK and GTK.
pub(super) type KeyPair<'a, C> = (&'a mut HardwareKey<Arc<C>>, &'a mut HardwareKey<Arc<C>>);

/// One key for a link: which, its cipher and index, its material and its
/// receive counters.
pub(super) struct LinkKey<'a> {
    pub(super) pairwise: bool,
    pub(super) cipher: Cipher,
    pub(super) index: u8,
    pub(super) material: &'a [u8],
    pub(super) seed: ReplaySeed,
}

/// Whether an advertised TID mapping excludes an associated link.
fn barred_link(links: u16, elements: &[u8]) -> bool {
    mappings(elements).map_or(true, |mappings| {
        mappings
            .iter()
            .any(|mapping| mapping.switch_time != 0 && links & !mapping.links != 0)
    })
}

/// Another link of an AP MLD association, as `mt7925_mac_link_bss_add` and
/// `mt7925_mac_link_sta_add` add one: an interface of its own, its AP's
/// WCID and keys, and the records configuring both.
pub(super) struct Link<C: 'static> {
    pub(super) link_id: u8,
    pub(super) profile: Profile,
    wire: wire::modern::Configuration,
    pub(super) context: Arc<AssociationContext>,
    pub(super) edca: Option<edca::Observed<C>>,
    pub(super) channel: ChannelDefinition,
    pub(super) channel_change: Option<Option<ChannelDefinition>>,
    pub(super) applied_edca: Option<edca::AppliedEdca>,
    prepared_edca: [station::phy::AccessCategory; 4],
    pub(super) interface: Interface<Arc<C>>,
    /// The MLDs' PTK, kept on this link's AP WCID as on every link's.
    pub(super) ptk: HardwareKey<Arc<C>>,
    /// The GTK of this link's AP.
    pub(super) gtk: HardwareKey<Arc<C>>,
    ptk_removed: Option<Removal>,
    gtk_removed: Option<Removal>,
    interface_removed: Option<Deactivated>,
    /// The link's unfinished channel tracking and action switch already reported.
    reported: [bool; 2],
    /// The link's BSS record, the setup link's AP record naming both links,
    /// the link's AP record and its EDCA, in `mt7925_mac_link_sta_add` order.
    commands: [(Command, Vec<u8>); 4],
}

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// The registration of the setup link of `affiliation`: the interface's
    /// own slots under the link's address.
    pub(super) fn setup_link_context(
        s: &Owned<I, Q, C>,
        affiliation: &Affiliation,
    ) -> Result<Context, Error> {
        let single = s.interface.registered_context().ok_or(Error::WrongPhase)?;
        if s.single_link.is_some() || affiliation.mld.local != single.address {
            return Err(Error::WrongProfile);
        }
        Ok(Context {
            address: affiliation.local,
            ..single
        })
    }

    /// `mt7925_change_vif_links` as the interface becomes an AP MLD
    /// association's setup link: its registration moves to `link`, the
    /// link's address.
    pub(super) fn bind_setup_link(s: &mut Owned<I, Q, C>, link: Context) -> Result<(), Error> {
        let single = s.interface.registered_context().ok_or(Error::WrongPhase)?;
        s.phase = Phase::Quarantined;
        s.interface
            .rebind(s.controller.state_mut().mailbox.as_mut().unwrap(), link)
            .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?;
        s.single_link = Some(single);
        Ok(())
    }

    /// The MLD record of the setup link of `affiliation`, whose AP
    /// `profile` names.
    pub(super) fn mld_link(affiliation: &Affiliation, profile: Profile) -> MldLink {
        MldLink {
            link_id: affiliation.link_id,
            local: affiliation.mld.local,
            ap: affiliation.mld.ap,
            primary_bss: profile.interface.bss_index,
            primary_wcid: profile.wcid,
            eml: None,
        }
    }

    /// `mt792x_tx` for a management frame of `association` to its AP: sent
    /// with the device's address translation off, it takes the setup link's
    /// addresses, the AP's there as its BSSID.
    pub(super) fn on_setup_link<'a>(
        bytes: &'a [u8],
        association: &AssociationContext,
    ) -> Cow<'a, [u8]> {
        let Some(affiliation) = association.affiliation() else {
            return Cow::Borrowed(bytes);
        };
        if bytes.len() < 22 || bytes[4] & 1 != 0 {
            return Cow::Borrowed(bytes);
        }
        let mut frame = bytes.to_vec();
        let ap = association.bssid();
        frame[4..10].copy_from_slice(&ap.0);
        frame[10..16].copy_from_slice(&affiliation.local.0);
        frame[16..22].copy_from_slice(&ap.0);
        Cow::Owned(frame)
    }

    /// `mt7925_change_vif_links` and `mt7925_mac_link_sta_add` prepared for
    /// the other links of the association whose setup link `p` and `setup`
    /// configure: each takes the next BSS slot and broadcast WCID under its
    /// own address, its AP the next peer WCID, and its records name both
    /// MLDs with the setup link primary.
    pub(super) fn prepare_links(
        s: &mut Owned<I, Q, C>,
        contexts: Vec<Arc<AssociationContext>>,
        p: Profile,
        setup: &wire::modern::Configuration,
    ) -> Result<(), Error> {
        if contexts.is_empty() {
            return Ok(());
        }
        let primary = *setup.link().ok_or(Error::WrongProfile)?;
        let peer = s.station.pairwise_peer().ok_or(Error::WrongPhase)?;
        let mut links = Vec::new();
        links
            .try_reserve_exact(contexts.len())
            .map_err(|_| Error::NoMemory)?;
        for (n, context) in (1u8..).zip(contexts) {
            let affiliation = context.affiliation().ok_or(Error::WrongProfile)?;
            let interface = Context {
                bss_index: p.interface.bss_index + n,
                omac_index: p.interface.omac_index,
                // The firmware picks the link's band.
                band_index: 0xff,
                wmm_index: p.interface.wmm_index,
                link_index: n,
                broadcast_wcid: p
                    .interface
                    .broadcast_wcid
                    .checked_sub(n.into())
                    .ok_or(Error::WrongProfile)?,
                address: affiliation.local,
            };
            let profile = associated(
                Profile {
                    interface,
                    peer: context.bssid(),
                    wcid: p.wcid + u16::from(n),
                    channel: context.channel(),
                    cipher: 0,
                    ..p
                },
                &context,
            );
            let actual =
                local::current(s, profile.channel.band).map_err(|_| Error::WrongProfile)?;
            actual
                .validate_capabilities(context.negotiated().local())
                .map_err(|_| Error::WrongProfile)?;
            let wire = wire::modern::Configuration::new(
                profile,
                context.negotiated().clone(),
                operation(&context, profile.channel.band)?,
                *actual.he_source_maps(),
            )
            .map_err(Error::Codec)?
            .with_link(MldLink {
                link_id: affiliation.link_id,
                ..primary
            });
            let commands = [
                (
                    Command::BssInfo,
                    wire.bss_request(profile, true).map_err(Error::Codec)?,
                ),
                (
                    Command::StationRecord,
                    setup.peer_request(p, true).map_err(Error::Codec)?,
                ),
                (
                    Command::StationRecord,
                    wire.peer_request(profile, true).map_err(Error::Codec)?,
                ),
                (
                    Command::Edca,
                    wire.edca_request(profile).map_err(Error::Codec)?,
                ),
            ];
            let key_context = KeyContext {
                bss_index: interface.bss_index,
                omac_index: interface.omac_index,
                station_disabled: false,
            };
            let m = s.controller.state().mailbox.as_ref().unwrap();
            let resource = s.controller.state().resource.clone();
            // SAFETY: the link's slots are the next of the client's own
            // interface on its own mailbox, which no other owner holds.
            let (interface, ptk, gtk) = unsafe {
                (
                    Interface::claim(m, interface, resource.clone())
                        .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?,
                    HardwareKey::claim(
                        m,
                        profile.wcid,
                        key_context,
                        KeyScope::Pairwise(peer),
                        resource.clone(),
                    )
                    .map_err(Error::Key)?,
                    HardwareKey::claim(
                        m,
                        interface.broadcast_wcid,
                        key_context,
                        KeyScope::Group(profile.peer),
                        resource,
                    )
                    .map_err(Error::Key)?,
                )
            };
            links.push(Link {
                link_id: affiliation.link_id,
                profile,
                prepared_edca: context.negotiated().edca(),
                channel: context.definition(),
                channel_change: None,
                context,
                edca: None,
                applied_edca: None,
                wire,
                interface,
                ptk,
                gtk,
                ptk_removed: None,
                gtk_removed: None,
                interface_removed: None,
                reported: [false; 2],
                commands,
            });
        }
        s.links = links;
        Ok(())
    }

    /// `mt7925_change_vif_links` adding `link` once the setup link `p` is
    /// configured: the link's interface registered, both links' channels
    /// granted together, then the link's records.
    pub(super) fn activate_link(
        roc: &mut radio::Roc<'_, I, Q, C>,
        replies: &mut Vec<mailbox::UnifiedReply>,
        p: Profile,
        link: &mut Link<C>,
    ) -> Result<(), Error> {
        let interface = link.profile.interface;
        roc.controller
            .state_mut()
            .control
            .clear_peer_admission(interface.broadcast_wcid)
            .map_err(|e| Error::Bootstrap(super::super::Error::Control(e)))?;
        // A link's registration while associating: both commands now.
        stela_driver_base::stepped::block_on(
            link.interface
                .activate(roc.controller.state_mut().mailbox.as_mut().unwrap()),
        )
        .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?;
        // mt7925_set_mlo_roc takes the channels while no other request holds
        // them: the association's own request ends first.
        if roc.roc_deadline.take().is_some() {
            let abort = p.abort_roc(*roc.roc_token).map_err(Error::Codec)?;
            roc.transport_facts
                .try_reserve(1)
                .map_err(|_| Error::NoMemory)?;
            let fact = roc
                .controller
                .state_mut()
                .mailbox
                .as_mut()
                .unwrap()
                .initial(&abort)
                .map_err(Error::Transport)?;
            roc.transport_facts.push(fact);
        }
        let token = roc.roc_token.checked_add(1).ok_or(Error::NeedsDrain)?;
        let request = p.mlo_roc(link.profile, token, 5).map_err(Error::Codec)?;
        let abort = p.abort_roc(token).map_err(Error::Codec)?;
        // MT7925_ROC_REQ_MLSR_AG or MT7925_ROC_REQ_MLSR_AA, granted to the
        // setup link.
        Self::wait_grant(roc, token, &request, &abort, |g| {
            g.bss == p.interface.bss_index && matches!(g.request_type, 10 | 11)
        })?;
        for (index, (command, body)) in link.commands.iter().enumerate() {
            if index == 2 {
                roc.controller
                    .state_mut()
                    .control
                    .clear_peer_admission(link.profile.wcid)
                    .map_err(|e| Error::Bootstrap(super::super::Error::Control(e)))?;
            }
            Self::command_parts(roc.controller, replies, *command, body)?;
        }
        link.applied_edca = Some(edca::AppliedEdca {
            parameter_set_count: link
                .context
                .negotiated()
                .wmm
                .as_ref()
                .map_or(0, |wmm| wmm.parameter_set_count),
            access_categories: link.prepared_edca,
        });
        Ok(())
    }

    pub(super) fn restrict_link_edca(
        &mut self,
        link_id: u8,
        effective: &[station::phy::AccessCategory; 4],
    ) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::AwaitingModernAdmission {
            return Err(Error::WrongPhase);
        }
        let link = s
            .links
            .iter_mut()
            .find(|link| link.link_id == link_id)
            .ok_or(Error::WrongProfile)?;
        link.commands[3].1 = link
            .wire
            .edca_restricted_request(link.profile, effective)
            .map_err(Error::Codec)?;
        link.prepared_edca = *effective;
        Ok(())
    }

    /// `mt7925_set_key` on `link`: the link's BSS takes the cipher with its
    /// first key, then the link's PTK or GTK is installed.
    pub(super) fn install_link_key(
        controller: &mut Controller<I, Q, C>,
        replies: &mut Vec<mailbox::UnifiedReply>,
        link: &mut Link<C>,
        key: &LinkKey<'_>,
    ) -> Result<KeyIdentity, Error> {
        let LinkKey {
            pairwise,
            cipher,
            index,
            material,
            seed,
        } = *key;
        if link.profile.cipher == 0 {
            let profile = Profile {
                cipher: cipher_code(cipher),
                ..link.profile
            };
            let body = link.wire.bss_request(profile, true).map_err(Error::Codec)?;
            Self::command_parts(controller, replies, Command::BssInfo, &body)?;
            link.profile = profile;
        }
        let key = if pairwise {
            &mut link.ptk
        } else {
            &mut link.gtk
        };
        key.install(
            controller.state_mut().mailbox.as_mut().unwrap(),
            index,
            cipher,
            material,
            seed,
        )
        .map_err(Error::Key)
    }

    /// The profile and keys of the link whose AP the device received a frame
    /// from as `wcid`: another link's, or the setup link's.
    pub(super) fn link_of(s: &Owned<I, Q, C>, wcid: u16) -> Result<LinkKeys<'_, C>, Error> {
        Ok(
            match s.links.iter().find(|link| link.profile.wcid == wcid) {
                Some(link) => (link.profile, Some(&link.ptk), Some(&link.gtk)),
                None => (
                    s.profile.ok_or(Error::WrongProfile)?,
                    s.ptk.as_ref(),
                    s.gtk.as_ref(),
                ),
            },
        )
    }

    /// The profile of the link other than the setup link that `radio` heard
    /// a frame on.
    pub(super) fn other_link(s: &Owned<I, Q, C>, radio: &wifi::RadioStatus) -> Option<Profile> {
        let link = s.links.iter().find(|link| {
            (link.profile.channel.band, link.profile.channel.number) == (radio.band, radio.channel)
        })?;
        Some(link.profile)
    }

    /// Track another link's completed WMM and bandwidth beacon under its
    /// own context. Channel-switch tracking remains reported once per link.
    /// Other unicast frames name the MLDs and follow the setup-link path.
    pub(super) fn beacon_of_other_link(
        s: &mut Owned<I, Q, C>,
        frame: Mpdu,
        events: &mut Vec<Event>,
    ) -> Result<(), Error> {
        let index = s
            .links
            .iter()
            .position(|link| {
                (link.profile.channel.band, link.profile.channel.number)
                    == (frame.radio.band, frame.radio.channel)
            })
            .ok_or(Error::WrongProfile)?;
        Self::observe_link_beacon(s, frame)?;
        if !core::mem::replace(&mut s.links[index].reported[0], true) {
            events.push(Event::Unsupported("MLO link beacon channel switches"));
        }
        Ok(())
    }

    /// Whether a channel switch announced on `radio` is another link's, which
    /// stays on its channel until its AP is gone: reported once per link.
    pub(super) fn switch_of_other_link(s: &mut Owned<I, Q, C>, radio: &wifi::RadioStatus) -> bool {
        let Some(link) = Self::link_on(s, radio) else {
            return false;
        };
        let first = !core::mem::replace(&mut link.reported[1], true);
        if first {
            s.csa.ready.push(Event::Unsupported(
                "a channel switch of an MLO link other than the setup link",
            ));
        }
        true
    }

    fn link_on<'a>(
        s: &'a mut Owned<I, Q, C>,
        radio: &wifi::RadioStatus,
    ) -> Option<&'a mut Link<C>> {
        s.links.iter_mut().find(|link| {
            (link.profile.channel.band, link.profile.channel.number) == (radio.band, radio.channel)
        })
    }

    /// The PTK and GTK of the link `radio` received on: another link's on
    /// its channel, or the setup link's.
    pub(super) fn receiving<'a>(
        ptk: &'a mut Option<HardwareKey<Arc<C>>>,
        gtk: &'a mut Option<HardwareKey<Arc<C>>>,
        links: &'a mut [Link<C>],
        radio: &wifi::RadioStatus,
    ) -> Result<KeyPair<'a, C>, Error> {
        let channel = |link: &&mut Link<C>| {
            (link.profile.channel.band, link.profile.channel.number) == (radio.band, radio.channel)
        };
        Ok(match links.iter_mut().find(channel) {
            Some(link) => (&mut link.ptk, &mut link.gtk),
            None => (
                ptk.as_mut().ok_or(Error::NoKey)?,
                gtk.as_mut().ok_or(Error::NoKey)?,
            ),
        })
    }

    /// The station's and the AP's addresses in the association's frames as
    /// `translated` gives them: the MLDs' for an MLD association, else the
    /// interface's and the BSS's of `p`.
    pub(super) fn association_addresses(
        s: &Owned<I, Q, C>,
        p: Profile,
    ) -> (crate::Mac, crate::Mac) {
        s.station
            .association_context()
            .and_then(|context| context.affiliation())
            .map_or((p.interface.address, p.peer), |affiliation| {
                (affiliation.mld.local, affiliation.mld.ap)
            })
    }

    /// `frame` as `ieee80211_prepare_and_rx_handle` hands it on: an MLD
    /// association's unicast frame from a link's AP names the MLDs.
    pub(super) fn translated<'a>(s: &Owned<I, Q, C>, frame: &'a Mpdu) -> Cow<'a, Mpdu> {
        s.station
            .mld_view(frame)
            .map_or(Cow::Borrowed(frame), |(view, _)| view)
    }

    /// `record`'s body for each link of the association, the setup link's
    /// first, as `mt7925_mcu_uni_rx_ba` and `mt7925_mcu_uni_tx_ba` program a
    /// block ack session on every link's AP WCID.
    pub(super) fn on_every_link(
        s: &Owned<I, Q, C>,
        record: impl Fn(Profile) -> Result<Vec<u8>, wire::Error>,
    ) -> Result<Vec<Vec<u8>>, Error> {
        let p = s.profile.ok_or(Error::WrongProfile)?;
        [p].into_iter()
            .chain(s.links.iter().map(|link| link.profile))
            .map(|p| record(p).map_err(Error::Codec))
            .collect()
    }

    /// The AP's WCID on each link of the association whose setup link `p`
    /// configures, the setup link's first.
    pub(super) fn peer_wcids(s: &Owned<I, Q, C>, p: Profile) -> Vec<u16> {
        [p.wcid]
            .into_iter()
            .chain(s.links.iter().map(|link| link.profile.wcid))
            .collect()
    }

    /// Each link's PTK and GTK, the setup link's first, as the station
    /// authorizes its port.
    pub(super) fn link_keys<'a>(
        ptk: &'a Option<HardwareKey<Arc<C>>>,
        gtk: &'a Option<HardwareKey<Arc<C>>>,
        links: &'a [Link<C>],
    ) -> Result<Vec<(&'a KeySlot, &'a KeySlot)>, Error> {
        let mut keys = Vec::new();
        keys.try_reserve_exact(1 + links.len())
            .map_err(|_| Error::NoMemory)?;
        keys.push((
            ptk.as_ref().ok_or(Error::NoKey)?.slot(),
            gtk.as_ref().ok_or(Error::NoKey)?.slot(),
        ));
        keys.extend(links.iter().map(|link| (link.ptk.slot(), link.gtk.slot())));
        Ok(keys)
    }

    /// Every key the association installed, as the station checks its port
    /// open.
    pub(super) fn key_slots(s: &Owned<I, Q, C>) -> Vec<&KeySlot> {
        [s.ptk.as_ref(), s.gtk.as_ref()]
            .into_iter()
            .flatten()
            .chain(s.links.iter().flat_map(|link| [&link.ptk, &link.gtk]))
            .map(HardwareKey::slot)
            .collect()
    }

    /// `mt7925_change_sta_links` and `mt792x_mac_link_bss_remove` for every
    /// other link, after the setup link's peer is removed: its keys, its AP's
    /// record and its interface.
    pub(super) fn remove_links(s: &mut Owned<I, Q, C>) -> Result<(), Error> {
        for link in &mut s.links {
            let mailbox = s.controller.state_mut().mailbox.as_mut().unwrap();
            for (key, receipt) in [
                (&mut link.ptk, &mut link.ptk_removed),
                (&mut link.gtk, &mut link.gtk_removed),
            ] {
                if key.slot().installed().is_some() {
                    *receipt = Some(key.remove(mailbox).map_err(Error::Key)?);
                }
            }
            // A link whose interface never registered has no records.
            if link.interface.registered_context().is_none() && !link.interface.is_quarantined() {
                continue;
            }
            let peer = link
                .profile
                .peer_request(false, false)
                .map_err(Error::Codec)?;
            Self::command_parts(
                &mut s.controller,
                &mut s.replies,
                Command::StationRecord,
                &peer,
            )?;
            s.controller
                .state_mut()
                .control
                .clear_peer_admission(link.profile.wcid)
                .map_err(|e| Error::Bootstrap(super::super::Error::Control(e)))?;
            link.interface_removed = Some(
                link.interface
                    .deactivate(s.controller.state_mut().mailbox.as_mut().unwrap())
                    .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?,
            );
        }
        Ok(())
    }

    /// Retire every other link's keys and interface once their old users
    /// drained, under `retire_peer_after_drain`'s contract.
    ///
    /// # Safety
    /// As `retire_peer_after_drain`.
    pub(super) unsafe fn retire_links(s: &mut Owned<I, Q, C>) -> Result<(), Error> {
        for mut link in core::mem::take(&mut s.links) {
            for (key, receipt) in [
                (&mut link.ptk, link.ptk_removed.take()),
                (&mut link.gtk, link.gtk_removed.take()),
            ] {
                if let Some(r) = receipt {
                    // SAFETY: covered by this function's own safety contract.
                    unsafe { key.retire_after_drain(r) }.map_err(Error::Key)?;
                }
            }
            if let Some(r) = link.interface_removed.take() {
                // SAFETY: covered by this function's own safety contract.
                unsafe { link.interface.retire_after_drain(r) }
                    .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?;
            }
        }
        Ok(())
    }

    /// `ieee80211_ml_reconfiguration`: a present reconfiguration replaces or
    /// cancels the removal countdown; absence leaves the earlier one running.
    /// Runtime per-link retirement remains unsupported, so expiry follows the
    /// existing reported association-disconnect response.
    pub(super) fn follow_links(
        s: &mut Owned<I, Q, C>,
        setup: &AssociationContext,
        elements: &[u8],
    ) {
        let Some(affiliation) = setup.affiliation() else {
            return;
        };
        let mut intervals = [None; 15];
        intervals[usize::from(affiliation.link_id)] = s.profile.map(|p| p.beacon_interval);
        for link in &s.links {
            intervals[usize::from(link.link_id)] = Some(link.profile.beacon_interval);
        }
        let links = intervals
            .iter()
            .enumerate()
            .fold(0, |mask, (link, interval)| {
                mask | if interval.is_some() { 1 << link } else { 0 }
            });
        if let Some(removing) = removals(elements) {
            // mac80211 records a profile's last timer when it occurs twice.
            let mut timers = [None; 15];
            for (link, count) in removing {
                timers[usize::from(link)] = Some(count);
            }
            let mut delay = None;
            for (interval, count) in intervals.into_iter().zip(timers) {
                if let (Some(interval), Some(count)) = (interval, count) {
                    let link_delay = u64::from(interval) * u64::from(count.saturating_sub(1));
                    // Preserve ieee80211_ml_reconfiguration's zero-delay
                    // initialization and subsequent minimum in link-ID order.
                    delay = Some(match delay {
                        None | Some(0) => link_delay,
                        Some(previous) => previous.min(link_delay),
                    });
                }
            }
            let now = s.controller.state().control.clock.now_us();
            s.link_removal = delay.map(|delay| now.saturating_add(delay * 1024));
        }
        if barred_link(links, elements) {
            s.link_taken = true;
            s.csa.drop_connection();
        }
    }

    /// Dispatch a due removal even if no further beacon or data arrives.
    pub(super) fn poll_link_removal(s: &mut Owned<I, Q, C>) {
        if s.phase == Phase::Associated
            && s.link_removal
                .is_some_and(|deadline| s.controller.state().control.clock.now_us() >= deadline)
        {
            s.link_removal = None;
            s.link_taken = true;
            s.csa.drop_connection();
        }
    }

    /// `mt7925_change_vif_links` restoring the single-link station once the
    /// association's peers retired.
    pub(super) fn unbind_links(s: &mut Owned<I, Q, C>) -> Result<(), Error> {
        let Some(single) = s.single_link.take() else {
            return Ok(());
        };
        s.phase = Phase::Quarantined;
        s.interface
            .rebind(s.controller.state_mut().mailbox.as_mut().unwrap(), single)
            .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?;
        s.phase = Phase::Idle;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mapping_ends_the_association_only_when_it_comes_and_leaves_a_link_out() {
        if crate::test_process::isolate_model() {
            return;
        }
        let links = 1 << 3 | 1 << 5;
        let mapped = |control: u8, to: u8| {
            let mut element = vec![255, 13, 109, control, 0xff, 9, 0];
            element.extend([to; 8]);
            element
        };
        assert!(barred_link(links, &mapped(2 | 0x08 | 0x20, 1 << 3)));
        assert!(!barred_link(
            links,
            &mapped(2 | 0x08 | 0x20, 1 << 3 | 1 << 5)
        ));
        // Refused outright, as mac80211 disconnects on it.
        assert!(barred_link(links, &mapped(0x08 | 0x20, 1 << 3)));
    }
}
