// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2023 MediaTek Inc.
// The announcement handling follows pinned Linux mac80211/mlme.c.
//! The AP's channel switch: mac80211's `ieee80211_sta_process_chanswitch`
//! records each announcement, mt7925's `channel_switch` timer runs
//! `mt7925_csa_work` when it is due under the target's own regulatory
//! admission, and transmission waits while the AP asked for silence.
use super::*;
use crate::wifi::station::csa::{self as announcement, ChannelSwitch, Frame};

/// Where an announcement came from, `ieee80211_csa_source`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Source {
    Beacon,
    ProtectedAction,
    UnprotectedAction,
}

struct Active {
    switch: ChannelSwitch,
    /// `csa_timer`: when the switch is due.
    deadline: u64,
    /// The regulatory owner admitted the target, as `mt7925_pre_channel_switch`
    /// checks it is usable.
    prepared: bool,
    /// `csa.waiting_bcn`: switched away from this channel, awaiting the AP's
    /// beacon on the new one.
    switched_from: Option<wifi::Channel>,
}

#[derive(Default)]
pub(super) struct State {
    active: Option<Active>,
    /// `csa.blocked_tx`: the AP asked for silence until the switch.
    blocked_tx: bool,
    /// `csa_connection_drop_work`: the announcement cannot be followed.
    dropped: bool,
    /// Notifications for the next poll.
    pub(super) ready: Vec<Event>,
}

impl State {
    /// The association goes at the next look at the switch's state.
    pub(super) fn drop_connection(&mut self) {
        self.dropped = true;
    }
}

/// What the owning policy does next for an announced channel switch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelSwitchStep {
    /// The switch cannot be followed: disconnect, as mac80211 does.
    Disconnect,
    /// Admit the target with the regulatory owner.
    Prepare(ChannelDefinition),
    /// The switch is due: move the firmware to the target.
    Switch(ChannelDefinition),
}

/// Target beacon transmission times, in 1024 us units, to microseconds.
fn after(now: u64, beacon_interval: u16, count: u8) -> u64 {
    now.saturating_add(u64::from(beacon_interval) * u64::from(count) * 1024)
}

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// `ieee80211_sta_process_chanswitch` for the current AP's elements;
    /// `ecsa` is a public action frame's own Extended Channel Switch fields.
    pub(super) fn process_channel_switch(
        s: &mut Owned<I, Q, C>,
        elements: &[u8],
        source: Source,
        ecsa: Option<[u8; 4]>,
    ) {
        let (Some(modern), Some(operating), Some(p)) = (
            s.modern.as_ref(),
            s.station.operating_definition(),
            s.profile,
        ) else {
            return;
        };
        if s.phase != Phase::Associated {
            return;
        }
        let now = s.controller.state().control.clock.now_us();
        let frame = match source {
            Source::Beacon => Frame::Beacon,
            _ => Frame::Action,
        };
        let parsed = announcement::parse(
            elements,
            frame,
            ecsa,
            operating.control.band,
            modern.context().negotiated(),
        );
        let current = s.channel.unwrap_or(operating);
        let csa = &mut s.csa;
        let switch = match parsed {
            Ok(switch) => switch,
            // Unprotected frames are not trusted to disconnect.
            Err(_) if source == Source::UnprotectedAction => return,
            Err(refusal) => {
                csa.dropped = true;
                csa.blocked_tx = refusal.block_tx;
                return;
            }
        };
        if let Some(active) = &mut csa.active {
            if source != Source::Beacon {
                return;
            }
            if active.switched_from.is_some() {
                // ieee80211_chswitch_post_beacon; a switch still announced
                // afterwards is a new one.
                csa.ready.push(Event::ChannelSwitched(current));
                csa.active = None;
                csa.blocked_tx = false;
            } else if let Some(switch) = switch {
                // mt7925_channel_switch_rx_beacon: a recount of the same
                // switch moves its timer.
                if switch.definition == active.switch.definition && switch.count != 0 {
                    active.deadline = after(now, p.beacon_interval, switch.count);
                }
                return;
            } else {
                // ieee80211_sta_abort_chanswitch: the AP withdrew it.
                csa.active = None;
                csa.blocked_tx = false;
                return;
            }
        }
        let Some(switch) = switch else {
            csa.blocked_tx = false;
            return;
        };
        // An unprotected action frame only quiets the station until a beacon
        // confirms or refutes it.
        if source == Source::UnprotectedAction {
            csa.blocked_tx |= switch.block_tx;
            return;
        }
        if switch.definition.control.band != operating.control.band {
            csa.dropped = true;
            csa.blocked_tx = switch.block_tx;
            return;
        }
        if switch.definition == current && (!switch.block_tx || source != Source::Beacon) {
            return;
        }
        csa.ready.push(Event::ChannelSwitchStarted {
            definition: switch.definition,
            count: switch.count,
            block_tx: switch.block_tx,
        });
        csa.active = Some(Active {
            switch,
            deadline: after(now, p.beacon_interval, switch.count),
            prepared: false,
            switched_from: None,
        });
        csa.blocked_tx = switch.block_tx;
    }

    /// A Spectrum Management or (Protected Dual of) Public channel switch
    /// action frame from the current AP, consumed as mac80211's MLME does.
    pub(super) fn channel_switch_action(s: &mut Owned<I, Q, C>, frame: &Mpdu, body: &[u8]) -> bool {
        let Some(p) = s.profile else {
            return false;
        };
        let frame = Self::translated(s, frame);
        let h = frame.header();
        let (_, peer) = Self::association_addresses(s, p);
        if h.control & 0x00fc != 0x00d0
            || h.transmitter != Some(peer)
            || frame.bytes().get(16..22) != Some(peer.0.as_slice())
        {
            return false;
        }
        match body {
            [0, 4, ..] | [4 | 9, 4, _, _, _, _, ..]
                if Self::switch_of_other_link(s, &frame.radio) => {}
            [0, 4, elements @ ..] => {
                Self::process_channel_switch(s, elements, Source::ProtectedAction, None)
            }
            [category @ (4 | 9), 4, a, b, c, d, elements @ ..] => {
                let source = if *category == 9 {
                    Source::ProtectedAction
                } else {
                    Source::UnprotectedAction
                };
                Self::process_channel_switch(s, elements, source, Some([*a, *b, *c, *d]));
            }
            _ => return false,
        }
        true
    }

    /// The step an announced switch needs at `now`, if any.
    pub fn channel_switch_step(&self, now: u64) -> Option<ChannelSwitchStep> {
        let csa = &self.state().csa;
        if csa.dropped {
            return Some(ChannelSwitchStep::Disconnect);
        }
        let active = csa
            .active
            .as_ref()
            .filter(|active| active.switched_from.is_none())?;
        let definition = active.switch.definition;
        if !active.prepared {
            Some(ChannelSwitchStep::Prepare(definition))
        } else if now >= active.deadline {
            Some(ChannelSwitchStep::Switch(definition))
        } else {
            None
        }
    }

    /// The regulatory owner admitted the announced target.
    pub fn channel_switch_prepared(&mut self) {
        if let Some(active) = &mut self.state_mut().csa.active {
            active.prepared = true;
        }
    }

    /// The AP asked for silence until its switch: hold transmission, as
    /// `ieee80211_vif_block_queues_csa` stops the queues.
    pub fn channel_switch_blocks_tx(&self) -> bool {
        self.state().csa.blocked_tx
    }

    /// Whether `channel` is the one a switch just left, whose late frames are
    /// dropped rather than treated as the current AP's.
    pub(super) fn switched_from(s: &Owned<I, Q, C>, channel: (wifi::Band, u16, u16)) -> bool {
        s.csa
            .active
            .as_ref()
            .and_then(|active| active.switched_from)
            .is_some_and(|from| (from.band, from.number, from.frequency_mhz) == channel)
    }

    /// Linux `mt7925_csa_work`: remain on the target for a join, move the
    /// channel context there through `mt7925_mcu_set_chctx` and abort the
    /// remain-on-channel; then `ieee80211_chswitch_done` waits for the AP's
    /// beacon there.
    ///
    /// # Safety
    /// The exact-controller, applied-policy and revocation contracts of
    /// `update_edca_modern` apply, for the switch admission of the target.
    pub unsafe fn switch_channel_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        let definition = s
            .csa
            .active
            .as_ref()
            .filter(|active| active.prepared && active.switched_from.is_none())
            .ok_or(Error::WrongPhase)?
            .switch
            .definition;
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        // MT76_STATE_ROC: one remain-on-channel at a time.
        if s.roc_deadline.is_some() {
            return Err(Error::Busy);
        }
        Self::command_ready(s)?;
        let modern = s.modern.as_ref().ok_or(Error::WrongPhase)?;
        let context = modern.context().clone();
        let current = s.profile.ok_or(Error::WrongPhase)?;
        let mut target = current;
        target.channel = definition.control;
        let operation = wire::modern::Operation {
            center_channel: modern::center_channel(target.channel.band, definition.center1_mhz)?,
            second_center_channel: definition
                .center2_mhz
                .map(|mhz| modern::center_channel(target.channel.band, mhz))
                .transpose()?
                .unwrap_or(0),
            punctured: definition.punctured,
            ..modern.wire.operation()
        };
        let wire = modern
            .wire
            .switched(target, operation, definition.width)
            .map_err(Error::Codec)?;
        let token = s.roc_token.checked_add(1).ok_or(Error::NeedsDrain)?;
        let codec = Self::codec(s);
        let request = codec.roc(target, token, 4000).map_err(Error::Codec)?;
        let abort = codec.abort_roc(target, token).map_err(Error::Codec)?;
        let body = codec.channel(target, &definition).map_err(Error::Codec)?;
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
        s.transport_facts
            .try_reserve(2)
            .map_err(|_| Error::NoMemory)?;
        let Owned {
            station,
            controller,
            transport_facts,
            asynchronous,
            phase,
            roc_token,
            roc_deadline,
            profile,
            replies,
            ..
        } = s;
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            authority.publish_association(
                admission,
                station,
                wifi::regulatory::AssociationUse::ChannelSwitch,
                |_, _| {
                    let mut roc = radio::Roc {
                        controller,
                        transport_facts,
                        asynchronous,
                        phase,
                        roc_token,
                        roc_deadline,
                        profile,
                    };
                    Self::grant(&mut roc, target, token, &request, &abort)?;
                    *roc.phase = Phase::Quarantined;
                    Self::command_parts(roc.controller, replies, Command::BssInfo, &body)?;
                    // mt7925_abort_roc
                    let fact = roc
                        .controller
                        .state_mut()
                        .mailbox
                        .as_mut()
                        .unwrap()
                        .initial(&abort)
                        .map_err(Error::Transport)?;
                    roc.transport_facts.push(fact);
                    *roc.roc_deadline = None;
                    *roc.phase = Phase::Associated;
                    Ok(())
                },
            )
        }
        .map_err(Error::Regulatory)??;
        s.station
            .switch_channel(&context, definition)
            .map_err(Error::Station)?;
        s.modern.as_mut().ok_or(Error::WrongPhase)?.wire = wire;
        s.channel = None;
        s.channel_change = None;
        if let Some(active) = &mut s.csa.active {
            active.switched_from = Some(current.channel);
        }
        Ok(())
    }
}
