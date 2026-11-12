// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Scan/ROC command ownership, source grants and explicit native drain.
use super::*;

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    pub fn begin_scan(
        &mut self,
        request: scan::Request,
        now: u64,
        budget: u64,
        policy: &mut impl scan::Policy,
    ) -> Result<scan::Ticket, Error> {
        let s = self.state_mut();
        if s.phase != Phase::Idle {
            return Err(Error::Busy);
        }
        let actual_policy = s
            .controller
            .state()
            .policy
            .as_ref()
            .ok_or(Error::WrongProfile)?;
        if request.channels().iter().any(|ch| {
            ch.number > 255 || !actual_policy.initial_channel_present(ch.band, ch.number as u8)
        }) {
            return Err(Error::WrongProfile);
        }
        s.transport_facts
            .try_reserve(1)
            .map_err(|_| Error::NoMemory)?;
        let sequence = (s.scan_sequence + 1) & 127;
        // Complete fallible codec/capacity checks before Scanner owns the
        // request. A refused RNR list must leave this same owner reusable.
        let mut elements = [Vec::new(), Vec::new(), Vec::new()];
        // A 6 GHz probe carries the HE 6 GHz capability, as mac80211 builds it.
        let prepared = !request.extra_elements().is_empty()
            || (request.active()
                && request
                    .channels()
                    .iter()
                    .any(|ch| ch.band == wifi::Band::Ghz6));
        if prepared {
            let he6 = if actual_policy.capabilities().observed().six_ghz() == Some(true) {
                local::current(s, wifi::Band::Ghz6)
                    .map_err(Error::Local)?
                    .capabilities()
                    .he_6ghz()
                    .ok_or(Error::WrongProfile)?
            } else {
                0
            };
            for (index, band) in [wifi::Band::Ghz2, wifi::Band::Ghz5, wifi::Band::Ghz6]
                .into_iter()
                .enumerate()
            {
                if request.channels().iter().any(|ch| ch.band == band) {
                    let actual = local::current(s, band).map_err(Error::Local)?;
                    let domain = actual_policy
                        .programmed_mt7925()
                        .ok_or(Error::WrongProfile)?
                        .0;
                    let selection = local::scan::Selection::programmed(domain, band, he6);
                    elements[index] =
                        local::scan::with_extra(&actual, selection, request.extra_elements())
                            .map_err(Error::Local)?;
                }
            }
        }
        let encoded = wire::scan_with_elements(
            s.interface.registered_context().unwrap(),
            sequence,
            &request,
            prepared.then_some(&elements),
        )
        .map_err(Error::Codec)?;
        let ticket = s
            .scanner
            .begin(request, now, budget, policy)
            .map_err(Error::Scan)?;
        s.scan_ticket = Some(ticket);
        s.pmf_observations.clear();
        // From request ownership through transport, any unexpected failure
        // keeps an explicit non-Idle owner. A policy refusal before handoff
        // can withdraw only the never-submitted Starting generation.
        s.phase = Phase::Quarantined;
        if let Err(error) = s.scanner.validate_submission(ticket, now, policy) {
            s.scanner
                .withdraw_unsubmitted(ticket)
                .map_err(Error::Scan)?;
            s.scan_ticket = None;
            s.phase = Phase::Idle;
            return Err(Error::Scan(error));
        }
        s.scanner.submitted(ticket).map_err(Error::Scan)?;
        s.scan_sequence = sequence;
        let fact = s
            .controller
            .state_mut()
            .mailbox
            .as_mut()
            .unwrap()
            .initial(&encoded)
            .map_err(Error::Transport)?;
        s.transport_facts.push(fact);
        s.phase = Phase::Scanning;
        Ok(ticket)
    }
    /// The source cancel's generic completion is held, never treated as
    /// abort completion/drain. A terminal event or independent reset is
    /// still required before scan resource retirement.
    pub fn cancel_scan(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        let t = s.scan_ticket.ok_or(Error::WrongPhase)?;
        s.scanner.cancel(t).map_err(Error::Scan)?;
        let request = wire::cancel_scan(s.interface.registered_context().unwrap(), s.scan_sequence);
        s.transport_facts
            .try_reserve(1)
            .map_err(|_| Error::NoMemory)?;
        s.phase = Phase::Quarantined;
        let f = s
            .controller
            .state_mut()
            .mailbox
            .as_mut()
            .unwrap()
            .initial(&request)
            .map_err(Error::Transport)?;
        s.transport_facts.push(f);
        s.phase = Phase::Scanning;
        Ok(())
    }
    /// # Safety
    /// This completed scan's radio/channel restoration, DMA/IRQ/work and
    /// all delayed scan notifications are drained. The source basic event has
    /// no host generation cookie; an event alone cannot check reuse.
    pub unsafe fn retire_scan_after_drain(&mut self) -> Result<scan::Outcome, Error> {
        let s = self.state_mut();
        if s.phase != Phase::ScanCompleted {
            return Err(Error::WrongPhase);
        }
        if !s.pending_rx.is_empty()
            || !s.controller.state().data_assembly.is_idle()
            || !s
                .controller
                .state()
                .mailbox
                .as_ref()
                .unwrap()
                .receive_idle()
        {
            return Err(Error::NeedsDrain);
        }
        let outcome = s
            .scanner
            .retire(s.scan_ticket.unwrap(), scan::Retirement::Confirmed)
            .map_err(Error::Scan)?
            .ok_or(Error::NeedsDrain)?;
        s.scan_ticket = None;
        s.phase = Phase::Idle;
        Ok(outcome)
    }
    pub(super) fn abort_channel(s: &mut Owned<I, Q, C>) -> Result<(), Error> {
        if s.roc_deadline.is_none() {
            return Ok(());
        }
        let req = match s.six_ghz.as_ref() {
            Some(six) => six.codec().abort_roc(s.roc_token),
            None => s.profile.ok_or(Error::WrongProfile)?.abort_roc(s.roc_token),
        }
        .map_err(Error::Codec)?;
        s.transport_facts
            .try_reserve(1)
            .map_err(|_| Error::NoMemory)?;
        let old = s.phase;
        s.phase = Phase::Quarantined;
        let fact = s
            .controller
            .state_mut()
            .mailbox
            .as_mut()
            .unwrap()
            .initial(&req)
            .map_err(Error::Transport)?;
        s.transport_facts.push(fact);
        s.roc_deadline = None;
        s.phase = old;
        Ok(())
    }
    pub(super) fn reserve_channel(s: &mut Owned<I, Q, C>) -> Result<(), Error> {
        Self::abort_channel(s)?;
        // Source's 8-bit ROC token has no firmware epoch. Never silently reuse
        // one in this owner while an old notification could still exist.
        let token = s.roc_token.checked_add(1).ok_or(Error::NeedsDrain)?;
        let p = s.profile.ok_or(Error::WrongProfile)?;
        let req = p.roc(token, 1000).map_err(Error::Codec)?;
        let abort = p.abort_roc(token).map_err(Error::Codec)?;
        Self::await_grant(s, p, token, &req, &abort)
    }
    /// Publish one ROC request and wait for its matching grant, which assigns
    /// the profile's hardware band.
    pub(super) fn await_grant(
        s: &mut Owned<I, Q, C>,
        p: Profile,
        token: u8,
        req: &crate::wifi::mediatek::mcu::init::Request,
        abort: &crate::wifi::mediatek::mcu::init::Request,
    ) -> Result<(), Error> {
        Self::grant(&mut Roc::of(s), p, token, req, abort)
    }
    /// `await_grant` over the state a remain-on-channel request uses.
    pub(super) fn grant(
        s: &mut Roc<'_, I, Q, C>,
        p: Profile,
        token: u8,
        req: &crate::wifi::mediatek::mcu::init::Request,
        abort: &crate::wifi::mediatek::mcu::init::Request,
    ) -> Result<(), Error> {
        *s.phase = Phase::Quarantined;
        let grant = Self::wait_grant(s, token, req, abort, |g| {
            g.bss == p.interface.bss_index
                && g.channel == p.channel.number as u8
                && g.center == g.channel
                && g.center2 == 0
                && g.band
                    == match p.channel.band {
                        wifi::Band::Ghz2 => 1,
                        wifi::Band::Ghz5 => 2,
                        wifi::Band::Ghz6 => 3,
                    }
                && g.width == 0
                && g.sco == 0
                && g.request_type == 0
                && g.hardware_band <= 3
        })?;
        let mut actual = p;
        actual.interface.band_index = grant.hardware_band;
        *s.profile = Some(actual);
        *s.phase = Phase::Joining;
        Ok(())
    }
    /// Publish one ROC request and wait for the grant of `token` that
    /// `accepts` takes, holding the channel for the grant's duration; abort
    /// the request if none comes in time.
    pub(super) fn wait_grant(
        s: &mut Roc<'_, I, Q, C>,
        token: u8,
        req: &crate::wifi::mediatek::mcu::init::Request,
        abort: &crate::wifi::mediatek::mcu::init::Request,
        accepts: impl Fn(&wire::RocGrant) -> bool,
    ) -> Result<wire::RocGrant, Error> {
        s.transport_facts
            .try_reserve(1)
            .map_err(|_| Error::NoMemory)?;
        s.asynchronous
            .try_reserve(64)
            .map_err(|_| Error::NoMemory)?;
        *s.roc_token = token;
        let fact = s
            .controller
            .state_mut()
            .mailbox
            .as_mut()
            .unwrap()
            .initial(req)
            .map_err(Error::Transport)?;
        s.transport_facts.push(fact);
        let clock = s.controller.state().control.clock;
        let deadline = clock.deadline(4_000_000);
        loop {
            let frames = s
                .controller
                .state_mut()
                .mailbox
                .as_mut()
                .unwrap()
                .poll_events()
                .map_err(|e| Error::Bootstrap(super::super::Error::Mailbox(e)))?;
            let mut match_grant = None;
            for f in frames {
                s.asynchronous.try_reserve(1).map_err(|_| Error::NoMemory)?;
                s.asynchronous.push(f);
                let retained = s.asynchronous.last().unwrap();
                if !retained.drop
                    && let Ok(e) =
                        crate::wifi::mediatek::mcu::Event::parse(Chip::Mt7925, &retained.bytes)
                    && e.is_unsolicited()
                    && e.event_id == 0x27
                {
                    for g in wire::roc_grants(e.payload).map_err(Error::Codec)? {
                        if g.token == token && g.duration_ms > 0 && accepts(&g) {
                            match_grant = Some(g);
                        }
                    }
                }
            }
            if let Some(g) = match_grant {
                *s.roc_deadline = Some(
                    clock
                        .now_us()
                        .checked_add(u64::from(g.duration_ms) * 1000)
                        .ok_or(Error::NeedsDrain)?,
                );
                return Ok(g);
            }
            if deadline.passed() {
                // Source set_roc's timeout sends the matching abort.
                // Generic abort completion still cannot retire uncertain radio
                // users or establish a new peer/notification epoch.
                s.transport_facts
                    .try_reserve(1)
                    .map_err(|_| Error::NoMemory)?;
                let fact = s
                    .controller
                    .state_mut()
                    .mailbox
                    .as_mut()
                    .unwrap()
                    .initial(abort)
                    .map_err(Error::Transport)?;
                s.transport_facts.push(fact);
                return Err(Error::GrantTimeout);
            }
            core::hint::spin_loop();
        }
    }
}

/// The owned state a remain-on-channel request uses, borrowed apart from the
/// station a regulatory publication holds meanwhile.
pub(super) struct Roc<'a, I: BarIo, Q: QueueIo, C: 'static> {
    pub(super) controller: &'a mut Controller<I, Q, C>,
    pub(super) transport_facts: &'a mut Vec<mailbox::init::Fact>,
    pub(super) asynchronous: &'a mut Vec<crate::wifi::mediatek::ring::rx::Frame>,
    pub(super) phase: &'a mut Phase,
    pub(super) roc_token: &'a mut u8,
    pub(super) roc_deadline: &'a mut Option<u64>,
    pub(super) profile: &'a mut Option<Profile>,
}
impl<'a, I: BarIo, Q: QueueIo, C: 'static> Roc<'a, I, Q, C> {
    pub(super) fn of(s: &'a mut Owned<I, Q, C>) -> Self {
        Self {
            controller: &mut s.controller,
            transport_facts: &mut s.transport_facts,
            asynchronous: &mut s.asynchronous,
            phase: &mut s.phase,
            roc_token: &mut s.roc_token,
            roc_deadline: &mut s.roc_deadline,
            profile: &mut s.profile,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::*;
    use super::*;
    use crate::Mac;
    use crate::wifi::mediatek::bootstrap::tests::NativeFixture;
    use crate::wifi::mediatek::mailbox::clc::remaining::tests::caps;
    use crate::wifi::mediatek::ring::rx::tests::device_receive;
    use crate::wifi::mediatek::ring::tests::Source;
    use crate::wifi::{Band, Channel};
    use stela_driver_base::stepped::block_on;

    const SIX: Channel = Channel {
        band: Band::Ghz6,
        number: 1,
        frequency_mhz: 5955,
    };
    struct PassivePolicy {
        receive: bool,
    }
    impl scan::Policy for PassivePolicy {
        fn allow_receive(&mut self, channel: Channel) -> bool {
            self.receive && channel == SIX
        }
        fn allow_probe(&mut self, _: Channel) -> bool {
            panic!("passive scan must not request probe authority")
        }
        fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
            (band == SIX.band && number == SIX.number).then_some(SIX)
        }
    }
    impl station::RegulatoryPolicy for PassivePolicy {
        fn allow_management_tx(&mut self, _: Channel) -> bool {
            false
        }
        fn allow_data_tx(&mut self, _: Channel) -> bool {
            false
        }
    }

    #[test]
    fn changed_prepublication_policy_withdraws_native_request_before_reuse() {
        if crate::test_process::isolate_model() {
            return;
        }
        struct Changing(usize);
        impl scan::Policy for Changing {
            fn allow_receive(&mut self, _: Channel) -> bool {
                self.0 += 1;
                self.0 == 1
            }
            fn allow_probe(&mut self, _: Channel) -> bool {
                true
            }
            fn lookup_channel(&mut self, _: Band, _: u16) -> Option<Channel> {
                Some(CHANNEL)
            }
        }
        let (mut driver, model) = setup();
        let before = model.response.lock().unwrap().client_commands.len();
        let request = scan::Request::new(vec![CHANNEL], vec![vec![]], vec![]).unwrap();
        let mut policy = Changing(0);
        assert!(matches!(
            driver.begin_scan(request, 0, 1_000_000, &mut policy),
            Err(Error::Scan(scan::Error::Regulatory))
        ));
        assert_eq!(policy.0, 2);
        assert_eq!(driver.phase(), Phase::Idle);
        assert_eq!(driver.state().scanner.phase(), scan::Phase::Idle);
        assert!(driver.state().scan_ticket.is_none());
        assert!(driver.state().scanner.request().is_none());
        assert_eq!(driver.state().scan_sequence, 0);
        assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
        let request = scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap();
        driver
            .begin_scan(request, 1, 1_000_000, &mut Policy(true))
            .unwrap();
        assert_eq!(driver.phase(), Phase::Scanning);
        assert_eq!(driver.state().scan_sequence, 1);
        assert_eq!(
            model.response.lock().unwrap().client_commands.len(),
            before + 1
        );
        let mut done = vec![0; 52];
        done[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
        done[36] = 14;
        done[38] = 6;
        done[50] = 4;
        device_receive(&model.queues[&0xd4500], &done, true, 0);
        driver.poll(2, &Source, &mut Policy(true)).unwrap();
        assert_eq!(driver.phase(), Phase::ScanCompleted);
        // SAFETY: the fixture consumed all scan RX/notifications; no
        // external hardware or delayed descriptor users exist in this model.
        unsafe { driver.retire_scan_after_drain() }.unwrap();
        driver.stop().unwrap();
        model.response.lock().unwrap().clc_processing = false;
        model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
        // SAFETY: model stop/reset has drained every DMA user.
        unsafe { driver.stop_after_quiesce() }.unwrap();
        drop(driver);
        assert_eq!(model.drops.load(core::sync::atomic::Ordering::Relaxed), 1);
        assert!(model.memory.lock().unwrap().mappings.is_empty());
    }

    #[test]
    fn mt7925_six_ghz_passive_scan_uses_programmed_policy_and_owned_dma() {
        if crate::test_process::isolate_model() {
            return;
        }
        let mut f = NativeFixture::new(Chip::Mt7925);
        f.model.lock().unwrap().capability_payload = Some(caps(Chip::Mt7925, [true; 3]));
        f.install().unwrap();
        f.boot(Chip::Mt7925);
        f.policy_bands(Chip::Mt7925, [true; 3]);
        // SAFETY: this fixture owns the real modeled controller, programmed
        // policy and firmware. No unrelated/caller-copied receipt enables it.
        unsafe { block_on(f.controller.initialize_mac()) }.unwrap();
        // SAFETY: source-checked modeled 5GHz initial radio ownership; the
        // hardware scan command will select the independently programmed 6GHz.
        unsafe { block_on(f.controller.start(Band::Ghz5, 36, 5180)) }.unwrap();
        // SAFETY: exact fixture-owned interface and controller resource.
        let interface =
            unsafe { block_on(f.controller.register_interface(profile().interface)) }.unwrap();
        f.model.lock().unwrap().client_execution = true;
        let NativeFixture {
            controller,
            model: response_model,
            bar,
            queues,
            memory,
            drops,
            ..
        } = f;
        let model = Model {
            response: response_model,
            bar,
            queues,
            memory,
            drops,
        };
        // SAFETY: the just-registered matching interface and native controller.
        let mut driver = unsafe { Client::new(controller, interface) }.unwrap();
        let before = model.response.lock().unwrap().client_commands.len();
        let request = scan::Request::new(vec![SIX], vec![], vec![]).unwrap();
        assert!(matches!(
            driver.begin_scan(request, 0, 1_000_000, &mut PassivePolicy { receive: false }),
            Err(Error::Scan(scan::Error::Regulatory))
        ));
        assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
        assert_eq!(driver.phase(), Phase::Idle);
        let request = scan::Request::new(vec![SIX], vec![], vec![]).unwrap();
        driver
            .begin_scan(request, 0, 1_000_000, &mut PassivePolicy { receive: true })
            .unwrap();
        let commands = model.response.lock().unwrap().client_commands.clone();
        let scan_body = &commands.last().unwrap()[48..];
        // No probe generation, and the channel list uses band code 3.
        assert_eq!(scan_body[8..10], [0, 0]);
        let mut rest = &scan_body[4..];
        let mut channels = None;
        while !rest.is_empty() {
            let length = u16::from_le_bytes(rest[2..4].try_into().unwrap()) as usize;
            if rest[..2] == 12u16.to_le_bytes() {
                channels = Some(rest[4..length].to_vec());
            }
            rest = &rest[length..];
        }
        assert_eq!(&channels.unwrap()[4..6], &[3, 1]);
        let mut beacon = response(
            0x80,
            &[
                0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0x11, 0, 0, 1, b'x', 255, 12, 36, 0, 0, 2, 0, 0, 0,
                1, 0, 1, 0, 6,
            ],
        );
        beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
        let mut raw = incoming_bytes(&beacon, None, false);
        raw[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
        // A real 6GHz AP may publish unsolicited broadcast probe responses;
        // source mac80211 accepts these without OCE or probe-TX permission.
        beacon[..2].copy_from_slice(&0x50u16.to_le_bytes());
        let mut probe = incoming_bytes(&beacon, None, false);
        probe[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
        device_receive(&model.queues[&0xd4520], &probe, true, 0);
        let events = driver
            .poll(1, &Source, &mut PassivePolicy { receive: true })
            .unwrap();
        assert!(matches!(events.as_slice(), [Event::Bss(bss)]
            if bss.channel == SIX && bss.observed.band == Band::Ghz6));
        // The native poll consumes one data-ring frame per call.
        let events = driver
            .poll(2, &Source, &mut PassivePolicy { receive: true })
            .unwrap();
        assert!(matches!(events.as_slice(), [Event::Bss(probe)]
            if probe.channel == SIX && probe.kind == scan::result::Kind::ProbeResponse));
        assert_eq!(driver.phase(), Phase::Scanning);
        let mut done = vec![0; 52];
        done[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
        done[36] = 14;
        done[38] = 6;
        done[50] = 4;
        device_receive(&model.queues[&0xd4500], &done, true, 0);
        driver
            .poll(3, &Source, &mut PassivePolicy { receive: true })
            .unwrap();
        assert_eq!(driver.phase(), Phase::ScanCompleted);
        // SAFETY: all modeled scan receive descriptors/notifications
        // were consumed; no real external DMA/IRQ user exists in this fixture.
        unsafe { driver.retire_scan_after_drain() }.unwrap();
        assert_eq!(driver.phase(), Phase::Idle);
        driver.stop().unwrap();
    }
}
