// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Domain/SAR-rate-power continuation of the same CLC owner.
//! Completion observations never implement a trusted HardwarePolicy provider.
use super::{Execution, Mailbox, PCI_TIMEOUT_US, Packet, Phase, QueueIo, Reply};
use crate::wifi::mediatek::{
    identity::Chip,
    mcu::{Event, domain, power},
};
use crate::wifi::stepped::yield_now;
use alloc::vec::Vec;

pub struct Inputs<'a> {
    /// `clc_chan_conf` once MTCL masked it: within the UNII flags of the
    /// last CLC rule sent.
    pub channel_configuration: u8,
    pub domain: domain::Inputs<'a>,
    pub power: power::Inputs<'a>,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Step {
    Domain,
    Power { request: usize, kind: power::Kind },
}
#[derive(Debug)]
pub enum Observation {
    OneWayDmaCompleted,
    GenericResponse(Reply),
    PseRead {
        reply: Reply,
        register: u32,
        value: u32,
    },
}
#[derive(Debug)]
pub struct Fact {
    pub step: Step,
    pub sequence: u8,
    pub observed: Observation,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Clc(super::Error),
    Domain(domain::Error),
    Power(power::Error),
    AlreadyPrepared,
    NotPrepared,
    WrongPhase,
    WrongCountry,
    CapabilitiesIncomplete,
    BandSelection,
    /// The channel configuration enables UNII bands the CLC did not.
    ChannelConfiguration,
    NoMemory,
    Transport {
        step: Step,
        failure: super::TransportFailure,
    },
    Malformed {
        step: Step,
        error: power::Error,
    },
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek domain/SAR-rate-power execution: {self:?}")
    }
}
impl core::error::Error for Error {}
pub(super) struct Chain {
    domain: domain::Request,
    power: power::Plan,
    facts: Vec<Fact>,
    interrupted: Option<Reply>,
    start_facts: Vec<Fact>,
}

impl<C: 'static> Execution<C> {
    /// # Safety
    /// Extend the existing kept `resource` permission/ownership to
    /// these exact channel and resolved per-channel/per-rate half-dBm inputs,
    /// resolved after consuming this owner's completed CLC facts.
    /// They must derive from chip/board/chain, completed CLC restrictions
    /// (including MT7925 BE_CTRL), requested power, trusted current OS/location
    /// domain, SAR and ACPI/DTS limits. Absence must be proven;
    /// missing values cannot use zero/127/unlimited defaults. For every source
    /// fixed power-list channel, limits faithfully represent all applicable
    /// constraints; disabled channels remain disabled in the domain. The real
    /// guards and board/chain/source-policy resources must remain held by C
    /// through all uncertain/completed firmware/radio users, with raw changes
    /// and revocation serialized as in `prepare`. Copied raw flags or arrays
    /// alone cannot satisfy this. This method creates no applied-policy receipt.
    pub unsafe fn prepare_remaining<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &Mailbox<W, F, R>,
        inputs: Inputs<'_>,
    ) -> Result<(), Error> {
        if inputs.channel_configuration & !self.channel_configuration() != 0 {
            return Err(Error::ChannelConfiguration);
        }
        let state = self.owned_mut();
        if state.owner != mailbox.owner {
            return Err(Error::Clc(super::Error::WrongMailbox));
        }
        if state.phase != Phase::CompletedFacts {
            return Err(Error::WrongPhase);
        }
        if state.remaining.is_some() {
            return Err(Error::AlreadyPrepared);
        }
        Self::check_inputs(
            mailbox,
            &state.auxiliary,
            &state.capabilities,
            &state.eeprom,
        )
        .map_err(Error::Clc)?;
        if inputs.domain.alpha2 != state.plan.requested_alpha2()
            || inputs.power.alpha2 != state.plan.requested_alpha2()
        {
            return Err(Error::WrongCountry);
        }
        let phy = state
            .capabilities
            .observed()
            .phy()
            .ok_or(Error::CapabilitiesIncomplete)?;
        // Optional absent6GHz observation selects no6GHz source protocol; the
        // original None remains in the owned report and grants no band.
        let mut expected = [
            phy.has_2ghz(),
            phy.has_5ghz(),
            state.capabilities.observed().six_ghz().unwrap_or(false),
        ];
        if mailbox.identity.chip == Chip::Mt7925 {
            // mt7925_mcu_set_rate_txpower skips 6 GHz without a channel
            // configuration.
            expected[2] &= inputs.channel_configuration != 0;
        }
        let domain =
            domain::Request::build(mailbox.identity.chip, inputs.domain).map_err(Error::Domain)?;
        let power =
            power::Plan::build(mailbox.identity.chip, inputs.power).map_err(Error::Power)?;
        if power.bands() != expected
            || domain
                .counts()
                .iter()
                .zip(expected)
                .any(|(&n, allowed)| n != 0 && !allowed)
        {
            return Err(Error::BandSelection);
        }
        let mut facts = Vec::new();
        facts
            .try_reserve_exact(1 + power.requests().len())
            .map_err(|_| Error::NoMemory)?;
        state.remaining = Some(Chain {
            domain,
            power,
            facts,
            interrupted: None,
            start_facts: Vec::new(),
        });
        Ok(())
    }
    pub fn remaining_facts(&self) -> &[Fact] {
        self.owned()
            .remaining
            .as_ref()
            .map_or(&[], |c| c.facts.as_slice())
    }
    pub fn interrupted_remaining_reply(&self) -> Option<&Reply> {
        self.owned()
            .remaining
            .as_ref()
            .and_then(|c| c.interrupted.as_ref())
    }
    pub fn remaining_completed(&self) -> bool {
        self.owned().phase == Phase::RemainingFacts
    }
    /// Continue the completed CLC with domain and complete source rate-
    /// power/PSE exactly once under the same permission. CLC is never
    /// restarted/replayed. Return means raw protocol facts, including source
    /// one-way boundaries, never an admitted radio or policy.
    pub async fn execute_all<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<(), Error> {
        self.execute_all_timed(mailbox, PCI_TIMEOUT_US).await
    }
    /// A step per request.
    pub(in super::super) async fn execute_all_timed<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
        timeout_us: u64,
    ) -> Result<(), Error> {
        if self.owned().remaining.is_none() {
            return Err(Error::NotPrepared);
        }
        let state = self.owned_mut();
        if state.owner != mailbox.owner {
            return Err(Error::Clc(super::Error::WrongMailbox));
        }
        if state.phase != Phase::CompletedFacts {
            return Err(Error::WrongPhase);
        }
        Self::check_inputs(
            mailbox,
            &state.auxiliary,
            &state.capabilities,
            &state.eeprom,
        )
        .map_err(Error::Clc)?;
        let chain = state.remaining.as_mut().unwrap();
        state.phase = Phase::RemainingUncertain;
        for index in 0..=chain.power.requests().len() {
            let (step, packet) = if index == 0 {
                (
                    Step::Domain,
                    Packet::Domain(mailbox.encoder.encode_domain(&chain.domain)),
                )
            } else {
                let request = index - 1;
                let body = &chain.power.requests()[request];
                (
                    Step::Power {
                        request,
                        kind: body.kind(),
                    },
                    Packet::Power(mailbox.encoder.encode_power(body)),
                )
            };
            let sequence = packet.sequence();
            let executed = mailbox.execute(&packet, timeout_us);
            yield_now().await;
            if let Err(failure) = executed {
                chain.interrupted = mailbox.interrupted_regulatory_reply.take();
                return Err(Error::Transport { step, failure });
            }
            let observed = if !packet.expects_response() {
                Observation::OneWayDmaCompleted
            } else {
                let reply = mailbox
                    .interrupted_regulatory_reply
                    .take()
                    .expect("actual completed domain/power response");
                if matches!(
                    step,
                    Step::Power {
                        kind: power::Kind::PseFlush,
                        ..
                    }
                ) {
                    let event = Event::parse(mailbox.identity.chip, &reply.frame.bytes)
                        .expect("matched actual PSE response");
                    match power::flush_value(event.payload) {
                        Ok((register, value)) => Observation::PseRead {
                            reply,
                            register,
                            value,
                        },
                        Err(error) => {
                            chain.interrupted = Some(reply);
                            return Err(Error::Malformed { step, error });
                        }
                    }
                } else {
                    Observation::GenericResponse(reply)
                }
            };
            chain.facts.push(Fact {
                step,
                sequence,
                observed,
            });
        }
        state.phase = Phase::RemainingFacts;
        Ok(())
    }
}

impl<C: 'static> Execution<C> {
    pub fn start_facts(&self) -> &[Fact] {
        self.owned()
            .remaining
            .as_ref()
            .map_or(&[], |c| c.start_facts.as_slice())
    }
    /// Vendor initial lifecycle only: source repeats the same resolved domain
    /// at first start. This does not replay CLC or manufacture new permission.
    pub(in crate::wifi::mediatek) async fn start_domain<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<(), Error> {
        let s = self.owned_mut();
        if s.owner != mailbox.owner {
            return Err(Error::Clc(super::Error::WrongMailbox));
        }
        if s.phase != Phase::RemainingFacts {
            return Err(Error::WrongPhase);
        }
        Self::check_inputs(mailbox, &s.auxiliary, &s.capabilities, &s.eeprom)
            .map_err(Error::Clc)?;
        let chain = s.remaining.as_mut().ok_or(Error::NotPrepared)?;
        chain
            .start_facts
            .try_reserve_exact(1 + chain.power.requests().len())
            .map_err(|_| Error::NoMemory)?;
        s.phase = Phase::StartUncertain;
        let packet = Packet::Domain(mailbox.encoder.encode_domain(&chain.domain));
        let exchanged = Self::start_exchange(mailbox, chain, Step::Domain, packet);
        yield_now().await;
        exchanged?;
        s.phase = Phase::StartDomainFacts;
        Ok(())
    }
    pub(in crate::wifi::mediatek) async fn start_power<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<(), Error> {
        let s = self.owned_mut();
        if s.owner != mailbox.owner {
            return Err(Error::Clc(super::Error::WrongMailbox));
        }
        if s.phase != Phase::StartDomainFacts || mailbox.identity.chip == Chip::Mt7925 {
            return Err(Error::WrongPhase);
        }
        Self::check_inputs(mailbox, &s.auxiliary, &s.capabilities, &s.eeprom)
            .map_err(Error::Clc)?;
        s.phase = Phase::StartUncertain;
        let chain = s.remaining.as_mut().ok_or(Error::NotPrepared)?;
        for request in 0..chain.power.requests().len() {
            let body = &chain.power.requests()[request];
            let step = Step::Power {
                request,
                kind: body.kind(),
            };
            let packet = Packet::Power(mailbox.encoder.encode_power(body));
            let exchanged = Self::start_exchange(mailbox, chain, step, packet);
            yield_now().await;
            exchanged?;
        }
        s.phase = Phase::StartPowerFacts;
        Ok(())
    }
    fn start_exchange<W: QueueIo, F: QueueIo, R: QueueIo>(
        mailbox: &mut Mailbox<W, F, R>,
        chain: &mut Chain,
        step: Step,
        packet: Packet,
    ) -> Result<(), Error> {
        let sequence = packet.sequence();
        if let Err(failure) = mailbox.execute(&packet, PCI_TIMEOUT_US) {
            chain.interrupted = mailbox.interrupted_regulatory_reply.take();
            return Err(Error::Transport { step, failure });
        }
        let observed = if !packet.expects_response() {
            Observation::OneWayDmaCompleted
        } else {
            let reply = mailbox
                .interrupted_regulatory_reply
                .take()
                .expect("actual initial repeat response");
            if matches!(
                step,
                Step::Power {
                    kind: power::Kind::PseFlush,
                    ..
                }
            ) {
                let event = Event::parse(mailbox.identity.chip, &reply.frame.bytes)
                    .expect("actual PSE event");
                match power::flush_value(event.payload) {
                    Ok((register, value)) => Observation::PseRead {
                        reply,
                        register,
                        value,
                    },
                    Err(error) => {
                        chain.interrupted = Some(reply);
                        return Err(Error::Malformed { step, error });
                    }
                }
            } else {
                Observation::GenericResponse(reply)
            }
        };
        chain.start_facts.push(Fact {
            step,
            sequence,
            observed,
        });
        Ok(())
    }
}

impl<C: 'static> Execution<C> {
    /// Private view of this owner's completed MT7925 chain.
    /// No public copied receipt can manufacture a programmed policy.
    pub(in crate::wifi::mediatek) fn programmed_mt7925(
        &self,
    ) -> Option<(&domain::Request, &power::Plan)> {
        let state = self.owned();
        let chain = state.remaining.as_ref()?;
        (state.phase == Phase::StartDomainFacts
            && chain.domain.chip() == Chip::Mt7925
            && chain.facts.len() == 1 + chain.power.requests().len()
            && chain.start_facts.len() == 1
            && chain.interrupted.is_none())
        .then_some((&chain.domain, &chain.power))
    }
    pub(in crate::wifi::mediatek) fn initial_channel_present(
        &self,
        band: crate::wifi::Band,
        channel: u8,
    ) -> bool {
        let Some(c) = self.owned().remaining.as_ref() else {
            return false;
        };
        let counts = c.domain.counts();
        let index = match band {
            crate::wifi::Band::Ghz2 => 0,
            crate::wifi::Band::Ghz5 => 1,
            crate::wifi::Band::Ghz6 => 2,
        };
        let first: usize = counts[..index].iter().map(|&n| usize::from(n)).sum();
        let header = if c.domain.chip() == Chip::Mt7925 {
            16
        } else {
            12
        };
        c.domain.body()[header..]
            .as_chunks::<8>()
            .0
            .iter()
            .skip(first)
            .take(counts[index] as usize)
            .any(|e| u16::from_le_bytes([e[0], e[1]]) == u16::from(channel))
    }
}

#[cfg(test)]
pub(crate) mod tests;
