// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Native CLC execution keeping the stopped-radio/source-policy owner
//! and exact downloaded input. Raw completions never check applied policy.
use super::{CapabilityReport, Mailbox, PCI_TIMEOUT_US, Packet, QueueIo, TransportFailure};
use crate::wifi::mediatek::{
    firmware::{
        auxiliary::{self, Auxiliary},
        clc::{self as plan, Inputs, Plan},
    },
    mcu::{Event, clc as codec},
    ring::rx::Frame,
};
use crate::wifi::stepped::yield_now;
use alloc::{alloc::alloc, boxed::Box, vec::Vec};
use core::alloc::Layout;

pub mod remaining;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Reply {
    pub sequence: u8,
    pub dma_released: bool,
    pub frame: Frame,
}
/// Only the source protocol observation. Neither variant proves that
/// a country, channel, power limit or radio permission has been applied.
#[derive(Debug)]
pub enum Fact {
    OneWayDmaCompleted {
        request: usize,
        sequence: u8,
    },
    Response {
        request: usize,
        reply: Reply,
        observed: codec::Response,
    },
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    WrongMailbox,
    PoisonedMailbox,
    MissingImageBinding,
    WrongImage,
    WrongReports,
    ChipFlagsMismatch,
    Selection(auxiliary::Error),
    Plan(plan::Error),
    NoMemory,
    AlreadyAttempted,
    Transport {
        request: usize,
        failure: TransportFailure,
    },
    Malformed {
        request: usize,
        error: codec::Error,
    },
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek native CLC execution: {self:?}")
    }
}
impl core::error::Error for Error {}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    Fresh,
    Quarantined,
    CompletedFacts,
    RemainingUncertain,
    RemainingFacts,
    StartUncertain,
    StartDomainFacts,
    StartPowerFacts,
    Retired,
}
struct Owned<C> {
    resource: C,
    owner: u64,
    auxiliary: Auxiliary,
    capabilities: CapabilityReport,
    eeprom: super::eeprom::Report,
    plan: Plan,
    inputs: Inputs,
    phase: Phase,
    facts: Vec<Fact>,
    interrupted: Option<Reply>,
    remaining: Option<remaining::Chain>,
}
/// The permission/resource guard is owned, not a borrowed revision or a
/// promise ending on Err. Any publication keeps this entire owner on Drop,
/// including successful raw completion, until independent quiescence.
pub struct Execution<C: 'static> {
    owned: Option<Box<Owned<C>>>,
}

impl<C: 'static> Execution<C> {
    /// # Safety
    /// `resource` keeps the exclusive function/controller claim and
    /// currently stopped/drained radio, firmware/reset/image epoch, and
    /// trusted source country/location/ACPI/MTCL/power-limit resources used by
    /// `inputs`. These resources/permission guards must remain valid
    /// through partial/uncertain firmware processing and completed raw facts,
    /// even if this value/mailbox is dropped or its DMA queues are released.
    /// No old radio/policy users remain. Serialize all raw firmware/reset/radio
    /// and policy mutations with this owner for its entire lifetime.
    /// Changing/revoking those permissions requires stopping/canceling/draining
    /// real pending radio/firmware/IRQ/work users or source-proven enforcement.
    /// Retaining copied integers or an Arc unrelated to those resources
    /// cannot satisfy this contract. The bound observations below do not
    /// authenticate source-opaque EEPROM fields or provide these permissions.
    /// This executes only CLC; the full CLC→domain→SAR/rate-power chain
    /// and trusted OS intersection remain required before any radio admission.
    pub unsafe fn prepare<W: QueueIo, F: QueueIo, R: QueueIo>(
        mailbox: &Mailbox<W, F, R>,
        auxiliary: Auxiliary,
        capabilities: CapabilityReport,
        eeprom: super::eeprom::Report,
        inputs: Inputs,
        resource: C,
    ) -> Result<Self, Error> {
        Self::check_inputs(mailbox, &auxiliary, &capabilities, &eeprom)?;
        // A completed optional-TLV-absent query is distinct from no
        // query and from observed Some(0). The pinned zero-initialized source
        // chip_cap selects one-way C2 protocol in that case, not a capability
        // or policy permission. Preserve the original None in the owned report.
        if inputs.chip_capabilities != capabilities.observed().chip_flags().unwrap_or(0) {
            return Err(Error::ChipFlagsMismatch);
        }
        let selection = auxiliary
            .select_clc(eeprom.observed().byte())
            .map_err(Error::Selection)?;
        let plan = Plan::build(&selection, inputs).map_err(Error::Plan)?;
        let mut facts = Vec::new();
        facts
            .try_reserve_exact(plan.requests().len())
            .map_err(|_| Error::NoMemory)?;
        // Reserve the owner itself fallibly before the first device mutation.
        // Existing plan/encoder/RX transport allocations keep their inherited
        // allocation contracts; this is not a claim that they are all fallible.
        // SAFETY: `Owned` is not zero-sized.
        let storage = unsafe { alloc(Layout::new::<Owned<C>>()) }.cast::<Owned<C>>();
        if storage.is_null() {
            return Err(Error::NoMemory);
        }
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            storage.write(Owned {
                resource,
                owner: mailbox.owner,
                auxiliary,
                capabilities,
                eeprom,
                plan,
                inputs,
                phase: Phase::Fresh,
                facts,
                interrupted: None,
                remaining: None,
            });
            Ok(Self {
                owned: Some(Box::from_raw(storage)),
            })
        }
    }
    fn check_inputs<W: QueueIo, F: QueueIo, R: QueueIo>(
        mailbox: &Mailbox<W, F, R>,
        auxiliary: &Auxiliary,
        capabilities: &CapabilityReport,
        eeprom: &super::eeprom::Report,
    ) -> Result<(), Error> {
        if mailbox.is_poisoned() {
            return Err(Error::PoisonedMailbox);
        }
        let binding = mailbox
            .download_binding()
            .ok_or(Error::MissingImageBinding)?;
        if !binding.matches_auxiliary(auxiliary) {
            return Err(Error::WrongImage);
        }
        if !capabilities.belongs_to(mailbox)
            || !eeprom.belongs_to(mailbox)
            || capabilities.identity() != mailbox.identity
            || eeprom.identity() != mailbox.identity
        {
            return Err(Error::WrongReports);
        }
        Ok(())
    }
    fn owned(&self) -> &Owned<C> {
        self.owned.as_deref().expect("owned native CLC context")
    }
    fn owned_mut(&mut self) -> &mut Owned<C> {
        self.owned.as_deref_mut().expect("owned native CLC context")
    }
    pub fn resource(&self) -> &C {
        &self.owned().resource
    }
    pub fn auxiliary(&self) -> &Auxiliary {
        &self.owned().auxiliary
    }
    pub fn capabilities(&self) -> &CapabilityReport {
        &self.owned().capabilities
    }
    pub fn eeprom(&self) -> &super::eeprom::Report {
        &self.owned().eeprom
    }
    pub fn plan(&self) -> &Plan {
        &self.owned().plan
    }
    /// `clc_chan_conf`: the UNII flags of the last CLC rule sent; Connac2's
    /// rules carry none.
    pub fn channel_configuration(&self) -> u8 {
        self.plan()
            .requests()
            .iter()
            .filter_map(|r| r.pending_channel_flags())
            .next_back()
            .unwrap_or(0)
    }
    /// Original source observations kept with their guard.
    /// This is not a new applied-policy or radio-permission receipt.
    pub(in crate::wifi::mediatek) fn source_inputs(&self) -> Inputs {
        self.owned().inputs
    }
    pub fn facts(&self) -> &[Fact] {
        &self.owned().facts
    }
    pub fn interrupted_reply(&self) -> Option<&Reply> {
        self.owned().interrupted.as_ref()
    }
    pub fn is_quarantined(&self) -> bool {
        matches!(
            self.owned().phase,
            Phase::Quarantined | Phase::RemainingUncertain
        )
    }
    pub fn completed_facts(&self) -> bool {
        matches!(
            self.owned().phase,
            Phase::CompletedFacts | Phase::RemainingFacts
        )
    }

    /// Publish each source request once through the existing
    /// mailbox/DMA/sequence owner. Successful return is raw protocol completion
    /// only; keep the stopped radio and policy resources kept afterward.
    pub async fn execute<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<(), Error> {
        self.execute_timed(mailbox, PCI_TIMEOUT_US).await
    }

    /// A step per request.
    pub(super) async fn execute_timed<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
        timeout_us: u64,
    ) -> Result<(), Error> {
        let state = self.owned_mut();
        if state.owner != mailbox.owner {
            return Err(Error::WrongMailbox);
        }
        if state.phase != Phase::Fresh {
            return Err(Error::AlreadyAttempted);
        }
        Self::check_inputs(
            mailbox,
            &state.auxiliary,
            &state.capabilities,
            &state.eeprom,
        )?;
        // From now even a partial/uncertain publication may have changed real
        // firmware. No error or Drop may release the context owner.
        state.phase = Phase::Quarantined;
        for (request, body) in state.plan.requests().iter().enumerate() {
            debug_assert_eq!(body.chip(), mailbox.identity.chip);
            let packet = Packet::Clc(mailbox.encoder.encode_clc(body));
            let executed = mailbox.execute(&packet, timeout_us);
            yield_now().await;
            if let Err(failure) = executed {
                state.interrupted = mailbox.interrupted_clc_reply.take();
                return Err(Error::Transport { request, failure });
            }
            let Packet::Clc(packet) = packet else {
                unreachable!()
            };
            if packet.completion() == plan::Completion::OneWay {
                state.facts.push(Fact::OneWayDmaCompleted {
                    request,
                    sequence: packet.sequence(),
                });
                continue;
            }
            let reply = mailbox
                .interrupted_clc_reply
                .take()
                .expect("actual completed CLC response");
            let event =
                Event::parse(mailbox.identity.chip, &reply.frame.bytes).expect("matched CLC event");
            let observed = match codec::Response::parse(&packet, event.payload) {
                Ok(observed) => observed,
                Err(error) => {
                    state.interrupted = Some(reply);
                    return Err(Error::Malformed { request, error });
                }
            };
            state.facts.push(Fact::Response {
                request,
                reply,
                observed,
            });
        }
        state.phase = Phase::CompletedFacts;
        Ok(())
    }
    /// # Safety
    /// Independently stop/cancel/drain firmware command processing and
    /// every radio TX/RX, policy/image consumer, IRQ/deferred worker and DMA user
    /// of this real context, including uncertain requests/late completions.
    /// WFDMA idle, host timeout, queue removal and one-way DMA completion alone
    /// are insufficient. This neither checks policy nor resets/rebinds the
    /// mailbox. The caller still owns any independent mailbox/queue teardown.
    pub unsafe fn retire_after_quiesce(&mut self) {
        self.owned_mut().phase = Phase::Retired;
    }
}
impl<C: 'static> Drop for Execution<C> {
    fn drop(&mut self) {
        if let Some(owned) = self.owned.take()
            && !matches!(owned.phase, Phase::Fresh | Phase::Retired)
        {
            core::mem::forget(owned);
        }
    }
}

#[cfg(test)]
pub(crate) mod tests;
