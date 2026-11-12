// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Initial ordinary station DEV_INFO/BSS resource ownership. Registration is
//! separate from association/radio configuration and never publishes a Link.
use super::{Mailbox, QueueIo, UnifiedReply};
use crate::wifi::mediatek::{
    download::TransportFailure,
    mcu::{
        device::{Context, Requests},
        runtime::Command,
    },
};
use crate::wifi::stepped::yield_now;
use alloc::{boxed::Box, vec::Vec};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Step {
    ActivateDevice,
    ActivateBss,
    DeactivateBss,
    DeactivateDevice,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Disposition {
    Accepted,
    Rejected,
    /// The source-defined WLAN_FAILURE while deactivating an already
    /// inactive BSS. Preserve the raw status; this is not an ACK of DMA drain.
    AlreadyInactive,
}
pub struct Reply {
    pub step: Step,
    pub disposition: Disposition,
    pub raw: UnifiedReply,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Context(crate::wifi::mediatek::mcu::device::Error),
    NoMemory,
    WrongMailbox,
    PoisonedMailbox,
    AlreadyAttempted,
    NoRegistration,
    WrongReceipt,
    /// A rebinding to other firmware indices than the registration's.
    OtherSlots,
    Firmware {
        step: Step,
        status: u32,
    },
    Transport {
        step: Step,
        failure: TransportFailure,
    },
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek initial device owner: {self:?}")
    }
}
impl core::error::Error for Error {}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    Fresh,
    Quarantined,
    Registered,
    Removed,
    Retired,
}
struct Owned<C> {
    context: Context,
    owner: u64,
    resource: C,
    phase: Phase,
    deactivate_attempted: bool,
    replies: Vec<Reply>,
    interrupted: Option<UnifiedReply>,
}
/// Keeps function/controller, interface and WCID resources through
/// partial configuration and uncertain hardware users, including after mailbox
/// teardown. No generic allocation/PCI admission framework is introduced.
pub struct Interface<C: 'static> {
    owned: Option<Box<Owned<C>>>,
}
/// Exact two-command deactivation receipt; old-user retirement remains
/// a separate unsafe operation. This contains no invented on-wire generation.
pub struct Deactivated {
    owner: u64,
    context: Context,
}

impl<C: 'static> Interface<C> {
    /// # Safety
    /// Caller exclusively owns this station interface's, or station MLD
    /// link's, firmware context/indices on this mailbox requester. Context fields follow
    /// the per-chip source caller ordering and firmware resource selector;
    /// RF band is not a substitute. No previous registration/key/RX/TX/IRQ/work
    /// users remain. `resource` keeps the real function/controller claim,
    /// BSS/OMAC/WMM/WCID/interface allocations and associated lifetime through
    /// quarantine, even if mailbox DMA queues are released or it is dropped.
    /// The caller serializes all raw context mutations and all future consumers
    /// with this owner. Successful initial registration is not association or
    /// full radio/BSS/WTBL/rate configuration.
    pub unsafe fn claim<W: QueueIo, F: QueueIo, R: QueueIo>(
        mailbox: &Mailbox<W, F, R>,
        context: Context,
        resource: C,
    ) -> Result<Self, Error> {
        Requests::station(context, true).map_err(Error::Context)?;
        if mailbox.is_poisoned() {
            return Err(Error::PoisonedMailbox);
        }
        let mut replies = Vec::new();
        // At most two activation and two deactivation replies; no allocation
        // should be needed to keep real response evidence after publication.
        replies.try_reserve(4).map_err(|_| Error::NoMemory)?;
        Ok(Self {
            owned: Some(Box::new(Owned {
                context,
                owner: mailbox.owner,
                resource,
                phase: Phase::Fresh,
                deactivate_attempted: false,
                replies,
                interrupted: None,
            })),
        })
    }
    fn owned(&self) -> &Owned<C> {
        self.owned.as_deref().expect("owned initial interface")
    }
    fn owned_mut(&mut self) -> &mut Owned<C> {
        self.owned.as_deref_mut().expect("owned initial interface")
    }
    pub(in crate::wifi::mediatek) fn belongs_to<W: QueueIo, F: QueueIo, R: QueueIo>(
        &self,
        mailbox: &Mailbox<W, F, R>,
    ) -> bool {
        self.owned().owner == mailbox.owner
    }
    pub fn resource(&self) -> &C {
        &self.owned().resource
    }
    /// Only the fully acknowledged initial registration. It does not satisfy
    /// shared Station firmware configuration, security or associated carrier.
    pub fn registered_context(&self) -> Option<Context> {
        (self.owned().phase == Phase::Registered).then_some(self.owned().context)
    }
    pub fn replies(&self) -> &[Reply] {
        &self.owned().replies
    }
    pub fn interrupted_reply(&self) -> Option<&UnifiedReply> {
        self.owned().interrupted.as_ref()
    }
    pub fn is_quarantined(&self) -> bool {
        self.owned().phase == Phase::Quarantined
    }
    pub fn needs_drain(&self) -> bool {
        self.owned().phase == Phase::Removed
    }
    fn check<W: QueueIo, F: QueueIo, R: QueueIo>(
        &self,
        mailbox: &Mailbox<W, F, R>,
    ) -> Result<(), Error> {
        if self.owned().owner != mailbox.owner {
            return Err(Error::WrongMailbox);
        }
        if mailbox.is_poisoned() {
            return Err(Error::PoisonedMailbox);
        }
        Ok(())
    }
    fn send<W: QueueIo, F: QueueIo, R: QueueIo>(
        state: &mut Owned<C>,
        mailbox: &mut Mailbox<W, F, R>,
        step: Step,
        command: Command,
        bytes: &[u8],
    ) -> Result<(), Error> {
        match mailbox.unified(command, bytes) {
            Ok(raw) => {
                let disposition = if raw.status == 0 {
                    Disposition::Accepted
                } else if step == Step::DeactivateBss && raw.status == 0xc000_0001 {
                    Disposition::AlreadyInactive
                } else {
                    let status = raw.status;
                    // Keep rejection evidence as well; it cannot establish
                    // a registered or retired context.
                    state.replies.push(Reply {
                        step,
                        disposition: Disposition::Rejected,
                        raw,
                    });
                    return Err(Error::Firmware { step, status });
                };
                // The mailbox returns only after both the exact MCU
                // response and TX DMA completion, never enqueue alone.
                state.replies.push(Reply {
                    step,
                    disposition,
                    raw,
                });
                Ok(())
            }
            Err(failure) => {
                state.interrupted = mailbox.interrupted_unified_reply().cloned();
                Err(Error::Transport { step, failure })
            }
        }
    }
    /// The two registration commands, a step each.
    pub async fn activate<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<(), Error> {
        self.check(mailbox)?;
        let state = self.owned_mut();
        if state.phase != Phase::Fresh {
            return Err(Error::AlreadyAttempted);
        }
        let request = Requests::station(state.context, true).map_err(Error::Context)?;
        // Publication may partially install real resources even without ACK.
        state.phase = Phase::Quarantined;
        let (command, body) = request.first();
        let sent = Self::send(state, mailbox, Step::ActivateDevice, command, body);
        yield_now().await;
        sent?;
        let (command, body) = request.second();
        let sent = Self::send(state, mailbox, Step::ActivateBss, command, body);
        yield_now().await;
        sent?;
        state.phase = Phase::Registered;
        Ok(())
    }
    /// Reverse source order, at most once. An acknowledged already-inactive
    /// BSS is preserved as that source disposition, never status zero.
    pub fn deactivate<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<Deactivated, Error> {
        self.check(mailbox)?;
        let state = self.owned_mut();
        if state.deactivate_attempted {
            return Err(Error::AlreadyAttempted);
        }
        if !matches!(state.phase, Phase::Registered | Phase::Quarantined) {
            return Err(Error::NoRegistration);
        }
        let request = Requests::station(state.context, false).map_err(Error::Context)?;
        state.deactivate_attempted = true;
        state.phase = Phase::Quarantined;
        let (command, body) = request.first();
        Self::send(state, mailbox, Step::DeactivateBss, command, body)?;
        let (command, body) = request.second();
        Self::send(state, mailbox, Step::DeactivateDevice, command, body)?;
        state.phase = Phase::Removed;
        Ok(Deactivated {
            owner: state.owner,
            context: state.context,
        })
    }
    /// `mt7925_change_vif_links` turning this station into a station MLD's
    /// link, or back: the BSS and device deactivated, then the same firmware
    /// indices activated under `context`'s address and link index. The
    /// owner's own peer, key and RX users must already be gone; the slots
    /// stay this owner's throughout.
    pub fn rebind<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
        context: Context,
    ) -> Result<(), Error> {
        self.check(mailbox)?;
        let state = self.owned_mut();
        if state.phase != Phase::Registered || state.deactivate_attempted {
            return Err(Error::NoRegistration);
        }
        let old = state.context;
        if (context.bss_index, context.omac_index, context.band_index)
            != (old.bss_index, old.omac_index, old.band_index)
            || (context.wmm_index, context.broadcast_wcid) != (old.wmm_index, old.broadcast_wcid)
        {
            return Err(Error::OtherSlots);
        }
        let removal = Requests::station(old, false).map_err(Error::Context)?;
        let addition = Requests::station(context, true).map_err(Error::Context)?;
        // The four replies of this rebinding replace the last registration's.
        state.replies.clear();
        state.phase = Phase::Quarantined;
        for (step, (command, body)) in [
            (Step::DeactivateBss, removal.first()),
            (Step::DeactivateDevice, removal.second()),
        ] {
            Self::send(state, mailbox, step, command, body)?;
        }
        state.context = context;
        for (step, (command, body)) in [
            (Step::ActivateDevice, addition.first()),
            (Step::ActivateBss, addition.second()),
        ] {
            Self::send(state, mailbox, step, command, body)?;
        }
        state.phase = Phase::Registered;
        Ok(())
    }

    /// # Safety
    /// After this exact deactivation, every interface-owned firmware key/WCID
    /// has acknowledged removal/reset as required by its owner; every
    /// old firmware/key, RX/TX DMA, completion, IRQ and deferred user is
    /// drained. No old
    /// event can touch later reused resources. The real controller serializes
    /// this operation with all old users/new publication. The source inactive
    /// BSS disposition alone is not a hardware/DMA/CPU fence.
    pub unsafe fn retire_after_drain(&mut self, receipt: Deactivated) -> Result<(), Error> {
        let state = self.owned_mut();
        if state.phase != Phase::Removed
            || state.owner != receipt.owner
            || state.context != receipt.context
        {
            return Err(Error::WrongReceipt);
        }
        state.phase = Phase::Retired;
        Ok(())
    }
}
impl<C: 'static> Interface<C> {
    /// Forget the registration after the firmware that held it was reset.
    ///
    /// # Safety
    /// The whole WLAN subsystem holding this interface was reset; no command
    /// may still name it.
    pub(in crate::wifi::mediatek) unsafe fn retire_after_reset(&mut self) {
        self.owned_mut().phase = Phase::Retired;
    }
}
impl<C: 'static> Drop for Interface<C> {
    fn drop(&mut self) {
        if let Some(owned) = self.owned.take()
            && !matches!(owned.phase, Phase::Fresh | Phase::Retired)
        {
            // Keep the real physical controller/allocation owner, not an ID.
            core::mem::forget(owned);
        }
    }
}
