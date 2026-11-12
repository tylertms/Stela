// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Canonical key ownership bound to the serialized WFDMA command owner.
//! This does not allocate stations or admit a physical controller. A successful
//! command programs a key; old users and radio/data attachment stay with
//! their native owner.
use super::{Mailbox, QueueIo, UnifiedReply};
use crate::wifi::{
    mediatek::{
        download::TransportFailure,
        mcu::{
            key::{Context, Request},
            runtime::Command,
        },
    },
    security::{
        self, Cipher, InstallOutcome, KeyIdentity, KeyScope, KeySlot, ReplaySeed, TransmitSeed,
    },
};
use alloc::boxed::Box;

pub mod bip;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Bip(bip::Error),
    WrongMailbox,
    PoisonedMailbox,
    InvalidContext,
    NoKey,
    NeedsDrain,
    WrongRemoval,
    Security(security::Error),
    Request(crate::wifi::mediatek::mcu::key::Error),
    Transport(TransportFailure),
    Firmware(u32),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek native key: {self:?}")
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    Empty,
    Installed,
    Quarantined,
    Removed,
}
struct Owned<C> {
    gtk: Option<bip::GtkCache>,
    igtk: Option<bip::Installed>,
    other_igtk: Option<bip::Installed>,
    igtk_history: [Option<[u8; 32]>; bip::HISTORY_CAPACITY],
    pending_igtk: Option<bip::Installed>,
    igtk_generation: u64,
    slot: KeySlot,
    context: Context,
    scope: KeyScope,
    resource: C,
    owner: u64,
    phase: Phase,
    candidate: Option<KeyIdentity>,
    reply: Option<UnifiedReply>,
}
/// Keeps the WCID/association resource and canonical material/nonce
/// guard on uncertain or still-live Drop. No vendor-local key generation or
/// digest substitutes for canonical KeySlot.
pub struct HardwareKey<C: 'static> {
    owned: Option<Box<Owned<C>>>,
}
/// Successful remove-command receipt. Not a hardware-user drain proof.
/// It is private, single-use and bound to this exact canonical key/command owner.
pub struct Removal {
    key: KeyIdentity,
    owner: u64,
}

impl<C: 'static> HardwareKey<C> {
    /// # Safety
    /// Caller exclusively owns this WCID and acknowledged BSS/OMAC/peer
    /// context on this mailbox's requester/firmware epoch. The physical
    /// key slot is empty, with all previous hardware RX/TX, notifications, IRQ
    /// and deferred users drained. `resource` must keep the requester/
    /// function/controller claim, station, association and WCID allocation for
    /// every future hardware/key user, including a quarantined owner after the
    /// mailbox itself is dropped or its DMA queues are released.
    /// Caller must preserve this owner across retries and prevent prior temporal
    /// material from being installed with reset PN after creating any new owner.
    /// No other owner or raw command may mutate this slot; all removal and key
    /// publication must serialize through this object and the controller.
    /// A mailbox identity is a native owner binding, not an on-wire generation.
    pub unsafe fn claim<W: QueueIo, F: QueueIo, R: QueueIo>(
        mailbox: &Mailbox<W, F, R>,
        wcid: u16,
        context: Context,
        scope: KeyScope,
        resource: C,
    ) -> Result<Self, Error> {
        if wcid >= 20
            || context.omac_index > 63
            || scope.peer().is_multicast()
            || scope.peer().0 == [0; 6]
        {
            return Err(Error::InvalidContext);
        }
        if mailbox.is_poisoned() {
            return Err(Error::PoisonedMailbox);
        }
        // SAFETY: inherited exclusive empty/drained slot and nonce-epoch
        // contract; resource and this object keep that physical owner.
        let slot = unsafe { KeySlot::new(wcid) }.map_err(Error::Security)?;
        Ok(Self {
            owned: Some(Box::new(Owned {
                gtk: None,
                igtk: None,
                other_igtk: None,
                igtk_history: [None; bip::HISTORY_CAPACITY],
                pending_igtk: None,
                igtk_generation: 0,
                slot,
                context,
                scope,
                resource,
                owner: mailbox.owner,
                phase: Phase::Empty,
                candidate: None,
                reply: None,
            })),
        })
    }
    fn owned(&self) -> &Owned<C> {
        self.owned.as_deref().expect("owned native key")
    }
    fn owned_mut(&mut self) -> &mut Owned<C> {
        self.owned.as_deref_mut().expect("owned native key")
    }
    /// Read-only canonical state for real station/key-generation validation.
    pub fn slot(&self) -> &KeySlot {
        &self.owned().slot
    }
    pub(in crate::wifi::mediatek) fn slot_mut(&mut self) -> &mut KeySlot {
        &mut self.owned_mut().slot
    }
    /// # Safety
    /// The old key and peer generation are removed and all RX/TX,
    /// firmware, IRQ/deferred users drained. The same physical slot remains
    /// exclusively owned; the new scope/context belongs to its real acknowledged
    /// peer. Preserve this owner across rebinding.
    ///
    /// The new association starts its own IGTK history: its handshake gives
    /// each IGTK's IPN, so an AP's unchanged IGTK installs again with the
    /// AP's replay floor, as mac80211 installs it on every association.
    pub(in crate::wifi::mediatek) unsafe fn rebind(
        &mut self,
        context: Context,
        scope: KeyScope,
    ) -> Result<(), Error> {
        if self.owned().phase != Phase::Empty
            || context.omac_index > 63
            || scope.peer().is_multicast()
            || scope.peer().0 == [0; 6]
        {
            return Err(Error::InvalidContext);
        }
        let s = self.owned_mut();
        s.context = context;
        s.scope = scope;
        s.igtk_history = [None; bip::HISTORY_CAPACITY];
        Ok(())
    }
    /// Apply canonical replay checks to a normalized vendor RX frame.
    /// The controller must bind its real association/key generation and decoded
    /// integrity/PN observations before this call; this creates no RX evidence.
    pub fn authorize<'a>(
        &mut self,
        mpdu: &'a crate::wifi::frame::Mpdu,
    ) -> Result<security::AuthenticatedData<'a>, security::Error> {
        self.owned_mut().slot.authorize(mpdu)
    }
    /// Apply the separate PTK management replay boundary to normalized
    /// RX. Caller keeps DMA, local-interface and current-peer provenance;
    /// the resulting proof does not authorize a delayed station transition.
    pub fn authorize_unicast_management<'a>(
        &mut self,
        mpdu: &'a crate::wifi::frame::Mpdu,
        local: crate::Mac,
    ) -> Result<security::unicast_management::Authenticated<'a>, security::Error> {
        self.owned_mut()
            .slot
            .authorize_unicast_management(mpdu, local)
    }
    pub fn resource(&self) -> &C {
        &self.owned().resource
    }
    pub fn last_reply(&self) -> Option<&UnifiedReply> {
        self.owned().reply.as_ref()
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
    /// Program only the canonical scope; use this exact material borrow
    /// for both canonical issuance and the source-defined wire command.
    /// An identical already-installed retry returns its existing identity
    /// without issuing a command, changing replay floors or resetting firmware PN.
    pub fn install<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
        index: u8,
        cipher: Cipher,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        self.check(mailbox)?;
        let state = self.owned_mut();
        if state.phase == Phase::Removed {
            return Err(Error::NeedsDrain);
        }
        let transmit = match state.scope {
            KeyScope::Pairwise(_) => TransmitSeed::FIRMWARE_FRESH,
            KeyScope::Group(_) => TransmitSeed::RECEIVE_ONLY,
        };
        let issuance =
            state
                .slot
                .begin_install(index, cipher, material, receive, state.scope, transmit);
        let ticket = match issuance {
            Ok(ticket) => ticket,
            Err(security::Error::AlreadyInstalled(key)) => return Ok(key),
            Err(e) => return Err(Error::Security(e)),
        };
        let request = match Request::install(mailbox.identity.chip, state.context, ticket, material)
        {
            Ok(request) => request,
            Err(e) => {
                // No command or material was published. This is proof
                // that this attempt did not install/use it, not a removal ACK.
                state
                    .slot
                    .complete_install(ticket, InstallOutcome::RejectedAndEmpty)
                    .map_err(Error::Security)?;
                return Err(Error::Request(e));
            }
        };
        state.candidate = Some(ticket.identity());
        state.reply = None;
        let outcome = mailbox.unified(Command::StationRecord, request.bytes());
        let accepted = outcome
            .as_ref()
            .is_ok_and(|r| r.status == 0 && r.dma_released);
        state.phase = if accepted {
            Phase::Installed
        } else {
            Phase::Quarantined
        };
        state
            .slot
            .complete_install(
                ticket,
                if accepted {
                    InstallOutcome::Accepted
                } else {
                    InstallOutcome::Uncertain
                },
            )
            .map_err(Error::Security)?;
        match outcome {
            Ok(reply) => {
                let status = reply.status;
                state.reply = Some(reply);
                if accepted {
                    if mailbox.identity.chip == crate::wifi::mediatek::identity::Chip::Mt7925 {
                        bip::remember(state, ticket.identity(), material);
                    }
                    Ok(ticket.identity())
                } else {
                    Err(Error::Firmware(status))
                }
            }
            Err(e) => {
                state.reply = mailbox.interrupted_unified_reply().cloned();
                Err(Error::Transport(e))
            }
        }
    }
    /// Invalidate before source removal. A zero result only permits the caller
    /// to perform and prove old-user drain; it does not retire this slot.
    pub fn remove<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
    ) -> Result<Removal, Error> {
        self.check(mailbox)?;
        let state = self.owned_mut();
        if state.phase == Phase::Removed {
            return Err(Error::NeedsDrain);
        }
        let key = state.candidate.ok_or(Error::NoKey)?;
        state.slot.invalidate();
        state.phase = Phase::Quarantined;
        state.reply = None;
        let request =
            Request::remove(mailbox.identity.chip, state.context, key).map_err(Error::Request)?;
        match mailbox.unified(Command::StationRecord, request.bytes()) {
            Ok(reply) => {
                let status = reply.status;
                let accepted = status == 0 && reply.dma_released;
                state.reply = Some(reply);
                if !accepted {
                    return Err(Error::Firmware(status));
                }
                state.phase = Phase::Removed;
                Ok(Removal {
                    key,
                    owner: state.owner,
                })
            }
            Err(e) => {
                state.reply = mailbox.interrupted_unified_reply().cloned();
                Err(Error::Transport(e))
            }
        }
    }
    /// # Safety
    /// After this exact removal ACK, every previous firmware/DMA RX/TX,
    /// completion, IRQ and deferred key/station user is drained, and
    /// no delayed frame/status can be attributed to a later key. The controller
    /// serializes this step with all old users and new publication. Removal
    /// does not prove temporal material was unused or allow resetting its PN.
    pub unsafe fn retire_after_drain(&mut self, receipt: Removal) -> Result<(), Error> {
        let state = self.owned_mut();
        if state.phase != Phase::Removed
            || state.candidate != Some(receipt.key)
            || state.owner != receipt.owner
        {
            return Err(Error::WrongRemoval);
        }
        // SAFETY: exact successful receipt is checked and old-user drain
        // is inherited from caller; canonical material history remains owned.
        unsafe { state.slot.retire_after_drain() };
        state.phase = Phase::Empty;
        state.candidate = None;
        state.gtk = None;
        state.igtk = None;
        state.other_igtk = None;
        state.pending_igtk = None;
        Ok(())
    }
}
impl<C: 'static> HardwareKey<C> {
    /// Forget the key after the firmware that held it was reset, as
    /// `mt7925_mac_reset_work` drops every key with its interfaces.
    ///
    /// # Safety
    /// The whole WLAN subsystem holding this key was reset, its DMA stopped
    /// and its receive users drained; no command may still name the key.
    pub(in crate::wifi::mediatek) unsafe fn retire_after_reset(&mut self) {
        let state = self.owned_mut();
        // SAFETY: the reset above removed the key from the hardware and the
        // caller drained every user.
        unsafe { state.slot.retire_after_drain() };
        state.phase = Phase::Empty;
        state.candidate = None;
        state.reply = None;
        state.gtk = None;
        state.igtk = None;
        state.other_igtk = None;
        state.pending_igtk = None;
    }
}
impl<C: 'static> Drop for HardwareKey<C> {
    fn drop(&mut self) {
        if let Some(owned) = self.owned.take()
            && owned.phase != Phase::Empty
        {
            // Firmware can still reference this real WCID, association and
            // material/nonce state. Keep the whole owner, not just an ID.
            core::mem::forget(owned);
        }
    }
}
