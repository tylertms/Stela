// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Initial source GTK/IGTK batch ownership and bound group BIP authentication.
//! This grants no PMF negotiation, MMIE generation or current-association RX
//! delivery permission; station/key-generation checks remain mandatory.
use super::*;
use crate::wifi::{
    mediatek::{identity::Chip, mcu::key::bip as wire},
    security::{
        PacketNumber,
        management::{self, ManagementCipher},
    },
};
use stela_crypto::{Sha256, digest::Digest};
use zeroize::Zeroizing;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    DataKeyRequired,
    GenerationExhausted,
    AlreadyInstalled,
    RetiredMaterial,
    HistoryExhausted,
    Authentication(management::Error),
}

/// One acknowledged management-key batch, bound to the installed GTK,
/// native mailbox owner and monotonically increasing management generation.
/// This is never a data KeyIdentity or a MIC/association/port authorization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Identity {
    gtk: KeyIdentity,
    owner: u64,
    generation: u64,
    index: u8,
}
impl Identity {
    pub const fn gtk(self) -> KeyIdentity {
        self.gtk
    }
    pub const fn index(self) -> u8 {
        self.index
    }
    pub const fn generation(self) -> u64 {
        self.generation
    }
}
pub(super) const HISTORY_CAPACITY: usize = 64;

pub(super) struct GtkCache {
    identity: KeyIdentity,
    material: Zeroizing<[u8; 16]>,
}
pub(super) struct Installed {
    identity: Identity,
    digest: [u8; 32],
    key: management::BipKey,
}

pub(super) fn remember<C>(state: &mut Owned<C>, key: KeyIdentity, material: &[u8]) {
    if key.cipher() == Cipher::Ccmp128 && matches!(key.scope(), KeyScope::Group(_)) {
        let mut bytes = Zeroizing::new([0; 16]);
        bytes.copy_from_slice(material);
        state.gtk = Some(GtkCache {
            identity: key,
            material: bytes,
        });
    }
}
impl<C: 'static> HardwareKey<C> {
    /// Successful initial IGTK batch, valid only while its original GTK
    /// and this native owner remain installed. Copies grant no RX permission.
    pub fn installed_igtk(&self) -> Option<Identity> {
        let s = self.owned();
        let bip = s.igtk.as_ref()?;
        (s.phase == Phase::Installed
            && bip.identity.owner == s.owner
            && s.slot.installed() == Some(bip.identity.gtk)
            && s.pending_igtk.is_none())
        .then_some(bip.identity)
    }
    pub fn igtk_receive_floor(&self) -> Option<PacketNumber> {
        self.installed_igtk()?;
        Some(self.owned().igtk.as_ref()?.key.receive_floor())
    }
    pub fn igtk_statistics(&self) -> Option<management::Statistics> {
        self.installed_igtk()?;
        Some(self.owned().igtk.as_ref()?.key.statistics())
    }
    /// Authenticate a complete group robust-management frame under this
    /// acknowledged IGTK and original still-installed GTK. Software CMAC and IPN
    /// checks always run, independently of copied hardware integrity flags.
    /// The returned frame proof grants no current association/port permission;
    /// its eventual consuming station must recheck the exact association
    /// and still-current GTK/IGTK before acting on a deferred frame.
    pub fn authenticate_bip<'a>(
        &mut self,
        frame: &'a crate::wifi::frame::Mpdu,
    ) -> Result<management::Authenticated<'a>, super::Error> {
        self.installed_igtk()
            .ok_or(super::Error::Bip(Error::DataKeyRequired))?;
        let bytes = frame.bytes();
        // The full authenticator validates MMIE length/type, peer, MIC and IPN.
        // Selection reads only the complete trailing source key ID; it does not
        // guess a key from the latest firmware update or copied RX flags. Both
        // windows share the negotiated suite and so its MMIE length.
        let mmie = self
            .owned()
            .igtk
            .as_ref()
            .unwrap()
            .key
            .cipher()
            .mmie_length();
        let index = bytes
            .len()
            .checked_sub(mmie)
            .and_then(|start| bytes.get(start + 2..start + 4))
            .and_then(|id| <[u8; 2]>::try_from(id).ok())
            .map(u16::from_le_bytes);
        let s = self.owned_mut();
        let key = if s
            .igtk
            .as_ref()
            .is_some_and(|key| Some(u16::from(key.identity.index)) == index)
        {
            s.igtk.as_mut().unwrap()
        } else if s
            .other_igtk
            .as_ref()
            .is_some_and(|key| Some(u16::from(key.identity.index)) == index)
        {
            s.other_igtk.as_mut().unwrap()
        } else {
            // Preserve the existing WrongKey/BadMmie distinction from the full
            // source parser, including hostile/truncated trailing elements.
            s.igtk.as_mut().unwrap()
        };
        key.key
            .authenticate(frame)
            .map_err(|error| super::Error::Bip(Error::Authentication(error)))
    }

    /// Program the source CCMP128 GTK || BIP-CMAC128 IGTK batch. The firmware
    /// batch repeats the same GTK, while its canonical data identity and replay
    /// floors remain unchanged. Exact identical retries publish no command and
    /// do not replace the original authenticated management floor. Every other
    /// suite, or a GTK that is not CCMP128, is kept in software alone, as
    /// `mt7925_set_key` leaves it to mac80211.
    ///
    /// # Safety
    /// The caller owns this GTK/WCID/BSS namespace and has independently
    /// drained all old GTK hardware RX/TX, DMA/IRQ/work and delayed key users.
    /// No data or management publication may use the namespace until this call
    /// returns successfully; uncertain outcomes require the removal/reset
    /// and drain path. `material` and `receive` come from the authenticated
    /// OS key exchange. This initial-batch API does not implement live IGTK rekey.
    pub unsafe fn install_igtk<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
        cipher: ManagementCipher,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Identity, super::Error> {
        self.check(mailbox)?;
        if mailbox.identity.chip != Chip::Mt7925 {
            return Err(super::Error::Request(
                crate::wifi::mediatek::mcu::key::Error::UnsupportedBip,
            ));
        }
        let s = self.owned_mut();
        if s.phase != Phase::Installed {
            return Err(super::Error::Bip(Error::DataKeyRequired));
        }
        let gtk = s.slot.installed().ok_or(super::Error::NoKey)?;
        // Complete scope/index/length/codec validation before changing ownership.
        let request = hardware_request(mailbox.identity.chip, s, gtk, cipher, index, material)?;
        let digest: [u8; 32] = Sha256::digest(material).into();
        if let Some(old) = &s.igtk {
            let same = old
                .digest
                .iter()
                .zip(digest)
                .fold(0u8, |difference, (a, b)| difference | (*a ^ b))
                == 0;
            if old.identity.gtk == gtk
                && old.identity.index == index
                && old.key.cipher() == cipher
                && same
            {
                return Ok(old.identity);
            }
            return Err(super::Error::Bip(Error::AlreadyInstalled));
        }
        let history = reserve_material(s, digest)?;
        let generation = s
            .igtk_generation
            .checked_add(1)
            .ok_or(super::Error::Bip(Error::GenerationExhausted))?;
        let identity = Identity {
            gtk,
            owner: s.owner,
            generation,
            index,
        };
        let key = management::BipKey::new(gtk, cipher, index, material, receive)
            .map_err(|error| super::Error::Bip(Error::Authentication(error)))?;
        s.igtk_history[history] = Some(digest);
        s.igtk_generation = generation;
        let installed = Installed {
            identity,
            digest,
            key,
        };
        let Some(request) = request else {
            s.igtk = Some(installed);
            return Ok(identity);
        };
        s.pending_igtk = Some(installed);
        s.phase = Phase::Quarantined;
        s.reply = None;
        let outcome = mailbox.unified(Command::StationRecord, request.bytes());
        let accepted = outcome
            .as_ref()
            .is_ok_and(|r| r.status == 0 && r.dma_released);
        if accepted {
            s.igtk = s.pending_igtk.take();
            s.phase = Phase::Installed;
        } else {
            // The device may already have installed the combined key record.
            // Neither data nor management may inherit an uncertain batch.
            s.slot.invalidate();
        }
        match outcome {
            Ok(reply) => {
                let status = reply.status;
                s.reply = Some(reply);
                if accepted {
                    Ok(identity)
                } else {
                    Err(super::Error::Firmware(status))
                }
            }
            Err(error) => {
                s.reply = mailbox.interrupted_unified_reply().cloned();
                Err(super::Error::Transport(error))
            }
        }
    }
}

/// The firmware batch `mt7925_mcu_sta_key_tlv` sends for BIP-CMAC-128 beside
/// the cached CCMP128 GTK; `mt7925_set_key` gives every other suite to
/// mac80211's software, so it has none.
fn hardware_request<C>(
    chip: Chip,
    s: &Owned<C>,
    gtk: KeyIdentity,
    cipher: ManagementCipher,
    index: u8,
    material: &[u8],
) -> Result<Option<wire::Request>, super::Error> {
    use crate::wifi::mediatek::mcu::key::Error as Request;
    if !matches!(gtk.scope(), KeyScope::Group(_)) {
        return Err(super::Error::Bip(Error::DataKeyRequired));
    }
    if !matches!(index, 4 | 5) {
        return Err(super::Error::Request(Request::InvalidBipIndex));
    }
    if material.len() != cipher.key_length() {
        return Err(super::Error::Request(Request::InvalidKeyLength));
    }
    let cache = s.gtk.as_ref().filter(|cache| cache.identity == gtk);
    let (ManagementCipher::Cmac128, Some(cache)) = (cipher, cache) else {
        return Ok(None);
    };
    wire::Request::install(
        chip,
        s.context,
        gtk,
        cache.material.as_ref(),
        index,
        material,
    )
    .map(Some)
    .map_err(super::Error::Request)
}

/// Management-material history survives removal within an association: a
/// new key index must not revive retired temporal material with a reset
/// replay namespace. A rebind to a new association clears it.
fn reserve_material<C>(s: &Owned<C>, digest: [u8; 32]) -> Result<usize, super::Error> {
    let mut empty = None;
    let mut reused = false;
    for (index, remembered) in s.igtk_history.iter().enumerate() {
        if let Some(old) = remembered {
            reused |= old.iter().zip(digest).fold(0u8, |d, (a, b)| d | (*a ^ b)) == 0;
        } else if empty.is_none() {
            empty = Some(index);
        }
    }
    if reused {
        return Err(super::Error::Bip(Error::RetiredMaterial));
    }
    empty.ok_or(super::Error::Bip(Error::HistoryExhausted))
}

impl<C: 'static> HardwareKey<C> {
    /// Both kept management receive windows, indexed by MMIE 4/5.
    /// Firmware still has the source single cached-GTK||IGTK update command;
    /// these software windows do not prove physical firmware/air overlap.
    pub fn installed_igtks(&self) -> [Option<Identity>; 2] {
        if self.installed_igtk().is_none() {
            return [None; 2];
        }
        let mut keys = [None; 2];
        for key in [&self.owned().igtk, &self.owned().other_igtk]
            .into_iter()
            .flatten()
        {
            keys[usize::from(key.identity.index - 4)] = Some(key.identity);
        }
        keys
    }

    /// Replay floor for that exact still receive key, including the
    /// previous index during overlap. A copied identity cannot create a window.
    pub fn retained_igtk_receive_floor(&self, identity: Identity) -> Option<PacketNumber> {
        self.installed_igtk()?;
        [&self.owned().igtk, &self.owned().other_igtk]
            .into_iter()
            .flatten()
            .find(|key| key.identity == identity)
            .map(|key| key.key.receive_floor())
    }

    /// Source-exact acknowledged live management-key update. Alternating 4/5
    /// keeps the other software MIC/IPN window; replacing the same index retires
    /// only that original window. Installed retries keep its advanced
    /// floor and issue no command, including retries of the other receive index.
    /// All historical material is bounded to 64 unique keys per owner;
    /// exhaustion refuses before publication rather than evicting replay history.
    ///
    /// # Safety
    /// The real authenticated OS exchange supplies this exact peer's new IGTK/IPN
    /// and authorizes same-index replacement only after the peer has ceased using
    /// that index's old material. Independently stop/drain all old GTK hardware
    /// RX/TX, DMA/IRQ/work and external deferred users before repeating the
    /// combined GTK batch; serialized empty CPU queues alone do not prove this.
    /// Preserve requester/association/GTK ownership and material history.
    /// Firmware source exposes one cached-GTK||IGTK batch without an IGTK index;
    /// two receive keys authenticate only intact delivered native frames. Actual
    /// firmware delivery/overlap is unchecked: never clear ICV/monitor errors,
    /// infer protection negotiation or release an old peer key from ACK.
    pub unsafe fn rotate_igtk<W: QueueIo, F: QueueIo, R: QueueIo>(
        &mut self,
        mailbox: &mut Mailbox<W, F, R>,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Identity, super::Error> {
        self.check(mailbox)?;
        self.installed_igtk()
            .ok_or(super::Error::Bip(Error::DataKeyRequired))?;
        let s = self.owned_mut();
        let gtk = s.slot.installed().ok_or(super::Error::NoKey)?;
        let cipher = s.igtk.as_ref().unwrap().key.cipher();
        let request = hardware_request(mailbox.identity.chip, s, gtk, cipher, index, material)?;
        let digest: [u8; 32] = Sha256::digest(material).into();
        for old in [&s.igtk, &s.other_igtk].into_iter().flatten() {
            let same = old
                .digest
                .iter()
                .zip(digest)
                .fold(0u8, |d, (a, b)| d | (*a ^ b))
                == 0;
            if old.identity.index == index && same {
                return Ok(old.identity);
            }
        }
        let history = reserve_material(s, digest)?;
        let generation = s
            .igtk_generation
            .checked_add(1)
            .ok_or(super::Error::Bip(Error::GenerationExhausted))?;
        let identity = Identity {
            gtk,
            owner: s.owner,
            generation,
            index,
        };
        let key = management::BipKey::new(gtk, cipher, index, material, receive)
            .map_err(|e| super::Error::Bip(Error::Authentication(e)))?;
        // Reserve history and every old/new key before publication. Any negative,
        // ambiguous or late reply keeps the entire quarantined native owner.
        s.igtk_history[history] = Some(digest);
        s.igtk_generation = generation;
        s.pending_igtk = Some(Installed {
            identity,
            digest,
            key,
        });
        let Some(request) = request else {
            rotated(s, index);
            return Ok(identity);
        };
        s.phase = Phase::Quarantined;
        s.reply = None;
        let outcome = mailbox.unified(Command::StationRecord, request.bytes());
        let accepted = outcome
            .as_ref()
            .is_ok_and(|r| r.status == 0 && r.dma_released);
        if accepted {
            rotated(s, index);
            s.phase = Phase::Installed;
        } else {
            s.slot.invalidate();
        }
        match outcome {
            Ok(reply) => {
                let status = reply.status;
                s.reply = Some(reply);
                if accepted {
                    Ok(identity)
                } else {
                    Err(super::Error::Firmware(status))
                }
            }
            Err(error) => {
                s.reply = mailbox.interrupted_unified_reply().cloned();
                Err(super::Error::Transport(error))
            }
        }
    }
}

/// The pending key replaces only the requested index. The previous latest key
/// is the untouched other window, preserving its MIC/IPN/statistics.
fn rotated<C>(s: &mut Owned<C>, index: u8) {
    let previous = s.igtk.take().unwrap();
    if previous.identity.index != index {
        s.other_igtk = Some(previous);
    }
    s.igtk = s.pending_igtk.take();
}

#[cfg(test)]
pub(crate) mod tests;
