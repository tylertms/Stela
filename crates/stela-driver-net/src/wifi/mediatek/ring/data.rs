// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
// Copyright (C) 2020-2023 MediaTek Inc.
//! Ordinary PCIe data TX: one 64-byte TXWI fetched by WFDMA, with independent
//! firmware-owned TXP payloads. Descriptor consumption alone cannot free them.
use super::{Allocation, DESC_BYTES, DONE, LAST, QueueIo, retain_queue};
use crate::bus::dma::{
    mapping::{Direction, DmaDomain},
    memory::PageSource,
};
use crate::wifi::mediatek::{identity::Chip, tx_free, txp, txwi};
use alloc::{collections::VecDeque, sync::Arc, vec::Vec};
use core::sync::atomic::{Ordering, fence};

const TOKENS: usize = 8192;
const MAX_COUNT: usize = 2048;
const MAX_SEGMENT: usize = 4095;
static NEXT_EPOCH: crate::wifi::Identities = crate::wifi::Identities::new();

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Ring(super::Error),
    Txwi(txwi::Error),
    Txp(txp::Error),
    Free(tx_free::Error),
    InvalidFrame,
    PublicationRefused,
    Full,
    WrongEpoch,
    EpochExhausted,
    UnknownToken(u16),
}
impl From<super::Error> for Error {
    fn from(e: super::Error) -> Self {
        Self::Ring(e)
    }
}
impl From<txwi::Error> for Error {
    fn from(e: txwi::Error) -> Self {
        Self::Txwi(e)
    }
}
impl From<txp::Error> for Error {
    fn from(e: txp::Error) -> Self {
        Self::Txp(e)
    }
}
impl From<tx_free::Error> for Error {
    fn from(e: tx_free::Error) -> Self {
        Self::Free(e)
    }
}
impl From<crate::bus::dma::mapping::DmaError> for Error {
    fn from(e: crate::bus::dma::mapping::DmaError) -> Self {
        Self::Ring(e.into())
    }
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek data TX: {self:?}")
    }
}
impl core::error::Error for Error {}

/// Host ownership epoch, not an extra field invented in firmware packets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Epoch(u64);
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Receipt {
    pub epoch: Epoch,
    pub serial: u64,
    pub token: u16,
    pub descriptor: u16,
    pub wcid: u16,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FreeEvidence {
    pub header: [u32; 2],
    pub wcid: Option<u16>,
    pub pair_word: Option<u32>,
    pub accounting: Option<tx_free::Accounting>,
    pub accounting_word: Option<u32>,
    pub release_word: u32,
}
#[cfg(test)]
impl Receipt {
    pub(crate) fn for_tests(serial: u64) -> Self {
        Self {
            epoch: Epoch(0),
            serial,
            token: 0,
            descriptor: 0,
            wcid: 0,
        }
    }
}
/// Mapping removal completed after both hardware receipts. No air ACK or TXS
/// status is synthesized. The caller still owns any separate PID/TXS context.
pub struct Released<C> {
    pub receipt: Receipt,
    pub evidence: FreeEvidence,
    pub context: C,
}
struct Packet<C> {
    txwi: Allocation,
    payload: Vec<Allocation>,
    context: Option<C>,
    receipt: Receipt,
    consumed: bool,
    free: Option<FreeEvidence>,
}

/// One data queue and the requester's sole ordinary 8192-token table.
/// `C` keeps the caller's real association/key-generation context. Completion
/// returns it only after DMA removal; uncertain Drop keeps it with the claim.
pub struct DataRing<Q: QueueIo, C: 'static> {
    io: Option<Q>,
    descriptors: Allocation,
    slots: Vec<Option<Receipt>>,
    tokens: Vec<Option<Packet<C>>>,
    ready: VecDeque<u16>,
    domain: Arc<dyn DmaDomain>,
    epoch: Epoch,
    chip: Chip,
    serial: u64,
    next_token: usize,
    head: usize,
    tail: usize,
    queued: usize,
    outstanding: usize,
    /// TX-free notifications or releases skipped as mt7925_mac_tx_free skips
    /// them: a malformed notification, or an unknown or already freed token.
    ignored_frees: u64,
    poisoned: bool,
    released: bool,
}
impl<Q: QueueIo, C: 'static> DataRing<Q, C> {
    /// How many TX-free notifications or releases were skipped.
    pub fn ignored_frees(&self) -> u64 {
        self.ignored_frees
    }
    /// # Safety
    /// All `super::TxRing::install` requester/coherent-WB/NoSnoop/ATS/idle
    /// validations apply. This owns the data queue and the complete
    /// token namespace; no other data submit/free/reset owner may exist. It is
    /// a fresh firmware/token epoch: prior TX, RX, IRQ/deferred work and late
    /// firmware completions are drained/reset, not forgotten.
    /// The caller serializes station/key retirement with submission and keeps
    /// any other users (including TXS) independently. Process/deferred context
    /// is required for canonical allocation, sync, unmap and final Drop.
    pub unsafe fn install(
        io: Q,
        source: &dyn PageSource,
        domain: Arc<dyn DmaDomain>,
        count: usize,
        chip: Chip,
    ) -> Result<Self, Error> {
        if !(4..=MAX_COUNT).contains(&count) || !count.is_power_of_two() {
            return Err(super::Error::InvalidSize.into());
        }
        let epoch = Epoch(NEXT_EPOCH.next().ok_or(Error::EpochExhausted)?);
        let descriptors = Allocation::new(
            source,
            domain.clone(),
            count * DESC_BYTES,
            Direction::Bidirectional,
        )?;
        for i in 0..count {
            descriptors.write32(i * DESC_BYTES + 4, DONE)?;
        }
        descriptors.dma.sync_for_device(0..count * DESC_BYTES)?;
        let base = descriptors.address()?;
        let mut ring = Self {
            io: Some(io),
            descriptors,
            slots: (0..count).map(|_| None).collect(),
            tokens: (0..TOKENS).map(|_| None).collect(),
            ready: VecDeque::new(),
            domain,
            epoch,
            chip,
            serial: 0,
            next_token: 0,
            head: 0,
            tail: 0,
            queued: 0,
            outstanding: 0,
            ignored_frees: 0,
            poisoned: true,
            released: false,
        };
        // Guard the first possible queue-base publication, including failed MMIO.
        ring.descriptors.published = true;
        ring.io
            .as_mut()
            .expect("retained queue")
            .configure(base, count as u16)?;
        if ring.io.as_mut().expect("retained queue").device_index()? != 0 {
            return Err(super::Error::InvalidDeviceIndex.into());
        }
        ring.poisoned = false;
        Ok(ring)
    }
    pub fn epoch(&self) -> Epoch {
        self.epoch
    }
    pub fn is_poisoned(&self) -> bool {
        self.poisoned
    }
    pub fn pending_descriptors(&self) -> usize {
        self.queued
    }
    pub fn outstanding_tokens(&self) -> usize {
        self.outstanding
    }
    /// Serialized native lifecycle view of kept packet contexts.
    /// False does not establish requester-wide or unrelated deferred quiescence.
    pub(in crate::wifi::mediatek) fn has_pending_context(
        &self,
        mut matches: impl FnMut(&C) -> bool,
    ) -> bool {
        self.tokens
            .iter()
            .flatten()
            .any(|p| p.context.as_ref().is_some_and(&mut matches))
    }
    fn live(&self) -> Result<(), Error> {
        if self.poisoned || self.released {
            Err(super::Error::Poisoned.into())
        } else {
            Ok(())
        }
    }
    /// Copies an already validated ordinary MPDU into owned canonical mappings.
    /// Config must describe those bytes and the acknowledged firmware
    /// station/key/rate tables; encoding cannot validate that shared state.
    /// `context` must keep that generation through completion/quarantine.
    /// Success means queued; an error after publication poisons and keeps all
    /// hardware-reachable mappings and context until quiescence.
    pub fn submit(
        &mut self,
        source: &dyn PageSource,
        config: txwi::Config,
        bytes: &[u8],
        context: C,
    ) -> Result<Receipt, Error> {
        self.submit_if(source, config, bytes, context, || true)
    }
    /// Recheck a time-sensitive native admission after all payload/TXWI
    /// allocation, mapping and visibility work, immediately before the first
    /// hardware-reachable descriptor address/context publication. Refusal has
    /// published nothing and does not consume a token or poison this ring.
    /// The real owner must still serialize policy/key/radio revocation through
    /// publication; this callback does not manufacture an atomic hardware gate.
    pub(in crate::wifi::mediatek) fn submit_if(
        &mut self,
        source: &dyn PageSource,
        config: txwi::Config,
        bytes: &[u8],
        context: C,
        admit: impl FnOnce() -> bool,
    ) -> Result<Receipt, Error> {
        self.live()?;
        if config.chip != self.chip
            || config.frame.bytes != bytes.len()
            || bytes.len() <= 14
            || bytes.len() > MAX_SEGMENT * 4
            || matches!(config.queue, txwi::Queue::Beacon | txwi::Queue::Discovery)
        {
            return Err(Error::InvalidFrame);
        }
        let txd = txwi::encode_mpdu(config, bytes)?;
        if self.queued + 1 >= self.slots.len() - 1 || self.outstanding == TOKENS {
            return Err(Error::Full);
        }
        let token = (0..TOKENS)
            .map(|n| (self.next_token + n) % TOKENS)
            .find(|&i| self.tokens[i].is_none())
            .ok_or(Error::Full)?;
        let serial = self.serial.checked_add(1).ok_or(Error::EpochExhausted)?;
        let receipt = Receipt {
            epoch: self.epoch,
            serial,
            token: token as u16,
            descriptor: self.head as u16,
            wcid: config.wcid,
        };
        // No address has been published yet. Failures here safely unmap fresh RAM.
        let mut payload = Vec::with_capacity(bytes.len().div_ceil(MAX_SEGMENT));
        for chunk in bytes.chunks(MAX_SEGMENT) {
            let data = Allocation::new(
                source,
                self.domain.clone(),
                chunk.len(),
                Direction::ToDevice,
            )?;
            // SAFETY: `pointer` bounds the fresh unpublished allocation to `chunk.len()`.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    chunk.as_ptr(),
                    data.pointer(0, chunk.len())?,
                    chunk.len(),
                );
            }
            data.dma.sync_for_device(0..chunk.len())?;
            payload.push(data);
        }
        let txwi = Allocation::new(
            source,
            self.domain.clone(),
            txp::TXWI_BYTES,
            Direction::ToDevice,
        )?;
        let spans = payload
            .iter()
            .zip(bytes.chunks(MAX_SEGMENT))
            .map(|(p, chunk)| txp::Span::new(&p.dma, 0, chunk.len()))
            .collect::<Result<Vec<_>, _>>()?;
        let txp = txp::Encoded::new(token as u16, &spans)?;
        // SAFETY: `pointer` bounds both 32-byte halves of the fresh unpublished TXWI.
        unsafe {
            core::ptr::copy_nonoverlapping(txd.as_ptr(), txwi.pointer(0, 32)?, 32);
            core::ptr::copy_nonoverlapping(txp.bytes().as_ptr(), txwi.pointer(32, 32)?, 32);
        }
        txwi.dma.sync_for_device(0..txp::TXWI_BYTES)?;
        drop(txp);
        drop(spans);
        let address = txwi.address()?;
        if !admit() {
            return Err(Error::PublicationRefused);
        }
        self.tokens[token] = Some(Packet {
            txwi,
            payload,
            context: Some(context),
            receipt,
            consumed: false,
            free: None,
        });
        self.next_token = (token + 1) % TOKENS;
        self.serial = serial;
        self.outstanding += 1;
        // Keep context before the first prefetchable descriptor address store.
        let packet = self.tokens[token].as_mut().expect("inserted token");
        packet.txwi.published = true;
        for data in &mut packet.payload {
            data.published = true;
        }
        self.slots[self.head] = Some(receipt);
        let result = self.publish(address);
        if result.is_err() {
            self.poisoned = true;
        }
        result?;
        Ok(receipt)
    }
    fn publish(&mut self, address: u32) -> Result<(), Error> {
        let at = self.head * DESC_BYTES;
        self.descriptors.write32(at, address)?;
        self.descriptors.write32(at + 8, 0)?;
        self.descriptors.write32(at + 12, 0)?;
        self.descriptors
            .write32(at + 4, LAST | ((txp::TXWI_BYTES as u32) << 16))?;
        self.descriptors.dma.sync_for_device(at..at + DESC_BYTES)?;
        self.head = (self.head + 1) % self.slots.len();
        self.queued += 1;
        fence(Ordering::SeqCst);
        self.io
            .as_mut()
            .expect("retained queue")
            .publish(self.head as u16)?;
        Ok(())
    }
    /// Observe one WFDMA index advancement. This permits slot reuse,
    /// never payload/TXWI removal or a successful air status by itself.
    pub fn poll_consumed(&mut self) -> Result<Option<Receipt>, Error> {
        self.live()?;
        let result = self.poll_inner();
        if result.is_err() {
            self.poisoned = true;
        }
        result
    }
    fn poll_inner(&mut self) -> Result<Option<Receipt>, Error> {
        let count = self.slots.len();
        let actual = usize::from(self.io.as_mut().expect("retained queue").device_index()?);
        if actual >= count || (actual + count - self.tail) % count > self.queued {
            return Err(super::Error::InvalidDeviceIndex.into());
        }
        if actual == self.tail {
            return Ok(None);
        }
        fence(Ordering::SeqCst);
        let at = self.tail * DESC_BYTES;
        // SAFETY: requester index passed this exact descriptor slot.
        unsafe {
            self.descriptors.dma.sync_for_cpu(at..at + DESC_BYTES)?;
        }
        let receipt = self.slots[self.tail].ok_or(super::Error::OwnershipMismatch)?;
        let packet = self.tokens[usize::from(receipt.token)]
            .as_mut()
            .ok_or(Error::UnknownToken(receipt.token))?;
        if packet.receipt != receipt || packet.consumed {
            return Err(super::Error::OwnershipMismatch.into());
        }
        packet.consumed = true;
        if packet.free.is_some() {
            self.ready.push_back(receipt.token);
        }
        self.slots[self.tail] = None;
        self.tail = (self.tail + 1) % count;
        self.queued -= 1;
        Ok(Some(receipt))
    }
    /// Apply a whole decoded firmware release batch only after every token and
    /// optional WCID has been validated. A bad batch mutates no packet state.
    ///
    /// # Safety
    /// Bytes must be a complete, non-dropped type-6 firmware packet from
    /// this requester's owned completed RX queue, in this fresh firmware
    /// epoch, delivered once by the serialized RX owner. They must never come
    /// from a network MPDU, invented frame or old queue/worker/reset epoch.
    /// Firmware must obey its source token-release contract: all references to
    /// each released TXWI/MSDU have ended and no delayed duplicate release can
    /// follow token reuse. The wire has no generation field; a host cookie or
    /// epoch cannot detect such a stale duplicate after a token is reallocated.
    pub unsafe fn apply_tx_free(&mut self, epoch: Epoch, bytes: &[u8]) -> Result<usize, Error> {
        self.live()?;
        let result = self.free_inner(epoch, bytes);
        if result.is_err() {
            self.poisoned = true;
        }
        result
    }
    fn free_inner(&mut self, epoch: Epoch, bytes: &[u8]) -> Result<usize, Error> {
        if epoch != self.epoch {
            return Err(Error::WrongEpoch);
        }
        // mt7925_mac_tx_free returns on a notification it cannot read and
        // skips a token mt76_token_release does not hold; neither stops the
        // queue. The token is the authority, whatever station it names.
        let Ok(notification) = tx_free::parse(self.chip, bytes) else {
            self.ignored_frees = self.ignored_frees.saturating_add(1);
            return Ok(0);
        };
        let mut pair_word = None;
        let mut accounting = None;
        let mut accounting_word = None;
        for event in notification.events {
            match event {
                tx_free::Event::Pair { raw, .. } => {
                    pair_word = Some(raw);
                    accounting = None;
                    accounting_word = None;
                }
                tx_free::Event::Accounting { value, raw, .. } => {
                    accounting = Some(value);
                    accounting_word = Some(raw);
                }
                tx_free::Event::Release {
                    wcid,
                    token,
                    accounting: local,
                    raw,
                } => {
                    let Some(packet) = self
                        .tokens
                        .get_mut(usize::from(token))
                        .and_then(Option::as_mut)
                        .filter(|packet| packet.free.is_none())
                    else {
                        self.ignored_frees = self.ignored_frees.saturating_add(1);
                        continue;
                    };
                    packet.free = Some(FreeEvidence {
                        header: notification.header,
                        wcid,
                        pair_word,
                        accounting: local.or(accounting),
                        accounting_word,
                        release_word: raw,
                    });
                    if packet.consumed {
                        self.ready.push_back(token);
                    }
                }
            }
        }
        Ok(usize::from(notification.declared_tokens))
    }
    /// Remove mappings only after both receipts. Every mapping and
    /// context stays owned if a sync/unmap fails; the epoch then fails closed.
    pub fn take_released(&mut self) -> Result<Option<Released<C>>, Error> {
        self.live()?;
        let result = self.release_inner();
        if result.is_err() {
            self.poisoned = true;
        }
        result
    }
    fn release_inner(&mut self) -> Result<Option<Released<C>>, Error> {
        let Some(&token) = self.ready.front() else {
            return Ok(None);
        };
        let packet = self.tokens[usize::from(token)]
            .as_mut()
            .ok_or(Error::UnknownToken(token))?;
        let evidence = packet.free.ok_or(super::Error::OwnershipMismatch)?;
        if !packet.consumed {
            return Err(super::Error::OwnershipMismatch.into());
        }
        // SAFETY: observed index consumption AND firmware TX-free ended
        // all hardware references. Serialized exclusive owner has no other users.
        unsafe {
            packet.txwi.dma.sync_for_cpu(0..txp::TXWI_BYTES)?;
            for data in &packet.payload {
                data.dma.sync_for_cpu(0..data.dma.bytes())?;
            }
            for data in &mut packet.payload {
                data.release()?;
            }
            packet.txwi.release()?;
        }
        let mut packet = self.tokens[usize::from(token)]
            .take()
            .expect("retained token");
        self.ready.pop_front();
        self.outstanding -= 1;
        Ok(Some(Released {
            receipt: packet.receipt,
            evidence,
            context: packet.context.take().expect("retained context"),
        }))
    }
    /// # Safety
    /// Full hardware/firmware and CPU-worker fence as for raw ring teardown,
    /// including firmware TXP payload accesses and all prior token completions.
    /// A DMA-engine idle bit alone is insufficient. Failed removal keeps all
    /// contexts; retry requires this proof still to hold.
    pub unsafe fn release_after_quiesce(&mut self) -> Result<Vec<C>, Error> {
        self.poisoned = true;
        for packet in self.tokens.iter_mut().flatten() {
            for data in &mut packet.payload {
                // SAFETY: covered by this function's own safety contract.
                unsafe {
                    data.release()?;
                }
            }
            // SAFETY: covered by this function's own safety contract.
            unsafe {
                packet.txwi.release()?;
            }
        }
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            self.descriptors.release()?;
        }
        self.released = true;
        let contexts = self
            .tokens
            .iter_mut()
            .filter_map(Option::take)
            .filter_map(|mut p| p.context.take())
            .collect();
        self.slots.fill(None);
        self.ready.clear();
        self.queued = 0;
        self.outstanding = 0;
        Ok(contexts)
    }
}
impl<Q: QueueIo, C: 'static> Drop for DataRing<Q, C> {
    fn drop(&mut self) {
        if self.descriptors.published && !self.released {
            retain_queue(self.io.take());
            // DMA leases quarantine their pins/domain; keep station/key
            // context too, including partial successful unmaps before failure.
            for packet in self.tokens.iter_mut().flatten() {
                if let Some(context) = packet.context.take() {
                    core::mem::forget(context);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
