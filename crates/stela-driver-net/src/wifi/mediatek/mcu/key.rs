// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Exact ordinary CCMP/GCMP station-key requests. Canonical tickets carry key
//! identity/scope/counter policy; wire construction never acknowledges them.
use super::super::identity::Chip;
use crate::wifi::security::{Cipher, InstallTicket, KeyIdentity, KeyScope, NonceOwner};
use alloc::vec::Vec;

pub mod bip;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    NoMemory,
    InvalidBipDataKey,
    InvalidBipIndex,
    InvalidWcid,
    InvalidOmac,
    InvalidKeyLength,
    UnsupportedTransmitState,
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek key request: {self:?}")
    }
}
impl core::error::Error for Error {}

/// Acknowledged firmware BSS/OMAC context. A byte-sized index alone is
/// not proof that firmware allocated it or associated it with this WCID/peer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Context {
    pub bss_index: u8,
    pub omac_index: u8,
    pub station_disabled: bool,
}
/// Sensitive command bytes have no Debug/Clone implementation.
/// These bytes keep no on-wire key generation; native ownership and
/// completion must bind the exact canonical ticket separately.
pub struct Request {
    key: KeyIdentity,
    bytes: Vec<u8>,
}
impl Request {
    /// The same material borrow must be used to create this InstallTicket and
    /// construct these bytes. This codec cannot inspect the slot's private
    /// material fingerprint; the owning installer enforces that binding.
    pub fn install(
        chip: Chip,
        context: Context,
        ticket: InstallTicket,
        material: &[u8],
    ) -> Result<Self, Error> {
        let key = ticket.identity();
        let expected = match key.scope() {
            KeyScope::Pairwise(_) => NonceOwner::Firmware,
            KeyScope::Group(_) => NonceOwner::ReceiveOnly,
        };
        if ticket.transmit_seed().owner() != expected
            || ticket.transmit_seed().initial_counter().value() != 0
        {
            return Err(Error::UnsupportedTransmitState);
        }
        if material.len() != key.cipher().key_length() {
            return Err(Error::InvalidKeyLength);
        }
        Self::build(chip, context, key, Some(material))
    }
    /// Source remove targets this WCID/peer context. It carries neither
    /// key bytes nor an invented per-index completion. ACK and old users/DMA
    /// drain remain separate prerequisites for canonical slot retirement.
    pub fn remove(chip: Chip, context: Context, key: KeyIdentity) -> Result<Self, Error> {
        Self::build(chip, context, key, None)
    }
    pub fn key(&self) -> KeyIdentity {
        self.key
    }
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
    fn build(
        chip: Chip,
        context: Context,
        key: KeyIdentity,
        material: Option<&[u8]>,
    ) -> Result<Self, Error> {
        if key.slot() >= 20 {
            return Err(Error::InvalidWcid);
        }
        if context.omac_index > 63 {
            return Err(Error::InvalidOmac);
        }
        let pairwise = matches!(key.scope(), KeyScope::Pairwise(_));
        let connac3 = chip == Chip::Mt7925;
        let length = 8 + if connac3 { 68 } else { 80 };
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(length)
            .map_err(|_| Error::NoMemory)?;
        bytes.resize(length, 0);
        bytes[0] = context.bss_index;
        bytes[1] = key.slot() as u8;
        bytes[2..4].copy_from_slice(&1u16.to_le_bytes()); // one TLV
        bytes[4] = 1; // is_tlv_append
        bytes[5] = if !pairwise && !context.station_disabled {
            0x0e
        } else {
            context.omac_index
        };
        // Admitted 20-entry WTBL keeps to_wcid_hi zero; no masked slot.
        if connac3 {
            bytes[8..10].copy_from_slice(&0x27u16.to_le_bytes()); // STA_REC_KEY_V3
            bytes[10..12].copy_from_slice(&68u16.to_le_bytes());
            bytes[12] = u8::from(material.is_some());
            // mt7925_set_link_key installs a static WEP key without a
            // station, so both of its records take the group form.
            let station_key = pairwise && !key.cipher().is_wep();
            bytes[13] = u8::from(station_key); // tx_key
            bytes[14] = u8::from(station_key); // key_type
            bytes[16..22].copy_from_slice(&key.scope().peer().0);
            bytes[22] = context.bss_index;
            bytes[26] = key.slot() as u8;
            bytes[27] = 1; // source mgmt_prot, only used in MLO mode
            if let Some(material) = material {
                bytes[23] = match key.cipher() {
                    Cipher::Wep40 => 1,
                    Cipher::Tkip => 2,
                    Cipher::Ccmp128 => 4,
                    Cipher::Ccmp256 => 10,
                    Cipher::Gcmp128 => 11,
                    Cipher::Gcmp256 => 12,
                    Cipher::Wep104 => 5,
                };
                bytes[24] = key.index();
                bytes[25] = material.len() as u8;
                put_key(&mut bytes[28..28 + material.len()], key.cipher(), material);
            }
            // Source key_rsc[16] stays zero. Authenticated per-TID receive
            // floors remain in the canonical ticket/KeySlot and cannot reset.
        } else {
            bytes[8..10].copy_from_slice(&17u16.to_le_bytes()); // STA_REC_KEY_V2
            bytes[10..12]
                .copy_from_slice(&(if material.is_some() { 44u16 } else { 8 }).to_le_bytes());
            // Connac2 copies enum set_key_cmd: SET_KEY0 / DISABLE_KEY1.
            // It is the reverse of Connac3's explicit add flag.
            bytes[12] = u8::from(material.is_none());
            if let Some(material) = material {
                bytes[13] = 1;
                bytes[16] = match key.cipher() {
                    Cipher::Wep40 => 1,
                    Cipher::Wep104 => 2,
                    Cipher::Tkip => 4,
                    Cipher::Ccmp128 => 5,
                    Cipher::Ccmp256 => 6,
                    Cipher::Gcmp128 => 7,
                    Cipher::Gcmp256 => 8,
                };
                bytes[17] = 36; // sec_key structure size
                bytes[18] = key.index();
                bytes[19] = material.len() as u8;
                put_key(&mut bytes[20..20 + material.len()], key.cipher(), material);
            }
            // Source reserves the full two-key structure in skb even though
            // its TLV length is44/8. Preserve the zero tail; do not invent trim.
        }
        Ok(Self { key, bytes })
    }
}

/// The key bytes the firmware takes. TKIP's TX and RX Michael keys swap
/// places, as `mt7925_mcu_sta_key_tlv` and `mt76_connac_mcu_sta_key_tlv` copy
/// them.
fn put_key(out: &mut [u8], cipher: Cipher, material: &[u8]) {
    out.copy_from_slice(material);
    if cipher == Cipher::Tkip {
        out[16..24].copy_from_slice(&material[24..32]);
        out[24..32].copy_from_slice(&material[16..24]);
    }
}

#[cfg(test)]
mod tests;
