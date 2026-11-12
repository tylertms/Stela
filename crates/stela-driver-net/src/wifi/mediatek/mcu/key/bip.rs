// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Exact MT7925 BIP-CMAC128 batch over an already installed CCMP128 GTK.
//! Wire bytes carry the source GTK index, not an invented IGTK index field.
use super::{Context, Error, Request as DataRequest};
use crate::wifi::{
    mediatek::identity::Chip,
    security::{Cipher, KeyIdentity, KeyScope},
};
use zeroize::Zeroize;

/// Sensitive source request. The native owning installer binds both material
/// borrows to its GTK and new management-key operation.
pub struct Request {
    data: DataRequest,
}
impl Request {
    pub fn install(
        chip: Chip,
        context: Context,
        gtk: KeyIdentity,
        gtk_material: &[u8],
        index: u8,
        igtk: &[u8],
    ) -> Result<Self, Error> {
        if gtk.cipher() != Cipher::Ccmp128 || !matches!(gtk.scope(), KeyScope::Group(_)) {
            return Err(Error::InvalidBipDataKey);
        }
        if !matches!(index, 4 | 5) {
            return Err(Error::InvalidBipIndex);
        }
        if gtk_material.len() != 16 || igtk.len() != 16 {
            return Err(Error::InvalidKeyLength);
        }
        let mut data = DataRequest::build(chip, context, gtk, Some(gtk_material))?;
        if chip == Chip::Mt7925 {
            const BIP_CMAC128: u8 = 6;
            data.bytes[23] = BIP_CMAC128;
            data.bytes[25] = 32;
            data.bytes[44..60].copy_from_slice(igtk);
        } else {
            // mt76_connac_mcu_sta_key_tlv: the GTK's sec_key, then a second
            // sec_key of MCU_CIPHER_BIP_CMAC_128 with key_id 0, n_cipher 2
            // and the whole sta_rec_sec as the TLV's length.
            const BIP_CMAC128: u8 = 10;
            data.bytes[10..12].copy_from_slice(&80u16.to_le_bytes());
            data.bytes[13] = 2;
            data.bytes[52] = BIP_CMAC128;
            data.bytes[53] = 36;
            data.bytes[55] = 16;
            data.bytes[56..72].copy_from_slice(igtk);
        }
        Ok(Self { data })
    }
    pub fn bytes(&self) -> &[u8] {
        self.data.bytes()
    }
}
impl Drop for Request {
    fn drop(&mut self) {
        self.data.bytes.as_mut_slice().zeroize();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Mac,
        wifi::security::{InstallOutcome, KeySlot, ReplaySeed, TransmitSeed},
    };
    use sha2::{Digest, Sha256};
    #[test]
    fn complete_source_cached_gtk_igtk_batch_and_removal_match_all_request_bytes() {
        let mut hash = Sha256::new();
        for i in 0u16..128 {
            let mut peer = core::array::from_fn(|j| (2 + i * 7 + j as u16) as u8);
            peer[0] &= 0xfe;
            let context = Context {
                bss_index: (i * 13) as u8,
                omac_index: ((i * 7) % 64) as u8,
                station_disabled: (i / 8) % 2 != 0,
            };
            let material: [u8; 16] = core::array::from_fn(|j| (i * 17 + j as u16) as u8);
            let igtk: [u8; 16] = core::array::from_fn(|j| (i * 31 + j as u16) as u8);
            // SAFETY: independent source request model, no physical slot/users.
            let mut slot = unsafe { KeySlot::new((i * 3) % 20) }.unwrap();
            let ticket = slot
                .begin_install(
                    ((i / 2) % 4) as u8,
                    Cipher::Ccmp128,
                    &material,
                    ReplaySeed::ZERO,
                    KeyScope::Group(Mac(peer)),
                    TransmitSeed::RECEIVE_ONLY,
                )
                .unwrap();
            let gtk = DataRequest::install(Chip::Mt7925, context, ticket, &material).unwrap();
            slot.complete_install(ticket, InstallOutcome::Accepted)
                .unwrap();
            let key = slot.installed().unwrap();
            let batch = Request::install(
                Chip::Mt7925,
                context,
                key,
                &material,
                4 + (i % 2) as u8,
                &igtk,
            )
            .unwrap();
            let remove = DataRequest::remove(Chip::Mt7925, context, key).unwrap();
            for (operation, bytes) in [gtk.bytes(), batch.bytes(), remove.bytes()]
                .into_iter()
                .enumerate()
            {
                hash.update([i as u8, operation as u8, bytes.len() as u8]);
                hash.update(bytes);
            }
        }
        assert_eq!(
            alloc::format!("{:x}", hash.finalize()),
            "b0c5eedd5c3889404576c54fca865ff88ddb13b15d6e469122f939d76454ea96"
        );
    }

    #[test]
    fn a_connac2_igtk_follows_its_gtk_as_the_second_sec_key() {
        let context = Context {
            bss_index: 1,
            omac_index: 2,
            station_disabled: false,
        };
        let gtk_material: [u8; 16] = core::array::from_fn(|i| 0x40 + i as u8);
        let igtk: [u8; 16] = core::array::from_fn(|i| 0x80 + i as u8);
        for chip in [Chip::Mt7921, Chip::Mt7922] {
            // SAFETY: independent request model, no physical slot or users.
            let mut slot = unsafe { KeySlot::new(9) }.unwrap();
            let ticket = slot
                .begin_install(
                    1,
                    Cipher::Ccmp128,
                    &gtk_material,
                    ReplaySeed::ZERO,
                    KeyScope::Group(Mac([2, 6, 7, 8, 9, 10])),
                    TransmitSeed::RECEIVE_ONLY,
                )
                .unwrap();
            DataRequest::install(chip, context, ticket, &gtk_material).unwrap();
            slot.complete_install(ticket, InstallOutcome::Accepted)
                .unwrap();
            let key = slot.installed().unwrap();
            let batch = Request::install(chip, context, key, &gtk_material, 4, &igtk).unwrap();
            let b = batch.bytes();
            assert_eq!(b.len(), 8 + 80);
            // sta_req_hdr: the group key's broadcast MUAR.
            assert_eq!(b[..8], [1, 9, 1, 0, 1, 0xe, 0, 0]);
            // sta_rec_sec: tag STA_REC_KEY_V2, len, add (SET_KEY 0), n_cipher.
            assert_eq!(b[8..16], [17, 0, 80, 0, 0, 2, 0, 0]);
            // sec_key[0]: AES_CCMP, cipher_len, key_id, key_len, key.
            assert_eq!(b[16..20], [5, 36, 1, 16]);
            assert_eq!(b[20..36], gtk_material);
            assert_eq!(b[36..52], [0; 16]);
            // sec_key[1]: BIP_CMAC_128 with key_id 0.
            assert_eq!(b[52..56], [10, 36, 0, 16]);
            assert_eq!(b[56..72], igtk);
            assert_eq!(b[72..88], [0; 16]);
        }
    }
}
