// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! WPA2-Personal key management for one station: the PSK from a passphrase
//! (IEEE 802.11-2020 J.4.1), the pairwise key hierarchy (12.7.1) and the
//! supplicant side of the EAPOL-Key 4-way and group key handshakes (12.7.6,
//! 12.7.7) for AKM 00-0F-AC:2 (PSK, HMAC-SHA1 MIC), 00-0F-AC:6
//! (PSK-SHA256, AES-128-CMAC MIC) and 00-0F-AC:8 (SAE, whose PMK
//! [`sae`](super::sae) derives; AES-128-CMAC MIC), in the order wpa_supplicant's
//! `wpa_sm_rx_eapol` checks them, between the two MLDs of a multi-link
//! association with its MLO KDEs (IEEE 802.11be-2024 12.7.6) and each link's
//! group keys. The suites come from the connection's security choice
//! ([`super::connection::security`]); the keys it yields are the
//! arguments of [`KeySlot::begin_install`] and the IGTK's of
//! [`management`](super::security::management).
//!
//! [`KeySlot::begin_install`]: super::security::KeySlot::begin_install
use super::security::{Cipher, KeyScope, PacketNumber, ReplaySeed, management::ManagementCipher};
use crate::Mac;
use alloc::vec::Vec;
use hmac::{Hmac, Mac as _};
use stela_crypto::{
    Aes128, Sha1, Sha256,
    cipher::{BlockDecrypt, BlockEncrypt, KeyInit},
};
use subtle::ConstantTimeEq;
use zeroize::Zeroizing;

/// The EAPOL ethertype.
pub const EAPOL: u16 = 0x888e;
const EAPOL_KEY: u8 = 3;
const RSN_DESCRIPTOR: u8 = 2;
const HEADER: usize = 4;
/// The EAPOL-Key body's fixed fields, from the descriptor type to the key
/// data length (802.11-2020 Figure 12-32).
const BODY: usize = 95;
const NONCE: usize = 32;
const MIC_OFFSET: usize = HEADER + 77;
const MIC_BYTES: usize = 16;

// Key Information bits.
const VERSION: u16 = 0b111;
const PAIRWISE: u16 = 1 << 3;
const INSTALL: u16 = 1 << 6;
const ACK: u16 = 1 << 7;
const MIC: u16 = 1 << 8;
const SECURE: u16 = 1 << 9;
const ERROR: u16 = 1 << 10;
const REQUEST: u16 = 1 << 11;
const ENCRYPTED: u16 = 1 << 12;

/// The 802.11 key data element and KDE codes.
const RSN_ELEMENT: u8 = 0x30;
const VENDOR: u8 = 0xdd;
const IEEE_OUI: [u8; 3] = [0x00, 0x0f, 0xac];
const GTK_KDE: u8 = 1;
const MAC_ADDRESS_KDE: u8 = 3;
const IGTK_KDE: u8 = 9;
const MLO_LINK_KDE: u8 = 12;
const MLO_GTK_KDE: u8 = 16;
const MLO_IGTK_KDE: u8 = 17;
/// An MLO Link KDE's link information: the link ID and whether the AP's
/// RSNE and RSNXE follow its address.
const LINK_ID: u8 = 0x0f;
const LINK_RSNE: u8 = 1 << 4;
const LINK_RSNXE: u8 = 1 << 5;

/// The KCK and KEK lengths of every AKM here.
const KCK_BYTES: usize = 16;
const KEK_BYTES: usize = 16;

/// The authentication and key management suites handled here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Akm {
    /// 00-0F-AC:2, key descriptor version 2.
    Psk,
    /// 00-0F-AC:6, key descriptor version 3.
    PskSha256,
    /// 00-0F-AC:8, key descriptor version 0: the AKM defines the SHA-256
    /// KDF and AES-128-CMAC MIC.
    Sae,
}

impl Akm {
    /// The AKM of a 00-0F-AC suite type, as [`pmf::Selection::akm`] holds
    /// it.
    pub const fn from_suite(suite: u8) -> Option<Self> {
        match suite {
            2 => Some(Self::Psk),
            6 => Some(Self::PskSha256),
            8 => Some(Self::Sae),
            _ => None,
        }
    }
    fn descriptor_version(self) -> u16 {
        match self {
            Self::Psk => 2,
            Self::PskSha256 => 3,
            Self::Sae => 0,
        }
    }
}

/// The suites of one association: its PMF profile's, or AKM 2 with the
/// ciphers of [`AssociationRequest::key_ciphers`] and no management cipher.
///
/// [`AssociationRequest::key_ciphers`]: super::station::wire::AssociationRequest::key_ciphers
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Suites {
    pub akm: Akm,
    pub pairwise: Cipher,
    pub group: Cipher,
    /// The group management cipher once PMF is negotiated.
    pub management: Option<ManagementCipher>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// An AKM or cipher this supplicant does not derive keys for: AKMs
    /// other than 2, 6 and 8, a TKIP or WEP pairwise cipher, a WEP group.
    Unsupported,
    /// Not an EAPOL-Key frame with the RSN descriptor.
    NotKey,
    /// A length or field the frame does not hold.
    Malformed,
    /// The descriptor version is not the one the AKM uses.
    Version,
    /// The replay counter did not advance.
    Replay,
    /// The MIC does not verify under the PTK's KCK.
    Mic,
    /// Message 3's ANonce differs from message 1's.
    Nonce,
    /// Message 3's RSN element differs from the one the AP advertised.
    RsnElement,
    /// The key data did not unwrap under the KEK.
    Unwrap,
    /// No GTK KDE where one is required.
    MissingGroupKey,
    /// A GTK or IGTK KDE whose index or length does not fit its cipher.
    GroupKey,
    /// A message out of the handshake's order.
    Sequence,
    /// A multi-link message 3 whose MAC Address KDE names another AP MLD,
    /// or whose MLO Link KDEs or group keys miss a link the AP MLD accepted
    /// or differ from what the AP advertises there.
    MultiLink,
}

/// HMAC-SHA1 over the concatenation of `parts`.
fn hmac_sha1(key: &[u8], parts: &[&[u8]]) -> [u8; 20] {
    let mut mac = <Hmac<Sha1> as KeyInit>::new_from_slice(key).expect("HMAC takes any key length");
    for part in parts {
        mac.update(part);
    }
    mac.finalize().into_bytes().into()
}

pub(super) fn hmac_sha256(key: &[u8], parts: &[&[u8]]) -> [u8; 32] {
    let mut mac =
        <Hmac<Sha256> as KeyInit>::new_from_slice(key).expect("HMAC takes any key length");
    for part in parts {
        mac.update(part);
    }
    mac.finalize().into_bytes().into()
}

/// The PSK of `passphrase` for `ssid`: PBKDF2-HMAC-SHA1 with 4096
/// iterations (J.4.1).
pub fn passphrase_psk(passphrase: &[u8], ssid: &[u8]) -> Zeroizing<[u8; 32]> {
    let mut psk = Zeroizing::new([0; 32]);
    for (block, out) in psk.chunks_mut(20).enumerate() {
        let index = (block as u32 + 1).to_be_bytes();
        let mut u = Zeroizing::new(hmac_sha1(passphrase, &[ssid, &index]));
        let mut t = Zeroizing::new(*u);
        for _ in 1..4096 {
            *u = hmac_sha1(passphrase, &[&*u]);
            for (t, u) in t.iter_mut().zip(u.iter()) {
                *t ^= u;
            }
        }
        out.copy_from_slice(&t[..out.len()]);
    }
    psk
}

/// PRF-n with HMAC-SHA1 (12.7.1.2): `bytes` of output.
fn prf_sha1(key: &[u8], label: &[u8], data: &[u8], bytes: usize) -> Zeroizing<Vec<u8>> {
    let mut out = Zeroizing::new(Vec::with_capacity(bytes + 20));
    let mut counter = 0u8;
    while out.len() < bytes {
        out.extend_from_slice(&hmac_sha1(key, &[label, &[0], data, &[counter]]));
        counter += 1;
    }
    out.truncate(bytes);
    out
}

/// The SHA-256 KDF (12.7.1.7.2): `bytes` of output.
pub(super) fn kdf_sha256(
    key: &[u8],
    label: &[u8],
    context: &[u8],
    bytes: usize,
) -> Zeroizing<Vec<u8>> {
    let mut out = Zeroizing::new(Vec::with_capacity(bytes + 32));
    let length = ((bytes * 8) as u16).to_le_bytes();
    let mut counter = 1u16;
    while out.len() < bytes {
        out.extend_from_slice(&hmac_sha256(
            key,
            &[&counter.to_le_bytes(), label, context, &length],
        ));
        counter += 1;
    }
    out.truncate(bytes);
    out
}

/// AES-128-CMAC (NIST SP 800-38B).
fn cmac(key: &[u8; 16], message: &[u8]) -> [u8; 16] {
    let cipher = Aes128::new(key.into());
    let encrypt = |block: &mut [u8; 16]| cipher.encrypt_block(block.into());
    let double = |value: [u8; 16]| {
        let mut out = [0; 16];
        for i in 0..15 {
            out[i] = (value[i] << 1) | (value[i + 1] >> 7);
        }
        out[15] = (value[15] << 1) ^ (0x87 * (value[0] >> 7));
        out
    };
    let mut l = Zeroizing::new([0; 16]);
    encrypt(&mut l);
    let first = Zeroizing::new(double(*l));
    let second = Zeroizing::new(double(*first));
    let blocks = message.len().div_ceil(16).max(1);
    let mut state = Zeroizing::new([0u8; 16]);
    for block in 0..blocks {
        let start = block * 16;
        let chunk = &message[start.min(message.len())..(start + 16).min(message.len())];
        let mut input = [0u8; 16];
        input[..chunk.len()].copy_from_slice(chunk);
        if block == blocks - 1 {
            let subkey = if chunk.len() == 16 {
                &*first
            } else {
                input[chunk.len()] = 0x80;
                &*second
            };
            for (byte, key) in input.iter_mut().zip(subkey) {
                *byte ^= key;
            }
        }
        for (state, byte) in state.iter_mut().zip(input) {
            *state ^= byte;
        }
        encrypt(&mut state);
    }
    *state
}

/// AES key unwrap (RFC 3394) of `wrapped` under `kek`, checking the
/// default integrity value.
fn unwrap(kek: &[u8; 16], wrapped: &[u8]) -> Result<Zeroizing<Vec<u8>>, Error> {
    if wrapped.len() < 24 || !wrapped.len().is_multiple_of(8) {
        return Err(Error::Unwrap);
    }
    let cipher = Aes128::new(kek.into());
    let n = wrapped.len() / 8 - 1;
    let mut a: [u8; 8] = wrapped[..8].try_into().unwrap();
    let mut r = Zeroizing::new(wrapped[8..].to_vec());
    for j in (0..6).rev() {
        for i in (1..=n).rev() {
            let t = (n * j + i) as u64;
            let mut block = [0u8; 16];
            for (byte, t) in a.iter_mut().zip(t.to_be_bytes()) {
                *byte ^= t;
            }
            block[..8].copy_from_slice(&a);
            block[8..].copy_from_slice(&r[(i - 1) * 8..i * 8]);
            cipher.decrypt_block((&mut block).into());
            a.copy_from_slice(&block[..8]);
            r[(i - 1) * 8..i * 8].copy_from_slice(&block[8..]);
        }
    }
    if a.ct_eq(&[0xa6; 8]).into() {
        Ok(r)
    } else {
        Err(Error::Unwrap)
    }
}

/// AES key wrap (RFC 3394), the authenticator's side, for tests.
#[cfg(test)]
fn wrap(kek: &[u8; 16], plain: &[u8]) -> Vec<u8> {
    let cipher = Aes128::new(kek.into());
    let n = plain.len() / 8;
    let mut a = [0xa6u8; 8];
    let mut r = plain.to_vec();
    for j in 0..6 {
        for i in 1..=n {
            let mut block = [0u8; 16];
            block[..8].copy_from_slice(&a);
            block[8..].copy_from_slice(&r[(i - 1) * 8..i * 8]);
            cipher.encrypt_block((&mut block).into());
            a.copy_from_slice(&block[..8]);
            for (byte, t) in a.iter_mut().zip(((n * j + i) as u64).to_be_bytes()) {
                *byte ^= t;
            }
            r[(i - 1) * 8..i * 8].copy_from_slice(&block[8..]);
        }
    }
    let mut out = a.to_vec();
    out.extend_from_slice(&r);
    out
}

/// The pairwise transient key: KCK, KEK and the pairwise cipher's TK.
struct Ptk(Zeroizing<Vec<u8>>);

impl Ptk {
    fn derive(
        suites: &Suites,
        pmk: &[u8; 32],
        aa: Mac,
        spa: Mac,
        anonce: &[u8; 32],
        snonce: &[u8; 32],
    ) -> Self {
        let (low_mac, high_mac) = if aa.0 < spa.0 { (aa, spa) } else { (spa, aa) };
        let (low_nonce, high_nonce) = if anonce < snonce {
            (anonce, snonce)
        } else {
            (snonce, anonce)
        };
        let mut data = Vec::with_capacity(76);
        data.extend_from_slice(&low_mac.0);
        data.extend_from_slice(&high_mac.0);
        data.extend_from_slice(low_nonce);
        data.extend_from_slice(high_nonce);
        let bytes = KCK_BYTES + KEK_BYTES + suites.pairwise.key_length();
        let label = b"Pairwise key expansion";
        Self(match suites.akm {
            Akm::Psk => prf_sha1(pmk, label, &data, bytes),
            Akm::PskSha256 | Akm::Sae => kdf_sha256(pmk, label, &data, bytes),
        })
    }
    fn kck(&self) -> &[u8; 16] {
        self.0[..KCK_BYTES].try_into().unwrap()
    }
    fn kek(&self) -> &[u8; 16] {
        self.0[KCK_BYTES..KCK_BYTES + KEK_BYTES].try_into().unwrap()
    }
    fn tk(&self) -> Zeroizing<Vec<u8>> {
        Zeroizing::new(self.0[KCK_BYTES + KEK_BYTES..].to_vec())
    }
}

/// The MIC of `frame` (its MIC field zero) under `kck`.
fn mic(akm: Akm, kck: &[u8; 16], frame: &[u8]) -> [u8; 16] {
    match akm {
        Akm::Psk => hmac_sha1(kck, &[frame])[..16].try_into().unwrap(),
        Akm::PskSha256 | Akm::Sae => cmac(kck, frame),
    }
}

/// One parsed EAPOL-Key frame.
struct Frame<'a> {
    frame: &'a [u8],
    info: u16,
    replay: u64,
    nonce: [u8; 32],
    data: &'a [u8],
}

impl<'a> Frame<'a> {
    fn parse(frame: &'a [u8]) -> Result<Self, Error> {
        if frame.len() < HEADER + BODY || frame[1] != EAPOL_KEY {
            return Err(Error::NotKey);
        }
        let length = usize::from(u16::from_be_bytes([frame[2], frame[3]]));
        let frame = frame.get(..HEADER + length).ok_or(Error::Malformed)?;
        let body = &frame[HEADER..];
        if body.len() < BODY || body[0] != RSN_DESCRIPTOR {
            return Err(Error::NotKey);
        }
        let data_length = usize::from(u16::from_be_bytes([body[93], body[94]]));
        let data = body.get(BODY..BODY + data_length).ok_or(Error::Malformed)?;
        Ok(Self {
            frame,
            info: u16::from_be_bytes([body[1], body[2]]),
            replay: u64::from_be_bytes(body[5..13].try_into().unwrap()),
            nonce: body[13..45].try_into().unwrap(),
            data,
        })
    }

    /// The Key RSC field: the group key's receive sequence counter.
    fn sequence(&self) -> [u8; 8] {
        self.frame[HEADER + 61..HEADER + 69].try_into().unwrap()
    }

    fn verify(&self, akm: Akm, kck: &[u8; 16]) -> Result<(), Error> {
        let mut zeroed = self.frame.to_vec();
        zeroed[MIC_OFFSET..MIC_OFFSET + MIC_BYTES].fill(0);
        let expected = mic(akm, kck, &zeroed);
        let received = &self.frame[MIC_OFFSET..MIC_OFFSET + MIC_BYTES];
        if bool::from(expected.ct_eq(received)) {
            Ok(())
        } else {
            Err(Error::Mic)
        }
    }
}

/// Build an EAPOL-Key frame, signed under `kck` when it has the MIC bit.
fn build(
    akm: Akm,
    info: u16,
    replay: u64,
    nonce: &[u8; 32],
    data: &[u8],
    kck: Option<&[u8; 16]>,
) -> Vec<u8> {
    let length = BODY + data.len();
    let mut frame = Vec::with_capacity(HEADER + length);
    // wpa_supplicant's default eapol_version.
    frame.extend_from_slice(&[1, EAPOL_KEY]);
    frame.extend_from_slice(&(length as u16).to_be_bytes());
    frame.push(RSN_DESCRIPTOR);
    frame.extend_from_slice(&(info | akm.descriptor_version()).to_be_bytes());
    frame.extend_from_slice(&0u16.to_be_bytes());
    frame.extend_from_slice(&replay.to_be_bytes());
    frame.extend_from_slice(nonce);
    frame.extend_from_slice(&[0; 16 + 8 + 8 + MIC_BYTES]);
    frame.extend_from_slice(&(data.len() as u16).to_be_bytes());
    frame.extend_from_slice(data);
    if let Some(kck) = kck {
        let signature = mic(akm, kck, &frame);
        frame[MIC_OFFSET..MIC_OFFSET + MIC_BYTES].copy_from_slice(&signature);
    }
    frame
}

/// A pairwise or group key, as [`KeySlot::begin_install`] takes it.
///
/// [`KeySlot::begin_install`]: super::security::KeySlot::begin_install
pub struct TemporalKey {
    pub index: u8,
    pub cipher: Cipher,
    pub material: Zeroizing<Vec<u8>>,
    /// The initial receive counters: zero for a pairwise key, the
    /// authenticated Key RSC for a group key.
    pub receive: ReplaySeed,
    pub scope: KeyScope,
}

impl core::fmt::Debug for TemporalKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TemporalKey")
            .field("index", &self.index)
            .field("cipher", &self.cipher)
            .field("scope", &self.scope)
            .finish_non_exhaustive()
    }
}

/// An IGTK from its KDE, as the BIP key of
/// [`management`](super::security::management) takes it.
pub struct IntegrityKey {
    /// 4 or 5.
    pub index: u8,
    pub cipher: ManagementCipher,
    pub material: Zeroizing<Vec<u8>>,
    /// The IPN the authenticator sent: the receive floor.
    pub receive: PacketNumber,
}

impl core::fmt::Debug for IntegrityKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("IntegrityKey")
            .field("index", &self.index)
            .field("cipher", &self.cipher)
            .finish_non_exhaustive()
    }
}

/// The bodies of the 00-0F-AC KDEs of `kind` among decrypted key data
/// (12.7.2, Table 12-10), after their OUI and type.
fn kdes(data: &[u8], kind: u8) -> Result<Vec<&[u8]>, Error> {
    let mut found = Vec::new();
    let mut rest = data;
    while rest.len() >= 2 {
        let (id, length) = (rest[0], usize::from(rest[1]));
        if id == VENDOR && length == 0 {
            break;
        }
        let element = rest.get(2..2 + length).ok_or(Error::Malformed)?;
        if id == VENDOR && length >= 4 && element[..3] == IEEE_OUI && element[3] == kind {
            found.push(&element[4..]);
        }
        rest = &rest[2 + length..];
    }
    Ok(found)
}

/// The body of the first KDE of `kind`.
fn kde(data: &[u8], kind: u8) -> Result<Option<&[u8]>, Error> {
    Ok(kdes(data, kind)?.first().copied())
}

/// A KDE of `kind` holding `body`.
fn put_kde(out: &mut Vec<u8>, kind: u8, body: &[u8]) {
    out.extend_from_slice(&[VENDOR, 4 + body.len() as u8]);
    out.extend_from_slice(&IEEE_OUI);
    out.push(kind);
    out.extend_from_slice(body);
}

/// The GTK of the GTK KDE. A TKIP GTK's Michael keys trade places, as
/// `wpa_supplicant_install_gtk` swaps them, so the receive key sits where
/// mac80211 reads it.
fn group_key(
    data: &[u8],
    rsc: [u8; 8],
    suites: &Suites,
    authenticator: Mac,
) -> Result<TemporalKey, Error> {
    let body = kde(data, GTK_KDE)?.ok_or(Error::MissingGroupKey)?;
    let (index, key) = match body {
        [flags, _, key @ ..] => (flags & 0b11, key),
        _ => return Err(Error::Malformed),
    };
    if index == 0 || key.len() != suites.group.key_length() {
        return Err(Error::GroupKey);
    }
    let mut material = Zeroizing::new(key.to_vec());
    if suites.group == Cipher::Tkip {
        material[16..24].copy_from_slice(&key[24..32]);
        material[24..32].copy_from_slice(&key[16..24]);
    }
    let receive = PacketNumber::new(u64::from_le_bytes(rsc)).ok_or(Error::Malformed)?;
    Ok(TemporalKey {
        index,
        cipher: suites.group,
        material,
        receive: ReplaySeed::uniform(receive),
        scope: KeyScope::Group(authenticator),
    })
}

/// The link and GTK of an MLO GTK KDE, `wpa_mlo_install_gtk`'s layout: its
/// key ID and link ID, then its PN and the key.
fn link_group_key(body: &[u8], suites: &Suites, ap: Mac) -> Result<TemporalKey, Error> {
    let [info, rest @ ..] = body else {
        return Err(Error::Malformed);
    };
    let (pn, key) = rest.split_at_checked(6).ok_or(Error::Malformed)?;
    let index = info & 0b11;
    if index == 0 || key.len() != suites.group.key_length() || suites.group == Cipher::Tkip {
        return Err(Error::GroupKey);
    }
    let mut counter = [0; 8];
    counter[..6].copy_from_slice(pn);
    let receive = PacketNumber::new(u64::from_le_bytes(counter)).ok_or(Error::Malformed)?;
    Ok(TemporalKey {
        index,
        cipher: suites.group,
        material: Zeroizing::new(key.to_vec()),
        receive: ReplaySeed::uniform(receive),
        scope: KeyScope::Group(ap),
    })
}

/// The IGTK of an MLO IGTK KDE: its key ID and IPN, the link, then the key.
fn link_integrity_key(body: &[u8], cipher: ManagementCipher) -> Result<IntegrityKey, Error> {
    let [low, high, ipn @ ..] = body else {
        return Err(Error::Malformed);
    };
    let index = u16::from_le_bytes([*low, *high]);
    if !matches!(index, 4 | 5) || ipn.len() != 7 + cipher.key_length() {
        return Err(Error::GroupKey);
    }
    let mut counter = [0; 8];
    counter[..6].copy_from_slice(&ipn[..6]);
    Ok(IntegrityKey {
        index: index as u8,
        cipher,
        material: Zeroizing::new(ipn[7..].to_vec()),
        receive: PacketNumber::new(u64::from_le_bytes(counter)).ok_or(Error::Malformed)?,
    })
}

/// The link an MLO GTK KDE (its first byte's high nibble) or MLO IGTK KDE
/// (its ninth byte's) keys.
fn kde_link(body: &[u8], at: usize) -> Option<u8> {
    body.get(at).map(|info| info >> 4)
}

/// The IGTK of the IGTK KDE, when PMF is negotiated and the authenticator
/// sent one.
fn integrity_key(data: &[u8], suites: &Suites) -> Result<Option<IntegrityKey>, Error> {
    let (Some(cipher), Some(body)) = (suites.management, kde(data, IGTK_KDE)?) else {
        return Ok(None);
    };
    let [low, high, ipn @ ..] = body else {
        return Err(Error::Malformed);
    };
    let index = u16::from_le_bytes([*low, *high]);
    if !matches!(index, 4 | 5) || ipn.len() != 6 + cipher.key_length() {
        return Err(Error::GroupKey);
    }
    let mut counter = [0; 8];
    counter[..6].copy_from_slice(&ipn[..6]);
    Ok(Some(IntegrityKey {
        index: index as u8,
        cipher,
        material: Zeroizing::new(ipn[6..].to_vec()),
        receive: PacketNumber::new(u64::from_le_bytes(counter)).ok_or(Error::Malformed)?,
    }))
}

/// The first RSN element in key data.
fn rsn_element(data: &[u8]) -> Option<&[u8]> {
    let mut rest = data;
    while rest.len() >= 2 {
        let length = usize::from(rest[1]);
        let element = rest.get(..2 + length)?;
        if rest[0] == RSN_ELEMENT {
            return Some(element);
        }
        rest = &rest[2 + length..];
    }
    None
}

/// What a received EAPOL-Key frame produced.
#[derive(Debug, Default)]
pub struct Step {
    /// The EAPOL frame to send the authenticator.
    pub reply: Option<Vec<u8>>,
    /// The pairwise key to install, sent with message 4.
    pub pairwise: Option<TemporalKey>,
    /// A group key to install: a multi-link association's setup link's.
    pub group: Option<TemporalKey>,
    /// An IGTK to install: a multi-link association's setup link's.
    pub integrity: Option<IntegrityKey>,
    /// The group keys of a multi-link association's other links.
    pub links: Vec<LinkKeys>,
}

/// One link's group keys of a multi-link association.
#[derive(Debug)]
pub struct LinkKeys {
    pub link_id: u8,
    pub group: TemporalKey,
    pub integrity: Option<IntegrityKey>,
}

/// One link a multi-link association requested.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Link {
    pub link_id: u8,
    /// The station's address on the link.
    pub local: Mac,
    /// The AP's address on the link.
    pub ap: Mac,
    /// The AP's whole RSN element and RSNXE on the link.
    pub rsn: Vec<u8>,
    pub rsnxe: Option<Vec<u8>>,
    /// Whether the AP MLD accepted the link.
    pub accepted: bool,
}

/// The links a multi-link association's handshakes key, as wpa_supplicant
/// keeps them in `sm->mlo`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MultiLink {
    /// The setup link's ID.
    pub association: u8,
    /// The setup link and each other link the association requested.
    pub links: Vec<Link>,
}

impl MultiLink {
    fn accepted(&self) -> impl Iterator<Item = &Link> {
        self.links.iter().filter(|link| link.accepted)
    }
}

/// The supplicant state of one association.
pub struct Supplicant {
    suites: Suites,
    pmk: Zeroizing<[u8; 32]>,
    own: Mac,
    authenticator: Mac,
    /// The RSN element the station sent in its association request.
    own_rsn: Vec<u8>,
    /// The RSN element the AP advertised.
    peer_rsn: Vec<u8>,
    /// The last replay counter of a frame whose MIC verified.
    replay: Option<u64>,
    snonce: Option<[u8; 32]>,
    /// The PTK of the latest message 1 and its ANonce, until message 3
    /// verifies under it (wpa_supplicant's TPTK).
    tentative: Option<(Ptk, [u8; 32])>,
    /// The PTK message 3 confirmed.
    ptk: Option<Ptk>,
    /// The ANonce of the 4-way handshake that confirmed `ptk`, which a
    /// retransmitted message 3 repeats.
    anonce: Option<[u8; 32]>,
    /// The group key and IGTK last installed, as index and material, which
    /// wpa_supplicant never reinstalls (`wpa_supplicant_install_gtk`).
    installed_group: Option<(u8, Zeroizing<Vec<u8>>)>,
    installed_integrity: Option<(u8, Zeroizing<Vec<u8>>)>,
    /// The links of a multi-link association.
    multi_link: Option<MultiLink>,
    /// The Key Replay Counter of the next EAPOL-Key request.
    request_counter: u64,
}

impl Supplicant {
    /// A handshake for the association of `own` with `authenticator` under
    /// `pmk`. `own_rsn` and `peer_rsn` are whole RSN elements.
    pub fn new(
        suites: Suites,
        pmk: Zeroizing<[u8; 32]>,
        own: Mac,
        authenticator: Mac,
        own_rsn: Vec<u8>,
        peer_rsn: Vec<u8>,
    ) -> Result<Self, Error> {
        let keyed = |cipher| {
            matches!(
                cipher,
                Cipher::Ccmp128 | Cipher::Ccmp256 | Cipher::Gcmp128 | Cipher::Gcmp256
            )
        };
        if !keyed(suites.pairwise) || !(keyed(suites.group) || suites.group == Cipher::Tkip) {
            return Err(Error::Unsupported);
        }
        Ok(Self {
            suites,
            pmk,
            own,
            authenticator,
            own_rsn,
            peer_rsn,
            replay: None,
            snonce: None,
            tentative: None,
            ptk: None,
            anonce: None,
            installed_group: None,
            installed_integrity: None,
            multi_link: None,
            request_counter: 0,
        })
    }

    /// Key a multi-link association between the MLDs `new` named: its
    /// setup link accepted among the links it requested.
    pub fn with_multi_link(mut self, multi_link: MultiLink) -> Result<Self, Error> {
        let mut ids: Vec<u8> = multi_link.links.iter().map(|link| link.link_id).collect();
        ids.sort_unstable();
        if ids.windows(2).any(|pair| pair[0] == pair[1])
            || !multi_link
                .accepted()
                .any(|link| link.link_id == multi_link.association)
        {
            return Err(Error::MultiLink);
        }
        self.multi_link = Some(multi_link);
        Ok(self)
    }

    /// The MAC Address KDE naming the station MLD that a multi-link
    /// association's messages 2 and 4 and group message 2 carry.
    fn own_address(&self) -> Vec<u8> {
        let mut data = Vec::new();
        if self.multi_link.is_some() {
            put_kde(&mut data, MAC_ADDRESS_KDE, &self.own.0);
        }
        data
    }

    /// `wpa_supplicant_validate_link_kde` for message 3 of a multi-link
    /// association: its MAC Address KDE names the AP MLD, and one MLO Link
    /// KDE for each link the AP MLD accepted names the AP there with the RSN
    /// element and RSNXE it advertises.
    fn validate_links(&self, data: &[u8]) -> Result<(), Error> {
        let Some(multi_link) = &self.multi_link else {
            return Ok(());
        };
        if kde(data, MAC_ADDRESS_KDE)? != Some(&self.authenticator.0[..]) {
            return Err(Error::MultiLink);
        }
        let reported = kdes(data, MLO_LINK_KDE)?;
        if reported.len() != multi_link.accepted().count() {
            return Err(Error::MultiLink);
        }
        for body in reported {
            let [info, address @ ..] = body else {
                return Err(Error::Malformed);
            };
            let (address, mut elements) = address.split_at_checked(6).ok_or(Error::Malformed)?;
            let link = multi_link
                .accepted()
                .find(|link| link.link_id == info & LINK_ID)
                .ok_or(Error::MultiLink)?;
            let mut next = || -> Result<&[u8], Error> {
                let length = usize::from(*elements.get(1).ok_or(Error::Malformed)?);
                let (element, rest) = elements
                    .split_at_checked(2 + length)
                    .ok_or(Error::Malformed)?;
                elements = rest;
                Ok(element)
            };
            if address != link.ap.0
                || info & LINK_RSNE == 0
                || next()? != link.rsn
                || (info & LINK_RSNXE != 0) != link.rsnxe.is_some()
                || link
                    .rsnxe
                    .as_ref()
                    .is_some_and(|rsnxe| next().ok() != Some(&rsnxe[..]))
            {
                return Err(Error::MultiLink);
            }
        }
        Ok(())
    }

    /// The group keys of `data`: its GTK and IGTK KDEs, or each accepted
    /// link's MLO GTK and IGTK KDEs, the setup link's first, under the Key
    /// RSC `rsc` a single link's GTK takes.
    fn group_keys(
        &self,
        data: &[u8],
        rsc: [u8; 8],
    ) -> Result<(TemporalKey, Option<IntegrityKey>, Vec<LinkKeys>), Error> {
        let Some(multi_link) = &self.multi_link else {
            let group = group_key(data, rsc, &self.suites, self.authenticator)?;
            return Ok((group, integrity_key(data, &self.suites)?, Vec::new()));
        };
        let groups = kdes(data, MLO_GTK_KDE)?;
        let integrities = kdes(data, MLO_IGTK_KDE)?;
        let mut keys = Vec::new();
        for link in multi_link.accepted() {
            let body = groups
                .iter()
                .find(|body| kde_link(body, 0) == Some(link.link_id))
                .ok_or(Error::MissingGroupKey)?;
            let group = link_group_key(body, &self.suites, link.ap)?;
            let integrity = match (
                self.suites.management,
                integrities
                    .iter()
                    .find(|body| kde_link(body, 8) == Some(link.link_id)),
            ) {
                (Some(cipher), Some(body)) => Some(link_integrity_key(body, cipher)?),
                _ => None,
            };
            keys.push(LinkKeys {
                link_id: link.link_id,
                group,
                integrity,
            });
        }
        let at = keys
            .iter()
            .position(|keys| keys.link_id == multi_link.association)
            .ok_or(Error::MultiLink)?;
        let setup = keys.remove(at);
        Ok((setup.group, setup.integrity, keys))
    }

    /// Whether a 4-way handshake finished.
    pub fn established(&self) -> bool {
        self.ptk.is_some()
    }

    /// `wpa_sm_key_request` with the error bit: tells the authenticator of a
    /// failed Michael MIC, once a PTK protects the telling.
    pub fn michael_report(&mut self, pairwise: bool) -> Option<Vec<u8>> {
        let ptk = self.ptk.as_ref()?;
        let kind = if pairwise { PAIRWISE } else { 0 };
        let report = build(
            self.suites.akm,
            REQUEST | ERROR | MIC | SECURE | kind,
            self.request_counter,
            &[0; NONCE],
            &[],
            Some(ptk.kck()),
        );
        self.request_counter += 1;
        Some(report)
    }

    /// Take one EAPOL frame from the authenticator. `random` gives the
    /// SNonce of each new 4-way handshake.
    pub fn receive(
        &mut self,
        frame: &[u8],
        random: &mut dyn FnMut() -> [u8; 32],
    ) -> Result<Step, Error> {
        let key = Frame::parse(frame)?;
        if key.info & VERSION != self.suites.akm.descriptor_version() {
            return Err(Error::Version);
        }
        if key.info & (REQUEST | ERROR) != 0 || key.info & ACK == 0 {
            return Err(Error::Sequence);
        }
        if self.replay.is_some_and(|last| key.replay <= last) {
            return Err(Error::Replay);
        }
        match (key.info & PAIRWISE != 0, key.info & MIC != 0) {
            (true, false) => Ok(self.message_1(&key, random)),
            (true, true) => self.message_3(&key),
            (false, true) => self.group_message_1(&key),
            (false, false) => Err(Error::Sequence),
        }
    }

    /// Message 1 carries no MIC, so it changes nothing message 3 has not
    /// confirmed: its PTK stays tentative and its counter unrecorded.
    fn message_1(&mut self, key: &Frame<'_>, random: &mut dyn FnMut() -> [u8; 32]) -> Step {
        let snonce = *self.snonce.get_or_insert_with(random);
        let ptk = Ptk::derive(
            &self.suites,
            &self.pmk,
            self.authenticator,
            self.own,
            &key.nonce,
            &snonce,
        );
        // wpa_supplicant_send_2_of_4: a multi-link association names the
        // station MLD and its address on each other requested link.
        let mut data = self.own_rsn.clone();
        data.extend(self.own_address());
        for link in self.multi_link.iter().flat_map(|multi_link| {
            multi_link
                .links
                .iter()
                .filter(|link| link.link_id != multi_link.association)
        }) {
            let mut body = Vec::from([link.link_id]);
            body.extend_from_slice(&link.local.0);
            put_kde(&mut data, MLO_LINK_KDE, &body);
        }
        let reply = build(
            self.suites.akm,
            PAIRWISE | MIC,
            key.replay,
            &snonce,
            &data,
            Some(ptk.kck()),
        );
        self.tentative = Some((ptk, key.nonce));
        Step {
            reply: Some(reply),
            ..Step::default()
        }
    }

    fn message_3(&mut self, key: &Frame<'_>) -> Result<Step, Error> {
        if key.info & (INSTALL | ENCRYPTED) != INSTALL | ENCRYPTED {
            return Err(Error::Sequence);
        }
        if self.tentative.is_none() {
            return self.repeated_message_3(key);
        }
        let (ptk, anonce) = self.tentative.as_ref().ok_or(Error::Sequence)?;
        key.verify(self.suites.akm, ptk.kck())?;
        if *anonce != key.nonce {
            return Err(Error::Nonce);
        }
        let data = unwrap(ptk.kek(), key.data)?;
        if rsn_element(&data) != Some(&self.peer_rsn[..]) {
            return Err(Error::RsnElement);
        }
        self.validate_links(&data)?;
        let (group, integrity, links) = self.group_keys(&data, key.sequence())?;
        let reply = build(
            self.suites.akm,
            PAIRWISE | MIC | SECURE,
            key.replay,
            &[0; NONCE],
            &self.own_address(),
            Some(ptk.kck()),
        );
        let pairwise = TemporalKey {
            index: 0,
            cipher: self.suites.pairwise,
            material: ptk.tk(),
            receive: ReplaySeed::ZERO,
            scope: KeyScope::Pairwise(self.authenticator),
        };
        self.replay = Some(key.replay);
        let (ptk, anonce) = self.tentative.take().ok_or(Error::Sequence)?;
        self.ptk = Some(ptk);
        self.anonce = Some(anonce);
        // A later 4-way handshake takes a fresh SNonce.
        self.snonce = None;
        let (group, integrity) = self.fresh_group_keys(group, integrity);
        Ok(Step {
            reply: Some(reply),
            pairwise: Some(pairwise),
            group,
            integrity,
            links,
        })
    }

    /// `wpa_supplicant_process_3_of_4` for a message 3 the authenticator
    /// sent again after message 4 was lost: under the PTK and ANonce it
    /// confirmed, message 4 goes again and no key is reinstalled.
    fn repeated_message_3(&mut self, key: &Frame<'_>) -> Result<Step, Error> {
        let (Some(ptk), Some(anonce)) = (self.ptk.as_ref(), self.anonce) else {
            return Err(Error::Sequence);
        };
        key.verify(self.suites.akm, ptk.kck())?;
        if anonce != key.nonce {
            return Err(Error::Nonce);
        }
        let data = unwrap(ptk.kek(), key.data)?;
        if rsn_element(&data) != Some(&self.peer_rsn[..]) {
            return Err(Error::RsnElement);
        }
        let reply = build(
            self.suites.akm,
            PAIRWISE | MIC | SECURE,
            key.replay,
            &[0; NONCE],
            &self.own_address(),
            Some(ptk.kck()),
        );
        self.replay = Some(key.replay);
        Ok(Step {
            reply: Some(reply),
            ..Step::default()
        })
    }

    /// The group key and IGTK of a handshake, each left out when it is the
    /// one already installed, as `wpa_supplicant_install_gtk` and
    /// `wpa_supplicant_install_igtk` leave an in-use key alone so that a
    /// repeated message cannot reset its receive counter.
    fn fresh_group_keys(
        &mut self,
        group: TemporalKey,
        integrity: Option<IntegrityKey>,
    ) -> (Option<TemporalKey>, Option<IntegrityKey>) {
        let same = |installed: &Option<(u8, Zeroizing<Vec<u8>>)>, index: u8, material: &[u8]| {
            installed
                .as_ref()
                .is_some_and(|(i, m)| *i == index && m[..] == *material)
        };
        let group = if same(&self.installed_group, group.index, &group.material) {
            None
        } else {
            self.installed_group = Some((group.index, group.material.clone()));
            Some(group)
        };
        let integrity = match integrity {
            Some(key) if same(&self.installed_integrity, key.index, &key.material) => None,
            Some(key) => {
                self.installed_integrity = Some((key.index, key.material.clone()));
                Some(key)
            }
            None => None,
        };
        (group, integrity)
    }

    fn group_message_1(&mut self, key: &Frame<'_>) -> Result<Step, Error> {
        let ptk = self.ptk.as_ref().ok_or(Error::Sequence)?;
        if key.info & (SECURE | ENCRYPTED) != SECURE | ENCRYPTED {
            return Err(Error::Sequence);
        }
        key.verify(self.suites.akm, ptk.kck())?;
        let data = unwrap(ptk.kek(), key.data)?;
        let (group, integrity, links) = self.group_keys(&data, key.sequence())?;
        let reply = build(
            self.suites.akm,
            MIC | SECURE,
            key.replay,
            &[0; NONCE],
            &self.own_address(),
            Some(ptk.kck()),
        );
        self.replay = Some(key.replay);
        let (group, integrity) = self.fresh_group_keys(group, integrity);
        Ok(Step {
            reply: Some(reply),
            group,
            integrity,
            links,
            ..Step::default()
        })
    }
}

#[cfg(test)]
pub(crate) mod tests;
