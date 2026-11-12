// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! WPA3-Personal Simultaneous Authentication of Equals over the NIST P-256
//! group 19 (IEEE 802.11-2020 12.4): the password element by hunting and
//! pecking (12.4.4.2.2) or hash-to-element (12.4.4.2.3), the commit and
//! confirm exchange (12.4.5, 12.4.7) and the PMK and PMKID it yields, in the
//! order hostapd's `sae_prepare_commit`, `sae_process_commit` and
//! `sae_check_confirm` take them. The station's frames travel through
//! [`Station::external_authentication_transmit`] and the AP's arrive from
//! [`Station::receive_external_authentication`] as transaction, status and
//! data; the PMK then keys the 4-way handshake of
//! [`Supplicant`](super::supplicant::Supplicant) under [`Akm::Sae`].
//!
//! [`Station::external_authentication_transmit`]: super::station::Station::external_authentication_transmit
//! [`Station::receive_external_authentication`]: super::station::Station::receive_external_authentication
//! [`Akm::Sae`]: super::supplicant::Akm::Sae
use super::{
    station::{
        external::{Received, Session},
        wire::multi_link::Basic,
    },
    supplicant::{hmac_sha256, kdf_sha256},
};
use crate::Mac;
use alloc::vec::Vec;
use p256::{
    AffinePoint, EncodedPoint, FieldBytes, FieldElement, NistP256, ProjectivePoint, Scalar, U256,
    elliptic_curve::{
        Curve, Field, PrimeField,
        bigint::Encoding,
        generic_array::GenericArray,
        hash2curve::{FromOkm, MapToCurve},
        point::DecompressPoint,
        sec1::{FromEncodedPoint, ToEncodedPoint},
        subtle::{Choice, ConditionallySelectable, ConstantTimeEq},
    },
};
use zeroize::{Zeroize, Zeroizing};

/// The finite cyclic group: 256-bit random ECP group (NIST P-256).
pub const GROUP: u16 = 19;
/// WLAN_STATUS_ANTI_CLOG_REQUIRED.
pub const STATUS_ANTI_CLOGGING: u16 = 76;
/// WLAN_STATUS_SAE_HASH_TO_ELEMENT: the status of every hash-to-element
/// commit.
pub const STATUS_HASH_TO_ELEMENT: u16 = 126;
const COMMIT: u16 = 1;
const CONFIRM: u16 = 2;
/// hostapd's `k`: hunting and pecking runs at least this many rounds,
/// whichever round finds the element, so its time does not reveal which.
const ROUNDS: u8 = 40;
const MAX_ROUNDS: u8 = 200;
/// Hunting-and-pecking rounds one [`Deriving::step`] runs, so that a
/// station poll derives the element over several polls, never in one.
pub const ROUNDS_PER_STEP: u8 = 4;
const ELEMENT_BYTES: usize = 64;
const SCALAR_BYTES: usize = 32;
/// Element ID Extension and its password identifier and anti-clogging token
/// container IDs.
const EXTENSION: u8 = 255;
const PASSWORD_IDENTIFIER: u8 = 33;
const REJECTED_GROUPS: u8 = 92;
const TOKEN_CONTAINER: u8 = 93;
/// The P-256 prime, the context of the hunting-and-pecking KDF.
const PRIME: [u8; 32] = [
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
];

/// The RSNXE a station's association request carries after a
/// hash-to-element exchange: SAE-H2E set, so the AP sees no downgrade.
pub const H2E_RSNXE: [u8; 3] = [244, 1, SAE_H2E];
/// RSNXE capability bit 5, WLAN_RSNX_CAPAB_SAE_H2E.
const SAE_H2E: u8 = 1 << 5;

/// How the password element comes from the password.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Derivation {
    /// Hunting and pecking, status 0 commits.
    HuntingAndPecking,
    /// Hash-to-element, status 126 commits, for an AP whose RSNXE sets
    /// SAE-H2E.
    HashToElement,
}

impl Derivation {
    /// The derivation for an AP's RSNXE body: hash-to-element when it sets
    /// SAE-H2E, as wpa_supplicant's `sae_pwe` 2 picks it.
    pub fn for_rsnxe(rsnxe: Option<&[u8]>) -> Self {
        match rsnxe.and_then(<[u8]>::first) {
            Some(capabilities) if capabilities & SAE_H2E != 0 => Self::HashToElement,
            _ => Self::HuntingAndPecking,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The peer used a group other than 19.
    Group,
    /// A length or field the frame does not hold.
    Malformed,
    /// The peer's scalar is not in (1, r).
    Scalar,
    /// The peer's element is not a point of the curve.
    Element,
    /// The peer sent the station's own scalar and element back.
    Reflection,
    /// The shared secret is the point at infinity.
    Secret,
    /// The peer's confirm does not verify under the KCK.
    Confirm,
    /// The AP ended the exchange with this status code.
    Status(u16),
    /// A frame out of the exchange's order.
    Sequence,
    /// No round of hunting and pecking found an element.
    Exhausted,
    /// The random source gave no usable scalar.
    Random,
    /// An AP MLD's frame does not name the AP MLD in its multi-link
    /// element, or an MLD exchange is not hash-to-element.
    MultiLink,
}

/// The larger then the smaller address, as 12.4.4.2 orders them.
fn addresses(a: Mac, b: Mac) -> [u8; 12] {
    let (high, low) = if a.0 > b.0 { (a, b) } else { (b, a) };
    let mut out = [0; 12];
    out[..6].copy_from_slice(&high.0);
    out[6..].copy_from_slice(&low.0);
    out
}

/// The password element by hunting and pecking, a few rounds at a time: the
/// first `pwd-value` below p that is the x of a curve point, with the y whose
/// parity is the `pwd-seed`'s (12.4.4.2.2).
struct Hunt {
    base: [u8; 12],
    password: Zeroizing<Vec<u8>>,
    counter: u8,
    found: Choice,
    element: AffinePoint,
}

impl Drop for Hunt {
    fn drop(&mut self) {
        self.element.zeroize();
    }
}

impl Hunt {
    fn new(password: &[u8], a: Mac, b: Mac) -> Self {
        Self {
            base: addresses(a, b),
            password: Zeroizing::new(password.to_vec()),
            counter: 1,
            found: Choice::from(0),
            element: AffinePoint::IDENTITY,
        }
    }

    /// Up to `rounds` more rounds: the element once every round hostapd
    /// runs has run and one found it.
    fn step(&mut self, rounds: u8) -> Result<Option<ProjectivePoint>, Error> {
        for _ in 0..rounds {
            if self.counter > ROUNDS && bool::from(self.found) {
                break;
            }
            if self.counter > MAX_ROUNDS {
                return Err(Error::Exhausted);
            }
            let seed = Zeroizing::new(hmac_sha256(
                &self.base,
                &[&self.password[..], &[self.counter]],
            ));
            let value = kdf_sha256(&seed[..], b"SAE Hunting and Pecking", &PRIME, 32);
            let candidate =
                AffinePoint::decompress(FieldBytes::from_slice(&value), Choice::from(seed[31] & 1));
            let take = candidate.is_some() & !self.found;
            self.element = AffinePoint::conditional_select(
                &self.element,
                &candidate.unwrap_or(AffinePoint::IDENTITY),
                take,
            );
            self.found |= take;
            self.counter += 1;
        }
        if self.counter > ROUNDS && bool::from(self.found) {
            Ok(Some(self.element.into()))
        } else if self.counter > MAX_ROUNDS {
            Err(Error::Exhausted)
        } else {
            Ok(None)
        }
    }
}

/// The whole hunt in one call, for the tests' known answers.
#[cfg(test)]
fn hunt_and_peck(password: &[u8], a: Mac, b: Mac) -> Result<ProjectivePoint, Error> {
    let mut hunt = Hunt::new(password, a, b);
    loop {
        if let Some(element) = hunt.step(MAX_ROUNDS)? {
            return Ok(element);
        }
    }
}

/// Where a [`Deriving`] exchange's password element stands.
enum Work {
    Hunt(Hunt),
    Element(ProjectivePoint),
}

/// An exchange whose password element is still being derived. Each
/// [`Self::step`] does a bounded share of the work, at most
/// [`ROUNDS_PER_STEP`] rounds of hunting and pecking, so that a station
/// derives across its polls; the step that finds the element also draws the
/// commit's secrets and gives the [`Exchange`].
pub struct Deriving {
    derivation: Derivation,
    identifier: Option<Vec<u8>>,
    work: Work,
}

impl core::fmt::Debug for Deriving {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Deriving")
            .field("derivation", &self.derivation)
            .finish_non_exhaustive()
    }
}

impl Drop for Deriving {
    fn drop(&mut self) {
        if let Work::Element(element) = &mut self.work {
            element.zeroize();
        }
    }
}

impl Deriving {
    /// Begin the exchange of `own` with `peer`. `template` serves
    /// [`Derivation::HashToElement`]; hunting and pecking derives the element
    /// from `password` over the steps.
    pub fn new(
        derivation: Derivation,
        password: &[u8],
        template: Option<&Template>,
        identifier: Option<&[u8]>,
        own: Mac,
        peer: Mac,
    ) -> Result<Self, Error> {
        let work = match (derivation, template) {
            (Derivation::HashToElement, Some(template)) => {
                Work::Element(template.element(own, peer))
            }
            (Derivation::HashToElement, None) => return Err(Error::Sequence),
            (Derivation::HuntingAndPecking, _) => Work::Hunt(Hunt::new(password, own, peer)),
        };
        Ok(Self {
            derivation,
            identifier: identifier.map(<[u8]>::to_vec),
            work,
        })
    }

    /// Begin the exchange of an external authentication session: its
    /// station, AP and, for hash-to-element, SSID; an MLD session's
    /// exchange runs between the two MLDs' addresses, as wpa_supplicant
    /// gives SAE its own and the AP MLD's, by hash-to-element, whose token
    /// container leaves the AP's multi-link element apart. The frames go out
    /// through [`Station::external_authentication_transmit`] under the same
    /// session.
    ///
    /// [`Station::external_authentication_transmit`]: super::station::Station::external_authentication_transmit
    pub fn for_session(
        session: &Session,
        derivation: Derivation,
        password: &[u8],
        identifier: Option<&[u8]>,
    ) -> Result<Self, Error> {
        let (own, peer) = match session.multi_link() {
            Some(_) if derivation != Derivation::HashToElement => return Err(Error::MultiLink),
            Some(affiliation) => (affiliation.mld.local, affiliation.mld.ap),
            None => (session.local(), session.peer()),
        };
        let template = (derivation == Derivation::HashToElement)
            .then(|| Template::derive(session.ssid(), password, identifier));
        Self::new(
            derivation,
            password,
            template.as_ref(),
            identifier,
            own,
            peer,
        )
    }

    /// One bounded share of the derivation: the exchange, its first commit
    /// ready, once the element is found.
    pub fn step(
        &mut self,
        random: &mut dyn FnMut() -> [u8; 32],
    ) -> Result<Option<Exchange>, Error> {
        let element = match &mut self.work {
            Work::Hunt(hunt) => match hunt.step(ROUNDS_PER_STEP)? {
                Some(element) => element,
                None => return Ok(None),
            },
            Work::Element(element) => *element,
        };
        let mut exchange = Exchange::commit_to(self.derivation, element, random)?;
        exchange.identifier = self.identifier.take();
        Ok(Some(exchange))
    }
}

/// The password element template of hash-to-element (12.4.4.2.3): the sum
/// of the simplified SWU images of two hashes of the password, its optional
/// identifier and the SSID. A station keeps it per network and password.
pub struct Template(ProjectivePoint);

impl core::fmt::Debug for Template {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("Template")
    }
}

impl Drop for Template {
    fn drop(&mut self) {
        self.0.zeroize();
    }
}

impl Template {
    pub fn derive(ssid: &[u8], password: &[u8], identifier: Option<&[u8]>) -> Self {
        // pwd-seed = HKDF-Extract(ssid, password [|| identifier])
        let mut secret = Zeroizing::new(password.to_vec());
        secret.extend_from_slice(identifier.unwrap_or_default());
        let seed = Zeroizing::new(hmac_sha256(ssid, &[&secret]));
        let map = |info: &[u8]| {
            // pwd-value = HKDF-Expand(pwd-seed, info, len(p) + len(p) / 2)
            let first = Zeroizing::new(hmac_sha256(&seed[..], &[info, &[1]]));
            let second = Zeroizing::new(hmac_sha256(&seed[..], &[&first[..], info, &[2]]));
            let mut value = Zeroizing::new([0; 48]);
            value[..32].copy_from_slice(&first[..]);
            value[32..].copy_from_slice(&second[..16]);
            FieldElement::from_okm(GenericArray::from_slice(&value[..])).map_to_curve()
        };
        Self(map(b"SAE Hash to Element u1 P1") + map(b"SAE Hash to Element u2 P2"))
    }

    /// The password element of the exchange between `a` and `b`:
    /// `((H(0, MAX || MIN) mod (r - 1)) + 1) * PT`.
    pub fn element(&self, a: Mac, b: Mac) -> ProjectivePoint {
        let value = U256::from_be_slice(&hmac_sha256(&[0; 32], &[&addresses(a, b)]));
        let order = NistP256::ORDER.wrapping_sub(&U256::ONE);
        // A 256-bit value is below 2(r - 1), so one subtraction reduces it.
        let reduced = if value >= order {
            value.wrapping_sub(&order)
        } else {
            value
        };
        let scale = Scalar::from_repr(reduced.wrapping_add(&U256::ONE).to_be_bytes().into())
            .expect("a value at most r - 1 is a scalar");
        self.0 * scale
    }
}

/// An SAE frame's transaction, status and data after the algorithm number.
#[derive(Clone, PartialEq, Eq)]
pub struct Frame {
    pub transaction: u16,
    pub status: u16,
    pub data: Vec<u8>,
}

impl core::fmt::Debug for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Frame")
            .field("transaction", &self.transaction)
            .field("status", &self.status)
            .field("length", &self.data.len())
            .finish()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// The station's commit is out.
    Committed,
    /// The station's confirm is out.
    Confirmed,
    /// The AP's confirm verified.
    Accepted,
}

/// The keys of an exchange whose commits both arrived.
struct Keys {
    kck: Zeroizing<[u8; 32]>,
    pmk: Zeroizing<[u8; 32]>,
    pmkid: [u8; 16],
}

/// The station's side of one SAE exchange with one AP.
pub struct Exchange {
    derivation: Derivation,
    identifier: Option<Vec<u8>>,
    element: ProjectivePoint,
    random: Scalar,
    own_scalar: Scalar,
    own_element: AffinePoint,
    /// The token of the AP's last anti-clogging request.
    token: Option<Vec<u8>>,
    peer: Option<(Scalar, AffinePoint)>,
    keys: Option<Keys>,
    /// The send-confirm counter of the station's last confirm.
    send_confirm: u16,
    state: State,
}

impl Drop for Exchange {
    fn drop(&mut self) {
        self.element.zeroize();
        self.random.zeroize();
    }
}

impl core::fmt::Debug for Exchange {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Exchange")
            .field("derivation", &self.derivation)
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

/// A scalar in (1, r) from `random`, as `sae_get_rand` draws them.
fn random_scalar(random: &mut dyn FnMut() -> [u8; 32]) -> Result<Scalar, Error> {
    for _ in 0..MAX_ROUNDS {
        let bytes = Zeroizing::new(random());
        let scalar = Option::<Scalar>::from(Scalar::from_repr(*FieldBytes::from_slice(&bytes[..])));
        if let Some(scalar) = scalar.filter(|scalar| !small(scalar)) {
            return Ok(scalar);
        }
    }
    Err(Error::Random)
}

/// Whether a scalar is 0 or 1.
fn small(scalar: &Scalar) -> bool {
    bool::from(scalar.is_zero() | scalar.ct_eq(&Scalar::ONE))
}

fn encode(scalar: &Scalar, element: &AffinePoint, out: &mut Vec<u8>) {
    out.extend_from_slice(&scalar.to_repr());
    let point = element.to_encoded_point(false);
    out.extend_from_slice(point.x().expect("an element is a finite point"));
    out.extend_from_slice(point.y().expect("an element is a finite point"));
}

impl Exchange {
    /// Begin the exchange of `own` with `peer` and write its first commit,
    /// running every step of its [`Deriving`] at once, as a peer that is not
    /// polled may.
    pub fn new(
        derivation: Derivation,
        password: &[u8],
        template: Option<&Template>,
        identifier: Option<&[u8]>,
        own: Mac,
        peer: Mac,
        random: &mut dyn FnMut() -> [u8; 32],
    ) -> Result<Self, Error> {
        let mut deriving = Deriving::new(derivation, password, template, identifier, own, peer)?;
        loop {
            if let Some(exchange) = deriving.step(random)? {
                return Ok(exchange);
            }
        }
    }

    /// The commit's secrets for the password element.
    fn commit_to(
        derivation: Derivation,
        element: ProjectivePoint,
        random: &mut dyn FnMut() -> [u8; 32],
    ) -> Result<Self, Error> {
        let random_value = random_scalar(random)?;
        let mut mask = random_scalar(random)?;
        let mut exchange = Self::with_secrets(derivation, element, random_value, &mask);
        // sae_derive_commit draws again when rand + mask mod r is below 2.
        while small(&exchange.own_scalar) {
            mask = random_scalar(random)?;
            exchange = Self::with_secrets(derivation, element, random_scalar(random)?, &mask);
        }
        mask.zeroize();
        Ok(exchange)
    }

    /// Take the AP's frame of the session, as
    /// [`Station::receive_external_authentication`] passed it on. An AP
    /// MLD's commit and confirm must name it in the multi-link element after
    /// their SAE fields, as `wpas_sme_ml_auth` checks.
    ///
    /// [`Station::receive_external_authentication`]: super::station::Station::receive_external_authentication
    pub fn receive_from(&mut self, received: &Received) -> Result<Option<Frame>, Error> {
        let (transaction, status, data) =
            (received.transaction(), received.status(), received.data());
        let Some(affiliation) = received.session().multi_link() else {
            return self.receive(transaction, status, data);
        };
        let end = fields_end(transaction, status, data);
        let named = Basic::find(&data[end..]).ok().flatten();
        if matches!(status, 0 | STATUS_HASH_TO_ELEMENT)
            && named.is_none_or(|basic| basic.address != affiliation.mld.ap)
        {
            return Err(Error::MultiLink);
        }
        self.receive(transaction, status, &data[..end])
    }

    /// commit-scalar = (rand + mask) mod r, COMMIT-ELEMENT = -(mask * PWE).
    fn with_secrets(
        derivation: Derivation,
        element: ProjectivePoint,
        random: Scalar,
        mask: &Scalar,
    ) -> Self {
        Self {
            derivation,
            identifier: None,
            element,
            random,
            own_scalar: random + mask,
            own_element: (-(element * mask)).to_affine(),
            token: None,
            peer: None,
            keys: None,
            send_confirm: 0,
            state: State::Committed,
        }
    }

    /// The station's commit: the group, the anti-clogging token the AP
    /// asked for, the scalar and the element, and for hash-to-element the
    /// password identifier and the token in its container (12.4.7.4).
    pub fn commit(&self) -> Frame {
        let mut data = Vec::with_capacity(2 + SCALAR_BYTES + ELEMENT_BYTES);
        data.extend_from_slice(&GROUP.to_le_bytes());
        let h2e = self.derivation == Derivation::HashToElement;
        if let (false, Some(token)) = (h2e, &self.token) {
            data.extend_from_slice(token);
        }
        encode(&self.own_scalar, &self.own_element, &mut data);
        if let Some(identifier) = &self.identifier {
            data.extend_from_slice(&[EXTENSION, identifier.len() as u8 + 1, PASSWORD_IDENTIFIER]);
            data.extend_from_slice(identifier);
        }
        if let (true, Some(token)) = (h2e, &self.token) {
            data.extend_from_slice(&[EXTENSION, token.len() as u8 + 1, TOKEN_CONTAINER]);
            data.extend_from_slice(token);
        }
        Frame {
            transaction: COMMIT,
            status: self.commit_status(),
            data,
        }
    }

    const fn commit_status(&self) -> u16 {
        match self.derivation {
            Derivation::HuntingAndPecking => 0,
            Derivation::HashToElement => STATUS_HASH_TO_ELEMENT,
        }
    }

    /// Whether the AP's confirm verified.
    pub fn accepted(&self) -> bool {
        self.state == State::Accepted
    }

    /// The PMK once the AP's confirm verified.
    pub fn pmk(&self) -> Option<Zeroizing<[u8; 32]>> {
        self.keys
            .as_ref()
            .filter(|_| self.accepted())
            .map(|keys| keys.pmk.clone())
    }

    /// The PMKID once the AP's confirm verified.
    pub fn pmkid(&self) -> Option<[u8; 16]> {
        self.keys
            .as_ref()
            .filter(|_| self.accepted())
            .map(|keys| keys.pmkid)
    }

    /// Take one SAE frame from the AP: the frame the station sends next, if
    /// any. An error ends the exchange.
    pub fn receive(
        &mut self,
        transaction: u16,
        status: u16,
        data: &[u8],
    ) -> Result<Option<Frame>, Error> {
        match (transaction, status) {
            (COMMIT, STATUS_ANTI_CLOGGING) if self.state == State::Committed => {
                self.anti_clogging(data)?;
                Ok(Some(self.commit()))
            }
            (COMMIT, status) if status == self.commit_status() => match self.state {
                State::Committed => {
                    self.peer_commit(data)?;
                    self.state = State::Confirmed;
                    Ok(Some(self.confirm()))
                }
                // The AP sent its commit again: it lost the confirm.
                State::Confirmed if self.peer == Some(parse_commit(data)?) => {
                    Ok(Some(self.confirm()))
                }
                _ => Err(Error::Sequence),
            },
            (CONFIRM, 0) if self.state == State::Confirmed => {
                self.peer_confirm(data)?;
                self.state = State::Accepted;
                Ok(None)
            }
            (COMMIT | CONFIRM, 0) => Err(Error::Sequence),
            (COMMIT | CONFIRM, status) => Err(Error::Status(status)),
            _ => Err(Error::Sequence),
        }
    }

    /// Keep the token of an anti-clogging request for the next commit.
    fn anti_clogging(&mut self, data: &[u8]) -> Result<(), Error> {
        let (group, token) = data.split_at_checked(2).ok_or(Error::Malformed)?;
        if u16::from_le_bytes([group[0], group[1]]) != GROUP {
            return Err(Error::Group);
        }
        let token = match self.derivation {
            Derivation::HuntingAndPecking => token,
            Derivation::HashToElement => match token {
                [EXTENSION, length, TOKEN_CONTAINER, rest @ ..]
                    if usize::from(*length) == rest.len() + 1 =>
                {
                    rest
                }
                _ => return Err(Error::Malformed),
            },
        };
        if token.is_empty() || token.len() > 254 {
            return Err(Error::Malformed);
        }
        self.token = Some(token.to_vec());
        Ok(())
    }

    /// `sae_process_commit`: the shared secret k = F(rand * (peer-scalar *
    /// PWE + PEER-ELEMENT)), keyseed = H(0, k), KCK || PMK = KDF-512(keyseed,
    /// "SAE KCK and PMK", (scalar + peer-scalar) mod r) and the PMKID.
    fn peer_commit(&mut self, data: &[u8]) -> Result<(), Error> {
        let (scalar, element) = parse_commit(data)?;
        if scalar == self.own_scalar && element == self.own_element {
            return Err(Error::Reflection);
        }
        let secret = ((self.element * scalar + element) * self.random).to_affine();
        if bool::from(secret.is_identity()) {
            return Err(Error::Secret);
        }
        let point = secret.to_encoded_point(false);
        let shared = Zeroizing::new(*point.x().ok_or(Error::Secret)?);
        let keyseed = Zeroizing::new(hmac_sha256(&[0; 32], &[&shared[..]]));
        let sum = (self.own_scalar + scalar).to_repr();
        let keys = kdf_sha256(&keyseed[..], b"SAE KCK and PMK", &sum, 64);
        self.keys = Some(Keys {
            kck: Zeroizing::new(keys[..32].try_into().unwrap()),
            pmk: Zeroizing::new(keys[32..].try_into().unwrap()),
            pmkid: sum[..16].try_into().unwrap(),
        });
        self.peer = Some((scalar, element));
        Ok(())
    }

    /// CN(KCK, send-confirm, scalar, element, peer-scalar, peer-element).
    fn confirm_value(
        &self,
        send_confirm: u16,
        first: (&Scalar, &AffinePoint),
        second: (&Scalar, &AffinePoint),
    ) -> [u8; 32] {
        let keys = self.keys.as_ref().expect("both commits arrived");
        let mut message = Vec::with_capacity(2 + 2 * (SCALAR_BYTES + ELEMENT_BYTES));
        message.extend_from_slice(&send_confirm.to_le_bytes());
        encode(first.0, first.1, &mut message);
        encode(second.0, second.1, &mut message);
        hmac_sha256(&keys.kck[..], &[&message])
    }

    /// The station's confirm under the next send-confirm.
    fn confirm(&mut self) -> Frame {
        self.send_confirm = self.send_confirm.saturating_add(1);
        let (scalar, element) = self.peer.expect("the peer committed");
        let value = self.confirm_value(
            self.send_confirm,
            (&self.own_scalar, &self.own_element),
            (&scalar, &element),
        );
        let mut data = Vec::with_capacity(34);
        data.extend_from_slice(&self.send_confirm.to_le_bytes());
        data.extend_from_slice(&value);
        Frame {
            transaction: CONFIRM,
            status: 0,
            data,
        }
    }

    /// `sae_check_confirm`: the AP's confirm over its commit then the
    /// station's.
    fn peer_confirm(&self, data: &[u8]) -> Result<(), Error> {
        let [low, high, value @ ..] = data else {
            return Err(Error::Malformed);
        };
        if value.len() != 32 {
            return Err(Error::Malformed);
        }
        let (scalar, element) = self.peer.ok_or(Error::Sequence)?;
        let expected = self.confirm_value(
            u16::from_le_bytes([*low, *high]),
            (&scalar, &element),
            (&self.own_scalar, &self.own_element),
        );
        if bool::from(expected.ct_eq(value)) {
            Ok(())
        } else {
            Err(Error::Confirm)
        }
    }
}

/// Where the SAE fields of an AP's hash-to-element frame end and its other
/// elements begin, as `sae_parse_commit` and `sae_check_confirm` give
/// wpa_supplicant its `ie_offset`: after a commit's group, scalar, element
/// and the password identifier, rejected groups and token container that
/// follow them, after an anti-clogging request's group and token container,
/// and after a confirm's send-confirm and value.
fn fields_end(transaction: u16, status: u16, data: &[u8]) -> usize {
    let mut end = match (transaction, status) {
        (COMMIT, STATUS_ANTI_CLOGGING) => 2,
        (COMMIT, _) => 2 + SCALAR_BYTES + ELEMENT_BYTES,
        _ => 2 + 32,
    };
    let fields: &[u8] = match (transaction, status) {
        (COMMIT, STATUS_ANTI_CLOGGING) => &[TOKEN_CONTAINER],
        (COMMIT, _) => &[PASSWORD_IDENTIFIER, REJECTED_GROUPS, TOKEN_CONTAINER],
        _ => &[],
    };
    while let Some(&[EXTENSION, length, id]) = data.get(end..end + 3)
        && fields.contains(&id)
    {
        end += 2 + usize::from(length);
    }
    end.min(data.len())
}

/// A commit's scalar in (1, r) and its element on the curve; elements after
/// the element are the AP's and carry nothing the station checks.
fn parse_commit(data: &[u8]) -> Result<(Scalar, AffinePoint), Error> {
    let fixed = data
        .get(..2 + SCALAR_BYTES + ELEMENT_BYTES)
        .ok_or(Error::Malformed)?;
    if u16::from_le_bytes([fixed[0], fixed[1]]) != GROUP {
        return Err(Error::Group);
    }
    let scalar = Option::<Scalar>::from(Scalar::from_repr(*FieldBytes::from_slice(
        &fixed[2..2 + SCALAR_BYTES],
    )))
    .filter(|scalar| !small(scalar))
    .ok_or(Error::Scalar)?;
    let coordinates = &fixed[2 + SCALAR_BYTES..];
    let point = EncodedPoint::from_affine_coordinates(
        FieldBytes::from_slice(&coordinates[..32]),
        FieldBytes::from_slice(&coordinates[32..]),
        false,
    );
    let element = Option::<AffinePoint>::from(AffinePoint::from_encoded_point(&point))
        .ok_or(Error::Element)?;
    Ok((scalar, element))
}

#[cfg(test)]
mod tests;
