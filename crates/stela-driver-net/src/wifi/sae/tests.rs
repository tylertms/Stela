// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! SAE against the IEEE 802.11-2020 Annex J.10 vectors hostapd's
//! `sae_tests` checks, and between two exchanges.
use super::*;

fn hex(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|at| u8::from_str_radix(&text[at..at + 2], 16).unwrap())
        .collect()
}

fn scalar(text: &str) -> Scalar {
    Scalar::from_repr(*FieldBytes::from_slice(&hex(text))).unwrap()
}

const PASSWORD: &[u8] = b"mekmitasdigoat";
const OWN: Mac = Mac([0x4d, 0x3f, 0x2f, 0xff, 0xe3, 0x87]);
const PEER: Mac = Mac([0xa5, 0xd8, 0xaa, 0x95, 0x8e, 0x3c]);

/// Distinct scalars below r from a counter.
fn counter() -> impl FnMut() -> [u8; 32] {
    let mut next = 0u8;
    move || {
        next += 1;
        [next; 32]
    }
}

/// Each step runs a bounded share of hostapd's 40 rounds and the stepped
/// hunt finds the element the whole one does.
#[test]
fn hunting_and_pecking_runs_a_bounded_share_of_its_rounds_a_step() {
    let mut hunt = Hunt::new(PASSWORD, OWN, PEER);
    let mut steps = 0;
    let element = loop {
        let before = hunt.counter;
        let found = hunt.step(ROUNDS_PER_STEP).unwrap();
        assert!(hunt.counter - before <= ROUNDS_PER_STEP);
        steps += 1;
        if let Some(element) = found {
            break element;
        }
    };
    assert_eq!(steps, ROUNDS / ROUNDS_PER_STEP);
    assert_eq!(element, hunt_and_peck(PASSWORD, OWN, PEER).unwrap());
    // The stepped exchange commits exactly as the one built at once.
    let mut deriving = Deriving::new(
        Derivation::HuntingAndPecking,
        PASSWORD,
        None,
        None,
        OWN,
        PEER,
    )
    .unwrap();
    let mut random = counter();
    let stepped = loop {
        if let Some(exchange) = deriving.step(&mut random).unwrap() {
            break exchange;
        }
    };
    let whole = Exchange::new(
        Derivation::HuntingAndPecking,
        PASSWORD,
        None,
        None,
        OWN,
        PEER,
        &mut counter(),
    )
    .unwrap();
    assert_eq!(stepped.commit(), whole.commit());
}

#[test]
fn hunting_and_pecking_derives_the_annex_j_keys() {
    let element = hunt_and_peck(PASSWORD, OWN, PEER).unwrap();
    let mask = scalar("9507a90f777a044d6a0830b91ea3d5dd70bece44e1acffb86983b5e1bf9fb322");
    let mut exchange = Exchange::with_secrets(
        Derivation::HuntingAndPecking,
        element,
        scalar("992465fd3daa3c60aa6565b7f62a2a7f2e12dd12f198faf4fbed89d7ff1ace94"),
        &mask,
    );
    let commit = exchange.commit();
    assert_eq!((commit.transaction, commit.status), (1, 0));
    assert_eq!(
        commit.data,
        hex(concat!(
            "13002e2c0f0db52440ad146d967114ce005ce1eab0aa2c2e5c2871b774f6c257",
            "5c65d5ad9e00829707aa36ba8b859738fc961d08243505f47c035376d7ac4bc8",
            "d7b95083bf43827d0fc31ed778dd3671fd21a46d1091d64b6f9a1e1272621325",
            "dbe1"
        ))
    );
    let peer = hex(concat!(
        "1300591b96f3397fb945100848e7b550543b6720d88337ee93fc49fd6df7e08b",
        "5223e71b9bb048d3873f20556953a96c91536fd8ee6ca9b4a68a148b056a909b",
        "e03e83ae208f60f8ef5537858074db06687032399862999b511e0a1552a5fea3",
        "17c2"
    ));
    let confirm = exchange.receive(1, 0, &peer).unwrap().unwrap();
    assert_eq!((confirm.transaction, confirm.status), (2, 0));
    assert_eq!(confirm.data[..2], [1, 0]);
    let keys = exchange.keys.as_ref().unwrap();
    assert_eq!(
        keys.kck[..],
        hex("1e733f6d9bd53256287304338831b09a39406d121017073a5c30db36f36cb81a")
    );
    assert_eq!(
        keys.pmk[..],
        hex("4e4dfab1a2dd8ac1a91790f953faaa452ae5c6873ab75b63605ba663f8a7fe59")
    );
    assert_eq!(keys.pmkid[..], hex("8747a600eea3f9f22475df58ca1e5498"));
    // The PMK stays the exchange's until the AP's confirm verifies.
    assert!(exchange.pmk().is_none());
}

#[test]
fn hash_to_element_derives_the_annex_j_password_element() {
    let template = Template::derive(b"byteme", PASSWORD, Some(b"psk4internet"));
    let element = template
        .element(
            Mac([0x00, 0x09, 0x5b, 0x66, 0xec, 0x1e]),
            Mac([0x00, 0x0b, 0x6b, 0xd9, 0x02, 0x46]),
        )
        .to_affine()
        .to_encoded_point(false);
    assert_eq!(
        element.x().unwrap()[..],
        hex("c93049b9e64000f848201649e999f2b5c22dea69b5632c9df4d633b8aa1f6c1e")
    );
    assert_eq!(
        element.y().unwrap()[..],
        hex("73634e94b53d82e7383a8d258199d9dc1a5ee8269d060382ccbf33e614ff59a0")
    );
}

fn pair(derivation: Derivation) -> (Exchange, Exchange) {
    let template = Template::derive(b"byteme", PASSWORD, None);
    let template = (derivation == Derivation::HashToElement).then_some(&template);
    let mut random = counter();
    let station = Exchange::new(derivation, PASSWORD, template, None, OWN, PEER, &mut random);
    let ap = Exchange::new(derivation, PASSWORD, template, None, PEER, OWN, &mut random);
    (station.unwrap(), ap.unwrap())
}

#[test]
fn derivations_agree_between_station_and_ap() {
    for derivation in [Derivation::HuntingAndPecking, Derivation::HashToElement] {
        let (mut station, mut ap) = pair(derivation);
        let (station_commit, ap_commit) = (station.commit(), ap.commit());
        let ap_confirm = ap
            .receive(1, station_commit.status, &station_commit.data)
            .unwrap()
            .unwrap();
        let station_confirm = station
            .receive(1, ap_commit.status, &ap_commit.data)
            .unwrap()
            .unwrap();
        assert_eq!(station.receive(2, 0, &ap_confirm.data), Ok(None));
        assert_eq!(ap.receive(2, 0, &station_confirm.data), Ok(None));
        assert!(station.accepted() && ap.accepted());
        assert_eq!(station.pmk().unwrap()[..], ap.pmk().unwrap()[..]);
        assert_eq!(station.pmkid(), ap.pmkid());
    }
}

#[test]
fn a_repeated_ap_commit_gets_the_confirm_again_with_the_next_send_confirm() {
    let (mut station, ap) = pair(Derivation::HuntingAndPecking);
    let commit = ap.commit();
    let first = station.receive(1, 0, &commit.data).unwrap().unwrap();
    let again = station.receive(1, 0, &commit.data).unwrap().unwrap();
    assert_eq!(
        (&first.data[..2], &again.data[..2]),
        (&[1, 0][..], &[2, 0][..])
    );
}

#[test]
fn a_wrong_confirm_a_reflected_commit_and_a_denial_end_the_exchange() {
    let (mut station, ap) = pair(Derivation::HuntingAndPecking);
    let own = station.commit();
    assert_eq!(station.receive(1, 0, &own.data), Err(Error::Reflection));
    station.receive(1, 0, &ap.commit().data).unwrap();
    let mut forged = [0u8; 34];
    forged[0] = 1;
    assert_eq!(station.receive(2, 0, &forged), Err(Error::Confirm));
    assert!(!station.accepted() && station.pmk().is_none());

    let (mut station, _) = pair(Derivation::HuntingAndPecking);
    assert_eq!(station.receive(1, 77, &[19, 0]), Err(Error::Status(77)));
    assert_eq!(station.receive(2, 0, &[0; 34]), Err(Error::Sequence));
    // A hash-to-element commit answers a hunting-and-pecking one only with
    // its own status.
    assert_eq!(
        station.receive(1, STATUS_HASH_TO_ELEMENT, &ap.commit().data),
        Err(Error::Status(STATUS_HASH_TO_ELEMENT))
    );
}

#[test]
fn a_commit_outside_the_group_or_curve_is_refused() {
    let (mut station, ap) = pair(Derivation::HuntingAndPecking);
    let mut commit = ap.commit().data;
    commit[0] = 20;
    assert_eq!(station.receive(1, 0, &commit), Err(Error::Group));
    let mut commit = ap.commit().data;
    commit[2..34].fill(0);
    assert_eq!(station.receive(1, 0, &commit), Err(Error::Scalar));
    let mut commit = ap.commit().data;
    commit[97] ^= 1;
    assert_eq!(station.receive(1, 0, &commit), Err(Error::Element));
    assert_eq!(station.receive(1, 0, &commit[..50]), Err(Error::Malformed));
}

#[test]
fn an_rsnxe_with_sae_h2e_picks_hash_to_element() {
    assert_eq!(
        Derivation::for_rsnxe(Some(&H2E_RSNXE[2..])),
        Derivation::HashToElement
    );
    assert_eq!(
        Derivation::for_rsnxe(Some(&[0x01])),
        Derivation::HuntingAndPecking
    );
    assert_eq!(Derivation::for_rsnxe(None), Derivation::HuntingAndPecking);
}

#[test]
fn an_anti_clogging_token_goes_after_the_group_or_in_its_container() {
    let (mut station, _) = pair(Derivation::HuntingAndPecking);
    let retry = station
        .receive(1, 76, &[19, 0, 0xaa, 0xbb])
        .unwrap()
        .unwrap();
    assert_eq!((retry.transaction, retry.status), (1, 0));
    assert_eq!(retry.data[..4], [19, 0, 0xaa, 0xbb]);
    assert_eq!(retry.data.len(), 4 + 96);

    let template = Template::derive(b"byteme", PASSWORD, Some(b"id"));
    let mut station = Exchange::new(
        Derivation::HashToElement,
        PASSWORD,
        Some(&template),
        Some(b"id"),
        OWN,
        PEER,
        &mut counter(),
    )
    .unwrap();
    assert_eq!(
        station.receive(1, 76, &[19, 0, 0xaa]),
        Err(Error::Malformed)
    );
    let retry = station
        .receive(1, 76, &[19, 0, 255, 3, 93, 0xaa, 0xbb])
        .unwrap()
        .unwrap();
    assert_eq!(retry.status, STATUS_HASH_TO_ELEMENT);
    assert_eq!(retry.data[..2], [19, 0]);
    assert_eq!(
        retry.data[98..],
        [255, 3, 33, b'i', b'd', 255, 3, 93, 0xaa, 0xbb]
    );
}

/// `ie_offset`: an AP MLD's multi-link element follows each frame's SAE
/// fields, the hash-to-element containers among them.
#[test]
fn an_ap_mld_element_follows_the_sae_fields() {
    let mld = [255, 10, 107, 0, 0, 7, 2, 9, 9, 9, 9, 9];
    let mut commit = vec![19, 0];
    commit.extend_from_slice(&[1; SCALAR_BYTES + ELEMENT_BYTES]);
    commit.extend_from_slice(&[EXTENSION, 3, REJECTED_GROUPS, 20, 0]);
    commit.extend_from_slice(&[EXTENSION, 2, TOKEN_CONTAINER, 7]);
    let fields = commit.len();
    commit.extend_from_slice(&mld);
    assert_eq!(fields_end(COMMIT, STATUS_HASH_TO_ELEMENT, &commit), fields);
    let mut clogging = vec![19, 0, EXTENSION, 2, TOKEN_CONTAINER, 7];
    clogging.extend_from_slice(&mld);
    assert_eq!(fields_end(COMMIT, STATUS_ANTI_CLOGGING, &clogging), 6);
    let mut confirm = vec![1, 0];
    confirm.extend_from_slice(&[0; 32]);
    confirm.extend_from_slice(&mld);
    assert_eq!(fields_end(CONFIRM, 0, &confirm), 34);
    // A short frame ends at its own length.
    assert_eq!(fields_end(CONFIRM, 0, &[1, 0]), 2);
}
