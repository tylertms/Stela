//! The station drives the managed link from scan to an open port against
//! the device model and a software authenticator.
use super::*;
use crate::wifi::mediatek::bootstrap::client::{
    policy::tests::{passive_country, setup_passive_five, setup_six, six_country},
    six_ghz::tests::beacon_elements,
};
use crate::wifi::{
    connection::security::Security,
    mediatek::{bootstrap::client::modern::tests::pmf_controls::inputs_akm, station::Station},
    network::{Network, Secret},
    sae::{Derivation, Exchange},
    supplicant::tests::Authenticator,
};
use alloc::rc::Rc;
use stela_driver_base::entropy::{Entropy, EntropyError};

struct Fixed;
impl Entropy for Fixed {
    fn fill(&self, output: &mut [u8]) -> Result<(), EntropyError> {
        output.fill(0x5c);
        Ok(())
    }
}

/// The receive descriptor's encoding of 6 GHz channel 1: 6 GHz channel n
/// is 181 + (n - 1) / 4.
const SIX_CHANNEL: u8 = 181;

/// An EAPOL frame from the AP, as the device receives it in the clear.
fn eapol(model: &Model, payload: &[u8]) {
    eapol_on(model, payload, false);
}

/// An EAPOL frame from the AP, received on 6 GHz channel 1 when `six`.
fn eapol_on(model: &Model, payload: &[u8], six: bool) {
    let mut bytes = data(false);
    bytes.truncate(24);
    bytes.extend_from_slice(&[0xaa, 0xaa, 3, 0, 0, 0, 0x88, 0x8e]);
    bytes.extend_from_slice(payload);
    bytes[1] &= !0x40;
    let mut raw = incoming_bytes(&bytes, None, false);
    raw[4] = 1;
    if six {
        raw[13] = SIX_CHANNEL;
    }
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
}

/// The EAPOL payload of the station's last transmitted data frame.
fn sent_eapol(model: &Model) -> Vec<u8> {
    sent_eapol_on(model, 36)
}

/// The EAPOL payload of the station's last data frame on `channel`.
fn sent_eapol_on(model: &Model, channel: u8) -> Vec<u8> {
    let frame = finish_tx_on(model, channel).pop().unwrap();
    let at = frame
        .windows(8)
        .position(|w| w == [0xaa, 0xaa, 3, 0, 0, 0, 0x88, 0x8e])
        .unwrap();
    frame[at + 8..].to_vec()
}

#[test]
fn the_station_scans_joins_and_opens_the_port_after_the_4_way_handshake() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let (_, bss) = modern_inputs(&client);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(Arc::new(AtomicUsize::new(0)), Default::default()),
    )
    .unwrap()
    .with_vector();
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    let pmk = network.secret.pmk(&network.ssid);
    let security = Security::select(&bss.elements, true).unwrap();
    let mut ap = Authenticator::between(security.suites(), pmk, AP, LOCAL, security.element());
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    // The first poll scans; the AP's beacon answers it.
    assert!(!station.carrier());
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&bss.capabilities.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    // The scan ends: the station authenticates, then associates.
    done(&model);
    assert!(!station.carrier());
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    station.carrier();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    station.carrier();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    station.carrier();
    let mut association = vec![0x11, 0, 0, 0, 1, 0xc0];
    association.extend_from_slice(&bss.elements);
    incoming(&model, &response(0x10, &association), None, false);
    assert!(!station.carrier());
    // Messages 1 and 2.
    eapol(&model, &ap.message_1());
    assert!(!station.carrier());
    ap.message_2(&sent_eapol(&model));
    // Message 3 installs the keys and opens the port after message 4.
    let rsn = crate::wifi::connection::find(&bss.elements, 48).unwrap();
    let mut whole = vec![48, rsn.len() as u8];
    whole.extend_from_slice(rsn);
    assert!(!station.signals(), "the handshake polls");
    eapol(&model, &ap.message_3(&whole));
    assert!(station.carrier());
    // Message 4: pairwise, MIC and secure.
    ap.signed_reply(&sent_eapol(&model), 0x308);
    assert!(station.signals());
    // A pass that found nothing sleeps on the vector, with a tick for timers.
    for _ in 0..4 {
        station.carrier();
    }
    let enable = || model.bar.0.lock().unwrap().values[&0xd4204];
    assert_eq!(enable(), 0x2e07_fff5);
    let clock = crate::Clock {
        ticks_per_microsecond: 1,
    };
    let before = clock.now_us();
    assert!(station.due_us().unwrap() >= before + 100_000);
    // A frame raises it: the vector stays off through a pass that takes it,
    // and one more follows at once.
    model.bar.0.lock().unwrap().values.insert(0xd4200, 1 << 2);
    eapol(&model, &ap.message_1());
    station.radio_mut().link_mut().service().unwrap();
    let log = model.bar.0.lock().unwrap().log.clone();
    assert!(log.contains(&(1, 0xd4204, 0)));
    assert!(log.contains(&(1, 0xd4200, 1 << 2)));
    assert_eq!(enable(), 0x2e07_fff5);
    assert!(station.due_us().unwrap() <= clock.now_us());
}

/// key_mgmt=NONE: an open network's station joins a BSS without privacy or
/// RSN by open-system authentication, and its port opens with the
/// association, no handshake or key behind it; a protected BSS of the same
/// name is never chosen.
#[test]
fn an_open_network_joins_its_unprotected_ap_and_opens_the_port_at_association() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (link, model) = managed_link();
    let (_, bss) = {
        let (client, _model) = setup_modern();
        modern_inputs(&client)
    };
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::Open,
        country: None,
    };
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    // The AP's elements without its RSN element, and no privacy.
    let mut open = Vec::new();
    let mut rest = &bss.elements[..];
    while let [id, length, tail @ ..] = rest {
        let (body, next) = tail.split_at(usize::from(*length));
        if *id != 48 {
            open.extend_from_slice(&[*id, *length]);
            open.extend_from_slice(body);
        }
        rest = next;
    }
    let capabilities = bss.capabilities & !0x10;
    assert!(!station.carrier());
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&capabilities.to_le_bytes());
    body.extend_from_slice(&open);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    // Open-system authentication, then an association request with neither
    // privacy nor an RSN element.
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    station.carrier();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    station.carrier();
    let request = finish_tx(&model).pop().unwrap();
    assert_eq!(request[..2], [0, 0]);
    assert_eq!(u16::from_le_bytes([request[24], request[25]]) & 0x10, 0);
    assert!(crate::wifi::connection::find(&request[28..], 48).is_none());
    station.carrier();
    let mut association = Vec::new();
    association.extend_from_slice(&(capabilities | 1).to_le_bytes());
    association.extend_from_slice(&[0, 0, 1, 0xc0]);
    association.extend_from_slice(&open);
    incoming(&model, &response(0x10, &association), None, false);
    // The association opens the port.
    let mut carrier = false;
    for _ in 0..4 {
        carrier |= station.carrier();
    }
    assert!(carrier);
    let reported: Vec<_> = core::iter::from_fn(|| station.take_diagnostic()).collect();
    assert!(reported.is_empty(), "{reported:?}");
}

/// The station's last transmitted SAE frame: its transaction, status and
/// data.
fn sent_sae(model: &Model) -> (u16, u16, Vec<u8>) {
    let frame = finish_tx(model).pop().unwrap();
    assert_eq!(frame[..2], [0xb0, 0]);
    assert_eq!(frame[24..26], [3, 0]);
    let field = |at: usize| u16::from_le_bytes([frame[at], frame[at + 1]]);
    (field(26), field(28), frame[30..].to_vec())
}

/// An SAE frame from the AP.
fn sae_from_ap(model: &Model, frame: &crate::wifi::sae::Frame) {
    let mut body = vec![3, 0];
    body.extend_from_slice(&frame.transaction.to_le_bytes());
    body.extend_from_slice(&frame.status.to_le_bytes());
    body.extend_from_slice(&frame.data);
    incoming(model, &response(0xb0, &body), None, false);
}

#[test]
fn a_passphrase_network_offering_sae_authenticates_by_it_before_the_handshake() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let (_, bss) = inputs_akm(&client, 8);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(
            Arc::new(AtomicUsize::new(0)),
            core::sync::atomic::AtomicBool::new(false),
        ),
    )
    .unwrap();
    let password = b"correct horse battery";
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(password).unwrap(),
        country: None,
    };
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    assert!(!station.carrier());
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    // The scan ends: the station commits instead of open-system
    // authentication.
    done(&model);
    assert!(!station.carrier());
    // Hunting and pecking runs hostapd's 40 rounds, a bounded share a
    // poll; the poll that finds the element sends the commit.
    for _ in 1..40 / crate::wifi::sae::ROUNDS_PER_STEP {
        assert!(!station.carrier());
        assert!(finish_tx(&model).is_empty());
    }
    assert!(!station.carrier());
    let (transaction, status, commit) = sent_sae(&model);
    assert_eq!((transaction, status), (1, 0));
    let mut random = 7u8;
    let mut next = || {
        random = random.wrapping_add(1);
        [random; 32]
    };
    let mut sae = Exchange::new(
        Derivation::HuntingAndPecking,
        password,
        None,
        None,
        AP,
        LOCAL,
        &mut next,
    )
    .unwrap();
    let confirm = sae.receive(1, 0, &commit).unwrap().unwrap();
    station.carrier();
    // The AP's commit draws the station's confirm; its confirm then
    // authenticates the station, which associates.
    sae_from_ap(&model, &sae.commit());
    assert!(!station.carrier());
    let (transaction, status, own_confirm) = sent_sae(&model);
    assert_eq!((transaction, status), (2, 0));
    assert_eq!(sae.receive(2, 0, &own_confirm).unwrap(), None);
    assert!(sae.accepted());
    station.carrier();
    sae_from_ap(&model, &confirm);
    assert!(!station.carrier());
    let request = finish_tx(&model).pop().unwrap();
    assert_eq!(request[..2], [0, 0]);
    // The RSN element carries no PMKID, as sae_pmkid_in_assoc=0 leaves.
    let security = Security::select(&bss.elements, true).unwrap();
    let own_rsn = security.element();
    assert!(request.windows(own_rsn.len()).any(|w| w == own_rsn));
    station.carrier();
    // An AP's association response carries no RSN element outside FT.
    let mut association = vec![0x11, 0, 0, 0, 1, 0xc0];
    let mut rest = &bss.elements[..];
    while let [id, length, tail @ ..] = rest {
        let (body, next) = tail.split_at(usize::from(*length));
        if *id != 48 {
            association.extend_from_slice(&[*id, *length]);
            association.extend_from_slice(body);
        }
        rest = next;
    }
    incoming(&model, &response(0x10, &association), None, false);
    assert!(!station.carrier());
    // The 4-way handshake proves the SAE PMK and delivers the IGTK.
    let mut ap = Authenticator::between(security.suites(), sae.pmk().unwrap(), AP, LOCAL, own_rsn);
    eapol(&model, &ap.message_1());
    assert!(!station.carrier());
    ap.message_2(&sent_eapol(&model));
    let rsn = crate::wifi::connection::find(&bss.elements, 48).unwrap();
    let mut whole = vec![48, rsn.len() as u8];
    whole.extend_from_slice(rsn);
    let mut igtk = vec![0xdd, 4 + 8 + 16, 0x00, 0x0f, 0xac, 9, 4, 0];
    igtk.extend_from_slice(&[0; 6]);
    igtk.extend_from_slice(&[0x77; 16]);
    eapol(&model, &ap.message_3_with(&whole, &igtk));
    assert!(!station.carrier());
    ap.signed_reply(&sent_eapol(&model), 0x308);
    // Once message 4 drained, the IGTK installs and the port opens.
    assert!(station.carrier());
}

#[test]
fn a_protected_psk_association_opens_its_port_once_the_igtk_installs() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let (_, bss) = inputs_akm(&client, 2);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(
            Arc::new(AtomicUsize::new(0)),
            core::sync::atomic::AtomicBool::new(false),
        ),
    )
    .unwrap();
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    let pmk = network.secret.pmk(&network.ssid);
    let security = Security::select(&bss.elements, true).unwrap();
    assert!(security.management.is_some());
    let mut ap = Authenticator::between(security.suites(), pmk, AP, LOCAL, security.element());
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    assert!(!station.carrier());
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    station.carrier();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    station.carrier();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    station.carrier();
    // An AP's association response carries no RSN element outside FT.
    let mut association = vec![0x11, 0, 0, 0, 1, 0xc0];
    let mut rest = &bss.elements[..];
    while let [id, length, tail @ ..] = rest {
        let (body, next) = tail.split_at(usize::from(*length));
        if *id != 48 {
            association.extend_from_slice(&[*id, *length]);
            association.extend_from_slice(body);
        }
        rest = next;
    }
    incoming(&model, &response(0x10, &association), None, false);
    assert!(!station.carrier());
    eapol(&model, &ap.message_1());
    assert!(!station.carrier());
    ap.message_2(&sent_eapol(&model));
    let rsn = crate::wifi::connection::find(&bss.elements, 48).unwrap();
    let mut whole = vec![48, rsn.len() as u8];
    whole.extend_from_slice(rsn);
    let mut igtk = vec![0xdd, 4 + 8 + 16, 0x00, 0x0f, 0xac, 9, 4, 0];
    igtk.extend_from_slice(&[0; 6]);
    igtk.extend_from_slice(&[0x77; 16]);
    eapol(&model, &ap.message_3_with(&whole, &igtk));
    // Message 4 is queued ahead of the IGTK, which installs once it drained;
    // only then does the port open.
    assert!(!station.carrier());
    ap.signed_reply(&sent_eapol(&model), 0x308);
    assert!(station.carrier());
}

#[test]
fn a_scan_listens_where_its_channels_bar_probes_and_finds_the_ap_there() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_passive_five();
    let (_, bss) = modern_inputs(&client);
    let client = ManagedClient::new(client, passive_country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(Arc::new(AtomicUsize::new(0)), Default::default()),
    )
    .unwrap();
    assert!(link.probes(Channel {
        band: Band::Ghz2,
        number: 1,
        frequency_mhz: 2412,
    }));
    assert!(!link.probes(CHANNEL));
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    let mut station = Station::new(
        link,
        profile().interface,
        passive_country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    // The first part probes 2.4 GHz and finds nothing.
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    // The second listens on 5 GHz, where the AP beacons; both parts' BSSes
    // stay known for the choice, which authenticates with the AP.
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&bss.capabilities.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
}

/// A frame the device received on 6 GHz channel 1.
fn incoming_six(model: &Model, frame: &[u8]) {
    let mut raw = incoming_bytes(frame, None, false);
    raw[13] = SIX_CHANNEL;
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
}

#[test]
fn a_reported_six_ghz_ap_is_scanned_and_joined_by_sae_hash_to_element() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_six(0);
    let client = ManagedClient::new(client, six_country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(Arc::new(AtomicUsize::new(0)), Default::default()),
    )
    .unwrap();
    let password = b"correct horse battery";
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(password).unwrap(),
        country: None,
    };
    let mut station = Station::new(
        link,
        profile().interface,
        six_country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    // The 2.4 and 5 GHz part finds another network's AP, whose Reduced
    // Neighbor Report names the colocated AP on 6 GHz channel 1.
    assert!(!station.carrier());
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&1u16.to_le_bytes());
    body.extend_from_slice(&[0, 1, b'y', 1, 1, 0x8c, 201, 17, 0, 13, 131, 1, 0]);
    body.extend_from_slice(&AP.0);
    body.extend_from_slice(&crate::wifi::scan::six_ghz::short_ssid(b"x").to_le_bytes());
    body.extend_from_slice(&[0x40, 0]);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    beacon[10..16].copy_from_slice(&[2, 0, 0, 0, 0, 9]);
    beacon[16..22].copy_from_slice(&[2, 0, 0, 0, 0, 9]);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    // The 6 GHz part cfg80211 plans from it finds the AP beaconing there.
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&beacon_elements());
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming_six(&model, &beacon);
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    // The station commits by hash-to-element on the AP's channel, the poll
    // after it began the exchange.
    assert!(!station.carrier());
    let sent = |model: &Model| {
        let frame = finish_tx_on(model, 1).pop().unwrap();
        assert_eq!(frame[..2], [0xb0, 0]);
        let field = |at: usize| u16::from_le_bytes([frame[at], frame[at + 1]]);
        (field(26), field(28), frame[30..].to_vec())
    };
    let from_ap = |model: &Model, frame: &crate::wifi::sae::Frame| {
        let mut body = vec![3, 0];
        body.extend_from_slice(&frame.transaction.to_le_bytes());
        body.extend_from_slice(&frame.status.to_le_bytes());
        body.extend_from_slice(&frame.data);
        incoming_six(model, &response(0xb0, &body));
    };
    let (transaction, status, commit) = sent(&model);
    assert_eq!((transaction, status), (1, 126));
    let mut random = 7u8;
    let mut next = || {
        random = random.wrapping_add(1);
        [random; 32]
    };
    let template = crate::wifi::sae::Template::derive(b"x", password, None);
    let mut sae = Exchange::new(
        Derivation::HashToElement,
        password,
        Some(&template),
        None,
        AP,
        LOCAL,
        &mut next,
    )
    .unwrap();
    let confirm = sae.receive(1, 126, &commit).unwrap().unwrap();
    station.carrier();
    from_ap(&model, &sae.commit());
    assert!(!station.carrier());
    let (transaction, status, own_confirm) = sent(&model);
    assert_eq!((transaction, status), (2, 0));
    assert_eq!(sae.receive(2, 0, &own_confirm).unwrap(), None);
    station.carrier();
    from_ap(&model, &confirm);
    assert!(!station.carrier());
    // The association carries the RSN element and RSNXE the exchange chose.
    let request = finish_tx_on(&model, 1).pop().unwrap();
    assert_eq!(request[..2], [0, 0]);
    let elements = beacon_elements();
    let security = Security::select_on(crate::wifi::Band::Ghz6, &elements, true).unwrap();
    let own_rsn = security.element();
    let mut selected = own_rsn.clone();
    selected.extend_from_slice(&crate::wifi::sae::H2E_RSNXE);
    assert!(request.windows(own_rsn.len()).any(|w| w == own_rsn));
    assert!(request.windows(3).any(|w| w == crate::wifi::sae::H2E_RSNXE));
    station.carrier();
    let mut association = vec![0x11, 0, 0, 0, 1, 0xc0];
    let mut rest = &elements[..];
    while let [id, length, tail @ ..] = rest {
        let (body, next) = tail.split_at(usize::from(*length));
        if !matches!(*id, 48 | 244) {
            association.extend_from_slice(&[*id, *length]);
            association.extend_from_slice(body);
        }
        rest = next;
    }
    incoming_six(&model, &response(0x10, &association));
    assert!(!station.carrier());
    // The 4-way handshake proves the SAE PMK.
    let mut ap = Authenticator::between(security.suites(), sae.pmk().unwrap(), AP, LOCAL, own_rsn);
    eapol_on(&model, &ap.message_1(), true);
    assert!(!station.carrier());
    ap.message_2(&sent_eapol_on(&model, 1));
    let rsn = crate::wifi::connection::find(&elements, 48).unwrap();
    let mut whole = vec![48, rsn.len() as u8];
    whole.extend_from_slice(rsn);
    let mut igtk = vec![0xdd, 4 + 8 + 16, 0x00, 0x0f, 0xac, 9, 4, 0];
    igtk.extend_from_slice(&[0; 6]);
    igtk.extend_from_slice(&[0x77; 16]);
    eapol_on(&model, &ap.message_3_with(&whole, &igtk), true);
    assert!(!station.carrier());
    ap.signed_reply(&sent_eapol_on(&model, 1), 0x308);
    assert!(station.carrier());
}

/// A UDP datagram in IPv4 between the DHCP ports, with its header checksum:
/// a BOOTP request (`op` 1, DHCPDISCOVER) or reply (`op` 2, DHCPOFFER).
fn dhcp_datagram(source: [u8; 4], destination: [u8; 4], op: u8) -> Vec<u8> {
    // op, htype, hlen, hops, xid, secs, flags (broadcast), four addresses,
    // chaddr, sname, file, magic cookie, message type, end.
    let mut bootp = vec![op, 1, 6, 0, 0x5a, 0x5a, 0x5a, 0x5a, 0, 0, 0x80, 0];
    bootp.extend_from_slice(&[0; 16]);
    bootp.extend_from_slice(&LOCAL.0);
    bootp.extend_from_slice(&[0; 10 + 64 + 128]);
    bootp.extend_from_slice(&[99, 130, 83, 99, 53, 1, op, 255]);
    let (sport, dport) = if op == 1 { (68u16, 67u16) } else { (67, 68) };
    let mut udp = Vec::new();
    udp.extend_from_slice(&sport.to_be_bytes());
    udp.extend_from_slice(&dport.to_be_bytes());
    udp.extend_from_slice(&((8 + bootp.len()) as u16).to_be_bytes());
    udp.extend_from_slice(&[0, 0]);
    udp.extend_from_slice(&bootp);
    let mut ip = vec![0x45, 0, 0, 0, 0, 0, 0, 0, 64, 17, 0, 0];
    ip[2..4].copy_from_slice(&((20 + udp.len()) as u16).to_be_bytes());
    ip.extend_from_slice(&source);
    ip.extend_from_slice(&destination);
    let sum = ip
        .chunks(2)
        .map(|w| u32::from(u16::from_be_bytes([w[0], w[1]])))
        .sum::<u32>();
    let sum = (sum & 0xffff) + (sum >> 16);
    ip[10..12].copy_from_slice(&(!(sum as u16)).to_be_bytes());
    ip.extend_from_slice(&udp);
    ip
}

/// A data frame from the AP that the device decrypted: `datagram` to
/// `destination`, under key `index` at packet number `pn`.
fn decrypted_from_ap(model: &Model, destination: Mac, datagram: &[u8], index: u8, pn: u64) {
    let mut frame = data(destination == Mac::BROADCAST);
    frame.truncate(24);
    frame.extend_from_slice(&[0xaa, 0xaa, 3, 0, 0, 0, 8, 0]);
    frame.extend_from_slice(datagram);
    incoming(model, &frame, Some((1, index, pn)), false);
}

/// A booted model's managed link and its device, as an attach gives them.
fn managed_link() -> (TestLink, Model) {
    let (client, model) = setup_modern();
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(Arc::new(AtomicUsize::new(0)), Default::default()),
    )
    .unwrap();
    (link, model)
}

/// A chip reset that takes `steps` pending steps, then hands over a newly
/// booted model's link.
struct Reset {
    steps: u32,
    addresses: Rc<core::cell::RefCell<Vec<Mac>>>,
    models: Rc<core::cell::RefCell<Vec<Model>>>,
    /// The lost link's device, which the reset stops.
    old: (
        Arc<std::sync::Mutex<crate::wifi::mediatek::mailbox::tests::Response>>,
        Bar,
    ),
}
impl crate::wifi::mediatek::station::Recover<Bar, Q, NativeGuard, Provider> for Reset {
    fn step(
        &mut self,
        address: Mac,
    ) -> core::task::Poll<Result<(TestLink, Context), alloc::string::String>> {
        self.addresses.borrow_mut().push(address);
        if self.steps > 0 {
            self.steps -= 1;
            return core::task::Poll::Pending;
        }
        // The WLAN subsystem reset stops the old device's DMA.
        self.old.0.lock().unwrap().clc_processing = false;
        self.old.1.0.lock().unwrap().values.insert(0xd4208, 0);
        let (link, model) = managed_link();
        self.models.borrow_mut().push(model);
        core::task::Poll::Ready(Ok((link, profile().interface)))
    }
}

/// mt7925_mac_reset_work: a hung firmware loses the link, the chip reset
/// runs a step per poll and the station scans again on the new link.
#[test]
fn a_lost_link_is_reset_and_the_station_scans_again_on_the_new_one() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (link, model) = managed_link();
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    let models = Rc::new(core::cell::RefCell::new(Vec::new()));
    let addresses = Rc::new(core::cell::RefCell::new(Vec::new()));
    let reset = Reset {
        steps: 2,
        addresses: addresses.clone(),
        models: models.clone(),
        old: (model.response.clone(), model.bar.clone()),
    };
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    )
    .with_recovery(Box::new(reset));
    model.response.lock().unwrap().pause_next_index_us =
        crate::wifi::connection::keeper::RUNTIME_COMMAND_TIMEOUT_US + 10_000;
    assert!(!station.carrier());
    // Lost, then two pending steps and the step that hands over the link,
    // one a poll: the station neither errs nor carries meanwhile.
    let mut polls = 0;
    while models.borrow().is_empty() {
        assert!(!station.carrier());
        assert_eq!(station.transmit(&[0; 60]), Err(LinkError::NoCarrier));
        assert!(station.lost().is_none());
        polls += 1;
        assert!(polls <= 3);
    }
    assert_eq!(polls, 3);
    assert_eq!(models.borrow().len(), 1);
    let reported: Vec<_> = core::iter::from_fn(|| station.take_diagnostic()).collect();
    assert!(
        reported
            .iter()
            .any(|r| r.contains("recovered by a chip reset")),
        "{reported:?}"
    );
    // The new link scans: its device takes the scan request on its MCU
    // queue.
    let commands = |models: &Rc<core::cell::RefCell<Vec<Model>>>| {
        models.borrow()[0].queues[&0xd43f0].lock().unwrap().head
    };
    let before = commands(&models);
    assert!(!station.carrier());
    assert_ne!(commands(&models), before);
    // The reset keeps the station's address.
    assert_eq!(addresses.borrow().len(), 3);
    assert!(addresses.borrow().iter().all(|address| *address == LOCAL));
}

#[test]
fn a_runtime_command_answered_past_the_bound_loses_the_radio_once() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(Arc::new(AtomicUsize::new(0)), Default::default()),
    )
    .unwrap();
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    // The firmware answers the first poll's scan request 60 ms late, which
    // the attach's 3 s would wait out: the poll ends the command instead.
    model.response.lock().unwrap().pause_next_index_us =
        crate::wifi::connection::keeper::RUNTIME_COMMAND_TIMEOUT_US + 10_000;
    let start = crate::Clock::ticks();
    assert!(!station.carrier());
    assert!(crate::Clock::ticks() - start < 100_000);
    // The next poll finds the radio lost and reports why once.
    assert!(!station.carrier());
    let Some(reason) = station.lost().map(alloc::string::String::from) else {
        let reported: Vec<_> = core::iter::from_fn(|| station.take_diagnostic()).collect();
        panic!("not lost: {reported:?}");
    };
    // The scan's own report names the command timeout before it.
    let reported: Vec<_> = core::iter::from_fn(|| station.take_diagnostic()).collect();
    assert_eq!(reported.last(), Some(&reason));
    assert_eq!(reported.iter().filter(|r| **r == reason).count(), 1);
    assert!(
        reported.iter().any(|r| r.contains("Timeout")),
        "{reported:?}"
    );
    for _ in 0..3 {
        assert!(!station.carrier());
    }
    assert_eq!(station.take_diagnostic(), None);
}
#[test]
fn an_attached_mt7925_carries_dhcp_under_the_handshake_keys_through_the_link() {
    if crate::test_process::isolate_model() {
        return;
    }
    // The booted model's controller, then the keeper's scan, join and
    // 4-way handshake, as the boot path runs them.
    let (client, model) = setup_modern();
    let (_, bss) = modern_inputs(&client);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let link = ManagedLink::new(
        client,
        Provider(Arc::new(AtomicUsize::new(0)), Default::default()),
    )
    .unwrap();
    let network = Network {
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    let pmk = network.secret.pmk(&network.ssid);
    let security = Security::select(&bss.elements, true).unwrap();
    let mut ap = Authenticator::between(security.suites(), pmk, AP, LOCAL, security.element());
    let mut station = Station::new(
        link,
        profile().interface,
        country(),
        network,
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    assert!(!station.carrier());
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&bss.capabilities.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    assert!(!station.carrier());
    done(&model);
    assert!(!station.carrier());
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    station.carrier();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    station.carrier();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    station.carrier();
    let mut association = vec![0x11, 0, 0, 0, 1, 0xc0];
    association.extend_from_slice(&bss.elements);
    incoming(&model, &response(0x10, &association), None, false);
    assert!(!station.carrier());
    // The stack waits for a station's scan, join and handshake.
    assert_eq!(
        station.carrier_wait_us(),
        crate::wifi::connection::keeper::CARRIER_WAIT_US
    );
    assert_eq!(station.carrier_wait_us(), 45_000_000);
    // Before the handshake the port is closed to data.
    let discover = dhcp_datagram([0; 4], [255; 4], 1);
    let mut ethernet = Vec::new();
    ethernet.extend_from_slice(&Mac::BROADCAST.0);
    ethernet.extend_from_slice(&LOCAL.0);
    ethernet.extend_from_slice(&[8, 0]);
    ethernet.extend_from_slice(&discover);
    assert!(station.transmit(&ethernet).is_err());
    eapol(&model, &ap.message_1());
    assert!(!station.carrier());
    ap.message_2(&sent_eapol(&model));
    let rsn = crate::wifi::connection::find(&bss.elements, 48).unwrap();
    let mut whole = vec![48, rsn.len() as u8];
    whole.extend_from_slice(rsn);
    eapol(&model, &ap.message_3(&whole));
    assert!(station.carrier());
    ap.signed_reply(&sent_eapol(&model), 0x308);
    // DHCPDISCOVER leaves as a protected data frame to the AP, which the
    // device encrypts under the installed pairwise key.
    station.transmit(&ethernet).unwrap();
    let sent = finish_tx(&model).pop().unwrap();
    // QoS data to the WMM AP, to DS, protected: BSSID, own address,
    // broadcast destination, then best effort.
    assert_eq!(sent[..2], [0x88, 0x41]);
    assert_eq!(sent[4..10], AP.0);
    assert_eq!(sent[10..16], LOCAL.0);
    assert_eq!(sent[16..22], Mac::BROADCAST.0);
    assert_eq!(sent[24] & 0x0f, 0);
    assert_eq!(sent[26..34], [0xaa, 0xaa, 3, 0, 0, 0, 8, 0]);
    assert_eq!(sent[34..], discover[..]);
    // DHCPOFFER to the station under the pairwise key and a broadcast one
    // under the group key, each out of the link as Ethernet.
    const SERVER: [u8; 6] = [4, 1, 2, 3, 4, 5];
    let offer = dhcp_datagram([192, 0, 2, 1], [192, 0, 2, 10], 2);
    decrypted_from_ap(&model, LOCAL, &offer, 0, 1);
    let broadcast = dhcp_datagram([192, 0, 2, 1], [255; 4], 2);
    // Message 3 gave GTK 1 at receive counter 7.
    decrypted_from_ap(&model, Mac::BROADCAST, &broadcast, 1, 8);
    let mut received = Vec::new();
    for _ in 0..8 {
        if let Some(frame) = station.receive().unwrap() {
            received.push(frame);
        }
    }
    assert_eq!(received.len(), 2);
    for (frame, (destination, datagram)) in received
        .iter()
        .zip([(LOCAL, &offer), (Mac::BROADCAST, &broadcast)])
    {
        assert_eq!(frame[..6], destination.0);
        assert_eq!(frame[6..12], SERVER);
        assert_eq!(frame[12..14], [8, 0]);
        assert_eq!(frame[14..], datagram[..]);
    }
    // A replayed DHCPOFFER is dropped, as ieee80211_crypto_ccmp_decrypt
    // drops it, and the link stays up for the next one.
    decrypted_from_ap(&model, LOCAL, &offer, 0, 1);
    decrypted_from_ap(&model, LOCAL, &offer, 0, 2);
    let mut received = Vec::new();
    for _ in 0..6 {
        if let Some(frame) = station.receive().unwrap() {
            received.push(frame);
        }
    }
    assert_eq!(received.len(), 1);
    assert_eq!(received[0][14..], offer[..]);
    assert_eq!(station.radio_mut().link_mut().rejected_frames(), 1);
    assert!(station.carrier());
    // No diagnostic so far became an error.
    while station.take_diagnostic().is_some() {}
    // A receive buffer the device cannot get back quarantines the adapter
    // for good: the keeper reports why once and stops retrying.
    model.queues[&0xd4520].lock().unwrap().fail_publish = true;
    decrypted_from_ap(&model, LOCAL, &offer, 0, 3);
    let error = station.receive().unwrap_err();
    let reason = alloc::string::String::from(station.lost().expect("the radio is lost"));
    assert_eq!(error, LinkError::Adapter(reason.clone()));
    assert_eq!(station.take_diagnostic(), Some(reason.clone()));
    let tx = model.queues[&0xd4300].lock().unwrap().head;
    for _ in 0..4 {
        assert!(!station.carrier());
        assert_eq!(
            station.receive().unwrap_err(),
            LinkError::Adapter(reason.clone())
        );
        assert!(station.transmit(&ethernet).is_err());
    }
    assert_eq!(station.take_diagnostic(), None);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, tx);
}
