// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! A PMF association through the sole owning link, driven as wpa_supplicant
//! drives mac80211: join, EAPOL, keys, then the authorized port.
use super::*;
use crate::wifi::mediatek::bootstrap::client::modern::tests::pmf_controls::{inputs, inputs_akm};
use crate::wifi::security::PacketNumber;
use crate::wifi::station::{
    self,
    multi_link::{Affiliation, Mld, Setup},
    wire::{
        AssociationRequest,
        multi_link::{self, Basic},
    },
};

#[test]
fn a_pmf_association_opens_its_port_after_the_handshake_keys() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let (join, bss) = inputs(&client);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let mut link = ManagedLink::new(
        client,
        Provider(
            Arc::new(AtomicUsize::new(0)),
            core::sync::atomic::AtomicBool::new(false),
        ),
    )
    .unwrap();
    associate(&mut link, &model, join, bss);
    assert!(link.station().requires_protected_management());
    let epoch = link.station().association_epoch().unwrap();
    // The 4-way handshake's EAPOL needs no key.
    link.send_control(&control(), None).unwrap();
    assert_eq!(finish_tx(&model)[0][1] & 0x40, 0);
    link.service().unwrap();
    assert!(link.authorize_port(epoch).is_err());
    let ptk = link
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 2, &[0x44; 16], ReplaySeed::ZERO)
        .unwrap();
    // The port also needs the IGTK.
    assert!(link.authorize_port(epoch).is_err());
    drop(drain_notices(&mut link));
    // SAFETY: modeled queues, notices and old key users are drained, and the
    // model OS's handshake supplied this IGTK.
    unsafe { link.install_management_key(epoch, 4, &[0x55; 16], PacketNumber::new(100).unwrap()) }
        .unwrap();
    assert!(!link.carrier());
    link.authorize_port(epoch).unwrap();
    assert!(link.carrier());
    link.send_control(&control(), Some(ptk)).unwrap();
    link.transmit(&eth(0x800)).unwrap();
    let frames = finish_tx(&model);
    assert_eq!(frames.len(), 2);
    assert!(frames.iter().all(|frame| frame[1] & 0x40 != 0));
    link.service().unwrap();
    incoming(&model, &data(false), Some((1, 0, 1)), false);
    link.service().unwrap();
    assert_eq!(
        &link.receive().unwrap().unwrap()[12..14],
        &0x800u16.to_be_bytes()
    );
    // An authenticated SA Query response reaches the OS that registered it.
    link.register_action(&[8, 1]).unwrap();
    incoming(
        &model,
        &response(0x40d0, &[8, 1, 3, 4]),
        Some((1, 0, 2)),
        false,
    );
    link.service().unwrap();
    assert!(drain_notices(&mut link).iter().any(|notice| matches!(
        &notice.event,
        LinkEvent::Controller(Event::ProtectedManagement(rx)) if rx.body() == [8, 1, 3, 4]
    )));
    // mac80211 answers the AP's SA Query request and returns an unknown
    // robust action, both protected.
    incoming(
        &model,
        &response(0x40d0, &[8, 0, 5, 6]),
        Some((1, 0, 3)),
        false,
    );
    incoming(
        &model,
        &response(0x40d0, &[126, 1, 2]),
        Some((1, 0, 4)),
        false,
    );
    let mut replies = Vec::new();
    for _ in 0..4 {
        link.service().unwrap();
        replies.extend(finish_tx(&model));
    }
    drop(drain_notices(&mut link));
    for body in [&[8, 1, 5, 6][..], &[0xfe, 1, 2]] {
        assert!(
            replies
                .iter()
                .any(|frame| frame[..2] == [0xd0, 0x40] && &frame[24..] == body)
        );
    }
    // The QoS data frame started its TID's block-ack session with a
    // protected ADDBA request, and the departure is encrypted too, as
    // IEEE80211_STA_MFP_ENABLED leaves it.
    let departed = depart(&mut link, &model);
    assert!(
        replies
            .iter()
            .chain(&departed)
            .any(|frame| frame[..2] == [0xd0, 0x40] && frame[24..26] == [3, 0])
    );
    assert_eq!(departed.last().unwrap()[..2], [0xc0, 0x40]);
    retire(&mut link, &model);
}

#[test]
fn an_sae_exchange_associates_like_ieee80211_mgd_assoc() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let (join, bss) = inputs_akm(&client, 8);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let mut link = ManagedLink::new(
        client,
        Provider(
            Arc::new(AtomicUsize::new(0)),
            core::sync::atomic::AtomicBool::new(false),
        ),
    )
    .unwrap();
    link.begin_scan(
        scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
        1_000_000,
    )
    .unwrap();
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    link.service().unwrap();
    let handle = link.bss().next().unwrap().0;
    done(&model);
    link.service().unwrap();
    // SAFETY: complete source-model terminal scan and old-user drain.
    unsafe { link.retire_scan_after_drain() }.unwrap();
    // An SAE request cannot authenticate with open system.
    assert!(link.join(handle, join.clone(), profile()).is_err());
    let session = link
        .begin_external_authentication(handle, profile(), None, 2_000_000)
        .unwrap();
    // ieee80211_mgd_assoc refuses while authentication is unfinished.
    assert!(
        link.associate_external(&session, handle, join.clone())
            .is_err()
    );
    for (transaction, data) in [(1, &[19, 0][..]), (2, &[1][..])] {
        link.transmit_external_authentication(
            &session,
            crate::wifi::station::external::Frame {
                transaction,
                status: 0,
                data,
            },
        )
        .unwrap();
        finish_tx(&model);
        link.service().unwrap();
        let mut frame = vec![3, 0, transaction as u8, 0, 0, 0];
        frame.extend_from_slice(data);
        incoming(&model, &response(0xb0, &frame), None, false);
        link.service().unwrap();
    }
    assert!(link.station().external_authenticated());
    drop(drain_notices(&mut link));
    link.associate_external(&session, handle, join).unwrap();
    let request = finish_tx(&model);
    assert_eq!(request[0][..2], [0, 0]);
    link.service().unwrap();
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&bss.elements);
    incoming(&model, &response(0x10, &assoc), None, false);
    link.service().unwrap();
    assert_eq!(link.phase(), Phase::AwaitingModernAdmission);
    assert!(link.station().external_authentication().is_none());
    link.configure().unwrap();
    let epoch = link.station().association_epoch().unwrap();
    assert_eq!(epoch, session.epoch());
    link.install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 2, &[0x44; 16], ReplaySeed::ZERO)
        .unwrap();
    drop(drain_notices(&mut link));
    // SAFETY: modeled queues, notices and old key users are drained, and the
    // model OS's handshake supplied this IGTK.
    unsafe { link.install_management_key(epoch, 4, &[0x55; 16], PacketNumber::new(100).unwrap()) }
        .unwrap();
    link.authorize_port(epoch).unwrap();
    assert!(link.carrier());
    stop(&mut link, &model);
}

const AP_MLD: Mac = Mac([2, 9, 9, 9, 9, 9]);
const SETUP_LINK: Mac = Mac([2, 0xa, 0, 0, 0, 1]);
/// The AP and the station's address on the AP MLD's link 1.
const OTHER: Mac = Mac([2, 6, 7, 8, 9, 11]);
const OTHER_LINK: Mac = Mac([2, 0xa, 0, 0, 0, 2]);
const CHANNEL_52: wifi::Channel = wifi::Channel {
    band: Band::Ghz5,
    number: 52,
    frequency_mhz: 5260,
};

/// A link whose SAE with the AP MLD on its setup link, link 0, succeeded
/// and whose association, requesting the links `links` takes from the
/// setup link's BSS and request and answered with the profiles it gives,
/// awaits its configuration.
fn mld_association(
    links: impl FnOnce(
        &scan::result::Bss,
        &AssociationRequest,
    ) -> (Vec<station::multi_link::Link>, Vec<multi_link::Profile>),
) -> (TestLink, Model, scan::result::Bss) {
    mld_association_with_cipher(6, links)
}

fn mld_association_with_cipher(
    cipher: u8,
    links: impl FnOnce(
        &scan::result::Bss,
        &AssociationRequest,
    ) -> (Vec<station::multi_link::Link>, Vec<multi_link::Profile>),
) -> (TestLink, Model, scan::result::Bss) {
    let to_link = |subtype, body: &[u8]| {
        let mut frame = response(subtype, body);
        frame[4..10].copy_from_slice(&SETUP_LINK.0);
        frame
    };
    let affiliation = Affiliation {
        mld: Mld {
            local: LOCAL,
            ap: AP_MLD,
        },
        link_id: 0,
        local: SETUP_LINK,
    };
    let (client, model) = setup_modern();
    let (mut join, bss) =
        crate::wifi::mediatek::bootstrap::client::modern::tests::pmf_controls::inputs_akm_cipher(
            &client, 8, cipher,
        );
    let (requested, profiles) = links(&bss, &join.request);
    join.request = join
        .request
        .with_multi_link(Setup {
            association: affiliation,
            mld_capabilities: 0,
            eml_capabilities: 0,
            links: requested,
        })
        .unwrap();
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let mut link = ManagedLink::new(
        client,
        Provider(
            Arc::new(AtomicUsize::new(0)),
            core::sync::atomic::AtomicBool::new(false),
        ),
    )
    .unwrap();
    link.begin_scan(
        scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
        1_000_000,
    )
    .unwrap();
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    link.service().unwrap();
    let handle = link.bss().next().unwrap().0;
    done(&model);
    link.service().unwrap();
    // SAFETY: complete source-model terminal scan and old-user drain.
    unsafe { link.retire_scan_after_drain() }.unwrap();
    let session = link
        .begin_external_authentication(handle, profile(), Some(affiliation), 2_000_000)
        .unwrap();
    for (transaction, status, data) in [(1, 126, &[19, 0][..]), (2, 0, &[1][..])] {
        link.transmit_external_authentication(
            &session,
            crate::wifi::station::external::Frame {
                transaction,
                status,
                data,
            },
        )
        .unwrap();
        finish_tx(&model);
        link.service().unwrap();
        let mut frame = vec![3, 0, transaction as u8, 0, status as u8, 0];
        frame.extend_from_slice(data);
        frame.extend(
            Basic {
                address: AP_MLD,
                ..Basic::default()
            }
            .encode(),
        );
        incoming(&model, &to_link(0xb0, &frame), None, false);
        link.service().unwrap();
    }
    assert!(link.station().external_authenticated());
    drop(drain_notices(&mut link));
    link.associate_external(&session, handle, join).unwrap();
    let request = &finish_tx(&model)[0];
    assert_eq!(request[10..16], SETUP_LINK.0);
    link.service().unwrap();
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&bss.elements);
    assoc.extend(
        Basic {
            address: AP_MLD,
            link_id: Some(0),
            mld_capabilities: Some(0),
            profiles,
            ..Basic::default()
        }
        .encode(),
    );
    incoming(&model, &to_link(0x10, &assoc), None, false);
    link.service().unwrap();
    assert_eq!(link.phase(), Phase::AwaitingModernAdmission);
    (link, model, bss)
}

/// Every `tag` record of the `command` packets of `commands`, after a
/// `header`-byte command header.
fn records(commands: &[Vec<u8>], command: u8, header: usize, tag: u16) -> Vec<Vec<u8>> {
    commands
        .iter()
        .filter(|packet| packet[34] == command)
        .filter_map(|packet| {
            let mut rest = &packet[48 + header..];
            while rest.len() >= 4 {
                let length = usize::from(u16::from_le_bytes([rest[2], rest[3]]));
                if rest[..2] == tag.to_le_bytes() {
                    return Some(rest[..length].to_vec());
                }
                rest = &rest[length..];
            }
            None
        })
        .collect()
}

/// An AP MLD's association on its setup link alone, as mac80211 makes one
/// when no other link is requested: SAE and the association run on the
/// link's address, the BSS and AP records name both MLDs, as
/// `mt7925_mcu_bss_mld_tlv` and `mt7925_mcu_sta_mld_tlv` write them, and the
/// MLDs' PTK opens the port.
#[test]
fn an_ap_mld_association_names_both_mlds_in_its_records() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, _) = mld_association(|_, _| (Vec::new(), Vec::new()));
    let before = model.response.lock().unwrap().client_commands.len();
    link.configure().unwrap();
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    // UNI_BSS_INFO_MLD: link 0 of the group of BSS 0, own MLD 32, the
    // station MLD's address.
    let bss = &records(&commands, 2, 4, 26)[0];
    assert_eq!(
        (bss[4], bss[5], &bss[6..12], bss[13]),
        (0, 32, &LOCAL.0[..], 0)
    );
    // STA_REC_MLD names the AP MLD with the setup link's WCID 1 alone.
    let peer = &records(&commands, 3, 8, 0x20)[0];
    assert_eq!((&peer[4..10], peer[16]), (&AP_MLD.0[..], 1));
    // mt792x_tx: a management frame the station addresses to the AP MLD
    // leaves the setup link from the station's address there.
    let mut action = vec![0xd0, 0, 0, 0];
    for address in [AP_MLD, LOCAL, AP_MLD] {
        action.extend_from_slice(&address.0);
    }
    action.extend_from_slice(&[0, 0, 3, 0]);
    let context = link.station().association_context().unwrap();
    let on_air = Client::<Bar, Q, NativeGuard>::on_setup_link(&action, context);
    assert_eq!(
        (&on_air[4..10], &on_air[10..16], &on_air[16..22]),
        (&AP.0[..], &SETUP_LINK.0[..], &AP.0[..])
    );
    // The PTK is the MLDs', the setup link's GTK its AP's: the port opens
    // with the keys the handshake gives.
    let epoch = link.station().association_epoch().unwrap();
    link.install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 2, &[0x44; 16], ReplaySeed::ZERO)
        .unwrap();
    drop(drain_notices(&mut link));
    // SAFETY: modeled queues, notices and old key users are drained, and the
    // model OS's handshake supplied this IGTK.
    unsafe { link.install_management_key(epoch, 4, &[0x55; 16], PacketNumber::new(100).unwrap()) }
        .unwrap();
    link.authorize_port(epoch).unwrap();
    assert!(link.carrier());
    let sent = depart(&mut link, &model);
    assert!(
        !sent.is_empty()
            && sent
                .iter()
                .all(|frame| frame[4..10] == AP.0 && frame[10..16] == SETUP_LINK.0)
    );
    retire(&mut link, &model);
}

/// An AP MLD's association adding link 1, as mt7925 adds a link: its
/// interface registers on the next BSS slot under its own address, one MLO
/// channel request takes both links' channels, and its BSS and AP records
/// name both MLDs with the setup link primary. The MLDs' PTK goes to both
/// APs' WCIDs, each link's GTK to its own, and the port opens with both
/// pairs; the link's records and interface leave with the association.
#[test]
fn an_ap_mld_association_adds_its_other_link_on_an_interface_of_its_own() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, _) = two_link_association();
    let before = model.response.lock().unwrap().client_commands.len();
    link.configure().unwrap();
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    // The link's DEV_INFO: band chosen by the firmware, link index 1, the
    // station's link address.
    let [device] = &commands
        .iter()
        .filter(|packet| packet[34] == 1)
        .collect::<Vec<_>>()[..]
    else {
        panic!("one interface registered");
    };
    assert_eq!(
        (device[49], device[56], device[57], &device[58..64]),
        (0xff, 1, 1, &OTHER_LINK.0[..])
    );
    // MT7925_ROC_REQ_MLSR_AA for BSS 0 on channel 36 and BSS 1 on 52.
    let roc = model
        .response
        .lock()
        .unwrap()
        .roc_requests
        .last()
        .unwrap()
        .clone();
    assert_eq!(
        (roc[8], roc[10], roc[19], roc[28], roc[32], roc[34]),
        (0, 36, 11, 3, 1, 52)
    );
    // UNI_BSS_INFO_MLD: link 1 of the group of BSS 0, own MLD 33.
    let bss = records(&commands, 2, 4, 26);
    assert_eq!(
        bss.iter()
            .map(|m| (m[4], m[5], &m[6..12], m[13]))
            .collect::<Vec<_>>(),
        [(0, 32, &LOCAL.0[..], 0), (0, 33, &LOCAL.0[..], 1)]
    );
    // STA_REC_MLD: the setup link's AP alone, again, then link 1's AP,
    // WCID 2, beside it.
    let peers = records(&commands, 3, 8, 0x20);
    let [setup, again, other] = &peers[..] else {
        panic!("three AP records");
    };
    assert_eq!(setup, again);
    assert_eq!(
        (
            &other[4..10],
            &other[10..12],
            &other[12..14],
            other[16],
            other[22]
        ),
        (&AP_MLD.0[..], &[1, 0][..], &[2, 0][..], 2, 0)
    );
    let epoch = link.station().association_epoch().unwrap();
    link.install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 2, &[0x44; 16], ReplaySeed::ZERO)
        .unwrap();
    drop(drain_notices(&mut link));
    // SAFETY: modeled queues, notices and old key users are drained, and the
    // model OS's handshake supplied this IGTK.
    unsafe { link.install_management_key(epoch, 4, &[0x55; 16], PacketNumber::new(100).unwrap()) }
        .unwrap();
    // Link 1's GTK is still missing.
    assert!(link.authorize_port(epoch).is_err());
    link.install_link_key(1, 1, &[0x46; 16], ReplaySeed::ZERO)
        .unwrap();
    {
        let model = model.response.lock().unwrap();
        assert_eq!(
            (model.keys.get(&1), model.keys.get(&2)),
            (Some(&vec![1; 16]), Some(&vec![1; 16]))
        );
        assert_eq!(model.keys.get(&18), Some(&vec![0x46; 16]));
    }
    link.authorize_port(epoch).unwrap();
    assert!(link.carrier());
    // Link 1's beacons reach the OS intact; unfinished channel tracking
    // is reported once. An unprotected robust action goes nowhere.
    let mut beacon = response(0x80, &[0; 12]);
    beacon.extend_from_slice(
        &link.state().client.client.state().links[0]
            .context
            .beacon()
            .elements,
    );
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    let mut action = response(0xd0, &[3, 0]);
    action[4..10].copy_from_slice(&OTHER_LINK.0);
    for frame in [&beacon, &action, &beacon] {
        let mut frame = frame.clone();
        frame[10..16].copy_from_slice(&OTHER.0);
        frame[16..22].copy_from_slice(&OTHER.0);
        let mut bytes = incoming_bytes(&frame, None, false);
        bytes[13] = 52;
        device_receive(&model.queues[&0xd4520], &bytes, true, 0);
    }
    link.service().unwrap();
    let reported = drain_notices(&mut link)
        .into_iter()
        .filter_map(|notice| match notice.event {
            LinkEvent::Controller(Event::Unsupported(operation)) => Some(operation),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(reported, ["MLO link beacon channel switches"]);
    assert!(link.carrier());
    // The AP MLD's protected ADDBA request for TID 0 on the setup link starts
    // a session on both links' AP WCIDs, as mt7925_mcu_uni_rx_ba programs it.
    let before = model.response.lock().unwrap().client_commands.len();
    let mut addba = response(0x40d0, &[3, 0, 1, 0x02, 0x10, 0, 0, 0x10, 0]);
    addba[4..10].copy_from_slice(&SETUP_LINK.0);
    incoming(&model, &addba, Some((1, 0, 1)), false);
    for _ in 0..4 {
        link.service().unwrap();
        finish_tx(&model);
    }
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    let sessions = records(&commands, 3, 8, 6);
    assert_eq!(sessions.len(), 2);
    let wcids: Vec<_> = commands
        .iter()
        .filter(|packet| packet[34] == 3 && packet.get(56..58) == Some(&[6, 0]))
        .map(|packet| (packet[48], packet[49]))
        .collect();
    assert_eq!(wcids, [(0, 1), (1, 2)]);
    // A request for TID 1 on link 1, under the MLDs' PTK as link 1 holds it,
    // does the same.
    let before = model.response.lock().unwrap().client_commands.len();
    let mut addba = response(0x40d0, &[3, 0, 2, 0x06, 0x10, 0, 0, 0x10, 0]);
    addba[4..10].copy_from_slice(&OTHER_LINK.0);
    addba[10..16].copy_from_slice(&OTHER.0);
    addba[16..22].copy_from_slice(&OTHER.0);
    let mut bytes = incoming_bytes(&addba, Some((2, 0, 2)), false);
    bytes[13] = 52;
    device_receive(&model.queues[&0xd4520], &bytes, true, 0);
    for _ in 0..4 {
        link.service().unwrap();
        finish_tx(&model);
    }
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    assert_eq!(records(&commands, 3, 8, 6).len(), 2);
    let before = model.response.lock().unwrap().client_commands.len();
    depart(&mut link, &model);
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    // The link's interface deactivates: its BSS, then its device.
    assert!(
        commands
            .iter()
            .any(|packet| packet[34] == 1 && packet[57] == 1 && packet[56] == 0)
    );
    assert!(!model.response.lock().unwrap().keys.contains_key(&2));
    retire(&mut link, &model);
}

fn two_link_association() -> (TestLink, Model, scan::result::Bss) {
    two_link_association_with_cipher(6)
}

fn two_link_association_with_cipher(cipher: u8) -> (TestLink, Model, scan::result::Bss) {
    mld_association_with_cipher(cipher, |bss, request| {
        let mut elements = bss.elements.clone();
        let mut rest = &mut elements[..];
        while !rest.is_empty() {
            let n = usize::from(rest[1]);
            let (element, next) = rest.split_at_mut(2 + n);
            match (element[0], element.get(2)) {
                (61, _) => element[2] = 52,
                (192, _) => element[3] = 58,
                (255, Some(106)) => element[9] = 58,
                _ => {}
            }
            rest = next;
        }
        let observation = Arc::new(station::NativeBssObservation {
            bss: scan::result::Bss {
                bssid: OTHER,
                transmitter: OTHER,
                profile: None,
                channel: CHANNEL_52,
                elements: elements.clone(),
                ..bss.clone()
            },
        });
        let mut answer = vec![0x11, 0, 0, 0];
        answer.extend_from_slice(&elements);
        (
            vec![station::multi_link::Link {
                link_id: 1,
                local: OTHER_LINK,
                observation,
                request: request.clone(),
            }],
            vec![multi_link::Profile {
                link_id: 1,
                complete: true,
                address: Some(OTHER),
                bss_parameters_change_count: Some(0),
                body: answer,
                ..multi_link::Profile::default()
            }],
        )
    })
}

#[test]
fn protected_deauthentication_on_second_link_ends_association() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();

    let mut deauth = response(0x40c0, &[7, 0]);
    deauth[4..10].copy_from_slice(&OTHER_LINK.0);
    deauth[10..16].copy_from_slice(&OTHER.0);
    deauth[16..22].copy_from_slice(&OTHER.0);
    let mut bytes = incoming_bytes(&deauth, Some((2, 0, 1)), false);
    bytes[13] = 52;
    device_receive(&model.queues[&0xd4520], &bytes, true, 0);
    link.service().unwrap();
    assert!(!link.carrier());
    assert_eq!(link.station().phase(), station::Phase::Disconnecting);
    assert!(drain_notices(&mut link).iter().any(|notice| matches!(
        &notice.event,
        LinkEvent::Controller(Event::ProtectedManagement(rx))
            if rx.frame().header().control & 0x00fc == 0x00c0 && rx.body() == [7, 0]
    )));
    for _ in 0..4 {
        finish_tx(&model);
        link.service().unwrap();
    }
    retire(&mut link, &model);
}

fn authorized_two_link_association() -> (TestLink, Model) {
    let (mut link, model) = keyed_two_link_association();
    link.authorize_port(link.station().association_epoch().unwrap())
        .unwrap();
    assert!(link.carrier());
    (link, model)
}

fn keyed_two_link_association() -> (TestLink, Model) {
    let (mut link, model, _) = two_link_association();
    link.configure().unwrap();
    let epoch = link.station().association_epoch().unwrap();
    link.install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 2, &[0x44; 16], ReplaySeed::ZERO)
        .unwrap();
    link.install_link_key(1, 1, &[0x46; 16], ReplaySeed::ZERO)
        .unwrap();
    drop(drain_notices(&mut link));
    // SAFETY: this model owns the handshake keys; all prior key users and
    // notices have drained before the IGTK installation.
    unsafe { link.install_management_key(epoch, 4, &[0x55; 16], PacketNumber::new(100).unwrap()) }
        .unwrap();
    (link, model)
}

fn other_link_beacon(link: &TestLink, count: u8) -> Vec<u8> {
    let mut elements = link.state().client.client.state().links[0]
        .context
        .beacon()
        .elements
        .clone();
    let mut at = 0;
    while at < elements.len() {
        let n = usize::from(elements[at + 1]);
        if elements[at] == 221 && elements[at + 2..at + 6] == [0, 0x50, 0xf2, 2] {
            elements[at + 8] = (elements[at + 8] & 0xf0) | count;
            elements[at + 12..at + 14].copy_from_slice(&100u16.to_le_bytes());
            break;
        }
        at += 2 + n;
    }
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&elements);
    let mut frame = response(0x80, &body);
    frame[4..10].copy_from_slice(&Mac::BROADCAST.0);
    frame[10..16].copy_from_slice(&OTHER.0);
    frame[16..22].copy_from_slice(&OTHER.0);
    let mut bytes = incoming_bytes(&frame, None, false);
    bytes[13] = 52;
    bytes
}

#[test]
fn second_link_beacon_updates_only_its_edca() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    let setup = *link.applied_edca().unwrap();
    let epoch = link.station().association_epoch();
    let before = model.response.lock().unwrap().client_commands.len();
    let beacon = other_link_beacon(&link, 1);
    device_receive(&model.queues[&0xd4520], &beacon, true, 0);
    link.service().unwrap();
    assert!(!link.pending_edca());
    let state = link.state().client.client.state();
    let applied = state.links[0].applied_edca.unwrap();
    assert_eq!(applied.parameter_set_count, 1);
    assert_eq!(link.applied_edca(), Some(&setup));
    assert_eq!(link.station().association_epoch(), epoch);
    assert!(link.carrier());
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    let updates = commands
        .iter()
        .filter(|packet| packet[34] == 4)
        .collect::<Vec<_>>();
    assert_eq!(updates.len(), 1);
    assert_eq!(updates[0][48], 1);
    // Equality is the firmware/mac80211 rule, including the count's wrap.
    device_receive(&model.queues[&0xd4520], &beacon, true, 0);
    link.service().unwrap();
    assert_eq!(
        model.response.lock().unwrap().client_commands[before..]
            .iter()
            .filter(|packet| packet[34] == 4)
            .count(),
        1
    );
    for count in [15, 0] {
        let beacon = other_link_beacon(&link, count);
        device_receive(&model.queues[&0xd4520], &beacon, true, 0);
        link.service().unwrap();
        assert_eq!(
            link.state().client.client.state().links[0]
                .applied_edca
                .unwrap()
                .parameter_set_count,
            count
        );
    }
    depart(&mut link, &model);
    retire(&mut link, &model);
}

#[test]
fn second_link_edca_failure_quarantines_without_claiming_application() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    let applied = link.state().client.client.state().links[0].applied_edca;
    let beacon = other_link_beacon(&link, 1);
    model.response.lock().unwrap().edca_status = Some(0xbad);
    device_receive(&model.queues[&0xd4520], &beacon, true, 0);
    assert!(link.service().is_err());
    assert!(!link.carrier());
    assert_eq!(link.phase(), Phase::Quarantined);
    assert!(link.pending_edca());
    assert_eq!(
        link.state().client.client.state().links[0].applied_edca,
        applied
    );
    drop(link);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn second_link_beacon_narrows_only_its_channel() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::wifi::station::phy::Width;
    let (mut link, model) = authorized_two_link_association();
    let setup = link.state().client.client.state().channel;
    assert_ne!(
        link.state().client.client.state().links[0].channel.width,
        Width::Mhz20
    );
    let before = model.response.lock().unwrap().client_commands.len();
    let mut beacon = other_link_beacon(&link, 0);
    let mut at = 32 + 36;
    while at + 2 < beacon.len() {
        let n = usize::from(beacon[at + 1]);
        match beacon[at] {
            61 => beacon[at + 3] &= !7,
            192 => beacon[at + 2] = 0,
            255 if beacon[at + 2] == 106 => beacon[at + 8..at + 10].copy_from_slice(&[0, 52]),
            _ => {}
        }
        at += 2 + n;
    }
    device_receive(&model.queues[&0xd4520], &beacon, true, 0);
    link.service().unwrap();
    let state = link.state().client.client.state();
    assert_eq!(state.channel, setup);
    assert_eq!(state.links[0].channel.width, Width::Mhz20);
    assert_eq!(state.links[0].channel.control, CHANNEL_52);
    assert_eq!(link.state().client.client.pending_channel(), None);
    assert!(link.carrier());
    let commands = model.response.lock().unwrap().client_commands[before..].to_vec();
    let updates = commands
        .iter()
        .filter(|packet| packet[34] == 2 && packet.get(52..54) == Some(&[2, 0]))
        .collect::<Vec<_>>();
    assert_eq!(updates.len(), 1);
    assert_eq!(updates[0][48], 1);
    assert_eq!(updates[0][59], 0);
    depart(&mut link, &model);
    retire(&mut link, &model);
}

#[test]
fn invalid_second_link_beacon_does_not_suppress_the_valid_beacons_gap_report() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    let beacon = other_link_beacon(&link, 1);
    let mut wrong_peer = beacon.clone();
    wrong_peer[32 + 10] ^= 4;
    device_receive(&model.queues[&0xd4520], &wrong_peer, true, 0);
    // Dropped and counted, as ieee80211_rx_handlers drops it.
    link.service().unwrap();
    assert_eq!(link.rejected_frames(), 1);
    drop(drain_notices(&mut link));
    device_receive(&model.queues[&0xd4520], &beacon, true, 0);
    link.service().unwrap();
    let notices = drain_notices(&mut link);
    assert_eq!(
        notices
            .iter()
            .filter(|notice| matches!(
                notice.event,
                LinkEvent::Controller(Event::Unsupported("MLO link beacon channel switches"))
            ))
            .count(),
        1
    );
    assert_eq!(
        link.state().client.client.state().links[0]
            .applied_edca
            .unwrap()
            .parameter_set_count,
        1
    );
    depart(&mut link, &model);
    retire(&mut link, &model);
}

mod link_keys;

mod reconfiguration_timer;
