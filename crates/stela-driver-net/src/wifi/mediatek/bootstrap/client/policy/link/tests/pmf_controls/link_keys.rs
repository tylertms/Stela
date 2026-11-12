// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! Per-link GTK/IGTK firmware batches and completed native group management RX.
use super::*;
use crate::wifi::mediatek::bootstrap::client::management::Received;
use crate::wifi::mediatek::mailbox::key::bip::tests::rotation::cmac;

fn install(link: &mut TestLink, index: u8, key: u8, floor: u64) -> Result<(), LinkControlError> {
    // SAFETY: this model owns the authenticated exchange, peer-index retirement
    // and all hardware/IRQ/work users. Frontend notices are explicitly drained.
    unsafe {
        link.install_link_management_key(1, index, &[key; 16], PacketNumber::new(floor).unwrap())
    }
}

fn group_frame(peer: Mac, index: u8, key: u8, pn: u64, departure: bool) -> Vec<u8> {
    let mut frame = response(
        if departure { 0xc0 } else { 0xd0 },
        if departure { &[7, 0] } else { &[126, 1] },
    );
    frame[4..10].copy_from_slice(&Mac::BROADCAST.0);
    frame[10..16].copy_from_slice(&peer.0);
    frame[16..22].copy_from_slice(&peer.0);
    frame.extend_from_slice(&[76, 16, index, 0]);
    frame.extend_from_slice(&pn.to_le_bytes()[..6]);
    frame.extend_from_slice(&[0; 8]);
    let mut aad = frame[..2].to_vec();
    aad.extend_from_slice(&frame[4..22]);
    aad.extend_from_slice(&frame[24..]);
    let mic = cmac(&[key; 16], &aad);
    frame[36..44].copy_from_slice(&mic[..8]);
    let mut raw = incoming_bytes(&frame, None, false);
    raw[4] = if peer == OTHER { 2 } else { 1 };
    if peer == OTHER {
        raw[13] = 52;
    }
    raw
}

fn receive(link: &mut TestLink, model: &Model, raw: &[u8]) -> Received {
    device_receive(&model.queues[&0xd4520], raw, true, 0);
    link.service().unwrap();
    let mut notices = drain_notices(link);
    let index = notices
        .iter()
        .position(|notice| {
            matches!(
                notice.event,
                LinkEvent::Controller(Event::ProtectedManagement(_))
            )
        })
        .expect("authenticated group observation");
    let LinkEvent::Controller(Event::ProtectedManagement(rx)) = notices.swap_remove(index).event
    else {
        unreachable!()
    };
    rx
}

#[test]
fn second_link_igtk_uses_its_gtk_batch_and_independent_ipn() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    install(&mut link, 4, 0x66, 100).unwrap();
    {
        let response = model.response.lock().unwrap();
        assert_eq!(
            response.keys[&18],
            [vec![0x46; 16], vec![0x66; 16]].concat()
        );
        assert_eq!(
            response.keys[&19],
            [vec![0x44; 16], vec![0x55; 16]].concat()
        );
    }
    link.register_action(&[126]).unwrap();
    for (peer, key, id) in [(OTHER, 0x66, Some(1)), (AP, 0x55, None)] {
        let rx = receive(&mut link, &model, &group_frame(peer, 4, key, 101, false));
        assert!(rx.group());
        assert_eq!(rx.link_id(), id);
        assert_eq!(
            link.with_management(rx, |_, body, pn| {
                assert_eq!(body, [126, 1]);
                pn.value()
            })
            .unwrap(),
            101
        );
    }
    let before = model.response.lock().unwrap().client_commands.len();
    install(&mut link, 4, 0x66, 0).unwrap();
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    let raw = group_frame(OTHER, 4, 0x66, 101, false);
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
    // A replayed IPN is dropped and counted.
    link.service().unwrap();
    assert_eq!(link.rejected_frames(), 1);
    assert_eq!(
        link.state().client.client.state().links[0]
            .gtk
            .igtk_receive_floor()
            .unwrap()
            .value(),
        101
    );
    depart(&mut link, &model);
    retire(&mut link, &model);
}

#[test]
fn second_link_rotation_revokes_its_delayed_envelope_only() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    install(&mut link, 4, 0x66, 100).unwrap();
    link.register_action(&[126]).unwrap();
    let old_link = receive(&mut link, &model, &group_frame(OTHER, 4, 0x66, 101, false));
    let setup = receive(&mut link, &model, &group_frame(AP, 4, 0x55, 101, false));
    install(&mut link, 5, 0x77, 200).unwrap();
    assert!(
        link.with_management(old_link, |_, _, _| panic!("rotated link envelope"))
            .is_err()
    );
    assert_eq!(
        link.with_management(setup, |_, _, pn| pn.value()).unwrap(),
        101
    );
    for (index, key, pn) in [(4, 0x66, 102), (5, 0x77, 201)] {
        let rx = receive(
            &mut link,
            &model,
            &group_frame(OTHER, index, key, pn, false),
        );
        assert_eq!(
            link.with_management(rx, |_, _, observed| observed.value())
                .unwrap(),
            pn
        );
    }
    depart(&mut link, &model);
    retire(&mut link, &model);
}

#[test]
fn second_link_bip_deauthentication_ends_association() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    install(&mut link, 4, 0x66, 100).unwrap();
    let rx = receive(&mut link, &model, &group_frame(OTHER, 4, 0x66, 101, true));
    assert_eq!(rx.link_id(), Some(1));
    assert_eq!(rx.body(), [7, 0]);
    assert!(!link.carrier());
    assert_eq!(link.station().phase(), station::Phase::Disconnecting);
    for _ in 0..4 {
        finish_tx(&model);
        link.service().unwrap();
    }
    retire(&mut link, &model);
}

#[test]
fn uncertain_second_link_igtk_keeps_the_whole_owner_quarantined() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = authorized_two_link_association();
    model.response.lock().unwrap().status = 9;
    assert!(install(&mut link, 4, 0x66, 100).is_err());
    assert_eq!(link.phase(), Phase::Quarantined);
    assert!(
        link.state().client.client.state().links[0]
            .gtk
            .is_quarantined()
    );
    assert!(!link.carrier());
    drop(link);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn secondary_igtk_waits_for_the_keepers_last_handshake_reply() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::wifi::{
        connection::keeper::tests::assert_secondary_key_waits_for_retirement,
        network::{Network, Secret},
        security::management::ManagementCipher,
        supplicant::IntegrityKey,
    };
    use stela_driver_base::entropy::{Entropy, EntropyError};
    struct Fixed;
    impl Entropy for Fixed {
        fn fill(&self, bytes: &mut [u8]) -> Result<(), EntropyError> {
            bytes.fill(0x5c);
            Ok(())
        }
    }
    let (mut link, model) = keyed_two_link_association();
    let epoch = link.station().association_epoch().unwrap();
    let ptk = link
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    // SAFETY: exact retry of the model's authenticated setup-link key after
    // all its earlier native and deferred users retired.
    let management = unsafe {
        link.install_management_key(epoch, 4, &[0x55; 16], PacketNumber::new(100).unwrap())
    }
    .unwrap();
    // The native reply is deliberately still in flight when the keeper tries
    // the secondary key. No synthetic successful retirement is supplied.
    let mut reply = control();
    reply[..6].copy_from_slice(&AP_MLD.0);
    link.send_control(&reply, Some(ptk)).unwrap();
    let station = crate::wifi::mediatek::station::Station::new(
        link,
        profile().interface,
        country(),
        Network {
            ssid: b"x".to_vec(),
            secret: Secret::parse(b"correct horse battery").unwrap(),
            country: None,
        },
        Arc::new(Fixed),
        crate::Clock {
            ticks_per_microsecond: 1,
        },
    );
    let mut station = assert_secondary_key_waits_for_retirement(
        station,
        ptk,
        management,
        IntegrityKey {
            index: 4,
            cipher: ManagementCipher::Cmac128,
            material: zeroize::Zeroizing::new(vec![0x66; 16]),
            receive: PacketNumber::new(100).unwrap(),
        },
        |radio| {
            assert!(!finish_tx(&model).is_empty());
            radio.link_mut().service().unwrap();
            drop(drain_notices(radio.link_mut()));
        },
        |radio| {
            radio.link_mut().state().client.client.state().links[0]
                .gtk
                .installed_igtk()
                .is_some()
        },
    );
    let link = station.radio_mut().link_mut();
    depart(link, &model);
    retire(link, &model);
}

#[test]
fn second_link_refuses_wrong_wcid_channel_and_mic_before_advancing_ipn() {
    if crate::test_process::isolate_model() {
        return;
    }
    for fault in 0..4 {
        let (mut link, model) = authorized_two_link_association();
        install(&mut link, 4, 0x66, 100).unwrap();
        link.register_action(&[126]).unwrap();
        let mut raw = group_frame(OTHER, 4, 0x66, 101, false);
        match fault {
            0 => raw[4] = 1,
            1 => raw[4] = 18,
            2 => raw[13] = 36,
            _ => *raw.last_mut().unwrap() ^= 1,
        }
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
        // Dropped and counted, as ieee80211_rx_handlers drops it.
        link.service().unwrap();
        assert_eq!(link.rejected_frames(), 1);
        assert_eq!(
            link.state().client.client.state().links[0]
                .gtk
                .igtk_receive_floor()
                .unwrap()
                .value(),
            100
        );
        let rx = receive(&mut link, &model, &group_frame(OTHER, 4, 0x66, 101, false));
        assert_eq!(
            link.with_management(rx, |_, _, pn| pn.value()).unwrap(),
            101
        );
        depart(&mut link, &model);
        retire(&mut link, &model);
    }
}

#[test]
fn second_link_key_rejects_unknown_link_and_held_notices_without_commands() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = keyed_two_link_association();
    let before = model.response.lock().unwrap().client_commands.len();
    assert!(
        // SAFETY: the model is the sole key owner; an unknown link is refused
        // before any firmware key publication or change of ownership.
        unsafe {
            link.install_link_management_key(15, 4, &[0x66; 16], PacketNumber::new(100).unwrap())
        }
        .is_err()
    );
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    link.authorize_port(link.station().association_epoch().unwrap())
        .unwrap();
    // Install a protected unicast observation and leave its notice queued.
    let mut deauth = response(0x40d0, &[8, 1, 3, 4]);
    deauth[4..10].copy_from_slice(&OTHER_LINK.0);
    deauth[10..16].copy_from_slice(&OTHER.0);
    deauth[16..22].copy_from_slice(&OTHER.0);
    link.register_action(&[8, 1]).unwrap();
    let mut raw = incoming_bytes(&deauth, Some((2, 0, 1)), false);
    raw[13] = 52;
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
    link.service().unwrap();
    let before = model.response.lock().unwrap().client_commands.len();
    assert!(matches!(
        install(&mut link, 4, 0x66, 100),
        Err(LinkControlError::Backpressure)
    ));
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    drop(drain_notices(&mut link));
    install(&mut link, 4, 0x66, 100).unwrap();
    depart(&mut link, &model);
    retire(&mut link, &model);
}

#[test]
fn second_link_all_bip_suites_authenticate_independent_vectors() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::wifi::security::management::ManagementCipher;
    // Frozen Python cryptography 46.0.5 CMAC/AESGCM vectors: A2/BSSID
    // 02:06:07:08:09:0b, reason 7, index 4, IPN 101, key bytes 0x66.
    // GMAC nonce is A2 || big-endian IPN, as pinned mac80211/wpa.c builds it.
    for (cipher, mic) in [
        (
            ManagementCipher::Cmac128,
            &[0x63, 0x9b, 0xcc, 0xdc, 0x19, 0xab, 0x7d, 0xc3][..],
        ),
        (
            ManagementCipher::Cmac256,
            &[
                0x13, 0x55, 0x1e, 0x21, 0x18, 0x0a, 0xc2, 0x54, 0x64, 0x79, 0xf2, 0x5b, 0x66, 0x47,
                0x2a, 0xbc,
            ][..],
        ),
        (
            ManagementCipher::Gmac128,
            &[
                0x56, 0x39, 0x0e, 0x35, 0xad, 0x27, 0xa7, 0x98, 0x67, 0x9c, 0xdd, 0x6f, 0xcf, 0x7c,
                0x0d, 0x44,
            ][..],
        ),
        (
            ManagementCipher::Gmac256,
            &[
                0x55, 0x83, 0xf8, 0x96, 0xaa, 0x1d, 0xb2, 0x8a, 0x7f, 0xd3, 0x86, 0xe6, 0x0f, 0x03,
                0x82, 0x16,
            ][..],
        ),
    ] {
        let (mut link, model, _) = two_link_association_with_cipher(cipher.suite());
        link.configure().unwrap();
        let epoch = link.station().association_epoch().unwrap();
        link.install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
            .unwrap();
        link.install_key(false, 2, &[0x44; 16], ReplaySeed::ZERO)
            .unwrap();
        link.install_link_key(1, 1, &[0x46; 16], ReplaySeed::ZERO)
            .unwrap();
        drop(drain_notices(&mut link));
        let before = model.response.lock().unwrap().client_commands.len();
        // SAFETY: this fixture owns the authenticated exchange and all prior
        // hardware/RX/IRQ/work/key users have retired, before either batch.
        unsafe {
            link.install_link_management_key(
                1,
                4,
                &vec![0x66; cipher.key_length()],
                PacketNumber::new(100).unwrap(),
            )
            .unwrap();
            link.install_management_key(
                epoch,
                4,
                &vec![0x55; cipher.key_length()],
                PacketNumber::new(100).unwrap(),
            )
            .unwrap();
        }
        // Linux leaves the other three suites to software; no successful
        // firmware cipher or dummy key command is substituted for that path.
        let sent = model.response.lock().unwrap().client_commands.len() - before;
        assert_eq!(
            sent,
            if cipher == ManagementCipher::Cmac128 {
                2
            } else {
                0
            }
        );
        link.authorize_port(epoch).unwrap();
        let mut frame = response(0xc0, &[7, 0]);
        frame[4..10].copy_from_slice(&Mac::BROADCAST.0);
        frame[10..16].copy_from_slice(&OTHER.0);
        frame[16..22].copy_from_slice(&OTHER.0);
        frame.extend_from_slice(&[
            76,
            (cipher.mmie_length() - 2) as u8,
            4,
            0,
            101,
            0,
            0,
            0,
            0,
            0,
        ]);
        frame.extend_from_slice(mic);
        let mut raw = incoming_bytes(&frame, None, false);
        raw[4] = 2;
        raw[13] = 52;
        let mut forged = raw.clone();
        *forged.last_mut().unwrap() ^= 1;
        device_receive(&model.queues[&0xd4520], &forged, true, 0);
        // A forged MIC is dropped and counted.
        let rejected = link.rejected_frames();
        link.service().unwrap();
        assert_eq!(link.rejected_frames(), rejected + 1);
        assert!(link.carrier());
        assert_eq!(
            link.state().client.client.state().links[0]
                .gtk
                .igtk_receive_floor()
                .unwrap()
                .value(),
            100
        );
        let rx = receive(&mut link, &model, &raw);
        assert_eq!(
            (rx.link_id(), rx.body(), rx.packet_number().value()),
            (Some(1), &[7, 0][..], 101)
        );
        assert!(!link.carrier());
        for _ in 0..4 {
            finish_tx(&model);
            link.service().unwrap();
        }
        retire(&mut link, &model);
    }
}
