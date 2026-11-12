extern crate std;
use super::super::super::tests::*;
use super::*;
use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

fn source_bip_packet(gtk: KeyIdentity) -> crate::wifi::frame::Mpdu {
    // Complete pinned CMAC helper native.trace, SHA256 3bbe84f7780d5cc948d8a6a0c9fff42a891370c7ecadfb650c2c1baf3119fe40.
    // Expected MIC comes from independent CommonCrypto-AES + unchanged Linux CMAC/BIP bodies.
    let bytes = vec![
        0xc0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 2, 6, 7, 8, 9, 10, 2, 6, 7, 8, 9, 10, 0, 0, 7,
        0, 76, 16, 4, 0, 101, 0, 0, 0, 0, 0, 0xc4, 0x80, 0xa2, 0x3a, 0x2c, 0x44, 0x77, 0x70,
    ];
    crate::wifi::frame::Mpdu::new(
        bytes,
        key_packet(gtk, 1).radio,
        security::SecurityStatus::default(),
    )
    .unwrap()
}

#[test]
fn bip_gmac_igtks_are_kept_in_software_as_mt7925_set_key_leaves_them() {
    if crate::test_process::isolate_model() {
        return;
    }
    let response = Arc::new(Mutex::new(Response {
        key_execution: true,
        ..Response::default()
    }));
    let (mut mailbox, wm, _, _) = setup_controlled(Chip::Mt7925, true, false, 0, response.clone());
    let drops = Arc::new(AtomicUsize::new(0));
    // SAFETY: exclusive empty source-modeled WCID and drained old users.
    let mut key = unsafe {
        HardwareKey::claim(
            &mailbox,
            7,
            key_context(),
            key_scope(true),
            KeyResource(drops.clone()),
        )
    }
    .unwrap();
    let gtk = key
        .install(
            &mut mailbox,
            2,
            Cipher::Ccmp128,
            &[0x44; 16],
            ReplaySeed::ZERO,
        )
        .unwrap();
    let commands = wm.lock().unwrap().completed.len();
    // SAFETY: initial modeled GTK request/ACK drained; no other key users exist.
    let identity = unsafe {
        key.install_igtk(
            &mut mailbox,
            ManagementCipher::Gmac256,
            4,
            &[0x55; 32],
            PacketNumber::new(100).unwrap(),
        )
    }
    .unwrap();
    assert_eq!(key.installed_igtk(), Some(identity));
    // The 26-byte MMIE's key ID and an independent AES-GCM tag.
    let mut bytes = vec![
        0xc0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 2, 6, 7, 8, 9, 10, 2, 6, 7, 8, 9, 10, 0, 0, 7,
        0, 76, 24, 4, 0, 101, 0, 0, 0, 0, 0,
    ];
    bytes.extend_from_slice(&[
        0x56, 0xf8, 0x6b, 0x82, 0x6a, 0x7b, 0xc6, 0xc9, 0x69, 0x5d, 0xf6, 0xf6, 0xa0, 0xe0, 0x30,
        0x9b,
    ]);
    let frame = crate::wifi::frame::Mpdu::new(
        bytes,
        key_packet(gtk, 1).radio,
        security::SecurityStatus::default(),
    )
    .unwrap();
    let authenticated = key.authenticate_bip(&frame).unwrap();
    assert_eq!(authenticated.packet_number().value(), 101);
    // SAFETY: unchanged drained namespace; the authenticated exchange rotates.
    unsafe { key.rotate_igtk(&mut mailbox, 5, &[0x66; 32], PacketNumber::new(0).unwrap()) }
        .unwrap();
    assert!(key.installed_igtks().iter().all(Option::is_some));
    assert_eq!(wm.lock().unwrap().completed.len(), commands);
    let removed = key.remove(&mut mailbox).unwrap();
    // SAFETY: removal ACK and complete model key/user drain.
    unsafe { key.retire_after_drain(removed) }.unwrap();
}

#[test]
fn acknowledged_igtk_authenticates_source_mic_without_hardware_flag_bypass() {
    if crate::test_process::isolate_model() {
        return;
    }
    let response = Arc::new(Mutex::new(Response {
        key_execution: true,
        ..Response::default()
    }));
    let (mut mailbox, wm, _, _) = setup_controlled(Chip::Mt7925, true, false, 0, response.clone());
    let drops = Arc::new(AtomicUsize::new(0));
    // SAFETY: exclusive empty source-modeled WCID and drained old users.
    let mut key = unsafe {
        HardwareKey::claim(
            &mailbox,
            7,
            key_context(),
            key_scope(true),
            KeyResource(drops.clone()),
        )
    }
    .unwrap();
    let gtk = key
        .install(
            &mut mailbox,
            2,
            Cipher::Ccmp128,
            &[0x44; 16],
            ReplaySeed::ZERO,
        )
        .unwrap();
    key.authorize(&key_packet(gtk, 9)).unwrap();
    let frame = source_bip_packet(gtk);
    assert!(matches!(
        key.authenticate_bip(&frame),
        Err(super::super::Error::Bip(Error::DataKeyRequired))
    ));
    // SAFETY: initial modeled GTK request/ACK drained; no other key users exist.
    let identity = unsafe {
        key.install_igtk(
            &mut mailbox,
            ManagementCipher::Cmac128,
            4,
            &[0x55; 16],
            PacketNumber::new(100).unwrap(),
        )
    }
    .unwrap();
    let commands = wm.lock().unwrap().completed.len();
    let authenticated = key.authenticate_bip(&frame).unwrap();
    assert_eq!(authenticated.body(), &[7, 0]);
    assert_eq!(authenticated.gtk(), gtk);
    assert_eq!(authenticated.index(), 4);
    assert_eq!(authenticated.packet_number().value(), 101);
    assert_eq!(key.igtk_receive_floor().unwrap().value(), 101);
    assert!(matches!(
        key.authenticate_bip(&frame),
        Err(super::super::Error::Bip(Error::Authentication(
            management::Error::Replay
        )))
    ));
    // Fake copied hardware integrity must not bypass the software MIC.
    let mut bytes = frame.bytes().to_vec();
    bytes[30] = 102;
    let mut forged = crate::wifi::frame::Mpdu::new(
        bytes,
        frame.radio.clone(),
        security::SecurityStatus::default(),
    )
    .unwrap();
    forged.security.decrypted = true;
    forged.security.integrity = security::Integrity::Verified;
    assert!(matches!(
        key.authenticate_bip(&forged),
        Err(super::super::Error::Bip(Error::Authentication(
            management::Error::Integrity
        )))
    ));
    assert_eq!(key.igtk_receive_floor().unwrap().value(), 101);
    assert_eq!(
        key.igtk_statistics(),
        Some(management::Statistics {
            replays: 1,
            mic_failures: 1
        })
    );
    // Authenticated management replay never consumes a canonical GTK data PN.
    assert!(matches!(
        key.authorize(&key_packet(gtk, 9)),
        Err(security::Error::Replay)
    ));
    key.authorize(&key_packet(gtk, 10)).unwrap();
    assert_eq!(
        // SAFETY: unchanged drained namespace; identical retry must preserve advanced
        // management IPN.
        unsafe {
            key.install_igtk(
                &mut mailbox,
                ManagementCipher::Cmac128,
                4,
                &[0x55; 16],
                PacketNumber::new(0).unwrap(),
            )
        }
        .unwrap(),
        identity
    );
    assert_eq!(key.igtk_receive_floor().unwrap().value(), 101);
    assert_eq!(wm.lock().unwrap().completed.len(), commands);
    let removed = key.remove(&mut mailbox).unwrap();
    assert!(matches!(
        key.authenticate_bip(&frame),
        Err(super::super::Error::Bip(Error::DataKeyRequired))
    ));
    // SAFETY: successful whole-batch removal and complete old model user drain.
    unsafe { key.retire_after_drain(removed) }.unwrap();
    assert!(key.igtk_statistics().is_none());
    assert!(key.igtk_receive_floor().is_none());
    assert!(!response.lock().unwrap().keys.contains_key(&7));
    drop(key);
    assert_eq!(drops.load(Ordering::Relaxed), 1);
}

#[test]
fn owned_gtk_igtk_batch_preserves_original_data_key_replay_and_exact_retry() {
    if crate::test_process::isolate_model() {
        return;
    }
    let response = Arc::new(Mutex::new(Response {
        key_execution: true,
        ..Response::default()
    }));
    let (mut mailbox, wm, _, _) = setup_controlled(Chip::Mt7925, true, false, 0, response.clone());
    let drops = Arc::new(AtomicUsize::new(0));
    // SAFETY: exclusive empty source-modeled WCID/BSS and old-user drain.
    let mut key = unsafe {
        HardwareKey::claim(
            &mailbox,
            7,
            key_context(),
            key_scope(true),
            KeyResource(drops.clone()),
        )
    }
    .unwrap();
    let gtk = key
        .install(
            &mut mailbox,
            2,
            Cipher::Ccmp128,
            &[0x44; 16],
            ReplaySeed::uniform(PacketNumber::new(7).unwrap()),
        )
        .unwrap();
    key.authorize(&key_packet(gtk, 9)).unwrap();
    let before = wm.lock().unwrap().completed.len();
    // SAFETY: every model GTK descriptor/ACK was consumed; no old key users remain.
    let installed = unsafe {
        key.install_igtk(
            &mut mailbox,
            ManagementCipher::Cmac128,
            5,
            &[0x55; 16],
            PacketNumber::new(100).unwrap(),
        )
    }
    .unwrap();
    assert_eq!(installed.gtk(), gtk);
    assert_eq!(installed.index(), 5);
    assert_eq!(key.installed_igtk(), Some(installed));
    assert_eq!(key.igtk_receive_floor().unwrap().value(), 100);
    assert_eq!(wm.lock().unwrap().completed.len(), before + 1);
    assert_eq!(
        response.lock().unwrap().keys[&7],
        [vec![0x44; 16], vec![0x55; 16]].concat()
    );
    let packets = wm.lock().unwrap();
    let body = &packets.completed.last().unwrap()[48..];
    assert_eq!(&body[23..26], &[6, 2, 32]);
    drop(packets);
    assert_eq!(key.slot().installed(), Some(gtk));
    assert!(matches!(
        key.authorize(&key_packet(gtk, 9)),
        Err(security::Error::Replay)
    ));
    key.authorize(&key_packet(gtk, 10)).unwrap();
    // SAFETY: same drained namespace; this exact retry must not publish.
    let retry = unsafe {
        key.install_igtk(
            &mut mailbox,
            ManagementCipher::Cmac128,
            5,
            &[0x55; 16],
            PacketNumber::new(0).unwrap(),
        )
    }
    .unwrap();
    assert_eq!(retry, installed);
    assert_eq!(key.igtk_receive_floor().unwrap().value(), 100);
    assert!(matches!(
        // SAFETY: no external model key users; different live initial-batch material is
        // refused.
        unsafe {
            key.install_igtk(
                &mut mailbox,
                ManagementCipher::Cmac128,
                4,
                &[0x66; 16],
                PacketNumber::new(0).unwrap(),
            )
        },
        Err(super::super::Error::Bip(Error::AlreadyInstalled))
    ));
    assert_eq!(wm.lock().unwrap().completed.len(), before + 1);
    let removed = key.remove(&mut mailbox).unwrap();
    assert!(key.installed_igtk().is_none());
    assert!(!response.lock().unwrap().keys.contains_key(&7));
    // SAFETY: whole source batch removal ACK and complete modeled-user drain.
    unsafe { key.retire_after_drain(removed) }.unwrap();
    drop(key);
    assert_eq!(drops.load(Ordering::Relaxed), 1);
}

#[test]
fn invalid_igtk_admission_never_publishes_or_changes_the_original_key() {
    if crate::test_process::isolate_model() {
        return;
    }
    for (chip, group, cipher) in [
        (Chip::Mt7921, false, Cipher::Ccmp128),
        (Chip::Mt7922, true, Cipher::Gcmp128),
        (Chip::Mt7925, false, Cipher::Ccmp128),
        (Chip::Mt7925, true, Cipher::Gcmp128),
        (Chip::Mt7925, true, Cipher::Ccmp128),
    ] {
        let response = Arc::new(Mutex::new(Response {
            key_execution: true,
            ..Response::default()
        }));
        let (mut mailbox, wm, _, _) = setup_controlled(chip, true, false, 0, response.clone());
        let drops = Arc::new(AtomicUsize::new(0));
        // SAFETY: exclusive empty/drained model WCID and peer context.
        let mut key = unsafe {
            HardwareKey::claim(
                &mailbox,
                7,
                key_context(),
                key_scope(group),
                KeyResource(drops.clone()),
            )
        }
        .unwrap();
        let empty = wm.lock().unwrap().completed.len();
        assert!(
            // SAFETY: no model key users; absence of a GTK must fail before publication.
            unsafe {
                key.install_igtk(
                    &mut mailbox,
                    ManagementCipher::Cmac128,
                    4,
                    &[0x55; 16],
                    PacketNumber::new(1).unwrap(),
                )
            }
            .is_err()
        );
        assert_eq!(wm.lock().unwrap().completed.len(), empty);
        let gtk = key
            .install(
                &mut mailbox,
                if group { 2 } else { 0 },
                cipher,
                &[0x44; 16],
                ReplaySeed::ZERO,
            )
            .unwrap();
        let before = wm.lock().unwrap().completed.len();
        for (index, length) in [(3, 16), (6, 16), (4, 15), (5, 17)] {
            let expected = if !group {
                super::super::Error::Bip(Error::DataKeyRequired)
            } else if !(4..=5).contains(&index) {
                super::super::Error::Request(
                    crate::wifi::mediatek::mcu::key::Error::InvalidBipIndex,
                )
            } else {
                super::super::Error::Request(
                    crate::wifi::mediatek::mcu::key::Error::InvalidKeyLength,
                )
            };
            assert_eq!(
                // SAFETY: every initial descriptor/ACK drained, no external model users.
                unsafe {
                    key.install_igtk(
                        &mut mailbox,
                        ManagementCipher::Cmac128,
                        index,
                        &vec![0x55; length],
                        PacketNumber::new(1).unwrap(),
                    )
                },
                Err(expected)
            );
            assert_eq!(wm.lock().unwrap().completed.len(), before);
            assert_eq!(key.slot().installed(), Some(gtk));
            assert!(key.installed_igtk().is_none());
            assert!(!key.is_quarantined());
        }
        if !group || cipher != Cipher::Ccmp128 {
            // SAFETY: model namespace is drained; unsupported scope stays explicit.
            let result = unsafe {
                key.install_igtk(
                    &mut mailbox,
                    ManagementCipher::Cmac128,
                    4,
                    &[0x55; 16],
                    PacketNumber::new(1).unwrap(),
                )
            };
            if !group {
                assert_eq!(
                    result,
                    Err(super::super::Error::Bip(Error::DataKeyRequired))
                );
            } else {
                // mt7925_set_key batches BIP only beside a CCMP GTK; this
                // one is kept in software.
                assert_eq!(key.installed_igtk(), Some(result.unwrap()));
            }
            assert_eq!(wm.lock().unwrap().completed.len(), before);
            assert_eq!(key.slot().installed(), Some(gtk));
        }
        let removed = key.remove(&mut mailbox).unwrap();
        // SAFETY: removal ACK and complete model key/user drain.
        unsafe { key.retire_after_drain(removed) }.unwrap();
        drop(key);
        assert_eq!(drops.load(Ordering::Relaxed), 1);
    }
}

#[test]
fn uncertain_igtk_batch_revokes_data_and_keeps_namespace() {
    if crate::test_process::isolate_model() {
        return;
    }
    for fault in 0..4 {
        let response = Arc::new(Mutex::new(Response {
            key_execution: true,
            ..Response::default()
        }));
        let (mut mailbox, _, _, _) =
            setup_controlled(Chip::Mt7925, true, false, 0, response.clone());
        let drops = Arc::new(AtomicUsize::new(0));
        // SAFETY: exclusive empty source-modeled WCID/BSS, with no old key users.
        let mut key = unsafe {
            HardwareKey::claim(
                &mailbox,
                7,
                key_context(),
                key_scope(true),
                KeyResource(drops.clone()),
            )
        }
        .unwrap();
        let gtk = key
            .install(
                &mut mailbox,
                2,
                Cipher::Ccmp128,
                &[0x44; 16],
                ReplaySeed::ZERO,
            )
            .unwrap();
        match fault {
            0 => response.lock().unwrap().status = 9,
            1 => response.lock().unwrap().command_delta = 1,
            2 => response.lock().unwrap().unsolicited = true,
            _ => response.lock().unwrap().drop = true,
        }
        assert!(
            // SAFETY: model initial key request is drained; late/invalid ACK must
            // quarantine.
            unsafe {
                key.install_igtk(
                    &mut mailbox,
                    ManagementCipher::Cmac128,
                    4,
                    &[0x55; 16],
                    PacketNumber::new(1).unwrap(),
                )
            }
            .is_err()
        );
        assert!(key.is_quarantined());
        assert!(key.slot().installed().is_none());
        assert!(key.installed_igtk().is_none());
        assert!(matches!(
            key.authenticate_bip(&source_bip_packet(gtk)),
            Err(super::super::Error::Bip(Error::DataKeyRequired))
        ));
        assert!(matches!(
            key.authorize(&key_packet(gtk, 2)),
            Err(security::Error::NoInstalledKey)
        ));
        assert_eq!(
            response.lock().unwrap().keys[&7],
            [vec![0x44; 16], vec![0x55; 16]].concat()
        );
        drop(key);
        drop(mailbox);
        assert_eq!(drops.load(Ordering::Relaxed), 0);
    }
}

pub(crate) mod rotation;
