use super::super::modern::tests::modern_inputs;
use super::super::tests::*;
use super::*;
use crate::wifi::mediatek::{
    bootstrap::tests::*,
    mcu::domain,
    ring::{rx::tests::device_receive, tests::Source},
};
use crate::wifi::stepped::block_on;
use core::sync::atomic::Ordering;
use reg::database::{
    Database,
    tests::{country, country_of, fixture},
};
use sha2::{Digest, Sha256};

pub(super) fn setup_modern() -> (Client<Bar, Q, NativeGuard>, Model) {
    setup_modern_power(false)
}
fn setup_modern_power(split: bool) -> (Client<Bar, Q, NativeGuard>, Model) {
    setup_modern_radio(split, CHANNEL)
}
pub(super) fn setup_modern_radio(
    split: bool,
    channel: Channel,
) -> (Client<Bar, Q, NativeGuard>, Model) {
    setup_modern_bands(split, channel, 0, None)
}
/// A modern client whose board also programs 6 GHz channel 1 with `flags`.
pub(crate) fn setup_six(flags: u32) -> (Client<Bar, Q, NativeGuard>, Model) {
    setup_modern_bands(false, CHANNEL, 0, Some(flags))
}
/// A modern client whose 5 GHz channels bar probes, under `passive_country`.
pub(crate) fn setup_passive_five() -> (Client<Bar, Q, NativeGuard>, Model) {
    setup_modern_bands(false, CHANNEL, channel::NO_IR, None)
}
/// The rules of `country()` with its 5 GHz band NO-IR.
pub(crate) fn passive_country() -> Country {
    country_of(&[
        (2_400_000, 2_500_000, 80_000, 3000, 0),
        (5_150_000, 5_350_000, 80_000, 3000, 8),
    ])
}
/// The US rules of `country()` and its 6 GHz band.
pub(crate) fn six_country() -> Country {
    country_of(&[
        (2_400_000, 2_500_000, 80_000, 3000, 0),
        (5_150_000, 5_350_000, 80_000, 3000, 0),
        (5_925_000, 7_125_000, 160_000, 2400, 0),
    ])
}
fn setup_modern_bands(
    split: bool,
    channel: Channel,
    five: u32,
    six: Option<u32>,
) -> (Client<Bar, Q, NativeGuard>, Model) {
    let mut f = NativeFixture::new(Chip::Mt7925);
    if six.is_some() {
        f.model.lock().unwrap().capability_payload = Some(
            crate::wifi::mediatek::mailbox::clc::remaining::tests::caps(Chip::Mt7925, [true; 3]),
        );
    }
    f.install().unwrap();
    f.boot(Chip::Mt7925);
    f.policy_channel_power(
        Chip::Mt7925,
        [true, true, six.is_some()],
        [
            vec![domain::Channel {
                hardware_value: 1,
                flags: WIDE,
            }],
            [36, 40, 44, 48, 52, 56, 60, 64]
                .map(|hardware_value| domain::Channel {
                    hardware_value,
                    flags: WIDE | five,
                })
                .to_vec(),
            six.map(|flags| domain::Channel {
                hardware_value: 1,
                flags: WIDE | flags,
            })
            .into_iter()
            .collect(),
        ],
        |band, n| {
            wifi::mediatek::mcu::power::Limits::uniform(if split && band == Band::Ghz5 {
                if n == 36 { 40 } else { 20 }
            } else {
                -3
            })
        },
    );
    // SAFETY: modeled completed bootstrap, source policy and owner.
    unsafe { block_on(f.controller.initialize_mac()) }.unwrap();
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe {
        block_on(f.controller.start(
            channel.band,
            u8::try_from(channel.number).unwrap(),
            channel.frequency_mhz,
        ))
    }
    .unwrap();
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let interface =
        unsafe { block_on(f.controller.register_interface(profile().interface)) }.unwrap();
    f.model.lock().unwrap().client_execution = true;
    f.model.lock().unwrap().key_execution = true;
    let NativeFixture {
        controller,
        model,
        bar,
        queues,
        memory,
        drops,
        ..
    } = f;
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let client = unsafe { Client::new(controller, interface) }.unwrap();
    (
        client,
        Model {
            response: model,
            bar,
            queues,
            memory,
            drops,
        },
    )
}
pub(super) fn done(model: &Model) {
    let mut done = vec![0; 52];
    done[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
    done[36] = 14;
    done[38] = 6;
    done[50] = 4;
    device_receive(&model.queues[&0xd4500], &done, true, 0);
}
fn finish_stop(driver: &mut ManagedClient<Bar, Q, NativeGuard>, model: &Model) {
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: all modeled device users have completed; no external IRQ/DMA.
    unsafe { driver.stop_after_quiesce() }.unwrap();
    assert_eq!(driver.phase(), Phase::Stopped);
}
#[test]
fn managed_scan_uses_policy_revokes_real_scan_then_stops_and_releases() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup();
    let mut driver = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let count = model.response.lock().unwrap().client_commands.len();
    // This programmed domain has NO_IR. A normal source country
    // does not widen it into active-scan or autonomous transmit permission.
    assert!(
        driver
            .begin_scan(
                scan::Request::new(vec![CHANNEL], vec![vec![]], vec![]).unwrap(),
                0,
                1000
            )
            .is_err()
    );
    assert_eq!(count, model.response.lock().unwrap().client_commands.len());
    driver
        .begin_scan(
            scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
            0,
            1000,
        )
        .unwrap();
    driver.revoke().unwrap();
    assert_eq!(driver.phase(), Phase::Scanning);
    assert!(matches!(
        driver.begin_scan(
            scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
            1,
            100
        ),
        Err(PolicyError::Revoked)
    ));
    assert!(model.response.lock().unwrap().client_commands.len() >= count + 2);
    done(&model);
    driver.poll(2, &Source).unwrap();
    assert_eq!(driver.phase(), Phase::ScanCompleted);
    // SAFETY: modeled terminal/notifications/DMA drained.
    unsafe { driver.retire_scan_after_drain() }.unwrap();
    assert_eq!(driver.phase(), Phase::ControllerDrain);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    finish_stop(&mut driver, &model);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}
fn awaiting(
    client: Client<Bar, Q, NativeGuard>,
    model: Model,
    country: Country,
) -> (ManagedClient<Bar, Q, NativeGuard>, Model) {
    let (join, bss) = modern_inputs(&client);
    let mut driver = ManagedClient::new(client, country, Environment::Indoor).unwrap();
    driver
        .begin_scan(
            scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
            0,
            1000,
        )
        .unwrap();
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&crate::Mac::BROADCAST.0);
    incoming(&model, &beacon, None, false);
    driver.poll(1, &Source).unwrap();
    let handle = driver.bss().next().expect("real DMA-observed AP").0;
    done(&model);
    driver.poll(2, &Source).unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_scan_after_drain() }.unwrap();
    driver
        .join_modern(handle, join, profile(), 100, &Source)
        .unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    driver.poll(101, &Source).unwrap();
    driver.poll_released().unwrap();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    driver.poll(102, &Source).unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    driver.poll(103, &Source).unwrap();
    driver.poll_released().unwrap();
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&bss.elements);
    incoming(&model, &response(0x10, &assoc), None, false);
    driver.poll(104, &Source).unwrap();
    assert_eq!(driver.phase(), Phase::AwaitingModernAdmission);
    (driver, model)
}
#[test]
fn managed_modern_ap_observation_configures_eht_keys_qos_and_revocation() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    let (mut driver, model) = awaiting(client, model, country());
    driver.configure_modern(105).unwrap();
    assert_eq!(driver.phase(), Phase::Associated);
    driver
        .install_key(true, 0, &[4; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .install_key(false, 2, &[5; 16], ReplaySeed::ZERO)
        .unwrap();
    let epoch = driver.station().association_epoch().unwrap();
    driver.authorize_port(epoch).unwrap();
    driver
        .transmit(epoch, &eth(0x0800), TxClass::Data, Some(5), &Source)
        .unwrap();
    let packets = finish_tx(&model);
    assert_eq!(packets.len(), 1);
    assert_eq!(packets[0][0] & 0xfc, 0x88);
    driver.poll(106, &Source).unwrap();
    driver.poll_released().unwrap();
    driver.revoke().unwrap();
    assert_eq!(driver.phase(), Phase::NeedsDrain);
    assert!(matches!(
        driver.transmit(epoch, &eth(0x0800), TxClass::Data, Some(5), &Source),
        Err(PolicyError::Revoked)
    ));
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    assert_eq!(driver.phase(), Phase::ControllerDrain);
    finish_stop(&mut driver, &model);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}
#[test]
fn source_hardware_footprint_refuses_before_configuration() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup();
    let (mut driver, model) = awaiting(client, model, country());
    let before = model.response.lock().unwrap().client_commands.len();
    assert!(matches!(
        driver.configure_modern(105),
        Err(PolicyError::Regulatory(reg::Error::HardwareDenied))
    ));
    assert_eq!(before, model.response.lock().unwrap().client_commands.len());
    assert_eq!(driver.phase(), Phase::AwaitingModernAdmission);
    driver.revoke().unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    finish_stop(&mut driver, &model);
}
#[test]
fn a_domain_wider_than_its_country_is_refused_at_attachment() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern();
    // 80 MHz programmed where the country allows 40 MHz.
    let narrow = country_of(&[
        (2_400_000, 2_500_000, 80_000, 3000, 0),
        (5_150_000, 5_350_000, 40_000, 3000, 0),
    ]);
    let error = match ManagedClient::new(client, narrow, Environment::Indoor) {
        Err(error) => error,
        Ok(_) => panic!("a wider domain admitted"),
    };
    assert!(matches!(error.error, PolicyError::RestrictionMismatch));
    let mut corrected = ManagedClient::new(error.client, country(), Environment::Indoor).unwrap();
    corrected.revoke().unwrap();
    finish_stop(&mut corrected, &model);
}
#[test]
fn unequal_wide_power_rows_refuse_split_eirp_rules_before_configuration() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup_modern_power(true);
    let original = fixture();
    let mut b = vec![0; 128];
    b[..24].copy_from_slice(&original[..24]);
    b[17] = 3;
    b[20..26].copy_from_slice(&[0, 8, 0, 14, 0, 20]);
    b[32..48].copy_from_slice(&original[24..40]);
    for (at, start, end, eirp) in [
        (56, 5_150_000u32, 5_190_000u32, 3000u16),
        (80, 5_190_000, 5_350_000, 1000),
    ] {
        b[at..at + 16].copy_from_slice(&original[48..64]);
        b[at + 1] = 16;
        b[at + 2..at + 4].copy_from_slice(&eirp.to_be_bytes());
        b[at + 4..at + 8].copy_from_slice(&start.to_be_bytes());
        b[at + 8..at + 12].copy_from_slice(&end.to_be_bytes());
        b[at + 12..at + 16].copy_from_slice(&20_000u32.to_be_bytes());
    }
    let country = Database::parse(Arc::from(b.clone()), Sha256::digest(&b).into())
        .unwrap()
        .select(*b"US")
        .unwrap();
    // Primary36/center rows20dBm; secondary40/44/48 rows10dBm.
    // Each enabled20MHz channel fits its own rule. EHT80 must still not
    // enable those20dBm wide/primary rows across the10dBm secondary rule.
    let (mut driver, model) = awaiting(client, model, country);
    let before = model.response.lock().unwrap().client_commands.len();
    assert!(matches!(
        driver.configure_modern(105),
        Err(PolicyError::Regulatory(reg::Error::OsDenied))
    ));
    assert_eq!(before, model.response.lock().unwrap().client_commands.len());
    driver.revoke().unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    finish_stop(&mut driver, &model);
}
#[test]
fn restricted_edca_publication_waits_for_ack_and_preserves_ap_metadata() {
    if crate::test_process::isolate_model() {
        return;
    }
    for accepted in [true, false] {
        let (client, model) = setup_modern();
        let mut b = fixture();
        b.resize(112, 0);
        b[48] = 20;
        b[66..68].copy_from_slice(&20u16.to_be_bytes());
        for (j, bytes) in [
            [0x68, 7, 0, 1],
            [0x79, 8, 0, 2],
            [0x5a, 9, 0, 3],
            [0x6b, 10, 0, 4],
        ]
        .into_iter()
        .enumerate()
        {
            b[80 + j * 4..84 + j * 4].copy_from_slice(&bytes);
            b[96 + j * 4..100 + j * 4].copy_from_slice(&bytes);
        }
        let country = Database::parse(Arc::from(b.clone()), Sha256::digest(&b).into())
            .unwrap()
            .select(*b"US")
            .unwrap();
        let (mut driver, model) = awaiting(client, model, country);
        let original = driver
            .client
            .modern_context()
            .unwrap()
            .negotiated()
            .wmm
            .clone();
        let before = model.response.lock().unwrap().client_commands.len();
        if !accepted {
            model.response.lock().unwrap().edca_status = Some(0xbad);
        }
        let result = driver.configure_modern(105);
        assert_eq!(
            driver.client.modern_context().unwrap().negotiated().wmm,
            original
        );
        let commands = model.response.lock().unwrap().client_commands.clone();
        let edca = commands[before..]
            .iter()
            .find(|p| p[34..36] == [4, 0])
            .expect("actual native EDCA command");
        let mut expected = vec![0, 0, 4, 0];
        for bytes in [
            [0, 0, 12, 0, 0, 15, 6, 8, 31, 0, 7, 0],
            [0, 0, 12, 0, 1, 15, 7, 9, 62, 0, 8, 0],
            [0, 0, 12, 0, 2, 15, 5, 10, 0, 0, 9, 0],
            [0, 0, 12, 0, 3, 15, 6, 11, 0, 0, 10, 0],
        ] {
            expected.extend_from_slice(&bytes);
        }
        assert_eq!(&edca[48..], expected);
        if !accepted {
            assert!(matches!(
                result,
                Err(PolicyError::Client(Error::Firmware(0xbad)))
            ));
            assert_eq!(driver.phase(), Phase::Quarantined);
            assert_eq!(driver.station().phase(), station::Phase::Configuring);
            assert_eq!(driver.client.completed_modern_requests(), 3);
            assert!(
                driver
                    .authorize_port(driver.station().association_epoch().unwrap())
                    .is_err()
            );
            drop(driver);
            assert_eq!(model.drops.load(Ordering::Relaxed), 0);
            assert!(!model.memory.lock().unwrap().mappings.is_empty());
            continue;
        }
        result.unwrap();
        assert_eq!(driver.phase(), Phase::Associated);
        driver
            .install_key(true, 0, &[4; 16], ReplaySeed::ZERO)
            .unwrap();
        driver
            .install_key(false, 2, &[5; 16], ReplaySeed::ZERO)
            .unwrap();
        let epoch = driver.station().association_epoch().unwrap();
        driver.authorize_port(epoch).unwrap();
        driver
            .transmit(epoch, &eth(0x0800), TxClass::Data, Some(5), &Source)
            .unwrap();
        assert_eq!(finish_tx(&model).len(), 1);
        driver.poll(106, &Source).unwrap();
        driver.poll_released().unwrap();
        driver.revoke().unwrap();
        // SAFETY: the model device is stopped and drained, so nothing still uses what this
        // releases.
        unsafe { driver.retire_peer_after_drain() }.unwrap();
        finish_stop(&mut driver, &model);
    }
}
#[test]
fn active_six_ghz_rnr_requires_programmed_probe_policy_and_reuses_refused_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::wifi::mediatek::mailbox::clc::remaining::tests::caps;
    let six = Channel {
        band: Band::Ghz6,
        number: 1,
        frequency_mhz: 5955,
    };
    let six5 = Channel {
        band: Band::Ghz6,
        number: 5,
        frequency_mhz: 5975,
    };
    let mut f = NativeFixture::new(Chip::Mt7925);
    f.model.lock().unwrap().capability_payload = Some(caps(Chip::Mt7925, [true; 3]));
    f.install().unwrap();
    f.boot(Chip::Mt7925);
    f.policy_channels(
        Chip::Mt7925,
        [true; 3],
        [
            vec![domain::Channel {
                hardware_value: 1,
                flags: WIDE,
            }],
            vec![domain::Channel {
                hardware_value: 36,
                flags: WIDE,
            }],
            vec![
                domain::Channel {
                    hardware_value: 1,
                    flags: SIX_GHZ,
                },
                domain::Channel {
                    hardware_value: 5,
                    flags: SIX_GHZ | reg::channel::NO_IR,
                },
            ],
        ],
    );
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe { block_on(f.controller.initialize_mac()) }.unwrap();
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe { block_on(f.controller.start(Band::Ghz5, 36, 5180)) }.unwrap();
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let interface =
        unsafe { block_on(f.controller.register_interface(profile().interface)) }.unwrap();
    f.model.lock().unwrap().client_execution = true;
    let NativeFixture {
        controller,
        model,
        bar,
        queues,
        memory,
        drops,
        ..
    } = f;
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let client = unsafe { Client::new(controller, interface) }.unwrap();
    let model = Model {
        response: model,
        bar,
        queues,
        memory,
        drops,
    };
    let country = three_band_country();
    let mut driver = ManagedClient::new(client, country, Environment::Indoor).unwrap();
    let target = scan::SixGhz {
        channel_index: 0,
        bssid: AP,
        short_ssid: Some(42),
        unsolicited_probe: true,
        psc_no_listen: false,
        psd_20: -70,
    };
    let before = model.response.lock().unwrap().client_commands.len();
    let custom =
        scan::Request::with_six_ghz(vec![six], vec![vec![]], vec![1, 1, 0xff], vec![target])
            .unwrap();
    assert!(matches!(
        driver.begin_scan(custom, 0, 1000),
        Err(PolicyError::Client(Error::Local(
            LocalError::UnsupportedProbeElements
        )))
    ));
    assert_eq!(driver.phase(), Phase::Idle);
    let request = scan::Request::with_six_ghz(
        vec![six, six5],
        vec![vec![]],
        vec![],
        vec![
            target,
            scan::SixGhz {
                channel_index: 1,
                bssid: crate::Mac([2, 0x42, 0, 0, 1, 1]),
                ..target
            },
        ],
    )
    .unwrap();
    assert!(matches!(
        driver.begin_scan(request, 1, 1000),
        Err(PolicyError::Client(Error::Scan(scan::Error::Regulatory)))
    ));
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    // Both probe-element and real NO_IR refusals leave the original owner
    // available. No copied RNR data can override programmed probe policy.
    let request = scan::Request::with_six_ghz(
        vec![
            Channel {
                band: Band::Ghz2,
                number: 1,
                frequency_mhz: 2412,
            },
            CHANNEL,
            six,
        ],
        vec![vec![]],
        vec![],
        vec![
            scan::SixGhz {
                channel_index: 2,
                ..target
            },
            scan::SixGhz {
                channel_index: 2,
                ..target
            },
        ],
    )
    .unwrap();
    driver.begin_scan(request, 2, 1000).unwrap();
    let commands = model.response.lock().unwrap().client_commands.clone();
    let body = &commands.last().unwrap()[48..];
    assert_eq!(&body[8..11], &[1, 2, 40]);
    let mut rest = &body[4..];
    let mut targets = Vec::new();
    let mut probes = Vec::new();
    while !rest.is_empty() {
        let n = usize::from(u16::from_le_bytes(rest[2..4].try_into().unwrap()));
        if rest[..2] == 11u16.to_le_bytes() {
            targets.push(rest[..n].to_vec());
        }
        if rest[..2] == 13u16.to_le_bytes() {
            probes.push(rest[..n].to_vec());
        }
        rest = &rest[n..];
    }
    assert_eq!(
        targets,
        vec![[vec![11, 0, 16, 0], AP.0.to_vec(), vec![1, 10, 0, 10, 0, 0]].concat()]
    );
    assert_eq!(probes.len(), 3);
    for (index, probe) in probes.iter().enumerate() {
        let band = [Band::Ghz2, Band::Ghz5, Band::Ghz6][index];
        assert_eq!(probe[6], index as u8 + 1);
        let local = driver.local_advertisement(band).unwrap();
        assert_eq!(
            &probe[8..],
            super::super::local::scan::elements(
                &local,
                super::super::local::scan::Selection::programmed(
                    driver
                        .client
                        .state()
                        .controller
                        .state()
                        .policy
                        .as_ref()
                        .unwrap()
                        .programmed_mt7925()
                        .unwrap()
                        .0,
                    band,
                    0x30b8
                )
            )
            .unwrap()
        );
        assert_eq!(&probe[probe.len() - 5..], &[255, 3, 59, 0xb8, 0x36]);
    }
    let mut beacon = response(
        0x80,
        &[
            0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0x11, 0, 0, 1, b'x', 255, 12, 36, 0, 0, 2, 0, 0, 0, 1,
            0, 1, 0, 6,
        ],
    );
    beacon[4..10].copy_from_slice(&crate::Mac::BROADCAST.0);
    let mut raw = incoming_bytes(&beacon, None, false);
    raw[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
    assert!(matches!(driver.poll(3,&Source).unwrap().as_slice(),[Event::Bss(b)]if b.channel==six));
    done(&model);
    driver.poll(4, &Source).unwrap();
    driver.revoke().unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_scan_after_drain() }.unwrap();
    finish_stop(&mut driver, &model);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}
#[test]
fn mismatched_country_returns_original_owner_and_sku_padding_is_not_power() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (client, model) = setup();
    let mut bytes = fixture();
    bytes[8..10].copy_from_slice(b"DE");
    let db = Database::parse(Arc::from(bytes.clone()), Sha256::digest(&bytes).into()).unwrap();
    let error = match ManagedClient::new(client, db.select(*b"DE").unwrap(), Environment::Indoor) {
        Err(error) => error,
        Ok(_) => panic!("wrong country admitted"),
    };
    assert!(matches!(error.error, PolicyError::CountryMismatch));
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    let mut corrected = ManagedClient::new(error.client, country(), Environment::Indoor).unwrap();
    corrected.revoke().unwrap();
    finish_stop(&mut corrected, &model);
    drop(corrected);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    let (client, model) = setup();
    // SKU -1.5dBm fits zero dBm but not a hypothetical copied
    // high-power value: directly verify every real used slot, not padding.
    assert_eq!(
        sku_max(
            &wifi::mediatek::mcu::power::Limits::uniform(-3).sku(Chip::Mt7925, Band::Ghz5),
            Band::Ghz5
        ),
        Some(-3)
    );
    let mut country = fixture();
    country[26..28].copy_from_slice(&0u16.to_be_bytes());
    let db = Database::parse(Arc::from(country.clone()), Sha256::digest(&country).into()).unwrap();
    let mut driver =
        ManagedClient::new(client, db.select(*b"US").unwrap(), Environment::Indoor).unwrap();
    driver.revoke().unwrap();
    finish_stop(&mut driver, &model);
}
