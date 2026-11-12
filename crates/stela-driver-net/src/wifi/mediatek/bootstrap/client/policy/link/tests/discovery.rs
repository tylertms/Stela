// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2020-2023 MediaTek Inc.
//! Owning frontend peers over modeled native commands and DMA queues.

use super::super::*;
use crate::wifi::mediatek::{
    bootstrap::{client::tests::*, tests::*},
    mailbox::clc::remaining::tests::caps,
    mcu::domain,
    ring::{rx::tests::device_receive, tests::Source},
};
use core::sync::atomic::Ordering;
use stela_driver_base::stepped::block_on;

mod custom_probe;

type TestLink = ManagedLink<Bar, Q, NativeGuard, Source>;
const SIX: Channel = Channel {
    band: Band::Ghz6,
    number: 1,
    frequency_mhz: 5955,
};
const SCAN_BUDGET_US: u64 = 10_000_000;

fn setup(six_capability: bool, six_programmed: bool, flags: u32) -> (TestLink, Model) {
    let mut f = NativeFixture::new(Chip::Mt7925);
    f.model.lock().unwrap().capability_payload =
        Some(caps(Chip::Mt7925, [true, true, six_capability]));
    f.install().unwrap();
    f.boot(Chip::Mt7925);
    f.policy_channels(
        Chip::Mt7925,
        [true, true, six_capability],
        [
            vec![domain::Channel {
                hardware_value: 1,
                flags: WIDE,
            }],
            vec![domain::Channel {
                hardware_value: 36,
                flags: WIDE,
            }],
            if six_programmed {
                vec![domain::Channel {
                    hardware_value: 1,
                    flags: SIX_GHZ | flags,
                }]
            } else {
                vec![]
            },
        ],
    );
    // SAFETY: model bootstrap and source policy programming completed.
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
    // It cannot override NIC, CLC, domain or per-channel power.
    let client = ManagedClient::new(client, three_band_country(), Environment::Indoor).unwrap();
    (ManagedLink::new(client, Source).unwrap(), model)
}

fn active_request() -> scan::Request {
    scan::Request::with_six_ghz(
        vec![SIX],
        vec![vec![]],
        vec![],
        vec![scan::SixGhz {
            channel_index: 0,
            bssid: AP,
            short_ssid: Some(42),
            unsolicited_probe: true,
            psc_no_listen: false,
            psd_20: -70,
        }],
    )
    .unwrap()
}
fn passive_request() -> scan::Request {
    scan::Request::new(vec![SIX], vec![], vec![]).unwrap()
}
fn queue_heads(model: &Model) -> Vec<(u32, u16)> {
    model
        .queues
        .iter()
        .map(|(&register, state)| (register, state.lock().unwrap().head))
        .collect()
}
fn tlv(command: &[u8], tag: u16) -> &[u8] {
    let mut rest = &command[48 + 4..];
    while !rest.is_empty() {
        let length = usize::from(u16::from_le_bytes(rest[2..4].try_into().unwrap()));
        assert!(length >= 4 && length <= rest.len());
        if rest[..2] == tag.to_le_bytes() {
            return &rest[..length];
        }
        rest = &rest[length..];
    }
    panic!("missing native scan TLV {tag}");
}
fn beacon(model: &Model, timestamp: u64) {
    let mut body = timestamp.to_le_bytes().to_vec();
    body.extend_from_slice(&[
        100, 0, 0x11, 0, 0, 1, b'x',
        // Complete HE Operation with six-GHz information, primary channel1.
        255, 12, 36, 0, 0, 2, 0, 0, 0, 1, 0, 1, 0, 6,
    ]);
    let mut bytes = response(0x80, &body);
    bytes[4..10].copy_from_slice(&Mac::BROADCAST.0);
    let mut raw = incoming_bytes(&bytes, None, false);
    // MT7925 RXD3_NORMAL_CH_FREQ uses181 for six-GHz channel1.
    raw[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
}
fn terminal(link: &mut TestLink, model: &Model, outcome: scan::Outcome) {
    super::super::super::tests::done(model);
    link.service().unwrap();
    assert_eq!(link.phase(), Phase::ScanCompleted);
    // SAFETY: terminal model scan, restored radio and all old native users drained.
    assert_eq!(unsafe { link.retire_scan_after_drain() }.unwrap(), outcome);
    assert_eq!(link.phase(), Phase::Idle);
}
fn stop(mut link: TestLink, model: &Model) {
    link.stop().unwrap();
    assert_eq!(link.phase(), Phase::ControllerDrain);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: independent model firmware, DMA, IRQ and deferred-user fence.
    unsafe { link.stop_after_quiesce() }.unwrap();
    drop(link);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn programmed_six_ghz_frontend_publishes_native_rnr_probe_and_routes_dma_bss() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(true, true, 0);
    let local = link.local_advertisement(Band::Ghz6).unwrap();
    assert_eq!(local.capabilities().band(), Band::Ghz6);
    assert_eq!(local.capabilities().he_6ghz(), Some(0x30b8));
    let ticket = link.begin_scan(active_request(), SCAN_BUDGET_US).unwrap();
    let command = model
        .response
        .lock()
        .unwrap()
        .client_commands
        .last()
        .unwrap()
        .clone();
    assert_eq!(&tlv(&command, 1)[4..7], &[1, 2, 40]);
    assert_eq!(
        &tlv(&command, 11)[4..14],
        &[AP.0.as_slice(), &[1, 10, 0, 10]].concat()
    );
    assert_eq!(&tlv(&command, 12)[4..10], &[4, 1, 0, 0, 3, 1]);
    let probe = tlv(&command, 13);
    assert_eq!(probe[6], 3);
    assert_eq!(&probe[8..18], &[1, 8, 12, 18, 24, 36, 48, 72, 96, 108]);
    assert_eq!(&probe[probe.len() - 5..], &[255, 3, 59, 0xb8, 0x36]);
    beacon(&model, 42);
    link.service().unwrap();
    let notice = link.take_notice().unwrap();
    assert_eq!(notice.association, None);
    assert!(matches!(notice.event, LinkEvent::Controller(Event::Bss(b))
        if b.ticket == ticket && b.channel == SIX && b.observed.band == Band::Ghz6
            && b.timestamp == 42 && b.bssid == AP && b.ssid.as_deref() == Some(b"x")));
    let handle = link.bss().next().unwrap().0;
    terminal(&mut link, &model, scan::Outcome::Completed);
    let mut join = join();
    join.channel = SIX;
    let mut profile = profile();
    profile.channel = SIX;
    let commands = model.response.lock().unwrap().client_commands.len();
    assert!(matches!(
        link.join(handle, join, profile),
        Err(LinkControlError::UnsupportedProfile)
    ));
    assert_eq!(
        model.response.lock().unwrap().client_commands.len(),
        commands
    );
    assert!(!link.carrier());
    assert_eq!(link.transmit(&eth(0x0800)), Err(LinkError::NoCarrier));
    assert!(matches!(
        link.send_control(&eth(0x888e), None),
        Err(LinkControlError::WrongPhase)
    ));
    stop(link, &model);
}

#[test]
fn no_ir_refuses_active_publication_but_passive_cancel_keeps_real_drain_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(true, true, 2);
    let before = model.response.lock().unwrap().client_commands.len();
    let heads = queue_heads(&model);
    let syncs = model.memory.lock().unwrap().syncs.len();
    assert!(matches!(
        link.begin_scan(active_request(), SCAN_BUDGET_US),
        Err(LinkControlError::Policy(PolicyError::Client(Error::Scan(
            scan::Error::Regulatory
        ))))
    ));
    assert_eq!(link.phase(), Phase::Idle);
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    assert_eq!(queue_heads(&model), heads);
    assert_eq!(model.memory.lock().unwrap().syncs.len(), syncs);
    link.begin_scan(passive_request(), SCAN_BUDGET_US).unwrap();
    let command = model
        .response
        .lock()
        .unwrap()
        .client_commands
        .last()
        .unwrap()
        .clone();
    assert_eq!(&tlv(&command, 1)[4..7], &[0, 0, 32]);
    assert_eq!(&tlv(&command, 12)[8..10], &[3, 1]);
    link.cancel_scan().unwrap();
    assert_eq!(link.phase(), Phase::Scanning);
    // A cancel ACK is not terminal scan completion or old-user drain.
    assert!(link.begin_scan(passive_request(), SCAN_BUDGET_US).is_err());
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
    terminal(&mut link, &model, scan::Outcome::Aborted);
    link.begin_scan(passive_request(), SCAN_BUDGET_US).unwrap();
    terminal(&mut link, &model, scan::Outcome::Completed);
    stop(link, &model);
}

#[test]
fn six_ghz_notice_backpressure_preserves_next_native_rx_descriptor() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(true, true, 0);
    link.begin_scan(passive_request(), SCAN_BUDGET_US).unwrap();
    for timestamp in 0..NOTICES {
        beacon(&model, timestamp as u64);
        link.service().unwrap();
    }
    assert_eq!(link.queues().notices, NOTICES);
    assert!(link.queues().backpressured);
    beacon(&model, NOTICES as u64);
    let (head, index, descriptor) = {
        let q = model.queues[&0xd4520].lock().unwrap();
        let memory = model.memory.lock().unwrap();
        let offset = usize::from((q.index + q.count - 1) % q.count) * 16;
        let pointer = (memory.mappings[&q.base].1 + offset) as *const u8;
        // The mapped device descriptor keeps DDONE and the buffer PA.
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
        // accesses it concurrently.
        let descriptor = unsafe { core::slice::from_raw_parts(pointer, 16) }.to_vec();
        assert_ne!(
            u32::from_le_bytes(descriptor[4..8].try_into().unwrap()) & (1 << 31),
            0
        );
        (q.head, q.index, descriptor)
    };
    link.service().unwrap();
    {
        let q = model.queues[&0xd4520].lock().unwrap();
        let memory = model.memory.lock().unwrap();
        assert_eq!((q.head, q.index), (head, index));
        let offset = usize::from((index + q.count - 1) % q.count) * 16;
        let pointer = (memory.mappings[&q.base].1 + offset) as *const u8;
        assert_eq!(
            // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
            // nothing accesses it concurrently.
            unsafe { core::slice::from_raw_parts(pointer, 16) },
            descriptor
        );
    }
    let mut observed = Vec::new();
    if let LinkEvent::Controller(Event::Bss(bss)) = link.take_notice().unwrap().event {
        observed.push(bss.timestamp);
    } else {
        panic!("original BSS notice lost");
    }
    link.service().unwrap();
    assert_ne!(model.queues[&0xd4520].lock().unwrap().head, head);
    while let Some(notice) = link.take_notice() {
        match notice.event {
            LinkEvent::Controller(Event::Bss(bss)) => observed.push(bss.timestamp),
            _ => panic!("unexpected discovery notice"),
        }
    }
    assert_eq!(observed, (0..=NOTICES as u64).collect::<Vec<_>>());
    assert!(!link.carrier());
    terminal(&mut link, &model, scan::Outcome::Completed);
    stop(link, &model);
}

#[test]
fn six_ghz_nic_and_programmed_band_requirements_are_not_frontend_grants() {
    if crate::test_process::isolate_model() {
        return;
    }
    for available in [false, true] {
        let (mut link, model) = setup(available, false, 0);
        if available {
            assert_eq!(
                link.local_advertisement(Band::Ghz6)
                    .unwrap()
                    .capabilities()
                    .band(),
                Band::Ghz6
            );
        } else {
            assert!(matches!(
                link.local_advertisement(Band::Ghz6),
                Err(LinkControlError::Policy(PolicyError::Client(Error::Local(
                    LocalError::UnsupportedBand
                ))))
            ));
        }
        let before = model.response.lock().unwrap().client_commands.len();
        let heads = queue_heads(&model);
        let syncs = model.memory.lock().unwrap().syncs.len();
        for request in [active_request(), passive_request()] {
            assert!(matches!(
                link.begin_scan(request, SCAN_BUDGET_US),
                Err(LinkControlError::Policy(PolicyError::Client(
                    Error::WrongProfile
                )))
            ));
            assert_eq!(link.phase(), Phase::Idle);
            assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
            assert_eq!(queue_heads(&model), heads);
            assert_eq!(model.memory.lock().unwrap().syncs.len(), syncs);
        }
        assert!(!link.carrier());
        stop(link, &model);
    }
}
