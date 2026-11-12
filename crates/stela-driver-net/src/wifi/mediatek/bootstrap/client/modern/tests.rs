extern crate std;
use crate::wifi::Band;
use crate::wifi::mediatek::bootstrap::{
    client::tests::{setup, *},
    tests::*,
};
use crate::wifi::mediatek::{
    mailbox::tests::*,
    ring::{
        data::Receipt,
        rx::tests::device_receive,
        tests::{Memory, Source},
    },
};
use crate::wifi::regulatory::{self as reg, AssociationUse};
use crate::wifi::station::phy::{Mode, Width};
use crate::wifi::{
    self,
    mediatek::bootstrap::client::{
        Client, Error as ClientError, Event, Msdu, Phase as ClientPhase, PmfJoin,
    },
    scan,
    security::ReplaySeed,
    station::{self, Join, data::TxClass, wire::AssociationRequest},
};
use alloc::rc::Rc;
use alloc::{sync::Arc, vec::Vec};
use core::cell::{Cell, RefCell};
use core::sync::atomic::Ordering;
use std::sync::Mutex;

fn append_ie(out: &mut Vec<u8>, id: u8, bytes: &[u8]) {
    out.extend_from_slice(&[id, bytes.len() as u8]);
    out.extend_from_slice(bytes);
}
fn append_extension(out: &mut Vec<u8>, id: u8, bytes: &[u8]) {
    let mut data = vec![id];
    data.extend_from_slice(bytes);
    append_ie(out, 255, &data);
}
pub(in super::super) fn modern_inputs(
    driver: &Client<Bar, Q, NativeGuard>,
) -> (Join, scan::result::Bss) {
    modern_inputs_cipher(driver, 4)
}
pub(in super::super) fn modern_inputs_cipher(
    driver: &Client<Bar, Q, NativeGuard>,
    suite: u8,
) -> (Join, scan::result::Bss) {
    let actual = driver.local_advertisement(Band::Ghz5).unwrap();
    let mut local = vec![0, 1, b'x'];
    local.extend_from_slice(actual.elements());
    // Real OS-controlled WPA2 exchange remains outside this software AP.
    append_ie(
        &mut local,
        48,
        &[
            1, 0, 0, 15, 172, suite, 1, 0, 0, 15, 172, suite, 1, 0, 0, 15, 172, 2,
        ],
    );
    let mut join = join();
    join.request = AssociationRequest::new_modern_with_local(
        0x11,
        10,
        local.clone(),
        actual.capabilities().clone(),
        // A connac2 chip advertises HE at most.
        if actual.capabilities().eht().is_some() {
            Mode::Eht
        } else {
            Mode::He
        },
        Width::Mhz80,
    )
    .unwrap();
    let mut ap = Vec::new();
    let mut at = 0;
    while at < local.len() {
        let n = usize::from(local[at + 1]);
        let bytes = &local[at + 2..at + 2 + n];
        if local[at] == 221 {
            append_ie(
                &mut ap,
                221,
                &[
                    0, 0x50, 0xf2, 2, 1, 1, 0x80, 0, 3, 0xa4, 0, 0, 0x27, 0xa4, 0, 0, 0x42, 0x43,
                    94, 0, 0x62, 0x32, 47, 0,
                ],
            );
        } else if local[at] == 1 {
            append_ie(&mut ap, 1, &[0x8c, 18, 0x98, 36, 0xb0, 72, 96, 108]);
        } else {
            append_ie(&mut ap, local[at], bytes);
        }
        at += 2 + n;
    }
    let mut ht = [0; 22];
    ht[0] = 36;
    ht[1] = 5;
    append_ie(&mut ap, 61, &ht);
    append_ie(&mut ap, 192, &[1, 42, 0, 0xff, 0xff]);
    append_extension(&mut ap, 36, &[3, 0x20, 0, 0x2a, 0xff, 0xff]);
    append_extension(&mut ap, 106, &[1, 0, 0, 0, 0, 2, 42, 0]);
    append_ie(&mut ap, 5, &[0, 2, 0, 0]);
    let mut scanner = scan::Scanner::new(LOCAL).unwrap();
    let ticket = scanner
        .begin(
            scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
            0,
            100,
            &mut Policy(true),
        )
        .unwrap();
    let bss = scan::result::Bss {
        ticket,
        kind: scan::result::Kind::Beacon,
        bssid: AP,
        transmitter: AP,
        profile: None,
        timestamp: 42,
        beacon_interval: 100,
        capabilities: 0x11,
        channel: CHANNEL,
        observed: wifi::RadioStatus {
            band: Band::Ghz5,
            channel: 36,
            frequency_mhz: 5180,
            signal_dbm: Some(-40),
            chain_signal_dbm: [None; 4],
            rate: wifi::RawRate {
                layout: 0x7925,
                words: [0; 4],
            },
            rx_rate: None,
            tsf: None,
        },
        signal_dbm: Some(-40),
        ssid: Some(vec![b'x']),
        elements: ap,
    };
    (join, bss)
}

fn awaiting() -> (Client<Bar, Q, NativeGuard>, Model, usize) {
    awaiting_cipher(4)
}
fn awaiting_cipher(suite: u8) -> (Client<Bar, Q, NativeGuard>, Model, usize) {
    let (mut driver, model) = setup();
    let (join, beacon) = modern_inputs_cipher(&driver, suite);
    let response_elements = beacon.elements.clone();
    driver
        .join_modern(join, beacon, profile(), 100, &Source, &mut Policy(true))
        .unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    driver.poll(101, &Source, &mut Policy(true)).unwrap();
    driver.poll_released().unwrap();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    driver.poll(102, &Source, &mut Policy(true)).unwrap();
    let assoc = finish_tx(&model);
    assert_eq!(assoc[0][..2], [0, 0]);
    driver.poll(103, &Source, &mut Policy(true)).unwrap();
    driver.poll_released().unwrap();
    let commands_before = model.response.lock().unwrap().client_commands.len();
    let mut assoc_body = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc_body.extend_from_slice(&response_elements);
    incoming(&model, &response(0x10, &assoc_body), None, false);
    driver.poll(104, &Source, &mut Policy(true)).unwrap();
    (driver, model, commands_before)
}

#[test]
fn modern_current_context_waits_for_real_admission_without_legacy_configuration_ack() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, commands_before) = awaiting();
    assert_eq!(driver.phase(), ClientPhase::AwaitingModernAdmission);
    assert_eq!(driver.station().phase(), station::Phase::Configuring);
    assert_eq!(
        driver.modern_context().unwrap().definition().width,
        Width::Mhz80
    );
    assert_eq!(driver.pending_modern_requests(), 7);
    assert_eq!(
        model.response.lock().unwrap().client_commands.len(),
        commands_before
    );
    assert!(matches!(
        driver.install_key(true, 0, &[4; 16], ReplaySeed::ZERO),
        Err(ClientError::WrongPhase)
    ));
    assert!(
        driver
            .authorize_port(driver.station().association_epoch().unwrap())
            .is_err()
    );
    // Drop preserves the real current context, original peer and all DMA.
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn modern_configuration_and_secure_qos_publication_hold_both_permissions() {
    if crate::test_process::isolate_model() {
        return;
    }
    use wifi::regulatory::AssociationUse;
    let (mut driver, model, before) = awaiting();
    let context = driver.modern_context().unwrap().clone();
    let (mut authority, state) = authority(&driver, &model);
    let revoked = admit(&mut authority, &driver, AssociationUse::Configure);
    state.allowed.set(false);
    assert!(matches!(
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { driver.configure_modern(&mut authority, revoked, 105) },
        Err(ClientError::Regulatory(_))
    ));
    assert_eq!(driver.phase(), ClientPhase::AwaitingModernAdmission);
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    state.allowed.set(true);
    let permit = admit(&mut authority, &driver, AssociationUse::Configure);
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe { driver.configure_modern(&mut authority, permit, 106) }.unwrap();
    assert_eq!(driver.phase(), ClientPhase::Associated);
    assert_eq!(driver.station().phase(), station::Phase::Associated);
    assert_eq!(driver.completed_modern_requests(), 7);
    assert_eq!(driver.pending_modern_requests(), 0);
    assert!(state.windows.borrow().iter().any(|(a, b)| b - a == 7));
    let epoch = driver.station().association_epoch().unwrap();
    modern_keys(&mut driver, &mut authority);
    driver.authorize_port(epoch).unwrap();
    assert!(matches!(
        driver.transmit(
            epoch,
            &eth(0x0800),
            TxClass::Data,
            &Source,
            &mut Policy(true)
        ),
        Err(ClientError::ModernAdmissionRequired)
    ));

    let stale = admit(&mut authority, &driver, AssociationUse::Data);
    state.revision.set(2);
    let head = model.queues[&0xd4300].lock().unwrap().head;
    let allocations = state.allocations.get();
    assert!(matches!(
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe {
            driver.transmit_modern(
                &mut authority,
                stale,
                epoch,
                Msdu {
                    ethernet: &eth(0x0800),
                    class: TxClass::Data,
                    tid: Some(6),
                },
                &GuardedSource(state.clone()),
                &mut Policy(true),
            )
        },
        Err(ClientError::Regulatory(_))
    ));
    assert_eq!(state.allocations.get(), allocations);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, head);
    let permit = admit(&mut authority, &driver, AssociationUse::Data);
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    let receipt = unsafe {
        driver.transmit_modern(
            &mut authority,
            permit,
            epoch,
            Msdu {
                ethernet: &eth(0x0800),
                class: TxClass::Data,
                tid: Some(6),
            },
            &GuardedSource(state.clone()),
            &mut Policy(true),
        )
    }
    .unwrap();
    assert_eq!(state.allocations.get() - allocations, 2);
    assert!(driver.poll_released().unwrap().is_empty());
    let transmitted = finish_tx(&model);
    assert_eq!(transmitted.len(), 1);
    assert_eq!(&transmitted[0][..2], &[0x88, 0x41]);
    assert_eq!(&transmitted[0][24..28], &[6, 0, 0xaa, 0xaa]);
    driver
        .poll(1_000_000 + 110, &Source, &mut Policy(true))
        .unwrap();
    let released = driver.poll_released().unwrap();
    assert_eq!(released.len(), 1);
    assert_eq!(released[0].receipt, receipt);
    assert!(core::ptr::eq(
        released[0].context.association_context().unwrap(),
        context.as_ref()
    ));
    drop(released);
    let mut packet = data(false);
    packet[0] |= 0x80;
    packet.splice(24..24, [6, 0]);
    incoming(&model, &packet, Some((1, 0, 9)), false);
    assert!(matches!(
        driver
            .poll(1_000_000 + 111, &Source, &mut Policy(true))
            .unwrap()
            .as_slice(),
        [Event::Ingress(station::data::Ingress::Data(_))]
    ));
    incoming(&model, &packet, Some((1, 0, 9)), false);
    assert!(matches!(
        driver.poll(1_000_000 + 112, &Source, &mut Policy(true)),
        Err(ClientError::Aggregate(
            wifi::security::aggregate::Error::Security(wifi::security::Error::Replay)
        ))
    ));
    // A separate TID starts from its original slot's independent floor.
    packet[24] = 5;
    incoming(&model, &packet, Some((1, 0, 1)), false);
    driver
        .poll(1_000_000 + 113, &Source, &mut Policy(true))
        .unwrap();
    driver.disconnect().unwrap();
    assert_eq!(driver.phase(), ClientPhase::NeedsDrain);
    assert!(Arc::ptr_eq(driver.modern_context().unwrap(), &context));
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    assert!(driver.modern_context().is_none());
    driver.stop().unwrap();
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.stop_after_quiesce() }.unwrap();
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}

fn secure_modern() -> (
    Client<Bar, Q, NativeGuard>,
    Model,
    Owner,
    alloc::rc::Rc<AuthorityState>,
) {
    let (mut driver, model, _) = awaiting();
    let (mut authority, state) = authority(&driver, &model);
    let permit = admit(
        &mut authority,
        &driver,
        wifi::regulatory::AssociationUse::Configure,
    );
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe { driver.configure_modern(&mut authority, permit, 105) }.unwrap();
    modern_keys(&mut driver, &mut authority);
    driver
        .authorize_port(driver.station().association_epoch().unwrap())
        .unwrap();
    (driver, model, authority, state)
}

/// Shared native setup for the TX cancellation peer controls.
pub(in crate::wifi::mediatek::bootstrap::client) struct TxFixture {
    pub(in crate::wifi::mediatek::bootstrap::client) driver: Client<Bar, Q, NativeGuard>,
    pub(in crate::wifi::mediatek::bootstrap::client) model: Model,
    authority: Owner,
    state: Rc<AuthorityState>,
}
impl TxFixture {
    pub(in crate::wifi::mediatek::bootstrap::client) fn new() -> Self {
        let (driver, model, authority, state) = secure_modern();
        Self {
            driver,
            model,
            authority,
            state,
        }
    }
    pub(in crate::wifi::mediatek::bootstrap::client) fn revoke(&self) {
        self.state.allowed.set(false);
    }
    pub(in crate::wifi::mediatek::bootstrap::client) fn progress(
        &mut self,
        now: u64,
    ) -> Result<(), ClientError> {
        let permit = admit(
            &mut self.authority,
            &self.driver,
            AssociationUse::Management,
        );
        // SAFETY: these guards check only this stateful simulated requester.
        unsafe {
            self.driver.progress_block_ack(
                &mut self.authority,
                permit,
                &GuardedSource(self.state.clone()),
                now,
            )
        }
    }
    pub(in crate::wifi::mediatek::bootstrap::client) fn try_transmit(
        &mut self,
    ) -> Result<Receipt, ClientError> {
        let permit = admit(&mut self.authority, &self.driver, AssociationUse::Data);
        let epoch = self.driver.station().association_epoch().unwrap();
        // SAFETY: same-requester model guards and kept native policy.
        unsafe {
            self.driver.transmit_modern(
                &mut self.authority,
                permit,
                epoch,
                Msdu {
                    ethernet: &eth(0x0800),
                    class: TxClass::Data,
                    tid: Some(6),
                },
                &GuardedSource(self.state.clone()),
                &mut Policy(true),
            )
        }
    }
    pub(in crate::wifi::mediatek::bootstrap::client) fn transmit(&mut self) {
        let permit = admit(&mut self.authority, &self.driver, AssociationUse::Data);
        let epoch = self.driver.station().association_epoch().unwrap();
        // SAFETY: the model's same-requester guards enclose this publication.
        unsafe {
            self.driver.transmit_modern(
                &mut self.authority,
                permit,
                epoch,
                Msdu {
                    ethernet: &eth(0x0800),
                    class: TxClass::Data,
                    tid: Some(6),
                },
                &GuardedSource(self.state.clone()),
                &mut Policy(true),
            )
        }
        .unwrap();
    }
}

fn modern_keys(driver: &mut Client<Bar, Q, NativeGuard>, authority: &mut Owner) {
    for (pairwise, index, material) in [(true, 0, 1), (false, 2, 2)] {
        let permit = admit(
            authority,
            driver,
            wifi::regulatory::AssociationUse::Management,
        );
        // SAFETY: the test's model owns this key slot alone; no other worker programs the
        // hardware key.
        unsafe {
            driver.install_key_modern(
                authority,
                permit,
                pairwise,
                index,
                &[material; 16],
                ReplaySeed::ZERO,
            )
        }
        .unwrap();
    }
}

#[test]
fn mixed_qos_and_nonqos_share_original_key_counter_zero() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, _authority, _state) = secure_modern();
    // Real native DMA key binding and complete-owned single-parent replay
    // gate, with distinct source sequence queues and one shared PN domain.
    for group in [false, true] {
        for (step, kind, pn, accepted) in [
            (0, None, 10, true),
            (1, Some(0), 10, false),
            (2, Some(0), 11, true),
            (3, None, 11, false),
            (4, Some(1), 1, true),
            (5, None, 12, true),
        ] {
            let mut packet = data(group);
            packet[22..24]
                .copy_from_slice(&((step + 1u16 + if group { 6 } else { 0 }) << 4).to_le_bytes());
            if let Some(tid) = kind {
                packet[0] |= 0x80;
                packet.splice(24..24, [tid, 0]);
            }
            incoming(
                &model,
                &packet,
                Some((1, if group { 2 } else { 0 }, pn)),
                false,
            );
            let result = driver.poll(
                1_000_000 + 110 + u64::from(step) + if group { 6 } else { 0 },
                &Source,
                &mut Policy(true),
            );
            if accepted {
                assert!(matches!(
                    result.unwrap().as_slice(),
                    [Event::Ingress(station::data::Ingress::Data(_))]
                ));
            } else {
                assert!(matches!(
                    result,
                    Err(ClientError::Aggregate(
                        wifi::security::aggregate::Error::Security(wifi::security::Error::Replay)
                    ))
                ));
            }
        }
    }
    driver.disconnect().unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    driver.stop().unwrap();
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.stop_after_quiesce() }.unwrap();
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}

fn split_bytes(format: u8, pn: Option<u64>, source: u8) -> Vec<u8> {
    let mut payload = Vec::new();
    payload.extend_from_slice(&LOCAL.0);
    payload.extend_from_slice(&[4, source, 2, 3, 4, 5]);
    payload.extend_from_slice(&11u16.to_be_bytes());
    payload.extend_from_slice(&[0xaa, 0xaa, 3, 0, 0, 0, 8, 0, 1, 2, 3]);
    let mut frame = response(0x4288, &[]);
    frame[16..22].copy_from_slice(&[4, source, 2, 3, 4, 5]);
    frame[22..24].copy_from_slice(&(17u16 << 4).to_le_bytes());
    frame.extend_from_slice(&[6 | 0x80, 0]);
    frame.extend_from_slice(&payload);
    let mut bytes = incoming_bytes(&frame, Some((1, 0, pn.unwrap_or(0))), false);
    // Split RXD carries the source two-byte pad after the MAC header.
    bytes.splice(48 + 26..48 + 26, [0, 0]);
    bytes[16..20].copy_from_slice(&u32::from(format).to_le_bytes());
    if pn.is_none() {
        let mut r1 = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
        r1 &= !(1 << 16);
        bytes[4..8].copy_from_slice(&r1.to_le_bytes());
        bytes.drain(32..48);
    }
    let len = bytes.len() as u32;
    bytes[..4].copy_from_slice(&((2 << 27) | len).to_le_bytes());
    bytes
}

#[test]
fn queue_bound_split_parent_uses_one_replay_commit_and_original_member_sources() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, _, _) = secure_modern();
    let head = split_bytes(3, Some(20), 1);
    let tail = split_bytes(1, None, 2);
    device_receive(&model.queues[&0xd4500], &head, true, 0);
    assert!(
        driver
            .poll(1_000_000 + 110, &Source, &mut Policy(true))
            .unwrap()
            .is_empty()
    );
    // The same apparent SSN/TID on another queue cannot complete it or
    // inherit its PN. A one-record parent needs its own observed PN.
    device_receive(&model.queues[&0xd4520], &tail, true, 0);
    assert!(matches!(
        driver.poll(1_000_000 + 111, &Source, &mut Policy(true)),
        Err(ClientError::Aggregate(
            wifi::security::aggregate::Error::Security(wifi::security::Error::MissingPacketNumber)
        ))
    ));
    device_receive(&model.queues[&0xd4500], &tail, true, 0);
    let events = driver
        .poll(1_000_000 + 112, &Source, &mut Policy(true))
        .unwrap();
    assert_eq!(events.len(), 2);
    for (index, event) in events.iter().enumerate() {
        let Event::Ingress(station::data::Ingress::Data(frame)) = event else {
            panic!("actual split data");
        };
        assert_eq!(&frame[..6], &LOCAL.0);
        assert_eq!(frame[7], index as u8 + 1);
        assert_eq!(&frame[12..], &[8, 0, 1, 2, 3]);
    }
    // Delivering two authenticated constituents consumed only the parent PN.
    device_receive(&model.queues[&0xd4500], &head, true, 0);
    driver
        .poll(1_000_000 + 113, &Source, &mut Policy(true))
        .unwrap();
    device_receive(&model.queues[&0xd4500], &tail, true, 0);
    assert!(matches!(
        driver.poll(1_000_000 + 114, &Source, &mut Policy(true)),
        Err(ClientError::Aggregate(
            wifi::security::aggregate::Error::Security(wifi::security::Error::Replay)
        ))
    ));
    // A malformed later constituent refuses the whole group before replay;
    // the same parent PN remains available for a valid replacement.
    let head = split_bytes(3, Some(21), 1);
    let mut bad = split_bytes(1, Some(22), 2);
    device_receive(&model.queues[&0xd4500], &head, true, 0);
    driver
        .poll(1_000_000 + 115, &Source, &mut Policy(true))
        .unwrap();
    device_receive(&model.queues[&0xd4500], &bad, true, 0);
    assert!(matches!(
        driver.poll(1_000_000 + 116, &Source, &mut Policy(true)),
        Err(ClientError::IncompleteParent)
    ));
    bad = split_bytes(1, None, 2);
    device_receive(&model.queues[&0xd4500], &head, true, 0);
    driver
        .poll(1_000_000 + 117, &Source, &mut Policy(true))
        .unwrap();
    device_receive(&model.queues[&0xd4500], &bad, true, 0);
    assert_eq!(
        driver
            .poll(1_000_000 + 118, &Source, &mut Policy(true))
            .unwrap()
            .len(),
        2
    );
}

fn qos_packet(sequence: u16) -> Vec<u8> {
    let mut packet = data(false);
    packet[0] |= 0x80;
    packet[22..24].copy_from_slice(&(sequence << 4).to_le_bytes());
    packet.splice(24..24, [6, 0]);
    packet
}

#[test]
fn ba_enable_sequence_replays_parent_once() {
    if crate::test_process::isolate_model() {
        return;
    }
    use wifi::regulatory::AssociationUse;
    let (mut driver, model, mut authority, state) = secure_modern();
    let parameters: u16 = 1 | 2 | (6 << 2) | (8 << 6);
    let mut body = vec![3, 0, 7];
    body.extend_from_slice(&parameters.to_le_bytes());
    body.extend_from_slice(&0u16.to_le_bytes());
    body.extend_from_slice(&(17u16 << 4).to_le_bytes());
    incoming(&model, &response(0xd0, &body), None, false);
    driver
        .poll(1_000_000 + 110, &Source, &mut Policy(true))
        .unwrap();
    assert!(driver.pending_block_ack());
    assert!(driver.station().active_rx_block_ack(6).is_none());
    assert!(
        !model
            .response
            .lock()
            .unwrap()
            .receive_ba
            .contains_key(&(1, 6))
    );
    let permit = admit(&mut authority, &driver, AssociationUse::Management);
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe {
        driver.progress_block_ack(
            &mut authority,
            permit,
            &GuardedSource(state.clone()),
            1_000_111,
        )
    }
    .unwrap();
    let session = driver.station().active_rx_block_ack(6).unwrap().clone();
    assert_eq!(session.start_sequence(), 17);
    assert!(
        model
            .response
            .lock()
            .unwrap()
            .receive_ba
            .contains_key(&(1, 6))
    );
    assert!(driver.pending_block_ack());
    let permit = admit(&mut authority, &driver, AssociationUse::Management);
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    unsafe {
        driver.progress_block_ack(
            &mut authority,
            permit,
            &GuardedSource(state.clone()),
            1_000_112,
        )
    }
    .unwrap();
    let sent = finish_tx(&model);
    assert_eq!(&sent[0][24..29], &[3, 1, 7, 0, 0]);
    assert!(!driver.pending_block_ack());
    driver
        .poll(1_000_000 + 113, &Source, &mut Policy(true))
        .unwrap();
    driver.poll_released().unwrap();
    // Deliver SSN17 to start; receiving19 before18 must not consume PN3 yet.
    incoming(&model, &qos_packet(17), Some((1, 0, 1)), false);
    assert_eq!(
        driver
            .poll(1_000_000 + 114, &Source, &mut Policy(true))
            .unwrap()
            .len(),
        1
    );
    incoming(&model, &qos_packet(19), Some((1, 0, 3)), false);
    assert!(
        driver
            .poll(1_000_000 + 115, &Source, &mut Policy(true))
            .unwrap()
            .is_empty()
    );
    assert_eq!(driver.next_receive_deadline(), Some(1_040_115));
    incoming(&model, &qos_packet(18), Some((1, 0, 2)), false);
    assert_eq!(
        driver
            .poll(1_000_000 + 116, &Source, &mut Policy(true))
            .unwrap()
            .len(),
        2
    );
    assert_eq!(driver.next_receive_deadline(), None);
    incoming(&model, &qos_packet(21), Some((1, 0, 5)), false);
    assert!(
        driver
            .poll(1_000_000 + 117, &Source, &mut Policy(true))
            .unwrap()
            .is_empty()
    );
    assert!(
        driver
            .poll(1_040_117, &Source, &mut Policy(true))
            .unwrap()
            .is_empty()
    );
    // Source timeout is strict after, and release then advances replay.
    assert_eq!(
        driver
            .poll(1_080_118, &Source, &mut Policy(true))
            .unwrap()
            .len(),
        1
    );
    // Peer DELBA invalidates before firmware stop; stop never needs a
    // new transmit permission even after model policy revocation.
    let mut delete = vec![3, 2];
    delete.extend_from_slice(&((6u16 << 12) | 0x0800).to_le_bytes());
    delete.extend_from_slice(&37u16.to_le_bytes());
    incoming(&model, &response(0xd0, &delete), None, false);
    driver.poll(1_080_119, &Source, &mut Policy(true)).unwrap();
    assert!(driver.station().validate_rx_block_ack(&session).is_err());
    state.allowed.set(false);
    driver.progress_block_ack_stop().unwrap();
    assert!(!driver.pending_block_ack());
    assert!(
        !model
            .response
            .lock()
            .unwrap()
            .receive_ba
            .contains_key(&(1, 6))
    );
    assert_eq!(driver.next_receive_deadline(), None);
}

#[test]
fn ccmp_and_gcmp_keys_bind_receive_layouts() {
    if crate::test_process::isolate_model() {
        return;
    }
    use wifi::regulatory::AssociationUse;
    for (suite, firmware, length) in [(4, 4u32, 16), (10, 10, 32), (8, 11, 16), (9, 12, 32)] {
        let (mut driver, model, _) = awaiting_cipher(suite);
        let (mut authority, state) = authority(&driver, &model);
        let permit = admit(&mut authority, &driver, AssociationUse::Configure);
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe { driver.configure_modern(&mut authority, permit, 105) }.unwrap();
        assert!(matches!(
            driver.install_key(true, 0, &[1; 16], ReplaySeed::ZERO),
            Err(ClientError::ModernAdmissionRequired)
        ));
        for (pairwise, index, value) in [(true, 0, 1), (false, 2, 2)] {
            let material = vec![value; length];
            let permit = admit(&mut authority, &driver, AssociationUse::Management);
            // SAFETY: the test's model owns this key slot alone; no other worker programs the
            // hardware key.
            let key = unsafe {
                driver.install_key_modern(
                    &mut authority,
                    permit,
                    pairwise,
                    index,
                    &material,
                    ReplaySeed::ZERO,
                )
            }
            .unwrap();
            assert_eq!(key.index(), index);
        }
        driver
            .authorize_port(driver.station().association_epoch().unwrap())
            .unwrap();
        let permit = admit(&mut authority, &driver, AssociationUse::Data);
        let epoch = driver.station().association_epoch().unwrap();
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        unsafe {
            driver.transmit_modern(
                &mut authority,
                permit,
                epoch,
                Msdu {
                    ethernet: &eth(0x0800),
                    class: TxClass::Data,
                    tid: Some(6),
                },
                &GuardedSource(state.clone()),
                &mut Policy(true),
            )
        }
        .unwrap();
        let packet = finish_tx(&model);
        assert_eq!(&packet[0][24..28], &[6, 0, 0xaa, 0xaa]);
        driver
            .poll(1_000_000 + 110, &Source, &mut Policy(true))
            .unwrap();
        driver.poll_released().unwrap();
        let mut raw = incoming_bytes(&qos_packet(1), Some((1, 0, 1)), false);
        let r2 = u32::from_le_bytes(raw[8..12].try_into().unwrap());
        raw[8..12].copy_from_slice(&((r2 & !(31 << 16)) | (firmware << 16)).to_le_bytes());
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
        assert_eq!(
            driver
                .poll(1_000_000 + 111, &Source, &mut Policy(true))
                .unwrap()
                .len(),
            1
        );
        // A valid descriptor layout for another cipher cannot inherit this
        // key identity because WCID/index/PN look plausible.
        let wrong = if firmware == 4 { 11 } else { 4 };
        raw[8..12].copy_from_slice(&((r2 & !(31 << 16)) | (wrong << 16)).to_le_bytes());
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
        assert!(matches!(
            driver.poll(1_000_000 + 112, &Source, &mut Policy(true)),
            Err(ClientError::WrongProfile)
        ));
    }
}

#[test]
fn peer_delba_before_native_start_cannot_publish_or_resurrect_the_agreement() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, _, _) = secure_modern();
    let parameters: u16 = 2 | (6 << 2) | (8 << 6);
    let mut request = vec![3, 0, 7];
    request.extend_from_slice(&parameters.to_le_bytes());
    request.extend_from_slice(&[0, 0, 0, 0]);
    incoming(&model, &response(0xd0, &request), None, false);
    driver
        .poll(1_000_000 + 110, &Source, &mut Policy(true))
        .unwrap();
    assert!(driver.pending_block_ack());
    let commands = model.response.lock().unwrap().client_commands.len();
    let mut delete = vec![3, 2];
    delete.extend_from_slice(&((6u16 << 12) | 0x0800).to_le_bytes());
    delete.extend_from_slice(&37u16.to_le_bytes());
    incoming(&model, &response(0xd0, &delete), None, false);
    driver
        .poll(1_000_000 + 111, &Source, &mut Policy(true))
        .unwrap();
    assert!(!driver.pending_block_ack());
    assert!(driver.station().active_rx_block_ack(6).is_none());
    assert_eq!(
        model.response.lock().unwrap().client_commands.len(),
        commands
    );
    assert!(
        !model
            .response
            .lock()
            .unwrap()
            .receive_ba
            .contains_key(&(1, 6))
    );
    assert_eq!(driver.next_receive_deadline(), None);
}

#[test]
fn modern_failed_real_configuration_keeps_current_context_and_peer() {
    if crate::test_process::isolate_model() {
        return;
    }
    use wifi::regulatory::AssociationUse;
    let (mut driver, model, before) = awaiting();
    let context = driver.modern_context().unwrap().clone();
    let (mut authority, _) = authority(&driver, &model);
    let permit = admit(&mut authority, &driver, AssociationUse::Configure);
    model.response.lock().unwrap().bss_status = 0xdead;
    // SAFETY: the model device, its pages and its domain belong to this test alone; no other
    // driver, worker or DMA touches them.
    assert!(unsafe { driver.configure_modern(&mut authority, permit, 105) }.is_err());
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    assert_eq!(driver.station().phase(), station::Phase::Configuring);
    assert_eq!(driver.completed_modern_requests(), 0);
    assert_eq!(
        model.response.lock().unwrap().client_commands.len(),
        before + 1
    );
    assert!(Arc::ptr_eq(driver.modern_context().unwrap(), &context));
    assert!(
        driver
            .join_modern(
                modern_inputs(&driver).0,
                modern_inputs(&driver).1,
                profile(),
                106,
                &Source,
                &mut Policy(true)
            )
            .is_err()
    );
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn late_firmware_completion_is_a_runtime_command_timeout() {
    if crate::test_process::isolate_model() {
        return;
    }
    use wifi::regulatory::AssociationUse;
    let (mut driver, model, _) = awaiting();
    let (mut authority, _) = authority(&driver, &model);
    let permit = admit(&mut authority, &driver, AssociationUse::Configure);
    // An ACK past the 500 ms Configure lifetime is past the runtime command
    // bound long before: the command ends as a timeout.
    model.response.lock().unwrap().pause_next_index_us = 600_000;
    let result = unsafe {
        // SAFETY: the model device, its pages and its domain belong to this test alone; no
        // other driver, worker or DMA touches them.
        driver.configure_modern(&mut authority, permit, 105)
    };
    assert!(
        matches!(result, Err(ClientError::Transport(_))),
        "{result:?}"
    );
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    assert_eq!(driver.station().phase(), station::Phase::Configuring);
    assert_eq!(driver.completed_modern_requests(), 0);
    assert!(
        driver
            .authorize_port(driver.station().association_epoch().unwrap())
            .is_err()
    );
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn copied_modern_local_streams_refuse_before_any_peer_or_packet_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    let (mut join, beacon) = modern_inputs(&driver);
    let mut elements = join.request.elements().to_vec();
    let mut at = 0;
    while elements[at] != 45 {
        at += 2 + usize::from(elements[at + 1]);
    }
    elements[at + 2 + 5] = 0xff; // Add an unobserved third HT RX stream.
    let local =
        station::phy::Capabilities::parse(Band::Ghz5, station::phy::Role::Station, &elements)
            .unwrap()
            .with_local_he_source_maps(
                *driver
                    .local_advertisement(Band::Ghz5)
                    .unwrap()
                    .he_source_maps(),
            )
            .unwrap();
    join.request = AssociationRequest::new_modern_with_local(
        0x11,
        10,
        elements,
        local,
        Mode::Eht,
        Width::Mhz80,
    )
    .unwrap();
    let before = model.response.lock().unwrap().client_commands.len();
    assert!(matches!(
        driver.join_modern(join, beacon, profile(), 100, &Source, &mut Policy(true)),
        Err(ClientError::WrongProfile)
    ));
    assert_eq!(driver.phase(), ClientPhase::Idle);
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, 0);
}

// Permission for the same stateful simulated PCI requester, never physical
// permission. The model's revocation and source allocations obey held guards.

struct EdcaInspection {
    hardware: bool,
    count: u8,
    observed: [station::phy::AccessCategory; 4],
    effective: [station::phy::AccessCategory; 4],
}
struct AuthorityState {
    hardware: Cell<bool>,
    os: Cell<bool>,
    allowed: Cell<bool>,
    revision: Cell<u64>,
    native: reg::NativeContext,
    context: Arc<station::AssociationContext>,
    response: Arc<Mutex<Response>>,
    // Keep this requester registry independently of diagnostic numbers.
    _memory: Arc<Mutex<Memory>>,
    windows: RefCell<Vec<(usize, usize)>>,
    allocations: Cell<usize>,
    edca_hardware: Cell<bool>,
    edca_os: Cell<bool>,
    edca_inspections: RefCell<Vec<EdcaInspection>>,
}
struct Hardware(Rc<AuthorityState>);
struct Os(Rc<AuthorityState>);
struct HardwareGuard {
    state: Rc<AuthorityState>,
    start: usize,
}
struct OsGuard(Rc<AuthorityState>);
impl Drop for HardwareGuard {
    fn drop(&mut self) {
        assert!(!self.state.os.get());
        let end = self.state.response.lock().unwrap().client_commands.len();
        self.state.windows.borrow_mut().push((self.start, end));
        self.state.hardware.set(false);
    }
}
impl Drop for OsGuard {
    fn drop(&mut self) {
        assert!(self.0.hardware.get());
        self.0.os.set(false);
    }
}
impl reg::HardwareLease for HardwareGuard {
    fn context(&self) -> reg::NativeContext {
        self.state.native
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<wifi::Channel> {
        (band == CHANNEL.band && number == CHANNEL.number).then_some(CHANNEL)
    }
    fn allows(&mut self, _: reg::Operation) -> bool {
        false
    }
    fn allows_association(&mut self, operation: &reg::AssociationOperation<'_>) -> bool {
        assert!(self.state.hardware.get() && self.state.os.get());
        let response = self.state.response.lock().unwrap();
        self.state.allowed.get()
            && core::ptr::eq(operation.context(), self.state.context.as_ref())
            && response.regulatory_domain
            && !response.regulatory_power.is_empty()
            && operation
                .context()
                .negotiated()
                .local()
                .local_he_source_maps()
                .is_some()
            && matches!(
                operation.context().definition().width,
                Width::Mhz20 | Width::Mhz40 | Width::Mhz80 | Width::Mhz160
            )
    }
    fn allows_edca(&mut self, operation: &reg::EdcaOperation<'_>) -> bool {
        assert!(self.state.hardware.get() && self.state.os.get());
        self.state
            .edca_inspections
            .borrow_mut()
            .push(EdcaInspection {
                hardware: true,
                count: operation.parameter_set_count(),
                observed: *operation.observed(),
                effective: *operation.effective(),
            });
        self.state.edca_hardware.get()
            && self.allows_association(operation.association())
            && operation
                .observed()
                .iter()
                .all(|ac| ac.aifsn >= 2 && ac.ecw_min != 0)
            && reg::wmm::station(operation.observed(), None).is_ok()
            && reg::wmm::station(operation.effective(), None).is_ok()
    }
}
impl reg::OsLease for OsGuard {
    fn revision(&self) -> u64 {
        self.0.revision.get()
    }
    fn allows(&mut self, _: reg::Operation) -> bool {
        false
    }
    fn allows_association(&mut self, operation: &reg::AssociationOperation<'_>) -> bool {
        assert!(self.0.hardware.get() && self.0.os.get());
        self.0.allowed.get() && core::ptr::eq(operation.context(), self.0.context.as_ref())
    }
    fn allows_edca(&mut self, operation: &reg::EdcaOperation<'_>) -> bool {
        assert!(self.0.hardware.get() && self.0.os.get());
        self.0.edca_inspections.borrow_mut().push(EdcaInspection {
            hardware: false,
            count: operation.parameter_set_count(),
            observed: *operation.observed(),
            effective: *operation.effective(),
        });
        self.0.edca_os.get()
            && self.allows_association(operation.association())
            && reg::wmm::station(operation.effective(), None).is_ok()
    }
}
// SAFETY: only this single-threaded model's requester is checked. Revocation
// respects these held guards; this supplies no physical platform permission.
unsafe impl reg::HardwarePolicy for Hardware {
    type Lease<'a> = HardwareGuard;
    fn acquire(&mut self) -> Option<Self::Lease<'_>> {
        assert!(!self.0.os.get());
        if self.0.hardware.replace(true) {
            return None;
        }
        let start = self.0.response.lock().unwrap().client_commands.len();
        Some(HardwareGuard {
            state: self.0.clone(),
            start,
        })
    }
}
// SAFETY: model OS permission belongs to that requester/context
// and remains inside its hardware guard, including real packet allocations.
unsafe impl reg::OsPolicy for Os {
    type Lease<'a> = OsGuard;
    fn acquire(&mut self, native: reg::NativeContext) -> Option<Self::Lease<'_>> {
        assert!(self.0.hardware.get());
        if native != self.0.native || self.0.os.replace(true) {
            return None;
        }
        Some(OsGuard(self.0.clone()))
    }
}
type Owner = reg::Owner<Hardware, Os>;
fn authority(driver: &Client<Bar, Q, NativeGuard>, model: &Model) -> (Owner, Rc<AuthorityState>) {
    let state = Rc::new(AuthorityState {
        hardware: Cell::new(false),
        os: Cell::new(false),
        allowed: Cell::new(true),
        revision: Cell::new(1),
        native: reg::NativeContext {
            device: Arc::as_ptr(&model.memory) as usize as u64,
            reset_epoch: 1,
            firmware_epoch: 1,
            policy_revision: 1,
        },
        context: driver.modern_context().unwrap().clone(),
        response: model.response.clone(),
        _memory: model.memory.clone(),
        windows: RefCell::new(Vec::new()),
        allocations: Cell::new(0),
        edca_hardware: Cell::new(true),
        edca_os: Cell::new(true),
        edca_inspections: RefCell::new(Vec::new()),
    });
    let owner = reg::Owner::new(Hardware(state.clone()), Os(state.clone())).unwrap();
    (owner, state)
}
fn admit(
    owner: &mut Owner,
    driver: &Client<Bar, Q, NativeGuard>,
    purpose: AssociationUse,
) -> reg::AssociationAdmission {
    owner
        .admit_association(
            driver.station(),
            driver.modern_context().unwrap().clone(),
            purpose,
        )
        .unwrap()
}
struct GuardedSource(Rc<AuthorityState>);
impl crate::bus::dma::memory::PageSource for GuardedSource {
    fn allocate(
        &self,
        bytes: usize,
        alignment: usize,
        attribute: crate::bus::dma::memory::CacheAttribute,
    ) -> Result<crate::bus::dma::memory::PinnedPages, crate::bus::dma::memory::MemoryError> {
        assert!(self.0.hardware.get() && self.0.os.get());
        self.0.allocations.set(self.0.allocations.get() + 1);
        crate::bus::dma::memory::PageSource::allocate(&Source, bytes, alignment, attribute)
    }
}

mod edca_controls;
mod management_controls;
pub(in super::super) mod pmf_controls;
