// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! The 6 GHz transport over the modeled native controller, with simulated
//! board and OS providers. These grant no physical-radio validation.
use super::super::tests::*;
use super::*;
use crate::wifi::mediatek::bootstrap::tests::*;
use crate::wifi::mediatek::mailbox::clc::remaining::tests::caps;
use crate::wifi::mediatek::ring::rx::tests::device_receive;
use crate::wifi::mediatek::ring::tests::Source;
use crate::wifi::regulatory as reg;
use crate::wifi::station::phy::{Mode, Width};
use crate::wifi::station::wire::{AssociationRequest, six_ghz::tests::selected};
use crate::wifi::{Band, Channel};
use alloc::rc::Rc;
use core::cell::Cell;
use stela_driver_base::stepped::block_on;

pub(crate) const SIX: Channel = Channel {
    band: Band::Ghz6,
    number: 1,
    frequency_mhz: 5955,
};
struct Policy;
impl scan::Policy for Policy {
    fn allow_receive(&mut self, channel: Channel) -> bool {
        channel == SIX
    }
    fn allow_probe(&mut self, _: Channel) -> bool {
        false
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
        (band == SIX.band && number == SIX.number).then_some(SIX)
    }
}
impl station::RegulatoryPolicy for Policy {
    fn allow_management_tx(&mut self, _: Channel) -> bool {
        true
    }
    fn allow_data_tx(&mut self, _: Channel) -> bool {
        false
    }
}

/// Which provider permits 6 GHz transport, how often the board was asked and
/// the AP power type it was asked about.
#[derive(Default)]
struct Permission {
    hardware: Cell<bool>,
    os: Cell<bool>,
    asked: Cell<usize>,
    power_type: Cell<Option<u8>>,
}
const NATIVE: reg::NativeContext = reg::NativeContext {
    device: 1,
    reset_epoch: 1,
    firmware_epoch: 1,
    policy_revision: 1,
};
struct Hardware(Rc<Permission>);
struct Os(Rc<Permission>);
impl reg::HardwareLease for Hardware {
    fn context(&self) -> reg::NativeContext {
        NATIVE
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
        (band == SIX.band && number == SIX.number).then_some(SIX)
    }
    fn allows(&mut self, _: reg::Operation) -> bool {
        false
    }
    fn allows_six_ghz_authentication(
        &mut self,
        operation: &reg::SixGhzAuthenticationOperation<'_>,
    ) -> bool {
        assert_eq!(operation.session().channel(), SIX);
        self.0.asked.set(self.0.asked.get() + 1);
        self.0
            .power_type
            .set(Some(operation.profile().raw_ap_power_type));
        self.0.hardware.get()
    }
}
impl reg::OsLease for Os {
    fn revision(&self) -> u64 {
        1
    }
    fn allows(&mut self, _: reg::Operation) -> bool {
        false
    }
    fn allows_six_ghz_authentication(
        &mut self,
        operation: &reg::SixGhzAuthenticationOperation<'_>,
    ) -> bool {
        assert_eq!(operation.profile().wcid, profile().wcid);
        self.0.os.get()
    }
}
// SAFETY: a single-threaded model requester; these leases grant no physical access.
unsafe impl reg::HardwarePolicy for Hardware {
    type Lease<'a> = Hardware;
    fn acquire(&mut self) -> Option<Self::Lease<'_>> {
        Some(Hardware(self.0.clone()))
    }
}
// SAFETY: as above, for the modeled OS selection service.
unsafe impl reg::OsPolicy for Os {
    type Lease<'a> = Os;
    fn acquire(&mut self, native: reg::NativeContext) -> Option<Self::Lease<'_>> {
        (native == NATIVE).then(|| Os(self.0.clone()))
    }
}

fn six_setup() -> (Client<Bar, Q, NativeGuard>, Model) {
    let mut f = NativeFixture::new(Chip::Mt7925);
    f.model.lock().unwrap().capability_payload = Some(caps(Chip::Mt7925, [true; 3]));
    f.install().unwrap();
    f.boot(Chip::Mt7925);
    f.policy_bands(Chip::Mt7925, [true; 3]);
    // SAFETY: this fixture owns the modeled controller, policy and firmware.
    unsafe { block_on(f.controller.initialize_mac()) }.unwrap();
    // SAFETY: modeled initial radio ownership, as the 6 GHz scan test uses.
    unsafe { block_on(f.controller.start(Band::Ghz5, 36, 5180)) }.unwrap();
    // SAFETY: exact fixture-owned interface and controller resource.
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
    // SAFETY: the just-registered matching interface and native controller.
    let driver = unsafe { Client::new(controller, interface) }.unwrap();
    let model = Model {
        response: model,
        bar,
        queues,
        memory,
        drops,
    };
    (driver, model)
}

/// A 20 MHz HE access point on 6 GHz channel 1 offering SAE H2E.
pub(crate) fn beacon_elements() -> Vec<u8> {
    let mut out = vec![0, 1, b'x'];
    out.extend_from_slice(&[1, 8, 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c]);
    out.extend_from_slice(&[5, 4, 0, 2, 0, 0]);
    out.extend_from_slice(&selected());
    // HE capabilities: 40/80 MHz in 5/6 GHz, MCS 0-11 on two streams.
    let mut he = vec![255, 22, 35];
    he.extend_from_slice(&[0; 17]);
    he[3 + 6] = 4;
    he.extend_from_slice(&[0xfa, 0xff, 0xfa, 0xff]);
    out.extend_from_slice(&he);
    // HE operation: 6 GHz information present, basic HE-MCS 0-7 on one stream.
    out.extend_from_slice(&[255, 12, 36, 0, 0, 2, 1, 0xfc, 0xff, 1, 0, 1, 0, 6]);
    out.extend_from_slice(&[255, 3, 59, 0x4a, 0]);
    out.extend_from_slice(&[
        221, 24, 0, 0x50, 0xf2, 2, 1, 1, 0x89, 0, 3, 0xa4, 0, 0, 0x27, 0xa4, 0, 0, 0x42, 0x43, 94,
        0, 0x62, 0x32, 47, 0,
    ]);
    out
}

fn observe(driver: &mut Client<Bar, Q, NativeGuard>, model: &Model) -> Arc<PmfBssObservation> {
    observe_with(driver, model, &beacon_elements())
}

fn observe_with(
    driver: &mut Client<Bar, Q, NativeGuard>,
    model: &Model,
    elements: &[u8],
) -> Arc<PmfBssObservation> {
    let request = scan::Request::new(vec![SIX], vec![], vec![]).unwrap();
    driver
        .begin_scan(request, 0, 1_000_000, &mut Policy)
        .unwrap();
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&crate::Mac::BROADCAST.0);
    let mut raw = incoming_bytes(&beacon, None, false);
    raw[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
    let events = driver.poll(1, &Source, &mut Policy).unwrap();
    assert!(matches!(events.as_slice(), [Event::Bss(bss)] if bss.channel == SIX));
    let observed = driver.pmf_bss_observations().next().unwrap().clone();
    let mut done = vec![0; 52];
    done[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
    done[36] = 14;
    done[38] = 6;
    done[50] = 4;
    device_receive(&model.queues[&0xd4500], &done, true, 0);
    driver.poll(2, &Source, &mut Policy).unwrap();
    // SAFETY: the modeled scan's terminal event and all model RX users drained.
    unsafe { driver.retire_scan_after_drain() }.unwrap();
    observed
}

pub(crate) fn six_profile() -> Profile {
    let mut profile = profile();
    profile.channel = SIX;
    profile
}

#[test]
fn a_wide_standard_power_ap_authenticates_on_its_primary_as_mac80211_does() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = six_setup();
    let mut elements = beacon_elements();
    // The HE operation's 6 GHz information: 80 MHz around channel 7 from a
    // standard-power AP, its primary still channel 1.
    let he = elements
        .windows(3)
        .position(|w| w == [255, 12, 36])
        .unwrap();
    elements[he + 10] = 2 | 1 << 3;
    elements[he + 11] = 7;
    let observed = observe_with(&mut driver, &model, &elements);
    let permission = Rc::new(Permission::default());
    permission.hardware.set(true);
    permission.os.set(true);
    let mut owner = reg::Owner::new(Hardware(permission.clone()), Os(permission.clone())).unwrap();
    // SAFETY: the modeled requester owns this controller and both providers.
    unsafe {
        driver.begin_six_ghz_external_authentication(
            &mut owner,
            &observed,
            Selection::new(selected()).unwrap(),
            six_profile(),
            10,
            1_000_000,
        )
    }
    .unwrap();
    assert_eq!(driver.phase(), Phase::Joining);
    // The board decides on the AP's power type.
    assert_eq!(permission.power_type.get(), Some(1));
}

#[test]
fn a_bound_six_ghz_session_is_fenced_and_removes_through_its_own_codec() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = six_setup();
    let observed = observe(&mut driver, &model);
    let permission = Rc::new(Permission::default());
    let mut owner = reg::Owner::new(Hardware(permission.clone()), Os(permission.clone())).unwrap();
    let selection = || Selection::new(selected()).unwrap();
    let commands = || model.response.lock().unwrap().client_commands.len();

    // A provider's refusal leaves no native effect.
    let before = commands();
    permission.hardware.set(true);
    // SAFETY: the modeled requester owns this controller and both providers.
    let refused = unsafe {
        driver.begin_six_ghz_external_authentication(
            &mut owner,
            &observed,
            selection(),
            six_profile(),
            10,
            1_000_000,
        )
    };
    assert!(matches!(
        refused,
        Err(Error::Regulatory(reg::Error::OsDenied))
    ));
    assert_eq!(commands(), before);
    assert_eq!(driver.phase(), Phase::Idle);

    permission.os.set(true);
    // SAFETY: as above.
    let session = unsafe {
        driver.begin_six_ghz_external_authentication(
            &mut owner,
            &observed,
            selection(),
            six_profile(),
            10,
            1_000_000,
        )
    }
    .unwrap();
    assert_eq!(driver.phase(), Phase::Joining);
    // BSS, peer and broadcast records, each requalified before publication.
    assert_eq!(commands(), before + 3);
    assert!(permission.asked.get() >= 5);

    // The unguarded poll and the generic SAE transmit cannot service it.
    assert!(matches!(
        driver.poll(20, &Source, &mut Policy),
        Err(Error::WrongPhase)
    ));
    assert!(matches!(
        driver.transmit_external_authentication(
            &session,
            crate::wifi::station::external::Frame {
                transaction: 1,
                status: 0,
                data: &[]
            },
            20,
            &Source,
            &mut Policy
        ),
        Err(Error::WrongPhase)
    ));

    // An SAE commit goes out on a ROC20 grant for 6 GHz channel 1.
    // SAFETY: as above.
    unsafe {
        driver.transmit_six_ghz_external_authentication(
            &mut owner,
            &session,
            crate::wifi::station::external::Frame {
                transaction: 1,
                status: 126,
                data: &[19, 0, 0xaa, 0, 0xbb],
            },
            25,
            &Source,
        )
    }
    .unwrap();
    assert_eq!(model.response.lock().unwrap().roc_channel, Some(1));
    let packet = finish_tx_on(&model, 1);
    assert_eq!(
        &packet[0][24..],
        &[3, 0, 1, 0, 126, 0, 19, 0, 0xaa, 0, 0xbb]
    );
    // The peer's commit is delivered only through the guarded poll.
    let mut raw = incoming_bytes(
        &response(0xb0, &[3, 0, 1, 0, 76, 0, 0, 0xff, 7, 0]),
        None,
        false,
    );
    raw[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
    let mut received = None;
    for now in [30, 31] {
        // SAFETY: as above.
        let events = unsafe {
            driver.poll_six_ghz_external_authentication(
                &mut owner,
                &session,
                now,
                &Source,
                &mut Policy,
            )
        }
        .unwrap();
        received = received.or(events.into_iter().find_map(|event| match event {
            Event::ExternalAuthentication(rx) => Some(rx),
            _ => None,
        }));
    }
    let received = received.unwrap();
    assert!(Arc::ptr_eq(received.session(), &session));
    assert_eq!((received.transaction(), received.status()), (1, 76));
    assert_eq!(received.data(), &[0, 0xff, 7, 0]);
    // The commit's TX-free arrived on the same guarded polls.
    assert_eq!(driver.poll_released().unwrap().len(), 1);

    // Cancellation needs no permission and removes the 6 GHz peer.
    permission.os.set(false);
    driver
        .cancel_six_ghz_external_authentication(&session)
        .unwrap();
    assert_eq!(driver.phase(), Phase::NeedsDrain);
    // Cleanup polls need no provider and report the ended exchange.
    let events = driver.poll(40, &Source, &mut Policy).unwrap();
    assert!(events.iter().any(|event| matches!(
        event,
        Event::ExternalAuthenticationEnded { session: ended, .. } if Arc::ptr_eq(ended, &session)
    )));
    // SAFETY: every modeled response is complete and no RX/TX user remains.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    assert_eq!(driver.phase(), Phase::Idle);

    // The retired transport no longer holds the client busy. Its native time
    // mapping advanced the station clock past the polls above.
    permission.os.set(true);
    // SAFETY: as above.
    unsafe {
        driver.begin_six_ghz_external_authentication(
            &mut owner,
            &observed,
            selection(),
            six_profile(),
            10_000_000,
            1_000_000,
        )
    }
    .unwrap();
}

#[test]
fn a_six_ghz_sae_frame_goes_again_once_its_wait_passes() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = six_setup();
    let observed = observe(&mut driver, &model);
    let permission = Rc::new(Permission::default());
    permission.hardware.set(true);
    permission.os.set(true);
    let mut owner = reg::Owner::new(Hardware(permission.clone()), Os(permission.clone())).unwrap();
    // SAFETY: the modeled requester owns this controller and both providers.
    let session = unsafe {
        driver.begin_six_ghz_external_authentication(
            &mut owner,
            &observed,
            Selection::new(selected()).unwrap(),
            six_profile(),
            10,
            1_000_000,
        )
    }
    .unwrap();
    // SAFETY: as above.
    unsafe {
        driver.transmit_six_ghz_external_authentication(
            &mut owner,
            &session,
            crate::wifi::station::external::Frame {
                transaction: 1,
                status: 0,
                data: &[19, 0],
            },
            25,
            &Source,
        )
    }
    .unwrap();
    let first = finish_tx_on(&model, 1).remove(0);
    for now in [30, 600_000] {
        // SAFETY: as above.
        unsafe {
            driver.poll_six_ghz_external_authentication(
                &mut owner,
                &session,
                now,
                &Source,
                &mut Policy,
            )
        }
        .unwrap();
        if now == 30 {
            // The first frame's TX-free; no TX status came, so its wait runs.
            assert!(finish_tx_on(&model, 1).is_empty());
            assert_eq!(driver.poll_released().unwrap().len(), 1);
        }
    }
    let second = finish_tx_on(&model, 1).remove(0);
    assert_eq!(second[24..], first[24..]);
}

#[test]
fn a_six_ghz_sae_exchange_associates_like_ieee80211_mgd_assoc() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = six_setup();
    let observed = observe(&mut driver, &model);
    let permission = Rc::new(Permission::default());
    permission.hardware.set(true);
    permission.os.set(true);
    let mut owner = reg::Owner::new(Hardware(permission.clone()), Os(permission.clone())).unwrap();
    // SAFETY: the modeled requester owns this controller and both providers.
    let session = unsafe {
        driver.begin_six_ghz_external_authentication(
            &mut owner,
            &observed,
            Selection::new(selected()).unwrap(),
            six_profile(),
            10,
            1_000_000,
        )
    }
    .unwrap();
    let rx = |frame: &[u8]| {
        let mut raw = incoming_bytes(frame, None, false);
        raw[12..16].copy_from_slice(&(181u32 << 8).to_le_bytes());
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
    };
    let local = driver.local_advertisement(Band::Ghz6).unwrap();
    let mut elements = vec![0, 1, b'x'];
    elements.extend_from_slice(local.elements());
    elements.extend_from_slice(&selected());
    let mut join = join();
    join.bssid = six_profile().peer;
    join.channel = SIX;
    join.request = AssociationRequest::new_modern_with_local_pmf(
        0x11,
        10,
        elements,
        local.capabilities().clone(),
        Mode::He,
        Width::Mhz20,
    )
    .unwrap();
    let mut now = 20;
    let poll = |driver: &mut Client<Bar, Q, NativeGuard>,
                owner: &mut reg::Owner<Hardware, Os>,
                now: u64| {
        // SAFETY: as above.
        unsafe {
            driver.poll_six_ghz_external_authentication(owner, &session, now, &Source, &mut Policy)
        }
        .unwrap();
    };
    for (transaction, status, data) in [(1u8, 126u8, &[19, 0][..]), (2, 0, &[1, 0][..])] {
        // ieee80211_mgd_assoc refuses until the exchange authenticates.
        // SAFETY: as above.
        assert!(
            // SAFETY: the model device, its pages and its domain belong to this test alone; no
            // other driver, worker or DMA touches them.
            unsafe {
                driver.associate_six_ghz_external(&mut owner, &session, join.clone(), now, &Source)
            }
            .is_err()
        );
        // SAFETY: as above.
        unsafe {
            driver.transmit_six_ghz_external_authentication(
                &mut owner,
                &session,
                crate::wifi::station::external::Frame {
                    transaction: u16::from(transaction),
                    status: 0,
                    data,
                },
                now,
                &Source,
            )
        }
        .unwrap();
        finish_tx_on(&model, 1);
        let mut body = vec![3, 0, transaction, 0, status, 0];
        body.extend_from_slice(data);
        rx(&response(0xb0, &body));
        for _ in 0..2 {
            now += 1;
            poll(&mut driver, &mut owner, now);
        }
        assert_eq!(driver.poll_released().unwrap().len(), 1);
    }
    assert!(driver.station().external_authenticated());
    // SAFETY: as above.
    unsafe { driver.associate_six_ghz_external(&mut owner, &session, join, now, &Source) }.unwrap();
    // The OS's association request goes out on the held 6 GHz channel.
    let request = finish_tx_on(&model, 1).remove(0);
    assert_eq!(request[..2], [0, 0]);
    // ieee80211_assoc_add_rates and the HE elements; no HT or VHT on 6 GHz.
    let mut ids = Vec::new();
    let mut rest = &request[28..];
    while let [id, length, tail @ ..] = rest {
        ids.push((*id, tail.first().copied().filter(|_| *id == 255)));
        rest = &tail[usize::from(*length)..];
    }
    assert!(ids.contains(&(1, None)));
    assert!(ids.contains(&(255, Some(35))) && ids.contains(&(255, Some(59))));
    assert!(!ids.iter().any(|(id, _)| matches!(id, 45 | 61 | 191 | 192)));
    now += 1;
    poll(&mut driver, &mut owner, now);
    assert_eq!(driver.poll_released().unwrap().len(), 1);
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&beacon_elements());
    rx(&response(0x10, &assoc));
    now += 1;
    poll(&mut driver, &mut owner, now);
    // The answered association waits for admission as any modern one does.
    assert_eq!(driver.phase(), Phase::AwaitingModernAdmission);
    assert!(driver.station().external_authentication().is_none());
    assert_eq!(driver.modern_context().unwrap().channel(), SIX);
}
