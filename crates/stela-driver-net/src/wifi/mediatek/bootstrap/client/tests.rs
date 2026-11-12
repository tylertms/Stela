extern crate std;
use crate::wifi::mediatek::bootstrap::tests::*;
use crate::wifi::mediatek::mailbox::clc::tests::*;
use crate::wifi::mediatek::{
    firmware::{
        Requests,
        tests::{catalog, patch_bytes},
    },
    identity::Chip,
    mailbox::tests::*,
    ring::{
        rx::tests::device_receive,
        tests::{Memory, Source},
    },
};
use crate::wifi::{Band, mediatek::bootstrap::Error as BootError};
use crate::{
    Mac,
    wifi::{
        self,
        mediatek::{
            bootstrap::client::{Client, Error as ClientError, Event, Phase as ClientPhase},
            mcu::{client::Profile, device::Context},
        },
        regulatory::{
            channel,
            database::{Country, tests::country_of},
        },
        scan,
        security::ReplaySeed,
        station::{self, Join, PortPolicy, data::TxClass, wire::AssociationRequest},
    },
};
use alloc::collections::BTreeMap;
use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use stela_driver_base::stepped::block_on;

pub(crate) const LOCAL: Mac = Mac([2, 1, 2, 3, 4, 5]);
pub(crate) const AP: Mac = Mac([2, 6, 7, 8, 9, 10]);
pub(crate) const CHANNEL: wifi::Channel = wifi::Channel {
    band: Band::Ghz5,
    number: 36,
    frequency_mhz: 5180,
};
/// The flags cfg80211 gives every channel under an 80 MHz rule.
pub(crate) const WIDE: u32 = channel::NO_160MHZ | channel::NO_320MHZ;
/// The flags cfg80211 gives the 6 GHz channels of [`three_band_country`].
pub(crate) const SIX_GHZ: u32 = channel::NO_80MHZ | WIDE;
/// The test "US", with a 40 MHz rule over 6 GHz channels 1 to 5.
pub(crate) fn three_band_country() -> Country {
    country_of(&[
        (2_400_000, 2_500_000, 80_000, 3000, 0),
        (5_150_000, 5_350_000, 80_000, 3000, 0),
        (5_945_000, 5_985_000, 40_000, 3000, 0),
    ])
}
pub(crate) struct Policy(pub(crate) bool);
impl station::RegulatoryPolicy for Policy {
    fn allow_management_tx(&mut self, _: wifi::Channel) -> bool {
        self.0
    }
    fn allow_data_tx(&mut self, _: wifi::Channel) -> bool {
        self.0
    }
}
impl scan::Policy for Policy {
    fn allow_receive(&mut self, _: wifi::Channel) -> bool {
        self.0
    }
    fn allow_probe(&mut self, _: wifi::Channel) -> bool {
        self.0
    }
    fn lookup_channel(&mut self, b: Band, n: u16) -> Option<wifi::Channel> {
        if self.0 && b == Band::Ghz5 && n == 36 {
            Some(CHANNEL)
        } else {
            None
        }
    }
}
pub(crate) fn profile() -> Profile {
    Profile {
        interface: Context {
            bss_index: 0,
            omac_index: 0,
            band_index: 255,
            wmm_index: 0,
            link_index: 0,
            broadcast_wcid: 19,
            address: LOCAL,
        },
        peer: AP,
        wcid: 1,
        channel: CHANNEL,
        aid: 0,
        basic_rates: 0x15,
        supported_rates: 0xff,
        beacon_interval: 100,
        dtim_period: 2,
        slot_time: 9,
        antenna_mask: 3,
        cipher: 0,
    }
}
pub(crate) fn join() -> Join {
    Join {
        bssid: AP,
        channel: CHANNEL,
        policy: PortPolicy::Controlled,
        request: AssociationRequest::new(
            0x11,
            10,
            vec![
                0, 1, b'x', 1, 8, 0x8c, 18, 0x98, 36, 0xb0, 72, 96, 108, 48, 18, 1, 0, 0, 15, 172,
                4, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 2,
            ],
        )
        .unwrap(),
    }
}
pub(crate) struct Model {
    pub(crate) response: Arc<Mutex<Response>>,
    pub(crate) bar: Bar,
    pub(crate) queues: BTreeMap<u32, ModelState>,
    pub(crate) memory: Arc<Mutex<Memory>>,
    pub(crate) drops: Arc<AtomicUsize>,
}
pub(crate) fn setup() -> (Client<Bar, Q, NativeGuard>, Model) {
    let mut f = NativeFixture::new(Chip::Mt7925);
    f.install().unwrap();
    f.boot(Chip::Mt7925);
    f.policy(Chip::Mt7925);
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
    let driver = unsafe { Client::new(controller, interface) }.unwrap();
    (
        driver,
        Model {
            response: model,
            bar,
            queues,
            memory,
            drops,
        },
    )
}
pub(crate) fn pointer(memory: &Memory, address: u64, bytes: usize) -> *const u8 {
    let (&base, (pages, pointer, direction)) =
        memory.mappings.range(..=address).next_back().unwrap();
    let offset = (address - base) as usize;
    assert!(offset + bytes <= pages.bytes());
    assert_ne!(*direction, crate::bus::dma::mapping::Direction::FromDevice);
    (*pointer + offset) as *const u8
}
// Model device fetches the published descriptor, then TXWI/TXP and
// payload via the same requester's registry. It keeps no CPU-address alias.
pub(crate) fn finish_tx(model: &Model) -> Vec<Vec<u8>> {
    finish_tx_on(model, 36)
}
/// The TXWI of the oldest descriptor the device has not fetched yet.
pub(crate) fn next_txwi(model: &Model) -> Vec<u8> {
    let queue = model.queues[&0xd4300].lock().unwrap();
    assert_ne!(queue.index, queue.head);
    let memory = model.memory.lock().unwrap();
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    let desc = unsafe {
        core::slice::from_raw_parts(
            pointer(&memory, queue.base + u64::from(queue.index) * 16, 16),
            16,
        )
    };
    let address = u32::from_le_bytes(desc[..4].try_into().unwrap()) as u64;
    // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
    // accesses it concurrently.
    unsafe { core::slice::from_raw_parts(pointer(&memory, address, 64), 64) }.to_vec()
}
pub(crate) fn finish_tx_on(model: &Model, channel: u8) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut queue = model.queues[&0xd4300].lock().unwrap();
    while queue.index != queue.head {
        {
            let r = model.response.lock().unwrap();
            assert_eq!(r.roc_channel.or(r.home_channel), Some(channel));
        }
        let memory = model.memory.lock().unwrap();
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
        // nothing accesses it concurrently.
        let desc = unsafe {
            core::slice::from_raw_parts(
                pointer(&memory, queue.base + u64::from(queue.index) * 16, 16),
                16,
            )
        };
        let address = u32::from_le_bytes(desc[..4].try_into().unwrap()) as u64;
        assert_eq!(
            u32::from_le_bytes(desc[4..8].try_into().unwrap()),
            (64 << 16) | (1 << 30)
        );
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
        // nothing accesses it concurrently.
        let txwi = unsafe { core::slice::from_raw_parts(pointer(&memory, address, 64), 64) };
        let token = u16::from_le_bytes(txwi[32..34].try_into().unwrap()) & 0x7fff;
        let wcid = u32::from_le_bytes(txwi[4..8].try_into().unwrap()) & 0xfff;
        assert!(
            model
                .response
                .lock()
                .unwrap()
                .raw_disabled
                .contains(&(wcid as u16))
        );
        let payload_addr = u32::from_le_bytes(txwi[40..44].try_into().unwrap()) as u64;
        let length = u16::from_le_bytes(txwi[44..46].try_into().unwrap());
        assert_ne!(length & 0x8000, 0);
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
        // nothing accesses it concurrently.
        let payload = unsafe {
            core::slice::from_raw_parts(
                pointer(&memory, payload_addr, usize::from(length & 0xfff)),
                usize::from(length & 0xfff),
            )
        }
        .to_vec();
        out.push(payload);
        let words = [
            (6u32 << 27) | (1 << 16),
            4 << 16,
            (1 << 31) | (wcid << 12),
            (1 << 30) | (3 << 24),
            u32::from(token) | (0x7fff << 15),
        ];
        let free = words
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect::<Vec<_>>();
        drop(memory);
        queue.index = (queue.index + 1) % queue.count;
        device_receive(&model.queues[&0xd4500], &free, true, 0);
    }
    out
}
pub(crate) fn incoming_bytes(
    mpdu: &[u8],
    protected: Option<(u16, u8, u64)>,
    translated: bool,
) -> Vec<u8> {
    let mut b = vec![0; 32 + if protected.is_some() { 16 } else { 0 }];
    let mut r1 = 0u32;
    let mut r2 = 1u32 << 30;
    if translated {
        r2 |= 1 << 7;
    }
    if let Some((slot, index, pn)) = protected {
        r1 = u32::from(slot) | (u32::from(index) << 21) | (1 << 16) | (1 << 31);
        r2 |= 4 << 16;
        for i in 0..6 {
            b[32 + i] = (pn >> (i * 8)) as u8;
        }
    }
    b[4..8].copy_from_slice(&r1.to_le_bytes());
    b[8..12].copy_from_slice(&r2.to_le_bytes());
    b[12..16].copy_from_slice(
        &((36u32 << 8) | if mpdu[4] & 1 == 0 { 1 << 16 } else { 0 }).to_le_bytes(),
    );
    b.extend_from_slice(mpdu);
    let len = b.len() as u32;
    b[..4].copy_from_slice(&((2 << 27) | len).to_le_bytes());
    b
}
pub(crate) fn incoming(
    model: &Model,
    mpdu: &[u8],
    protected: Option<(u16, u8, u64)>,
    translated: bool,
) {
    let bytes = incoming_bytes(mpdu, protected, translated);
    device_receive(&model.queues[&0xd4520], &bytes, true, 0);
}
pub(crate) fn response(subtype: u16, body: &[u8]) -> Vec<u8> {
    let mut b = vec![0; 24];
    b[..2].copy_from_slice(&subtype.to_le_bytes());
    b[4..10].copy_from_slice(&LOCAL.0);
    b[10..16].copy_from_slice(&AP.0);
    b[16..22].copy_from_slice(&AP.0);
    b.extend_from_slice(body);
    b
}
// Frontend protocol steps are 100 ms apart.
pub(crate) fn frontend_tick(step: u64) -> u64 {
    step * 100_000
}
pub(crate) fn associated(driver: &mut Client<Bar, Q, NativeGuard>, model: &Model, now: u64) {
    associated_with_configuration_delay(driver, model, now, 0).unwrap();
}
pub(crate) fn associated_with_configuration_delay(
    driver: &mut Client<Bar, Q, NativeGuard>,
    model: &Model,
    now: u64,
    delay: u64,
) -> Result<(), ClientError> {
    driver
        .join(
            join(),
            profile(),
            frontend_tick(now),
            &Source,
            &mut Policy(true),
        )
        .unwrap();
    assert_eq!(driver.phase(), ClientPhase::Joining);
    assert_eq!(finish_tx(model)[0][..2], [0xb0, 0]);
    driver
        .poll(frontend_tick(now + 1), &Source, &mut Policy(true))
        .unwrap();
    driver.poll_released().unwrap();
    incoming(model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    driver
        .poll(frontend_tick(now + 2), &Source, &mut Policy(true))
        .unwrap();
    assert_eq!(finish_tx(model)[0][..2], [0, 0]);
    driver
        .poll(frontend_tick(now + 3), &Source, &mut Policy(true))
        .unwrap();
    driver.poll_released().unwrap();
    incoming(
        model,
        &response(
            0x10,
            &[
                0x11, 0, 0, 0, 1, 0xc0, 1, 8, 0x8c, 18, 0x98, 36, 0xb0, 72, 96, 108,
            ],
        ),
        None,
        false,
    );
    model.response.lock().unwrap().pause_next_index_us = delay;
    driver.poll(frontend_tick(now + 4), &Source, &mut Policy(true))?;
    assert_eq!(driver.phase(), ClientPhase::Associated);
    Ok(())
}
pub(crate) fn eth(protocol: u16) -> Vec<u8> {
    let mut b = vec![4, 1, 2, 3, 4, 5];
    b.extend_from_slice(&LOCAL.0);
    b.extend_from_slice(&protocol.to_be_bytes());
    b.extend_from_slice(&[1, 2, 3]);
    b
}
#[test]
fn rssi_events_report_crossings_like_mt7925_mcu_rssi_monitor_event() {
    if crate::test_process::isolate_model() {
        return;
    }
    let mut payload = vec![3, 0, 0, 0];
    payload.extend_from_slice(&[0, 0, 8, 0]);
    payload.extend_from_slice(&(-75i32).to_le_bytes());
    let events = super::rssi_events(&payload, -70);
    assert!(matches!(
        events.as_slice(),
        [Event::Rssi {
            rssi: -75,
            high: false
        }]
    ));
    let events = super::rssi_events(&payload, -80);
    assert!(matches!(
        events.as_slice(),
        [Event::Rssi {
            rssi: -75,
            high: true
        }]
    ));
    payload[6] = 0;
    assert!(super::rssi_events(&payload, -70).is_empty());
    assert!(super::rssi_events(&payload[..6], -70).is_empty());
}
#[test]
fn late_legacy_configuration_ack_is_a_runtime_command_timeout() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    // An ACK past the configuration lifetime is past the runtime command
    // bound long before: the command ends as a timeout.
    let result = associated_with_configuration_delay(&mut driver, &model, 0, 600_000);
    assert!(
        matches!(result, Err(ClientError::Transport(_))),
        "{result:?}"
    );
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    assert_eq!(driver.station().phase(), station::Phase::Configuring);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}
pub(crate) fn data(group: bool) -> Vec<u8> {
    let mut b = response(0x4208, &[0xaa, 0xaa, 3, 0, 0, 0, 8, 0, 1, 2, 3]);
    if group {
        b[4..10].fill(255)
    }
    b[16..22].copy_from_slice(&[4, 1, 2, 3, 4, 5]);
    b
}
#[test]
fn mt7925_init_scan_secure_client_reconnect_and_stop() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    let request = scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap();
    driver
        .begin_scan(request, 0, 1_000_000, &mut Policy(true))
        .unwrap();
    assert_eq!(driver.phase(), ClientPhase::Scanning);
    // Generic scan command response has no status/scan completion permission.
    assert!(matches!(
        driver.transport_facts().last(),
        Some(crate::wifi::mediatek::mailbox::init::Fact::GenericResponse(
            _
        ))
    ));
    let mut beacon = response(
        0x80,
        &[
            0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0x11, 0, 0, 1, b'x', 1, 1, 0x8c, 5, 4, 0, 2, 0, 0,
        ],
    );
    beacon[4..10].fill(255);
    incoming(&model, &beacon, None, false);
    assert!(matches!(
        driver
            .poll(frontend_tick(1), &Source, &mut Policy(true))
            .unwrap()
            .as_slice(),
        [Event::Bss(_)]
    ));
    let mut done = vec![0; 52];
    done[..4].copy_from_slice(&(7u32 << 27).to_le_bytes());
    done[36] = 14;
    done[38] = 6;
    done[50] = 4;
    device_receive(&model.queues[&0xd4500], &done, true, 0);
    driver
        .poll(frontend_tick(2), &Source, &mut Policy(true))
        .unwrap();
    assert_eq!(driver.phase(), ClientPhase::ScanCompleted);
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_scan_after_drain() }.unwrap();
    associated(&mut driver, &model, 10);
    let epoch = driver.station().association_epoch().unwrap();
    assert!(matches!(
        driver.transmit(
            epoch,
            &eth(0x0800),
            TxClass::Data,
            &Source,
            &mut Policy(true)
        ),
        Err(ClientError::Data(station::data::Error::ClosedPort))
    ));
    driver
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .install_key(false, 2, &[2; 16], ReplaySeed::ZERO)
        .unwrap();
    driver.authorize_port(epoch).unwrap();
    let receipt = driver
        .transmit(
            epoch,
            &eth(0x0800),
            TxClass::Data,
            &Source,
            &mut Policy(true),
        )
        .unwrap();
    assert!(driver.poll_released().unwrap().is_empty());
    let transmitted = finish_tx(&model);
    assert_eq!(&transmitted[0][..2], &[8, 0x41]);
    assert_eq!(&transmitted[0][24..30], &[0xaa, 0xaa, 3, 0, 0, 0]);
    driver
        .poll(frontend_tick(20), &Source, &mut Policy(true))
        .unwrap();
    let released = driver.poll_released().unwrap();
    assert_eq!(released.len(), 1);
    assert_eq!(released[0].receipt, receipt);
    assert_eq!(released[0].context.association_epoch(), epoch);
    assert!(released[0].context.key().is_some());
    drop(released);
    incoming(&model, &data(false), Some((1, 0, 1)), false);
    assert!(matches!(
        driver
            .poll(frontend_tick(21), &Source, &mut Policy(true))
            .unwrap()
            .as_slice(),
        [Event::Ingress(station::data::Ingress::Data(_))]
    ));
    incoming(&model, &data(false), Some((1, 0, 1)), false);
    assert!(matches!(
        driver.poll(frontend_tick(22), &Source, &mut Policy(true)),
        Err(ClientError::Data(station::data::Error::Crypto(
            wifi::security::Error::Replay
        )))
    ));
    incoming(&model, &data(true), Some((1, 2, 1)), false);
    driver
        .poll(frontend_tick(23), &Source, &mut Policy(true))
        .unwrap();
    incoming(&model, &data(false), Some((1, 0, 2)), true);
    assert!(matches!(
        driver.poll(frontend_tick(24), &Source, &mut Policy(true)),
        Err(ClientError::Rx(
            crate::wifi::mediatek::rx::Error::UnsupportedTranslation
        ))
    ));
    driver.disconnect().unwrap();
    assert_eq!(driver.phase(), ClientPhase::NeedsDrain);
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    associated(&mut driver, &model, 30);
    assert_ne!(driver.station().association_epoch().unwrap(), epoch);
    // As in Linux, removed keys install again; the GTK takes the RSC the OS
    // learned in its key exchange, here PN1.
    driver
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .install_key(
            false,
            2,
            &[2; 16],
            ReplaySeed::uniform(wifi::security::PacketNumber::new(1).unwrap()),
        )
        .unwrap();
    let next = driver.station().association_epoch().unwrap();
    driver.authorize_port(next).unwrap();
    incoming(&model, &data(true), Some((1, 2, 1)), false);
    assert!(matches!(
        driver.poll(frontend_tick(41), &Source, &mut Policy(true)),
        Err(ClientError::Data(station::data::Error::Crypto(
            wifi::security::Error::Replay
        )))
    ));
    incoming(&model, &data(true), Some((1, 2, 2)), false);
    assert!(matches!(
        driver
            .poll(frontend_tick(42), &Source, &mut Policy(true))
            .unwrap()
            .as_slice(),
        [Event::Ingress(station::data::Ingress::Data(_))]
    ));
    assert!(matches!(
        driver.transmit(
            epoch,
            &eth(0x0800),
            TxClass::Data,
            &Source,
            &mut Policy(true)
        ),
        Err(ClientError::Data(station::data::Error::Stale))
    ));
    driver.disconnect().unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    driver.stop().unwrap();
    assert_eq!(driver.phase(), ClientPhase::ControllerDrain);
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.stop_after_quiesce() }.unwrap();
    assert_eq!(driver.phase(), ClientPhase::Stopped);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn combo_initial_boot_is_consumed_once_on_success_or_uncertainty() {
    if crate::test_process::isolate_model() {
        return;
    }
    for failed in [false, true] {
        let mut f = NativeFixture::new(Chip::Mt7925);
        f.install().unwrap();
        if failed {
            f.model.lock().unwrap().initial_fail_at = Some(1);
        }
        let requests = Requests::new(&identity(Chip::Mt7925));
        let provider = catalog(&requests, patch_bytes(), ram(&clc_bytes(Chip::Mt7925)));
        let prepared = requests.load(&provider).unwrap();
        assert_eq!(block_on(f.controller.boot(&prepared)).is_err(), failed);
        let count = f.model.lock().unwrap().initial_commands.len();
        assert_eq!(count, 1);
        assert!(matches!(
            block_on(f.controller.boot(&prepared)),
            Err(BootError::WrongPhase)
        ));
        assert_eq!(f.model.lock().unwrap().initial_commands.len(), count);
        if !failed {
            f.retire();
        } else {
            drop(f.controller);
            assert_eq!(f.drops.load(Ordering::Relaxed), 0);
        }
    }
}

#[test]
fn failed_peer_ack_keeps_programmed_peer_without_packet_admission() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    model.response.lock().unwrap().status = 0xdeadbeef;
    assert!(matches!(
        driver.join(join(), profile(), 0, &Source, &mut Policy(true)),
        Err(ClientError::Firmware(0xdeadbeef))
    ));
    // Device already consumed and programmed the full peer command;
    // response failure cannot become a clean rollback, nor format permission.
    assert!(model.response.lock().unwrap().peers.contains_key(&1));
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, 0);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn wrong_roc_token_never_publishes_authentication_or_reuses_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    model.response.lock().unwrap().wrong_roc_token = true;
    assert!(matches!(
        driver.join(join(), profile(), 0, &Source, &mut Policy(true)),
        Err(ClientError::GrantTimeout)
    ));
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, 0);
    // The generic response and foreign grant do not authorize channel use.
    assert!(model.response.lock().unwrap().roc_channel.is_none());
    assert!(
        model
            .response
            .lock()
            .unwrap()
            .initial_commands
            .iter()
            .any(|p| p[34] == 0x27 && p[52] == 1)
    );
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
}

#[test]
fn peer_removal_cannot_retire_outstanding_packet_owners() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    associated(&mut driver, &model, 0);
    let epoch = driver.station().association_epoch().unwrap();
    driver
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .install_key(false, 2, &[2; 16], ReplaySeed::ZERO)
        .unwrap();
    driver.authorize_port(epoch).unwrap();
    driver
        .transmit(
            epoch,
            &eth(0x0800),
            TxClass::Data,
            &Source,
            &mut Policy(true),
        )
        .unwrap();
    driver.disconnect().unwrap();
    assert!(matches!(
        // SAFETY: the model device is stopped and drained, so nothing still uses what this
        // releases.
        unsafe { driver.retire_peer_after_drain() },
        Err(ClientError::NeedsDrain)
    ));
    // Even real remove ACKs cannot release still-reachable data mappings/key
    // owners. Complete hardware users before independent RX/work drain.
    finish_tx(&model);
    driver
        .poll(frontend_tick(10), &Source, &mut Policy(true))
        .unwrap();
    driver.poll_released().unwrap();
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
}

#[test]
fn partial_receive_chain_blocks_peer_generation_retirement() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    associated(&mut driver, &model, 0);
    driver
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .install_key(false, 2, &[2; 16], ReplaySeed::ZERO)
        .unwrap();
    let bytes = incoming_bytes(&data(false), Some((1, 0, 9)), false);
    device_receive(&model.queues[&0xd4520], &bytes[..40], false, 0);
    driver
        .poll(frontend_tick(10), &Source, &mut Policy(true))
        .unwrap();
    driver.disconnect().unwrap();
    assert!(matches!(
        // SAFETY: the model device is stopped and drained, so nothing still uses what this
        // releases.
        unsafe { driver.retire_peer_after_drain() },
        Err(ClientError::NeedsDrain)
    ));
    // End and consume the old chain before the independent old-user
    // drain can check a new key/peer generation. No old bytes are retagged.
    device_receive(&model.queues[&0xd4520], &bytes[40..], true, 0);
    assert!(
        driver
            .poll(frontend_tick(11), &Source, &mut Policy(true))
            .is_err()
    );
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
}

#[test]
fn roc_expiry_before_packet_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::bus::dma::memory::{CacheAttribute, MemoryError, PageSource, PinnedPages};
    struct DelayedSource {
        allocations: AtomicUsize,
        clock: crate::Clock,
    }
    impl PageSource for DelayedSource {
        fn allocate(
            &self,
            bytes: usize,
            alignment: usize,
            attribute: CacheAttribute,
        ) -> Result<PinnedPages, MemoryError> {
            if self.allocations.fetch_add(1, Ordering::Relaxed) == 0 {
                // Expire the 1000 ms grant during payload allocation.
                crate::test_clock::advance(self.clock, 1_100_000);
            }
            Source.allocate(bytes, alignment, attribute)
        }
    }
    let (mut driver, model) = setup();
    let _clock = crate::test_clock::virtualize();
    let before = model.memory.lock().unwrap().mappings.len();
    let source = DelayedSource {
        allocations: AtomicUsize::new(0),
        clock: driver.state().controller.state().control.clock,
    };
    assert!(matches!(
        driver.join(join(), profile(), 0, &source, &mut Policy(true)),
        Err(ClientError::GrantTimeout)
    ));
    assert_eq!(source.allocations.load(Ordering::Relaxed), 2);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, 0);
    assert_eq!(model.memory.lock().unwrap().mappings.len(), before);
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    // This never-published packet may unmap, but the real programmed peer,
    // grant and all controller/IRQ resources remain held.
    assert!(model.response.lock().unwrap().peers.contains_key(&1));
    assert_eq!(model.response.lock().unwrap().roc_channel, Some(36));
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

pub(crate) fn retired_group_for_reinstall() -> (
    Client<Bar, Q, NativeGuard>,
    Model,
    wifi::security::KeyIdentity,
) {
    let (mut driver, model) = setup();
    associated(&mut driver, &model, 0);
    driver
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    let old = driver
        .install_key(false, 2, &[2; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .authorize_port(driver.station().association_epoch().unwrap())
        .unwrap();
    incoming(&model, &data(true), Some((1, 2, 7)), false);
    driver
        .poll(frontend_tick(12), &Source, &mut Policy(true))
        .unwrap();
    driver.disconnect().unwrap();
    // SAFETY: the model device is stopped and drained, so nothing still uses what this
    // releases.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    associated(&mut driver, &model, 30);
    driver
        .install_key(true, 0, &[3; 16], ReplaySeed::ZERO)
        .unwrap();
    (driver, model, old)
}
#[test]
fn reinstalled_gtk_keeps_its_slot_and_takes_the_os_rsc() {
    if crate::test_process::isolate_model() {
        return;
    }
    for rsc in [3, 9] {
        let (mut driver, model, old) = retired_group_for_reinstall();
        let seed = ReplaySeed::uniform(wifi::security::PacketNumber::new(rsc).unwrap());
        let key = driver.install_key(false, 2, &[2; 16], seed).unwrap();
        assert_eq!(key.slot(), old.slot());
        assert_eq!(key.native_epoch(), old.native_epoch());
        assert_ne!(key.generation(), old.generation());
        driver
            .authorize_port(driver.station().association_epoch().unwrap())
            .unwrap();
        let floor = rsc;
        incoming(&model, &data(true), Some((1, 2, floor)), false);
        assert!(matches!(
            driver.poll(frontend_tick(41), &Source, &mut Policy(true)),
            Err(ClientError::Data(station::data::Error::Crypto(
                wifi::security::Error::Replay
            )))
        ));
        incoming(&model, &data(true), Some((1, 2, floor + 1)), false);
        assert!(matches!(
            driver
                .poll(frontend_tick(42), &Source, &mut Policy(true))
                .unwrap()
                .as_slice(),
            [Event::Ingress(station::data::Ingress::Data(_))]
        ));
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
    }
}
#[test]
fn uncertain_gtk_install_never_authorizes_port_or_drops_real_peer_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model, _) = retired_group_for_reinstall();
    model.response.lock().unwrap().status = 0xdead;
    assert!(matches!(
        driver.install_key(false, 2, &[2; 16], ReplaySeed::ZERO),
        Err(ClientError::Key(
            crate::wifi::mediatek::mailbox::key::Error::Firmware(0xdead)
        ))
    ));
    assert_eq!(driver.phase(), ClientPhase::Quarantined);
    assert!(matches!(
        driver.authorize_port(driver.station().association_epoch().unwrap()),
        Err(ClientError::WrongPhase)
    ));
    assert!(matches!(
        driver.install_key(false, 2, &[2; 16], ReplaySeed::ZERO),
        Err(ClientError::WrongPhase)
    ));
    // Model processes the install before returning its nonzero status;
    // no RejectedAndEmpty claim can release the programmed GTK or old floors.
    assert!(
        model
            .response
            .lock()
            .unwrap()
            .keys
            .contains_key(&(profile().interface.broadcast_wcid as u8))
    );
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn local_advertisement_uses_kept_real_report_image_eeprom_and_policy_inputs() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (driver, model) = setup();
    let count = model.response.lock().unwrap().client_commands.len();
    for band in [Band::Ghz2, Band::Ghz5] {
        let a = driver.local_advertisement(band).unwrap();
        assert_eq!(a.antenna_mask(), 3);
        assert_eq!(a.source_country(), *b"US");
        assert_eq!(a.capabilities().he().unwrap().mcs_80.rx_nss(), 2);
        assert_eq!(a.validate_capabilities(a.capabilities()), Ok(()));
        assert_eq!(a.he_source_maps()[0].rx, 0xfffa);
        assert_eq!(a.he_source_maps()[1].tx, 0xfffa);
        assert_eq!(a.he_source_maps()[2].rx, 0);
        assert_eq!(a.he_source_maps()[2].tx, 0);
        if band == Band::Ghz2 {
            assert_eq!(a.capabilities().he().unwrap().mcs_160, None);
        }
        assert!(a.capabilities().eht().unwrap().mcs.bw320.is_none());
        assert_eq!(a.capabilities().he().unwrap().phy[0] & 16, 0);
    }
    assert!(matches!(
        driver.local_advertisement(Band::Ghz6),
        Err(crate::wifi::mediatek::bootstrap::client::LocalError::UnsupportedBand)
    ));
    // A new unqueried model reply is not a current bound hardware observation.
    model.response.lock().unwrap().capability_payload = Some(vec![0xff; 100]);
    assert_eq!(
        driver
            .local_advertisement(Band::Ghz5)
            .unwrap()
            .antenna_mask(),
        3
    );
    assert_eq!(model.response.lock().unwrap().client_commands.len(), count);
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, 0);
    assert_eq!(driver.phase(), ClientPhase::Idle);
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
}
// Key ACK + completed requester-DMA management RX model.
use super::{Integrity, Mpdu, PacketNumber, SecurityStatus};
use station::data::Ingress;
fn completed(
    driver: &mut Client<Bar, Q, NativeGuard>,
    model: &Model,
    raw: &[u8],
) -> Result<Mpdu, ClientError> {
    device_receive(&model.queues[&0xd4520], raw, true, 0);
    let received = driver
        .state_mut()
        .controller
        .poll_data()
        .map_err(ClientError::Bootstrap)?
        .unwrap();
    Client::normalized(
        driver.state_mut(),
        &received.bytes,
        false,
        &mut Policy(true),
    )
}
fn raw(pn: u64) -> Vec<u8> {
    incoming_bytes(&response(0x40c0, &[7, 0]), Some((1, 0, pn)), false)
}
#[test]
fn completed_ptk_management_rx_preserves_data_pn_and_retires_with_peer() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut driver, model) = setup();
    associated(&mut driver, &model, 0);
    let ptk = driver
        .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
        .unwrap();
    driver
        .install_key(false, 2, &[2; 16], ReplaySeed::ZERO)
        .unwrap();
    let epoch = driver.station().association_epoch().unwrap();
    driver.authorize_port(epoch).unwrap();
    for case in 0..12 {
        let mut bad = raw(9);
        match case {
            0 => bad[4] = 2,                     // wrong raw peer WCID
            1 => bad[6] |= 1 << 5,               // wrong key index
            2 => bad[10] = (bad[10] & !31) | 11, // GCMP status cannot bind CCMP PTK
            3 => bad[7] |= 1 << 1,               // ICV failure
            4 => {
                bad.drain(32..48);
                bad[6] &= !1;
            } // no group1 PN
            5 => bad[6] |= 1 << 7,               // cipher mismatch
            6 => bad[7] |= 1,                    // cipher length mismatch
            7 => bad[48 + 10] ^= 2,              // wrong transmitter
            8 => bad[48 + 16] ^= 2,              // wrong BSSID
            9 => bad[48 + 4] ^= 2,               // wrong local receiver
            10 => bad[13] = 40,                  // wrong channel
            _ => {
                bad.pop();
            } // incomplete reason code
        }
        if let Ok(frame) = completed(&mut driver, &model, &bad) {
            assert!(
                driver
                    .state_mut()
                    .ptk
                    .as_mut()
                    .unwrap()
                    .authorize_unicast_management(&frame, LOCAL)
                    .is_err(),
                "case {case}"
            );
        }
    }
    let frame = completed(&mut driver, &model, &raw(9)).unwrap();
    assert_eq!(frame.security.key, Some(ptk));
    assert_eq!(frame.radio.rate.layout, 0x7925);
    let proof = driver
        .state_mut()
        .ptk
        .as_mut()
        .unwrap()
        .authorize_unicast_management(&frame, LOCAL)
        .unwrap();
    assert_eq!(proof.key(), ptk);
    assert_eq!(proof.body(), &[7, 0]);
    assert_eq!(proof.packet_number().value(), 9);
    // This crypto proof itself changes neither station phase nor controlled port.
    assert_eq!(driver.phase(), ClientPhase::Associated);
    let again = completed(&mut driver, &model, &raw(9)).unwrap();
    assert!(matches!(
        driver
            .state_mut()
            .ptk
            .as_mut()
            .unwrap()
            .authorize_unicast_management(&again, LOCAL),
        Err(wifi::security::Error::Replay)
    ));
    incoming(&model, &data(false), Some((1, 0, 9)), false);
    assert!(matches!(
        driver
            .poll(frontend_tick(12), &Source, &mut Policy(true))
            .unwrap()
            .as_slice(),
        [Event::Ingress(Ingress::Data(_))]
    ));
    // Native exact reinstall is a no-op and cannot rewind management counter16.
    assert_eq!(
        driver
            .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
            .unwrap(),
        ptk
    );
    assert!(matches!(
        driver
            .state_mut()
            .ptk
            .as_mut()
            .unwrap()
            .authorize_unicast_management(&again, LOCAL),
        Err(wifi::security::Error::Replay)
    ));
    driver.disconnect().unwrap();
    assert!(matches!(
        driver
            .state_mut()
            .ptk
            .as_mut()
            .unwrap()
            .authorize_unicast_management(&frame, LOCAL),
        Err(wifi::security::Error::NoInstalledKey)
    ));
    // SAFETY: explicit model peer removal ACK, no firmware/DMA/IRQ/deferred old users.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    associated(&mut driver, &model, 30);
    let next = driver
        .install_key(true, 0, &[3; 16], ReplaySeed::ZERO)
        .unwrap();
    assert_ne!(next.generation(), ptk.generation());
    assert!(matches!(
        driver
            .state_mut()
            .ptk
            .as_mut()
            .unwrap()
            .authorize_unicast_management(&frame, LOCAL),
        Err(wifi::security::Error::WrongKey)
    ));
    let fresh = completed(&mut driver, &model, &raw(1)).unwrap();
    driver
        .state_mut()
        .ptk
        .as_mut()
        .unwrap()
        .authorize_unicast_management(&fresh, LOCAL)
        .unwrap();
    driver.disconnect().unwrap();
    // SAFETY: the model has explicitly completed removal and all old users.
    unsafe { driver.retire_peer_after_drain() }.unwrap();
    driver.stop().unwrap();
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: independent complete modeled controller/radio/DMA/work quiescence.
    unsafe { driver.stop_after_quiesce() }.unwrap();
    drop(driver);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}

#[test]
fn uncertain_ptk_publication_never_authenticates_management_or_releases_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    for dropped in [false, true] {
        let (mut driver, model) = setup();
        associated(&mut driver, &model, 0);
        // First real GTK ACK also completes the BSS cipher update. Faults below
        // therefore reach the real PTK publication rather than an earlier BSS.
        let gtk = driver
            .install_key(false, 2, &[2; 16], ReplaySeed::ZERO)
            .unwrap();
        {
            let mut fault = model.response.lock().unwrap();
            if dropped {
                fault.drop = true;
            } else {
                fault.status = 0xdead;
            }
        }
        assert!(
            driver
                .install_key(true, 0, &[1; 16], ReplaySeed::ZERO)
                .is_err()
        );
        assert_eq!(driver.phase(), ClientPhase::Quarantined);
        assert!(model.response.lock().unwrap().keys.contains_key(&1));
        assert!(completed(&mut driver, &model, &raw(9)).is_err());
        let copied = Mpdu::new(
            response(0x40c0, &[7, 0]),
            wifi::RadioStatus {
                band: wifi::Band::Ghz5,
                channel: 36,
                frequency_mhz: 5180,
                signal_dbm: None,
                chain_signal_dbm: [None; 4],
                rate: wifi::RawRate {
                    layout: 0x7925,
                    words: [0; 4],
                },
                rx_rate: None,
                tsf: None,
            },
            SecurityStatus {
                key: Some(gtk),
                pn: PacketNumber::new(9),
                decrypted: true,
                integrity: Integrity::Verified,
                iv_stripped: true,
                mic_stripped: true,
                ..SecurityStatus::default()
            },
        )
        .unwrap();
        assert!(matches!(
            driver
                .state_mut()
                .ptk
                .as_mut()
                .unwrap()
                .authorize_unicast_management(&copied, LOCAL),
            Err(wifi::security::Error::NoInstalledKey)
        ));
        drop(driver);
        assert_eq!(model.drops.load(Ordering::Relaxed), 0);
        assert!(!model.memory.lock().unwrap().mappings.is_empty());
    }
}
