extern crate std;
use super::super::tests::{done, setup_modern, setup_modern_radio};
use super::*;
use crate::bus::dma::memory::{CacheAttribute, MemoryError, PageGeometry, PinnedPages};
use crate::wifi::mediatek::{
    bootstrap::{
        client::{
            modern::tests::{modern_inputs, modern_inputs_cipher},
            tests::*,
        },
        tests::*,
    },
    ring::{rx::tests::device_receive, tests::Source},
};
use core::sync::atomic::{AtomicUsize, Ordering};
use reg::database::tests::country;

struct Provider(Arc<AtomicUsize>, core::sync::atomic::AtomicBool);
impl PageSource for Provider {
    fn page_geometry(&self) -> Result<PageGeometry, MemoryError> {
        Source.page_geometry()
    }
    fn allocate(
        &self,
        bytes: usize,
        alignment: usize,
        attribute: CacheAttribute,
    ) -> Result<PinnedPages, MemoryError> {
        if self.1.load(Ordering::Relaxed) {
            return Err(MemoryError::OutOfMemory);
        }
        Source.allocate(bytes, alignment, attribute)
    }
}
impl Drop for Provider {
    fn drop(&mut self) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}
type TestLink = ManagedLink<Bar, Q, NativeGuard, Provider>;
fn setup_link() -> (TestLink, Model, Join, scan::result::Bss, Arc<AtomicUsize>) {
    let (client, model) = setup_modern();
    let (join, bss) = modern_inputs(&client);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let drops = Arc::new(AtomicUsize::new(0));
    let link = ManagedLink::new(
        client,
        Provider(drops.clone(), core::sync::atomic::AtomicBool::new(false)),
    )
    .unwrap();
    (link, model, join, bss, drops)
}
fn scan_join(
    link: &mut TestLink,
    model: &Model,
    join: Join,
    bss: scan::result::Bss,
) -> scan::result::Bss {
    assert_eq!(link.mac(), LOCAL);
    assert!(!link.carrier());
    link.begin_scan(
        scan::Request::new(vec![CHANNEL], vec![], vec![]).unwrap(),
        1_000_000,
    )
    .unwrap();
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&bss.capabilities.to_le_bytes());
    body.extend_from_slice(&bss.elements);
    let mut beacon = response(0x80, &body);
    beacon[4..10].copy_from_slice(&Mac::BROADCAST.0);
    incoming(model, &beacon, None, false);
    link.service().unwrap();
    let handle = link.bss().next().unwrap().0;
    // The link's BSS list plans a split scan's 6 GHz part.
    let six = wifi::Channel {
        band: Band::Ghz6,
        number: 5,
        frequency_mhz: 5975,
    };
    let options = scan::six_ghz::Options {
        colocated: true,
        bssid: None,
    };
    let split = scan::Request::new(vec![CHANNEL, six], vec![vec![]], vec![]).unwrap();
    let planned = link.six_ghz_scan(&split, options).unwrap().unwrap();
    assert_eq!(planned.channels(), &[six]);
    done(model);
    link.service().unwrap();
    // SAFETY: complete source-model terminal scan and old-user drain.
    unsafe { link.retire_scan_after_drain() }.unwrap();
    link.join(handle, join, profile()).unwrap();
    bss
}
fn associate(link: &mut TestLink, model: &Model, join: Join, bss: scan::result::Bss) {
    let bss = scan_join(link, model, join, bss);
    assert_eq!(finish_tx(model)[0][..2], [0xb0, 0]);
    link.service().unwrap();
    incoming(model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    link.service().unwrap();
    assert_eq!(finish_tx(model)[0][..2], [0, 0]);
    link.service().unwrap();
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&bss.elements);
    incoming(model, &response(0x10, &assoc), None, false);
    link.service().unwrap();
    assert_eq!(link.phase(), Phase::AwaitingModernAdmission);
    link.configure().unwrap();
    assert_eq!(link.phase(), Phase::Associated);
    assert!(!link.carrier());
}
fn secure(link: &mut TestLink) -> KeyIdentity {
    let ptk = link
        .install_key(true, 0, &[4; 16], ReplaySeed::ZERO)
        .unwrap();
    assert!(!link.carrier());
    link.install_key(false, 2, &[5; 16], ReplaySeed::ZERO)
        .unwrap();
    assert!(!link.carrier());
    link.authorize_port(link.station().association_epoch().unwrap())
        .unwrap();
    assert!(link.carrier());
    ptk
}
fn drain_notices(link: &mut TestLink) -> Vec<LinkNotice<NativeGuard>> {
    let mut out = Vec::new();
    loop {
        while let Some(n) = link.take_notice() {
            out.push(n);
        }
        link.service().unwrap();
        if link.queues().notices == 0 {
            break;
        }
    }
    out
}
/// The OS's deauthentication, returning the frames that left first.
fn depart(link: &mut TestLink, model: &Model) -> Vec<Vec<u8>> {
    link.disconnect().unwrap();
    // The deauthentication and any block-ack frame still on the air
    // retire before removal.
    let mut sent = Vec::new();
    while link.phase() == Phase::Removing {
        let channel = model.response.lock().unwrap().home_channel;
        sent.extend(finish_tx_on(model, channel.unwrap_or(36)));
        link.service().unwrap();
    }
    sent
}
fn stop(link: &mut TestLink, model: &Model) {
    depart(link, model);
    retire(link, model);
}
fn retire(link: &mut TestLink, model: &Model) {
    assert!(!link.carrier());
    assert_eq!(link.phase(), Phase::NeedsDrain);
    // SAFETY: source-model peer/key removal and no remaining native old users.
    unsafe { link.retire_peer_after_drain() }.unwrap();
    link.stop().unwrap();
    assert_eq!(link.phase(), Phase::ControllerDrain);
    model.response.lock().unwrap().clc_processing = false;
    model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
    // SAFETY: independent model controller, firmware, DMA and IRQ/work fence.
    unsafe { link.stop_after_quiesce() }.unwrap();
    assert_eq!(link.phase(), Phase::Stopped);
}
fn control() -> Vec<u8> {
    let mut frame = eth(0x888e);
    frame[..6].copy_from_slice(&AP.0);
    frame
}
fn incoming_control(model: &Model, pn: Option<u64>, marker: u8) {
    let mut bytes = data(false);
    bytes[24 + 6..24 + 8].copy_from_slice(&0x888eu16.to_be_bytes());
    *bytes.last_mut().unwrap() = marker;
    if pn.is_none() {
        bytes[1] &= !0x40;
    }
    let mut raw = incoming_bytes(&bytes, pn.map(|n| (1, 0, n)), false);
    raw[4] = 1; // source WCID remains required for plaintext data too
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
}
#[test]
fn link_routes_control_port_then_real_ethernet_and_retires_exact_packets() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    let original_epoch = link.station().association_epoch().unwrap();
    assert!(matches!(
        link.transmit(&eth(0x800)),
        Err(LinkError::NoCarrier)
    ));
    // EAPOL is available before keys, kept for the real OS supplicant and
    // never promoted to ordinary Ethernet data or handshake completion.
    incoming_control(&model, None, 0x55);
    link.service().unwrap();
    assert!(link.receive().unwrap().is_none());
    let notices = drain_notices(&mut link);
    assert!(notices.iter().any(|n|n.association==Some(original_epoch)&&matches!(&n.event,LinkEvent::Controller(Event::Ingress(Ingress::ControlPort{ethernet,encrypted:false,key:None})) if ethernet.last()==Some(&0x55))));
    assert!(!link.carrier());
    let plain = link.send_control(&control(), None).unwrap();
    assert_eq!(finish_tx(&model)[0][1] & 0x40, 0);
    link.service().unwrap();
    assert!(drain_notices(&mut link).into_iter().any(
        |n| matches!(n.event,LinkEvent::Released(r) if r.receipt==plain&&r.context.key().is_none())
    ));
    let ptk = secure(&mut link);
    let before = model.queues[&0xd4300].lock().unwrap().head;
    assert!(matches!(
        link.transmit(&control()),
        Err(LinkError::Adapter(_))
    ));
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, before);
    let protected = link.send_control(&control(), Some(ptk)).unwrap();
    assert_ne!(finish_tx(&model)[0][1] & 0x40, 0);
    link.service().unwrap();
    assert!(drain_notices(&mut link).into_iter().any(|n|matches!(n.event,LinkEvent::Released(r) if r.receipt==protected&&r.context.key()==Some(ptk))));
    assert!(link.transmit(&eth(0x800)).is_ok());
    // Original DMA packet has no release until both consumption and TX-free.
    assert!(
        !drain_notices(&mut link)
            .iter()
            .any(|n| matches!(n.event, LinkEvent::Released(_)))
    );
    let frames = finish_tx(&model);
    assert_eq!(frames.len(), 1);
    assert_ne!(frames[0][1] & 0x40, 0);
    link.service().unwrap();
    assert!(
        drain_notices(&mut link)
            .iter()
            .any(|n| matches!(&n.event,LinkEvent::Released(r) if r.context.key()==Some(ptk)))
    );
    incoming(&model, &data(false), Some((1, 0, 1)), false);
    link.service().unwrap();
    let ethernet = link.receive().unwrap().unwrap();
    assert_eq!(&ethernet[..6], &LOCAL.0);
    assert_eq!(&ethernet[12..14], &0x800u16.to_be_bytes());
    // ieee80211_mgd_deauth tells the AP, in the clear without PMF.
    // An ADDBA request queued before it still leaves first.
    let departed = depart(&mut link, &model);
    let deauth = departed.last().unwrap();
    assert_eq!(deauth[..2], [0xc0, 0]);
    assert_eq!(&deauth[4..10], &AP.0);
    assert_eq!(deauth[24..], 3u16.to_le_bytes());
    retire(&mut link, &model);
    assert_eq!(link.mac(), LOCAL);
    drop(notices);
    drop(link);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
    assert!(model.memory.lock().unwrap().mappings.is_empty());
}
#[test]
fn an_open_network_carries_plaintext_ethernet_once_associated() {
    if crate::test_process::isolate_model() {
        return;
    }
    use station::{
        phy::{Mode, Width},
        wire::AssociationRequest,
    };
    let without_rsn = |elements: &[u8]| {
        let mut out = Vec::new();
        let mut rest = elements;
        while let [id, length, tail @ ..] = rest {
            let (data, tail) = tail.split_at(usize::from(*length));
            if *id != 48 {
                out.extend_from_slice(&[*id, *length]);
                out.extend_from_slice(data);
            }
            rest = tail;
        }
        out
    };
    let (client, model) = setup_modern();
    let (mut join, mut bss) = modern_inputs(&client);
    let local = client.local_advertisement(Band::Ghz5).unwrap();
    join.request = AssociationRequest::new_modern_with_local(
        0x01,
        10,
        without_rsn(join.request.elements()),
        local.capabilities().clone(),
        Mode::Eht,
        Width::Mhz80,
    )
    .unwrap();
    join.policy = station::PortPolicy::Open;
    bss.elements = without_rsn(&bss.elements);
    bss.capabilities = 0x01;
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let drops = Arc::new(AtomicUsize::new(0));
    let mut link = ManagedLink::new(
        client,
        Provider(drops, core::sync::atomic::AtomicBool::new(false)),
    )
    .unwrap();
    let bss = scan_join(&mut link, &model, join, bss);
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    link.service().unwrap();
    incoming(&model, &response(0xb0, &[0, 0, 2, 0, 0, 0]), None, false);
    link.service().unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    link.service().unwrap();
    let mut assoc = vec![0x01, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&bss.elements);
    incoming(&model, &response(0x10, &assoc), None, false);
    link.service().unwrap();
    link.configure().unwrap();
    assert_eq!(link.phase(), Phase::Associated);
    assert!(link.carrier());
    link.transmit(&eth(0x800)).unwrap();
    let frames = finish_tx(&model);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0][1] & 0x40, 0);
    let mut bytes = data(false);
    bytes[1] &= !0x40;
    let mut raw = incoming_bytes(&bytes, None, false);
    raw[4] = 1;
    device_receive(&model.queues[&0xd4520], &raw, true, 0);
    link.service().unwrap();
    let ethernet = link.receive().unwrap().unwrap();
    assert_eq!(&ethernet[12..14], &0x800u16.to_be_bytes());
}
#[test]
fn an_ap_without_wmm_or_ht_associates_in_legacy_mode_like_mac80211() {
    if crate::test_process::isolate_model() {
        return;
    }
    use station::{
        phy::{Mode, Width},
        wire::AssociationRequest,
    };
    // Neither side advertises HT, VHT, HE, EHT or WMM.
    let legacy = |elements: &[u8]| {
        let mut out = Vec::new();
        let mut rest = elements;
        while let [id, length, tail @ ..] = rest {
            let (data, tail) = tail.split_at(usize::from(*length));
            let wmm = *id == 221 && data.starts_with(&[0, 0x50, 0xf2, 2]);
            if !matches!(id, 45 | 61 | 191 | 192 | 255) && !wmm {
                out.extend_from_slice(&[*id, *length]);
                out.extend_from_slice(data);
            }
            rest = tail;
        }
        out
    };
    let (client, model) = setup_modern();
    let (mut join, mut bss) = modern_inputs(&client);
    let local = client.local_advertisement(Band::Ghz5).unwrap();
    join.request = AssociationRequest::new_modern_with_local(
        0x11,
        10,
        legacy(join.request.elements()),
        local.capabilities().clone(),
        Mode::Legacy,
        Width::Mhz20,
    )
    .unwrap();
    bss.elements = legacy(&bss.elements);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let drops = Arc::new(AtomicUsize::new(0));
    let mut link = ManagedLink::new(
        client,
        Provider(drops, core::sync::atomic::AtomicBool::new(false)),
    )
    .unwrap();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    // Data goes without a QoS header, as ieee80211_select_queue does
    // for a non-WMM station.
    link.transmit(&eth(0x800)).unwrap();
    let frames = finish_tx(&model);
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0][0], 0x08);
    assert_ne!(frames[0][1] & 0x40, 0);
}
#[test]
fn wpa1_tkip_carries_data_and_reports_michael_failures_like_mac80211() {
    if crate::test_process::isolate_model() {
        return;
    }
    use station::{
        phy::{Mode, Width},
        wire::AssociationRequest,
    };
    const WPA: [u8; 24] = [
        221, 22, 0, 0x50, 0xf2, 1, 1, 0, 0, 0x50, 0xf2, 2, 1, 0, 0, 0x50, 0xf2, 2, 1, 0, 0, 0x50,
        0xf2, 2,
    ];
    // TKIP keeps both sides in legacy mode; the AP's RSN becomes WPA1.
    let wpa = |elements: &[u8]| {
        let mut out = Vec::new();
        let mut rest = elements;
        while let [id, length, tail @ ..] = rest {
            let (data, tail) = tail.split_at(usize::from(*length));
            let wmm = *id == 221 && data.starts_with(&[0, 0x50, 0xf2, 2]);
            if !matches!(id, 45 | 48 | 61 | 191 | 192 | 255) && !wmm {
                out.extend_from_slice(&[*id, *length]);
                out.extend_from_slice(data);
            }
            rest = tail;
        }
        out.extend_from_slice(&WPA);
        out
    };
    let (client, model) = setup_modern();
    let (mut join, mut bss) = modern_inputs(&client);
    let local = client.local_advertisement(Band::Ghz5).unwrap();
    join.request = AssociationRequest::new_modern_with_local(
        0x11,
        10,
        wpa(join.request.elements()),
        local.capabilities().clone(),
        Mode::Legacy,
        Width::Mhz20,
    )
    .unwrap();
    bss.elements = wpa(&bss.elements);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let drops = Arc::new(AtomicUsize::new(0));
    let mut link = ManagedLink::new(
        client,
        Provider(drops, core::sync::atomic::AtomicBool::new(false)),
    )
    .unwrap();
    associate(&mut link, &model, join, bss);
    let ptk = link
        .install_key(true, 0, &[4; 32], ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 1, &[5; 32], ReplaySeed::ZERO)
        .unwrap();
    link.authorize_port(link.station().association_epoch().unwrap())
        .unwrap();
    assert!(link.carrier());
    link.transmit(&eth(0x800)).unwrap();
    let frames = finish_tx(&model);
    assert_eq!((frames[0][0], frames[0][1] & 0x40), (0x08, 0x40));
    link.service().unwrap();
    drain_notices(&mut link);
    // MT_CIPHER_TKIP with its TSC; then the same frame failing Michael.
    let tkip = |pn: u64, mic_error: bool| {
        let mut raw = incoming_bytes(&data(false), Some((1, 0, pn)), false);
        let mut r1 = u32::from_le_bytes(raw[4..8].try_into().unwrap());
        let mut r2 = u32::from_le_bytes(raw[8..12].try_into().unwrap());
        r2 = (r2 & !(31 << 16)) | (2 << 16);
        if mic_error {
            r1 |= 1 << 26;
        }
        raw[4..8].copy_from_slice(&r1.to_le_bytes());
        raw[8..12].copy_from_slice(&r2.to_le_bytes());
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
    };
    tkip(1, false);
    link.service().unwrap();
    let ethernet = link.receive().unwrap().unwrap();
    assert_eq!(&ethernet[12..14], &0x800u16.to_be_bytes());
    tkip(2, true);
    link.service().unwrap();
    assert!(link.receive().unwrap().is_none());
    assert!(drain_notices(&mut link).iter().any(|n| matches!(
        n.event,
        LinkEvent::Controller(Event::MichaelMicFailure {
            transmitter: AP,
            group: false,
            key_index: Some(0),
        })
    )));
    assert_eq!(ptk.cipher(), crate::wifi::security::Cipher::Tkip);
}
#[test]
fn static_wep_with_shared_key_auth_carries_data_like_mt7925_set_key() {
    if crate::test_process::isolate_model() {
        return;
    }
    use crate::wifi::security::{Cipher, wep};
    use station::{
        phy::{Mode, Width},
        wire::{self, AssociationRequest},
    };
    // WEP keeps both sides in legacy mode, without RSN.
    let legacy = |elements: &[u8]| {
        let mut out = Vec::new();
        let mut rest = elements;
        while let [id, length, tail @ ..] = rest {
            let (data, tail) = tail.split_at(usize::from(*length));
            let wmm = *id == 221 && data.starts_with(&[0, 0x50, 0xf2, 2]);
            if !matches!(id, 45 | 48 | 61 | 191 | 192 | 255) && !wmm {
                out.extend_from_slice(&[*id, *length]);
                out.extend_from_slice(data);
            }
            rest = tail;
        }
        out
    };
    let key = wep::Key::new(&[3; 13], 1).unwrap();
    let (client, model) = setup_modern();
    let (mut join, mut bss) = modern_inputs(&client);
    let local = client.local_advertisement(Band::Ghz5).unwrap();
    join.request = AssociationRequest::new_modern_with_local(
        0x11,
        10,
        legacy(join.request.elements()),
        local.capabilities().clone(),
        Mode::Legacy,
        Width::Mhz20,
    )
    .unwrap()
    .with_wep(key.clone(), wire::SHARED_KEY, 0)
    .unwrap();
    bss.elements = legacy(&bss.elements);
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let drops = Arc::new(AtomicUsize::new(0));
    let mut link = ManagedLink::new(
        client,
        Provider(drops, core::sync::atomic::AtomicBool::new(false)),
    )
    .unwrap();
    let bss = scan_join(&mut link, &model, join, bss);
    assert_eq!(finish_tx(&model)[0][24..], [1, 0, 1, 0, 0, 0]);
    link.service().unwrap();
    incoming(
        &model,
        &response(0xb0, &[1, 0, 2, 0, 0, 0, 16, 2, 0xaa, 0xbb]),
        None,
        false,
    );
    link.service().unwrap();
    // Transaction 3 leaves as mac80211 encrypted it, under the first IV.
    let third = finish_tx(&model);
    let expected = wire::shared_key_response(LOCAL, AP, 0, &[0xaa, 0xbb], &key, [0, 0, 1]).unwrap();
    assert_eq!(third[0][..2], [0xb0, 0x40]);
    assert_eq!(third[0][24..], expected[24..]);
    link.service().unwrap();
    incoming(&model, &response(0xb0, &[1, 0, 4, 0, 0, 0]), None, false);
    link.service().unwrap();
    assert_eq!(finish_tx(&model)[0][..2], [0, 0]);
    link.service().unwrap();
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&bss.elements);
    incoming(&model, &response(0x10, &assoc), None, false);
    link.service().unwrap();
    link.configure().unwrap();
    assert_eq!(link.phase(), Phase::Associated);
    // The default key goes to both the AP's and the BSS's records.
    let ptk = link
        .install_key(true, 1, key.material(), ReplaySeed::ZERO)
        .unwrap();
    link.install_key(false, 1, key.material(), ReplaySeed::ZERO)
        .unwrap();
    assert_eq!(ptk.cipher(), Cipher::Wep104);
    link.authorize_port(link.station().association_epoch().unwrap())
        .unwrap();
    assert!(link.carrier());
    link.transmit(&eth(0x800)).unwrap();
    let frames = finish_tx(&model);
    assert_eq!((frames[0][0], frames[0][1] & 0x40), (0x08, 0x40));
    link.service().unwrap();
    // MT_CIPHER_WEP104 has no packet number, so a repeat is delivered;
    // an ICV error is not.
    let wep = |icv_error: bool| {
        let mut raw = incoming_bytes(&data(false), Some((1, 1, 0)), false);
        let mut r1 = u32::from_le_bytes(raw[4..8].try_into().unwrap());
        let mut r2 = u32::from_le_bytes(raw[8..12].try_into().unwrap());
        r2 = (r2 & !(31 << 16)) | (5 << 16);
        if icv_error {
            r1 |= 1 << 25;
        }
        raw[4..8].copy_from_slice(&r1.to_le_bytes());
        raw[8..12].copy_from_slice(&r2.to_le_bytes());
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
    };
    for _ in 0..2 {
        wep(false);
        link.service().unwrap();
        let ethernet = link.receive().unwrap().unwrap();
        assert_eq!(&ethernet[12..14], &0x800u16.to_be_bytes());
    }
    // The ICV error is mt7925_mac_fill_rx's RX_FLAG_ONLY_MONITOR: never
    // delivered, and counted as a refused frame.
    wep(true);
    link.service().unwrap();
    assert_eq!(link.rejected_frames(), 1);
    assert!(link.receive().unwrap().is_none());
}
#[test]
fn aborting_a_join_deauthenticates_like_ieee80211_mgd_deauth() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, _) = setup_link();
    scan_join(&mut link, &model, join, bss);
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    link.service().unwrap();
    let departed = depart(&mut link, &model);
    assert_eq!(departed.len(), 1);
    assert_eq!(departed[0][..2], [0xc0, 0]);
    assert_eq!(departed[0][24..], 3u16.to_le_bytes());
    retire(&mut link, &model);
}
#[test]
fn the_link_carries_each_aes_cipher_like_mt7925_set_key() {
    if crate::test_process::isolate_model() {
        return;
    }
    for (suite, length) in [(10, 32), (8, 16), (9, 32)] {
        let (client, model) = setup_modern();
        let (join, bss) = modern_inputs_cipher(&client, suite);
        let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
        let provider = Provider(
            Arc::new(AtomicUsize::new(0)),
            core::sync::atomic::AtomicBool::new(false),
        );
        let mut link = ManagedLink::new(client, provider).unwrap();
        associate(&mut link, &model, join, bss);
        for (pairwise, index, value) in [(true, 0, 1), (false, 2, 2)] {
            link.install_key(pairwise, index, &vec![value; length], ReplaySeed::ZERO)
                .unwrap();
        }
        link.authorize_port(link.station().association_epoch().unwrap())
            .unwrap();
        link.transmit(&eth(0x800)).unwrap();
        assert_ne!(finish_tx(&model)[0][1] & 0x40, 0);
        stop(&mut link, &model);
    }
}
#[test]
fn full_data_and_control_queues_keep_native_batches_in_order() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    drop(drain_notices(&mut link));
    for n in 1..=65 {
        let mut bytes = data(false);
        *bytes.last_mut().unwrap() = n;
        incoming(&model, &bytes, Some((1, 0, u64::from(n))), false);
        link.service().unwrap();
    }
    assert_eq!(link.queues().data_frames, DATA_FRAMES);
    assert!(link.queues().backpressured);
    let before = model.queues[&0xd4300].lock().unwrap().head;
    assert!(matches!(
        link.transmit(&eth(0x800)),
        Err(LinkError::Adapter(_))
    ));
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, before);
    for n in 1..=65 {
        assert_eq!(link.receive().unwrap().unwrap().last(), Some(&n));
    }
    assert!(link.receive().unwrap().is_none());
    for n in 1..=65 {
        incoming_control(&model, Some(65 + u64::from(n)), n);
        link.service().unwrap();
    }
    assert_eq!(link.queues().notices, NOTICES);
    assert!(link.queues().backpressured);
    for n in 1..=65 {
        link.service().unwrap();
        let notice = link.take_notice().unwrap();
        assert!(
            matches!(notice.event,LinkEvent::Controller(Event::Ingress(Ingress::ControlPort{ethernet,encrypted:true,key:Some(_)})) if ethernet.last()==Some(&n))
        );
    }
    link.service().unwrap();
    assert_eq!(link.queues(), LinkProgress::default());
    // Carrier loss discards held data, keeping control notices separately.
    incoming(&model, &data(false), Some((1, 0, 131)), false);
    link.service().unwrap();
    assert_eq!(link.queues().data_frames, 1);
    stop(&mut link, &model);
    assert_eq!(link.queues().data_frames, 0);
    drop(link);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
}
#[test]
fn removal_waits_for_both_real_tx_facts_and_uncertain_stop_keeps_frontend_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    drop(drain_notices(&mut link));
    link.transmit(&eth(0x800)).unwrap();
    link.disconnect().unwrap();
    assert_eq!(link.phase(), Phase::Removing);
    assert!(!link.carrier());
    assert!(model.response.lock().unwrap().peers.contains_key(&1));
    finish_tx(&model);
    link.service().unwrap();
    assert_eq!(link.phase(), Phase::NeedsDrain);
    assert!(!model.response.lock().unwrap().peers.contains_key(&1));
    assert!(link.queues().notices > 0);
    // SAFETY: exact removal ACKs and all old model users have ended.
    unsafe { link.retire_peer_after_drain() }.unwrap();
    model.response.lock().unwrap().bss_status = 0xdead;
    assert!(link.stop().is_err());
    assert_eq!(link.phase(), Phase::Quarantined);
    let notices = link.queues().notices;
    assert!(notices > 0);
    assert!(!link.carrier());
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 0);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}
#[test]
fn unpublished_allocation_failure_and_bad_rx_never_forge_completion_or_close_valid_port() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    drop(drain_notices(&mut link));
    let before = model.queues[&0xd4300].lock().unwrap().head;
    let mappings = model.memory.lock().unwrap().mappings.len();
    link.state().source.1.store(true, Ordering::Relaxed);
    assert!(matches!(
        link.transmit(&eth(0x800)),
        Err(LinkError::Adapter(_))
    ));
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, before);
    assert_eq!(model.memory.lock().unwrap().mappings.len(), mappings);
    assert!(link.carrier());
    assert_eq!(link.queues().notices, 0);
    link.state().source.1.store(false, Ordering::Relaxed);
    incoming(&model, &data(false), Some((1, 0, 1)), false);
    assert!(link.receive().unwrap().is_some());
    // The replay is dropped and counted.
    incoming(&model, &data(false), Some((1, 0, 1)), false);
    assert!(link.receive().unwrap().is_none());
    assert_eq!(link.rejected_frames(), 1);
    link.transmit(&eth(0x800)).unwrap();
    finish_tx(&model);
    let mut bad = incoming_bytes(&data(false), Some((1, 0, 2)), false);
    bad[7] |= 2;
    device_receive(&model.queues[&0xd4520], &bad, true, 0);
    assert!(link.receive().unwrap().is_none());
    assert_eq!(link.rejected_frames(), 2);
    assert!(
        link.state()
            .client
            .client
            .state()
            .controller
            .state()
            .data_tx
            .as_ref()
            .unwrap()
            .outstanding_tokens()
            == 0
    );
    assert!(link.queues().notices > 0);
    assert!(link.carrier());
    incoming(&model, &data(false), Some((1, 0, 2)), false);
    assert!(link.receive().unwrap().is_some());
    stop(&mut link, &model);
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
}
#[test]
fn actual_2ghz_he20_profile_scans_associates_and_carries_ethernet_on_its_real_channel() {
    if crate::test_process::isolate_model() {
        return;
    }
    use station::{
        phy::{Capabilities, Mode, Role, Width},
        wire::AssociationRequest,
    };
    let channel = wifi::Channel {
        band: Band::Ghz2,
        number: 1,
        frequency_mhz: 2412,
    };
    let (client, model) = setup_modern_radio(false, channel);
    let actual = client.local_advertisement(Band::Ghz2).unwrap();
    let mut local = vec![0, 1, b'x'];
    let mut at = 0;
    while at < actual.elements().len() {
        let n = usize::from(actual.elements()[at + 1]);
        let ie = &actual.elements()[at..at + 2 + n];
        // A real frontend can select the HE subset. Preserve its
        // original source maps after parsing exactly the advertised bytes.
        if ie[0] != 255 || ie[2] != 108 {
            local.extend_from_slice(ie);
        }
        at += 2 + n;
    }
    local.extend_from_slice(&[
        48, 18, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 4, 1, 0, 0, 15, 172, 2,
    ]);
    let caps = Capabilities::parse(Band::Ghz2, Role::Station, &local)
        .unwrap()
        .with_local_he_source_maps(*actual.he_source_maps())
        .unwrap();
    let request = AssociationRequest::new_modern_with_local(
        0x11,
        10,
        local.clone(),
        caps,
        Mode::He,
        Width::Mhz20,
    )
    .unwrap();
    let mut ap = Vec::new();
    at = 0;
    while at < local.len() {
        let n = usize::from(local[at + 1]);
        let ie = &local[at..at + 2 + n];
        if ie[0] == 221 {
            ap.extend_from_slice(&[
                221, 24, 0, 0x50, 0xf2, 2, 1, 1, 0x80, 0, 3, 0xa4, 0, 0, 0x27, 0xa4, 0, 0, 0x42,
                0x43, 94, 0, 0x62, 0x32, 47, 0,
            ]);
        } else {
            let offset = ap.len();
            ap.extend_from_slice(ie);
            if ie[0] == 1 {
                for q in 0..4 {
                    ap[offset + 2 + q] |= 0x80;
                }
            }
        }
        at += 2 + n;
    }
    let mut ht = [0; 22];
    ht[0] = 1;
    ap.extend_from_slice(&[61, 22]);
    ap.extend_from_slice(&ht);
    ap.extend_from_slice(&[255, 7, 36, 3, 0, 0, 0, 0xff, 0xff, 5, 4, 0, 2, 0, 0]);
    let drops = Arc::new(AtomicUsize::new(0));
    let client = ManagedClient::new(client, country(), Environment::Indoor).unwrap();
    let mut link = ManagedLink::new(
        client,
        Provider(drops.clone(), core::sync::atomic::AtomicBool::new(false)),
    )
    .unwrap();
    let receive = |bytes: &[u8], pn: Option<u64>| {
        let mut raw = incoming_bytes(bytes, pn.map(|n| (1, 0, n)), false);
        raw[13] = 1;
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
    };
    link.begin_scan(
        scan::Request::new(vec![channel], vec![], vec![]).unwrap(),
        1_000_000,
    )
    .unwrap();
    let mut body = vec![0; 8];
    body.extend_from_slice(&100u16.to_le_bytes());
    body.extend_from_slice(&0x11u16.to_le_bytes());
    body.extend_from_slice(&ap);
    let mut beacon = response(0x80, &body);
    beacon[4..10].fill(255);
    receive(&beacon, None);
    link.service().unwrap();
    let handle = link.bss().next().unwrap().0;
    done(&model);
    link.service().unwrap();
    // SAFETY: completed model scan/radio/RX work independently drained.
    unsafe { link.retire_scan_after_drain() }.unwrap();
    let mut profile = profile();
    profile.channel = channel;
    profile.basic_rates = 15;
    profile.supported_rates = 0xfff;
    profile.slot_time = 20;
    let join = Join {
        bssid: AP,
        channel,
        policy: station::PortPolicy::Controlled,
        request,
    };
    link.join(handle, join, profile).unwrap();
    assert_eq!(finish_tx_on(&model, 1)[0][..2], [0xb0, 0]);
    link.service().unwrap();
    receive(&response(0xb0, &[0, 0, 2, 0, 0, 0]), None);
    link.service().unwrap();
    assert_eq!(finish_tx_on(&model, 1)[0][..2], [0, 0]);
    link.service().unwrap();
    let mut assoc = vec![0x11, 0, 0, 0, 1, 0xc0];
    assoc.extend_from_slice(&ap);
    receive(&response(0x10, &assoc), None);
    link.service().unwrap();
    link.configure().unwrap();
    secure(&mut link);
    // The authorized port enters power save first.
    link.service().unwrap();
    // A beacon turning on Short Slot Time moves the BSS to 9 us, as
    // mt7925_mcu_set_timing sends it.
    let mut short = beacon.clone();
    short[35] |= 0x04;
    receive(&short, None);
    link.service().unwrap();
    let command = model
        .response
        .lock()
        .unwrap()
        .client_commands
        .last()
        .unwrap()
        .clone();
    assert_eq!(command[48 + 4..48 + 14], [23, 0, 20, 0, 1, 0, 0, 0, 9, 0]);
    assert_eq!(
        link.state()
            .client
            .client
            .state()
            .profile
            .unwrap()
            .slot_time,
        9
    );
    link.transmit(&eth(0x800)).unwrap();
    assert_eq!(finish_tx_on(&model, 1).len(), 1);
    link.service().unwrap();
    // Its TX-free starts TID 0's block-ack session, as
    // mt7925_tx_check_aggr does, and the next frame waits for it.
    drain_notices(&mut link);
    assert!(link.state().client.client.state().transmit_ba[0].is_some());
    link.transmit(&eth(0x800)).unwrap();
    assert_eq!(link.state().held.len(), 1);
    receive(&data(false), Some(1));
    let frame = link.receive().unwrap().unwrap();
    assert_eq!(&frame[12..14], &0x800u16.to_be_bytes());
    assert_eq!(model.response.lock().unwrap().home_channel, Some(1));
    stop(&mut link, &model);
    drop(link);
    assert_eq!(drops.load(Ordering::Relaxed), 1);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
}
#[test]
fn native_rx_ring_failure_cannot_leave_cached_association_carrier_up() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    drop(drain_notices(&mut link));
    incoming_control(&model, Some(1), 0x33);
    link.service().unwrap();
    incoming(&model, &data(false), Some((1, 0, 2)), false);
    link.service().unwrap();
    assert!(link.carrier());
    assert_eq!(link.queues().data_frames, 1);
    assert_eq!(link.queues().notices, 1);
    model.queues[&0xd4520].lock().unwrap().fail_publish = true;
    incoming(&model, &data(false), Some((1, 0, 3)), false);
    assert!(link.service().is_err());
    assert_eq!(
        link.state().client.client.state().controller.phase(),
        ControllerPhase::Quarantined
    );
    // The historical Station association is still held for teardown;
    // controller failure independently closes carrier and held data.
    assert_eq!(link.station().phase(), station::Phase::Associated);
    assert!(!link.carrier());
    assert_eq!(link.queues().data_frames, 0);
    assert_eq!(link.queues().notices, 1);
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 0);
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
}
#[test]
fn malformed_rx_preserves_authentication_timeout() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    let _clock = crate::test_clock::virtualize();
    scan_join(&mut link, &model, join, bss);
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    let clock = link
        .state()
        .client
        .client
        .state()
        .controller
        .state()
        .control
        .clock;
    crate::test_clock::advance(clock, 550_000);
    incoming(&model, &response(0xb1, &[0, 0, 2, 0, 0, 0]), None, false);
    link.service().unwrap();
    assert_eq!(link.rejected_frames(), 1);
    assert_eq!(link.phase(), Phase::Joining);
    // The rejected frame still permits the timed retry and packet release.
    assert_eq!(finish_tx(&model)[0][..2], [0xb0, 0]);
    assert!(link.queues().notices > 0);
    link.service().unwrap();
    stop(&mut link, &model);
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
    assert_eq!(model.drops.load(Ordering::Relaxed), 1);
}
fn queued_data(link: &mut TestLink, model: &Model, pn: u64, marker: u8) {
    let mut bytes = data(false);
    *bytes.last_mut().unwrap() = marker;
    incoming(model, &bytes, Some((1, 0, pn)), false);
    link.service().unwrap();
}
fn rejected_rx(model: &Model, protocol_version: bool) {
    if protocol_version {
        incoming(model, &response(0xb1, &[0, 0, 2, 0, 0, 0]), None, false);
    } else {
        let mut raw = incoming_bytes(&data(false), Some((1, 0, 100)), false);
        raw[6] |= 0x80; // source RXD1_NORMAL_CM, bit23; not ICV_ERR bit25
        let decoded = rx::decode(Chip::Mt7925, &raw).unwrap();
        assert!(decoded.security.cipher_mismatch);
        assert!(!decoded.security.decrypted);
        assert!(!decoded.security.icv_error);
        device_receive(&model.queues[&0xd4520], &raw, true, 0);
    }
}
#[test]
fn refused_frames_are_dropped_and_counted_while_valid_data_flows() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    drop(drain_notices(&mut link));
    for n in 1..=3 {
        queued_data(&mut link, &model, u64::from(n), n);
    }
    assert_eq!(link.queues().data_frames, 3);
    for n in 1..=3 {
        // ieee80211_rx_handlers drops a frame of another protocol version
        // or with a cipher mismatch and goes on.
        rejected_rx(&model, n % 2 == 0);
        let head = model.queues[&0xd4520].lock().unwrap().head;
        let received = link.receive().unwrap().unwrap();
        assert_eq!(received.last(), Some(&n));
        assert_eq!(link.queues().data_frames, usize::from(3 - n));
        assert_eq!(link.rejected_frames(), u64::from(n));
        // The refused frame was taken from the ring and its buffer reposted.
        assert_ne!(model.queues[&0xd4520].lock().unwrap().head, head);
        assert!(link.carrier());
    }
    assert!(link.receive().unwrap().is_none());
    // Valid frames around a refused one both arrive.
    incoming(&model, &data(false), Some((1, 0, 4)), false);
    rejected_rx(&model, true);
    let mut last = data(false);
    *last.last_mut().unwrap() = 0x44;
    incoming(&model, &last, Some((1, 0, 5)), false);
    let mut received = Vec::new();
    for _ in 0..6 {
        if let Some(frame) = link.receive().unwrap() {
            received.push(frame);
        }
    }
    assert_eq!(received.len(), 2);
    assert_eq!(received[1].last(), Some(&0x44));
    assert_eq!(link.rejected_frames(), 4);
    stop(&mut link, &model);
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
}
#[test]
fn a_refused_frame_never_holds_back_transmission() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    let ptk = secure(&mut link);
    drop(drain_notices(&mut link));
    queued_data(&mut link, &model, 1, 0x41);
    rejected_rx(&model, false);
    rejected_rx(&model, true);
    // Each pass takes one data frame, as one NAPI budget slot.
    link.service().unwrap();
    link.service().unwrap();
    assert_eq!(link.rejected_frames(), 2);
    rejected_rx(&model, true);
    link.transmit(&eth(0x800)).unwrap();
    let receipt = link.send_control(&control(), Some(ptk)).unwrap();
    assert_eq!(link.rejected_frames(), 3);
    let frames = finish_tx(&model);
    assert_eq!(frames.len(), 2);
    assert_ne!(frames[1][1] & 0x40, 0);
    link.service().unwrap();
    assert_eq!(link.receive().unwrap().unwrap().last(), Some(&0x41));
    let notices = drain_notices(&mut link);
    assert_eq!(
        notices
            .into_iter()
            .filter(|n| matches!(&n.event,LinkEvent::Released(r) if r.receipt==receipt))
            .count(),
        1
    );
    stop(&mut link, &model);
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
}
#[test]
fn data_and_eapol_alternate_single_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    for control_port in [false, true] {
        let (mut link, model, join, bss, provider) = setup_link();
        associate(&mut link, &model, join, bss);
        if !control_port {
            secure(&mut link);
        }
        drop(drain_notices(&mut link));
        // Each data frame takes a new TID: its TX-free starts that TID's
        // block-ack session, whose ADDBA is a separate publication.
        for (n, dscp) in [0u8, 8, 32].into_iter().enumerate() {
            assert_eq!(link.carrier(), !control_port);
            rejected_rx(&model, true);
            let receipt = if control_port {
                Some(link.send_control(&control(), None).unwrap())
            } else {
                let mut frame = eth(0x800);
                frame.resize(frame.len().max(34), 0);
                frame[14..16].copy_from_slice(&[0x45, dscp << 2]);
                link.transmit(&frame).unwrap();
                None
            };
            assert_eq!(link.rejected_frames(), n as u64 + 1);
            let frames = finish_tx(&model);
            assert_eq!(frames.len(), 1 + usize::from(!control_port && n > 0));
            assert_eq!(frames.last().unwrap()[1] & 0x40 == 0, control_port);
            link.service().unwrap();
            let notices = drain_notices(&mut link);
            let released: Vec<_> = notices
                .into_iter()
                .filter_map(|n| match n.event {
                    LinkEvent::Released(r) if r.context.tid().is_some() || control_port => Some(r),
                    _ => None,
                })
                .collect();
            assert_eq!(released.len(), 1);
            if let Some(receipt) = receipt {
                assert_eq!(released[0].receipt, receipt);
                assert!(released[0].context.key().is_none());
            }
        }
        stop(&mut link, &model);
        drop(link);
        assert_eq!(provider.load(Ordering::Relaxed), 1);
    }
}
#[test]
fn a_refused_frame_does_not_mask_a_native_submission_failure() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model, join, bss, provider) = setup_link();
    associate(&mut link, &model, join, bss);
    secure(&mut link);
    drop(drain_notices(&mut link));
    rejected_rx(&model, true);
    link.state().source.1.store(true, Ordering::Relaxed);
    let head = model.queues[&0xd4300].lock().unwrap().head;
    let submission = link.transmit(&eth(0x800)).unwrap_err();
    assert!(!format!("{submission:?}").contains("ProtocolVersion"));
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, head);
    assert_eq!(link.rejected_frames(), 1);
    link.state().source.1.store(false, Ordering::Relaxed);
    assert_eq!(link.receive(), Ok(None));
    assert_eq!(model.queues[&0xd4300].lock().unwrap().head, head);
    link.transmit(&eth(0x800)).unwrap();
    assert_eq!(finish_tx(&model).len(), 1);
    link.service().unwrap();
    stop(&mut link, &model);
    drop(link);
    assert_eq!(provider.load(Ordering::Relaxed), 1);
}
#[test]
fn a_native_hard_failure_after_a_refused_frame_fails_the_next_packet_io() {
    if crate::test_process::isolate_model() {
        return;
    }
    for operation in 0..3 {
        let (mut link, model, join, bss, provider) = setup_link();
        associate(&mut link, &model, join, bss);
        secure(&mut link);
        drop(drain_notices(&mut link));
        queued_data(&mut link, &model, 1, 0x51);
        queued_data(&mut link, &model, 2, 0x52);
        rejected_rx(&model, true);
        assert_eq!(link.receive().unwrap().unwrap().last(), Some(&0x51));
        assert_eq!(link.rejected_frames(), 1);
        model.queues[&0xd4520].lock().unwrap().fail_publish = true;
        incoming(&model, &data(false), Some((1, 0, 3)), false);
        let tx = model.queues[&0xd4300].lock().unwrap().head;
        // The failing RX DDONE refill closes carrier and discards held data
        // before any delivery or submission.
        let result = match operation {
            0 => link.receive().map(|_| ()),
            1 => link.transmit(&eth(0x800)),
            _ => link
                .send_control(&control(), None)
                .map(|_| ())
                .map_err(|e| LinkError::Adapter(format!("{e}"))),
        };
        assert!(result.is_err());
        assert_eq!(
            link.state().client.client.state().controller.phase(),
            ControllerPhase::Quarantined
        );
        assert!(!link.carrier());
        assert_eq!(link.queues().data_frames, 0);
        assert_eq!(model.queues[&0xd4300].lock().unwrap().head, tx);
        drop(link);
        assert_eq!(provider.load(Ordering::Relaxed), 0);
        assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    }
}
#[test]
fn revocation_and_new_association_never_deliver_held_old_epoch_data() {
    if crate::test_process::isolate_model() {
        return;
    }
    for revoke in [false, true] {
        let (mut link, model, join, bss, provider) = setup_link();
        associate(&mut link, &model, join, bss);
        secure(&mut link);
        drop(drain_notices(&mut link));
        let old_epoch = link.station().association_epoch().unwrap();
        queued_data(&mut link, &model, 1, 0x61);
        queued_data(&mut link, &model, 2, 0x62);
        rejected_rx(&model, true);
        assert_eq!(link.receive().unwrap().unwrap().last(), Some(&0x61));
        if revoke {
            link.revoke().unwrap();
        } else {
            depart(&mut link, &model);
        }
        assert!(!link.carrier());
        assert_eq!(link.queues().data_frames, 0);
        // SAFETY: model peer/key removal and all old users drained.
        unsafe { link.retire_peer_after_drain() }.unwrap();
        if revoke {
            assert!(link.send_control(&control(), None).is_err());
            assert!(link.transmit(&eth(0x800)).is_err());
            assert!(!matches!(link.receive(), Ok(Some(_))));
            model.response.lock().unwrap().clc_processing = false;
            model.bar.0.lock().unwrap().values.insert(0xd4208, 0);
            // SAFETY: independent whole-controller/radio/DMA/IRQ model fence.
            unsafe { link.stop_after_quiesce() }.unwrap();
        } else {
            let (join, bss) = modern_inputs(&link.state().client.client);
            associate(&mut link, &model, join, bss);
            secure(&mut link);
            assert_ne!(link.station().association_epoch(), Some(old_epoch));
            assert!(link.receive().unwrap().is_none());
            queued_data(&mut link, &model, 1, 0x63);
            assert_eq!(link.receive().unwrap().unwrap().last(), Some(&0x63));
            stop(&mut link, &model);
        }
        drop(link);
        assert_eq!(provider.load(Ordering::Relaxed), 1);
        assert_eq!(model.drops.load(Ordering::Relaxed), 1);
    }
}
#[test]
fn a_tx_retirement_poison_after_a_refused_frame_fails_at_once() {
    if crate::test_process::isolate_model() {
        return;
    }
    for operation in 0..3 {
        let (mut link, model, join, bss, provider) = setup_link();
        associate(&mut link, &model, join, bss);
        secure(&mut link);
        drop(drain_notices(&mut link));
        queued_data(&mut link, &model, 1, 0x71);
        assert!(link.carrier());
        rejected_rx(&model, true);
        model.queues[&0xd4300].lock().unwrap().invalid_index = true;
        let head = model.queues[&0xd4300].lock().unwrap().head;
        let result = match operation {
            0 => link.receive().map(|_| ()),
            1 => link.transmit(&eth(0x800)),
            _ => link
                .send_control(&control(), None)
                .map(|_| ())
                .map_err(|e| LinkError::Adapter(format!("{e}"))),
        };
        // The refused frame is only counted; the native retirement failure
        // is the error.
        assert!(matches!(result, Err(LinkError::Adapter(ref e)) if !e.contains("ProtocolVersion")));
        assert_eq!(link.rejected_frames(), 1);
        let c = link.state().client.client.state();
        assert!(c.controller.state().data_tx.as_ref().unwrap().is_poisoned());
        assert_eq!(c.phase, Phase::Associated);
        assert_eq!(c.controller.phase(), ControllerPhase::StartFacts);
        assert_eq!(link.station().phase(), station::Phase::Associated);
        assert!(!link.carrier());
        assert_eq!(link.queues().data_frames, 0);
        assert_eq!(model.queues[&0xd4300].lock().unwrap().head, head);
        assert!(link.transmit(&eth(0x800)).is_err());
        assert!(link.send_control(&control(), None).is_err());
        assert!(!matches!(link.receive(), Ok(Some(_))));
        assert_eq!(model.queues[&0xd4300].lock().unwrap().head, head);
        drop(link);
        assert_eq!(provider.load(Ordering::Relaxed), 0);
        assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    }
}

mod edca_controls;
mod management_controls;
mod pmf_controls;

mod discovery;
mod kept_connected;
