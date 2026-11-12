// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! Native custom probe metadata, real command ownership and receive/drain peers.
use super::*;
use crate::wifi::mediatek::bootstrap::client::local;

const EXTRA: &[u8] = &[
    10, 1, 0xaa, 59, 2, 81, 115, 127, 1, 0, 255, 2, 2, 0x55, 239, 1, 7, 221, 4, 0x12, 0x34, 0x56,
    0x99, 10, 1, 0xbb, 200, 0,
];
fn request(extra: Vec<u8>, bands: bool) -> scan::Request {
    let channels = if bands {
        vec![
            Channel {
                band: Band::Ghz2,
                number: 1,
                frequency_mhz: 2412,
            },
            CHANNEL,
            SIX,
        ]
    } else {
        vec![SIX]
    };
    let index = channels.len() - 1;
    scan::Request::with_six_ghz(
        channels,
        vec![vec![]],
        extra,
        vec![scan::SixGhz {
            channel_index: index,
            bssid: AP,
            short_ssid: Some(42),
            unsolicited_probe: true,
            psc_no_listen: false,
            psd_20: -70,
        }],
    )
    .unwrap()
}
fn parsed(mut bytes: &[u8]) -> Vec<(u8, Vec<u8>)> {
    let mut out = Vec::new();
    while !bytes.is_empty() {
        let length = usize::from(bytes[1]);
        out.push((bytes[0], bytes[2..2 + length].to_vec()));
        bytes = &bytes[2 + length..];
    }
    out
}

fn assert_generic_ack(link: &TestLink, kind: crate::wifi::mediatek::mcu::init::Kind) {
    use crate::wifi::mediatek::{mailbox::init::Fact, mcu};
    let Some(Fact::GenericResponse(reply)) = link.state().client.client.transport_facts().last()
    else {
        panic!("native command must consume the genuine generic response");
    };
    assert_eq!(reply.kind, kind);
    assert!(reply.dma_released);
    assert!(!reply.frame.drop);
    let event = mcu::Event::parse(Chip::Mt7925, &reply.frame.bytes).unwrap();
    assert_eq!(event.sequence, reply.sequence);
    assert!(!event.is_unsolicited());
}

#[test]
fn custom_metadata_reaches_all_three_native_band_commands_without_phy_substitution() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(true, true, 0);
    let originals: Vec<_> = [Band::Ghz2, Band::Ghz5, Band::Ghz6]
        .into_iter()
        .map(|band| {
            let actual = link.local_advertisement(band).unwrap();
            let state = link.state().client.client.state();
            let domain = state
                .controller
                .state()
                .policy
                .as_ref()
                .unwrap()
                .programmed()
                .unwrap()
                .0;
            local::scan::elements(
                &actual,
                local::scan::Selection::programmed(domain, band, 0x30b8),
            )
            .unwrap()
        })
        .collect();
    let before = model.response.lock().unwrap().client_commands.len();
    let ticket = link
        .begin_scan(request(EXTRA.to_vec(), true), SCAN_BUDGET_US)
        .unwrap();
    assert_eq!(link.phase(), Phase::Scanning);
    assert_generic_ack(&link, crate::wifi::mediatek::mcu::init::Kind::Scan);
    let commands = model.response.lock().unwrap().client_commands.clone();
    assert_eq!(commands.len(), before + 1);
    let body = &commands.last().unwrap()[48..];
    assert_eq!(&tlv(commands.last().unwrap(), 1)[4..7], &[1, 2, 40]);
    let mut rest = &body[4..];
    let mut probes = Vec::new();
    while !rest.is_empty() {
        let length = usize::from(u16::from_le_bytes(rest[2..4].try_into().unwrap()));
        if rest[..2] == 13u16.to_le_bytes() {
            probes.push(&rest[..length]);
        }
        rest = &rest[length..];
    }
    assert_eq!(probes.len(), 3);
    for (index, probe) in probes.into_iter().enumerate() {
        assert_eq!(probe[6], index as u8 + 1);
        assert_eq!(
            usize::from(u16::from_le_bytes(probe[4..6].try_into().unwrap())),
            probe.len() - 8
        );
        let ies = parsed(&probe[8..]);
        let metadata = parsed(EXTRA);
        let without_metadata: Vec<_> = ies
            .iter()
            .filter(|ie| !metadata.contains(ie))
            .cloned()
            .collect();
        // The metadata's Extended Capabilities replace the device's.
        let generated: Vec<_> = parsed(&originals[index])
            .into_iter()
            .filter(|ie| ie.0 != 127)
            .collect();
        assert_eq!(without_metadata, generated);
        // Explicit source prefix placement and intentional common-tail retention.
        let position = |id, payload: &[u8]| {
            ies.iter()
                .position(|ie| ie.0 == id && ie.1 == payload)
                .unwrap()
        };
        assert_eq!(position(10, &[0xaa]), 1);
        assert!(position(59, &[81, 115]) < position(127, &[0]));
        assert!(position(127, &[0]) < position(255, &[2, 0x55]));
        assert!(position(255, &[2, 0x55]) < position(239, &[7]));
        assert!(position(239, &[7]) < position(255, &[59, 0xb8, 0x36]));
        let phy_position = |id, extension| {
            ies.iter()
                .position(|ie| ie.0 == id && (id != 255 || ie.1.first() == Some(&extension)))
                .unwrap()
        };
        if index < 2 {
            assert!(position(59, &[81, 115]) < phy_position(45, 0));
            assert!(phy_position(45, 0) < position(127, &[0]));
        }
        if index == 1 {
            assert!(position(127, &[0]) < phy_position(191, 0));
            assert!(phy_position(191, 0) < position(255, &[2, 0x55]));
        }
        assert!(position(239, &[7]) < phy_position(255, 35));
        assert_eq!(&ies[ies.len() - 3..], &metadata[metadata.len() - 3..]);
        assert!(position(10, &[0xbb]) > position(221, &[0x12, 0x34, 0x56, 0x99]));
    }
    // Receive ownership and BSS routing still use the scan generation.
    beacon(&model, 73);
    link.service().unwrap();
    assert!(
        matches!(link.take_notice().unwrap().event, LinkEvent::Controller(Event::Bss(b))
        if b.ticket == ticket && b.channel == SIX && b.timestamp == 73)
    );
    link.cancel_scan().unwrap();
    assert_generic_ack(&link, crate::wifi::mediatek::mcu::init::Kind::CancelScan);
    assert_eq!(link.phase(), Phase::Scanning);
    assert_eq!(
        model.response.lock().unwrap().client_commands.len(),
        before + 2
    );
    assert!(
        link.begin_scan(request(EXTRA.to_vec(), false), SCAN_BUDGET_US)
            .is_err()
    );
    assert_eq!(model.drops.load(Ordering::Relaxed), 0);
    assert!(!model.memory.lock().unwrap().mappings.is_empty());
    terminal(&mut link, &model, scan::Outcome::Aborted);
    link.begin_scan(request(EXTRA.to_vec(), false), SCAN_BUDGET_US)
        .unwrap();
    terminal(&mut link, &model, scan::Outcome::Completed);
    stop(link, &model);
}

#[test]
fn custom_probe_refusals_precede_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(true, true, 0);
    let before = model.response.lock().unwrap().client_commands.len();
    let heads = queue_heads(&model);
    let syncs = model.memory.lock().unwrap().syncs.len();
    for extra in [
        vec![1, 1, 0xff],
        vec![3, 1, 99],
        vec![50, 1, 0xff],
        vec![255, 1, 152],
        vec![255, 1, 60],
        vec![255, 1, 109],
        vec![255, 1, 136],
        vec![255, 1, 137],
        vec![217, 0],
        vec![242, 0],
        vec![221, 4, 0, 0x50, 0xf2, 2],
        vec![221, 4, 0, 0x90, 0x4c, 4],
    ] {
        assert!(matches!(
            link.begin_scan(request(extra, true), SCAN_BUDGET_US),
            Err(LinkControlError::Policy(PolicyError::Client(Error::Local(
                LocalError::UnsupportedProbeElements
            ))))
        ));
        assert_eq!(link.phase(), Phase::Idle);
    }
    let mut huge = vec![221, 255];
    huge.extend_from_slice(&[0x42; 255]);
    huge.extend_from_slice(&[200, 125]);
    huge.extend_from_slice(&[0x24; 125]);
    assert_eq!(huge.len(), 384);
    assert!(matches!(
        link.begin_scan(request(huge, true), SCAN_BUDGET_US),
        Err(LinkControlError::Policy(PolicyError::Client(Error::Codec(
            wire::Error::InvalidScanElements
        ))))
    ));
    for extra in [vec![221], vec![221, 2, 1], vec![255, 0]] {
        // Generic request rejects malformed lengths; native rejects empty extension selectors.
        if let Ok(request) = scan::Request::new(vec![CHANNEL], vec![vec![]], extra) {
            assert!(matches!(
                link.begin_scan(request, SCAN_BUDGET_US),
                Err(LinkControlError::Policy(PolicyError::Client(Error::Local(
                    LocalError::WrongAdvertisement
                ))))
            ));
        }
    }
    assert_eq!(link.phase(), Phase::Idle);
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    assert_eq!(queue_heads(&model), heads);
    assert_eq!(model.memory.lock().unwrap().syncs.len(), syncs);
    link.begin_scan(request(EXTRA.to_vec(), false), SCAN_BUDGET_US)
        .unwrap();
    terminal(&mut link, &model, scan::Outcome::Completed);
    stop(link, &model);
}

#[test]
fn custom_probe_metadata_cannot_override_programmed_no_ir_or_dfs_policy() {
    if crate::test_process::isolate_model() {
        return;
    }
    for flags in [2, 8] {
        let (mut link, model) = setup(true, true, flags);
        let before = model.response.lock().unwrap().client_commands.len();
        let heads = queue_heads(&model);
        let syncs = model.memory.lock().unwrap().syncs.len();
        assert!(matches!(
            link.begin_scan(request(EXTRA.to_vec(), false), SCAN_BUDGET_US),
            Err(LinkControlError::Policy(PolicyError::Client(Error::Scan(
                scan::Error::Regulatory
            ))))
        ));
        assert_eq!(link.phase(), Phase::Idle);
        assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
        assert_eq!(queue_heads(&model), heads);
        assert_eq!(model.memory.lock().unwrap().syncs.len(), syncs);
        // Passive firmware work with the same bytes is still receive-only.
        link.begin_scan(
            scan::Request::new(vec![SIX], vec![], EXTRA.to_vec()).unwrap(),
            SCAN_BUDGET_US,
        )
        .unwrap();
        let command = model
            .response
            .lock()
            .unwrap()
            .client_commands
            .last()
            .unwrap()
            .clone();
        assert_eq!(&tlv(&command, 1)[4..7], &[0, 0, 32]);
        terminal(&mut link, &model, scan::Outcome::Completed);
        stop(link, &model);
    }
}

#[test]
fn two_and_five_ghz_custom_probe_metadata_needs_no_six_ghz_capability() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(false, false, 0);
    let request = scan::Request::new(
        vec![
            Channel {
                band: Band::Ghz2,
                number: 1,
                frequency_mhz: 2412,
            },
            CHANNEL,
        ],
        vec![vec![]],
        EXTRA.to_vec(),
    )
    .unwrap();
    link.begin_scan(request, SCAN_BUDGET_US).unwrap();
    let command = model
        .response
        .lock()
        .unwrap()
        .client_commands
        .last()
        .unwrap()
        .clone();
    assert_eq!(&tlv(&command, 1)[4..7], &[1, 2, 32]);
    let mut rest = &command[52..];
    let mut bands = Vec::new();
    while !rest.is_empty() {
        let length = usize::from(u16::from_le_bytes(rest[2..4].try_into().unwrap()));
        if rest[..2] == 13u16.to_le_bytes() {
            bands.push(rest[6]);
            let ies = parsed(&rest[8..length]);
            assert!(
                !ies.iter()
                    .any(|ie| ie.0 == 255 && ie.1.first() == Some(&59))
            );
            assert_eq!(
                &rest[length - 11..length],
                &[221, 4, 0x12, 0x34, 0x56, 0x99, 10, 1, 0xbb, 200, 0]
            );
        }
        rest = &rest[length..];
    }
    assert_eq!(bands, [1, 2]);
    terminal(&mut link, &model, scan::Outcome::Completed);
    stop(link, &model);
}

#[test]
fn fragment_after_short_final_fragment_is_refused_before_native_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    let (mut link, model) = setup(true, true, 0);
    let before = model.response.lock().unwrap().client_commands.len();
    let heads = queue_heads(&model);
    let syncs = model.memory.lock().unwrap().syncs.len();
    let mut extra = vec![221, 255];
    extra.extend_from_slice(&[0x42; 255]);
    extra.extend_from_slice(&[242, 1, 0x24, 242, 1, 0x18]);
    assert_eq!(extra.len(), 263);
    assert!(matches!(
        link.begin_scan(request(extra, false), SCAN_BUDGET_US),
        Err(LinkControlError::Policy(PolicyError::Client(Error::Local(
            LocalError::UnsupportedProbeElements
        ))))
    ));
    assert_eq!(link.phase(), Phase::Idle);
    assert_eq!(model.response.lock().unwrap().client_commands.len(), before);
    assert_eq!(queue_heads(&model), heads);
    assert_eq!(model.memory.lock().unwrap().syncs.len(), syncs);
    assert_eq!(
        link.state().client.client.state().scanner.phase(),
        scan::Phase::Idle
    );
    assert!(link.state().client.client.state().scan_ticket.is_none());
    link.begin_scan(request(EXTRA.to_vec(), false), SCAN_BUDGET_US)
        .unwrap();
    assert_generic_ack(&link, crate::wifi::mediatek::mcu::init::Kind::Scan);
    terminal(&mut link, &model, scan::Outcome::Completed);
    stop(link, &model);
}
