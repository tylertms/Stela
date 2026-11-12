extern crate std;
use crate::wifi::mediatek::mailbox::clc::tests::*;
use crate::wifi::mediatek::mailbox::clc::{Execution, Fact};
use crate::wifi::mediatek::{
    download::{PortState, TransportFailure},
    identity::Chip,
    mcu::Event,
};
use crate::wifi::stepped::block_on;
use crate::wifi::{
    Band,
    mediatek::{
        mailbox::clc::remaining::{
            Error as RemainingError, Inputs as RemainingInputs, Observation, Step,
        },
        mcu::{domain, power},
    },
};
use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};

pub(crate) fn caps(chip: Chip, bands: [bool; 3]) -> Vec<u8> {
    let mut bytes = vec![3, 0, 0xa5, 0x5a];
    let mut phy = [1, 1, 1, 0, 2, 0, 1, 1, 0, 0, 0, 1, 1];
    phy[10] = u8::from(bands[0]) | (u8::from(bands[1]) << 1);
    for (tag, body) in [
        (8u32, &phy[..if chip == Chip::Mt7925 { 13 } else { 12 }]),
        (0x18, &[u8::from(bands[2])][..]),
        (0x20, &1u64.to_le_bytes()[..]),
    ] {
        if chip == Chip::Mt7925 {
            bytes.extend_from_slice(&(tag as u16).to_le_bytes());
            bytes.extend_from_slice(&((body.len() + 4) as u16).to_le_bytes());
        } else {
            bytes.extend_from_slice(&tag.to_le_bytes());
            bytes.extend_from_slice(&(body.len() as u32).to_le_bytes());
        }
        bytes.extend_from_slice(body);
    }
    bytes
}
pub(crate) fn complete_caps(fixture: &mut Fixture, bands: [bool; 3]) {
    fixture.model.lock().unwrap().capability_payload =
        Some(caps(fixture.mailbox.identity.chip, bands));
    fixture.capabilities = Some(fixture.mailbox.query_capabilities().unwrap());
}
pub(crate) fn rates(chip: Chip, band: Band) -> Vec<power::Channel> {
    power::channels(chip, band)
        .iter()
        .map(|&hardware_value| power::Channel {
            hardware_value,
            limits: power::Limits::uniform(-3),
        })
        .collect()
}
pub(crate) fn domain_channels(band: Band) -> Vec<domain::Channel> {
    vec![domain::Channel {
        hardware_value: match band {
            Band::Ghz2 => 1,
            Band::Ghz5 => 36,
            Band::Ghz6 => 1,
        },
        flags: 2 | 16 | 32 | 128 | 256 | 8192 | (1 << 19) | (1 << 20),
    }]
}
pub(crate) fn prepare(
    owner: &mut Execution<Guard>,
    fixture: &Fixture,
    alpha2: [u8; 2],
    bands: &[Vec<power::Channel>; 3],
) -> Result<(), RemainingError> {
    let chan: [Vec<domain::Channel>; 3] = core::array::from_fn(|i| {
        if bands[i].is_empty() {
            Vec::new()
        } else {
            domain_channels([Band::Ghz2, Band::Ghz5, Band::Ghz6][i])
        }
    });
    prepare_with_channels(owner, fixture, alpha2, bands, &chan)
}
pub(crate) fn prepare_with_channels(
    owner: &mut Execution<Guard>,
    fixture: &Fixture,
    alpha2: [u8; 2],
    bands: &[Vec<power::Channel>; 3],
    chan: &[Vec<domain::Channel>; 3],
) -> Result<(), RemainingError> {
    // Install the source-policy fixture behind the same Guard,
    // not an unrelated drop counter. Every device DMA consumer checks these
    // exact restrictions/rate inputs before mutating modeled hardware.
    {
        let mut model = fixture.model.lock().unwrap();
        if model.regulatory_expected_domain.is_none() {
            model.regulatory_expected_domain = domain::Request::build(
                fixture.mailbox.identity.chip,
                domain::Inputs {
                    alpha2,
                    channels_2ghz: &chan[0],
                    channels_5ghz: &chan[1],
                    channels_6ghz: &chan[2],
                },
            )
            .ok()
            .map(|r| r.body().to_vec());
            for (band, entries) in bands.iter().enumerate() {
                for entry in entries {
                    model.regulatory_expected_power.insert(
                        (band as u8 + 1, entry.hardware_value),
                        entry.limits.sku(
                            fixture.mailbox.identity.chip,
                            [Band::Ghz2, Band::Ghz5, Band::Ghz6][band],
                        ),
                    );
                }
            }
        }
    }
    // SAFETY: the model guard owns stopped radio and the
    // fixture source-policy state for these explicit restricted channels and
    // resolved rate arrays. The same guard is checked by each real DMA read;
    // no native HardwarePolicy/OS permission is manufactured by this test.
    unsafe {
        owner.prepare_remaining(
            &fixture.mailbox,
            RemainingInputs {
                channel_configuration: owner.channel_configuration(),
                domain: domain::Inputs {
                    alpha2,
                    channels_2ghz: &chan[0],
                    channels_5ghz: &chan[1],
                    channels_6ghz: &chan[2],
                },
                power: power::Inputs {
                    alpha2,
                    ghz2: (!bands[0].is_empty()).then_some(bands[0].as_slice()),
                    ghz5: (!bands[1].is_empty()).then_some(bands[1].as_slice()),
                    ghz6: (!bands[2].is_empty()).then_some(bands[2].as_slice()),
                },
            },
        )
    }
}
#[test]
fn coherent_clc_domain_power_dma_response_and_pse_facts_keep_owner() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        for retain in [false, true] {
            let mut fixture = Fixture::new(chip, Some(1));
            complete_caps(&mut fixture, [true, true, false]);
            let drops = Arc::new(AtomicUsize::new(0));
            let mut owner = fixture.claim(inputs(Some(1)), &drops).unwrap();
            assert_eq!(
                block_on(owner.execute_all(&mut fixture.mailbox)),
                Err(RemainingError::NotPrepared)
            );
            let bands = [rates(chip, Band::Ghz2), rates(chip, Band::Ghz5), Vec::new()];
            block_on(owner.execute(&mut fixture.mailbox)).unwrap();
            prepare(&mut owner, &fixture, *b"US", &bands).unwrap();
            assert_eq!(
                prepare(&mut owner, &fixture, *b"US", &bands),
                Err(RemainingError::AlreadyPrepared)
            );
            block_on(owner.execute_all(&mut fixture.mailbox)).unwrap();
            assert!(owner.remaining_completed() && !owner.is_quarantined());
            let model = fixture.model.lock().unwrap();
            assert!(model.regulatory_domain && model.clc_guard && model.clc_processing);
            assert_eq!(
                model.regulatory_power.len(),
                if chip == Chip::Mt7925 { 60 } else { 62 }
            );
            assert_eq!(
                owner.remaining_facts().len(),
                model.regulatory_commands.len()
            );
            for (fact, packet) in owner
                .remaining_facts()
                .iter()
                .zip(&model.regulatory_commands)
            {
                assert_eq!(fact.sequence, packet[39]);
                match &fact.observed {
                    Observation::OneWayDmaCompleted => assert_ne!(chip, Chip::Mt7925),
                    Observation::GenericResponse(reply) => {
                        assert_eq!(chip, Chip::Mt7925);
                        assert!(reply.dma_released);
                        assert!(
                            Event::parse(chip, &reply.frame.bytes)
                                .unwrap()
                                .payload
                                .is_empty()
                        );
                    }
                    Observation::PseRead {
                        reply,
                        register,
                        value,
                    } => {
                        assert_ne!(chip, Chip::Mt7925);
                        assert_eq!(*register, 0x820c8000);
                        assert_eq!(*value, 0xefbeadde);
                        assert!(reply.dma_released);
                        assert!(matches!(
                            fact.step,
                            Step::Power {
                                kind: power::Kind::PseFlush,
                                ..
                            }
                        ));
                    }
                }
            }
            drop(model);
            assert_eq!(
                block_on(owner.execute_all(&mut fixture.mailbox)),
                Err(RemainingError::WrongPhase)
            );
            if retain {
                drop(owner);
                assert_eq!(drops.load(Ordering::Relaxed), 0);
            } else {
                fixture.drain();
                // SAFETY: the model device is stopped and drained, so nothing still uses what
                // this releases.
                unsafe { owner.retire_after_quiesce() };
                drop(owner);
                assert_eq!(drops.load(Ordering::Relaxed), 1);
            }
            fixture.release();
            assert_eq!(drops.load(Ordering::Relaxed), usize::from(!retain));
        }
    }
}
#[test]
fn remaining_refusal_keeps_completed_clc_before_any_domain_or_power_publication() {
    if crate::test_process::isolate_model() {
        return;
    }
    for mode in 0..5 {
        let chip = Chip::Mt7925;
        let mut fixture = Fixture::new(chip, Some(1));
        if mode != 0 {
            complete_caps(&mut fixture, [true, true, false]);
        }
        let drops = Arc::new(AtomicUsize::new(0));
        let mut owner = fixture.claim(inputs(Some(1)), &drops).unwrap();
        block_on(owner.execute(&mut fixture.mailbox)).unwrap();
        let mut bands = [rates(chip, Band::Ghz2), rates(chip, Band::Ghz5), Vec::new()];
        let expected = match mode {
            0 => RemainingError::CapabilitiesIncomplete,
            1 => RemainingError::WrongCountry,
            2 => {
                bands[1].clear();
                RemainingError::BandSelection
            }
            3 => {
                bands[2] = rates(chip, Band::Ghz6);
                RemainingError::BandSelection
            }
            4 => {
                bands[0].pop();
                RemainingError::Power(power::Error::Channels {
                    band: Band::Ghz2,
                    expected: 14,
                    actual: 13,
                })
            }
            _ => unreachable!(),
        };
        assert_eq!(
            prepare(
                &mut owner,
                &fixture,
                if mode == 1 { *b"CN" } else { *b"US" },
                &bands
            ),
            Err(expected)
        );
        assert_eq!(fixture.model.lock().unwrap().clc_commands.len(), 2);
        assert!(owner.remaining_facts().is_empty());
        drop(owner);
        assert_eq!(drops.load(Ordering::Relaxed), 0);
        fixture.release();
    }
}
#[test]
fn partial_domain_power_failure_early_reply_and_bad_pse_never_release_policy_guard() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        for mode in 0..5 {
            if mode == 4 && chip == Chip::Mt7925 {
                continue;
            }
            let mut fixture = Fixture::new(chip, Some(1));
            complete_caps(&mut fixture, [true, true, false]);
            let drops = Arc::new(AtomicUsize::new(0));
            let mut owner = fixture.claim(inputs(Some(1)), &drops).unwrap();
            let bands = [rates(chip, Band::Ghz2), rates(chip, Band::Ghz5), Vec::new()];
            block_on(owner.execute(&mut fixture.mailbox)).unwrap();
            prepare(&mut owner, &fixture, *b"US", &bands).unwrap();
            {
                let mut model = fixture.model.lock().unwrap();
                match mode {
                    0 => model.regulatory_fail_publication_at = Some(1),
                    1 => model.regulatory_fail_publication_at = Some(2),
                    2 => {
                        model.regulatory_no_reply_at =
                            Some(if chip == Chip::Mt7925 { 2 } else { 3 })
                    }
                    3 => {
                        model.regulatory_hide_tx_index_at =
                            Some(if chip == Chip::Mt7925 { 2 } else { 3 })
                    }
                    4 => model.regulatory_malformed_pse = true,
                    _ => unreachable!(),
                }
            }
            let failure =
                block_on(owner.execute_all_timed(&mut fixture.mailbox, 1000)).unwrap_err();
            assert!(owner.is_quarantined() && !owner.remaining_completed());
            assert!(fixture.model.lock().unwrap().regulatory_domain);
            assert_eq!(drops.load(Ordering::Relaxed), 0);
            if mode == 4 {
                assert!(matches!(
                    failure,
                    RemainingError::Malformed {
                        error: power::Error::TruncatedFlush,
                        ..
                    }
                ));
                assert!(!fixture.mailbox.is_poisoned());
                assert!(owner.interrupted_remaining_reply().unwrap().dma_released);
            } else {
                assert!(matches!(
                    failure,
                    RemainingError::Transport {
                        failure: TransportFailure {
                            port: PortState::Uncertain,
                            ..
                        },
                        ..
                    }
                ));
                assert!(fixture.mailbox.is_poisoned());
                if mode == 3 {
                    assert!(!owner.interrupted_remaining_reply().unwrap().dma_released);
                }
            }
            let count = fixture.model.lock().unwrap().regulatory_commands.len();
            assert_eq!(
                block_on(owner.execute_all(&mut fixture.mailbox)),
                Err(RemainingError::WrongPhase)
            );
            assert_eq!(
                fixture.model.lock().unwrap().regulatory_commands.len(),
                count
            );
            drop(owner);
            assert_eq!(drops.load(Ordering::Relaxed), 0);
            fixture.release();
            assert_eq!(drops.load(Ordering::Relaxed), 0);
        }
    }
}
#[test]
#[ignore = "oracle: STELA_MT792X_POWER_ORACLE"]
fn all_source_rate_plans_cross_coherent_native_clc_domain_power_and_pse_dma() {
    if crate::test_process::isolate_model() {
        return;
    }
    let root = std::path::PathBuf::from(std::env::var("STELA_MT792X_POWER_ORACLE").unwrap());
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        for source in power::tests::sources(&root, chip) {
            let mut bytes = Vec::new();
            for slot in [0, 1] {
                let mut rec = record(chip, slot);
                let at = if chip == Chip::Mt7925 { 32 } else { 16 };
                rec[at..at + 2].copy_from_slice(&source.alpha2);
                if chip == Chip::Mt7925 && !source.clc_nonzero {
                    rec[37] = 0;
                }
                bytes.extend_from_slice(&rec);
            }
            let mut fixture = Fixture::with_clc(chip, Some(1), &bytes, 0);
            complete_caps(&mut fixture, source.hardware_bands);
            let drops = Arc::new(AtomicUsize::new(0));
            let mut input = inputs(Some(1));
            input.alpha2 = source.alpha2;
            let mut owner = fixture.claim(input, &drops).unwrap();
            block_on(owner.execute(&mut fixture.mailbox)).unwrap();
            prepare(&mut owner, &fixture, source.alpha2, &source.bands).unwrap();
            block_on(owner.execute_all(&mut fixture.mailbox)).unwrap();
            assert!(owner.remaining_completed());
            let model = fixture.model.lock().unwrap();
            assert_eq!(model.regulatory_commands.len(), source.wire.len() + 1);
            for (actual, expected) in model.regulatory_commands[1..].iter().zip(&source.wire) {
                let mut packet = expected.packet.clone();
                packet[39] = actual[39];
                assert_eq!(*actual, packet);
            }
            let expected: usize = source.bands.iter().map(Vec::len).sum();
            assert_eq!(model.regulatory_power.len(), expected);
            drop(model);
            fixture.drain();
            // SAFETY: the model device is stopped and drained, so nothing still uses what this
            // releases.
            unsafe { owner.retire_after_quiesce() };
            drop(owner);
            assert_eq!(drops.load(Ordering::Relaxed), 1);
            fixture.release();
        }
    }
}

#[test]
#[ignore = "oracle: STELA_MT792X_FIRMWARE"]
fn images_execute_kept_clc_domain_power_chain_past_provider_lifetime() {
    if crate::test_process::isolate_model() {
        return;
    }
    let images = std::path::PathBuf::from(std::env::var("STELA_MT792X_FIRMWARE").unwrap());
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let mut fixture = Fixture::with_ram(
            chip,
            Some(1),
            std::fs::read(images.join(chip.ram_name())).unwrap(),
            std::fs::read(images.join(chip.patch_name())).unwrap(),
            0,
        );
        complete_caps(&mut fixture, [true, true, false]);
        let drops = Arc::new(AtomicUsize::new(0));
        let mut owner = fixture.claim(inputs(Some(1)), &drops).unwrap();
        let bands = [rates(chip, Band::Ghz2), rates(chip, Band::Ghz5), Vec::new()];
        block_on(owner.execute(&mut fixture.mailbox)).unwrap();
        prepare(&mut owner, &fixture, *b"US", &bands).unwrap();
        block_on(owner.execute_all(&mut fixture.mailbox)).unwrap();
        assert!(owner.remaining_completed());
        assert!(!owner.auxiliary().release_information().is_empty());
        fixture.drain();
        // SAFETY: the model device is stopped and drained, so nothing still uses what this
        // releases.
        unsafe { owner.retire_after_quiesce() };
        drop(owner);
        assert_eq!(drops.load(Ordering::Relaxed), 1);
        fixture.release();
    }
}

#[test]
fn clc_reply_resolves_domain_and_power_once() {
    if crate::test_process::isolate_model() {
        return;
    }
    for chip in [Chip::Mt7921, Chip::Mt7922] {
        for returned_flags in [0u8, 1] {
            for fail_plan in [false, true] {
                let mut fixture = Fixture::new(chip, Some(1));
                complete_caps(&mut fixture, [true, true, false]);
                fixture.model.lock().unwrap().clc_payload.as_mut().unwrap()[8] = returned_flags;
                let drops = Arc::new(AtomicUsize::new(0));
                let mut owner = fixture.claim(inputs(Some(1)), &drops).unwrap();
                assert_eq!(owner.facts().len(), 0);
                block_on(owner.execute(&mut fixture.mailbox)).unwrap();
                let flags = owner
                    .facts()
                    .iter()
                    .filter_map(|fact| match fact {
                        Fact::Response { observed, .. } => {
                            observed.connac2().map(|info| info.channel_flags)
                        }
                        _ => None,
                    })
                    .next_back()
                    .unwrap();
                assert_eq!(flags, returned_flags);
                assert_eq!(owner.facts().len(), 2);
                // The same source-policy guard now resolves source UNII4
                // restrictions from the completed reply. Source get_ch_power applies
                // the channel regulatory cap only while enabled; the disabled channel
                // remains excluded from the domain even if its raw rate limit differs.
                let enabled = flags & 1 != 0;
                let mut bands = [rates(chip, Band::Ghz2), rates(chip, Band::Ghz5), Vec::new()];
                bands[1]
                    .iter_mut()
                    .find(|c| c.hardware_value == 169)
                    .unwrap()
                    .limits = power::Limits::uniform(if enabled { -10 } else { -3 });
                let chans = [
                    domain_channels(Band::Ghz2),
                    vec![
                        domain::Channel {
                            hardware_value: 36,
                            flags: 2,
                        },
                        domain::Channel {
                            hardware_value: 169,
                            flags: if enabled { 2 } else { 3 },
                        },
                    ],
                    Vec::new(),
                ];
                if fail_plan {
                    bands[0].pop();
                }
                let result = prepare_with_channels(&mut owner, &fixture, *b"US", &bands, &chans);
                if fail_plan {
                    assert!(matches!(
                        result,
                        Err(RemainingError::Power(power::Error::Channels {
                            expected: 14,
                            actual: 13,
                            ..
                        }))
                    ));
                    assert_eq!(owner.facts().len(), 2);
                    assert!(owner.completed_facts());
                    assert!(owner.remaining_facts().is_empty());
                    assert!(fixture.model.lock().unwrap().regulatory_commands.is_empty());
                    assert!(
                        fixture
                            .mailbox
                            .download_binding()
                            .unwrap()
                            .matches_auxiliary(owner.auxiliary())
                    );
                    drop(owner);
                    assert_eq!(drops.load(Ordering::Relaxed), 0);
                    fixture.release();
                    assert_eq!(drops.load(Ordering::Relaxed), 0);
                    continue;
                }
                result.unwrap();
                block_on(owner.execute_all(&mut fixture.mailbox)).unwrap();
                let model = fixture.model.lock().unwrap();
                assert_eq!(model.clc_commands.len(), 2, "never replay completed CLC");
                assert_eq!(
                    model.regulatory_commands[0][64 + 9],
                    if enabled { 2 } else { 1 }
                );
                assert_eq!(
                    model.regulatory_power[&(2, 169)],
                    power::Limits::uniform(if enabled { -10 } else { -3 }).sku(chip, Band::Ghz5)
                );
                drop(model);
                fixture.drain();
                // SAFETY: the model device is stopped and drained, so nothing still uses what
                // this releases.
                unsafe { owner.retire_after_quiesce() };
                drop(owner);
                assert_eq!(drops.load(Ordering::Relaxed), 1);
                fixture.release();
            }
        }
    }
}
