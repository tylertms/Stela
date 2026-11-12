use super::*;
fn base(chip: Chip) -> Config {
    Config {
        chip,
        wcid: 7,
        vif: Some(Vif {
            omac: 2,
            wmm: 1,
            band: 0,
            mld: false,
        }),
        phy_band_one: false,
        queue: Queue::Data { access_category: 0 },
        pid: 3,
        frame: Frame {
            bytes: 128,
            header_bytes: 26,
            kind: Kind::Data,
            subtype: 8,
            tid: 5,
            multicast: false,
            fragment: Fragment::None,
            sequence: Sequence::Hardware,
            minimum_rate: false,
        },
        protect: true,
        no_ack: false,
        hardware_amsdu: true,
        fixed_rate: None,
    }
}
fn words(bytes: [u8; 32]) -> [u32; 8] {
    core::array::from_fn(|i| u32::from_le_bytes(bytes[4 * i..4 * i + 4].try_into().unwrap()))
}
#[test]
fn distinct_raw_data_layouts_keep_firmware_sequence_and_pn_ownership_explicit() {
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let mut c = base(chip);
        let w = words(encode(c).unwrap());
        assert_eq!(w[0], 160 | (7 << 25));
        assert_eq!(w[3] & ((1 << 31) | (1 << 30) | (0xfff << 16)), 0);
        assert_eq!(w[4], 0); // no invented host PN
        assert_eq!(w[5], 3 | (1 << 10));
        assert_eq!(w[3] & 2, 2);
        if chip == Chip::Mt7925 {
            assert_eq!(w[1], 7 | (2 << 25) | (5 << 21) | (13 << 16) | (2 << 14));
            assert_eq!(w[3], (15 << 11) | (1 << 28) | 2);
            assert_eq!(w[6], 0x1c);
            assert_eq!(w[7], 0);
        } else {
            assert_eq!(
                w[1],
                7 | (1 << 31) | (2 << 24) | (5 << 20) | (2 << 16) | (13 << 11)
            );
            assert_eq!(w[2], 0x28);
            assert_eq!(w[3], (15 << 11) | 2);
            assert_eq!(w[6], 0);
            assert_eq!(w[7], (2 << 20) | (8 << 16));
        }
        c.pid = 0;
        c.frame.sequence = Sequence::Explicit(0xabc);
        let w = words(encode(c).unwrap());
        assert_eq!(
            w[3] & ((1 << 31) | (0xfff << 16)),
            (1 << 31) | (0xabc << 16)
        );
        assert_eq!(w[3] & (1 << 30), 0);
        assert_eq!(
            if chip == Chip::Mt7925 {
                w[3] & (1 << 5)
            } else {
                w[7] & (1 << 10)
            },
            0
        );
        c.frame.sequence = Sequence::Hardware;
        let w = words(encode(c).unwrap());
        assert_ne!(
            if chip == Chip::Mt7925 {
                w[3] & (1 << 5)
            } else {
                w[7] & (1 << 10)
            },
            0
        );
    }
}
#[test]
fn fixed_management_multicast_paths_require_chip_rate_context() {
    for chip in [Chip::Mt7921, Chip::Mt7925] {
        let mut c = base(chip);
        c.frame.kind = Kind::Management;
        c.frame.subtype = 0xb;
        c.frame.header_bytes = 24;
        c.queue = Queue::Management;
        c.pid = 0;
        assert_eq!(encode(c), Err(Error::MissingRate));
        c.fixed_rate = Some(if chip == Chip::Mt7925 {
            FixedRate::Connac3(31)
        } else {
            FixedRate::Connac2(0x4b)
        });
        let w = words(encode(c).unwrap());
        assert_eq!(w[0] >> 25, 16);
        assert_ne!(w[3] & (1 << 28), 0);
        if chip == Chip::Mt7925 {
            assert_eq!((w[1] >> 21) & 15, 0);
            assert_ne!(w[1] & (1 << 31), 0);
            assert_eq!((w[6] >> 16) & 63, 31);
            assert_ne!(w[3] & (1 << 5), 0);
        } else {
            assert_ne!(w[2] & (1 << 31), 0);
            assert_ne!(w[2] & (1 << 13), 0);
            assert_eq!(w[6], (0x4b << 16) | 4);
            assert_eq!(w[7] & (1 << 10), 0);
        }
        c.frame.kind = Kind::Data;
        c.frame.multicast = true;
        let w = words(encode(c).unwrap());
        assert_ne!(
            if chip == Chip::Mt7925 {
                w[3] & (1 << 4)
            } else {
                w[2] & (1 << 10)
            },
            0
        );
        c.fixed_rate = Some(if chip == Chip::Mt7925 {
            FixedRate::Connac2(0)
        } else {
            FixedRate::Connac3(0)
        });
        assert_eq!(encode(c), Err(Error::InvalidRate));
    }
}
#[test]
fn fields_and_unsupported_profiles_do_not_silently_truncate() {
    let c = base(Chip::Mt7921);
    for bytes in [usize::MAX, 65504] {
        assert_eq!(
            encode(Config {
                frame: Frame { bytes, ..c.frame },
                ..c
            }),
            Err(Error::InvalidLength)
        );
    }
    for header_bytes in [0, 9, 11, 64] {
        assert_eq!(
            encode(Config {
                frame: Frame {
                    header_bytes,
                    ..c.frame
                },
                ..c
            }),
            Err(Error::InvalidHeader)
        );
    }
    assert_eq!(encode(Config { wcid: 20, ..c }), Err(Error::InvalidStation));
    assert_eq!(
        encode(Config {
            frame: Frame { tid: 8, ..c.frame },
            ..c
        }),
        Err(Error::InvalidTid)
    );
    assert_eq!(
        encode(Config {
            frame: Frame {
                sequence: Sequence::Explicit(4096),
                ..c.frame
            },
            ..c
        }),
        Err(Error::InvalidSequence)
    );
    for v in [
        Vif {
            omac: 64,
            wmm: 0,
            band: 0,
            mld: false,
        },
        Vif {
            omac: 0,
            wmm: 4,
            band: 0,
            mld: false,
        },
        Vif {
            omac: 0,
            wmm: 0,
            band: 2,
            mld: false,
        },
        Vif {
            omac: 0,
            wmm: 0,
            band: 0,
            mld: true,
        },
    ] {
        assert_eq!(encode(Config { vif: Some(v), ..c }), Err(Error::InvalidVif));
    }
    let c = base(Chip::Mt7925);
    assert_eq!(
        encode(Config {
            phy_band_one: true,
            ..c
        }),
        Err(Error::InvalidVif)
    );
    assert_eq!(
        encode(Config {
            frame: Frame {
                fragment: Fragment::First,
                ..c.frame
            },
            ..c
        }),
        Err(Error::UnsupportedFragment)
    );
    let w = words(
        encode(Config {
            vif: Some(Vif {
                omac: 1,
                wmm: 0,
                band: 255,
                mld: true,
            }),
            ..c
        })
        .unwrap(),
    );
    assert_eq!(w[1] & (3 << 12), 0);
    assert_eq!(w[6] & 8, 0);
}

#[test]
fn bar_and_action_require_the_unimplemented_body_specific_profile() {
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        for sequence in [Sequence::Hardware, Sequence::Explicit(123)] {
            for (kind, subtype) in [(Kind::Control, 8), (Kind::Management, 13)] {
                let c = base(chip);
                assert_eq!(
                    encode(Config {
                        frame: Frame {
                            kind,
                            subtype,
                            sequence,
                            ..c.frame
                        },
                        ..c
                    }),
                    Err(Error::UnsupportedFrame)
                );
            }
        }
    }
}
#[test]
fn block_ack_action_body_selects_source_request_or_normal_management_tid() {
    for action in 0..=2u8 {
        let mut bytes = [0; 33];
        bytes[..2].copy_from_slice(&0x00d0u16.to_le_bytes());
        bytes[4] = 2;
        bytes[24] = 3;
        bytes[25] = action;
        let mut c = base(Chip::Mt7925);
        c.frame = Frame {
            bytes: bytes.len(),
            header_bytes: 24,
            kind: Kind::Management,
            subtype: 13,
            tid: 7,
            multicast: false,
            fragment: Fragment::None,
            sequence: Sequence::Hardware,
            minimum_rate: true,
        };
        c.protect = false;
        c.hardware_amsdu = false;
        c.fixed_rate = Some(FixedRate::Connac3(18));
        assert_eq!(encode(c), Err(Error::UnsupportedFrame));
        let w = words(encode_mpdu(c, &bytes).unwrap());
        assert_eq!((w[1] >> 21) & 15, if action == 0 { 2 } else { 0 });
        assert_eq!(w[2] & 0x3f, 13);
        assert_ne!(w[1] & (1 << 31), 0);
        assert_eq!(encode_mpdu(c, &bytes[..25]), Err(Error::UnsupportedFrame));
        // Another category is ordinary management.
        bytes[24] = 4;
        let w = words(encode_mpdu(c, &bytes).unwrap());
        assert_eq!(((w[1] >> 21) & 15, w[2] & 0x3f), (0, 13));
    }
}
#[test]
fn connac2_addba_request_asks_for_the_session_on_its_requested_tid() {
    for chip in [Chip::Mt7921, Chip::Mt7922] {
        for action in 0..=2u8 {
            let mut bytes = [0; 33];
            bytes[..2].copy_from_slice(&0x00d0u16.to_le_bytes());
            bytes[4] = 2;
            bytes[24] = 3;
            bytes[25] = action;
            // Block Ack Parameter Set: TID 5.
            bytes[27..29].copy_from_slice(&(5u16 << 2 | 2).to_le_bytes());
            let mut c = base(chip);
            c.frame = Frame {
                bytes: bytes.len(),
                header_bytes: 24,
                kind: Kind::Management,
                subtype: 13,
                tid: 0,
                multicast: false,
                fragment: Fragment::None,
                sequence: Sequence::Hardware,
                minimum_rate: true,
            };
            c.protect = false;
            c.hardware_amsdu = false;
            c.fixed_rate = Some(FixedRate::Connac2(11 | 1 << 6));
            assert_eq!(encode(c), Err(Error::UnsupportedFrame));
            let w = words(encode_mpdu(c, &bytes).unwrap());
            let request = action == 0;
            assert_eq!((w[1] >> 20) & 7, if request { 5 } else { 0 });
            assert_eq!(w[5] & (1 << 14) != 0, request);
            assert_eq!((w[7] >> 16) & 15, 13);
            assert_ne!(w[2] & (1 << 31), 0);
            // Another category is ordinary management.
            bytes[24] = 4;
            let w = words(encode_mpdu(c, &bytes).unwrap());
            assert_eq!(((w[1] >> 20) & 7, w[5] & (1 << 14)), (0, 0));
        }
    }
}
#[test]
fn protected_sa_query_requires_full_protection() {
    for action in 0..=1u8 {
        let mut bytes = [0; 28];
        bytes[..2].copy_from_slice(&0x40d0u16.to_le_bytes());
        bytes[4] = 2;
        bytes[24..].copy_from_slice(&[8, action, 0xa5, 0x5a]);
        let mut config = base(Chip::Mt7925);
        config.frame = Frame {
            bytes: bytes.len(),
            header_bytes: 24,
            kind: Kind::Management,
            subtype: 13,
            tid: 7,
            multicast: false,
            fragment: Fragment::None,
            sequence: Sequence::Hardware,
            minimum_rate: true,
        };
        config.queue = Queue::Management;
        config.hardware_amsdu = false;
        config.fixed_rate = Some(FixedRate::Connac3(18));
        assert_eq!(encode(config), Err(Error::UnsupportedFrame));
        let encoded = words(encode_mpdu(config, &bytes).unwrap());
        assert_eq!((encoded[1] >> 21) & 15, 0);
        assert_eq!(encoded[3] & 2, 2);
        for fault in 0..10 {
            let mut bad = bytes;
            let mut selected = config;
            match fault {
                0 => bad[24] = 4,
                1 => bad[25] = 2,
                2 => bad[1] &= !0x40,
                3 => selected.protect = false,
                4 => bad[4] |= 1,
                5 => bad[1] |= 1,
                6 => bad[22] = 1,
                7 => selected.frame.multicast = true,
                8 => selected.queue = Queue::Data { access_category: 0 },
                _ => selected.chip = Chip::Mt7922,
            }
            assert!(encode_mpdu(selected, &bad).is_err(), "fault {fault}");
        }
        let mut short = config;
        short.frame.bytes -= 1;
        assert!(encode_mpdu(short, &bytes[..27]).is_err());
        let mut long = config;
        long.frame.bytes += 1;
        let mut trailing = bytes.to_vec();
        trailing.push(0);
        assert!(encode_mpdu(long, &trailing).is_err());
    }
}

#[test]
fn protected_block_ack_uses_source_tid_and_cipher_with_exact_body_qualification() {
    for action in 0..=2u8 {
        let len = if action == 2 { 30 } else { 33 };
        let mut bytes = vec![0; len];
        bytes[..2].copy_from_slice(&0x00d0u16.to_le_bytes());
        bytes[4] = 2;
        bytes[24] = 3;
        bytes[25] = action;
        let mut config = base(Chip::Mt7925);
        config.frame = Frame {
            bytes: len,
            header_bytes: 24,
            kind: Kind::Management,
            subtype: 13,
            tid: 7,
            multicast: false,
            fragment: Fragment::None,
            sequence: Sequence::Hardware,
            minimum_rate: true,
        };
        config.queue = Queue::Management;
        config.hardware_amsdu = false;
        config.fixed_rate = Some(FixedRate::Connac3(18));
        config.protect = false;
        let plain = words(encode_mpdu(config, &bytes).unwrap());
        bytes[1] |= 0x40;
        config.protect = true;
        let protected = words(encode_mpdu(config, &bytes).unwrap());
        assert_eq!((protected[1] >> 21) & 15, if action == 0 { 2 } else { 0 });
        for index in 0..8 {
            assert_eq!(
                protected[index],
                plain[index] | if index == 3 { 2 } else { 0 }
            );
        }
        for fault in 0..11 {
            let mut bad = bytes.clone();
            let mut selected = config;
            match fault {
                0 => bad[1] &= !0x40,
                1 => selected.protect = false,
                2 => bad[4] |= 1,
                3 => bad[1] |= 1,
                4 => bad[22] |= 1,
                5 => selected.frame.multicast = true,
                6 => selected.queue = Queue::Data { access_category: 0 },
                7 => bad[24] = 4,
                8 => bad[25] = 3,
                9 => selected.chip = Chip::Mt7922,
                _ => {
                    bad.pop();
                    selected.frame.bytes -= 1;
                }
            }
            assert!(
                encode_mpdu(selected, &bad).is_err(),
                "action {action}, fault {fault}"
            );
        }
    }
}

#[test]
#[ignore = "oracle: STELA_MT792X_TXWI_ORACLE"]
fn unchanged_pinned_raw80211_writers_match_every_descriptor_word() {
    extern crate std;
    let root = std::env::var_os("STELA_MT792X_TXWI_ORACLE").expect("reference trace directory");
    for chip in [Chip::Mt7921, Chip::Mt7922, Chip::Mt7925] {
        let name = match chip {
            Chip::Mt7921 => "7921",
            Chip::Mt7922 => "7922",
            Chip::Mt7925 => "7925",
        };
        let bytes =
            std::fs::read(std::path::Path::new(&root).join(format!("mt{name}.bin"))).unwrap();
        assert_eq!(bytes.len(), 256 * 128);
        for (case, record) in bytes.as_chunks::<128>().0.iter().enumerate() {
            let v: [u32; 24] = core::array::from_fn(|i| {
                u32::from_le_bytes(record[i * 4..i * 4 + 4].try_into().unwrap())
            });
            let f = Frame {
                bytes: v[10] as usize,
                header_bytes: v[11] as u8,
                kind: match v[12] {
                    0 => Kind::Management,
                    1 => Kind::Control,
                    2 => Kind::Data,
                    _ => panic!("frame"),
                },
                subtype: v[13] as u8,
                tid: v[14] as u8,
                multicast: v[15] != 0,
                fragment: match v[16] {
                    0 => Fragment::None,
                    1 => Fragment::First,
                    2 => Fragment::Middle,
                    3 => Fragment::Last,
                    _ => panic!("fragment"),
                },
                sequence: if v[17] != 0 {
                    Sequence::Explicit(v[18] as u16)
                } else {
                    Sequence::Hardware
                },
                minimum_rate: v[22] != 0,
            };
            let config = Config {
                chip,
                wcid: v[0] as u16,
                vif: (v[1] != 0).then_some(Vif {
                    omac: v[2] as u8,
                    wmm: v[3] as u8,
                    band: v[4] as u8,
                    mld: v[5] != 0,
                }),
                phy_band_one: v[6] != 0,
                queue: match v[7] {
                    0 => Queue::Data {
                        access_category: v[8] as u8,
                    },
                    1 => Queue::Management,
                    2 => Queue::Beacon,
                    3 => Queue::Discovery,
                    _ => panic!("queue"),
                },
                pid: v[9] as u8,
                frame: f,
                protect: v[19] != 0,
                no_ack: v[20] != 0,
                hardware_amsdu: v[21] != 0,
                fixed_rate: Some(if chip == Chip::Mt7925 {
                    FixedRate::Connac3(v[23] as u8)
                } else {
                    FixedRate::Connac2(v[23] as u16)
                }),
            };
            assert_eq!(
                encode(config).unwrap(),
                &record[96..],
                "chip {name} case {case} {config:?}"
            );
        }
    }
}

// Plaintext pre-encryption bodies with canonical fields; no CCMP header,
// MIC or host PN is inserted by this descriptor-only fixture.
fn protected_action_reference_frame(action: u8, tid: u8, sequence: u16) -> alloc::vec::Vec<u8> {
    let mut bytes = vec![
        0;
        if action < 2 {
            33
        } else if action == 2 {
            30
        } else {
            28
        }
    ];
    bytes[..2].copy_from_slice(&0x40d0u16.to_le_bytes());
    bytes[4] = 2;
    bytes[10] = 4;
    bytes[16] = 6;
    bytes[22..24].copy_from_slice(&(sequence << 4).to_le_bytes());
    if action < 3 {
        bytes[24] = 3;
        bytes[25] = action;
        let cap = 2 | (u16::from(tid) << 2) | (16 << 6);
        match action {
            0 => {
                bytes[26] = 1;
                bytes[27..29].copy_from_slice(&cap.to_le_bytes());
                bytes[31..33].copy_from_slice(&0x1230u16.to_le_bytes());
            }
            1 => {
                bytes[26] = 1;
                bytes[29..31].copy_from_slice(&cap.to_le_bytes());
            }
            _ => {
                bytes[26..28].copy_from_slice(&((u16::from(tid) << 12) | (1 << 11)).to_le_bytes());
                bytes[28..30].copy_from_slice(&37u16.to_le_bytes());
            }
        }
    } else {
        bytes[24..28].copy_from_slice(&[8, action - 3, 0xa5, 0x5a]);
    }
    bytes
}

#[test]
fn protected_action_reference_inputs_keep_refusals_separate_from_linux_descriptor_trace() {
    for action in 0..5 {
        let bytes = protected_action_reference_frame(action, 7, 0);
        let mut c = base(Chip::Mt7925);
        c.frame = Frame {
            bytes: bytes.len(),
            header_bytes: 24,
            kind: Kind::Management,
            subtype: 13,
            tid: 0,
            multicast: false,
            fragment: Fragment::None,
            sequence: Sequence::Hardware,
            minimum_rate: true,
        };
        c.queue = Queue::Management;
        c.hardware_amsdu = false;
        c.fixed_rate = Some(FixedRate::Connac3(18));
        assert!(encode_mpdu(c, &bytes).is_ok());
        assert_eq!(encode(c), Err(Error::UnsupportedFrame));
        for fault in 0..13 {
            let mut bad = bytes.clone();
            let mut selected = c;
            match fault {
                0 => bad[1] &= !0x40,
                1 => selected.protect = false,
                2 => bad[4] |= 1,
                3 => selected.frame.multicast = true,
                4 => selected.queue = Queue::Data { access_category: 0 },
                5 => bad[22] |= 1,
                6 => bad[1] |= 1,
                7 => bad[24] = 4,
                8 => bad[25] = 255,
                9 => selected.chip = Chip::Mt7922,
                10 => selected.fixed_rate = None,
                11 => selected.frame.bytes += 1,
                _ => {
                    bad.pop();
                    selected.frame.bytes -= 1;
                }
            }
            assert!(
                encode_mpdu(selected, &bad).is_err(),
                "action {action}, fault {fault}"
            );
        }
    }
}

#[test]
#[ignore = "oracle: STELA_MT7925_PROTECTED_ACTION_TXWI_ORACLE"]
fn unchanged_pinned_mt7925_writers_match_protected_block_ack_and_sa_query_words() {
    extern crate std;
    let root = std::env::var_os("STELA_MT7925_PROTECTED_ACTION_TXWI_ORACLE")
        .expect("protected Action reference trace directory");
    let bytes =
        std::fs::read(std::path::Path::new(&root).join("mt7925-protected-actions.bin")).unwrap();
    assert_eq!(bytes.len(), 480 * 132);
    let mut counts = [0; 5];
    for (case, record) in bytes.as_chunks::<132>().0.iter().enumerate() {
        let v: [u32; 16] = core::array::from_fn(|i| {
            u32::from_le_bytes(record[i * 4..i * 4 + 4].try_into().unwrap())
        });
        let action = case / 96;
        let profile = case % 96;
        let expected = [
            case as u32,
            action as u32,
            [7, 0, 19][profile % 3],
            [0, 2, 63][(profile / 3) % 3],
            [0, 3][(profile / 9) % 2],
            [0, 1, 2, 3, 255][profile % 5],
            [0, 2, 3, 255][profile % 4],
            [0, 7, 15][(profile / 4) % 3],
            ((profile / 12) % 2) as u32,
            if (profile / 24) % 2 != 0 { 4095 } else { 0 },
            ((profile / 2) % 2) as u32,
            ((profile / 6) % 2) as u32,
            u32::from(profile % 2 == 0),
            [18, 14, 31, 63][profile % 4],
            if action < 2 {
                33
            } else if action == 2 {
                30
            } else {
                28
            },
            [0, 7, 15][(profile / 8) % 3],
        ];
        assert_eq!(v, expected, "reference fixture case {case}");
        let frame = protected_action_reference_frame(action as u8, v[15] as u8, v[9] as u16);
        assert_eq!(&record[64..64 + frame.len()], frame);
        assert!(record[64 + frame.len()..100].iter().all(|byte| *byte == 0));
        let config = Config {
            chip: Chip::Mt7925,
            wcid: v[2] as u16,
            vif: Some(Vif {
                omac: v[3] as u8,
                wmm: v[4] as u8,
                band: v[5] as u8,
                mld: false,
            }),
            phy_band_one: false,
            queue: Queue::Management,
            pid: v[6] as u8,
            frame: Frame {
                bytes: v[14] as usize,
                header_bytes: 24,
                kind: Kind::Management,
                subtype: 13,
                tid: v[7] as u8,
                multicast: false,
                fragment: Fragment::None,
                sequence: if v[8] == 0 {
                    Sequence::Hardware
                } else {
                    Sequence::Explicit(v[9] as u16)
                },
                minimum_rate: v[12] != 0,
            },
            protect: true,
            no_ack: v[10] != 0,
            hardware_amsdu: v[11] != 0,
            fixed_rate: Some(FixedRate::Connac3(v[13] as u8)),
        };
        assert_eq!(
            encode_mpdu(config, &frame).unwrap(),
            &record[100..132],
            "case {case} {config:?}"
        );
        counts[action] += 1;
    }
    assert_eq!(counts, [96; 5]);
}
