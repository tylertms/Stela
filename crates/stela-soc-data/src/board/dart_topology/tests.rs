use super::*;
use crate::board::fixture_support::flat_fixture_extent;
use alloc::vec;
use stela_contracts::adt::encode;

fn bytes64(values: &[u64]) -> Vec<u8> {
    values
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect()
}

fn legacy(
    compatible: &[u8],
    descriptors: &[u8],
    registers: &[u64],
    extra: &[(&str, &[u8])],
    children: &[Vec<u8>],
) -> Vec<u8> {
    let registers = bytes64(registers);
    let page = 0x4000u32.to_le_bytes();
    let irq = 1035u32.to_le_bytes();
    let mut properties = vec![
        ("name", b"dart\0".as_slice()),
        ("compatible", compatible),
        ("instance", descriptors),
        ("reg", registers.as_slice()),
        ("page-size", page.as_slice()),
        ("interrupts", irq.as_slice()),
    ];
    // Explicit replacements avoid duplicate properties in malformed cases.
    for &(key, value) in extra {
        if let Some(slot) = properties.iter_mut().find(|(existing, _)| *existing == key) {
            slot.1 = value;
        } else {
            properties.push((key, value));
        }
    }
    encode(&properties, children)
}

#[test]
fn legacy_layout_preserves_original_owner_bytes_register_ordinals_and_bitmap_sids() {
    // Original J274AP/J314SAP USB descriptor bytes, independently of
    // their unsupported remap requirement. This fixture admits only MMIO
    // topology; it does not claim that either complete USB node is admitted.
    let descriptors = [b"TRADDARTLLT\0".as_slice(), b"TRADDARTBLK\0"].concat();
    for (compatible, profile, base) in [
        (b"dart,t8020\0".as_slice(), Profile::T8020, 0x382f00000),
        (b"dart,t6000\0".as_slice(), Profile::T6000, 0x702f00000),
    ] {
        let tree = legacy(
            compatible,
            &descriptors,
            &[base, 0x4000, base + 0x80000, 0x4000],
            &[
                ("sids", &0xbu32.to_le_bytes()),
                ("vm-base", &0x80000000u32.to_le_bytes()),
            ],
            &[],
        );
        let node = Node::root(&tree).unwrap();
        let bank = iommu(
            &node,
            &flat_fixture_extent(&|offset| offset.checked_add(0x200000000)),
        )
        .unwrap();
        assert_eq!(bank.profile, profile);
        assert_eq!(bank.streams, 16);
        assert_eq!(bank.vm_base, 0x80000000);
        assert_eq!(bank.sids, [0, 1, 3]);
        assert_eq!(
            bank.instances
                .iter()
                .map(|instance| (instance.number, instance.reg_index))
                .collect::<Vec<_>>(),
            [(0, 0), (1, 1)]
        );
        assert_eq!(bank.instances[0].owner, *b"DARTLLT\0");
        assert_eq!(bank.instances[1].owner, *b"DARTBLK\0");
        assert_eq!(bank.instances[1].physical, base + 0x80000 + 0x200000000);
        assert!(super::super::models(
            core::str::from_utf8(&compatible[..compatible.len() - 1]).unwrap(),
            |key| node.property(key)
        ));
    }
}

#[test]
fn legacy_native_feature_gates_match_public_admission_and_leave_requirements_named() {
    let descriptor = b"TRADDART\0\0\0\0";
    for (extra, requirement) in [
        (vec![("sid-count", 64u32.to_le_bytes().to_vec())], "16-SID"),
        (
            vec![("page-size", 0x1000u32.to_le_bytes().to_vec())],
            "16 KiB page-size",
        ),
        (
            vec![("reg", bytes64(&[0x382f00000, 0x8000]))],
            "register geometry",
        ),
        (
            vec![("remap", vec![0, 0])],
            "incomplete padded legacy remap",
        ),
        (
            vec![("filter-data-instance-0", vec![0; 0x40])],
            "address-permission filter",
        ),
        (
            vec![("sids", 0x10000u32.to_le_bytes().to_vec())],
            "outside the native",
        ),
        (
            vec![("sids", 1u64.to_le_bytes().to_vec())],
            "wide SID bitmap",
        ),
        (
            vec![("sid", 0u32.to_le_bytes().to_vec())],
            "not a modern sid list",
        ),
        (
            vec![("vm-base", 0u64.to_le_bytes().to_vec())],
            "32-bit vm-base",
        ),
        (
            vec![("instance", b"FPADDARTLLT\0".to_vec())],
            "FPAD translation and appended-filter",
        ),
        (
            vec![("instance", b"UMMSSMMURT\0\0".to_vec())],
            "UMMS capture",
        ),
    ] {
        let extra = extra
            .iter()
            .map(|(key, value)| (*key, value.as_slice()))
            .collect::<Vec<_>>();
        let tree = legacy(
            b"dart,t8020\0",
            descriptor,
            &[0x382f00000, 0x4000],
            &extra,
            &[],
        );
        let node = Node::root(&tree).unwrap();
        assert!(
            iommu(&node, &flat_fixture_extent(&Some))
                .unwrap_err()
                .to_string()
                .contains(requirement),
            "{requirement}"
        );
        assert!(!super::super::models("dart,t8020", |key| node.property(key)));
    }
}

#[test]
fn t6000_paired_capture_requires_its_exact_unlocked_policy_and_order() {
    let descriptors = b"TRADDART\0\0\0\0UMMSSMMU\0\0\0\0";
    let registers = [0x89304000, 0x4000, 0x89300000, 0x4000];
    for flags in [
        vec![],
        vec![("never-lock", &[][..])],
        vec![
            ("never-lock", &[][..]),
            ("ignore-inconsistent-hw-locks", &[1][..]),
        ],
    ] {
        let bytes = legacy(b"dart,t6000\0", descriptors, &registers, &flags, &[]);
        assert!(iommu(&Node::root(&bytes).unwrap(), &flat_fixture_extent(&Some)).is_err());
    }
    let flags = [
        ("never-lock", &[][..]),
        ("ignore-inconsistent-hw-locks", &[][..]),
    ];
    let bytes = legacy(b"dart,t6000\0", descriptors, &registers, &flags, &[]);
    let bank = iommu(&Node::root(&bytes).unwrap(), &flat_fixture_extent(&Some)).unwrap();
    assert_eq!(bank.instances.len(), 1);
    assert_eq!(bank.instances[0].reg_index, 0);
    assert_eq!(bank.smmu, Some((0x89300000, 0x4000)));
    for descriptors in [
        b"UMMSSMMU\0\0\0\0TRADDART\0\0\0\0".as_slice(),
        b"TRADDART\0\0\0\0TRADDART\0\0\0\0",
    ] {
        let bytes = legacy(b"dart,t6000\0", descriptors, &registers, &flags, &[]);
        if &descriptors[..4] == b"UMMS" {
            assert!(iommu(&Node::root(&bytes).unwrap(), &flat_fixture_extent(&Some)).is_err());
        } else {
            assert_eq!(
                iommu(&Node::root(&bytes).unwrap(), &flat_fixture_extent(&Some))
                    .unwrap()
                    .instances
                    .len(),
                2
            );
        }
    }
}

#[test]
fn t6000_remaps_do_not_admit_an_unselected_translation_owner() {
    let one = b"TRADDART\0\0\0\0";
    let descriptors = [one.as_slice(), one].concat();
    let bytes = legacy(
        b"dart,t6000\0",
        &descriptors,
        &[0x89304000, 0x4000, 0x8930c000, 0x4000],
        &[("remap", &[0, 1, 0, 0])],
        &[],
    );
    let node = Node::root(&bytes).unwrap();
    assert!(
        iommu(&node, &flat_fixture_extent(&Some))
            .unwrap_err()
            .to_string()
            .contains("multi-instance DMA remap")
    );
    assert!(!super::super::models("dart,t6000", |key| node.property(key)));
    // M1 (`dart,t8020`) SPTM checks the same remap bytes (T8103 VA
    // 0xfffffff0270dc51c..0xfffffff0270dc564).
    for compatible in [b"dart,t8020\0", b"dart,t6000\0"] {
        for remap in [[5, 0, 6, 0], [5, 0, 0, 0], [0, 1, 1, 0], [5, 0, 5, 1]] {
            let bytes = legacy(
                compatible,
                one,
                &[0x89304000, 0x4000],
                &[("remap", &remap)],
                &[],
            );
            let node = Node::root(&bytes).unwrap();
            assert_eq!(
                iommu(&node, &flat_fixture_extent(&Some)).is_ok(),
                remap[1] == 0 && remap[3] == 0
            );
        }
    }
}

#[test]
fn mapper_sid_and_multi_instance_dma_require_native_routing() {
    let mapper = |sid: u32| {
        encode(
            &[
                ("name", b"mapper\0"),
                ("compatible", b"iommu-mapper\0"),
                ("AAPL,phandle", &7u32.to_le_bytes()),
                ("reg", &sid.to_le_bytes()),
            ],
            &[],
        )
    };
    let one = b"TRADDART\0\0\0\0";
    let tree = legacy(
        b"dart,t6000\0",
        one,
        &[0x702f00000, 0x4000],
        &[],
        &[mapper(15)],
    );
    assert_eq!(
        iommu(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some))
            .unwrap()
            .mappers,
        [(7, 15)]
    );
    let tree = legacy(
        b"dart,t6000\0",
        one,
        &[0x702f00000, 0x4000],
        &[],
        &[mapper(16)],
    );
    assert!(
        iommu(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some))
            .unwrap_err()
            .to_string()
            .contains("mapper SID")
    );
    let first = legacy(
        b"dart,t6000\0",
        one,
        &[0x702f00000, 0x4000],
        &[],
        &[mapper(1)],
    );
    let second = legacy(
        b"dart,t6000\0",
        one,
        &[0x702f80000, 0x4000],
        &[],
        &[mapper(2)],
    );
    let tree = encode(&[("name", b"arm-io\0")], &[first, second]);
    assert!(
        iommus(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some))
            .unwrap_err()
            .to_string()
            .contains("ambiguous native DMA routing")
    );
    let two = [one.as_slice(), one].concat();
    let tree = legacy(
        b"dart,t6000\0",
        &two,
        &[0x702f00000, 0x4000, 0x702f80000, 0x4000],
        &[],
        &[mapper(1)],
    );
    assert!(
        iommu(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some))
            .unwrap_err()
            .to_string()
            .contains("multi-instance DMA routing")
    );
}

#[test]
fn native_family_traversal_keeps_mixed_groups_and_refuses_unknown_generations() {
    let one = b"TRADDART\0\0\0\0";
    let legacy = legacy(b"dart,t6000\0", one, &[0x702f00000, 0x4000], &[], &[]);
    let modern = encode(
        &[
            ("name", b"dart-modern\0"),
            ("compatible", b"dart,t8110\0"),
            (
                "instance",
                &[b"TRAD".as_slice(), &[0; 4], b"DART\0\0\0\0"].concat(),
            ),
            ("reg", &bytes64(&[0x703000000, 0x4000])),
            ("sid-count", &64u32.to_le_bytes()),
            ("sid", &15u32.to_le_bytes()),
            ("interrupts", &1u32.to_le_bytes()),
            ("vm-base", &0x100000000u64.to_le_bytes()),
        ],
        &[],
    );
    let tree = encode(&[("name", b"arm-io\0")], &[legacy, modern]);
    let banks = iommus(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some)).unwrap();
    assert_eq!(
        banks.iter().map(|bank| bank.profile).collect::<Vec<_>>(),
        [Profile::T6000, Profile::T8110]
    );
    assert_eq!(banks[1].sids, [15]);
    assert_eq!(banks[1].vm_base, 0x100000000);
    let unknown = encode(&[("compatible", b"dart,gen3\0dart,t8110\0")], &[]);
    assert!(
        iommus(&Node::root(&unknown).unwrap(), &flat_fixture_extent(&Some))
            .unwrap_err()
            .to_string()
            .contains("unsupported or conflicting")
    );
    assert!(!super::super::models("dart,gen3", |_| None));
    let node = Node::root(&unknown).unwrap();
    assert!(!super::super::models("dart,t8110", |key| node.property(key)));
}

#[test]
fn t8110_windows_select_one_register_file_revision_per_node() {
    let node = |windows: &[u64]| {
        let descriptors = (0..windows.len() as u32)
            .flat_map(|number| {
                [b"TRAD".as_slice(), &number.to_le_bytes(), b"DART\0\0\0\0"].concat()
            })
            .collect::<Vec<_>>();
        let registers = windows
            .iter()
            .enumerate()
            .flat_map(|(index, &bytes)| [0x7_0300_0000 + 0x40000 * index as u64, bytes])
            .collect::<Vec<_>>();
        encode(
            &[
                ("name", b"dart-m4\0"),
                ("compatible", b"dart,t8110\0"),
                ("instance", &descriptors),
                ("reg", &bytes64(&registers)),
                ("interrupts", &1u32.to_le_bytes()),
                ("vm-base", &(1u64 << 40).to_le_bytes()),
            ],
            &[],
        )
    };
    // J604AP/J714cAP windows are 0xc000 or 0x20000 bytes; J700AP
    // `dart-ane` mixes both within revision 2.2.
    for (windows, expected) in [
        (&[0x4000][..], Ok(Profile::T8110)),
        (&[0xc000], Ok(Profile::T8110Rev22)),
        (&[0x20000], Ok(Profile::T8110Rev22)),
        (&[0xc000, 0x20000, 0x20000], Ok(Profile::T8110Rev22)),
        (&[0x4000, 0xc000], Err("mixes DART register-file revisions")),
        (
            &[0x20000, 0x4000],
            Err("mixes DART register-file revisions"),
        ),
        (&[0x8000], Err("register window of a modelled revision")),
        (&[0x24000], Err("register window of a modelled revision")),
    ] {
        let tree = node(windows);
        let bank = iommu(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some));
        match expected {
            Ok(profile) => {
                let bank = bank.unwrap();
                assert_eq!(bank.profile, profile, "{windows:x?}");
                assert_eq!(
                    bank.instances.iter().map(|i| i.bytes).collect::<Vec<_>>(),
                    windows
                );
                assert_eq!((bank.streams, bank.vm_base), (16, 1 << 40));
            }
            Err(reason) => assert!(
                bank.unwrap_err().to_string().contains(reason),
                "{windows:x?}"
            ),
        }
    }
}

#[test]
fn t8110_small_window_with_dva_flush_selects_revision_2_0() {
    let tree = encode(
        &[
            ("name", b"dart-usb0\0"),
            ("compatible", b"dart,t8110\0"),
            ("instance", b"TRAD\0\0\0\0DART\0\0\0\0"),
            ("reg", &bytes64(&[0x7_0300_0000, 0x4000])),
            ("interrupts", &1u32.to_le_bytes()),
            ("vm-base", &(1u64 << 40).to_le_bytes()),
            ("flush-by-dva", &[0; 4]),
        ],
        &[],
    );
    let bank = iommu(&Node::root(&tree).unwrap(), &flat_fixture_extent(&Some)).unwrap();
    assert_eq!(bank.profile, Profile::T8110Rev20);
    assert_eq!(bank.vm_base, 1 << 40);
}
