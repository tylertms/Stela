use crate::board::USB_DRD;
/// Explicit physical bus for synthetic full-PA fixture records.
fn flat_bus(children: &[alloc::vec::Vec<u8>]) -> alloc::vec::Vec<u8> {
    let ranges = [0u64, 0, 1u64 << 40]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<alloc::vec::Vec<_>>();
    stela_contracts::adt::encode(&[("name", b"arm-io"), ("ranges", &ranges)], children)
}

use super::*;
use crate::board::fixture_support::flat_fixture_extent;
use {crate::board::DART, stela_contracts::adt::encode};

#[test]
fn usb_and_phy_resource_owners_require_checked_complete_lists() {
    for (compatible, owner) in [
        (
            b"usb-drd,t6000\0usb-drd,t8103\0".as_slice(),
            Some(PowerOwner::Usb(0)),
        ),
        (b"usb-drd,t6020\0usb-drd,t8112\0", Some(PowerOwner::Usb(0))),
        (b"usb-drd,t8122\0", Some(PowerOwner::Usb(0))),
        (b"atc-phy,t6000\0", Some(PowerOwner::Phy(0))),
        (b"atc-phy,t6020\0atc-phy,t8112\0", Some(PowerOwner::Phy(0))),
        (b"usb-drd,t9999\0usb-drd,t8112\0", None),
        (b"usb-drd,t8112\0usb-drd,t9999\0", None),
        (b"atc-phy,t9999\0atc-phy,t8112\0", None),
        (b"atc-phy,t8112\0atc-phy,t9999\0", None),
    ] {
        let child = encode(&[("name", b"port"), ("usb-port-number", &words(&[1]))], &[]);
        let controller = encode(
            &[
                ("name", b"consumer"),
                ("compatible", compatible),
                ("clock-gates", &words(&[1])),
            ],
            &[child],
        );
        let bytes = tree(&[(1, Some(0), [0, 0])], &[controller]);
        let states = power_states(
            &Node::root(&bytes).unwrap(),
            &flat_fixture_extent(&Some),
            0x8112,
        )
        .unwrap()
        .unwrap();
        let resource = states
            .resources
            .iter()
            .find(|resource| resource.name == "/arm-io/consumer")
            .unwrap();
        assert_eq!(resource.owner, owner, "{compatible:?}");
        assert_eq!(resource.clocks, [domain_id(1)]);
        if compatible.starts_with(b"usb-drd,") {
            let child = states
                .resources
                .iter()
                .find(|resource| resource.name == "/arm-io/consumer/port")
                .unwrap();
            assert_eq!(child.owner, owner);
            assert_eq!(child.clocks, [domain_id(1)]);
        }
    }
}

#[test]
fn pmgr_register_child_must_match_parent_family() {
    let parent = encode(&[("name", b"pmgr"), ("compatible", b"pmgr2,arch")], &[]);
    let child = encode(
        &[("name", b"pmgr-child"), ("compatible", b"pmgr2,t8152")],
        &[],
    );
    let bytes = encode(&[("name", b"arm-io")], &[parent.clone(), child]);
    let io = Node::root(&bytes).unwrap();
    let owner = register_node(&io, &io.child("pmgr").unwrap()).unwrap();
    assert!(owner.compatible("pmgr2,t8152"));
    assert_eq!(name_of(&owner), "pmgr-child");
    let wrong = encode(
        &[("name", b"pmgr-child"), ("compatible", b"pmgr1,t8112")],
        &[],
    );
    for children in [alloc::vec![parent.clone()], alloc::vec![parent, wrong]] {
        let bytes = encode(&[("name", b"arm-io")], &children);
        let io = Node::root(&bytes).unwrap();
        assert!(register_node(&io, &io.child("pmgr").unwrap()).is_err());
    }
}

#[test]
#[ignore = "oracle: STELA_TEST_PMGR_ADTS"]
fn original_v2_board_graphs_preserve_all_parents_and_physical_locations() {
    use stela_crypto::{Sha256, digest::Digest};
    let directory = std::env::var("STELA_TEST_PMGR_ADTS").expect("original board ADT directory");
    for (file, expected_sha, domains, registers) in [
        (
            "DeviceTree.j614cap.adt",
            "d91252105f7335211ee5e2a6975cd0602b5de0725c47374aeb29811567de3971",
            490,
            263,
        ),
        (
            "DeviceTree.j700ap.adt",
            "b0fbf661a25626403fa1035ba681d5a7ab8ce13130e2a6d2320d4167255d0a88",
            296,
            138,
        ),
        (
            "DeviceTree.j704ap.adt",
            "66596bdffc33f951d20170b20dfbda79e28b144252144a6a7010ad4fa54bcd2e",
            387,
            141,
        ),
    ] {
        let bytes = std::fs::read(std::path::Path::new(&directory).join(file)).unwrap();
        assert_eq!(hex::encode(Sha256::digest(&bytes)), expected_sha);
        let root = Node::root(&bytes).unwrap();
        let io = root.child("arm-io").unwrap();
        let node = io.child("pmgr").unwrap();
        let states = decode_domain_table(&node, Dies::of(&root, &node).unwrap(), &|a, b| {
            io.translate_range(a, b)
        })
        .unwrap();
        assert_eq!(states.domains.len(), domains);
        assert_eq!(states.registers.len(), registers);
        let coverage = coverage::analyze(&states.domains, &[]).unwrap();
        assert!(coverage.iter().all(|domain| !domain.bindable()));
        // Account for every original resource, including repeated sibling
        // display names. This remains source accounting, not chip/device
        // admission or a lifecycle sink validation.
        let complete = decode_power_states(&root, &node, &|address, bytes| {
            io.translate_range(address, bytes)
        })
        .unwrap();
        let consumers = complete
            .resources
            .iter()
            .enumerate()
            .map(|(id, resource)| Consumer {
                id,
                name: &resource.name,
                clocks: resource.activated_clocks(),
                power: &resource.power,
                sinks: LifecycleSinks::default(),
                admission: resource.admission,
            })
            .collect::<Vec<_>>();
        assert!(
            coverage::analyze(&complete.domains, &consumers)
                .unwrap()
                .iter()
                .all(|domain| !domain.bindable())
        );
        let mut admissions = complete.clone();
        assert_eq!(admissions.apply_admission(&bytes, &[]), Ok(0));
        assert_eq!(admissions, complete);
        assert!(
            states
                .domains
                .iter()
                .any(|domain| domain.parents[2].is_some())
        );
        if file == "DeviceTree.j704ap.adt" {
            let domain = states
                .domains
                .iter()
                .find(|domain| domain.name == "AFISOCNI0")
                .unwrap();
            assert_eq!(
                domain.parents,
                [53, 55, 36, 35].map(|id| Some(domain_id(id)))
            );
        }
    }
}

fn words(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|word| word.to_le_bytes()).collect()
}

fn tree(records: &[(u16, Option<u8>, [u16; 2])], children: &[Vec<u8>]) -> Vec<u8> {
    let devices = records
        .iter()
        .flat_map(|&(id, index, parents)| {
            let mut record = [0; PMGR_DEVICE];
            record[..4]
                .copy_from_slice(&if index.is_none() { PMGR_VIRTUAL } else { 0u32 }.to_le_bytes());
            record[4..6].copy_from_slice(&parents[0].to_le_bytes());
            record[6..8].copy_from_slice(&parents[1].to_le_bytes());
            record[10] = index.unwrap_or(0);
            record[26..28].copy_from_slice(&id.to_le_bytes());
            record
        })
        .collect::<Vec<_>>();
    let reg = [0x1000u64, 0x1000]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<Vec<_>>();
    let pmgr = encode(
        &[
            ("name", b"pmgr"),
            ("compatible", b"pmgr1,t8112"),
            ("reg", &reg),
            ("ps-regs", &words(&[0, 0, 0])),
            ("devices", &devices),
        ],
        &[],
    );
    let mut children = children.to_vec();
    children.insert(0, pmgr);
    flat_bus(&children)
}

#[test]
fn mmio_ownership_follows_supported_parent_aliases() {
    use crate::board::UART;
    let reg = |address: u64| {
        [address, 0x1000]
            .into_iter()
            .flat_map(u64::to_le_bytes)
            .collect::<Vec<_>>()
    };
    let debug = encode(&[("name", b"debug-console")], &[]);
    let unknown = encode(&[("name", b"firmware-endpoint")], &[]);
    let uart = encode(
        &[
            ("name", b"uart0"),
            ("compatible", UART.as_bytes()),
            ("reg", &reg(0x2000)),
            ("clock-gates", &words(&[1])),
        ],
        &[debug, unknown],
    );
    let independent = encode(&[("name", b"debug-console"), ("reg", &reg(0x4000))], &[]);
    let second = encode(
        &[
            ("name", b"uart1"),
            ("compatible", UART.as_bytes()),
            ("reg", &reg(0x3000)),
            ("clock-gates", &words(&[1])),
        ],
        &[independent],
    );
    let encoded = tree(&[(1, Some(0), [0, 0])], &[uart, second]);
    let resources = inventory(&Node::root(&encoded).unwrap()).unwrap();
    let owner = |name: &str| {
        resources
            .iter()
            .find(|resource| resource.name == name)
            .unwrap()
            .owner
    };
    assert_eq!(owner("/arm-io/uart0"), Some(PowerOwner::Mmio(0x2000)));
    assert_eq!(
        owner("/arm-io/uart0/debug-console"),
        Some(PowerOwner::Mmio(0x2000))
    );
    assert_eq!(owner("/arm-io/uart0/firmware-endpoint"), None);
    assert_eq!(owner("/arm-io/uart1/debug-console"), None);
    assert!(
        resources
            .iter()
            .all(|resource| resource.admission == Admission::Required)
    );
    // A candidate owner is metadata, not evidence of complete sinks.
    let parsed = power_states(
        &Node::root(&encoded).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    let consumers = parsed
        .resources
        .iter()
        .enumerate()
        .map(|(id, resource)| Consumer {
            id,
            name: &resource.name,
            clocks: resource.activated_clocks(),
            power: &resource.power,
            sinks: LifecycleSinks::default(),
            admission: resource.admission,
        })
        .collect::<Vec<_>>();
    assert!(!coverage::analyze(&parsed.domains, &consumers).unwrap()[0].bindable());
}

#[test]
fn t6020_hpm_children_use_the_i2c_controller_lifecycle() {
    let hpm = encode(&[("name", b"hpm0"), ("compatible", b"usbc,cd3217")], &[]);
    let manager = encode(
        &[("name", b"hpmBusManager"), ("compatible", b"usbc,manager")],
        &[hpm],
    );
    let reg = [0x4000u64, 0x4000]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<Vec<_>>();
    let i2c = encode(
        &[
            ("name", b"i2c0"),
            ("compatible", b"i2c,t6020\0i2c,s5l8940x\0iic,soft\0"),
            ("reg", &reg),
            ("clock-gates", &words(&[1])),
        ],
        &[manager],
    );
    let bytes = flat_bus(&[i2c]);
    let resources = inventory(&Node::root(&bytes).unwrap()).unwrap();
    for path in [
        "/arm-io/i2c0",
        "/arm-io/i2c0/hpmBusManager",
        "/arm-io/i2c0/hpmBusManager/hpm0",
    ] {
        let resource = resources.iter().find(|entry| entry.name == path).unwrap();
        assert_eq!(resource.owner, Some(PowerOwner::Mmio(0x4000)), "{path}");
        assert_eq!(resource.clocks, [domain_id(1)], "{path}");
    }
}

#[test]
fn dma_dependencies_do_not_inherit_mapper_power_ownership() {
    let mapper = encode(
        &[
            ("name", b"mapper"),
            ("compatible", IOMMU_MAPPER.as_bytes()),
            ("AAPL,phandle", &words(&[1])),
            ("reg", &words(&[0])),
        ],
        &[],
    );
    let dart = encode(
        &[
            ("name", b"dart"),
            ("compatible", DART.as_bytes()),
            ("sid-count", &words(&[16])),
            ("clock-gates", &words(&[0x210, 0x210])),
        ],
        &[mapper],
    );
    let endpoint = encode(&[("name", b"endpoint")], &[]);
    let nub = encode(&[("name", b"nub")], &[endpoint]);
    // Hidden hardware remains a consumer, while iommu-parent alone does
    // not make a DMA client share the DART's clock island.
    let client = encode(
        &[
            ("name", b"controller"),
            ("power-gates", &words(&[2])),
            ("iommu-parent", &words(&[1])),
        ],
        &[nub],
    );
    let bytes = tree(
        &[
            (1, Some(0), [0, 0]),
            (2, Some(1), [1, 0]),
            (0x210, None, [2, 0]),
        ],
        &[dart, client],
    );
    let states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    let find = |name| {
        states
            .resources
            .iter()
            .find(|resource| resource.name == name)
            .unwrap()
    };
    for name in ["/arm-io/dart", "/arm-io/dart/mapper"] {
        assert_eq!(find(name).clocks, [domain_id(0x210)]);
        assert!(find(name).power.is_empty());
        assert_eq!(find(name).owner, Some(PowerOwner::Dart(0)));
    }
    for name in [
        "/arm-io/controller",
        "/arm-io/controller/nub",
        "/arm-io/controller/nub/endpoint",
    ] {
        assert!(find(name).clocks.is_empty());
        assert_eq!(find(name).power, [domain_id(2)]);
        assert_eq!(find(name).owner, None);
    }
}

#[test]
fn repeated_sibling_names_keep_ancestry_and_ambiguous_admission_is_atomic() {
    let first = encode(&[("name", b"view")], &[]);
    let leaf = encode(&[("name", b"leaf")], &[]);
    let second = encode(&[("name", b"view"), ("reg", &words(&[0]))], &[leaf]);
    let hidden = encode(
        &[("name", b"hidden"), ("clock-gates", &words(&[1]))],
        &[first, second],
    );
    let bytes = tree(&[(1, Some(0), [0, 0])], &[hidden]);
    let mut states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    assert_eq!(
        states.apply_admission(&bytes, &["/arm-io/hidden".into()]),
        Ok(2)
    );
    let repeated = states
        .resources
        .iter()
        .filter(|r| r.name == "/arm-io/hidden/view")
        .map(|r| r.admission)
        .collect::<Vec<_>>();
    assert_eq!(repeated, [Admission::Unavailable, Admission::Required]);
    assert_eq!(
        states.resources.last().unwrap().admission,
        Admission::Required
    );
    let before = states.clone();
    for entry in ["/arm-io/hidden/view", "/arm-io/hidden/view +no-pmu"] {
        assert!(
            states
                .apply_admission(&bytes, &[entry.into()])
                .unwrap_err()
                .to_string()
                .contains("ambiguous")
        );
        assert_eq!(states, before);
    }
    // A full path with only one matching node remains usable even when
    // a shorter ancestor path has multiple matches.
    assert_eq!(
        states.apply_admission(&bytes, &["/arm-io/hidden/view/leaf".into()]),
        Ok(1)
    );
    assert_eq!(
        states.resources.last().unwrap().admission,
        Admission::Unavailable
    );
    assert!(
        states.resources[..states.resources.len() - 1]
            .iter()
            .all(|r| r.admission == Admission::Required)
    );
}

#[test]
fn positive_admission_is_atomic_and_preserves_independent_hardware_obligations() {
    let logical = encode(&[("name", b"logical")], &[]);
    let unknown = encode(&[("name", b"unknown"), ("reg", &words(&[0]))], &[]);
    let visible = encode(
        &[("name", b"visible"), ("compatible", b"required,unknown")],
        &[],
    );
    let native = encode(
        &[
            ("name", b"dart"),
            ("compatible", DART.as_bytes()),
            ("sid-count", &words(&[16])),
        ],
        &[],
    );
    let hidden = encode(
        &[("name", b"hidden"), ("clock-gates", &words(&[1]))],
        &[logical, unknown, visible, native],
    );
    let leaf = encode(&[("name", b"leaf"), ("power-gates", &words(&[2]))], &[]);
    let bytes = tree(
        &[(1, Some(0), [0, 0]), (2, Some(1), [0, 0])],
        &[hidden, leaf],
    );
    let mut states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    let original = states.clone();
    assert_eq!(states.apply_admission(&bytes, &[]), Ok(0));
    assert_eq!(
        states.apply_admission(&bytes, &["/arm-io/hidden".into(), "/arm-io/leaf".into()]),
        Ok(3)
    );
    for path in ["/arm-io/hidden", "/arm-io/hidden/logical", "/arm-io/leaf"] {
        assert_eq!(
            states
                .resources
                .iter()
                .find(|r| r.name == path)
                .unwrap()
                .admission,
            Admission::Unavailable
        );
    }
    for path in [
        "/arm-io/hidden/unknown",
        "/arm-io/hidden/visible",
        "/arm-io/hidden/dart",
    ] {
        assert_eq!(
            states
                .resources
                .iter()
                .find(|r| r.name == path)
                .unwrap()
                .admission,
            Admission::Required
        );
    }
    let accepted = states.clone();
    for ledger in [
        vec!["/arm-io/missing".into()],
        vec!["/arm-io/hidden/visible".into()],
        vec!["/arm-io/hidden/dart".into()],
        vec!["/arm-io".into()],
        vec!["/arm-io/hidden/".into()],
        vec!["/arm-io/hidden/../leaf".into()],
        vec!["/arm-io/hidden".into(), "/arm-io/hidden".into()],
        vec!["/arm-io/hidden +unmodeled".into()],
    ] {
        assert!(states.apply_admission(&bytes, &ledger).is_err());
        assert_eq!(states, accepted);
    }
    let mut mismatched = original;
    mismatched.resources[0].clocks.push(domain_id(1));
    assert!(
        mismatched
            .apply_admission(&bytes, &["/arm-io/leaf".into()])
            .is_err()
    );
}

#[test]
fn preparation_notes_are_validated_without_retiring_resources() {
    let watched = encode(
        &[
            ("name", b"watchdog"),
            ("clock-gates", &words(&[1])),
            ("no-pmu", b""),
        ],
        &[],
    );
    let bytes = tree(&[(1, Some(0), [0, 0])], &[watched]);
    let mut states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    assert_eq!(
        states.apply_admission(&bytes, &["/arm-io/watchdog +no-pmu".into()]),
        Ok(0)
    );
    assert!(
        states
            .resources
            .iter()
            .all(|r| r.admission == Admission::Required)
    );
    assert!(
        states
            .apply_admission(&bytes, &["/arm-io/watchdog +gptbat-ready".into()])
            .is_err()
    );
    // A retired capability note holds only once the property is gone.
    assert_eq!(
        states.apply_admission(&bytes, &["/arm-io/watchdog ane-type".into()]),
        Ok(0)
    );
    let typed = encode(
        &[
            ("name", b"watchdog"),
            ("clock-gates", &words(&[1])),
            ("ane-type", &words(&[0x80])),
        ],
        &[],
    );
    let bytes = tree(&[(1, Some(0), [0, 0])], &[typed]);
    let mut states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    assert!(
        states
            .apply_admission(&bytes, &["/arm-io/watchdog ane-type".into()])
            .is_err()
    );
}

#[test]
#[ignore = "oracle: STELA_TEST_PMGR_BUNDLE"]
fn prepared_j473_admission_keeps_complete_graph_and_names_all_remaining_sinks() {
    use stela_crypto::{Sha256, digest::Digest};
    let directory = std::path::PathBuf::from(
        std::env::var("STELA_TEST_PMGR_BUNDLE").expect("prepared bundle directory"),
    );
    let bytes = std::fs::read(directory.join("devicetree.adt")).unwrap();
    assert_eq!(
        hex::encode(Sha256::digest(&bytes)),
        "d60dbd13b11a6188f6bfa19a6fe173d49950d5b619197a5a7f4e9d76734d1680"
    );
    let manifest = stela_formats::boot_bundle::Manifest::parse(
        &std::fs::read(directory.join("bundle.json")).unwrap(),
    )
    .unwrap();
    let root = Node::root(&bytes).unwrap();
    let mut states = power_states(
        &root,
        &|address, bytes| {
            root.child("arm-io")
                .unwrap()
                .translate_range(address, bytes)
        },
        0x8112,
    )
    .unwrap()
    .unwrap();
    let original = states.clone();
    let retired = states
        .apply_admission(&bytes, &manifest.hidden_devices)
        .unwrap();
    assert_eq!(states.domains, original.domains);
    assert_eq!(states.registers, original.registers);
    assert_eq!(states.resources.len(), original.resources.len());
    let complete = LifecycleSinks {
        clock: true,
        power: true,
        quiesce: true,
        reset: true,
    };
    let consumers = states
        .resources
        .iter()
        .enumerate()
        .map(|(id, resource)| Consumer {
            id,
            name: &resource.name,
            clocks: resource.activated_clocks(),
            power: &resource.power,
            sinks: if matches!(
                resource.owner,
                Some(PowerOwner::Dart(_) | PowerOwner::Usb(_) | PowerOwner::Phy(_))
            ) {
                complete
            } else {
                LifecycleSinks::default()
            },
            admission: resource.admission,
        })
        .collect::<Vec<_>>();
    let coverage = coverage::analyze(&states.domains, &consumers).unwrap();
    let ave = coverage
        .iter()
        .find(|record| record.address == 0x23b708020)
        .unwrap();
    assert!(ave.bindable());
    assert!(
        ave.consumers
            .iter()
            .all(|c| c.admission == Admission::Unavailable && !c.sinks.complete())
    );
    let mut missing = BTreeSet::new();
    for domain in coverage.iter().filter(|domain| !domain.bindable()) {
        let names = domain
            .consumers
            .iter()
            .filter(|c| c.admission == Admission::Required && !c.sinks.complete())
            .map(|c| c.name.as_str())
            .collect::<Vec<_>>();
        missing.extend(names.iter().map(|name| String::from(*name)));
        std::println!("PMGR unresolved {:#x}: {:?}", domain.address, names);
    }
    std::println!(
        "PMGR retired={} domains={} physical={} bindable={} remaining={:?}",
        retired,
        states.domains.len(),
        coverage.len(),
        coverage.iter().filter(|d| d.bindable()).count(),
        missing
    );
    assert!(!missing.is_empty());
    assert!(missing.contains("/arm-io/ans"));
    assert!(
        coverage
            .iter()
            .filter(|d| d.consumers.is_empty())
            .all(|d| !d.bindable())
    );
}

#[test]
fn inventory_uses_exact_admitted_dart_profiles_and_legacy_mapper_default() {
    let mapper = encode(
        &[
            ("name", b"mapper"),
            ("compatible", IOMMU_MAPPER.as_bytes()),
            ("AAPL,phandle", &words(&[7])),
            ("reg", &words(&[15])),
        ],
        &[],
    );
    let dart = |name: &[u8], compatible: &[u8]| {
        encode(
            &[
                ("name", name),
                ("compatible", compatible),
                ("clock-gates", &words(&[1])),
            ],
            core::slice::from_ref(&mapper),
        )
    };
    let phy = encode(
        &[
            ("name", b"phy"),
            ("compatible", b"atc-phy,t8112"),
            ("clock-gates", &words(&[1])),
        ],
        &[],
    );
    let bytes = tree(
        &[(1, Some(0), [0, 0])],
        &[
            dart(b"legacy", b"dart,t8020"),
            dart(b"unknown", b"dart,t8110\0dart,gen3"),
            dart(b"modern", b"dart,t8110"),
            phy,
        ],
    );
    let states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    let owner = |name| {
        states
            .resources
            .iter()
            .find(|resource| resource.name == name)
            .unwrap()
            .owner
    };
    assert_eq!(owner("/arm-io/legacy"), Some(PowerOwner::Dart(0)));
    assert_eq!(owner("/arm-io/legacy/mapper"), Some(PowerOwner::Dart(0)));
    assert_eq!(owner("/arm-io/unknown"), None);
    assert_eq!(owner("/arm-io/unknown/mapper"), None);
    assert_eq!(owner("/arm-io/modern"), Some(PowerOwner::Dart(1)));
    // An absent sid-count is 16 streams, so SID 15 is a native mapper.
    assert_eq!(owner("/arm-io/modern/mapper"), Some(PowerOwner::Dart(1)));
    assert_eq!(owner("/arm-io/phy"), Some(PowerOwner::Phy(0)));
}

#[test]
fn usb_port_ownership_requires_valid_number_and_no_register_range() {
    let port = |number: u32, register: bool| {
        let number = words(&[number]);
        let mut properties = alloc::vec![
            ("name", b"port".as_slice()),
            ("usb-port-number", number.as_slice())
        ];
        if register {
            properties.push(("reg", b"\0\0\0\0".as_slice()));
        }
        encode(&properties, &[])
    };
    for (number, register, owned) in [
        (1, false, true),
        (2, false, true),
        (3, false, false),
        (1, true, false),
    ] {
        let controller = encode(
            &[
                ("name", b"usb"),
                ("compatible", USB_DRD.as_bytes()),
                ("clock-gates", &words(&[1])),
            ],
            &[port(number, register)],
        );
        let bytes = tree(&[(1, Some(0), [0, 0])], &[controller]);
        let states = power_states(
            &Node::root(&bytes).unwrap(),
            &flat_fixture_extent(&Some),
            0x8112,
        )
        .unwrap()
        .unwrap();
        let controller = states
            .resources
            .iter()
            .find(|resource| resource.name == "/arm-io/usb")
            .unwrap();
        let port = states
            .resources
            .iter()
            .find(|resource| resource.name == "/arm-io/usb/port")
            .unwrap();
        assert_eq!(controller.owner, Some(PowerOwner::Usb(0)));
        assert_eq!(port.owner, owned.then_some(PowerOwner::Usb(0)));
        assert_eq!(port.clocks, [domain_id(1)]);
    }
}

#[test]
fn parsing_rejects_broken_graphs() {
    let records = [
        (1, Some(0), [0, 0]),
        (2, Some(1), [0, 0]),
        (3, Some(2), [1, 0]),
        (4, Some(2), [2, 0]),
    ];
    let bytes = tree(&records, &[]);
    let states = power_states(
        &Node::root(&bytes).unwrap(),
        &flat_fixture_extent(&Some),
        0x8112,
    )
    .unwrap()
    .unwrap();
    assert_eq!(states.domains.len(), 4);
    assert_eq!(states.registers, [0x1000, 0x1008, 0x1010]);
    assert_eq!(states.domains[2].address, states.domains[3].address);
    assert_eq!(states.domains[2].parents[0], Some(domain_id(1)));
    assert_eq!(states.domains[3].parents[0], Some(domain_id(2)));
    for records in [
        vec![(1, Some(0), [99, 0])],
        vec![(1, Some(0), [2, 0]), (2, Some(1), [1, 0])],
        vec![(1, Some(0), [0, 0]), (1, Some(1), [0, 0])],
        vec![
            (1, Some(0), [2, 0]),
            (2, Some(1), [0, 0]),
            (3, Some(1), [4, 0]),
            (4, Some(0), [0, 0]),
        ],
    ] {
        let bytes = tree(&records, &[]);
        assert!(
            power_states(
                &Node::root(&bytes).unwrap(),
                &flat_fixture_extent(&Some),
                0x8112
            )
            .is_err()
        );
    }
    for property in ["clock-gates", "power-gates"] {
        let malformed = encode(&[("name", b"controller"), (property, &[1, 0, 0])], &[]);
        let bytes = tree(&records, &[malformed]);
        assert!(
            power_states(
                &Node::root(&bytes).unwrap(),
                &flat_fixture_extent(&Some),
                0x8112
            )
            .is_err()
        );
    }
}

/// Immutable original fixture stays local. Run with STELA_TEST_PMGR_ADT
/// pointing at decoded UniversalMac 26A434 DeviceTree.j473ap.adt.
#[test]
#[ignore = "oracle: STELA_TEST_PMGR_ADT"]
fn original_j473_resource_graph_keeps_shared_hardware_and_clock_only_usb_darts() {
    use stela_crypto::{Sha256, digest::Digest};
    let path = std::env::var("STELA_TEST_PMGR_ADT").expect("original ADT path");
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(
        hex::encode(Sha256::digest(&bytes)),
        "a99fa2260f292236fbad210fcee380fc69f101e591125bce347819e84e68034c"
    );
    let root = Node::root(&bytes).unwrap();
    let states = power_states(
        &root,
        &|address, bytes| {
            root.child("arm-io")
                .unwrap()
                .translate_range(address, bytes)
        },
        0x8112,
    )
    .unwrap()
    .unwrap();
    assert_eq!(states.domains.len(), 303);
    assert_eq!(states.resources.len(), 330);
    assert!(states.domains.iter().all(|domain| !domain.name.is_empty()));
    let isp = states
        .domains
        .iter()
        .find(|domain| domain.id == domain_id(0x7a))
        .unwrap();
    assert_eq!(isp.name, "ISP_SYS");
    assert_eq!(isp.address, Some(0x2_3b70_0438));
    for (number, id) in [(0, 0x210), (1, 0x211)] {
        let path = format!("/arm-io/dart-usb{number}");
        let resource = states
            .resources
            .iter()
            .find(|resource| resource.name == path)
            .unwrap();
        assert_eq!(resource.clocks, [domain_id(id)]);
        assert!(resource.power.is_empty());
        assert!(resource.owner.is_some());
        let mapper = states
            .resources
            .iter()
            .find(|resource| resource.name == format!("{path}/mapper-usb{number}"))
            .unwrap();
        assert_eq!(mapper.owner, resource.owner);
    }
    let complete = LifecycleSinks {
        clock: true,
        power: true,
        quiesce: true,
        reset: true,
    };
    let consumers = states
        .resources
        .iter()
        .enumerate()
        .map(|(id, resource)| Consumer {
            id,
            name: &resource.name,
            clocks: resource.activated_clocks(),
            power: &resource.power,
            sinks: if matches!(
                resource.owner,
                Some(PowerOwner::Dart(_) | PowerOwner::Usb(_) | PowerOwner::Phy(_))
            ) {
                complete
            } else {
                LifecycleSinks::default()
            },
            admission: resource.admission,
        })
        .collect::<Vec<_>>();
    let coverage = coverage::analyze(&states.domains, &consumers).unwrap();
    assert_eq!(coverage.len(), 153);
    for device in [0x108, 0x109] {
        let address = states
            .domains
            .iter()
            .find(|domain| domain.id == domain_id(device))
            .unwrap()
            .address
            .unwrap();
        let leaf = coverage
            .iter()
            .find(|record| record.address == address)
            .unwrap();
        assert!(
            leaf.bindable(),
            "USB leaf {device:#x} has both hardware sinks"
        );
        assert!(
            leaf.consumers
                .iter()
                .any(|consumer| consumer.name.contains("/dart-usb"))
        );
        assert!(
            leaf.consumers
                .iter()
                .any(|consumer| consumer.name.contains("/usb-drd"))
        );
    }
    let sio_busif = states
        .domains
        .iter()
        .find(|domain| domain.id == domain_id(38))
        .unwrap()
        .address
        .unwrap();
    let shared = coverage
        .iter()
        .find(|record| record.address == sio_busif)
        .unwrap();
    assert!(!shared.bindable());
    assert!(
        shared
            .consumers
            .iter()
            .any(|consumer| consumer.name == "/arm-io/dart-sio" && consumer.sinks.complete())
    );
    assert!(shared.consumers.iter().any(|consumer| consumer.name
        == "/arm-io/sio/iop-sio-nub/sio-dma"
        && !consumer.sinks.complete()));
    std::println!(
        "original PMGR domains={} resources={} physical={} bindable={} SIO_BUSIF-consumers={}",
        states.domains.len(),
        states.resources.len(),
        coverage.len(),
        coverage.iter().filter(|record| record.bindable()).count(),
        shared.consumers.len()
    );
}
/// A synthetic SoC with CPUs on `dies` dies, two die-0 records (device 2
/// parented by device 1) and one consumer gating device 2 on each die.
fn multi_die(dies: u32, compatible: &[u8], stride: Option<u64>) -> Vec<u8> {
    let devices = [(1u16, 0u8, 0u16), (2, 1, 1)]
        .into_iter()
        .flat_map(|(id, index, parent)| {
            let mut record = [0; PMGR_DEVICE];
            record[4..6].copy_from_slice(&parent.to_le_bytes());
            record[10] = index;
            record[26..28].copy_from_slice(&id.to_le_bytes());
            record
        })
        .collect::<Vec<_>>();
    let reg = [0x1000u64, 0x1000]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<Vec<_>>();
    let ps = words(&[0, 0, 0]);
    let stride = stride.map(u64::to_le_bytes);
    let mut properties = alloc::vec![
        ("name", b"pmgr".as_slice()),
        ("compatible", compatible),
        ("reg", &reg),
        ("ps-regs", &ps),
        ("devices", &devices),
    ];
    if let Some(stride) = &stride {
        properties.push(("die-stride", stride));
    }
    let pmgr = encode(&properties, &[]);
    let gates = words(&[2, 0x1000_0002]);
    let usb = encode(&[("name", b"usb"), ("clock-gates", &gates)], &[]);
    let cpus = (0..2)
        .map(|cpu| {
            let die = words(&[cpu * (dies - 1)]);
            encode(&[("name", b"cpu"), ("die-id", &die)], &[])
        })
        .collect::<Vec<_>>();
    encode(
        &[("name", b"device-tree")],
        &[encode(&[("name", b"cpus")], &cpus), flat_bus(&[pmgr, usb])],
    )
}

fn multi_die_states(bytes: &[u8]) -> Result<PowerStates, crate::Refusal> {
    let root = Node::root(bytes).unwrap();
    let io = root.child("arm-io").unwrap();
    decode_power_states(&root, &io.child("pmgr").unwrap(), &|address, bytes| {
        io.translate_range(address, bytes)
    })
}

#[test]
fn die_checked_gates_reach_each_dies_strided_pmgr_records() {
    let id = |die, device| DomainId { die, device };
    for (compatible, property, stride) in [
        (&b"pmgr1,t8112"[..], Some(0x40_0000_0000), 0x40_0000_0000),
        // Without die-stride the bound family driver's stride applies.
        (b"pmgr1,t6001", None, 1 << 37),
        (b"pmgr1,t6021", Some(0), 1 << 37),
    ] {
        let states = multi_die_states(&multi_die(2, compatible, property)).unwrap();
        assert_eq!(
            states.domains.iter().map(|d| d.id).collect::<Vec<_>>(),
            [id(0, 1), id(0, 2), id(1, 1), id(1, 2)]
        );
        assert_eq!(states.domains[2].address, Some(0x1000 + stride));
        assert_eq!(states.domains[3].address, Some(0x1008 + stride));
        assert_eq!(states.domains[3].parents[0], Some(id(1, 1)));
        assert_eq!(
            states.registers,
            [0x1000, 0x1008, 0x1000 + stride, 0x1008 + stride]
        );
        assert_eq!(states.blocks, [(0x1000, 0xc), (0x1000 + stride, 0xc)]);
        let usb = states
            .resources
            .iter()
            .find(|resource| resource.name == "/arm-io/usb")
            .unwrap();
        assert_eq!(usb.clocks, [id(0, 2), id(1, 2)]);
        let consumers = [Consumer {
            id: 0,
            name: &usb.name,
            clocks: &usb.clocks,
            power: &[],
            sinks: LifecycleSinks::default(),
            admission: Admission::Required,
        }];
        // Each die's request reaches that die's register and its parent.
        let coverage = coverage::analyze(&states.domains, &consumers).unwrap();
        assert_eq!(coverage.len(), 4);
        assert!(coverage.iter().all(|domain| domain.consumers.len() == 1));
    }
}

#[test]
fn unplaced_dies_and_reserved_gate_bits_stay_explicit_requirements() {
    // A die without a checked stride has no placed PMGR instance.
    let error = multi_die_states(&multi_die(2, b"pmgr1,t8112", None)).unwrap_err();
    assert!(
        error.to_string().contains("without a qualified die-stride"),
        "{error}"
    );
    // A one-die SoC places no die-1 records for a die-1 gate.
    let error = multi_die_states(&multi_die(1, b"pmgr1,t8112", None)).unwrap_err();
    assert!(error.to_string().contains("UnknownRequirement"), "{error}");
    // A stride inside one die's bank never aliases two dies' registers.
    let error = multi_die_states(&multi_die(2, b"pmgr1,t8112", Some(8))).unwrap_err();
    assert!(
        error.to_string().contains("overlaps another die"),
        "{error}"
    );
    // The 4-bit die field reaches 16 dies.
    let error = multi_die_states(&multi_die(17, b"pmgr1,t8112", Some(1))).unwrap_err();
    assert!(
        error.to_string().contains("exceeds the PMGR die field"),
        "{error}"
    );
    let bytes = encode(&[("clock-gates", &words(&[0x0001_0002]))], &[]);
    assert!(
        gate_ids(&Node::root(&bytes).unwrap(), "/node", "clock-gates")
            .unwrap_err()
            .to_string()
            .contains("reserved")
    );
    let bytes = encode(&[("clock-gates", &words(&[0xf000_0002]))], &[]);
    assert_eq!(
        gate_ids(&Node::root(&bytes).unwrap(), "/node", "clock-gates"),
        Ok(alloc::vec![DomainId { die: 15, device: 2 }])
    );
}

#[test]
fn physical_candidates_follow_nested_bus_domains_and_never_alias_protocol_slaves() {
    let reg = [0x100u64, 0x1000]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<Vec<_>>();
    let uart = encode(
        &[
            ("name", b"uart"),
            ("compatible", super::super::UART.as_bytes()),
            ("reg", &reg),
        ],
        &[],
    );
    let ranges = [0u64, 0x2000, 0x2000]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<Vec<_>>();
    let nested = encode(
        &[("name", b"nested"), ("ranges", &ranges)],
        core::slice::from_ref(&uart),
    );
    let protocol = encode(
        &[
            ("name", b"serial"),
            ("compatible", super::super::SPI.as_bytes()),
        ],
        &[uart],
    );
    let root_ranges = [0u64, 0x100000000, 0x10000]
        .into_iter()
        .flat_map(u64::to_le_bytes)
        .collect::<Vec<_>>();
    let bytes = encode(
        &[("name", b"arm-io"), ("ranges", &root_ranges)],
        &[nested, protocol],
    );
    let resources = inventory(&Node::root(&bytes).unwrap()).unwrap();
    let owner = |path| {
        resources
            .iter()
            .find(|entry| entry.name == path)
            .unwrap()
            .owner
    };
    assert_eq!(
        owner("/arm-io/nested/uart"),
        Some(PowerOwner::Mmio(0x100002100))
    );
    assert_eq!(owner("/arm-io/serial/uart"), None);
}
