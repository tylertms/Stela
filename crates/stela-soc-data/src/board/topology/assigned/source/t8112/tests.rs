use super::*;

type Properties = Vec<(&'static str, Vec<u8>)>;
fn encode(properties: Properties, children: &[Vec<u8>]) -> Vec<u8> {
    stela_contracts::adt::encode(
        &properties
            .iter()
            .map(|(name, value)| (*name, value.as_slice()))
            .collect::<Vec<_>>(),
        children,
    )
}
fn words(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}
fn wide(values: &[u64]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}
fn authored(prepared: bool) -> Vec<u8> {
    authored_config(prepared, None)
}
fn authored_config(prepared: bool, harvesting: Option<&[u8]>) -> Vec<u8> {
    let mut cpus = Vec::new();
    let mut records = Vec::new();
    for pe in 0..8u32 {
        let cluster = pe / 4;
        let base = 0x2_1000_0000 + u64::from(cluster) * 0x100_0000;
        cpus.push(encode(
            vec![
                ("name", format!("cpu{pe}\0").into_bytes()),
                ("reg", words(&[(cluster << 8) | (pe & 3)])),
                ("cpu-id", words(&[pe])),
                ("cluster-id", words(&[cluster])),
                (
                    "cluster-type",
                    vec![if cluster == 0 { b'E' } else { b'P' }, 0],
                ),
                (
                    "state",
                    if prepared && pe == 4 {
                        b"running\0".to_vec()
                    } else {
                        b"waiting\0".to_vec()
                    },
                ),
                (
                    "l2-cache-size",
                    words(&[if cluster == 0 { 4 << 20 } else { 16 << 20 }]),
                ),
                (
                    "cpu-impl-reg",
                    wide(&[base + 0x50000 + u64::from(pe & 3) * 0x100000, 0x9010]),
                ),
                ("acc-impl-reg", wide(&[base + 0xf00000, 0x40088])),
                ("cpm-impl-reg", wide(&[base + 0xe40000, 0x9010])),
                ("function-enable_core", words(&[0x74, 0x436f7265, 1 << pe])),
                ("interrupt-parent", words(&[0x63])),
                ("interrupts", words(&[0x47 + pe])),
            ],
            &[],
        ));
        records.push(record(
            pe as u16 + 1,
            if cluster == 0 { 2 } else { 5 },
            pe as u8,
            &format!("CPU{pe}"),
            false,
        ));
    }
    records.push(record(9, 2, 8, "ECPM", false));
    records.push(record(10, 5, 9, "PCPM", false));
    records.push(record(0x163, 2, 0, "ECPM_NRG", true));
    records.push(record(0x164, 5, 0, "PCPM_NRG", true));
    let mut groups = vec![0; 72];
    groups[60..].copy_from_slice(&words(&[0, 0, 0x3ff]));
    let mut perf = vec![0; 56];
    perf[2..4].copy_from_slice(&[1, 2]);
    perf[30..32].copy_from_slice(&[1, 5]);
    let mut properties = vec![
        ("name", b"pmgr\0".to_vec()),
        ("compatible", b"pmgr1,t8112\0".to_vec()),
        ("AAPL,phandle", words(&[0x74])),
        ("reg", wide(&[0x3b700000, 0xcc000])),
        ("ps-regs", groups),
        ("perf-domains", perf),
        ("devices", records.into_iter().flatten().collect()),
    ];
    if let Some(value) = harvesting {
        properties.push(("acc-harvesting", value.to_vec()));
    }
    let pmgr = encode(properties, &[]);
    let aic = encode(
        vec![
            ("name", b"aic\0".to_vec()),
            ("compatible", b"aic,2\0".to_vec()),
            ("AAPL,phandle", words(&[0x63])),
            ("#main-cpus", words(&[8])),
        ],
        &[],
    );
    let io = encode(
        vec![
            ("name", b"arm-io\0".to_vec()),
            ("compatible", b"arm-io,t8110\0".to_vec()),
            ("#address-cells", words(&[2])),
            ("#size-cells", words(&[2])),
            ("ranges", wide(&[0, 0x2_0000_0000, 0x1_0000_0000])),
        ],
        &[pmgr, aic],
    );
    let chosen = encode(
        vec![
            ("name", b"chosen\0".to_vec()),
            ("chip-id", words(&[if prepared { 0x8112 } else { 0 }])),
            ("board-id", words(&[if prepared { 0x24 } else { 0 }])),
        ],
        &[],
    );
    let product = encode(
        vec![
            ("name", b"product\0".to_vec()),
            ("product-soc-name", b"Apple M2\0".to_vec()),
        ],
        &[],
    );
    let cpus = encode(
        vec![
            ("name", b"cpus\0".to_vec()),
            ("#address-cells", words(&[1])),
            ("#size-cells", words(&[0])),
        ],
        &cpus,
    );
    encode(
        vec![
            ("name", b"device-tree\0".to_vec()),
            ("#address-cells", words(&[2])),
            ("#size-cells", words(&[2])),
        ],
        &[chosen, product, io, cpus],
    )
}
fn record(id: u16, perf: u8, slot: u8, name: &str, virtual_: bool) -> [u8; 48] {
    let mut row = [0; 48];
    row[0] = if virtual_ { 0x10 } else { 0 };
    row[2] = perf;
    row[10] = slot;
    row[11] = 5;
    row[26..28].copy_from_slice(&id.to_le_bytes());
    row[32..32 + name.len()].copy_from_slice(name.as_bytes());
    row
}
fn identity(original: &[u8], board: u32) -> SourceIdentity {
    SourceIdentity {
        original_tree_sha256: Sha256::digest(original).into(),
        profile: ResourceProfile {
            chip: Chip::AppleM2,
            soc: 0x8112,
            board,
            input_set_sha256: [4; 32],
            adapted_set_sha256: None,
        },
    }
}
fn replace(bytes: &mut [u8], path: &[&str], field: &str, value: &[u8]) {
    let mut node = Node::root(bytes).unwrap();
    for name in path {
        node = node.child(name).unwrap();
    }
    let source = property(&node, field).unwrap();
    assert_eq!(source.len(), value.len());
    let offset = source.as_ptr() as usize - bytes.as_ptr() as usize;
    bytes[offset..offset + value.len()].copy_from_slice(value);
}
#[test]
fn authored_t8112_nonzero_bsp_keeps_canonical_full_graph_and_source_coordinates() {
    let original = authored(false);
    let prepared = authored(true);
    let source =
        BspSource::t8112(&original, &prepared, identity(&original, 0x24), 0x100, 0).unwrap();
    assert_eq!(source.source(), original);
    assert_eq!(source.name(), "cpu4");
    let r = source.resources();
    assert_eq!(r.slots[0].reg, 0x100);
    assert_eq!(r.kinds[0].cluster_type, b'P');
    assert_eq!(r.clusters[0].aff1, 1);
    assert_eq!(r.clusters[0].die_cluster, 1);
    assert_eq!(r.slots[0].cluster_core, 0);
    assert_eq!(r.powers[0].id.device, 5);
    assert_eq!(r.powers[1].id.device, 10);
    assert_eq!(source.power_addresses(), (0x2_3b70_0020, 0x2_3b70_0048));
    assert_eq!(source.graph().domains.len(), 12);
    assert_eq!(r.caches[0].geometry.bytes, 16 << 20);
    struct Roster([AssignedCore; 1]);
    impl RetainedRoster for Roster {
        fn revision(&self) -> u64 {
            1
        }
        fn cores(&self) -> &[AssignedCore] {
            &self.0
        }
    }
    let host = Roster([AssignedCore {
        worker: 1,
        package: 0,
        core: 0,
        apic: 0,
        started: true,
    }]);
    let assignments = [Assignment {
        worker: 1,
        slot: source.slot(),
        boot: true,
    }];
    let plan = Plan::build(&host, &source, &assignments).unwrap();
    assert_eq!(plan.pes()[0].reg, 0x100);
    assert_eq!(plan.pes()[0].logical, 0);
    assert_eq!(plan.clusters()[0].resource.aff1, 1);
    for mode in [1, 2, 3] {
        assert!(
            BspSource::t8112(&original, &prepared, identity(&original, 0x24), 0x100, mode).is_err()
        );
    }
    assert!(BspSource::t8112(&original, &prepared, identity(&original, 0x24), 0, 0).is_err());
    let mut bad = prepared.clone();
    replace(
        &mut bad,
        &["cpus", "cpu4"],
        "function-enable_core",
        &words(&[0x74, 0x436f7265, 1]),
    );
    assert!(BspSource::t8112(&original, &bad, identity(&original, 0x24), 0x100, 0).is_err());
    let mut bad = prepared.clone();
    replace(
        &mut bad,
        &["cpus", "cpu4"],
        "cpu-impl-reg",
        &wide(&[0x2_1105_0000, 0x8000]),
    );
    assert!(BspSource::t8112(&original, &bad, identity(&original, 0x24), 0x100, 0).is_err());
    let mut wrong = identity(&original, 0x24);
    wrong.original_tree_sha256 = [0; 32];
    assert!(BspSource::t8112(&original, &prepared, wrong, 0x100, 0).is_err());
}

#[test]
fn selected_t8112_resources_cover_each_worker_once_and_keep_boot_state() {
    struct Roster(Vec<AssignedCore>);
    impl RetainedRoster for Roster {
        fn revision(&self) -> u64 {
            1
        }
        fn cores(&self) -> &[AssignedCore] {
            &self.0
        }
    }
    let original = authored(false);
    let prepared = authored(true);
    for count in 2..=8 {
        let source = BspSource::t8112_assigned(
            &original,
            &prepared,
            identity(&original, 0x24),
            0x100,
            0,
            count,
        )
        .unwrap();
        let resources = source.resources();
        assert_eq!(source.source(), original);
        assert_eq!(resources.slots.len(), count);
        assert_eq!(resources.slots[0].reg, 0x100);
        assert_eq!(source.name_for(resources.slots[0].id).unwrap(), "cpu4");
        assert_eq!(source.graph().domains.len(), 12);
        let roster = Roster(
            (0..count)
                .map(|worker| AssignedCore {
                    worker,
                    package: 0,
                    core: worker as u32,
                    apic: worker as u32 + 10,
                    started: true,
                })
                .collect(),
        );
        let assignments = resources
            .slots
            .iter()
            .enumerate()
            .map(|(worker, slot)| Assignment {
                worker,
                slot: slot.id,
                boot: worker == 0,
            })
            .collect::<Vec<_>>();
        let plan = Plan::build(&roster, &source, &assignments).unwrap();
        assert_eq!(plan.pes().len(), count);
        assert_eq!(plan.pes()[0].reg, 0x100);
        assert_eq!(plan.pes()[0].core_power.device, 5);
        let mut completed = BTreeSet::new();
        let mut previous = None;
        for pe in plan.pes() {
            if previous != Some(pe.cluster) {
                assert!(!completed.contains(&pe.cluster));
                if let Some(cluster) = previous {
                    completed.insert(cluster);
                }
                previous = Some(pe.cluster);
            }
        }
        assert_eq!(
            resources
                .slots
                .iter()
                .map(|slot| slot.reg)
                .collect::<Vec<_>>(),
            // The boot complex holds up to four PEs, and the other complex
            // always keeps its first PE for the PMGR's complex 0 and 1.
            [0x100, 0x101, 0x102, 0x103][..count.min(5) - 1]
                .iter()
                .chain(&[0, 1, 2, 3][..count.saturating_sub(4).max(1)])
                .copied()
                .collect::<Vec<_>>()
        );
        assert_eq!(resources.clusters.len(), 2);
        assert_eq!(
            resources
                .powers
                .iter()
                .flat_map(|power| &power.cores)
                .count(),
            count
        );
        assert_eq!(
            resources
                .powers
                .iter()
                .flat_map(|power| &power.clusters)
                .count(),
            resources.clusters.len()
        );
    }
    for count in [0, 1, 9, usize::MAX] {
        assert!(
            BspSource::t8112_assigned(
                &original,
                &prepared,
                identity(&original, 0x24),
                0x100,
                0,
                count
            )
            .is_err()
        );
    }
    assert!(
        BspSource::t8112_assigned(&original, &prepared, identity(&original, 0x24), 0, 0, 8)
            .is_err()
    );
}

#[test]
fn equal_provider_settings_must_be_an_exact_u32_zero_even_with_declared_normal_mode() {
    let zero = 0u32.to_le_bytes();
    let original = authored_config(false, Some(&zero));
    let prepared = authored_config(true, Some(&zero));
    assert!(BspSource::t8112(&original, &prepared, identity(&original, 0x24), 0x100, 0).is_ok());
    for value in [1u32.to_le_bytes().to_vec(), vec![], vec![0], vec![0; 8]] {
        let original = authored_config(false, Some(&value));
        let prepared = authored_config(true, Some(&value));
        assert!(
            BspSource::t8112(&original, &prepared, identity(&original, 0x24), 0x100, 0).is_err(),
            "provider bytes {value:?}"
        );
    }
}
