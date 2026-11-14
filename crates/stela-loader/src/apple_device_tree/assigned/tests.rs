use super::*;
use alloc::vec;
use core::cell::Cell;
use {
    stela_contracts::Chip, stela_contracts::caches::Cache,
    stela_soc_data::board::assigned::AssignedCore, stela_soc_data::board::assigned::Assignment,
    stela_soc_data::board::assigned::Capacity, stela_soc_data::board::assigned::CapacityKind,
    stela_soc_data::board::assigned::ClusterId, stela_soc_data::board::assigned::ClusterSlot,
    stela_soc_data::board::assigned::CoreKind, stela_soc_data::board::assigned::InterruptTarget,
    stela_soc_data::board::assigned::KindId, stela_soc_data::board::assigned::PeSlot,
    stela_soc_data::board::assigned::PowerResource, stela_soc_data::board::assigned::RegisterRange,
    stela_soc_data::board::assigned::ResourceProfile, stela_soc_data::board::assigned::Resources,
    stela_soc_data::board::assigned::RetainedResources,
    stela_soc_data::board::assigned::RetainedRoster, stela_soc_data::board::assigned::SharedCache,
    stela_soc_data::board::assigned::StartResource,
};
struct Roster {
    cores: Vec<AssignedCore>,
    revision: Cell<u64>,
}
impl RetainedRoster for Roster {
    fn revision(&self) -> u64 {
        self.revision.get()
    }
    fn cores(&self) -> &[AssignedCore] {
        &self.cores
    }
}
struct Inventory {
    soc: u32,
    capacities: Vec<Capacity>,
    kinds: Vec<CoreKind>,
    slots: Vec<PeSlot>,
    clusters: Vec<ClusterSlot>,
    caches: Vec<SharedCache>,
    interrupts: Vec<InterruptTarget>,
    starts: Vec<StartResource>,
    powers: Vec<PowerResource>,
}
impl RetainedResources for Inventory {
    fn revision(&self) -> u64 {
        1
    }
    fn resources(&self) -> Resources<'_> {
        Resources {
            profile: ResourceProfile {
                chip: Chip::AppleM1,
                soc: self.soc,
                board: 8,
                input_set_sha256: [1; 32],
                adapted_set_sha256: None,
            },
            register_granule: 0x4000,
            provenance: "synthetic test resource ownership",
            capacities: &self.capacities,
            kinds: &self.kinds,
            slots: &self.slots,
            clusters: &self.clusters,
            caches: &self.caches,
            interrupts: &self.interrupts,
            starts: &self.starts,
            powers: &self.powers,
        }
    }
}
fn power(id: u16) -> DomainId {
    DomainId { die: 0, device: id }
}
fn range(index: u64) -> RegisterRange {
    RegisterRange {
        base: 0x1000_0000 + index * 0x10000,
        bytes: 0x9008,
    }
}
fn set_word(node: &mut Node, name: &str, value: u32) {
    node.set_property(name, value.to_le_bytes().to_vec())
        .unwrap();
}
fn set_range(node: &mut Node, name: &str, range: RegisterRange) {
    let mut v = range.base.to_le_bytes().to_vec();
    v.extend(range.bytes.to_le_bytes());
    node.set_property(name, v).unwrap();
}
struct Fixture {
    tree: DeviceTree,
    inventory: Inventory,
    host: Roster,
    assignments: Vec<Assignment>,
    bindings: Vec<NodeBinding<'static>>,
    counts: Vec<CountField<'static>>,
}
impl Fixture {
    fn new() -> Self {
        let data = Cache {
            bytes: 64 << 10,
            ways: 4,
            line: 64,
        };
        let mut inventory = Inventory {
            soc: 0x6000,
            capacities: [
                CapacityKind::KernelCpus,
                CapacityKind::KernelClusters,
                CapacityKind::KernelAffinitySpan,
                CapacityKind::KernelPhysicalCluster,
                CapacityKind::KernelCpuMask,
                CapacityKind::MonitorCpus,
                CapacityKind::PlatformCpus,
                CapacityKind::PlatformClusters,
            ]
            .into_iter()
            .map(|kind| Capacity {
                kind,
                maximum: Some(20),
                provenance: "synthetic",
            })
            .collect(),
            kinds: vec![],
            slots: vec![],
            clusters: vec![],
            caches: vec![],
            interrupts: vec![],
            starts: vec![],
            powers: vec![],
        };
        for (i, kind) in (*b"EP").into_iter().enumerate() {
            inventory.kinds.push(CoreKind {
                id: KindId(i as u32),
                cluster_type: kind,
                midr: 0x6100_0000 + i as u64,
                aff2: i as u8,
                clock: None,
                instruction: data,
                data,
            });
            inventory.clusters.push(ClusterSlot {
                id: ClusterId(i as u32),
                aff1: i as u8,
                firmware_id: i as u32 + 10,
                die: 0,
                die_cluster: i as u32,
                cache: Some(CacheId(i as u32)),
                power: Some(power(10 + i as u16)),
                acc_impl: Some(range(10 + i as u64 * 2)),
                cpm_impl: Some(range(11 + i as u64 * 2)),
            });
            inventory.caches.push(SharedCache {
                id: CacheId(i as u32),
                cluster: ClusterId(i as u32),
                geometry: Cache {
                    bytes: (4 << 20) << i,
                    ways: 16,
                    line: 128,
                },
            });
            inventory.powers.push(PowerResource {
                id: power(10 + i as u16),
                cores: vec![],
                clusters: vec![ClusterId(i as u32)],
            });
        }
        let mut cpus = Node::named("cpus").unwrap();
        set_word(&mut cpus, "#address-cells", 1);
        set_word(&mut cpus, "#size-cells", 0);
        let mut bindings = vec![];
        for (i, name) in ["cpu0", "cpu1", "cpu2"].into_iter().enumerate() {
            let cluster = usize::from(i == 2);
            let kind = inventory.kinds[cluster];
            let reg = if i == 2 { 0x100 } else { i as u32 };
            let id = GuestSlotId(7 + i as u32);
            inventory.slots.push(PeSlot {
                id,
                reg,
                kind: Some(kind.id),
                cluster: ClusterId(cluster as u32),
                cluster_core: if i == 2 { 0 } else { i as u32 },
                interrupt: Some(InterruptId(i as u32)),
                start: Some(StartId(i as u32)),
                power: Some(power(i as u16)),
                cpu_impl: Some(range(i as u64)),
            });
            inventory.interrupts.push(InterruptTarget {
                id: InterruptId(i as u32),
                reg,
            });
            inventory.starts.push(StartResource {
                id: StartId(i as u32),
                reg,
            });
            inventory.powers.push(PowerResource {
                id: power(i as u16),
                cores: vec![id],
                clusters: vec![],
            });
            let c = inventory.clusters[cluster];
            let mut n = Node::named(name).unwrap();
            for (key, val) in [
                ("reg", reg),
                ("cpu-id", 40 + i as u32),
                ("AAPL,phandle", 100 + i as u32),
                ("cluster-id", c.firmware_id),
                ("die-id", c.die),
                ("die-cluster-id", c.die_cluster),
                ("cluster-core-id", if i == 2 { 0 } else { i as u32 }),
                ("l2-cache-size", inventory.caches[cluster].geometry.bytes),
                ("l2-cache-id", 0),
            ] {
                set_word(&mut n, key, val);
            }
            for (key, val) in [
                ("cpu-impl-reg", range(i as u64)),
                ("acc-impl-reg", c.acc_impl.unwrap()),
                ("cpm-impl-reg", c.cpm_impl.unwrap()),
            ] {
                set_range(&mut n, key, val);
            }
            for (key, val) in [
                ("device_type", b"cpu\0".as_slice()),
                ("state", WAITING),
                (
                    "compatible",
                    if i == 2 {
                        b"apple,firestorm\0ARM,v8\0".as_slice()
                    } else {
                        b"apple,icestorm\0ARM,v8\0".as_slice()
                    },
                ),
            ] {
                n.set_property(key, val.to_vec()).unwrap();
            }
            n.set_property("cluster-type", vec![kind.cluster_type, 0])
                .unwrap();
            n.set_property(
                "function-enable_core",
                vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
            )
            .unwrap();
            cpus.children.push(n);
            bindings.push(NodeBinding {
                slot: id,
                name,
                midr: kind.midr,
                cache: CacheId(cluster as u32),
                interrupt: InterruptId(i as u32),
                start: StartId(i as u32),
                core_power: power(i as u16),
                cluster_power: c.power.unwrap(),
            });
        }
        let mut root = Node::named("device-tree").unwrap();
        root.children.push(cpus);
        let host = Roster {
            cores: vec![
                AssignedCore {
                    worker: 80,
                    package: 0,
                    core: 4,
                    apic: 20,
                    started: true,
                },
                AssignedCore {
                    worker: 90,
                    package: 0,
                    core: 5,
                    apic: 22,
                    started: true,
                },
            ],
            revision: Cell::new(1),
        };
        let assignments = vec![
            Assignment {
                worker: 90,
                slot: GuestSlotId(9),
                boot: false,
            },
            Assignment {
                worker: 80,
                slot: GuestSlotId(7),
                boot: true,
            },
        ];
        let counts = vec![
            CountField {
                path: CPUS,
                property: "max_cpus",
                count: Count::Pes,
            },
            CountField {
                path: CPUS,
                property: "e-core-count",
                count: Count::CoreType(b'E'),
            },
            CountField {
                path: CPUS,
                property: "p-core-count",
                count: Count::CoreType(b'P'),
            },
        ];
        Self {
            tree: DeviceTree { root },
            inventory,
            host,
            assignments,
            bindings,
            counts,
        }
    }
    fn source(&self) -> SourceBindings<'_> {
        SourceBindings {
            source_set_sha256: [1; 32],
            tree_sha256: digest(&self.tree),
            nodes: &self.bindings,
            counts: &self.counts,
            references: &[],
            cpu_ids: CpuIdRecipe::Preserve,
            harvesting: None,
        }
    }
    fn plan(&self) -> Plan<'_> {
        Plan::build(&self.host, &self.inventory, &self.assignments).unwrap()
    }
}
#[test]
fn enumeration_roundtrips_ids_and_recipes() {
    let f = Fixture::new();
    let plan = f.plan();
    let mut tree = f.tree.clone();
    let projection = Projection::prepare(&plan, &tree, &f.source()).unwrap();
    assert_eq!(projection.receipt().plan, plan.identity());
    let receipt = projection.apply(&mut tree).unwrap();
    assert_eq!(receipt.boot_path, "/cpus/cpu0");
    assert_eq!(receipt.removed, vec!["/cpus/cpu1"]);
    let cpus = tree.node(CPUS).unwrap();
    assert_eq!(
        cpus.children.iter().map(Node::name).collect::<Vec<_>>(),
        vec![Some("cpu2"), Some("cpu0")]
    );
    assert_eq!(word(cpus, "max_cpus").unwrap(), 2);
    assert_eq!(word(cpus, "e-core-count").unwrap(), 1);
    assert_eq!(word(cpus, "p-core-count").unwrap(), 1);
    for (index, node) in cpus.children.iter().enumerate() {
        let original = f
            .tree
            .node(&format!("/cpus/{}", node.name().unwrap()))
            .unwrap();
        for property in &original.properties {
            if property.name() != "state" {
                assert_eq!(node.property(property.name()), Some(property));
            }
        }
        assert_eq!(
            bytes(node, "state").unwrap(),
            if index == 1 { RUNNING } else { WAITING }
        );
    }
    assert_eq!(word(&cpus.children[0], "cpu-id").unwrap(), 42);
    assert_eq!(word(&cpus.children[0], "l2-cache-id").unwrap(), 0);
    assert_eq!(digest(&tree), receipt.projected_sha256);
    assert_eq!(DeviceTree::parse(&tree.serialize()).unwrap(), tree);
}
#[test]
fn bad_late_resource_and_unbound_extra_slot_leave_tree_unchanged() {
    let mut f = Fixture::new();
    f.inventory.slots[0].cpu_impl = Some(range(99));
    let plan = f.plan();
    let tree = f.tree.clone();
    assert!(
        Projection::prepare(&plan, &tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("cpu-impl-reg")
    );
    assert_eq!(tree, f.tree);
    let f = Fixture::new();
    let plan = f.plan();
    let mut source = f.source();
    source.nodes = &f.bindings[..2];
    assert!(Projection::prepare(&plan, &tree, &source).is_err());
}
#[test]
fn removed_source_cpu_needs_no_native_resource_ids_but_kept_cpu_does() {
    let f = Fixture::new();
    let plan = f.plan();
    let bindings = [f.bindings[0], f.bindings[2]];
    let mut source = f.source();
    source.nodes = &bindings;
    let mut tree = f.tree.clone();
    let receipt = Projection::prepare(&plan, &tree, &source)
        .unwrap()
        .apply(&mut tree)
        .unwrap();
    assert_eq!(receipt.removed, vec!["/cpus/cpu1"]);
    assert_eq!(f.tree.node(CPUS).unwrap().children.len(), 3);
    for selected in &tree.node(CPUS).unwrap().children {
        let original = f
            .tree
            .node(CPUS)
            .unwrap()
            .child(selected.name().unwrap())
            .unwrap();
        for property in &original.properties {
            if property.name() != "state" {
                assert_eq!(selected.property(property.name()), Some(property));
            }
        }
    }
    source.nodes = &bindings[..1];
    let unchanged = f.tree.clone();
    assert!(
        Projection::prepare(&plan, &unchanged, &source)
            .unwrap_err()
            .to_string()
            .contains("missing declared source CPU slot")
    );
    assert_eq!(unchanged, f.tree);
}
#[test]
fn unbound_removed_source_shape_identity_and_affinity_are_still_atomic_requirements() {
    for field in ["state", "AAPL,phandle", "cpu-id", "reg", "cpu-impl-reg"] {
        let mut f = Fixture::new();
        let value = match field {
            "state" => b"disabled\0".to_vec(),
            "AAPL,phandle" => 100u32.to_le_bytes().to_vec(),
            "cpu-id" => 40u32.to_le_bytes().to_vec(),
            "reg" => 0u32.to_le_bytes().to_vec(),
            _ => vec![1; 15],
        };
        f.tree.set_property("/cpus/cpu1", field, value).unwrap();
        let plan = f.plan();
        let bindings = [f.bindings[0], f.bindings[2]];
        let mut source = f.source();
        source.nodes = &bindings;
        let original = f.tree.clone();
        assert!(
            Projection::prepare(&plan, &original, &source).is_err(),
            "{field}"
        );
        assert_eq!(original, f.tree);
    }
}
#[test]
fn stale_source_and_roster_fail_before_the_atomic_replacement() {
    let f = Fixture::new();
    let plan = f.plan();
    let mut tree = f.tree.clone();
    let projection = Projection::prepare(&plan, &tree, &f.source()).unwrap();
    tree.set_property(CPUS, "other", vec![7]).unwrap();
    let changed = tree.clone();
    assert!(projection.apply(&mut tree).is_err());
    assert_eq!(tree, changed);
    let mut tree = f.tree.clone();
    let projection = Projection::prepare(&plan, &tree, &f.source()).unwrap();
    f.host.revision.set(2);
    assert!(projection.apply(&mut tree).is_err());
    assert_eq!(tree, f.tree);
}
#[test]
fn identity_bindings_counts_and_unknown_state_cannot_be_silently_accepted() {
    let mut f = Fixture::new();
    f.bindings[0].midr = 0;
    let plan = f.plan();
    assert!(Projection::prepare(&plan, &f.tree, &f.source()).is_err());
    let f = Fixture::new();
    let plan = f.plan();
    let mut source = f.source();
    source.tree_sha256 = [0; 32];
    assert!(Projection::prepare(&plan, &f.tree, &source).is_err());
    let mut source = f.source();
    source.source_set_sha256 = [2; 32];
    assert!(Projection::prepare(&plan, &f.tree, &source).is_err());
    let mut f = Fixture::new();
    f.tree
        .set_property(CPUS, "cpu-mask", vec![3, 0, 0, 0])
        .unwrap();
    let plan = f.plan();
    assert!(
        Projection::prepare(&plan, &f.tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("unqualified CPU consumer")
    );
    let mut f = Fixture::new();
    f.tree
        .set_property("/cpus/cpu1", "state", b"disabled\0".to_vec())
        .unwrap();
    let plan = f.plan();
    assert!(
        Projection::prepare(&plan, &f.tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("unqualified CPU state")
    );
}
#[test]
fn removed_external_references_refuse_instead_of_guessing_an_id_or_mask_rewrite() {
    let mut f = Fixture::new();
    f.tree
        .set_property("/", "cpu-refs", 100u32.to_le_bytes().to_vec())
        .unwrap();
    let plan = f.plan();
    let reference = [ReferenceField {
        path: "/",
        property: "cpu-refs",
        encoding: ReferenceEncoding::Phandles,
    }];
    let mut source = f.source();
    source.references = &reference;
    assert!(Projection::prepare(&plan, &f.tree, &source).is_ok());
    let mut f = Fixture::new();
    f.tree
        .set_property("/", "cpu-refs", 101u32.to_le_bytes().to_vec())
        .unwrap();
    let plan = f.plan();
    let mut source = f.source();
    source.references = &reference;
    assert!(
        Projection::prepare(&plan, &f.tree, &source)
            .unwrap_err()
            .to_string()
            .contains("removed or unknown")
    );
    assert!(f.tree.node("/cpus/cpu1").is_some());
    for (id, accepted) in [(42u32, true), (41, false), (0, false)] {
        let mut f = Fixture::new();
        f.tree
            .set_property("/", "firmware-cpu-refs", id.to_le_bytes().to_vec())
            .unwrap();
        let plan = f.plan();
        let fields = [ReferenceField {
            path: "/",
            property: "firmware-cpu-refs",
            encoding: ReferenceEncoding::FirmwareCpuIds,
        }];
        let mut source = f.source();
        source.references = &fields;
        assert_eq!(
            Projection::prepare(&plan, &f.tree, &source).is_ok(),
            accepted
        );
    }
}

#[test]
fn noncontiguous_clusters_and_unqualified_status_or_cache_consumers_refuse() {
    let mut f = Fixture::new();
    f.host.cores.push(AssignedCore {
        worker: 100,
        package: 0,
        core: 6,
        apic: 24,
        started: true,
    });
    f.assignments = vec![
        Assignment {
            worker: 80,
            slot: GuestSlotId(7),
            boot: true,
        },
        Assignment {
            worker: 90,
            slot: GuestSlotId(9),
            boot: false,
        },
        Assignment {
            worker: 100,
            slot: GuestSlotId(8),
            boot: false,
        },
    ];
    let plan = f.plan();
    assert!(
        Projection::prepare(&plan, &f.tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("noncontiguous")
    );
    for (path, property, value) in [
        ("/cpus/cpu1", "status", b"okay\0".to_vec()),
        (
            "/cpus/cpu1",
            "l3-cache-size",
            0x10000u32.to_le_bytes().to_vec(),
        ),
        (CPUS, "cpu-cluster-count", 3u32.to_le_bytes().to_vec()),
    ] {
        let mut f = Fixture::new();
        f.tree.set_property(path, property, value).unwrap();
        let plan = f.plan();
        assert!(Projection::prepare(&plan, &f.tree, &f.source()).is_err());
    }
    let mut f = Fixture::new();
    f.counts.push(CountField {
        path: "cpus/cpu0",
        property: "reg",
        count: Count::Pes,
    });
    let plan = f.plan();
    assert!(
        Projection::prepare(&plan, &f.tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("noncanonical")
    );
    let mut f = Fixture::new();
    f.counts.push(CountField {
        path: CPUS,
        property: "cpu-cluster-count",
        count: Count::Clusters,
    });
    let plan = f.plan();
    let mut tree = f.tree.clone();
    Projection::prepare(&plan, &tree, &f.source())
        .unwrap()
        .apply(&mut tree)
        .unwrap();
    // A mechanical synthetic count test, not validation of the Apple
    // fixed-count producer's reduced-cluster consumers.
    assert_eq!(
        word(tree.node(CPUS).unwrap(), "cpu-cluster-count").unwrap(),
        2
    );
}

#[test]
fn interrupt_provider_cpu_count_requires_an_explicit_assigned_plan_binding() {
    let mut f = Fixture::new();
    f.tree.root.children.push(Node::named("aic").unwrap());
    f.tree
        .set_property("/aic", "#main-cpus", 10u32.to_le_bytes().to_vec())
        .unwrap();
    let plan = f.plan();
    assert!(
        Projection::prepare(&plan, &f.tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("#main-cpus")
    );
    f.counts.push(CountField {
        path: "/aic",
        property: "#main-cpus",
        count: Count::Pes,
    });
    let plan = f.plan();
    let mut tree = f.tree.clone();
    Projection::prepare(&plan, &tree, &f.source())
        .unwrap()
        .apply(&mut tree)
        .unwrap();
    assert_eq!(
        word(tree.node("/aic").unwrap(), "#main-cpus").unwrap(),
        plan.pes().len() as u32
    );
}

#[test]
fn t8112_projection_preserves_absent_coordinates_and_checks_affinity_derived_bindings() {
    let mut f = Fixture::new();
    f.inventory.soc = 0x8112;
    for (index, cluster) in f.inventory.clusters.iter_mut().enumerate() {
        cluster.firmware_id = index as u32;
    }
    for node in &mut f.tree.node_mut(CPUS).unwrap().children {
        let reg = word(node, "reg").unwrap();
        set_word(node, "cluster-id", reg >> 8);
        for key in ["die-id", "die-cluster-id", "cluster-core-id"] {
            node.remove_property(key);
        }
    }
    let plan = f.plan();
    let mut projected = f.tree.clone();
    Projection::prepare(&plan, &projected, &f.source())
        .unwrap()
        .apply(&mut projected)
        .unwrap();
    for node in &projected.node(CPUS).unwrap().children {
        for key in ["die-id", "die-cluster-id", "cluster-core-id"] {
            assert!(node.property(key).is_none());
        }
    }
    let mut wrong = f.tree.clone();
    set_word(wrong.node_mut("/cpus/cpu2").unwrap(), "die-cluster-id", 0);
    let mut source = f.source();
    source.tree_sha256 = digest(&wrong);
    assert!(Projection::prepare(&plan, &wrong, &source).is_err());
    drop(plan);
    f.inventory.soc = 0x6000;
    assert!(Projection::prepare(&f.plan(), &f.tree, &f.source()).is_err());
}

fn dense_fixture() -> Fixture {
    let mut f = Fixture::new();
    f.inventory.soc = 0x8112;
    for (index, cluster) in f.inventory.clusters.iter_mut().enumerate() {
        cluster.firmware_id = index as u32;
    }
    let mut table = Vec::new();
    for (index, node) in f
        .tree
        .node_mut(CPUS)
        .unwrap()
        .children
        .iter_mut()
        .enumerate()
    {
        let reg = word(node, "reg").unwrap();
        set_word(node, "cluster-id", reg >> 8);
        set_word(node, "cpu-id", index as u32);
        // The recipe's families give each CPU its own cpu-id bit.
        let mut enable = vec![1, 2, 3, 4, 5, 6, 7, 8];
        enable.extend((1u32 << index).to_le_bytes());
        node.set_property("function-enable_core", enable).unwrap();
        let region = range(100 + index as u64);
        set_range(node, "coresight-reg", region);
        table.extend((region.base + 4).to_le_bytes());
        table.extend(0xfu32.to_le_bytes());
        table.extend(1u32.to_le_bytes());
    }
    let mut wdt = Node::named("watchdog").unwrap();
    wdt.set_property("compatible", b"wdt,s5l8960x\0".to_vec())
        .unwrap();
    set_word(&mut wdt, "AAPL,phandle", 900);
    set_range(&mut wdt, "reg", range(200));
    wdt.set_property("trigger-config", table).unwrap();
    f.tree.root.children.push(wdt);
    f
}

#[test]
fn dense_cpu_ids_and_watchdog_records_follow_one_plan_without_changing_physical_recipes() {
    let f = dense_fixture();
    let original = f.tree.clone();
    let plan = f.plan();
    let mut source = f.source();
    source.cpu_ids = CpuIdRecipe::DenseWithWatchdog {
        original: &original,
        original_sha256: digest(&original),
        watchdog_path: "/watchdog",
    };
    let mut tree = f.tree.clone();
    let receipt = Projection::prepare(&plan, &tree, &source)
        .unwrap()
        .apply(&mut tree)
        .unwrap();
    assert_eq!(word(tree.node(CPUS).unwrap(), "max_cpus").unwrap(), 2);
    assert_eq!(
        receipt
            .cpu_ids
            .iter()
            .map(|m| (m.original_id, m.projected_id))
            .collect::<Vec<_>>(),
        [(2, 0), (0, 1)]
    );
    let old_table = bytes(original.node("/watchdog").unwrap(), "trigger-config").unwrap();
    assert_eq!(
        bytes(tree.node("/watchdog").unwrap(), "trigger-config").unwrap(),
        [&old_table[32..48], &old_table[..16]].concat()
    );
    for mapping in &receipt.cpu_ids {
        let before = original.node(&mapping.path).unwrap();
        let after = tree.node(&mapping.path).unwrap();
        assert_eq!(word(after, "cpu-id").unwrap(), mapping.projected_id);
        // The enable_core mask moves with the ID; its provider stays.
        let enable = bytes(after, "function-enable_core").unwrap();
        assert_eq!(
            enable[..8],
            bytes(before, "function-enable_core").unwrap()[..8]
        );
        assert_eq!(enable[8..], (1u32 << mapping.projected_id).to_le_bytes());
        for property in &before.properties {
            if !matches!(property.name(), "state" | "cpu-id" | "function-enable_core") {
                assert_eq!(after.property(property.name()), Some(property));
            }
        }
    }
    let wdt = receipt.watchdog.unwrap();
    assert_eq!((wdt.original_records, wdt.projected_records), (3, 2));
    assert_eq!(wdt.original_tree_sha256, digest(&original));
    assert_eq!(f.tree, original);
    assert_eq!(DeviceTree::parse(&tree.serialize()).unwrap(), tree);
}

#[test]
fn dense_recipe_refuses_bad_sources_tables_ranges_counts_and_references_atomically() {
    let f = dense_fixture();
    let original = f.tree.clone();
    let plan = f.plan();
    let mut source = f.source();
    source.cpu_ids = CpuIdRecipe::DenseWithWatchdog {
        original: &original,
        original_sha256: digest(&original),
        watchdog_path: "/watchdog",
    };
    let mut changed_inputs = Vec::new();
    for len in [0, 16, 47, 64] {
        let mut tree = f.tree.clone();
        tree.set_property("/watchdog", "trigger-config", vec![0; len])
            .unwrap();
        changed_inputs.push(tree);
    }
    let mut missing = f.tree.clone();
    missing
        .node_mut("/watchdog")
        .unwrap()
        .remove_property("trigger-config");
    changed_inputs.push(missing);
    let mut duplicate = f.tree.clone();
    let p = duplicate
        .property("/watchdog", "trigger-config")
        .unwrap()
        .clone();
    duplicate.node_mut("/watchdog").unwrap().properties.push(p);
    changed_inputs.push(duplicate);
    let mut changed = f.tree.clone();
    set_word(changed.node_mut("/cpus/cpu2").unwrap(), "cpu-id", 99);
    changed_inputs.push(changed);
    let mut changed = f.tree.clone();
    let mut table = bytes(changed.node("/watchdog").unwrap(), "trigger-config")
        .unwrap()
        .to_vec();
    table[32] ^= 4;
    changed
        .set_property("/watchdog", "trigger-config", table)
        .unwrap();
    changed_inputs.push(changed);
    for tree in changed_inputs {
        source.tree_sha256 = digest(&tree);
        let before = tree.clone();
        assert!(Projection::prepare(&plan, &tree, &source).is_err());
        assert_eq!(tree, before);
    }
    for (key, value) in [
        ("cpu-id", 3u32.to_le_bytes().to_vec()),
        ("cpu-id", 0u32.to_le_bytes().to_vec()),
        ("coresight-reg", vec![0; 16]),
    ] {
        let mut invalid = original.clone();
        invalid.set_property("/cpus/cpu2", key, value).unwrap();
        let mut invalid_source = f.source();
        invalid_source.tree_sha256 = digest(&invalid);
        invalid_source.cpu_ids = CpuIdRecipe::DenseWithWatchdog {
            original: &invalid,
            original_sha256: digest(&invalid),
            watchdog_path: "/watchdog",
        };
        assert!(Projection::prepare(&plan, &invalid, &invalid_source).is_err());
    }
    source.tree_sha256 = digest(&f.tree);
    source.cpu_ids = CpuIdRecipe::DenseWithWatchdog {
        original: &original,
        original_sha256: [0; 32],
        watchdog_path: "/watchdog",
    };
    assert!(Projection::prepare(&plan, &f.tree, &source).is_err());
    source.cpu_ids = CpuIdRecipe::DenseWithWatchdog {
        original: &original,
        original_sha256: digest(&original),
        watchdog_path: "/watchdog",
    };
    source.counts = &[];
    assert!(Projection::prepare(&plan, &f.tree, &source).is_err());
    let mut referenced = f.tree.clone();
    referenced
        .set_property("/", "cpu-refs", 2u32.to_le_bytes().to_vec())
        .unwrap();
    source.counts = &f.counts;
    source.tree_sha256 = digest(&referenced);
    let references = [ReferenceField {
        path: "/",
        property: "cpu-refs",
        encoding: ReferenceEncoding::FirmwareCpuIds,
    }];
    source.references = &references;
    assert!(Projection::prepare(&plan, &referenced, &source).is_err());
}

#[test]
#[ignore = "oracle: STELA_TEST_CPU_DT_DIR"]
fn original_j473_first_p_projection_keeps_source_fields_and_absent_coordinates() {
    use stela_soc_data::board::assigned::{BspSource, SourceIdentity};
    let file = std::path::PathBuf::from(std::env::var("STELA_TEST_CPU_DT_DIR").unwrap())
        .join("DeviceTree.j473ap.adt");
    let original = std::fs::read(file).unwrap();
    assert_eq!(
        stela_formats::sha256(&original),
        "a99fa2260f292236fbad210fcee380fc69f101e591125bce347819e84e68034c"
    );
    let mut tree = DeviceTree::parse(&original).unwrap();
    tree.set_property("/chosen", "chip-id", 0x8112u32.to_le_bytes().to_vec())
        .unwrap();
    tree.set_property("/chosen", "board-id", 0x24u32.to_le_bytes().to_vec())
        .unwrap();
    tree.set_property("/cpus/cpu4", "state", RUNNING.to_vec())
        .unwrap();
    let prepared = tree.serialize();
    let owner = BspSource::t8112(
        &original,
        &prepared,
        SourceIdentity {
            original_tree_sha256: Sha256::digest(&original).into(),
            profile: ResourceProfile {
                chip: Chip::AppleM2,
                soc: 0x8112,
                board: 0x24,
                input_set_sha256: [1; 32],
                adapted_set_sha256: None,
            },
        },
        0x100,
        0,
    )
    .unwrap();
    let roster = Roster {
        revision: Cell::new(1),
        cores: vec![AssignedCore {
            worker: 1,
            package: 0,
            core: 0,
            apic: 0,
            started: true,
        }],
    };
    let assignments = [Assignment {
        worker: 1,
        slot: owner.slot(),
        boot: true,
    }];
    let plan = Plan::build(&roster, &owner, &assignments).unwrap();
    let resources = owner.resources();
    let pe = &plan.pes()[0];
    let bindings = [NodeBinding {
        slot: owner.slot(),
        name: owner.name(),
        midr: pe.identity.midr,
        cache: pe.cache,
        interrupt: pe.interrupt,
        start: pe.start,
        core_power: pe.core_power,
        cluster_power: pe.cluster_power,
    }];
    let counts = [
        CountField {
            path: CPUS,
            property: "max_cpus",
            count: Count::Pes,
        },
        CountField {
            path: CPUS,
            property: "e-core-count",
            count: Count::CoreType(b'E'),
        },
        CountField {
            path: CPUS,
            property: "p-core-count",
            count: Count::CoreType(b'P'),
        },
        CountField {
            path: "/arm-io/aic",
            property: "#main-cpus",
            count: Count::Pes,
        },
    ];
    let source = SourceBindings {
        source_set_sha256: resources.profile.input_set_sha256,
        tree_sha256: Sha256::digest(&prepared).into(),
        nodes: &bindings,
        counts: &counts,
        references: &[],
        cpu_ids: CpuIdRecipe::Preserve,
        harvesting: None,
    };
    let receipt = Projection::prepare(&plan, &tree, &source)
        .unwrap()
        .apply(&mut tree)
        .unwrap();
    assert_eq!(receipt.boot_path, "/cpus/cpu4");
    assert_eq!(tree.node(CPUS).unwrap().children.len(), 1);
    let prior = DeviceTree::parse(&prepared).unwrap();
    assert_eq!(tree.node("/cpus/cpu4"), prior.node("/cpus/cpu4"));
    for key in ["die-id", "die-cluster-id", "cluster-core-id"] {
        assert!(tree.property("/cpus/cpu4", key).is_none());
    }
    assert_eq!(owner.source(), original);
    // The separately selected recipe changes only the driver CPU ID and
    // its coordinated per-PE watchdog record, preserving original source.
    let immutable = DeviceTree::parse(&original).unwrap();
    assert_eq!(immutable.serialize(), original);
    let mut dense_source = source;
    dense_source.cpu_ids = CpuIdRecipe::DenseWithWatchdog {
        original: &immutable,
        original_sha256: Sha256::digest(&original).into(),
        watchdog_path: "/arm-io/wdt",
    };
    let mut dense = prior.clone();
    let dense_receipt = Projection::prepare(&plan, &dense, &dense_source)
        .unwrap()
        .apply(&mut dense)
        .unwrap();
    let selected = dense.node("/cpus/cpu4").unwrap();
    assert_eq!(word(selected, "cpu-id").unwrap(), 0);
    assert_eq!(word(selected, "reg").unwrap(), 0x100);
    assert_eq!(word(dense.node(CPUS).unwrap(), "max_cpus").unwrap(), 1);
    let original_table = bytes(immutable.node("/arm-io/wdt").unwrap(), "trigger-config").unwrap();
    assert_eq!(
        stela_formats::sha256(original_table),
        "e9e9875ee26054ce3129dcb658d3d39b819819ef5bc568ad885d15bca048980c"
    );
    assert_eq!(
        bytes(dense.node("/arm-io/wdt").unwrap(), "trigger-config").unwrap(),
        &original_table[64..80]
    );
    assert_eq!(
        dense_receipt
            .cpu_ids
            .iter()
            .map(|m| (m.original_id, m.projected_id))
            .collect::<Vec<_>>(),
        [(4, 0)]
    );
    for property in &prior.node("/cpus/cpu4").unwrap().properties {
        if property.name() != "cpu-id" {
            assert_eq!(selected.property(property.name()), Some(property));
        }
    }
    assert_eq!(owner.source(), original);
}
#[test]
#[ignore = "oracle: STELA_TEST_ASSIGNED_DT"]
fn original_j314s_subsets_roundtrip_without_firmware_id_or_resource_extrapolation() {
    use stela_soc_data::board;
    let path = std::env::var("STELA_TEST_ASSIGNED_DT").unwrap();
    let raw = std::fs::read(path).unwrap();
    let expected = "44fa41bfd0fe0027fd862b8615ae956885614636625d44a59cab226d682558ad";
    assert_eq!(hex::encode(Sha256::digest(&raw)), expected);
    let tree = DeviceTree::parse(&raw).unwrap();
    assert_eq!(tree.serialize(), raw);
    let nodes = &tree.node(CPUS).unwrap().children;
    let processors: Vec<_> = nodes
        .iter()
        .map(|n| board::Processor {
            cluster_type: bytes(n, "cluster-type").unwrap()[0],
            reg: word(n, "reg").unwrap(),
            l2_bytes: Some(word(n, "l2-cache-size").unwrap()),
        })
        .collect();
    let identities = board::cpus(
        0x6000,
        tree.property("/product", "product-soc-name")
            .unwrap()
            .as_str()
            .unwrap(),
        &processors,
    )
    .unwrap()
    .processors;
    let mut inventory = Fixture::new().inventory;
    inventory.kinds.clear();
    inventory.slots.clear();
    inventory.clusters.clear();
    inventory.caches.clear();
    inventory.interrupts.clear();
    inventory.starts.clear();
    inventory.powers.clear();
    let mut bindings = vec![];
    let read_range = |node: &Node, name: &str| {
        let v = bytes(node, name).unwrap();
        assert_eq!(v.len(), 16);
        RegisterRange {
            base: u64::from_le_bytes(v[..8].try_into().unwrap()),
            bytes: u64::from_le_bytes(v[8..].try_into().unwrap()),
        }
    };
    for (i, node) in nodes.iter().enumerate() {
        let kind_letter = processors[i].cluster_type;
        let reg = processors[i].reg;
        let kind = KindId(u32::from(kind_letter));
        let cluster = ClusterId(reg >> 8);
        let identity = identities[i];
        if !inventory.kinds.iter().any(|v| v.id == kind) {
            let (instruction, data) = board::level1_caches(0x6000, kind_letter).unwrap();
            inventory.kinds.push(CoreKind {
                id: kind,
                cluster_type: kind_letter,
                midr: identity.midr,
                aff2: (identity.mpidr >> 16 & 7) as u8,
                clock: identity.clock,
                instruction,
                data,
            });
        }
        if !inventory.clusters.iter().any(|v| v.id == cluster) {
            let ccsidr = identity.caches.unwrap().ccsidr(2);
            let line = 16 << (ccsidr & 7);
            let ways = (ccsidr >> 3 & 0x3ff) + 1;
            let sets = (ccsidr >> 13 & 0x7fff) + 1;
            inventory.caches.push(SharedCache {
                id: CacheId(cluster.0),
                cluster,
                geometry: Cache {
                    bytes: (line * ways * sets) as u32,
                    ways: ways as u32,
                    line: line as u32,
                },
            });
            inventory.clusters.push(ClusterSlot {
                id: cluster,
                aff1: (reg >> 8) as u8,
                firmware_id: word(node, "cluster-id").unwrap(),
                die: word(node, "die-id").unwrap(),
                die_cluster: word(node, "die-cluster-id").unwrap(),
                cache: Some(CacheId(cluster.0)),
                power: Some(power(100 + cluster.0 as u16)),
                acc_impl: Some(read_range(node, "acc-impl-reg")),
                cpm_impl: Some(read_range(node, "cpm-impl-reg")),
            });
            // Test-only association IDs: original PMGR/executor ownership
            // is not manufactured or checked by a DT projection replay.
            inventory.powers.push(PowerResource {
                id: power(100 + cluster.0 as u16),
                cores: vec![],
                clusters: vec![cluster],
            });
        }
        let slot = GuestSlotId(i as u32);
        inventory.slots.push(PeSlot {
            id: slot,
            reg,
            kind: Some(kind),
            cluster,
            cluster_core: word(node, "cluster-core-id").unwrap(),
            interrupt: Some(InterruptId(i as u32)),
            start: Some(StartId(i as u32)),
            power: Some(power(i as u16)),
            cpu_impl: Some(read_range(node, "cpu-impl-reg")),
        });
        inventory.interrupts.push(InterruptTarget {
            id: InterruptId(i as u32),
            reg,
        });
        inventory.starts.push(StartResource {
            id: StartId(i as u32),
            reg,
        });
        inventory.powers.push(PowerResource {
            id: power(i as u16),
            cores: vec![slot],
            clusters: vec![],
        });
        bindings.push(NodeBinding {
            slot,
            name: node.name().unwrap(),
            midr: identity.midr,
            cache: CacheId(cluster.0),
            interrupt: InterruptId(i as u32),
            start: StartId(i as u32),
            core_power: power(i as u16),
            cluster_power: power(100 + cluster.0 as u16),
        });
    }
    let mut counts = Fixture::new().counts;
    // Original AppleInterruptControllerV2 reads this provider CPU count
    // and iterates that many CPU indices (26A434 KC file0x2822dc4 and
    // 0x282375c). It follows the same assigned PE plan as /cpus.
    counts.push(CountField {
        path: "/arm-io/aic",
        property: "#main-cpus",
        count: Count::Pes,
    });
    for selected in [
        vec![0],
        vec![0, 1, 2, 3],
        vec![0, 1, 2, 3, 4, 6, 7, 8],
        (0..10).collect(),
    ] {
        let host = Roster {
            cores: selected
                .iter()
                .enumerate()
                .map(|(i, _)| AssignedCore {
                    worker: 50 + i,
                    package: 0,
                    core: i as u32,
                    apic: i as u32 * 2,
                    started: true,
                })
                .collect(),
            revision: Cell::new(1),
        };
        let assignments: Vec<_> = selected
            .iter()
            .enumerate()
            .map(|(i, slot)| Assignment {
                worker: 50 + i,
                slot: GuestSlotId(*slot),
                boot: i == 0,
            })
            .collect();
        let plan = Plan::build(&host, &inventory, &assignments).unwrap();
        let source = SourceBindings {
            source_set_sha256: [1; 32],
            tree_sha256: Sha256::digest(&raw).into(),
            nodes: &bindings,
            counts: &counts,
            references: &[],
            cpu_ids: CpuIdRecipe::Preserve,
            harvesting: None,
        };
        let mut projected = tree.clone();
        let receipt = Projection::prepare(&plan, &projected, &source)
            .unwrap()
            .apply(&mut projected)
            .unwrap();
        assert_eq!(receipt.plan, plan.identity());
        let cpus = projected.node(CPUS).unwrap();
        assert_eq!(cpus.children.len(), selected.len());
        assert_eq!(word(cpus, "max_cpus").unwrap(), selected.len() as u32);
        assert_eq!(
            DeviceTree::parse(&projected.serialize()).unwrap(),
            projected
        );
        for (i, node) in cpus.children.iter().enumerate() {
            let source = &nodes[selected[i] as usize];
            assert_eq!(
                word(node, "cpu-id").unwrap(),
                word(source, "cpu-id").unwrap()
            );
            assert_eq!(word(node, "reg").unwrap(), word(source, "reg").unwrap());
            for prop in &source.properties {
                if prop.name() != "state" {
                    assert_eq!(node.property(prop.name()), Some(prop));
                }
            }
        }
    }
}

const PMGR: HarvestingField<'static> = HarvestingField {
    path: "/arm-io/pmgr",
    property: "acc-harvesting",
};
/// The fixture with a PMGR and the listed slots, in this order, the first
/// booting, on as many host workers.
fn harvesting_fixture(slots: &[u32], mode: Option<&[u8]>) -> Fixture {
    let mut f = Fixture::new();
    let mut io = Node::named("arm-io").unwrap();
    let mut pmgr = Node::named("pmgr").unwrap();
    if let Some(mode) = mode {
        pmgr.set_property("acc-harvesting", mode.to_vec()).unwrap();
    }
    io.children.push(pmgr);
    f.tree.root.children.push(io);
    f.host.cores = (0..slots.len())
        .map(|i| AssignedCore {
            worker: 80 + i * 10,
            package: 0,
            core: 4 + i as u32,
            apic: 20 + 2 * i as u32,
            started: true,
        })
        .collect();
    f.assignments = slots
        .iter()
        .enumerate()
        .map(|(i, &slot)| Assignment {
            worker: 80 + i * 10,
            slot: GuestSlotId(slot),
            boot: i == 0,
        })
        .collect();
    f
}
fn harvest(f: &Fixture) -> Result<(Receipt, DeviceTree)> {
    let plan = f.plan();
    let mut source = f.source();
    source.harvesting = Some(PMGR);
    let mut tree = f.tree.clone();
    let receipt = Projection::prepare(&plan, &tree, &source)?.apply(&mut tree)?;
    Ok((receipt, tree))
}

#[test]
fn reduced_rosters_declare_the_harvested_pmgr_and_keep_its_complex_shape() {
    // Slots 7, 8 are E cpu0, cpu1 (cluster 10); slot 9 is P cpu2 (cluster 11).
    for slots in [&[7, 8][..], &[7, 9], &[7]] {
        let f = harvesting_fixture(slots, None);
        let (receipt, tree) = harvest(&f).unwrap();
        assert_eq!(receipt.harvesting, Some(HARVESTED), "{slots:?}");
        assert_eq!(
            tree.property(PMGR.path, PMGR.property).unwrap().as_u32(),
            Some(HARVESTED)
        );
        assert!(f.tree.property(PMGR.path, PMGR.property).is_none());
    }
    // An explicit constructor default becomes the harvested mode; mode 1 stays.
    for mode in [0u32, 1] {
        let f = harvesting_fixture(&[7, 8], Some(&mode.to_le_bytes()));
        let (receipt, tree) = harvest(&f).unwrap();
        assert_eq!(receipt.harvesting, Some(HARVESTED));
        assert_eq!(
            tree.property(PMGR.path, PMGR.property).unwrap().as_u32(),
            Some(HARVESTED)
        );
    }
    // A complete roster keeps the source mode, absent or present.
    for mode in [None, Some(&0u32.to_le_bytes()[..])] {
        let f = harvesting_fixture(&[7, 8, 9], mode);
        let (receipt, tree) = harvest(&f).unwrap();
        assert_eq!(receipt.harvesting, None);
        assert_eq!(
            tree.node(PMGR.path),
            f.tree.node(PMGR.path),
            "complete roster changed the PMGR"
        );
    }
}

#[test]
fn reduced_rosters_the_harvested_pmgr_cannot_bind_refuse_atomically() {
    for (slots, reason) in [
        // Kernel cluster 0 would be P while PMGR complex 0 is E.
        (&[9][..], "prefix of the original clusters"),
        (&[8, 9], "first CPUs of each kept cluster"),
        (&[8], "first CPUs of each kept cluster"),
        // The PMGR numbers cores in original record order.
        (&[9, 7], "original CPU order"),
        (&[8, 7], "original CPU order"),
    ] {
        let f = harvesting_fixture(slots, None);
        let error = harvest(&f).unwrap_err().to_string();
        assert!(error.contains(reason), "{slots:?}: {error}");
    }
    for mode in [
        2u32.to_le_bytes().to_vec(),
        vec![1],
        1u64.to_le_bytes().to_vec(),
    ] {
        let f = harvesting_fixture(&[7, 8], Some(&mode));
        assert!(
            harvest(&f)
                .unwrap_err()
                .to_string()
                .contains("unqualified harvesting mode"),
            "{mode:?}"
        );
    }
    let mut f = harvesting_fixture(&[7, 8], None);
    f.tree
        .root
        .children
        .retain(|node| node.name() != Some("arm-io"));
    assert!(
        harvest(&f)
            .unwrap_err()
            .to_string()
            .contains("needs the PMGR")
    );
    // A source mode the caller does not list is an unchecked consumer.
    let f = harvesting_fixture(&[7, 8, 9], Some(&1u32.to_le_bytes()));
    let plan = f.plan();
    assert!(
        Projection::prepare(&plan, &f.tree, &f.source())
            .unwrap_err()
            .to_string()
            .contains("unqualified CPU consumer")
    );
}
