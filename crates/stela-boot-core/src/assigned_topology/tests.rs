use super::*;
use alloc::vec;
use core::cell::Cell;
use sha2::Digest;
use stela_crypto::Sha256;
use stela_loader::apple_device_tree::Node;
use {
    stela_contracts::Chip, stela_contracts::caches::Cache, stela_contracts::caches::Caches,
    stela_contracts::identity::Identity, stela_soc_data::board::assigned::CacheId,
    stela_soc_data::board::assigned::Capacity, stela_soc_data::board::assigned::CapacityKind,
    stela_soc_data::board::assigned::ClusterId, stela_soc_data::board::assigned::ClusterSlot,
    stela_soc_data::board::assigned::CoreKind, stela_soc_data::board::assigned::InterruptId,
    stela_soc_data::board::assigned::InterruptTarget, stela_soc_data::board::assigned::KindId,
    stela_soc_data::board::assigned::PeSlot, stela_soc_data::board::assigned::PowerResource,
    stela_soc_data::board::assigned::ResourceProfile, stela_soc_data::board::assigned::Resources,
    stela_soc_data::board::assigned::SharedCache, stela_soc_data::board::assigned::StartId,
    stela_soc_data::board::assigned::StartResource, stela_soc_data::pmgr::coverage::DomainId,
};

mod t8112_pmgr;

fn cpuid_identity(maximum: u32, vendor: [u32; 3]) -> native::Cpuid {
    native::Cpuid {
        leaf: 0,
        eax: maximum,
        ebx: vendor[0],
        edx: vendor[1],
        ecx: vendor[2],
        ..Default::default()
    }
}

#[test]
fn bsp_collector_keeps_maxima_and_zero_extended_apic_identity() {
    use stela_compiler::TargetCpu;
    let mut queries = Vec::new();
    let snapshot = current_identity(|leaf, subleaf| {
        queries.push((leaf, subleaf));
        match (leaf, subleaf) {
            (0, 0) => cpuid_identity(0x1f, TargetCpu::INTEL_VENDOR),
            (0x8000_0000, 0) => native::Cpuid {
                leaf,
                eax: 0x8000_0001,
                ..Default::default()
            },
            (1, 0) => native::Cpuid {
                leaf,
                ebx: 0x7b << 24,
                ..Default::default()
            },
            (0xb, 0) | (0x1f, 0) => native::Cpuid {
                leaf,
                ebx: 2,
                ecx: 1 << 8,
                edx: if leaf == 0x1f { 0 } else { 0x1234 },
                ..Default::default()
            },
            (0x8000_0001, 0) => native::Cpuid {
                leaf,
                ..Default::default()
            },
            _ => panic!("unsupported CPUID query {leaf:#x}/{subleaf}"),
        }
    });
    assert_eq!(snapshot.get(0, 0).unwrap().eax, 0x1f);
    assert_eq!(snapshot.get(0x8000_0000, 0).unwrap().eax, 0x8000_0001);
    assert_eq!(snapshot.apic_id(), Some(0));
    assert_eq!(
        queries,
        vec![
            (0, 0),
            (0x8000_0000, 0),
            (1, 0),
            (0xb, 0),
            (0x1f, 0),
            (0x8000_0001, 0)
        ]
    );
    let mut deficient = snapshot;
    deficient.records[0].leaf = 2;
    assert_eq!(deficient.apic_id(), None); // decoder remains strict
}

#[test]
fn bsp_collector_does_not_probe_beyond_either_observed_maximum() {
    use stela_compiler::TargetCpu;
    for vendor in [TargetCpu::INTEL_VENDOR, TargetCpu::AMD_VENDOR] {
        let mut queries = Vec::new();
        let snapshot = current_identity(|leaf, subleaf| {
            queries.push((leaf, subleaf));
            match (leaf, subleaf) {
                (0, 0) => cpuid_identity(1, vendor),
                (0x8000_0000, 0) => native::Cpuid {
                    leaf,
                    eax: 0x8000_0000,
                    ..Default::default()
                },
                (1, 0) => native::Cpuid {
                    leaf,
                    ebx: 0x7b << 24,
                    ..Default::default()
                },
                _ => panic!("unsupported CPUID query {leaf:#x}/{subleaf}"),
            }
        });
        assert_eq!(snapshot.apic_id(), Some(0x7b));
        assert_eq!(queries, vec![(0, 0), (0x8000_0000, 0), (1, 0)]);
        assert_eq!(snapshot.count, 3);
    }
}

#[test]
fn bsp_collector_keeps_amd_full_width_identity_and_vendor_gate() {
    use stela_compiler::TargetCpu;
    for vendor in [TargetCpu::AMD_VENDOR, TargetCpu::INTEL_VENDOR, [0; 3]] {
        let snapshot = current_identity(|leaf, subleaf| match (leaf, subleaf) {
            (0, 0) => cpuid_identity(1, vendor),
            (0x8000_0000, 0) => native::Cpuid {
                leaf,
                eax: 0x8000_001e,
                ..Default::default()
            },
            (1, 0) => native::Cpuid {
                leaf,
                ebx: 0x7b << 24,
                ..Default::default()
            },
            (0x8000_0001, 0) => native::Cpuid {
                leaf,
                ecx: 1 << 22,
                ..Default::default()
            },
            (0x8000_001e, 0) => native::Cpuid {
                leaf,
                eax: 0x1234_5678,
                ..Default::default()
            },
            _ => panic!("unsupported CPUID query {leaf:#x}/{subleaf}"),
        });
        assert_eq!(
            snapshot.apic_id(),
            Some(if vendor == TargetCpu::AMD_VENDOR {
                0x1234_5678
            } else {
                0x7b
            })
        );
        if vendor == TargetCpu::AMD_VENDOR {
            let mut deficient = snapshot;
            deficient.records[1].leaf = 0x8000_0002;
            assert_eq!(deficient.apic_id(), None); // no guessed legacy fallback
        }
    }
}

struct Inventory {
    revision: Cell<u64>,
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
        self.revision.get()
    }
    fn resources(&self) -> Resources<'_> {
        Resources {
            profile: ResourceProfile {
                chip: Chip::AppleM1,
                soc: 0x6000,
                board: 8,
                input_set_sha256: [2; 32],
                adapted_set_sha256: None,
            },
            register_granule: 0x4000,
            provenance: "synthetic owner; no Apple hardware authority",
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
fn domain(device: u16) -> DomainId {
    DomainId { die: 0, device }
}
fn range(base: u64) -> RegisterRange {
    RegisterRange {
        base,
        bytes: 0x9008,
    }
}
fn capture() -> alloc::boxed::Box<Capture> {
    let mut capture = Capture::boxed(2).unwrap();
    capture.processor_count = 2;
    capture.core_count = 2;
    capture.logical_count = 2;
    capture.enabled_count = 2;
    for index in 0..2 {
        capture.processors[index] = native::Processor {
            processor_number: index as u32 + 7,
            apic_id: 42 + index as u64 * 2,
            flags: native::ENABLED | native::HEALTHY | if index == 0 { native::BSP } else { 0 },
            topology_version: 1,
            location: native::Location {
                package: 3,
                core: index as u32 + 8,
                ..Default::default()
            },
            thread: 0,
        };
        capture.selected[index] = index as u32;
        capture.snapshots[index] = native::Snapshot::default();
        capture.snapshots[index].count = 2;
        capture.snapshots[index].records[0] = native::Cpuid {
            leaf: 1,
            ebx: (42 + index as u32 * 2) << 24,
            ..Default::default()
        };
        capture.snapshots[index].records[1] = native::Cpuid {
            leaf: 0,
            eax: 1,
            ..Default::default()
        };
    }
    capture
}
fn fixture() -> (DeviceTree, Inventory, Vec<NodeBinding<'static>>, CpuProfile) {
    let cache = Cache {
        bytes: 4 << 20,
        ways: 16,
        line: 128,
    };
    let capacities = [
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
        maximum: Some(2),
        provenance: "explicit synthetic capacity",
    })
    .collect();
    let mut inventory = Inventory {
        revision: Cell::new(1),
        capacities,
        kinds: vec![CoreKind {
            id: KindId(8),
            cluster_type: b'E',
            midr: 0x611f0220,
            aff2: 0,
            clock: None,
            instruction: cache,
            data: cache,
        }],
        slots: vec![],
        clusters: vec![ClusterSlot {
            id: ClusterId(3),
            aff1: 0,
            firmware_id: 0,
            die: 0,
            die_cluster: 0,
            cache: Some(CacheId(9)),
            power: Some(domain(100)),
            acc_impl: Some(range(0x210000000)),
            cpm_impl: Some(range(0x210010000)),
        }],
        caches: vec![SharedCache {
            id: CacheId(9),
            cluster: ClusterId(3),
            geometry: cache,
        }],
        interrupts: vec![],
        starts: vec![],
        powers: vec![PowerResource {
            id: domain(100),
            cores: vec![],
            clusters: vec![ClusterId(3)],
        }],
    };
    let mut root = Node::named("device-tree").unwrap();
    let mut cpus = Node::named("cpus").unwrap();
    cpus.set_property("#address-cells", 1u32.to_le_bytes().to_vec())
        .unwrap();
    cpus.set_property("#size-cells", 0u32.to_le_bytes().to_vec())
        .unwrap();
    let mut bindings = vec![];
    let mut identities = vec![];
    for (index, name) in ["cpu-first", "cpu-second"].into_iter().enumerate() {
        let reg = index as u32;
        let slot = GuestSlotId(90 + index as u32 * 17);
        let id = domain(index as u16 + 200);
        let irq = InterruptId(index as u32 + 600);
        let start = StartId(index as u32 + 700);
        let implementation = range(0x210020000 + index as u64 * 0x10000);
        inventory.slots.push(PeSlot {
            id: slot,
            reg,
            kind: Some(KindId(8)),
            cluster: ClusterId(3),
            cluster_core: reg,
            interrupt: Some(irq),
            start: Some(start),
            power: Some(id),
            cpu_impl: Some(implementation),
        });
        inventory.interrupts.push(InterruptTarget { id: irq, reg });
        inventory.starts.push(StartResource { id: start, reg });
        inventory.powers.push(PowerResource {
            id,
            cores: vec![slot],
            clusters: vec![],
        });
        let mut node = Node::named(name).unwrap();
        for (key, value) in [
            ("reg", reg),
            ("cpu-id", reg + 30),
            ("AAPL,phandle", reg + 50),
            ("cluster-id", 0),
            ("die-id", 0),
            ("die-cluster-id", 0),
            ("cluster-core-id", reg),
            ("l2-cache-size", cache.bytes),
            ("interrupt-parent", 0x99),
            ("interrupts", reg + 110),
        ] {
            node.set_property(key, value.to_le_bytes().to_vec())
                .unwrap();
        }
        for (key, value) in [
            ("state", &b"waiting\0"[..]),
            ("device_type", &b"cpu\0"[..]),
            ("compatible", &b"synthetic-cpu\0"[..]),
            ("cluster-type", &b"E\0"[..]),
        ] {
            node.set_property(key, value.to_vec()).unwrap();
        }
        for (name, range) in [
            ("cpu-impl-reg", implementation),
            ("acc-impl-reg", inventory.clusters[0].acc_impl.unwrap()),
            ("cpm-impl-reg", inventory.clusters[0].cpm_impl.unwrap()),
        ] {
            node.set_property(
                name,
                [range.base, range.bytes]
                    .into_iter()
                    .flat_map(u64::to_le_bytes)
                    .collect(),
            )
            .unwrap();
        }
        node.set_property(
            "function-enable_core",
            [0x88u32, 0x436f7265, 1 << index]
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect(),
        )
        .unwrap();
        cpus.children.push(node);
        bindings.push(NodeBinding {
            slot,
            name,
            midr: 0x611f0220,
            cache: CacheId(9),
            interrupt: irq,
            start,
            core_power: id,
            cluster_power: domain(100),
        });
        identities.push(Identity {
            midr: 0x611f0220,
            mpidr: 0x80000000 | u64::from(reg),
            clock: None,
            caches: Some(Caches::split_unified(cache, cache, cache).unwrap()),
        });
    }
    let mut io = Node::named("arm-io").unwrap();
    let mut aic = Node::named("aic").unwrap();
    aic.set_property("#main-cpus", 2u32.to_le_bytes().to_vec())
        .unwrap();
    aic.set_property("AAPL,phandle", 0x99u32.to_le_bytes().to_vec())
        .unwrap();
    let mut pmgr = Node::named("pmgr").unwrap();
    pmgr.set_property("AAPL,phandle", 0x88u32.to_le_bytes().to_vec())
        .unwrap();
    pmgr.set_property(
        "clusters",
        [2u32, 4, 4]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect(),
    )
    .unwrap();
    io.children.extend([aic, pmgr]);
    root.children.extend([cpus, io]);
    (
        DeviceTree { root },
        inventory,
        bindings,
        CpuProfile {
            name: "synthetic source",
            chip: Chip::AppleM1,
            counter_frequency: 24000000,
            fixed: vec![],
            processors: identities,
        },
    )
}
fn source<'a>(tree: &DeviceTree, nodes: &'a [NodeBinding<'a>]) -> SourceBindings<'a> {
    SourceBindings {
        source_set_sha256: [2; 32],
        tree_sha256: Sha256::digest(tree.serialize()).into(),
        nodes,
        counts: &COUNT_FIELDS,
        references: &[],
        cpu_ids: CpuIdRecipe::Preserve,
        harvesting: None,
    }
}
fn captured_live_tree(mut tree: DeviceTree, command: &str) -> LiveTree {
    use crate::bundle::live_tree::Ram;
    use stela_formats::boot_bundle::LoadedRegion;
    use stela_loader::boot_args::{self, BootArgs, Video};
    const ARGS: u64 = 0x1000;
    const TREE: u64 = 0x10000;
    const BYTES: usize = 1 << 20;
    struct Input(Vec<u8>);
    impl Ram for Input {
        fn read(&mut self, pa: u64, bytes: &mut [u8]) -> bool {
            let Some(range) = usize::try_from(pa)
                .ok()
                .and_then(|start| start.checked_add(bytes.len()).map(|end| start..end))
                .and_then(|range| self.0.get(range))
            else {
                return false;
            };
            bytes.copy_from_slice(range);
            true
        }
        fn write(&mut self, _: u64, _: &[u8]) -> bool {
            panic!("capture must not write")
        }
        fn fence(&mut self) {
            panic!("capture must not publish")
        }
    }
    tree.set_property(
        "/chosen/memory-map",
        "DeviceTree",
        [TREE.to_le_bytes(), (BYTES as u64 - TREE).to_le_bytes()].concat(),
    )
    .unwrap();
    let bytes = tree.serialize();
    let args = boot_args::encode(&BootArgs {
        revision: boot_args::REVISION,
        version: boot_args::VERSION,
        virt_base: 0xfffffe0000000000,
        phys_base: 0,
        mem_size: BYTES as u64,
        top_of_kernel_data: BYTES as u64,
        video: Video::default(),
        machine_type: 0,
        device_tree: 0xfffffe0000000000 + TREE,
        device_tree_length: bytes.len() as u32,
        command_line: command.into(),
        boot_flags: 0,
        mem_size_actual: BYTES as u64,
    })
    .unwrap();
    let mut input = Input(vec![0; BYTES]);
    input.0[ARGS as usize..ARGS as usize + args.len()].copy_from_slice(&args);
    input.0[TREE as usize..TREE as usize + bytes.len()].copy_from_slice(&bytes);
    let regions = [
        LoadedRegion {
            name: "DeviceTree".into(),
            guest_pa: TREE,
            bytes: bytes.len() as u64,
            memory_bytes: BYTES as u64 - TREE,
        },
        LoadedRegion {
            name: "BootArgs".into(),
            guest_pa: ARGS,
            bytes: args.len() as u64,
            memory_bytes: 4096,
        },
    ];
    LiveTree::capture(&mut input, &regions, ARGS).unwrap()
}
#[test]
#[ignore = "oracle: STELA_TEST_CPU_DT_DIR"]
fn original_t8112_adapter_projects_dense_driver_ids_for_every_worker_count() {
    use stela_soc_data::board;
    let file = std::path::PathBuf::from(std::env::var("STELA_TEST_CPU_DT_DIR").unwrap())
        .join("DeviceTree.j473ap.adt");
    let original = std::fs::read(file).unwrap();
    assert_eq!(
        stela_formats::sha256(&original),
        "a99fa2260f292236fbad210fcee380fc69f101e591125bce347819e84e68034c"
    );
    let mut tree = DeviceTree::parse(&original).unwrap();
    for (key, value) in [("chip-id", 0x8112u32), ("board-id", 0x24)] {
        tree.set_property("/chosen", key, value.to_le_bytes().to_vec())
            .unwrap();
    }
    for cpu in &mut tree.node_mut("/cpus").unwrap().children {
        let running = cpu.name() == Some("cpu4");
        cpu.set_property(
            "state",
            if running {
                b"running\0".to_vec()
            } else {
                b"waiting\0".to_vec()
            },
        )
        .unwrap();
    }
    let identity = SourceIdentity {
        original_tree_sha256: Sha256::digest(&original).into(),
        profile: ResourceProfile {
            chip: Chip::AppleM2,
            soc: 0x8112,
            board: 0x24,
            input_set_sha256: [7; 32],
            adapted_set_sha256: None,
        },
    };
    let cpus = tree
        .node("/cpus")
        .unwrap()
        .children
        .iter()
        .map(|cpu| board::Processor {
            reg: cpu.property("reg").unwrap().as_u32().unwrap(),
            cluster_type: cpu.property("cluster-type").unwrap().value[0],
            l2_bytes: cpu.property("l2-cache-size").unwrap().as_u32(),
        })
        .collect::<Vec<_>>();
    let actual = board::cpus(0x8112, "Apple M2", &cpus).unwrap();
    let profile = CpuProfile {
        name: "original component fixture",
        chip: actual.chip,
        processors: actual.processors,
        counter_frequency: 24_000_000,
        fixed: vec![],
    };
    let live = captured_live_tree(tree.clone(), "-v");
    let source = Source::t8112(&original, &live, identity, 0x100).unwrap();
    let capture = capture();
    let roster = BspRoster::observed(&capture, 0, 42).unwrap();
    // P cpu4 alone would be kernel cluster 0 while the PMGR's first complex
    // is the E one; the harvested constructor cannot bind that roster.
    assert!(
        source
            .prepare(&roster, profile.clone())
            .err()
            .unwrap()
            .to_string()
            .contains("keep a prefix of the original clusters")
    );
    assert_eq!(source.resources.source(), original);
    // Construction fixture only: exercise the selected-source to
    // loader cache-interval/watchdog/profile projection boundary at every
    // admitted cardinality, booting from the first CPU. This does not create
    // an AP executor.
    let mut first = tree.clone();
    for cpu in &mut first.node_mut("/cpus").unwrap().children {
        let running = cpu.name() == Some("cpu0");
        cpu.set_property(
            "state",
            if running {
                b"running\0".to_vec()
            } else {
                b"waiting\0".to_vec()
            },
        )
        .unwrap();
    }
    let first = captured_live_tree(first, "-v");
    struct Roster(Vec<AssignedCore>);
    impl RetainedRoster for Roster {
        fn revision(&self) -> u64 {
            1
        }
        fn cores(&self) -> &[AssignedCore] {
            &self.0
        }
    }
    // The PMGR enables both CPU complexes by index, so one PE cannot form
    // this guest and every other count keeps a PE of each complex.
    assert!(
        Source::t8112_assigned(&original, &first, identity, 0, 1)
            .err()
            .unwrap()
            .to_string()
            .contains("each of the two CPU complexes")
    );
    for count in 2..=8 {
        let roster = Roster(
            (0..count)
                .map(|worker| AssignedCore {
                    worker,
                    package: 0,
                    core: worker as u32,
                    apic: 42 + worker as u32,
                    started: true,
                })
                .collect(),
        );
        let selected = Source::t8112_assigned(&original, &first, identity, 0, count).unwrap();
        let prepared = selected.prepare(&roster, profile.clone()).unwrap();
        let board = board::board(&prepared.tree.serialize()).unwrap();
        assert_eq!(board.boot_processor.unwrap().cpu, 0);
        assert_eq!(board.cpus.processors, prepared.profile.processors);
        assert_eq!(prepared.worker_bindings().unwrap().len(), count);
        assert_eq!(
            prepared
                .tree
                .node("/cpus")
                .unwrap()
                .children
                .iter()
                .map(|node| node.property("reg").unwrap().as_u32().unwrap())
                .collect::<Vec<_>>(),
            [0, 1, 2, 3][..count.min(5) - 1]
                .iter()
                .chain(&[0x100, 0x101, 0x102, 0x103][..count.saturating_sub(4).max(1)])
                .copied()
                .collect::<Vec<_>>()
        );
        assert_eq!(prepared.receipt.cpu_ids.len(), count);
        assert_eq!(prepared.receipt.harvesting, (count < 8).then_some(1));
    }
    let override_live = captured_live_tree(tree, "-v acc-harvesting=0");
    assert!(
        Source::t8112(&original, &override_live, identity, 0x100)
            .err()
            .unwrap()
            .to_string()
            .contains("boot-argument override")
    );
}
#[test]
#[ignore = "oracle: STELA_TEST_ASSIGNED_DT, STELA_TEST_ASSIGNED_PREPARED_DT"]
fn original_mode1_factory_projects_live_tree_without_public_pmgr_admission() {
    use {stela_contracts::adt, stela_soc_data::board};
    let original = std::fs::read(std::env::var("STELA_TEST_ASSIGNED_DT").unwrap()).unwrap();
    let raw = std::fs::read(std::env::var("STELA_TEST_ASSIGNED_PREPARED_DT").unwrap()).unwrap();
    assert_eq!(
        format!("{:x}", Sha256::digest(&original)),
        "44fa41bfd0fe0027fd862b8615ae956885614636625d44a59cab226d682558ad"
    );
    assert_eq!(
        format!("{:x}", Sha256::digest(&raw)),
        "9acc3e66694a2821356388ee32132064f56aef06c93bcff3d1070721b1111780"
    );
    let identity = SourceIdentity {
        original_tree_sha256: Sha256::digest(&original).into(),
        profile: board::assigned::ResourceProfile {
            chip: Chip::AppleM1,
            soc: 0x6000,
            board: 8,
            input_set_sha256: [3; 32],
            adapted_set_sha256: None,
        },
    };
    // Stand in only for unrelated live runtime patch bytes. Projection
    // happens after those patches and binds THIS tree, not the template.
    let mut live = DeviceTree::parse(&raw).unwrap();
    live.set_property("/chosen", "random-seed", vec![0x31; 64])
        .unwrap();
    let live_bytes = live.serialize();
    let source = Source::t6000(&original, &live_bytes, identity, 0).unwrap();
    let pmgr = source.tree.node("/arm-io/pmgr").unwrap().clone();
    assert!(pmgr.property("compatible").is_none());
    let cpus = source
        .tree
        .node("/cpus")
        .unwrap()
        .children
        .iter()
        .map(|cpu| board::Processor {
            reg: cpu.property("reg").unwrap().as_u32().unwrap(),
            cluster_type: cpu.property("cluster-type").unwrap().value[0],
            l2_bytes: cpu.property("l2-cache-size").unwrap().as_u32(),
        })
        .collect::<Vec<_>>();
    let soc_name = &source
        .tree
        .property("/product", "product-soc-name")
        .unwrap()
        .value;
    let soc_name = core::str::from_utf8(soc_name)
        .unwrap()
        .trim_end_matches('\0');
    let actual = board::cpus(0x6000, soc_name, &cpus).unwrap();
    let profile = CpuProfile {
        name: "source topology regression",
        chip: actual.chip,
        processors: actual.processors,
        counter_frequency: 24_000_000,
        fixed: vec![],
    };
    let capture = capture();
    let roster = BspRoster::observed(&capture, 37, 42).unwrap();
    let prepared = source.prepare(&roster, profile).unwrap();
    assert_eq!(prepared.plan.pes().len(), 1);
    assert_eq!(prepared.profile.processors.len(), 1);
    assert_eq!(prepared.tree.node("/cpus").unwrap().children.len(), 1);
    assert_eq!(prepared.tree.node("/arm-io/pmgr"), Some(&pmgr));
    assert_eq!(
        prepared.tree.property("/chosen", "random-seed"),
        live.property("/chosen", "random-seed")
    );
    for field in &COUNT_FIELDS {
        assert_eq!(
            word(&prepared.tree, field.path, field.property),
            Ok(if field.property == "p-core-count" {
                0
            } else {
                1
            })
        );
    }
    assert_eq!(word(&prepared.tree, "/cpus/cpu0", "AAPL,phandle"), Ok(26));
    let join = prepared.join(0).unwrap();
    assert_eq!(
        (join.worker, join.apic, join.error_interrupt),
        (37, 42, 110)
    );
    assert_eq!(
        (join.enable_core.phandle, join.enable_core.argument),
        (0x99, 1)
    );
    assert_eq!((join.core_power.device, join.cluster_power.device), (1, 11));
    assert!(prepared.join(1).is_err());
    assert!(
        prepared
            .tree
            .property("/cpus", "cpu-cluster-count")
            .is_none()
    );
    assert_eq!(source.resources.graph().domains.len(), 355);
    assert_eq!(source.resources.graph().registers.len(), 196);
    let original_root = adt::Node::root(&original).unwrap();
    assert_eq!(
        original_root.child("chosen").unwrap().property("chip-id"),
        Some(&[0; 4][..])
    );
    let mut changed = live.clone();
    changed
        .set_property("/cpus/cpu0", "interrupts", 111u32.to_le_bytes().to_vec())
        .unwrap();
    assert!(Source::t6000(&original, &changed.serialize(), identity, 0).is_err());
    changed = live.clone();
    changed
        .set_property(
            "/arm-io/pmgr",
            "acc-harvesting",
            2u32.to_le_bytes().to_vec(),
        )
        .unwrap();
    assert!(Source::t6000(&original, &changed.serialize(), identity, 0).is_err());
    changed = live.clone();
    changed
        .set_property("/arm-io/pmgr", "power-gates", 197u32.to_le_bytes().to_vec())
        .unwrap();
    assert!(Source::t6000(&original, &changed.serialize(), identity, 0).is_err());
    changed = live.clone();
    changed
        .set_property("/cpus/cpu1", "AAPL,phandle", 26u32.to_le_bytes().to_vec())
        .unwrap();
    assert!(Source::t6000(&original, &changed.serialize(), identity, 0).is_err());
}
#[test]
#[ignore = "oracle: STELA_TEST_ASSIGNED_DT, STELA_TEST_ASSIGNED_PREPARED_DT"]
fn original_mode1_keeps_discovery_order_with_nonzero_logical_boot_pe() {
    use stela_soc_data::board;
    let original = std::fs::read(std::env::var("STELA_TEST_ASSIGNED_DT").unwrap()).unwrap();
    let raw = std::fs::read(std::env::var("STELA_TEST_ASSIGNED_PREPARED_DT").unwrap()).unwrap();
    let tree = DeviceTree::parse(&raw).unwrap();
    let board = board::board(&raw).unwrap();
    let boot = board.boot_processor.as_ref().unwrap();
    assert_ne!(boot.cpu, 0);
    assert!(
        stela_soc::machine::AppleDevices::native_pmp_system_endpoint_refusal(&board,)
            .unwrap()
            .unwrap()
            .contains("T8112")
    );
    let regs = tree
        .node("/cpus")
        .unwrap()
        .children
        .iter()
        .map(|node| node.property("reg").unwrap().as_u32().unwrap())
        .collect::<Vec<_>>();
    let profile = CpuProfile {
        name: "original component projection fixture; no native AP authority",
        chip: board.cpus.chip,
        processors: board.cpus.processors.clone(),
        counter_frequency: 24_000_000,
        fixed: vec![],
    };
    let identity = SourceIdentity {
        original_tree_sha256: Sha256::digest(&original).into(),
        profile: board::assigned::ResourceProfile {
            chip: profile.chip,
            soc: word(&tree, "/chosen", "chip-id").unwrap(),
            board: word(&tree, "/chosen", "board-id").unwrap(),
            input_set_sha256: Sha256::digest(&raw).into(),
            adapted_set_sha256: None,
        },
    };
    struct Roster(Vec<AssignedCore>);
    impl RetainedRoster for Roster {
        fn revision(&self) -> u64 {
            1
        }
        fn cores(&self) -> &[AssignedCore] {
            &self.0
        }
    }
    // Keep a member of every source complex. Shorter selections need the
    // reviewed constructor skip rule, not an inferred cap.
    let minimum = regs
        .iter()
        .enumerate()
        .filter(|(i, reg)| !regs[..*i].iter().any(|earlier| earlier >> 8 == **reg >> 8))
        .map(|(i, _)| i + 1)
        .max()
        .unwrap();
    let live = captured_live_tree(tree.clone(), "-v");
    for count in minimum..=regs.len() {
        let roster = Roster(
            (0..count)
                .map(|worker| AssignedCore {
                    worker,
                    package: 0,
                    core: worker as u32,
                    apic: worker as u32 + 42,
                    started: true,
                })
                .collect(),
        );
        let selected =
            Source::t6000_assigned(&original, &live, identity, boot.affinity, count).unwrap();
        let prepared = selected.prepare(&roster, profile.clone()).unwrap();
        let projected = board::board(&prepared.tree.serialize()).unwrap();
        assert_eq!(projected.boot_processor.unwrap().cpu, boot.cpu);
        assert_eq!(projected.cpus.processors, prepared.profile.processors);
        assert_eq!(
            prepared.tree.node("/arm-io/pmgr"),
            tree.node("/arm-io/pmgr")
        );
        assert_eq!(
            prepared
                .tree
                .node("/cpus")
                .unwrap()
                .children
                .iter()
                .map(|node| node.property("reg").unwrap().as_u32().unwrap())
                .collect::<Vec<_>>(),
            regs[..count]
        );
        for (logical, &reg) in regs[..count].iter().enumerate() {
            let join = prepared.join(reg).unwrap();
            assert_eq!(join.logical, logical);
            let name = selected
                .resources
                .name_for(selected.resources.resources().slots[logical].id)
                .unwrap();
            let path = format!("/cpus/{name}");
            assert_eq!(
                prepared.tree.property(&path, "cpu-id"),
                tree.property(&path, "cpu-id")
            );
        }
        assert_eq!(prepared.join(boot.affinity).unwrap().worker, 0);
        let bindings = prepared.worker_bindings().unwrap();
        assert_eq!(bindings.iter().filter(|b| b.worker.0 == 0).count(), 1);
        assert_eq!(
            bindings.iter().find(|b| b.worker.0 == 0).unwrap().pe,
            boot.cpu
        );
    }
}
#[test]
fn selected_bsp_only_does_not_start_captured_aps() {
    let capture = capture();
    let roster = BspRoster::observed(&capture, 37, 42).unwrap();
    assert_eq!(roster.cores().len(), 1);
    assert_eq!(roster.source().processor_number, 7);
    assert_eq!(
        roster.cores()[0],
        AssignedCore {
            worker: 37,
            package: 3,
            core: 8,
            apic: 42,
            started: true
        }
    );
    assert!(BspRoster::observed(&capture, 37, 44).is_err());
    let mut altered = self::capture();
    altered.selected[0] = 1;
    assert!(BspRoster::observed(&altered, 37, 42).is_err());
}
#[test]
fn one_plan_projects_the_source_slot_profile_counts_and_owner_joins() {
    let capture = capture();
    let roster = BspRoster::observed(&capture, 37, 42).unwrap();
    let (tree, owner, nodes, profile) = fixture();
    let original = tree.clone();
    let selected = nodes[1].slot;
    let prepared = prepare(
        &roster,
        &owner,
        selected,
        &tree,
        &source(&tree, &nodes),
        profile.clone(),
    )
    .unwrap();
    assert_eq!(tree, original);
    assert_eq!(prepared.plan.pes().len(), 1);
    assert_eq!(prepared.profile.processors, [profile.processors[1]]);
    assert_eq!(prepared.receipt.plan, prepared.plan.identity());
    assert_eq!(prepared.tree.node("/cpus").unwrap().children.len(), 1);
    assert!(prepared.tree.node("/cpus/cpu-first").is_none());
    assert_eq!(word(&prepared.tree, "/cpus/cpu-second", "cpu-id"), Ok(31));
    for field in &COUNT_FIELDS {
        assert_eq!(
            word(&prepared.tree, field.path, field.property),
            Ok(if field.property == "p-core-count" {
                0
            } else {
                1
            })
        );
    }
    assert_eq!(
        prepared.tree.property("/arm-io/pmgr", "clusters"),
        tree.property("/arm-io/pmgr", "clusters")
    );
    assert!(
        prepared
            .tree
            .property("/cpus", "cpu-cluster-count")
            .is_none()
    );
    assert!(prepared.join(0).is_err());
    let join = prepared.join(1).unwrap();
    assert_eq!(join.plan, prepared.plan.identity());
    assert_eq!(join.reg, 1);
    assert_eq!(
        (join.worker, join.apic, join.error_interrupt),
        (37, 42, 111)
    );
    assert_eq!(
        (join.start, join.enable_core.argument, join.core_power),
        (StartId(701), 2, domain(201))
    );
    assert_ne!(join.enable_core.argument, u32::from(join.core_power.device));
    assert_eq!(join.cpu_impl, owner.slots[1].cpu_impl.unwrap());
    owner.revision.set(2);
    assert!(prepared.plan.revalidate().is_err());
    assert!(prepared.join(1).is_err());
}
#[test]
fn missing_owner_capacity_source_counts_or_profile_refuse_before_publication() {
    let capture = capture();
    let roster = BspRoster::observed(&capture, 37, 42).unwrap();
    let (tree, mut owner, nodes, profile) = fixture();
    let original = tree.clone();
    owner.capacities[0].maximum = None;
    assert!(
        prepare(
            &roster,
            &owner,
            nodes[0].slot,
            &tree,
            &source(&tree, &nodes),
            profile.clone()
        )
        .err()
        .unwrap()
        .to_string()
        .contains("MissingCapacity")
    );
    owner.capacities[0].maximum = Some(2);
    owner.starts.clear();
    assert!(
        prepare(
            &roster,
            &owner,
            nodes[0].slot,
            &tree,
            &source(&tree, &nodes),
            profile.clone()
        )
        .is_err()
    );
    assert_eq!(tree, original);
    let (tree, owner, nodes, mut profile) = fixture();
    profile.processors[0].midr ^= 1;
    assert!(
        prepare(
            &roster,
            &owner,
            nodes[0].slot,
            &tree,
            &source(&tree, &nodes),
            profile
        )
        .err()
        .unwrap()
        .to_string()
        .contains("source board profile")
    );
    let (tree, owner, nodes, profile) = fixture();
    let mut s = source(&tree, &nodes);
    s.counts = &COUNT_FIELDS[..3];
    assert!(
        prepare(&roster, &owner, nodes[0].slot, &tree, &s, profile.clone())
            .err()
            .unwrap()
            .to_string()
            .contains("#main-cpus")
    );
    let mut counts = COUNT_FIELDS.to_vec();
    counts.push(CountField {
        path: "/cpus",
        property: "cpu-cluster-count",
        count: Count::Clusters,
    });
    s.counts = &counts;
    assert!(
        prepare(&roster, &owner, nodes[0].slot, &tree, &s, profile)
            .err()
            .unwrap()
            .to_string()
            .contains("cannot add")
    );
}
