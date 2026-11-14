//! Reduced T8112 rosters against the kernel and ApplePMGR consumers of the
//! projected tree, for every worker count.
//!
//! The model follows the 26B123 mac14g binaries (see the loader's
//! `assigned::harvesting`): XNU's topology parser takes each CPU's
//! `die-cluster-id`/`cluster-core-id` from the node or, absent, from MPIDR
//! Aff1/Aff0 (`ml_get_topology_info` helper at 0xfffffe000bda6da0), and
//! numbers its clusters in discovery order; ApplePMGR builds complexes and
//! numbers core records in the mode `acc-harvesting` selects; and
//! `AppleT8110PMGR::setCPUCoreEnergyAccumulator` checks each numbered core
//! against that topology.
use super::*;
use stela_soc_data::board;

type Properties = Vec<(&'static str, Vec<u8>)>;
fn encode(properties: Properties, children: &[Node]) -> Node {
    let (_, name) = properties.iter().find(|(key, _)| *key == "name").unwrap();
    let mut node = Node::named(core::str::from_utf8(&name[..name.len() - 1]).unwrap()).unwrap();
    for (key, value) in properties {
        if key != "name" {
            node.set_property(key, value).unwrap();
        }
    }
    node.children = children.to_vec();
    node
}
fn words(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}
fn wide(values: &[u64]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}
fn record(id: u16, perf: u8, index: u8, name: &str, virtual_: bool) -> [u8; 48] {
    let mut row = [0; 48];
    row[0] = if virtual_ { 0x10 } else { 0 };
    row[2] = perf;
    row[10] = index;
    row[11] = 5;
    row[26..28].copy_from_slice(&id.to_le_bytes());
    row[32..32 + name.len()].copy_from_slice(name.as_bytes());
    row
}
const PMGR_PHANDLE: u32 = 0x74;
const AIC_PHANDLE: u32 = 0x63;

fn coresight(pe: u32) -> u64 {
    0x2_1001_0000 + u64::from(pe / 4) * 0x100_0000 + u64::from(pe & 3) * 0x10_0000
}

/// A J473-shaped original tree: E cpu0..3 (cluster 0), P cpu4..7 (cluster
/// 1), the PMGR's ECPU0..3, PCPU0..3, ECPM, PCPM records in that order and
/// no `acc-harvesting`, as in the original J473 template.
fn original() -> DeviceTree {
    let mut cpus = Vec::new();
    let mut records = Vec::new();
    let mut triggers = Vec::new();
    for pe in 0..8u32 {
        let cluster = pe / 4;
        let base = 0x2_1000_0000 + u64::from(cluster) * 0x100_0000;
        let p = cluster == 1;
        cpus.push(encode(
            vec![
                ("name", format!("cpu{pe}\0").into_bytes()),
                ("device_type", b"cpu\0".to_vec()),
                (
                    "compatible",
                    if p {
                        b"apple,avalanche\0ARM,v8\0".to_vec()
                    } else {
                        b"apple,blizzard\0ARM,v8\0".to_vec()
                    },
                ),
                ("AAPL,phandle", words(&[0x20 + pe])),
                ("reg", words(&[(cluster << 8) | (pe & 3)])),
                ("cpu-id", words(&[pe])),
                ("cluster-id", words(&[cluster])),
                ("cluster-type", vec![if p { b'P' } else { b'E' }, 0]),
                ("state", b"waiting\0".to_vec()),
                (
                    "l2-cache-size",
                    words(&[if p { 16 << 20 } else { 4 << 20 }]),
                ),
                (
                    "cpu-impl-reg",
                    wide(&[base + 0x50000 + u64::from(pe & 3) * 0x100000, 0x9010]),
                ),
                ("acc-impl-reg", wide(&[base + 0xf00000, 0x40088])),
                ("cpm-impl-reg", wide(&[base + 0xe40000, 0x9010])),
                ("coresight-reg", wide(&[coresight(pe), 0x300c8])),
                (
                    "function-enable_core",
                    words(&[PMGR_PHANDLE, 0x436f7265, 1 << pe]),
                ),
                ("interrupt-parent", words(&[AIC_PHANDLE])),
                ("interrupts", words(&[0x47 + pe])),
            ],
            &[],
        ));
        let name = format!("{}CPU{}", if cluster == 0 { 'E' } else { 'P' }, pe & 3);
        records.push(record(
            pe as u16 + 1,
            if cluster == 0 { 2 } else { 5 },
            pe as u8,
            &name,
            false,
        ));
        triggers.extend(wide(&[coresight(pe) + 0x30000, 0x0d]));
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
    let pmgr = encode(
        vec![
            ("name", b"pmgr\0".to_vec()),
            ("compatible", b"pmgr1,t8112\0".to_vec()),
            ("AAPL,phandle", words(&[PMGR_PHANDLE])),
            ("reg", wide(&[0x3b700000, 0xcc000])),
            ("ps-regs", groups),
            ("perf-domains", perf),
            ("devices", records.into_iter().flatten().collect()),
            ("clusters", words(&[4, 4])),
        ],
        &[],
    );
    let aic = encode(
        vec![
            ("name", b"aic\0".to_vec()),
            ("compatible", b"aic,2\0".to_vec()),
            ("AAPL,phandle", words(&[AIC_PHANDLE])),
            ("#main-cpus", words(&[8])),
        ],
        &[],
    );
    let wdt = encode(
        vec![
            ("name", b"wdt\0".to_vec()),
            ("compatible", b"wdt,s5l8960x\0".to_vec()),
            ("AAPL,phandle", words(&[0x40])),
            ("reg", wide(&[0x2_3d2b_0000, 0x4000])),
            ("trigger-config", triggers),
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
        &[pmgr, aic, wdt],
    );
    let memory_map = encode(vec![("name", b"memory-map\0".to_vec())], &[]);
    let chosen = encode(
        vec![
            ("name", b"chosen\0".to_vec()),
            ("chip-id", words(&[0])),
            ("board-id", words(&[0])),
        ],
        &[memory_map],
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
    DeviceTree {
        root: encode(
            vec![
                ("name", b"device-tree\0".to_vec()),
                ("#address-cells", words(&[2])),
                ("#size-cells", words(&[2])),
            ],
            &[chosen, product, io, cpus],
        ),
    }
}

struct Roster(Vec<AssignedCore>);
impl RetainedRoster for Roster {
    fn revision(&self) -> u64 {
        1
    }
    fn cores(&self) -> &[AssignedCore] {
        &self.0
    }
}

/// The projected tree for `count` workers booting from the CPU at `boot`.
fn project(original: &[u8], boot: u32, count: usize) -> Result<DeviceTree> {
    let mut live = DeviceTree::parse(original).unwrap();
    for (key, value) in [("chip-id", 0x8112u32), ("board-id", 0x24)] {
        live.set_property("/chosen", key, value.to_le_bytes().to_vec())
            .unwrap();
    }
    let mut processors = Vec::new();
    for cpu in &mut live.node_mut("/cpus").unwrap().children {
        let reg = cpu.property("reg").unwrap().as_u32().unwrap();
        cpu.set_property(
            "state",
            if reg == boot {
                b"running\0".to_vec()
            } else {
                b"waiting\0".to_vec()
            },
        )
        .unwrap();
        processors.push(board::Processor {
            reg,
            cluster_type: cpu.property("cluster-type").unwrap().value[0],
            l2_bytes: cpu.property("l2-cache-size").unwrap().as_u32(),
        });
    }
    let actual = board::cpus(0x8112, "Apple M2", &processors).unwrap();
    let profile = CpuProfile {
        name: "J473-shaped synthetic source",
        chip: actual.chip,
        processors: actual.processors,
        counter_frequency: 24_000_000,
        fixed: vec![],
    };
    let identity = SourceIdentity {
        original_tree_sha256: Sha256::digest(original).into(),
        profile: ResourceProfile {
            chip: Chip::AppleM2,
            soc: 0x8112,
            board: 0x24,
            input_set_sha256: [7; 32],
            adapted_set_sha256: None,
        },
    };
    let live = captured_live_tree(live, "-v");
    let source = Source::t8112_assigned(original, &live, identity, boot, count)?;
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
    Ok(source.prepare(&roster, profile)?.tree)
}

fn u32_of(tree: &DeviceTree, path: &str, property: &str) -> Option<u32> {
    tree.property(path, property).and_then(|p| p.as_u32())
}

/// XNU's topology: each CPU's (die-cluster-id, cluster-core-id) and the
/// discovery-ordered clusters' CPU counts.
struct Topology {
    cpus: Vec<(u32, u32)>,
    clusters: Vec<usize>,
}
fn topology(tree: &DeviceTree) -> Topology {
    let mut cpus = Vec::new();
    let mut ids: Vec<u32> = Vec::new();
    let mut clusters = Vec::new();
    for (index, node) in tree.node("/cpus").unwrap().children.iter().enumerate() {
        let word = |name| node.property(name).and_then(|p| p.as_u32());
        let reg = word("reg").unwrap();
        assert_eq!(word("cpu-id"), Some(index as u32), "dense kernel CPU IDs");
        cpus.push((
            word("die-cluster-id").unwrap_or((reg >> 8) & 0xff),
            word("cluster-core-id").unwrap_or(reg & 0xff),
        ));
        let id = word("cluster-id").unwrap();
        match ids.iter().position(|&known| known == id) {
            Some(cluster) => clusters[cluster] += 1,
            None => {
                ids.push(id);
                clusters.push(1);
            }
        }
    }
    Topology { cpus, clusters }
}

/// ApplePMGR's numbered cores (CPU number, complex, index in complex) in
/// `mode`, from the tree's perf domains and records.
fn pmgr_cores(tree: &DeviceTree, mode: u32, topology: &Topology) -> Vec<(usize, usize, u32)> {
    let pmgr = tree.node("/arm-io/pmgr").unwrap();
    let mut complexes = [None; 256];
    let mut created = 0;
    for row in pmgr.property("perf-domains").unwrap().value.chunks(28) {
        if row[2] == 1 {
            if mode == 0 || created < topology.clusters.len() {
                complexes[usize::from(row[3])] = Some(created);
                created += 1;
            }
        } else {
            complexes[usize::from(row[3])] = None;
        }
    }
    let cpu_domains = pmgr
        .property("perf-domains")
        .unwrap()
        .value
        .chunks(28)
        .filter(|row| row[2] == 1)
        .map(|row| row[3])
        .collect::<Vec<_>>();
    let mut cores = vec![0u32; created];
    let mut numbered = Vec::new();
    for row in pmgr.property("devices").unwrap().value.chunks(48) {
        if row[0] & 0x10 != 0 || !cpu_domains.contains(&row[2]) {
            continue;
        }
        let Some(complex) = complexes[usize::from(row[2])] else {
            continue;
        };
        let name = &row[32..];
        if !["CPU", "ECPU", "PCPU", "MCPU"]
            .iter()
            .any(|prefix| name.starts_with(prefix.as_bytes()))
        {
            continue;
        }
        if mode == 1 && cores[complex] as usize >= topology.clusters[complex] {
            continue;
        }
        numbered.push((numbered.len(), complex, cores[complex]));
        cores[complex] += 1;
    }
    numbered
}

/// `setCPUCoreEnergyAccumulator`'s three REQUIREs for every numbered core.
fn accumulators(
    topology: &Topology,
    cores: &[(usize, usize, u32)],
) -> core::result::Result<(), String> {
    for &(core, _, _) in cores {
        if core >= topology.cpus.len() {
            return Err(format!("core {core} >= num_cpus {}", topology.cpus.len()));
        }
        let (cluster, cluster_core) = topology.cpus[core];
        if cluster as usize >= topology.clusters.len() {
            return Err(format!("cluster {cluster} >= num_clusters"));
        }
        if cluster_core as usize >= topology.clusters[cluster as usize] {
            return Err(format!("clusterCore {cluster_core} >= cluster CPUs"));
        }
    }
    Ok(())
}

#[test]
fn every_reduced_t8112_roster_matches_the_kernel_and_pmgr_topology() {
    let original = original().serialize();
    for count in 1..=8usize {
        let tree = project(&original, 0, count).unwrap();
        let topology = topology(&tree);
        let e = count.min(4);
        let p = count - e;
        // The kept CPUs are each cluster's first, E before P, as a prefix.
        assert_eq!(
            tree.node("/cpus")
                .unwrap()
                .children
                .iter()
                .map(|node| node.property("reg").unwrap().as_u32().unwrap())
                .collect::<Vec<_>>(),
            [0, 1, 2, 3, 0x100, 0x101, 0x102, 0x103][..count]
        );
        assert_eq!(
            topology.clusters,
            [e, p].into_iter().filter(|&n| n > 0).collect::<Vec<_>>()
        );
        // Counts agree across /cpus, the AIC and the topology.
        assert_eq!(u32_of(&tree, "/cpus", "max_cpus"), Some(count as u32));
        assert_eq!(u32_of(&tree, "/cpus", "e-core-count"), Some(e as u32));
        assert_eq!(u32_of(&tree, "/cpus", "p-core-count"), Some(p as u32));
        assert_eq!(
            u32_of(&tree, "/arm-io/aic", "#main-cpus"),
            Some(count as u32)
        );
        // The PMGR keeps its physical layout and declares the harvested
        // constructor only for a reduced roster.
        assert_eq!(
            tree.property("/arm-io/pmgr", "clusters").unwrap().value,
            words(&[4, 4])
        );
        let mode = u32_of(&tree, "/arm-io/pmgr", "acc-harvesting");
        assert_eq!(mode, (count < 8).then_some(1), "count {count}");
        let cores = pmgr_cores(&tree, mode.unwrap_or(0), &topology);
        accumulators(&topology, &cores).unwrap_or_else(|e| panic!("count {count}: {e}"));
        // PMGR core n is kernel CPU n, physically: complex n is cluster n
        // and the index in the complex is the core in its cluster.
        assert_eq!(cores.len(), count);
        for (core, complex, index) in cores {
            assert_eq!(
                topology.cpus[core],
                (complex as u32, index),
                "count {count}"
            );
            // enableCPUCores tests the CPU number's bit, and the PMGR's
            // physical widths [4, 4] place that bit on the same core.
            let node = &tree.node("/cpus").unwrap().children[core];
            let enable = &node.property("function-enable_core").unwrap().value;
            assert_eq!(enable[8..], (1u32 << core).to_le_bytes());
            assert_eq!((core / 4, core % 4), (complex, index as usize));
        }
        if count < 8 {
            // The source constructor mode numbers every original record and
            // fails the accumulator check: the reported panic.
            let source_mode = pmgr_cores(&tree, 0, &topology);
            assert_eq!(source_mode.len(), 8);
            assert!(
                accumulators(&topology, &source_mode)
                    .unwrap_err()
                    .contains(">= num_cpus")
            );
        }
    }
}

#[test]
fn a_reduced_roster_without_its_first_cluster_refuses_instead_of_misbinding_the_pmgr() {
    let original = original().serialize();
    // Booting from P cpu4 keeps P before E, or P alone: kernel cluster 0
    // would be P while the PMGR's first complex is E.
    for count in 1..8 {
        let error = project(&original, 0x100, count).unwrap_err().to_string();
        let reason = if count <= 4 {
            "keep a prefix of the original clusters"
        } else {
            "keep the original CPU order"
        };
        assert!(error.contains(reason), "count {count}: {error}");
    }
    let tree = project(&original, 0x100, 8).unwrap();
    assert!(tree.property("/arm-io/pmgr", "acc-harvesting").is_none());
}

/// The Mac Studio profile prunes through the same projection; its PMGR is
/// already in mode 1 and its selection keeps every complex's first cores.
#[test]
#[ignore = "oracle: STELA_TEST_J475D_DIR"]
fn original_j475d_reduced_rosters_keep_the_harvested_pmgr_shape() {
    let dir = std::path::PathBuf::from(std::env::var("STELA_TEST_J475D_DIR").unwrap());
    let original = std::fs::read(dir.join("original-devicetree.adt")).unwrap();
    let prepared = std::fs::read(dir.join("devicetree.adt")).unwrap();
    let source_board = board::board(&prepared).unwrap();
    let boot = source_board.boot_processor.as_ref().unwrap().affinity;
    let profile = CpuProfile {
        name: "J475D source",
        chip: source_board.cpus.chip,
        processors: source_board.cpus.processors.clone(),
        counter_frequency: 24_000_000,
        fixed: vec![],
    };
    let tree = DeviceTree::parse(&prepared).unwrap();
    let identity = SourceIdentity {
        original_tree_sha256: Sha256::digest(&original).into(),
        profile: ResourceProfile {
            chip: Chip::AppleM2,
            soc: u32_of(&tree, "/chosen", "chip-id").unwrap(),
            board: u32_of(&tree, "/chosen", "board-id").unwrap(),
            input_set_sha256: [7; 32],
            adapted_set_sha256: None,
        },
    };
    let live = captured_live_tree(tree, "-v");
    let total = profile.processors.len();
    for count in 6..=total {
        let source = Source::t6020_assigned(&original, &live, identity, boot, count).unwrap();
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
        let prepared = source
            .prepare(&roster, profile.clone())
            .unwrap_or_else(|e| panic!("count {count}: {e}"));
        assert_eq!(
            prepared.receipt.harvesting,
            (count < total).then_some(1),
            "count {count}"
        );
        assert_eq!(
            u32_of(&prepared.tree, "/arm-io/pmgr", "acc-harvesting"),
            Some(1)
        );
        assert_eq!(
            u32_of(&prepared.tree, "/cpus", "max_cpus"),
            Some(count as u32)
        );
    }
}
