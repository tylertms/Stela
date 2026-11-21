//! Original T6020-family (M2 Pro, Max and Ultra) normal mode-1 constructor
//! membership, not lifecycle state. The 26B123 mac14j kernelcache (SHA-256
//! below) builds every die's CPU owners from die 0's `/arm-io/pmgr`
//! `devices`, in record order, in `ApplePMGR::initDriver`:
//!
//! - A perf-domain descriptor with type 1 makes the next CPU complex
//!   (0xfffffe0009b594d4..0xfffffe0009b59564), bounded by the kernel's
//!   `ml_get_topology_info` cluster count, so complex `n` is kernel
//!   cluster `n`: each die's complexes in descriptor order, die after die.
//! - A record whose name has a CPU prefix (0xfffffe0009b5b198..
//!   0xfffffe0009b5b228) reaches the mode-1 core writer
//!   (`acc-harvesting` at self+0x238c, 0xfffffe0009b5b548), which gives
//!   it the next XNU CPU number, one counter for every die, while its
//!   complex has fewer cores than its kernel cluster's CPU count
//!   (0xfffffe0009b5b7b0..0xfffffe0009b5b8f4).
//! - A CPM-prefixed record (0xfffffe0009b5b230..0xfffffe0009b5b2b8)
//!   stores its ID, group and index in its complex (0xfffffe0009b5b2bc):
//!   the cluster's owner. J414s has device cluster IDs 1, 2 and 3 in
//!   byte 25 of ECPM, PCPM0 and PCPM1. For records with byte 15 bit 6
//!   clear, `ApplePMGR::_getDeviceClusterID` (0xfffffe0009b63310)
//!   returns that byte. `initDriver` uses it for the separate cluster
//!   power-gating lookup (0xfffffe0009b5b8f8), after writing the owner.
//! - A virtual record (flags bit 4, 0xfffffe0009b5b108) owns no PS and
//!   names its complex's `_deviceIDToCPUComplex` ID, once per complex
//!   (0xfffffe0009b5b34c).
//!
//! The J475D and J180D trees list die 0's records only (`die-stride`
//! places die 1's), with no virtual CPU record and the P CPMs' flags 0x2,
//! which the CPU path does not test. Each core's global CPU number is also
//! its `function-enable_core` bit. No record name suffix or numeric ID
//! alone selects a PE.
use super::*;
use alloc::vec;

/// `board::soc::rows::T6020_LIMITS`: the mac14j kernel's 24 CPUs, six
/// clusters and highest physical cluster 10.
const KERNEL_MAX_CPUS: usize = 24;
const KERNEL_MAX_CLUSTERS: usize = 6;
const KERNEL_MAX_PHYSICAL_CLUSTER: u32 = 10;
const AFFINITY_PART_MASK: u32 = 0xff;
const CLUSTER_SHIFT: u32 = 8;
/// A die's Aff1 values start at `die << DIE_SHIFT` (J475D die 1 is 8).
const DIE_SHIFT: u32 = 3;
const ENABLE_CORE_SELECTOR: u32 = 0x436f_7265;
const HARVESTING: u32 = 1;
/// The PMGR families of the T6020-family dies: the Pro's, and the Max's,
/// which the Ultra's two dies use.
const COMPATIBLES: [&[u8]; 2] = [b"pmgr1,t6020\0", b"pmgr1,t6021\0"];
/// The flags of the CPM records the trees give: none on the E CPM, 0x2 on
/// the P CPMs.
const CPM_FLAGS: [u8; 2] = [0, 0x2];

pub(super) const EVIDENCE: &str = "26B123 mac14j SHA256 2d6520798a32bee6e377a2906d3dd02a788d4ed396ae9c6c5884dbe909d8a47e; mode1 complex 0xfffffe0009b594d4, prefixes 0xfffffe0009b5b198, core writer 0xfffffe0009b5b7b0, CPM writer 0xfffffe0009b5b2bc, virtual 0xfffffe0009b5b34c, device cluster ID 0xfffffe0009b63310; die-major record order";

/// Whether this resolver owns the PMGR's family.
pub(super) fn owns(root: &Node<'_>) -> bool {
    child(root, b"arm-io")
        .and_then(|io| child(&io, b"pmgr"))
        .and_then(|pmgr| required(&pmgr, "compatible"))
        .is_ok_and(|compatible| COMPATIBLES.contains(&compatible))
}

pub(super) fn resolve(
    root: &Node<'_>,
    records: &[PmgrCpuRecord<'_>],
    processors: &[Identity],
    harvesting: u32,
) -> Result<Vec<ProcessorAssociation>, Error> {
    let io = child(root, b"arm-io")?;
    let pmgr = child(&io, b"pmgr")?;
    if !COMPATIBLES.contains(&required(&pmgr, "compatible")?)
        || required(&io, "compatible")? != b"arm-io,t6020\0"
    {
        return Err(Error::UnsupportedCompatible);
    }
    if harvesting != HARVESTING {
        return Err(Error::UnsupportedHarvesting(harvesting));
    }
    if word(required(&pmgr, "acc-harvesting")?)? != harvesting {
        return Err(Error::ConflictingHarvesting);
    }
    for node in [root, &io] {
        if word(required(node, "#address-cells")?)? != 2
            || word(required(node, "#size-cells")?)? != 2
        {
            return Err(Error::UnqualifiedGeometry("T6020 bus cells"));
        }
    }
    if property(&pmgr, "ps-groups")?.is_some() {
        return Err(Error::UnqualifiedGeometry("T6020 PMGR v2 groups"));
    }
    let cpus = child(root, b"cpus")?;
    if word(required(&cpus, "#address-cells")?)? != 1 || word(required(&cpus, "#size-cells")?)? != 0
    {
        return Err(Error::Topology("T6020 CPU address cells"));
    }
    let nodes = cpus.children().ok_or(Error::Malformed("CPU children"))?;
    if nodes.is_empty() || nodes.len() != processors.len() || nodes.len() > KERNEL_MAX_CPUS {
        return Err(Error::Topology("T6020 complete original CPU envelope"));
    }
    let pmgr_provider = word(required(&pmgr, "AAPL,phandle")?)?;
    // Complexes in source order with their dies; each die's are contiguous
    // and every complex's cores contiguous, in Aff0 order.
    let mut members: Vec<Vec<usize>> = Vec::new();
    let mut complex_dies: Vec<u32> = Vec::new();
    let mut complex_aff1: Vec<u32> = Vec::new();
    for (pe, (node, identity)) in nodes.iter().zip(processors).enumerate() {
        let reg = word(required(node, "reg")?)?;
        let aff1 = reg >> CLUSTER_SHIFT;
        let die = word(required(node, "die-id")?)?;
        if reg & !crate::board::mpidr::CLUSTER_CORE != 0
            || aff1 > KERNEL_MAX_PHYSICAL_CLUSTER
            || aff1 >> DIE_SHIFT != die
        {
            return Err(Error::Topology("T6020 physical affinity"));
        }
        if complex_aff1.last() != Some(&aff1) {
            if complex_aff1.contains(&aff1) || complex_dies.last().is_some_and(|&last| last > die) {
                return Err(Error::Topology("T6020 reordered source cluster"));
            }
            members.push(Vec::new());
            complex_dies.push(die);
            complex_aff1.push(aff1);
        }
        let cluster = members.last_mut().expect("complex pushed above");
        if (reg & AFFINITY_PART_MASK) as usize != cluster.len() {
            return Err(Error::Topology("T6020 cluster/core source order"));
        }
        let kind = required(node, "cluster-type")?;
        if let Some(&first) = cluster.first()
            && required(&nodes[first], "cluster-type")? != kind
        {
            return Err(Error::Topology("T6020 mixed core kinds in a complex"));
        }
        let aff2 = match kind {
            b"E\0" => 0,
            b"P\0" => 1,
            _ => return Err(Error::Topology("T6020 CPU kind")),
        };
        if word(required(node, "cpu-id")?)? as usize != pe
            || word(required(node, "cluster-id")?)? != aff1
            || word(required(node, "die-cluster-id")?)? != aff1 & ((1 << DIE_SHIFT) - 1)
            || word(required(node, "cluster-core-id")?)? != reg & AFFINITY_PART_MASK
            || crate::board::mpidr::compose(aff2, reg) != Some(identity.mpidr)
        {
            return Err(Error::Topology("T6020 source/profile coordinate mismatch"));
        }
        let enable = required(node, "function-enable_core")?;
        if enable.len() != 12
            || word(&enable[..4])? != pmgr_provider
            || word(&enable[4..8])? != ENABLE_CORE_SELECTOR
            || word(&enable[8..])? != 1u32 << pe
        {
            return Err(Error::Topology("T6020 original enable_core global ordinal"));
        }
        cluster.push(pe);
    }
    if members.len() > KERNEL_MAX_CLUSTERS {
        return Err(Error::Topology("T6020 kernel cluster capacity"));
    }
    let dies = complex_dies.last().map_or(0, |&die| die as usize + 1);
    if (0..dies as u32).any(|die| !complex_dies.contains(&die)) {
        return Err(Error::Topology("T6020 die without CPUs"));
    }
    let stride = if dies > 1 {
        match required(&pmgr, "die-stride")? {
            bytes if bytes.len() == 8 => u64::from_le_bytes(bytes.try_into().unwrap()),
            _ => return Err(Error::Malformed("die-stride")),
        }
    } else {
        0
    };

    let descriptors = required(&pmgr, "perf-domains")?;
    if descriptors.is_empty() || !descriptors.len().is_multiple_of(PERF_DOMAIN_BYTES) {
        return Err(Error::Malformed("T6020 perf descriptors"));
    }
    let mut ids = BTreeSet::new();
    let mut die_complexes = Vec::new();
    for row in descriptors.as_chunks::<PERF_DOMAIN_BYTES>().0 {
        if row[3] == 0 || usize::from(row[3]) >= DOMAIN_LIMIT || !ids.insert(row[3]) {
            return Err(Error::Malformed("T6020 perf-domain lookup"));
        }
        if row[2] == 1 {
            die_complexes.push(row[3]);
        }
    }
    // Every die repeats die 0's complexes: complex `die * n + k` is die
    // `die`'s copy of descriptor `k`.
    if die_complexes.len() * dies != members.len()
        || complex_dies
            .iter()
            .enumerate()
            .any(|(complex, &die)| complex / die_complexes.len() != die as usize)
    {
        return Err(Error::Topology("T6020 perf/kernel cluster order envelope"));
    }
    let raw = required(&pmgr, "devices")?;
    if raw.is_empty() || !raw.len().is_multiple_of(DEVICE_BYTES) {
        return Err(Error::Malformed("T6020 device records"));
    }
    let per_die = raw.len() / DEVICE_BYTES;
    if records.len() != per_die * dies {
        return Err(Error::Malformed("T6020 complete record views"));
    }
    let rows = raw.as_chunks::<DEVICE_BYTES>().0;
    let mut domains = BTreeSet::new();
    let mut physical = BTreeSet::new();
    let mut core_counts = vec![0usize; members.len()];
    let mut cpm = vec![false; members.len()];
    let mut virtuals = vec![false; members.len()];
    let mut global = 0;
    let mut output = Vec::new();
    for (index, view) in records.iter().enumerate() {
        let row = &rows[index % per_die];
        let die = index / per_die;
        let parsed =
            Record::parse(Layout::V1, row).map_err(|record| Error::Malformed(record.what()))?;
        if usize::from(view.id.die) != die {
            return Err(Error::RecordMismatch(index));
        }
        if view.id.device == 0 || !domains.insert(view.id) {
            return Err(Error::DuplicateDomain(view.id));
        }
        if view.id.device != parsed.id
            || view.flags != row[0]
            || view.perf_domain != row[2]
            || view.name != parsed.name
            || view.location.map(|v| (v.group, v.index))
                != parsed.location.map(|v| (v.group, v.index))
        {
            return Err(Error::RecordMismatch(index));
        }
        if view.perf_domain != 0 && !ids.contains(&view.perf_domain) {
            return Err(Error::UnknownPerfDomain(view.id));
        }
        let Some(local) = die_complexes.iter().position(|&id| id == view.perf_domain) else {
            continue;
        };
        let complex = die * die_complexes.len() + local;
        // Byte 25 is the device cluster ID used by power gating. CPU
        // ownership follows the perf domain and constructor order.
        if row[1] != 0 || row[15] != 0 {
            return Err(Error::UnqualifiedGeometry(
                "T6020 CPU record replication or die selector",
            ));
        }
        if row[0] == VIRTUAL {
            if core::mem::replace(&mut virtuals[complex], true) || view.location.is_some() {
                return Err(Error::IncompleteOwners);
            }
            continue;
        }
        let location = view.location.ok_or(Error::RecordMismatch(index))?;
        let placed = ps_address(&io, &pmgr, location.group, location.index)?
            .checked_add(stride * die as u64)
            .ok_or(Error::UnqualifiedGeometry("T6020 die PS placement"))?;
        if location.address != placed {
            return Err(Error::RecordMismatch(index));
        }
        let is_core = CORE_PREFIXES
            .iter()
            .any(|prefix| view.name.starts_with(prefix));
        let target = if is_core {
            if row[0] != 0 {
                return Err(Error::UnqualifiedGeometry("T6020 core record flags"));
            }
            let pe = *members[complex]
                .get(core_counts[complex])
                .ok_or(Error::Topology("T6020 extra constructor core"))?;
            if pe != global {
                return Err(Error::Topology("T6020 core record/global CPU order"));
            }
            core_counts[complex] += 1;
            global += 1;
            ProcessorTarget::Core { pe }
        } else {
            if !CPM_PREFIXES
                .iter()
                .any(|prefix| view.name.starts_with(prefix))
            {
                return Err(Error::UnknownCpuProtocol(view.id));
            }
            if !CPM_FLAGS.contains(&row[0]) || core::mem::replace(&mut cpm[complex], true) {
                return Err(Error::IncompleteOwners);
            }
            ProcessorTarget::Cluster {
                members: members[complex].clone(),
            }
        };
        if !physical.insert(location.address) {
            return Err(Error::ConflictingAddress(location.address));
        }
        output.push(ProcessorAssociation {
            domain: view.id,
            address: location.address,
            target,
            evidence: EVIDENCE,
        });
    }
    if global != processors.len()
        || core_counts.iter().zip(&members).any(|(n, m)| *n != m.len())
        || cpm.iter().any(|v| !*v)
    {
        return Err(Error::IncompleteOwners);
    }
    for view in records {
        if let Some(location) = view.location
            && physical.contains(&location.address)
            && !output.iter().any(|o| o.domain == view.id)
        {
            return Err(Error::ConflictingAddress(location.address));
        }
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;

    const STRIDE: u64 = 0x20_0000_0000;
    const PROVIDER: u32 = 0x99;

    fn words(values: &[u32]) -> Vec<u8> {
        values.iter().flat_map(|v| v.to_le_bytes()).collect()
    }
    fn wide(values: &[u64]) -> Vec<u8> {
        values.iter().flat_map(|v| v.to_le_bytes()).collect()
    }
    fn node(properties: Vec<(&str, Vec<u8>)>, children: &[Vec<u8>]) -> Vec<u8> {
        stela_contracts::adt::encode(
            &properties
                .iter()
                .map(|(k, v)| (*k, v.as_slice()))
                .collect::<Vec<_>>(),
            children,
        )
    }
    fn row(id: u16, perf: u8, flags: u8, index: u8, name: &str) -> [u8; DEVICE_BYTES] {
        let mut row = [0; DEVICE_BYTES];
        row[0] = flags;
        row[2] = perf;
        row[10] = index;
        row[26..28].copy_from_slice(&id.to_le_bytes());
        row[32..32 + name.len()].copy_from_slice(name.as_bytes());
        row
    }

    /// An authored two-die tree, smaller than J475D: each die has an E and
    /// a P complex of two cores. `/pmgr` lists die 0's records only, and
    /// `die-stride` places die 1's, as on the original tree.
    fn authored(mutation: u8) -> (Vec<u8>, Vec<Identity>) {
        let mut cpus = Vec::new();
        let mut profiles = Vec::new();
        for pe in 0..8u32 {
            let die = pe / 4;
            let local = (pe / 2) % 2;
            let core = pe % 2;
            let aff1 = die << DIE_SHIFT | local;
            let reg = aff1 << 8 | core;
            let kind = if local == 0 { b'E' } else { b'P' };
            profiles.push(crate::board::Processor {
                cluster_type: kind,
                reg,
                l2_bytes: Some(if kind == b'E' { 4 << 20 } else { 16 << 20 }),
            });
            let enable = if mutation == 6 {
                1 << (pe % 4)
            } else {
                1 << pe
            };
            cpus.push(node(
                vec![
                    ("name", format!("cpu{pe}\0").into_bytes()),
                    ("reg", words(&[reg])),
                    ("cpu-id", words(&[pe])),
                    ("cluster-id", words(&[aff1])),
                    (
                        "die-id",
                        words(&[die ^ u32::from(mutation == 2 && pe == 5)]),
                    ),
                    ("die-cluster-id", words(&[local])),
                    ("cluster-core-id", words(&[core])),
                    ("cluster-type", vec![kind, 0]),
                    (
                        "function-enable_core",
                        words(&[PROVIDER, ENABLE_CORE_SELECTOR, enable]),
                    ),
                ],
                &[],
            ));
        }
        let mut rows = Vec::new();
        for (complex, (perf, kind)) in [(2u8, 'E'), (5, 'P')].into_iter().enumerate() {
            for core in 0..2u8 {
                rows.push(row(
                    0x30 + complex as u16 * 2 + u16::from(core),
                    perf,
                    0,
                    complex as u8 * 2 + core,
                    &format!("{kind}CPU{core}"),
                ));
            }
        }
        let p_flags = if mutation == 5 { 0x4 } else { 0x2 };
        let mut ecpm = row(0x40, 2, 0, 4, "ECPM");
        let mut pcpm = row(0x41, 5, p_flags, 5, "PCPM");
        if mutation == 7 {
            ecpm[25] = 1;
            pcpm[25] = 2;
        }
        rows.extend([ecpm, pcpm]);
        rows.push(row(0x50, 0, 0, 6, "SOC_DPE"));
        let mut descriptors = Vec::new();
        for (kind, id) in [(0u8, 1u8), (1, 2), (1, 5)] {
            let mut descriptor = [0; PERF_DOMAIN_BYTES];
            descriptor[2] = kind;
            descriptor[3] = id;
            descriptors.extend_from_slice(&descriptor);
        }
        let mut pmgr = vec![
            ("name", b"pmgr\0".to_vec()),
            ("compatible", b"pmgr1,t6021\0".to_vec()),
            ("AAPL,phandle", words(&[PROVIDER])),
            ("acc-harvesting", words(&[u32::from(mutation != 1)])),
            ("perf-domains", descriptors),
            ("devices", rows.into_iter().flatten().collect()),
            ("reg", wide(&[0x1000, 0x200])),
            ("ps-regs", words(&[0, 0, 0xffff])),
        ];
        if mutation != 3 {
            pmgr.push(("die-stride", wide(&[STRIDE])));
        }
        let io = node(
            vec![
                ("name", b"arm-io\0".to_vec()),
                ("compatible", b"arm-io,t6020\0".to_vec()),
                ("#address-cells", words(&[2])),
                ("#size-cells", words(&[2])),
                ("ranges", wide(&[0, 0x2_0000_0000, 0x1_0000_0000])),
            ],
            &[node(pmgr, &[])],
        );
        let cpus = node(
            vec![
                ("name", b"cpus\0".to_vec()),
                ("#address-cells", words(&[1])),
                ("#size-cells", words(&[0])),
            ],
            &cpus,
        );
        let bytes = node(
            vec![
                ("#address-cells", words(&[2])),
                ("#size-cells", words(&[2])),
            ],
            &[io, cpus],
        );
        let identities = crate::board::cpus(0x6022, "Apple M2 Ultra", &profiles)
            .unwrap()
            .processors;
        (bytes, identities)
    }

    /// The parent adapter's views: die 0's records once per die, each die's
    /// PS placed `die-stride` further (not placed when `stride` is false).
    fn resolve_authored(
        bytes: &[u8],
        identities: &[Identity],
        stride: bool,
    ) -> Result<Vec<ProcessorAssociation>, Error> {
        let root = Node::root(bytes).unwrap();
        let io = child(&root, b"arm-io").unwrap();
        let pmgr = child(&io, b"pmgr").unwrap();
        let rows = required(&pmgr, "devices")
            .unwrap()
            .as_chunks::<DEVICE_BYTES>()
            .0;
        let views = (0..2u8)
            .flat_map(|die| rows.iter().map(move |row| (die, row)))
            .map(|(die, row)| {
                let parsed = Record::parse(Layout::V1, row).unwrap();
                PmgrCpuRecord {
                    id: DomainId {
                        die,
                        device: parsed.id,
                    },
                    flags: row[0],
                    perf_domain: row[2],
                    name: parsed.name,
                    location: parsed.location.map(|location| PmgrCpuLocation {
                        group: location.group,
                        index: location.index,
                        address: ps_address(&io, &pmgr, location.group, location.index).unwrap()
                            + if stride { STRIDE * u64::from(die) } else { 0 },
                    }),
                }
            })
            .collect::<Vec<_>>();
        resolve_cpu_power(Chip::AppleM2, &root, &views, identities, HARVESTING)
    }

    #[test]
    fn t6020_mode1_counts_cores_over_both_dies_and_places_die_one_by_stride() {
        let (bytes, identities) = authored(0);
        let owners = resolve_authored(&bytes, &identities, true).unwrap();
        assert_eq!(owners.len(), 12);
        for die in 0..2usize {
            let owners = &owners[die * 6..die * 6 + 6];
            let base = 0x2_0000_1000 + STRIDE * die as u64;
            for (local, owner) in owners.iter().enumerate() {
                assert_eq!(owner.domain.die, die as u8);
                assert_eq!(owner.address, base + local as u64 * PS_STRIDE);
                assert_eq!(owner.evidence, EVIDENCE);
            }
            for (core, owner) in owners[..4].iter().enumerate() {
                assert_eq!(owner.target, ProcessorTarget::Core { pe: die * 4 + core });
            }
            for complex in 0..2 {
                let first = die * 4 + complex * 2;
                assert_eq!(
                    owners[4 + complex].target,
                    ProcessorTarget::Cluster {
                        members: vec![first, first + 1]
                    }
                );
            }
        }
        assert!(!owners.iter().any(|o| o.domain.device == 0x50));
    }

    #[test]
    fn t6020_device_cluster_ids_preserve_cpu_owners_and_die_placement() {
        let (bytes, identities) = authored(0);
        let expected = resolve_authored(&bytes, &identities, true).unwrap();
        let (bytes, identities) = authored(7);
        assert_eq!(
            resolve_authored(&bytes, &identities, true).unwrap(),
            expected
        );
    }

    #[test]
    fn t6020_mode1_refuses_unqualified_dies_flags_masks_and_settings() {
        // Mutation 4, die 1's views without the stride, is checked below.
        for mutation in [1, 2, 3, 5, 6] {
            let (bytes, identities) = authored(mutation);
            assert!(
                resolve_authored(&bytes, &identities, true).is_err(),
                "mutation {mutation}"
            );
        }
        let (bytes, identities) = authored(0);
        assert!(resolve_authored(&bytes, &identities, false).is_err());
        let root = Node::root(&bytes).unwrap();
        assert_eq!(
            resolve_cpu_power(Chip::AppleM2, &root, &[], &identities, 0),
            Err(Error::UnsupportedHarvesting(0))
        );
    }
}
