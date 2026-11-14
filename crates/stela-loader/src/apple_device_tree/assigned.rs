//! Atomic ADT enumeration projection from one checked assigned-core plan.
//!
//! XNU's CPU parser enumerates every `/cpus` child; `state` identifies the
//! boot PE, it does not disable an unassigned PE. Firmware `cpu-id` and
//! `l2-cache-id` are separate from the kernel's dense IDs and our resource
//! IDs. IDs are preserved by default; an explicitly checked recipe may
//! project CPU IDs and their watchdog records together. Source identities and
//! startup/interrupt/power payloads are preserved in either case.
//! Source coordinates: original 26A434 J314s iBoot SHA-256
//! 20dfa3a5e6d4e4b2844c7558e6d4f8ab74397d11eae63312f00a559cf9612b03,
//! file 0x3ab04..0x3ab7c publishes u32 max_cpus/cpu-cluster-count/E/P
//! counts; 0x3abd4..0x3abf8 publishes running. Original mac13j kernel
//! SHA-256 5109fca5209bdbb89cec29e3241cc42a90418cb737d65e31962b059b9879af90,
//! VA 0xfffffe000bda7168 CPU parser enumerates the children and derives
//! logical IDs/physical cluster/cache topology. Public XNU xnu-12377.121.6
//! machine_routines.c, IOPlatformExpert.cpp and AppleARMSMP.cpp independently
//! explain these interfaces; that distinct release is not the binary's source.
//! The fixed cluster-count producer alone does not check reduced-cluster
//! consumer semantics: callers must supply checked field rules explicitly.
//! A caller that names the PMGR's harvesting property gets the reduced-roster
//! shape and PMGR mode that ApplePMGR's CPU constructor binds (`harvesting`).
//!
//! This module does not admit the supplied source/resource bindings or create
//! additional hardware slots, native workers or initial lifecycle permission.
use super::{DeviceTree, Node};
use crate::Result;
use alloc::{collections::BTreeSet, format, string::String, vec::Vec};
use stela_contracts::{refuse, require};
use stela_crypto::{Sha256, digest::Digest};
use stela_soc_data::board::assigned::{
    CacheId, GuestSlotId, InterruptId, Plan, PlanIdentity, StartId,
};
use stela_soc_data::pmgr::coverage::DomainId;
mod cpu_ids;
mod harvesting;
pub use cpu_ids::{CpuIdMapping, CpuIdRecipe, WatchdogProjection};
pub use harvesting::{HARVESTED, HarvestingField};

const CPUS: &str = "/cpus";
const RUNNING: &[u8] = b"running\0";
const WAITING: &[u8] = b"waiting\0";

/// Explicit source-node/resource association supplied by the checked owner.
/// IDs are not interpreted as firmware IDs, addresses, masks or phandles.
#[derive(Debug, Clone, Copy)]
pub struct NodeBinding<'a> {
    pub slot: GuestSlotId,
    pub name: &'a str,
    pub midr: u64,
    pub cache: CacheId,
    pub interrupt: InterruptId,
    pub start: StartId,
    pub core_power: DomainId,
    pub cluster_power: DomainId,
}
#[derive(Debug, Clone, Copy)]
pub enum Count {
    Pes,
    Clusters,
    CoreType(u8),
}
/// A source-checked count property. Values have the original u32 encoding.
/// Absent fields are created only when explicitly listed by the caller.
#[derive(Debug, Clone, Copy)]
pub struct CountField<'a> {
    pub path: &'a str,
    pub property: &'a str,
    pub count: Count,
}
#[derive(Debug, Clone, Copy)]
pub enum ReferenceEncoding {
    /// Every u32 cell names a CPU node's AAPL,phandle.
    Phandles,
    /// Every u32 cell names the original firmware cpu-id (not logical ID).
    FirmwareCpuIds,
}
/// A source-checked external CPU reference list. A reference to a removed
/// CPU refuses projection; its consumers need their own checked adaptation.
#[derive(Debug, Clone, Copy)]
pub struct ReferenceField<'a> {
    pub path: &'a str,
    pub property: &'a str,
    pub encoding: ReferenceEncoding,
}
/// Immutable input identity and kept CPU resource bindings. Every
/// original CPU is independently shape/identity validated before projection;
/// removed CPUs need no invented executor or lifecycle IDs. The source owner
/// must still check all topology consumers; arbitrary property bytes
/// are not guessed to be phandles, IDs or CPU masks. This metadata grants no
/// device/executor permission or completeness credit for unchecked consumers.
#[derive(Debug)]
pub struct SourceBindings<'a> {
    pub source_set_sha256: [u8; 32],
    pub tree_sha256: [u8; 32],
    pub nodes: &'a [NodeBinding<'a>],
    pub counts: &'a [CountField<'a>],
    pub references: &'a [ReferenceField<'a>],
    /// Explicitly checked coordinated consumer recipe; ordinary projections
    /// preserve source IDs and cannot opt into renumbering implicitly.
    pub cpu_ids: CpuIdRecipe<'a>,
    /// The PMGR's CPU-harvesting mode property. A reduced roster must then
    /// have the shape the harvested PMGR constructor binds, and the property
    /// declares that mode; a complete roster leaves it unchanged.
    pub harvesting: Option<HarvestingField<'a>>,
}

#[derive(Debug)]
pub struct Projection<'p, 's> {
    plan: &'p Plan<'s>,
    original: DeviceTree,
    projected: DeviceTree,
    receipt: Receipt,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Receipt {
    pub plan: PlanIdentity,
    pub source_sha256: [u8; 32],
    pub projected_sha256: [u8; 32],
    pub boot_path: String,
    pub removed: Vec<String>,
    pub cpu_ids: Vec<CpuIdMapping>,
    pub watchdog: Option<WatchdogProjection>,
    /// The PMGR harvesting mode published for a reduced roster.
    pub harvesting: Option<u32>,
}
fn canonical_path(path: &str) -> bool {
    path == "/"
        || (path.starts_with('/')
            && path[1..]
                .split('/')
                .all(|part| !part.is_empty() && part != "." && part != ".."))
}
fn validate_tree(node: &Node, depth: usize) -> Result<()> {
    require!(
        depth <= super::MAX_DEPTH,
        "DeviceTree topology nesting is too deep"
    );
    let mut properties = BTreeSet::new();
    for p in &node.properties {
        require!(
            properties.insert(p.name()),
            "duplicate DeviceTree property {name}",
            name = p.name()
        );
    }
    let mut children = BTreeSet::new();
    for child in &node.children {
        let name = child.name().ok_or(refuse!("unnamed DeviceTree child"))?;
        require!(
            !name.is_empty() && !name.contains('/') && children.insert(name),
            "ambiguous DeviceTree child name"
        );
        validate_tree(child, depth + 1)?;
    }
    Ok(())
}
fn digest(tree: &DeviceTree) -> [u8; 32] {
    Sha256::digest(tree.serialize()).into()
}
fn word(node: &Node, name: &str) -> Result<u32> {
    let p = node
        .property(name)
        .ok_or_else(|| refuse!("CPU missing {name}", name))?;
    require!(
        !p.placeholder,
        "CPU {name} is an unresolved template value",
        name
    );
    p.as_u32()
        .ok_or_else(|| refuse!("CPU {name} is not a u32", name).into())
}
fn bytes<'a>(node: &'a Node, name: &str) -> Result<&'a [u8]> {
    let p = node
        .property(name)
        .ok_or_else(|| refuse!("CPU missing {name}", name))?;
    require!(
        !p.placeholder,
        "CPU {name} is an unresolved template value",
        name
    );
    Ok(&p.value)
}
fn check_range(node: &Node, name: &str, base: u64, size: u64) -> Result<()> {
    let v = bytes(node, name)?;
    require!(v.len() == 16, "CPU {name} is not a base/size pair", name);
    require!(
        v[..8] == base.to_le_bytes() && v[8..] == size.to_le_bytes(),
        "CPU {name} differs from the plan's retained resource",
        name
    );
    Ok(())
}
/// T8112's original normal-mode MPIDR/CPU association contract supplies these
/// coordinates from raw reg (original J473 CPU power ownership).
/// The original templates omit the three fields; never add them to the ADT.
fn t8112_coordinates(node: &Node, soc: u32) -> Result<Option<[u32; 3]>> {
    if soc != 0x8112 {
        return Ok(None);
    }
    let reg = word(node, "reg")?;
    require!(
        reg & !0x103 == 0 && word(node, "cluster-id")? == reg >> 8,
        "unqualified T8112 CPU affinity/cluster coordinates"
    );
    let coordinates = [0, reg >> 8, reg & 3];
    for (field, expected) in ["die-id", "die-cluster-id", "cluster-core-id"]
        .into_iter()
        .zip(coordinates)
    {
        if node.property(field).is_some() {
            require!(
                word(node, field)? == expected,
                "T8112 {field} differs from source affinity",
                field
            );
        }
    }
    Ok(Some(coordinates))
}
fn shape(node: &Node, soc: u32) -> Result<()> {
    require!(
        node.children.is_empty(),
        "nested CPU topology needs a qualified schema"
    );
    let mut names = BTreeSet::new();
    for p in &node.properties {
        require!(
            names.insert(p.name()),
            "duplicate CPU property {name}",
            name = p.name()
        );
    }
    require!(
        bytes(node, "device_type")? == b"cpu\0",
        "non-CPU child under /cpus"
    );
    require!(
        [WAITING, RUNNING].contains(&bytes(node, "state")?),
        "unqualified CPU state"
    );
    require!(
        !bytes(node, "compatible")?.is_empty(),
        "missing CPU ISA compatible"
    );
    require!(
        node.property("status").is_none(),
        "CPU status needs a qualified enumeration schema"
    );
    for field in ["l3-cache-size", "l3-cache-id"] {
        if node.property(field).is_some() {
            require!(
                word(node, field)? == 0,
                "CPU L3 resources are not represented by this plan"
            );
        }
    }
    word(node, "cpu-id")?;
    require!(
        word(node, "AAPL,phandle")? != 0,
        "zero original CPU phandle"
    );
    for field in ["cluster-id", "l2-cache-size"] {
        word(node, field)?;
    }
    if t8112_coordinates(node, soc)?.is_none() {
        for field in ["die-id", "die-cluster-id", "cluster-core-id"] {
            word(node, field)?;
        }
    }
    let kind = bytes(node, "cluster-type")?;
    require!(
        !kind.is_empty() && kind[0] != 0 && kind[1..].iter().all(|v| *v == 0),
        "invalid original CPU cluster-type"
    );
    for field in ["cpu-impl-reg", "acc-impl-reg", "cpm-impl-reg"] {
        let raw = bytes(node, field)?;
        require!(
            raw.len() == 16,
            "CPU {field} is not a base/size pair",
            field
        );
        let base = u64::from_le_bytes(raw[..8].try_into().unwrap());
        let size = u64::from_le_bytes(raw[8..].try_into().unwrap());
        require!(
            base != 0 && size != 0 && base.checked_add(size).is_some(),
            "invalid original CPU {field} extent",
            field
        );
    }
    Ok(())
}

impl<'p, 's> Projection<'p, 's> {
    /// Preflight the complete plan and source before creating a candidate.
    /// Source nodes are never cloned into additional slots or renumbered.
    pub fn prepare(
        plan: &'p Plan<'s>,
        tree: &DeviceTree,
        source: &SourceBindings<'_>,
    ) -> Result<Self> {
        plan.revalidate()?;
        validate_tree(&tree.root, 0)?;
        require!(
            source.source_set_sha256 == plan.profile().input_set_sha256,
            "topology source set differs from plan"
        );
        require!(
            source.tree_sha256 != [0; 32] && digest(tree) == source.tree_sha256,
            "original DeviceTree identity mismatch"
        );
        // The pinned public XNU L2-sharing consumer walks a contiguous
        // first_cpu_id..first_cpu_id+num_cpus interval for each cluster.
        // Preserve plan logical IDs; refuse an unsupported order instead of
        // silently changing that identity during projection.
        let mut completed = BTreeSet::new();
        let mut last = None;
        for pe in plan.pes() {
            if last != Some(pe.cluster) {
                require!(
                    !completed.contains(&pe.cluster),
                    "noncontiguous cluster enumeration requires a qualified cache consumer"
                );
                if let Some(cluster) = last {
                    completed.insert(cluster);
                }
                last = Some(pe.cluster);
            }
        }
        let cpus = tree.node(CPUS).ok_or(refuse!("DeviceTree has no /cpus"))?;
        require!(
            word(cpus, "#address-cells")? == 1 && word(cpus, "#size-cells")? == 0,
            "unqualified /cpus address/size encoding"
        );
        let mut phandles = BTreeSet::new();
        let mut firmware_ids = BTreeSet::new();
        let mut affinities = BTreeSet::new();
        // Source enumeration and native ownership are separate phases. Validate
        // ALL original nodes, including those that will be removed, without
        // requiring invented resources for an unassigned hardware slot.
        for node in &cpus.children {
            shape(node, plan.profile().soc)?;
            let reg = word(node, "reg")?;
            require!(
                reg <= 0xffff && affinities.insert(reg),
                "invalid or duplicate original CPU affinity"
            );
            require!(
                phandles.insert(word(node, "AAPL,phandle")?)
                    && firmware_ids.insert(word(node, "cpu-id")?),
                "duplicate original CPU phandle/firmware ID"
            );
        }
        let mut names = BTreeSet::new();
        let mut slots = BTreeSet::new();
        for binding in source.nodes {
            require!(
                !binding.name.contains('/') && !binding.name.is_empty(),
                "invalid CPU source name"
            );
            require!(
                names.insert(binding.name) && slots.insert(binding.slot),
                "duplicate CPU source binding"
            );
            cpus.child(binding.name)
                .ok_or(refuse!("CPU binding has no original node"))?;
        }
        let mut projected = tree.clone();
        let mut selected = Vec::with_capacity(plan.pes().len());
        let mut retained_phandles = BTreeSet::new();
        let mut retained_firmware_ids = BTreeSet::new();
        let mut boot_path = String::new();
        for pe in plan.pes() {
            let binding = source
                .nodes
                .iter()
                .find(|b| b.slot == pe.slot)
                .ok_or_else(|| {
                    refuse!(
                        "missing declared source CPU slot {slot}",
                        slot = alloc::format!("{:?}", pe.slot)
                    )
                })?;
            require!(
                binding.midr == pe.identity.midr
                    && binding.cache == pe.cache
                    && binding.interrupt == pe.interrupt
                    && binding.start == pe.start
                    && binding.core_power == pe.core_power
                    && binding.cluster_power == pe.cluster_power,
                "CPU source ISA/resource binding differs from plan slot {slot}",
                slot = alloc::format!("{:?}", pe.slot)
            );
            let node = cpus.child(binding.name).unwrap();
            let cluster = plan
                .clusters()
                .iter()
                .find(|c| c.resource.id == pe.cluster)
                .unwrap();
            for (key, expected) in [
                ("reg", pe.reg),
                ("cluster-id", cluster.resource.firmware_id),
                ("l2-cache-size", cluster.cache.geometry.bytes),
            ] {
                require!(
                    word(node, key)? == expected,
                    "CPU {key} differs from plan slot {slot}",
                    key,
                    slot = alloc::format!("{:?}", pe.slot)
                );
            }
            let implicit = t8112_coordinates(node, plan.profile().soc)?;
            for (index, (key, expected)) in [
                ("die-id", cluster.resource.die),
                ("die-cluster-id", cluster.resource.die_cluster),
                ("cluster-core-id", pe.cluster_core),
            ]
            .into_iter()
            .enumerate()
            {
                let actual = if node.property(key).is_some() {
                    word(node, key)?
                } else {
                    implicit.ok_or(refuse!("CPU coordinate has no qualified source schema"))?[index]
                };
                require!(
                    actual == expected,
                    "CPU {key} differs from plan slot {slot}",
                    key,
                    slot = alloc::format!("{:?}", pe.slot)
                );
            }
            let kind = bytes(node, "cluster-type")?;
            require!(
                kind.first() == Some(&pe.cluster_type) && kind[1..].iter().all(|v| *v == 0),
                "CPU cluster-type differs from plan"
            );
            check_range(node, "cpu-impl-reg", pe.cpu_impl.base, pe.cpu_impl.bytes)?;
            check_range(node, "acc-impl-reg", pe.acc_impl.base, pe.acc_impl.bytes)?;
            check_range(node, "cpm-impl-reg", pe.cpm_impl.base, pe.cpm_impl.bytes)?;
            retained_phandles.insert(word(node, "AAPL,phandle")?);
            retained_firmware_ids.insert(word(node, "cpu-id")?);
            let mut node = node.clone();
            node.set_property("state", if pe.boot { RUNNING } else { WAITING }.to_vec())?;
            if pe.boot {
                boot_path = format!("{CPUS}/{}", binding.name);
            }
            selected.push(node);
        }
        let cpu_ids = cpu_ids::Prepared::prepare(plan, tree, source)?;
        // Explicitly typed external lists are checked, never rewritten with
        // guessed mask/ID semantics. Ordinary matching integer values are not
        // evidence of a CPU reference.
        let mut locations = BTreeSet::new();
        for field in source.references {
            require!(canonical_path(field.path), "noncanonical CPU consumer path");
            require!(
                locations.insert((field.path, field.property)),
                "duplicate CPU consumer location"
            );
            require!(
                !field.path.starts_with("/cpus/"),
                "external reference points inside CPU node"
            );
            let p = tree
                .property(field.path, field.property)
                .ok_or(refuse!("missing CPU reference property"))?;
            require!(
                !p.placeholder && p.value.len().is_multiple_of(4),
                "invalid CPU reference encoding"
            );
            let retained = match field.encoding {
                ReferenceEncoding::Phandles => &retained_phandles,
                ReferenceEncoding::FirmwareCpuIds => &retained_firmware_ids,
            };
            for cell in p.value.as_chunks::<4>().0 {
                let id = u32::from_le_bytes(*cell);
                require!(
                    retained.contains(&id),
                    "CPU consumer {path}/{property} references removed or unknown CPU {id}",
                    path = field.path,
                    property = field.property,
                    id
                );
                if matches!(field.encoding, ReferenceEncoding::FirmwareCpuIds) {
                    require!(
                        cpu_ids.preserves_reference(id),
                        "CPU ID consumer needs a coordinated projection recipe"
                    );
                }
            }
        }
        let harvesting = match source.harvesting {
            Some(field) => {
                require!(
                    canonical_path(field.path) && !field.path.starts_with("/cpus"),
                    "noncanonical PMGR harvesting path"
                );
                require!(
                    locations.insert((field.path, field.property)),
                    "duplicate CPU consumer location"
                );
                let kept = selected
                    .iter()
                    .map(|node| node.name().expect("validated original name"))
                    .collect::<Vec<_>>();
                harvesting::prepare(tree, &kept, field)?
            }
            None => None,
        };
        projected.node_mut(CPUS).unwrap().children = selected;
        if let (Some(field), Some(mode)) = (source.harvesting, harvesting) {
            projected.set_property(field.path, field.property, mode.to_le_bytes().to_vec())?;
        }
        for field in source.counts {
            require!(canonical_path(field.path), "noncanonical CPU count path");
            require!(
                locations.insert((field.path, field.property)),
                "duplicate CPU consumer location"
            );
            require!(
                !field.path.starts_with("/cpus/"),
                "count property overwrites a CPU field"
            );
            if let Some(p) = tree.property(field.path, field.property) {
                require!(
                    !p.placeholder && p.as_u32().is_some(),
                    "unqualified CPU count encoding"
                );
            }
            let count = match field.count {
                Count::Pes => plan.pes().len(),
                Count::Clusters => plan.clusters().len(),
                Count::CoreType(kind) => plan
                    .pes()
                    .iter()
                    .filter(|pe| pe.cluster_type == kind)
                    .count(),
            };
            projected.set_property(
                field.path,
                field.property,
                (count as u32).to_le_bytes().to_vec(),
            )?;
        }
        // Known topology fields cannot silently keep stale counts/masks or
        // unchecked reference schemas. Source owners must list each one.
        fn consumers(node: &Node, path: &str, locations: &BTreeSet<(&str, &str)>) -> Result<()> {
            if !path.starts_with("/cpus/") {
                for p in &node.properties {
                    if matches!(
                        p.name(),
                        "max_cpus"
                            | "e-core-count"
                            | "p-core-count"
                            | "m-core-count"
                            | "cpu-count"
                            | "cpu-cluster-count"
                            | "cluster-count"
                            | "cpu-map"
                            | "cpu-mask"
                            | "cpumask"
                            | "#main-cpus"
                            | "acc-harvesting"
                    ) {
                        require!(
                            locations.contains(&(path, p.name())),
                            "unqualified CPU consumer {path}/{name}",
                            path,
                            name = p.name()
                        );
                    }
                }
            }
            for child in &node.children {
                let name = child.name().ok_or(refuse!("unnamed DeviceTree node"))?;
                let next = format!("{}/{name}", path.trim_end_matches('/'));
                consumers(child, &next, locations)?;
            }
            Ok(())
        }
        consumers(&tree.root, "/", &locations)?;
        let (cpu_ids, watchdog) = cpu_ids.apply(&mut projected)?;
        let removed = cpus
            .children
            .iter()
            .filter(|node| {
                !projected
                    .node(CPUS)
                    .unwrap()
                    .children
                    .iter()
                    .any(|selected| selected.name() == node.name())
            })
            .map(|node| format!("{CPUS}/{}", node.name().expect("validated original name")))
            .collect();
        let receipt = Receipt {
            plan: plan.identity(),
            source_sha256: source.tree_sha256,
            projected_sha256: digest(&projected),
            boot_path,
            removed,
            cpu_ids,
            watchdog,
            harvesting,
        };
        plan.revalidate()?;
        Ok(Self {
            plan,
            original: tree.clone(),
            projected,
            receipt,
        })
    }
    pub fn receipt(&self) -> &Receipt {
        &self.receipt
    }
    /// Complete validation precedes this single replacement. No fallible
    /// operation follows mutation; stale source/plan leaves the tree intact.
    pub fn apply(self, tree: &mut DeviceTree) -> Result<Receipt> {
        self.plan.revalidate()?;
        require!(
            *tree == self.original,
            "DeviceTree changed after topology preflight"
        );
        *tree = self.projected;
        Ok(self.receipt)
    }
}

#[cfg(test)]
mod tests;
