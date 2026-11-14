//! The PMGR's CPU-harvesting mode for a reduced CPU roster.
//!
//! ApplePMGR builds its CPU complexes and cores from `/arm-io/pmgr`
//! `perf-domains` and `devices`, in record order, in the mode its
//! `acc-harvesting` property selects (self+0x238c; an absent property keeps
//! the constructor's default 0). 26B123 mac14g `ApplePMGR::initDriver`:
//!
//! - Mode 0 (0xfffffe0009b92a84) gives every CPU-prefixed core record the
//!   next CPU number, whatever the kernel topology holds.
//! - Mode 1 creates a complex for a CPU perf domain only while fewer exist
//!   than `ml_get_topology_info()->num_clusters` (0xfffffe0009b90764), so
//!   complex n is kernel cluster n. It skips every record of a perf domain
//!   that got no complex (0xfffffe0009b92418), and numbers a core record only
//!   while its complex has fewer cores than its kernel cluster's CPU count
//!   (0xfffffe0009b92a40..0xfffffe0009b92a80). Each core keeps its index in
//!   its complex as its physical core (`getPhysCoreInComplex`).
//!
//! `ApplePMGR::start` restores every complex's core energy counters through
//! `restoreHW`, `restoreACC` and `_restoreCPUEnergyAccumulators`, and
//! `AppleT8110PMGR::setCPUCoreEnergyAccumulator` (0xfffffe0009fc01a4)
//! requires each core's number below `num_cpus`, its `die-cluster-id` below
//! `num_clusters` and its `cluster-core-id` below that cluster's CPU count.
//! In mode 0 a reduced roster leaves records numbered at or above `num_cpus`,
//! and that check panics. Mode 1 matches the kernel when kernel cluster n is
//! the n-th original cluster and each kept cluster keeps its first cores, in
//! original order: complex n then binds cluster n, and its numbered records
//! are that cluster's CPUs, with the CPU numbers of the dense roster.
use super::{CPUS, Result};
use crate::apple_device_tree::DeviceTree;
use alloc::vec::Vec;
use stela_contracts::{refuse, require};

/// Mode 1, the mode Apple's own harvested CPU rosters use.
pub const HARVESTED: u32 = 1;

/// The PMGR property that selects the constructor's harvesting mode.
#[derive(Debug, Clone, Copy)]
pub struct HarvestingField<'a> {
    pub path: &'a str,
    pub property: &'a str,
}

/// The harvesting mode to publish for the kept CPUs `kept` (names, in
/// projected order), or `None` when every original CPU is kept and the
/// source mode stays. A reduced roster must keep a prefix of the original
/// clusters and the first CPUs of each kept cluster, in original order.
pub(super) fn prepare(
    tree: &DeviceTree,
    kept: &[&str],
    field: HarvestingField<'_>,
) -> Result<Option<u32>> {
    let cpus = tree.node(CPUS).ok_or(refuse!("DeviceTree has no /cpus"))?;
    let mut original = Vec::with_capacity(cpus.children.len());
    for node in &cpus.children {
        let name = node.name().ok_or(refuse!("unnamed original CPU"))?;
        original.push((name, super::word(node, "cluster-id")?));
    }
    if kept.len() == original.len() {
        return Ok(None);
    }
    let mut clusters: Vec<u32> = Vec::new();
    for &(_, cluster) in &original {
        if clusters.last() != Some(&cluster) {
            require!(
                !clusters.contains(&cluster),
                "a reduced CPU roster needs each original cluster contiguous for the PMGR's complexes"
            );
            clusters.push(cluster);
        }
    }
    let in_order = original
        .iter()
        .filter(|(name, _)| kept.contains(name))
        .map(|(name, _)| *name)
        .collect::<Vec<_>>();
    require!(
        in_order == kept,
        "a reduced CPU roster must keep the original CPU order the PMGR numbers its cores in"
    );
    let mut emptied = false;
    for cluster in clusters {
        let members = original
            .iter()
            .filter(|(_, own)| *own == cluster)
            .map(|(name, _)| kept.contains(name))
            .collect::<Vec<_>>();
        let first = members.iter().take_while(|kept| **kept).count();
        require!(
            members[first..].iter().all(|kept| !kept),
            "a reduced CPU roster must keep the first CPUs of each kept cluster for the PMGR's complexes"
        );
        if first == 0 {
            emptied = true;
        } else {
            require!(
                !emptied,
                "a reduced CPU roster must keep a prefix of the original clusters for the PMGR's complexes"
            );
        }
    }
    match tree.property(field.path, field.property) {
        Some(p) => require!(
            !p.placeholder && matches!(p.as_u32(), Some(0 | HARVESTED)),
            "PMGR {path}/{property} has an unqualified harvesting mode",
            path = field.path,
            property = field.property
        ),
        None => require!(
            tree.node(field.path).is_some(),
            "a reduced CPU roster needs the PMGR at {path}",
            path = field.path
        ),
    }
    Ok(Some(HARVESTED))
}
