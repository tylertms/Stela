//! Selected original T8112 resources. Membership is derived by the same
//! normal-mode resolver for each PE, preserving the running boot PE.
//! Capacities stay inside the complete original eight-PE/two-complex envelope;
//! this adapter cannot expand an Apple source profile to additional host cores.
use super::*;
use crate::Result;
use stela_contracts::refuse;

impl<'a> BspSource<'a> {
    /// Construct metadata for `count` assigned workers, BSP first. The
    /// caller must keep a started physical roster and join native permission.
    /// Selection keeps the source boot PE first and each complex contiguous
    /// for XNU's L2-sharing interval consumer, preserving source order within
    /// each complex after the boot member. Both complexes keep at least one
    /// PE; the other complex takes only the PEs the boot complex cannot hold.
    /// No running state, power ACK, interrupt or executor is manufactured here.
    pub fn t8112_assigned(
        original: &'a [u8],
        prepared: &[u8],
        identity: SourceIdentity,
        boot_reg: u32,
        acc_harvesting: u32,
        count: usize,
    ) -> Result<Self> {
        let root =
            Node::root(original).ok_or(refuse!("assigned original source tree is malformed"))?;
        let cpus = root
            .child("cpus")
            .ok_or(refuse!("assigned original source lacks CPUs"))?;
        let nodes = cpus
            .children()
            .ok_or(refuse!("assigned original CPU list is malformed"))?;
        // The normal-mode association resolver checks this original family
        // as two four-core complexes. Counts beyond this envelope require a
        // separate whole-consumer capacity proof, never cloning a source slot.
        if nodes.len() != 8 || count == 0 || count > nodes.len() {
            return Err(refuse!(
                "assigned T8112 worker count exceeds the qualified source envelope"
            ));
        }
        // AppleT8110PMGR::restoreHW and quiesceHW enable and disable CPU
        // complexes 0 and 1 by constant index, and
        // ApplePMGR::enableCPUComplex requires that index below the number of
        // complexes it built from the kept CPUs. Each complex therefore keeps
        // at least one PE, and a single PE cannot form this guest.
        // AppleT8112PMGR's SRAM energy accessors pick a bank by CPU number
        // (0xfffffe0009fc43c4), so with a partial E complex a P core's
        // samples use a pruned E core's bank; every original bank stays
        // mapped from `hw-dpe-reg` and holds what software writes.
        if count < 2 {
            return Err(refuse!(
                "assigned T8112 roster needs a PE in each of the two CPU complexes its PMGR enables"
            ));
        }
        let boot_complex = boot_reg >> 8;
        let mut boot_size = 0;
        for node in &nodes {
            if word(node, "reg")? >> 8 == boot_complex {
                boot_size += 1;
            }
        }
        let other = count.saturating_sub(boot_size).max(1);
        let mut regs = vec![boot_reg];
        for (same_complex, limit) in [(true, count - other), (false, count)] {
            for node in &nodes {
                let reg = word(node, "reg")?;
                if reg != boot_reg
                    && (reg >> 8 == boot_complex) == same_complex
                    && regs.len() < limit
                {
                    regs.push(reg);
                }
            }
        }
        let mut result = Self::t8112(original, prepared, identity, boot_reg, acc_harvesting)?;
        result.remap_member(0);
        for (ordinal, &reg) in regs.iter().enumerate().skip(1) {
            let mut member =
                Self::t8112_member(original, prepared, identity, boot_reg, reg, acc_harvesting)?;
            member.remap_member(ordinal as u32);
            for cluster in member.clusters {
                if let Some(old) = result.clusters.iter().find(|old| old.id == cluster.id) {
                    if old != &cluster {
                        return Err(refuse!(
                            "assigned source cluster metadata conflicts across PEs"
                        ));
                    }
                } else {
                    result.clusters.push(cluster);
                }
            }
            for cache in member.caches {
                if let Some(old) = result.caches.iter().find(|old| old.id == cache.id) {
                    if old != &cache {
                        return Err(refuse!(
                            "assigned source cache metadata conflicts across PEs"
                        ));
                    }
                } else {
                    result.caches.push(cache);
                }
            }
            for power in member.powers {
                if let Some(old) = result.powers.iter_mut().find(|old| old.id == power.id) {
                    for core in power.cores {
                        if old.cores.contains(&core) {
                            return Err(refuse!("assigned source repeats core power membership"));
                        }
                        old.cores.push(core);
                    }
                    for cluster in power.clusters {
                        if !old.clusters.contains(&cluster) {
                            old.clusters.push(cluster);
                        }
                    }
                } else {
                    result.powers.push(power);
                }
            }
            result.kinds.extend(member.kinds);
            result.slots.extend(member.slots);
            result.interrupts.extend(member.interrupts);
            result.starts.extend(member.starts);
        }
        for capacity in &mut result.capacities {
            capacity.maximum = Some(match capacity.kind {
                CapacityKind::KernelClusters
                | CapacityKind::PlatformClusters
                | CapacityKind::KernelPhysicalCluster => 2,
                _ => 8,
            });
            capacity.provenance = "T8112 normal-mode original eight-PE/two-complex source envelope; no source expansion";
        }
        result.provenance = "T8112 selected original normal-mode associations; actual worker and platform adoption required";
        Ok(result)
    }

    fn remap_member(&mut self, ordinal: u32) {
        let slot = GuestSlotId(ordinal);
        let cluster = ClusterId(self.slots[0].reg >> 8);
        self.kinds[0].id = KindId(ordinal);
        self.slots[0].id = slot;
        self.slots[0].kind = Some(KindId(ordinal));
        self.slots[0].cluster = cluster;
        self.slots[0].interrupt = Some(InterruptId(ordinal));
        self.slots[0].start = Some(StartId(ordinal));
        self.clusters[0].id = cluster;
        self.clusters[0].cache = Some(CacheId(cluster.0));
        self.caches[0].id = CacheId(cluster.0);
        self.caches[0].cluster = cluster;
        self.interrupts[0].id = InterruptId(ordinal);
        self.starts[0].id = StartId(ordinal);
        self.powers[0].cores = vec![slot];
        self.powers[1].clusters = vec![cluster];
    }

    /// Original node name for this exact selected resource slot.
    pub fn name_for(&self, slot: GuestSlotId) -> Result<&str> {
        let reg = self
            .slots
            .iter()
            .find(|member| member.id == slot)
            .ok_or(refuse!("assigned source slot is absent"))?
            .reg;
        let root =
            Node::root(self.source).ok_or(refuse!("assigned retained source is malformed"))?;
        let cpus = root
            .child("cpus")
            .ok_or(refuse!("assigned retained source lacks CPUs"))?;
        let nodes = cpus
            .children()
            .ok_or(refuse!("assigned retained CPU list is malformed"))?;
        let node = nodes
            .iter()
            .find(|node| word(node, "reg") == Ok(reg))
            .ok_or(refuse!("assigned retained source PE is absent"))?;
        text(node, "name")
    }
}
