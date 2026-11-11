//! Runtime assignment, before firmware publication. Metadata is joined to the
//! BSP observation and a resource owner; it is never a stock
//! SKU count or executor/lifecycle permission. Parent owns RAM/device adoption.
//!
//! Original J314s source contract: iBoot file3ab04..3ab7c publishes max/E/P;
//! mac13j XNU VAfffffe000bda7168 enumerates CPU children, not waiting flags;
//! AppleInterruptControllerV2 file2822dc4/282375c uses #main-cpus. SPTM
//! VAfffffff0270ca10c requires the three implementation ranges. Source
//! hashes/coordinates remain in the immutable j314s-single-core-topology audit.
//! No source hash selects production behavior and no capacity is inferred here.
use crate::Result;
use alloc::{format, vec::Vec};
use stela_contracts::refuse;
use stela_engine::machine::CpuProfile;
use stela_loader::apple_device_tree::{
    DeviceTree,
    assigned::{Count, CountField, CpuIdRecipe, NodeBinding, Projection, Receipt, SourceBindings},
};
use stela_soc_data::board::assigned::AssignedCore;
use stela_soc_data::board::assigned::Assignment;
use stela_soc_data::board::assigned::BspSource;
use stela_soc_data::board::assigned::Plan;
use stela_soc_data::board::assigned::PlanIdentity;
use stela_soc_data::board::assigned::RegisterRange;
use stela_soc_data::board::assigned::RetainedResources;
use stela_soc_data::board::assigned::RetainedRoster;
use stela_soc_data::board::assigned::SourceIdentity;

use crate::bundle::live_tree::LiveTree;
use crate::{record::Capture, topology as native};
#[cfg(test)]
use stela_soc_data::board::assigned::GuestSlotId;

/// These count consumers are independently source-checked for the kept
/// Apple ARM CPU/AIC schema (original J314s audit). They all use the same Plan.
/// This adds no cpu-cluster-count or PMGR hardware-cluster rule.
pub const COUNT_FIELDS: [CountField<'static>; 4] = [
    CountField {
        path: "/cpus",
        property: "max_cpus",
        count: Count::Pes,
    },
    CountField {
        path: "/cpus",
        property: "e-core-count",
        count: Count::CoreType(b'E'),
    },
    CountField {
        path: "/cpus",
        property: "p-core-count",
        count: Count::CoreType(b'P'),
    },
    CountField {
        path: "/arm-io/aic",
        property: "#main-cpus",
        count: Count::Pes,
    },
];

/// Keeps the immutable pre-EBS observation. Only the executing BSP is
/// started by this adapter. Captured AP observations do not imply AP startup.
pub struct BspRoster<'a> {
    capture: &'a Capture,
    #[cfg(test)]
    source: native::Processor,
    cores: [AssignedCore; 1],
}
impl<'a> BspRoster<'a> {
    /// `capture` must be the record already accepted by native_smp's
    /// reservation/source validation. The supplied owner is the installed BSP
    /// Host, which the caller must keep through subsequent execution. This
    /// view does not bind a runtime WorkerOwner, loan a frame or authorize RAM.
    #[cfg(target_arch = "x86_64")]
    pub fn current(capture: &'a Capture, worker: usize) -> Result<Self> {
        Self::observed(capture, worker, current_apic()?)
    }
    #[cfg(any(test, target_arch = "x86_64"))]
    fn observed(capture: &'a Capture, worker: usize, current: u32) -> Result<Self> {
        let processors = capture
            .processors
            .get(..capture.processor_count as usize)
            .ok_or(refuse!(
                "assigned topology has an invalid captured processor count"
            ))?;
        let logical = native::select(processors)
            .map_err(|e| refuse!("assigned BSP source roster: {e}", ?e))?;
        let selected = logical.get(..capture.core_count as usize).ok_or(refuse!(
            "assigned topology has more captured processors than the inventory"
        ))?;
        if logical.len() != capture.logical_count as usize
            || selected
                .iter()
                .enumerate()
                .any(|(slot, &index)| capture.selected[slot] as usize != index)
        {
            return Err(refuse!(
                "assigned topology differs from the retained processor selection"
            ));
        }
        let index = *selected
            .first()
            .ok_or(refuse!("assigned topology has no retained BSP"))?;
        let source = processors[index];
        if source.flags & native::BSP == 0
            || !source.available()
            || source.apic_id != u64::from(current)
            || !capture.snapshots[0].matches(source)
        {
            return Err(refuse!(
                "assigned topology requires the actual executing captured BSP"
            ));
        }
        Ok(Self {
            capture,
            #[cfg(test)]
            source,
            cores: [AssignedCore {
                worker,
                package: source.location.package,
                core: source.location.core,
                apic: current,
                started: true,
            }],
        })
    }
    #[cfg(test)]
    pub fn source(&self) -> native::Processor {
        self.source
    }
}
impl RetainedRoster for BspRoster<'_> {
    fn revision(&self) -> u64 {
        1
    }
    fn cores(&self) -> &[AssignedCore] {
        // Keep the source allocation borrowed for the entire Plan.
        let _ = self.capture;
        &self.cores
    }
}

#[cfg(target_arch = "x86_64")]
fn current_apic() -> Result<u32> {
    use core::arch::x86_64::__cpuid_count;
    // CPUID is unprivileged. Reuse the canonical captured APIC decoder; this
    // observation establishes current identity, not native capability admission.
    current_identity(|leaf, subleaf| {
        let value = __cpuid_count(leaf, subleaf);
        native::Cpuid {
            leaf,
            subleaf,
            eax: value.eax,
            ebx: value.ebx,
            ecx: value.ecx,
            edx: value.edx,
        }
    })
    .apic_id()
    .ok_or(refuse!("assigned BSP has no actual CPUID APIC identity"))
}

/// Keep the maximum/vendor records together with the leaves they
/// authorize. Keeping maxima only in local variables creates an incomplete
/// Snapshot that the shared identity decoder must correctly refuse. The query
/// seam permits direct collector regression tests without a privileged owner
/// or guessed processor identity; production above queries this CPU.
#[cfg(any(test, target_arch = "x86_64"))]
fn current_identity(mut query: impl FnMut(u32, u32) -> native::Cpuid) -> native::Snapshot {
    let identity = query(0, 0);
    let extended = query(0x8000_0000, 0);
    let mut snapshot = native::Snapshot::default();
    snapshot.records[0] = identity;
    snapshot.records[1] = extended;
    snapshot.count = 2;
    for (leaf, subleaf) in [
        (1, 0),
        (0xb, 0),
        (0x1f, 0),
        (0x8000_0001, 0),
        (0x8000_001e, 0),
    ] {
        if (leaf < 0x8000_0000 && leaf > identity.eax)
            || (leaf >= 0x8000_0000 && leaf > extended.eax)
        {
            continue;
        }
        snapshot.records[snapshot.count as usize] = query(leaf, subleaf);
        snapshot.count += 1;
    }
    snapshot
}

/// A source protocol reference, never a DeviceID inferred from its payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Function {
    pub phandle: u32,
    pub selector: u32,
    pub argument: u32,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Join {
    pub plan: PlanIdentity,
    pub logical: usize,
    pub worker: usize,
    pub apic: u32,
    pub reg: u32,
    pub cluster: stela_soc_data::board::assigned::ClusterId,
    pub cache: stela_soc_data::board::assigned::CacheId,
    pub interrupt: stela_soc_data::board::assigned::InterruptId,
    /// Original CPU error input, distinct from the PE/IPI routing owner above.
    pub error_interrupt_parent: u32,
    pub error_interrupt: u32,
    pub start: stela_soc_data::board::assigned::StartId,
    pub enable_core: Function,
    pub core_power: stela_soc_data::pmgr::coverage::DomainId,
    pub cluster_power: stela_soc_data::pmgr::coverage::DomainId,
    pub cpu_impl: RegisterRange,
    pub acc_impl: RegisterRange,
    pub cpm_impl: RegisterRange,
}

/// Source owner and immutable ADT. Full source power records remain
/// independently of the selected PE subset and unknown PS state.
pub struct Source<'a> {
    pub resources: BspSource<'a>,
    tree: DeviceTree,
    dense_ids: Option<(DeviceTree, [u8; 32])>,
    boot_reg: u32,
    acc_harvesting: u32,
}
impl<'a> Source<'a> {
    #[cfg(test)]
    pub fn t6000(
        original: &'a [u8],
        prepared: &[u8],
        identity: SourceIdentity,
        boot_reg: u32,
    ) -> Result<Self> {
        let tree = DeviceTree::parse(prepared)
            .map_err(|error| refuse!("assigned source ADT: {error}", %error))?;
        let resources = BspSource::t6000(original, prepared, identity, boot_reg)?;
        Ok(Self {
            resources,
            tree,
            dense_ids: None,
            boot_reg,
            acc_harvesting: 1,
        })
    }
    /// Normal T6000 mode1 preserves original cluster-discovery order. The
    /// selected boot PE remains the BSP independently of its logical PE.
    pub fn t6000_assigned(
        original: &'a [u8],
        live: &LiveTree,
        identity: SourceIdentity,
        boot_reg: u32,
        workers: usize,
    ) -> Result<Self> {
        live.require_no_harvesting_override()?;
        let tree = DeviceTree::parse(live.bytes())
            .map_err(|error| refuse!("assigned live source ADT: {error}", %error))?;
        let resources =
            BspSource::t6000_assigned(original, live.bytes(), identity, boot_reg, 1, workers)?;
        Ok(Self {
            resources,
            tree,
            dense_ids: None,
            boot_reg,
            acc_harvesting: 1,
        })
    }
    /// Explicit source-family adapter. Checks the boot_args snapshot
    /// before selecting normal harvesting; no kernel override is interpreted
    /// as a checked source mode. Physical PMGR/error recipes stay original.
    pub fn t8112(
        original: &'a [u8],
        live: &LiveTree,
        identity: SourceIdentity,
        boot_reg: u32,
    ) -> Result<Self> {
        live.require_no_harvesting_override()?;
        let tree = DeviceTree::parse(live.bytes())
            .map_err(|error| refuse!("assigned live source ADT: {error}", %error))?;
        let immutable = DeviceTree::parse(original)
            .map_err(|error| refuse!("assigned immutable source ADT: {error}", %error))?;
        let resources = BspSource::t8112(original, live.bytes(), identity, boot_reg, 0)?;
        Ok(Self {
            resources,
            tree,
            dense_ids: Some((immutable, identity.original_tree_sha256)),
            boot_reg,
            acc_harvesting: 0,
        })
    }
    /// T6020-family mode1 adapter (M2 Pro, Max and Ultra). Its kernel binds
    /// each complex's records to CPU numbers in order, so a reduced roster
    /// takes dense CPU IDs, watchdog records and enable_core masks.
    pub fn t6020_assigned(
        original: &'a [u8],
        live: &LiveTree,
        identity: SourceIdentity,
        boot_reg: u32,
        workers: usize,
    ) -> Result<Self> {
        const HARVESTING: u32 = 1;
        live.require_no_harvesting_override()?;
        let tree = DeviceTree::parse(live.bytes())
            .map_err(|error| refuse!("assigned live source ADT: {error}", %error))?;
        let immutable = DeviceTree::parse(original)
            .map_err(|error| refuse!("assigned immutable source ADT: {error}", %error))?;
        let resources = BspSource::t6020_assigned(
            original,
            live.bytes(),
            identity,
            boot_reg,
            HARVESTING,
            workers,
        )?;
        Ok(Self {
            resources,
            tree,
            dense_ids: Some((immutable, identity.original_tree_sha256)),
            boot_reg,
            acc_harvesting: HARVESTING,
        })
    }
    pub fn acc_harvesting(&self) -> u32 {
        self.acc_harvesting
    }
    /// Full selected-source adapter for the started-worker roster.
    pub fn t8112_assigned(
        original: &'a [u8],
        live: &LiveTree,
        identity: SourceIdentity,
        boot_reg: u32,
        workers: usize,
    ) -> Result<Self> {
        let mut source = Self::t8112(original, live, identity, boot_reg)?;
        source.resources =
            BspSource::t8112_assigned(original, live.bytes(), identity, boot_reg, 0, workers)?;
        Ok(source)
    }
    #[cfg(test)]
    pub fn prepare<'s>(
        &'s self,
        roster: &'s dyn RetainedRoster,
        profile: CpuProfile,
    ) -> Result<Prepared<'s>> {
        self.prepare_with_classes(
            roster,
            profile,
            &alloc::vec![native::CoreClass::Unknown; roster.cores().len()],
            &[],
        )
    }

    pub fn prepare_on<'s>(
        &'s self,
        roster: &'s dyn RetainedRoster,
        profile: CpuProfile,
        capture: &Capture,
    ) -> Result<Prepared<'s>> {
        let classes = Self::host_classes(roster, capture)?;
        let cores = Self::host_cores(roster, capture)?;
        self.prepare_with_classes(roster, profile, &classes, &cores)
    }

    /// Each assigned worker's physical core, as `host_classes` finds its
    /// processor.
    fn host_cores(roster: &dyn RetainedRoster, capture: &Capture) -> Result<Vec<native::Location>> {
        roster
            .cores()
            .iter()
            .map(|core| {
                let index = *capture
                    .selected
                    .get(core.worker)
                    .filter(|_| core.worker < capture.core_count as usize)
                    .ok_or(refuse!("assigned worker has no captured processor"))?
                    as usize;
                capture
                    .processors
                    .get(index)
                    .map(|processor| processor.location)
                    .ok_or(refuse!("assigned worker has no captured processor"))
            })
            .collect()
    }

    fn host_classes(
        roster: &dyn RetainedRoster,
        capture: &Capture,
    ) -> Result<Vec<native::CoreClass>> {
        roster
            .cores()
            .iter()
            .map(|core| {
                let worker = core.worker;
                let index = *capture
                    .selected
                    .get(worker)
                    .filter(|_| worker < capture.core_count as usize)
                    .ok_or(refuse!("assigned worker has no captured core class"))?
                    as usize;
                let processor = capture
                    .processors
                    .get(index)
                    .ok_or(refuse!("assigned worker has no captured processor"))?;
                let snapshot = &capture.snapshots[worker];
                if u64::from(core.apic) != processor.apic_id || !snapshot.matches(*processor) {
                    return Err(refuse!(
                        "assigned core class differs from its processor identity"
                    ));
                }
                Ok(snapshot.core_class())
            })
            .collect()
    }

    fn guest_classes(&self) -> Result<Vec<native::CoreClass>> {
        let resources = self.resources.resources();
        resources
            .slots
            .iter()
            .map(|slot| {
                let kind = resources
                    .kinds
                    .iter()
                    .find(|kind| Some(kind.id) == slot.kind)
                    .ok_or(refuse!("assigned source core kind is unknown"))?;
                Ok(match kind.cluster_type {
                    b'P' => native::CoreClass::Performance,
                    b'E' => native::CoreClass::Efficiency,
                    _ => native::CoreClass::Unknown,
                })
            })
            .collect()
    }

    fn prepare_with_classes<'s>(
        &'s self,
        roster: &'s dyn RetainedRoster,
        profile: CpuProfile,
        host: &[native::CoreClass],
        cores: &[native::Location],
    ) -> Result<Prepared<'s>> {
        use stela_crypto::Sha256;
        use stela_crypto::digest::Digest;
        let resources = self.resources.resources();
        if resources.slots.len() != roster.cores().len() {
            return Err(refuse!(
                "assigned source and actual started worker cardinality differ"
            ));
        }
        let mut nodes = Vec::with_capacity(resources.slots.len());
        let mut assignments = Vec::with_capacity(resources.slots.len());
        let boot = resources
            .slots
            .iter()
            .position(|slot| slot.reg == self.boot_reg)
            .ok_or(refuse!("assigned source omits the original boot PE"))?;
        let guest = self.guest_classes()?;
        let placement = native::placement(&guest, host, boot)
            .map_err(|error| refuse!("assigned host core placement: {error}", ?error))?;
        // On a host of one core class whose workers share physical cores,
        // pair the guest's slots on them by cluster (`native::paired`).
        let placement = if host.windows(2).all(|pair| pair[0] == pair[1]) {
            native::paired(&guest, cores, boot).unwrap_or(placement)
        } else {
            placement
        };
        for (ordinal, slot) in resources.slots.iter().enumerate() {
            // Source PE order keeps PMGR's cluster ordinal coupling; only
            // the host worker assigned to each source slot changes.
            let worker = &roster.cores()[placement[ordinal]];
            let kind = resources
                .kinds
                .iter()
                .find(|kind| Some(kind.id) == slot.kind)
                .ok_or(refuse!("assigned source core kind is unknown"))?;
            let cluster = resources
                .clusters
                .iter()
                .find(|cluster| cluster.id == slot.cluster)
                .ok_or(refuse!("assigned source cluster is unknown"))?;
            nodes.push(NodeBinding {
                slot: slot.id,
                name: self.resources.name_for(slot.id)?,
                midr: kind.midr,
                cache: cluster
                    .cache
                    .ok_or(refuse!("assigned source cache is unknown"))?,
                interrupt: slot
                    .interrupt
                    .ok_or(refuse!("assigned source interrupt owner is unknown"))?,
                start: slot
                    .start
                    .ok_or(refuse!("assigned source start owner is unknown"))?,
                core_power: slot
                    .power
                    .ok_or(refuse!("assigned source power identity is unknown"))?,
                cluster_power: cluster
                    .power
                    .ok_or(refuse!("assigned source complex power identity is unknown"))?,
            });
            assignments.push(Assignment {
                worker: worker.worker,
                slot: slot.id,
                boot: ordinal == boot,
            });
        }
        let source = SourceBindings {
            source_set_sha256: resources.profile.input_set_sha256,
            tree_sha256: Sha256::digest(self.tree.serialize()).into(),
            nodes: &nodes,
            counts: &COUNT_FIELDS,
            references: &[],
            cpu_ids: match &self.dense_ids {
                Some((original, original_sha256)) => CpuIdRecipe::DenseWithWatchdog {
                    original,
                    original_sha256: *original_sha256,
                    watchdog_path: "/arm-io/wdt",
                },
                None => CpuIdRecipe::Preserve,
            },
        };
        prepare_assigned(
            roster,
            &self.resources,
            &assignments,
            &self.tree,
            &source,
            profile,
        )
    }
}

/// The Plan, projected ADT and runtime profile are one checked publication cut.
/// Parent must join its exact physical worker, AIC/start/PMGR and canonical-RAM
/// owners before copying this tree into guest memory or executing firmware.
pub struct Prepared<'a> {
    pub plan: Plan<'a>,
    pub tree: DeviceTree,
    pub profile: CpuProfile,
    pub receipt: Receipt,
    pub joins: Vec<Join>,
}
impl Prepared<'_> {
    /// Exact checked Plan-to-runtime assignment, preserving physical
    /// worker identity independently of the projected guest logical index.
    pub fn worker_bindings(
        &self,
    ) -> Result<Vec<stela_engine::machine::engine::publication::WorkerBinding>> {
        use stela_engine::machine::engine::publication::WorkerBinding;
        use stela_engine::machine::engine::publication::WorkerId;
        self.plan
            .revalidate()
            .map_err(|e| refuse!("assigned worker route is stale: {e}", ?e))?;
        Ok(self
            .plan
            .pes()
            .iter()
            .map(|pe| WorkerBinding {
                pe: pe.logical,
                worker: WorkerId(pe.host.worker),
            })
            .collect())
    }
    /// Lookup only; original error/start inputs never manufacture an assigned
    /// PE. The same owner revision must still be current at adoption.
    pub fn join(&self, reg: u32) -> Result<&Join> {
        self.plan
            .revalidate()
            .map_err(|e| refuse!("assigned route is stale: {e}", ?e))?;
        self.joins
            .iter()
            .find(|join| join.reg == reg)
            .ok_or_else(|| {
                refuse!(
                    "source PE {reg:#x} is outside the actual assigned plan",
                    reg
                )
            })
    }
}
fn word(tree: &DeviceTree, path: &str, field: &str) -> Result<u32> {
    tree.property(path, field)
        .filter(|p| !p.placeholder)
        .and_then(|p| p.as_u32())
        .ok_or_else(|| {
            refuse!(
                "assigned topology requires complete {path}/{field}",
                path,
                field
            )
        })
}
fn matching_count(actual: &CountField<'_>, expected: &CountField<'_>) -> bool {
    actual.path == expected.path
        && actual.property == expected.property
        && match (actual.count, expected.count) {
            (Count::Pes, Count::Pes) | (Count::Clusters, Count::Clusters) => true,
            (Count::CoreType(a), Count::CoreType(b)) => a == b,
            _ => false,
        }
}

/// Does not invent an inventory from the loader's synthetic Fixture. The
/// owner must supply checked kinds, slots, cache geometry,
/// interrupt/start/power identities and whole-consumer capacities. Unknown
/// ownership or initial state is not supplied by this topology operation.
#[cfg(test)]
pub fn prepare<'a>(
    roster: &'a BspRoster<'_>,
    owner: &'a dyn RetainedResources,
    slot: GuestSlotId,
    tree: &DeviceTree,
    source: &SourceBindings<'_>,
    profile: CpuProfile,
) -> Result<Prepared<'a>> {
    prepare_assigned(
        roster,
        owner,
        &[Assignment {
            worker: roster.cores[0].worker,
            slot,
            boot: true,
        }],
        tree,
        source,
        profile,
    )
}

/// Additive owner boundary for AP executors. A captured snapshot alone
/// cannot establish started workers: the caller must keep the real admitted
/// roster/resources and separately join every native publication/Host owner.
/// BspSource itself remains one-PE only and cannot supply additional resources.
pub fn prepare_assigned<'a>(
    roster: &'a dyn RetainedRoster,
    owner: &'a dyn RetainedResources,
    assignments: &[Assignment],
    tree: &DeviceTree,
    source: &SourceBindings<'_>,
    mut profile: CpuProfile,
) -> Result<Prepared<'a>> {
    for expected in &COUNT_FIELDS {
        if source
            .counts
            .iter()
            .filter(|field| matching_count(field, expected))
            .count()
            != 1
        {
            return Err(refuse!(
                "assigned topology requires the same-plan {path}/{property} count rule",
                path = expected.path,
                property = expected.property
            ));
        }
    }
    // The original producer's literal3 does not check an absent property's
    // reduced-cluster consumers. Preserve all PMGR hardware descriptors.
    if source.counts.iter().any(|field| {
        field.property == "cpu-cluster-count" && tree.property(field.path, field.property).is_none()
    }) {
        return Err(refuse!(
            "assigned topology cannot add an unqualified cpu-cluster-count"
        ));
    }
    let aic_phandle = word(tree, "/arm-io/aic", "AAPL,phandle")?;
    let pmgr_phandle = word(tree, "/arm-io/pmgr", "AAPL,phandle")?;
    let plan = Plan::build(roster, owner, assignments).map_err(|e| {
        refuse!(
            "assigned topology requires actual retained resources: {e}",
            ?e
        )
    })?;
    if profile.chip != plan.chip() {
        return Err(refuse!("assigned runtime profile and resource chip differ"));
    }
    let mut joins = Vec::with_capacity(plan.pes().len());
    for pe in plan.pes() {
        let binding: &NodeBinding<'_> = source
            .nodes
            .iter()
            .find(|b| b.slot == pe.slot)
            .ok_or(refuse!("assigned PE has no actual source CPU binding"))?;
        let path = format!("/cpus/{}", binding.name);
        let source_index = tree
            .node("/cpus")
            .and_then(|cpus| {
                cpus.children
                    .iter()
                    .position(|node| node.name() == Some(binding.name))
            })
            .ok_or(refuse!("assigned PE has no unique original CPU index"))?;
        if profile.processors.get(source_index) != Some(&pe.identity) {
            return Err(refuse!(
                "assigned PE identity differs from the actual source board profile"
            ));
        }
        let parent = word(tree, &path, "interrupt-parent")?;
        if parent == 0 || parent != aic_phandle {
            return Err(refuse!(
                "assigned CPU error input has a different actual interrupt provider"
            ));
        }
        let raw = tree
            .property(&path, "function-enable_core")
            .filter(|p| !p.placeholder && p.value.len() == 12)
            .ok_or(refuse!(
                "assigned CPU has no complete original enable_core function"
            ))?;
        let cells = raw.value.as_chunks::<4>().0;
        let function = Function {
            phandle: u32::from_le_bytes(cells[0]),
            selector: u32::from_le_bytes(cells[1]),
            argument: u32::from_le_bytes(cells[2]),
        };
        if function.phandle == 0
            || function.phandle != pmgr_phandle
            || function.selector != 0x436f7265
            || !function.argument.is_power_of_two()
        {
            return Err(refuse!("assigned CPU has an empty source start protocol"));
        }
        joins.push(Join {
            plan: plan.identity(),
            logical: pe.logical,
            worker: pe.host.worker,
            apic: pe.host.apic,
            reg: pe.reg,
            cluster: pe.cluster,
            cache: pe.cache,
            interrupt: pe.interrupt,
            error_interrupt_parent: parent,
            error_interrupt: word(tree, &path, "interrupts")?,
            start: pe.start,
            enable_core: function,
            core_power: pe.core_power,
            cluster_power: pe.cluster_power,
            cpu_impl: pe.cpu_impl,
            acc_impl: pe.acc_impl,
            cpm_impl: pe.cpm_impl,
        });
    }
    let mut projected = tree.clone();
    let receipt = Projection::prepare(&plan, &projected, source)
        .and_then(|projection| projection.apply(&mut projected))
        .map_err(|error| refuse!("assigned firmware topology: {error}", %error))?;
    profile.processors = plan.pes().iter().map(|pe| pe.identity).collect();
    plan.revalidate()
        .map_err(|e| refuse!("assigned topology became stale: {e}", ?e))?;
    Ok(Prepared {
        plan,
        tree: projected,
        profile,
        receipt,
        joins,
    })
}

#[cfg(test)]
mod tests;
