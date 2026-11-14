//! DeviceTree placement and validation for power registers.
use super::CpuCluster;
use super::Identity;
use super::Node;
use super::PMGR2_ARCH;
use super::SOCS;
use super::Vec;
use super::WakeLayout;
use super::apertures;
use super::cluster;
use super::energy;
use super::fuse;
use super::power_resources::PMGR_DEVICE;
use super::wake;
use crate::Result;
use crate::pmgr::dvfs;
use stela_contracts::refuse;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeBank {
    pub physical: u64,
    pub fields: Vec<(u64, u32)>,
    pub header: [u8; 4],
    pub configs: Vec<[u8; 36]>,
    pub axi: Option<[u8; 4]>,
    /// IDs whose `device-bridges` dependency list names this bridge.
    pub devices: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SocDpe {
    pub physical: u64,
    pub samples: Vec<(u64, bool)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PowerBank {
    pub physical: u64,
    pub kind: crate::power::Kind,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WakeBank {
    pub physical: u64,
    pub valid: [u32; wake::WORDS],
    pub sources: Vec<u8>,
    /// `ap-wake-sources`' `SMC.CPUWakeupAP`: the SMC waking the AP.
    pub smc_wake: Option<u8>,
}

/// A version-1 `perf-regs` group: translated global word and the union of
/// selectors named by the DeviceTree's counter-bearing records.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PerfBank {
    pub physical: u64,
    pub selectors: Vec<u16>,
}

/// A direct CPU energy bank, with its optional CPM enable configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnergyBank {
    pub physical: u64,
    pub cpm: bool,
}

/// Independently checked PMGR register families. A missing optional family
/// does not revoke power-state ownership or another family's register model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PmgrBank {
    CpuClusters,
    Bridges,
    SocPower,
    SocDpe,
    WakeMasks,
    Performance,
    CpuEnergy,
    Services,
    /// Source-checked protocol, awaiting native controller effects.
    Dvfs(dvfs::Controller),
}

/// A visible register family whose semantics have not been established.
/// The mount reports these through its unsupported-operation policy; no register
/// or lifecycle effect is supplied by keeping this descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PmgrBankRefusal {
    pub bank: PmgrBank,
    pub reason: &'static str,
}

impl From<dvfs::UnqualifiedEffects> for PmgrBankRefusal {
    fn from(refusal: dvfs::UnqualifiedEffects) -> Self {
        Self {
            bank: PmgrBank::Dvfs(refusal.controller),
            reason: refusal.reason(),
        }
    }
}

/// Validate advertised legacy record envelopes even when their register model
/// is unavailable. An unsupported family is not permission to discard malformed
/// data. The 26A434 v1 driver records are described by the parser below and
/// `power_resources::records`; v2 performance metadata has a separate envelope.
fn validate_bank_records(node: &Node<'_>) -> Result<()> {
    for (name, bytes) in [
        ("devices", PMGR_DEVICE),
        ("clocks", 24),
        ("events", 24),
        ("power-domains", 24),
        ("hw-dpe-reg", 40),
        ("perf-regs", 16),
        ("bridge-counters", 4),
        ("bridge-counter-configs", 36),
        ("device-bridges", 76),
        ("axi2af-axi-config", 4),
        ("optional-bridge-mask", 4),
        ("clusters", 4),
        ("ap-wake-sources", 36),
    ] {
        if node
            .property(name)
            .is_some_and(|raw| !raw.len().is_multiple_of(bytes))
        {
            return Err(refuse!("/arm-io/pmgr {name} has incomplete records", name));
        }
    }
    for name in [
        "bridge-reg-index",
        "#bridges",
        "bridge-settings-version",
        "bridge-counter-version",
    ] {
        if node.property(name).is_some_and(|raw| raw.len() != 4) {
            return Err(refuse!(
                "/arm-io/pmgr {name} is not one complete word",
                name
            ));
        }
    }
    if let Some(count) = node.first_u32("#bridges") {
        for name in ["bridge-counters", "axi2af-axi-config"] {
            if node
                .property(name)
                .is_some_and(|raw| raw.len() / 4 != count as usize)
            {
                return Err(refuse!("/arm-io/pmgr {name} disagrees with #bridges", name));
            }
        }
    }
    if node.first_u32("bridge-settings-version") == Some(1)
        && node.first_u32("bridge-counter-version") == Some(3)
    {
        let count = node.first_u32("#bridges").ok_or(refuse!(
            "PMGR has no bridge count for its known source table"
        ))?;
        super::power_resources::bridges::Records::new(node, count as usize)?;
    }
    Ok(())
}

/// The SoC's optional register families, independently of its common
/// power-state table and CPU-start bank. Driver matching comes from the SoC
/// row; each family's checked layout remains a separate requirement.
pub(super) fn pmgr_registers(
    io: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
    chip_id: u32,
) -> Result<PmgrRegisters> {
    let mut result = PmgrRegisters::default();
    let Some(soc) = SOCS.iter().find(|soc| soc.chip_id == chip_id) else {
        return Ok(result);
    };
    let Some(parent) = io.child("pmgr").filter(|node| node.compatible(soc.pmgr)) else {
        return Ok(result);
    };
    let node = super::power_resources::register_node(io, &parent)?;
    validate_bank_records(&node)?;
    // Each source view is checked in full before any optional model is kept.
    let reg: Vec<Option<(u64, u64)>> = apertures::registers(&node)
        .ok_or(refuse!("PMGR reg has incomplete physical pairs"))?
        .into_iter()
        .enumerate()
        .map(|(index, (offset, bytes))| {
            if bytes == 0 {
                Ok(None)
            } else {
                translate(offset, bytes)
                    .map(|physical| Some((physical, bytes)))
                    .ok_or_else(|| refuse!("PMGR reg[{index}] complete view {offset:#x}/{bytes:#x} is outside or ambiguous in its bus domain", index, offset, bytes))
            }
        })
        .collect::<Result<_, _>>()?;
    let mut refuse = |bank, reason| result.refusals.push(PmgrBankRefusal { bank, reason });
    let clusters = soc
        .clusters
        .map(|clusters| clusters.to_vec())
        .unwrap_or_default();
    if soc.clusters.is_none() && node.property("clusters").is_some() {
        refuse(
            PmgrBank::CpuClusters,
            "CPU cluster register layout is not qualified for this SoC",
        );
    }
    validate_clusters(&reg, &clusters)?;
    result.clusters = clusters;
    result.aon_fuses = soc
        .aon_fuses
        .map(|index| {
            reg.get(index)
                .copied()
                .flatten()
                .filter(|&(_, bytes)| bytes == fuse::FUSE_BYTES)
                .ok_or_else(|| refuse!("/arm-io/pmgr reg[{index}] is not the eFuse block", index))
        })
        .transpose()?;
    if chip_id == 0x6020 {
        // The 26B123 J414S pmgr reg property identifies the complete bank
        // roster. Other builds may use the same layout.
        const J414S_REG: &str = "26f89207f9e50f84d42962b502bf78a92b4da73f0808ae912361de56c82fd6ba";
        if node
            .property("reg")
            .is_some_and(|raw| stela_formats::sha256(raw) == J414S_REG)
        {
            // getCPUVoltageRailIDS and getVoltageRailIDS read three words here.
            result.rail_calibration = reg[1].and_then(|(base, _)| base.checked_add(0x4c1f0));
            if let Some(words) = t6020_perf_boot(&reg, soc.cores) {
                result.t6020_perf_boot = words;
            } else {
                refuse(
                    PmgrBank::Performance,
                    "J414S PMGR complex banks do not match their source layout",
                );
            }
            result.t6020_unmodeled_pmgr = t6020_unmodeled_pmgr(&reg);
        } else {
            refuse(
                PmgrBank::Performance,
                "T6020 PMGR register roster is unqualified",
            );
        }
    }
    if node.property("bridge-reg-index").is_some() && soc.bridge_fields.is_none() {
        refuse(
            PmgrBank::Bridges,
            "bridge register layout is not qualified for this SoC",
        );
    } else if node
        .first_u32("bridge-settings-version")
        .is_some_and(|v| v != 1)
        || node
            .first_u32("bridge-counter-version")
            .is_some_and(|v| v != 3)
        || node
            .property("optional-bridge-mask")
            .is_some_and(|v| v.iter().any(|&b| b != 0))
    {
        refuse(
            PmgrBank::Bridges,
            "bridge version or optional-bank selection is not qualified",
        );
    } else {
        result.bridges = place_bridges(&node, &reg, soc.bridge_fields)?;
    }
    if result.clusters.is_empty() {
        result.cluster_events = Vec::new();
    } else {
        result.cluster_events = acc_events(&node, result.clusters.len())?;
    }
    result.power_banks = place_power_banks(&reg, soc.power_banks)?;
    if soc.power_banks.is_empty() {
        refuse(
            PmgrBank::SocPower,
            "SoC-specific power register roles are not qualified",
        );
    }
    result.soc_dpe = if let Some(index) = soc.dpe_reg {
        Some(place_soc_dpe(&node, &reg, index)?)
    } else {
        if node.property("soc-dpe").is_some() {
            refuse(PmgrBank::SocDpe, "SoC DPE register layout is not qualified");
        }
        None
    };
    // Without the cluster layout, an absolute DPE address cannot be classified
    // as a CPM accumulator because it lies inside a broad reg window.
    if result.clusters.is_empty() && node.property("hw-dpe-reg").is_some() {
        refuse(
            PmgrBank::CpuEnergy,
            "CPU/CPM energy register ownership is not qualified",
        );
    } else {
        result.energy = energy_banks(&node, &reg, &result.clusters)?;
    }
    let unsupported_perf = node.property("perf-groups").is_some()
        || node.property("perf-regs").is_some_and(|raw| {
            raw.as_chunks::<16>()
                .0
                .iter()
                .any(|entry| !matches!(entry[12], 0 | 3) || entry[13] > 1 || entry[15] != 0)
        });
    if unsupported_perf {
        refuse(
            PmgrBank::Performance,
            "performance counter class, layout or flags are not qualified",
        );
    } else {
        result.performance = performance_banks(&node, &reg)?;
    }
    result.wake_masks = if let Some(layout) = soc.wake_masks {
        Some(wake_bank(&node, &reg, layout)?)
    } else {
        if node.property("ap-wake-sources").is_some() {
            refuse(
                PmgrBank::WakeMasks,
                "wake-mask register layout is not qualified",
            );
        }
        None
    };
    if parent.compatible(PMGR2_ARCH) {
        refuse(
            PmgrBank::Services,
            "PMGRv2 PSD service-gate command interface is not qualified",
        );
    }
    Ok(result)
}

/// J414S has one E and two P complexes. Each maps a command/status bank and
/// a separate table. XNU 26B123 `cpuComplexIdleEnter` polls bit 31 of the
/// command word; `getDVMRMaxWeight` reads bits 41:38 of a 0x40-byte table row.
fn t6020_perf_boot(reg: &[Option<(u64, u64)>], cores: &[super::CoreType]) -> Option<Vec<u64>> {
    let mut words = Vec::new();
    for (cluster, (command, table)) in [(8, 12), (17, 21), (26, 30)].into_iter().enumerate() {
        let Some((base, 0x15000)) = reg.get(command).copied().flatten() else {
            return None;
        };
        let Some((table, 0x90000)) = reg.get(table).copied().flatten() else {
            return None;
        };
        if base.checked_add(0x5_0000) != Some(table) {
            return None;
        }
        let first = if cluster == 0 { 2 } else { 1 };
        let points = cores.get(usize::from(cluster != 0))?.operating_points?;
        words.push(base + 0x20);
        // getTunedDVMRWeightType's E immediate and P lookup table. The
        // frontend has no physical DVMR weight tuning to hand off.
        let tuning: &[u64] = if cluster == 0 {
            &[0x6a8]
        } else {
            &[0x6a8, 0x6b0, 0x6e0, 0x6e8]
        };
        words.extend(tuning.iter().map(|offset| base + offset));
        for row in first..first + points.len() as u64 {
            words.push(table + 0x8008 + row * 0x40);
            // getFANEIndex reads word 2 of P-complex rows. There is no
            // native ANE clock selection in the cacheless frontend.
            if cluster != 0 {
                words.push(table + 0x8010 + row * 0x40);
            }
        }
    }
    Some(words)
}

/// The J414S 26B123 PMGR driver accesses these pages through its own reg
/// maps while programming counters, bridge tunables and CPU power settings.
/// This describes only bus ownership for the unsupported remainder. Keep
/// other pages unmapped; this does not supply effects for any register.
fn t6020_unmodeled_pmgr(reg: &[Option<(u64, u64)>]) -> Vec<(u64, u64)> {
    const PAGES: &[(usize, &[u64])] = &[
        (0, &[0x8, 0xc, 0x38, 0x4c, 0x60, 0x61, 0x68, 0x69, 0x6a]),
        (1, &[0x8, 0x48]),
        (2, &[0x22, 0x2c, 0x30]),
        (4, &[0x70]),
        (5, &[0x40]),
        (7, &[0]),
        (8, &[0]),
        (10, &[4, 8]),
        (17, &[0]),
        (19, &[4, 8]),
        (21, &[8]),
        (26, &[0]),
        (28, &[4, 8]),
        (30, &[8]),
        (40, &[0]),
        (41, &[2]),
        (43, &[0, 2, 3, 4]),
        (44, &[0x10]),
        (54, &[0]),
        (55, &[0]),
        (56, &[0]),
        (57, &[0]),
        (59, &[0]),
        // J414S 26B123 reads the first word while starting the USB controllers.
        (60, &[0]),
        (61, &[0]),
        (63, &[0]),
        (64, &[0]),
        (69, &[0]),
        (70, &[0]),
        (71, &[0]),
    ];
    let mut result = Vec::new();
    for &(index, pages) in PAGES {
        let Some((base, bytes)) = reg.get(index).copied().flatten() else {
            continue;
        };
        for &page in pages {
            let offset = page << 12;
            if offset + 0x1000 <= bytes
                && base.is_multiple_of(0x1000)
                && let Some(physical) = base.checked_add(offset)
            {
                result.push((physical, 0x1000));
            }
        }
    }
    // J414S hw-dpe-reg locates two 64-bit energy words per core.
    for index in (13..=16).chain(22..=25).chain(31..=34) {
        if let Some((base, energy::CORE_BYTES)) = reg[index] {
            result.push((base + energy::CORE_OFFSET, energy::COUNTER_BYTES));
        }
    }
    result.sort_unstable();
    result.dedup();
    result
}

fn validate_clusters(reg: &[Option<(u64, u64)>], clusters: &[CpuCluster]) -> Result<()> {
    let covered = |start: u64, bytes: u64| {
        reg.iter().flatten().any(|&(base, size)| {
            start
                .checked_sub(base)
                .and_then(|offset| offset.checked_add(bytes))
                .is_some_and(|end| end <= size)
        })
    };
    for cluster in clusters {
        let cpm_base = cluster
            .base
            .checked_add(crate::cpm::T8112_OFFSET)
            .ok_or(refuse!("CPM base overflows"))?;
        for offset in crate::cpm::offsets(cluster.counter_map) {
            if !covered(cpm_base + offset, 8) {
                return Err(refuse!("PMGR reg does not cover a CPM power register"));
            }
        }
        for offset in cluster::SCRATCH {
            if !covered(cluster.base + offset, 8) {
                return Err(refuse!("PMGR reg does not cover CPU state scratch"));
            }
        }
        let window = cluster.base + cluster::ACC_OFFSET;
        let table = cluster.pstates as u64 * cluster::PSTATE_BYTES;
        if !reg.contains(&Some((window, cluster::ACC_BYTES)))
            || !covered(cluster.base + cluster::PSTATE_OFFSET, table)
        {
            return Err(refuse!(
                "/arm-io/pmgr reg does not place the CPU cluster at {base:#x}",
                base = cluster.base
            ));
        }
    }
    Ok(())
}

/// The CPU start registers' guest PA and the clusters they start, where the
/// SoC row gives their offset ([`super::soc::rows::CPU_START_T8112`] and its siblings) from
/// `reg[0]` of the PMGR node: `/arm-io/pmgr`, or `/arm-io/pmgr-child` where
/// `/arm-io/pmgr` is the generic `pmgr2,arch` (m1n1 `pmgr.c`
/// `pmgr_adt_path_offset_trace`; the 26A434 ApplePMGR "Looked up under
/// 'pmgr' and 'pmgr-child'"), when `/arm-io/pmgr` is the row's `pmgr`. The
/// registers must lie in that `reg[0]` with a start word for each cluster
/// of the board's PEs.
pub(super) fn cpu_start(
    io: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
    chip_id: u32,
    processors: &[Identity],
) -> Result<Option<(u64, u8)>> {
    let Some(soc) = SOCS.iter().find(|soc| soc.chip_id == chip_id) else {
        return Ok(None);
    };
    let (Some(offset), Some(pmgr)) = (soc.cpu_start, io.child("pmgr")) else {
        return Ok(None);
    };
    if !pmgr.compatible(soc.pmgr) {
        return Ok(None);
    }
    let (name, node) = if pmgr.compatible(PMGR2_ARCH) {
        let child = io.child("pmgr-child").ok_or(refuse!(
            "/arm-io/pmgr is pmgr2,arch without /arm-io/pmgr-child"
        ))?;
        ("pmgr-child", child)
    } else {
        ("pmgr", pmgr)
    };
    let (base, bytes) = apertures::registers(&node)
        .and_then(|reg| reg.first().copied())
        .ok_or_else(|| refuse!("/arm-io/{name} has no reg", name))?;
    let clusters = processors
        .iter()
        .map(|pe| ((pe.mpidr >> 8) & 0xff) as u8 + 1)
        .max()
        .unwrap_or(0);
    if offset + crate::cpustart::bytes(clusters) > bytes {
        return Err(refuse!(
            "/arm-io/{name} reg[0] does not hold the CPU start registers",
            name
        ));
    }
    let physical = translate(base + offset, crate::cpustart::bytes(clusters))
        .ok_or_else(|| refuse!("/arm-io/{name} reg[0] is outside /arm-io ranges", name))?;
    Ok(Some((physical, clusters)))
}

/// Special CPU event types2/7 select E/P ACC accessors, not the SOC-bank
/// selector formula (kernel 0x2b93ac4..0x2b93cec). Byte6 is the event index.
fn acc_events(node: &Node<'_>, clusters: usize) -> Result<Vec<Vec<u8>>> {
    let mut events = alloc::vec![Vec::new();clusters];
    let data = node.property("events").unwrap_or_default();
    if !data.len().is_multiple_of(24) {
        return Err(refuse!("ACC events property is not whole records"));
    }
    for entry in data.as_chunks::<24>().0 {
        let cluster = match entry[2] {
            2 => 0,
            7 => 1,
            _ => continue,
        };
        let bank = events
            .get_mut(cluster)
            .ok_or(refuse!("ACC event names a missing CPU cluster"))?;
        if entry[6] >= 27 {
            return Err(refuse!("ACC event selector exceeds the accessor range"));
        }
        if !bank.contains(&entry[6]) {
            bank.push(entry[6]);
        }
    }
    Ok(events)
}

/// Bridge map and source decoding: kernel file 0x2b71bd8..0x2b71fd8,
/// device dependencies 0x2b73c80 and version3 counter programmer 0x2b96214.
/// `reg` always passes through arm-io ranges, including high PCIe windows.
fn place_bridges(
    node: &Node<'_>,
    reg: &[Option<(u64, u64)>],
    fields: Option<&[&[(u64, u32)]]>,
) -> Result<Vec<BridgeBank>> {
    let Some(first) = node.first_u32("bridge-reg-index") else {
        return Ok(Vec::new());
    };
    let fields = fields.ok_or(refuse!(
        "this SoC has no documented bridge register layouts"
    ))?;
    let count = node
        .first_u32("#bridges")
        .ok_or(refuse!("PMGR has no bridge count"))? as usize;
    if count > fields.len()
        || node.first_u32("bridge-settings-version") != Some(1)
        || node.first_u32("bridge-counter-version") != Some(3)
    {
        return Err(refuse!("unsupported bridge count or configuration version"));
    }
    let records = super::power_resources::bridges::Records::new(node, count)?;
    let mut banks = Vec::new();
    let optional = node.first_u32("optional-bridge-mask").unwrap_or(0);
    if optional != 0 {
        return Err(refuse!(
            "optional bridge suppression requires an established selection policy"
        ));
    }
    for (id, tunables) in fields.iter().enumerate().take(count) {
        let index = (first as usize)
            .checked_add(id)
            .ok_or(refuse!("bridge reg index overflows"))?;
        let (physical, bytes) = reg
            .get(index)
            .copied()
            .flatten()
            .ok_or(refuse!("bridge names an unmapped DT reg window"))?;
        let record = records.record(id).expect("validated bridge ordinal");
        let model =
            crate::bridge::Layout::new(tunables, record.header, &record.configs, record.axi)?;
        for (o, n) in model.spans() {
            if o.checked_add(n).is_none_or(|end| end > bytes)
                || physical.checked_add(o + n).is_none()
            {
                return Err(refuse!("bridge configuration lies outside its DT range"));
            }
        }
        banks.push(BridgeBank {
            physical,
            fields: tunables.to_vec(),
            header: record.header,
            configs: record.configs,
            axi: record.axi,
            devices: record.devices,
        });
    }
    Ok(banks)
}

/// SOC DPE direct split-word records have group 0x1d, die mask 1,
/// command flag 0, split flag 1 and width 64 (PMGR parser 0x2b7796c).
fn place_soc_dpe(node: &Node<'_>, reg: &[Option<(u64, u64)>], index: usize) -> Result<SocDpe> {
    let (physical, bytes) = reg
        .get(index)
        .copied()
        .flatten()
        .ok_or(refuse!("SOC DPE has no mapped DT reg bank"))?;
    let data = node.property("hw-dpe-reg").unwrap_or_default();
    if !data.len().is_multiple_of(40) {
        return Err(refuse!("SOC DPE hw-dpe-reg has partial records"));
    }
    let mut samples = Vec::new();
    for entry in data.as_chunks::<40>().0 {
        if entry[16..18] != [0x1d, 0] || entry[19] != 0 {
            continue;
        }
        if entry[18] != 1 || entry[20] != 1 || entry[23] != 64 {
            return Err(refuse!("unsupported SOC DPE sampling flags"));
        }
        let address = u64::from_le_bytes(entry[..8].try_into().expect("8 bytes"));
        let offset = address
            .checked_sub(physical)
            .ok_or(refuse!("SOC DPE sample below its reg bank"))?;
        samples.push((offset, crate::dpe::RESTORE.contains(&offset)));
    }
    let model = crate::dpe::Layout::new(&samples)?;
    for (o, n) in model.spans() {
        if o.checked_add(n).is_none_or(|end| end > bytes) || physical.checked_add(o + n).is_none() {
            return Err(refuse!("SOC DPE register outside its DT bank"));
        }
    }
    Ok(SocDpe { physical, samples })
}

/// Bind documented RegMap roles to DT addresses, validating every modeled
/// word against the selected range rather than claiming a broad PMGR bank.
fn place_power_banks(
    reg: &[Option<(u64, u64)>],
    layouts: &[(usize, crate::power::Kind)],
) -> Result<Vec<PowerBank>> {
    layouts
        .iter()
        .map(|&(index, kind)| {
            let (physical, bytes) = reg
                .get(index)
                .copied()
                .flatten()
                .ok_or(refuse!("PMGR power bank names an unmapped reg entry"))?;
            for offset in kind.offsets() {
                if offset.checked_add(4).is_none_or(|end| end > bytes)
                    || physical.checked_add(offset + 4).is_none()
                    || !physical.is_multiple_of(4)
                {
                    return Err(refuse!("PMGR power register lies outside its DT range"));
                }
            }
            Ok(PowerBank { physical, kind })
        })
        .collect()
}

/// `ap-wake-sources` is (u32 source, 32-byte name), kernel parser
/// `ApplePMGR` and `setAPWakeupSources` 0x2fa3f84. Reported sources are
/// validated against hardware capabilities, not used as a reset mask.
fn wake_bank(node: &Node<'_>, reg: &[Option<(u64, u64)>], layout: WakeLayout) -> Result<WakeBank> {
    const RECORD: usize = 36;
    let (base, bytes) = reg
        .get(layout.reg)
        .copied()
        .flatten()
        .ok_or(refuse!("PMGR AOP wake masks have no mapped reg bank"))?;
    if bytes < wake::OFFSET + wake::BYTES {
        return Err(refuse!("PMGR AOP wake masks exceed their reg bank"));
    }
    base.checked_add(wake::OFFSET + wake::BYTES)
        .ok_or(refuse!("PMGR AOP wake mask span overflows"))?;
    let physical = base + wake::OFFSET;
    let data = node.property("ap-wake-sources").unwrap_or_default();
    if !data.len().is_multiple_of(RECORD) {
        return Err(refuse!("PMGR ap-wake-sources is not whole records"));
    }
    let mut sources = Vec::new();
    let mut smc_wake = None;
    for entry in data.as_chunks::<RECORD>().0 {
        let source = u32::from_le_bytes(entry[..4].try_into().expect("4 bytes"));
        if entry[4..].split(|&b| b == 0).next() == Some(SMC_WAKE_SOURCE) {
            smc_wake = u8::try_from(source).ok();
        }
        let id = u8::try_from(source)
            .map_err(|_| refuse!("PMGR wake source exceeds hardware mask width"))?;
        if layout.valid[source as usize / u32::BITS as usize] & (1 << (source % u32::BITS)) == 0
            || sources.contains(&id)
        {
            return Err(refuse!(
                "PMGR ap-wake-sources contains an unsupported or duplicate source"
            ));
        }
        sources.push(id);
    }
    Ok(WakeBank {
        physical,
        valid: layout.valid,
        sources,
        smc_wake,
    })
}

/// The `ap-wake-sources` name of the SMC's wake of the AP.
const SMC_WAKE_SOURCE: &[u8] = b"SMC.CPUWakeupAP";

/// Version-1 group layout: `_initPerfCountersV1` (kernel file 0x2b79ec0),
/// `_getPerfCounterOffset` (0x2b8e974) and the selector consumers
/// (0x2b92ddc..0x2b93010, 0x2b95884..0x2b95964). No physical address or
/// counter index is inferred from a reporting name or device ID.
fn performance_banks(node: &Node<'_>, reg: &[Option<(u64, u64)>]) -> Result<Vec<PerfBank>> {
    use crate::perf::{COUNTER_BASE, STRIDE};
    const GROUP: usize = 16;
    const SOURCE: usize = 24;
    const DEVICE_COUNTER: u8 = 0x20;
    let data = node.property("perf-regs").unwrap_or_default();
    if !data.len().is_multiple_of(GROUP) {
        return Err(refuse!("/arm-io/pmgr perf-regs is not whole groups"));
    }
    let mut banks = Vec::new();
    let mut counts = Vec::new();
    for entry in data.as_chunks::<GROUP>().0 {
        let index = u32::from_le_bytes(entry[..4].try_into().expect("4 bytes")) as usize;
        let offset = u64::from(u32::from_le_bytes(entry[4..8].try_into().expect("4 bytes")));
        let count = u16::from_le_bytes(entry[8..10].try_into().expect("2 bytes"));
        // Version zero defaults to one. Parent/global indirection and the
        // busy protocol of classes 1/2 need separate semantics.
        if !matches!(entry[12], 0 | 3) || entry[13] > 1 || entry[15] != 0 {
            return Err(refuse!(
                "/arm-io/pmgr perf-regs has an unsupported class, layout or flags"
            ));
        }
        let (base, bytes) = reg
            .get(index)
            .copied()
            .flatten()
            .ok_or(refuse!("/arm-io/pmgr perf-regs names an unmapped reg bank"))?;
        if count == 0
            || !offset.is_multiple_of(4)
            || offset + COUNTER_BASE + u64::from(count) * STRIDE > bytes
        {
            return Err(refuse!(
                "/arm-io/pmgr perf-regs counters are empty or exceed their reg bank"
            ));
        }
        base.checked_add(offset + COUNTER_BASE + u64::from(count) * STRIDE)
            .ok_or(refuse!("PMGR performance bank span overflows"))?;
        let physical = base + offset;
        banks.push(PerfBank {
            physical,
            selectors: Vec::new(),
        });
        counts.push(count);
    }
    if banks.is_empty() {
        return Ok(banks);
    }
    let mut select = |group: u8, index: u8| -> Result<()> {
        let bank = banks.get_mut(usize::from(group)).ok_or(refuse!(
            "PMGR counter source names a missing perf-regs group"
        ))?;
        let index = u16::from(index);
        if index >= counts[usize::from(group)] {
            return Err(refuse!("PMGR counter source exceeds its perf-regs count"));
        }
        if !bank.selectors.contains(&index) {
            bank.selectors.push(index);
        }
        Ok(())
    };
    let devices = node.property("devices").unwrap_or_default();
    if !devices.len().is_multiple_of(PMGR_DEVICE) {
        return Err(refuse!("/arm-io/pmgr devices is not whole entries"));
    }
    for entry in devices.as_chunks::<PMGR_DEVICE>().0 {
        if entry[0] & DEVICE_COUNTER != 0 {
            select(entry[9], entry[8])?;
        }
    }
    for property in ["clocks", "power-domains", "events"] {
        let data = node.property(property).unwrap_or_default();
        if !data.len().is_multiple_of(SOURCE) {
            return Err(refuse!(
                "/arm-io/pmgr {property} is not whole counter records",
                property
            ));
        }
        for entry in data.as_chunks::<SOURCE>().0 {
            match property {
                "clocks" if entry[2] & 1 != 0 => select(entry[1], entry[0])?,
                "power-domains" if (entry[2], entry[1]) != (0, 0) => select(entry[2], entry[1])?,
                // Types 2/7 use the ACC accessor, and flag bit 0 selects
                // another event path; neither authorizes a SOC selector.
                "events" if entry[1] & 1 == 0 => match entry[2] {
                    0 => select(entry[7], entry[6])?,
                    1 | 5 | 6 => {
                        select(entry[7], entry[6])?;
                        select(entry[5], entry[4])?;
                    }
                    2 | 7 => (),
                    _ => return Err(refuse!("PMGR events has an unsupported counter type")),
                },
                _ => (),
            }
        }
    }
    for bank in &mut banks {
        bank.selectors.sort_unstable();
    }
    Ok(banks)
}

/// `hw-dpe-reg` records are 40 bytes (`ApplePMGR` parser, kernel file
/// 0x2b7796c..0x2b77af8). The first u64 is an absolute PA, not an arm-io
/// bus address. Direct, single-load, 64-bit CPU records have group 0,
/// die mask 1, command/split flags 0 and behavior flags 1/0. Other groups
/// (GPU/PMP, ANE, SOC DPE) belong to separate models, not CPU accumulators.
fn energy_banks(
    node: &Node<'_>,
    reg: &[Option<(u64, u64)>],
    clusters: &[CpuCluster],
) -> Result<Vec<EnergyBank>> {
    const RECORD: usize = 40;
    let data = node.property("hw-dpe-reg").unwrap_or_default();
    if !data.len().is_multiple_of(RECORD) {
        return Err(refuse!("/arm-io/pmgr hw-dpe-reg is not whole records"));
    }
    let mut banks = Vec::new();
    for entry in data.as_chunks::<RECORD>().0 {
        if entry[16..18] != [0, 0] || entry[19] != 0 {
            continue;
        }
        if entry[18..24] != [1, 0, 0, 1, 0, 64] {
            return Err(refuse!(
                "PMGR CPU hw-dpe-reg has unsupported die, sampling, behavior or width flags"
            ));
        }
        let physical = u64::from_le_bytes(entry[..8].try_into().expect("8 bytes"));
        let cpm = clusters
            .iter()
            .any(|c| c.base.checked_add(energy::CPM_OFFSET) == Some(physical));
        let bytes = if cpm {
            energy::CONTROL_OFFSET + 8
        } else {
            energy::COUNTER_BYTES
        };
        let covered = reg.iter().flatten().any(|&(base, size)| {
            base <= physical
                && physical
                    .checked_add(bytes)
                    .zip(base.checked_add(size))
                    .is_some_and(|(end, limit)| end <= limit)
                && (cpm
                    || size == energy::CORE_BYTES
                        && base.checked_add(energy::CORE_OFFSET) == Some(physical))
        });
        if !physical.is_multiple_of(8)
            || !covered
            || banks.iter().any(|b: &EnergyBank| b.physical == physical)
        {
            return Err(refuse!(
                "/arm-io/pmgr CPU hw-dpe-reg PA {physical:#x} is duplicated or outside its reg bank",
                physical
            ));
        }
        banks.push(EnergyBank { physical, cpm });
    }
    Ok(banks)
}

/// The PMGR registers the machine models besides its power states.
#[derive(Default)]
pub(super) struct PmgrRegisters {
    pub(super) refusals: Vec<PmgrBankRefusal>,
    pub(super) cluster_events: Vec<Vec<u8>>,
    pub(super) bridges: Vec<BridgeBank>,
    pub(super) soc_dpe: Option<SocDpe>,
    pub(super) power_banks: Vec<PowerBank>,
    pub(super) wake_masks: Option<WakeBank>,
    pub(super) performance: Vec<PerfBank>,
    pub(super) energy: Vec<EnergyBank>,
    pub(super) clusters: Vec<CpuCluster>,
    pub(super) aon_fuses: Option<(u64, u64)>,
    pub(super) rail_calibration: Option<u64>,
    pub(super) t6020_perf_boot: Vec<u64>,
    pub(super) t6020_unmodeled_pmgr: Vec<(u64, u64)>,
}

#[cfg(test)]
mod tests;
