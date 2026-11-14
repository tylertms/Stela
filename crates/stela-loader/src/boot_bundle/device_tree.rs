//! Shaping the device tree: overlays, platform gaps, the boot CPU and die
//! counts.

use super::*;
use stela_contracts::{refuse, require};

pub(super) fn apply_overlay(tree: &mut DeviceTree, overlay: &[OverlayProperty]) -> Result<()> {
    for entry in overlay {
        let mut parent = String::from("/");
        for component in entry.path.split('/').filter(|c| !c.is_empty()) {
            tree.ensure_child(&parent, component)?;
            parent = format!("{}/{component}", parent.trim_end_matches('/'));
        }
        tree.set_property(&entry.path, &entry.property, entry.value.clone())?;
    }
    Ok(())
}

/// Bootloader-provided inputs SPTM reads that no restore member supplies.
pub(super) fn platform_gaps(tree: &DeviceTree, inputs: &Inputs) -> Vec<String> {
    let mut gaps = vec![];
    if tree.property(lock_regs::AMCC, "aperture-count").is_none() {
        gaps.extend(lock_regs::gaps(None));
    }
    gaps.extend(carveouts::gaps(tree, inputs.target.chip_id));
    if inputs.extra_trust_caches.is_empty() {
        gaps.push(
            "TrustCache: static module only; Cryptex1 trust caches load later through their manifest"
                .into(),
        );
    }
    gaps.push("ExclaveOSTrustCache: the identity lists no Ap,ExclaveOS component".into());
    gaps.push("SEPPatches: iBoot's SEP patch region is not in the restore image".into());
    gaps
}

pub(super) const CPU_RUNNING: &[u8] = b"running\0";
pub(super) const CPU_WAITING: &[u8] = b"waiting\0";

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct CpuIdentity {
    pub(super) path: String,
    pub(super) affinity: u32,
    pub(super) cluster_type: u8,
}

/// Original 26A434 ADTs encode cluster-type as E/P/M plus NUL, reg as u32
/// and state as running/waiting plus NUL. These are source identities, not
/// inferred CPU ordinals.
pub(super) fn cpu_identities(tree: &DeviceTree) -> Result<Vec<CpuIdentity>> {
    let cpus = tree
        .node("/cpus")
        .ok_or(refuse!("DeviceTree has no /cpus"))?;
    require!(!cpus.children.is_empty(), "DeviceTree has no CPU nodes");
    let mut identities: Vec<CpuIdentity> = Vec::new();
    for cpu in &cpus.children {
        let name = cpu.name().ok_or(refuse!("DeviceTree CPU has no name"))?;
        require!(
            !name.is_empty() && !name.contains('/') && !matches!(name, "." | ".."),
            "DeviceTree CPU name is not one child component"
        );
        let path = format!("/cpus/{name}");
        let property = |name: &str| -> Result<&crate::apple_device_tree::Property> {
            let mut matching = cpu
                .properties
                .iter()
                .filter(|property| property.name() == name);
            let property = matching
                .next()
                .ok_or_else(|| refuse!("{path} has no {name}", path, name))?;
            require!(
                matching.next().is_none(),
                "{path} has ambiguous {name} properties",
                path,
                name
            );
            require!(
                !property.placeholder,
                "{path} has no resolved {name}",
                path,
                name
            );
            Ok(property)
        };
        property("name")?;
        let cluster_type = match property("cluster-type")?.value.as_slice() {
            [kind @ (b'E' | b'P' | b'M'), 0] => *kind,
            _ => {
                return Err(
                    refuse!("{path} cluster-type is not an original E/P/M string", path).into(),
                );
            }
        };
        let affinity = property("reg")?
            .as_u32()
            .ok_or_else(|| refuse!("{path} reg is not one original u32 affinity", path))?;
        require!(
            [CPU_RUNNING, CPU_WAITING].contains(&property("state")?.value.as_slice()),
            "{path} state is not running or waiting",
            path
        );
        require!(
            !identities.iter().any(|cpu| cpu.path == path),
            "duplicate original CPU path {path}",
            path
        );
        require!(
            !identities.iter().any(|cpu| cpu.affinity == affinity),
            "duplicate original CPU affinity {affinity:#x}",
            affinity
        );
        identities.push(CpuIdentity {
            path,
            affinity,
            cluster_type,
        });
    }
    Ok(identities)
}

pub(super) fn select_boot_cpu(
    tree: &DeviceTree,
    original: &DeviceTree,
    requested: Option<&str>,
) -> Result<BootCpu> {
    let cpus = cpu_identities(tree)?;
    let source = cpu_identities(original)?;
    let (selected, selection) = match requested {
        Some(name) => {
            require!(
                !name.is_empty() && !name.contains('/') && !matches!(name, "." | ".."),
                "boot CPU override must name one original /cpus child"
            );
            let path = format!("/cpus/{name}");
            (
                cpus.iter()
                    .find(|cpu| cpu.path == path)
                    .ok_or_else(|| refuse!("DeviceTree has no boot CPU {path}", path))?,
                BootCpuSelection::Explicit,
            )
        }
        None => (
            cpus.first().ok_or(refuse!(
                "DeviceTree has no CPU for automatic boot selection"
            ))?,
            BootCpuSelection::First,
        ),
    };
    let original_index = source
        .iter()
        .position(|cpu| cpu.path == selected.path)
        .ok_or(refuse!(
            "selected boot CPU is absent from the original DeviceTree"
        ))?;
    require!(
        source[original_index] == *selected,
        "selected boot CPU identity differs from the original DeviceTree"
    );
    Ok(BootCpu {
        path: selected.path.clone(),
        affinity: selected.affinity,
        cluster_type: selected.cluster_type,
        original_index,
        selection,
    })
}

/// Preflight every state before publishing exactly one running node. The
/// original iBoot publishes running at 26A434 J314s 0x3abd4..0x3abf8. The
/// first CPU is required by the kernel's own CPU numbering: 26A434 T8112 PMGR
/// indexes its per-core register table by the topology's (cluster, core) and
/// panics for an E-core once a P-core boots first.
pub(super) fn mark_boot_cpu(tree: &mut DeviceTree, selected: &BootCpu) -> Result<()> {
    let cpus = cpu_identities(tree)?;
    require!(
        cpus.iter().any(|cpu| cpu.path == selected.path
            && cpu.affinity == selected.affinity
            && cpu.cluster_type == selected.cluster_type),
        "selected boot CPU does not match the prepared topology"
    );
    for cpu in cpus {
        let state = if cpu.path == selected.path {
            CPU_RUNNING
        } else {
            CPU_WAITING
        };
        tree.set_property(&cpu.path, "state", state.to_vec())?;
    }
    Ok(())
}

/// User overlays cannot replace the selected PE, rename/reorder CPUs or
/// change a source affinity. Existing fused-variant filtering is
/// independent of this selection and keeps original relative order.
pub(super) fn verify_boot_cpu(
    tree: &DeviceTree,
    original: &DeviceTree,
    selected: &BootCpu,
) -> Result<()> {
    let cpus = cpu_identities(tree)?;
    let source = cpu_identities(original)?;
    let original_cpu = source.get(selected.original_index).ok_or(refuse!(
        "boot CPU source index is outside the original DeviceTree"
    ))?;
    require!(
        original_cpu.path == selected.path
            && original_cpu.affinity == selected.affinity
            && original_cpu.cluster_type == selected.cluster_type,
        "boot CPU provenance differs from the original DeviceTree"
    );
    let mut previous = None;
    let mut running = Vec::new();
    for cpu in &cpus {
        let index = source
            .iter()
            .position(|original| original == cpu)
            .ok_or_else(|| {
                refuse!(
                    "{path} identity differs from the original DeviceTree",
                    path = cpu.path
                )
            })?;
        require!(
            previous.is_none_or(|previous| index > previous),
            "CPU order differs from the original DeviceTree"
        );
        previous = Some(index);
        if tree
            .property(&cpu.path, "state")
            .is_some_and(|p| p.value == CPU_RUNNING)
        {
            running.push(cpu);
        }
    }
    require!(
        running.len() == 1 && running[0].path == selected.path,
        "prepared DeviceTree must have exactly the selected boot CPU running"
    );
    if selected.selection == BootCpuSelection::First {
        require!(
            cpus.first().is_some_and(|cpu| cpu.path == selected.path),
            "automatic boot CPU is not the first original CPU"
        );
    }
    Ok(())
}

/// Board facts and boot-time slots that iBoot adds to its template tree.
pub(super) fn shape_device_tree(
    tree: &mut DeviceTree,
    target: &Target,
    options: &Options,
    boot_cpu: &BootCpu,
) -> Result<Vec<String>> {
    let mut unresolved = vec![];
    let chosen = "/chosen";
    tree.set_property(
        chosen,
        "dram-base",
        options.dram_base.to_le_bytes().to_vec(),
    )?;
    tree.set_property(
        chosen,
        "dram-size",
        options.dram_size.to_le_bytes().to_vec(),
    )?;
    for (name, value) in [("chip-id", target.chip_id), ("board-id", target.board_id)] {
        match value {
            Some(value) => tree.set_property(chosen, name, value.to_le_bytes().to_vec())?,
            None => unresolved.push(format!("/chosen {name}: absent from the restore identity")),
        }
    }
    // m1n1 load_macos_sptm creates these keys for every boot.
    tree.ensure_child(chosen, "hibernation")?;
    for key in ["key-sptm-ctrr", "key-xnu-ctrr"] {
        tree.set_property("/chosen/hibernation", key, vec![0; HIBERNATION_KEY_BYTES])?;
    }
    mark_boot_cpu(tree, boot_cpu)?;
    // iBoot adds the clock rates at hand-off (handoff.rs, J473 0x397f0..
    // 0x398f0): the boot CPU gets all six, at its 24 MHz reference.
    for cpu in cpu_nodes(tree)? {
        let boot = cpu == boot_cpu.path;
        for property in CPU_FREQUENCY_PROPERTIES {
            let hz = if property == TIMEBASE_PROPERTY {
                options.timebase_frequency
            } else {
                board::BOOT_CLOCK_HZ
            };
            tree.set_property(&cpu, property, hz.to_le_bytes().to_vec())?;
        }
        if boot {
            for property in BOOT_CPU_CLOCK_PROPERTIES {
                tree.set_property(&cpu, property, board::BOOT_CLOCK_HZ.to_le_bytes().to_vec())?;
            }
        }
    }
    unresolved.push(
        "/cpus: the clock rows on CPUs other than the boot CPU are Stela's choice; \
         whether iBoot writes them there is untraced"
            .into(),
    );
    unresolved.extend(set_die_count(tree, target.chip_id)?);
    publish_j414s_mcc_handoff(tree, target)?;
    unresolved.push(
        "/chosen/boot-object-manifests: no signed LocalPolicy or personalized manifest objects; \
         a supplied original boot manifest is forwarded separately"
            .into(),
    );
    unresolved.push("preoslog: iBoot log region omitted".into());
    unresolved.push("other rows iBoot writes stay as the template ships them or absent".into());
    Ok(unresolved)
}

/// J414S iBoot 20457.41.3 (26B123, file 0x3b1c0..0x3b254) publishes
/// these u64s on `mcc`. The IPSW tree lacks them, and the matching kernel's
/// `_initMemHashParam` requires all three. These values describe SRAM geometry.
fn publish_j414s_mcc_handoff(tree: &mut DeviceTree, target: &Target) -> Result<()> {
    const PATH: &str = "/arm-io/mcc";
    // The installed bundle may name the development Cryptex ticket instead
    // of the original board. Its DeviceTree still describes the same MCC.
    if target.chip_id != Some(0x6020)
        || !matches!(target.device_class.as_str(), "j414sap" | "j414dev")
        || target.product_type != "Mac14,9"
        || target.product_build != "26B123"
        || tree
            .property(PATH, "compatible")
            .map(|p| p.value.as_slice())
            != Some(b"mcc,t6020\0")
    {
        return Ok(());
    }
    for (name, value) in [
        ("ecs-nub-sram-base-0", 0x2_9ef0_0000u64),
        ("ecs-nub-sram-base-1", 0x22_9ef0_0000),
        ("ecs-nub-sram-size", 0x300),
    ] {
        if tree.property(PATH, name).is_none() {
            tree.set_property(PATH, name, value.to_le_bytes().to_vec())?;
        }
    }
    Ok(())
}

/// The ApChipIDs whose 26A434 iBoots carry no `die-count` row: every T8103
/// (J274, J293, J313, J456, J457), T8112 (J413, J415, J473, J493) and T8140
/// (J700) iBoot lacks the string. Every other board's iBoot writes it.
pub(super) const NO_DIE_COUNT: [u32; 3] = [0x8103, 0x8112, 0x8140];
/// The Ultras, for which iBoot writes 2: J375D/J314C/J375C compute
/// `chip == 0x6002 ? 2 : 1` (J375D 0x36454, compare 0x3c1d0), J575D/J514C/
/// J514M `chip == 0x6032 ? 2 : 1` (J575D 0x3a66c, compare 0x41a00), and the
/// T6022 boards' builds store 2 (J180D 0x39c1c, J475D 0x39bf0) where the
/// T6020/T6021 builds store 1 (J414C 0x3a634).
pub(super) const ULTRAS: [u32; 3] = [0x6002, 0x6022, 0x6032];
/// The M5 Pro, Max and Ultra share one ApChipID; their iBoot counts two dies
/// only when the die-configuration fuses at 0x2_8837_4010 read
/// `0xa050_c030`, `0xa55a_c33c` (J775D 0x520d8, J714C 0x50a5c), which is the
/// `h17d` variant whose CPUs the variant pass keeps on both dies.
pub(super) const FUSED_DIES: u32 = 0x6050;

/// iBoot's `/arm-io` `die-count` (J180D writes it beside `usbphy-frequency`
/// on the `arm-io` node, file 0x39d84..0x39d98, creating the property).
/// `ApplePMGR::start` reads it from its provider's provider's provider, the
/// `arm-io` device (mac14j KC VA 0xfffffe0009b35804..0xfffffe0009b358e0),
/// defaulting to one die (0xfffffe0009b357fc). The value is the chip's rule
/// above; the CPU nodes' dies (`die-id`, which the runtime's PMGR reader
/// counts) must agree, and give the fused count on the T6050.
pub(super) fn set_die_count(tree: &mut DeviceTree, chip_id: Option<u32>) -> Result<Option<String>> {
    let Some(chip) = chip_id else {
        return Ok(Some(
            "/arm-io die-count: no ApChipID selects iBoot's row".into(),
        ));
    };
    if NO_DIE_COUNT.contains(&chip) {
        return Ok(None);
    }
    let dies = cpu_dies(tree)?;
    let count = if chip == FUSED_DIES {
        dies
    } else {
        let count = if ULTRAS.contains(&chip) { 2 } else { 1 };
        require!(
            dies == count,
            "ApChipID {chip:#x}'s iBoot writes die-count {count}, but its CPUs are on {dies} dies",
            chip,
            count,
            dies
        );
        count
    };
    require!(tree.node("/arm-io").is_some(), "DeviceTree has no /arm-io");
    tree.set_property("/arm-io", "die-count", count.to_le_bytes().to_vec())?;
    Ok(None)
}

/// The dies the CPU nodes name: their highest `die-id` plus one.
pub(super) fn cpu_dies(tree: &DeviceTree) -> Result<u32> {
    let mut highest = 0u32;
    for cpu in cpu_nodes(tree)? {
        if let Some(die) = tree.property(&cpu, "die-id") {
            let word = die
                .value
                .first_chunk::<4>()
                .ok_or_else(|| refuse!("{cpu} die-id is not a u32", cpu))?;
            highest = highest.max(u32::from_le_bytes(*word));
        }
    }
    highest
        .checked_add(1)
        .ok_or(refuse!("CPU die-id exceeds the die count").into())
}

/// Paths of the `/cpus` children that describe processors.
pub(super) fn cpu_nodes(tree: &DeviceTree) -> Result<Vec<String>> {
    let cpus = tree
        .node("/cpus")
        .ok_or(refuse!("DeviceTree has no /cpus"))?;
    Ok(cpus
        .children
        .iter()
        .filter(|cpu| cpu.property("state").is_some())
        .filter_map(|cpu| cpu.name().map(|name| format!("/cpus/{name}")))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn j414s_mcc_sram_handoff_is_build_specific_and_keeps_source_properties() -> Result<()> {
        const PATH: &str = "/arm-io/mcc";
        for device_class in ["j414sap", "j414dev"] {
            let mut target = Target {
                product_type: "Mac14,9".into(),
                device_class: device_class.into(),
                product_version: "27.2".into(),
                product_build: "26B124".into(),
                variant: String::new(),
                chip_id: Some(0x6020),
                board_id: None,
            };
            let mut tree = DeviceTree {
                root: crate::apple_device_tree::Node::named("device-tree")?,
            };
            tree.ensure_child("/", "arm-io")?;
            tree.ensure_child("/arm-io", "mcc")?;
            tree.set_property(PATH, "compatible", b"mcc,t6020\0".to_vec())?;
            publish_j414s_mcc_handoff(&mut tree, &target)?;
            assert!(tree.property(PATH, "ecs-nub-sram-base-0").is_none());

            target.product_build = "26B123".into();
            let source_size = 0x400u64.to_le_bytes();
            tree.set_property(PATH, "ecs-nub-sram-size", source_size.to_vec())?;
            publish_j414s_mcc_handoff(&mut tree, &target)?;
            assert_eq!(
                tree.property(PATH, "ecs-nub-sram-base-0").unwrap().value,
                0x2_9ef0_0000u64.to_le_bytes()
            );
            assert_eq!(
                tree.property(PATH, "ecs-nub-sram-size").unwrap().value,
                source_size
            );
        }
        Ok(())
    }
}
