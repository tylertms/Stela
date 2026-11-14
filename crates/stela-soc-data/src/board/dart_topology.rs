//! Generation-specific original DART topology and native placement gates.
use super::{DartInstance, IOMMU_MAPPER, Iommu, Node, name_of, power_resources};
use crate::Result;
use crate::dart::Profile;
use alloc::{format, vec::Vec};
use stela_contracts::refusal::Within;
use stela_contracts::refuse;

/// Every visible DART requirement, preserving separate translation windows.
/// Modern t8110 and checked T6000 paired capture keep distinct cacheless
/// owners; FPAD and unchecked DMA routing remain explicit requirements.
pub(super) fn iommus(
    io: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
) -> Result<Vec<Iommu>> {
    let mut found = Vec::new();
    let mut failure = None;
    super::apertures::visit(io, translate, &mut |node, translate| {
        if failure.is_some() || !is_dart(node) {
            return;
        }
        match iommu(node, translate) {
            Ok(iommu) => found.push(iommu),
            Err(error) => failure = Some(error),
        }
    })
    .ok_or(refuse!("DeviceTree walk failed"))?;
    match failure {
        Some(error) => Err(error),
        None => {
            let mut handles = Vec::new();
            for bank in &found {
                for &(handle, _) in &bank.mappers {
                    if handles.contains(&handle) {
                        return Err(refuse!(
                            "DART mapper phandle {handle:#x} has ambiguous native DMA routing",
                            handle
                        ));
                    }
                    handles.push(handle);
                }
            }
            Ok(found)
        }
    }
}

pub(super) fn iommu(node: &Node<'_>, translate: &dyn Fn(u64, u64) -> Option<u64>) -> Result<Iommu> {
    let path = format!("/arm-io/{}", name_of(node));
    let mut profile = node
        .property("compatible")
        .and_then(Profile::from_compatible)
        .ok_or_else(|| {
            refuse!(
                "{path} requires an unsupported or conflicting native DART generation",
                path
            )
        })?;
    if !profile.is_t8110() {
        legacy_requirements(profile, |key| node.property(key))
            .within(|| refuse!("{path}", path))?;
    }
    let reg = node.pairs("reg").unwrap_or_default();
    if node
        .property("reg")
        .is_none_or(|bytes| bytes.len() != reg.len() * 16)
    {
        return Err(refuse!("{path} has incomplete reg pairs", path));
    }
    let descriptors = node.property("instance").unwrap_or_default();
    let descriptor_bytes = if profile.is_t8110() { 16 } else { 12 };
    if descriptors.is_empty()
        || !descriptors.len().is_multiple_of(descriptor_bytes)
        || descriptors.len() / descriptor_bytes != reg.len()
    {
        return Err(refuse!(
            "{path} instance descriptors do not match its reg entries",
            path
        ));
    }
    let mut instances = Vec::new();
    let flush_by_dva = node
        .property("flush-by-dva")
        .is_some_and(|bytes| bytes.len() == 4);
    let vm_base = match node.property("vm-base") {
        None => 0,
        Some(bytes) if !profile.is_t8110() => u64::from(u32::from_le_bytes(
            bytes.try_into().expect("validated legacy vm-base"),
        )),
        Some(bytes) => u64::from_le_bytes(
            bytes
                .try_into()
                .map_err(|_| refuse!("{path} vm-base is not one 64-bit address", path))?,
        ),
    };
    let mut apf = None;
    let mut smmu = None;
    for (reg_index, (descriptor, &(offset, bytes))) in descriptors
        .chunks_exact(descriptor_bytes)
        .zip(&reg)
        .enumerate()
    {
        let (number, owner_start) = if profile.is_t8110() {
            (
                u32::from_le_bytes(descriptor[4..8].try_into().expect("instance number")),
                8,
            )
        } else {
            (instances.len() as u32, 4)
        };
        match &descriptor[..4] {
            b"TRAD" if apf.is_none() && smmu.is_none() && number as usize == instances.len() => {
                // The window selects the register file's revision; every
                // instance of one node shares it (J700AP `dart-ane` mixes
                // 0xc000- and 0x20000-byte windows of revision 2.2).
                let (physical, revision) = translate(offset, bytes)
                    .filter(|at| at.is_multiple_of(crate::dart::DAPF_BYTES))
                    .zip(profile.for_device_window(bytes, flush_by_dva, vm_base))
                    .ok_or_else(|| refuse!("{path} DART instance is not one translated aligned register window of a modelled revision", path))?;
                if !instances.is_empty() && revision != profile {
                    return Err(refuse!("{path} mixes DART register-file revisions", path));
                }
                profile = revision;
                if instances
                    .iter()
                    .any(|i: &DartInstance| i.physical == physical)
                {
                    return Err(refuse!("{path} has duplicate DART register windows", path));
                }
                instances.push(DartInstance {
                    physical,
                    bytes,
                    number,
                    reg_index,
                    owner: descriptor[owner_start..owner_start + 8]
                        .try_into()
                        .expect("eight-byte owner"),
                });
            }
            b"FPAD" if instances.len() == 1 && number == 0 && apf.is_none() && smmu.is_none() => {
                apf = Some((offset, bytes))
            }
            b"UMMS"
                if instances.len() == 1
                    && (!profile.is_t8110() || number == 0)
                    && apf.is_none()
                    && smmu.is_none() =>
            {
                let physical = translate(offset, bytes)
                    .filter(|at| {
                        at.is_multiple_of(crate::dart::DAPF_BYTES)
                            && bytes == crate::dart::DAPF_BYTES
                    })
                    .ok_or_else(|| {
                        refuse!(
                            "{path} SMMU instance is not one translated aligned 16 KiB block",
                            path
                        )
                    })?;
                smmu = Some((physical, bytes));
            }
            _ => {
                return Err(refuse!(
                    "{path} has unsupported instance kinds, numbering or APF topology",
                    path
                ));
            }
        }
    }
    if instances.is_empty() {
        return Err(refuse!("{path} has no TRAD instance", path));
    }
    let apf = match apf {
        None => None,
        Some((filter, filter_bytes)) => {
            let at = translate(filter, crate::dart::DAPF_BYTES)
                .filter(|at| {
                    at.is_multiple_of(crate::dart::DAPF_BYTES)
                        && filter_bytes == crate::dart::DAPF_BYTES
                })
                .ok_or_else(|| refuse!("{path} APF is not one aligned 16 KiB block", path))?;
            let slices = node
                .first_u32("n-apfs-instance-0")
                .and_then(|count| usize::try_from(count).ok())
                .filter(|&count| count > 0 && count as u64 * 0x40 <= crate::dart::DAPF_BYTES)
                .ok_or_else(|| refuse!("{path} has no n-apfs-instance-0 slice count", path))?;
            Some((at, slices))
        }
    };
    // Every generation's SPTM defaults an absent sid-count to 16 (T6020 VA
    // 0xfffffff0270d8184..0xfffffff0270d81f8; J414cAP `dart-sio` has none).
    let streams = node
        .first_u32("sid-count")
        .or(Some(16))
        .and_then(|count| u16::try_from(count).ok())
        .filter(|&count| count > 0 && usize::from(count) <= profile.stream_limit())
        .ok_or_else(|| {
            refuse!(
                "{path} sid-count exceeds the selected native DART profile",
                path
            )
        })?;
    let irq = node
        .first_u32("interrupts")
        .filter(|irq| u16::try_from(*irq).is_ok())
        .ok_or_else(|| {
            refuse!(
                "{path} requires an interrupt within the native AIC input encoding",
                path
            )
        })?;
    let clock_gates = power_resources::gate_ids(node, &path, "clock-gates")?;
    let power_gates = power_resources::gate_ids(node, &path, "power-gates")?;
    let retention = node.property("retention").is_some();
    if retention && node.property("no-sleep").is_some() {
        return Err(refuse!(
            "{path} retention and no-sleep are mutually exclusive",
            path
        ));
    }
    let mut mappers = Vec::new();
    let mut mapper_failure = None;
    node.visit(&mut |child| {
        if child.compatible(IOMMU_MAPPER) {
            let word = |key| {
                child
                    .property(key)
                    .and_then(|bytes| <[u8; 4]>::try_from(bytes).ok().map(u32::from_le_bytes))
            };
            match (word("AAPL,phandle"), word("reg")) {
                (Some(handle), Some(sid)) => mappers.push((handle, sid)),
                _ => {
                    mapper_failure = Some(refuse!(
                        "{path} mapper requires one native phandle and SID word",
                        path
                    ))
                }
            }
        }
    })
    .ok_or(refuse!("DeviceTree walk failed"))?;
    if let Some(error) = mapper_failure {
        return Err(error);
    }
    let sids = if profile.is_t8110() {
        match node.property("sid") {
            None => Vec::new(),
            Some(bytes) if bytes.len().is_multiple_of(4) => node
                .cells32("sid")
                .filter(|sids| sids.iter().all(|sid| *sid < u32::from(streams)))
                .ok_or_else(|| refuse!("{path} has malformed or out-of-range sid entries", path))?,
            Some(_) => return Err(refuse!("{path} has incomplete sid cells", path)),
        }
    } else {
        match node.property("sids") {
            None => Vec::new(),
            Some(bytes) => {
                let bitmap =
                    u32::from_le_bytes(bytes.try_into().expect("validated legacy SID bitmap"));
                (0..u32::from(streams))
                    .filter(|sid| bitmap & (1 << sid) != 0)
                    .collect()
            }
        }
    };
    if sids
        .iter()
        .enumerate()
        .any(|(index, sid)| sids[..index].contains(sid))
    {
        return Err(refuse!("{path} has duplicate sid entries", path));
    }
    let remap_bytes = node.property("remap").unwrap_or_default();
    if !remap_bytes.len().is_multiple_of(4) {
        return Err(refuse!("{path} remap is not padded byte pairs", path));
    }
    let remap = remap_bytes
        .as_chunks::<2>()
        .0
        .iter()
        .filter(|pair| **pair != [0, 0])
        .map(|pair| (pair[0], pair[1]))
        .collect::<Vec<_>>();
    profile
        .validate_remaps(usize::from(streams), &remap)
        .within(|| refuse!("{path}", path))?;
    if mappers.iter().any(|&(_, sid)| sid >= u32::from(streams)) {
        return Err(refuse!(
            "{path} mapper SID exceeds its native DART profile",
            path
        ));
    }
    if !remap.is_empty() {
        for &(_, context) in &mappers {
            profile
                .mapper_input_stream(usize::from(streams), context as usize, &remap)
                .within(|| refuse!("{path}", path))?;
        }
    }
    if !profile.is_t8110() && instances.len() > 1 && !mappers.is_empty() {
        return Err(refuse!(
            "{path} requires explicit legacy multi-instance DMA routing",
            path
        ));
    }
    Ok(Iommu {
        profile,
        instances,
        sids,
        remap,
        vm_base,
        smmu,
        streams,
        irq,
        mappers,
        apf,
        clock_gates,
        power_gates,
        retention,
    })
}

/// Recognize DART requirements even when a generation cannot be admitted.
/// Unknown families must produce a named requirement instead of disappearing.
fn is_dart(node: &Node<'_>) -> bool {
    node.property("compatible").is_some_and(|bytes| {
        bytes
            .split(|&byte| byte == 0)
            .any(|name| name.starts_with(b"dart,"))
    })
}

/// Pure optional-feature gate shared with public compatible admission.
/// Legacy FPAD counts as a translation descriptor plus an appended filter
/// window; accepting it as a modern standalone APF would misplace registers.
pub(super) fn legacy_requirements<'a>(
    profile: Profile,
    property: impl Fn(&str) -> Option<&'a [u8]>,
) -> Result<()> {
    if profile.is_t8110() {
        return Ok(());
    }
    let streams = match property("sid-count") {
        None => 16,
        Some(bytes) => u32::from_le_bytes(
            bytes
                .try_into()
                .map_err(|_| refuse!("has an incomplete legacy sid-count"))?,
        ),
    };
    if streams > 16 {
        return Err(refuse!(
            "requires a legacy stream layout beyond the supported 16-SID profile (including USB4's 64-SID layout)"
        ));
    }
    if streams == 0 {
        return Err(refuse!("has an empty native DART stream set"));
    }
    if property("page-size") != Some(0x4000u32.to_le_bytes().as_slice()) {
        return Err(refuse!(
            "requires an explicit supported legacy 16 KiB page-size"
        ));
    }
    let descriptors =
        property("instance").ok_or(refuse!("has no legacy 12-byte instance descriptors"))?;
    if descriptors.is_empty() || !descriptors.len().is_multiple_of(12) {
        return Err(refuse!(
            "has incomplete legacy 12-byte instance descriptors"
        ));
    }
    if descriptors.len() / 12 > 5 {
        return Err(refuse!(
            "requires a legacy descriptor count beyond the recovered five-instance topology"
        ));
    }
    for descriptor in descriptors.as_chunks::<12>().0 {
        match &descriptor[..4] {
            b"TRAD" => (),
            b"FPAD" => {
                return Err(refuse!(
                    "requires unsupported legacy FPAD translation and appended-filter semantics"
                ));
            }
            b"UMMS"
                if profile == Profile::T6000
                    && descriptors.len() == 24
                    && &descriptors[..4] == b"TRAD"
                    && &descriptors[12..16] == b"UMMS" =>
            {
                // The declared cacheless UMMS lacks lock capability. Original
                // SPTM permits the differing TRAD/UMMS modes under this exact
                // policy; preserve both source presence properties.
                if property("never-lock") != Some(&[])
                    || property("ignore-inconsistent-hw-locks") != Some(&[])
                {
                    return Err(refuse!(
                        "requires qualified empty-presence UMMS lock policies"
                    ));
                }
            }
            b"UMMS" => {
                return Err(refuse!(
                    "requires unsupported legacy UMMS capture semantics"
                ));
            }
            _ => return Err(refuse!("requires an unknown legacy DART instance kind")),
        }
    }
    if (0..5).any(|index| {
        property(&format!("filter-data-instance-{index}")).is_some()
            || property(&format!("dapf-instance-{index}")).is_some()
            || property(&format!("n-apfs-instance-{index}")).is_some()
    }) {
        return Err(refuse!(
            "requires unsupported legacy address-permission filter semantics"
        ));
    }
    for flag in ["never-lock", "ignore-inconsistent-hw-locks"] {
        if property(flag).is_some_and(|bytes| !bytes.is_empty()) {
            return Err(refuse!(
                "requires qualified empty-presence legacy lock policies"
            ));
        }
    }
    let reg = property("reg").ok_or(refuse!("has no legacy register windows"))?;
    if reg.len() != descriptors.len() / 12 * 16 {
        return Err(refuse!(
            "legacy translation descriptors do not match their physical reg ordinals"
        ));
    }
    if reg.as_chunks::<16>().0.iter().any(|pair| {
        u64::from_le_bytes(pair[8..16].try_into().expect("complete reg size"))
            != crate::dart::DAPF_BYTES
    }) {
        return Err(refuse!(
            "requires unsupported legacy DART register geometry"
        ));
    }
    if property("sid").is_some() {
        return Err(refuse!(
            "requires legacy sids bitmap semantics, not a modern sid list"
        ));
    }
    if let Some(bytes) = property("sids") {
        let bitmap = u32::from_le_bytes(
            bytes
                .try_into()
                .map_err(|_| refuse!("requires the unsupported legacy wide SID bitmap"))?,
        );
        if bitmap >> streams != 0 {
            return Err(refuse!(
                "has legacy sids bits outside the native stream set"
            ));
        }
    }
    if let Some(bytes) = property("remap") {
        if !bytes.len().is_multiple_of(4) {
            return Err(refuse!("has incomplete padded legacy remap entries"));
        }
        if bytes.iter().any(|&byte| byte != 0) {
            if descriptors
                .as_chunks::<12>()
                .0
                .iter()
                .filter(|d| &d[..4] == b"TRAD")
                .count()
                != 1
            {
                return Err(refuse!(
                    "requires explicit legacy multi-instance DMA remap routing"
                ));
            }
            let mut remaps = Vec::new();
            for pair in bytes.as_chunks::<2>().0 {
                if pair != &[0, 0] {
                    remaps.push((pair[0], pair[1]));
                }
            }
            profile.validate_remaps(streams as usize, &remaps)?;
        }
    }
    if property("vm-base").is_some_and(|bytes| bytes.len() != 4) {
        return Err(refuse!("requires one legacy 32-bit vm-base address"));
    }
    Ok(())
}
#[cfg(test)]
mod tests;
