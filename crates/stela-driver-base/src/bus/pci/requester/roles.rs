// SPDX-License-Identifier: GPL-2.0-only
// VF requester arithmetic follows Linux drivers/pci/iov.c, Copyright (C)
// 2009 Intel Corporation, Yu Zhao <yu.zhao@intel.com>, and register fields
// follow include/uapi/linux/pci_regs.h at cc9de5d4b5ab6459f5774946aa3195ad51236fa8.
//! PF/VF classification across a complete, stable PCI configuration inventory.
//! These observations grant no configuration, reset or DMA ownership.
use super::{Address, CapabilityError, FunctionRole};
use crate::bus::pci::capability::{Capabilities, ExtendedCapabilities, Transport};
use alloc::{collections::BTreeMap, vec::Vec};
const SRIOV: u16 = 0x10;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Issue {
    Config {
        address: Address,
        error: crate::bus::pci::ConfigError,
    },
    Read {
        address: Address,
        offset: u16,
    },
    Capability {
        address: Address,
        error: CapabilityError,
    },
    Sriov(Address),
    Collision(Address),
    Topology(Address),
}
impl Issue {
    fn address(&self) -> Address {
        match *self {
            Self::Config { address, .. } => address,
            Self::Read { address, .. }
            | Self::Capability { address, .. }
            | Self::Sriov(address)
            | Self::Collision(address)
            | Self::Topology(address) => address,
        }
    }
}

#[derive(Default)]
pub struct Inventory {
    present: Vec<Address>,
    virtuals: BTreeMap<Address, Address>,
    sources: Vec<Address>,
    issues: Vec<Issue>,
}
impl Inventory {
    pub const fn empty() -> Self {
        Self {
            present: Vec::new(),
            virtuals: BTreeMap::new(),
            sources: Vec::new(),
            issues: Vec::new(),
        }
    }

    /// `functions` is the complete present-function inventory for the admitted
    /// buses/segments. Configuration, reset, removal and SR-IOV writers must
    /// remain excluded throughout capture and use. An incomplete input cannot
    /// establish Physical: the caller's platform grant must supply completeness.
    pub fn capture(
        functions: &[Address],
        mut read32: impl FnMut(Address, u16) -> Result<Option<u32>, crate::bus::pci::ConfigError>,
    ) -> Result<Self, crate::bus::pci::ConfigError> {
        let mut inventory = Self {
            present: functions.to_vec(),
            ..Self::default()
        };
        inventory.present.sort_unstable();
        inventory.present.dedup();
        let mut virtuals = BTreeMap::new();
        let mut sources = Vec::new();
        for &address in &inventory.present {
            if inventory
                .issues
                .iter()
                .any(|issue| issue.address().segment == address.segment)
            {
                continue;
            }
            match function(address, &mut read32) {
                Err(Issue::Config { error, .. }) => return Err(error),
                Err(issue) => inventory.issues.push(issue),
                Ok(None) => {}
                Ok(Some(vfs)) => {
                    sources.push(address);
                    for vf in vfs {
                        if virtuals.insert(vf, address).is_some() {
                            inventory.issues.push(Issue::Collision(vf));
                            break;
                        }
                    }
                }
            }
        }
        for &source in &sources {
            if virtuals.contains_key(&source) {
                inventory.issues.push(Issue::Sriov(source));
            }
        }
        inventory.virtuals = virtuals;
        inventory.sources = sources;
        Ok(inventory)
    }
    /// Add a complete newly adopted subtree while the previous configuration
    /// and SR-IOV epoch remains unchanged. Present but unclaimed functions may
    /// occur in both inventories; an existing PF's VF assignment cannot change.
    /// This joins observations and grants no access to either subtree.
    pub fn extend(&mut self, fresh: Self) {
        for &source in &self.sources {
            if fresh.present.contains(&source)
                && (!fresh.sources.contains(&source)
                    || self
                        .virtuals
                        .iter()
                        .filter(|(_, owner)| **owner == source)
                        .ne(fresh.virtuals.iter().filter(|(_, owner)| **owner == source)))
            {
                self.issues.push(Issue::Sriov(source));
            }
        }
        for (&vf, &source) in &fresh.virtuals {
            if self.virtuals.get(&vf).is_some_and(|owner| *owner != source)
                || (self.present.contains(&vf) && !self.virtuals.contains_key(&vf))
            {
                self.issues.push(Issue::Collision(vf));
            }
        }
        for &source in &fresh.sources {
            if self.virtuals.contains_key(&source)
                || (self.present.contains(&source) && !self.sources.contains(&source))
            {
                self.issues.push(Issue::Sriov(source));
            }
        }
        for &source in &self.sources {
            if fresh.virtuals.contains_key(&source) {
                self.issues.push(Issue::Sriov(source));
            }
        }
        self.present.extend(fresh.present);
        self.present.sort_unstable();
        self.present.dedup();
        self.sources.extend(fresh.sources);
        self.sources.sort_unstable();
        self.sources.dedup();
        self.virtuals.extend(fresh.virtuals);
        self.issues.extend(fresh.issues);
    }
    /// Refuse this segment after its owned topology observation failed.
    pub fn invalidate_topology(&mut self, address: Address) {
        self.issues.push(Issue::Topology(address));
    }
    pub fn role(&self, address: Address) -> FunctionRole {
        if self
            .issues
            .iter()
            .any(|issue| issue.address().segment == address.segment)
        {
            FunctionRole::Unverified
        } else if self.virtuals.contains_key(&address) {
            FunctionRole::Virtual
        } else if self.present.binary_search(&address).is_ok() {
            FunctionRole::Physical
        } else {
            FunctionRole::Unverified
        }
    }
    pub fn issues(&self) -> &[Issue] {
        &self.issues
    }
}

fn read(
    read32: &mut impl FnMut(Address, u16) -> Result<Option<u32>, crate::bus::pci::ConfigError>,
    address: Address,
    offset: u16,
) -> Result<u32, Issue> {
    read32(address, offset)
        .map_err(|error| Issue::Config { address, error })?
        .filter(|value| *value != u32::MAX)
        .ok_or(Issue::Read { address, offset })
}
fn fields(
    read32: &mut impl FnMut(Address, u16) -> Result<Option<u32>, crate::bus::pci::ConfigError>,
    address: Address,
    at: u16,
) -> Result<[u16; 5], Issue> {
    let control = read(read32, address, at + 8)? as u16;
    let total = (read(read32, address, at + 12)? >> 16) as u16;
    let count = read(read32, address, at + 16)? as u16;
    let geometry = read(read32, address, at + 20)?;
    Ok([
        control,
        total,
        count,
        geometry as u16,
        (geometry >> 16) as u16,
    ])
}
// Conventional PCI functions (including chipset SMBus) need not decode
// configuration bytes above 0xff: some return aliased vendor/device words.
// Follow Linux pci_cfg_space_size: host bridges, PCIe and PCI-X 2 qualify.
// Local absence never decides VF status; Inventory still examines all PFs.
fn has_extended_configuration(
    address: Address,
    read32: &mut impl FnMut(Address, u16) -> Result<Option<u32>, crate::bus::pci::ConfigError>,
) -> Result<bool, Issue> {
    let host_bridge = read(read32, address, 8)? >> 16 == 0x0600;
    let transport = Transport(|offset: u16, bytes: u8| {
        let value = read32(address, offset & !3)
            .map_err(|error| CapabilityError::Config { offset, error })?
            .filter(|value| *value != u32::MAX)
            .ok_or(CapabilityError::FailedRead { offset })?;
        Ok::<_, CapabilityError>(match bytes {
            1 => value >> ((offset & 3) * 8) & 0xff,
            2 => value >> ((offset & 2) * 8) & 0xffff,
            _ => value,
        })
    });
    let mut extended = host_bridge;
    let entries = Capabilities::new(transport)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| match error {
            CapabilityError::Config { error, .. } => Issue::Config { address, error },
            error => Issue::Capability { address, error },
        })?;
    for entry in entries {
        if entry.id == 0x10 {
            extended = true;
        } else if entry.id == 0x07 {
            let status = read(read32, address, entry.offset + 4)?;
            extended |= status & 0xc0000000 != 0;
        }
    }
    Ok(extended)
}

fn function(
    address: Address,
    read32: &mut impl FnMut(Address, u16) -> Result<Option<u32>, crate::bus::pci::ConfigError>,
) -> Result<Option<Vec<Address>>, Issue> {
    let identity = read(read32, address, 0)?;
    if matches!(identity as u16, 0 | 1 | u16::MAX) || address.device >= 32 || address.function >= 8
    {
        return Err(Issue::Sriov(address));
    }
    if !has_extended_configuration(address, read32)? {
        return Ok(None);
    }
    // An all-ones first extended header denotes no extended configuration,
    // including conventional PCI. Classification still examines every PF,
    // rather than inferring this function's role from that local absence.
    let first = read32(address, 0x100)
        .map_err(|error| Issue::Config { address, error })?
        .ok_or(Issue::Read {
            address,
            offset: 0x100,
        })?;
    let mut sriov = None;
    if first != u32::MAX {
        let transport = Transport(|offset: u16, bytes: u8| {
            let value = read32(address, offset & !3)
                .map_err(|error| CapabilityError::Config { offset, error })?
                .ok_or(CapabilityError::FailedRead { offset })?;
            Ok::<_, CapabilityError>(match bytes {
                1 => value >> ((offset & 3) * 8) & 0xff,
                2 => value >> ((offset & 2) * 8) & 0xffff,
                _ => value,
            })
        });
        for entry in ExtendedCapabilities::census(transport) {
            let entry = entry.map_err(|error| match error {
                CapabilityError::Config { error, .. } => Issue::Config { address, error },
                error => Issue::Capability { address, error },
            })?;
            if entry.id == SRIOV && (entry.version != 1 || sriov.replace(entry.offset).is_some()) {
                return Err(Issue::Sriov(address));
            }
        }
    }
    let virtuals = if let Some(at) = sriov {
        if read(read32, address, 0x0c)? >> 16 & 0x7f != 0 {
            return Err(Issue::Sriov(address));
        }
        let snapshot @ [control, total, count, offset, stride] = fields(read32, address, at)?;
        let mut vfs = Vec::new();
        if control & 1 != 0 {
            if count == 0 || count > total || offset == 0 || (count > 1 && stride == 0) {
                return Err(Issue::Sriov(address));
            }
            let rid = u16::from(address.bus) << 8
                | u16::from(address.device) << 3
                | u16::from(address.function);
            for index in 0..count {
                let vf = u32::from(rid) + u32::from(offset) + u32::from(stride) * u32::from(index);
                let vf = u16::try_from(vf).map_err(|_| Issue::Sriov(address))?;
                vfs.push(Address {
                    segment: address.segment,
                    bus: (vf >> 8) as u8,
                    device: ((vf >> 3) & 31) as u8,
                    function: (vf & 7) as u8,
                });
            }
        }
        if fields(read32, address, at)? != snapshot {
            return Err(Issue::Sriov(address));
        }
        Some(vfs)
    } else {
        None
    };
    if read(read32, address, 0)? != identity
        || read32(address, 0x100).map_err(|error| Issue::Config { address, error })? != Some(first)
    {
        return Err(Issue::Sriov(address));
    }
    Ok(virtuals)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::BTreeMap;
    const PF: Address = Address {
        segment: 0,
        bus: 0,
        device: 31,
        function: 0,
    };
    fn at(segment: u16, rid: u16) -> Address {
        Address {
            segment,
            bus: (rid >> 8) as u8,
            device: ((rid >> 3) & 31) as u8,
            function: (rid & 7) as u8,
        }
    }
    fn plain() -> [u32; 1024] {
        let mut words = [0; 1024];
        words[0] = 0x12348086;
        words
    }
    fn pf(count: u16, offset: u16, stride: u16) -> [u32; 1024] {
        let mut words = plain();
        // PCI Express, which alone has the extended list SR-IOV is in.
        words[1] |= 1 << 20;
        words[0x34 / 4] = 0x40;
        words[0x40 / 4] = 0x0002_0010;
        words[0x100 / 4] = 0x10010;
        words[0x108 / 4] = 1;
        words[0x10c / 4] = 16 << 16;
        words[0x110 / 4] = u32::from(count);
        words[0x114 / 4] = u32::from(offset) | u32::from(stride) << 16;
        words
    }
    fn capture(spaces: &BTreeMap<Address, [u32; 1024]>) -> Inventory {
        let functions: Vec<_> = spaces.keys().copied().collect();
        Inventory::capture(&functions, |address, offset| {
            Ok(spaces
                .get(&address)
                .map(|words| words[usize::from(offset) / 4]))
        })
        .unwrap()
    }
    #[test]
    fn legacy_smbus_alias_does_not_poison_gpu_or_read_extended_space() {
        let smbus = at(0, 0xfc);
        let gpu = at(0, 0x100);
        let mut legacy = plain();
        legacy[0] = 0x7aa38086;
        legacy[2] = 0x0c050000;
        legacy[0x100 / 4] = legacy[0]; // Produces bogus next pointer 0x7aa.
        let mut endpoint = pf(0, 0, 0);
        endpoint[0x100 / 4] = 0; // A GPU endpoint without SR-IOV.
        let spaces = BTreeMap::from([(smbus, legacy), (gpu, endpoint)]);
        let functions: Vec<_> = spaces.keys().copied().collect();
        let inventory = Inventory::capture(&functions, |address, offset| {
            assert!(address != smbus || offset < 0x100);
            Ok(Some(spaces[&address][usize::from(offset) / 4]))
        })
        .unwrap();
        assert!(inventory.issues().is_empty());
        assert_eq!(inventory.role(gpu), FunctionRole::Physical);
    }
    #[test]
    fn malformed_extended_space_on_pcie_still_refuses_the_segment() {
        let mut pcie = pf(0, 0, 0);
        pcie[0x100 / 4] = 0x7aa38086;
        let inventory = capture(&BTreeMap::from([(PF, pcie), (at(0, 0x100), plain())]));
        assert!(!inventory.issues().is_empty());
        assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Unverified);
    }
    #[test]
    fn legacy_local_absence_does_not_hide_a_vf_from_its_pf() {
        let mut legacy = plain();
        legacy[0x100 / 4] = 0x7aa38086;
        let inventory = capture(&BTreeMap::from([(PF, pf(1, 8, 1)), (at(0, 0x100), legacy)]));
        assert!(inventory.issues().is_empty());
        assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Virtual);
    }
    #[test]
    fn host_bridges_and_pcix2_still_have_extended_configuration_checked() {
        let mut host = plain();
        host[2] = 0x06000000;
        let mut pcix = plain();
        pcix[1] = 1 << 20;
        pcix[0x34 / 4] = 0x40;
        pcix[0x40 / 4] = 0x07;
        pcix[0x44 / 4] = 0x40000000;
        for mut words in [host, pcix] {
            words[0x100 / 4] = 0x7aa38086;
            let inventory = capture(&BTreeMap::from([(PF, words)]));
            assert!(!inventory.issues().is_empty());
        }
    }
    #[test]
    fn transport_denial_aborts_without_absence_or_a_physical_role() {
        use crate::bus::pci::{ConfigError, ConfigFailure, assignment};
        let words = pf(2, 8, 1);
        let mut accesses = 0;
        Inventory::capture(&[PF], |_, offset| {
            accesses += 1;
            Ok(Some(words[usize::from(offset) / 4]))
        })
        .unwrap();
        for fault in 1..=accesses {
            let mut observed = 0;
            let denied =
                ConfigError::before_effect(ConfigFailure::Ownership(assignment::Error::Closed));
            let result = Inventory::capture(&[PF], |_, offset| {
                observed += 1;
                if observed == fault {
                    Err(denied)
                } else {
                    Ok(Some(words[usize::from(offset) / 4]))
                }
            });
            assert_eq!(result.err(), Some(denied));
            assert_eq!(observed, fault, "no read after denied original permission");
        }
    }
    #[test]
    fn vfs_without_local_capabilities_are_classified_from_their_pf_across_a_bus_boundary() {
        let spaces = BTreeMap::from([
            (PF, pf(2, 8, 2)),
            (at(0, 0x100), plain()),
            (at(0, 0x101), plain()),
            (at(0, 0x102), plain()),
            (at(3, 0x100), plain()),
        ]);
        let inventory = capture(&spaces);
        assert!(inventory.issues().is_empty());
        assert_eq!(inventory.role(PF), FunctionRole::Physical);
        for rid in [0x100, 0x102] {
            assert_eq!(inventory.role(at(0, rid)), FunctionRole::Virtual);
        }
        assert_eq!(inventory.role(at(0, 0x101)), FunctionRole::Physical);
        assert_eq!(inventory.role(at(3, 0x100)), FunctionRole::Physical);
        assert_eq!(inventory.role(at(0, 0x200)), FunctionRole::Unverified);
    }
    #[test]
    fn disabled_vfs_do_not_reclassify_an_ordinary_present_function() {
        let mut source = pf(2, 8, 2);
        source[0x108 / 4] = 0;
        let spaces = BTreeMap::from([(PF, source), (at(0, 0x100), plain())]);
        assert_eq!(capture(&spaces).role(at(0, 0x100)), FunctionRole::Physical);
    }
    #[test]
    fn malformed_or_overflowing_enabled_vf_geometry_invalidates_the_segment() {
        for (source, words) in [
            (PF, pf(0, 8, 1)),
            (PF, pf(17, 8, 1)),
            (PF, pf(2, 0, 1)),
            (PF, pf(2, 8, 0)),
            (at(0, 0xff00), pf(2, 0x100, 1)),
        ] {
            let spaces = BTreeMap::from([
                (source, words),
                (at(0, 0x100), plain()),
                (at(1, 0x100), plain()),
            ]);
            let inventory = capture(&spaces);
            assert!(!inventory.issues().is_empty());
            assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Unverified);
            assert_eq!(inventory.role(at(1, 0x100)), FunctionRole::Physical);
        }
    }
    #[test]
    fn failed_pf_reads_prevent_a_local_capability_absence_from_becoming_a_physical_grant() {
        let spaces = BTreeMap::from([(PF, pf(2, 8, 1)), (at(0, 0x100), plain())]);
        let functions: Vec<_> = spaces.keys().copied().collect();
        let inventory = Inventory::capture(&functions, |address, offset| {
            if address == PF && offset == 0x100 {
                Ok(None)
            } else {
                Ok(Some(spaces[&address][usize::from(offset) / 4]))
            }
        })
        .unwrap();
        assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Unverified);
    }
    #[test]
    fn overlapping_pf_vf_ranges_and_nested_pf_capabilities_are_refused() {
        let spaces = BTreeMap::from([(at(0, 0), pf(2, 0x100, 1)), (PF, pf(2, 8, 1))]);
        assert!(
            capture(&spaces)
                .issues()
                .iter()
                .any(|issue| matches!(issue, Issue::Collision(_)))
        );
        let spaces = BTreeMap::from([(PF, pf(1, 8, 1)), (at(0, 0x100), pf(1, 8, 1))]);
        assert_eq!(capture(&spaces).role(PF), FunctionRole::Unverified);
    }
    #[test]
    fn changing_numvfs_during_capture_refuses_the_whole_segment() {
        let words = pf(2, 8, 1);
        let mut samples = 0;
        let inventory = Inventory::capture(&[PF], |_, offset| {
            if offset == 0x110 {
                samples += 1;
                return Ok(Some(if samples == 1 { 2 } else { 1 }));
            }
            Ok(Some(words[usize::from(offset) / 4]))
        })
        .unwrap();
        assert_eq!(inventory.role(PF), FunctionRole::Unverified);
    }
    #[test]
    fn duplicate_and_unsupported_sriov_refuse_classification() {
        let mut duplicate = pf(2, 8, 1);
        duplicate[0x100 / 4] |= 0x140 << 20;
        duplicate[0x140 / 4] = 0x10010;
        let mut unknown = pf(2, 8, 1);
        unknown[0x100 / 4] = 0x20010;
        for words in [duplicate, unknown] {
            assert_eq!(
                capture(&BTreeMap::from([(PF, words)])).role(PF),
                FunctionRole::Unverified
            );
        }
        let mut conventional = plain();
        conventional[0x100 / 4] = u32::MAX;
        assert_eq!(
            capture(&BTreeMap::from([(PF, conventional)])).role(PF),
            FunctionRole::Physical
        );
    }

    /// Linux `pci_find_next_ext_capability`: a loop, a pointer with its low
    /// bits set or a vendor entry whose length cannot be read ends or passes
    /// the walk, and the PF is still classified.
    #[test]
    fn an_odd_extended_list_still_classifies_the_pf() {
        let mut cycle = pf(2, 8, 1);
        cycle[0x100 / 4] |= 0x100 << 20;
        let mut unaligned = pf(2, 8, 1);
        unaligned[0x100 / 4] |= 0x142 << 20;
        let mut vendor = pf(2, 8, 1);
        vendor[0x100 / 4] |= 0x200 << 20;
        vendor[0x200 / 4] = 0x1000b;
        for words in [cycle, unaligned, vendor] {
            let inventory = capture(&BTreeMap::from([(PF, words)]));
            assert!(inventory.issues().is_empty());
            assert_eq!(inventory.role(PF), FunctionRole::Physical);
        }
    }

    #[test]
    fn an_arriving_vf_keeps_the_initial_pf_classification_and_other_segments_stay_independent() {
        let mut inventory = capture(&BTreeMap::from([(PF, pf(2, 8, 2))]));
        inventory.extend(capture(&BTreeMap::from([
            (at(0, 0x100), plain()),
            (at(0, 0x102), plain()),
            (at(3, 0x100), plain()),
        ])));
        assert!(inventory.issues().is_empty());
        assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Virtual);
        assert_eq!(inventory.role(at(0, 0x102)), FunctionRole::Virtual);
        assert_eq!(inventory.role(at(3, 0x100)), FunctionRole::Physical);
        assert_eq!(inventory.role(PF), FunctionRole::Physical);
    }
    #[test]
    fn repeated_unclaimed_observations_preserve_the_same_pf_epoch() {
        let spaces = BTreeMap::from([(PF, pf(2, 8, 2)), (at(0, 0x100), plain())]);
        let mut inventory = capture(&spaces);
        inventory.extend(capture(&spaces));
        assert!(inventory.issues().is_empty());
        assert_eq!(inventory.role(PF), FunctionRole::Physical);
        assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Virtual);
    }
    #[test]
    fn arrival_cannot_change_an_existing_pf_or_reclassify_an_owned_physical_function() {
        for fresh in [
            BTreeMap::from([(PF, pf(1, 8, 1))]),
            BTreeMap::from([(PF, plain())]),
            BTreeMap::from([(at(0, 0), pf(1, 0xf8, 1))]),
            BTreeMap::from([(at(0, 0x100), pf(1, 8, 1))]),
        ] {
            let mut inventory = capture(&BTreeMap::from([(PF, pf(2, 8, 2))]));
            inventory.extend(capture(&fresh));
            assert!(!inventory.issues().is_empty());
            assert_eq!(inventory.role(PF), FunctionRole::Unverified);
        }
        let mut inventory = capture(&BTreeMap::from([(at(0, 0x100), plain())]));
        inventory.extend(capture(&BTreeMap::from([(PF, pf(1, 8, 1))])));
        assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Unverified);
    }

    #[test]
    fn an_unready_pf_makes_the_segment_incomplete_instead_of_certifying_other_functions() {
        for vendor in [0, 1, u16::MAX] {
            let mut words = plain();
            words[0] = u32::from(vendor);
            let inventory = capture(&BTreeMap::from([(PF, words), (at(0, 0x100), plain())]));
            assert_eq!(inventory.role(at(0, 0x100)), FunctionRole::Unverified);
        }
    }
}
