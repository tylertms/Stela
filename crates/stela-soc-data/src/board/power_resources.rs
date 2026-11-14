//! Original PMGR graph and complete resource inventory. A logical IOMMU
//! mapper is a view of its ancestor DART's stream state, not another power
//! island. Registerless USB root-port views likewise belong to their USB
//! controller. Conversely, sharing an iommu-parent does not make a DMA client's
//! controller part of the DART's physical supply. Hardware resources inherit
//! their ADT ancestors' activated gates, including nubs and firmware endpoints.
//! Source PMGR provider gate-index metadata remains inventoried separately from
//! the consumer dependencies its IOService start activates.
use super::{IOMMU_MAPPER, Node, PMGR2_ARCH, PMGR2_IMPLEMENTATION, SOCS, name_of};
use crate::Result;
use alloc::{collections::BTreeSet, format, string::String, vec::Vec};
use stela_contracts::refusal::Within;
use stela_contracts::refuse;
use {
    crate::dart::Profile, crate::pmgr::coverage, crate::pmgr::coverage::Admission,
    crate::pmgr::coverage::Consumer, crate::pmgr::coverage::Domain,
    crate::pmgr::coverage::DomainId, crate::pmgr::coverage::LifecycleSinks,
};

pub(super) mod bridges;
mod clock_role;
pub use clock_role::{ClockRole, PmgrClockProvider};

mod cpu;
pub use cpu::ProcessorAssociation;
mod pmp;
mod records;
pub use pmp::PmpDomain;
use records::{Layout, Record};
pub(super) mod implicit;

/// Bytes of one m1n1 `struct pmgr_device` record.
pub(super) const PMGR_DEVICE: usize = records::BYTES;
#[cfg(test)]
pub(super) const PMGR_VIRTUAL: u32 = 0x10;

/// Hardware owners, indexed in the board's corresponding placement vectors.
/// Logical mapper and root-port views share their controller owner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerOwner {
    Dart(usize),
    Usb(usize),
    Phy(usize),
    /// Candidate canonical physical aperture; only a registered
    /// type-erased lifecycle object can satisfy this ownership key.
    Mmio(u64),
}

/// One original resource with separate inherited clock/power requirements.
/// Other hardware still has no registered lifecycle sink.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PowerResource {
    pub name: String,
    pub clocks: Vec<DomainId>,
    /// Source-checked driver use; provider metadata alone activates no gate.
    pub clock_role: ClockRole,
    pub power: Vec<DomainId>,
    pub owner: Option<PowerOwner>,
    /// Explicit preparation evidence; defaults to required even without owner
    /// or compatible metadata. Unavailable never grants functional support.
    pub admission: Admission,
}

impl PowerResource {
    /// Start/lifecycle requirements, keeping passive provider indices
    /// in clocks for source accounting and later explicit gate operations.
    pub fn activated_clocks(&self) -> &[DomainId] {
        match &self.clock_role {
            ClockRole::Consumer => &self.clocks,
            ClockRole::PmgrProvider(provider) => provider.ancestors(),
        }
    }
}

/// PMGR blocks, complete original domain graph and all original ADT resources,
/// including nodes whose admission/compatible was removed by the loader.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PowerStates {
    pub blocks: Vec<(u64, u64)>,
    pub registers: Vec<u64>,
    pub domains: Vec<Domain>,
    pub resources: Vec<PowerResource>,
    /// Source-checked requirements absent from the original gate arrays.
    /// Keep the original inventory unchanged for admission and source review.
    pub(crate) implicit: Vec<implicit::Requirement>,
}

impl PowerStates {
    /// Positively checked physical CPU owners. The effective harvesting
    /// setting must include supplied start-time overrides; this method supplies
    /// no initial LifecycleState and never registers a participant or ACKs PMGR.
    pub fn processor_associations(
        &self,
        chip: stela_contracts::Chip,
        root: &Node<'_>,
        processors: &[stela_contracts::identity::Identity],
        acc_harvesting: u32,
    ) -> Result<Vec<ProcessorAssociation>> {
        let io = root
            .child("arm-io")
            .ok_or(refuse!("CPU PMGR requires arm-io"))?;
        let parent = io.child("pmgr").ok_or(refuse!("CPU PMGR requires pmgr"))?;
        let node = register_node(&io, &parent)?;
        // Reconstruct every original domain through the existing reader. The
        // association resolver may trust normalized non-CPU locations only
        // after this equality check, including aliases and parent identities.
        let dies = Dies::of(root, &node)?;
        let table = decode_domain_table(&node, dies, &|address, bytes| {
            cpu::physical(&io, address, bytes).ok()
        })?;
        if self.domains != table.domains
            || self.blocks != table.blocks
            || self.registers != table.registers
        {
            return Err(refuse!(
                "CPU PMGR canonical inventory differs from the source tree"
            ));
        }
        // Each further die repeats die 0's records (`decode_domain_table`),
        // so every die's views come from the one `devices` table. A resolver
        // whose source contract covers one die refuses another die's view.
        let (layout, _) = Layout::select(node.property("ps-regs"), node.property("ps-groups"))?;
        let devices = node
            .property("devices")
            .ok_or(refuse!("CPU PMGR requires devices"))?;
        if !devices.len().is_multiple_of(PMGR_DEVICE)
            || devices.len() / PMGR_DEVICE * usize::from(dies.count) != self.domains.len()
        {
            return Err(refuse!(
                "CPU PMGR views differ from the complete domain inventory"
            ));
        }
        let per_die = devices.len() / PMGR_DEVICE;
        let views = devices
            .as_chunks::<PMGR_DEVICE>()
            .0
            .iter()
            .cycle()
            .zip(&self.domains)
            .enumerate()
            .map(|(index, (raw, domain))| {
                let record = Record::parse(layout, raw)?;
                let die = u8::try_from(index / per_die)
                    .map_err(|_| refuse!("CPU PMGR die index overflows"))?;
                if domain.id
                    != (DomainId {
                        die,
                        ..domain_id(record.id)
                    })
                    || domain.name != record.name
                {
                    return Err(refuse!("CPU PMGR record and canonical domain differ"));
                }
                let location = match (record.location, domain.address) {
                    (Some(location), Some(address)) => Some(cpu::PmgrCpuLocation {
                        group: location.group,
                        index: location.index,
                        address,
                    }),
                    (None, None) => None,
                    _ => return Err(refuse!("CPU PMGR physical/virtual domain mismatch")),
                };
                Ok(cpu::PmgrCpuRecord {
                    id: domain.id,
                    flags: raw[0],
                    perf_domain: raw[2],
                    name: record.name,
                    location,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        cpu::resolve_cpu_power(chip, root, &views, processors, acc_harvesting)
            .map_err(|error| refuse!("CPU PMGR associations: {error}", ?error))
    }

    /// Original power gates plus checked physical ownership.
    /// Stable resource indices also identify the lifecycle owner.
    pub fn power_requirements(&self) -> Result<Vec<Vec<DomainId>>> {
        implicit::power_requirements(self)
    }

    /// Apply the bundle's existing positive device-admission ledger to the
    /// matching prepared resource inventory. No node/property is removed.
    /// Missing evidence remains required. A hidden controller's registerless
    /// logical descendants follow it, until an independently admitted child
    /// or a placed hardware owner establishes its own obligations.
    pub fn apply_admission(&mut self, prepared: &[u8], hidden: &[String]) -> Result<usize> {
        let root = Node::root(prepared).ok_or(refuse!("PMGR admission tree is invalid"))?;
        let expected = inventory_with_nodes(&root)?;
        let io = root.child("arm-io");
        let pmgr = io.as_ref().unwrap_or(&root).child("pmgr");
        let translate =
            |address, bytes| io.as_ref().unwrap_or(&root).translate_range(address, bytes);
        let implicit = pmgr
            .as_ref()
            .map(|pmgr| implicit::resolve(pmgr, &self.domains, &expected, &translate))
            .transpose()?
            .unwrap_or_default();
        if expected.resources.len() != self.resources.len()
            || implicit != self.implicit
            || expected
                .resources
                .iter()
                .zip(&self.resources)
                .any(|(expected, actual)| {
                    expected.name != actual.name
                        || expected.clocks != actual.clocks
                        || expected.clock_role != actual.clock_role
                        || expected.power != actual.power
                        || expected.owner != actual.owner
                })
        {
            return Err(refuse!(
                "PMGR admission evidence differs from the complete resource inventory"
            ));
        }
        let mut unavailable = BTreeSet::new();
        let mut evidence = BTreeSet::new();
        for entry in hidden {
            if !evidence.insert(entry.as_str()) {
                return Err(refuse!("PMGR admission repeats evidence {entry}", entry));
            }
            // The historical ledger also carries these preparation notes.
            // They establish no unavailable resource and cannot waive sinks.
            if let Some((path, note)) = entry.split_once(' ') {
                let node = &expected.nodes[expected.locate(path)?];
                let valid = match (path, note) {
                    ("/defaults", "serial-device") => node.property("serial-device").is_none(),
                    ("/", "secure-root-prefix") => node.property("secure-root-prefix").is_none(),
                    (_, "acio-parent") => node.property("acio-parent").is_none(),
                    (_, "+no-pmu") => node.property("no-pmu").is_some(),
                    (_, "+gptbat-ready") => node.property("gptbat-ready").is_some(),
                    (_, note) if super::CAPABILITY_PROPERTIES.contains(&note) => {
                        node.property(note).is_none()
                    }
                    _ => false,
                };
                if !valid {
                    return Err(refuse!(
                        "PMGR admission has invalid preparation note {entry}",
                        entry
                    ));
                }
                continue;
            }
            if entry == "/" || entry == "/arm-io" || entry == "/cpus" || entry.starts_with("/cpus/")
            {
                return Err(refuse!(
                    "PMGR admission cannot retire structural {entry}",
                    entry
                ));
            }
            let index = expected.locate(entry)?;
            let node = &expected.nodes[index];
            let resource = &self.resources[index];
            if resource
                .owner
                .is_some_and(|owner| !matches!(owner, PowerOwner::Mmio(_)))
                || node.property("compatible").is_some() && node.property("AAPL,ignore").is_none()
            {
                return Err(refuse!(
                    "PMGR unavailable admission conflicts with visible hardware {entry}",
                    entry
                ));
            }
            if !unavailable.insert(index) {
                return Err(refuse!("PMGR admission repeats resource {entry}", entry));
            }
        }
        let mut admissions = Vec::with_capacity(self.resources.len());
        for (index, resource) in self.resources.iter().enumerate() {
            let mut retired = expected.parents[index]
                .is_some_and(|parent| admissions[parent] == Admission::Unavailable);
            let node = &expected.nodes[index];
            // Resolve ancestry by the original tree position. A display name
            // can repeat among siblings and must not select another node.
            if ["compatible", "reg", "clock-gates", "power-gates"]
                .iter()
                .any(|key| node.property(key).is_some())
            {
                retired = false;
            }
            if unavailable.contains(&index) {
                retired = true;
            }
            if resource
                .owner
                .is_some_and(|owner| !matches!(owner, PowerOwner::Mmio(_)))
            {
                retired = false;
            }
            admissions.push(if retired {
                Admission::Unavailable
            } else {
                Admission::Required
            });
        }
        let count = admissions
            .iter()
            .filter(|&&admission| admission == Admission::Unavailable)
            .count();
        for (resource, admission) in self.resources.iter_mut().zip(admissions) {
            resource.admission = admission;
        }
        Ok(count)
    }
}

struct ResourceInventory<'a> {
    resources: Vec<PowerResource>,
    nodes: Vec<Node<'a>>,
    parents: Vec<Option<usize>>,
    ranges: Vec<(u64, u64, u64)>,
}

impl ResourceInventory<'_> {
    /// Historical admission ledgers contain paths, not node identities. Such
    /// evidence is usable only when it names exactly one original resource.
    fn locate(&self, path: &str) -> Result<usize> {
        let mut matches = self
            .resources
            .iter()
            .enumerate()
            .filter_map(|(index, resource)| (resource.name == path).then_some(index));
        let index = matches
            .next()
            .ok_or_else(|| refuse!("PMGR admission names missing resource {path}", path))?;
        if matches.next().is_some() {
            return Err(refuse!("PMGR admission path is ambiguous: {path}", path));
        }
        Ok(index)
    }
}

/// Die field of a `clock-gates`/`power-gates` word. 26A434 mac14j
/// `ApplePMGRFunctionClockGate::callFunction` takes the die as the word's
/// bits `31:28` (`lsr w22, w20, #28`, KC VA 0xfffffe0009b7e1b0) and the
/// device as the rest (`and x1, x20, #0xfffffff`, 0xfffffe0009b7e25c);
/// `ApplePMGRNub::_enableDeviceStatusChangeNotifications` splits it the same
/// way and passes only `& 0xffff` (0xfffffe0009b8b7d8, 0xfffffe0009b8b82c) to
/// the per-die `(device, die)` entry points such as `_enableDeviceGated`.
const GATE_DIE_SHIFT: u32 = 28;
/// Bits `27:16` reach no 16-bit device record; no checked meaning.
const GATE_RESERVED: u32 = 0x0fff_0000;

/// Decode each gate word into its die-checked device. The record tables
/// hold die 0's devices; [`decode_domain_table`] places every other die's
/// copy, so a die without a placed PMGR remains an unknown requirement.
pub(super) fn gate_ids(node: &Node<'_>, path: &str, property: &str) -> Result<Vec<DomainId>> {
    let bytes = node.property(property).unwrap_or_default();
    if !bytes.len().is_multiple_of(4) {
        return Err(refuse!(
            "{path} {property} contains a partial device ID",
            path,
            property
        ));
    }
    bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|word| {
            let word = u32::from_le_bytes(*word);
            if word & GATE_RESERVED != 0 {
                return Err(refuse!(
                    "{path} {property} {word:#x} sets reserved PMGR device ID bits",
                    path,
                    property,
                    word
                ));
            }
            Ok(DomainId {
                die: (word >> GATE_DIE_SHIFT) as u8,
                device: word as u16,
            })
        })
        .collect()
}

fn domain_id(device: u16) -> DomainId {
    DomainId { die: 0, device }
}

/// Default die strides of the PMGR drivers that bind these families when
/// `/arm-io/pmgr` has no `die-stride`: `AppleT6000PMGR::getDieStride` in
/// AppleT6001PMGR (26A434 mac13j KC VA 0xfffffe0009f59164) and
/// `AppleT6020PMGR::getDieStride` in AppleT6021PMGR (mac14j 0xfffffe0009f6ceac)
/// both return 0x20_0000_0000, and their `getDieOffset(die)` is `die << 37`.
const DRIVER_DIE_STRIDES: [(&str, u64); 2] = [("pmgr1,t6001", 1 << 37), ("pmgr1,t6021", 1 << 37)];

/// The PMGR instances of a multi-die SoC: die `d`'s power-state registers are
/// die 0's plus `d * stride` (`ApplePMGR::getDieOffset`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Dies {
    count: u8,
    stride: u64,
}

impl Dies {
    /// `ApplePMGR::start` (mac14j KC VA 0xfffffe0009b358e4..0xfffffe0009b35934)
    /// reads `/arm-io/pmgr` `die-stride`, falls back to the driver's virtual
    /// `getDieStride` when it is absent or zero, and refuses a die count above
    /// one without a stride. The die count is the CPU dies (`/cpus/*` `die-id`,
    /// which XNU `ml_parse_cpu_topology` reads), as the `die-count` iBoot sets
    /// at boot (J180D iBoot file 0x39d88) is not in the firmware tree; a tree
    /// without CPU nodes is one die.
    fn of(root: &Node<'_>, pmgr: &Node<'_>) -> Result<Self> {
        let cpus = if name_of(root) == "arm-io" {
            None
        } else {
            root.child("cpus")
        };
        let mut highest = 0;
        for cpu in cpus.and_then(|cpus| cpus.children()).unwrap_or_default() {
            if let Some(die) = cpu.first_u32("die-id") {
                highest = highest.max(die);
            }
        }
        let count = highest
            .checked_add(1)
            .filter(|&count| count <= 1 << (32 - GATE_DIE_SHIFT))
            .and_then(|count| u8::try_from(count).ok())
            .ok_or_else(|| refuse!("CPU die-id {highest} exceeds the PMGR die field", highest))?;
        if count == 1 {
            return Ok(Self { count, stride: 0 });
        }
        let stride = match pmgr.property("die-stride") {
            Some(bytes) if bytes.len() == 8 => {
                u64::from_le_bytes(bytes.try_into().expect("eight bytes"))
            }
            Some(_) => return Err(refuse!("/arm-io/pmgr die-stride is not one 64-bit stride")),
            None => 0,
        };
        let stride = if stride != 0 {
            stride
        } else {
            DRIVER_DIE_STRIDES
                .iter()
                .find(|(family, _)| pmgr.compatible(family))
                .map(|&(_, stride)| stride)
                .ok_or_else(|| {
                    refuse!(
                        "/arm-io/pmgr spans {count} dies without a qualified die-stride",
                        count
                    )
                })?
        };
        Ok(Self { count, stride })
    }

    /// Die `die`'s copy of a die-0 physical address.
    fn place(self, die: u8, address: u64) -> Result<u64> {
        self.stride
            .checked_mul(u64::from(die))
            .and_then(|offset| address.checked_add(offset))
            .ok_or_else(|| refuse!("/arm-io/pmgr die {die} power state overflows", die))
    }
}

#[cfg(test)]
fn inventory(root: &Node<'_>) -> Result<Vec<PowerResource>> {
    Ok(inventory_with_nodes(root)?.resources)
}

fn inventory_with_nodes<'a>(root: &Node<'a>) -> Result<ResourceInventory<'a>> {
    let provider = clock_role::provider(root)?;
    let ranges = root
        .child("arm-io")
        .and_then(|io| io.ranges())
        .or_else(|| root.ranges())
        .unwrap_or_default();
    let mut physical_views = Vec::new();
    let io = root.child("arm-io");
    let bus = io.as_ref().unwrap_or(root);
    super::apertures::visit(
        bus,
        &|address, bytes| bus.translate_range(address, bytes),
        &mut |node, translate| {
            if let Some(regs) = super::apertures::registers(node)
                && let Some(&(address, bytes)) = regs.first()
                && let Some(physical) = translate(address, bytes)
            {
                physical_views.push((node.property("reg").unwrap().as_ptr(), physical));
            }
        },
    )
    .ok_or(refuse!("invalid physical resource tree"))?;
    let physical = |node: &Node<'_>| {
        let key = node.property("reg")?.as_ptr();
        physical_views
            .iter()
            .find(|(source, _)| *source == key)
            .map(|(_, physical)| *physical)
    };
    let path = if name_of(root) == "arm-io" {
        "/arm-io"
    } else {
        ""
    };
    let clocks = gate_ids(root, path, "clock-gates")?;
    let power = gate_ids(root, path, "power-gates")?;
    let mut resources = alloc::vec![PowerResource {
        name: if path.is_empty() {
            "/".into()
        } else {
            path.into()
        },
        clocks: clocks.clone(),
        clock_role: ClockRole::Consumer,
        power: power.clone(),
        owner: None,
        admission: Admission::Required,
    }];
    let mut nodes = alloc::vec![root.clone()];
    let mut parents = alloc::vec![None];
    let mut pending = root
        .children()
        .ok_or(refuse!("invalid resource tree"))?
        .into_iter()
        .rev()
        .map(|node| {
            (
                0,
                node,
                clocks.clone(),
                power.clone(),
                None,
                Vec::<&'static str>::new(),
            )
        })
        .collect::<Vec<_>>();
    let mut darts = 0;
    let mut dart_streams = Vec::new();
    let mut usbs = 0;
    let mut phys = 0;
    while let Some((parent, node, mut clocks, mut power, owner, views)) = pending.pop() {
        let path = format!(
            "{}/{}",
            resources[parent].name.trim_end_matches('/'),
            name_of(&node)
        );
        let clock_role = provider.as_ref().map_or(ClockRole::Consumer, |provider| {
            provider.role(&node, &clocks)
        });
        let provider_ancestors =
            matches!(clock_role, ClockRole::PmgrProvider(_)).then(|| clocks.clone());
        clocks.extend(gate_ids(&node, &path, "clock-gates")?);
        power.extend(gate_ids(&node, &path, "power-gates")?);
        clocks.sort_unstable();
        clocks.dedup();
        power.sort_unstable();
        power.dedup();
        let dart = node
            .property("compatible")
            .and_then(Profile::from_compatible)
            .is_some();
        let owner = if dart && path.starts_with("/arm-io/") {
            let index = darts;
            darts += 1;
            // SPTM defaults an absent sid-count to 16 on every generation.
            dart_streams.push(node.first_u32("sid-count").unwrap_or(16));
            Some(PowerOwner::Dart(index))
        } else if path.starts_with("/arm-io/") && super::usb_topology::is_usb_controller(&node) {
            let index = usbs;
            usbs += 1;
            Some(PowerOwner::Usb(index))
        } else if path.starts_with("/arm-io/") && super::usb_topology::is_typec_phy(&node) {
            let index = phys;
            phys += 1;
            Some(PowerOwner::Phy(index))
        } else if node.compatible(IOMMU_MAPPER)
            && node.first_u32("AAPL,phandle").is_some()
            && owner.is_some_and(|owner| match owner {
                PowerOwner::Dart(index) => node
                    .first_u32("reg")
                    .is_some_and(|stream| stream < dart_streams[index]),
                PowerOwner::Usb(_) | PowerOwner::Phy(_) | PowerOwner::Mmio(_) => false,
            })
            || node.property("reg").is_none()
                && node.property("compatible").is_none()
                && node.property("clock-gates").is_none()
                && node.property("power-gates").is_none()
                && node
                    .first_u32("usb-port-number")
                    .is_some_and(|number| (1..=2).contains(&number))
                && matches!(owner, Some(PowerOwner::Usb(_)))
        {
            owner
        } else {
            None
        };
        let owner = owner.or_else(|| {
            if !path.starts_with("/arm-io/") {
                return None;
            }
            let modeled = node
                .property("compatible")
                .into_iter()
                .flat_map(|list| list.split(|byte| *byte == 0))
                .filter_map(|name| core::str::from_utf8(name).ok())
                .any(|name| super::models(name, |key| node.property(key)));
            if modeled && let Some(address) = physical(&node) {
                return Some(PowerOwner::Mmio(address));
            }
            if views.iter().any(|view| {
                node.compatible(view)
                    || *view == "debug-console"
                        && name_of(&node) == "debug-console"
                        && ["compatible", "reg", "clock-gates", "power-gates"]
                            .iter()
                            .all(|key| node.property(key).is_none())
            }) {
                return match resources[parent].owner {
                    Some(PowerOwner::Mmio(address)) => Some(PowerOwner::Mmio(address)),
                    _ => None,
                };
            }
            None
        });
        let mut views = Vec::new();
        for name in node
            .property("compatible")
            .into_iter()
            .flat_map(|list| list.split(|byte| *byte == 0))
            .filter_map(|name| core::str::from_utf8(name).ok())
        {
            views.extend(
                super::model_children(name, |key| node.property(key))
                    .iter()
                    .map(|view| view.compatible),
            );
            views.extend(match name {
                super::UART => &["debug-console"][..],
                super::SPI => &[super::NOR][..],
                super::NOR => &[super::NVRAM, super::SYSCFG][..],
                name if super::usb_topology::i2c_compatible(name, node.property("compatible")) => {
                    &[super::HPM_MANAGER][..]
                }
                super::HPM_MANAGER => &[super::HPM][..],
                _ => &[],
            });
        }
        let index = resources.len();
        resources.push(PowerResource {
            name: path.clone(),
            clocks: clocks.clone(),
            clock_role,
            power: power.clone(),
            owner,
            admission: Admission::Required,
        });
        nodes.push(node.clone());
        parents.push(Some(parent));
        for child in node
            .children()
            .ok_or_else(|| refuse!("{path} has an invalid child", path))?
            .into_iter()
            .rev()
        {
            pending.push((
                index,
                child,
                provider_ancestors.as_ref().unwrap_or(&clocks).clone(),
                power.clone(),
                owner,
                views.clone(),
            ));
        }
    }
    Ok(ResourceInventory {
        resources,
        nodes,
        parents,
        ranges,
    })
}

pub(super) fn power_states(
    root: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
    chip_id: u32,
) -> Result<Option<PowerStates>> {
    let io = root.child("arm-io");
    let io = io.as_ref().unwrap_or(root);
    let Some(parent) = io.child("pmgr").filter(|node| {
        SOCS.iter()
            .any(|soc| soc.chip_id == chip_id && node.compatible(soc.pmgr))
    }) else {
        return Ok(None);
    };
    let node = register_node(io, &parent)?;
    decode_power_states(root, &node, translate).map(Some)
}

/// Select the register-bearing implementation of a PMGR interface. On T8152,
/// `pmgr2,arch` publishes the controller and `pmgr2,t8152` owns the physical
/// groups and device table.
/// Selection grants neither PSD service-gate semantics nor lifecycle coverage.
pub(super) fn register_node<'a>(io: &Node<'a>, parent: &Node<'a>) -> Result<Node<'a>> {
    if !parent.compatible(PMGR2_ARCH) {
        return Ok(parent.clone());
    }
    io.child("pmgr-child")
        .filter(|node| node.compatible(PMGR2_IMPLEMENTATION))
        .ok_or_else(|| refuse!("pmgr2,arch has no qualified register-bearing pmgr-child"))
}

/// Normalize firmware records while preserving complete graph validation.
/// Optional CPU/bridge register models are checked.
fn decode_power_states(
    root: &Node<'_>,
    node: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
) -> Result<PowerStates> {
    let states = source_power_states(root, node, translate)?;
    validate_power_graph(&states)?;
    Ok(states)
}

/// Complete original metadata, without granting lifecycle coverage. Private
/// resource preparation keeps any independent graph requirement explicitly.
pub(super) fn source_power_states(
    root: &Node<'_>,
    node: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
) -> Result<PowerStates> {
    let table = decode_domain_table(node, Dies::of(root, node)?, translate)?;
    let inventory = inventory_with_nodes(root)?;
    let implicit = implicit::resolve(node, &table.domains, &inventory, translate)?;
    let states = PowerStates {
        blocks: table.blocks,
        registers: table.registers,
        domains: table.domains,
        resources: inventory.resources,
        implicit,
    };
    Ok(states)
}

pub(super) fn validate_power_graph(states: &PowerStates) -> Result<()> {
    let power = states.power_requirements()?;
    let consumers = states
        .resources
        .iter()
        .enumerate()
        .map(|(id, resource)| Consumer {
            id,
            name: &resource.name,
            clocks: resource.activated_clocks(),
            power: &power[id],
            sinks: LifecycleSinks::default(),
            admission: resource.admission,
        })
        .collect::<Vec<_>>();
    // Validate the entire graph even when a broken record is unreferenced.
    coverage::analyze(&states.domains, &consumers)
        .map_err(|error| refuse!("PMGR coverage: {error}", ?error))?;
    Ok(())
}

struct DomainTable {
    blocks: Vec<(u64, u64)>,
    registers: Vec<u64>,
    domains: Vec<Domain>,
}

/// Record normalization is separate from the full resource inventory and
/// coverage checks. This table alone never establishes lifecycle admission.
fn decode_domain_table(
    node: &Node<'_>,
    dies: Dies,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
) -> Result<DomainTable> {
    const PATH: &str = "/arm-io/pmgr";
    let reg = super::apertures::registers(node).ok_or(refuse!("PMGR reg has incomplete pairs"))?;
    let (layout, raw_blocks) =
        Layout::select(node.property("ps-regs"), node.property("ps-groups"))?;
    let blocks = raw_blocks
        .as_chunks::<4>()
        .0
        .iter()
        .map(|word| u32::from_le_bytes(*word))
        .collect::<Vec<_>>();
    let devices = node.property("devices").unwrap_or_default();
    if !raw_blocks.len().is_multiple_of(12) || !devices.len().is_multiple_of(PMGR_DEVICE) {
        return Err(refuse!(
            "{PATH} power-state groups or devices is not whole entries",
            PATH
        ));
    }
    let mut spans: Vec<Option<(u64, u64)>> = alloc::vec![None; blocks.len() / 3];
    let mut registers = Vec::new();
    let mut domains = Vec::new();
    for (number, entry) in devices.as_chunks::<PMGR_DEVICE>().0.iter().enumerate() {
        let record = Record::parse(layout, entry)
            .within(|| refuse!("{PATH} record {number}", PATH, number))?;
        let parents = record
            .parents
            .map(|parent| (parent != 0).then(|| domain_id(parent)));
        let address = if let Some(location) = record.location {
            let (index, block) = (u64::from(location.index), location.group);
            let cells = blocks.get(block * 3..block * 3 + 3).ok_or_else(|| {
                refuse!(
                    "{PATH} device names absent power-state block {block}",
                    PATH,
                    block
                )
            })?;
            let &(range, bytes) = reg.get(cells[0] as usize).ok_or_else(|| {
                refuse!("{PATH} ps-regs block {block} names absent reg", PATH, block)
            })?;
            let offset = u64::from(cells[1])
                .checked_add(index * 8)
                .filter(|offset| offset.checked_add(4).is_some_and(|end| end <= bytes))
                .ok_or_else(|| refuse!("{PATH} power state exceeds reg bank", PATH))?;
            let view = translate(range, bytes).ok_or_else(|| {
                refuse!(
                    "{PATH} power-state view crosses its actual bus domain",
                    PATH
                )
            })?;
            let physical = view
                .checked_add(offset)
                .ok_or_else(|| refuse!("{PATH} power-state physical word overflows", PATH))?;
            let base = view
                .checked_add(u64::from(cells[1]))
                .ok_or_else(|| refuse!("{PATH} power-state base overflows", PATH))?;
            let end = physical
                .checked_add(4)
                .filter(|_| base.checked_add(index * 8) == Some(physical))
                .ok_or_else(|| refuse!("{PATH} ps-regs block is not one contiguous range", PATH))?;
            let span = spans[block].get_or_insert((base, 0));
            span.1 = span.1.max(end - base);
            if !registers.contains(&physical) {
                registers.push(physical);
            }
            Some(physical)
        } else {
            None
        };
        domains.push(Domain {
            name: record.name.into(),
            id: domain_id(record.id),
            address,
            parents,
        });
    }
    // Each further die repeats die 0's records in its own PMGR instance.
    // `_enableDeviceGated(device, ..., die)` (mac14j KC VA 0xfffffe0009b6990c)
    // indexes per-die state by `die * 800 + device` and passes its own die to
    // the parent walk (0xfffffe0009b69ac4), so parents stay on that die.
    let spans = spans.into_iter().flatten().collect::<Vec<_>>();
    let mut blocks = spans.clone();
    let first = domains.len();
    let first_registers = registers.len();
    for die in 1..dies.count {
        for &(base, bytes) in &spans {
            blocks.push((dies.place(die, base)?, bytes));
        }
        for index in 0..first_registers {
            registers.push(dies.place(die, registers[index])?);
        }
        for index in 0..first {
            let domain = &domains[index];
            let copy = Domain {
                name: domain.name.clone(),
                id: DomainId { die, ..domain.id },
                address: domain
                    .address
                    .map(|address| dies.place(die, address))
                    .transpose()?,
                parents: domain
                    .parents
                    .map(|parent| parent.map(|parent| DomainId { die, ..parent })),
            };
            domains.push(copy);
        }
    }
    registers.sort_unstable();
    if registers.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(refuse!(
            "{PATH} die-stride overlaps another die's power states",
            PATH
        ));
    }
    Ok(DomainTable {
        blocks,
        registers,
        domains,
    })
}

#[cfg(test)]
mod tests;
