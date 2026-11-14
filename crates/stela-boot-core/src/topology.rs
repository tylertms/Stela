//! Copied firmware processor identity and per-processor CPUID observations.
//! Every enabled, healthy logical processor can run one guest PE: the first
//! thread of each physical core is assigned before any SMT sibling.

use crate::Result;
use alloc::vec::Vec;
use stela_contracts::refuse;

/// Logical processors the native executor can own. Hosts with more run on
/// the first `MAX_CORES` of `select`'s order.
pub const MAX_CORES: usize = 64;

/// Logical processors the executor owns from `available` selectable ones.
/// Without a request this is `min(available, MAX_CORES)`; an explicit count
/// narrows it and refuses when it exceeds either bound.
pub fn assignment_count(requested: Option<usize>, available: usize) -> Result<usize> {
    let assignable = available.min(MAX_CORES);
    let count = requested.unwrap_or(assignable);
    if assignable == 0 || count == 0 || count > assignable {
        return Err(refuse!(
            "native assignment is outside the host's processor roster"
        ));
    }
    Ok(count)
}
pub const MAX_CPUID: usize = 48;
pub const MAX_MSRS: usize = 64;

pub const BSP: u32 = 1;
pub const ENABLED: u32 = 1 << 1;
pub const HEALTHY: u32 = 1 << 2;

/// Number of guest PEs and host processors admitted by both the host
/// roster and the source profile. An explicit request must fit both.
pub fn guest_assignment_count(
    requested: Option<usize>,
    available: usize,
    source_slots: usize,
) -> Result<usize> {
    assignment_count(requested, available.min(source_slots))
}

/// Coordinates copied from UEFI MP Services Location/Location2. Legacy
/// coordinates flatten the core identity within a package.
#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Location {
    pub package: u32,
    pub module: u32,
    pub tile: u32,
    pub die: u32,
    pub core: u32,
}

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Processor {
    pub processor_number: u32,
    pub apic_id: u64,
    pub flags: u32,
    /// 1: original three-level UEFI Location; 2: extended Location2.
    pub topology_version: u32,
    pub location: Location,
    pub thread: u32,
}

impl Processor {
    /// Enabled, and healthy unless it is the bootstrap processor: the
    /// processor this code runs on works, whatever its firmware health bit
    /// says (Linux reads no health bit at all). An AP marked unhealthy, or
    /// left out by [`select_observed`], runs no PE.
    pub fn available(self) -> bool {
        self.flags & ENABLED != 0 && self.flags & (HEALTHY | BSP) != 0
    }
}

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Cpuid {
    pub leaf: u32,
    pub subleaf: u32,
    pub eax: u32,
    pub ebx: u32,
    pub ecx: u32,
    pub edx: u32,
}

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Msr {
    pub index: u32,
    pub value: u64,
}

/// The core class reported by the processor's CPUID interface. Unknown
/// includes unsupported leaves and reserved class values.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum CoreClass {
    #[default]
    Unknown,
    Performance,
    Efficiency,
    LowPower,
}

/// Invalid input to a host/guest core assignment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlacementError {
    Cardinality,
    Capacity,
    Bootstrap,
}

/// Assign guest slots to host indices without changing guest slot order.
/// Host index zero runs the guest boot slot. Exact class matches are reserved
/// before an unmatched guest can consume another class's worker. Unknown
/// workers, then other unused workers, fill any remaining slots in host order.
/// Homogeneous host classes keep the original BSP-first assignment. When the
/// guest has fewer slots, choose from the complete host roster; a
/// matching class beyond the first guest-count workers remains eligible.
pub fn placement(
    guest: &[CoreClass],
    host: &[CoreClass],
    boot: usize,
) -> Result<Vec<usize>, PlacementError> {
    if guest.is_empty() || guest.len() > host.len() {
        return Err(PlacementError::Cardinality);
    }
    if host.len() > MAX_CORES {
        return Err(PlacementError::Capacity);
    }
    if boot >= guest.len() {
        return Err(PlacementError::Bootstrap);
    }
    let original: Vec<_> = (0..guest.len())
        .map(|slot| {
            if slot == boot {
                0
            } else if slot < boot {
                slot + 1
            } else {
                slot
            }
        })
        .collect();
    if host.windows(2).all(|pair| pair[0] == pair[1]) {
        return Ok(original);
    }
    let mut assigned = alloc::vec![usize::MAX; guest.len()];
    let mut used = alloc::vec![false; host.len()];
    assigned[boot] = 0;
    used[0] = true;
    for (slot, class) in guest.iter().enumerate() {
        if assigned[slot] != usize::MAX || *class == CoreClass::Unknown {
            continue;
        }
        if let Some(worker) = host
            .iter()
            .enumerate()
            .position(|(worker, kind)| !used[worker] && kind == class)
        {
            assigned[slot] = worker;
            used[worker] = true;
        }
    }
    // Low-power and efficiency cores can host the other's E-class work once
    // every exact match has been reserved.
    for (slot, class) in guest.iter().enumerate() {
        let alternate = match class {
            CoreClass::Efficiency => CoreClass::LowPower,
            CoreClass::LowPower => CoreClass::Efficiency,
            _ => continue,
        };
        if assigned[slot] == usize::MAX
            && let Some(worker) = host
                .iter()
                .enumerate()
                .position(|(worker, kind)| !used[worker] && *kind == alternate)
        {
            assigned[slot] = worker;
            used[worker] = true;
        }
    }
    for unknown_only in [true, false] {
        for worker in &mut assigned {
            if *worker != usize::MAX {
                continue;
            }
            if let Some(index) = host.iter().enumerate().position(|(index, kind)| {
                !used[index] && (!unknown_only || *kind == CoreClass::Unknown)
            }) {
                *worker = index;
                used[index] = true;
            }
        }
    }
    Ok(assigned)
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub records: [Cpuid; MAX_CPUID],
    pub count: u32,
    pub msrs: [Msr; MAX_MSRS],
    pub msr_count: u32,
    /// The firmware's CR0/CR4/EFER, without inventing OS XSTATE enablement.
    pub cr0: u64,
    pub cr3: u64,
    pub cr4: u64,
    pub efer: u64,
    pub pat: u64,
    pub pat_valid: bool,
    pub apic_base: u64,
    pub xcr0: u64,
    pub xcr0_valid: bool,
    /// AMD fixed RdMem/WrMem were read with the visibility gate open,
    /// then the original SYSCFG was restored. Masked zeros are not evidence.
    pub amd_fixed_raw: bool,
}

impl Default for Snapshot {
    fn default() -> Self {
        Self {
            records: [Cpuid::default(); MAX_CPUID],
            count: 0,
            msrs: [Msr::default(); MAX_MSRS],
            msr_count: 0,
            cr0: 0,
            cr3: 0,
            cr4: 0,
            efer: 0,
            pat: 0,
            pat_valid: false,
            apic_base: 0,
            xcr0: 0,
            xcr0_valid: false,
            amd_fixed_raw: false,
        }
    }
}

impl Snapshot {
    pub fn get(&self, leaf: u32, subleaf: u32) -> Option<Cpuid> {
        self.records
            .get(..self.count as usize)?
            .iter()
            .find(|record| record.leaf == leaf && record.subleaf == subleaf)
            .copied()
    }

    /// Prefer the original extended APIC ID. The legacy eight-bit field is
    /// accepted only when no implemented extended topology leaf supplies one.
    pub fn apic_id(&self) -> Option<u32> {
        let identity = self.get(0, 0)?;
        let maximum = identity.eax;
        if maximum < 1 {
            return None;
        }
        let basic = self.get(1, 0)?;
        for leaf in [0x1f, 0xb] {
            if maximum >= leaf {
                // A missing declared observation is not an unsupported leaf.
                let record = self.get(leaf, 0)?;
                if record.ebx & 0xffff != 0 && (record.ecx >> 8) & 0xff != 0 {
                    return Some(record.edx);
                }
            }
        }
        if [identity.ebx, identity.edx, identity.ecx] == stela_compiler::TargetCpu::AMD_VENDOR {
            let extended = self.get(0x8000_0000, 0)?;
            if extended.eax >= 0x8000_001e && self.get(0x8000_0001, 0)?.ecx & (1 << 22) != 0 {
                return self.get(0x8000_001e, 0).map(|record| record.eax);
            }
        }
        Some(basic.ebx >> 24)
    }

    pub fn matches(&self, processor: Processor) -> bool {
        self.apic_id()
            .is_some_and(|id| u64::from(id) == processor.apic_id)
    }

    /// Intel SDM CPUID.1AH and AMD CPUID.80000026H, decoded as Linux's
    /// `get_topology_cpu_type` and `parse_topology_amd` do. Each snapshot
    /// belongs to one processor; the BSP's class cannot describe an AP.
    pub fn core_class(&self) -> CoreClass {
        use stela_compiler::TargetCpu;
        let Some(identity) = self.get(0, 0) else {
            return CoreClass::Unknown;
        };
        match [identity.ebx, identity.edx, identity.ecx] {
            TargetCpu::INTEL_VENDOR if identity.eax >= 0x1a => {
                // Intel permits this leaf on non-hybrid processors too.
                match self.get(0x1a, 0).map(|record| record.eax >> 24) {
                    Some(0x20) => CoreClass::Efficiency,
                    Some(0x40) => CoreClass::Performance,
                    _ => CoreClass::Unknown,
                }
            }
            TargetCpu::AMD_VENDOR
                if self
                    .get(0x8000_0000, 0)
                    .is_some_and(|record| record.eax >= 0x8000_0026) =>
            {
                let Some(record) = self.get(0x8000_0026, 0) else {
                    return CoreClass::Unknown;
                };
                // A valid level with the Heterogeneous Core Topology bit.
                if record.eax & (1 << 30) == 0
                    || record.ebx & 0xffff == 0
                    || record.ecx >> 8 & 0xff == 0
                {
                    return CoreClass::Unknown;
                }
                match record.ebx >> 28 {
                    0 => CoreClass::Performance,
                    1 => CoreClass::Efficiency,
                    2 => CoreClass::LowPower,
                    _ => CoreClass::Unknown,
                }
            }
            _ => CoreClass::Unknown,
        }
    }
}

/// Conservative compiler policy from the assigned workers' installed
/// observations. This cannot grant execution: registration and each native
/// entry must still validate the destination's current hardware and controls.
pub fn common_native_target(
    observations: &[&Snapshot],
) -> Result<stela_compiler::TargetCpu, CapabilityRefusal> {
    use stela_compiler::TargetCpu;
    const SSSE3: u32 = 1 << 9;
    const SSE41: u32 = 1 << 19;
    const SSE42: u32 = 1 << 20;
    const PCLMULQDQ: u32 = 1 << 1;
    const POPCNT: u32 = 1 << 23;
    const AES: u32 = 1 << 25;
    const SHA: u32 = 1 << 29;
    const GFNI: u32 = 1 << 8;
    const LZCNT: u32 = 1 << 5;
    const CX8: u32 = 1 << 8;
    const CX16: u32 = 1 << 13;
    const MOVBE: u32 = 1 << 22;
    const AVX: u32 = 1 << 28;
    const BMI1: u32 = 1 << 3;
    const BMI2: u32 = 1 << 8;
    const FMA: u32 = 1 << 12;
    const F16C: u32 = 1 << 29;
    const AVX2: u32 = 1 << 5;
    /// XCR0's SSE and AVX bits: the OS saves XMM and YMM state.
    const XCR0_YMM: u64 = 0x6;
    /// AMD's first family whose PDEP and PEXT are not microcoded (Zen 3).
    const FAST_PDEP_FAMILY: u32 = 0x19;
    if observations.is_empty() || observations.len() > MAX_CORES {
        return Err(CapabilityRefusal::InvalidSnapshot);
    }
    let (mut ecx, mut edx, mut structured_ebx, mut structured_ecx) =
        (u32::MAX, u32::MAX, u32::MAX, u32::MAX);
    let mut atomic16 = true;
    let mut lzcnt = true;
    // VEX-encoded scalar GPR extensions, which use no extended state:
    // Intel's Pentium and Celeron parts without AVX fault on them.
    let mut vex = true;
    // VEX vector recipes: AVX hardware whose XMM and YMM state the
    // installed worker's XCR0 enables, which the interrupt entries save.
    let mut ymm = true;
    // AMD's Zen 1 and Zen 2, and Hygon's Zen-derived cores, microcode PDEP
    // and PEXT; the display family tells them apart.
    let mut fast_pdep = true;
    for snapshot in observations {
        let vendor = snapshot
            .get(0, 0)
            .filter(|record| record.eax >= 1)
            .ok_or(CapabilityRefusal::InvalidSnapshot)?;
        let basic = snapshot
            .get(1, 0)
            .ok_or(CapabilityRefusal::InvalidSnapshot)?;
        ecx &= basic.ecx;
        edx &= basic.edx;
        let structured = if vendor.eax >= 7 {
            snapshot.get(7, 0).unwrap_or_default()
        } else {
            Cpuid::default()
        };
        structured_ebx &= structured.ebx;
        structured_ecx &= structured.ecx;
        lzcnt &= snapshot
            .get(0x8000_0000, 0)
            .is_some_and(|maximum| maximum.eax >= 0x8000_0001)
            && snapshot
                .get(0x8000_0001, 0)
                .is_some_and(|record| record.ecx & LZCNT != 0);
        let vendor = [vendor.ebx, vendor.edx, vendor.ecx];
        atomic16 &= TargetCpu::atomic_load16_guaranteed(vendor, basic.ecx);
        vex &= vendor != TargetCpu::INTEL_VENDOR || basic.ecx & AVX != 0;
        ymm &= basic.ecx & AVX != 0 && snapshot.xcr0_valid && snapshot.xcr0 & XCR0_YMM == XCR0_YMM;
        let base_family = (basic.eax >> 8) & 0xf;
        let family = if base_family == 0xf {
            base_family + ((basic.eax >> 20) & 0xff)
        } else {
            base_family
        };
        let zen = vendor == TargetCpu::AMD_VENDOR || vendor == TargetCpu::HYGON_VENDOR;
        fast_pdep &= !zen || family >= FAST_PDEP_FAMILY;
    }
    let bmi2 = vex && structured_ebx & BMI2 != 0;
    let mut target = match (ecx & SSSE3 != 0, ecx & SSE41 != 0, ecx & SSE42 != 0) {
        (true, true, true) => TargetCpu::Sse42,
        (true, true, false) => TargetCpu::Sse41,
        (true, false, _) => TargetCpu::Ssse3,
        _ => TargetCpu::Baseline,
    };
    for (present, with) in [
        (
            ecx & PCLMULQDQ != 0,
            TargetCpu::with_pclmulqdq as fn(_) -> _,
        ),
        (ecx & POPCNT != 0, TargetCpu::with_popcnt),
        (ecx & AES != 0, TargetCpu::with_aes),
        (structured_ebx & SHA != 0, TargetCpu::with_sha),
        (structured_ecx & GFNI != 0, TargetCpu::with_gfni),
        (lzcnt, TargetCpu::with_lzcnt),
        (edx & CX8 != 0, TargetCpu::with_cx8),
        (ecx & CX16 != 0, TargetCpu::with_cx16),
        (atomic16, TargetCpu::with_atomic_load16),
        (vex && structured_ebx & BMI1 != 0, TargetCpu::with_bmi1),
        (bmi2, TargetCpu::with_bmi2),
        (bmi2 && fast_pdep, TargetCpu::with_fast_pdep),
        (ecx & MOVBE != 0, TargetCpu::with_movbe),
        (ymm && ecx & FMA != 0, TargetCpu::with_fma),
        (ymm && ecx & F16C != 0, TargetCpu::with_f16c),
        (ymm && structured_ebx & AVX2 != 0, TargetCpu::with_avx2),
    ] {
        if present {
            target = with(target);
        }
    }
    Ok(target)
}

/// The native executor's floor is legacy XMM state. It
/// derives this floor on each physical core rather than borrowing the
/// BSP's CPUID or firmware OSXSAVE state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapabilityRefusal {
    InvalidSnapshot,
    /// The core lacks these features of the floor.
    NativeFloor(Missing),
    FiveLevelPaging,
    MemoryTypes,
}

/// The native executor's CPU floor, by the name people know each feature
/// by: x86-64-v2's (SSE3, SSSE3, CX16, SSE4.1, SSE4.2, POPCNT, LAHF/SAHF)
/// and the x86-64 baseline every 64-bit PC has, with PAT and an enabled
/// local APIC.
const FLOOR: [&str; 22] = [
    "SSE3",
    "SSSE3",
    "CX16",
    "SSE4.1",
    "SSE4.2",
    "POPCNT",
    "x87",
    "TSC",
    "MSR",
    "PAE",
    "CX8",
    "APIC",
    "MTRR",
    "PAT",
    "FXSR",
    "SSE",
    "SSE2",
    "LAHF/SAHF",
    "NX",
    "64-bit mode",
    "a valid PAT setting",
    "an enabled local APIC",
];
/// [`FLOOR`]'s CPUID bits: leaf 1's ECX, leaf 1's EDX, then leaf
/// 0x8000_0001's ECX and EDX.
const ECX_BITS: [u32; 6] = [0, 9, 13, 19, 20, 23];
const EDX_BITS: [u32; 11] = [0, 4, 5, 6, 8, 9, 12, 16, 24, 25, 26];
const EXTENDED_ECX_BITS: [u32; 1] = [0];
const EXTENDED_EDX_BITS: [u32; 2] = [20, 29];

/// Features of [`FLOOR`] a core lacks, one bit each, in its order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Missing(u32);

impl Missing {
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// The names of the missing features.
    pub fn names(self) -> impl Iterator<Item = &'static str> {
        FLOOR
            .into_iter()
            .enumerate()
            .filter(move |(index, _)| self.0 & (1 << index) != 0)
            .map(|(_, name)| name)
    }

    /// The stop a core without the floor gives, naming what it lacks and
    /// what Stela needs.
    pub fn refusal(self) -> crate::Refusal {
        let mut lacks = alloc::string::String::new();
        let names: Vec<&str> = self.names().collect();
        for (index, name) in names.iter().enumerate() {
            if index > 0 {
                lacks.push_str(if index + 1 == names.len() {
                    " and "
                } else {
                    ", "
                });
            }
            lacks.push_str(name);
        }
        refuse!(
            "this PC's CPU lacks {lacks}; Stela needs an x86-64-v2 CPU (Intel Nehalem, AMD \
             Bulldozer or newer)",
            lacks = %lacks
        )
    }
}

impl CapabilityRefusal {
    /// The stop this refusal gives, for messages: a missing floor names
    /// the features ([`Missing::refusal`]).
    pub fn refusal(self, what: &'static str) -> crate::Refusal {
        match self {
            Self::NativeFloor(missing) => missing.refusal(),
            _ => refuse!(what),
        }
    }
}

impl Snapshot {
    pub fn qualify_native(&self, five_level: bool) -> Result<(), CapabilityRefusal> {
        if self.count as usize > MAX_CPUID || self.msr_count as usize > MAX_MSRS {
            return Err(CapabilityRefusal::InvalidSnapshot);
        }
        // A missing leaf lacks every feature it reports.
        let basic = self.get(1, 0);
        let extended = self.get(0x8000_0001, 0);
        let registers = [
            (basic.map(|leaf| leaf.ecx), &ECX_BITS[..]),
            (basic.map(|leaf| leaf.edx), &EDX_BITS[..]),
            (extended.map(|leaf| leaf.ecx), &EXTENDED_ECX_BITS[..]),
            (extended.map(|leaf| leaf.edx), &EXTENDED_EDX_BITS[..]),
        ];
        let present = registers
            .into_iter()
            .flat_map(|(value, bits)| {
                bits.iter()
                    .map(move |bit| value.is_some_and(|v| v & (1 << bit) != 0))
            })
            .chain([self.pat_valid, self.apic_base & (1 << 11) != 0]);
        let missing = Missing(
            present
                .enumerate()
                .filter(|(_, present)| !present)
                .fold(0, |bits, (index, _)| bits | 1 << index),
        );
        if !missing.is_empty() {
            return Err(CapabilityRefusal::NativeFloor(missing));
        }
        if five_level && !self.get(7, 0).is_some_and(|leaf| leaf.ecx & (1 << 16) != 0) {
            return Err(CapabilityRefusal::FiveLevelPaging);
        }
        Ok(())
    }

    /// Require the installed memory-type/routing image to equal the
    /// intended source image before sharing RAM or Stela metadata. Initial
    /// offline installation may establish this equality; TSC_ADJUST is separate.
    pub fn same_memory_types(&self, other: &Self) -> Result<(), CapabilityRefusal> {
        use crate::memory_types::amd;
        // What the installation writes of a register: all of it, but for
        // the legacy hole's DRAM routing, which stays each processor's own.
        let raw = self.amd_fixed_raw;
        let installed = move |msr: &Msr| {
            if raw && amd::legacy_hole(msr.index) {
                (msr.index, msr.value & !amd::FIXED_ROUTING)
            } else {
                (msr.index, msr.value)
            }
        };
        fn memory(index: u32) -> bool {
            index == 0xfe
                || index == 0x2ff
                || (0x200..=0x21f).contains(&index)
                || matches!(index, 0x250 | 0x258 | 0x259 | 0x268..=0x26f)
                || matches!(index, 0xc001_0010 | 0xc001_0016..=0xc001_001a | 0xc001_001d)
        }
        let left = self
            .msrs
            .get(..self.msr_count as usize)
            .ok_or(CapabilityRefusal::InvalidSnapshot)?;
        let right = other
            .msrs
            .get(..other.msr_count as usize)
            .ok_or(CapabilityRefusal::InvalidSnapshot)?;
        if self.amd_fixed_raw != other.amd_fixed_raw
            || !self.pat_valid
            || !other.pat_valid
            || self.pat != other.pat
            || !left
                .iter()
                .filter(|msr| memory(msr.index))
                .map(installed)
                .eq(right.iter().filter(|msr| memory(msr.index)).map(installed))
            || !left.iter().any(|msr| msr.index == 0xfe)
            || !left.iter().any(|msr| msr.index == 0x2ff)
        {
            return Err(CapabilityRefusal::MemoryTypes);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    NoBootstrapProcessor,
    MultipleBootstrapProcessors,
    BootstrapUnavailable,
    InvalidTopology,
    DuplicateProcessorNumber,
    DuplicateApicId,
    DuplicateThreadLocation,
    MixedTopologyVersions,
    InvalidAssignment,
}

/// Whether two listed processors share one package, core and thread while
/// every APIC ID is distinct: firmware that fills no usable coordinates.
pub fn locations_collide(processors: &[Processor]) -> bool {
    bootstrap_index(processors) == Err(Refusal::DuplicateThreadLocation)
}

/// The two APIC ID shifts [`place_by_apic_id`] takes, for an AMD processor
/// without CPUID leaf 0xb, such as family 17h models 00h to 0fh, from
/// leaves 8000_0008h (ECX) and 8000_001Eh (EBX) as Linux's
/// `topology_amd.c` reads them: the threads of a core give the thread bits,
/// and ApicIdCoreIdSize, or the processor count where it is zero, the bits
/// of thread and core together.
pub fn amd_apic_id_shifts(size_ecx: u32, unit_ebx: u32) -> (u32, u32) {
    let bits = |count: u32| count.next_power_of_two().trailing_zeros();
    let thread = bits((unit_ebx >> 8 & 0xff) + 1);
    let core = match size_ecx >> 12 & 0xf {
        0 => bits((size_ecx & 0xff) + 1),
        size => size,
    };
    (thread, core.max(thread))
}

/// Place every processor by its APIC ID, as Linux does, for firmware whose
/// coordinates [`locations_collide`]. `shifts` are CPUID leaf 0xb's two
/// level shifts from the bootstrap processor: the bits of an APIC ID that
/// name the thread, and those that name the thread and the core. Without
/// them every APIC ID is a core of its own. The records become the
/// three-level kind, which these coordinates are.
pub fn place_by_apic_id(processors: &mut [Processor], shifts: Option<(u32, u32)>) {
    let shifts = shifts.filter(|&(thread, core)| thread <= core && core < 32);
    for processor in processors {
        let id = processor.apic_id;
        let (package, core, thread) = match shifts {
            Some((thread, core)) => (
                id >> core,
                (id >> thread) & ((1 << (core - thread)) - 1),
                id & ((1 << thread) - 1),
            ),
            None => (0, id, 0),
        };
        processor.location = Location {
            package: package as u32,
            core: core as u32,
            ..Location::default()
        };
        processor.thread = thread as u32;
        processor.topology_version = 1;
    }
}

fn bootstrap_index(processors: &[Processor]) -> Result<usize, Refusal> {
    let mut bsp = None;
    for (index, processor) in processors.iter().enumerate() {
        if !matches!(processor.topology_version, 1 | 2) {
            return Err(Refusal::InvalidTopology);
        }
        if processor.flags & BSP != 0 {
            if bsp.replace(index).is_some() {
                return Err(Refusal::MultipleBootstrapProcessors);
            }
            if !processor.available() {
                return Err(Refusal::BootstrapUnavailable);
            }
        }
        for previous in &processors[..index] {
            if processor.processor_number == previous.processor_number {
                return Err(Refusal::DuplicateProcessorNumber);
            }
            if processor.apic_id == previous.apic_id {
                return Err(Refusal::DuplicateApicId);
            }
            if processor.location == previous.location && processor.thread == previous.thread {
                return Err(Refusal::DuplicateThreadLocation);
            }
            if processor.topology_version != previous.topology_version {
                return Err(Refusal::MixedTopologyVersions);
            }
        }
    }
    bsp.ok_or(Refusal::NoBootstrapProcessor)
}

/// Return the original index of every enabled, healthy logical processor,
/// without renumbering its firmware identity: the BSP, then the first thread
/// of each other physical core by location, then each core's second thread by
/// location, and so on. Assignment takes a prefix (`assignment_count`), so a
/// host with enough physical cores gives every PE a core of its own. No AP is
/// started and no capability is inferred here.
pub fn select(processors: &[Processor]) -> Result<Vec<usize>, Refusal> {
    let bsp = bootstrap_index(processors)?;
    let mut candidates: Vec<_> = processors
        .iter()
        .enumerate()
        .filter(|(_, processor)| processor.available())
        .map(|(index, processor)| {
            (
                index,
                processor.location,
                (index != bsp, processor.thread, processor.processor_number),
            )
        })
        .collect();
    // The BSP leads its own core even when its thread number is nonzero.
    candidates.sort_by_key(|&(_, location, order)| (location, order));
    // A processor's rank is its position among its core's threads.
    let mut ranked: Vec<_> = Vec::with_capacity(candidates.len());
    for (position, &(index, location, order)) in candidates.iter().enumerate() {
        let rank = match position.checked_sub(1).map(|previous| ranked[previous]) {
            Some((rank, _, previous, _)) if previous == location => rank + 1,
            _ => 0usize,
        };
        ranked.push((rank, index, location, order));
    }
    ranked.sort_by_key(|&(rank, index, location, order)| (index != bsp, rank, location, order));
    Ok(ranked.into_iter().map(|(_, index, _, _)| index).collect())
}

/// [`select`], observing each processor among the first [`MAX_CORES`] of its
/// order once with `observe(slot, index, processor)`: `Ok(true)` keeps it,
/// `Ok(false)` leaves that AP out (its health bit is cleared, so that it
/// runs no PE and a later [`select`] of the same inventory agrees) and the
/// order is taken
/// again, observing those that moved into it. A bootstrap processor that
/// cannot be observed, and every `Err`, refuse.
pub fn select_observed<E>(
    processors: &mut [Processor],
    mut observe: impl FnMut(usize, usize, Processor) -> Result<bool, E>,
    refused: impl Fn(Refusal) -> E,
) -> Result<Vec<usize>, E> {
    let mut observed = alloc::vec![false; processors.len()];
    loop {
        let logical = select(processors).map_err(&refused)?;
        let mut left_out = false;
        for (slot, &index) in logical.iter().take(MAX_CORES).enumerate() {
            if observed[index] {
                continue;
            }
            if observe(slot, index, processors[index])? {
                observed[index] = true;
            } else if slot == 0 {
                return Err(refused(Refusal::BootstrapUnavailable));
            } else {
                processors[index].flags &= !HEALTHY;
                left_out = true;
            }
        }
        if !left_out {
            return Ok(logical);
        }
    }
}

/// Distinct physical cores among the enabled, healthy logical processors.
pub fn physical_cores(processors: &[Processor]) -> usize {
    let mut locations: Vec<_> = processors
        .iter()
        .filter(|processor| processor.available())
        .map(|processor| processor.location)
        .collect();
    locations.sort_unstable();
    locations.dedup();
    locations.len()
}

/// Enabled logical processors outside the requested roster.
/// Unlike admission, this includes enabled unhealthy APs: health does not prove
/// that their firmware execution has stopped. Disabled processors stay disabled.
/// No allocation or firmware access is needed at the post-EBS boundary.
pub fn enabled_unselected<'a>(
    processors: &'a [Processor],
    selected: &'a [u32],
) -> Result<impl Iterator<Item = usize> + Clone + 'a, Refusal> {
    let bsp = bootstrap_index(processors)?;
    if selected.is_empty() || selected.len() > MAX_CORES || selected[0] as usize != bsp {
        return Err(Refusal::InvalidAssignment);
    }
    for (slot, &index) in selected.iter().enumerate() {
        let available = processors
            .get(index as usize)
            .is_some_and(|processor| processor.available());
        if !available || selected[..slot].contains(&index) {
            return Err(Refusal::InvalidAssignment);
        }
    }
    Ok(processors
        .iter()
        .enumerate()
        .filter(move |(index, processor)| {
            processor.flags & ENABLED != 0 && !selected.contains(&(*index as u32))
        })
        .map(|(index, _)| index))
}

#[cfg(test)]
mod tests;
