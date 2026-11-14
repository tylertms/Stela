//! System mode, engine phase: runs in the Stela window after the loader has
//! loaded the engine's tables, switching to LA57 when the host has it. Host
//! physical memory is reached through the physmap.
//! Stela's host drivers address registers and DMA pages by identity, so
//! every access runs on the loader's identity map and its uncached device
//! mappings (`storage`). The hardware is probed once, before the host is
//! installed; `--preboot` builds the boot bundle from the installed
//! system's partition that probe opened.

mod reports;
mod run;
use crate::Result;
use reports::*;
use run::*;
use stela_contracts::refuse;
mod start;
pub(crate) use start::entry;
mod aml;
mod bluetooth;
mod boot_image;
pub(crate) mod bundle;
mod bundle_repair;
mod cursor;
mod display;
pub(crate) mod fdr;
mod firmware_requests;
mod gpu;
pub(crate) mod host;
mod input;
pub(crate) mod install;
pub(crate) mod interrupts;
mod kernel_work;
mod large_blocks;
pub(crate) use kernel_work::FirmwareContext;
mod network;
pub(crate) mod nor;
mod parallel;
#[cfg(feature = "nvk")]
pub(crate) mod pipelines;
pub(crate) mod power;
use stela_boot_core::boot_progress as progress;
mod remapping;
mod remote;
mod restore;
mod service;
mod services;
mod speculation;
pub(crate) mod storage;
mod store_file;
mod topology;
pub(crate) mod translations;
mod usb;
pub(crate) mod xstate;

use crate::{
    handoff::{Handoff, MAGIC},
    layout, output,
};
use alloc::{collections::BTreeMap, format, rc::Rc, string::String, vec::Vec};
use core::{
    arch::x86_64::{__cpuid, __cpuid_count},
    cell::RefCell,
    fmt,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
use host::{Config, RamRange};
use stela_boot_core::{
    console_lines,
    options::{Base, System},
    ram,
};
use stela_compiler::TargetCpu;
use stela_contracts::Chip;
use stela_engine::machine::Cpu;
use stela_engine::machine::CpuProfile;
use stela_engine::machine::DeviceRegister;
use stela_engine::machine::Event;
use stela_engine::machine::Host;
use stela_engine::machine::Machine;
use stela_engine::machine::PeerState;
use stela_engine::machine::Reset;
use stela_engine::machine::Stop;
use stela_engine::machine::Transition;
use stela_engine::machine::engine::Call;
use stela_engine::machine::engine::step_timing;
use stela_engine::machine::exception::Pstate;
use stela_engine::machine::identification::Speculation;
use stela_engine::machine::registers::encodings::DCZID_EL0;
use stela_engine::machine::system::Register;
use stela_loader::{apple_device_tree::DeviceTree, boot_bundle, installed};
use stela_runtime::machine::mount;
use stela_runtime::pc::{gpt::Guid, storage::Target};
use stela_runtime::restore::store::Verify;
use stela_soc::machine::AicInput;
use stela_soc::machine::AppleDevices;
use stela_soc::machine::ConsoleBusy;
use stela_soc::machine::ConsoleSink;
use stela_soc::machine::Range;
use stela_soc::machine::memory_resources::MemoryResources;
use stela_soc_data::aic::AicLayout;
use stela_soc_data::board;
use stela_soc_data::pmu::WallClock;

/// The most of the engine heap (at least 512 MiB) `--storage-ram` may
/// take for written blocks; a restore or recovery boot writes far less.
const RAM_STORAGE_LIMIT: usize = 64 << 20;
const LARGE: u64 = 2 * 1024 * 1024;
/// Synchronous exceptions and returns logged before the log goes quiet;
/// interrupts are logged up to `REPORTED_INTERRUPTS`. A guarded level's
/// synchronous exceptions other than SVC, which its monitor otherwise
/// handles without a trace, are always logged.
const REPORTED_EVENTS: u64 = 24;
const REPORTED_INTERRUPTS: u64 = 64;
const ESR_EC_SHIFT: u32 = 26;
/// ESR EC of an SVC from AArch64 state.
const EC_SVC64: u64 = 0x15;
/// Instruction and data aborts taken from a lower EL.
const EC_LOWER_ABORTS: [u64; 2] = [0x20, 0x24];

/// `stela.cfg`'s options (`stela_boot_core::options::System`) and the guest
/// RAM the engine sizes from the host.
struct Options {
    system: System,
    /// Guest DRAM backed by host RAM: all the host has free
    /// ([`guest_ram`]).
    ram_bytes: u64,
    /// The DRAM the Mac reports: the host's RAM, of which what Stela and
    /// the firmware keep shows as memory the kernel does not manage.
    dram_bytes: u64,
}

impl core::ops::Deref for Options {
    type Target = System;
    fn deref(&self) -> &System {
        &self.system
    }
}

impl core::ops::DerefMut for Options {
    fn deref_mut(&mut self) -> &mut System {
        &mut self.system
    }
}

/// Present the selected board through restored's original development-board
/// predicate, and boot a system so installed as the same board, whose Cryptex1
/// tickets carry the model. MobileGestalt's `HWModelStr` (`HW.target` in restored's log) is
/// the first string of the root `compatible` list. Keep the original hardware
/// model as its second string so platform matching still sees it, and keep the
/// root `target-sub-type` consistent with the development identity.
///
/// 26A434 restored accepts an FDR failure only when its last three characters
/// are `DEV` and the independent restore option is true. The assigned tree has
/// a fixed serialized span, so replace a conventional `AP` suffix when the
/// property's existing word-aligned slot can hold the result; otherwise use the
/// final three characters. The product model already has its own root property,
/// so omit its duplicate from `compatible` if the new development alias needs
/// that space. Preserve the original board and platform compatibility strings.
fn development_hardware_model(tree: &mut DeviceTree) -> Result<String> {
    let value = tree
        .property("/", "target-sub-type")
        .filter(|property| !property.placeholder)
        .ok_or(refuse!(
            "the selected Apple board has no concrete target-sub-type"
        ))?
        .value
        .clone();
    let compatible = tree
        .property("/", "compatible")
        .filter(|property| !property.placeholder)
        .ok_or(refuse!(
            "the selected Apple board has no concrete compatible list"
        ))?
        .value
        .clone();
    let product = tree
        .property("/", "model")
        .filter(|property| !property.placeholder)
        .and_then(|property| property.as_str())
        .ok_or(refuse!(
            "the selected Apple board has no concrete product model"
        ))?
        .as_bytes()
        .to_vec();
    let value = value.as_slice();
    let Some(value) = value.strip_suffix(&[0]) else {
        return Err(refuse!(
            "the selected Apple board target-sub-type is not NUL terminated"
        ));
    };
    if value.is_empty() || value.contains(&0) {
        return Err(refuse!(
            "the selected Apple board target-sub-type is not one string"
        ));
    }
    let target = core::str::from_utf8(value)
        .map_err(|_| refuse!("the selected Apple board target-sub-type is not UTF-8"))?;
    if !target.is_ascii() || target.len() < 4 {
        return Err(refuse!(
            "the selected Apple board target-sub-type is not a hardware model"
        ));
    }
    let target = if target.ends_with("DEV") {
        String::from(target)
    } else {
        let base = target.strip_suffix("AP").unwrap_or(target);
        let preferred = format!("{base}DEV");
        let available = (value.len() + 1).next_multiple_of(size_of::<u32>());
        if preferred.len() < available {
            preferred
        } else {
            format!("{}DEV", &target[..target.len() - 3])
        }
    };
    let mut encoded = target.as_bytes().to_vec();
    encoded.push(0);
    let entries: Vec<_> = compatible
        .split(|&byte| byte == 0)
        .filter(|entry| !entry.is_empty())
        .collect();
    let Some((hardware, rest)) = entries.split_first() else {
        return Err(refuse!("the selected Apple board compatible list is empty"));
    };
    if *hardware != value {
        return Err(refuse!(
            "the selected Apple board hardware model is not first in compatible"
        ));
    }
    // A prepared bundle may already carry the development identity.
    if value.ends_with(b"DEV") {
        return Ok(target);
    }
    let mut development_compatible = Vec::with_capacity(compatible.len());
    development_compatible.extend_from_slice(target.as_bytes());
    development_compatible.push(0);
    development_compatible.extend_from_slice(value);
    development_compatible.push(0);
    for entry in rest.iter().copied().filter(|entry| *entry != product) {
        development_compatible.extend_from_slice(entry);
        development_compatible.push(0);
    }
    if development_compatible.len() > compatible.len() {
        return Err(refuse!(
            "the selected Apple board compatible slot cannot hold a development alias"
        ));
    }
    development_compatible.resize(compatible.len(), 0);
    tree.set_property("/", "target-sub-type", encoded)
        .map_err(|error| refuse!("development target-sub-type: {error}", %error))?;
    tree.set_property("/", "compatible", development_compatible)
        .map_err(|error| refuse!("development compatible list: {error}", %error))?;
    Ok(target)
}

impl Options {
    fn parse(text: &str) -> Result<Self> {
        Ok(Self {
            system: System::parse_resolved(text)
                .map_err(stela_boot_core::options::Error::message)?,
            ram_bytes: 0,
            dram_bytes: 0,
        })
    }
}

/// The host CPU's compiler target, from CPUID, after setting Intel's data
/// operand independent timing mode where the CPU has it ([`enable_doit`]).
/// The host CPU's target. With `parallel` it declares no data operand
/// independent timing: other cores run PEs too, without its control, and
/// their atomics lock and fence, which Intel's guidance lists no timing for.
fn target(parallel: bool) -> TargetCpu {
    let (vendor, basic, extended) = (__cpuid(0), __cpuid(1), __cpuid_count(7, 0));
    let (ssse3, sse41, sse42) = (
        basic.ecx & (1 << 9) != 0,
        basic.ecx & (1 << 19) != 0,
        basic.ecx & (1 << 20) != 0,
    );
    let vendor_id = [vendor.ebx, vendor.edx, vendor.ecx];
    let vex = vendor_id != TargetCpu::INTEL_VENDOR || basic.ecx & (1 << 28) != 0;
    // AMD's Zen 1 and Zen 2, and Hygon's Zen-derived cores, microcode PDEP
    // and PEXT; the display family tells them apart.
    let base_family = (basic.eax >> 8) & 0xf;
    let family = if base_family == 0xf {
        base_family + ((basic.eax >> 20) & 0xff)
    } else {
        base_family
    };
    let zen = vendor_id == TargetCpu::AMD_VENDOR || vendor_id == TargetCpu::HYGON_VENDOR;
    let fast_pdep = !zen || family >= 0x19;
    let mut target = match (ssse3, sse41, sse42) {
        (true, true, true) => TargetCpu::Sse42,
        (true, true, false) => TargetCpu::Sse41,
        (true, false, _) => TargetCpu::Ssse3,
        _ => TargetCpu::Baseline,
    };
    for (present, with) in [
        (
            basic.ecx & (1 << 1) != 0,
            TargetCpu::with_pclmulqdq as fn(_) -> _,
        ),
        (basic.ecx & (1 << 23) != 0, TargetCpu::with_popcnt),
        (basic.ecx & (1 << 25) != 0, TargetCpu::with_aes),
        (basic.ecx & (1 << 13) != 0, TargetCpu::with_cx16),
        (basic.edx & (1 << 8) != 0, TargetCpu::with_cx8),
        // Optional plain 128-bit loads need a documented vendor guarantee.
        (
            TargetCpu::atomic_load16_guaranteed([vendor.ebx, vendor.edx, vendor.ecx], basic.ecx),
            TargetCpu::with_atomic_load16,
        ),
        // VEX-encoded scalar GPR extensions, which use no extended state:
        // Intel's Pentium and Celeron parts without AVX fault on them.
        (vex && extended.ebx & (1 << 3) != 0, TargetCpu::with_bmi1),
        (vex && extended.ebx & (1 << 8) != 0, TargetCpu::with_bmi2),
        (
            vex && extended.ebx & (1 << 8) != 0 && fast_pdep,
            TargetCpu::with_fast_pdep,
        ),
        (basic.ecx & (1 << 22) != 0, TargetCpu::with_movbe),
        (extended.ebx & (1 << 29) != 0, TargetCpu::with_sha),
        (
            [vendor.ebx, vendor.edx, vendor.ecx] == TargetCpu::INTEL_VENDOR,
            TargetCpu::with_line_atomic,
        ),
    ] {
        if present {
            target = with(target);
        }
    }
    // SAFETY: once on the one CPU, before any guest code runs.
    if !parallel && unsafe { enable_doit() } {
        target = target.with_doit();
    }
    target
}

/// IA32_UARCH_MISC_CTL bit 0: data operand independent timing mode.
const UARCH_MISC_CTL_DOITM: u64 = 1;

/// Set Intel's data operand independent timing mode where the CPU
/// enumerates it, and whether it is now set (Intel, "Data Operand
/// Independent Timing Instruction Guidance"). Stela runs guest code on this
/// CPU alone. Without the control there is no guarantee Stela can
/// establish: AMD documents none, and the Intel cores before Ice Lake and
/// Gracemont that always behave so are not told apart. The engine then
/// advertises no FEAT_DIT (`TargetCpu::data_independent_timing_target`).
///
/// # Safety
/// Ring 0, with no other code depending on IA32_UARCH_MISC_CTL.
unsafe fn enable_doit() -> bool {
    use crate::cpu::msr::UARCH_MISC_CTL;
    // IA32_ARCH_CAPABILITIES (CPUID 7 EDX 29) bit 12 enumerates the control.
    if !UARCH_MISC_CTL.implemented() {
        return false;
    }
    // SAFETY: the control is enumerated; ring 0.
    unsafe {
        let control = UARCH_MISC_CTL.read();
        UARCH_MISC_CTL.write(control | UARCH_MISC_CTL_DOITM);
        UARCH_MISC_CTL.read() & UARCH_MISC_CTL_DOITM != 0
    }
}

/// Back guest DRAM with every free host range, lowest first, and report
/// the backed bytes as the Mac's DRAM, so kernel and user-space policies
/// size themselves from memory that exists
/// (`stela_boot_core::host_memory::guest_dram`).
fn guest_ram(handoff: &Handoff, options: &mut Options) -> Result<Vec<RamRange>> {
    let mut ranges = Vec::new();
    let mut assigned = 0;
    for region in &handoff.guest[..handoff.guest_count] {
        ranges.push(RamRange {
            guest: options.dram_base + assigned,
            host: region.physical,
            bytes: region.bytes,
        });
        assigned += region.bytes;
    }
    if assigned < LARGE {
        return Err(refuse!("no free host RAM is left for the guest"));
    }
    options.ram_bytes = assigned;
    let dram = stela_boot_core::host_memory::guest_dram(handoff.installed, assigned);
    options.dram_bytes = dram.reported;
    output::arguments(format_args!(
        "GUEST DRAM backed={assigned:#x} reported={:#x} kept={:#x}\n",
        dram.reported, dram.kept
    ));
    Ok(ranges)
}

/// Stela's own test kernel reads no identification register, but the
/// canonical memory owner declares its checked memory from them: the
/// ID_AA64* and CTR_EL0 registers advertise what this engine implements, as
/// a board profile's do. Its DC ZVA block is Stela's choice of 64 bytes.
fn test_profile(target: TargetCpu, speculation: Speculation) -> CpuProfile {
    let chip = Chip::DEFAULT;
    let mut fixed = stela_engine::machine::identification::derive(chip, target, true, speculation);
    fixed.push((DCZID_EL0, 4));
    CpuProfile {
        name: "stela-test-kernel",
        chip,
        counter_frequency: 24_000_000,
        fixed,
        processors: Vec::new(),
    }
}

/// The profile of the board the bundle's DeviceTree describes
/// (`machine::board::board`), with the selected handoff PE running with EL2 as a VHE host. The
/// ID_AA64* and CTR_EL0 registers advertise what this engine implements,
/// capped at the chip's features (`machine::identification`); MIDR, MPIDR and
/// DCZID are the board's own values. Every other identification read refuses.
/// The CSV fields disclose the host's `speculation` guarantees.
fn board_profile(board: &board::Board, target: TargetCpu, speculation: Speculation) -> CpuProfile {
    let chip = board.cpus.chip;
    let mut fixed = stela_engine::machine::identification::derive(chip, target, true, speculation);
    report_steering(chip, target);
    // 64-byte DC ZVA blocks, enabled: XNU `osfmk/arm64/bzero.s` zeroes with
    // `dc zva` in fixed 64-byte steps.
    fixed.push((DCZID_EL0, 4));
    CpuProfile {
        name: chip.name(),
        chip,
        counter_frequency: 24_000_000,
        fixed,
        processors: board.cpus.processors.clone(),
    }
}

/// Report once which steered features the identification advertises on
/// this host and why the others are hidden
/// (`identification::Basis::Steered`): the guest kernel's
/// `hw.optional.arm.FEAT_*` sysctls and commpage bits follow the fields.
fn report_steering(chip: Chip, target: TargetCpu) {
    let steering = stela_engine::machine::identification::steering(chip, target);
    let names = |advertised: bool| {
        let mut names = String::new();
        for entry in steering
            .iter()
            .filter(|entry| entry.advertised == advertised)
        {
            if !names.is_empty() {
                names.push(',');
            }
            names.push_str(entry.feature);
        }
        if names.is_empty() {
            names.push('-');
        }
        names
    };
    let mut reasons = String::new();
    for entry in steering.iter().filter(|entry| !entry.advertised) {
        if !reasons.is_empty() {
            reasons.push_str("; ");
        }
        reasons.push_str(entry.feature);
        reasons.push_str(": ");
        reasons.push_str(entry.reason);
    }
    output::arguments(format_args!(
        "GUEST FEATURES advertised={} hidden={} because {}\n",
        names(true),
        names(false),
        if reasons.is_empty() { "-" } else { &reasons }
    ));
}

/// How long the host's serial port goes without a look once it gave no
/// input. Each look is a port read that a hypervisor takes as an exit to its
/// device model, waiting for that model's lock, and real hardware as a slow
/// bus cycle. While someone types, the port's 16-byte FIFO, more than a
/// millisecond at 115200 baud, is looked at every [`INPUT_TYPING_MICROS`];
/// once the console has been quiet for [`INPUT_TYPING_SECONDS`], every
/// [`INPUT_IDLE_MICROS`]. A real UART can then drop what a paste sends past
/// its FIFO before the first look; a hypervisor's port holds it.
const INPUT_TYPING_MICROS: u64 = 1000;
const INPUT_IDLE_MICROS: u64 = 20_000;
const INPUT_TYPING_SECONDS: u64 = 5;

/// The host serial port's input on its way to the guest's console. The
/// host's service reads the port ([`feed_console_input`]), holding no lock
/// of the devices': the console's device model asks for input at each of
/// its passes, under the lock every PE's device accesses wait for, and a
/// port read there holds them all for as long as a hypervisor's device
/// model takes to answer. The model takes what the service read.
struct ConsoleInput {
    bytes: [core::sync::atomic::AtomicU8; CONSOLE_INPUT_BYTES],
    /// Bytes put and taken so far: one service puts at a time and one
    /// device model takes.
    put: core::sync::atomic::AtomicUsize,
    taken: core::sync::atomic::AtomicUsize,
    /// The host counter before which the port is not looked at again, and
    /// at the last byte it gave.
    next: AtomicU64,
    last: AtomicU64,
}

/// A few of the port's FIFOs.
const CONSOLE_INPUT_BYTES: usize = 64;

static CONSOLE_INPUT: ConsoleInput = ConsoleInput {
    bytes: [const { core::sync::atomic::AtomicU8::new(0) }; CONSOLE_INPUT_BYTES],
    put: core::sync::atomic::AtomicUsize::new(0),
    taken: core::sync::atomic::AtomicUsize::new(0),
    next: AtomicU64::new(0),
    last: AtomicU64::new(0),
};

/// Read what the host's serial port holds for the guest's console, when a
/// look is due. The host's service calls this, one core at a time.
pub(crate) fn feed_console_input() {
    let input = &CONSOLE_INPUT;
    let now = host::fresh_counter();
    if now < input.next.load(Ordering::Relaxed) {
        return;
    }
    let mut put = input.put.load(Ordering::Relaxed);
    while put.wrapping_sub(input.taken.load(Ordering::Acquire)) < CONSOLE_INPUT_BYTES {
        let Some(byte) = crate::serial::read() else {
            let last = input.last.load(Ordering::Relaxed);
            let typing =
                last != 0 && now.wrapping_sub(last) < host::COUNTER_HZ * INPUT_TYPING_SECONDS;
            let micros = if typing {
                INPUT_TYPING_MICROS
            } else {
                INPUT_IDLE_MICROS
            };
            input.next.store(
                now + host::COUNTER_HZ / 1_000_000 * micros,
                Ordering::Relaxed,
            );
            return;
        };
        input.bytes[put % CONSOLE_INPUT_BYTES].store(byte, Ordering::Relaxed);
        put = put.wrapping_add(1);
        input.put.store(put, Ordering::Release);
        input.last.store(now, Ordering::Relaxed);
    }
}

/// The guest's console UART, shown line by line as `GUEST| ...`. In the
/// automatic Metal plugin repair's boot it also types the repair's commands
/// (`bundle_repair`), and the host's own input waits until it is done.
#[derive(Default)]
struct GuestConsole {
    lines: console_lines::Lines,
    repair: ConsoleRepair,
    /// Host input was dropped while the repair typed; reported once.
    dropped: bool,
}

#[derive(Default)]
enum ConsoleRepair {
    /// Not looked at yet: the boot's kind is known once the guest runs.
    #[default]
    Unknown,
    Driving(stela_boot_core::bundle_repair::Driver),
    Off,
}

impl GuestConsole {
    fn millis() -> u64 {
        host::fresh_counter() / (host::COUNTER_HZ / 1000)
    }

    fn repair(&mut self) -> Option<&mut stela_boot_core::bundle_repair::Driver> {
        if matches!(self.repair, ConsoleRepair::Unknown) {
            self.repair = match bundle_repair::driver(Self::millis()) {
                Some(driver) => ConsoleRepair::Driving(driver),
                None => ConsoleRepair::Off,
            };
        }
        match &mut self.repair {
            ConsoleRepair::Driving(driver) => Some(driver),
            _ => None,
        }
    }

    /// Log what the driver did. A repair whose shell stopped answering or
    /// left, or whose guest did not restart, restarts the PC, once: the
    /// next normal boot tries the kext, and NVRAM's record makes a still
    /// missing plugin stop there with the manual remedy instead of
    /// repairing again. A shell that never took input asks for one more
    /// repair boot instead (`bundle_repair::unanswered`). A failed step the
    /// shell answered restarts the guest itself.
    fn report_repair(&mut self) {
        let Some(driver) = self.repair() else {
            return;
        };
        let events = driver.take_events();
        let restart = driver.failed();
        let unanswered = driver
            .failure()
            .is_some_and(stela_boot_core::bundle_repair::Failure::unanswered);
        for event in &events {
            output::arguments(format_args!(
                "{}\n",
                stela_boot_core::bundle_repair::describe(event)
            ));
        }
        if restart {
            self.repair = ConsoleRepair::Off;
            if unanswered && bundle_repair::unanswered() {
                output::arguments(format_args!(
                    "BUNDLE REPAIR nothing ran; restarting the PC into one more repair boot\n"
                ));
            } else {
                output::arguments(format_args!(
                    "BUNDLE REPAIR restarting the PC; the next boot tries the kext once\n"
                ));
            }
            service::request_restart();
        }
    }
}

impl ConsoleSink for GuestConsole {
    fn poll(&mut self) -> Option<u8> {
        if let Some(driver) = self.repair() {
            driver.tick(Self::millis());
            let byte = driver.next_byte();
            self.report_repair();
            if byte.is_some() {
                return byte;
            }
        }
        let input = &CONSOLE_INPUT;
        let taken = input.taken.load(Ordering::Relaxed);
        let put = input.put.load(Ordering::Acquire);
        if taken == put {
            return None;
        }
        if matches!(self.repair, ConsoleRepair::Driving(_)) {
            // Bytes between the repair's would break into its commands, and
            // a break or ^C would stop the shell.
            input.taken.store(put, Ordering::Release);
            if !core::mem::replace(&mut self.dropped, true) {
                output::arguments(format_args!(
                    "BUNDLE REPAIR host console input dropped while the repair types\n"
                ));
            }
            return None;
        }
        let byte = input.bytes[taken % CONSOLE_INPUT_BYTES].load(Ordering::Relaxed);
        input.taken.store(taken.wrapping_add(1), Ordering::Release);
        Some(byte)
    }

    fn transmit(&mut self, byte: u8) -> Result<(), ConsoleBusy> {
        // Each byte, for the shell's prompt, which ends no line.
        if let Some(driver) = self.repair() {
            driver.output(byte, Self::millis());
            self.report_repair();
        }
        if let Some(fragment) = self.lines.push(byte) {
            if let Ok(mut panic) = GUEST_PANIC.0.try_borrow_mut() {
                panic.observe(&fragment);
            }
            let line = fragment.text;
            output::arguments(format_args!("GUEST| {line}\n"));
            if !fragment.complete {
                output::arguments(format_args!(
                    "GUEST-FRAGMENT bytes={} continues=1\n",
                    fragment.bytes
                ));
            }
            if let Some(driver) = self.repair() {
                let now = Self::millis();
                driver.tick(now);
                driver.line(&line, now);
                self.report_repair();
            }
            if !line.trim().is_empty()
                && let Ok(mut lines) = GUEST_LINES.0.try_borrow_mut()
            {
                if lines.len() == crate::status::STREAM_LINES {
                    lines.remove(0);
                }
                lines.push(line.clone());
            }
        }
        Ok(())
    }
}

/// J473AP placement from Apple's DeviceTree (26A434 `DeviceTree.j473ap`):
/// `/arm-io` maps child 0 to PA 0x2_0000_0000; `uart0` is `reg` 0x35200000
/// +0x4000 with AIC input 739 and `wdt` is `reg` 0x3d2b0000 +0x4000 with AIC
/// input 379. Used when no DeviceTree is loaded (the test kernel).
const ARM_IO: u64 = 0x2_0000_0000;
const J473_UART: (u64, u64, u32) = (0x3520_0000, 0x4000, 739);
const J473_WDT: (u64, u64, u32) = (0x3d2b_0000, 0x4000, 379);
/// `aic` `reg` 0x3b0c0000, and NR_IRQ derived from that DeviceTree as below.
const J473_AIC: (u64, u16) = (0x3b0c_0000, 1152);
/// The J473AP AIC2 event register's offset from the core block: Linux
/// `arch/arm64/boot/dts/apple/t8112.dtsi` gives the `aic` second `reg` at
/// 0x3b0c8000, as the AIC2 model documents; the ADT has no
/// `aic-iack-offset`. A loaded DeviceTree's board takes its SoC's offset
/// (`machine::soc`).
const J473_AIC_EVENT: u64 = 0x8000;
/// The J473AP ADT `aic` `#shared-timestamps`.
const J473_AIC_TIMESTAMPS: u8 = 16;

/// The board's devices ([`mount::devices`]), or without a DeviceTree (the
/// test kernel) the J473AP console UART, watchdog and AIC2.
fn devices(
    board: Option<&board::Board>,
    inputs: mount::Inputs<'_>,
    memory: Option<MemoryResources>,
) -> Result<(AppleDevices, Option<stela_soc::machine::UsbId>)> {
    if let Some(board) = board {
        return mount::devices_with_memory(
            board,
            GuestConsole::default(),
            inputs,
            memory,
            &mut |arguments| output::arguments(arguments),
        );
    }
    if memory.is_some() {
        return Err(refuse!(
            "native board memory requires a live assigned DeviceTree"
        ));
    }
    if inputs.backing.is_some() {
        return Err(refuse!(
            "--storage was supplied but this DeviceTree exposes no modeled ANS2"
        ));
    }
    if inputs.restore {
        return Err(refuse!("restore study requires a DT-placed USB device"));
    }
    let place = |(offset, bytes, irq): (u64, u64, u32)| {
        (
            Range {
                physical: ARM_IO + offset,
                bytes,
            },
            AicInput::new(0, irq as u16),
        )
    };
    let (uart, wdt) = (place(J473_UART), place(J473_WDT));
    let mut devices = AppleDevices::new();
    devices
        .add_uart(GuestConsole::default(), &[uart.0], Some(uart.1))
        .map_err(|error| refuse!("UART placement: {error}", ?error))?;
    devices
        .add_watchdog_with_side(wdt.0, None, Some(wdt.1), inputs.now)
        .map_err(|error| refuse!("watchdog placement: {error}", ?error))?;
    devices.add_aic(
        ARM_IO + J473_AIC.0,
        J473_AIC.1,
        AicLayout::aic2(J473_AIC_EVENT, J473_AIC_TIMESTAMPS),
    )?;
    Ok((devices, None))
}

static EVENTS: AtomicU64 = AtomicU64::new(0);
static INTERRUPTS: AtomicU64 = AtomicU64::new(0);

/// Count one more event on `counter` and give its count before, while it is
/// below `limit`; `limit` once it is reached. Every PE reports each of its
/// exceptions and returns here: past the allowance the shared line stays
/// read-only, so they do not contend for it.
fn allowance(counter: &AtomicU64, limit: u64) -> u64 {
    if counter.load(Ordering::Relaxed) >= limit {
        return limit;
    }
    counter.fetch_add(1, Ordering::Relaxed)
}

pub(crate) fn report(event: &Event) {
    // Explicit monitor traces bypass the ordinary progress-event limit
    // without consuming its reporting allowance.
    if let Event::Monitor(trace) = event {
        output::arguments(format_args!(
            "MONITOR pe={} kind={:?} from={}{}:{:#x} il={} to={}{}:{:#x} il={} \
             x0-x7={:x?} x16={:#x} x17={:#x} lr={:#x}\n",
            trace.pe,
            trace.kind,
            level_name(trace.from.guarded),
            trace.from.el,
            trace.from.pc,
            trace.from.illegal,
            level_name(trace.to.guarded),
            trace.to.el,
            trace.to.pc,
            trace.to.illegal,
            trace.arguments,
            trace.x16,
            trace.x17,
            trace.lr,
        ));
        return;
    }
    let count = allowance(&EVENTS, REPORTED_EVENTS);
    let interrupt = matches!(
        event,
        Event::Exception { vector, .. } if *vector != stela_engine::machine::exception::Vector::Synchronous
    );
    let guarded_fault = matches!(
        event,
        Event::Exception { vector, from_guarded: true, syndrome, .. }
            if *vector == stela_engine::machine::exception::Vector::Synchronous
                && syndrome >> ESR_EC_SHIFT != EC_SVC64
    );
    // An abort on an address no translation regime can hold, such as a
    // pointer whose PAC was never stripped, is always worth its line.
    let noncanonical_fault = matches!(
        event,
        Event::Exception { vector, syndrome, fault_address: Some(address), .. }
            if *vector == stela_engine::machine::exception::Vector::Synchronous
                && EC_LOWER_ABORTS.contains(&(syndrome >> ESR_EC_SHIFT))
                && !matches!((address >> 48) as u8, 0 | 0xff)
    );
    let quiet = if guarded_fault || noncanonical_fault {
        false
    } else if interrupt {
        allowance(&INTERRUPTS, REPORTED_INTERRUPTS) >= REPORTED_INTERRUPTS
    } else {
        count >= REPORTED_EVENTS
    };
    if quiet {
        return;
    }
    match event {
        Event::Monitor(_) => unreachable!("monitor traces are emitted before event throttling"),
        Event::Exception {
            vector,
            from_el,
            from_guarded,
            syndrome,
            return_address,
            fault_address,
        } => output::arguments(format_args!(
            "EXCEPTION {vector:?} from={}{from_el} esr={syndrome:#x} elr={return_address:#x} far={}\n",
            level_name(*from_guarded),
            match fault_address {
                Some(address) => alloc::format!("{address:#x}"),
                None => "-".into(),
            }
        )),
        Event::Return { to_el, guarded, pc } => output::arguments(format_args!(
            "ERET to={}{to_el} pc={pc:#x}\n",
            level_name(*guarded)
        )),
        Event::Guarded {
            enter,
            el,
            guarded,
            pc,
        } => output::arguments(format_args!(
            "{} now={}{el} pc={pc:#x}\n",
            if *enter { "GENTER" } else { "GEXIT" },
            level_name(*guarded)
        )),
    }
}

/// How long a restart or power-off that follows an error waits, so its
/// reason can be read on the screen first. An orderly one goes on at once.
const STOP_HOLD_SECONDS: u64 = 10;

/// Whether `reset` is a restart or power-off asked for in order, through
/// the SMC, as macOS restarts or turns the Mac off and as a held power
/// button turns it off, with no kernel panic before it. Anything else, a
/// watchdog's reset among them, left an error to read.
fn orderly(reset: &Reset) -> bool {
    !matches!(reset.transition, Transition::Halt)
        && reset.reason.starts_with("smc: ")
        && guest_panic().is_none()
}

/// What the stop took before the restart or power-off, by phase, in
/// milliseconds (`STOP TIMES`).
#[derive(Default)]
struct StopTimes {
    /// The NOR store and the GPUs' stop (`service::Service::stop`).
    service: u64,
    /// The stop's reports.
    report: u64,
}

/// Show why the machine stopped on Stela's own screen, which it takes back
/// from the guest's last frame: the running step fails with `text`.
fn show_stop(text: &str) {
    crate::console::reclaim();
    crate::status::fail(text);
}

/// Begin `action`, the restart or power-off that ends the run. After an
/// error, show why the machine went down, `otherwise` without a panic, and
/// where its full log is, and hold `hold` seconds so they can be read.
fn show_reset(otherwise: &str, action: &str, hold: u64) {
    if hold == 0 {
        crate::console::reclaim();
        crate::status::begin(action);
        return;
    }
    show_stop(&guest_down(String::from(otherwise)));
    output::flush_log();
    if output::usb_log_kept() {
        crate::status::info(
            "Its full log is on this USB: on another computer, plug in this USB and run `stela \
             log`, which saves it to log.txt.",
        );
    }
    crate::status::countdown(action, hold);
}

/// [`show_stop`], then halt until the PC is turned off.
fn stop_here(text: &str, code: u32) -> ! {
    show_stop(text);
    output::flush_log();
    crate::status::info(output::stopped_text());
    output::terminate(code)
}

/// The guest's console lines since the run loop showed them, the last
/// `status::STREAM_LINES` at most, reached under the board's lock.
static GUEST_LINES: output::Global<Vec<String>> = output::Global(RefCell::new(Vec::new()));

/// The restore step `line` begins, by name, as the running step's detail.
fn installer_step(line: &str) -> Option<String> {
    // `[08:34:20.0721-GMT]{5>7} CHECKPOINT BEGIN: RESTORED:[0x067C] fdr_create`
    let (_, step) = line.split_once("CHECKPOINT BEGIN: ")?;
    let name = step.rsplit_once(']').map_or(step, |(_, name)| name);
    Some(format!("installer: {}", name.trim().replace('_', " ")))
}

/// The first kernel panic line the guest printed, which names a stop
/// better than how the machine went down after it.
static GUEST_PANIC: output::Global<console_lines::Panic> =
    output::Global(RefCell::new(console_lines::Panic::new()));

/// The first explicit panic header, falling back to nested metadata if the
/// buffered original header has not arrived. Repeated headers never replace it.
fn guest_panic() -> Option<String> {
    GUEST_PANIC
        .0
        .try_borrow()
        .ok()
        .and_then(|panic| panic.message().map(String::from))
}

/// The panic region's guest-physical base and size, from the booted
/// DeviceTree's `/pram reg` (the loader publishes it; zero when absent).
static PANIC_REGION: [AtomicU64; 2] = [AtomicU64::new(0), AtomicU64::new(0)];

/// Keep the panic region the booted DeviceTree names, for
/// [`panic_region_report`].
pub(crate) fn remember_panic_region(tree: &DeviceTree) {
    let Some(reg) = tree.property(stela_loader::carveouts::PRAM, "reg") else {
        return;
    };
    let Some((base, bytes)) = reg.value.get(..16).map(|pair| {
        let (base, bytes) = pair.split_at(8);
        (
            u64::from_le_bytes(base.try_into().expect("8 bytes")),
            u64::from_le_bytes(bytes.try_into().expect("8 bytes")),
        )
    }) else {
        return;
    };
    PANIC_REGION[0].store(base, Ordering::Relaxed);
    PANIC_REGION[1].store(bytes, Ordering::Relaxed);
}

/// When the guest went down without a panic on its console, what its panic
/// region holds: XNU logs a panic there before it stops the other CPUs'
/// work, and a CPU that never answers (parked in SPTM, say) keeps the
/// console from ever printing it, until a watchdog resets the machine.
fn panic_region_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    use stela_boot_core::panic_region::{Contents, HEADER_BYTES, TEXT_LIMIT, read};
    let base = PANIC_REGION[0].load(Ordering::Relaxed);
    let bytes = PANIC_REGION[1].load(Ordering::Relaxed);
    if base == 0 || bytes < HEADER_BYTES as u64 {
        output::arguments(format_args!("GUEST PANIC region=none\n"));
        return;
    }
    // The header, the log offset XNU's header gives (0x331 bytes in) and the
    // text limit, read a page at a time so a short region stays in bounds.
    let wanted = bytes.min(0x1000 + TEXT_LIMIT as u64) as usize;
    let mut region = alloc::vec![0u8; wanted];
    for (index, chunk) in region.chunks_mut(0x1000).enumerate() {
        if !machine.read_physical(base + index as u64 * 0x1000, chunk) {
            output::arguments(format_args!(
                "GUEST PANIC region={base:#x} unreadable at {:#x}\n",
                base + index as u64 * 0x1000
            ));
            return;
        }
    }
    match read(&region) {
        Contents::NoHeader => output::arguments(format_args!(
            "GUEST PANIC region={base:#x} no panic header: the kernel did not begin a panic log\n"
        )),
        Contents::BadOffset(offset) => output::arguments(format_args!(
            "GUEST PANIC region={base:#x} header log offset {offset:#x} lies outside it\n"
        )),
        Contents::Empty => output::arguments(format_args!(
            "GUEST PANIC region={base:#x} header present, no log text yet\n"
        )),
        Contents::Text { text, truncated } => {
            output::arguments(format_args!(
                "GUEST PANIC region={base:#x} truncated={truncated} text={text:?}\n"
            ));
        }
    }
}

/// How the guest went down: the first panic, else `otherwise`.
fn guest_down(otherwise: String) -> String {
    match guest_panic() {
        Some(message) => format!("macOS crashed: {message}"),
        None => otherwise,
    }
}

/// Host interfaces used by both ordinary authored inputs and the
/// canonical original-image execution path. Reporting cannot choose a fallback.
struct RunIo {
    restore: Option<restore::Experiment>,
    host_input: Option<input::HostInput>,
    host_usb: Option<usb::HostUsb>,
    host_bluetooth: Option<bluetooth::HostBluetooth>,
    guest_network: Option<network::GuestNetwork>,
    host_nor: Option<nor::HostNor>,
    host_disks: storage::HostDisks,
    host_acpi: Option<&'static aml::HostAcpi>,
    boot_progress: progress::BootProgress,
}

fn run_machine<H: Host>(
    mut machine: Machine<H, parallel::Board>,
    options: &Options,
    handoff: &Handoff,
    io: RunIo,
    mut execute: impl FnMut(&mut Machine<H, parallel::Board>, u64) -> Stop,
) -> ! {
    if options.survey {
        output::arguments(format_args!(
            "SURVEY RUN: device accesses without a model read 0 and drop writes; \
             the guest's behavior is unverified\n"
        ));
        machine.begin_survey();
    }
    if let Err(reason) = machine.set_idle_skip(options.idle_skip) {
        output::arguments(format_args!(
            "IDLE SKIP unavailable: {reason}; waits use host time\n"
        ));
    }
    machine.set_step_timing(options.profile);
    if options.profile {
        host::pmu::start(machine.processor());
    }
    if options.profile {
        output::arguments(format_args!(
            "PROFILE on: each heartbeat reports the engine's time by phase and exit kind\n"
        ));
        #[cfg(feature = "nvk")]
        gpu::gate::profile_passes();
    }
    if options.install.is_some() {
        output::arguments(format_args!(
            "INSTALL SIGNING: the restore host answers its root tickets, Cryptex1 ticket and \
             recoveryOS LocalPolicy from the USB's staged answers when they match, else sends \
             this machine's ECID, the restore's nonces and its component hashes to gs.apple.com\n"
        ));
    }
    // Waits return to this loop often enough that a press of the host's
    // power button reaches a sleeping SoC, and other host inputs a waiting
    // guest, promptly.
    machine.set_wait_limit(Some(host::COUNTER_HZ / WAIT_POLLS_PER_SECOND));
    if !options.breakpoints.is_empty() {
        machine.set_breakpoints(&options.breakpoints);
    }
    machine
        .devices
        .with(|devices| devices.set_display_swap_tracing(options.display_trace));
    if options.display_trace {
        output::arguments(format_args!(
            "DISPLAY TRACE enabled: per-swap DCP reports\n"
        ));
    }
    machine.set_monitor_tracing(options.monitor_trace);
    if options.monitor_trace {
        output::arguments(format_args!(
            "MONITOR TRACE enabled: raw guarded boundaries; endpoint ABI uninterpreted\n"
        ));
    }
    if !options.traces.is_empty() {
        machine.set_traces(&options.traces);
    }
    if !options.watches.is_empty() {
        machine.set_watches(&options.watches);
    }
    let RunIo {
        restore,
        host_input,
        host_usb,
        host_bluetooth,
        guest_network,
        host_nor,
        host_disks,
        host_acpi,
        mut boot_progress,
    } = io;
    crate::status::begin(match restore {
        Some(_) => "Starting the macOS installer",
        None => "Starting macOS",
    });
    service::install(service::Service::new(
        machine.devices.worker(),
        restore,
        service::Peripherals {
            input: host_input,
            usb: host_usb,
            bluetooth: host_bluetooth,
        },
        guest_network,
        host_nor,
        power::HostPowerControl::new(
            handoff.power.clone(),
            host_acpi,
            power::EfiReset::new(handoff.efi_reset, handoff.root),
            handoff.tsc_hz,
            host_disks,
        ),
        handoff.tsc_hz,
    ));
    let started = host::rdtsc();
    let stop = run(
        &mut machine,
        options.budget,
        handoff.tsc_hz,
        &mut boot_progress,
        |machine, limit| {
            if let Some(acpi) = host_acpi {
                acpi.poll();
            }
            execute(machine, limit)
        },
    );
    let seconds = (host::rdtsc() - started) as f64 / handoff.tsc_hz as f64;
    // The home Engine has returned with its materialized state. Report its
    // completed phase maxima before taking host services or persisting disks.
    if options.profile
        && let Some(timing) = machine.step_timing()
    {
        profile_lifetime_maxima(timing, handoff.tsc_hz, "guest-stop");
    }
    let mut service = service::take().expect("the run's service");
    if options.profile {
        service.power().disks().report_callbacks(handoff.tsc_hz);
    }
    let started = output::now();
    service.stop();
    let times = StopTimes {
        service: output::millis_since(started),
        ..StopTimes::default()
    };
    finish(&mut machine, stop, seconds, options, times, service.power())
}

fn finish<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    stop: Stop,
    seconds: f64,
    options: &Options,
    mut times: StopTimes,
    host_power: &power::HostPowerControl,
) -> ! {
    let started = output::now();
    // An orderly stop has nothing to debug: its registers and frames stay
    // out of the log unless `--profile` asks for them.
    let full = options.profile || !matches!(&stop, Stop::Reset(reset) if orderly(reset));
    parallel::report_source_observations(machine);
    survey_report(machine);
    let drained = machine.devices.with(Drained::take);
    unmodeled_report(machine, drained.unmodeled);
    // Machine answers are counted per access; coprocessor requests are
    // counted among the distinct reports only.
    output::arguments(format_args!(
        "UNSUPPORTED SUMMARY answered={} distinct={}\n",
        machine.statistics.degraded,
        UNMODELED_REPORTED.load(Ordering::Relaxed)
    ));
    if full {
        stack_report(machine);
    }
    dump_report(machine, &options.dumps);
    let host = host::statistics();
    output::arguments(format_args!(
        "STATISTICS {:?} shadow_fills={} shadow_prefills={} shadow_resets={} shadow_exhausted={} shadow_peak={}M/{}M/{}M/{}M lower_resets={} scoped_invalidations={} stray_interrupts={} helper_calls={} helper_fills={} bounces={} guard_faults={} events={} seconds={seconds:.3}\n",
        machine.statistics,
        host.fills,
        host.prefills,
        host.resets,
        host.exhausted,
        host.shadow_peak[0] >> 20,
        host.shadow_peak[1] >> 20,
        host.shadow_bytes[0] >> 20,
        host.shadow_bytes[1] >> 20,
        host.lower_resets,
        host.scoped,
        host.stray,
        host.helper_calls,
        host.helper_fills,
        host.bounces,
        host.guard_faults,
        EVENTS.load(Ordering::Relaxed),
    ));
    watchdog_report(machine, drained.watchdog);
    for line in drained.sep {
        output::arguments(format_args!("SEP OBSERVE {line}\n"));
    }
    if full {
        machine_report(machine);
    }
    times.report = output::millis_since(started);
    match stop {
        Stop::Idle => {
            output::arguments(format_args!(
                "STOP idle: every PE is off or waits with no wake source\n"
            ));
            stop_here(
                &guest_down(String::from(
                    "macOS stopped: every processor is off or waits with nothing to wake it",
                )),
                0x10,
            )
        }
        Stop::Waiting => {
            output::arguments(format_args!(
                "STOP waiting: every PE waits for an input from outside the board\n"
            ));
            stop_here(
                &guest_down(String::from(
                    "macOS stopped: it waits for something from outside the Mac",
                )),
                0x10,
            )
        }
        Stop::Reset(reset) => {
            if let Some(message) = guest_panic() {
                output::arguments(format_args!("STOP macOS crashed: {message}\n"));
                output::arguments(format_args!("RESET after panic: {reset}\n"));
            } else {
                // Before the report: a watchdog's reset leaves RAM as the
                // panicking PE left it.
                if !orderly(&reset) {
                    panic_region_report(machine);
                }
                output::arguments(format_args!("STOP reset: {reset}\n"));
            }
            // The run is over: its blocks' memory goes to proving the new
            // ones.
            machine.discard_every_block();
            let started = output::now();
            #[cfg(feature = "nvk")]
            pipelines::write();
            let pipelines = output::millis_since(started);
            let started = output::now();
            translations::write();
            let translations = output::millis_since(started);
            let hold = match reset.transition {
                Transition::Restart | Transition::PowerOff => {
                    stela_boot_core::shutdown::stop_hold_seconds(
                        orderly(&reset),
                        reset.reason == stela_soc::machine::POWER_BUTTON_HELD,
                        STOP_HOLD_SECONDS,
                    )
                }
                _ => 0,
            };
            output::arguments(format_args!(
                "STOP TIMES service_ms={} report_ms={} pipelines_ms={pipelines} \
                 translations_ms={translations} hold_ms={}\n",
                times.service,
                times.report,
                hold * 1000
            ));
            match reset.transition {
                Transition::Restart => {
                    show_reset("macOS restarted the Mac", "Restarting the PC", hold);
                    host_power.restart()
                }
                Transition::PowerOff => {
                    show_reset("macOS turned the Mac off", "Turning the PC off", hold);
                    host_power.power_off()
                }
                Transition::Halt => stop_here(
                    &guest_down(format!("macOS halted the Mac: {}", reset.reason)),
                    power::RESTART_CODE,
                ),
            }
        }
        Stop::Budget => {
            output::arguments(format_args!("STOP budget\n"));
            stop_here("the run reached its --budget", 0x14)
        }
        Stop::NativeFailure(failure) => {
            output::arguments(format_args!("STOP NATIVE_FAILURE {failure:?}\n"));
            stop_here(
                "Stela's own code failed while running macOS. This is a Stela bug",
                0x13,
            )
        }
        Stop::Unsupported(requirement) => {
            output::arguments(format_args!("STOP UNSUPPORTED {requirement}\n"));
            stop_here(
                &format!("macOS needs {requirement}, which Stela does not support yet"),
                0x13,
            )
        }
        Stop::DeviceRead(request) => {
            output::arguments(format_args!(
                "STOP canonical read requires its live continuation: {request:?}\n"
            ));
            stop_here(
                "a device read was left unanswered. This is a Stela bug",
                0x13,
            )
        }
        Stop::DeviceWrite(request) => {
            output::arguments(format_args!(
                "STOP canonical write requires its live continuation: {request:?}\n"
            ));
            stop_here(
                "a device write was left unanswered. This is a Stela bug",
                0x13,
            )
        }
        Stop::Breakpoint(pc) => {
            output::arguments(format_args!("STOP breakpoint pc={pc:#x}\n"));
            stop_here(&format!("Stela's breakpoint at {pc:#x}"), 0x16)
        }
    }
}

/// The stopped machine's state for debugging an unexpected stop: the exits
/// that left compiled code most, every PE's registers and frames, and the
/// implementation-defined registers the guest used.
fn machine_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    exit_report(machine);
    output::arguments(format_args!("FINAL {}\n", Registers(&machine.cpu)));
    let mut parked_frames = Vec::new();
    for (number, state) in machine.peer_states().into_iter().enumerate() {
        let PeerState::Parked { cpu, wfi } = state else {
            output::arguments(format_args!("PE {number} {state:?}\n"));
            continue;
        };
        let wait = match wfi {
            Some(true) => "wfi",
            Some(false) => "wfe",
            None => "none",
        };
        output::arguments(format_args!(
            "PE {number} parked wait={wait} {}\n",
            Registers(cpu)
        ));
        general_registers(cpu);
        parked_frames.push((number, cpu.state.x[29]));
    }
    for (number, frame) in parked_frames {
        frames(machine, &alloc::format!("PE {number} FRAME"), frame);
    }
    // Every other PE's last materialized frame chain, stale but enough to
    // name what it was doing when the run stopped.
    let sources = {
        let hub = machine.devices.hub();
        (0..hub.pes())
            .filter(|&pe| pe != machine.processor())
            .filter_map(|pe| Some((pe, hub.source_observation(pe)?.source)))
            .filter(|(_, source)| source.el > 0)
            .map(|(pe, source)| (pe, source.fp, source.sp))
            .collect::<Vec<_>>()
    };
    for &(pe, frame, _) in &sources {
        frames(machine, &alloc::format!("PE {pe} SOURCE FRAME"), frame);
    }
    for (pe, _, sp) in sources {
        stack_text_report(machine, pe, sp);
    }
    let cpu = &machine.cpu;
    for (entry, reads, writes) in cpu.apple.impdef.used() {
        output::arguments(format_args!(
            "IMPDEF {} {:?} reads={reads} writes={writes}: {}\n",
            entry.name, entry.class, entry.evidence
        ));
    }
    general_registers(cpu);
}

/// Whether this processor holds the host drivers' lock where a log store
/// from here would wait on itself: stopped inside a driver call, or inside
/// the device start other than between two drivers' calls
/// ([`storage::between_drivers_here`]).
pub(crate) fn inside_host_drivers() -> bool {
    storage::host_drivers_held_here() && !storage::between_drivers_here()
}

/// Whether this processor is starting the host's devices, holding their
/// lock by design rather than stopped inside a driver: the start stores
/// the USB's log itself between two devices.
pub(crate) fn starting_host_devices() -> bool {
    storage::starting_devices_here()
}

/// Store the USB's log from inside the GPU's start, which holds the host
/// drivers' lock outside every storage driver's call; whether this
/// processor is where that can be done (`storage::store_usb_log_in_gpu_start`).
pub(crate) fn store_usb_log_in_gpu_start() -> bool {
    storage::store_usb_log_in_gpu_start()
}

/// Run a stop's firmware call in the loader's root `root`
/// (`storage::at_stop_in_loader_root`).
///
/// # Safety
/// As for `storage::at_stop_in_loader_root`.
pub(crate) unsafe fn in_loader_root_at_stop<T>(root: u64, call: impl FnOnce() -> T) -> T {
    // SAFETY: the caller's.
    unsafe { storage::at_stop_in_loader_root(root, call) }
}

/// Unload the GPUs' firmware before a fatal stop, where one can
/// (`gpu::gate::stop_for_fatal`).
pub(crate) fn stop_gpus_for_fatal() {
    #[cfg(feature = "nvk")]
    gpu::gate::stop_for_fatal();
}
