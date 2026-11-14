//! UEFI application that runs an ARM image on the shared compiler and runtime.
//!
//! The firmware starts `stela.efi`, on a USB as its removable-media boot
//! loader `\EFI\BOOT\BOOTX64.EFI`. It connects every controller to its
//! drivers and reads `\EFI\stela\stela.cfg` from its own volume. A
//! `stela.cfg` that starts with `--system` runs the guest on the system-mode
//! engine; one that starts
//! with `--host-check` reports on the PC (`host_check`). Any other line runs
//! no ARM code: install mode (`payload`), or the host storage check
//! (`storage`), which leaves boot services and installs Stela's own page
//! tables first. With `\EFI\stela\choices.cfg` beside it, the startup
//! choices (`choice`) pick the option line first.
#![no_std]
#![no_main]
extern crate alloc;
pub use stela_contracts::Refusal;
pub type Result<T, E = Refusal> = core::result::Result<T, E>;
use core::{arch::asm, panic::PanicInfo};
use stela_boot_core::apic;
use uefi::{
    Status,
    boot::{AllocateType, MemoryType},
    mem::memory_map::MemoryMap,
};
#[cfg(feature = "diagnostics")]
mod audio_check;
mod choice;
mod console;
mod cpu;
mod engine;
mod entropy;
mod firmware;
mod handoff;
use stela_boot_core::heap;
#[cfg(feature = "diagnostics")]
mod host_check;
mod host_firmware;
mod host_inventory;
mod installer;
mod la57;
mod layout;
mod log_file;
pub(crate) mod native_clock;
pub mod native_dma;
mod native_idle;
mod native_image;
pub mod native_irq;
pub mod native_memory;
pub(crate) mod native_mmio;
mod native_smp;
mod output;
mod paging;
mod payload;
mod pci_io;
mod random;
mod selection;
mod serial;
mod status;
mod stop;
mod stop_record;
mod storage;
mod system;
mod tables;
mod ui;
mod usb_legacy;
#[global_allocator]
pub(crate) static ALLOCATOR: heap::Heap = heap::Heap::new().reporting_failures(|report| {
    use core::fmt::Write;
    // Serial is bounded, lock-free port I/O: no allocation or queue.
    let _ = serial::Serial.write_fmt(report);
});
/// The live firmware keeps its boot-service pages. Budget the first heap from
/// the conventional pages still available after the pristine image copy.
/// Before the heap exists, so its refusals are an [`EarlyText`].
fn initial_heap_bytes() -> Result<usize, EarlyText> {
    let map = uefi::boot::memory_map(MemoryType::LOADER_DATA)
        .map_err(|_| EarlyText::from("firmware memory map unavailable for the boot heap"))?;
    let free = map
        .entries()
        .filter(|entry| entry.ty == MemoryType::CONVENTIONAL)
        .try_fold(0u64, |total, entry| {
            entry
                .page_count
                .checked_mul(paging::PAGE as u64)
                .and_then(|bytes| total.checked_add(bytes))
        })
        .ok_or_else(|| EarlyText::from("firmware memory map overflows the boot heap budget"))?;
    stela_boot_core::memory_budget::bootstrap_heap(free)
        .map(|bytes| bytes as usize)
        .map_err(|error| {
            let mut text = EarlyText::default();
            let _ = core::fmt::Write::write_fmt(&mut text, format_args!("{error}"));
            text
        })
}

/// The boot log's identity line (`stela_boot_core::host_identity`): this
/// build, the CPU, the RAM the firmware describes and the firmware.
fn host_identity() -> alloc::string::String {
    use core::arch::x86_64::__cpuid;
    use stela_boot_core::host_identity::{Identity, brand, line};
    let signature = __cpuid(1).eax;
    let mut words = [0u32; 12];
    if __cpuid(0x8000_0000).eax >= 0x8000_0004 {
        for (index, leaf) in (0x8000_0002..=0x8000_0004u32).enumerate() {
            let registers = __cpuid(leaf);
            words[index * 4..index * 4 + 4].copy_from_slice(&[
                registers.eax,
                registers.ebx,
                registers.ecx,
                registers.edx,
            ]);
        }
    }
    let cpu = brand(words);
    let ram_bytes = uefi::boot::memory_map(MemoryType::LOADER_DATA)
        .map(|map| {
            map.entries()
                .filter(|entry| {
                    !matches!(
                        entry.ty,
                        MemoryType::RESERVED
                            | MemoryType::MMIO
                            | MemoryType::MMIO_PORT_SPACE
                            | MemoryType::UNUSABLE
                            | MemoryType::PAL_CODE
                    )
                })
                .map(|entry| entry.page_count * paging::PAGE as u64)
                .sum()
        })
        .unwrap_or(0);
    let vendor = alloc::string::String::from(uefi::system::firmware_vendor());
    let (family, model, stepping) = stela_boot_core::host_identity::signature_parts(signature);
    let leaf0 = __cpuid(0);
    host_inventory::text("StelaVersion", env!("CARGO_PKG_VERSION"));
    host_inventory::text("CPUBrand", cpu.clone());
    host_inventory::text(
        "CPUVendor",
        stela_boot_core::host_identity::vendor([leaf0.ebx, leaf0.edx, leaf0.ecx]),
    );
    host_inventory::number("CPUFamily", family.into());
    host_inventory::number("CPUModel", model.into());
    host_inventory::number("CPUStepping", stepping.into());
    host_inventory::number("HostMemoryBytes", ram_bytes);
    host_inventory::text("FirmwareVendor", vendor.clone());
    host_inventory::number("FirmwareRevision", uefi::system::firmware_revision().into());
    line(&Identity {
        version: env!("CARGO_PKG_VERSION"),
        cpu: &cpu,
        signature,
        ram_bytes,
        firmware_vendor: &vendor,
        firmware_revision: uefi::system::firmware_revision(),
    })
}

/// The longest [`EarlyText`]: room for the low-RAM refusal, small enough
/// to return by value.
const EARLY_BYTES: usize = 112;

/// A message formatted before the heap exists, into a fixed buffer; a
/// longer one is cut short.
struct EarlyText {
    bytes: [u8; EARLY_BYTES],
    len: usize,
}

impl Default for EarlyText {
    fn default() -> Self {
        Self {
            bytes: [0; EARLY_BYTES],
            len: 0,
        }
    }
}

impl EarlyText {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("")
    }
}

impl From<&str> for EarlyText {
    fn from(text: &str) -> Self {
        let mut early = Self::default();
        let _ = core::fmt::Write::write_str(&mut early, text);
        early
    }
}

impl core::fmt::Write for EarlyText {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        // Whole characters only, so the kept bytes stay UTF-8.
        for character in text.chars() {
            let mut buffer = [0; 4];
            let encoded = character.encode_utf8(&mut buffer).as_bytes();
            let end = self.len + encoded.len();
            if end > self.bytes.len() {
                return Err(core::fmt::Error);
            }
            self.bytes[self.len..end].copy_from_slice(encoded);
            self.len = end;
        }
        Ok(())
    }
}

/// What this build reads, for `stela` to check before installing it
/// (`stela_loader::mount_interface`).
#[used]
static MOUNT_INTERFACE: [u8; stela_loader::mount_interface::MARKER_BYTES] =
    stela_loader::mount_interface::marker(stela_loader::mount_interface::CURRENT);

/// Optional native host resource capability read by setup before publication.
#[used]
static HOST_FIRMWARE_INTERFACE: [u8; 21] = stela_loader::host_firmware::MOUNT_CAPABILITY;

/// Turn off the watchdog firmware arms before it starts a boot program,
/// which resets the machine unless boot services end within 5 minutes: the
/// menu, the disk chooser, a slow stick and install verification all take
/// longer. Nothing arms it again. Firmware without the service is common,
/// so a refusal is reported once and the boot continues.
fn stop_firmware_watchdog() {
    match uefi::boot::set_watchdog_timer(0, 0x10000, None) {
        Ok(()) => output::arguments(format_args!("FIRMWARE watchdog off\n")),
        Err(error) => output::arguments(format_args!(
            "FIRMWARE watchdog not turned off: {:?}; if the firmware armed it, the machine resets 5 minutes after Stela's start\n",
            error.status()
        )),
    }
}

#[uefi::entry]
fn main() -> Status {
    // Before any Rust state exists: the system engine runs from this copy.
    let snapshot = system::take_snapshot();
    // Referenced, so the linker keeps the marker setup reads.
    core::hint::black_box(&MOUNT_INTERFACE);
    core::hint::black_box(&HOST_FIRMWARE_INTERFACE);
    serial::initialize();
    output::arguments(format_args!(
        "STELA UEFI MOUNT version={}\n",
        env!("CARGO_PKG_VERSION")
    ));
    if !serial::present() {
        output::arguments(format_args!(
            "SERIAL no UART at COM1: nothing is sent to it or read from it\n"
        ));
    }
    stop_firmware_watchdog();
    let heap_bytes =
        initial_heap_bytes().unwrap_or_else(|error| output::early_fatal(error.as_str()));
    let heap = uefi::boot::allocate_pages(
        AllocateType::AnyPages,
        MemoryType::LOADER_DATA,
        heap_bytes / paging::PAGE,
    )
    .unwrap_or_else(|_| output::early_fatal("firmware has no memory for the heap"));
    // SAFETY: fresh LOADER_DATA pages that Stela owns for the whole boot,
    // initialized before any allocation.
    unsafe { ALLOCATOR.initialize(heap.as_ptr(), heap_bytes) };
    // SAFETY: the registered heap is the reserved WB identity extent.
    unsafe {
        native_memory::initialize(heap.as_ptr() as usize, heap.as_ptr() as u64, heap_bytes, 0)
    };
    status::set_clock(firmware::tsc_hz());
    firmware::start_ticker();
    // A stop the USB's log cannot keep leaves its record in the firmware.
    stop_record::arm(stop_record::Services::capture(), None);
    let log = log_file::open();
    // The record of such a stop at an earlier start reaches the log now.
    if log.is_some() {
        stop_record::carry();
    }
    output::arguments(format_args!("{}", host_identity()));
    if output::hypervisor_exit_port() {
        output::arguments(format_args!(
            "HOST hypervisor present: stop codes also go to its exit port\n"
        ));
    }
    status::begin("Connecting the firmware's drivers to the boot's devices");
    let connected = firmware::connect_needed();
    output::arguments(format_args!(
        "FIRMWARE connected {} of {} handles, left {} network controllers to Stela's drivers, {} slow\n",
        connected.connected, connected.handles, connected.network, connected.slow
    ));
    status::done("");
    status::begin("Reading Stela's settings");
    let (mut volume, mut firmware_root) = firmware::volume().unwrap_or_else(|e| output::fatal(e));
    let host_firmware =
        host_firmware::read(&mut firmware_root).unwrap_or_else(|e| output::fatal(e));
    drop(firmware_root);
    let options = firmware::options(&mut volume).unwrap_or_else(|e| output::fatal(e));
    let options = choice::resolve(&mut volume, options);
    // What this start changes on a PC disk is pending until Stela's own
    // drivers run: before the firmware exits, a PC disk is only read.
    let mut pending = stela_runtime::pc::disk_writes::Pending::default();
    status::done("");
    let held = system::held_at_start(&options, &pending);
    let choice::Chosen { options, plan } = choice::choose(&mut volume, options, held, &pending);
    pending.replace_plan(plan);
    status::take_screen(console::Framebuffer::from_firmware());
    if options.starts_with(b"--system") {
        system::boot(options, pending, volume, log, host_firmware, snapshot);
    }
    // Only the system engine does the pending work on the boot's disks.
    if pending.plan.is_some() {
        output::fatal(
            "the chosen startup option is not a system-mode boot, so the disk chosen for macOS \
             cannot be prepared; nothing was written to it",
        );
    }
    drop(snapshot);
    if options.starts_with(stela_boot_core::options::HOST_CHECK.as_bytes()) {
        drop(volume);
        #[cfg(feature = "diagnostics")]
        host_check::boot(options);
        #[cfg(not(feature = "diagnostics"))]
        output::fatal("host checks require a diagnostics build");
    }
    drop(volume);
    let (mut handoff, tables) =
        firmware::take_over(options, host_firmware).unwrap_or_else(|e| output::fatal(e));
    if let Some(screen) = handoff.screen {
        console::install(screen);
        output::arguments(format_args!("STELA UEFI MOUNT\n"));
    }
    // The storage check's tables do not map the firmware's runtime regions.
    stop_record::disarm();
    // SAFETY: boot services have ended; the new tables map the image, heap,
    // the stack and every RAM byte in use.
    unsafe {
        asm!("cli", options(nomem, nostack));
        handoff.active_tables = Some(paging::activate(tables).unwrap_or_else(|error| {
            output::fatal(format_args!("native table activation failed: {error}"));
        }));
        asm!(
            "mov rsp, {top}",
            "call {run}",
            "ud2",
            top = in(reg) handoff.stack_top,
            run = sym run,
            in("rdi") &raw mut handoff,
            options(noreturn),
        )
    }
}

/// Run the host storage check on Stela's own stack.
extern "sysv64" fn run(handoff: &mut firmware::Handoff) -> ! {
    let root = handoff
        .active_tables
        .as_ref()
        .unwrap_or_else(|| output::fatal("actual activated native table owner is missing"));
    root.check_current().unwrap_or_else(|error| {
        output::fatal(format_args!(
            "activated native root is unavailable: {error}"
        ));
    });
    if let Some(storage) = handoff.storage.take() {
        storage.run(
            #[cfg(feature = "diagnostics")]
            root,
        );
    }
    output::arguments(format_args!("STORAGE CHECK END\n"));
    output::terminate(0x10)
}
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    // No allocator, formatting of caller-supplied values, Rust lock, unwind,
    // poison recovery or continuation is allowed before the port has the
    // report; the screen and the boot log are tried after it.
    output::panic(info)
}
