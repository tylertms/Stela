//! The UEFI mount: build `stela.efi` and run it in x86 QEMU.
//!
//! The run puts the application on a FAT volume as the removable-media boot
//! file, beside `\EFI\stela\stela.cfg` (the mount options) and
//! `\EFI\stela\source.bin` (the system-mode guest image, when the run has
//! one), and boots it with QEMU's edk2 firmware, as a PC's firmware boots the
//! USB `stela setup` prepares. By default the guest is Stela's system-mode
//! test kernel.
//!
//! `--payload` runs install mode on a synthetic IPSW (no Apple bytes) split
//! onto the boot volume: `verify` checks every Erase member,
//! `verify-archive` does so from the whole IPSW instead, `stage` and
//! `stage-path`
//! also partitions a blank second disk and copies them into staging, which
//! the run then reads back on the host.
//!
//! `--preboot-disk IMAGE` attaches a disk image holding an installed
//! system's APFS Preboot volume; the mount builds the boot bundle from it at
//! boot and enters it, and the run checks the result against the bundle the
//! host builds from the same image.
use crate::{
    check::{BOOT_MANIFEST, BOOT_TARGET, BOOT_TARGET_DIR},
    tools::{binding, cargo, exec, qemu, root, stop, wait},
};
use anyhow::{Context, Result, bail, ensure};
use clap::{Args, Subcommand, ValueEnum};
use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command as Process, ExitCode, Stdio},
    time::{Duration, Instant},
};
use {
    stela_formats::payload::PayloadError, stela_formats::payload::PayloadSource,
    stela_formats::payload::ReadAt, stela_formats::payload::copy_verified,
    stela_formats::payload::gpt, stela_formats::payload::staging::StagedSource,
};

mod audio_check;
mod disks;
mod fat32;
mod host_display;
mod host_firmware;
mod lan;
mod machine;
mod network;
mod network_check;
mod preboot;
mod report;
mod storage;

/// `isa-debug-exit` reports `(code << 1) | 1`; a timeout reports 124.
const TIMEOUT_EXIT: u8 = 124;
const MAX_TIMEOUT_SECONDS: f64 = 120.0;
/// QEMU's FAT16 directory volume holds at most 516 MB; keep a margin.
const VVFAT_BYTES: u64 = 480 * 1024 * 1024;
/// A real restore payload verifies, and a kernel bundle boots, in tens of
/// minutes under TCG; a whole install takes hours.
const MAX_LONG_TIMEOUT_SECONDS: f64 = 24.0 * 3600.0;
const FIRMWARE_CODE: &str = "edk2-x86_64-code.fd";
const FIRMWARE_VARIABLES: &str = "edk2-i386-vars.fd";
/// The `ovmf` package's code and variables, which need no SMM.
const PACKAGED_FIRMWARE: [&str; 2] = [
    "/usr/share/OVMF/OVMF_CODE_4M.fd",
    "/usr/share/OVMF/OVMF_VARS_4M.fd",
];

/// The blank target disk of a staging run, sparse on the host.
const TARGET_BYTES: u64 = 4 * 1024 * 1024 * 1024;
/// Its PCI slot, which fixes its UEFI device path.
const TARGET_SLOT: u8 = 5;
/// The serial the NVMe target reports, which `stela.cfg` names.
const TARGET_SERIAL: &str = "STELA-TARGET-01";
/// Small slices, so the synthetic members span several.
const SYNTHETIC_SLICE_BYTES: u64 = 400_000;

/// Stela's system-mode test kernel (`boot/kernel.S`), loaded 16 KiB into the
/// J473AP DRAM base. Empty when xtask was built without clang.
const KERNEL: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/kernel.bin"));

fn kernel() -> Result<Vec<u8>> {
    ensure!(
        !KERNEL.is_empty(),
        "the system-mode test kernel was not assembled because clang was not found \
         when xtask was built; install clang (or set CLANG) and run again"
    );
    Ok(KERNEL.to_vec())
}
const KERNEL_OPTIONS: &str = "--system --load 0x800004000 --entry 0x800004000";
/// What the test kernel must leave in X0-X11 when it idles; X1 and X2, its
/// tasks' SVC counts, need only be nonzero.
const KERNEL_RESULTS: [(usize, Option<u64>); 12] = [
    (0, Some(0x5354_454c_4120_4f4b)),
    (1, None),
    (2, None),
    (3, Some(6)),
    // EL2 data abort: EC 0x25, IL, level 2 translation fault.
    (4, Some(0x9600_0006)),
    (5, Some(0xffff_ff80_0400_0000)),
    // EL2 SVC #0x10 and EL0 SVC #1.
    (6, Some(0x5600_0010)),
    (7, Some(0x5600_0001)),
    // EL2 load and store at the Stela window's address, outside both
    // ranges: level 0 translation faults, the store's with WnR.
    (8, Some(0x9600_0004)),
    (9, Some(0x9600_0044)),
    (10, Some(0x0040_0000_0000_0000)),
    // TEST_VA's pages across the TTBR0 switches: ASID 1's, 2's, 1's again,
    // 2's, 1's after a TLBI VAE1 while 2's tables were current, 2's, and
    // 1's after an ASIDE1.
    (11, Some(0x121_2321)),
];

/// System options for a boot bundle, which gets the VM's free RAM.
const BUNDLE_OPTIONS: &str = "--system --bundle \\EFI\\stela\\bundle";

/// The line the test kernel writes through the S5L UART model.
const KERNEL_BANNER: &str = "GUEST| STELA TEST KERNEL: MMU ON, UART MMIO";

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Scenario {
    /// The system-mode engine running Stela's own test kernel: a VHE host at
    /// EL2, 16 KiB granule, high-half kernel, EL2 abort and SVC, TTBR0
    /// switches, timer FIQ, two EL0 tasks.
    SystemKernel,
}

#[derive(Subcommand)]
pub enum Command {
    /// Build `stela.efi` with its NVK GPU stack and guest components.
    Build {
        #[arg(long)]
        release: bool,
        /// Include authored hardware fixtures in the mount.
        #[arg(long)]
        diagnostics: bool,
    },
    /// Boot `stela.efi` with an ARM image in x86 QEMU.
    Run(Box<Run>),
}

#[derive(Args)]
pub struct Run {
    /// The application; defaults to a fresh release build.
    #[arg(long)]
    image: Option<PathBuf>,
    /// Authorized local host-device firmware package, loaded before boot services end.
    #[arg(long, conflicts_with = "usb_image")]
    host_firmware: Option<PathBuf>,
    /// A system-mode guest image, run with the test kernel's options unless
    /// `--append` gives others; defaults to the scenario's image.
    #[arg(long)]
    module: Option<PathBuf>,
    /// The built-in image and options to run when no module is given.
    #[arg(long, value_enum, default_value_t = Scenario::SystemKernel)]
    scenario: Scenario,
    /// QEMU CPU model/properties. System mode defaults to an explicit Intel
    /// model: QEMU's AMD models lack the required AMD routing-control MSRs.
    #[arg(long)]
    cpu: Option<String>,
    /// Firmware-visible x86 cores, one thread each. This supplies the host
    /// inventory; the mount independently selects/checks native workers.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u16).range(1..=64))]
    host_cores: u16,
    /// Firmware-visible SMT threads per core. Native execution independently
    /// selects one checked thread per physical core.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u16).range(1..))]
    host_threads: u16,
    /// Guest machine memory for QEMU; defaults to 1G, 8G for system mode
    /// and 12G with a boot bundle.
    #[arg(long)]
    memory: Option<String>,
    /// A kernel boot bundle directory (`stela-dev boot prepare` output) to start
    /// on the system-mode engine; it is copied to `\EFI\stela\bundle`.
    #[arg(long, conflicts_with = "module")]
    bundle: Option<PathBuf>,
    /// A GPT disk image whose APFS container holds an installed system's
    /// Preboot volume, attached over NVMe through a snapshot: the mount
    /// builds the boot bundle from it with its own storage drivers and
    /// starts it on the system-mode engine.
    #[arg(long, conflicts_with_all = ["module", "bundle", "append", "payload"])]
    preboot_disk: Option<PathBuf>,
    /// A restore payload inventory (`stela-dev image-manifest
    /// --payload-manifest`), placed at `\EFI\stela\payload.json` for the
    /// mount's `--restore-manifest`, with the block hash file it names
    /// (`blocks.bin` beside it).
    #[arg(long, requires = "bundle")]
    restore_manifest: Option<PathBuf>,
    /// Have the restore host refuse a payload without recorded block
    /// hashes, which it would read whole before StartRestore: adds the
    /// mount's `--verify-on-read`. A USB image carries it in its install
    /// entry (`stela setup --verify-on-read`).
    #[arg(long, requires = "bundle")]
    verify_on_read: bool,
    /// Use QEMU's directory-backed FAT16 for a bundle that fits its
    /// volume, avoiding host disk-image utilities (QEMU rejects overflow).
    #[arg(long, requires = "bundle")]
    directory_volume: bool,
    /// Attach a raw GPT disk over NVMe, preserving writes; repeat for more
    /// disks. Select a partition with --storage SERIAL:GUID in --append, or
    /// give a USB image's entries the PC disk they name by its serial.
    #[arg(long, conflicts_with_all = ["storage_check", "payload", "preboot_disk"])]
    storage_disk: Vec<PathBuf>,
    /// NVMe serial of the --storage-disk at the same position. One disk
    /// defaults to STELAANS; several disks need distinct serials each.
    #[arg(long, requires = "storage_disk", value_parser = parse_storage_serial)]
    storage_serial: Vec<String>,
    /// Optional QEMU MDTS exponent for every --storage-disk controller.
    /// Values 1..=9 advertise 2^MDTS minimum pages per command, within
    /// QEMU's 1024-entry I/O vector limit. Omission keeps QEMU's default.
    #[arg(long, requires = "storage_disk", value_parser = clap::value_parser!(u8).range(1..=9))]
    storage_mdts: Option<u8>,
    /// Fresh directory for the boot volume, serial log and run report.
    #[arg(long)]
    output: PathBuf,
    /// Mount options, for example `--entry 0x100000 --budget 1000`.
    #[arg(long, allow_hyphen_values = true)]
    append: Option<String>,
    #[arg(long, default_value_t = 60.0)]
    timeout: f64,
    #[arg(long, value_enum, default_value_t = Display::None)]
    display: Display,
    /// Pause at reset with a GDB stub on this loopback port.
    #[arg(long)]
    gdb: Option<u16>,
    /// Serve the QEMU monitor over telnet on this loopback port.
    #[arg(long)]
    monitor: Option<u16>,
    /// Serve the UART on this loopback port as well as logging it, so that
    /// something can type at the guest's console. A window then shows no UART.
    #[arg(long)]
    console: Option<u16>,
    /// Serve QEMU's QMP on this loopback port, whose `input-send-event` takes
    /// the absolute pointer positions the monitor's commands do not.
    #[arg(long)]
    qmp: Option<u16>,
    /// Hide five-level paging from the guest CPU, so system mode runs on
    /// its four-level placement, as on a host without LA57.
    #[arg(long)]
    no_la57: bool,
    /// Advance QEMU's clocks by 2^SHIFT ns per executed x86 instruction
    /// (`-icount shift=SHIFT,sleep=off`) instead of host time, so that guest
    /// timeouts measure emulated work, as they would on hardware, and an idle
    /// guest skips ahead to its next timer.
    #[arg(long, value_parser = clap::value_parser!(u8).range(0..=10))]
    icount: Option<u8>,
    /// Run the x86 guest on the host's processor through KVM instead of
    /// translating it with TCG, with the host's CPU model and its invariant
    /// TSC unless `--cpu` names one. Needs a Linux host with `/dev/kvm`.
    #[arg(long, conflicts_with = "icount")]
    kvm: bool,
    /// Attach a GPT disk image through virtio-blk, NVMe and AHCI, have the
    /// mount overwrite and read back one partition of each, and verify the
    /// mount's hashes and every byte of the images.
    #[arg(long)]
    storage_check: bool,
    /// Check native conventional HDA after ExitBootServices. Playback compares
    /// WAV PCM; duplex checks both DMA engines with a silent null source.
    #[arg(long, value_enum, num_args = 0..=1, default_missing_value = "duplex",
        conflicts_with_all = ["module", "usb_image", "payload", "storage_check"])]
    audio_check: Option<audio_check::Mode>,
    /// Give the PC an IOMMU, which the mount takes over for the requesters
    /// its policy allows: Intel VT-d or AMD-Vi.
    #[arg(long, value_enum)]
    iommu: Option<Iommu>,
    /// Hand the mount a physical GPU through VFIO: this PCI function of the
    /// host (such as `0000:01:00.0`) and the other functions of its slot, in
    /// a slot of their own behind a PCIe root port. The emulated VGA stays as
    /// the firmware's display, for a card with no monitor; one with a monitor
    /// takes the display. `xtask/scripts/vfio-gpu.sh` binds it for a run.
    #[arg(long, requires = "kvm", conflicts_with = "iommu", value_parser = parse_pci_function)]
    vfio_gpu: Option<String>,
    /// Boot an existing USB disk image, such as a copy of a stick `stela
    /// setup` prepared, from its removable-media boot loader, attached as a
    /// USB mass-storage device; writes go to a temporary overlay unless
    /// `--usb-writable`.
    #[arg(long, conflicts_with_all = [
        "payload", "module", "bundle", "append", "storage_check", "preboot_disk"
    ])]
    usb_image: Option<PathBuf>,
    /// The `--usb-image` stick's USB serial number, which its setup's
    /// `--storage` and `--nor` options name.
    #[arg(long, requires = "usb_image", default_value = "STELAUSBIMAGE")]
    usb_serial: String,
    /// Keep the guest's writes in the `--usb-image` file, so a restore's
    /// install boots in a later run.
    #[arg(long, requires = "usb_image")]
    usb_writable: bool,
    /// Resume a `savevm` checkpoint of a `.qcow2` `--usb-image`, whose UEFI
    /// variables live beside it in `<image>.vars.qcow2` so checkpoints keep
    /// every writable drive.
    #[arg(long, requires = "usb_image")]
    loadvm: Option<String>,
    /// Give a bundle's guest the native USB keyboard and pointer, fed by
    /// QEMU's virtio keyboard and tablet: adds both devices and the mount's
    /// `--input`.
    #[arg(long, conflicts_with_all = ["module", "usb_image", "payload"])]
    input: bool,
    /// Attach QEMU's ICH6 HD Audio controller with a duplex codec on a silent
    /// backend, which probe's native HD Audio owner attaches whenever the
    /// mount starts the host's devices (for example with `--input`).
    #[arg(long, conflicts_with = "audio_check")]
    host_audio: bool,
    /// Give the mount a host network adapter: a `virtio-net-pci`, which the
    /// mount drives after ExitBootServices for its own requests (an
    /// install's TSS, `--network-check`), on a QEMU `stream` netdev to xtask's own LAN
    /// peer (`boot/lan.rs`): DHCP, DNS and TCP relayed to the host, logged
    /// to `network.log`. QEMU needs no slirp. A USB image's own entries say
    /// whether the mount uses it.
    #[arg(long, group = "network_adapter", conflicts_with_all = ["module", "payload"])]
    network: bool,
    /// PCI network controller exercised by --network. Physical-controller
    /// models test the same driver the mount uses on a PC.
    #[arg(long, value_enum, requires = "network")]
    network_model: Option<NetworkModel>,
    /// Have the mount POST once to this `http://` or `https://` URL over the
    /// host network at start and report the reply: adds `--network-check
    /// URL`. HTTPS needs `--kvm`: native TLS dates certificates by the
    /// invariant TSC, which TCG does not offer.
    #[arg(long, requires = "network_adapter")]
    network_check: Option<String>,
    /// Check DHCP, DNS, TCP and NIC ring reuse against a local HTTP peer
    /// with a 200,000-byte response whose digest the mount must reproduce.
    /// Requires system mode, such as --scenario system-kernel.
    #[arg(long, requires = "network_adapter", conflicts_with = "network_check")]
    network_self_check: bool,
    /// Give the mount its LAN address by hand instead of from the peer's
    /// DHCP: adds `--ipv4 10.0.2.15/24 --ipv4-gateway 10.0.2.2 --dns 10.0.2.2`.
    #[arg(long, requires = "network_adapter")]
    static_ipv4: bool,
    /// Give the guest its USB Ethernet adapter: a second `virtio-net-pci`
    /// on a LAN peer of its own (logged to `guest-network.log`), which the
    /// mount bridges to the guest's CDC-ECM adapter: adds `--guest-network`.
    /// A `--usb-image` boot takes `--guest-network` from the stick's options
    /// instead.
    #[arg(long, conflicts_with_all = ["module", "payload"])]
    guest_network: bool,
    /// Attach a raw disk over NVMe, preserving writes, that the guest sees
    /// whole as a USB mass storage device behind its native hub; repeat for
    /// more disks (at most six). Adds `--usb-storage STELAUSBn`.
    #[arg(long, conflicts_with_all = ["storage_check", "payload", "preboot_disk", "usb_image"])]
    usb_storage_disk: Vec<PathBuf>,
    /// Feed the guest's keyboard and pointer from USB HID devices the mount
    /// drives itself: a `qemu-xhci` with `usb-kbd`, `usb-tablet` and
    /// `usb-mouse`, and the mount's `--input`. A `--usb-image` boot's mount
    /// takes `--input` from the stick's options instead.
    #[arg(long, conflicts_with_all = ["module", "payload"])]
    usb_input: bool,
    /// Lend the mount a USB device of this PC, by `VENDOR:PRODUCT` in hex,
    /// on the `qemu-xhci` (`usb-host`), such as a Bluetooth adapter for the
    /// mount to bridge; repeat for more. QEMU needs write access to the
    /// device's `/dev/bus/usb` node.
    #[arg(long, value_parser = parse_usb_id, conflicts_with_all = ["module", "payload"])]
    usb_host: Vec<(u16, u16)>,
    /// Give the mount its host network adapter as a USB CDC-ECM device on
    /// the `qemu-xhci` (`usb-net`), on a LAN peer of its own
    /// (`usb-network.log`), for an install's TSS and `--network-check`.
    #[arg(long, group = "network_adapter", conflicts_with_all = ["module", "usb_image", "payload"])]
    usb_network: bool,
    /// Attach a raw disk as a QEMU `usb-storage` device on the `qemu-xhci`,
    /// with serial `STELAUSBDISKn`, preserving writes; the mount drives it
    /// over Bulk-Only and selects it by that serial like any host disk.
    #[arg(long, conflicts_with_all = ["storage_check", "payload", "preboot_disk", "usb_image"])]
    usb_disk: Vec<PathBuf>,
    /// Physical USB mass-storage model, including UAS with xHCI streams.
    #[arg(long, value_enum, requires = "usb_disk")]
    usb_disk_model: Option<UsbDiskModel>,
    /// Run install mode on a synthetic restore payload.
    #[arg(long, value_enum, conflicts_with_all = ["module", "append"])]
    payload: Option<PayloadMode>,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum UsbDiskModel {
    Bot,
    Uas,
    /// Disable SuperSpeed root ports to exercise the USB2 READY handshake.
    UasUsb2,
}

#[derive(Clone, Copy, PartialEq, Eq, ValueEnum)]
enum PayloadMode {
    /// Verify the Erase members and write nothing.
    Verify,
    /// Verify them from the whole IPSW, read through its ZIP directory.
    VerifyArchive,
    /// Also partition a blank NVMe disk, named by serial, and stage the
    /// members on it.
    Stage,
    /// The same on a virtio disk named by device path and size.
    StagePath,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Iommu {
    /// QEMU's `intel-iommu`, without interrupt remapping.
    Intel,
    /// QEMU's `amd-iommu`, without interrupt remapping.
    Amd,
}

#[derive(Clone, Copy, ValueEnum)]
enum Display {
    None,
    /// Show the UART in a host window, Cocoa's on macOS and GTK's elsewhere;
    /// adds no guest display device.
    Window,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum NetworkModel {
    Virtio,
    /// Intel 82574L, PCI ID 8086:10d3.
    E1000e,
    /// Intel 82576, PCI ID 8086:10c9.
    Igb,
}

impl NetworkModel {
    fn device(self) -> &'static str {
        match self {
            Self::Virtio => "virtio-net-pci",
            Self::E1000e => "e1000e",
            Self::Igb => "igb",
        }
    }
}

/// A path inside a QEMU option list, where a comma separates options and a
/// literal comma is written twice (QEMU `docs/system/invocation`), so a
/// directory such as `bundles/Mac14,2` stays one value.
pub(crate) fn option_path(path: &std::path::Path) -> String {
    path.display().to_string().replace(',', ",,")
}

/// A PCI function's sysfs name, `0000:01:00.0`, as VFIO and sysfs spell it.
fn parse_pci_function(value: &str) -> Result<String, String> {
    let bytes = value.as_bytes();
    let hex = |range: std::ops::Range<usize>| bytes[range].iter().all(u8::is_ascii_hexdigit);
    if bytes.len() == 12
        && (bytes[4], bytes[7], bytes[10]) == (b':', b':', b'.')
        && hex(0..4)
        && hex(5..7)
        && hex(8..10)
        && matches!(bytes[11], b'0'..=b'7')
    {
        Ok(value.to_ascii_lowercase())
    } else {
        Err(format!("{value:?} is not a PCI function like 0000:01:00.0"))
    }
}

/// A USB device's `VENDOR:PRODUCT`, in hex.
fn parse_usb_id(text: &str) -> Result<(u16, u16), String> {
    let (vendor, product) = text
        .split_once(':')
        .ok_or_else(|| format!("{text}: not VENDOR:PRODUCT"))?;
    let hex = |part: &str| {
        u16::from_str_radix(part.trim_start_matches("0x"), 16)
            .map_err(|_| format!("{text}: {part} is not a 16-bit hex number"))
    };
    Ok((hex(vendor)?, hex(product)?))
}

fn parse_storage_serial(value: &str) -> Result<String, String> {
    if !value.is_empty()
        && value.len() <= 20
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-')
    {
        Ok(value.into())
    } else {
        Err("storage serial must be one to 20 ASCII letters, digits or hyphens".into())
    }
}

/// The NVMe serial of the --usb-storage-disk at `index`, which the mount's
/// `--usb-storage` names.
fn usb_serial(index: usize) -> String {
    format!("STELAUSB{index}")
}

/// Pair each --storage-disk with its serial. Distinct serials keep the
/// guest's `--storage SERIAL:GUID` selection unambiguous.
fn storage_disks(paths: &[PathBuf], serials: &[String]) -> Result<Vec<(PathBuf, String)>> {
    let serials = match (paths.len(), serials.len()) {
        (1, 0) => vec!["STELAANS".to_owned()],
        (disks, named) if disks == named => serials.to_vec(),
        _ => bail!("each --storage-disk needs its own --storage-serial"),
    };
    for (index, serial) in serials.iter().enumerate() {
        ensure!(
            !serials[..index].contains(serial),
            "storage serial {serial} is attached twice"
        );
    }
    for path in paths {
        ensure!(
            path.is_file(),
            "storage disk is not a file: {}",
            path.display()
        );
    }
    Ok(paths.iter().cloned().zip(serials).collect())
}

pub fn run(command: Command) -> Result<ExitCode> {
    match command {
        Command::Build {
            release,
            diagnostics,
        } => {
            crate::gpu::fetch::fetch(false)?;
            crate::gpu::build_nvk(crate::gpu::default_jobs())?;
            println!("{}", build_for_boot(release, diagnostics, true)?.display());
            Ok(ExitCode::SUCCESS)
        }
        Command::Run(arguments) => launch(*arguments),
    }
}

pub fn build(release: bool) -> Result<PathBuf> {
    crate::gpu::build_nvk(crate::gpu::default_jobs())?;
    build_with_diagnostics(release, false, true)
}

fn build_for_boot(release: bool, diagnostics: bool, nvk: bool) -> Result<PathBuf> {
    let built = build_with_diagnostics(release, diagnostics, nvk)?;
    if !release {
        return Ok(built);
    }
    let image = built.with_file_name("stela-deploy.efi");
    deploy_image(&built, &image)?;
    Ok(image)
}

/// Keep the linked image and its symbols beside a smaller bootable image.
pub(crate) fn deploy_image(built: &Path, output: &Path) -> Result<()> {
    let objcopy = crate::tools::toolchain_llvm_tool("llvm-objcopy")
        .or_else(|| crate::tools::which("llvm-objcopy"))
        .context("install the Rust llvm-tools component to prepare a boot image")?;
    let parent = output
        .parent()
        .context("the boot image needs a parent directory")?;
    let temporary = tempfile::NamedTempFile::new_in(parent)?;
    exec(
        Process::new(objcopy)
            .arg("--strip-debug")
            .arg(built)
            .arg(temporary.path()),
    )?;
    temporary.persist(output)?;
    Ok(())
}

fn build_with_diagnostics(release: bool, diagnostics: bool, nvk: bool) -> Result<PathBuf> {
    let mut command = cargo();
    command.args(["build", "--locked", "--manifest-path", BOOT_MANIFEST]);
    command.args(["--target", BOOT_TARGET, "--target-dir", BOOT_TARGET_DIR]);
    if diagnostics {
        command.args(["--features", "diagnostics"]);
    }
    if nvk {
        command.args(["--features", "nvk"]);
    }
    if release {
        command.arg("--release");
    }
    crate::clang::toolset()?.apply(&mut command);
    command.env(
        crate::clang::TARGET_VARIABLE,
        &crate::clang::target_toolset()?.clang,
    );
    exec(&mut command)?;
    let profile = if release { "release" } else { "debug" };
    Ok(root()
        .join(BOOT_TARGET_DIR)
        .join(BOOT_TARGET)
        .join(profile)
        .join("stela.efi"))
}

/// The firmware a run's QEMU loads and the variable store it starts from.
struct Firmware {
    code: PathBuf,
    variables: PathBuf,
}

/// QEMU's bundled firmware, from `qemu -L help`, else the OVMF that Debian
/// and Ubuntu package apart from their QEMU.
fn firmware(qemu: &Path) -> Result<Firmware> {
    let output = Process::new(qemu).args(["-L", "help"]).output()?;
    String::from_utf8(output.stdout)?
        .lines()
        .map(Path::new)
        .find(|directory| directory.join(FIRMWARE_CODE).exists())
        .map(|directory| Firmware {
            code: directory.join(FIRMWARE_CODE),
            variables: directory.join(FIRMWARE_VARIABLES),
        })
        .or_else(|| {
            let [code, variables] = PACKAGED_FIRMWARE.map(PathBuf::from);
            code.exists().then_some(Firmware { code, variables })
        })
        .with_context(|| format!("QEMU has no {FIRMWARE_CODE}"))
}

/// Version reported by the configured launcher, which may establish its
/// runtime library environment before executing the native QEMU binary.
fn qemu_version(qemu: &Path) -> Result<String> {
    let output = Process::new(qemu).arg("--version").output()?;
    ensure!(output.status.success(), "QEMU --version failed");
    let version = String::from_utf8(output.stdout)?;
    ensure!(
        version.starts_with("QEMU emulator version "),
        "unexpected QEMU version output"
    );
    Ok(version.trim_end().to_owned())
}

/// Kernel-observed image of the launched process. If a custom wrapper forks
/// rather than execs, this identifies the wrapper process, not its child.
#[cfg(target_os = "macos")]
fn process_image(pid: u32) -> Option<PathBuf> {
    use std::os::unix::ffi::OsStringExt;
    let mut bytes = vec![0u8; 4096];
    // SAFETY: the buffer is writable for its advertised length; proc_pidpath
    // performs a read-only query on this task's child process.
    let count =
        unsafe { libc::proc_pidpath(pid as i32, bytes.as_mut_ptr().cast(), bytes.len() as u32) };
    if count <= 0 {
        return None;
    }
    bytes.truncate(bytes.iter().position(|byte| *byte == 0)?);
    Some(std::ffi::OsString::from_vec(bytes).into())
}

#[cfg(target_os = "linux")]
fn process_image(pid: u32) -> Option<PathBuf> {
    fs::read_link(format!("/proc/{pid}/exe")).ok()
}

#[cfg(not(any(target_os = "macos", target_os = "linux")))]
fn process_image(_pid: u32) -> Option<PathBuf> {
    None
}

fn qemu_cpu(requested: Option<&str>, no_la57: bool, system: bool) -> String {
    let model = requested.unwrap_or(if system {
        "max,vendor=GenuineIntel"
    } else {
        "qemu64"
    });
    if no_la57 {
        format!("{model},la57=off")
    } else {
        model.into()
    }
}

/// QEMU MTTCG runs one host thread per virtual CPU. Instruction counting
/// requires its single-thread engine (QEMU devel/tcg-icount.html).
/// Create a qcow2 `overlay` of raw `base` through QEMU's own monitor, the
/// tool this checkout has.
fn overlay(qemu: &Path, base: &Path, overlay: &Path) -> Result<()> {
    let mut child = Process::new(qemu)
        .args(["-machine", "none", "-nodefaults", "-display", "none"])
        .args(["-monitor", "stdio", "-drive"])
        .arg(format!(
            "if=none,id=base,format=raw,readonly=on,file={}",
            option_path(base)
        ))
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .spawn()?;
    use std::io::Write;
    writeln!(
        child.stdin.take().context("QEMU monitor")?,
        "snapshot_blkdev base {} qcow2\nquit",
        overlay.display()
    )?;
    ensure!(
        child.wait()?.success() && overlay.exists(),
        "create {}",
        overlay.display()
    );
    Ok(())
}

fn qemu_accelerator(icount: Option<u8>, kvm: bool) -> &'static str {
    if kvm {
        "kvm"
    } else if icount.is_some() {
        "tcg,thread=single"
    } else {
        "tcg,thread=multi"
    }
}

fn qemu_machine(keyboard: &str, hpet: bool) -> String {
    let hpet = if hpet { "on" } else { "off" };
    format!("q35,smm=off,sata=off,usb=off,smbus=off,hpet={hpet},i8042={keyboard},vmport=off")
}

fn usb_hpet_preflight(arguments: &Run) -> Result<()> {
    if arguments.usb_hpet_handoff() {
        let text = arguments.append.as_deref().unwrap_or_default();
        ensure!(
            !matches!(
                text.split_ascii_whitespace().next(),
                Some("--system" | "--host-check")
            ) && text
                .split_ascii_whitespace()
                .any(|word| matches!(word, "--usb-reset-check" | "--usb-media-check"))
                && arguments.icount.is_none(),
            "USB HPET ownership requires standalone USB diagnostics with the ordinary QEMU clock"
        );
    }
    Ok(())
}

fn launch(arguments: Run) -> Result<ExitCode> {
    // Refuse an incompatible diagnostic before building or opening inputs.
    usb_hpet_preflight(&arguments)?;
    let limit = if arguments.usb_image.is_some() || arguments.bundle.is_some() {
        MAX_LONG_TIMEOUT_SECONDS
    } else {
        MAX_TIMEOUT_SECONDS
    };
    ensure!(
        arguments.timeout > 0.0 && arguments.timeout <= limit,
        "timeout must be in (0, {limit}] seconds"
    );
    let attached = storage_disks(&arguments.storage_disk, &arguments.storage_serial)?;
    let image = match &arguments.image {
        Some(image) => image.clone(),
        None => build_for_boot(true, true, false)?,
    };
    let paths = storage::Paths::new(arguments.output.clone());
    if paths.output.exists() {
        bail!("{} already exists", paths.output.display());
    }
    let preboot = arguments
        .preboot_disk
        .as_deref()
        .map(preboot::prepare)
        .transpose()?;
    let (bytes, mut options) = guest(&arguments, preboot.as_ref())?;
    let disks = storage::check_disks(&arguments, &paths, &mut options)?;
    if arguments.input || arguments.usb_input {
        options.push_str(" --input");
    }
    storage::usb_options(&arguments, &mut options)?;
    if arguments.guest_network {
        options.push_str(" --guest-network");
    }
    if arguments.static_ipv4 {
        options.push_str(" --ipv4 10.0.2.15/24 --ipv4-gateway 10.0.2.2 --dns 10.0.2.2");
    }
    if arguments.verify_on_read {
        options.push_str(" --verify-on-read");
    }
    let system = options.split_whitespace().next() == Some("--system");
    if arguments.audio_check.is_some()
        || arguments.network_self_check
        || arguments.network_check.is_some()
    {
        ensure!(
            system,
            "host audio/network checks require system mode: use --scenario system-kernel, --bundle or --append starting with --system"
        );
    }
    if let Some(mode) = arguments.audio_check {
        options.push_str(&format!(
            " --audio-check {} --audio-own-hpet",
            mode.option()
        ));
    }
    let network = network::Check::start(&arguments, &mut options)?;
    let options = storage::payload(&arguments, &paths, options)?;
    ensure!(
        arguments.host_firmware.is_none() || system,
        "host firmware qualification requires --scenario system-kernel or another --system boot"
    );
    let fat = storage::write_volume(
        &arguments,
        &paths,
        storage::VolumeContents {
            image: &image,
            bytes,
            options: &options,
        },
    )?;

    let qemu = qemu()?;
    let firmware = firmware(&qemu)?;
    let (variables, format) = machine::variables(
        &arguments,
        machine::VariableStoreOptions {
            qemu: &qemu,
            firmware: &firmware,
            output: &paths.output,
        },
    )?;
    // A USB image carries the boot bundle that `stela setup` wrote.
    let bundle = arguments.bundle.is_some() || preboot.is_some() || arguments.usb_image.is_some();
    // QEMU numbers PCI devices in the order they are given, so the parts
    // attach theirs in a fixed order.
    let mut command = machine::command(
        &arguments,
        machine::MachineOptions {
            qemu: &qemu,
            firmware: &firmware,
            variables: &variables,
            format,
            system,
            bundle,
        },
    );
    storage::attach_volume(
        &arguments,
        &paths,
        storage::VolumeOptions {
            fat: fat.as_deref(),
            format,
        },
        &mut command,
    );
    machine::attach_exit(&arguments, &paths.output, &mut command);
    storage::attach_disks(
        &arguments,
        &paths,
        storage::DiskImages {
            disks: &disks,
            preboot: preboot.as_ref(),
        },
        &mut command,
    );
    machine::attach_display(&arguments, bundle, &mut command);
    machine::attach_vfio_gpu(&arguments, &mut command)?;
    machine::attach_audio(&arguments, &mut command);
    network::attach(&arguments, &paths.output, &mut command)?;
    machine::attach_usb(&arguments, &mut command);
    storage::attach_usb_boot(&arguments, &mut command);
    machine::attach_usb_input(&arguments, &mut command);
    machine::attach_usb_host(&arguments, &mut command);
    network::attach_usb(&arguments, &paths.output, &mut command)?;
    storage::attach_usb_disks(&arguments, &mut command);
    machine::attach_console(&arguments, &paths.serial, &mut command);
    storage::attach_storage_disks(&attached, arguments.storage_mdts, &mut command);
    machine::attach_debugger(&arguments, &mut command);
    let command_line: Vec<String> = std::iter::once(command.get_program())
        .chain(command.get_args())
        .map(|argument| argument.to_string_lossy().into_owned())
        .collect();
    let qemu_version = qemu_version(&qemu)?;
    // Keep launch-time input identities: another build can replace these
    // paths while a long restore is still running. Completion must not bind
    // the old run to the replacement's bytes.
    let qemu_binding = binding(&qemu)?;
    let image_binding = binding(&image)?;
    let source_binding = paths
        .source
        .exists()
        .then(|| binding(&paths.source))
        .transpose()?;

    let started = Instant::now();
    let mut child = command
        .stdout(fs::File::create(paths.output.join("stdout.txt"))?)
        .stderr(fs::File::create(paths.output.join("stderr.txt"))?)
        .stdin(if arguments.audio_check.is_some() {
            Stdio::piped()
        } else {
            Stdio::null()
        })
        .spawn()?;
    // The local launcher can be a shell wrapper. Observe its running image
    // after exec, keeping the launcher binding and QEMU version separately.
    std::thread::sleep(Duration::from_millis(10));
    let qemu_process_image = process_image(child.id()).and_then(|path| binding(&path).ok());
    // A paused debugging session has no deadline.
    let limit = match arguments.gdb {
        Some(_) => Duration::MAX,
        None => Duration::from_secs_f64(arguments.timeout).saturating_sub(started.elapsed()),
    };
    let waited = if arguments.audio_check.is_some() {
        audio_check::wait_for_stop(&mut child, &paths.serial, limit)
    } else {
        wait(&mut child, limit)
    };
    let status = match waited {
        Ok(status) => status,
        Err(error) => {
            stop(&mut child);
            return Err(error);
        }
    };
    report::finish(
        &arguments,
        report::Outcome {
            paths,
            preboot,
            disks,
            attached,
            network,
            options,
            command_line,
            qemu: qemu_binding,
            qemu_version,
            qemu_process_image,
            image: image_binding,
            source: source_binding,
            started,
            status,
        },
    )
}

impl Run {
    fn usb_hpet_handoff(&self) -> bool {
        self.append.as_deref().is_some_and(|options| {
            options
                .split_ascii_whitespace()
                .any(|word| word == "--usb-own-hpet")
        })
    }

    fn hpet_enabled(&self) -> bool {
        self.audio_check.is_some() || self.usb_hpet_handoff()
    }

    /// Install mode and the storage check run no guest.
    fn guestless(&self) -> bool {
        self.payload.is_some() || self.storage_check
    }
}

/// What the mount boots and its options: the system-mode guest image
/// `source.bin` holds (none for a bundle, a Preboot disk or a run without a
/// guest) and `stela.cfg`'s first options.
fn guest(arguments: &Run, preboot: Option<&preboot::Preboot>) -> Result<(Vec<u8>, String)> {
    let append = arguments.append.clone();
    let (bytes, options) = match (&arguments.module, arguments.scenario) {
        _ if preboot.is_some() => (Vec::new(), preboot.unwrap().options()),
        _ if arguments.bundle.is_some() => {
            (Vec::new(), append.unwrap_or_else(|| BUNDLE_OPTIONS.into()))
        }
        _ if arguments.guestless() => (Vec::new(), append.unwrap_or_default()),
        (Some(module), _) => (
            fs::read(module)?,
            append.unwrap_or_else(|| KERNEL_OPTIONS.into()),
        ),
        (None, Scenario::SystemKernel) => {
            (kernel()?, append.unwrap_or_else(|| KERNEL_OPTIONS.into()))
        }
    };
    Ok((bytes, options))
}

/// The first printed requirement, including the diagnostics of operations
/// answered degraded. Buffered diagnostics are not a causal clock.
fn first_unsupported(serial: &str) -> Option<&str> {
    serial.lines().find_map(|line| {
        line.strip_prefix("UNMODELED ")
            .or_else(|| line.strip_prefix("STOP UNSUPPORTED "))
    })
}

/// The immediate guest console's first panic, before any nested panic or
/// subsequent machine reset. Ordinary messages about panic support do not match.
fn first_guest_panic(serial: &str) -> Option<&str> {
    serial.lines().find_map(|line| {
        let message = line.strip_prefix("GUEST|")?.trim_start();
        (message.starts_with("panic(") || message.starts_with("panic:")).then_some(message)
    })
}

/// Copy `bundle.json` and every file it lists.
fn copy_bundle(from: &Path, to: &Path) -> Result<()> {
    let manifest = fs::read(from.join("bundle.json"))
        .with_context(|| format!("{} has no bundle.json", from.display()))?;
    let value: Value = serde_json::from_slice(&manifest)?;
    fs::create_dir_all(to)?;
    fs::write(to.join("bundle.json"), &manifest)?;
    for file in value["files"]
        .as_array()
        .context("bundle.json lists no files")?
    {
        let name = file["name"]
            .as_str()
            .context("bundle file without a name")?;
        ensure!(
            !name.contains(['/', '\\']),
            "bundle file {name} names a directory"
        );
        fs::copy(from.join(name), to.join(name))?;
    }
    Ok(())
}

/// QEMU's exit status when the engine reports a terminal idle state.
const IDLE_EXIT: u8 = (0x10 << 1) | 1;
const FAILED_EXIT: u8 = 1;

/// Check the test kernel's results in the serial log.
fn verify_kernel(serial: &str) -> Result<(), String> {
    if !serial.contains("STOP idle") {
        return Err("the engine did not reach the kernel's final idle state".into());
    }
    if !serial.contains(KERNEL_BANNER) {
        return Err("the kernel's UART banner did not reach the console".into());
    }
    let mut registers = [0u64; 32];
    for line in serial.lines() {
        let Some(rest) = line.strip_prefix('X') else {
            continue;
        };
        let Some((first, values)) = rest.split_once(':') else {
            continue;
        };
        let Ok(first) = first.parse::<usize>() else {
            continue;
        };
        for (offset, value) in values.split_whitespace().enumerate() {
            let value = u64::from_str_radix(value.trim_start_matches("0x"), 16)
                .map_err(|_| format!("bad register value {value}"))?;
            if let Some(slot) = registers.get_mut(first + offset) {
                *slot = value;
            }
        }
    }
    for (index, expected) in KERNEL_RESULTS {
        let actual = registers[index];
        let good = match expected {
            Some(value) => actual == value,
            None => actual != 0,
        };
        if !good {
            return Err(format!(
                "X{index} = {actual:#x}, expected {}",
                expected.map_or("nonzero".into(), |value| format!("{value:#x}"))
            ));
        }
    }
    Ok(())
}

/// A byte range of a host disk image.
struct Range {
    file: fs::File,
    start: u64,
    bytes: u64,
}

impl ReadAt for Range {
    type Error = std::io::Error;
    fn len(&self) -> u64 {
        self.bytes
    }
    fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> std::io::Result<()> {
        use std::io::{Read, Seek, SeekFrom};
        self.file.seek(SeekFrom::Start(self.start + offset))?;
        self.file.read_exact(buffer)
    }
}

/// Read a staging run's target disk back: its GPT, and every staged member
/// checked against the staging header's SHA-256.
fn staged(target: &Path) -> Result<Value> {
    let disk = gpt::Disk {
        block_bytes: 512,
        blocks: fs::metadata(target)?.len() / 512,
    };
    let mut whole = Range {
        file: fs::File::open(target)?,
        start: 0,
        bytes: disk.blocks * 512,
    };
    let mut header = vec![0u8; 512];
    whole.read_at(512, &mut header)?;
    let error = |e: PayloadError| anyhow::anyhow!("{e}");
    let Some(info) = gpt::parse_header(disk, 1, &header).map_err(error)? else {
        return Ok(json!({"gpt": false}));
    };
    let mut entries = vec![0u8; info.entry_array_bytes() as usize];
    whole.read_at(info.entries_lba * 512, &mut entries)?;
    let table = gpt::parse_entries(&info, &entries).map_err(error)?;
    let partitions: Vec<Value> = table
        .partitions
        .iter()
        .map(|p| {
            json!({"role": format!("{:?}", p.role()), "first_lba": p.first_lba,
            "last_lba": p.last_lba, "guid": p.unique_guid.to_string()})
        })
        .collect();
    let staging = table
        .partitions
        .iter()
        .find(|p| p.role() == Some(gpt::Role::Staging))
        .context("no staging partition")?;
    let mut source = StagedSource::open(Range {
        file: fs::File::open(target)?,
        start: staging.first_lba * 512,
        bytes: staging.blocks() * 512,
    })
    .map_err(error)?;
    let members = source.manifest().members.clone();
    let mut buffer = vec![0u8; 1 << 20];
    for member in &members {
        copy_verified(&mut source, member, &mut buffer, |_| Ok::<(), ()>(()))
            .map_err(|e| anyhow::anyhow!("{e}"))?;
    }
    Ok(json!({
        "gpt": true,
        "disk_guid": table.disk_guid.to_string(),
        "partitions": partitions,
        "members_verified": members.iter().map(|m| &m.name).collect::<Vec<_>>(),
    }))
}

fn directory_bytes(path: &Path) -> Result<u64> {
    let mut total = 0;
    for entry in fs::read_dir(path)? {
        let entry = entry?;
        total += match entry.file_type()?.is_dir() {
            true => directory_bytes(&entry.path())?,
            false => entry.metadata()?.len(),
        };
    }
    Ok(total)
}

/// A raw FAT32 image of `volume`, without a partition table, which the
/// firmware mounts whole.
fn fat_image(volume: &Path, output: &Path) -> Result<PathBuf> {
    let bytes = directory_bytes(volume)?;
    let size = (bytes + bytes / 8 + (64 << 20)).next_multiple_of(1 << 20);
    let image = output.join("volume.img");
    fat32::write(volume, &image, size, "STELA")?;
    Ok(image)
}

#[cfg(test)]
mod tests;
