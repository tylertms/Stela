//! Engine start: from the loader's handoff to the first guest instruction, in
//! phases that each own what they start.

use super::*;
use crate::Result;
use alloc::string::ToString;
use stela_contracts::refuse;

use stela_boot_core::guest_timeouts::DEBUG_ACK_SECONDS;

/// The boot-args that scale XNU's lock timeouts for this host, each with a
/// leading space. They assume the modeled chip's cores; a slower host's
/// translated holders get that many times longer, by the clock this core
/// sustains as measured against its TSC at `tsc_hz` (`guest_timeouts`):
/// about 4 ms, before any AP or guest instruction runs.
pub(super) fn lock_timeout_arguments(tsc_hz: u64) -> String {
    use stela_boot_core::guest_timeouts::{
        MODELED_CORE_HZ, lock_scale, lock_scale_arguments, measure_core_hz,
    };
    let core_hz = measure_core_hz(tsc_hz);
    let scale = lock_scale(core_hz);
    output::arguments(format_args!(
        "GUEST TIMEOUTS lock_scale={scale} core_hz={core_hz} tsc_hz={tsc_hz} \
         modeled_hz={MODELED_CORE_HZ}\n"
    ));
    lock_scale_arguments(scale)
}

/// The boot bundle `--preboot` names, built from the installed system on
/// `partition`: its macOS, its staged update or its recoveryOS.
fn from_disk(
    partition: alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>,
    options: &Options,
    bluetooth: bool,
    tsc_hz: u64,
) -> Vec<(String, Vec<u8>)> {
    crate::status::begin("Reading macOS's boot files from its disk");
    // The kernel's text on the screen and on the debug UART, which the
    // boot log records, as setup's bundles have it. A stackshot or panic
    // waits for every PE to answer its debugger cross-call
    // (DebuggerXCallEnter), and a non-panic one panics "Debugger synch
    // pending" at the timeout. A press of the power button takes such a
    // stackshot: AppleM68Buttons' diagnostic chord (`stackshot-mask`, which
    // J473's tree leaves to the default, its one button). XNU's default is
    // 1,440,000 ticks (60 ms), short for PEs that answer at their next
    // exit, so this gives `DEBUG_ACK_SECONDS`, well inside the watchdog
    // mechanism a panic arms (`guest_timeouts`). Machine timeout boot-args are in picoseconds
    // (osfmk/kern/startup.h), converted to ticks only when overridden.
    // System sleep is not modeled, so `noidle` stops the idle timer that
    // would ask for it.
    // The guest's Bluetooth is a host controller bridged over USB, when the
    // host lends one: bluetoothd and BlueTool then take USB, transport 0
    // (26A434 bluetoothd's transport switch at x86_64 0x1001f84e7). Without
    // one there is no chip on any transport, and bluetoothd does what it does
    // on a Mac whose radio fails to boot: it logs "Bluetooth error -
    // restarting" (reason 1214, chip boot failure) and restarts its stack
    // every 40 s or so, whether it looks on its default UART (3) or the
    // board's PCIe (7). Overriding it to USB then would be worse: its HCI
    // initialization fails (`OI_HCIAPI_Init_Vars`) and it aborts with
    // "Bluetooth fatal error" (reason 0x5c), each time with a crash report.
    let mut defaults = boot_bundle::Options {
        ecid: options.ecid,
        boot_command: options.boot_command,
        boot_args: format!(
            "-v serial=3 serial-device-name=uart0 ml-timeout-debug-ack={} noidle=1",
            stela_boot_core::guest_timeouts::picoseconds(DEBUG_ACK_SECONDS)
        ),
        ..boot_bundle::Options::default()
    };
    if bluetooth {
        defaults.boot_args.push_str(" bt-transport-override=0");
    }
    defaults.boot_args.push_str(&lock_timeout_arguments(tsc_hz));
    // The automatic Metal plugin repair NVRAM asked for is the manual
    // remedy's boot: single-user, filesystem protection off.
    let maintenance_write = options.maintenance_write || options.bundle_repair;
    let single_user = options.single_user || options.bundle_repair;
    if maintenance_write {
        // CSR_ALLOW_UNRESTRICTED_FS; the kext grant and other policy bits stay separate.
        defaults.security.sip_bits |= 2;
        output::arguments(format_args!(
            "MAINTENANCE filesystem protection disabled for this boot ({})\n",
            if options.maintenance_write {
                "--maintenance-write"
            } else {
                "the automatic Metal plugin repair"
            }
        ));
    }
    if single_user {
        defaults.boot_args.push_str(" -s");
    }
    bundle_repair::boot(if options.bundle_repair {
        stela_boot_core::bundle_repair::Boot::Repair
    } else if options.single_user {
        stela_boot_core::bundle_repair::Boot::SingleUser
    } else {
        stela_boot_core::bundle_repair::Boot::Normal
    });
    let result = if options.upgrade {
        bundle::upgrade_from_disk(partition, options.boot_volume, &defaults)
    } else if let Some(kind) = options.recovery {
        bundle::recovery_from_disk(partition, kind, options.boot_volume, &defaults)
    } else {
        let kexts = gpu::kexts();
        let request = installed::Request {
            volume: options.boot_volume,
            kexts: &kexts,
            ..installed::Request::default()
        };
        let mut defaults = defaults;
        if options.safe_boot {
            defaults.boot_args.push_str(" -x");
        }
        bundle::from_disk(partition, &request, &defaults)
    };
    let files = result.unwrap_or_else(|error| {
        output::fatal(format_args!(
            "no boot bundle could be built from the Preboot volume: {error}"
        ))
    });
    let bytes: usize = files.iter().map(|(_, contents)| contents.len()).sum();
    crate::status::done(&format!("{} MB", bytes.div_ceil(1 << 20)));
    files
}

/// Engine-phase entry, called by the loader on the window stack.
pub extern "sysv64" fn entry(handoff: u64) -> ! {
    let Started {
        handoff,
        mut options,
        host_firmware,
        native_processors,
    } = start(handoff);
    let mut host = open_host(handoff, &mut options, host_firmware, native_processors);
    let storage = open_storage(handoff, &options, &mut host);
    // A single-user boot is a console, which draws through no GPU.
    if options.boots_macos()
        && !options.single_user
        && !options.bundle_repair
        && let Some(reason) = gpu::no_metal_device()
    {
        output::fatal(format_args!("macOS cannot run without a GPU: {reason}"));
    }
    let boot = Boot {
        handoff,
        options: &options,
        native_processors,
    };
    #[cfg(feature = "diagnostics")]
    if options.native_bootstrap_check || options.native_shared_check {
        let _ = start_processors(boot, &mut host);
        output::fatal("native processor diagnostic returned");
    }
    run_guest(boot, host, storage)
}

/// The part of the engine heap that holes cannot take from large requests.
const HEAP_RESERVE_SHARE: usize = 16;

/// What every phase after the host's reads: the loader's handoff,
/// `stela.cfg` and the validated physical processors.
#[derive(Clone, Copy)]
struct Boot<'a> {
    handoff: &'static Handoff,
    options: &'a Options,
    native_processors: Option<&'static crate::native_smp::firmware::Capture>,
}

/// The engine's own start: its interrupts, heap, entropy and screen taken,
/// `stela.cfg` parsed and the physical processors validated.
struct Started {
    handoff: &'static Handoff,
    options: Options,
    host_firmware: Option<alloc::sync::Arc<stela_drivers::firmware::Catalog>>,
    native_processors: Option<&'static crate::native_smp::firmware::Capture>,
}

/// This processor's GS base, which names it to a device's start budget:
/// devices start after the host record is installed, so the starting
/// processor's base is its own record, which no other processor's base
/// equals. Read only once an armed budget has passed.
fn processor_gs() -> u64 {
    // SAFETY: read-only architectural CPL0 GS context, not device MMIO.
    unsafe { crate::cpu::msr::GS_BASE.read() }
}

/// A driver starts on `device`: the boot log names it before the start
/// begins, so a start that never ends leaves its culprit in the record the
/// USB log receives when it resumes, and the screen's step shows it.
fn device_starting(device: &str) {
    output::arguments(format_args!("HOST DEVICE {device} start\n"));
    crate::status::progress(device);
}

fn start(handoff: u64) -> Started {
    output::disable_firmware_console();
    // SAFETY: the first engine code; interrupts are off and nothing else runs.
    unsafe { interrupts::install(interrupts::interrupt) };
    // SAFETY: the loader wrote the handoff page and passed its physmap address.
    let handoff = unsafe { &*(handoff as *const Handoff) };
    if handoff.magic != MAGIC {
        output::fatal("engine handoff has the wrong magic");
    }
    // Before any output or console input: this image's copy has not looked
    // at COM1.
    crate::serial::adopt(handoff.serial_present);
    // From here a stop the USB's log cannot keep leaves its record in the
    // firmware; an earlier one has only the screen.
    crate::stop_record::arm(handoff.stop_record, Some(handoff.root));
    layout::set(handoff.placement);
    // Before this image's first line: the loader's lines that no log stored.
    let lines = handoff
        .loader_lines
        .bytes
        .min(stela_boot_core::loader_lines::BYTES as u64) as usize;
    // SAFETY: the loader wrote these bytes in the handoff allocation's
    // tail, which the physmap keeps mapped like the handoff itself.
    output::carry_loader_lines(unsafe {
        core::slice::from_raw_parts(
            layout::physical(handoff.loader_lines.physical) as *const u8,
            lines,
        )
    });
    // Use the physmap directly: its CPU and physical bases have the
    // same alignment phase. A separately placed heap alias cannot in general
    // satisfy higher-order CPU/physical page alignment simultaneously.
    let heap = layout::physical(handoff.heap.physical);
    // SAFETY: this reserved heap is unused and its RW+NX physmap is shared by
    // every native root for the complete engine lifetime.
    unsafe { crate::ALLOCATOR.initialize(heap as *mut u8, handoff.heap.bytes as usize) };
    crate::ALLOCATOR.reserve_tail(handoff.heap.bytes as usize / HEAP_RESERVE_SHARE);
    super::large_blocks::install();
    // SAFETY: this loader-owned private seed is held in the shared physmap,
    // exclusively consumed here before any native service is published.
    if let Err(error) = unsafe { crate::random::initialize(handoff.random_seed) } {
        output::arguments(format_args!("HOST entropy unavailable: {error:?}\n"));
    }
    // SAFETY: this phase owns that reserved heap and kept WB mapping.
    unsafe {
        crate::native_memory::initialize(
            heap as usize,
            handoff.heap.physical,
            handoff.heap.bytes as usize,
            layout::physical(0),
        )
    };
    // SAFETY: original private reservation held in the shared physmap, before
    // any remapper or AP has started.
    if let Err(error) = unsafe { crate::native_memory::initialize_amd_tables(handoff.amd_tables) } {
        output::arguments(format_args!(
            "HOST AMD-Vi table memory unavailable: {error:?}\n"
        ));
    }
    // SAFETY: loader-reserved immutable pages, kept in every shared-window
    // physmap; metadata and Arc are created in this active engine allocator.
    let host_firmware = unsafe { crate::host_firmware::after_ebs(&handoff.host_firmware) }
        .unwrap_or_else(|error| output::fatal(error));
    // Before any user of the rate: this image's copy has not been found.
    crate::firmware::adopt_tsc(
        handoff.tsc_hz,
        handoff.tsc_source,
        handoff.tsc_tolerance_ppm,
    );
    crate::status::set_clock(handoff.tsc_hz);
    stela_drivers::on_wait(crate::status::tick);
    stela_drivers::on_work(device_starting);
    stela_drivers::budget::on_processor(processor_gs);
    crate::status::take_screen(
        handoff
            .screen
            .map(|screen| screen.relocated(layout::physical(screen.base as u64) as usize)),
    );
    crate::status::replay(&handoff.history[..handoff.history_bytes]);
    crate::host_inventory::restore(&handoff.host[..handoff.host_bytes.min(handoff.host.len())]);
    output::arguments(format_args!(
        "STELA SYSTEM ENGINE window={:#x} placement={:?} tsc_hz={} guest_ranges={} \
         heap={}M code={}M pool={}M\n",
        layout::window(),
        handoff.placement,
        handoff.tsc_hz,
        handoff.guest_count,
        handoff.heap.bytes >> 20,
        handoff.code.bytes >> 20,
        handoff.pool.bytes >> 20,
    ));
    // SAFETY: the options were left allocated by the loader.
    let text = unsafe {
        core::slice::from_raw_parts(
            layout::physical(handoff.options.physical) as *const u8,
            handoff.options.bytes as usize,
        )
    };
    let options = match core::str::from_utf8(text)
        .map_err(|_| refuse!("stela.cfg is not UTF-8"))
        .and_then(Options::parse)
    {
        Ok(options) => options,
        Err(error) => output::fatal(error),
    };
    // SAFETY: the loader handoff and all reservations stay pinned in the physmap.
    let native_processors = unsafe { crate::native_smp::validate(handoff, options.native_cores) }
        .unwrap_or_else(|error| output::fatal(error));
    if (options.native_bootstrap_check || options.native_shared_check)
        && options.native_cores.is_none()
    {
        output::fatal("native bootstrap check needs an explicit processor count");
    }
    Started {
        handoff,
        options,
        host_firmware,
        native_processors,
    }
}

/// The installed host with its platform and devices, and the guest's RAM
/// placed at the DRAM base its bundle was prepared for.
struct OpenedHost {
    owner: host::HostOwner,
    apic_hz: u64,
    /// The TSC origin every physical executor shares.
    tsc_base: u64,
    acpi: Option<&'static aml::HostAcpi>,
    /// The owned MMIO aperture, which the processor diagnostics check.
    #[cfg(feature = "diagnostics")]
    mmio: Option<&'static crate::native_mmio::Source>,
    devices: HostDevices,
    ram: Vec<RamRange>,
    /// The boot bundle `--preboot` built from the installed system.
    built: Option<Vec<(String, Vec<u8>)>>,
    /// Whether `--nor` keeps the machine state in the `--storage`
    /// partition's tail.
    state_in_storage: bool,
    /// Whether an install streams its remote image.
    remote_image: bool,
}

/// Install the host, start its platform and devices, and place the guest's
/// RAM.
fn open_host(
    handoff: &'static Handoff,
    options: &mut Options,
    host_firmware: Option<alloc::sync::Arc<stela_drivers::firmware::Catalog>>,
    native_processors: Option<&'static crate::native_smp::firmware::Capture>,
) -> OpenedHost {
    // The guest's RAM at the configured DRAM base. A bundle's own base is
    // read from its disk by the host drivers, which run after the host is
    // installed; the placements then move there (`HostOwner::place_ram`).
    let mut ram = guest_ram(handoff, options).unwrap_or_else(|error| output::fatal(error));
    // The translation store the loader read, and its partition when the
    // pack can be rewritten.
    let translation_store = handoff.files[..handoff.file_count]
        .iter()
        .find(|file| file.name() == translations::FILE)
        // SAFETY: the store's file is read here alone, once, and its
        // region is the store's for the rest of the boot.
        .and_then(|file| translations::install(unsafe { bundle::contents_mut(file) }))
        .filter(|_| handoff.storage.present);
    // The Metal device's stored pipelines, before any GPU opens.
    #[cfg(feature = "nvk")]
    if let Some(file) = handoff.files[..handoff.file_count]
        .iter()
        .find(|file| file.name() == super::pipelines::FILE)
    {
        super::pipelines::install(bundle::contents(file));
    }
    let fdr_placement = restore::inventory_file(handoff, stela_runtime::restore::fdr::FILE)
        .map(fdr::Placement::parse)
        .transpose()
        .unwrap_or_else(|error| output::fatal(&error));
    let fdr_partition = fdr_placement.as_ref().map(|placement| placement.partition);
    // `--nor` naming the `--storage` partition keeps the machine state in
    // that partition's tail (`pc::machine_state`): one device, split below.
    let state_in_storage = options.nor.is_some() && options.nor == options.storage;
    let targets: Vec<&Target> = options
        .storage
        .iter()
        .chain(options.restore_source.iter())
        .chain(options.nor.iter().filter(|_| !state_in_storage))
        .collect();
    #[cfg(feature = "diagnostics")]
    if options.audio_check.is_some() && !options.audio_own_hpet {
        output::fatal("the audio check requires an explicit --audio-own-hpet handoff");
    }
    let InstalledHost {
        owner: mut host_owner,
        apic_hz,
        tsc_base: native_tsc_base,
    } = install_host(handoff, options, &ram);
    let Platform {
        acpi: host_acpi,
        mmio: native_mmio,
        remappers,
        work: kernel_work,
    } = start_platform(handoff, &mut host_owner, options);
    // The signing service is asked only during an install, so a boot that
    // installs nothing leaves the adapter to the guest. An install without a
    // medium partition also streams its remote image.
    let remote_image = options.install.is_some() && options.install_source.is_none();
    // macOS on the USB and Apple's answers to every signing request the
    // erase install makes: the install needs no network, so none is opened.
    // An update sends other requests and needs the signing server.
    let offline = options.install.is_some()
        && stela_runtime::restore::tss::staged::Use::of(
            restore::staged_answers(handoff).as_ref(),
            options.update_restore,
            remote_image,
        )
        .offline();
    if offline {
        output::arguments(format_args!(
            "INSTALL OFFLINE macOS and Apple's signing answers are on the USB; no network is opened\n"
        ));
    } else if options.install.is_some() {
        output::arguments(format_args!(
            "INSTALL NETWORK {}\n",
            match remote_image {
                true => "macOS downloads and Apple's signing server is asked during the install",
                false => "Apple's signing server is asked during the install",
            }
        ));
    }
    let wants_network = (options.install.is_some() && !offline) || options.network_check.is_some();
    let mut devices = start_devices(
        handoff,
        options,
        &Wanted {
            firmware: host_firmware.clone(),
            processors: native_processors,
            targets: &targets,
            store: translation_store,
            fdr: fdr_partition,
            network: wants_network,
        },
        host_acpi,
        &native_mmio,
        remappers,
        kernel_work
            .as_ref()
            .copied()
            .map_err(alloc::string::String::as_str),
    );
    remappers.report_faults();
    // An install without staged signing answers cannot finish without
    // Apple's signing server: stop before the restore changes the Mac's
    // disk, not at its first signing request.
    if options.install.is_some()
        && let Some(Err(reason)) = &devices.host_network
    {
        // The network step's lines name each adapter; the one stop line
        // reads them as sentences.
        output::fatal(stela_runtime::restore::tss::no_network(
            remote_image,
            &reason.to_string().replace('\n', " "),
        ));
    }
    let built = devices.preboot.take().map(|partition| {
        from_disk(
            partition,
            options,
            devices.host_bluetooth.is_some(),
            handoff.tsc_hz,
        )
    });
    // A system boots as the board it was installed on: libignition finds its
    // Cryptex1 tickets by that board's model.
    if options.bundle || built.is_some() {
        let files = &handoff.files[..handoff.file_count];
        let manifest = bundle::manifest(files, built.as_deref()).unwrap_or_else(|error| {
            output::fatal(format_args!(
                "the boot bundle's manifest could not be read: {error}"
            ))
        });
        options.development_board = manifest
            .target
            .is_some_and(|target| target.device_class.ends_with("dev"));
        // A bundle runs at the DRAM base it was prepared for, its SoC's.
        if options.bundle && !options.dram_explicit {
            let base = manifest.options.dram_base;
            if !base.is_multiple_of(LARGE) {
                output::fatal("the bundle's DRAM base is not 2 MiB aligned");
            }
            options.dram_base = base;
        }
    }
    ram::place(&mut ram, options.dram_base);
    host_owner
        .place_ram(options.dram_base)
        .unwrap_or_else(|error| output::fatal(error));
    for range in &ram {
        output::arguments(format_args!(
            "GUEST DRAM pa={:#x} bytes={:#x} host={:#x}\n",
            range.guest, range.bytes, range.host
        ));
    }
    #[cfg(feature = "diagnostics")]
    if let Some(url) = &options.network_check {
        let Some(Ok(network)) = &devices.host_network else {
            output::fatal("the network check failed")
        };
        // Any HTTP status proves the path end to end; only a transport
        // failure (link, DHCP, DNS, TCP, malformed reply) fails the check.
        let checked = network.0.borrow_mut().post_url(url, &[], "text/plain", b"");
        match checked {
            Ok(body) => {
                output::arguments(format_args!(
                    "NETWORK CHECK body bytes={} sha256={}\n",
                    body.len(),
                    stela_formats::sha256(&body)
                ));
                output::arguments(format_args!("NETWORK CHECK passed\n"))
            }
            Err(stela_runtime::net::NetError::Status(_)) => {
                output::arguments(format_args!("NETWORK CHECK passed\n"))
            }
            Err(error) => output::fatal(format_args!("the network check failed: {error}")),
        }
    }
    OpenedHost {
        owner: host_owner,
        apic_hz,
        tsc_base: native_tsc_base,
        acpi: host_acpi,
        #[cfg(feature = "diagnostics")]
        mmio: native_mmio,
        devices,
        ram,
        built,
        state_in_storage,
        remote_image,
    }
}

/// The guest's storage, its restore source and its machine state.
struct Storage {
    /// What backs the guest's ANS namespaces.
    backing: Option<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    /// The restore host's IPSW.
    source: Option<restore::Source>,
    host_nor: Option<nor::HostNor>,
    nor_image: Option<stela_soc_data::nor::Image>,
    nvram: Option<stela_formats::boot_bundle::Nvram>,
    sep_state: Option<stela_soc::machine::sep_state::Location>,
    board_flash: Option<nor::BoardFlash>,
}

/// Open the guest's storage, the restore source and the machine state from
/// the partitions the host devices opened.
fn open_storage(handoff: &'static Handoff, options: &Options, host: &mut OpenedHost) -> Storage {
    let OpenedHost {
        devices:
            HostDevices {
                ref mut services,
                ref mut medium,
                ref host_network,
                ..
            },
        state_in_storage,
        remote_image,
        ..
    } = *host;
    let medium = medium.take();
    // The restore client's signing-server transport (`restore::tss`) and a
    // remote install image share `host_network`; it lives for the whole run.
    let mut services = core::mem::take(services).into_iter();
    let backing = options.storage.as_ref().map(|_| services.next().unwrap());
    let backing = backing.or_else(|| {
        let bytes = options.storage_ram?;
        let disk = stela_drivers::block::Volatile::new("STELARAM", bytes, RAM_STORAGE_LIMIT)
            .unwrap_or_else(|error| {
                output::fatal(format_args!(
                    "the RAM-backed storage could not be made: {error}"
                ))
            });
        output::arguments(format_args!(
            "STORAGE volatile: {bytes:#x} bytes of engine memory back the ANS namespaces, \
             at most {RAM_STORAGE_LIMIT:#x} written; nothing persists\n"
        ));
        Some(alloc::boxed::Box::new(disk)
            as alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>)
    });
    let source = options
        .restore_source
        .as_ref()
        .map(|_| services.next().unwrap());
    // The restore host's IPSW: the study's raw extent, the install medium's
    // pieces, found through FAT32 and joined, or the remote image.
    let source = match (source, medium) {
        (Some(partition), _) => Some(
            restore::inventory(handoff)
                .and_then(|manifest| restore::Source::extent(partition, manifest.source.bytes)),
        ),
        (None, Some(partition)) => Some(restore::inventory(handoff).and_then(|manifest| {
            let path = options.install.as_deref().unwrap_or_default();
            let directory = path
                .rsplit_once('\\')
                .map_or("", |(directory, _)| directory);
            let hashes = restore::inventory_file(handoff, crate::system::INSTALL_PIECES_FILE)
                .ok_or(refuse!("the install medium's piece hashes were not loaded"))?;
            crate::status::begin("Finding the macOS install files on the USB");
            let opened = install::open(partition, &manifest, directory, hashes)?;
            crate::status::done("");
            Ok(restore::Source::new(opened))
        })),
        (None, None) if remote_image => Some(restore::inventory(handoff).and_then(|manifest| {
            let network = match &host_network {
                Some(Ok(network)) => network.clone(),
                Some(Err(reason)) => return Err(reason.clone()),
                None => unreachable!("a remote install opens the host network"),
            };
            Ok(restore::Source::remote(remote::Remote::new(
                network, &manifest,
            )?))
        })),
        (None, None) => None,
    }
    .map(|source| {
        source.unwrap_or_else(|error| {
            output::fatal(format_args!(
                "the restore source could not be opened: {error}"
            ))
        })
    });
    let (backing, nor) = match backing {
        Some(partition) if state_in_storage => {
            let (namespace, state) = nor::split(partition);
            (Some(namespace), Some(state))
        }
        backing => (
            backing,
            options.nor.as_ref().map(|_| services.next().unwrap()),
        ),
    };
    // The host disk's I/O for the ANS model, and only for it, runs outside
    // the devices' lock the model takes its commands under
    // (`storage::stage`); the machine state's waits stay on the disk itself.
    let backing = backing.map(|disk| match options.storage {
        Some(_) => super::storage::stage(disk),
        None => disk,
    });
    let nor = nor.map(|device| {
        crate::status::begin(&format!(
            "Opening the machine state on {}",
            device.identity().model
        ));
        let opened = nor::HostNor::open(device);
        crate::status::done("");
        bundle_repair::nvram(&opened.2.proxy);
        opened
    });
    let (host_nor, nor_image, nvram, sep_state, board_flash) = match nor {
        Some((host_nor, image, nvram, sep_state, board_flash)) => (
            Some(host_nor),
            Some(image),
            Some(nvram),
            Some(sep_state),
            board_flash,
        ),
        None => (None, None, None, None, None),
    };
    Storage {
        backing,
        source,
        host_nor,
        nor_image,
        nvram,
        sep_state,
        board_flash,
    }
}

/// The other host processors, started and parked, and the boot
/// processor's local APIC that signals them.
struct Processors {
    executors: crate::native_smp::executor::Executors,
    sender: crate::apic::Apic,
}

/// Start the other host processors over the guest's RAM, and run the
/// bounded diagnostics that stop on them.
fn start_processors(boot: Boot<'_>, host: &mut OpenedHost) -> Processors {
    let Boot {
        handoff,
        options,
        native_processors,
    } = boot;
    let OpenedHost {
        tsc_base: native_tsc_base,
        ram: ref worker_ram,
        ..
    } = *host;
    crate::status::begin("Starting the other host processors");
    // SAFETY: the physical inventory and loader-owned reservations
    // were validated before any SIPI; the BSP Host is installed, guest entry
    // is excluded, and APs remain parked until canonical worker publication.
    let executors = unsafe {
        crate::native_smp::executor::start(
            handoff,
            native_processors.expect("validated native roster"),
            worker_ram,
            native_tsc_base,
            options.native_bootstrap_check,
        )
    }
    .unwrap_or_else(|error| {
        output::fatal(format_args!("owned native AP bootstrap refused: {error:?}"))
    });
    match executors.failure() {
        Some(failure) => {
            output::arguments(format_args!("NATIVE AP startup failed: {failure:?}\n"));
            if options.native_bootstrap_check || options.native_shared_check {
                output::fatal(format_args!(
                    "owned native AP bootstrap refused: {failure:?}"
                ));
            }
            crate::status::warn(&alloc::format!(
                "{} of {} cores started; macOS runs on {}",
                executors.started(),
                handoff.native_resources.cores,
                executors.started(),
            ));
        }
        None => crate::status::done(""),
    }
    let sender = crate::apic::Apic::new(
        handoff.apic_base,
        layout::physical(handoff.apic_base & crate::apic::BASE_ADDRESS),
    );
    #[cfg(feature = "diagnostics")]
    if options.native_bootstrap_check || options.native_shared_check {
        check_processors(boot, host, &executors, &sender);
    }
    Processors { executors, sender }
}

/// Load the guest's image, model its board and assign its PEs to the host's
/// processors, place the board's devices over its RAM, publish its topology
/// and run it until it stops. The assigned topology borrows the loaded
/// source and the roster for the whole run.
/// Writes what the boot learned of the PC (`crate::host_inventory`) into each
/// of the `gates` StelaGPU gate nodes' `stela-host` property, which the kext
/// publishes as `StelaHost`, with the translator's target and identity and
/// each GPU's VRAM.
#[cfg(feature = "nvk")]
fn describe_host(
    tree: &mut stela_loader::apple_device_tree::DeviceTree,
    gates: usize,
    target: TargetCpu,
) {
    use crate::host_inventory::{entries, number, text};
    text("TranslatorTarget", target.id());
    let id = stela_engine::translations::translator_id();
    text(
        "TranslatorID",
        id.0[..8]
            .iter()
            .map(|byte| alloc::format!("{byte:02x}"))
            .collect::<alloc::string::String>(),
    );
    for index in 0..gates {
        if let Some(bytes) = gpu::client::vram_bytes(index as u32) {
            number(alloc::format!("GPU{index}VRAMBytes"), bytes);
        }
    }
    let record = match stela_loader::stela_gpu::encode_host(&entries()) {
        Ok(record) => record,
        Err(error) => {
            crate::status::warn(&alloc::format!(
                "The PC's description has no place: {error}"
            ));
            return;
        }
    };
    for index in 0..gates {
        if let Err(error) = stela_loader::stela_gpu::set_host(tree, index, record.clone()) {
            crate::status::warn(&alloc::format!(
                "GPU {index}'s PC description has no place: {error}"
            ));
        }
    }
    output::arguments(format_args!(
        "HOST INVENTORY entries={} bytes={}\n",
        entries().len(),
        record.len()
    ));
}

fn run_guest(boot: Boot<'_>, mut host: OpenedHost, mut storage: Storage) -> ! {
    let Boot {
        handoff,
        options,
        native_processors,
    } = boot;
    let target = target(options.parallel);
    let speculation = speculation::probe();
    let boot_image::GuestImage {
        entry,
        x0,
        read_only_region,
        core_locks,
        #[cfg_attr(not(feature = "nvk"), allow(unused_mut))]
        mut hidden_devices,
        live_tree: mut loaded_tree,
        source: loaded_source,
        boot_processor: bundle_boot_processor,
        hidden_clpc,
    } = boot_image::load(
        &mut host.owner.host(),
        handoff,
        options,
        host.built.as_deref(),
        storage.nvram.take(),
    );
    // The APs start before the board is modeled, so that the guest gets one
    // PE for each core that started.
    let Processors { executors, sender } = start_processors(boot, &mut host);
    let Modeled {
        mut board,
        boot_processor,
        full_profile,
        active_workers,
    } = model_board(
        boot,
        loaded_tree.as_ref(),
        bundle_boot_processor,
        target,
        speculation,
        executors.started(),
    );
    let topology_source = source_topology(
        loaded_source.as_ref(),
        loaded_tree.as_ref(),
        board.as_ref(),
        &full_profile,
        active_workers,
    );
    if loaded_tree.is_some() {
        crate::status::done("");
    }
    output::arguments(format_args!(
        "NATIVE ACTIVE workers={active_workers} parked=0 source_PEs={}\n",
        full_profile.processors.len().max(1),
    ));
    if loaded_tree.is_some() {
        crate::status::begin("Setting up the Mac's hardware");
    }
    let roster = topology::AssignedRoster::new(
        native_processors.expect("validated native roster"),
        &executors,
        &host.owner,
    )
    .unwrap_or_else(|error| output::fatal(&error));
    let OpenedHost {
        owner: mut host_owner,
        apic_hz,
        acpi: host_acpi,
        devices:
            HostDevices {
                host_input,
                host_usb,
                host_bluetooth,
                host_network,
                usb_disks,
                guest_network,
                host_disks,
                fdr: mut fdr_device,
                ..
            },
        ram: worker_ram,
        mut built,
        ..
    } = host;
    let Storage {
        backing,
        source,
        host_nor,
        nor_image,
        sep_state,
        board_flash,
        ..
    } = storage;
    let mut bare = host_owner.host();
    #[cfg_attr(not(feature = "nvk"), allow(unused_mut))]
    let mut assigned = assign(boot, &roster, topology_source.as_ref(), &full_profile);
    #[cfg(feature = "nvk")]
    let gpu_gates = assigned
        .as_mut()
        .map(|assigned| {
            let gates = gpu::gate::place(&mut assigned.tree);
            if !gates.is_empty() {
                hidden_devices.extend(stela_loader::devices::hide_agx(&mut assigned.tree));
            }
            gates
        })
        .unwrap_or_default();
    // SAFETY: all raw guest input copies have ended, AP executors are waiting
    // for their guest contexts, and no guest/device DMA producer has entered.
    // The same loader bytes are for every subsequent native CPU and synchronous device writer.
    let canonical_ram = unsafe {
        crate::native_smp::canonical::reserve_for_workers(
            handoff,
            &worker_ram,
            native_processors.expect("validated native roster"),
            &executors,
        )
    }
    .unwrap_or_else(|error| output::fatal(error));
    // The translator compiles for the canonical RAM's target (the BSP's live
    // features under the common worker ceiling), not the board profile's
    // ID-register list in `target`.
    #[cfg(feature = "nvk")]
    if let Some(assigned) = &mut assigned {
        describe_host(&mut assigned.tree, gpu_gates.len(), canonical_ram.target());
    }
    if let Some(assigned) = &assigned {
        board = Some(assigned_board(
            handoff,
            assigned,
            &full_profile,
            boot_processor,
        ));
    }
    // The verified image is placed and its provenance is owned. The
    // remaining device and flash consumers read board firmware by name.
    if let Some(files) = &mut built {
        let firmware = board
            .as_ref()
            .map(board::Board::firmware_files)
            .unwrap_or_default();
        files.retain(|(name, _)| firmware.contains(name));
    }
    let projected_bytes = assigned.as_ref().map(|assigned| assigned.tree.serialize());
    let tree = projected_bytes
        .as_deref()
        .or_else(|| loaded_tree.as_ref().map(|tree| tree.bytes()));
    let boot_processor = board
        .as_ref()
        .and_then(|board| board.boot_processor.as_ref())
        .map_or(0, |boot| boot.cpu);
    mount::clpc_absence(&hidden_clpc, &mut |arguments| output::arguments(arguments));
    admit_board(&mut board, tree, &hidden_devices, nor_image);
    let profile = assigned
        .as_ref()
        .map_or_else(|| full_profile.clone(), |assigned| assigned.profile.clone());
    // The real kernelcache starts SPTM at EL2: it writes TPIDR_EL2 and keeps
    // its per-CPU pointer in TPIDR_GL2. HCR_EL2.E2H is RES1 on this profile
    // (SPTM installs its vectors through VBAR_EL1 before it ORs TGE and E2H
    // into HCR_EL2, which reaches VBAR_EL2 only with E2H already set); the
    // other HCR_EL2 bits reset to 0. Every other image starts there too:
    // the canonical memory owner admits MMU-off execution at EL2 alone,
    // and Stela's test kernel makes itself a
    // VHE host as XNU does.
    let el = 2;
    output::arguments(format_args!(
        "ENTRY pc={entry:#x} x0={x0:#x} el={el} mmu=off profile={} target={} apic_hz={apic_hz} \
         prediction_isolation={} faulting_data_barrier={}\n",
        profile.name,
        canonical_ram.target().id(),
        speculation.contexts,
        speculation.faulting_data,
    ));
    let mut cpu = Cpu::of(profile.chip, entry, Pstate::reset(el));
    cpu.state.x[0] = x0;
    let now = stela_engine::machine::Host::counter(&mut bare);
    let Placed {
        mut devices,
        restore_usb,
        cacheless_memory,
    } = place_devices(
        boot,
        Placement {
            built: built.as_deref(),
            entry,
            now,
            read_only_region,
            backing,
            sep_state,
            board_flash,
            rtc_anchor: host_nor.as_ref().and_then(nor::HostNor::loaded_anchor),
            usb_disks,
            guest_mac: guest_network.as_ref().map(network::GuestNetwork::mac),
            guest_bluetooth: host_bluetooth.is_some(),
        },
        board.as_ref(),
        &core_locks,
        &canonical_ram,
        loaded_source.as_ref(),
        tree,
    );
    #[cfg(feature = "nvk")]
    if let Some(board) = &board {
        for (placement, service) in gpu_gates {
            // The board's AIC took its dies and inputs from this vector too.
            let per_die = u32::from(board.aic.layout.max_irq());
            let irq = stela_contracts::interrupt::AicInput::new(
                (placement.irq / per_die) as u8,
                (placement.irq % per_die) as u16,
            );
            devices
                .add_gpu_gate(placement.physical, irq, service)
                .unwrap_or_else(|error| output::fatal(&error));
        }
    }
    // Device placement has copied the carried firmware. This function
    // never returns, so release the remaining input buffers here.
    drop(built);
    let processor_power = connect_processor_power(
        &mut devices,
        topology_source.as_ref(),
        assigned.as_ref(),
        loaded_source.as_ref(),
        &full_profile,
    );
    let worker_bindings = worker_bindings(assigned.as_ref());
    if tree.is_some() {
        // The CPU profile's name is for the log, not the screen.
        output::arguments(format_args!("BOOT CPU profile {}\n", profile.name));
        crate::status::done("");
    }
    let restore = source.map(|source| {
        prepare_restore(
            boot,
            source,
            restore_usb.unwrap(),
            tree,
            &host_network,
            fdr_device.take(),
        )
    });
    // All source/resource preparation precedes the first publication.
    // A reached write failure is terminal before any guest entry.
    let _published_tree = assigned.as_ref().map(|assigned| {
        publish_assigned(
            assigned,
            loaded_tree.take().expect("retained source tree"),
            &canonical_ram,
            cacheless_memory,
            projected_bytes.as_deref(),
        )
    });
    let boot_progress = boot_progress(loaded_source.as_ref());
    drop(assigned);
    drop(roster);
    start_machine(
        boot,
        MachineParts {
            owner: host_owner,
            executors,
            sender,
            ram: canonical_ram,
            cpu,
            devices,
            profile,
            boot_processor,
            worker_bindings,
            processor_power,
            core_locks,
            _published: _published_tree,
            io: RunIo {
                restore,
                host_input,
                host_usb,
                host_bluetooth,
                guest_network,
                host_nor,
                host_disks,
                host_acpi,
                boot_progress,
            },
        },
    )
}

/// The guest's board as its live DeviceTree describes it, its boot PE and
/// CPU profile, and how many host processors run its PEs.
struct Modeled {
    board: Option<board::Board>,
    boot_processor: usize,
    full_profile: CpuProfile,
    active_workers: usize,
}

/// Model the board of the loaded DeviceTree, whose boot PE must be the one
/// the verified bundle hands off to, for the `started` host processors.
fn model_board(
    boot: Boot<'_>,
    loaded_tree: Option<&bundle::live_tree::LiveTree>,
    bundle_boot_processor: Option<usize>,
    target: TargetCpu,
    speculation: Speculation,
    started: usize,
) -> Modeled {
    let source_tree = loaded_tree.as_ref().map(|tree| tree.bytes());
    if source_tree.is_some() {
        crate::status::begin("Checking the Mac's CPU topology");
    }
    let board = source_tree.map(|tree| {
        board::board(tree).unwrap_or_else(|error| {
            output::fatal(format_args!("the board could not be modeled: {error}"))
        })
    });
    let boot_processor = match board.as_ref() {
        Some(board) => {
            let boot = board
                .boot_processor
                .as_ref()
                .unwrap_or_else(|| output::fatal("the DeviceTree has no running boot PE"));
            if bundle_boot_processor != Some(boot.cpu) {
                output::fatal("the board boot PE differs from the verified bundle handoff");
            }
            output::arguments(format_args!(
                "BOOT PE index={} path={} affinity={:#x} cluster={}\n",
                boot.cpu,
                boot.path,
                boot.affinity,
                char::from(boot.cluster_type),
            ));
            boot.cpu
        }
        None => {
            if bundle_boot_processor.is_some() {
                output::fatal("a selected boot PE requires a live DeviceTree");
            }
            0
        }
    };
    let full_profile = match &board {
        Some(board) => board_profile(board, target, speculation),
        None => test_profile(target, speculation),
    };
    let active_workers = started;
    let source_pes = full_profile.processors.len().max(1);
    let available = boot
        .native_processors
        .expect("validated native roster")
        .logical_count as usize;
    boot.options
        .validate_native_cores(available, active_workers, source_pes)
        .unwrap_or_else(|error| output::fatal(&error));
    Modeled {
        board,
        boot_processor,
        full_profile,
        active_workers,
    }
}

/// Connect the assigned PEs' power to the PMGR resources of the source
/// board they were assigned from.
fn connect_processor_power(
    devices: &mut AppleDevices,
    topology_source: Option<&topology::Source<'_>>,
    assigned: Option<&topology::Prepared<'_>>,
    loaded_source: Option<&bundle::source::Intake>,
    full_profile: &CpuProfile,
) -> Vec<stela_engine::machine::engine::ProcessorBinding> {
    match (topology_source, assigned, loaded_source) {
        (Some(source), Some(assigned), Some(intake)) => {
            let original = stela_contracts::adt::Node::root(intake.original())
                .unwrap_or_else(|| output::fatal("retained original source ADT is malformed"));
            devices
                .connect_processor_power(
                    source.resources.graph(),
                    &original,
                    full_profile,
                    &assigned.plan,
                    source.acc_harvesting(),
                )
                .unwrap_or_else(|error| output::fatal(&error))
        }
        (None, None, None) => Vec::new(),
        _ => output::fatal("assigned processor power lost its retained source owner"),
    }
}

/// Which host worker runs each PE: the assignment's, else the boot
/// processor runs PE 0.
fn worker_bindings(
    assigned: Option<&topology::Prepared<'_>>,
) -> Vec<stela_engine::machine::engine::publication::WorkerBinding> {
    match assigned {
        Some(assigned) => assigned
            .worker_bindings()
            .unwrap_or_else(|error| output::fatal(&error)),
        None => alloc::vec![stela_engine::machine::engine::publication::WorkerBinding {
            pe: 0,
            worker: stela_engine::machine::engine::publication::WorkerId(0),
        }],
    }
}

/// What the machine is made from once the guest's topology is published.
struct MachineParts {
    owner: host::HostOwner,
    executors: crate::native_smp::executor::Executors,
    sender: crate::apic::Apic,
    ram: alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>,
    cpu: Cpu,
    devices: AppleDevices,
    profile: CpuProfile,
    boot_processor: usize,
    worker_bindings: Vec<stela_engine::machine::engine::publication::WorkerBinding>,
    processor_power: Vec<stela_engine::machine::engine::ProcessorBinding>,
    core_locks: stela_engine::machine::ctrr::CoreLocks,
    /// The published DeviceTree, kept for the whole run.
    _published: Option<bundle::live_tree::LiveTree>,
    io: RunIo,
}

/// Make the machine on the host's processors, in parallel when asked, and
/// run it until it stops.
fn start_machine(boot: Boot<'_>, parts: MachineParts) -> ! {
    let Boot {
        handoff,
        options,
        native_processors,
    } = boot;
    let MachineParts {
        owner: mut host_owner,
        executors,
        sender,
        ram: canonical_ram,
        cpu,
        devices,
        profile,
        boot_processor,
        worker_bindings,
        processor_power,
        core_locks,
        _published,
        io,
    } = parts;
    let capture = native_processors.expect("validated native roster");
    let hub = stela_engine::machine::engine::Hub::new(
        profile.processors.len().max(1),
        parallel::Doorbells::new(profile.processors.len().max(1), &worker_bindings, |slot| {
            let selected = *capture.selected.get(slot)? as usize;
            u32::try_from(capture.processors.get(selected)?.apic_id).ok()
        }),
    );
    // Profile observations copy only the owners' last materialized images;
    // they are stale diagnostics, not coherent source-state receipts.
    hub.set_source_observations(options.profile);
    gpu::set_hub(hub.clone(), boot_processor);
    let devices = parallel::Board::new(devices, hub.clone());
    let settings = parallel::Settings {
        traces: options.traces.clone(),
        watches: options.watches.clone(),
        step_timing: options.profile,
    };
    // The loader handle's last RAM copy preceded canonical registration; its
    // borrow has ended. It never executes a guest instruction.
    // SAFETY: started executors, kept source Plan/profile and exact
    // RAM/device owners remain live for the whole non-returning run below.
    let mut machine = unsafe {
        crate::native_smp::canonical::prepare_with_locks(
            crate::native_smp::canonical::Preparation {
                handoff,
                capture: native_processors.expect("validated native roster"),
                executors: &executors,
                sender: &sender,
                owner: &mut host_owner,
                ram: canonical_ram,
                cpu,
                devices,
                profile,
                boot_processor,
                assignments: &worker_bindings,
                processor_power: &processor_power,
            },
            options.native_artifacts,
            Some(core_locks),
        )
    }
    .unwrap_or_else(|error| output::fatal(error));
    if options.parallel {
        // The workers are held for good, and their cores run PEs instead.
        // SAFETY: as for `run_serialized` below.
        if let Err(stop) = unsafe { machine.hold_serialized() } {
            output::fatal(format_args!(
                "the serial workers could not be held: {stop:?}"
            ));
        }
        machine
            .enter_parallel(hub)
            .unwrap_or_else(|error| output::fatal(error));
        output::arguments(format_args!(
            "PARALLEL home count={}\n",
            if machine.count_reads_natively() {
                "native"
            } else {
                "still"
            }
        ));
        translations::share(handoff.heap.bytes as usize / parallel::SHARED_CODE_SHARE);
        parallel::spawn(
            &machine,
            &worker_bindings,
            &executors,
            &sender,
            handoff.tsc_hz,
            &settings,
        )
        .unwrap_or_else(|error| output::fatal(error));
    }
    // The run's driver calls keep the engine's TLB entries; the serialized
    // workers' validations require PCIDE clear.
    let _tagged = options
        .parallel
        .then(storage::TaggedRoots::enable)
        .flatten();
    run_machine(machine, options, handoff, io, |machine, limit| {
        // SAFETY: the same fixed workers and canonical producer set stay
        // kept; an unsupported native service returns without fallback.
        unsafe {
            if options.native_rounds {
                crate::native_smp::canonical::run_slice_for(
                    machine,
                    limit,
                    handoff.tsc_hz / WAIT_POLLS_PER_SECOND,
                )
            } else {
                machine.run_serialized_with_budget(parallel::RunBudget {
                    blocks: limit,
                    host_ticks: Some(gpu::slice(host::COUNTER_HZ / HOME_SLICES_PER_SECOND)),
                })
            }
        }
    })
}

/// Claim the GPUs from probe and start them in the host's Linux environment.
/// A GPU that cannot start is reported, and the Mac boots without it; whether
/// any started is the answer.
fn open_gpus(
    work: &'static kernel_work::KernelWork,
    mmio: Option<&'static crate::native_mmio::Source>,
    remappers: &'static remapping::Remappers,
    handoff: &Handoff,
    firmware: &Option<alloc::sync::Arc<stela_drivers::firmware::Catalog>>,
    adapters: Vec<stela_drivers::gpu::Adapter>,
) -> bool {
    let heap = handoff.heap.physical..handoff.heap.physical + handoff.heap.bytes;
    let guest_ram = handoff.guest[..handoff.guest_count]
        .iter()
        .map(|region| region.physical..region.physical + region.bytes)
        .collect();
    let Some(window) = mmio.and_then(|mmio| mmio.window()) else {
        crate::status::warn("this PC cannot map a GPU's memory, so it is left unused");
        return false;
    };
    let Some(mmio) = mmio else { return false };
    if let Err(error) = work.service_clock().and_then(|clock| clock.epoch()) {
        crate::status::warn(&stela_boot_core::work_status::gpu_without_clock(
            crate::cpu::INVARIANT_TSC.present(),
            &format!("{error}"),
        ));
        return false;
    }
    let tsc_hz = handoff.tsc_hz;
    #[cfg(feature = "nvk")]
    let heap_bytes = heap.end - heap.start;
    crate::status::begin("Starting the GPU");
    let gop = if handoff.screen.is_some() {
        mmio.verified_framebuffer()
    } else {
        Ok(crate::pci_io::ScreenOwner::NoScreen)
    };
    let firmware = firmware.clone();
    // Named on the screen when no GPU starts; the claim takes `adapters`.
    let found = adapters.clone();
    let claiming = host::rdtsc();
    let claimed = work.call_owned(60_000_000, move || {
        let services = services::Services::new(mmio, remappers, None, heap, tsc_hz, Vec::new())
            .with_guest_ram(guest_ram);
        let random = stela_drivers::probe::NativeServices::entropy(&services)
            .ok()
            .and_then(|entropy| stela_linux::random::Seeded::from_entropy(&*entropy).ok());
        let (claimed, refused) = random.as_ref().map_or_else(Default::default, |_| {
            gpu::claim_all(&services, &adapters, gop, firmware.as_ref())
        });
        (claimed, refused, random)
    });
    let (claimed, random) = match claimed {
        Ok((claimed, _, Some(random))) if !claimed.is_empty() => (claimed, random),
        Ok((_, refused, _)) => {
            // Each GPU that did not start, and why, or that there is none.
            use stela_boot_core::gpu_status;
            match refused.split_first() {
                None => crate::status::warn(&gpu_status::none(&found)),
                Some((first, rest)) => {
                    crate::status::warn(&gpu_status::refused(
                        &first.address,
                        first.device,
                        first.reason,
                    ));
                    for gpu in rest {
                        crate::status::info(&gpu_status::refused(
                            &gpu.address,
                            gpu.device,
                            gpu.reason,
                        ));
                    }
                }
            }
            // The display adapters Stela's GPU driver does not run, and why.
            for adapter in found
                .iter()
                .filter(|adapter| adapter.vendor != stela_drivers::gpu::Vendor::Nvidia)
            {
                crate::status::info(&gpu_status::unused(adapter));
            }
            return false;
        }
        Err(error) => {
            crate::status::warn(&format!(
                "the GPU could not be claimed: {error}. This is a Stela bug; the boot log on \
                 this USB holds the details"
            ));
            return false;
        }
    };
    let machine = gpu::Machine {
        work,
        window,
        random,
        tsc_hz,
        wall: handoff.wall,
        #[cfg(feature = "nvk")]
        heap_bytes,
    };
    // Store the claim's lines before Linux starts, so the USB keeps them
    // even when the start never returns.
    output::flush_log();
    let opening = host::rdtsc();
    // A stop inside the start stores the USB's log from there.
    let opened = storage::in_gpu_start(|| gpu::open(machine, claimed));
    // `GPU START claim_ms=… linux_ms=… probe_ms=… total_ms=…`: claiming the
    // functions, starting Linux, and nouveau's probe, which boots the GSP
    // and binds DRM.
    let ms = |ticks: u64| ticks / (tsc_hz / 1000).max(1);
    let (linux, probe) = gpu::start_times();
    output::arguments(format_args!(
        "GPU START claim_ms={} linux_ms={} probe_ms={} total_ms={}\n",
        ms(opening.wrapping_sub(claiming)),
        ms(linux),
        ms(probe),
        ms(host::rdtsc().wrapping_sub(claiming)),
    ));
    match opened {
        Ok(()) => {
            crate::status::done("started");
            true
        }
        Err(error) => {
            crate::status::warn(&stela_boot_core::gpu_status::failed(
                &found,
                &format!("{error}"),
            ));
            false
        }
    }
}

/// What the board's devices are placed from besides the board itself.
struct Placement<'a> {
    built: Option<&'a [(alloc::string::String, Vec<u8>)]>,
    entry: u64,
    now: u64,
    read_only_region: Option<(u64, u64)>,
    backing: Option<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    sep_state: Option<stela_soc::machine::sep_state::Location>,
    board_flash: Option<nor::BoardFlash>,
    rtc_anchor: Option<stela_soc::machine::RtcAnchor>,
    usb_disks: Vec<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    guest_mac: Option<stela_drivers::net::Mac>,
    guest_bluetooth: bool,
}

/// The placed devices, the USB port a restore drives, and whether guest
/// memory is placed without the translation cache.
struct Placed {
    devices: AppleDevices,
    restore_usb: Option<stela_soc::machine::UsbId>,
    cacheless_memory: bool,
}

/// Place the board's devices with their firmware, scanout, clocks and
/// storage, and bind the native PMP when guest memory is cacheless.
fn place_devices(
    boot: Boot<'_>,
    placement: Placement<'_>,
    board: Option<&board::Board>,
    core_locks: &stela_engine::machine::ctrr::CoreLocks,
    canonical_ram: &alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>,
    loaded_source: Option<&bundle::source::Intake>,
    tree: Option<&[u8]>,
) -> Placed {
    let Boot {
        handoff,
        options,
        native_processors,
    } = boot;
    let Placement {
        built,
        entry,
        now,
        mut board_flash,
        ..
    } = placement;
    let reset = options.bundle.then_some(entry);
    // Board device firmware images travel in the bundle by guest file name;
    // a bundle built from an installed disk finds them in the board's flash.
    let files = &handoff.files[..handoff.file_count];
    let carried = |name: &str| -> Option<&[u8]> {
        match built {
            Some(built) => built
                .iter()
                .find(|(file, _)| file == name)
                .map(|(_, bytes)| bytes.as_slice()),
            None => files
                .iter()
                .find(|file| file.name() == name)
                .map(bundle::contents),
        }
    };
    let flash = board_flash.as_ref();
    let firmware = |name: &str| -> Option<&[u8]> {
        carried(name).or_else(|| flash.and_then(|flash| flash.image(name)))
    };
    let firmware_scanout = || {
        handoff
            .screen
            .filter(|screen| screen.available())
            .map(|screen| {
                let screen = screen.relocated(layout::physical(screen.base as u64) as usize);
                alloc::boxed::Box::new(display::GopScanout::new(screen, options.scanout_mirror))
                    as alloc::boxed::Box<dyn stela_soc::machine::Scanout>
            })
    };
    // The firmware's scanout keeps its newest frame itself; a GPU's is
    // wrapped to keep one for `--scanout-mirror`. With neither a GPU's
    // display nor the firmware's screen, which a bound GPU's reset ended, the
    // guest keeps that screen's mode and a GPU's display found later shows it.
    #[cfg(feature = "nvk")]
    let mirrored = |scanout: alloc::boxed::Box<dyn stela_soc::machine::Scanout>| match options
        .scanout_mirror
    {
        true => alloc::boxed::Box::new(display::MirroredScanout::new(scanout)) as _,
        false => scanout,
    };
    #[cfg(feature = "nvk")]
    let scanout = gpu::display::scanout()
        .map(mirrored)
        .or_else(firmware_scanout)
        .or_else(|| gpu::display::awaited(handoff.screen?).map(mirrored));
    #[cfg(not(feature = "nvk"))]
    let scanout = firmware_scanout();
    let programmed = mount::memory_locks(placement.read_only_region, core_locks);
    let memory = board.and_then(|board| memory_resources(board, canonical_ram, programmed));
    let cacheless_memory = memory.is_some();
    let (mut devices, restore_usb) = match devices(
        board,
        mount::Inputs {
            programmed,
            reset,
            now,
            wall: handoff.wall.map(|wall| WallClock {
                // The firmware's time, carried forward to the counter's `now`.
                unix_seconds: wall.unix_seconds_at(host::rdtsc(), handoff.tsc_hz),
                counter: now,
            }),
            rtc_anchor: placement.rtc_anchor,
            backing: placement.backing,
            sep_state: placement.sep_state,
            sep_observe: options.sep_observe,
            // The install's first recoveryOS policy nonce is the one its
            // staged LocalPolicy answer binds.
            sep_recovery_policy_nonce: options
                .install
                .is_some()
                .then(|| restore::staged_answers(handoff))
                .flatten()
                .and_then(|set| set.policy_nonce),
            entropy: alloc::boxed::Box::new(bundle::entropy),
            restore: options.restore || options.install.is_some(),
            input: options.input,
            usb: stela_soc::machine::UsbCables {
                disks: placement.usb_disks,
                network: placement.guest_mac,
                bluetooth: placement.guest_bluetooth,
            },
            scanout,
            firmware: &firmware,
            build: loaded_source.map(|source| {
                stela_formats::builds::find(&source.selected().product_build)
                    .unwrap_or_else(|error| output::fatal(error))
            }),
        },
        memory,
    ) {
        Ok(devices) => devices,
        Err(error) => output::fatal(format_args!(
            "the board devices could not be placed: {error}"
        )),
    };
    if let (Some(flash), Some(board)) = (&mut board_flash, board) {
        let images: Vec<_> = board
            .firmware_files()
            .into_iter()
            .filter_map(|name| {
                let bytes = carried(&name)?;
                Some((name, bytes))
            })
            .collect();
        flash.keep(&images);
    }
    if cacheless_memory
        && let (Some(board), Some(intake), Some(tree)) = (board, loaded_source, tree)
    {
        bind_native_pmp(
            &mut devices,
            board,
            intake,
            tree,
            native_processors,
            canonical_ram,
        );
    }
    Placed {
        devices,
        restore_usb,
        cacheless_memory,
    }
}

/// Publish the assigned topology over the loaded DeviceTree in guest
/// memory. A write failure is terminal before any guest entry.
fn publish_assigned(
    assigned: &topology::Prepared<'_>,
    live: bundle::live_tree::LiveTree,
    canonical_ram: &alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>,
    cacheless_memory: bool,
    projected: Option<&[u8]>,
) -> bundle::live_tree::LiveTree {
    // SAFETY: cold owner, parked APs, no frame loans or device DMA;
    // every earlier raw guest copy has completed.
    let mut memory = unsafe { bundle::live_tree::registered::ColdRam::new(canonical_ram) };
    let published = if cacheless_memory {
        live.publish_cacheless(&mut memory, &assigned.tree)
    } else {
        live.publish(&mut memory, &assigned.tree)
    }
    .unwrap_or_else(|error| output::fatal(&error));
    if Some(published.bytes()) != projected {
        output::fatal("published assigned topology differs from the prepared device graph");
    }
    published
}

/// Track boot progress through the source manifest's regions.
fn boot_progress(loaded_source: Option<&bundle::source::Intake>) -> progress::BootProgress {
    loaded_source.map_or_else(progress::BootProgress::default, |source| {
        progress::BootProgress::new(
            source
                .manifest()
                .regions
                .iter()
                .map(|region| (region.name.as_str(), region.guest_pa, region.memory_bytes)),
        )
    })
}

/// Assign the source board's projected PEs to the entire started roster.
fn assign<'s>(
    boot: Boot<'_>,
    roster: &'s topology::AssignedRoster<'_>,
    topology_source: Option<&'s topology::Source<'_>>,
    full_profile: &CpuProfile,
) -> Option<topology::Prepared<'s>> {
    let Boot {
        options,
        native_processors,
        ..
    } = boot;
    let mut assigned = topology_source.map(|source| {
        source
            .prepare_on(
                roster,
                full_profile.clone(),
                native_processors.expect("validated native roster"),
            )
            .unwrap_or_else(|error| output::fatal(&error))
    });
    if options.development_restore || options.development_board {
        let assigned = assigned.as_mut().unwrap_or_else(|| {
            output::fatal("a development board needs a selected original Apple board")
        });
        let target = development_hardware_model(&mut assigned.tree)
            .unwrap_or_else(|error| output::fatal(&error));
        if options.development_restore {
            output::arguments(format_args!(
                "DEVELOPMENT RESTORE hw-model={target} FDR failures remain logged and unprovisioned\n"
            ));
        } else {
            output::arguments(format_args!(
                "DEVELOPMENT BOARD hw-model={target}, as the disk's system was installed\n"
            ));
        }
    }
    assigned
}

/// `--native-bootstrap-check` and `--native-shared-check`: each started
/// processor's doorbell, descriptors and host state, the MMIO aperture's
/// shootdown and the shared-kernel fixture. Both stop the engine.
#[cfg(feature = "diagnostics")]
fn check_processors(
    boot: Boot<'_>,
    host: &mut OpenedHost,
    executors: &crate::native_smp::executor::Executors,
    sender: &crate::apic::Apic,
) -> ! {
    let Boot {
        handoff,
        options,
        native_processors,
    } = boot;
    let OpenedHost {
        owner: ref mut host_owner,
        mmio: native_mmio,
        ram: ref worker_ram,
        ..
    } = *host;
    for processor in executors.processors() {
        // SAFETY: BSP owns its ICR, physical AP/private descriptors ready.
        let doorbells = unsafe { processor.check_doorbell(sender, handoff.tsc_hz) }
            .unwrap_or_else(|_| output::fatal("native AP doorbell did not complete"));
        let observed = processor.observation();
        let descriptors = processor.descriptors();
        output::arguments(format_args!(
            "NATIVE DESCRIPTORS slot={} rsp={:#x} gdt={:#x} idt={:#x} tr={:#x} gs={:#x} retained=boot-lifetime\n",
            processor.slot(),
            descriptors.rsp,
            descriptors.gdt,
            descriptors.idt,
            descriptors.task,
            descriptors.gs,
        ));
        output::arguments(format_args!(
            "NATIVE HOST slot={} apic={:#x} cr3={:#x} cr0={:#x} cr4={:#x} efer={:#x} pat={:#x} doorbells={}\n",
            processor.slot(),
            processor.apic_id(),
            observed.cr3,
            observed.cr0,
            observed.cr4,
            observed.efer,
            observed.pat,
            doorbells,
        ));
    }
    if handoff.native_resources.cores == 1 && executors.processors().is_empty() {
        output::arguments(format_args!(
            "CHECK native-shootdown SKIP reason=no-remote-processor\n"
        ));
    } else if let Some(mmio) = &native_mmio {
        let waiter = executors
            .processors()
            .first()
            .unwrap_or_else(|| output::fatal("the shootdown check requires a started AP"));
        // SAFETY: both processors keep the live loader root. This diagnostic
        // owns the host's devices and only reads bus 0's first function.
        let generation = unsafe {
            waiter.check_driver_wait(handoff.root, sender, handoff.tsc_hz, || {
                mmio.check_shootdown()
            })
        }
        .unwrap_or_else(|error| output::fatal(error))
        .unwrap_or_else(|error| {
            output::fatal(format_args!(
                "the MMIO aperture's shootdown failed: {error:?}"
            ))
        });
        output::arguments(format_args!(
            "CHECK native-shootdown PASS generation={generation} processors={} waiter=driver-lock-if0\n",
            executors.processors().len()
        ));
        // SAFETY: the BSP still owns the diagnostic's configuration space.
        // It reads the mapped page before handing the sole lease to the AP.
        let (scope, identity) = unsafe { mmio.check_shootdown_mapping() }
            .unwrap_or_else(|_| output::fatal("AP shootdown diagnostic mapping refused"));
        let bsp = host::shootdown()
            .0
            .unwrap_or_else(|| output::fatal("the BSP has no shootdown slot"));
        // SAFETY: the sole read completed on the BSP and no consumer remains.
        unsafe {
            scope.retire_remotely(|mapping| {
                waiter.check_aperture_retire(mapping, sender, handoff.tsc_hz)
            })
        }
        .unwrap_or_else(|error| output::fatal(error));
        let processors = &crate::native_mmio::PROCESSORS;
        let retired = processors.requested();
        if identity == u32::MAX || retired <= generation || processors.acknowledged(bsp) < retired {
            output::fatal("AP shootdown did not receive the BSP's acknowledgement");
        }
        output::arguments(format_args!(
            "CHECK native-shootdown AP-origin PASS generation={retired} bsp=acknowledged waiter=mailbox-if0\n"
        ));
    }
    if options.native_shared_check {
        // SAFETY: the bounded fixture owns all source RAM and frame
        // loans; all AP returns are acquired before any borrowed release.
        unsafe {
            crate::native_smp::kernel_check::check(
                handoff,
                native_processors.expect("validated native roster"),
                executors,
                host_owner,
                sender,
                worker_ram,
            )
        }
        .unwrap_or_else(|error| output::fatal(error));
        output::terminate(0x10);
    }
    output::arguments(format_args!(
        "CHECK native-bootstrap PASS physical_cores={} owned_APs={}; guest SMP execution unqualified\n",
        handoff.native_resources.cores,
        executors.processors().len(),
    ));
    output::terminate(0x10);
}

/// The qualified native topology of the selected source board, which
/// `active_workers` PEs run: none for an authored single-PE image.
fn source_topology<'a>(
    loaded_source: Option<&'a bundle::source::Intake>,
    loaded_tree: Option<&bundle::live_tree::LiveTree>,
    board: Option<&board::Board>,
    full_profile: &CpuProfile,
    active_workers: usize,
) -> Option<topology::Source<'a>> {
    match (loaded_source, loaded_tree, board) {
        (Some(intake), Some(live), Some(source_board)) => {
            let boot = source_board
                .boot_processor
                .as_ref()
                .expect("validated source boot PE");
            let identity = intake
                .topology_identity(full_profile.chip)
                .unwrap_or_else(|error| output::fatal(&error));
            let prepared = match identity.profile.soc {
                0x8112 => topology::Source::t8112_assigned(
                    intake.original(),
                    live,
                    identity,
                    boot.affinity,
                    active_workers,
                ),
                0x6020..=0x6022 => topology::Source::t6020_assigned(
                    intake.original(),
                    live,
                    identity,
                    boot.affinity,
                    active_workers,
                ),
                0x6000 => topology::Source::t6000_assigned(
                    intake.original(),
                    live,
                    identity,
                    boot.affinity,
                    active_workers,
                ),
                _ => Err(refuse!(
                    "native assigned topology for this source family is not qualified"
                )),
            };
            Some(prepared.unwrap_or_else(|error| output::fatal(&error)))
        }
        (None, None, None) if active_workers == 1 => None,
        (None, None, None) => {
            output::fatal("an authored multi-PE image requires an explicit qualified topology")
        }
        _ => output::fatal("selected bundle source, live tree and board must be retained together"),
    }
}

/// The board of the assigned topology's DeviceTree, whose boot PE must be
/// the one the host's boot processor runs and the source handed off.
fn assigned_board(
    handoff: &Handoff,
    assigned: &topology::Prepared<'_>,
    full_profile: &CpuProfile,
    boot_processor: usize,
) -> board::Board {
    let candidate = assigned.tree.serialize();
    let parsed = board::board(&candidate).unwrap_or_else(|error| output::fatal(&error));
    let boot = parsed
        .boot_processor
        .as_ref()
        .unwrap_or_else(|| output::fatal("assigned topology has no running boot PE"));
    let join = assigned
        .join(boot.affinity)
        .unwrap_or_else(|error| output::fatal(&error));
    if boot.cpu != join.logical
        || join.worker != 0
        || assigned
            .plan
            .pes()
            .iter()
            .find(|pe| pe.boot)
            .map(|pe| pe.logical)
            != Some(join.logical)
    {
        output::fatal("prepared boot PE differs from the actual assigned BSP");
    }
    if full_profile.processors.get(boot_processor) != assigned.profile.processors.get(boot.cpu) {
        output::fatal("assigned boot identity differs from the original source handoff");
    }
    output::arguments(format_args!(
        "NATIVE ASSIGNMENT original_boot={} assigned_boot={} physical_cores={} PEs={} boot_path={} removed={} pmgr_harvesting={}\n",
        boot_processor,
        boot.cpu,
        handoff.native_resources.cores,
        assigned.joins.len(),
        assigned.receipt.boot_path,
        assigned.receipt.removed.len(),
        match assigned.receipt.harvesting {
            Some(mode) => alloc::format!("{mode}"),
            None => "source".into(),
        }
    ));
    parsed
}

/// Apply the PMGR's resource admission from the DeviceTree the guest boots
/// with, and give the modeled NOR `--nor`'s image.
fn admit_board(
    board: &mut Option<board::Board>,
    tree: Option<&[u8]>,
    hidden_devices: &[String],
    nor_image: Option<stela_soc_data::nor::Image>,
) {
    if let (Some(board), Some(tree)) = (board.as_mut(), tree)
        && let Some(states) = board.power_states.as_mut()
    {
        let unavailable = states
            .apply_admission(tree, hidden_devices)
            .unwrap_or_else(|error| {
                output::fatal(format_args!(
                    "the PMGR resource admission evidence is inconsistent: {error}"
                ))
            });
        output::arguments(format_args!(
            "DEVICES pmgr unavailable-resources={unavailable}\n"
        ));
    }
    if let Some(image) = nor_image {
        match board.as_mut().and_then(|board| board.spi_nor.as_mut()) {
            Some(spi) => spi.image = image,
            None => output::fatal("--nor was supplied but this DeviceTree exposes no modeled NOR"),
        }
    }
}

/// The cold plan of the memory cache's controls for a cacheless frontend:
/// the board's direct memory cache, else its memory cache, else none.
fn memory_resources(
    board: &board::Board,
    canonical_ram: &alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>,
    programmed: stela_soc_data::amcc::Programmed,
) -> Option<MemoryResources> {
    if let Some(direct) = board.direct_memory_cache.as_ref() {
        use stela_soc::machine::memory_resources::t6000_cold;
        // Equal nonzero software quota weights; these are policy for a
        // new cacheless frontend, never a measured Apple reset image.
        let configuration =
            t6000_cold::Configuration::zeroed_control_image(direct.apertures.len(), [1; 32])
                .unwrap_or_else(|error| output::fatal(&error));
        let plan =
            t6000_cold::Plan::prepare_ap_frontend(board, canonical_ram, programmed, &configuration)
                .unwrap_or_else(|error| output::fatal(&error));
        Some(MemoryResources::T6000Cold(plan))
    } else if let Some(descriptor) = board.t6020_memory.as_ref() {
        use stela_soc::machine::memory_resources::t6020::cold;
        use stela_soc::mcc::t6020::Configuration;
        let controller = board
            .memory_controller
            .as_ref()
            .unwrap_or_else(|| output::fatal("T602x memory cache has no AMCC descriptor"));
        let power = board
            .power_states
            .as_ref()
            .unwrap_or_else(|| output::fatal("T602x memory cache has no PMGR power states"));
        let layout = descriptor
            .layout()
            .unwrap_or_else(|error| output::fatal(&error));
        // Cleared software controls: policy for a new cacheless frontend,
        // never a measured Apple reset image.
        let configuration = Configuration::clear_software_controls(layout);
        // A refused plan leaves the controller unmodeled, reported once, as
        // the board was before this owner existed.
        match cold::Plan::prepare_ap_frontend(
            controller,
            descriptor,
            power,
            canonical_ram,
            programmed,
            &configuration,
        ) {
            Ok(plan) => Some(MemoryResources::T6020Cold(plan)),
            Err(error) => {
                output::arguments(format_args!("UNMODELED T602x memory cache: {error}\n"));
                None
            }
        }
    } else if board.memory_cache.is_some() {
        use stela_soc::machine::{
            memory_resources::t8112::cache::ZeroWayConfiguration, t8112_cold,
        };
        let configuration = t8112_cold::Configuration::clear_software_controls(
            ZeroWayConfiguration::DisableArgument,
        )
        .unwrap_or_else(|error| output::fatal(&error));
        let plan =
            t8112_cold::Plan::prepare_ap_frontend(board, canonical_ram, programmed, &configuration)
                .unwrap_or_else(|error| output::fatal(&error));
        Some(MemoryResources::T8112Cold(plan))
    } else {
        None
    }
}

/// Bind the native PMP services of a cacheless frontend to the canonical
/// RAM, unless the board's PMP system endpoint is unmodeled.
fn bind_native_pmp(
    devices: &mut AppleDevices,
    board: &board::Board,
    intake: &bundle::source::Intake,
    tree: &[u8],
    native_processors: Option<&'static crate::native_smp::firmware::Capture>,
    canonical_ram: &alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>,
) {
    let refusal = AppleDevices::native_pmp_system_endpoint_refusal(board)
        .unwrap_or_else(|error| output::fatal(&error));
    if let Some(reason) = refusal {
        output::arguments(format_args!("UNMODELED {reason}\n"));
    } else {
        use stela_soc::machine::{ExplicitRuntimeConfiguration, NativePmpClock, SourceProvenance};
        let pmp = board
            .pmp
            .as_ref()
            .unwrap_or_else(|| output::fatal("native memory's PMP descriptor is missing"));
        let source = SourceProvenance::from_trees(intake.original(), tree, pmp)
            .unwrap_or_else(|error| output::fatal(&error));
        let observation = native_processors
            .expect("validated native roster")
            .snapshots[0]
            .get(1, 0)
            .unwrap_or_else(|| {
                output::fatal("native PMP has no actual admitted CPUID leaf1 observation")
            });
        let configuration = ExplicitRuntimeConfiguration::from_observed_cpuid(
            source,
            observation.ebx,
            observation.edx,
        )
        .unwrap_or_else(|error| output::fatal(&error));
        // This is the native Stela Host's configured counter source,
        // with an explicit software offset0, not an Apple frequency observation.
        let clock = NativePmpClock::new(host::fresh_counter, host::COUNTER_HZ as u32, 0)
            .unwrap_or_else(|error| output::fatal(error));
        // SAFETY: cold before first guest/frame publication; same canonical
        // RAM, parked real APs and exclusive closed devices. Every later
        // transfer runs under held/fenced canonical worker sessions.
        unsafe {
            devices.bind_native_pmp_services(
                board,
                alloc::sync::Arc::clone(canonical_ram),
                configuration,
                clock,
            )
        }
        .unwrap_or_else(|error| output::fatal(&error));
    }
}

/// The install's way to Apple's signing service: the answers setup fetched
/// for its requests, then the host network, or the reason there is none.
fn signing_transport(
    handoff: &Handoff,
    host_network: &Option<Result<network::Shared>>,
) -> alloc::boxed::Box<dyn stela_runtime::restore::tss::Transport> {
    let inner: alloc::boxed::Box<dyn stela_runtime::restore::tss::Transport> = match host_network {
        Some(Ok(network)) => alloc::boxed::Box::new(network::Signing::new(network.clone())),
        Some(Err(reason)) => alloc::boxed::Box::new(network::Offline(reason.clone())),
        None => alloc::boxed::Box::new(network::Offline(refuse!(
            "the install opened no network: the USB holds Apple's answers to its signing requests, and this request is not one of them"
        ))),
    };
    match restore::staged_answers(handoff) {
        Some(set) => alloc::boxed::Box::new(stela_runtime::restore::tss::staged::Staging::new(
            set,
            inner,
            alloc::boxed::Box::new(
                |kind: stela_runtime::restore::tss::staged::Kind, sha256: &str| {
                    output::arguments(format_args!(
                        "TSS STAGED kind={} sha256={sha256}\n",
                        kind.name()
                    ));
                },
            ),
        )),
        None => inner,
    }
}

/// The restore the guest's installer runs against `source`: the signing
/// service and guest-facing proxy an install uses, and the restore's mode.
fn prepare_restore(
    boot: Boot<'_>,
    source: restore::Source,
    usb: stela_soc::machine::UsbId,
    tree: Option<&[u8]>,
    host_network: &Option<Result<network::Shared>>,
    fdr_storage: Option<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
) -> restore::Experiment {
    let Boot {
        handoff, options, ..
    } = boot;
    let boot_ecid = tree
        .and_then(stela_contracts::adt::Node::root)
        .and_then(|root| root.child("chosen"))
        .and_then(|chosen| chosen.property("unique-chip-id"))
        .and_then(|bytes| <[u8; 8]>::try_from(bytes).ok())
        .map(u64::from_le_bytes)
        .filter(|&ecid| ecid != 0);
    // An install signs with Apple's signing service (TSS), for the machine
    // whose ECID the bundle names (`/chosen unique-chip-id`).
    let signer = options.install.is_some().then(|| {
        let ecid = boot_ecid.unwrap_or_else(|| {
            output::fatal(
                "install: the bundle names no machine ECID; on another computer, plug in this \
                 USB and run `stela repair`",
            )
        });
        stela_runtime::restore::tss::Signer {
            ecid,
            // The restore takes the boot ROM's and the SEP ROM's nonces
            // from the ticket the bundle boots under (`Answers::boot_with`).
            nonces: stela_runtime::restore::tss::Nonces {
                ap: [0; 32],
                sep: [0; 20],
            },
            transport: signing_transport(handoff, host_network),
        }
    });
    // Development restores keep the original DEV-board FDR failure
    // policy without exposing Apple's FDR service. TSS uses the host
    // signer directly; the image reader also owns its own network access
    // independently of this guest reverse proxy.
    let proxy_network = (options.install.is_some() && !options.development_restore)
        .then(|| {
            host_network
                .as_ref()
                .and_then(|network| network.as_ref().ok())
                .cloned()
        })
        .flatten();
    output::arguments(format_args!(
        "RESTORE PROXY wiring host={} ready={} enabled={}\n",
        host_network.is_some(),
        matches!(host_network, Some(Ok(_))),
        proxy_network.is_some()
    ));
    let mode = match (
        &options.install,
        options.update_restore,
        options.development_restore,
    ) {
        (Some(_), false, true) => restore::Mode::DevelopmentInstall,
        (Some(_), false, false) => restore::Mode::Install,
        (Some(_), true, false) => restore::Mode::Update,
        (None, _, false) => restore::Mode::Study,
        _ => unreachable!("validated restore-mode combination"),
    };
    restore::Experiment::new(
        source,
        usb,
        handoff,
        mode,
        options.restore_identity,
        match options.verify_on_read {
            true => Verify::OnRead,
            false => Verify::Full,
        },
        restore::Services {
            signer,
            fdr_storage,
            network: proxy_network,
            signing_network: matches!(host_network, Some(Ok(_))),
        },
    )
    .unwrap_or_else(|error| output::fatal(format_args!("restore preparation failed: {error}")))
}

/// The boot processor's installed host, with what the later phases need.
struct InstalledHost {
    owner: host::HostOwner,
    apic_hz: u64,
    /// The TSC origin every physical executor shares.
    tsc_base: u64,
}

/// Install the boot processor's host state over `ram`, and run the bounded
/// diagnostics that stop on it.
fn install_host(handoff: &'static Handoff, options: &Options, ram: &[RamRange]) -> InstalledHost {
    #[cfg(not(feature = "diagnostics"))]
    let _ = options;
    let cores = handoff.native_resources.cores as usize;
    let partition = crate::native_smp::resources::partition(
        cores,
        cores,
        0,
        handoff.code.bytes,
        handoff.pool.bytes,
    )
    .unwrap_or_else(|error| output::fatal(error));
    let native_tsc_base = host::rdtsc();
    let config = Config {
        ram: ram.to_vec(),
        pool_physical: handoff.pool.physical,
        pool_bytes: partition.pool_bytes,
        window_root: handoff.root,
        loader_tables: &handoff.native_loader_tables,
        code_virtual: layout::code(handoff.code.bytes),
        code_physical: handoff.code.physical,
        code_bytes: partition.code_bytes,
        code_parts: handoff.native_resources.code_parts,
        window: (partition.window_bytes != 0).then(|| {
            (
                layout::code(handoff.code.bytes) + partition.window_offset,
                handoff.code.physical + partition.window_offset,
                partition.window_bytes,
            )
        }),
        tsc_base: native_tsc_base,
        tsc_hz: handoff.tsc_hz,
        apic_base: handoff.apic_base,
        placement: handoff.placement,
        bounce: Some((layout::bounce(), handoff.bounce)),
    };
    crate::status::begin("Installing the host processor state");
    // SAFETY: once on the BSP, with interrupts off and owned native resources. Keep the
    // processor-local state through every native call and handler.
    let host_owner = unsafe { host::install(config) }.unwrap_or_else(|error| output::fatal(error));
    #[cfg(feature = "diagnostics")]
    let mut host_owner = host_owner;
    #[cfg(feature = "diagnostics")]
    if options.native_bootstrap_check {
        host::fp_diagnostic::check(&mut host_owner.host())
            .unwrap_or_else(|error| output::fatal(error));
    }
    let apic_hz = host::statistics().apic_hz;
    crate::status::done("");
    InstalledHost {
        owner: host_owner,
        apic_hz,
        tsc_base: native_tsc_base,
    }
}

/// The firmware's ACPI namespace and the owned MMIO permission, each `None`
/// when the host does not check, which is reported. The device interrupt
/// registry is attached here too, or its refusal reported.
struct Platform {
    acpi: Option<&'static aml::HostAcpi>,
    mmio: Option<&'static crate::native_mmio::Source>,
    remappers: &'static remapping::Remappers,
    /// Affine runtime/source remain held through paused/quarantined frames.
    /// Its producer services require their own complete admission graph.
    /// Without it, the screen's reason why.
    work: core::result::Result<&'static kernel_work::KernelWork, alloc::string::String>,
}

/// # Safety
/// The post-EBS diagnostic exclusively owns the conventional HDA controller
/// and entire original HPET, including firmware/SMM, reset and comparator
/// exclusion. It runs on the installed BSP before APs or other drivers start.
#[cfg(feature = "diagnostics")]
unsafe fn check_audio(
    handoff: &'static Handoff,
    owner: &mut host::HostOwner,
    source: &crate::native_mmio::Source,
    mode: stela_boot_core::options::AudioCheck,
) -> Result<()> {
    use crate::native_clock::Prepared;
    let audio = handoff
        .storage
        .handoff()
        .ok_or(refuse!("the loader found no host device handoff"))?;
    let identity = owner
        .root_identity()
        .map_err(|error| refuse!("audio root identity: {error}", ?error))?;
    // SAFETY: original loader root on the installed BSP. The audio handoff's
    // low ECAM, BAR and DMA addresses are usable only while this guard is held.
    let _root = unsafe { storage::RestoreRoot::enter(handoff.root) };
    // SAFETY: same real source and CPU, with the explicit complete controller
    // handoff. No AP, other driver or outside HPET writer can enter here.
    let hpet = unsafe { source.take_hpet(identity) }
        .map_err(|error| refuse!("audio HPET handoff: {error}", ?error))?;
    let prepared = Prepared::exclusive_native_hpet(hpet)
        .map_err(|error| refuse!("audio HPET counter: {error}", ?error))?;
    let counter = prepared
        .hpet_counter()
        .ok_or(refuse!("audio HPET counter unavailable"))?
        .clone();
    let result = prepared
        .pci_minimum()
        .map_err(|error| refuse!("audio HPET minimum: {error}", ?error))
        .and_then(|minimum| {
            // SAFETY: the caller supplied the complete HDA handoff and the
            // loader root still maps its exclusive ECAM/BARs and DMA tail.
            unsafe { crate::audio_check::run(&audio, mode, minimum) }
        });
    drop(prepared);
    let retired = (|| {
        let mut counter = alloc::sync::Arc::try_unwrap(counter)
            .map_err(|_| refuse!("audio retained an HPET timing user"))?;
        // SAFETY: the only timer was scoped to the diagnostic, and unique
        // counter ownership proves no retained provider/reader remains.
        unsafe { counter.retire() }
            .map_err(|error| refuse!("audio HPET retirement: {error}", ?error))?;
        drop(counter);
        source
            .join_hpet()
            .map_err(|error| refuse!("audio HPET join: {error}", ?error))
    })();
    match (result, retired) {
        (Err(error), Err(retire)) => Err(refuse!("{error}; also {retire}", %error, %retire)),
        (Err(error), _) | (_, Err(error)) => Err(error),
        (Ok(()), Ok(())) => {
            output::arguments(format_args!("AUDIO CHECK passed\n"));
            Ok(())
        }
    }
}

/// The kernel work clock: the invariant TSC, which reads fastest and needs
/// no controller, else the HPET when this boot retained the whole block.
/// Attach checks the local timer's quiescence and ARAT before publishing.
///
/// # Safety
/// The installed BSP owns the existing local timer and IDT, before any
/// native driver or AP, and a retained HPET's complete handoff.
unsafe fn service_clock(
    handoff: &Handoff,
    host_owner: &mut host::HostOwner,
    native_mmio: Option<&crate::native_mmio::Source>,
) -> Result<crate::native_clock::Installed, stela_drivers::platform::work::WorkError> {
    let identity = host_owner.root_identity()?;
    match crate::native_clock::Prepared::tsc(handoff.tsc_hz) {
        // SAFETY: the caller's timer/IDT handoff; the TSC has no owner.
        Ok(tsc) => unsafe { crate::native_clock::Installed::attach_native(identity, tsc) },
        // SAFETY: the caller's timer/IDT and retained HPET handoffs.
        Err(_) => native_mmio.map_or(
            Err(stela_drivers::platform::work::WorkError::Unavailable),
            |source| unsafe { source.install_service_clock() },
        ),
    }
}

fn start_platform(
    handoff: &'static Handoff,
    host_owner: &mut host::HostOwner,
    options: &Options,
) -> Platform {
    // The owned MMIO permission the host devices' services map through. A
    // host it cannot check is reported, and those services refuse.
    // Its time is logged: it walks every mapping the firmware left, which
    // no step's time shows.
    // A freeze here was a screen that stopped after the step before with
    // nothing named (issue 36), so this is a step and names its parts.
    crate::status::begin("Taking over the PC's devices from the firmware");
    crate::status::progress("memory-mapped registers");
    let activating = host::rdtsc();
    // SAFETY: this boot's capture, on the installed boot processor before
    // the other host processors start.
    let native_mmio = unsafe { crate::native_mmio::Source::activate(handoff, host_owner) }
        .inspect(|_| {
            output::arguments(format_args!(
                "NATIVE MMIO aperture owned ms={}\n",
                host::rdtsc().wrapping_sub(activating) / (handoff.tsc_hz / 1000).max(1)
            ))
        })
        .inspect_err(|error| {
            output::arguments(format_args!("NATIVE MMIO unavailable: {error:?}\n"));
        })
        .ok()
        .map(|source| &*alloc::boxed::Box::leak(alloc::boxed::Box::new(source)));
    #[cfg(feature = "diagnostics")]
    if let Some(mode) = options.audio_check {
        let source =
            native_mmio.unwrap_or_else(|| output::fatal("audio HPET aperture unavailable"));
        // SAFETY: the explicit audio and whole HPET handoffs were checked at
        // the entry. The sole BSP owns both resources before other devices.
        unsafe { check_audio(handoff, host_owner, source, mode) }.unwrap_or_else(|error| {
            output::arguments(format_args!("AUDIO CHECK failed: {error}\n"));
            output::fatal(format_args!("the audio check failed: {error}"))
        });
    }
    // Without an invariant TSC the HPET is the drivers' minimum delays and
    // the kernel work clock, as Linux falls back to it. A diagnostic that
    // takes the HPET itself keeps it.
    if !crate::cpu::INVARIANT_TSC.present() && options.diagnostic_option().is_none() {
        let result = native_mmio
            .ok_or(refuse!("native MMIO aperture unavailable"))
            .and_then(|source| {
                // SAFETY: firmware has stopped and no diagnostic owns the
                // HPET, so this boot owns the whole controller until
                // shutdown. The installed BSP owns the source before any
                // native driver.
                unsafe { source.install_pci_clock() }
            });
        match result {
            Ok(()) => output::arguments(format_args!("NATIVE PCI HPET retained\n")),
            Err(error) => output::arguments(format_args!("NATIVE PCI HPET unavailable: {error}\n")),
        }
    }
    crate::status::progress("interrupt vectors");
    match host_owner
        .root_identity()
        .map_err(|_| stela_drivers::bus::irq::IrqError::WrongCpu)
        .and_then(|identity| {
            // SAFETY: the installed BSP owns the existing IDT and the same
            // all-root APIC mapping before any external route can publish.
            unsafe {
                crate::native_irq::start(
                    layout::physical(handoff.apic_base & crate::apic::BASE_ADDRESS),
                    handoff.tsc_hz,
                    identity,
                )
            }
        }) {
        Ok(()) => output::arguments(format_args!("NATIVE IRQ device vectors attached\n")),
        Err(error) => output::arguments(format_args!("NATIVE IRQ unavailable: {error:?}\n")),
    }
    // Retire the configuration census and park the contained function owners
    // before either SCI publication or remapper takeover.
    crate::status::progress("PCI functions");
    let contained_pci = native_mmio.and_then(|mmio| {
        // SAFETY: firmware is stopped, APs have not started, and native
        // probing has not published any PCI/controller owner yet.
        match unsafe { mmio.contain_pci(handoff.tsc_hz) } {
            Ok(contained) => {
                output::arguments(format_args!(
                    "HOST PCI contained: {} functions\n",
                    contained.functions()
                ));
                Some(contained)
            }
            Err(error) => {
                output::arguments(format_args!("HOST PCI containment unavailable: {error}\n"));
                mmio.refuse_framebuffer(format!(
                    "the firmware framebuffer's PCI function could not be checked because PCI \
                     containment failed: {error}"
                ));
                None
            }
        }
    });
    if let (Some(screen), Some(mmio)) = (handoff.screen, native_mmio) {
        let refusal = match (&contained_pci, screen.pci_source) {
            (Some(pci), Some(source)) => {
                // SAFETY: the original parked PCI fabric still excludes all
                // competing configuration writers at this point in startup.
                match unsafe { pci.check_framebuffer(source) } {
                    Ok(()) => {
                        output::arguments(format_args!(
                            "HOST GOP PCI source {} BAR{} observed before and after firmware \
                             exit\n",
                            source.address,
                            source.bar.index()
                        ));
                        None
                    }
                    Err(error) => Some(format!(
                        "the firmware framebuffer's PCI function {} changed across firmware \
                         exit or cannot be read ({error:?})",
                        source.address
                    )),
                }
            }
            (Some(_), None) => {
                output::arguments(format_args!(
                    "HOST GOP PCI source unknown: no GOP device path or display function's \
                     BAR holds the firmware framebuffer; the firmware screen stops before a \
                     GPU starts\n"
                ));
                mmio.framebuffer_owner_unknown();
                None
            }
            // The containment failure is kept and logged above.
            (None, _) => None,
        };
        if let Some(refusal) = refusal {
            output::arguments(format_args!("HOST GOP PCI source unavailable: {refusal}\n"));
            mmio.refuse_framebuffer(refusal);
        }
    }
    crate::status::progress("DMA remapping and interrupt controllers");
    // The remapping units, taken before any host driver starts DMA.
    let mut remappers = match (&contained_pci, handoff.storage.handoff()) {
        // SAFETY: no host driver has started; the loader root maps the ACPI
        // tables and firmware translation was turned off at ExitBootServices.
        (Some(pci), Some(storage)) => unsafe {
            remapping::Remappers::take(
                handoff.rsdp,
                handoff.root,
                storage.dma.clone(),
                pci,
                &handoff.inactive_firmware_units,
                handoff.tsc_hz,
            )
        },
        _ => remapping::Remappers::none(),
    };
    // The IOAPICs are the operating system's after ExitBootServices, as
    // every OS takes them: firmware and SMM program no routes at runtime.
    if let Some(pci) = &contained_pci {
        // SAFETY: firmware has exited and no driver or AP has started. The
        // original stopped PCI census and all-unit remapper pins remain held
        // while the sole BSP contains the IOAPICs.
        match unsafe { remappers.install_ioapics(pci) } {
            Ok(units) => output::arguments(format_args!(
                "HOST IOAPIC contained: {units} original controllers in direct delivery\n"
            )),
            Err(error) => output::arguments(format_args!(
                "HOST IOAPIC containment unavailable: {error:?}\n"
            )),
        }
    }
    if let Some(pci) = &contained_pci {
        remappers.forward_pci_bridges(pci);
    }
    let prepared_sci = match (&contained_pci, &handoff.power) {
        (Some(pci), Some(power)) => {
            match aml::sci::Prepared::capture(pci.source(), power.clone()) {
                Ok(prepared) => Some(prepared),
                Err(error) => {
                    output::arguments(format_args!("HOST ACPI SCI unavailable: {error:?}\n"));
                    None
                }
            }
        }
        _ => None,
    };
    crate::status::progress("clocks");
    let clock = if options.boots_macos() {
        // SAFETY: this BSP owns the existing timer/IDT before any driver,
        // and the HPET only after its whole-controller handoff above.
        unsafe { service_clock(handoff, host_owner, native_mmio) }
            .inspect_err(|error| {
                output::arguments(format_args!(
                    "NATIVE SERVICE CLOCK unavailable: {error:?}\n"
                ));
            })
            .ok()
            .map(|clock| &*alloc::boxed::Box::leak(alloc::boxed::Box::new(clock)))
    } else {
        None
    };
    // SAFETY: validated BSP handoff, before AP/driver producers.
    let work = match unsafe { kernel_work::KernelWork::prepare(handoff, host_owner, clock) } {
        Ok(work) => {
            #[cfg(feature = "diagnostics")]
            output::arguments(format_args!(
                "NATIVE WORK owned stacks=32 foreground=verified services=unpublished\n"
            ));
            Ok(&*alloc::boxed::Box::leak(alloc::boxed::Box::new(work)))
        }
        // The record keeps this line until the USB log opens again, which
        // the devices' start on the boot stack (`start_devices`) reaches.
        Err(error) => {
            output::arguments(format_args!("NATIVE WORK unavailable: {error}\n"));
            Err(error.reason())
        }
    };
    crate::status::progress("ACPI tables");
    let host_acpi = aml::HostAcpi::start(
        handoff.rsdp,
        handoff.root,
        handoff
            .storage
            .handoff()
            .map(|storage| storage.ecam)
            .unwrap_or_default(),
        native_mmio,
        handoff.tsc_hz,
        prepared_sci,
        work.as_ref().ok().copied(),
    );
    crate::native_idle::install(
        host_acpi.and_then(|acpi| acpi.processor_idle(crate::native_idle::capabilities())),
    );
    crate::status::done("");
    Platform {
        acpi: host_acpi,
        mmio: native_mmio,
        remappers: alloc::boxed::Box::leak(alloc::boxed::Box::new(remappers)),
        work,
    }
}

/// The network `--wifi` names, with the secret of the key file the loader
/// read, whose bytes are wiped here. A file it cannot use is reported once,
/// without its contents, and no network is joined.
fn wifi_network(
    handoff: &Handoff,
    options: &Options,
) -> Option<stela_drivers::net::wifi::network::Network> {
    let ssid = options.ssid.as_ref()?;
    let path = options
        .wifi_key
        .as_deref()
        .unwrap_or(stela_boot_core::options::DEFAULT_WIFI_KEY);
    let bytes = handoff.wifi_key.bytes as usize;
    if bytes == 0 {
        output::arguments(format_args!(
            "HOST Wi-Fi key {path} is missing, empty or unreadable: the network is not joined\n"
        ));
        return None;
    }
    // SAFETY: the loader left the key's bytes allocated, and only this
    // reads them.
    let key = unsafe {
        core::slice::from_raw_parts_mut(
            layout::physical(handoff.wifi_key.physical) as *mut u8,
            bytes,
        )
    };
    let secret = stela_drivers::net::wifi::network::Secret::parse(key);
    for byte in key.iter_mut() {
        // SAFETY: a byte of the key, which nothing else reads.
        unsafe { core::ptr::write_volatile(byte, 0) };
    }
    let secret = match secret {
        Ok(secret) => secret,
        Err(error) => {
            output::arguments(format_args!(
                "HOST Wi-Fi key {path} unusable ({error}): the network is not joined\n"
            ));
            return None;
        }
    };
    let country = options
        .wifi_country
        .as_deref()
        .and_then(|code| code.as_bytes().try_into().ok());
    Some(stela_drivers::net::wifi::network::Network {
        ssid: ssid.as_bytes().to_vec(),
        secret,
        country,
    })
}

/// The disk work the loader decided before the firmware exited
/// (`Handoff::disk_writes`): the change Install macOS's disk picker planned
/// for the `--storage` partition's disk, when the picker ran this boot, and
/// the updates of the boot's partitions. The host disks' start does it,
/// once (`storage::prepare`, `storage::update`).
fn disk_writes(handoff: &Handoff) -> stela_runtime::pc::disk_writes::Pending {
    let bytes = handoff.disk_writes.bytes as usize;
    if bytes == 0 {
        return Default::default();
    }
    // SAFETY: the loader left the pending work's bytes allocated, and
    // nothing writes them.
    let encoded = unsafe {
        core::slice::from_raw_parts(
            layout::physical(handoff.disk_writes.physical) as *const u8,
            bytes,
        )
    };
    stela_runtime::pc::disk_writes::Pending::decode(encoded).unwrap_or_else(|| {
        output::fatal(
            "what this start was to write to the PC's disks did not reach Stela's drivers \
             whole; nothing was written. Start again",
        )
    })
}

/// The whole disk and USB start's bound, which only a stall outside every
/// PCI function's own budget reaches.
const DEVICES_START_US: u64 = 300_000_000;
/// The most network adapters the screen names when none started; the boot
/// log has them all.
const NETWORK_ACCOUNT_LINES: usize = 6;

/// The fail line for a disk and USB start the owned call did not complete.
fn devices_failed(error: &kernel_work::Error) -> alloc::string::String {
    let timed_out = matches!(
        error,
        kernel_work::Error::Work(stela_drivers::platform::work::WorkError::TimedOut)
    );
    stela_boot_core::work_status::devices_failed(
        timed_out,
        DEVICES_START_US / 1_000_000,
        &error.to_string(),
    )
}

/// What the host devices are opened for.
struct Wanted<'a> {
    firmware: Option<alloc::sync::Arc<stela_drivers::firmware::Catalog>>,
    processors: Option<&'static crate::native_smp::firmware::Capture>,
    targets: &'a [&'a Target],
    store: Option<Guid>,
    fdr: Option<Guid>,
    /// Whether the mount's own network is needed.
    network: bool,
}

/// The host devices the engine and the guest use.
struct HostDevices {
    services: Vec<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    medium: Option<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    preboot: Option<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    host_input: Option<input::HostInput>,
    host_usb: Option<usb::HostUsb>,
    host_bluetooth: Option<bluetooth::HostBluetooth>,
    host_network: Option<Result<network::Shared>>,
    usb_disks: Vec<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
    guest_network: Option<network::GuestNetwork>,
    host_disks: storage::HostDisks,
    fdr: Option<alloc::boxed::Box<dyn stela_drivers::block::BlockDevice>>,
}

/// Probe the host's devices once, with the native services when the
/// platform has them, and open the partitions, inputs and networks.
fn start_devices(
    handoff: &Handoff,
    options: &Options,
    wanted: &Wanted<'_>,
    host_acpi: Option<&'static aml::HostAcpi>,
    native_mmio: &Option<&'static crate::native_mmio::Source>,
    remappers: &'static remapping::Remappers,
    kernel_work: core::result::Result<&'static kernel_work::KernelWork, &str>,
) -> HostDevices {
    let &Wanted {
        firmware: ref host_firmware,
        processors: native_processors,
        targets,
        store: translation_store,
        fdr: fdr_partition,
        network: wants_network,
    } = wanted;
    let mut fdr_device = None;
    let mut host_usb = None;
    let host_bluetooth: Option<bluetooth::HostBluetooth> = None;
    let (services, medium, preboot, host_input, host_network, usb_disks, guest_network, host_disks) =
        if targets.is_empty()
            && options.install_source.is_none()
            && options.preboot.is_none()
            && !options.input
            && !wants_network
            && options.usb_storage.is_empty()
            && !options.external_disks
            && !options.guest_network
            && translation_store.is_none()
            && fdr_partition.is_none()
        {
            (
                Vec::new(),
                None,
                None,
                None,
                None,
                Vec::new(),
                None,
                storage::HostDisks::none(),
            )
        } else {
            let mut storage_handoff = handoff
                .storage
                .handoff()
                .unwrap_or_else(|| output::fatal(stela_boot_core::work_status::NO_DEVICE_ACCESS));
            storage_handoff.firmware = host_firmware.clone();
            // Disks that can signal their completions wake the BSP, which
            // runs the engine's devices.
            let bsp = native_processors.expect("validated native roster");
            let bsp_apic = bsp.processors[bsp.selected[0] as usize].apic_id;
            storage_handoff.completion = host::completion_signal(bsp_apic);
            // A disk step given up to another core's hold wakes it too.
            storage::ring_owed_steps_at(bsp_apic);
            #[cfg(feature = "diagnostics")]
            if options.audio_check.is_some() {
                // The audio check keeps the final slice even after shutdown.
                storage_handoff.dma.end -= crate::audio_check::DMA_BYTES;
            }
            if let Err(reason) = kernel_work {
                output::arguments(format_args!(
                    "NATIVE WORK degraded: devices start on the boot stack, without ACPI methods or a GPU\n"
                ));
                crate::status::info(&stela_boot_core::work_status::foreground(reason));
            }
            crate::status::begin("Starting the PC's disks and USB devices");
            // Every admitted input is owned or a boot owner.
            // No probe callback can borrow this dispatcher stack on timeout.
            let owned_targets: Vec<_> = targets.iter().map(|target| (**target).clone()).collect();
            let owned_preboot = options.preboot.clone();
            let owned_usb = options.usb_storage.clone();
            let owned_wifi = wifi_network(handoff, options);
            let owned_writes = disk_writes(handoff);
            let external = options.external_disks.then(|| {
                stela_soc::machine::UsbCables::disk_ports(options.guest_network, options.input)
                    .saturating_sub(owned_usb.len())
            });
            let root = handoff.root;
            let heap = handoff.heap.physical..handoff.heap.physical + handoff.heap.bytes;
            let tsc_hz = handoff.tsc_hz;
            let host_clock = storage_handoff.clock;
            let captures = handoff.storage.captures;
            let mmio = *native_mmio;
            let medium_partition = options.install_source;
            let log_partition = handoff.log_partition.map(Guid);
            let usb4_approved = options.usb4_approved.clone();
            let boot_disk = handoff.boot_disk().map(alloc::string::String::from);
            let device_skips = handoff.device_skips;
            let device_needs = handoff.device_needs;
            // Each PCI function's start has its own budget
            // (`stela_drivers::probe::DEVICE_START_US`) and is given up past
            // it; this whole bound only catches a stall outside every
            // function's start, so it leaves room for several given up.
            let open = move || {
                let native = mmio.map(|mmio| {
                    services::Services::new(mmio, remappers, host_acpi, heap, tsc_hz, usb4_approved)
                });
                let targets: Vec<_> = owned_targets.iter().collect();
                // SAFETY: one admitted foreground task under the work
                // dispatcher's loader-root/driver guard, or the boot stack
                // under the same guard when no owned stack exists. Owned
                // handoff, inputs and kept services survive completion or
                // quarantine.
                unsafe {
                    storage::open(
                        &storage_handoff,
                        captures,
                        root,
                        native
                            .as_ref()
                            .map(|native| native as &dyn stela_drivers::probe::NativeServices),
                        &storage::Selection {
                            targets: &targets,
                            pending: &owned_writes,
                            medium: medium_partition,
                            preboot: owned_preboot.as_ref(),
                            store: translation_store,
                            log: log_partition,
                            fdr: fdr_partition,
                            usb_disks: &owned_usb,
                            wifi: owned_wifi.as_ref(),
                            external,
                            boot_disk: boot_disk.as_deref(),
                            device_skips,
                            device_needs,
                        },
                    )
                }
            };
            let opened = match kernel_work {
                Ok(work) => work
                    .call_owned(DEVICES_START_US, open)
                    .map_err(|error| devices_failed(&error)),
                // Without owned stacks the start runs here, on the boot
                // stack, under the same host-driver guard the owned call
                // takes. Each function keeps its own start budget, whose
                // waits are the drivers' own; only the whole bound, which
                // needs a continuation to leave, is not there.
                Err(_) => {
                    // SAFETY: the loader's live root on the BSP, with no
                    // driver guard held; nothing else runs the drivers.
                    let _root = unsafe { storage::RestoreRoot::enter(root) };
                    Ok(open())
                }
            };
            let storage::Opened {
                services,
                medium,
                preboot,
                store,
                log,
                fdr,
                inputs,
                mut links,
                network_account,
                gpus,
                usb,
                bluetooth: host_controller,
                usb_disks,
                disks,
                power,
            } = opened
                .unwrap_or_else(|error| output::fatal(error))
                .unwrap_or_else(|refused| {
                    let error = refused.refusal;
                    output::arguments(format_args!("host storage refusal: {error}\n"));
                    // The USB's log, when its disk was found, keeps the
                    // step's lines before the stop.
                    if let Some(log) = refused.log {
                        crate::log_file::resume(log);
                    }
                    output::fatal(format_args!("host storage could not be opened: {error}"))
                });
            if let Some(acpi) = host_acpi {
                acpi.retain_power(power);
            }
            // The disks' step ends with what it found before the GPU's
            // begins, which would end it without.
            let count = |count: usize, one: &str, many: &str| {
                format!("{count} {}", if count == 1 { one } else { many })
            };
            let partitions = count(
                services.len() + usize::from(medium.is_some()),
                "partition",
                "partitions",
            );
            crate::status::done(&match usb_disks.len() {
                0 => partitions,
                disks => format!(
                    "{partitions}, and {} macOS sees as USB drives",
                    count(disks, "disk", "disks")
                ),
            });
            // The log first, as soon as its disk is open: it then holds every
            // line since the engine started (`output::add_log`), and a stall
            // or stop in the GPU's start below is kept on the USB.
            if let Some(log) = log {
                crate::log_file::resume(log);
            }
            let work = kernel_work.inspect_err(|reason| {
                if options.boots_macos() {
                    crate::status::warn(&stela_boot_core::work_status::gpu_without_work(reason));
                }
            });
            if options.boots_macos()
                && let Ok(work) = work
                && open_gpus(work, mmio, remappers, handoff, host_firmware, gpus)
            {
                let firmware = handoff
                    .screen
                    .and_then(|screen| screen.pci_source)
                    .map(|source| format!("{}", source.address));
                gpu::serve(
                    handoff.screen.map(|screen| screen.resolution()),
                    firmware.as_deref(),
                );
            }
            disks.profile_callbacks(options.profile);
            if let Some(store) = store {
                translations::set_partition(store);
                #[cfg(feature = "nvk")]
                super::pipelines::prepare();
            }
            fdr_device = fdr;
            if usb.firmware_missing() {
                crate::status::info(super::firmware_requests::MISSING);
            }
            host_usb = Some(usb);
            // macOS gets no Bluetooth controller. Its bluetoothd picks the
            // guest's USB controller and never sends it a command, in every
            // run with a host controller or the model, and Setup's Language
            // Chooser then waits for bluetoothd with a black screen. Without
            // a controller bluetoothd restarts and Setup goes on. The host's
            // controller is released in the loader's root by its drop.
            if host_controller.is_some() && options.input {
                output::arguments(format_args!(
                    "BLUETOOTH UNSUPPORTED the guest gets no controller: macOS does not \
                     configure the guest's USB Bluetooth controller and its Setup waits for it\n"
                ));
            }
            drop(host_controller);
            // With no adapter started, the screen names each one found and
            // why it stayed off, before the steps that need one say so.
            if links.devices.is_empty() && (wants_network || options.guest_network) {
                for line in network_account.iter().take(NETWORK_ACCOUNT_LINES) {
                    crate::status::info(&format!("Network adapter not started: {line}."));
                }
                if let Some(rest) = network_account
                    .len()
                    .checked_sub(NETWORK_ACCOUNT_LINES)
                    .filter(|&rest| rest > 0)
                {
                    crate::status::info(&format!("{rest} more network adapters not started."));
                }
            }
            // The guest's adapter is its own: the mount's client keeps the
            // first one. Without one the Mac boots with no network, which
            // is reported once.
            let guest_network = options
                .guest_network
                .then(|| {
                    network::GuestNetwork::open(&mut links, wants_network)
                        .inspect_err(|error| {
                            output::arguments(format_args!("guest network refusal: {error}\n"));
                            // Optional: macOS boots without it.
                            crate::status::info(
                                "macOS starts without a network: no supported network \
                                 adapter is free for it",
                            );
                        })
                        .ok()
                })
                .flatten();
            // A missing network is reported here and refused only by what
            // needs it, so the mount still boots and logs how far it gets.
            let network = wants_network.then(|| {
                crate::status::begin("Getting a network address");
                network::HostNetwork::open(links, host_clock, handoff, options)
                    .inspect(|network| crate::status::done(network.describe()))
                    .map(|network| network::Shared(Rc::new(RefCell::new(network))))
                    .inspect_err(|error| {
                        output::arguments(format_args!("host network refusal: {error}\n"));
                        // The step ends with the first line; each adapter's
                        // line and the advice follow on lines of their own.
                        let text = error.to_string();
                        let mut lines = text.lines();
                        crate::status::warn(lines.next().unwrap_or_default());
                        lines.for_each(crate::status::info);
                    })
            });
            (
                services,
                medium,
                preboot,
                if options.input {
                    Some(input::HostInput::new(inputs))
                } else {
                    // Unused devices are released where their registers and
                    // DMA pages are mapped, as the adapters are.
                    let root = inputs.root;
                    network::in_loader_root(root, || drop(inputs));
                    None
                },
                network,
                usb_disks,
                guest_network,
                disks,
            )
        };
    HostDevices {
        services,
        medium,
        preboot,
        host_input,
        host_usb,
        host_bluetooth,
        host_network,
        usb_disks,
        guest_network,
        host_disks,
        fdr: fdr_device,
    }
}
