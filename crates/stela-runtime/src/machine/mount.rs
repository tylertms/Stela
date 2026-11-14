//! A board's whole machine: every device model its DeviceTree describes
//! (`stela_soc_data::board::board`), placed on one [`AppleDevices`] and connected to its PMGR
//! lifecycle. The host's inputs (console, disks, cables, scanout, entropy and
//! board firmware files) come from the caller, so `stela.efi` and
//! `stela-dev boot check` mount the same machine.
use crate::Result;
use alloc::{boxed::Box, string::String, vec, vec::Vec};
use core::fmt;
use stela_contracts::refusal::Within;
use stela_contracts::refuse;
use stela_drivers::block::BlockDevice;
use stela_engine::machine::ctrr;
use stela_soc::machine::AicInput;
use stela_soc::machine::AppleDevices;
use stela_soc::machine::AscLines;
use stela_soc::machine::ConsoleSink;
use stela_soc::machine::DartId;
use stela_soc::machine::DcpextDma;
use stela_soc::machine::DcpextNub;
use stela_soc::machine::Range;
use stela_soc::machine::Scanout;
use stela_soc::machine::SepEntropy;
use stela_soc::machine::SepProvisioning;
use stela_soc::machine::UsbCables;
use stela_soc::machine::UsbId;
use stela_soc::machine::memory_resources::MemoryResources;
use stela_soc::machine::sep_state;
use stela_soc::machine::typec;
use stela_soc::machine::{RtcAnchor, RtcEpoch};
use stela_soc_data::board::{Board, PmgrBankRefusal};
use stela_soc_data::pmu::WallClock;

/// The regions iBoot leaves in the memory controller's lock groups:
/// CTRR-A over the bundle's read-only run, and CTRR-C and CTRR-D over the
/// cores' regions of those names, which SPTM requires the AMCC's to match
/// before it locks them (`amcc`). CTRR-B stays unprogrammed.
pub fn memory_locks(
    read_only_region: Option<(u64, u64)>,
    core_locks: &ctrr::CoreLocks,
) -> stela_soc_data::amcc::Programmed {
    let mut programmed = stela_soc_data::amcc::ctrr_a(read_only_region);
    for locked in &core_locks.locked {
        let group = match locked.region {
            ctrr::LockRegion::CtrrC => 2,
            ctrr::LockRegion::CtrrD => 3,
            _ => continue,
        };
        programmed[group] = Some((locked.first, locked.last + 1 - locked.first));
    }
    programmed
}

/// Report unsupported PMGR validations once, before a device is placed.
/// A diagnostic supplies no successful MMIO or lifecycle response.
pub fn pmgr_qualifications(refusals: &[PmgrBankRefusal], log: &mut dyn FnMut(fmt::Arguments<'_>)) {
    for (index, refusal) in refusals.iter().enumerate() {
        if refusals[..index].contains(refusal) {
            continue;
        }
        log(format_args!(
            "UNMODELED PMGR {:?}: {}; no register/lifecycle model\n",
            refusal.bank, refusal.reason,
        ));
    }
}

/// Paths checked by immutable source identity and bundle admission. Keeping
/// CLPC unavailable is a documented degradation, not a successful controller.
pub fn clpc_absence(hidden: &[String], log: &mut dyn FnMut(fmt::Arguments<'_>)) {
    for (index, path) in hidden.iter().enumerate() {
        if hidden[..index].contains(path) {
            continue;
        }
        log(format_args!(
            "UNMODELED CLPC {path}: controller hidden; closed-loop CPU power control unavailable\n"
        ));
    }
}

/// What the host connects to the board.
pub struct Inputs<'f> {
    /// The region iBoot leaves in each memory-controller lock group.
    pub programmed: stela_soc_data::amcc::Programmed,
    /// The entry point a boot bundle hands off to, on which boot firmware
    /// locks every PE's RVBAR.
    pub reset: Option<u64>,
    /// The system counter at placement and the host's wall time at it.
    pub now: u64,
    pub wall: Option<WallClock>,
    /// The PMU upcount the machine state kept (`pc::rtc_anchor`); `None`
    /// on a new machine or one whose state an older Stela wrote.
    pub rtc_anchor: Option<RtcAnchor>,
    /// The internal SSD's backing (`--storage`).
    pub backing: Option<Box<dyn BlockDevice>>,
    /// The SEP's state on the machine-state partition (`--nor`); volatile
    /// without one.
    pub sep_state: Option<sep_state::Location>,
    /// Advertise and log the SEP `sks`/`pnon`/`scrd` services.
    pub sep_observe: bool,
    /// The recoveryOS policy nonce the SEP proposes first: an install
    /// bundle's, whose staged LocalPolicy answer binds its digest.
    pub sep_recovery_policy_nonce: Option<[u8; 32]>,
    pub entropy: Box<dyn SepEntropy>,
    /// A restore host on the first USB-C connector.
    pub restore: bool,
    /// The native keyboard and pointer, with `usb`'s disks and Ethernet
    /// adapter, on the last connector.
    pub input: bool,
    pub usb: UsbCables,
    /// The display coprocessor's monitor.
    pub scanout: Option<Box<dyn Scanout>>,
    /// Board device firmware images by guest file name.
    pub firmware: &'f dyn Fn(&str) -> Option<&'f [u8]>,
    /// The boot bundle's macOS build, whose firmware protocols the
    /// coprocessor models speak; `None` without a bundle.
    pub build: Option<&'static stela_formats::builds::Build>,
}

/// Place `board`'s devices, writing `DEVICES` summaries to `log`, and return
/// them with the USB controller a restore host is cabled to.
pub fn devices<C: ConsoleSink + 'static>(
    board: &Board,
    console: C,
    inputs: Inputs<'_>,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<(AppleDevices, Option<UsbId>)> {
    devices_with_memory(board, console, inputs, None, log)
}

/// Mount the ordinary board around native memory owners. Callers must
/// keep their registered RAM and close all native/host writers before guest
/// entry. No legacy memory constructor is used when native owners are supplied.
///
/// Devices are placed in a fixed order: later steps borrow or overlap the
/// windows of earlier ones.
pub fn devices_with_memory<C: ConsoleSink + 'static>(
    board: &Board,
    console: C,
    inputs: Inputs<'_>,
    memory: Option<MemoryResources>,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<(AppleDevices, Option<UsbId>)> {
    validate_memory(board, memory.as_ref())?;
    let native_memory = memory.is_some();
    pmgr_qualifications(&board.pmgr_refusals, log);
    let Inputs {
        programmed,
        reset,
        now,
        wall,
        rtc_anchor,
        backing,
        sep_state,
        sep_observe,
        sep_recovery_policy_nonce,
        entropy,
        restore,
        input,
        usb,
        scanout,
        firmware,
        build,
    } = inputs;
    let build = build.ok_or(refuse!(
        "the board's coprocessors need the boot bundle's supported macOS build"
    ))?;
    let place = |device: stela_soc_data::board::Device| {
        (
            Range {
                physical: device.physical,
                bytes: device.bytes,
            },
            AicInput::new(0, device.irq as u16),
        )
    };
    let (uart, wdt, aic) = (place(board.uart), place(board.watchdog), board.aic);
    // Templates with no running node are useful for mount diagnostics. Actual
    // boot intake independently requires a verified selected handoff PE.
    let boot = board.boot_processor.as_ref().map_or(0, |boot| boot.cpu);
    let mut devices = AppleDevices::for_boot_processor(boot);
    devices
        .add_uart(console, &[uart.0], Some(uart.1))
        .map_err(|error| refuse!("UART placement: {error}", ?error))?;
    devices
        .add_watchdog_with_side(wdt.0, board.watchdog_side.as_ref(), Some(wdt.1), now)
        .map_err(|error| refuse!("watchdog placement: {error}", ?error))?;
    devices.add_aic_for_processors(
        aic.physical,
        aic.inputs,
        aic.layout,
        board.cpus.processors.len(),
    )?;
    add_gpio_and_nor(&mut devices, board)?;
    add_storage(&mut devices, board, backing, build)?;
    add_power_controllers(&mut devices, board, build, wall, rtc_anchor, now)?;
    add_processor_blocks(&mut devices, board, reset)?;
    for controller in &board.i2cs {
        devices.add_i2c(controller, firmware)?;
    }
    for phy in &board.typec_phys {
        devices
            .add_atc_phy(phy)
            .map_err(|error| refuse!("ATC PHY placement: {error}", ?error))?;
    }
    let darts = Darts::place(&mut devices, board)?;
    // The PMGR's power states precede the display, whose views reach into
    // its windows.
    add_power_states(&mut devices, board, log)?;
    // The display's PTD update view must borrow this publication bank,
    // before a broad display container can claim any overlapping bytes.
    if let Some(memory) = memory {
        add_native_memory(&mut devices, memory, log)?;
    }
    add_display(&mut devices, board, &darts, scanout, build)?;
    if let Some(sep) = board.sep {
        add_sep(
            &mut devices,
            &sep,
            &darts,
            Sep {
                entropy,
                state: sep_state,
                observe: sep_observe,
                recovery_policy_nonce: sep_recovery_policy_nonce,
            },
            log,
        )?;
    }
    let restore_usb = add_usbs(&mut devices, board, &darts, usb, input, restore, log)?;
    if let Some(aes) = board.aes {
        let ranges: Vec<Range> = aes
            .ranges
            .into_iter()
            .chain(aes.dkey)
            .map(|(physical, bytes)| Range { physical, bytes })
            .collect();
        // No per-installation UID1 key is provisioned yet: the engine
        // reports every hardware key disabled.
        devices.add_aes(
            &ranges,
            aes.config,
            Some(AicInput::new(0, aes.irq as u16)),
            (darts.native(aes.iommu, aes.instance)?, aes.stream as usize),
            None,
        )?;
    }
    add_soc_banks(&mut devices, board)?;
    add_memory_controllers(&mut devices, board, programmed, native_memory, log)?;
    log(format_args!(
        "DEVICES uart={:#x}+{:#x} irq={} wdt={:#x}+{:#x} irq={} aic={:#x} nr_irq={} \
         lockdown-blocks={} amcc-apertures={} darts={} aes={} aic-timebase=unplaced\n",
        uart.0.physical,
        uart.0.bytes,
        uart.1.irq,
        wdt.0.physical,
        wdt.0.bytes,
        wdt.1.irq,
        aic.physical,
        aic.inputs,
        board.lockdown.len(),
        board
            .memory_controller
            .as_ref()
            .map_or(0, |controller| controller.apertures.len()),
        darts.mounted.len(),
        board.aes.is_some()
    ));
    // Last: the `sgx` window spans other banks (gfx-asc, a PMGR fabric
    // bridge) and owns only the bytes they leave.
    if let Some(gfx) = board.gfx.as_ref() {
        devices.add_gfx(gfx, build)?;
    }
    devices.add_t6020_unmodeled_pmgr(&board.t6020_unmodeled_pmgr)?;
    // Finalize only after every native constructor, including SGX's final
    // overlapping-window placement, has registered its banks.
    connect_power(&mut devices, board, &darts, log)?;
    if restore && restore_usb.is_none() {
        return Err(refuse!("restore study requires a DT-placed USB device"));
    }
    Ok((devices, restore_usb))
}

/// Check the native memory owners against the board before anything is
/// placed. A board with a direct memory cache requires them.
fn validate_memory(board: &Board, memory: Option<&MemoryResources>) -> Result<()> {
    if board.direct_memory_cache.is_some() && memory.is_none() {
        return Err(refuse!(
            "direct MCC/AMCC/PTD requires its actual qualified native memory owner"
        ));
    }
    if let Some(memory) = memory {
        match memory {
            MemoryResources::T8112Cold(plan) => plan.validate_power(board.power_states.as_ref())?,
            MemoryResources::T6000Cold(plan) => plan.validate_power(board.power_states.as_ref())?,
            MemoryResources::T6020Cold(plan) => plan.validate(
                board.memory_controller.as_ref(),
                board.t6020_memory.as_ref(),
                board.power_states.as_ref(),
            )?,
            MemoryResources::T8112 { .. } => {}
        }
        memory.validate(
            board.memory_controller.as_ref(),
            board.memory_cache.as_ref(),
            board.direct_memory_cache.as_ref(),
        )?;
    }
    Ok(())
}

/// The GPIO controllers, with the SPI NOR's selects and the I2C targets'
/// interrupts, and the SPI NOR itself.
fn add_gpio_and_nor(devices: &mut AppleDevices, board: &Board) -> Result<()> {
    if let Some(gpio) = &board.gpio {
        let selects: Vec<u32> = board.spi_nor.iter().filter_map(|spi| spi.select).collect();
        let interrupts: Vec<u32> = board
            .i2cs
            .iter()
            .flat_map(|controller| &controller.targets)
            .filter_map(|target| target.interrupt)
            .collect();
        devices.add_gpio(gpio, &selects, &interrupts)?;
    }
    for aux in &board.aux_gpios {
        devices.add_aux_gpio(aux)?;
    }
    if let Some(spi) = board.spi_nor.as_ref() {
        devices.add_spi_nor(spi)?;
    }
    Ok(())
}

/// The internal SSD's ANS2, which needs `--storage` exactly when the
/// DeviceTree exposes one.
fn add_storage(
    devices: &mut AppleDevices,
    board: &Board,
    backing: Option<Box<dyn BlockDevice>>,
    build: &'static stela_formats::builds::Build,
) -> Result<()> {
    match (board.ans.as_ref(), backing) {
        (Some(ans), Some(backing)) => devices.add_ans(ans, backing, build)?,
        (Some(_), None) => return Err(refuse!("a visible ANS2 requires --storage SERIAL:GUID")),
        (None, Some(_)) => {
            return Err(refuse!(
                "--storage was supplied but this DeviceTree exposes no modeled ANS2"
            ));
        }
        (None, None) => (),
    }
    Ok(())
}

/// The SMC, the PMP and each SPMI bus with its PMICs.
fn add_power_controllers(
    devices: &mut AppleDevices,
    board: &Board,
    build: &'static stela_formats::builds::Build,
    wall: Option<WallClock>,
    rtc_anchor: Option<RtcAnchor>,
    now: u64,
) -> Result<()> {
    if let Some(smc) = board.smc.as_ref() {
        devices.add_smc(smc, build)?;
    }
    if let Some(pmp) = board.pmp.as_ref() {
        devices.add_pmp(pmp, build)?;
    }
    for spmi in &board.spmis {
        // A newly created Stela PMIC starts with its own oscillator origin
        // and no pending native fault or off-to-wake events. These are explicit
        // cold resource inputs, not Apple persistent/reset-state measurements.
        // PMIC resources must instead supply their history.
        let history = spmi
            .pmus
            .iter()
            .flat_map(|pmu| {
                pmu.retained()
                    .map(|(kind, window)| (pmu.sid, kind, vec![0; usize::from(window.bytes)]))
            })
            .collect::<Vec<_>>();
        // The upcount goes on from the machine state's anchor, as a Mac's
        // PMIC keeps counting while the machine is off.
        devices.add_spmi_with_resources(
            spmi,
            wall,
            RtcEpoch::anchored(now, wall, rtc_anchor),
            &history,
        )?;
    }
    devices.keep_rtc_origin(wall, rtc_anchor);
    Ok(())
}

/// Each PE's CoreSight, lockdown, startup and counter registers, the fast
/// IPIs and the CPU start block. With a reset entry, the startup registers
/// of PEs without a lockdown block lock their RVBAR to it.
fn add_processor_blocks(
    devices: &mut AppleDevices,
    board: &Board,
    reset: Option<u64>,
) -> Result<()> {
    for component in &board.coresight {
        devices
            .add_coresight_lock(component.physical)
            .map_err(|error| refuse!("CoreSight lock placement: {error}", ?error))?;
        if let Some(physical) = component.pc_sampler() {
            devices
                .add_coresight_sampler(component.cpu, physical)
                .map_err(|error| refuse!("UTTDBG sampler placement: {error}", ?error))?;
        }
    }
    for block in &board.lockdown {
        let range = Range {
            physical: block.physical,
            bytes: block.bytes,
        };
        let rvbar = block.cpu.zip(reset);
        devices
            .add_lockdown(block.name, block.control, range, rvbar)
            .map_err(|error| refuse!("{name} placement: {error}", name = block.name, ?error))?;
    }
    if let Some(entry) = reset {
        for block in &board.processor_registers {
            if board
                .lockdown
                .iter()
                .any(|legacy| legacy.cpu == Some(block.cpu))
            {
                continue;
            }
            devices
                .add_processor_registers(
                    block.cpu,
                    Range {
                        physical: block.physical,
                        bytes: block.bytes,
                    },
                    entry,
                )
                .map_err(|error| refuse!("PE startup register placement: {error}", ?error))?;
        }
    }
    for block in &board.counters {
        devices
            .add_core_counters(block.cpu, block.physical, block.layout)
            .map_err(|error| refuse!("sampler counter placement: {error}", ?error))?;
    }
    let mpidrs: Vec<u64> = board.cpus.processors.iter().map(|pe| pe.mpidr).collect();
    devices.add_fast_ipi(&mpidrs);
    if let Some((physical, clusters)) = board.cpu_start {
        devices.add_cpu_start(physical, clusters, &mpidrs)?;
    }
    Ok(())
}

/// The placed DART groups, in the board's order, and the board's
/// declarations they were placed from.
struct Darts<'b> {
    mounted: Vec<Vec<DartId>>,
    board: &'b Board,
}

impl<'b> Darts<'b> {
    /// Place every DART group with its SMMU.
    fn place(devices: &mut AppleDevices, board: &'b Board) -> Result<Self> {
        let mut mounted = Vec::new();
        for dart in &board.darts {
            let ranges: Vec<_> = dart
                .instances
                .iter()
                .map(|instance| Range {
                    physical: instance.physical,
                    bytes: instance.bytes,
                })
                .collect();
            let irq = AicInput::new(0, dart.irq as u16);
            let placed = devices
                .add_dart_group_for_profile(
                    dart.profile,
                    &ranges,
                    usize::from(dart.streams),
                    Some(irq),
                    dart.apf,
                )
                .within(|| refuse!("DART placement"))?;
            if let Some((physical, bytes)) = dart.smmu {
                devices
                    .add_dart_smmu(placed[0], Range { physical, bytes })
                    .map_err(|error| refuse!("DART SMMU placement: {error}", ?error))?;
            }
            mounted.push(placed);
        }
        Ok(Self { mounted, board })
    }

    /// The DART a native DMA route names, from a group placed in full.
    fn native(&self, group: usize, instance: usize) -> Result<DartId> {
        let mounted = self
            .mounted
            .get(group)
            .ok_or(refuse!("native DMA route names an unmounted DART group"))?;
        let declared = self
            .board
            .darts
            .get(group)
            .ok_or(refuse!("native DMA route has no declared DART group"))?;
        if mounted.len() != declared.instances.len() {
            return Err(refuse!(
                "native DMA mounted DART group differs from its complete declaration"
            ));
        }
        mounted
            .get(instance)
            .copied()
            .ok_or(refuse!("native DMA route names an unmounted DART instance"))
    }
}

/// The PMGR's power-state blocks.
fn add_power_states(
    devices: &mut AppleDevices,
    board: &Board,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<()> {
    if let Some(states) = board.power_states.as_ref() {
        let blocks: Vec<Range> = states
            .blocks
            .iter()
            .map(|&(physical, bytes)| Range { physical, bytes })
            .collect();
        devices.add_power_states(&blocks, &states.registers)?;
        log(format_args!(
            "DEVICES pmgr power-states={} blocks={}\n",
            states.registers.len(),
            blocks.len()
        ));
    }
    Ok(())
}

/// The native memory owners' controller, cache and PTD models.
fn add_native_memory(
    devices: &mut AppleDevices,
    memory: MemoryResources,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<()> {
    match memory {
        MemoryResources::T6000Cold(plan) => {
            plan.install(devices)?;
            log(format_args!(
                "DEVICES mcc native direct MCC/AMCC/PTD share canonical RAM and source power graph\n"
            ));
        }
        MemoryResources::T6020Cold(plan) => {
            plan.install(devices)?;
            log(format_args!(
                "DEVICES mcc native T602x MCC/AMCC/PTD share canonical RAM and source power graph\n"
            ));
        }
        MemoryResources::T8112Cold(plan) => {
            plan.install(devices)?;
            log(format_args!(
                "DEVICES mcc native cold cache/DCS/PTD follow actual PMGR and canonical RAM\n"
            ));
        }
        MemoryResources::T8112 { memory, cache } => {
            devices.add_t8112_empty_memory_resources(memory, cache)?;
            log(format_args!(
                "DEVICES mcc native DCS/PTD and cache share registered RAM\n"
            ));
        }
    }
    Ok(())
}

/// The dcpext display coprocessor and its register apertures, presenting
/// to the host's scanout, mastering through its DART and reading swap
/// surfaces through the display pipe's.
fn add_display(
    devices: &mut AppleDevices,
    board: &Board,
    darts: &Darts<'_>,
    scanout: Option<Box<dyn Scanout>>,
    build: &'static stela_formats::builds::Build,
) -> Result<()> {
    let Some(dcp) = board.dcp.as_ref() else {
        return Ok(());
    };
    for route in &dcp.surface_routes {
        devices.validate_native_dart(
            "DCP declared surface mapper",
            darts.native(route.iommu, route.instance)?,
            route.stream as usize,
        )?;
    }
    let scanout = scanout.ok_or(refuse!("visible external DCP requires one actual scanout"))?;
    let range = |(physical, bytes): (u64, u64)| Range { physical, bytes };
    devices.add_dcpext(
        DcpextNub {
            wrapper: range(dcp.wrapper),
            interrupts: dcp.interrupts,
            quiesced: dcp.quiesced,
            role: dcp.role,
            identity: dcp.identity,
        },
        scanout,
        DcpextDma {
            link: (darts.native(dcp.iommu, dcp.instance)?, dcp.stream as usize),
            window: dcp.window,
            surface: match (
                dcp.surface,
                dcp.surface_instance,
                dcp.surface_routes.first(),
            ) {
                (Some((iommu, stream)), Some(instance), Some(primary))
                    if (primary.iommu, primary.instance, primary.stream)
                        == (iommu, instance, stream) =>
                {
                    Some((darts.native(iommu, instance)?, stream as usize))
                }
                (None, None, None) => None,
                _ => return Err(refuse!("DCP surface DMA owner metadata is incomplete")),
            },
        },
        build,
    )?;
    let regions: Vec<(&'static str, Range)> = dcp
        .regions
        .iter()
        .map(|&(name, reg)| (name, range(reg)))
        .collect();
    let mut display_darts = Vec::new();
    for group in
        core::iter::once(dcp.iommu).chain(dcp.surface_routes.iter().map(|route| route.iommu))
    {
        let instances = darts
            .mounted
            .get(group)
            .ok_or(refuse!("display DART group is missing"))?;
        for instance in 0..instances.len() {
            let owner = darts.native(group, instance)?;
            if !display_darts.contains(&owner) {
                display_darts.push(owner);
            }
        }
    }
    devices.add_display_with_darts(&regions, dcp.role, &display_darts)
}

/// The host's inputs to the SEP.
struct Sep {
    entropy: Box<dyn SepEntropy>,
    /// Where its state lives; volatile without one.
    state: Option<sep_state::Location>,
    /// Advertise and log the `sks`/`pnon`/`scrd` services.
    observe: bool,
    recovery_policy_nonce: Option<[u8; 32]>,
}

/// The SEP, mastering through the sole TRAD of its DART group.
fn add_sep(
    devices: &mut AppleDevices,
    sep: &stela_soc_data::board::SepDevice,
    darts: &Darts<'_>,
    host: Sep,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<()> {
    let group = darts
        .mounted
        .get(sep.iommu)
        .ok_or(refuse!("SEP DART group is missing"))?;
    let [dart] = group.as_slice() else {
        return Err(refuse!(
            "SEP DMA owner requires explicit multiple-TRAD selection"
        ));
    };
    if sep.instance != 0 {
        return Err(refuse!("SEP selected TRAD is outside its sole-owner group"));
    }
    let ranges = sep
        .ranges
        .map(|(physical, bytes)| Range { physical, bytes });
    let lines = AscLines::from_interrupts(sep.irqs.map(|irq| AicInput::new(0, irq as u16)));
    let ecid = sep.ecid.ok_or(refuse!(
        "SEP needs /chosen unique-chip-id (the machine ECID); none in the tree"
    ))?;
    let state = match host.state {
        Some(location) => {
            log(format_args!("SEP state on the machine-state partition\n"));
            location
        }
        None => {
            // Without `--nor` no host partition holds it: the roots
            // and anti-replay store last this power cycle.
            log(format_args!(
                "SEP state volatile: nonces end at power-off\n"
            ));
            sep_state::Location::Volatile
        }
    };
    if let Some(nonce) = &host.recovery_policy_nonce {
        use sha2::Digest;
        // The digest the staged LocalPolicy request binds as
        // `Ap,RecoveryOSPolicyNonceHash`; the nonce itself stays secret.
        log(format_args!(
            "SEP first recoveryOS policy nonce from the install bundle: digest={}\n",
            hex::encode(sha2::Sha384::digest(nonce))
        ));
    }
    devices.add_sep(
        ranges,
        lines,
        (*dart, sep.stream as usize),
        host.entropy,
        SepProvisioning {
            security: sep.security,
            ecid,
            kexts: sep.kexts,
            observe: host.observe,
            state,
            recovery_policy_nonce: host.recovery_policy_nonce,
        },
    )
}

/// Every USB controller. The native input and `usb`'s cables attach to the
/// last; a restore host, else the native client, to the first, whose ID is
/// returned when a restore host is cabled to it.
fn add_usbs(
    devices: &mut AppleDevices,
    board: &Board,
    darts: &Darts<'_>,
    mut usb: UsbCables,
    input: bool,
    restore: bool,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<Option<UsbId>> {
    let mut restore_usb = None;
    let usbs = board.usbs.len();
    for (index, controller) in board.usbs.iter().enumerate() {
        // The controller's stream translates through the group's first TRAD
        // instance, as SEP and AES masters do; per-instance selection is the
        // bus master's contract (`stela_soc_data::board::Iommu`).
        let group = darts
            .mounted
            .get(controller.iommu)
            .ok_or(refuse!("USB DART group is missing"))?;
        for (instance, _) in group.iter().enumerate() {
            devices.validate_native_dart(
                "USB declared bank",
                darts.native(controller.iommu, instance)?,
                controller.stream as usize,
            )?;
        }
        let id = devices.add_usb(controller, darts.native(controller.iommu, 0)?)?;
        // The native keyboard and pointer plug into the last connector
        // (J473AP usb-drd1), the Mac being the downstream-facing port.
        let hub = input || !usb.disks.is_empty() || usb.network.is_some();
        if hub && index + 1 == usbs && index != 0 {
            let disks = usb.disks.len();
            let network = usb.network;
            devices.attach_input(controller.port, core::mem::take(&mut usb))?;
            log(format_args!(
                "INPUT attached to USB-C port {} with {disks} USB disks, network={network:?}\n",
                controller.port
            ));
        }
        // The first controller (J473AP usb-drd0, the back-left connector) is
        // the one a restore host or the native client uses; the others are
        // left uncabled.
        if index != 0 {
            continue;
        }
        if restore {
            // A restore host is cabled to this connector from power-on: it
            // is the USB host and sources VBUS, so the Mac is a USB2 device.
            // The connector's HPM die and controller observe the same cable.
            let host = typec::Attached {
                orientation: typec::Orientation::Normal,
                role: typec::DataRole::Ufp,
                source: false,
                vbus: true,
                speed: typec::Usb2Speed::High,
            };
            devices.set_connection(controller.port, Some(host))?;
            restore_usb = Some(id);
        } else {
            devices.usb_client(id)?.query_type()?;
        }
    }
    Ok(restore_usb)
}

/// The SoC's register banks: fabric bridges, DPE, power, wake masks,
/// performance and energy counters, CPU clusters and the AON fuses.
fn add_soc_banks(devices: &mut AppleDevices, board: &Board) -> Result<()> {
    for bank in &board.bridges {
        devices.add_bridge(
            bank.physical,
            &bank.fields,
            bank.header,
            &bank.configs,
            bank.axi,
        )?;
    }
    if let Some(bank) = board.soc_dpe.as_ref() {
        devices.add_soc_dpe(bank.physical, &bank.samples)?;
    }
    for bank in &board.power_banks {
        devices.add_soc_power(bank.physical, bank.kind)?;
    }
    if let Some(bank) = board.wake_masks.as_ref() {
        devices.add_wake_masks(bank.physical, bank.valid, bank.smc_wake)?;
    }
    for bank in &board.performance {
        devices.add_performance_counters(bank.physical, &bank.selectors)?;
    }
    for bank in &board.energy {
        devices.add_cpu_energy(bank.physical, bank.cpm)?;
    }
    for (index, cluster) in board.clusters.iter().enumerate() {
        let events = board
            .cluster_events
            .get(index)
            .ok_or(refuse!("CPU cluster has no DT event selection"))?;
        // The board validated each CPM word against the translated DT reg.
        devices.add_cpm_power(
            cluster.base + stela_soc_data::cpm::T8112_OFFSET,
            cluster.counter_map,
        )?;
        devices.add_cpu_cluster(
            cluster.base,
            cluster.dvmr,
            cluster.pstates,
            cluster.counter_capacity,
            cluster.counter_map,
            events,
        )?;
    }
    if let Some((physical, _)) = board.aon_fuses {
        devices.add_aon_fuses(physical)?;
    }
    if let Some(physical) = board.rail_calibration {
        devices.add_rail_calibration(physical)?;
    }
    if !board.t6020_perf_boot.is_empty() {
        devices.add_t6020_perf_boot(&board.t6020_perf_boot)?;
    }
    Ok(())
}

/// The memory and I/O access controllers and the memory cache's channels.
/// Native memory owners already placed the memory controller and cache.
fn add_memory_controllers(
    devices: &mut AppleDevices,
    board: &Board,
    programmed: stela_soc_data::amcc::Programmed,
    native_memory: bool,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<()> {
    let memory_cache = board.memory_cache.as_ref();
    if !native_memory && let Some(controller) = board.memory_controller.as_ref() {
        let mut programmed = programmed;
        for (region, group) in programmed.iter_mut().zip(controller.layout.groups) {
            if group.is_none() {
                *region = None;
            }
        }
        devices.add_memory_controller(
            &controller.apertures,
            controller.layout,
            programmed,
            memory_cache.map(|mcc| mcc.cache),
        )?;
    }
    if let Some(controller) = board.io_access_controller.as_ref() {
        devices.add_io_access_controller(
            &controller.apertures,
            controller.layout,
            stela_soc_data::amcc::ctrr_a(programmed[0]),
        )?;
    }
    if !native_memory && let Some(mcc) = memory_cache {
        let (physical, bytes, channels) = mcc.channels;
        devices.add_memory_channels(
            Range { physical, bytes },
            channels,
            mcc.ptd.physical,
            &mcc.ptd.ranges,
        )?;
        log(format_args!(
            "DEVICES mcc cache={}MiB ways={} dcs-channels={channels} ptd={:#x} ptd-ranges={}\n",
            mcc.cache.bytes >> 20,
            mcc.cache.ways,
            mcc.ptd.physical,
            mcc.ptd.ranges.len()
        ));
    }
    Ok(())
}

/// Connect the PMGR's lifecycle to every placed bank. DART clock or power
/// gates need a modeled PMGR.
fn connect_power(
    devices: &mut AppleDevices,
    board: &Board,
    darts: &Darts<'_>,
    log: &mut dyn FnMut(fmt::Arguments<'_>),
) -> Result<()> {
    if let Some(states) = board.power_states.as_ref() {
        let retention = board
            .darts
            .iter()
            .map(|dart| dart.retention)
            .collect::<Vec<_>>();
        devices.connect_power_resources(states, &darts.mounted, &retention)?;
        log(format_args!(
            "DEVICES pmgr lifecycle-refusals={} resources={}\n",
            devices.power_refusals().len(),
            states.resources.len()
        ));
    } else if board
        .darts
        .iter()
        .any(|dart| !dart.clock_gates.is_empty() || !dart.power_gates.is_empty())
    {
        return Err(refuse!("DART resource gates require a modeled PMGR"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_soc_data::board::PmgrBank;

    #[test]
    fn clpc_absence_reports_each_path_once() {
        let paths = [
            "/arm-io/pmgr/clpc".into(),
            "/arm-io/pmgr/clpc".into(),
            "/arm-io/die1/clpc".into(),
        ];
        let mut messages = Vec::new();
        clpc_absence(&paths, &mut |message| messages.push(format!("{message}")));
        assert_eq!(messages.len(), 2);
        assert!(messages[0].contains("UNMODELED CLPC /arm-io/pmgr/clpc: controller hidden"));
    }

    #[test]
    fn pmgr_qualifications_report_each_requirement_once_without_a_model() {
        let unsupported = PmgrBankRefusal {
            bank: PmgrBank::CpuClusters,
            reason: "unknown cluster layout",
        };
        let bridge = PmgrBankRefusal {
            bank: PmgrBank::Bridges,
            reason: "unknown bridge layout",
        };
        let mut messages = Vec::new();
        pmgr_qualifications(&[unsupported, unsupported, bridge], &mut |message| {
            messages.push(format!("{message}"))
        });
        assert_eq!(messages.len(), 2);
        assert!(messages[0].contains("UNMODELED PMGR CpuClusters"));
        assert!(
            messages
                .iter()
                .all(|message| message.contains("no register/lifecycle model"))
        );
    }
}
