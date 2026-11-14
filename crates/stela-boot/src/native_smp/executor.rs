//! Physical AP lifetime and processor-local native resources. Startup ACKs
//! prove entry/ownership only; the runtime publication protocol separately
//! requires a recovered native loan before any CPU settlement.
use super::{bootstrap, firmware, resources, topology, tsc_sync};
use crate::Refusal;
use crate::{
    apic::{self, Apic},
    engine::{host, interrupts},
    handoff::Handoff,
    layout,
    tables::PAGE,
};
use alloc::{boxed::Box, vec::Vec};
use core::{
    arch::asm,
    cell::UnsafeCell,
    sync::atomic::{AtomicU8, Ordering},
};
use stela_contracts::refuse;

mod mailbox;
mod park;

const STARTING: u8 = 0;
const READY: u8 = 1;
const REFUSED: u8 = 2;
/// Initialized, and bringing its time-stamp counter into agreement with
/// the BSP's, which takes part ([`tsc_sync`]).
const SYNCING: u8 = 3;
/// On its private descriptors and stack: the AP reads the low SIPI page no
/// more, so the next AP may start from it.
const ENTERED: u8 = 4;

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum StartupFailure {
    Resources,
    Bootstrap(Refusal),
    FaultRecord {
        slot: usize,
        cause: stela_boot_core::early_fault::RecordError,
    },
    EarlyException {
        slot: usize,
        cause: bootstrap::EarlyFault,
    },
    Delivery(apic::StartupError),
    FirmwareTopology(topology::Refusal),
    FirmwareInit {
        processor_number: u32,
        apic_id: u64,
        cause: apic::StartupError,
    },
    Timeout {
        slot: usize,
    },
    Refused {
        slot: usize,
        reason: Refusal,
    },
}

/// This object and its descriptors remain pinned for the whole boot, even
/// after a startup timeout or a parked worker: NMI/MCE can still enter them.
struct Argument {
    processor: topology::Processor,
    source: topology::Snapshot,
    effective_pat: u64,
    // An explicit authored diagnostic request, never execution/ISA permission.
    #[cfg(feature = "diagnostics")]
    fp_qualification: bool,
    stack_bottom: u64,
    stack_top: u64,
    config: UnsafeCell<Option<host::Config>>,
    tables: UnsafeCell<Option<&'static mut interrupts::Tables>>,
    observed: UnsafeCell<Option<topology::Snapshot>>,
    descriptors: UnsafeCell<Option<interrupts::Observation>>,
    doorbells: UnsafeCell<Option<&'static core::sync::atomic::AtomicU64>>,
    refusal: UnsafeCell<Option<Refusal>>,
    status: AtomicU8,
    tsc: tsc_sync::Sync,
    mailbox: mailbox::Mailbox,
}

// SAFETY: the BSP initializes immutable identity/source fields before SIPI.
// Only the assigned physical AP takes config/tables and writes observed, once;
// a Release status publication precedes every BSP observation. No Cpu, Host,
// guest bytes or runtime native loan are made Send/Sync by this startup record.
unsafe impl Sync for Argument {}

pub(crate) struct Processor {
    slot: usize,
    argument: &'static Argument,
}

impl Processor {
    pub(crate) fn slot(&self) -> usize {
        self.slot
    }
    pub(crate) fn apic_id(&self) -> u64 {
        self.argument.processor.apic_id
    }

    /// This is startup evidence, never runtime quiescence permission.
    pub(crate) fn observation(&self) -> &topology::Snapshot {
        assert_eq!(self.argument.status.load(Ordering::Acquire), READY);
        // SAFETY: the owning AP wrote this once before Release READY and never
        // changes it; the record stays pinned for the complete executor life.
        unsafe {
            (&*self.argument.observed.get())
                .as_ref()
                .expect("ready AP observation")
        }
    }

    pub(crate) fn descriptors(&self) -> interrupts::Observation {
        assert_eq!(self.argument.status.load(Ordering::Acquire), READY);
        // SAFETY: one immutable publication before READY, kept for boot.
        unsafe { (&*self.argument.descriptors.get()).expect("ready AP descriptors") }
    }

    /// Directed interrupt completion evidence only; this never grants a
    /// stopped generation or returned native frame to the runtime.
    #[cfg(feature = "diagnostics")]
    pub(crate) unsafe fn check_doorbell(
        &self,
        sender: &Apic,
        tsc_hz: u64,
    ) -> Result<u64, StartupFailure> {
        if self.argument.status.load(Ordering::Acquire) != READY {
            return Err(StartupFailure::Refused {
                slot: self.slot,
                reason: refuse!("executor is not ready"),
            });
        }
        // SAFETY: the AP published this pinned read-only atomic reference before
        // READY. No Cpu, native frame, Host or recovery state is read here.
        let counter = unsafe { (&*self.argument.doorbells.get()).as_ref().copied() }
            .ok_or(StartupFailure::Resources)?;
        let before = counter.load(Ordering::Acquire);
        // SAFETY: caller owns the BSP ICR and this AP's private IDT is ready.
        unsafe {
            sender.notify(
                u32::try_from(self.apic_id()).map_err(|_| StartupFailure::Resources)?,
                interrupts::NATIVE_WAKE_VECTOR,
                tsc_hz,
            )
        }
        .map_err(StartupFailure::Delivery)?;
        let started = apic::rdtsc();
        while counter.load(Ordering::Acquire) == before {
            if apic::rdtsc().wrapping_sub(started) >= tsc_hz {
                return Err(StartupFailure::Timeout { slot: self.slot });
            }
            core::hint::spin_loop();
        }
        Ok(counter.load(Ordering::Acquire))
    }
}

pub(crate) struct Executors {
    /// BSP is absent: it already owns its local HostOwner. Slots 1 to
    /// `processors.len()`, in order.
    processors: Vec<Processor>,
    /// The first AP that did not start, when one did not: it and every AP
    /// after it run no PE.
    failure: Option<StartupFailure>,
}

impl Executors {
    pub(crate) fn processors(&self) -> &[Processor] {
        &self.processors
    }
    /// The executors that run PEs: the BSP and the started APs.
    pub(crate) fn started(&self) -> usize {
        self.processors.len() + 1
    }
    /// Why fewer cores started than were reserved.
    pub(crate) fn failure(&self) -> Option<&StartupFailure> {
        self.failure.as_ref()
    }
}

unsafe extern "sysv64" fn enter(argument: u64) -> ! {
    // SAFETY: BSP supplied one pinned Argument to exactly this physical AP.
    let argument = unsafe { &*(argument as *const Argument) };
    // SAFETY: this AP alone consumes its descriptor owner, once, before any
    // interrupts/native calls can use this CPU's GS/IST/private handler.
    let tables = unsafe {
        (&mut *argument.tables.get())
            .take()
            .expect("AP descriptors")
    };
    let tables_pointer = tables as *const interrupts::Tables;
    // SAFETY: this AP alone owns `tables`, taken once above, and installs them before any interrupt
    // or native call.
    unsafe {
        interrupts::install_on(tables, interrupts::interrupt);
    }
    // Its own GDT, IDT and stack: the low page is the next AP's.
    argument.status.store(ENTERED, Ordering::Release);
    let initialized = (|| {
        // SAFETY: CPL0 on the selected AP, with private descriptors.
        let observed =
            unsafe { firmware::observe() }.map_err(|_| refuse!("AP source-control capture refused"))?;
        if !observed.matches(argument.processor) {
            return Err(refuse!("AP physical identity changed"));
        }
        // SAFETY: this AP alone consumes its private disjoint host resources.
        let mut config = unsafe {
            (&mut *argument.config.get())
                .take()
                .ok_or(refuse!("AP resources absent"))?
        };
        observed
            .qualify_native(matches!(
                config.placement,
                stela_engine::machine::mmu::placement::Placement::FiveLevel
            ))
            .map_err(|refusal| refusal.refusal("AP native controls refused"))?;
        // The trampoline installs the BSP's already-checked effective PAT;
        // The offline trampoline installed the complete validated BSP image.
        let source = topology::Snapshot {
            pat: argument.effective_pat,
            ..argument.source
        };
        observed
            .same_memory_types(&source)
            .map_err(|_| refuse!("AP memory types differ"))?;
        // The BSP's APIC mode, which `config.apic_base` carries: an AP
        // takes x2APIC when the BSP did, before any APIC handle of it.
        config.apic_base = if config.apic_base & apic::X2APIC_MODE != 0 {
            // SAFETY: CPL0 on this AP with interrupts off; its host state,
            // the first APIC handle, is installed below from this base.
            let base = unsafe { apic::prefer_x2apic() };
            if base & apic::X2APIC_MODE == 0 {
                return Err(refuse!("AP offers no x2APIC, which the BSP runs"));
            }
            base
        } else {
            observed.apic_base
        };
        let loader_root = config.window_root;
        // SAFETY: configured pool/arena/bounce entries belong only to this AP;
        // tables/local record remain pinned through every native call and park.
        let owner =
            unsafe { host::install(config) }.map_err(|_| refuse!("AP private host installation failed"))?;
        #[cfg(feature = "diagnostics")]
        let mut owner = owner;
        let apic = u32::try_from(argument.processor.apic_id).map_err(|_| refuse!("AP APIC ID"))?;
        host::join_shootdown(
            crate::native_mmio::PROCESSORS
                .slot(apic)
                .ok_or(refuse!("AP missing from the aperture's processors"))?,
        );
        #[cfg(feature = "diagnostics")]
        super::context_check::check(&mut owner.host())?;
        #[cfg(feature = "diagnostics")]
        if argument.fp_qualification {
            host::fp_diagnostic::check(&mut owner.host())?;
        }
        // Keep the installed private root/control state, not the
        // earlier loader root observed before Host initialization.
        let installed =
            // SAFETY: CPL0 on the selected AP, after its private descriptors were installed.
            unsafe { firmware::observe() }.map_err(|_| refuse!("AP installed-control capture refused"))?;
        if installed.cr3 == loader_root || !installed.matches(argument.processor) {
            return Err(refuse!("AP private root or physical identity differs"));
        }
        // SAFETY: reads this CPU's descriptor registers and RSP and changes nothing.
        let descriptors = unsafe { interrupts::observe() };
        if !(argument.stack_bottom..argument.stack_top).contains(&descriptors.rsp)
                // SAFETY: `tables` were installed on this AP and live for the rest of its
                // execution.
            || !unsafe { &*tables_pointer }.matches(descriptors)
        {
            return Err(refuse!("AP private descriptor or stack differs"));
        }
        // SAFETY: only this AP publishes these immutable observations, before
        // Release READY. The counter lives in its permanently pinned GS record.
        unsafe {
            *argument.observed.get() = Some(installed);
            *argument.descriptors.get() = Some(descriptors);
            *argument.doorbells.get() = Some(host::doorbell_counter());
        }
        Ok(owner)
    })()
    .and_then(|owner| {
        argument.status.store(SYNCING, Ordering::Release);
        let offset = argument.tsc.target()?;
        // This AP reads the BSP's counter, and counts its guest clock,
        // through its software correction from now on.
        host::agree_tsc(offset);
        Ok(owner)
    });
    match initialized {
        Ok(mut owner) => {
            argument.status.store(READY, Ordering::Release);
            let mut worker = None;
            loop {
                crate::native_mmio::shootdown::service();
                if argument.mailbox.poll(&mut owner, &mut worker) {
                    continue;
                }
                // SAFETY: interrupts are disabled and this AP's IDT takes its
                // 0x60 doorbell, which ends the wait even when already pending.
                unsafe { crate::native_idle::wait(|| None) };
            }
        }
        Err(reason) => {
            // SAFETY: only this AP writes the immutable reason before Release
            // REFUSED. The BSP reads it only after the matching Acquire.
            unsafe { *argument.refusal.get() = Some(reason) };
            argument.status.store(REFUSED, Ordering::Release);
            // Host state may not exist after refusal, so keep maskable IRQs
            // disabled rather than route a doorbell into an uninstalled Host.
            loop {
                // SAFETY: halts this refused AP with maskable interrupts off; it touches no memory.
                unsafe {
                    asm!("cli", "hlt", options(nomem, nostack));
                }
            }
        }
    }
}

/// Remove every enabled firmware AP from its idle code before reclaiming boot
/// services RAM, including APs that will later receive owned startup/SIPI.
/// No AP stack, callback, heap allocation or execution acknowledgement is used.
/// Intel SDM3A, "Local APIC State After an INIT Reset (Wait-for-SIPI State)";
/// UEFI PI MP Services' firmware idle state is not a Stela-owned executor.
///
/// # Safety
/// Run immediately after EBS and before reusing any firmware boot-services RAM,
/// on the captured BSP with IF clear and exclusive access to its local ICR.
/// `capture` is the immutable loader-owned copy, validated before EBS. `sender`
/// addresses the live local APIC (identity UC mapping or x2APIC), and `tsc_hz`
/// is the calibrated source clock. No firmware call may follow. Keep all AP
/// resources kept on delivery failure; no uncertain AP is admitted here.
pub(crate) unsafe fn quiesce_firmware_aps(
    capture: &firmware::Capture,
    sender: &Apic,
    tsc_hz: u64,
) -> Result<usize, StartupFailure> {
    let source = capture
        .processors
        .get(..capture.processor_count as usize)
        .ok_or(StartupFailure::Resources)?;
    let selected = capture
        .selected
        .get(..capture.core_count as usize)
        .filter(|selected| !selected.is_empty())
        .ok_or(StartupFailure::Resources)?;
    // Validate the entire captured assignment before exposing any target. Only
    // the BSP is excluded from INIT at this boundary; selected APs still use
    // firmware idle code until their later owned INIT/SIPI sequence.
    let _validated =
        topology::enabled_unselected(source, selected).map_err(StartupFailure::FirmwareTopology)?;
    let targets = topology::enabled_unselected(source, &selected[..1])
        .map_err(StartupFailure::FirmwareTopology)?;
    if source[selected[0] as usize].apic_id != u64::from(sender.id()) {
        return Err(StartupFailure::Bootstrap(refuse!(
            "firmware BSP differs from the live local APIC"
        )));
    }
    if tsc_hz == 0 {
        return Err(StartupFailure::Delivery(
            apic::StartupError::ClockUnavailable,
        ));
    }
    // Validate every destination before sending the first INIT: a wide xAPIC
    // target, duplicate source identity or malformed assignment fails closed.
    let destination = |index: usize| -> Result<u32, StartupFailure> {
        let processor = source[index];
        let failure = |cause| StartupFailure::FirmwareInit {
            processor_number: processor.processor_number,
            apic_id: processor.apic_id,
            cause,
        };
        let id = u32::try_from(processor.apic_id)
            .map_err(|_| failure(apic::StartupError::DestinationTooWide))?;
        sender.validate_init(id, tsc_hz).map_err(failure)?;
        Ok(id)
    };
    for index in targets.clone() {
        destination(index)?;
    }
    let mut delivered = 0;
    for index in targets {
        let processor = source[index];
        let id = destination(index)?;
        // SAFETY: source identity, BSP exclusion, full destination set
        // and source clock were checked before any effect. No SIPI is sent.
        unsafe { sender.init_only(id, tsc_hz) }.map_err(|cause| StartupFailure::FirmwareInit {
            processor_number: processor.processor_number,
            apic_id: processor.apic_id,
            cause,
        })?;
        delivered += 1;
        crate::output::arguments(format_args!(
            "NATIVE FIRMWARE INIT delivered apic={id:#x} firmware_cpu={} destination_ack=unobserved\n",
            processor.processor_number,
        ));
    }
    Ok(delivered)
}

/// Start the selected physical APs using Stela-owned resources after EBS.
///
/// # Safety
/// Runs only on the validated BSP with interrupts disabled and its local Host
/// installed. The immutable loader capture and every reserved stack/root/code/
/// pool/bounce/physmap mapping stay pinned through this boot. RAM ranges remain
/// owned and immutable as metadata. The caller already called native_smp::validate.
/// The post-EBS loader already called quiesce_firmware_aps before reclaiming
/// firmware boot-services RAM; INIT delivery is not an AP READY observation.
/// No native guest execution is admitted by returning this startup set.
pub(crate) unsafe fn start(
    handoff: &'static Handoff,
    capture: &'static firmware::Capture,
    ram: &[host::RamRange],
    tsc_base: u64,
    _fp_qualification: bool,
) -> Result<Executors, StartupFailure> {
    let cores = handoff.native_resources.cores as usize;
    if cores == 0 || cores > capture.core_count as usize || handoff.tsc_hz == 0 {
        return Err(StartupFailure::Resources);
    }
    let sender = Apic::new(
        handoff.apic_base,
        layout::physical(handoff.apic_base & apic::BASE_ADDRESS),
    );
    let mut processors = Vec::with_capacity(cores - 1);
    let mut started: Vec<Starting> = Vec::with_capacity(cores - 1);
    // SAFETY: CPL0, BSP; PAT exists under the validated native floor.
    let memory_source = unsafe { firmware::observe() }.map_err(|_| StartupFailure::Resources)?;
    let source_expected = topology::Snapshot {
        pat: memory_source.pat,
        ..capture.snapshots[0]
    };
    memory_source
        .same_memory_types(&source_expected)
        .map_err(|_| StartupFailure::Resources)?;
    let memory_types = super::memory_types::Image::from_snapshot(&memory_source)
        .map_err(StartupFailure::Bootstrap)?;
    if memory_types.amd.is_some() {
        for range in ram {
            memory_types
                .writeback_ram(range.host, range.bytes)
                .map_err(StartupFailure::Bootstrap)?;
        }
        for region in [
            handoff.heap,
            handoff.code,
            handoff.pool,
            handoff.native_resources.stacks,
            handoff.native_resources.invocations,
            handoff.native_resources.tasks,
        ]
        .into_iter()
        .filter(|region| region.bytes != 0)
        {
            memory_types
                .writeback_ram(region.physical, region.bytes)
                .map_err(StartupFailure::Bootstrap)?;
        }
        if cores > 1 {
            memory_types
                .writeback_ram(handoff.native_resources.sipi, PAGE)
                .map_err(StartupFailure::Bootstrap)?;
        }
    }
    let effective_pat = memory_source.pat;
    let bsp = sender.id();
    if u64::from(bsp) != capture.processors[capture.selected[0] as usize].apic_id {
        return Err(StartupFailure::Resources);
    }
    // The BSP's host and handlers are installed. Publish its local slot
    // before any AP can run or issue a retirement.
    host::join_shootdown(
        crate::native_mmio::PROCESSORS
            .join_bsp(bsp)
            .ok_or(StartupFailure::Resources)?,
    );
    // An AP that does not start ends the startup of the ones after it, and
    // the guest runs on the cores before it (`Executors::failure`).
    let mut failed: Vec<(usize, StartupFailure)> = Vec::new();
    for slot in 1..cores {
        if !failed.is_empty() {
            break;
        }
        let attempt = (|| -> Result<(), StartupFailure> {
            let part =
                resources::partition(cores, cores, slot, handoff.code.bytes, handoff.pool.bytes)
                    .map_err(|_| StartupFailure::Resources)?;
            let processor = capture.processors[capture.selected[slot] as usize];
            let destination =
                u32::try_from(processor.apic_id).map_err(|_| StartupFailure::Resources)?;
            let config = host::Config {
                ram: ram.to_vec(),
                pool_physical: handoff.pool.physical + part.pool_offset,
                pool_bytes: part.pool_bytes,
                window_root: handoff.root,
                loader_tables: &handoff.native_loader_tables,
                code_virtual: layout::code(handoff.code.bytes) + part.code_offset,
                code_physical: handoff.code.physical + part.code_offset,
                code_bytes: part.code_bytes,
                code_parts: handoff.native_resources.code_parts,
                window: (part.window_bytes != 0).then(|| {
                    (
                        layout::code(handoff.code.bytes) + part.window_offset,
                        handoff.code.physical + part.window_offset,
                        part.window_bytes,
                    )
                }),
                tsc_base,
                tsc_hz: handoff.tsc_hz,
                // As the firmware left it, in the BSP's APIC mode, which
                // the AP takes at entry.
                apic_base: capture.snapshots[slot].apic_base
                    | handoff.apic_base & apic::X2APIC_MODE,
                placement: handoff.placement,
                bounce: Some((
                    layout::bounce() + slot as u64 * 2 * PAGE,
                    handoff.native_resources.bounce[slot],
                )),
            };
            let argument: &'static Argument = Box::leak(Box::new(Argument {
                processor,
                source: memory_source,
                effective_pat,
                #[cfg(feature = "diagnostics")]
                fp_qualification: _fp_qualification,
                stack_bottom: layout::stack() + slot as u64 * resources::STACK_STRIDE + PAGE,
                stack_top: layout::stack()
                    + slot as u64 * resources::STACK_STRIDE
                    + PAGE
                    + resources::STACK_BYTES,
                config: UnsafeCell::new(Some(config)),
                tables: UnsafeCell::new(Some(Box::leak(Box::new(interrupts::Tables::new())))),
                observed: UnsafeCell::new(None),
                descriptors: UnsafeCell::new(None),
                doorbells: UnsafeCell::new(None),
                refusal: UnsafeCell::new(None),
                status: AtomicU8::new(STARTING),
                tsc: tsc_sync::Sync::new(
                    handoff.tsc_hz,
                    stela_boot_core::tsc::sync::round_millis(
                        processor.location.package,
                        (0..slot).map(|synced| {
                            capture.processors[capture.selected[synced] as usize]
                                .location
                                .package
                        }),
                    ),
                ),
                mailbox: mailbox::Mailbox::new(),
            }));
            let physical = handoff.native_resources.sipi;
            crate::output::arguments(format_args!(
                "NATIVE ATTEMPT slot={slot} apic={:#x} firmware_cpu={} package={} core={} thread={} sipi={physical:#x} stack={:#x}..{:#x}\n",
                processor.apic_id,
                processor.processor_number,
                processor.location.package,
                processor.location.core,
                processor.thread,
                argument.stack_bottom,
                argument.stack_top,
            ));
            // SAFETY: this one owned low page has no previous reader: the preceding
            // AP acknowledged high-window entry after its last low-page fetch.
            let vector = unsafe {
                bootstrap::prepare(
                    layout::physical(physical) as *mut u8,
                    physical,
                    bootstrap::Entry {
                        processor_signature: capture.snapshots[slot]
                            .get(1, 0)
                            .ok_or(StartupFailure::Resources)?
                            .eax,
                        root: handoff.root,
                        five_level: matches!(
                            handoff.placement,
                            stela_engine::machine::mmu::placement::Placement::FiveLevel
                        ),
                        pat: effective_pat,
                        memory_types: &memory_types,
                        stack_top: layout::stack()
                            + slot as u64 * resources::STACK_STRIDE
                            + PAGE
                            + resources::STACK_BYTES,
                        argument: argument as *const Argument as u64,
                        entry: enter,
                    },
                )
            }
            .map_err(StartupFailure::Bootstrap)?;
            // Only the BSP writes the shared boot log. Persist the last completed
            // preparation before any real INIT/SIPI can fault or reset the machine;
            // a buffered ATTEMPT alone does not localize an early hardware stop.
            crate::output::arguments(format_args!(
                "NATIVE PREPARED slot={slot} apic={destination:#x} vector={vector:#x} root={:#x}\n",
                handoff.root,
            ));
            crate::output::flush_log();
            // It may translate through the MMIO aperture from its first fetch,
            // so it shares the aperture's shootdowns from now on, and answers
            // them while it waits for an engine or driver lock whose holder may
            // be retiring.
            crate::native_mmio::PROCESSORS
                .join_ap(destination)
                .ok_or(StartupFailure::Resources)?;
            stela_engine::platform::sync::set_wait(crate::native_mmio::shootdown::service);
            stela_drivers::on_spin(crate::native_mmio::shootdown::service);
            // SAFETY: validated physical AP and pinned complete startup resources.
            unsafe { sender.startup(destination, vector, handoff.tsc_hz) }
                .map_err(StartupFailure::Delivery)?;
            crate::output::arguments(format_args!(
                "NATIVE SIPI delivered slot={slot} vector={vector:#x}\n"
            ));
            let since = apic::rdtsc();
            // Linux's parallel bringup: the next AP starts as soon as this one
            // has left the low page, and each installs its private host beside
            // the others while the BSP takes their TSC synchronization in turn.
            loop {
                // A sealed hardware exception is a failed startup, not a
                // timeout or a synthetic READY. Return without reusing any owned
                // AP/low-page resource whose execution remains uncertain.
                if let Some(cause) =
                    // SAFETY: the low startup page stays owned by this bringup until the AP leaves it
                    // or fails.
                    unsafe { bootstrap::failure(layout::physical(physical) as *const u8) }
                            .map_err(|cause| StartupFailure::FaultRecord { slot, cause })?
                {
                    return Err(StartupFailure::EarlyException { slot, cause });
                }
                match argument.status.load(Ordering::Acquire) {
                    STARTING => {}
                    REFUSED => return Err(refused(slot, argument)),
                    _ => break,
                }
                if apic::rdtsc().wrapping_sub(since) >= handoff.tsc_hz {
                    return Err(StartupFailure::Timeout { slot });
                }
                serve(&mut started, &mut processors, &mut failed, handoff.tsc_hz);
                core::hint::spin_loop();
            }
            started.push(Starting {
                processor: Processor { slot, argument },
                destination,
                since,
                synced: false,
            });
            Ok(())
        })();
        if let Err(error) = attempt {
            if let Ok(apic) =
                u32::try_from(capture.processors[capture.selected[slot] as usize].apic_id)
            {
                crate::native_mmio::PROCESSORS.exclude(apic);
            }
            failed.push((slot, error));
        }
    }
    while !started.is_empty() {
        serve(&mut started, &mut processors, &mut failed, handoff.tsc_hz);
        core::hint::spin_loop();
    }
    crate::output::flush_log();
    processors.sort_unstable_by_key(|processor| processor.slot);
    failed.sort_unstable_by_key(|(slot, _)| *slot);
    let failure = failed.into_iter().next();
    if let Some((first, _)) = &failure {
        // Every AP before the first failure is ready; later ones idle in
        // their mailbox loop, answering shootdowns, and run no PE.
        processors.retain(|processor| processor.slot < *first);
    }
    // SAFETY: every started AP is ready, or failed and then nothing starts
    // from the page again; the memory types are the BSP's, checked above.
    unsafe {
        park::park(
            handoff,
            capture,
            &sender,
            &memory_types,
            effective_pat,
            failure.is_some(),
        )
    };
    Ok(Executors {
        processors,
        failure: failure.map(|(_, error)| error),
    })
}

/// A started AP that is not ready yet.
struct Starting {
    processor: Processor,
    destination: u32,
    /// When its one second began, moved on by the BSP's time with others.
    since: u64,
    synced: bool,
}

/// Take part in each started AP's TSC synchronization, one at a time as
/// Linux's `check_tsc_sync_source` does, and take the APs that became
/// ready. An AP that refuses, or is not ready within a second of its own,
/// goes to `failed`, out of the aperture's shootdowns.
fn serve(
    started: &mut Vec<Starting>,
    ready: &mut Vec<Processor>,
    failed: &mut Vec<(usize, StartupFailure)>,
    tsc_hz: u64,
) {
    let mut index = 0;
    while index < started.len() {
        let (slot, argument) = (
            started[index].processor.slot,
            started[index].processor.argument,
        );
        match argument.status.load(Ordering::Acquire) {
            SYNCING if !started[index].synced => {
                let begun = apic::rdtsc();
                let met = argument.tsc.source();
                started[index].synced = true;
                // The APs waiting meanwhile lose none of their second.
                let spent = apic::rdtsc().wrapping_sub(begun);
                for other in started.iter_mut() {
                    other.since = other.since.wrapping_add(spent);
                }
                if !met {
                    // The AP stopped mid-synchronization; the abandoned
                    // rendezvous refuses its later points, so it never
                    // reports ready on its own.
                    crate::output::arguments(format_args!(
                        "NATIVE TSC slot={slot} missed a synchronization point: left out\n"
                    ));
                    let failing = started.swap_remove(index);
                    crate::native_mmio::PROCESSORS.exclude(failing.destination);
                    failed.push((slot, StartupFailure::Timeout { slot }));
                    continue;
                }
                let outcome = argument.tsc.outcome();
                crate::output::arguments(format_args!(
                    "NATIVE TSC slot={slot} adjusted={} offset={} rounds={}\n",
                    outcome.adjusted, outcome.offset, outcome.rounds,
                ));
                static OFFSET_REPORTED: core::sync::atomic::AtomicBool =
                    core::sync::atomic::AtomicBool::new(false);
                if outcome.offset != 0 && !OFFSET_REPORTED.swap(true, Ordering::Relaxed) {
                    // The guest clock and the host's counter reads
                    // (`host::rdtsc`) take the offset; the raw counter
                    // stays apart, and only durations measured on one core
                    // read it.
                    crate::output::arguments(format_args!(
                        "NATIVE TSC slot={slot} agrees through a software offset of {}: \
                     its counter has no IA32_TSC_ADJUST or did not converge, and is \
                     never written\n",
                        outcome.offset,
                    ));
                }
            }
            READY => {
                let Starting {
                    processor,
                    destination,
                    ..
                } = started.swap_remove(index);
                let installed = processor.observation();
                let descriptors = processor.descriptors();
                // This is the Acquire READY publication after high
                // entry, private Host/context validation and kept
                // observations; no new success flag or runtime stopped-worker
                // permission is inferred from the log record.
                crate::output::arguments(format_args!(
                    "NATIVE READY slot={slot} apic={destination:#x} cr3={:#x} cr0={:#x} cr4={:#x} efer={:#x} pat={:#x} rsp={:#x} gdt={:#x} idt={:#x} gs={:#x}\n",
                    installed.cr3,
                    installed.cr0,
                    installed.cr4,
                    installed.efer,
                    installed.pat,
                    descriptors.rsp,
                    descriptors.gdt,
                    descriptors.idt,
                    descriptors.gs,
                ));
                ready.push(processor);
                continue;
            }
            REFUSED => {
                let failing = started.swap_remove(index);
                crate::native_mmio::PROCESSORS.exclude(failing.destination);
                failed.push((slot, refused(slot, argument)));
                continue;
            }
            _ if apic::rdtsc().wrapping_sub(started[index].since) >= tsc_hz => {
                // A slow AP that still runs later is outside every shootdown
                // from here on. That is safe only while it idles and takes no
                // aperture translations, which an AP that never reached
                // READY does not.
                let failing = started.swap_remove(index);
                crate::native_mmio::PROCESSORS.exclude(failing.destination);
                failed.push((slot, StartupFailure::Timeout { slot }));
                continue;
            }
            _ => {}
        }
        index += 1;
    }
}

fn refused(slot: usize, argument: &Argument) -> StartupFailure {
    // SAFETY: the owner published this immutable reason before the Acquire
    // REFUSED observation; storage remains pinned.
    let reason =
        unsafe { (*argument.refusal.get()).clone() }.expect("refused AP publishes a reason");
    StartupFailure::Refused { slot, reason }
}
