//! Time-free native root admission. Intended child of engine::host::local.
//! No timer, work::Cpu implementation, interrupt delivery or device access.
use super::{MmioRoots, Owner, current, irq_active, native_frames_idle, state};
use crate::Result;
use crate::{
    engine::interrupts,
    native_smp::{firmware, topology::Snapshot},
};
use alloc::sync::Arc;
use core::{
    arch::asm,
    marker::PhantomData,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
use stela_contracts::refuse;
use stela_drivers::{bus::mmio::owned::PhysicalSpan, platform::work::WorkError};

const IF: u64 = 1 << 9;
const ADDRESS: u64 = 0x000f_ffff_ffff_f000;

/// This processor's GS base, as its record's self pointer names it. Every
/// owner and identity here belongs to an activated processor, whose GS base
/// is its own record from activation on, so a record of another processor
/// names that processor's base. The clock and IRQ checks run this many times
/// per guest clock read, where RDMSR would cost far more than the read.
fn gs_base() -> u64 {
    current() as u64
}
fn root() -> u64 {
    let value;
    // SAFETY: read-only CPL0 architectural observation.
    unsafe {
        asm!("mov {}, cr3", out(reg) value, options(nomem, nostack, preserves_flags));
    }
    value
}
fn disable() -> u64 {
    let saved;
    // SAFETY: native CPL0 dispatcher. Memory clobber encloses the save.
    unsafe {
        asm!("pushfq", "pop {}", "cli", out(reg) saved);
    }
    saved
}
unsafe fn restore(saved: u64) {
    // SAFETY: caller keeps the same CPU and enclosing IRQ save.
    unsafe {
        asm!("push {}", "popfq", in(reg) saved);
    }
}
fn unavailable<T>(_: T) -> WorkError {
    WorkError::Unavailable
}

/// Immutable CPU/descriptor/fixed-root identity, pinned by HostOwner.
/// This survives only checked RAM ownership joins. It grants no RAM-absence,
/// mutable mapping-publication, task switching, or cross-CPU execution permission.
/// Descriptor/local storage is static or permanently pinned through terminal IRQs.
pub(crate) struct RootIdentity {
    roots: Arc<MmioRoots>,
    loader_tables: &'static crate::paging::loader::LoaderTables,
    snapshot: Snapshot,
    gs: u64,
    gdt: u64,
    idt: u64,
    task: u16,
    ram_generation: AtomicU64,
}
/// Borrowed immutable root-read scope. It is neither an execution/stack loan
/// nor a firmware/controller capability. Every observation checks its live CPU,
/// entered root, IF and LA57 again; the original allocation owners stay pinned.
pub(crate) struct ProcessRootScope<'a> {
    identity: &'a RootIdentity,
    root: u64,
    _local: PhantomData<*mut ()>,
}
impl ProcessRootScope<'_> {
    /// Require the manager's entered original owner, not a Host
    /// root with equivalent shared mappings. This remains no stack/service grant.
    pub(crate) fn entered_original(
        &self,
        owner: &crate::paging::loader::LoaderTables,
    ) -> Result<(), WorkError> {
        self.identity.validate_process_root()?;
        if root() != self.root
            || !core::ptr::eq(self.identity.loader_tables, owner)
            || !crate::engine::storage::entered_loader_root(owner)
        {
            return Err(WorkError::CorruptContext);
        }
        Ok(())
    }
    pub(crate) fn observe_original(
        &self,
        address: u64,
    ) -> Result<Option<stela_boot_core::guarded_stacks::mappings::Observation>, WorkError> {
        self.identity.validate_process_root()?;
        if root() != self.root {
            return Err(WorkError::CorruptContext);
        }
        // SAFETY: same HostOwner/original loader owner, kept shared
        // aliases and IF0 scope were checked immediately above. No callbacks.
        let result =
            unsafe { self.identity.loader_tables.observe_shared(address) }.map_err(unavailable)?;
        self.identity.validate_process_root()?;
        if root() != self.root {
            return Err(WorkError::CorruptContext);
        }
        Ok(result)
    }
}
impl RootIdentity {
    fn validate_process_root(&self) -> Result<(), WorkError> {
        self.affinity()?;
        let flags: u64;
        // SAFETY: native CPU, after affinity before table access. The paging
        // depth was checked at identity capture: CR4.LA57 is fixed while
        // paging is on, and reading CR4 exits to a hypervisor.
        unsafe {
            asm!("pushfq", "pop {}", out(reg) flags, options(nomem, preserves_flags));
        }
        if flags & IF != 0 {
            return Err(WorkError::Busy);
        }
        if root() != self.snapshot.cr3
            && !crate::engine::storage::entered_loader_root(self.loader_tables)
        {
            return Err(WorkError::CorruptContext);
        }
        // SAFETY: fixed Host root is held by roots._lease, and the
        // entered original loader root is held by loader_tables. Both shared
        // their window at the real Host installation; no copied root grants it.
        unsafe { self.loader_tables.check_shared_root(root() & ADDRESS) }.map_err(unavailable)
    }
    pub(crate) fn process_root(&self) -> Result<ProcessRootScope<'_>, WorkError> {
        self.validate_process_root()?;
        Ok(ProcessRootScope {
            identity: self,
            root: root(),
            _local: PhantomData,
        })
    }
    /// Read-only clock admission for ONLY the Host root or its manager's
    /// entered original loader root. No arbitrary guest/root or task permission.
    pub(crate) fn check_clock_root(&self) -> Result<(), WorkError> {
        self.affinity()?;
        let saved = disable();
        let result = (|| {
            let scope = self.process_root()?;
            // Revalidate the original catalog's shared-window metadata
            // and page ownership; returned leaf data is not memory permission.
            let address = crate::layout::physical(self.loader_tables.root());
            let observed = scope
                .observe_original(address)?
                .ok_or(WorkError::CorruptContext)?;
            if observed.physical != self.loader_tables.root() || observed.user || !observed.writable
            {
                return Err(WorkError::CorruptContext);
            }
            Ok(())
        })();
        // SAFETY: read-only validation never switches/calls/migrates; restore
        // the exact saved flags even on a typed refusal before any table access.
        unsafe {
            restore(saved);
        }
        result
    }
    fn affinity(&self) -> Result<(), WorkError> {
        // BEFORE any GS-local dereference: no other CPU has this GS record,
        // so none can borrow this processor's removable scope. Every Linux
        // context check comes here; CPUID or reading APIC_BASE would exit to
        // a hypervisor, and the GS record already names one processor.
        if gs_base() != self.gs {
            return Err(WorkError::WrongCpu);
        }
        // SAFETY: immutable architectural observation; original owner kept.
        let observed = unsafe { interrupts::observe_descriptors() };
        if observed.gdt != self.gdt || observed.idt != self.idt || observed.task != self.task {
            return Err(WorkError::CorruptContext);
        }
        Ok(())
    }
    pub(crate) fn check_current(&self) -> Result<(), WorkError> {
        self.affinity()?;
        if root() != self.snapshot.cr3 {
            return Err(WorkError::CorruptContext);
        }
        Ok(())
    }
    pub(crate) fn check_irq_cpu(&self) -> Result<(), WorkError> {
        self.affinity()?;
        let current = root();
        if current & !ADDRESS != 0 || !self.roots.roots().contains(&current) {
            // Preserve the existing pinned guest IRQ roots above. The only
            // additional tagged root is this manager's entered loader
            // root, backed by its original allocation owner and same CPU guard.
            if !crate::engine::storage::entered_loader_root(self.loader_tables) {
                return Err(WorkError::CorruptContext);
            }
            let saved = disable();
            let result = self.validate_process_root();
            // SAFETY: read-only validation cannot switch/call/migrate.
            unsafe {
                restore(saved);
            }
            return result;
        }
        Ok(())
    }
    pub(crate) fn in_interrupt(&self) -> bool {
        self.affinity().is_err() || irq_active()
    }
    /// Immutable-root identity admission for process work, without a RAM grant.
    /// A native/helper frame cannot suspend its independent guest-state/code
    /// loan by starting a separate work executor underneath it.
    pub(crate) fn check_dispatch(&self) -> Result<(), WorkError> {
        self.check_irq_cpu()?;
        let flags: u64;
        // SAFETY: read the CPL0 caller's flags without changing IF.
        // Reject an interruptible boundary before reading affine frame slots.
        unsafe { asm!("pushfq", "pop {}", out(reg) flags) };
        if flags & IF != 0 || irq_active() || !native_frames_idle() {
            return Err(WorkError::Busy);
        }
        Ok(())
    }

    pub(crate) fn fatal(&self, error: WorkError) -> ! {
        crate::output::fatal(format_args!("native root contract failed: {error:?}"))
    }
    pub(crate) fn root_pages(&self) -> &[u64] {
        self.roots.roots()
    }
    pub(crate) fn memory_snapshot(&self) -> Result<Snapshot, WorkError> {
        self.check_current()?;
        Ok(self.snapshot)
    }
}

/// One generation's RAM-absence/publication view. Retaining this still prevents
/// RAM adoption; keeping only immutable RootIdentity does not keep this grant.
pub(crate) struct RootContext {
    identity: Arc<RootIdentity>,
    ram_generation: u64,
    executing: AtomicBool,
}
impl RootContext {
    /// Immutable identity only; this reference carries no generation permission.
    pub(crate) fn identity(&self) -> &Arc<RootIdentity> {
        &self.identity
    }
    fn generation(&self) -> Result<(), WorkError> {
        if self.ram_generation != self.identity.ram_generation.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    pub(crate) fn check_current(&self) -> Result<(), WorkError> {
        self.generation()?;
        self.identity.check_current()
    }
    pub(crate) fn in_interrupt(&self) -> bool {
        self.generation().is_err() || self.identity.in_interrupt()
    }
    pub(crate) fn fatal(&self, error: WorkError) -> ! {
        self.identity.fatal(error)
    }
    pub(crate) fn roots(&self) -> &Arc<MmioRoots> {
        &self.identity.roots
    }
    pub(crate) fn memory_snapshot(&self) -> Result<Snapshot, WorkError> {
        self.check_current()?;
        self.identity.memory_snapshot()
    }
}

struct Epoch {
    context: Arc<RootContext>,
    active: AtomicBool,
}
/// CPU-affine process epoch. This owner never moves to another processor.
/// Bootstrap begins with IF0 and grants no interrupt enable or runtime entry.
pub(crate) struct RootExecution {
    epoch: Arc<Epoch>,
    saved: u64,
    _local: PhantomData<*mut ()>,
}
impl RootExecution {
    pub(crate) fn check(&self) -> Result<(), WorkError> {
        if !self.epoch.active.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        self.context().check_current()?;
        if self.context().in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        Ok(())
    }
    pub(crate) fn context(&self) -> &Arc<RootContext> {
        &self.epoch.context
    }
}
impl Drop for RootExecution {
    fn drop(&mut self) {
        if let Err(error) = self.check() {
            self.context().fatal(error);
        }
        let context = self.context();
        let _saved = disable();
        // Fail BEFORE invalidating scope or restoring flags. An outstanding
        // mapping/IRQ source must keep its exact epoch, including quarantine.
        if Arc::strong_count(&self.epoch) != 1 {
            context.fatal(WorkError::Busy);
        }
        self.epoch.active.store(false, Ordering::Release);
        context.executing.store(false, Ordering::Release);
        // SAFETY: same-CPU entry, no remaining epoch views or callbacks.
        unsafe {
            restore(self.saved);
        }
    }
}

impl Owner {
    /// Keep CPU/root identity without extending a RAM-absence epoch.
    /// Only pristine same-CPU process dispatch, IF0, may create/borrow it.
    pub(crate) fn root_identity(&mut self) -> Result<Arc<RootIdentity>, WorkError> {
        if gs_base() != self.local.as_ptr() as u64 {
            return Err(WorkError::WrongCpu);
        }
        let flags: u64;
        // SAFETY: CPL0 observation before any local-state borrow.
        unsafe {
            asm!("pushfq", "pop {}", out(reg) flags, options(preserves_flags));
        }
        if flags & IF != 0 || irq_active() || !native_frames_idle() {
            return Err(WorkError::Busy);
        }
        if let Some(identity) = &self.root_identity {
            identity.check_current()?;
            return Ok(identity.clone());
        }
        let lease = self.retain_clock_root()?;
        // SAFETY: original physical Owner, IF0, no native frame or ISR.
        let snapshot = unsafe { firmware::observe() }.map_err(unavailable)?;
        snapshot.apic_id().ok_or(WorkError::Unavailable)?;
        // SAFETY: processor's installed descriptor set.
        let observed = unsafe { interrupts::observe() };
        if observed.gs != self.local.as_ptr() as u64 || snapshot.cr3 & !ADDRESS != 0 {
            return Err(WorkError::CorruptContext);
        }
        let state = state();
        let levels = if snapshot.cr4 & (1 << 12) != 0 { 5 } else { 4 };
        if state._loader_tables.levels() != levels {
            return Err(WorkError::CorruptContext);
        }
        if state.native || snapshot.cr3 != state.host_root {
            return Err(WorkError::Busy);
        }
        let mut roots = alloc::vec::Vec::new();
        roots
            .try_reserve_exact(2 + super::super::ROOTS * (1 + super::super::BANKS))
            .map_err(unavailable)?;
        roots.push(state.window_root);
        roots.push(state.host_root);
        roots.extend(state.all_roots());
        roots.sort_unstable();
        roots.dedup();
        if roots.iter().any(|r| *r == 0 || r & !ADDRESS != 0) {
            return Err(WorkError::CorruptContext);
        }
        let mut ram = alloc::vec::Vec::new();
        ram.try_reserve_exact(state.ram.len())
            .map_err(unavailable)?;
        for range in &state.ram {
            ram.push(PhysicalSpan::new(range.host, range.bytes).map_err(unavailable)?);
        }
        let identity = Arc::new(RootIdentity {
            roots: Arc::new(MmioRoots {
                roots,
                ram,
                _lease: lease,
            }),
            loader_tables: state._loader_tables,
            snapshot,
            gs: observed.gs,
            gdt: observed.gdt,
            idt: observed.idt,
            task: observed.task,
            ram_generation: AtomicU64::new(0),
        });
        identity.check_current()?;
        self.root_identity = Some(identity.clone());
        Ok(identity)
    }

    /// Join the new canonical RAM owner without granting it additional host bytes.
    /// Old publication/absence views must already be closed; no ISR/native/helper
    /// frame can overlap. CPU/root identity survives, but its old generation ends.
    pub(super) fn join_ram_identity(&mut self, incoming: &[super::super::RamRange]) -> Result<()> {
        if self.root_context.strong_count() != 0 {
            return Err(refuse!("RAM absence/publication views are still retained"));
        }
        let Some(identity) = &self.root_identity else {
            return Ok(());
        };
        identity
            .check_current()
            .map_err(|_| refuse!("retained CPU/root identity changed before RAM adoption"))?;
        if irq_active() || !native_frames_idle() {
            return Err(refuse!("active native/IRQ frame prevents RAM adoption"));
        }
        let flags: u64;
        // SAFETY: current physical Owner, read-only IRQ observation.
        unsafe {
            asm!("pushfq", "pop {}", out(reg) flags, options(preserves_flags));
        }
        if flags & IF != 0 {
            return Err(refuse!("RAM adoption requires excluded local interrupts"));
        }
        let state = state();
        let mut roots = alloc::vec::Vec::new();
        roots
            .try_reserve_exact(2 + super::super::ROOTS * (1 + super::super::BANKS))
            .map_err(|_| refuse!("RAM transition root catalog allocation failed"))?;
        roots.extend([state.window_root, state.host_root]);
        roots.extend(state.all_roots());
        roots.sort_unstable();
        roots.dedup();
        if roots != identity.root_pages() {
            return Err(refuse!(
                "retained native root catalog changed before RAM adoption"
            ));
        }
        let backing = identity.roots.ram().iter().map(|r| (r.start(), r.bytes()));
        let existing = state.ram.iter().map(|r| (r.host, r.bytes));
        let incoming = incoming.iter().map(|r| (r.host, r.bytes));
        let existing_matches =
            stela_boot_core::root_transition::same_backing_iter(backing.clone(), existing)
                .map_err(|_| refuse!("malformed old physical backing before RAM adoption"))?;
        let incoming_matches =
            stela_boot_core::root_transition::same_backing_iter(backing, incoming)
                .map_err(|_| refuse!("malformed canonical physical backing before RAM adoption"))?;
        if !existing_matches || !incoming_matches {
            return Err(refuse!(
                "canonical RAM adoption changes retained physical placements"
            ));
        }
        let next = identity
            .ram_generation
            .load(Ordering::Acquire)
            .checked_add(1)
            .ok_or(refuse!("native RAM ownership generation exhausted"))?;
        // Every old absence view is gone, proven above. There is no callback,
        // root mutation or IRQ between this invalidation and the caller's join.
        identity.ram_generation.store(next, Ordering::Release);
        self.root_context = alloc::sync::Weak::new();
        Ok(())
    }

    /// Called only by the post-EBS native dispatcher on its owner CPU.
    /// The initial supported profile enters with IF0; timed Clock installation
    /// later supplies its own checked interrupt-delivery scope.
    pub(crate) fn enter_root(&mut self) -> Result<RootExecution, WorkError> {
        // Architectural preguard BEFORE current()/state()/local slot access.
        if gs_base() != self.local.as_ptr() as u64 {
            return Err(WorkError::WrongCpu);
        }
        let saved = disable();
        let result = (|| {
            if saved & IF != 0 {
                return Err(WorkError::InterruptContext);
            }
            if current() != self.local.as_ptr() || irq_active() || !native_frames_idle() {
                return Err(WorkError::Busy);
            }
            let identity = self.root_identity()?;
            let context = if let Some(context) = self.root_context.upgrade() {
                context.check_current()?;
                if !Arc::ptr_eq(&context.identity, &identity) {
                    return Err(WorkError::CorruptContext);
                }
                context
            } else {
                let context = Arc::new(RootContext {
                    ram_generation: identity.ram_generation.load(Ordering::Acquire),
                    identity,
                    executing: AtomicBool::new(false),
                });
                context.check_current()?;
                self.root_context = Arc::downgrade(&context);
                context
            };
            context
                .executing
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .map_err(|_| WorkError::Busy)?;
            Ok(RootExecution {
                epoch: Arc::new(Epoch {
                    context,
                    active: AtomicBool::new(true),
                }),
                saved,
                _local: PhantomData,
            })
        })();
        if result.is_err() {
            // SAFETY: same-CPU save above, no scope was published.
            unsafe {
                restore(saved);
            }
        }
        result
    }
}
