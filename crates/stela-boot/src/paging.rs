//! Stela's own identity page tables and fatal exception handlers, which
//! replace the firmware's once boot services end.
use crate::Result;
use crate::cpu::msr;
use alloc::vec::Vec;
use stela_contracts::refuse;
mod active;
#[cfg(feature = "diagnostics")]
pub(crate) mod config;
pub(crate) mod loader;
pub(crate) use active::{ActivatedTables, RootError};
use core::{
    arch::{asm, naked_asm},
    marker::PhantomData,
};

use stela_boot_core::paging_controls as controls;

pub const PAGE: usize = 4096;
const ENTRIES: usize = 512;
pub const LARGE_PAGE: usize = PAGE * ENTRIES;
pub const PRESENT: u64 = 1;
pub const WRITABLE: u64 = 1 << 1;
const LARGE: u64 = 1 << 7;
/// Write-through and cache-disable: PAT entry 3, uncached.
const UNCACHED: u64 = 1 << 3 | 1 << 4;
const DEVICE: u64 = PRESENT | WRITABLE | UNCACHED | NX;
pub const NX: u64 = 1 << 63;
const ADDRESS: u64 = 0x000f_ffff_ffff_f000;

const CR4_PAE: u64 = 1 << 5;
const CR4_GLOBAL: u64 = 1 << 7;
const CR4_OSFXSR: u64 = 1 << 9;
const CR4_OSXMMEXCPT: u64 = 1 << 10;
const CR4_LA57: u64 = 1 << 12;
const CR4_PCID: u64 = 1 << 17;
const EFER_NX_ENABLE: u64 = 1 << 11;
/// The memory type of PAT entry 1, which a page with PWT alone selects
/// (Intel SDM vol. 3A, "Selecting a Memory Type from the PAT"). Reset makes
/// it write-through; Stela makes it write-combining for the framebuffer
/// (`tables::WRITE_COMBINING`).
const PAT_ENTRY_1: u64 = 0xff << 8;
const PAT_WRITE_COMBINING: u64 = 0x01 << 8;

type Table = [u64; ENTRIES];

/// Four-level identity tables under construction. Every nonleaf entry is
/// supervisor RW, as the runtime's frame arenas require of their path.
pub struct Tables {
    root: u64,
    /// Exact allocations of this builder, not reconstructed pointer metadata.
    pages: Vec<AllocatedTablePage>,
    _local: PhantomData<*mut ()>,
}

/// One successful UEFI allocation kept as LOADER_DATA for this boot.
/// This non-Clone private owner is minted only by new_table's real allocation;
/// its internal physical identifier is not itself a construction capability.
struct AllocatedTablePage {
    physical: u64,
}

fn new_table() -> Result<AllocatedTablePage> {
    allocate_table(uefi::boot::AllocateType::AnyPages)
}

fn allocate_table(policy: uefi::boot::AllocateType) -> Result<AllocatedTablePage> {
    let page = uefi::boot::allocate_pages(policy, uefi::boot::MemoryType::LOADER_DATA, 1)
        .map_err(|_| refuse!("no memory for a page table"))?;
    let table = page.as_ptr().cast::<Table>();
    // SAFETY: a fresh page owned by Stela for the rest of the boot.
    unsafe { table.write_bytes(0, 1) };
    Ok(AllocatedTablePage {
        physical: table as u64,
    })
}

impl Tables {
    pub fn new() -> Result<Self> {
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(1)
            .map_err(|_| refuse!("page-table catalog allocation failed"))?;
        let allocation = new_table()?;
        let root = allocation.physical;
        pages.push(allocation);
        Ok(Self {
            root,
            pages,
            _local: PhantomData,
        })
    }

    pub fn root(&self) -> u64 {
        self.root
    }

    /// The child table under `table[index]`, created when absent.
    fn child(&mut self, table: u64, index: usize) -> Result<u64> {
        if index >= ENTRIES || !self.pages.iter().any(|page| page.physical == table) {
            return Err(refuse!("foreign page-table frame"));
        }
        // SAFETY: `table` is one of this builder's live pages.
        let entry = unsafe { &mut (*(table as *mut Table))[index] };
        if *entry & LARGE != 0 {
            return Err(refuse!("2 MiB chunk mapped twice"));
        }
        if *entry == 0 {
            self.pages
                .try_reserve(1)
                .map_err(|_| refuse!("page-table catalog allocation failed"))?;
            let allocation = new_table()?;
            let physical = allocation.physical;
            self.pages.push(allocation);
            *entry = physical | PRESENT | WRITABLE;
        }
        let child = *entry & ADDRESS;
        if !self.pages.iter().any(|page| page.physical == child) {
            return Err(refuse!("page-table child is not owned by this builder"));
        }
        Ok(child)
    }

    /// The page-directory entry of the 2 MiB chunk at `address`.
    fn directory_entry(&mut self, address: usize) -> Result<*mut u64> {
        let pointers = self.child(self.root, (address >> 39) & (ENTRIES - 1))?;
        let directory = self.child(pointers, (address >> 30) & (ENTRIES - 1))?;
        // SAFETY: the directory is one of this builder's live pages.
        Ok(unsafe { &raw mut (*(directory as *mut Table))[(address >> 21) & (ENTRIES - 1)] })
    }

    /// Map the 2 MiB chunk at `address` as one RW+NX page.
    pub fn large(&mut self, address: usize) -> Result<()> {
        let entry = self.directory_entry(address)?;
        // SAFETY: a live directory entry, unused until now.
        unsafe {
            if *entry != 0 {
                return Err(refuse!("2 MiB chunk mapped twice"));
            }
            *entry = address as u64 | PRESENT | WRITABLE | LARGE | NX;
        }
        Ok(())
    }

    /// Give the 2 MiB chunk at `address` an empty 4 KiB page table.
    pub fn small(&mut self, address: usize) -> Result<()> {
        let entry = self.directory_entry(address)?;
        // SAFETY: a live directory entry, unused until now.
        unsafe {
            if *entry != 0 {
                return Err(refuse!("2 MiB chunk mapped twice"));
            }
            self.pages
                .try_reserve(1)
                .map_err(|_| refuse!("page-table catalog allocation failed"))?;
            let allocation = new_table()?;
            let physical = allocation.physical;
            self.pages.push(allocation);
            *entry = physical | PRESENT | WRITABLE;
        }
        Ok(())
    }

    /// Map device registers at `start..end` uncached RW+NX, with 2 MiB pages
    /// where a chunk is otherwise unmapped. RAM mappings are never changed.
    pub fn device(&mut self, start: usize, end: usize) -> Result<()> {
        let start = start & !(PAGE - 1);
        let end = end.next_multiple_of(PAGE);
        let mut chunk = start & !(LARGE_PAGE - 1);
        while chunk < end {
            let entry = self.directory_entry(chunk)?;
            let whole = start <= chunk && chunk + LARGE_PAGE <= end;
            // SAFETY: a live directory entry of this builder.
            let current = unsafe { *entry };
            if current == 0 && whole {
                // SAFETY: as above, unused until now.
                unsafe { *entry = chunk as u64 | DEVICE | LARGE };
            } else if current & LARGE != 0 {
                if current & !ADDRESS & !LARGE != DEVICE {
                    return Err(refuse!("device registers overlap mapped RAM"));
                }
            } else {
                if current == 0 {
                    self.small(chunk)?;
                }
                // SAFETY: the entry now names one of this builder's tables.
                let table = unsafe { *entry & ADDRESS };
                if !self.pages.iter().any(|page| page.physical == table) {
                    return Err(refuse!("device table is not owned"));
                }
                for page in (chunk.max(start)..(chunk + LARGE_PAGE).min(end)).step_by(PAGE) {
                    // SAFETY: as above.
                    let slot =
                        unsafe { &mut (*(table as *mut Table))[(page >> 12) & (ENTRIES - 1)] };
                    if *slot != 0 && *slot & !ADDRESS != DEVICE {
                        return Err(refuse!("device registers overlap mapped RAM"));
                    }
                    *slot = page as u64 | DEVICE;
                }
            }
            chunk += LARGE_PAGE;
        }
        Ok(())
    }

    /// Map one 4 KiB page of a chunk that [`Tables::small`] prepared.
    pub fn page(&mut self, address: usize, rights: u64) -> Result<()> {
        let entry = self.directory_entry(address)?;
        // SAFETY: the entry names one of this builder's 4 KiB tables.
        unsafe {
            if *entry & LARGE != 0 || *entry == 0 {
                return Err(refuse!("page outside a 4 KiB chunk"));
            }
            let table = *entry & ADDRESS;
            if !self.pages.iter().any(|page| page.physical == table) {
                return Err(refuse!("leaf table is not owned"));
            }
            (*(table as *mut Table))[(address >> 12) & (ENTRIES - 1)] = address as u64 | rights;
        }
        Ok(())
    }
    fn read_owned(&self, table: u64, index: usize) -> Option<u64> {
        if index >= ENTRIES || !self.pages.iter().any(|page| page.physical == table) {
            return None;
        }
        // SAFETY: original allocation of this builder; caller
        // keeps its current identity mapping across the root transition.
        Some(unsafe { (table as *const u64).add(index).read_volatile() })
    }
    fn observe(
        &self,
        address: u64,
    ) -> Result<
        Option<stela_boot_core::guarded_stacks::mappings::Observation>,
        stela_boot_core::guarded_stacks::mappings::Error,
    > {
        stela_boot_core::guarded_stacks::mappings::observe(
            address,
            self.root(),
            4,
            |table, index| self.read_owned(table, index),
        )
    }
    fn validate_table_aliases(&self) -> Result<(), RootError> {
        for page in &self.pages {
            let physical = page.physical;
            let leaf = self
                .observe(physical)
                .map_err(RootError::Traversal)?
                .ok_or(RootError::AliasMissing { physical })?;
            if leaf.physical != physical || !leaf.writable || leaf.user {
                return Err(RootError::AliasMismatch { physical });
            }
        }
        Ok(())
    }
}

/// Allocations, moved out of the consumed builder. Numeric
/// table roots supplied by a caller cannot construct this private owner.
struct FrameCatalog {
    root: u64,
    pages: Vec<AllocatedTablePage>,
}
impl FrameCatalog {
    fn from_builder(tables: Tables) -> Self {
        Self {
            root: tables.root,
            pages: tables.pages,
        }
    }
    fn read_owned(&self, table: u64, index: usize) -> Option<u64> {
        if index >= ENTRIES || !self.pages.iter().any(|page| page.physical == table) {
            return None;
        }
        // SAFETY: this catalog keeps the original allocated frame;
        // callers establish CPU/current-root lifetime before access.
        Some(unsafe { (table as *const u64).add(index).read_volatile() })
    }
    fn observe(
        &self,
        address: u64,
    ) -> Result<
        Option<stela_boot_core::guarded_stacks::mappings::Observation>,
        stela_boot_core::guarded_stacks::mappings::Error,
    > {
        stela_boot_core::guarded_stacks::mappings::observe(address, self.root, 4, |table, index| {
            self.read_owned(table, index)
        })
    }
    fn validate_table_aliases(&self) -> Result<(), RootError> {
        for page in &self.pages {
            let physical = page.physical;
            let leaf = self
                .observe(physical)
                .map_err(RootError::Traversal)?
                .ok_or(RootError::AliasMissing { physical })?;
            if leaf.physical != physical || !leaf.writable || leaf.user {
                return Err(RootError::AliasMismatch { physical });
            }
        }
        Ok(())
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct Gate {
    low: u16,
    selector: u16,
    ist: u8,
    attributes: u8,
    middle: u16,
    high: u32,
    zero: u32,
}

impl Gate {
    const MISSING: Self = Self::new(0, 0);

    const fn new(handler: usize, selector: u16) -> Self {
        Self {
            low: handler as u16,
            selector,
            ist: 0,
            attributes: if handler == 0 { 0 } else { 0x8e },
            middle: (handler >> 16) as u16,
            high: (handler >> 32) as u32,
            zero: 0,
        }
    }
}

#[repr(C, packed)]
struct Descriptor {
    limit: u16,
    base: u64,
}

static mut IDT: [Gate; 256] = [Gate::MISSING; 256];

/// Terminal handlers: report one letter on the UART, write stop code 0x12 to
/// the hypervisor exit port when CPUID reports a hypervisor
/// ([`stela_boot_core::exit_port`]) and halt. They never enter Rust,
/// allocate, take a lock or resume native code.
macro_rules! fatal_handler {
    ($name:ident, $letter:literal) => {
        #[unsafe(naked)]
        extern "C" fn $name() {
            naked_asm!(
                concat!("mov al, ", $letter),
                "mov dx, 0x3f8",
                "out dx, al",
                "mov eax, 1",
                "cpuid",
                "bt ecx, {hypervisor}",
                "jnc 2f",
                "mov eax, 0x12",
                "mov dx, {port}",
                "out dx, eax",
                "2: cli",
                "hlt",
                "jmp 2b",
                hypervisor = const stela_boot_core::exit_port::HYPERVISOR_BIT,
                port = const stela_boot_core::exit_port::PORT,
            )
        }
    };
}
fatal_handler!(fatal_interrupt, "'I'");
fatal_handler!(fatal_nmi, "'N'");
fatal_handler!(fatal_page_fault, "'P'");

/// The handlers [`report_faults`] installs: each passes its vector and the
/// processor's frame to [`early_fault`] on the interrupted stack.
macro_rules! early_handlers {
    ($($name:ident $vector:literal),*) => {
        $(
            #[unsafe(naked)]
            extern "C" fn $name() {
                naked_asm!(
                    "mov rdi, {vector}",
                    "mov rsi, rsp",
                    "and rsp, -16",
                    "call {report}",
                    "2: cli",
                    "hlt",
                    "jmp 2b",
                    vector = const $vector,
                    report = sym early_fault,
                )
            }
        )*
        const EARLY_HANDLERS: [extern "C" fn(); 32] = [$($name),*];
    };
}
early_handlers!(early_0 0, early_1 1, early_2 2, early_3 3, early_4 4, early_5 5, early_6 6, early_7 7, early_8 8, early_9 9, early_10 10, early_11 11, early_12 12, early_13 13, early_14 14, early_15 15, early_16 16, early_17 17, early_18 18, early_19 19, early_20 20, early_21 21, early_22 22, early_23 23, early_24 24, early_25 25, early_26 26, early_27 27, early_28 28, early_29 29, early_30 30, early_31 31);

static mut EARLY_IDT: [Gate; 32] = [Gate::MISSING; 32];
static EARLY_FAULT: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// An exception between the end of boot services and [`take_over`], where
/// the firmware's own handlers are gone and a fault would otherwise reset
/// the PC with nothing on the screen. A second one, raised while this one
/// is reported, stops where it is.
extern "sysv64" fn early_fault(vector: u64, frame: *const u64) -> ! {
    if EARLY_FAULT.swap(true, core::sync::atomic::Ordering::AcqRel) {
        loop {
            // SAFETY: stops this processor; nothing else runs.
            unsafe { asm!("cli", "hlt", options(nomem, nostack)) };
        }
    }
    // Vectors that push an error code under the return address (Intel SDM
    // Vol. 3A table 6-1).
    let coded = matches!(vector, 8 | 10..=14 | 17 | 21 | 29 | 30);
    // SAFETY: the processor pushed the frame at `frame` on the stack in use.
    let (error, rip) = unsafe {
        if coded {
            (frame.read(), frame.add(1).read())
        } else {
            (0, frame.read())
        }
    };
    let cr2: u64;
    // SAFETY: CPL0 read of the fault address register.
    unsafe { asm!("mov {}, cr2", out(reg) cr2, options(nomem, nostack, preserves_flags)) };
    unsafe extern "C" {
        static __ImageBase: u8;
    }
    crate::output::fatal(format_args!(
        "unexpected x86 exception while taking the PC over from the firmware: vector={vector} \
         error={error:#x} rip={rip:#x} (image+{:#x}) cr2={cr2:#x}",
        rip.wrapping_sub(&raw const __ImageBase as u64),
    ))
}

/// Report the first 32 vectors on the screen until [`take_over`] installs
/// the terminal handlers.
///
/// # Safety
/// Boot services have ended and interrupts are disabled; nothing else runs,
/// and no later code before [`take_over`] enables interrupts.
pub unsafe fn report_faults() {
    let selector: u16;
    // SAFETY: ring 0 after boot services, with interrupts disabled; the
    // table is image data with no other references.
    unsafe {
        asm!("mov {0:x}, cs", out(reg) selector, options(nomem, nostack, preserves_flags));
        let idt = &raw mut EARLY_IDT;
        for (gate, handler) in (*idt).iter_mut().zip(EARLY_HANDLERS) {
            *gate = Gate::new(handler as usize, selector);
        }
        let descriptor = Descriptor {
            limit: (size_of::<[Gate; 32]>() - 1) as u16,
            base: idt as u64,
        };
        asm!("lidt [{}]", in(reg) &descriptor, options(readonly, nostack, preserves_flags));
    }
}

/// Take over the machine: fatal handlers, the controls the runtime requires
/// and the new page tables.
///
/// # Safety
/// Boot services have ended and interrupts are disabled. `tables` maps every
/// byte the code, stack and data in use touch. The current firmware mappings
/// still reach every table allocation until CR3 changes. Nothing else
/// runs or mutates those allocations during this transition.
pub unsafe fn activate(tables: Tables) -> Result<ActivatedTables, RootError> {
    // Original firmware identity mappings still reach the fresh
    // LoaderData allocations until this controlled transition completes.
    active::validate_cpu()?;
    tables.validate_table_aliases()?;
    // SAFETY: the caller's contract.
    unsafe {
        take_over().map_err(RootError::Transition)?;
        asm!("mov cr3, {}", in(reg) tables.root(), options(nostack, preserves_flags));
    }
    ActivatedTables::retain(tables)
}

/// Install the fatal handlers and the controls the runtime requires, still
/// under the firmware's four-level tables.
///
/// # Safety
/// Boot services have ended and interrupts are disabled; nothing else runs.
pub unsafe fn take_over() -> Result<()> {
    let selector: u16;
    let (mut cr0, mut cr4): (u64, u64);
    let efer: u64;
    // SAFETY: ring 0 after boot services, with interrupts disabled.
    unsafe {
        asm!("mov {0:x}, cs", out(reg) selector, options(nomem, nostack, preserves_flags));
        asm!("mov {}, cr0", out(reg) cr0, options(nomem, nostack, preserves_flags));
        asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack, preserves_flags));
        efer = msr::EFER.read();
    }
    if cr4 & CR4_LA57 != 0 {
        return Err(refuse!("five-level paging firmware is unsupported"));
    }
    // SAFETY: the table is image data with no other references.
    unsafe {
        let idt = &raw mut IDT;
        for gate in (*idt).iter_mut() {
            *gate = Gate::new(fatal_interrupt as *const () as usize, selector);
        }
        (*idt)[2] = Gate::new(fatal_nmi as *const () as usize, selector);
        (*idt)[14] = Gate::new(fatal_page_fault as *const () as usize, selector);
        let descriptor = Descriptor {
            limit: (size_of::<[Gate; 256]>() - 1) as u16,
            base: idt as u64,
        };
        asm!("lidt [{}]", in(reg) &descriptor, options(readonly, nostack, preserves_flags));
    }
    cr0 = controls::native_cr0(cr0);
    // CR4.MCE: a machine check reaches the fatal handler installed above,
    // and the engine's after it, instead of shutting the PC down.
    cr4 = (cr4 | CR4_PAE | CR4_OSFXSR | CR4_OSXMMEXCPT | stela_boot_core::machine_check::CR4_MCE)
        & !(CR4_GLOBAL | CR4_PCID);
    let efer = efer | EFER_NX_ENABLE;
    // SAFETY: NX is enabled before tables that use it are loaded; clearing
    // PGE flushes every global firmware translation.
    unsafe {
        msr::EFER.write(efer);
        asm!("mov cr4, {}", in(reg) cr4, options(nostack, preserves_flags));
        asm!("mov cr0, {}", in(reg) cr0, options(nostack, preserves_flags));
    }
    // Make the framebuffer write-combining whatever type the firmware's
    // MTRRs give its range: a write-combining PAT type over an uncached MTRR
    // range is write-combining (SDM vol. 3A, "Memory Type Setting for the
    // PAT and MTRRs"). Otherwise a video BAR that the firmware leaves
    // uncached takes one bus write per pixel. Every x86-64 processor has the
    // PAT. No Stela table uses PWT alone except for the framebuffer, and the
    // firmware's tables, which might, are replaced next. Prepare the shared
    // page window's WC slot here too, before ActivatedTables records PAT;
    // a later memory-type observation must not change that owned policy.
    // The caches are
    // written back on both sides of the change, as SDM vol. 3A "Programming
    // the PAT" orders.
    // SAFETY: ring 0 with interrupts disabled; IA32_PAT is architectural.
    unsafe {
        let pat = stela_boot_core::page_window::with_write_combining(
            msr::PAT.read() & !PAT_ENTRY_1 | PAT_WRITE_COMBINING,
        );
        asm!("wbinvd", options(nostack, preserves_flags));
        msr::PAT.write(pat);
        asm!("wbinvd", options(nostack, preserves_flags));
    }
    Ok(())
}
