//! Owned INIT/SIPI entry, independent of firmware AP stacks and callbacks.
//! The single low page is reused only after the preceding AP has acknowledged
//! entry into permanently pinned high-window code and stopped reading it.
//!
//! Early32 covers privileged setup after its low stack/IDT installation;
//! early64 covers the mapped transition/high-entry prefix until private IDT
//! installation. Real mode/first PE setup and the PG/far-jump/64-IDTR window
//! remain outside this containment. No early IST/task gate is supplied: a
//! corrupt stack, IDT or faulting exception delivery can still double/triple
//! fault. This contains ordinary delivered exceptions, not every CPU shutdown.
use crate::Result;
use core::arch::global_asm;
use stela_contracts::refuse;

use super::memory_types::{Image, MAX_WRITABLE, Write, amd};
use crate::{layout, tables::PAGE};
mod early;
pub(crate) use early::EarlyFault;

global_asm!(
    ".balign 16",
    ".global stela_ap_start",
    "stela_ap_start:",
    ".code16",
    "cli",
    "cld",
    "mov %cs, %ax",
    "mov %ax, %ds",
    "mov %ax, %ss",
    "movl ${page}, %esp",
    "lgdtl (stela_ap_gdtr - stela_ap_start)",
    "movl (stela_ap_base - stela_ap_start), %ebp",
    "mov %cr0, %eax",
    "orl $1, %eax", // PE; remaining controls change under the early32 IDT.
    "mov %eax, %cr0",
    "ljmpl *(stela_ap_far32 - stela_ap_start)",
    ".code32",
    ".global stela_ap_protected",
    "stela_ap_protected:",
    "mov $0x10, %ax",
    "mov %ax, %ds",
    "mov %ax, %es",
    "mov %ax, %ss",
    "lea {page}(%ebp), %esp",
    "lidtl (stela_ap_idtr32 - stela_ap_start)(%ebp)",
    // No shared high memory is reached before the cache/MTRR image. INIT
    // keeps cache contents, so flush under the now-installed early32 IDT.
    "mov %cr0, %eax",
    "andl $0xdfffffff, %eax", // NW=0
    "orl $0x40000000, %eax", // CD=1
    "mov %eax, %cr0",
    "wbinvd",
    "mov (stela_ap_cr4 - stela_ap_start)(%ebp), %eax",
    "mov %eax, %cr4",
    "mov (stela_ap_root - stela_ap_start)(%ebp), %eax",
    "mov %eax, %cr3",
    // The AP has checked long mode/NX/PAT/FXSR/SSE before INIT. It never
    // inherits firmware PCID, CET or unchecked OS XSTATE controls.
    "mov $0xc0000080, %ecx",
    "mov $0x900, %eax", // LME | NXE; LMA is hardware-owned.
    "xor %edx, %edx",
    "wrmsr",
    // The BSP configuration is not changed. This initial, never-admitted
    // AP must match source capacity/physical width before writing any MTRR.
    "mov $0x80000008, %eax",
    "xor %ecx, %ecx",
    "cpuid",
    "cmp (stela_ap_phys_bits - stela_ap_start)(%ebp), %al",
    "jne 90f",
    "mov $0xfe, %ecx",
    "rdmsr",
    "cmp (stela_ap_mtrr_cap - stela_ap_start)(%ebp), %eax",
    "jne 90f",
    "cmp (stela_ap_mtrr_cap - stela_ap_start + 4)(%ebp), %edx",
    "jne 90f",
    // AMD's fixed attribute visibility is separate from effective routing.
    // Recheck this AP's captured identity before touching AMD64 control MSRs.
    "cmpl $0, (stela_ap_amd - stela_ap_start)(%ebp)",
    "je 6f",
    "xor %eax, %eax",
    "cpuid",
    "cmp ${amd_ebx}, %ebx", // AuthenticAMD, independently rechecked after INIT
    "jne 90f",
    "cmp ${amd_edx}, %edx",
    "jne 90f",
    "cmp ${amd_ecx}, %ecx",
    "jne 90f",
    "mov $1, %eax",
    "xor %ecx, %ecx",
    "cpuid",
    "cmp (stela_ap_signature - stela_ap_start)(%ebp), %eax",
    "jne 90f",
    "mov ${amd_syscfg}, %ecx",
    "rdmsr",
    "mov %eax, %esi",
    "xor (stela_ap_syscfg - stela_ap_start)(%ebp), %eax",
    "and ${amd_preserved}, %eax", // only MFDM visibility may differ
    "jnz 90f",
    "cmp (stela_ap_syscfg - stela_ap_start + 4)(%ebp), %edx",
    "jne 90f",
    "testl $0x100, (stela_ap_mtrr_cap - stela_ap_start)(%ebp)",
    "jz 6f", // no fixed attributes to expose without MTRRcap.FIX
    "testl ${amd_fixed_dram}, (stela_ap_syscfg - stela_ap_start)(%ebp)",
    "jz 6f", // RdMem/WrMem have no effect without MtrrFixDramEn
    "mov %esi, %eax",
    "or ${amd_fixed_modify}, %eax", // MFDM: expose actual RdMem/WrMem for writes
    "wrmsr", // preserve every effective routing/non-memory control for now
    "6: mov $0x2ff, %ecx",
    "xor %eax, %eax",
    "xor %edx, %edx",
    "wrmsr", // disabled, UC while the complete image is installed
    "cmpl $0, (stela_ap_amd - stela_ap_start)(%ebp)",
    "jne 3f",
    "lea (stela_ap_mtrrs - stela_ap_start)(%ebp), %esi",
    "mov (stela_ap_mtrr_count - stela_ap_start)(%ebp), %edi",
    "test %edi, %edi",
    "jz 3f",
    "2: mov (%esi), %ecx",
    "wrmsr", // initialize all fields to zero, including invalid pairs
    "add ${mtrr_stride}, %esi",
    "dec %edi",
    "jnz 2b",
    "3: lea (stela_ap_mtrrs - stela_ap_start)(%ebp), %esi",
    "mov (stela_ap_mtrr_count - stela_ap_start)(%ebp), %edi",
    "test %edi, %edi",
    "jz 5f",
    "4: mov (%esi), %ecx",
    // On AMD the legacy hole (0xa0000 to 0xfffff: MTRRfix16K_A0000 and the
    // 4K registers) takes the BSP's types and keeps this processor's own
    // RdMem/WrMem routing, which firmware may set differently on each one
    // and Linux leaves as found. Masked reads give zero there, as the image
    // then holds.
    "cmpl $0, (stela_ap_amd - stela_ap_start)(%ebp)",
    "je 41f",
    "cmp $0x259, %ecx",
    "jb 41f",
    "cmp $0x26f, %ecx",
    "ja 41f",
    "rdmsr",
    "and ${amd_routing}, %eax",
    "and ${amd_routing}, %edx",
    "mov 4(%esi), %ebx",
    "and ${amd_types}, %ebx",
    "or %ebx, %eax",
    "mov 8(%esi), %ebx",
    "and ${amd_types}, %ebx",
    "or %ebx, %edx",
    "jmp 42f",
    "41: mov 4(%esi), %eax",
    "mov 8(%esi), %edx",
    "42: wrmsr",
    "add ${mtrr_stride}, %esi",
    "dec %edi",
    "jnz 4b",
    "5: cmpl $0, (stela_ap_amd - stela_ap_start)(%ebp)",
    "je 7f",
    "mov ${amd_syscfg}, %ecx",
    "rdmsr", // write only a change: a hypervisor may refuse SYSCFG writes
    "cmp (stela_ap_syscfg - stela_ap_start)(%ebp), %eax",
    "jne 8f",
    "cmp (stela_ap_syscfg - stela_ap_start + 4)(%ebp), %edx",
    "je 7f",
    "8: mov (stela_ap_syscfg - stela_ap_start)(%ebp), %eax",
    "mov (stela_ap_syscfg - stela_ap_start + 4)(%ebp), %edx",
    "wrmsr", // complete known memory controls; retain identical opaque fields
    "7: mov $0x277, %ecx",
    "mov (stela_ap_pat - stela_ap_start)(%ebp), %eax",
    "mov (stela_ap_pat - stela_ap_start + 4)(%ebp), %edx",
    "wrmsr",
    "mov $0x2ff, %ecx",
    "mov (stela_ap_mtrr_default - stela_ap_start)(%ebp), %eax",
    "mov (stela_ap_mtrr_default - stela_ap_start + 4)(%ebp), %edx",
    "wrmsr",
    "wbinvd",
    "mov %cr3, %eax",
    "mov %eax, %cr3",
    "mov $0x80010033, %eax", // PG | WP | NE | ET | MP | PE; caches enabled.
    "mov %eax, %cr0",
    "ljmp *(stela_ap_far64 - stela_ap_start)(%ebp)",
    "90: cli",
    "hlt",
    "jmp 90b",
    ".code64",
    ".global stela_ap_long",
    "stela_ap_long:",
    // Exception frames must use the writable physmap alias, not the read-only
    // executable SIPI identity page. No CALL/PUSH precedes this stack switch.
    "mov stela_ap_early_stack(%rip), %rsp",
    "lidt stela_ap_idtr64(%rip)",
    "mov $0x10, %ax",
    "mov %ax, %ds",
    "mov %ax, %es",
    "mov %ax, %ss",
    "xor %eax, %eax",
    "mov %ax, %fs",
    "mov %ax, %gs",
    "mov stela_ap_stack(%rip), %rsp",
    "and $-16, %rsp",
    "xor %ebp, %ebp",
    "fninit",
    "ldmxcsr stela_ap_mxcsr(%rip)",
    "mov stela_ap_argument(%rip), %rdi",
    "mov stela_ap_entry(%rip), %rax",
    "call *%rax",
    // Returning cannot authorize low-page reuse or resource reclamation.
    "1: cli",
    "hlt",
    "jmp 1b",
    include_str!("bootstrap/early.S"),
    ".balign 8",
    ".global stela_ap_gdt",
    "stela_ap_gdt:",
    ".quad 0",
    // Intel SDM3A §3.4.5 / AMD APM2 §4.7.2: loading a descriptor may set
    // Accessed. The identity SIPI page is read-only after CR0.WP/paging;
    // pre-set A so the long-mode CS load needs no hidden low-page write.
    ".quad 0x00af9b000000ffff", // 0x08 long code, Accessed
    ".quad 0x00cf93000000ffff", // 0x10 data, Accessed
    ".quad 0x00cf9b000000ffff", // 0x18 protected code, Accessed
    ".global stela_ap_gdtr",
    "stela_ap_gdtr: .word 31",
    ".long 0",
    ".global stela_ap_far32",
    "stela_ap_far32: .long 0",
    ".word 0x18",
    ".global stela_ap_far64",
    "stela_ap_far64: .long 0",
    ".word 0x08",
    ".balign 8",
    ".global stela_ap_base",
    "stela_ap_base: .long 0",
    ".global stela_ap_cr4",
    "stela_ap_cr4: .long 0",
    ".global stela_ap_root",
    "stela_ap_root: .long 0",
    "stela_ap_mxcsr: .long 0x1f80",
    ".global stela_ap_pat",
    "stela_ap_pat: .quad 0",
    ".global stela_ap_stack",
    "stela_ap_stack: .quad 0",
    ".global stela_ap_argument",
    "stela_ap_argument: .quad 0",
    ".global stela_ap_entry",
    "stela_ap_entry: .quad 0",
    ".balign 4",
    ".global stela_ap_phys_bits",
    "stela_ap_phys_bits: .long 0",
    ".global stela_ap_mtrr_count",
    "stela_ap_mtrr_count: .long 0",
    ".global stela_ap_mtrr_cap",
    "stela_ap_mtrr_cap: .quad 0",
    ".global stela_ap_mtrr_default",
    "stela_ap_mtrr_default: .quad 0",
    ".global stela_ap_amd",
    "stela_ap_amd: .long 0",
    ".global stela_ap_signature",
    "stela_ap_signature: .long 0",
    ".global stela_ap_syscfg",
    "stela_ap_syscfg: .quad 0",
    ".global stela_ap_mtrrs",
    "stela_ap_mtrrs: .zero {mtrr_bytes}",
    ".global stela_ap_end",
    "stela_ap_end:",
    page = const PAGE,
    faults = const early::EXCEPTIONS,
    stub_bytes = const early::STUB_BYTES,
    error_mask = const early::ERROR_MASK,
    fault_writing = const early::WRITING,
    fault_ready = const early::SEALED,
    fault_record_bytes = const core::mem::size_of::<early::Record>(),
    amd_syscfg = const amd::SYSCFG,
    amd_ebx = const amd::VENDOR[0],
    amd_edx = const amd::VENDOR[1],
    amd_ecx = const amd::VENDOR[2],
    amd_fixed_modify = const amd::FIXED_MODIFY,
    amd_fixed_dram = const amd::FIXED_DRAM,
    amd_preserved = const !(amd::FIXED_MODIFY as u32),
    amd_routing = const amd::FIXED_ROUTING as u32,
    amd_types = const !(amd::FIXED_ROUTING as u32),
    mtrr_stride = const core::mem::size_of::<Write>(),
    mtrr_bytes = const MAX_WRITABLE * core::mem::size_of::<Write>(),
    options(att_syntax),
);

unsafe extern "C" {
    static stela_ap_start: u8;
    static stela_ap_stubs32: u8;
    static stela_ap_stubs64: u8;
    static stela_ap_idt32: u8;
    static stela_ap_idt64: u8;
    static stela_ap_idtr32: u8;
    static stela_ap_idtr64: u8;
    static stela_ap_early_stack: u8;
    static stela_ap_fault_record: u8;
    static stela_ap_fault_alias: u8;
    static stela_ap_end: u8;
    static stela_ap_protected: u8;
    static stela_ap_long: u8;
    static stela_ap_gdt: u8;
    static stela_ap_gdtr: u8;
    static stela_ap_far32: u8;
    static stela_ap_far64: u8;
    static stela_ap_base: u8;
    static stela_ap_cr4: u8;
    static stela_ap_root: u8;
    static stela_ap_pat: u8;
    static stela_ap_stack: u8;
    static stela_ap_argument: u8;
    static stela_ap_entry: u8;
    static stela_ap_phys_bits: u8;
    static stela_ap_mtrr_count: u8;
    static stela_ap_mtrr_cap: u8;
    static stela_ap_mtrr_default: u8;
    static stela_ap_mtrrs: u8;
    static stela_ap_amd: u8;
    static stela_ap_signature: u8;
    static stela_ap_syscfg: u8;
}

fn offset(symbol: *const u8) -> usize {
    // SAFETY: each caller supplies a symbol inside this single assembly block.
    unsafe { symbol.offset_from(&raw const stela_ap_start) as usize }
}

/// The exception stubs, IDTs and fault record lie inside the copied image,
/// and the record is aligned for its sealed 8-byte fields.
fn image_holds_its_tables(bytes: usize) -> bool {
    use early::{EXCEPTIONS, Record, STUB_BYTES};
    let record = offset(&raw const stela_ap_fault_record);
    record.is_multiple_of(8)
        && [
            (offset(&raw const stela_ap_stubs32), EXCEPTIONS * STUB_BYTES),
            (offset(&raw const stela_ap_stubs64), EXCEPTIONS * STUB_BYTES),
            (offset(&raw const stela_ap_idt32), EXCEPTIONS * 8),
            (offset(&raw const stela_ap_idt64), EXCEPTIONS * 16),
            (record, core::mem::size_of::<Record>()),
        ]
        .into_iter()
        .all(|(table, size)| table + size <= bytes)
}

pub(crate) struct Entry<'a> {
    /// Identity-mapped paging root, below 4 GiB, containing the low page and
    /// the permanently pinned high-window entry/stack/argument mappings.
    pub root: u64,
    pub five_level: bool,
    /// This selected physical AP's own immutable firmware CPUID1 EAX, not
    /// the BSP's signature or a model name selected by the ARM guest profile.
    pub processor_signature: u32,
    /// The BSP's installed PAT, checked against each AP's memory-type state.
    pub pat: u64,
    pub memory_types: &'a Image,
    pub stack_top: u64,
    pub argument: u64,
    pub entry: unsafe extern "sysv64" fn(u64) -> !,
}

/// Copy and patch the low page without using a firmware callback or stack.
///
/// # Safety
/// `mapped` is the exclusive writable alias of one owned page at `physical`.
/// No AP is reading that page, including a timed-out prior startup. The page,
/// root, entry, stack and argument mappings remain live through AP entry and
/// recovery; the caller has checked this AP's features and memory types.
/// The low identity alias is executable and the high stack is writable/NX.
pub(crate) unsafe fn prepare(mapped: *mut u8, physical: u64, entry: Entry<'_>) -> Result<u8> {
    let bytes = offset(&raw const stela_ap_end);
    if !image_holds_its_tables(bytes) {
        return Err(refuse!("AP bootstrap tables lie outside its image"));
    }
    if !(PAGE..1 << 20).contains(&physical)
        || !physical.is_multiple_of(PAGE)
        || bytes > PAGE as usize - early::STACK_BYTES
        || entry.memory_types.count > MAX_WRITABLE
        || entry.root == 0
        || entry.root >= 1 << 32
        || !entry.root.is_multiple_of(PAGE)
        || entry.stack_top == 0
        || entry.stack_top & 15 != 0
    {
        return Err(refuse!("invalid owned AP bootstrap mappings"));
    }
    // SAFETY: exclusive owned destination and immutable assembly text. All
    // patch symbols lie inside `bytes`, which fits in the supplied page.
    unsafe {
        core::ptr::copy_nonoverlapping(&raw const stela_ap_start, mapped, bytes);
        let put32 = |symbol, value: u32| {
            mapped
                .add(offset(symbol))
                .cast::<u32>()
                .write_unaligned(value)
        };
        let put64 = |symbol, value: u64| {
            mapped
                .add(offset(symbol))
                .cast::<u64>()
                .write_unaligned(value)
        };
        put32(&raw const stela_ap_base, physical as u32);
        put64(
            &raw const stela_ap_early_stack,
            layout::physical(physical + PAGE),
        );
        put64(
            &raw const stela_ap_fault_alias,
            layout::physical(physical + offset(&raw const stela_ap_fault_record) as u64),
        );
        mapped
            .add(offset(&raw const stela_ap_idtr32) + 2)
            .cast::<u32>()
            .write_unaligned((physical + offset(&raw const stela_ap_idt32) as u64) as u32);
        mapped
            .add(offset(&raw const stela_ap_idtr64) + 2)
            .cast::<u64>()
            .write_unaligned(physical + offset(&raw const stela_ap_idt64) as u64);
        for vector in 0..early::EXCEPTIONS {
            let handler32 = physical
                + offset(&raw const stela_ap_stubs32) as u64
                + (vector * early::STUB_BYTES) as u64;
            let handler64 = physical
                + offset(&raw const stela_ap_stubs64) as u64
                + (vector * early::STUB_BYTES) as u64;
            let gate32 = early::gate32(handler32 as u32);
            let gate64 = early::gate64(handler64);
            core::ptr::copy_nonoverlapping(
                gate32.as_ptr(),
                mapped.add(offset(&raw const stela_ap_idt32) + vector * gate32.len()),
                gate32.len(),
            );
            core::ptr::copy_nonoverlapping(
                gate64.as_ptr(),
                mapped.add(offset(&raw const stela_ap_idt64) + vector * gate64.len()),
                gate64.len(),
            );
        }
        put32(&raw const stela_ap_root, entry.root as u32);
        // PAE, MCE, OSFXSR and OSXMMEXCPT: a machine check from here on
        // reaches the early IDT installed before this write, then the
        // engine's, instead of shutting the PC down.
        put32(
            &raw const stela_ap_cr4,
            0x620
                | stela_boot_core::machine_check::CR4_MCE as u32
                | if entry.five_level { 1 << 12 } else { 0 },
        );
        put32(
            &raw const stela_ap_far32,
            (physical + offset(&raw const stela_ap_protected) as u64) as u32,
        );
        put32(
            &raw const stela_ap_far64,
            (physical + offset(&raw const stela_ap_long) as u64) as u32,
        );
        mapped
            .add(offset(&raw const stela_ap_gdtr) + 2)
            .cast::<u32>()
            .write_unaligned((physical + offset(&raw const stela_ap_gdt) as u64) as u32);
        put32(
            &raw const stela_ap_amd,
            u32::from(entry.memory_types.amd.is_some()),
        );
        put32(&raw const stela_ap_signature, entry.processor_signature);
        put64(
            &raw const stela_ap_syscfg,
            entry.memory_types.amd.map_or(0, |amd| amd.syscfg),
        );
        put32(
            &raw const stela_ap_phys_bits,
            u32::from(entry.memory_types.physical_bits),
        );
        put32(
            &raw const stela_ap_mtrr_count,
            entry.memory_types.count as u32,
        );
        put64(&raw const stela_ap_mtrr_cap, entry.memory_types.cap);
        put64(&raw const stela_ap_mtrr_default, entry.memory_types.default);
        core::ptr::copy_nonoverlapping(
            entry.memory_types.writes.as_ptr(),
            mapped
                .add(offset(&raw const stela_ap_mtrrs))
                .cast::<Write>(),
            entry.memory_types.count,
        );
        put64(&raw const stela_ap_pat, entry.pat);
        put64(&raw const stela_ap_stack, entry.stack_top);
        put64(&raw const stela_ap_argument, entry.argument);
        put64(&raw const stela_ap_entry, entry.entry as *const () as u64);
    }
    // Serializes the current core's stores before APIC sends; the AP's INIT/
    // SIPI fetch supplies its architectural startup serialization separately.
    core::arch::x86_64::__cpuid(0);
    // SAFETY: initial executor bootstrap only, before any guest/native loan;
    // caller owns the system's unchanged shared memory-type configuration.
    unsafe {
        core::arch::asm!("wbinvd", options(nostack, preserves_flags));
    }
    Ok((physical / PAGE) as u8)
}

/// Observe only a sealed early AP failure; an empty/unfinished record
/// never supplies READY or permission to reuse the low page.
///
/// # Safety
/// `mapped` is the owned SIPI page, prepared for this one AP.
pub(crate) unsafe fn failure(mapped: *const u8) -> Result<Option<EarlyFault>, early::RecordError> {
    // SAFETY: the caller keeps this AP's owned SIPI page, whose assembly
    // publishes the aligned record once and leaves its sealed payload immutable.
    unsafe { early::read(mapped.add(offset(&raw const stela_ap_fault_record)).cast()) }
}
