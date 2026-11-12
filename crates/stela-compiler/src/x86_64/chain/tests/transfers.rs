//! What a transfer between translated blocks costs on the host: guest
//! programs compiled as the system-mode engine compiles EL0 blocks, laid
//! out in one arena and linked as the engine links them, then run under
//! the host's single-step trap. Each executed host instruction is counted
//! once, and its memory operand classed as a load, a store or a
//! read-modify-write, so a change to the transfer path shows here as
//! numbers, not as a reading of its bytes.
use super::super::*;
use crate::{
    GuestState, Options, TargetCpu,
    block::{BlockProfile, CompilationPlan, Features},
    state::Limit,
    x86_64::{MemoryChecks, NativeSystem, RangeCheck, pinned},
};
use alloc::{vec, vec::Vec};
use core::ffi::c_void;
use std::sync::{
    Mutex,
    atomic::{AtomicU64, AtomicUsize, Ordering},
};

unsafe extern "C" {
    fn mmap(
        address: *mut c_void,
        bytes: usize,
        protection: i32,
        flags: i32,
        fd: i32,
        offset: i64,
    ) -> *mut c_void;
    fn munmap(address: *mut c_void, bytes: usize) -> i32;
    fn sigaction(signal: i32, action: *const SigAction, old: *mut SigAction) -> i32;
}

/// Linux x86-64 `struct sigaction`, as glibc's wrapper takes it.
#[repr(C)]
struct SigAction {
    handler: usize,
    mask: [u64; 16],
    flags: i32,
    restorer: usize,
}

const SIGTRAP: i32 = 5;
const SA_SIGINFO: i32 = 4;
/// `ucontext_t.uc_mcontext.gregs[REG_RIP]` on Linux x86-64.
const CONTEXT_RIP: usize = 40 + 16 * 8;
/// EFLAGS.TF.
const TRAP_FLAG: u64 = 1 << 8;

const ARENA_BYTES: usize = 1 << 16;
/// Where the entry stub goes, and where blocks start after it.
const STUB_AT: usize = 0;
const BLOCKS_AT: usize = 0x200;
/// Blocks start at this alignment, which keeps each patch point where the
/// compiler aligned it (code installs at 8-byte aligned addresses).
const BLOCK_ALIGN: usize = 64;
const PAGE: u64 = 4096;
/// The guest page programs run from.
const BASE: u64 = 0x40_0000;

/// The most host instructions one trace records.
const TRACE_WORDS: usize = 1 << 20;
static TRACE: Mutex<()> = Mutex::new(());
static STEPS: AtomicUsize = AtomicUsize::new(0);
static RIPS: [AtomicU64; TRACE_WORDS] = [const { AtomicU64::new(0) }; TRACE_WORDS];

extern "C" fn stepped(_signal: i32, _info: *mut c_void, context: *mut c_void) {
    // SAFETY: the kernel passes the interrupted thread's ucontext, whose
    // general registers hold RIP at this offset on Linux x86-64.
    let rip = unsafe { context.cast::<u8>().add(CONTEXT_RIP).cast::<u64>().read() };
    let step = STEPS.fetch_add(1, Ordering::Relaxed);
    if let Some(word) = RIPS.get(step) {
        word.store(rip, Ordering::Relaxed);
    }
}

/// How one host instruction touches memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Access {
    None,
    Load,
    Store,
    Modify,
    /// PUSH, POP, CALL and RET.
    Stack,
}

/// The legacy one-byte opcodes that take a ModRM byte.
fn one_byte_modrm(opcode: u8) -> bool {
    matches!(opcode,
        0x00..=0x03 | 0x08..=0x0b | 0x10..=0x13 | 0x18..=0x1b
        | 0x20..=0x23 | 0x28..=0x2b | 0x30..=0x33 | 0x38..=0x3b
        | 0x63 | 0x69 | 0x6b | 0x80..=0x8f | 0xc0 | 0xc1 | 0xc6 | 0xc7
        | 0xd0..=0xd3 | 0xd8..=0xdf | 0xf6 | 0xf7 | 0xfe | 0xff)
}

/// The two-byte (0F) opcodes without a ModRM byte.
fn two_byte_plain(opcode: u8) -> bool {
    matches!(opcode,
        0x05..=0x09 | 0x0b | 0x30..=0x35 | 0x77 | 0x80..=0x8f | 0xa0..=0xa2
        | 0xa8 | 0xa9 | 0xc8..=0xcf)
}

/// How the instruction at the start of `bytes` touches memory: enough of
/// the x86 encoding for the code blocks and their transfers emit.
fn access(bytes: &[u8]) -> Access {
    let mut at = 0;
    while matches!(
        bytes[at],
        0x66 | 0x67 | 0xf0 | 0xf2 | 0xf3 | 0x2e | 0x3e | 0x26 | 0x36 | 0x64 | 0x65
    ) {
        at += 1;
    }
    if bytes[at] & 0xf0 == 0x40 {
        at += 1;
    }
    let memory = |modrm: u8| modrm >> 6 != 3;
    let (map, opcode) = match bytes[at] {
        // VEX and EVEX: every form here has a ModRM byte.
        0xc5 => (1, bytes[at + 2]),
        0xc4 => (bytes[at + 1] & 0x1f, bytes[at + 3]),
        0x62 => (bytes[at + 1] & 0x7, bytes[at + 4]),
        0x0f => match bytes[at + 1] {
            0x38 => (2, bytes[at + 2]),
            0x3a => (3, bytes[at + 2]),
            opcode => (1, opcode),
        },
        opcode => (0, opcode),
    };
    let modrm_at = match bytes[at] {
        0xc5 => at + 3,
        0xc4 => at + 4,
        0x62 => at + 5,
        0x0f if map == 1 => at + 2,
        0x0f => at + 3,
        _ => at + 1,
    };
    let vector = matches!(bytes[at], 0xc4 | 0xc5 | 0x62);
    if map == 0 {
        if matches!(opcode, 0x50..=0x5f | 0xc3 | 0xe8 | 0x9c | 0x9d) {
            return Access::Stack;
        }
        if !one_byte_modrm(opcode) {
            return Access::None;
        }
        let modrm = bytes[modrm_at];
        let reg = (modrm >> 3) & 7;
        if opcode == 0xff && matches!(reg, 2 | 6) || opcode == 0x8f {
            return Access::Stack;
        }
        if !memory(modrm) || opcode == 0x8d {
            return Access::None;
        }
        return match opcode {
            0x88 | 0x89 | 0xc6 | 0xc7 => Access::Store,
            0x38..=0x3b | 0x84 | 0x85 | 0x8a | 0x8b | 0x63 | 0x69 | 0x6b => Access::Load,
            0x02 | 0x03 | 0x0a | 0x0b | 0x12 | 0x13 | 0x1a | 0x1b | 0x22 | 0x23 | 0x2a | 0x2b
            | 0x32 | 0x33 => Access::Load,
            0x80..=0x83 if reg == 7 => Access::Load,
            0xf6 | 0xf7 if !(2..=3).contains(&reg) => Access::Load,
            0xff if reg == 4 => Access::Load,
            0xd8..=0xdf => Access::Load,
            _ => Access::Modify,
        };
    }
    if map == 1 && !vector && two_byte_plain(opcode) {
        return Access::None;
    }
    let modrm = bytes[modrm_at];
    if !memory(modrm) || (map == 1 && !vector && matches!(opcode, 0x18 | 0x1f | 0x0d)) {
        return Access::None;
    }
    match (map, opcode) {
        (1, 0x11 | 0x13 | 0x17 | 0x29 | 0x2b | 0x7e | 0x7f | 0xd6 | 0xe7 | 0xc3) => Access::Store,
        (1, 0x90..=0x9f) => Access::Store,
        (1, 0xb0 | 0xb1 | 0xc0 | 0xc1 | 0xab | 0xb3 | 0xbb) if !vector => Access::Modify,
        _ => Access::Load,
    }
}

/// Host instructions, by how they touch memory.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(super) struct Cost {
    pub instructions: usize,
    pub loads: usize,
    pub stores: usize,
    pub modifies: usize,
}

impl Cost {
    fn add(&mut self, access: Access) {
        self.instructions += 1;
        match access {
            Access::Load | Access::Stack => self.loads += 1,
            Access::Store => self.stores += 1,
            Access::Modify => self.modifies += 1,
            Access::None => {}
        }
    }
}

/// One mapping of RWX memory, the arena.
struct Arena(*mut u8);

impl Arena {
    fn new() -> Self {
        // SAFETY: private anonymous memory this value owns until Drop.
        let base = unsafe { mmap(core::ptr::null_mut(), ARENA_BYTES, 7, 0x22, -1, 0) };
        assert_ne!(base as usize, usize::MAX);
        Self(base.cast())
    }
    fn at(&self, offset: usize) -> u64 {
        self.0 as u64 + offset as u64
    }
    fn write(&self, offset: usize, bytes: &[u8]) {
        assert!(offset + bytes.len() <= ARENA_BYTES);
        // SAFETY: within the owned mapping; nothing runs while it is written.
        unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), self.0.add(offset), bytes.len()) };
    }
    fn bytes(&self, address: u64) -> &[u8] {
        let offset = (address - self.at(0)) as usize;
        // SAFETY: within the owned mapping, which nothing writes while read.
        unsafe { core::slice::from_raw_parts(self.0.add(offset), ARENA_BYTES - offset) }
    }
}

impl Drop for Arena {
    fn drop(&mut self) {
        // SAFETY: the owned mapping; every run has returned.
        assert_eq!(unsafe { munmap(self.0.cast(), ARENA_BYTES) }, 0);
    }
}

/// One compiled block in the arena.
pub(super) struct Placed {
    pub pc: u64,
    pub at: u64,
    pub instructions: u32,
    pub self_loop: bool,
}

/// A guest program's blocks, compiled and linked as the engine does.
pub(super) struct Program {
    arena: Arena,
    pub blocks: Vec<Placed>,
    pub control: Box<ChainControl>,
}

/// The options the system-mode engine compiles EL0 blocks with on a host
/// of `target`.
fn options(target: TargetCpu, self_loops: bool) -> Options {
    let checks = MemoryChecks::system(true, false, true, true)
        .with_range(RangeCheck::FIVE_LEVEL.one_sided(true));
    Options::new(BlockProfile::new(Features::SYSTEM_MODE), target)
        .with_memory_checks(checks)
        .with_native_system(NativeSystem {
            exclusive: true,
            self_loops,
            ..NativeSystem::NONE
        })
}

impl Program {
    /// `words` at [`BASE`], entered at word `entry`: every block a run from
    /// there may reach in the page, each direct exit in the page linked
    /// through its slot.
    pub(super) fn new(words: &[u32], entry: usize, self_loops: bool) -> Self {
        Self::with_target(words, entry, self_loops, TargetCpu::Baseline)
    }

    pub(super) fn with_target(
        words: &[u32],
        entry: usize,
        self_loops: bool,
        target: TargetCpu,
    ) -> Self {
        let source: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let options = options(target, self_loops);
        let arena = Arena::new();
        let mut blocks: Vec<Placed> = Vec::new();
        let mut exits = Vec::new();
        let mut pending = vec![BASE + 4 * entry as u64];
        let mut next = BLOCKS_AT;
        while let Some(pc) = pending.pop() {
            if blocks.iter().any(|block| block.pc == pc) {
                continue;
            }
            let start = (pc - BASE) as usize;
            let compiled = CompilationPlan::new(&source[start..], GuestAddress(pc), &options)
                .and_then(CompilationPlan::compile)
                .expect("a compiled block");
            let code = &compiled.native.x86_64_code;
            arena.write(next, code);
            for exit in &compiled.direct_exits {
                let in_page = (exit.target.0 ^ pc) & !(PAGE - 1) == 0;
                let in_program = (exit.target.0 - BASE) / 4 < words.len() as u64;
                if in_page && in_program {
                    pending.push(exit.target.0);
                }
            }
            exits.push(compiled.direct_exits.clone());
            blocks.push(Placed {
                pc,
                at: arena.at(next),
                instructions: (compiled.source_bytes / 4) as u32,
                self_loop: compiled.self_loop,
            });
            next = (next + code.len()).next_multiple_of(BLOCK_ALIGN);
        }
        for (index, block) in blocks.iter().enumerate() {
            for exit in &exits[index] {
                let Some(target) = blocks.iter().position(|other| other.pc == exit.target.0) else {
                    continue;
                };
                let slot = link_slot(Some(block.pc), exit.target, exit.kind);
                if slot == 0 {
                    continue;
                }
                let at = block.at + exit.offset as u64 - slot as u64;
                let retired = block.instructions - u32::from(exit.skipped);
                let (body, bytes) = link(at, slot, retired, target as u32, blocks[target].at)
                    .expect("a link in reach");
                let offset = |address: u64| (address - arena.at(0)) as usize;
                arena.write(offset(body), &bytes);
                let point = slot_patch(at);
                assert!(patchable(point));
                arena.write(offset(point), &NOP5);
            }
        }
        // The entry: the pinned registers loaded around a jump to the
        // first block.
        let mut stub = pinned::entered(&[0xe9, 0, 0, 0, 0]);
        let end = arena.at(STUB_AT + stub.len());
        let distance = relative(end, blocks[0].at).expect("the first block in reach");
        let field = stub.len() - 4;
        stub[field..].copy_from_slice(&distance.to_le_bytes());
        assert!(stub.len() <= BLOCKS_AT);
        arena.write(STUB_AT, &stub);
        let control = Box::new(ChainControl {
            budget: 1 << 20,
            ..ChainControl::default()
        });
        Self {
            arena,
            blocks,
            control,
        }
    }

    /// A frame for a run: registers from `x`, every access in range.
    pub(super) fn frame(&mut self, x: &[(usize, u64)]) -> Box<GuestState> {
        let mut frame = Box::new(GuestState {
            lower_limit: Limit(0x7fff_ffff_ffff),
            ..GuestState::default()
        });
        for &(register, value) in x {
            frame.x[register] = value;
        }
        frame.anchors[Anchor::Control] = (&mut *self.control as *mut ChainControl) as u64;
        frame
    }

    /// Run from the entry block: RAX and RDX the last exit returned.
    pub(super) fn run(&mut self, frame: &mut GuestState) -> (u64, u64) {
        let entry = self.arena.at(STUB_AT);
        let (rax, rdx): (u64, u64);
        // SAFETY: the stub keeps the callee-saved registers and returns
        // through the blocks' exits with the System V return registers;
        // the blocks touch only the frame, the control and the guest
        // memory the caller gave in the frame's registers.
        unsafe {
            core::arch::asm!(
                "call {entry}",
                entry = in(reg) entry,
                in("rdi") frame as *mut GuestState,
                lateout("rax") rax,
                lateout("rdx") rdx,
                clobber_abi("sysv64"),
            );
        }
        (rax, rdx)
    }

    /// [`Self::run`] under the single-step trap: the address of each host
    /// instruction it ran in the arena, in order.
    pub(super) fn trace(&mut self, frame: &mut GuestState) -> ((u64, u64), Vec<u64>) {
        let _held = TRACE.lock().unwrap_or_else(|poison| poison.into_inner());
        let action = SigAction {
            handler: stepped as *const () as usize,
            mask: [0; 16],
            flags: SA_SIGINFO,
            restorer: 0,
        };
        let mut old = SigAction {
            handler: 0,
            mask: [0; 16],
            flags: 0,
            restorer: 0,
        };
        // SAFETY: a handler that only reads its context and writes the
        // trace buffer, installed for this trace and restored after it.
        assert_eq!(unsafe { sigaction(SIGTRAP, &action, &mut old) }, 0);
        STEPS.store(0, Ordering::Relaxed);
        let entry = self.arena.at(STUB_AT);
        let (rax, rdx): (u64, u64);
        // SAFETY: as for `run`; TF is set for the call only, and every
        // trap it raises goes to `stepped`.
        unsafe {
            core::arch::asm!(
                "pushfq",
                "or qword ptr [rsp], {flag}",
                "popfq",
                "call {entry}",
                "pushfq",
                "and qword ptr [rsp], {clear}",
                "popfq",
                flag = const TRAP_FLAG,
                clear = const !TRAP_FLAG,
                entry = in(reg) entry,
                in("rdi") frame as *mut GuestState,
                lateout("rax") rax,
                lateout("rdx") rdx,
                clobber_abi("sysv64"),
            );
        }
        // SAFETY: the handler this trace replaced goes back.
        let restored = unsafe { sigaction(SIGTRAP, &old, core::ptr::null_mut()) };
        assert_eq!(restored, 0);
        let steps = STEPS.load(Ordering::Relaxed);
        assert!(steps < TRACE_WORDS, "a trace within its buffer");
        let (low, high) = (self.arena.at(BLOCKS_AT), self.arena.at(ARENA_BYTES));
        let rips = RIPS[..steps]
            .iter()
            .map(|word| word.load(Ordering::Relaxed))
            .filter(|rip| (low..high).contains(rip))
            .collect();
        ((rax, rdx), rips)
    }

    /// The block that starts at guest word `word`.
    pub(super) fn block(&self, word: usize) -> usize {
        let pc = BASE + 4 * word as u64;
        self.blocks
            .iter()
            .position(|block| block.pc == pc)
            .expect("a block there")
    }

    /// What the instructions at `rips` cost.
    pub(super) fn cost(&self, rips: &[u64]) -> Cost {
        let mut cost = Cost::default();
        for &rip in rips {
            cost.add(access(self.arena.bytes(rip)));
        }
        cost
    }
}

/// The host instructions a run of `words` from word `entry` with `x` ran
/// from the entry block's start up to the first instruction of the block
/// at word `to`.
fn transfer(words: &[u32], entry: usize, to: usize, x: &[(usize, u64)]) -> Cost {
    let mut program = Program::new(words, entry, false);
    let mut frame = program.frame(x);
    let (_, rips) = program.trace(&mut frame);
    let target = program.blocks[program.block(to)].at;
    let end = rips
        .iter()
        .position(|&rip| rip == target)
        .expect("the run entered the target");
    program.cost(&rips[..end])
}

const BRK: u32 = 0xd420_0000;
const NOP: u32 = 0xd503_201f;
/// SUB X0, X0, #1.
const SUB_X0: u32 = 0xd100_0400;

fn b(words: i32) -> u32 {
    0x1400_0000 | (words as u32 & 0x03ff_ffff)
}
/// CBZ or CBNZ X0, by words.
fn cbz_x0(nonzero: bool, words: i32) -> u32 {
    0xb400_0000 | u32::from(nonzero) << 24 | ((words as u32 & 0x7ffff) << 5)
}

impl Cost {
    const fn new(instructions: usize, loads: usize, stores: usize, modifies: usize) -> Self {
        Self {
            instructions,
            loads,
            stores,
            modifies,
        }
    }
}

/// The host cost of each transfer kind. Each is a run from a block holding
/// only the branch (and for a side exit, the instructions after it) to the
/// next block's first instruction: a conditional branch includes its own
/// test and jump, two instructions.
pub(super) fn transfer_costs() -> Vec<(&'static str, Cost)> {
    let taken = [(0, 0)];
    let untaken = [(0, 1)];
    vec![
        // B forward in the page: a link without the interrupt check.
        ("b forward", transfer(&[b(2), BRK, BRK], 0, 2, &[])),
        // B back in the page: a polled link.
        ("b back", transfer(&[BRK, b(-1)], 1, 0, &[])),
        // CBZ back, not taken: its fallthrough exit's link.
        (
            "cbz back, fallthrough",
            transfer(&[BRK, cbz_x0(false, -1), BRK], 1, 2, &untaken),
        ),
        // CBZ back, taken: a polled link.
        (
            "cbz back, taken",
            transfer(&[BRK, cbz_x0(false, -1), BRK], 1, 0, &taken),
        ),
        // CBZ forward, taken: the side exit of a branch the block runs past.
        (
            "cbz forward, side exit",
            transfer(&[cbz_x0(false, 3), NOP, BRK, BRK], 0, 3, &taken),
        ),
    ]
}

/// Host instructions per iteration of `sub x0,x0,#1; cbnz x0, back`, run
/// inside its block (`self_loops`) or through its polled link.
pub(super) fn loop_iteration(self_loops: bool) -> Cost {
    let words = [SUB_X0, cbz_x0(true, -1), BRK];
    let run = |iterations: u64| {
        let mut program = Program::new(&words, 0, self_loops);
        assert_eq!(program.blocks[0].self_loop, self_loops);
        let mut frame = program.frame(&[(0, iterations)]);
        let (_, rips) = program.trace(&mut frame);
        let end = program.blocks[program.block(2)].at;
        let end = rips
            .iter()
            .position(|&rip| rip == end)
            .expect("the loop ended");
        program.cost(&rips[..end])
    };
    let (short, long) = (run(10), run(30));
    let per = |long: usize, short: usize| (long - short) / 20;
    Cost::new(
        per(long.instructions, short.instructions),
        per(long.loads, short.loads),
        per(long.stores, short.stores),
        per(long.modifies, short.modifies),
    )
}

/// A text scanner of the shape hot integer loops have: blocks of two to
/// five instructions, each byte through two to four of them by forward
/// and backward branches and side exits. X11 walks the text and X14
/// counts its bytes down; X9 counts words, X12 newlines, W10 is set inside
/// a word.
const SCANNER: [u32; 16] = [
    0x3840_156d, // 0: ldrb w13, [x11], #1
    0x7100_81bf, // 1: cmp w13, #32
    0x5400_00c9, // 2: b.ls 8
    0x3400_004a, // 3: cbz w10, 5
    0x1400_0009, // 4: b 13
    0x5280_002a, // 5: mov w10, #1
    0x9100_0529, // 6: add x9, x9, #1
    0x1400_0006, // 7: b 13
    0x7100_29bf, // 8: cmp w13, #10
    0x5400_0041, // 9: b.ne 11
    0x9100_058c, // 10: add x12, x12, #1
    0x5280_000a, // 11: mov w10, #0
    0x1400_0001, // 12: b 13
    0xd100_05ce, // 13: sub x14, x14, #1
    0xb5ff_fe4e, // 14: cbnz x14, 0
    BRK,         // 15
];

/// Words of one to eight letters, a space or a newline after each.
fn text(bytes: usize) -> Vec<u8> {
    let mut seed = 0x9e37_79b9_u32;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    let mut text = Vec::with_capacity(bytes);
    while text.len() < bytes {
        for _ in 0..1 + next() % 8 {
            text.push(b'a' + (next() % 26) as u8);
        }
        text.push(if next().is_multiple_of(8) {
            b'\n'
        } else {
            b' '
        });
    }
    text.truncate(bytes);
    text
}

/// What [`SCANNER`] counts in `text`: words, newlines, the guest
/// instructions it retires up to its BRK and the transfers between its
/// blocks.
fn scanned(text: &[u8]) -> (u64, u64, u64, u64) {
    let (mut words, mut lines, mut retired, mut transfers) = (0, 0, 0, 0);
    let mut inside = false;
    for &byte in text {
        let (instructions, blocks) = if byte > b' ' {
            if inside {
                (7, 2)
            } else {
                inside = true;
                words += 1;
                (9, 3)
            }
        } else {
            inside = false;
            if byte == b'\n' {
                lines += 1;
                (10, 3)
            } else {
                (9, 4)
            }
        };
        retired += instructions;
        transfers += blocks;
    }
    (words, lines, retired, transfers)
}

/// A frame that runs [`SCANNER`] over `text`.
fn scanner_frame(program: &mut Program, text: &[u8]) -> Box<GuestState> {
    program.frame(&[(11, text.as_ptr() as u64), (14, text.len() as u64)])
}

/// The decoder classes the forms transfers emit.
#[test]
fn the_cost_decoder_classes_transfer_instructions() {
    for (bytes, expected) in [
        (&[0x48, 0x8b, 0x8f, 0, 1, 0, 0][..], Access::Load),
        (&[0x48, 0x83, 0x29, 0x01], Access::Modify),
        (&[0x48, 0x81, 0x41, 0x08, 7, 0, 0, 0], Access::Modify),
        (&[0x48, 0xc7, 0x41, 0x10, 3, 0, 0, 0], Access::Store),
        (&[0x48, 0x3b, 0x51, 0x48], Access::Load),
        (&NOP5[..], Access::None),
        (&[0x0f, 0x1f, 0x00], Access::None),
        (&[0x72, 0x10], Access::None),
        (&[0xe9, 0, 0, 0, 0], Access::None),
        (&[0x48, 0xb8, 0, 0, 0, 0, 0, 0, 0, 0], Access::None),
        (&[0xc3], Access::Stack),
        (&[0x4d, 0x85, 0xff], Access::None),
        (&[0x48, 0x8d, 0x41, 0x08], Access::None),
        (&[0x80, 0xbf, 0x01, 1, 0, 0, 0], Access::Load),
    ] {
        assert_eq!(access(bytes), expected, "{bytes:02x?}");
    }
}

/// Each transfer kind costs what the transfer path the compiler emits now
/// runs: a change to that path changes this table.
#[test]
fn transfers_cost_what_their_path_runs() {
    let costs = transfer_costs();
    let expected = [
        ("b forward", Cost::new(7, 1, 1, 2)),
        ("b back", Cost::new(10, 3, 1, 2)),
        ("cbz back, fallthrough", Cost::new(9, 1, 1, 2)),
        ("cbz back, taken", Cost::new(13, 3, 1, 2)),
        ("cbz forward, side exit", Cost::new(9, 1, 1, 2)),
    ];
    assert_eq!(costs, expected);
    // An iteration inside a self-loop, and one through its polled link.
    assert_eq!(loop_iteration(true), Cost::new(6, 1, 0, 1));
    assert_eq!(loop_iteration(false), Cost::new(13, 3, 1, 2));
}

/// The scanner's linked blocks count what they ran: its counts, the
/// instructions of every block a transfer left and one budget unit per
/// transfer.
#[test]
fn a_linked_scanner_counts_every_transfer_and_instruction() {
    let text = text(4096);
    let (words, lines, retired, transfers) = scanned(&text);
    let mut program = Program::new(&SCANNER, 0, true);
    assert_eq!(program.blocks.len(), 6);
    let mut frame = scanner_frame(&mut program, &text);
    let budget = program.control.budget;
    let (next, _) = program.run(&mut frame);
    assert_eq!(next, BASE + 15 * 4);
    assert_eq!((frame.x[9], frame.x[12], frame.x[14]), (words, lines, 0));
    let control = &program.control;
    assert_eq!(control.retired, retired);
    assert_eq!(budget - control.budget, transfers);
}

/// The host time and instructions the scanner takes per guest instruction
/// on this host: a measurement, run on demand.
#[test]
#[ignore = "a measurement: prints host time and instructions per guest instruction"]
fn scanner_time_on_this_host() {
    const REPEATS: usize = 2000;
    let text = text(4096);
    let (_, _, retired, transfers) = scanned(&text);
    let mut program = Program::with_target(&SCANNER, 0, true, TargetCpu::detect());
    let short = &text[..256];
    let mut frame = scanner_frame(&mut program, short);
    let (_, rips) = program.trace(&mut frame);
    let (_, _, short_retired, _) = scanned(short);
    let cost = program.cost(&rips);
    let mut best = f64::MAX;
    for _ in 0..5 {
        let start = std::time::Instant::now();
        for _ in 0..REPEATS {
            program.control.budget = 1 << 20;
            let mut frame = scanner_frame(&mut program, &text);
            program.run(&mut frame);
        }
        best = best.min(start.elapsed().as_secs_f64());
    }
    let per_guest = |count: usize| count as f64 / short_retired as f64;
    std::eprintln!(
        "scanner: {:.3} ns per guest instruction; per guest instruction {:.2} host \
         instructions, {:.2} loads, {:.2} stores, {:.2} modifies; {:.2} guest \
         instructions per transfer",
        best * 1e9 / (retired * REPEATS as u64) as f64,
        per_guest(cost.instructions),
        per_guest(cost.loads),
        per_guest(cost.stores),
        per_guest(cost.modifies),
        retired as f64 / transfers as f64,
    );
}
