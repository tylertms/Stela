//! Today's probes against the ones before them (a probe that kept what it
//! needed on the stack and returned to a tail that called it): for random
//! ITLB, block table, cell and control states, with entries put in, taken
//! out, rekeyed and emptied between runs, both take the same transfer into
//! the same block or the same exit, and leave the control the same.
use super::super::*;
use crate::GuestState;
use alloc::{boxed::Box, vec::Vec};
use core::ffi::c_void;
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
}

/// The probes before: a call from the tail, PUSH and POP for what they kept,
/// ZF and a return to the tail, which jumped or returned.
fn legacy_found(asm: &mut crate::x86_64::asm::Asm) {
    asm.alu(Alu::Cmp, Qword, R8, R8);
    asm.ret();
}

fn legacy_missed(asm: &mut crate::x86_64::asm::Asm) {
    asm.test(Qword, Gpr::Rsp, Gpr::Rsp);
    asm.ret();
}

/// The tail before: `mov edx, word; call probe; jne exit; jmp r9; exit: ret`,
/// after `movabs r9, cell` for a cell tail.
fn legacy_tail(at: u64, cell: Option<u64>, word: u32, probe: u64) -> Vec<u8> {
    let mut code = Code::new();
    let mut asm = code.asm();
    if let Some(cell) = cell {
        asm.mov_imm64(R9, cell);
    }
    asm.mov_imm32(Rdx, word);
    let call = asm.call();
    let exit = asm.jcc8(Cond::Ne);
    asm.jmp_rm(R9);
    asm.bind(exit);
    asm.ret();
    let distance = relative(at + call.end() as u64, probe).unwrap();
    code.patch(call.offset(), distance.to_le_bytes());
    code.into_bytes()
}

fn legacy_dispatcher(btype: u8, policy: Interruption) -> Option<Vec<u8>> {
    if btype > 3 {
        return None;
    }
    let mut code = Code::new();
    // The cell probe, R9 at the cell.
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let mut misses = Vec::new();
    if let Some(interrupted) = far_interruption_gate(&mut asm, policy, R8) {
        misses.push(interrupted);
    }
    asm.push(R9);
    let mut untranslated = Vec::new();
    translate(&mut asm, &mut untranslated);
    asm.pop(R9);
    // The cell is the target's with its page's key, or the table decides.
    // ECX = the cell's last word before its fields ([`Slot`]).
    asm.mov_load(Dword, Rcx, Mem::base(R9, ENTRY_TARGETS));
    asm.alu_load(Alu::Cmp, Qword, Rax, Mem::base(R9, 0));
    let stale = asm.jcc8(Cond::Ne);
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R9, ENTRY_KEY));
    let other = asm.jcc8(Cond::Ne);
    asm.push_rm(Mem::base(R9, ENTRY_BLOCK));
    asm.push_rm(Mem::base(R9, ENTRY_CODE));
    let torn = legacy_seqlock_check(&mut asm);
    asm.pop(R9);
    asm.pop(R8);
    asm.mov(Dword, R8, R8);
    asm.place(Rcx, CONTROL);
    spend_budget(&mut asm, policy);
    misses.push(asm.jcc(Cond::B));
    asm.shift_imm(ShiftOp::Shr, Dword, Rdx, ExitWord::RAN_SHIFT as u8);
    asm.alu(Alu::Add, Qword, Mem::base(Rcx, RETIRED), Rdx);
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    legacy_found(&mut asm);
    for miss in misses {
        asm.bind(miss);
    }
    legacy_missed(&mut asm);
    for miss in untranslated {
        asm.bind(miss);
    }
    asm.pop(R9);
    legacy_missed(&mut asm);
    // A cell written while it was read leaves the target to the table too,
    // with the key still in R8 and the control in RCX again.
    for miss in torn {
        asm.bind(miss);
    }
    asm.alu_imm8(Alu::Add, Qword, Gpr::Rsp, 16);
    asm.bind(stale);
    asm.bind(other);
    asm.place(Rcx, CONTROL);
    let to_table = asm.jmp();
    let padding = CELL_PROBE_BYTES
        .checked_sub(code.len())
        .expect("the cell probe fits its bytes");
    for _ in 0..padding {
        code.asm().int3();
    }
    debug_assert_eq!(code.len(), PROBE_ENTRY);
    // The table probe.
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let mut misses = Vec::new();
    if let Some(interrupted) = far_interruption_gate(&mut asm, policy, R9) {
        misses.push(interrupted);
    }
    translate(&mut asm, &mut misses);
    let table = code.len();
    let mut asm = code.asm();
    // R9 = the target's entry of the key in R8.
    asm.mov(Dword, R9, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R9, LOOKUP_BITS as u8);
    asm.alu(Alu::Xor, Dword, R9, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R9, 2);
    asm.alu(Alu::Xor, Dword, R9, R8);
    asm.alu_load(Alu::And, Dword, R9, Mem::base(Rcx, MASKS));
    asm.shift_imm(ShiftOp::Shl, Dword, R9, 5);
    asm.alu_load(Alu::Add, Qword, R9, Mem::base(Rcx, TABLE));
    // ECX = each entry's last word before its fields ([`Slot`]).
    asm.mov_load(Dword, Rcx, Mem::base(R9, ENTRY_TARGETS));
    asm.alu_load(Alu::Cmp, Qword, Rax, Mem::base(R9, 0));
    let other_pc = asm.jcc8(Cond::Ne);
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R9, ENTRY_KEY));
    let hit = asm.jcc8(Cond::E);
    asm.bind(other_pc);
    asm.alu_imm8(
        Alu::Xor,
        Qword,
        R9,
        core::mem::size_of::<LookupEntry>() as i8,
    );
    asm.mov_load(Dword, Rcx, Mem::base(R9, ENTRY_TARGETS));
    asm.alu_load(Alu::Cmp, Qword, Rax, Mem::base(R9, 0));
    misses.push(asm.jcc(Cond::Ne));
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R9, ENTRY_KEY));
    misses.push(asm.jcc(Cond::Ne));
    asm.bind(hit);
    if btype != 0 {
        asm.test_imm(Size::Byte, Rcx, 1 << btype);
        misses.push(asm.jcc(Cond::E));
    }
    asm.mov_load(Dword, R8, Mem::base(R9, ENTRY_BLOCK));
    asm.push_rm(Mem::base(R9, ENTRY_CODE));
    let torn = legacy_seqlock_check(&mut asm);
    asm.pop(R9);
    asm.place(Rcx, CONTROL);
    spend_budget(&mut asm, policy);
    misses.push(asm.jcc(Cond::B));
    asm.shift_imm(ShiftOp::Shr, Dword, Rdx, ExitWord::RAN_SHIFT as u8);
    asm.alu(Alu::Add, Qword, Mem::base(Rcx, RETIRED), Rdx);
    asm.alu_imm8(Alu::Add, Qword, Mem::base(Rcx, HITS), 1);
    for bit in 0..2 {
        if btype & 1 << bit != 0 {
            asm.bit_imm(BitTest::Bts, Qword, R8, BLOCK_BTYPE_SHIFT as u8 + bit);
        }
    }
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    legacy_found(&mut asm);
    for miss in torn {
        asm.bind(miss);
    }
    asm.pop(R9);
    for miss in misses {
        asm.bind(miss);
    }
    legacy_missed(&mut asm);
    crate::x86_64::asm::resolve(&mut code, to_table, table);
    Some(code.into_bytes())
}

// The check that ends a probe's read of the entry at R9, whose last word
// it read into ECX before the fields: the entry's word is unchanged and
// no writer was in it ([`Slot`]). The jumps of a torn read.
fn legacy_seqlock_check(asm: &mut crate::x86_64::asm::Asm) -> [crate::x86_64::asm::Patch; 2] {
    asm.alu_load(Alu::Cmp, Dword, Rcx, Mem::base(R9, ENTRY_TARGETS));
    let changed = asm.jcc8(Cond::Ne);
    asm.test_imm(Dword, Rcx, META_BUSY as i32);
    let busy = asm.jcc8(Cond::Ne);
    [changed, busy]
}

const PAGE: usize = 4096;
/// Per variant: the entry stub and tail at 0, the four dispatchers from
/// [`PROBES`]; the blocks are in a page of their own both enter.
const PROBES: usize = 512;
const DISPATCHER_BYTES: usize = 2048;
const REGION: usize = 3 * PAGE;
const BLOCKS: usize = 8;
const CONTEXTS: [u64; 3] = [
    5 << LOOKUP_SALT_BITS,
    9 << LOOKUP_SALT_BITS,
    13 << LOOKUP_SALT_BITS,
];
const KEYS: [u64; 2] = [block_key(0x8_0040_0000, 1), block_key(0x8_0040_0000, 2)];

fn marker(block: usize) -> u64 {
    0x7700 + block as u64
}

/// Target PCs: two pages in each half, two PCs in each page.
fn pcs() -> [u64; 8] {
    let lower = 0x1_8000_0000_u64;
    let upper = 0xffff_fe00_1234_0000_u64;
    [
        lower,
        lower + 0x40,
        lower + 0x1000,
        lower + 0x1080,
        upper,
        upper + 0x10,
        upper + 0x3000,
        upper + 0x3ff0,
    ]
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

struct Pages(*mut u8, usize);
impl Drop for Pages {
    fn drop(&mut self) {
        // SAFETY: our mapping, nothing runs it any more.
        assert_eq!(unsafe { munmap(self.0.cast(), self.1) }, 0);
    }
}
fn pages(bytes: usize) -> Pages {
    // SAFETY: private anonymous memory, unmapped on drop.
    let base = unsafe { mmap(core::ptr::null_mut(), bytes, 7, 0x22, -1, 0) };
    assert_ne!(base as usize, usize::MAX);
    Pages(base.cast(), bytes)
}
fn put(pages: &Pages, offset: usize, bytes: &[u8]) {
    assert!(offset + bytes.len() <= pages.1);
    // SAFETY: inside the mapping; nothing runs it while it is written.
    unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), pages.0.add(offset), bytes.len()) };
}

/// Run the stub at `region` (`mov rax, rsi`, then the tail) with `control`:
/// what it returned.
fn run(region: &Pages, control: &mut ChainControl, target: u64) -> (u64, u64) {
    // A native entry, as the engine's: the jump cache starts over.
    // SAFETY: the control's stamps are 0 or the live table's.
    unsafe { next_jump_serial(control.stamps) };
    let mut frame = Box::new(GuestState::default());
    frame.anchors[Anchor::Control] = control as *mut ChainControl as u64;
    // SAFETY: the region holds the stub, a tail and probes with the block
    // ABI; they touch the frame's control, the tables and the blocks, which
    // outlive the call.
    let result = unsafe {
        let run: unsafe extern "sysv64" fn(*mut GuestState, u64) -> u128 =
            core::mem::transmute(region.0);
        run(&mut *frame, target)
    };
    (result as u64, (result >> 64) as u64)
}

#[test]
fn probes_without_the_stack_transfer_as_the_probes_before_them() {
    let pcs = pcs();
    let blocks = pages(PAGE);
    let new = pages(REGION);
    let old = pages(REGION);
    for block in 0..BLOCKS {
        let mut code = vec![0x48, 0xb8];
        code.extend(marker(block).to_le_bytes());
        code.extend([0xba, 99, 0, 0, 0, 0xc3]);
        put(&blocks, 64 * block, &code);
    }
    for btype in 0..4u8 {
        let offset = PROBES + DISPATCHER_BYTES * btype as usize;
        let fresh = dispatcher_with_policy(btype, Interruption::Epoch).unwrap();
        let before = legacy_dispatcher(btype, Interruption::Epoch).unwrap();
        assert!(fresh.len() <= DISPATCHER_BYTES && before.len() <= DISPATCHER_BYTES);
        put(&new, offset, &fresh);
        put(&old, offset, &before);
    }
    let mut table = LookupTable::with_entries(MIN_LOOKUP_ENTRIES);
    let stamps = table.new_stamps();
    let mut itlb = LookupTable::itlb();
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let (mut entered, mut exited) = (0u32, 0u32);
    for (block, &pc) in pcs.iter().enumerate() {
        itlb.insert(
            itlb.page_slot(pc, CONTEXTS[half(pc)]),
            LookupEntry::page(pc & !0xfff, CONTEXTS[half(pc)], KEYS[0]),
        );
        let entry = LookupEntry {
            pc,
            key: KEYS[0],
            entry: blocks.0 as u64 + 64 * block as u64,
            block: block as u32,
            targets: 0b1111,
        };
        table.insert(table.slot(pc, KEYS[0]), entry);
    }
    let mut sites: Vec<(u64, u64)> = (0..4)
        .map(|site| (0x1000 + site, pcs[site as usize]))
        .collect();
    for &(site, pc) in &sites {
        let held = table.find(table.slot(pc, KEYS[0]), pc, KEYS[0]);
        table.link_cell(site, pc, Some(held));
    }
    for trial in 0..120_000u32 {
        // Change the tables as the engine does between transfers.
        match rng.below(10) {
            0..=2 => {
                let pc = pcs[rng.below(8) as usize];
                // Mostly the context the control names for the PC's half.
                let context = match rng.below(4) {
                    0 => CONTEXTS[rng.below(3) as usize],
                    _ => CONTEXTS[half(pc)],
                };
                let key = KEYS[usize::from(rng.below(4) == 0)];
                itlb.insert(
                    itlb.page_slot(pc, context),
                    LookupEntry::page(pc & !0xfff, context, key),
                );
            }
            3..=5 => {
                let pc = pcs[rng.below(8) as usize];
                let block = rng.below(BLOCKS as u64) as usize;
                let entry = LookupEntry {
                    pc,
                    key: KEYS[usize::from(rng.below(4) == 0)],
                    entry: blocks.0 as u64 + 64 * block as u64,
                    block: block as u32,
                    targets: rng.below(16) as u32,
                };
                table.insert(table.slot(entry.pc, entry.key), entry);
            }
            6 => {
                let frame = key_frame(KEYS[0]);
                table.forget_frames(frame..=frame, |_| rng.below(2) == 0);
            }
            7 => {
                let pc = pcs[rng.below(8) as usize] & !0xfff;
                itlb.forget_range_where(pc..=pc, |_| true);
            }
            8 => {
                // Relink a site's cell, to the entry the table holds now or none.
                let index = rng.below(sites.len() as u64) as usize;
                let pc = pcs[rng.below(8) as usize];
                let key = KEYS[rng.below(2) as usize];
                let held = table.find(table.slot(pc, key), pc, key);
                let current =
                    (held.pc == pc && held.key == key && rng.below(4) != 0).then_some(held);
                table.link_cell(sites[index].0, pc, current);
                sites[index].1 = pc;
            }
            _ => {
                if rng.below(50) == 0 {
                    itlb.clear();
                }
            }
        }
        // A transfer: through a site's cell or the table probe of a BTYPE.
        let through_cell = rng.below(2) == 0;
        let (site, cell_pc) = sites[rng.below(sites.len() as u64) as usize];
        let target = match rng.below(8) {
            0 => pcs[rng.below(8) as usize] + 4,
            1..=3 if through_cell => cell_pc,
            _ => pcs[rng.below(8) as usize],
        };
        let btype = if through_cell { 0 } else { rng.below(4) as u8 };
        let word = ExitWord::from(ExitKind::Direct).encoded()
            | (1 + rng.below(9) as u32) << ExitWord::RAN_SHIFT;
        let cell =
            LookupTable::cell_slot(&table.cells, table.cell_sites[&site]) as *const Slot as u64;
        let probe = PROBES
            + DISPATCHER_BYTES * btype as usize
            + if through_cell {
                CELL_PROBE_ENTRY
            } else {
                PROBE_ENTRY
            };
        let stub = [0x48, 0x89, 0xf0]; // mov rax,rsi
        let ran = word >> ExitWord::RAN_SHIFT;
        let kind = ExitWord::from(ExitKind::Direct);
        let fresh = match through_cell {
            true => cell_tail(
                new.0 as u64 + 3,
                cell,
                kind,
                ran,
                new.0 as u64 + probe as u64,
            )
            .unwrap()
            .to_vec(),
            false => lookup_tail(new.0 as u64 + 3, kind, ran, new.0 as u64 + probe as u64, 0)
                .unwrap()
                .to_vec(),
        };
        let before = legacy_tail(
            old.0 as u64 + 3,
            through_cell.then_some(cell),
            word,
            old.0 as u64 + probe as u64,
        );
        put(&new, 0, &stub);
        put(&new, 3, &fresh);
        put(&old, 0, &stub);
        put(&old, 3, &before);
        let base = ChainControl {
            budget: [0, 1, 2, 5][rng.below(4) as usize],
            retired: 100,
            block: 7,
            contexts: match rng.below(4) {
                0 => [
                    CONTEXTS[rng.below(3) as usize],
                    CONTEXTS[rng.below(3) as usize],
                ],
                _ => [CONTEXTS[0], CONTEXTS[1]],
            },
            table: table.address(),
            hits: 3,
            interrupt_epoch: u64::from(rng.below(10) == 0),
            salts: [0; 2],
            itlb: itlb.address(),
            masks: [table.mask(), itlb.mask()],
            stamps,
            ..ChainControl::default()
        };
        let mut base = base;
        base.salts = base.contexts.map(|context| itlb.salt(context));
        let (mut a, mut b) = (base, base);
        let ra = run(&new, &mut a, target);
        let rb = run(&old, &mut b, target);
        assert_eq!(
            ra, rb,
            "trial {trial}: target {target:#x} cell {through_cell} btype {btype}"
        );
        // Only today's cell probes count heat.
        a.heat = 0;
        assert_eq!(a, b, "trial {trial}: the controls differ");
        if ra.1 == 99 {
            entered += 1;
        } else {
            assert_eq!(ra, (target, u64::from(word)));
            exited += 1;
        }
    }
    assert!(entered > 8000 && exited > 8000, "{entered} {exited}");
}
