//! Blocks that branch back to their own start, run inside themselves
//! (`NativeSystem::self_loops`), end in the state the same blocks reach
//! through links: registers, flags, memory, retired instructions and any
//! data abort's syndrome, address and return address. The bodies are the
//! hot loops of the macOS kernel and libsystem_platform (memcmp, strcmp,
//! strlen, word and 32-byte copies, a vector OR fold, a vector compare,
//! an array scan), with random lengths, contents and pages that fault.
//! Both forms also end where blocks that store NZCV at each flag-setting
//! instruction do (`NativeSystem::capture_every_nzcv`): the flags a B.cond's
//! exit leaves, and those a fault after a compare finds, stay exact.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::{Result, ensure};
use stela_compiler::{GuestState, TargetCpu};
use stela_engine::machine::{Stop, system::Register};
#[path = "support/store_host.rs"]
mod store_host;
use store_host::{Mapping, machine};

const PAGE: u64 = 4096;
const CSV3_SHIFT: u32 = 60;
const BYTES: u64 = 4 * PAGE;
const SUFFIX: u32 = 0x9100_04e7; // add x7,x7,#1
const PREFIX: u32 = 0x9100_04c6; // add x6,x6,#1

/// B.cond, CBZ/CBNZ (W) and TBNZ by word offset from the branch.
fn b_cond(condition: u32, words: i32) -> u32 {
    0x5400_0000 | (((words as u32) & 0x7ffff) << 5) | condition
}
fn cbz(nonzero: bool, rt: u32, words: i32) -> u32 {
    0x3400_0000 | (u32::from(nonzero) << 24) | (((words as u32) & 0x7ffff) << 5) | rt
}
fn tbnz(rt: u32, bit: u32, words: i32) -> u32 {
    0x3700_0000 | ((bit >> 5) << 31) | ((bit & 31) << 19) | (((words as u32) & 0x3fff) << 5) | rt
}
const EQ: u32 = 0;
const NE: u32 = 1;
const HS: u32 = 2;
const LO: u32 = 3;
const VS: u32 = 6;
const HI: u32 = 8;
const LT: u32 = 11;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }
}

/// A loop: its words (head first, the suffix after the loop's end) and how
/// a case sets registers and data from `base`, the data mapping.
struct Program {
    name: &'static str,
    words: Vec<u32>,
    setup: fn(&mut Rng, u64, &mut GuestState, &mut [u8]),
}

fn programs() -> Vec<Program> {
    vec![
        Program {
            name: "byte checks whose side exits keep the flags",
            words: vec![
                0x3840_1408,     // ldrb w8,[x0],#1
                0x6b09_011f,     // cmp w8,w9
                b_cond(EQ, 9),   // b.eq past the loop
                0xab08_014a,     // adds x10,x10,x8
                b_cond(VS, 7),   // b.vs past the loop
                0x6b0b_011f,     // cmp w8,w11
                b_cond(HI, 5),   // b.hi past the loop
                0xeb08_018c,     // subs x12,x12,x8
                b_cond(LT, 3),   // b.lt past the loop
                0xf100_0442,     // subs x2,x2,#1
                b_cond(NE, -10), // b.ne to the ldrb
            ],
            setup: |rng, base, state, _| {
                state.x[0] = base + rng.below(2 * PAGE);
                state.x[2] = 1 + rng.below(3 * PAGE);
                state.x[9] = rng.below(256);
                state.x[10] = i64::MAX as u64 - rng.below(1 << 17);
                state.x[11] = 250 + rng.below(6);
                state.x[12] = rng.below(1 << 17);
            },
        },
        Program {
            name: "a count check before a load that faults",
            words: vec![
                0xf100_0442,    // subs x2,x2,#1
                b_cond(LO, 4),  // b.lo past the loop
                0x3840_1408,    // ldrb w8,[x0],#1
                0x6b09_011f,    // cmp w8,w9
                b_cond(NE, -4), // b.ne to the subs
            ],
            setup: |rng, base, state, _| {
                state.x[0] = base + rng.below(2 * PAGE);
                state.x[2] = rng.below(2 * PAGE);
                state.x[9] = rng.below(256);
            },
        },
        Program {
            name: "kernel bzero by DC ZVA",
            words: vec![
                0xd50b_7423, // dc zva, x3
                0x9101_0063, // add x3,x3,#64
                0xf101_0042, // subs x2,x2,#64
                b_cond(HI, -3),
            ],
            setup: |rng, base, state, _| {
                state.x[3] = base + 64 * rng.below(64);
                state.x[2] = 64 * (1 + rng.below(160));
            },
        },
        Program {
            name: "strlen to an unconditional back branch",
            words: vec![
                0x3840_1408,               // ldrb w8,[x0],#1
                cbz(false, 8, 2),          // cbz w8, past the loop
                0x1400_0000 | 0x03ff_fffe, // b -2
            ],
            setup: |rng, base, state, data| {
                let start = rng.below(2 * PAGE);
                state.x[0] = base + start;
                for byte in &mut data[start as usize..] {
                    *byte |= 1;
                }
                let end = (start + rng.below(3 * PAGE)).min(BYTES - 1);
                data[end as usize] = 0;
            },
        },
        Program {
            name: "count equal bytes past an inner branch",
            words: vec![
                0x3840_1408,    // ldrb w8,[x0],#1
                0x6b09_011f,    // cmp w8,w9
                b_cond(NE, 2),  // b.ne to the subs
                0x9100_054a,    // add x10,x10,#1
                0xf100_0442,    // subs x2,x2,#1
                b_cond(NE, -5), // b.ne to the ldrb
            ],
            setup: |rng, base, state, data| {
                state.x[0] = base + rng.below(2 * PAGE);
                state.x[2] = 1 + rng.below(3 * PAGE);
                state.x[9] = rng.below(4);
                for byte in data.iter_mut() {
                    *byte &= 3;
                }
            },
        },
        Program {
            name: "sum odd words past a branch to the back edge",
            words: vec![
                0xf840_8408,                                       // ldr x8,[x0],#8
                0xd100_0442,                                       // sub x2,x2,#1
                0x3600_0048,                                       // tbz x8,#0, to the cbnz
                0x8b08_014a,                                       // add x10,x10,x8
                0xb500_0000 | ((-4i32 as u32 & 0x7ffff) << 5) | 2, // cbnz x2, to the ldr
            ],
            setup: |rng, base, state, _| {
                state.x[0] = base + 8 * rng.below(PAGE / 8);
                state.x[2] = 1 + rng.below(1000);
            },
        },
        Program {
            name: "memcmp bytes",
            words: vec![
                0x3840_1408, // ldrb w8,[x0],#1
                0x3840_1429, // ldrb w9,[x1],#1
                0x6b09_0108, // subs w8,w8,w9
                b_cond(NE, 3),
                0xf100_0442, // subs x2,x2,#1
                b_cond(NE, -5),
            ],
            setup: |rng, base, state, data| {
                let (a, b) = (rng.below(2 * PAGE), rng.below(2 * PAGE));
                state.x[0] = base + a;
                state.x[1] = base + b;
                state.x[2] = 1 + rng.below(3 * PAGE);
                let same = rng.below(3 * PAGE) as usize;
                for index in 0..same.min(data.len() - a.max(b) as usize) {
                    data[b as usize + index] = data[a as usize + index];
                }
            },
        },
        Program {
            name: "strcmp bytes",
            words: vec![
                0x3840_1404, // ldrb w4,[x0],#1
                0x3840_1425, // ldrb w5,[x1],#1
                0xeb05_0083, // subs x3,x4,x5
                0x7a40_0884, // ccmp w4,#0,#4,eq
                b_cond(EQ, -4),
            ],
            setup: |rng, base, state, data| {
                let (a, b) = (rng.below(2 * PAGE), rng.below(2 * PAGE));
                state.x[0] = base + a;
                state.x[1] = base + b;
                let same = rng.below(3 * PAGE) as usize;
                for index in 0..same.min(data.len() - a.max(b) as usize) {
                    let byte = data[a as usize + index] | 1;
                    data[a as usize + index] = byte;
                    data[b as usize + index] = byte;
                }
                if rng.below(2) == 0 {
                    let end = (a as usize + same).min(data.len() - 1);
                    data[end] = 0;
                }
            },
        },
        Program {
            name: "strlen bytes",
            words: vec![
                0x3840_1408, // ldrb w8,[x0],#1
                cbz(true, 8, -1),
            ],
            setup: |rng, base, state, data| {
                let a = rng.below(3 * PAGE);
                state.x[0] = base + a;
                for byte in &mut data[a as usize..] {
                    *byte |= 1;
                }
                let end = a as usize + rng.below(2 * PAGE) as usize;
                if end < data.len() && rng.below(4) != 0 {
                    data[end] = 0;
                }
            },
        },
        Program {
            name: "copy words",
            words: vec![
                0xf840_8426, // ldr x6,[x1],#8
                0xf800_8466, // str x6,[x3],#8
                0xf100_2042, // subs x2,x2,#8
                b_cond(HS, -3),
            ],
            setup: |rng, base, state, _| {
                state.x[1] = base + rng.below(2 * PAGE) / 8 * 8;
                state.x[3] = base + rng.below(4 * PAGE) / 8 * 8;
                state.x[2] = rng.below(2 * PAGE);
            },
        },
        Program {
            name: "copy 32 non-temporal",
            words: vec![
                0xa800_2468, // stnp x8,x9,[x3]
                0xa801_2c6a, // stnp x10,x11,[x3,#16]
                0x9100_8063, // add x3,x3,#32
                0xa840_2428, // ldnp x8,x9,[x1]
                0xa841_2c2a, // ldnp x10,x11,[x1,#16]
                0x9100_8021, // add x1,x1,#32
                0xf100_8042, // subs x2,x2,#32
                b_cond(HI, -7),
            ],
            setup: |rng, base, state, _| {
                state.x[1] = base + rng.below(2 * PAGE) / 16 * 16;
                state.x[3] = base + rng.below(4 * PAGE) / 16 * 16;
                state.x[2] = rng.below(2 * PAGE);
            },
        },
        Program {
            name: "vector or fold",
            words: vec![
                0xad40_0400, // ldp q0,q1,[x0]
                0xad41_0c02, // ldp q2,q3,[x0,#32]
                0x4ea0_1c84, // orr v4.16b,v4.16b,v0.16b
                0x4ea1_1ca5, // orr v5.16b,v5.16b,v1.16b
                0x4ea2_1cc6, // orr v6.16b,v6.16b,v2.16b
                0x4ea3_1ce7, // orr v7.16b,v7.16b,v3.16b
                0x9101_0000, // add x0,x0,#64
                0xf101_0021, // subs x1,x1,#64
                b_cond(HI, -8),
            ],
            setup: |rng, base, state, _| {
                state.x[0] = base + rng.below(3 * PAGE) / 16 * 16;
                state.x[1] = rng.below(3 * PAGE);
            },
        },
        Program {
            name: "vector compare",
            words: vec![
                0x3cc1_0400, // ldr q0,[x0],#16
                0x3cc1_0421, // ldr q1,[x1],#16
                0x6e21_8c01, // cmeq v1.16b,v0.16b,v1.16b
                0x6e31_a820, // uminv b0,v1.16b
                0x1e26_0003, // fmov w3,s0
                cbz(false, 3, 3),
                0xf100_4042, // subs x2,x2,#16
                b_cond(HI, -7),
            ],
            setup: |rng, base, state, data| {
                let (a, b) = (rng.below(2 * PAGE), rng.below(2 * PAGE));
                state.x[0] = base + a;
                state.x[1] = base + b;
                state.x[2] = rng.below(3 * PAGE);
                let same = rng.below(3 * PAGE) as usize;
                for index in 0..same.min(data.len() - a.max(b) as usize) {
                    data[b as usize + index] = data[a as usize + index];
                }
            },
        },
        Program {
            name: "count past an entry's share",
            words: vec![
                0x9100_04a5, // add x5,x5,#1
                0xf100_0442, // subs x2,x2,#1
                b_cond(NE, -2),
            ],
            setup: |rng, _, state, _| {
                state.x[2] = 1 + rng.below(3000);
            },
        },
        Program {
            name: "array scan",
            words: vec![
                0xd37d_f00a, // lsl x10,x0,#3
                0xf86a_690a, // ldr x10,[x8,x10]
                0xeb01_015f, // cmp x10,x1
                b_cond(EQ, 4),
                0x9100_0400, // add x0,x0,#1
                0xeb00_013f, // cmp x9,x0
                b_cond(NE, -6),
            ],
            setup: |rng, base, state, data| {
                state.x[8] = base + rng.below(PAGE) / 8 * 8;
                state.x[0] = rng.below(64);
                state.x[9] = state.x[0] + 1 + rng.below(2000);
                let at = (state.x[8] - base) as usize + 8 * rng.below(2000) as usize;
                state.x[1] = 0x5eed_0000_0000_0000 | rng.below(1 << 20);
                if at + 8 <= data.len() {
                    data[at..at + 8].copy_from_slice(&state.x[1].to_le_bytes());
                }
            },
        },
        Program {
            name: "store bytes while bit 4",
            words: vec![
                0x3800_1402, // strb w2,[x0],#1
                0xd100_0442, // sub x2,x2,#1
                tbnz(2, 4, -2),
            ],
            setup: |rng, base, state, _| {
                state.x[0] = base + rng.below(4 * PAGE);
                state.x[2] = rng.below(1 << 12);
            },
        },
    ]
}

/// What a run ends in.
#[derive(Debug, PartialEq)]
struct Outcome {
    stop: Stop,
    pc: u64,
    state: GuestState,
    retired: u64,
    memory: Vec<u8>,
    abort: Option<(u64, u64, u64)>,
}

/// One case's start: registers, data, the page made to fault, and whether
/// every access masks its address.
struct Case<'a> {
    initial: &'a GuestState,
    contents: &'a [u8],
    protected: Option<u64>,
    masked: bool,
}

fn run(
    words: &[u32],
    code: &Mapping,
    data: &Mapping,
    loops: bool,
    every_nzcv: bool,
    case: &Case,
) -> Result<(Outcome, u64)> {
    let Case {
        initial,
        contents,
        protected,
        masked,
    } = *case;
    let mut machine = machine(code, data, words, TargetCpu::detect())?;
    machine.set_self_loops(loops);
    machine.set_capture_every_nzcv(every_nzcv);
    // FEAT_CSV3 disclosed: every access masks its address, and the loop
    // carries its masked bases across the back edge.
    let pfr0 = machine.cpu.registers.get(Register::ID_AA64PFR0_EL1, 0);
    let pfr0 = pfr0 & !(0xf << CSV3_SHIFT) | u64::from(masked) << CSV3_SHIFT;
    machine
        .cpu
        .registers
        .set_raw(Register::ID_AA64PFR0_EL1, 0, pfr0);
    // SCTLR_EL1.DZE: EL0 runs DC ZVA.
    let sctlr = machine.cpu.registers.get(Register::SCTLR_EL1, 0);
    machine
        .cpu
        .registers
        .set_raw(Register::SCTLR_EL1, 0, sctlr | 1 << 14);
    ensure!(data.write(data.0, contents), "write data");
    if let Some(page) = protected {
        data.protect(page as usize * PAGE as usize, PAGE as usize, false)?;
    }
    machine.cpu.state = initial.clone();
    machine.cpu.pc = code.0;
    let retired = machine.cpu.retired;
    let stop = machine.run(machine.statistics.blocks + 200_000);
    if let Some(page) = protected {
        data.protect(page as usize * PAGE as usize, PAGE as usize, true)?;
    }
    let abort = (stop == Stop::Breakpoint(machine.host.lower_synchronous())).then(|| {
        (
            machine.cpu.registers.get(Register::ESR_EL1, 0),
            machine.cpu.registers.get(Register::FAR_EL1, 0),
            machine.cpu.registers.get(Register::ELR_EL1, 0),
        )
    });
    Ok((
        Outcome {
            stop,
            pc: machine.cpu.pc,
            state: machine.cpu.state.clone(),
            retired: machine.cpu.retired - retired,
            memory: data.read(data.0, BYTES as usize).expect("readable data"),
            abort,
        },
        machine.statistics.blocks,
    ))
}

#[test]
fn self_loops_end_where_their_linked_blocks_do() -> Result<()> {
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let (mut cases, mut aborts, mut fewer) = (0, 0, 0);
    for program in programs() {
        let (mut program_cases, mut program_fewer) = (0, 0);
        for prefix in [false, true] {
            let words = [
                if prefix { vec![PREFIX] } else { vec![] },
                program.words.clone(),
                vec![SUFFIX],
            ]
            .concat();
            for case in 0..60 {
                let mut initial = GuestState {
                    x: std::array::from_fn(|_| rng.next()),
                    v: std::array::from_fn(|_| [rng.next(), rng.next()]),
                    nzcv: (rng.next() as u32) & 0xf000_0000,
                    ..GuestState::default()
                };
                let mut contents: Vec<u8> = (0..BYTES).map(|_| rng.next() as u8).collect();
                (program.setup)(&mut rng, data.0, &mut initial, &mut contents);
                // A third of the cases fault on a page the loop reaches.
                let protected = (case % 3 == 2).then(|| 1 + rng.below(3));
                // Half mask every access (FEAT_CSV3), as the M2 guest does.
                let masked = case % 2 == 1;
                let start = Case {
                    initial: &initial,
                    contents: &contents,
                    protected,
                    masked,
                };
                let run = |loops, every| run(&words, &code, &data, loops, every, &start);
                let (captured, _) = run(false, true)?;
                let (linked, linked_blocks) = run(false, false)?;
                let (looped, looped_blocks) = run(true, false)?;
                let what = format!(
                    "{} prefix {prefix} case {case} masked {masked} protected {protected:?}",
                    program.name
                );
                assert_eq!(linked, captured, "linked: {what}");
                assert_eq!(looped, captured, "looped: {what}");
                cases += 1;
                aborts += usize::from(linked.abort.is_some());
                fewer += usize::from(looped_blocks < linked_blocks);
                program_cases += 1;
                program_fewer += usize::from(looped_blocks < linked_blocks);
            }
        }
        // Each body runs inside its block, the joins and DC ZVA included.
        assert!(
            program_fewer > program_cases / 4,
            "{}: {program_fewer} of {program_cases} ran fewer blocks",
            program.name
        );
    }
    // The loops ran inside their blocks in most cases, and faulted in some.
    assert!(fewer > cases / 2, "{fewer} of {cases} ran fewer blocks");
    assert!(aborts > cases / 10, "{aborts} of {cases} aborted");
    Ok(())
}

/// The host bytes a mapping of `address` lies in: its first and last
/// address and whether any access is allowed, from `/proc/self/maps`.
fn host_mapping(address: u64) -> Result<Option<(u64, u64, bool)>> {
    for line in std::fs::read_to_string("/proc/self/maps")?.lines() {
        let mut fields = line.split_whitespace();
        let (Some(range), Some(rights)) = (fields.next(), fields.next()) else {
            continue;
        };
        let Some((first, end)) = range.split_once('-') else {
            continue;
        };
        let (first, end) = (
            u64::from_str_radix(first, 16)?,
            u64::from_str_radix(end, 16)?,
        );
        if (first..end).contains(&address) {
            return Ok(Some((first, end - 1, !rights.starts_with("---"))));
        }
    }
    Ok(None)
}

/// A store loop that runs past the end of its data stops there with a data
/// abort, as the guest's tables map nothing beyond: native code reaches
/// guest addresses as host addresses, so the test host keeps inaccessible
/// guards beside each region. Without them a loop ran into whatever the
/// process had mapped next (an engine allocation, another region) and the
/// run crashed or diverged depending on the host's layout.
#[test]
fn a_store_loop_past_its_data_stops_at_the_end() -> Result<()> {
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    let end = data.0 + BYTES;
    for (address, before) in [(end, false), (data.0 - 1, true)] {
        let Some((first, last, accessible)) = host_mapping(address)? else {
            continue;
        };
        // At least 16 inaccessible pages on each side.
        let room = if before {
            address - first
        } else {
            last - address
        };
        ensure!(
            !accessible && room >= 16 * PAGE - 1,
            "host bytes {first:#x}..={last:#x} beside the data at {address:#x}"
        );
    }
    let copy = [
        0xf840_8426,
        0xf800_8466,
        0xf100_2042,
        b_cond(HS, -3),
        SUFFIX,
    ];
    for loops in [false, true] {
        let mut machine = machine(&code, &data, &copy, TargetCpu::detect())?;
        machine.set_self_loops(loops);
        machine.cpu.state.x[1] = data.0;
        machine.cpu.state.x[3] = end - 64;
        machine.cpu.state.x[2] = 1024;
        machine.cpu.pc = code.0;
        let stop = machine.run(machine.statistics.blocks + 10_000);
        ensure!(
            stop == Stop::Breakpoint(machine.host.lower_synchronous())
                && machine.cpu.registers.get(Register::FAR_EL1, 0) == end
                && machine.cpu.state.x[3] == end,
            "loops {loops}: {stop:?}, FAR {:#x}, x3 {:#x}",
            machine.cpu.registers.get(Register::FAR_EL1, 0),
            machine.cpu.state.x[3]
        );
    }
    Ok(())
}

/// A loop that never falls through leaves its block through the polled
/// link after at most a block's instructions, so a run spends its budget
/// (and takes the link's interrupt check) as often as blocks of
/// `MAX_BLOCK_INSTRUCTIONS` would: interrupt delivery stays bounded.
#[test]
fn an_endless_self_loop_takes_its_link_within_a_blocks_instructions() -> Result<()> {
    const MAX_BLOCK: u64 = stela_compiler::block::MAX_BLOCK_INSTRUCTIONS as u64;
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    // add x5,x5,#1; add x6,x6,#1; cbz xzr, back to the first add: three
    // instructions per iteration, which do not divide a block's.
    let words = [
        0x9100_04a5,
        0x9100_04c6,
        0xb400_0000 | ((-2i32 as u32 & 0x7ffff) << 5) | 31,
    ];
    for loops in [false, true] {
        let mut machine = machine(&code, &data, &words, TargetCpu::detect())?;
        machine.set_self_loops(loops);
        machine.cpu.pc = code.0;
        let (retired, blocks) = (machine.cpu.retired, machine.statistics.blocks);
        machine.run(blocks + 1000);
        let (retired, blocks) = (
            machine.cpu.retired - retired,
            machine.statistics.blocks - blocks,
        );
        ensure!(blocks >= 1000, "the run spent its budget: {blocks}");
        ensure!(
            machine.cpu.state.x[5] == retired / 3,
            "every iteration counted"
        );
        ensure!(
            retired <= blocks * MAX_BLOCK,
            "loops {loops}: {retired} instructions in {blocks} transfers"
        );
        if loops {
            // Each entry ran a whole share inside the block.
            ensure!(retired >= blocks * (MAX_BLOCK - 3), "{retired} in {blocks}");
        }
    }
    Ok(())
}

/// A loop rewritten by the guest (store, DC CVAU, IC IVAU, ISB) runs its new
/// body the next time round, with self-loops as through links.
#[test]
fn a_rewritten_self_loop_runs_its_new_body() -> Result<()> {
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    let words = [
        0x9100_04a5,                                 // 0: add x5,x5,#1 (rewritten to #2)
        0xf100_0442,                                 // 1: subs x2,x2,#1
        b_cond(NE, -2),                              // 2: b.ne 0
        cbz(true, 6, 10),                            // 3: cbnz w6, 13
        0xb900_0149,                                 // 4: str w9,[x10]
        0xd50b_7b2a,                                 // 5: dc cvau, x10
        0xd503_3b9f,                                 // 6: dsb ish
        0xd50b_752a,                                 // 7: ic ivau, x10
        0xd503_3b9f,                                 // 8: dsb ish
        0xd503_3fdf,                                 // 9: isb
        0xaa0b_03e2,                                 // 10: mov x2,x11
        0x5280_0026,                                 // 11: mov w6,#1
        0x1400_0000 | (-12i32 as u32 & 0x03ff_ffff), // 12: b 0
        SUFFIX,                                      // 13
    ];
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    for case in 0..8 {
        let (first, second) = (1 + rng.below(700), 1 + rng.below(700));
        let (mut outcomes, mut transfers) = (vec![], vec![]);
        for loops in [false, true] {
            let mut machine = machine(&code, &data, &words, TargetCpu::detect())?;
            machine.set_self_loops(loops);
            // SCTLR_EL1.UCI: EL0 cleans and invalidates by VA.
            let sctlr = machine.cpu.registers.get(Register::SCTLR_EL1, 0);
            machine
                .cpu
                .registers
                .set_raw(Register::SCTLR_EL1, 0, sctlr | 1 << 26);
            let mut state = GuestState::default();
            state.x[2] = first;
            state.x[9] = 0x9100_08a5; // add x5,x5,#2
            state.x[10] = code.0;
            state.x[11] = second;
            machine.cpu.state = state;
            machine.cpu.pc = code.0;
            let (retired, blocks) = (machine.cpu.retired, machine.statistics.blocks);
            let stop = machine.run(blocks + 200_000);
            transfers.push(machine.statistics.blocks - blocks);
            ensure!(
                machine.cpu.state.x[5] == first + 2 * second && machine.cpu.state.x[7] == 1,
                "case {case} loops {loops}: x5 {} after {first} and {second}, {stop:?}",
                machine.cpu.state.x[5]
            );
            outcomes.push((
                stop,
                machine.cpu.pc,
                machine.cpu.state.clone(),
                machine.cpu.retired - retired,
            ));
        }
        assert_eq!(outcomes[0], outcomes[1], "case {case}");
        assert!(transfers[1] < transfers[0], "case {case}: {transfers:?}");
    }
    Ok(())
}
/// An endless loop whose iterations take two paths (a branch skipping one
/// instruction to the back edge) retires exactly what each path runs, and
/// still leaves through its link within a block's instructions.
#[test]
fn an_endless_loop_with_a_join_counts_each_path_within_its_bound() -> Result<()> {
    const MAX_BLOCK: u64 = stela_compiler::block::MAX_BLOCK_INSTRUCTIONS as u64;
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    // add x5,x5,#1; tbz x5,#0, to the cbz; add x6,x6,#1; cbz xzr, back:
    // four instructions on odd x5, three on even.
    let words = [
        0x9100_04a5,
        0x3600_0045,
        0x9100_04c6,
        0xb400_0000 | ((-3i32 as u32 & 0x7ffff) << 5) | 31,
    ];
    for loops in [false, true] {
        let mut machine = machine(&code, &data, &words, TargetCpu::detect())?;
        machine.set_self_loops(loops);
        machine.cpu.pc = code.0;
        let (retired, blocks) = (machine.cpu.retired, machine.statistics.blocks);
        machine.run(blocks + 1000);
        let (retired, blocks) = (
            machine.cpu.retired - retired,
            machine.statistics.blocks - blocks,
        );
        let state = &machine.cpu.state;
        ensure!(blocks >= 1000, "the run spent its budget: {blocks}");
        ensure!(
            retired == 3 * state.x[5] + state.x[6] && state.x[6] == state.x[5].div_ceil(2),
            "loops {loops}: {retired} instructions for x5 {} x6 {}",
            state.x[5],
            state.x[6]
        );
        ensure!(
            retired <= blocks * MAX_BLOCK,
            "loops {loops}: {retired} instructions in {blocks} transfers"
        );
        if loops {
            ensure!(retired >= blocks * (MAX_BLOCK - 4), "{retired} in {blocks}");
        }
    }
    Ok(())
}

/// The engine counts the self-loops it compiles, and while it profiles,
/// their entries and what their iterations retired past each entry's first.
#[test]
fn self_loops_are_counted_and_entries_only_while_profiling() -> Result<()> {
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    // add x5,x5,#1; add x6,x6,#1; cbz xzr, back to the first add.
    let words = [
        0x9100_04a5,
        0x9100_04c6,
        0xb400_0000 | ((-2i32 as u32 & 0x7ffff) << 5) | 31,
    ];
    for profiling in [false, true] {
        let mut machine = machine(&code, &data, &words, TargetCpu::detect())?;
        machine.set_step_timing(profiling);
        machine.cpu.pc = code.0;
        let retired = machine.cpu.retired;
        machine.run(machine.statistics.blocks + 1000);
        let retired = machine.cpu.retired - retired;
        let [formed, entries, iterated] = machine.counts().loops;
        ensure!(formed == 1, "one loop block compiled: {formed}");
        if profiling {
            // Each entry retires one iteration of 3 past what it iterated.
            ensure!(entries > 0 && iterated > 0, "{entries} entries, {iterated}");
            ensure!(
                iterated + 3 * entries == retired,
                "{iterated} iterated in {entries} entries of {retired}"
            );
        } else {
            ensure!(
                entries == 0 && iterated == 0,
                "{entries} entries, {iterated}"
            );
        }
    }
    Ok(())
}
