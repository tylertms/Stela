//! What NZCV setters and readers cost on a block's fallthrough path
//! ([`super::host_cost`]), case by case, so that each change to their
//! lowering shows in a test.
use super::host_cost::{Cost, snippet};
use alloc::{format, string::String, vec, vec::Vec};

const CMP_X0_X1: u32 = 0xeb01_001f;
const CMP_W0_W1: u32 = 0x6b01_001f;
const CMP_X0_5: u32 = 0xf100_141f;
const CMP_X0_7: u32 = 0xf100_1c1f;
const CMP_W0_13: u32 = 0x7100_341f;
const SUBS_X2: u32 = 0xeb01_0002;
const SUBS_W2: u32 = 0x6b01_0002;
const ADDS_X2: u32 = 0xab01_0002;
const ADDS_W2_1: u32 = 0x3100_0402;
const ANDS_X2: u32 = 0xea01_0002;
const ANDS_W2_FF: u32 = 0x7200_1c02;
const TST_X0_1: u32 = 0xf240_001f;
const TST_W0_W1: u32 = 0x6a01_001f;
const ADD_X3: u32 = 0x9100_0463;
const ADD_X4: u32 = 0x9100_0884;
const CSEL_X5: u32 = 0x9a81_0005;
const CSINC_X5: u32 = 0x9a81_0405;
const CSINV_X5: u32 = 0xda81_0005;
const CSNEG_X5: u32 = 0xda81_0405;
/// CSET X5 is CSINC X5,XZR,XZR with the condition inverted.
const CSET_X5: u32 = 0x9a9f_07e5;
const CSETM_X5: u32 = 0xda9f_03e5;
const CCMP_X0_X1_4: u32 = 0xfa41_0004;
const CCMP_W0_10_4: u32 = 0x7a4a_0804;
const CCMN_X0_X1_2: u32 = 0xba41_0002;
const ADCS_X3: u32 = 0xba01_0003;
const ADCS_X4: u32 = 0xba01_0004;
const SBCS_X3: u32 = 0xfa01_0003;
const FCMP_D0_D1: u32 = 0x1e61_2000;

/// B.cond to `offset` bytes from itself.
const fn b_cond(condition: u32, offset: i32) -> u32 {
    0x5400_0000 | ((offset >> 2) as u32 & 0x7_ffff) << 5 | condition
}
/// B to `offset` bytes from itself.
const fn b(offset: i32) -> u32 {
    0x1400_0000 | ((offset >> 2) as u32 & 0x3ff_ffff)
}
/// CBZ X0 to `offset` bytes from itself.
const fn cbz_x0(offset: i32) -> u32 {
    0xb400_0000 | ((offset >> 2) as u32 & 0x7_ffff) << 5
}
/// TBZ X0,#3 to `offset` bytes from itself.
const fn tbz_x0_3(offset: i32) -> u32 {
    0x3600_0000 | 3 << 19 | ((offset >> 2) as u32 & 0x3fff) << 5
}
/// `word` with its condition field (bits 15:12) set to `condition`.
const fn with_condition(word: u32, condition: u32) -> u32 {
    word & !(0xf << 12) | condition << 12
}

const CONDITIONS: [&str; 14] = [
    "eq", "ne", "hs", "lo", "mi", "pl", "vs", "vc", "hi", "ls", "ge", "lt", "gt", "le",
];

/// The snippets, by name: the setter and its reader, a forward branch's
/// side exit or a select then an ADD, and a far B that ends the block.
fn cases() -> Vec<(String, Vec<u32>)> {
    let mut cases = vec![];
    let tail = [ADD_X3, b(0x100)];
    for (condition, name) in CONDITIONS.iter().enumerate() {
        let condition = condition as u32;
        let mut side = vec![CMP_X0_X1, b_cond(condition, 12)];
        side.extend(tail);
        cases.push((format!("cmp+b.{name} side"), side));
        cases.push((
            format!("cmp+b.{name} end"),
            vec![CMP_X0_X1, b_cond(condition, -0x40)],
        ));
        let mut select = vec![CMP_X0_X1, with_condition(CSEL_X5, condition)];
        select.extend(tail);
        cases.push((format!("cmp+csel {name}"), select));
    }
    let side = |setter: u32| {
        let mut words = vec![setter, b_cond(1, 12)];
        words.extend(tail);
        words
    };
    for (name, setter) in [
        ("cmp w", CMP_W0_W1),
        ("subs x", SUBS_X2),
        ("subs w", SUBS_W2),
        ("adds x", ADDS_X2),
        ("adds w", ADDS_W2_1),
        ("ands x", ANDS_X2),
        ("ands w", ANDS_W2_FF),
        ("tst x", TST_X0_1),
        ("tst w", TST_W0_W1),
    ] {
        cases.push((format!("{name}+b.ne side"), side(setter)));
    }
    // HI after ADDS needs the carry turned over.
    let mut adds_hi = vec![ADDS_X2, b_cond(8, 12)];
    adds_hi.extend(tail);
    cases.push(("adds x+b.hi side".into(), adds_hi));
    for (name, select) in [
        ("cset hi", with_condition(CSET_X5, 9)),
        ("csinc lt", with_condition(CSINC_X5, 11)),
        ("csinv hs", with_condition(CSINV_X5, 2)),
        ("csneg ge", with_condition(CSNEG_X5, 10)),
        ("csetm ne", with_condition(CSETM_X5, 0)),
    ] {
        let mut words = vec![CMP_X0_X1, select];
        words.extend(tail);
        cases.push((format!("cmp+{name}"), words));
    }
    let mut chain = vec![CMP_W0_13, CCMP_W0_10_4, b_cond(1, 12)];
    chain.extend(tail);
    cases.push(("cmp+ccmp+b.ne side".into(), chain));
    let mut chain = vec![CMP_X0_X1, CCMP_X0_X1_4, CCMP_W0_10_4, b_cond(0, 12)];
    chain.extend(tail);
    cases.push(("cmp+ccmp+ccmp+b.eq side".into(), chain));
    let mut ccmn = vec![CMP_X0_X1, CCMN_X0_X1_2, b_cond(0, 12)];
    ccmn.extend(tail);
    cases.push(("cmp+ccmn+b.eq side".into(), ccmn));
    let mut compares = vec![CMP_X0_5, b_cond(0, 16), CMP_X0_7, b_cond(0, 8)];
    compares.extend(tail);
    cases.push(("cmp+b.eq cmp+b.eq side".into(), compares));
    let mut zero = vec![cbz_x0(12)];
    zero.extend(tail);
    cases.push(("cbz side".into(), zero));
    let mut bit = vec![tbz_x0_3(12)];
    bit.extend(tail);
    cases.push(("tbz side".into(), bit));
    cases.push((
        "adds+adcs+adcs end".into(),
        vec![ADDS_X2, ADCS_X3, ADCS_X4, b(0x100)],
    ));
    cases.push(("subs+sbcs end".into(), vec![SUBS_X2, SBCS_X3, b(0x100)]));
    let mut fp = vec![FCMP_D0_D1, b_cond(4, 12)];
    fp.extend(tail);
    cases.push(("fcmp+b.mi side".into(), fp));
    cases.push(("cmp live out".into(), vec![CMP_X0_X1, ADD_X3, b(0x100)]));
    let mut dead = vec![CMP_X0_X1, ADDS_X2, b_cond(1, 12)];
    dead.extend(tail);
    cases.push(("dead cmp+adds+b.ne side".into(), dead));
    let mut between = vec![CMP_X0_X1, ADD_X4, b_cond(1, 12)];
    between.extend(tail);
    cases.push(("cmp+add+b.ne side".into(), between));
    // A loop's body: readers of one setter, then the back edge's own.
    cases.push((
        "cmp+b.eq side, subs+b.ne end".into(),
        vec![CMP_X0_5, b_cond(0, 0x40), ADD_X3, SUBS_X2, b_cond(1, -0x40)],
    ));
    cases.push((
        "cmp+csel lt, cmp+b.lt end".into(),
        vec![
            CMP_X0_X1,
            with_condition(CSEL_X5, 11),
            CMP_X0_7,
            b_cond(11, -0x40),
        ],
    ));
    cases.push((
        "tst+b.ne side, cmp+b.hi end".into(),
        vec![
            TST_X0_1,
            b_cond(1, 0x40),
            ADD_X4,
            CMP_X0_X1,
            b_cond(8, -0x40),
        ],
    ));
    cases.push((
        "cmp+ccmp+b.ne side, cmp+b.lt end".into(),
        vec![
            CMP_W0_13,
            CCMP_W0_10_4,
            b_cond(1, 0x40),
            CMP_X0_X1,
            b_cond(11, -0x40),
        ],
    ));
    cases
}

/// Each case's cost, as `name instructions memory flags branches` lines.
fn report() -> String {
    let mut lines = String::new();
    for (name, words) in cases() {
        let (
            _,
            Cost {
                instructions,
                memory,
                flags,
                branches,
            },
        ) = snippet(&words);
        lines += &format!("{name}: {instructions} {memory} {flags} {branches}\n");
    }
    lines
}

/// The costs, recorded: a change that lowers one updates its line. Each
/// line is `name: instructions memory flags branches`.
const EXPECTED: &str = "\
cmp+b.eq side: 10 3 4 2
cmp+b.eq end: 8 1 4 2
cmp+csel eq: 13 4 4 1
cmp+b.ne side: 10 3 4 2
cmp+b.ne end: 8 1 4 2
cmp+csel ne: 13 4 4 1
cmp+b.hs side: 10 3 4 2
cmp+b.hs end: 8 1 4 2
cmp+csel hs: 13 4 4 1
cmp+b.lo side: 10 3 4 2
cmp+b.lo end: 8 1 4 2
cmp+csel lo: 13 4 4 1
cmp+b.mi side: 10 3 4 2
cmp+b.mi end: 8 1 4 2
cmp+csel mi: 13 4 4 1
cmp+b.pl side: 10 3 4 2
cmp+b.pl end: 8 1 4 2
cmp+csel pl: 13 4 4 1
cmp+b.vs side: 10 3 4 2
cmp+b.vs end: 8 1 4 2
cmp+csel vs: 13 4 4 1
cmp+b.vc side: 10 3 4 2
cmp+b.vc end: 8 1 4 2
cmp+csel vc: 13 4 4 1
cmp+b.hi side: 10 3 4 2
cmp+b.hi end: 9 1 5 2
cmp+csel hi: 13 4 4 1
cmp+b.ls side: 10 3 4 2
cmp+b.ls end: 9 1 5 2
cmp+csel ls: 13 4 4 1
cmp+b.ge side: 10 3 4 2
cmp+b.ge end: 8 1 4 2
cmp+csel ge: 13 4 4 1
cmp+b.lt side: 10 3 4 2
cmp+b.lt end: 8 1 4 2
cmp+csel lt: 13 4 4 1
cmp+b.gt side: 10 3 4 2
cmp+b.gt end: 8 1 4 2
cmp+csel gt: 13 4 4 1
cmp+b.le side: 10 3 4 2
cmp+b.le end: 8 1 4 2
cmp+csel le: 13 4 4 1
cmp w+b.ne side: 10 3 4 2
subs x+b.ne side: 12 4 4 2
subs w+b.ne side: 12 4 4 2
adds x+b.ne side: 12 4 3 2
adds w+b.ne side: 12 4 3 2
ands x+b.ne side: 12 4 3 2
ands w+b.ne side: 11 4 3 2
tst x+b.ne side: 11 3 3 2
tst w+b.ne side: 10 3 3 2
adds x+b.hi side: 13 4 5 2
cmp+cset hi: 13 4 4 1
cmp+csinc lt: 15 4 4 1
cmp+csinv hs: 15 4 4 1
cmp+csneg ge: 18 5 6 1
cmp+csetm ne: 13 4 4 1
cmp+ccmp+b.ne side: 21 5 9 2
cmp+ccmp+ccmp+b.eq side: 30 7 14 2
cmp+ccmn+b.eq side: 21 5 8 2
cmp+b.eq cmp+b.eq side: 13 3 4 3
cbz side: 7 2 0 2
tbz side: 7 2 0 2
adds+adcs+adcs end: 17 5 6 1
subs+sbcs end: 17 4 9 1
fcmp+b.mi side: 19 7 2 5
cmp live out: 9 3 4 1
dead cmp+adds+b.ne side: 12 4 3 2
cmp+add+b.ne side: 15 6 5 2
cmp+b.eq side, subs+b.ne end: 14 4 4 3
cmp+csel lt, cmp+b.lt end: 12 2 4 2
tst+b.ne side, cmp+b.hi end: 15 3 5 3
cmp+ccmp+b.ne side, cmp+b.lt end: 23 4 13 3
";

#[test]
fn flag_costs_are_as_recorded() {
    let report = report();
    std::eprint!("{report}");
    assert_eq!(report, EXPECTED);
}

/// The host time of a compare-and-branch loop that scans bytes for two
/// values, run as the engine runs a self-loop: `cargo test --release -p
/// stela-compiler --lib time_compare_loop -- --ignored --nocapture`.
#[cfg(all(target_arch = "x86_64", target_os = "linux"))]
#[test]
#[ignore = "a timing, not a check"]
fn time_compare_loop() {
    use crate::{GuestState, state::Limit, x86_64::pinned};
    const WORDS: [u32; 8] = [
        0x3840_1408, // 0: ldrb w8,[x0],#1
        0x7100_811f, // 4: cmp w8,#0x20
        b_cond(0, 0x40),
        0x7100_291f, // c: cmp w8,#10
        b_cond(0, 0x40),
        ADD_X3,
        0xf100_0442, // 18: subs x2,x2,#1
        b_cond(1, -0x1c),
    ];
    const BYTES: usize = 1 << 16;
    let source: Vec<u8> = WORDS.iter().flat_map(|word| word.to_le_bytes()).collect();
    let mut options = super::host_cost::options(crate::TargetCpu::detect());
    options = options.with_native_system(crate::x86_64::NativeSystem {
        exclusive: true,
        self_loops: true,
        ..crate::x86_64::NativeSystem::NONE
    });
    let compiled =
        crate::block::CompilationPlan::new(&source, crate::GuestAddress(0x1000), &options)
            .and_then(|plan| plan.compile())
            .unwrap();
    assert!(compiled.self_loop);
    let code = pinned::entered(&compiled.native.x86_64_code);
    let text = vec![b'a'; BYTES];
    let entry = Mapped::new(&code);
    let mut control = crate::x86_64::chain::ChainControl::default();
    let mut best = f64::MAX;
    for _ in 0..20 {
        let mut state = GuestState {
            lower_limit: Limit(u64::MAX),
            ..GuestState::default()
        };
        // An exit settles the loop's count there.
        state.anchors[crate::Anchor::Control] = (&raw mut control) as u64;
        state.x[0] = text.as_ptr() as u64;
        state.x[2] = BYTES as u64;
        let start = std::time::Instant::now();
        while state.x[2] != 0 {
            entry.run(&mut state);
        }
        let nanoseconds = start.elapsed().as_nanos() as f64 / BYTES as f64;
        assert_eq!(state.x[3], BYTES as u64);
        best = best.min(nanoseconds);
    }
    std::eprintln!(
        "compare loop: {best:.3} ns per iteration, {} code bytes",
        compiled.native.x86_64_code.len()
    );
}

/// Code mapped once, executable, for repeated runs.
#[cfg(all(target_arch = "x86_64", target_os = "linux"))]
struct Mapped(*mut core::ffi::c_void);

#[cfg(all(target_arch = "x86_64", target_os = "linux"))]
impl Mapped {
    fn new(code: &[u8]) -> Self {
        unsafe extern "C" {
            fn mmap(
                address: *mut core::ffi::c_void,
                bytes: usize,
                protection: i32,
                flags: i32,
                fd: i32,
                offset: i64,
            ) -> *mut core::ffi::c_void;
            fn mprotect(address: *mut core::ffi::c_void, bytes: usize, protection: i32) -> i32;
        }
        let bytes = (code.len() + 1).next_multiple_of(4096);
        // SAFETY: a private anonymous mapping, RX after the copy, which the
        // test keeps for its runs.
        unsafe {
            let page = mmap(core::ptr::null_mut(), bytes, 3, 0x22, -1, 0);
            assert_ne!(page as usize, usize::MAX);
            core::ptr::copy_nonoverlapping(code.as_ptr(), page.cast(), code.len());
            page.cast::<u8>().add(code.len()).write(0xc3);
            assert_eq!(mprotect(page, bytes, 5), 0);
            Self(page)
        }
    }

    fn run(&self, state: &mut crate::GuestState) -> u64 {
        // SAFETY: the mapping holds an entered block and a RET.
        unsafe {
            let entry: unsafe extern "sysv64" fn(*mut crate::GuestState) -> u64 =
                core::mem::transmute(self.0);
            entry(state)
        }
    }
}
