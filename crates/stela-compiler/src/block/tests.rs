use crate::x86_64::asm::Code;
#[test]
fn shared_system_uses_checked_recipes_and_refuses_serial_native_specialization() {
    use crate::TargetCpu;
    let source: Vec<_> = [
        0xf900_0001u32,
        0xc85f_7c02,
        0xc803_7c01,
        0xf821_0002,
        0xc8df_fc01,
        0xd65f_03c0,
    ]
    .into_iter()
    .flat_map(u32::to_le_bytes)
    .collect();
    for specialization in [
        CheckedMemoryProfile::SpAlignmentEnabled,
        CheckedMemoryProfile::SpAlignmentDisabled,
    ] {
        let shared = BlockProfile::new(Features::SYSTEM_MODE).with_checked_memory(specialization);
        let checked = BlockProfile::ORDINARY.with_checked_memory(specialization);
        assert_ne!(shared.abi(), checked.abi());
        assert_ne!(shared.abi(), BlockProfile::new(Features::SYSTEM_MODE).abi());
        let target = TargetCpu::X86_64_V2.with_atomic_load16();
        let options = crate::Options::new(shared, target);
        let plan = || CompilationPlan::new(&source, GuestAddress(0x4000), &options).unwrap();
        assert!(
            CompilationPlan::new(
                &source,
                GuestAddress(0x4000),
                &options.with_memory_checks(crate::x86_64::MemoryChecks {
                    alignment: true,
                    ..crate::x86_64::MemoryChecks::DEFAULT
                })
            )
            .is_err()
        );
        let expected = CompilationPlan::new(
            &source,
            GuestAddress(0x4000),
            &crate::Options::new(checked, target),
        )
        .unwrap();
        assert_eq!(plan().memory_requirements(), expected.memory_requirements());
        let native = plan().compile().unwrap().native;
        let expected = expected.compile().unwrap().native;
        assert_eq!(native.x86_64_code, expected.x86_64_code);
        assert_eq!(native.source_map, expected.source_map);
    }
}

#[test]
fn shared_system_no_avx_requires_the_distinct_cx16_rw_read_footprint() {
    use crate::{
        TargetCpu,
        context::memory::{host_caps, profile_requirements},
    };
    let source: Vec<_> = [0xc85f_7c02u32, 0xc8df_fc01, 0xd65f_03c0]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let profile = CheckedMemoryProfile::SpAlignmentDisabled;
    let plan = |outer, target| {
        CompilationPlan::new(
            &source,
            GuestAddress(0x4000),
            &crate::Options::new(outer, target),
        )
        .unwrap()
    };
    let shared = plan(
        BlockProfile::new(Features::SYSTEM_MODE).with_checked_memory(profile),
        TargetCpu::X86_64_V2,
    );
    let required = shared.memory_requirements().unwrap();
    assert_eq!(required.required_host_caps, host_caps::CX16);
    assert_ne!(
        required.required_profile & profile_requirements::SYSTEM_ATOMIC_READ_WRITEBACK,
        0
    );
    assert!(
        CompilationPlan::new(
            &source,
            GuestAddress(0x4000),
            &crate::Options::new(
                BlockProfile::ORDINARY.with_checked_memory(profile),
                TargetCpu::X86_64_V2
            )
        )
        .is_err()
    );
    let legacy = plan(
        BlockProfile::ORDINARY.with_checked_memory(profile),
        TargetCpu::X86_64_V2.with_atomic_load16(),
    );
    let required = legacy.memory_requirements().unwrap();
    assert_eq!(required.required_host_caps, host_caps::ATOMIC_LOAD16);
    assert_eq!(
        required.required_profile & profile_requirements::SYSTEM_ATOMIC_READ_WRITEBACK,
        0
    );
    let optional = plan(
        BlockProfile::new(Features::SYSTEM_MODE).with_checked_memory(profile),
        TargetCpu::X86_64_V2.with_atomic_load16(),
    );
    assert_eq!(
        shared.compile().unwrap().native.x86_64_code,
        optional.compile().unwrap().native.x86_64_code
    );
}

#[test]
fn shared_system_control_and_pauth_terminals_keep_the_exact_unexecuted_source_pc() {
    use crate::TargetCpu;
    let base = GuestAddress(0x4000);
    let shared = BlockProfile::new(Features::SYSTEM_MODE)
        .with_checked_memory(CheckedMemoryProfile::SpAlignmentDisabled);
    assert!(shared.has(Features::SYSTEM_REGISTERS) && shared.has(Features::FULL_SYSTEM));
    assert!(!shared.has(Features::PROCESS_HELPERS));
    for terminal in [0xd53b_e040u32, 0xd503_3fdf, 0xd503_237f, 0xd400_0001] {
        let source: Vec<_> = [0x9100_0400u32, terminal, 0xd65f_03c0]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let plan = CompilationPlan::new(
            &source,
            base,
            &crate::Options::new(shared, TargetCpu::Baseline),
        )
        .unwrap();
        assert_eq!(plan.source_bytes(), 8);
        assert_eq!(
            plan.parsed().terminal.as_ref().unwrap().0,
            GuestAddress(base.0 + 4)
        );
        assert_eq!(
            plan.options.native_system(),
            crate::x86_64::NativeSystem::NONE
        );
        let body = plan.compile().unwrap();
        assert_eq!(
            body.native.source_map.last().unwrap().0,
            GuestAddress(base.0 + 4)
        );
        assert_eq!(body.source_bytes, 8);
    }
}

#[test]
fn shared_system_numerical_forms_recover_the_exact_unexecuted_terminal() {
    let base = GuestAddress(0x4000);
    const SUB_X8_X21_X23: u32 = 0xcb17_02a8;
    for specialization in [
        CheckedMemoryProfile::SpAlignmentEnabled,
        CheckedMemoryProfile::SpAlignmentDisabled,
    ] {
        let shared = BlockProfile::new(Features::SYSTEM_MODE).with_checked_memory(specialization);
        for word in [
            0x9e63_0100, // UCVTF D0,X8, reached by the original kernel
            0x1e22_2820, // FADD S0,S1,S2
            0x1e62_0820, // FMUL D0,D1,D2
            0x4e22_d420, // FADD V0.4S,V1.4S,V2.4S
            0x1e61_c020, // FSQRT D0,D1
            0x1e62_2020, // FCMP D1,D2
            0x1e62_4820, // FMAX D0,D1,D2
            0x1f42_0c20, // FMADD D0,D1,D2,D3
        ] {
            for prefix in [false, true] {
                let mut words = Vec::new();
                if prefix {
                    words.push(SUB_X8_X21_X23);
                }
                words.extend([word, u32::MAX]);
                let source: Vec<_> = words.into_iter().flat_map(u32::to_le_bytes).collect();
                let plan = CompilationPlan::new(
                    &source,
                    base,
                    &crate::Options::new(shared, crate::TargetCpu::Baseline),
                )
                .unwrap();
                let terminal_pc = GuestAddress(base.0 + u64::from(prefix) * 4);
                assert!(matches!(plan.parsed().terminal,
                    Some((pc, Terminator::FloatingPoint { .. })) if pc == terminal_pc));
                assert_eq!(plan.source_bytes(), (usize::from(prefix) + 1) * 4);
                let analysis = analyze_with_profile(&source, base, shared).unwrap();
                assert_eq!(analysis.terminator_pc, Some(terminal_pc));
                assert_eq!(
                    analysis.static_successors,
                    [StaticSuccessor {
                        kind: SuccessorKind::Fallthrough,
                        target: GuestAddress(terminal_pc.0 + 4),
                    }]
                );
                let compiled = plan.compile().unwrap();
                assert_eq!(compiled.source_bytes, analysis.source_bytes);
                assert_eq!(compiled.native.source_map.len(), usize::from(prefix) + 1);
                assert_eq!(compiled.native.source_map.last().unwrap().0, terminal_pc);
                assert!(compiled.fault_recovery.is_empty());
            }
        }
    }
}

use alloc::vec;
use alloc::vec::Vec;

#[test]
fn one_parse_analyzes_and_plans_the_block_it_finds() {
    const BASE: GuestAddress = GuestAddress(0x4000);
    // add x0, x0, #1; ret; then a word after the block
    let source: Vec<u8> = [0x9100_0400_u32, 0xd65f_03c0, 0x9100_0400]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    for profile in [
        BlockProfile::ORDINARY,
        BlockProfile::new(Features::SYSTEM_MODE),
    ] {
        let target = crate::TargetCpu::Baseline;
        let (analysis, plan) =
            analyze_and_plan(&source, BASE, &crate::Options::new(profile, target)).unwrap();
        assert_eq!(
            analysis,
            analyze_with_profile(&source, BASE, profile).unwrap()
        );
        let consumed = &source[..analysis.source_bytes];
        assert_eq!(consumed.len(), 8);
        // CompiledBlock has no PartialEq; its Debug form shows every field.
        assert_eq!(
            alloc::format!("{:?}", plan.unwrap().compile().unwrap()),
            alloc::format!(
                "{:?}",
                crate::testing::block_for(consumed, BASE, profile, target).unwrap()
            )
        );
    }
    assert!(
        analyze_and_plan(
            &[],
            BASE,
            &crate::Options::new(BlockProfile::ORDINARY, crate::TargetCpu::Baseline)
        )
        .is_err()
    );
}

#[test]
fn checked_system_admission_stops_at_denial_error_or_terminal() {
    const BASE: GuestAddress = GuestAddress(0x4ff0);
    const NOP: u32 = 0xd503_201f;
    const BRANCH: u32 = 0x5400_0020; // b.eq to the next word
    const DENIED: u32 = 0xd65f_03c0; // ret
    const UNSUPPORTED: u32 = 0xffff_ffff;
    let checked = CheckedMemoryProfile::SpAlignmentEnabled;
    let target = crate::TargetCpu::X86_64_V2;
    let profile = BlockProfile::new(Features::SYSTEM_MODE).with_checked_memory(checked);
    let options = crate::Options::new(profile, target);
    let admits = |word| word != DENIED;

    for (words, reached) in [
        (&[DENIED, NOP][..], 1),
        (&[NOP, DENIED, BRANCH][..], 2),
        (&[NOP, BRANCH, NOP, DENIED][..], 2),
        (&[NOP, UNSUPPORTED, DENIED][..], 2),
    ] {
        let source = bytes(words);
        let boundary = words.iter().position(|&word| !admits(word)).unwrap();
        let mut seen = Vec::new();
        let lazy = CompilationPlan::new_admitted_system(&source, BASE, checked, target, |word| {
            seen.push(word);
            admits(word)
        });
        assert_eq!(seen.as_slice(), &words[..reached]);
        let eager = if boundary == 0 {
            Ok(None)
        } else {
            CompilationPlan::new(&source[..4 * boundary], BASE, &options).map(Some)
        };
        match (eager, lazy) {
            (Ok(None), Ok(None)) => assert_eq!(boundary, 0),
            (Err(eager), Err(lazy)) => assert_eq!(eager, lazy),
            (Ok(Some(eager)), Ok(Some(lazy))) => {
                assert_eq!(eager.parsed().instructions.capacity(), 0);
                assert_eq!(lazy.parsed().instructions.capacity(), 0);
                assert_eq!(eager.memory_requirements(), lazy.memory_requirements());
                let (eager, lazy) = (eager.compile().unwrap(), lazy.compile().unwrap());
                assert_eq!(eager.source_bytes, lazy.source_bytes);
                assert_eq!(eager.native.x86_64_code, lazy.native.x86_64_code);
                assert_eq!(eager.native.source_map, lazy.native.source_map);
                assert_eq!(eager.direct_exits, lazy.direct_exits);
                assert_eq!(eager.fault_recovery, lazy.fault_recovery);
            }
            _ => panic!("eager and lazy admission outcomes differ"),
        }
    }
}

#[test]
fn system_mode_blocks_can_omit_the_sp_alignment_check() {
    // test bl,15, on SP's host register.
    const SP_CHECK: [u8; 5] = [0xf6, 0xc3, 15, 0x0f, 0x85];
    // ldr x0, [sp, #8]; stp x0, x1, [sp, #-16]!; ldr q0, [sp, x2];
    // ldar x3, [sp]; ldadd x4, x5, [sp]
    let source: Vec<u8> = [
        0xf940_07e0_u32,
        0xa9bf_07e0,
        0x3ce2_6be0,
        0xc8df_ffe3,
        0xf824_03e5,
    ]
    .into_iter()
    .flat_map(u32::to_le_bytes)
    .collect();
    let unaligned = crate::x86_64::MemoryChecks {
        stack_alignment: false,
        ..crate::x86_64::MemoryChecks::DEFAULT
    };
    let compile = |checks| {
        let options = crate::Options::new(
            BlockProfile::new(Features::SYSTEM_MODE),
            crate::TargetCpu::detect(),
        )
        .with_memory_checks(checks);
        crate::compile(&source, GuestAddress(0x1000), &options)
            .unwrap()
            .native
            .x86_64_code
    };
    let has_check = |code: &[u8]| code.windows(SP_CHECK.len()).any(|bytes| bytes == SP_CHECK);
    assert!(has_check(&compile(crate::x86_64::MemoryChecks::DEFAULT)));
    assert!(!has_check(&compile(unaligned)));
    // Only the system-mode profile admits it.
    let ordinary = crate::Options::new(BlockProfile::ORDINARY, crate::TargetCpu::Baseline)
        .with_memory_checks(unaligned);
    assert!(crate::compile(&source[..4], GuestAddress(0x1000), &ordinary).is_err());
}

/// Every access of a system-mode block, in a clean-load region or not,
/// checks its address against the guest's ranges before it is made;
/// process blocks make no such check.
#[test]
fn system_mode_blocks_check_every_access_against_the_guest_ranges() {
    let check = crate::x86_64::RangeCheck::FIVE_LEVEL.pinned_code();
    let count = |code: &[u8]| {
        code.windows(check.len())
            .filter(|bytes| *bytes == check)
            .count()
    };
    let compile = |words: &[u32], profile, options: fn(crate::Options) -> crate::Options| {
        let source: Vec<u8> = words.iter().copied().flat_map(u32::to_le_bytes).collect();
        let options = options(crate::Options::new(profile, crate::TargetCpu::detect()));
        crate::compile(&source, GuestAddress(0x1000), &options)
            .unwrap()
            .native
            .x86_64_code
    };
    // ldr w0, [x1]; add w0, w0, #1; ldr x0, [x1, #8]; ldp w2, w3, [x1];
    // ldr h0, [x1, x2]; ld1 {v0.4s}, [x1]; ldr x4, <literal>; ldadd x4,
    // x5, [x1]; ldar w6, [x1]; stlr w6, [x1]; ldnp x8, x9, [x1];
    // str x0, [x1]; ldr q0, [x1]; stp q0, q1, [x1]; prfm pldl1keep, [x1]
    let words = [
        0xb940_0020_u32,
        0x1100_0400,
        0xf940_0420,
        0x2940_0c22,
        0x7c62_6820,
        0x4c40_7820,
        0x5800_0084,
        0xf824_0025,
        0x88df_fc26,
        0x889f_fc26,
        0xa840_2428,
        0xf900_0020,
        0x3dc0_0020,
        0xad00_0420,
        0xf980_0020,
    ];
    // Alone, each access checks, but the prefetch, which never faults.
    for (index, word) in words.into_iter().enumerate() {
        let alone = compile(
            &[word],
            BlockProfile::new(Features::SYSTEM_MODE),
            |options| options,
        );
        assert_eq!(
            count(&alone),
            usize::from(![1, 14].contains(&index)),
            "{word:#x}"
        );
    }
    // Together, an immediate-offset access through X1 after another
    // needs none (`x86_64::guest_range`): the SIMD register offset,
    // which forgets every base, makes the LD1 check again, and the
    // literal always checks.
    let system = compile(
        &words,
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| options,
    );
    assert_eq!(count(&system), 4);
    // Under FEAT_CSV3 each access masks its address, but the LDR and LDP
    // after the first LDR, and the LDR Q and STP Q after the STR, address
    // memory from the masked address kept before them (`x86_64::masked_base`).
    let masked = compile(
        &words,
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| {
            options.with_memory_checks(crate::x86_64::MemoryChecks::system(true, false, true, true))
        },
    );
    assert_eq!(count(&masked), words.len() - 6);
    // A four-level host's blocks, clean-load regions included, check
    // against its placement's ranges.
    const FOUR_LEVEL: crate::x86_64::MemoryChecks =
        crate::x86_64::MemoryChecks::system(true, false, true, false)
            .with_range(crate::x86_64::RangeCheck::four_level(2));
    let four_level = compile(
        &words,
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| options.with_memory_checks(FOUR_LEVEL),
    );
    let four_level_check = crate::x86_64::RangeCheck::four_level(2).pinned_code();
    assert_eq!(count(&four_level), 0);
    assert_eq!(
        four_level
            .windows(four_level_check.len())
            .filter(|bytes| *bytes == four_level_check)
            .count(),
        count(&system)
    );
    assert_eq!(
        count(&compile(&words, BlockProfile::ORDINARY, |options| options)),
        0
    );
    // DC ZVA and DC CVAC completed natively check Xt.
    let zero = compile(
        &[0xd50b_7423],
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| {
            options.with_native_system(crate::x86_64::NativeSystem {
                zero_block: Some(64),
                ..crate::x86_64::NativeSystem::NONE
            })
        },
    );
    let clean = compile(
        &[0xd50b_7a20],
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| {
            options.with_native_system(crate::x86_64::NativeSystem {
                cache_clean: crate::x86_64::cache_clean::Operations::ALL,
                ..crate::x86_64::NativeSystem::NONE
            })
        },
    );
    assert_eq!((count(&zero), count(&clean)), (1, 1));
    // The translated variant checks each access, and each failed check
    // names the access to the helper: its bytes and whether it writes
    // (1) or is atomic (4).
    const SLOT: u64 = 0xffff_8000_0123_4560;
    const TRANSLATED: crate::x86_64::MemoryChecks =
        crate::x86_64::MemoryChecks::system(true, false, true, false)
            .with_range(crate::x86_64::RangeCheck::four_level(2))
            .with_translation(SLOT);
    let accesses = |code: &[u8]| {
        let mut found = vec![];
        let mut cursor = 0;
        while cursor < code.len() {
            match crate::x86_64::RangeCheck::at(&code[cursor..]) {
                Some((bytes, ends)) => {
                    assert_eq!(ends.len(), 11, "a translated check");
                    let call = &code[cursor + ends[5]..cursor + ends[7]];
                    assert_eq!(call[..2], [0x48, 0xa1]);
                    assert_eq!(call[2..10], SLOT.to_le_bytes());
                    let access = &code[cursor + ends[7]..cursor + ends[8]];
                    found.push((access[0], access[1]));
                    cursor += bytes;
                }
                None => cursor += 1,
            }
        }
        found
    };
    let all = compile(
        &words,
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| options.with_memory_checks(TRANSLATED),
    );
    assert_eq!(
        accesses(&all),
        [
            (4, 0),
            (8, 0),
            (8, 0),
            (2, 0),
            (16, 0),
            (8, 0),
            (8, 5),
            (4, 0),
            (4, 1),
            (16, 0),
            (8, 1),
            (16, 0),
            (32, 1),
        ]
    );
    assert_eq!(count(&all), 0);
    let zero = compile(
        &[0xd50b_7423],
        BlockProfile::new(Features::SYSTEM_MODE),
        |options| {
            options
                .with_memory_checks(TRANSLATED)
                .with_native_system(crate::x86_64::NativeSystem {
                    zero_block: Some(64),
                    ..crate::x86_64::NativeSystem::NONE
                })
        },
    );
    assert_eq!(accesses(&zero), [(64, 1)]);
}

#[test]
fn system_mode_blocks_check_sctlr_a_and_the_ordered_granule_when_asked() {
    // ldr x0, [x1, #8]; ldp w2, w3, [x1]; ldr h0, [x1, x2]; ld1 {v0.4s},
    // [x1]; ldr x4, <literal>; ldadd x4, x5, [x1]; ldar w6, [x1];
    // stlr w6, [x1].
    let words = [
        0xf940_0420_u32,
        0x2940_0c22,
        0x7c62_6820,
        0x4c40_7820,
        0x5800_0084,
        0xf824_0025,
        0x88df_fc26,
        0x889f_fc26,
    ];
    let source: Vec<u8> = words.into_iter().flat_map(u32::to_le_bytes).collect();
    let compile = |checks| {
        let options = crate::Options::new(
            BlockProfile::new(Features::SYSTEM_MODE),
            crate::TargetCpu::detect(),
        )
        .with_memory_checks(checks);
        crate::compile(&source, GuestAddress(0x1000), &options)
            .unwrap()
            .native
            .x86_64_code
    };
    // test al,unit-1; cmovne rax,[rdi+unreachable] for each element size.
    let check = |unit: u8| [0xa8, unit - 1, 0x48, 0x0f, 0x45, 0x87, 0x28, 0x03, 0, 0];
    let count = |code: &[u8], unit: u8| {
        code.windows(10)
            .filter(|bytes| *bytes == check(unit))
            .count()
    };
    let default = compile(crate::x86_64::MemoryChecks::DEFAULT);
    assert_eq!([2, 4, 8].map(|unit| count(&default, unit)), [0, 0, 0]);
    let aligned = compile(crate::x86_64::MemoryChecks {
        alignment: true,
        ..crate::x86_64::MemoryChecks::DEFAULT
    });
    // LDR H and the halfword vector are 2; LDP W, LD1 .4S, LDAR and STLR
    // W are 4; LDR X, the literal and LDADD X are 8.
    assert_eq!([2, 4, 8].map(|unit| count(&aligned, unit)), [1, 4, 3]);
    // Without the granule check (nAA set) the ordered accesses keep a
    // crossing path; the atomic keeps its LSE2 granule check.
    let granule = |code: &[u8]| {
        code.windows(4)
            .filter(|bytes| *bytes == [0x80, 0xf9, 16 - 4, 0x76])
            .count()
    };
    let relaxed = compile(crate::x86_64::MemoryChecks {
        ordered_granule: false,
        ..crate::x86_64::MemoryChecks::DEFAULT
    });
    assert_eq!(granule(&default), 2);
    assert_eq!(granule(&relaxed), 0);
    assert_ne!(default, relaxed);
}

/// The direct exits a system-mode block reports are the exact stubs at
/// the end of its code, the conditional fallthrough before the target.
#[test]
fn system_mode_blocks_report_their_direct_successor_exits() {
    use crate::x86_64::chain::{DIRECT_EXIT_BYTES, direct_exit_at};
    const BASE: u64 = 0x4000;
    const ADD: u32 = 0x9100_0400; // add x0, x0, #1
    let fallthrough = (BASE + 8, ExitKind::Fallthrough);
    for (word, expected) in [
        (0x1400_0002_u32, vec![(BASE + 12, ExitKind::Direct)]), // b +8
        (
            0x5400_0040,
            vec![fallthrough, (BASE + 12, ExitKind::Direct)],
        ), // b.eq +8
        (0x5400_004e, vec![(BASE + 12, ExitKind::Direct)]),     // b.al +8
        (
            0xb400_0040,
            vec![fallthrough, (BASE + 12, ExitKind::Direct)],
        ), // cbz x0, +8
        (
            0x3700_0040,
            vec![fallthrough, (BASE + 12, ExitKind::Direct)],
        ), // tbnz w0, #0, +8
        (0x9400_0002, vec![(BASE + 12, ExitKind::Call)]),       // bl +8
        (ADD, vec![fallthrough]),
        (0xd65f_03c0, vec![]), // ret
        (0xd63f_0000, vec![]), // blr x0
        (0xd518_1000, vec![]), // msr sctlr_el1, x0
    ] {
        let source: Vec<u8> = [ADD, word].into_iter().flat_map(u32::to_le_bytes).collect();
        let compiled = crate::testing::block_for(
            &source,
            GuestAddress(BASE),
            BlockProfile::new(Features::SYSTEM_MODE),
            crate::TargetCpu::Baseline,
        )
        .unwrap();
        let code = &compiled.native.x86_64_code;
        let exits: Vec<_> = compiled
            .direct_exits
            .iter()
            .map(|exit| {
                // Padded for its offset: patchable where it lies.
                assert_eq!(
                    code[exit.offset..exit.offset + DIRECT_EXIT_BYTES],
                    direct_exit_at(exit.offset as u64, exit.target, exit.kind)
                );
                (exit.target.0, exit.kind)
            })
            .collect();
        assert_eq!(exits, expected, "{word:#x}");
        if let Some(last) = compiled.direct_exits.last() {
            assert_eq!(last.offset + DIRECT_EXIT_BYTES, code.len(), "{word:#x}");
        }
    }
}

/// A run's side exits keep the stubs `successor_exits` finds, so a link
/// patches each exit's own slot: SPTM's chained-fixup loop, whose run keeps
/// X3 in R9 past both branches.
#[test]
fn a_pinned_runs_side_exits_are_where_its_successors_say() {
    use crate::x86_64::chain::{DIRECT_EXIT_BYTES, direct_exit_at};
    const BASE: u64 = 0x4100;
    let words = [
        0xf940_0043_u32, // ldr x3, [x2]
        0x7100_183f,     // cmp w1, #6
        0x5400_01a1,     // b.ne +0x34
        0xb7f8_03e3,     // tbnz x3, #63, +0x7c
        0x9240_8c70,     // and x16, x3, #0xfffffffff
        0xd364_fc71,     // lsr x17, x3, #36
        0x8b0c_0210,     // add x16, x16, x12
        0x8b11_e210,     // add x16, x16, x17, lsl #56
        0xf900_0050,     // str x16, [x2]
        0xd373_fc70,     // lsr x16, x3, #51
        0xf100_021f,     // cmp x16, #0
        0x1a9f_17f0,     // cset w16, eq
        0xd371_fc71,     // lsr x17, x3, #49
        0x927e_2e31,     // and x17, x17, #0x3ffc
        0x17ff_ffec,     // b -0x50
    ];
    let source: Vec<u8> = words.into_iter().flat_map(u32::to_le_bytes).collect();
    let compiled = crate::testing::block_for(
        &source,
        GuestAddress(BASE),
        BlockProfile::new(Features::SYSTEM_MODE),
        crate::TargetCpu::Baseline,
    )
    .unwrap();
    let code = &compiled.native.x86_64_code;
    // MOV [RDI+0x18], R9: the run stores X3 from its local.
    assert!(
        code.windows(4)
            .any(|bytes| bytes == [0x4c, 0x89, 0x4f, 0x18])
    );
    let exits: Vec<_> = compiled
        .direct_exits
        .iter()
        .map(|exit| {
            assert_eq!(
                code[exit.offset..exit.offset + DIRECT_EXIT_BYTES],
                direct_exit_at(exit.offset as u64, exit.target, exit.word())
            );
            (exit.target.0, exit.skipped)
        })
        .collect();
    assert_eq!(
        exits,
        [
            (BASE + 0x38 - 0x50, 0),
            (BASE + 0x3c, 12),
            (BASE + 0x88, 11)
        ]
    );
    // The range check's failure follows the exits.
    let last = compiled.direct_exits.last().unwrap();
    assert_eq!(
        last.offset + DIRECT_EXIT_BYTES,
        crate::x86_64::chain::without_failures(code).len()
    );
}

/// With a DC ZVA block size, a terminal DC ZVA zeroes natively and falls
/// through a chainable direct exit; every other System instruction, and
/// DC ZVA without one, keeps its System exit.
#[test]
fn system_mode_blocks_complete_dc_zva_natively_only_with_a_block_size() {
    use crate::x86_64::chain::{DIRECT_EXIT_BYTES, is_direct_exit};
    const BASE: u64 = 0x4000;
    const ADD: u32 = 0x9100_0400; // add x0, x0, #1
    const DC_ZVA: u32 = 0xd50b_7423; // dc zva, x3
    const DC_GVA: u32 = 0xd50b_7463; // dc gva, x3
    const MRS: u32 = 0xd538_d080; // mrs x0, tpidr_el1
    let options = |profile, zero_block| {
        crate::Options::new(profile, crate::TargetCpu::Baseline).with_native_system(
            crate::x86_64::NativeSystem {
                zero_block,
                ..crate::x86_64::NativeSystem::NONE
            },
        )
    };
    let compile = |word: u32, bytes| {
        let source: Vec<u8> = [ADD, word, ADD]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let options = options(BlockProfile::new(Features::SYSTEM_MODE), bytes);
        crate::compile(&source, GuestAddress(BASE), &options).unwrap()
    };
    let native = compile(DC_ZVA, Some(64));
    assert_eq!(native.source_bytes, 8, "DC ZVA still ends the block");
    let code = &native.native.x86_64_code;
    let at = native.native.source_map[1].1;
    let mut zeroing = Code::pinned();
    crate::x86_64::zero_block::emit(&mut zeroing, 3, 64, crate::x86_64::MemoryChecks::DEFAULT);
    assert_eq!(code[at..at + zeroing.len()], zeroing);
    let next = GuestAddress(BASE + 8);
    let slot = crate::x86_64::chain::link_slot(Some(BASE), next, ExitKind::Fallthrough);
    let exit = at + zeroing.len() + slot;
    assert_eq!(
        code[exit..],
        crate::x86_64::chain::direct_exit_at(exit as u64, next, ExitKind::Fallthrough)
    );
    assert_eq!(native.direct_exits.len(), 1);
    assert_eq!(
        (native.direct_exits[0].target, native.direct_exits[0].kind),
        (next, ExitKind::Fallthrough)
    );
    assert_eq!(
        native.direct_exits[0].offset + DIRECT_EXIT_BYTES,
        code.len()
    );
    let system = GuestAddress(BASE + 4);
    for (word, bytes) in [(DC_ZVA, None), (DC_GVA, Some(64)), (MRS, Some(64))] {
        let compiled = compile(word, bytes);
        let code = &compiled.native.x86_64_code;
        let exit = &code[code.len() - DIRECT_EXIT_BYTES..];
        assert!(is_direct_exit(exit, system, ExitKind::System), "{word:#x}");
        assert!(compiled.direct_exits.is_empty(), "{word:#x}");
    }
    let add = |profile, bytes| {
        crate::compile(
            &ADD.to_le_bytes(),
            GuestAddress(BASE),
            &options(profile, bytes),
        )
    };
    for bytes in [2, 48, 4096] {
        assert!(add(BlockProfile::new(Features::SYSTEM_MODE), Some(bytes)).is_err());
    }
    assert!(add(BlockProfile::new(Features::SYSTEM), Some(64)).is_err());
    assert!(add(BlockProfile::new(Features::SYSTEM), None).is_ok());
}

/// A DC clean operation in the plan's set reads the byte at Xt and falls
/// through; one outside it, and DC IVAC, keep the System exit.
#[test]
fn planned_dc_clean_operations_check_xt_and_fall_through() {
    use crate::x86_64::cache_clean::Operations;
    use crate::x86_64::chain::{DIRECT_EXIT_BYTES, is_direct_exit};
    const BASE: u64 = 0x4000;
    const ADD: u32 = 0x9100_0400; // add x0, x0, #1
    const DC_CIVAC: u32 = 0xd50b_7e20; // dc civac, x0
    const DC_CVAU: u32 = 0xd50b_7b20; // dc cvau, x0
    const DC_IVAC: u32 = 0xd508_7620; // dc ivac, x0
    let options = |profile, cache_clean| {
        crate::Options::new(profile, crate::TargetCpu::Baseline).with_native_system(
            crate::x86_64::NativeSystem {
                cache_clean,
                ..crate::x86_64::NativeSystem::NONE
            },
        )
    };
    let compile = |word: u32, operations| {
        let source: Vec<u8> = [ADD, word, ADD]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let options = options(BlockProfile::new(Features::SYSTEM_MODE), operations);
        crate::compile(&source, GuestAddress(BASE), &options).unwrap()
    };
    let native = compile(DC_CIVAC, Operations::CIVAC);
    assert_eq!(native.source_bytes, 8, "the DC still ends the block");
    let code = &native.native.x86_64_code;
    let at = native.native.source_map[1].1;
    let mut check = Code::pinned();
    crate::x86_64::cache_clean::emit(&mut check, 0, crate::x86_64::MemoryChecks::DEFAULT);
    assert_eq!(code[at..at + check.len()], check);
    let next = GuestAddress(BASE + 8);
    let slot = crate::x86_64::chain::link_slot(Some(BASE), next, ExitKind::Fallthrough);
    let exit = at + check.len() + slot;
    assert_eq!(
        code[exit..],
        crate::x86_64::chain::direct_exit_at(exit as u64, next, ExitKind::Fallthrough)
    );
    assert_eq!(native.direct_exits.len(), 1);
    assert_eq!(native.direct_exits[0].target, next);
    let system = GuestAddress(BASE + 4);
    for (word, operations) in [
        (DC_CIVAC, Operations::CVAC.with(Operations::CVAU)),
        (DC_CVAU, Operations::NONE),
        (DC_IVAC, Operations::ALL),
    ] {
        let compiled = compile(word, operations);
        let code = &compiled.native.x86_64_code;
        let exit = &code[code.len() - DIRECT_EXIT_BYTES..];
        assert!(is_direct_exit(exit, system, ExitKind::System), "{word:#x}");
    }
    let system = options(BlockProfile::new(Features::SYSTEM), Operations::CVAC);
    assert!(crate::compile(&ADD.to_le_bytes(), GuestAddress(BASE), &system).is_err());
}

#[test]
fn system_mode_profile_keeps_ordinary_memory_and_exits_on_system_instructions() {
    const BASE: GuestAddress = GuestAddress(0x4000);
    const ADD: u32 = 0x9100_0400; // add x0, x0, #1
    const LDR: u32 = 0xf940_0020; // ldr x0, [x1]
    const STP: u32 = 0xa900_0be1; // stp x1, x2, [sp]
    const DMB: u32 = 0xd503_3bbf; // dmb ish
    const YIELD: u32 = 0xd503_203f;
    const NOP: u32 = 0xd503_201f;
    const BTI_C: u32 = 0xd503_245f;
    // ISB retires as NOP: what it synchronizes exits on its own.
    const ISB: u32 = 0xd503_3fdf;
    let profile = BlockProfile::new(Features::SYSTEM_MODE);
    assert!(profile.has(Features::SYSTEM) && profile.has(Features::SYSTEM_REGISTERS));
    assert!(profile.has(Features::POINTER_AUTHENTICATION));
    assert!(profile.checked_memory().is_none() && !profile.has(Features::CLOCK));
    let prefix = [ADD, LDR, STP, DMB, YIELD, NOP, BTI_C, ISB];
    for (word, kind) in [
        (0xd518_1000_u32, ExitKind::System), // msr sctlr_el1, x0
        (0xd538_d080, ExitKind::System),     // mrs x0, tpidr_el1
        (0xd500_43df, ExitKind::System),     // msr daifset, #3
        (0xd508_8723, ExitKind::System),     // tlbi vae1, x3
        (0xd50b_7420, ExitKind::System),     // dc zva, x0
        (0xd50b_7520, ExitKind::System),     // ic ivau, x0
        (0xd503_3f9f, ExitKind::System),     // dsb sy
        (0xd503_207f, ExitKind::System),     // wfi
        (0xd400_0001, ExitKind::System),     // svc #0
        (0xd400_0002, ExitKind::System),     // hvc #0
        (0xd69f_03e0, ExitKind::System),     // eret
        (0x0000_0000, ExitKind::System),     // udf #0
        (0x0020_1420, ExitKind::System),     // genter (Apple GXF)
        (0x0020_1400, ExitKind::System),     // gexit (Apple GXF)
        (0x0020_0820, ExitKind::System),     // wkdmc x0, x1 (Apple)
        (0x0020_0c20, ExitKind::System),     // wkdmd x0, x1 (Apple)
        (0x0020_1428, ExitKind::System),     // genter #8 (Apple GXF)
        (0x0020_1308, ExitKind::System),     // dc incpa4k, x8 (Apple)
        (0x0020_1330, ExitKind::System),     // Apple operation 25, x16
        (0x0020_134d, ExitKind::System),     // Apple operation 26, x13
        (0x0020_1401, ExitKind::System),     // Apple 0x00201401
        (0xc04c_73e5, ExitKind::System),     // movt x5, zt0[7] (Debug state)
        (0xc04e_03e0, ExitKind::System),     // movt zt0[0], x0 (Debug state)
        (0xd503_237f, ExitKind::PointerAuthentication), // pacibsp
        (0x0020_1220, ExitKind::Amx),        // AMX set
        (0x0020_1003, ExitKind::Amx),        // AMX ldx x3
        (0x0020_12e0, ExitKind::Amx),        // AMX operation 23
    ] {
        let source: Vec<u8> = prefix
            .iter()
            .chain([word, ADD].iter())
            .flat_map(|word| word.to_le_bytes())
            .collect();
        let terminal_pc = GuestAddress(BASE.0 + 4 * prefix.len() as u64);
        let analysis = analyze_with_profile(&source, BASE, profile).unwrap();
        assert_eq!(analysis.terminator_pc, Some(terminal_pc), "{word:#x}");
        assert_eq!(analysis.source_bytes, 4 * (prefix.len() + 1));
        let compiled =
            crate::testing::block_for(&source, BASE, profile, crate::TargetCpu::Baseline).unwrap();
        let map = &compiled.native.source_map;
        assert_eq!(map.len(), prefix.len() + 1, "{word:#x}");
        assert!(
            map.iter()
                .enumerate()
                .all(|(index, (pc, _))| pc.0 == BASE.0 + 4 * index as u64)
        );
        let at = map.last().unwrap().1;
        let end = compiled.native.x86_64_code.len()
            - 2 * compiled.native.x86_64_code[at..]
                .rchunks_exact(2)
                .take_while(|bytes| *bytes == [0x0f, 0x0b])
                .count();
        assert!(
            crate::x86_64::chain::is_direct_exit(
                &compiled.native.x86_64_code[at..end],
                terminal_pc,
                kind
            ),
            "{word:#x}"
        );
    }
    // The shared lane's PEs still end a block at ISB.
    let shared = BlockProfile::new(Features::SYSTEM_MODE)
        .with_checked_memory(CheckedMemoryProfile::SpAlignmentEnabled);
    let source: Vec<u8> = [ADD, ISB, ADD]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    let analysis = analyze_with_profile(&source, BASE, shared).unwrap();
    assert_eq!(analysis.terminator_pc, Some(GuestAddress(BASE.0 + 4)));
    // Each access checks the guest range.
    let ordinary: Vec<u8> = [ADD, LDR, ADD, ADD, ADD, 0x1400_0000]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let compiled =
        crate::testing::block_for(&ordinary, BASE, profile, crate::TargetCpu::Baseline).unwrap();
    let check = crate::x86_64::RangeCheck::FIVE_LEVEL.pinned_code();
    assert!(
        compiled
            .native
            .x86_64_code
            .windows(check.len())
            .any(|bytes| bytes == check),
        "the load checks its address"
    );
    // The trap and system decoders of other profiles do not see SYS.
    assert!(
        analyze_with_profile(
            &0xd508_8723_u32.to_le_bytes(),
            BASE,
            BlockProfile::new(Features::SYSTEM)
        )
        .is_err()
    );
}

#[test]
fn system_mode_compiles_locked_atomics_and_fences_for_concurrent_pes() {
    const BASE: GuestAddress = GuestAddress(0x1000);
    // casp x2, x3, x2, x3, [x4] (Rs = Rt), ldar x0, [x1], dmb sy, b .
    const CASP_X2_X3_X4: u32 = 0x4822_7c82;
    const LDAR_X0_X1: u32 = 0xc8df_fc20;
    const DMB_SY: u32 = 0xd503_3fbf;
    const B_SELF: u32 = 0x1400_0000;
    const LOCK: u8 = 0xf0;
    const MFENCE: [u8; 3] = [0x0f, 0xae, 0xf0];
    let source: Vec<u8> = [CASP_X2_X3_X4, LDAR_X0_X1, DMB_SY, B_SELF]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    // The pair needs CMPXCHG16B, which the baseline target lacks.
    for profile in [
        BlockProfile::ORDINARY,
        BlockProfile::new(Features::SYSTEM_MODE),
    ] {
        let plan = CompilationPlan::new(
            &source,
            BASE,
            &crate::Options::new(profile, crate::TargetCpu::Baseline),
        );
        assert!(plan.is_err(), "{profile:?}");
    }
    let target = crate::TargetCpu::X86_64_V2.with_atomic_load16();
    let code = CompilationPlan::new(
        &source,
        BASE,
        &crate::Options::new(BlockProfile::new(Features::SYSTEM_MODE), target),
    )
    .unwrap()
    .compile()
    .unwrap()
    .native
    .x86_64_code;
    assert!(code.contains(&LOCK), "{code:02x?}");
    assert!(code.windows(MFENCE.len()).any(|window| window == MFENCE));
}

/// A fault applies exactly the entries covering its offset, SP as 31.
#[test]
fn fault_recovery_applies_the_covering_entries() {
    let table = [
        FaultRecovery {
            site: crate::block::RecoverySite::native_internal(),
            start: 4,
            end: 10,
            retirement: RetirementPoint::before(GuestAddress(0x1000), 0).unwrap(),
            bindings: vec![(3, 9), (31, 6)]
                .into_iter()
                .map(|(guest, host)| RecoveryBinding::gpr(guest, host))
                .collect(),
        },
        FaultRecovery {
            site: crate::block::RecoverySite::native_internal(),
            start: 10,
            end: 20,
            retirement: RetirementPoint::before(GuestAddress(0x1000), 0).unwrap(),
            bindings: vec![(4, 10)]
                .into_iter()
                .map(|(guest, host)| RecoveryBinding::gpr(guest, host))
                .collect(),
        },
    ];
    let host: [u64; 16] = core::array::from_fn(|register| 0x100 + register as u64);
    let mut state = crate::GuestState::default();
    apply_recovery(&table, 9, &HostGprSnapshot::legacy(host), &mut state).unwrap();
    assert_eq!((state.x[3], state.sp, state.x[4]), (0x109, 0x106, 0));
    let mut state = crate::GuestState::default();
    apply_recovery(&table, 10, &HostGprSnapshot::legacy(host), &mut state).unwrap();
    assert_eq!((state.x[3], state.sp, state.x[4]), (0, 0, 0x10a));
    let mut state = crate::GuestState::default();
    apply_recovery(&table, 3, &HostGprSnapshot::legacy(host), &mut state).unwrap();
    assert_eq!(state, crate::GuestState::default());
}

/// The process and service profiles share the Ordinary emission of an
/// ordinary prefix: locked atomics and no guest range checks, which
/// belong to the system-mode engine alone.
#[test]
fn only_system_mode_compiles_range_checks() {
    const BASE: GuestAddress = GuestAddress(0x1000);
    // ldaddal x1, x0, [x2]; ldr x3, [x4]; add x0, x0, #1; b .
    let source: Vec<u8> = [0xf8e1_0040_u32, 0xf940_0083, 0x9100_0400, 0x1400_0000]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let code = |profile| {
        crate::testing::block_for(&source, BASE, profile, crate::TargetCpu::Baseline)
            .unwrap()
            .native
            .x86_64_code
    };
    let ordinary = code(BlockProfile::ORDINARY);
    for profile in [
        BlockProfile::new(Features::THREAD_REGISTERS.union(Features::AMX)),
        BlockProfile::new(Features::POINTER_AUTHENTICATION),
        BlockProfile::new(Features::POINTER_AUTHENTICATION.union(Features::CLOCK)),
        BlockProfile::new(
            Features::POINTER_AUTHENTICATION
                .union(Features::THREAD_REGISTERS)
                .union(Features::AMX),
        ),
        BlockProfile::new(Features::PROCESS_HELPERS.union(Features::AMX)),
    ] {
        assert_eq!(code(profile), ordinary, "{profile:?}");
    }
    let check = crate::x86_64::RangeCheck::FIVE_LEVEL.pinned_code();
    let has_check = |code: &[u8]| code.windows(check.len()).any(|bytes| bytes == check);
    assert!(!has_check(&ordinary));
    let system = code(BlockProfile::new(Features::SYSTEM_MODE));
    assert!(has_check(&system));
    assert_ne!(system, ordinary);
}

#[test]
fn system_profile_has_one_source_bound_exit_and_rejects_memory_prefixes() {
    const BASE: GuestAddress = GuestAddress(0x1000);
    const ADD: u32 = 0x9100_0400;
    const SVC: u32 = 0xd400_0001;
    const UDF: u32 = 0x0000_0042;
    const IRG: u32 = 0x9ac0_1000;
    const LDG: u32 = 0xd960_0000;
    const RETURN: u8 = 0xc3;
    const MOV_RAX_IMMEDIATE: [u8; 2] = [0x48, 0xb8];
    const MOV_EDX_IMMEDIATE: u8 = 0xba;
    for word in [SVC, UDF, IRG, LDG] {
        let source: Vec<_> = [ADD, word, ADD]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let analysis =
            analyze_with_profile(&source, BASE, BlockProfile::new(Features::SYSTEM)).unwrap();
        let instruction = analysis.system_instruction.unwrap();
        assert_eq!(analysis.source_bytes, 2 * size_of::<u32>());
        assert_eq!(
            instruction.instruction_pc,
            GuestAddress(BASE.0 + size_of::<u32>() as u64)
        );
        let compiled = crate::testing::block_for(
            &source,
            BASE,
            BlockProfile::new(Features::SYSTEM),
            crate::TargetCpu::Baseline,
        )
        .unwrap();
        let at = compiled.native.source_map.last().unwrap().1;
        let mut expected = MOV_RAX_IMMEDIATE.to_vec();
        expected.extend(instruction.instruction_pc.0.to_le_bytes());
        expected.push(MOV_EDX_IMMEDIATE);
        expected.extend((ExitKind::System as u32).to_le_bytes());
        expected.push(RETURN);
        // The room a jump at the RET and its pad take.
        expected.extend([0xcc; 8]);
        assert_eq!(&compiled.native.x86_64_code[at..], expected);
    }
    // Ordinary RAM, exclusives and legacy FP/SIMD cannot bypass shared owners.
    for word in [0xf940_0000_u32, 0xf900_0000, 0xc85f_fc00, 0x1e20_2800] {
        let source: Vec<_> = [word, SVC].into_iter().flat_map(u32::to_le_bytes).collect();
        assert!(analyze_with_profile(&source, BASE, BlockProfile::new(Features::SYSTEM)).is_err());
    }
    assert_eq!(
        ExitKind::try_from(ExitKind::System as u64),
        Ok(ExitKind::System)
    );
    assert_ne!(
        BlockProfile::new(Features::SYSTEM).abi(),
        BlockProfile::ORDINARY.abi()
    );
    assert!(
        analyze_with_profile(&SVC.to_le_bytes(), BASE, BlockProfile::ORDINARY)
            .unwrap()
            .trap
            .is_some()
    );
}
use super::*;

fn bytes(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|word| word.to_le_bytes()).collect()
}

#[test]
fn half_division_continues_its_block_on_every_target() {
    let base = GuestAddress(0x1000);
    let continuation = GuestAddress(base.0 + size_of::<u32>() as u64);
    // FDIV V0.8H,V0.8H,V0.8H
    let source = bytes(&[0x6e403c00]);
    let compile = |target| {
        crate::compile(
            &source,
            base,
            &crate::Options::new(BlockProfile::ORDINARY, target),
        )
        .unwrap()
    };
    let exit = |pc: GuestAddress, kind: ExitKind| {
        let mut exit = vec![0x48, 0xb8]; // MOVABS RAX,PC
        exit.extend(pc.0.to_le_bytes());
        exit.push(0xba); // MOV EDX,kind; RET
        exit.extend((kind as u32).to_le_bytes());
        exit.extend([0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc]);
        exit
    };
    // VDIVPH XMM0,XMM0,XMM1 with EVEX.
    let native = |code: &[u8]| {
        code.windows(4)
            .any(|window| window == [0x7c, 0x89, 0x5e, 0xc1])
    };
    let helper_exit = exit(base, ExitKind::FloatingPoint);
    for (target, half_instruction) in [
        (crate::TargetCpu::Baseline, false),
        (crate::TargetCpu::X86_64_V3, false),
        (crate::TargetCpu::X86_64_V4, false),
        (crate::TargetCpu::X86_64_V4.with_avx512fp16(), true),
    ] {
        let compiled = compile(target);
        assert_eq!(compiled.source_bytes, size_of::<u32>());
        let code = &compiled.native.x86_64_code;
        assert!(
            code.windows(helper_exit.len())
                .any(|window| window == helper_exit)
        );
        assert_eq!(native(code), half_instruction);
        let fallthrough = exit(continuation, ExitKind::Fallthrough);
        assert!(
            code.windows(fallthrough.len())
                .any(|window| window == fallthrough)
        );
    }
}

#[test]
fn guarded_half_arithmetic_validates_its_inline_suffix_in_every_profile() {
    const NOP: u32 = 0xd503201f;
    const UNSUPPORTED: u32 = 0xffffffff;
    let base = GuestAddress(0x1000);
    for operation in [
        0x1ee02800, 0x1ee03800, 0x1ee00800, // scalar ADD/SUB/MUL
        0x0e401400, 0x0ec01400, 0x2e401c00, // four lanes
        0x4e401400, 0x4ec01400, 0x6e401c00, // eight lanes
        0x1ee01800, 0x2e403c00, 0x6e403c00, // FDIV H, 4H, 8H
    ] {
        for profile in [
            BlockProfile::ORDINARY,
            BlockProfile::new(Features::POINTER_AUTHENTICATION),
            BlockProfile::new(Features::POINTER_AUTHENTICATION.union(Features::CLOCK)),
            BlockProfile::new(Features::THREAD_REGISTERS.union(Features::AMX)),
            BlockProfile::new(
                Features::POINTER_AUTHENTICATION
                    .union(Features::THREAD_REGISTERS)
                    .union(Features::AMX),
            ),
            BlockProfile::new(Features::PROCESS_HELPERS.union(Features::AMX)),
            BlockProfile::new(Features::SYSTEM_MODE),
        ] {
            let source = bytes(&[NOP, operation, NOP]);
            let analysis = analyze_with_profile(&source, base, profile).unwrap();
            let compiled = crate::testing::block_with(&source, base, profile).unwrap();
            assert_eq!(analysis.source_bytes, source.len());
            assert_eq!(compiled.source_bytes, analysis.source_bytes);
            assert_eq!(
                source_bytes_with_profile(&source, base, profile).unwrap(),
                source.len()
            );
            assert_eq!(compiled.native.source_map.len(), 3);
            let invalid = bytes(&[NOP, operation, UNSUPPORTED]);
            let suffix_pc = GuestAddress(base.0 + 2 * size_of::<u32>() as u64);
            assert_eq!(
                analyze_with_profile(&invalid, base, profile)
                    .unwrap_err()
                    .pc,
                suffix_pc
            );
            assert!(crate::testing::block_with(&invalid, base, profile).is_err());
            assert!(source_bytes_with_profile(&invalid, base, profile).is_err());
        }
    }
}

#[test]
fn analysis_reports_transfer_roles_for_each_branch_kind() {
    let base = GuestAddress(0x1000);
    for (word, kinds, targets, dynamic) in [
        (0x14000002, vec![SuccessorKind::Direct], vec![0x1008], None),
        (
            0x54000040,
            vec![
                SuccessorKind::ConditionalTarget,
                SuccessorKind::ConditionalFallthrough,
            ],
            vec![0x1008, 0x1004],
            None,
        ),
        (
            0xb4000040,
            vec![
                SuccessorKind::ConditionalTarget,
                SuccessorKind::ConditionalFallthrough,
            ],
            vec![0x1008, 0x1004],
            None,
        ),
        (
            0x36000040,
            vec![
                SuccessorKind::ConditionalTarget,
                SuccessorKind::ConditionalFallthrough,
            ],
            vec![0x1008, 0x1004],
            None,
        ),
        (
            0x94000002,
            vec![SuccessorKind::CallTarget, SuccessorKind::CallContinuation],
            vec![0x1008, 0x1004],
            None,
        ),
        (
            0xd61f0120,
            vec![],
            vec![],
            Some((DynamicEdgeKind::Branch, 9)),
        ),
        (
            0xd63f0120,
            vec![SuccessorKind::CallContinuation],
            vec![0x1004],
            Some((DynamicEdgeKind::Call, 9)),
        ),
        (
            0xd65f03c0,
            vec![],
            vec![],
            Some((DynamicEdgeKind::Return, 30)),
        ),
        (
            0xd61f03e0,
            vec![],
            vec![],
            Some((DynamicEdgeKind::Branch, 31)),
        ),
    ] {
        let source = bytes(&[word, 0]);
        let analysis = analyze(&source, base).unwrap();
        assert_eq!(analysis.source_bytes, source_bytes(&source, base).unwrap());
        assert_eq!(
            analysis.source_bytes,
            crate::testing::block(&source, base).unwrap().source_bytes
        );
        assert_eq!(analysis.source_bytes, 4);
        assert_eq!(
            analysis
                .static_successors
                .iter()
                .map(|edge| edge.kind)
                .collect::<Vec<_>>(),
            kinds
        );
        assert_eq!(
            analysis
                .static_successors
                .iter()
                .map(|edge| edge.target.0)
                .collect::<Vec<_>>(),
            targets
        );
        assert_eq!(
            analysis.dynamic_edge.map(|edge| (edge.kind, edge.register)),
            dynamic
        );
    }
    let linear = analyze(&bytes(&[0xd503201f; 256]), base).unwrap();
    assert_eq!(
        linear.static_successors[0],
        StaticSuccessor {
            kind: SuccessorKind::Fallthrough,
            target: GuestAddress(0x1400)
        }
    );
}
#[test]
fn analysis_reports_exact_decline_pc_and_never_follows_trap_or_post_terminator_data() {
    let base = GuestAddress(0x1000);
    let source = bytes(&[0xd503201f, 0, 0xd65f03c0]);
    let failure = analyze(&source, base).unwrap_err();
    assert_eq!(failure.pc, GuestAddress(0x1004));
    assert!(matches!(
        failure.kind,
        crate::CompileErrorKind::Unsupported(_)
    ));
    assert_eq!(failure.encoding, Some(0));
    for word in [0xd4001001, 0xd43fffe0] {
        let analysis = analyze(&bytes(&[0xd503201f, word, 0]), base).unwrap();
        assert_eq!(analysis.source_bytes, 8);
        assert!(analysis.static_successors.is_empty());
        assert!(analysis.dynamic_edge.is_none());
        assert!(analysis.trap.is_some());
    }
    assert!(analyze(&[], base).is_err());
    assert!(analyze(&bytes(&[0xd503201f; 257]), base).is_err());
    assert_eq!(
        analyze(&bytes(&[0xd503201f]), GuestAddress(u64::MAX - 3))
            .unwrap_err()
            .pc,
        GuestAddress(u64::MAX - 3)
    );
}

#[test]
fn exit_layout_and_kind_validation_are_explicit() {
    assert_eq!(core::mem::size_of::<BlockExit>(), 16);
    assert_eq!(core::mem::offset_of!(BlockExit, next_pc), 0);
    assert_eq!(core::mem::offset_of!(BlockExit, kind), 8);
    for kind in 0..=ExitKind::Amx as u64 {
        assert_eq!(ExitKind::try_from(kind).unwrap() as u64, kind);
    }
    assert_eq!(
        ExitKind::try_from(ExitKind::Amx as u64 + 1),
        Err(InvalidExitKind {
            value: ExitKind::Amx as u64 + 1
        })
    );
    assert_eq!(
        ExitKind::try_from(u64::MAX),
        Err(InvalidExitKind { value: u64::MAX })
    );
}

#[test]
fn clock_decode_accepts_all_counter_destinations_and_only_exact_controls() {
    let pc = GuestAddress(0x1000);
    for destination in 0..32 {
        assert_eq!(
            decode_clock_instruction(pc, 0xd53be040 | destination).unwrap(),
            Some(ClockInstruction {
                operation: ClockOperation::ReadVirtualCounter {
                    destination: destination as u8
                },
                instruction_pc: pc,
                next_pc: GuestAddress(pc.0 + 4),
            })
        );
    }
    assert_eq!(
        decode_clock_instruction(pc, 0xd5033fdf).unwrap(),
        Some(ClockInstruction {
            operation: ClockOperation::InstructionSynchronization,
            instruction_pc: pc,
            next_pc: GuestAddress(pc.0 + 4),
        })
    );
    for bit in 5..32 {
        assert!(
            decode_clock_instruction(pc, 0xd53be040 ^ (1 << bit))
                .unwrap()
                .is_none()
        );
    }
    for bit in 0..32 {
        assert!(
            decode_clock_instruction(pc, 0xd5033fdf ^ (1 << bit))
                .unwrap()
                .is_none()
        );
    }
    // CNTFRQ/CNTPCT/CNTVCTSS, MSR and other barrier options are not claimed.
    for word in [0xd53be000, 0xd53be020, 0xd53be0c0, 0xd51be040, 0xd5033bdf] {
        assert!(decode_clock_instruction(pc, word).unwrap().is_none());
    }
    for word in [0xd53be040, 0xd5033fdf] {
        assert!(decode_clock_instruction(GuestAddress(pc.0 + 1), word).is_err());
        assert!(decode_clock_instruction(GuestAddress(u64::MAX - 3), word).is_err());
    }
}

#[test]
fn clock_profile_boundaries_share_analysis_and_do_not_widen_old_profiles() {
    let base = GuestAddress(0x4000);
    let profile = BlockProfile::new(Features::POINTER_AUTHENTICATION.union(Features::CLOCK));
    for word in [0xd5033fdf, 0xd53be040, 0xd53be05f] {
        let source = bytes(&[0xd2800020, word, 0]);
        for previous in [
            BlockProfile::ORDINARY,
            BlockProfile::new(Features::POINTER_AUTHENTICATION),
        ] {
            assert_eq!(
                analyze_with_profile(&source, base, previous)
                    .unwrap_err()
                    .pc,
                GuestAddress(base.0 + 4)
            );
            assert!(crate::testing::block_with(&source, base, previous).is_err());
            assert!(source_bytes_with_profile(&source, base, previous).is_err());
        }
        let analysis = analyze_with_profile(&source, base, profile).unwrap();
        let compiled = crate::testing::block_with(&source, base, profile).unwrap();
        assert_eq!(analysis.source_bytes, 8);
        assert_eq!(compiled.source_bytes, 8);
        assert_eq!(
            source_bytes_with_profile(&source, base, profile).unwrap(),
            8
        );
        let request = analysis.clock_instruction.unwrap();
        assert_eq!(request.instruction_pc, GuestAddress(base.0 + 4));
        assert_eq!(
            analysis.static_successors,
            [StaticSuccessor {
                kind: SuccessorKind::Fallthrough,
                target: request.next_pc
            }]
        );
        assert!(
            analysis.dynamic_edge.is_none()
                && analysis.trap.is_none()
                && analysis.pointer_authentication.is_none()
        );
        assert_eq!(
            compiled
                .native
                .source_map
                .iter()
                .map(|(pc, _)| *pc)
                .collect::<Vec<_>>(),
            [base, request.instruction_pc]
        );
    }
    for word in [0xd503237f, 0xd50323ff, 0xd65f0fff] {
        let source = bytes(&[word, 0]);
        assert_eq!(
            analyze_with_profile(&source, base, profile).unwrap(),
            analyze_with_profile(
                &source,
                base,
                BlockProfile::new(Features::POINTER_AUTHENTICATION)
            )
            .unwrap()
        );
    }
    assert!(profile.has(Features::POINTER_AUTHENTICATION) && profile.has(Features::CLOCK));
    assert!(!BlockProfile::ORDINARY.has(Features::POINTER_AUTHENTICATION));
    assert!(!BlockProfile::new(Features::POINTER_AUTHENTICATION).has(Features::CLOCK));
}

#[test]
fn clock_boundary_respects_window_limit_and_does_not_skip_prior_unknown_words() {
    let base = GuestAddress(0x1000);
    let profile = BlockProfile::new(Features::POINTER_AUTHENTICATION.union(Features::CLOCK));
    let first = analyze_with_profile(&bytes(&[0xd53be05f, 0]), base, profile).unwrap();
    assert_eq!(first.source_bytes, 4);
    assert_eq!(
        first.clock_instruction.unwrap().operation,
        ClockOperation::ReadVirtualCounter { destination: 31 }
    );
    let mut last = [0xd503201f; MAX_BLOCK_INSTRUCTIONS];
    last[MAX_BLOCK_INSTRUCTIONS - 1] = 0xd5033fdf;
    let analysis = analyze_with_profile(&bytes(&last), base, profile).unwrap();
    assert_eq!(analysis.source_bytes, MAX_BLOCK_INSTRUCTIONS * 4);
    assert_eq!(
        analysis.clock_instruction.unwrap().instruction_pc,
        GuestAddress(base.0 + (MAX_BLOCK_INSTRUCTIONS as u64 - 1) * 4)
    );
    assert!(
        crate::testing::block_with(
            &bytes(&[0xd53be040; MAX_BLOCK_INSTRUCTIONS + 1]),
            base,
            profile
        )
        .is_err()
    );
    assert_eq!(
        analyze_with_profile(&bytes(&[0xd503201f, 0, 0xd53be040]), base, profile)
            .unwrap_err()
            .pc,
        GuestAddress(base.0 + 4)
    );
}

#[test]
fn pointer_authentication_exact_encodings_and_address_guards() {
    let pc = GuestAddress(0x12340000);
    let encodings = [
        (0xd503237f, PointerAuthenticationOperation::SignIbSp),
        (0xd50323ff, PointerAuthenticationOperation::AuthenticateIbSp),
        (0xd65f0fff, PointerAuthenticationOperation::ReturnIbSp),
    ];
    for (word, operation) in encodings {
        assert_eq!(
            decode_pointer_authentication(pc, word).unwrap(),
            Some(PointerAuthentication {
                operation,
                instruction_pc: pc,
                next_pc: GuestAddress(pc.0 + 4),
            })
        );
        for bit in 0..32 {
            let changed = word ^ (1 << bit);
            if !encodings.iter().any(|(word, _)| *word == changed) {
                assert_ne!(
                    decode_pointer_authentication(pc, changed)
                        .unwrap()
                        .map(|request| request.operation),
                    Some(operation)
                );
            }
        }
        assert!(decode_pointer_authentication(GuestAddress(pc.0 + 1), word).is_err());
        assert!(decode_pointer_authentication(GuestAddress(u64::MAX - 3), word).is_err());
    }
    // Reserved zero-modifier encodings remain outside this boundary.
    for word in [0xd61f0800, 0xd63f0800, 0xdac12000, 0xdac15000] {
        assert_eq!(decode_pointer_authentication(pc, word).unwrap(), None);
    }
}

#[test]
fn basic_pac_integer_fields_distinguish_sp_zero_and_discard() {
    use PointerAuthenticationKey::{Da, Db, Ia, Ib};
    use PointerAuthenticationModifier::{Register, StackPointer, Zero};
    let pc = GuestAddress(0x1000);
    for authenticate in [false, true] {
        for (key_index, key) in [Ia, Ib, Da, Db].into_iter().enumerate() {
            for rn in 0..32u32 {
                for rd in 0..32u32 {
                    let opcode =
                        0xdac10000 | ((authenticate as u32) << 12) | ((key_index as u32) << 10);
                    let expected = |modifier| {
                        if authenticate {
                            PointerAuthenticationOperation::Authenticate {
                                key,
                                destination: rd as u8,
                                modifier,
                            }
                        } else {
                            PointerAuthenticationOperation::Sign {
                                key,
                                destination: rd as u8,
                                modifier,
                            }
                        }
                    };
                    let word = opcode | (rn << 5) | rd;
                    let request = decode_pointer_authentication(pc, word).unwrap().unwrap();
                    assert_eq!(
                        request.operation,
                        expected(if rn == 31 {
                            StackPointer
                        } else {
                            Register(rn as u8)
                        })
                    );
                    assert_eq!(request.operation.required_key(), Some(key));
                    assert!(crate::testing::block(&bytes(&[word]), pc).is_err());
                    let zero_word = word | (1 << 13);
                    let zero = decode_pointer_authentication(pc, zero_word).unwrap();
                    assert_eq!(
                        zero.map(|request| request.operation),
                        (rn == 31).then(|| expected(Zero))
                    );
                }
            }
        }
    }
    // dyld's pointer signer uses X17 and X16 as the data-key operands.
    for (word, key) in [(0xdac10a11, Da), (0xdac10e11, Db)] {
        assert_eq!(
            decode_pointer_authentication(pc, word)
                .unwrap()
                .unwrap()
                .operation,
            PointerAuthenticationOperation::Sign {
                key,
                destination: 17,
                modifier: Register(16)
            }
        );
    }
    for word in [
        0x5ac10000, 0xfac10000, 0xdac00000, 0xdac18000, 0xdac12000, 0xdac15000, 0xd61f0800,
        0xd63f0c00,
    ] {
        assert!(decode_pointer_authentication(pc, word).unwrap().is_none());
    }
}

#[test]
fn generic_code_requires_explicit_profile_and_keeps_helper_source_boundary() {
    const PACGA_X1_X2_X3: u32 = 0x9ac3_3041;
    let pc = GuestAddress(0x1000);
    let source = bytes(&[PACGA_X1_X2_X3]);
    assert!(crate::testing::block(&source, pc).is_err());
    let instruction = decode_pointer_authentication(pc, PACGA_X1_X2_X3)
        .unwrap()
        .unwrap();
    assert_eq!(
        instruction.operation,
        PointerAuthenticationOperation::GenericCode(pauth_memory::GenericCode {
            destination: 1,
            source: 2,
            modifier: 3,
        })
    );
    let analysis = analyze_with_profile(
        &source,
        pc,
        BlockProfile::new(Features::POINTER_AUTHENTICATION),
    )
    .unwrap();
    assert_eq!(analysis.pointer_authentication, Some(instruction));
    assert_eq!(analysis.terminator_pc, Some(pc));
    assert_eq!(
        analysis.static_successors,
        vec![StaticSuccessor {
            kind: SuccessorKind::Fallthrough,
            target: GuestAddress(pc.0 + 4),
        }]
    );
    assert!(analysis.dynamic_edge.is_none());
}

#[test]
fn authenticated_branch_discovery_preserves_dynamic_target_and_call_continuation() {
    const BRAA_X1_SP: u32 = 0xd71f_083f;
    const BLRAA_X1_SP: u32 = 0xd73f_083f;
    let pc = GuestAddress(0x1000);
    for (word, link) in [(BRAA_X1_SP, false), (BLRAA_X1_SP, true)] {
        let analysis = analyze_with_profile(
            &bytes(&[word]),
            pc,
            BlockProfile::new(Features::POINTER_AUTHENTICATION),
        )
        .unwrap();
        assert_eq!(
            analysis.dynamic_edge,
            Some(DynamicEdge {
                kind: if link {
                    DynamicEdgeKind::Call
                } else {
                    DynamicEdgeKind::Branch
                },
                register: 1,
            })
        );
        assert_eq!(
            analysis.static_successors,
            if link {
                vec![StaticSuccessor {
                    kind: SuccessorKind::CallContinuation,
                    target: GuestAddress(pc.0 + 4),
                }]
            } else {
                vec![]
            }
        );
    }
}

#[test]
fn authenticated_branch_fields_distinguish_zero_sp_link_and_instruction_keys() {
    use PointerAuthenticationKey::{Ia, Ib};
    use PointerAuthenticationModifier::{Register, StackPointer, Zero};
    let pc = GuestAddress(0x1000);
    for link in [false, true] {
        for (key_bit, key) in [(0, Ia), (1, Ib)] {
            for zero in [false, true] {
                for target in 0..32u32 {
                    for rm in 0..32u32 {
                        let word = 0xd61f0800
                            | (u32::from(!zero) << 24)
                            | (u32::from(link) << 21)
                            | (key_bit << 10)
                            | (target << 5)
                            | rm;
                        let decoded = decode_pointer_authentication(pc, word).unwrap();
                        if zero && rm != 31 {
                            assert!(decoded.is_none());
                            continue;
                        }
                        let value = decoded.unwrap();
                        assert_eq!(
                            value.operation,
                            PointerAuthenticationOperation::Branch {
                                key,
                                target: target as u8,
                                modifier: if zero {
                                    Zero
                                } else if rm == 31 {
                                    StackPointer
                                } else {
                                    Register(rm as u8)
                                },
                                link,
                            }
                        );
                        assert!(value.operation.is_transfer());
                        assert!(!value.operation.is_return());
                        assert_eq!(value.operation.required_key(), Some(key));
                        assert!(crate::testing::block(&bytes(&[word]), pc).is_err());
                    }
                }
            }
        }
    }
    for word in [0xd71f0000, 0xd75f0800, 0xd79f0800, 0xd71f1800, 0x571f0800] {
        assert!(decode_pointer_authentication(pc, word).unwrap().is_none());
    }
}

#[test]
fn link_register_helpers_preserve_canonical_source_and_legacy_distinctions() {
    let pc = GuestAddress(0x4000);
    for (word, id) in [
        (0xdac1bbfe, "AUTIA171615_64LR_dp_1src"),
        (0xdac1901e, "AUTIASPPCR_64LRR_dp_1src"),
        (0xf380001f, "AUTIASPPC_only_dp_1src_imm"),
        (0xdac1bffe, "AUTIB171615_64LR_dp_1src"),
        (0xdac1941e, "AUTIBSPPCR_64LRR_dp_1src"),
        (0xf3a0001f, "AUTIBSPPC_only_dp_1src_imm"),
        (0xdac18bfe, "PACIA171615_64LR_dp_1src"),
        (0xdac1a3fe, "PACIASPPC_64LR_dp_1src"),
        (0xdac18ffe, "PACIB171615_64LR_dp_1src"),
        (0xdac1a7fe, "PACIBSPPC_64LR_dp_1src"),
        (0xd50324ff, "PACM_HI_hints"),
        (0xdac183fe, "PACNBIASPPC_64LR_dp_1src"),
        (0xdac187fe, "PACNBIBSPPC_64LR_dp_1src"),
        (0xd65f0be0, "RETAASPPCR_64M_branch_reg"),
        (0x5500001f, "RETAASPPC_only_miscbranch"),
        (0xd65f0fe0, "RETABSPPCR_64M_branch_reg"),
        (0x5520001f, "RETABSPPC_only_miscbranch"),
    ] {
        let request = decode_pointer_authentication(pc, word).unwrap().unwrap();
        let PointerAuthenticationOperation::LinkRegister(instruction) = request.operation else {
            panic!("canonical LR form must retain its descriptor");
        };
        assert_eq!(instruction.canonical_id(), id);
        assert_eq!(request.instruction_pc, pc);
        assert_eq!(request.next_pc, GuestAddress(pc.0 + 4));
        assert_eq!(request.operation.is_return(), instruction.is_return());
        let key = instruction.key().map(|key| match key {
            pauth_lr::Key::Ia => PointerAuthenticationKey::Ia,
            pauth_lr::Key::Ib => PointerAuthenticationKey::Ib,
        });
        assert_eq!(request.operation.required_key(), key);
    }
    for (word, key, authenticate) in [
        (0xdac10211, PointerAuthenticationKey::Ia, false),
        (0xdac10611, PointerAuthenticationKey::Ib, false),
        (0xdac11211, PointerAuthenticationKey::Ia, true),
        (0xdac11611, PointerAuthenticationKey::Ib, true),
    ] {
        let request = decode_pointer_authentication(pc, word).unwrap().unwrap();
        let expected = if authenticate {
            PointerAuthenticationOperation::Authenticate {
                key,
                destination: 17,
                modifier: PointerAuthenticationModifier::Register(16),
            }
        } else {
            PointerAuthenticationOperation::Sign {
                key,
                destination: 17,
                modifier: PointerAuthenticationModifier::Register(16),
            }
        };
        assert_eq!(request.operation, expected);
    }
}

#[test]
fn basic_pac_fixed_hints_strip_and_return_have_exact_operands() {
    use PointerAuthenticationKey::{Ia, Ib};
    use PointerAuthenticationModifier::{StackPointer, Zero};
    use PointerAuthenticationOperation::{Authenticate, Sign, Strip};
    let pc = GuestAddress(0x4000);
    let fixed = [
        (
            0xd503211f,
            PointerAuthenticationOperation::Sign1716 { key: Ia },
        ),
        (
            0xd503215f,
            PointerAuthenticationOperation::Sign1716 { key: Ib },
        ),
        (
            0xd503219f,
            PointerAuthenticationOperation::Authenticate1716 { key: Ia },
        ),
        (
            0xd50321df,
            PointerAuthenticationOperation::Authenticate1716 { key: Ib },
        ),
        (
            0xd503231f,
            Sign {
                key: Ia,
                destination: 30,
                modifier: Zero,
            },
        ),
        (
            0xd503233f,
            Sign {
                key: Ia,
                destination: 30,
                modifier: StackPointer,
            },
        ),
        (
            0xd503235f,
            Sign {
                key: Ib,
                destination: 30,
                modifier: Zero,
            },
        ),
        (0xd503237f, PointerAuthenticationOperation::SignIbSp),
        (
            0xd503239f,
            Authenticate {
                key: Ia,
                destination: 30,
                modifier: Zero,
            },
        ),
        (
            0xd50323bf,
            Authenticate {
                key: Ia,
                destination: 30,
                modifier: StackPointer,
            },
        ),
        (
            0xd50323df,
            Authenticate {
                key: Ib,
                destination: 30,
                modifier: Zero,
            },
        ),
        (0xd50323ff, PointerAuthenticationOperation::AuthenticateIbSp),
        (
            0xd50320ff,
            Strip {
                data: false,
                destination: 30,
            },
        ),
        (0xd65f0bff, PointerAuthenticationOperation::ReturnIaSp),
        (0xd65f0fff, PointerAuthenticationOperation::ReturnIbSp),
    ];
    for (word, expected) in fixed {
        assert_eq!(
            decode_pointer_authentication(pc, word)
                .unwrap()
                .unwrap()
                .operation,
            expected
        );
        for bit in 0..32 {
            let changed = word ^ (1 << bit);
            let decoded = decode_pointer_authentication(pc, changed).unwrap();
            assert_eq!(
                decoded.is_some_and(|request| !matches!(
                    request.operation,
                    PointerAuthenticationOperation::Branch { .. }
                        | PointerAuthenticationOperation::LinkRegister(_)
                )),
                fixed.iter().any(|(fixed, _)| *fixed == changed)
            );
        }
        for profile in [
            BlockProfile::new(Features::POINTER_AUTHENTICATION),
            BlockProfile::new(Features::POINTER_AUTHENTICATION.union(Features::CLOCK)),
        ] {
            let analysis = analyze_with_profile(&bytes(&[word, 0]), pc, profile).unwrap();
            assert_eq!(analysis.source_bytes, 4);
            assert_eq!(analysis.dynamic_edge.is_some(), expected.is_return());
            assert_eq!(analysis.static_successors.is_empty(), expected.is_return());
        }
        assert!(crate::testing::block(&bytes(&[word]), pc).is_err());
    }
    for data in [false, true] {
        for destination in 0..32u32 {
            let word = 0xdac143e0 | ((data as u32) << 10) | destination;
            let request = decode_pointer_authentication(pc, word).unwrap().unwrap();
            assert_eq!(
                request.operation,
                Strip {
                    data,
                    destination: destination as u8
                }
            );
            assert_eq!(request.operation.required_key(), None);
            for bit in 0..32 {
                if bit < 5 || bit == 10 {
                    continue;
                }
                assert_ne!(
                    decode_pointer_authentication(pc, word ^ (1 << bit))
                        .unwrap()
                        .map(|request| request.operation),
                    Some(request.operation)
                );
            }
        }
    }
}

#[test]
fn pointer_authentication_profile_consumes_only_native_prefix_and_boundary() {
    let base = GuestAddress(0x4000);
    let profile = BlockProfile::new(Features::POINTER_AUTHENTICATION);
    for word in [0xd503237f, 0xd50323ff, 0xd65f0fff] {
        let source = bytes(&[0xd2800020, word, 0]); // MOV X0,#1; helper; unknown data
        let ordinary = analyze(&source, base).unwrap_err();
        assert_eq!(ordinary.pc, GuestAddress(base.0 + 4));
        assert!(crate::testing::block(&source, base).is_err());
        assert!(source_bytes(&source, base).is_err());
        let analysis = analyze_with_profile(&source, base, profile).unwrap();
        let compiled = crate::testing::block_with(&source, base, profile).unwrap();
        assert_eq!(
            source_bytes_with_profile(&source, base, profile).unwrap(),
            8
        );
        assert_eq!(compiled.source_bytes, 8);
        assert_eq!(analysis.source_bytes, 8);
        assert_eq!(analysis.terminator_pc, Some(GuestAddress(base.0 + 4)));
        assert!(analysis.trap.is_none());
        let request = analysis.pointer_authentication.unwrap();
        assert_eq!(request.instruction_pc, GuestAddress(base.0 + 4));
        assert_eq!(request.next_pc, GuestAddress(base.0 + 8));
        assert_eq!(
            compiled
                .native
                .source_map
                .iter()
                .map(|(pc, _)| *pc)
                .collect::<Vec<_>>(),
            [base, request.instruction_pc]
        );
        if word == 0xd65f0fff {
            assert!(analysis.static_successors.is_empty());
            assert_eq!(
                analysis.dynamic_edge,
                Some(DynamicEdge {
                    kind: DynamicEdgeKind::Return,
                    register: 30
                })
            );
        } else {
            assert_eq!(
                analysis.static_successors,
                [StaticSuccessor {
                    kind: SuccessorKind::Fallthrough,
                    target: request.next_pc
                }]
            );
            assert!(analysis.dynamic_edge.is_none());
        }
        let failure = analyze_with_profile(&bytes(&[0, word]), base, profile).unwrap_err();
        assert_eq!(failure.pc, base);
    }
    assert_ne!(profile.abi(), BlockProfile::ORDINARY.abi());
    // Ordinary transfers and trap behavior are identical in either profile.
    for word in [0xd65f03c0, 0x14000002, 0xd4001001] {
        let source = bytes(&[word, 0]);
        assert_eq!(
            analyze(&source, base).unwrap(),
            analyze_with_profile(&source, base, profile).unwrap()
        );
    }
}

#[test]
fn register_transfer_masks_accept_only_ordinary_br_blr_ret() {
    for (encoding, kind) in [
        (0xd61f0000, ExitKind::Indirect),
        (0xd63f0000, ExitKind::Call),
        (0xd65f0000, ExitKind::Return),
    ] {
        for index in 0..32u32 {
            assert_eq!(
                decode_terminator(GuestAddress(0x1000), encoding | index << 5).unwrap(),
                Some(Terminator::Register {
                    source: if index == 31 {
                        Register::Zero
                    } else {
                        Register::X(index as u8)
                    },
                    kind,
                })
            );
        }
        // Every fixed bit must remain fixed; adjacent authenticated and
        // exception-transfer encodings must not enter this ABI.
        for bit in (0..32).filter(|bit| !(5..10).contains(bit)) {
            let altered = encoding ^ (1 << bit);
            assert!(!matches!(
                decode_terminator(GuestAddress(0x1000), altered),
                Ok(Some(Terminator::Register { kind: actual, .. })) if actual == kind
            ));
        }
    }
    for encoding in [0xd65f0bff, 0xd65f0fff, 0xd71f0800, 0xd73f0800, 0xd69f03e0] {
        assert!(crate::testing::block(&bytes(&[encoding]), GuestAddress(0x1000)).is_err());
    }
}

#[test]
fn direct_calls_sign_extend_and_reject_overflow() {
    assert_eq!(
        decode_terminator(GuestAddress(0x1000), 0x94000003).unwrap(),
        Some(Terminator::DirectCall(GuestAddress(0x100c)))
    );
    assert_eq!(
        decode_terminator(GuestAddress(0x1000), 0x97ffffff).unwrap(),
        Some(Terminator::DirectCall(GuestAddress(0xffc)))
    );
    assert!(decode_terminator(GuestAddress(0), 0x97ffffff).is_err());
    assert!(decode_terminator(GuestAddress(u64::MAX - 3), 0x94000001).is_err());
}

#[test]
fn consumes_only_the_real_prefix_and_maps_no_synthetic_return() {
    for terminator in [
        0x14000001, 0x54000020, 0xb4000020, 0x36000020, 0x94000001, 0xd61f0000, 0xd63f03c0,
        0xd65f03c0,
    ] {
        let compiled = crate::testing::block(
            &bytes(&[0xd503201f, terminator, 0xffffffff]),
            GuestAddress(0x1000),
        )
        .unwrap();
        assert_eq!(compiled.source_bytes, 8);
        assert_eq!(
            source_bytes(
                &bytes(&[0xd503201f, terminator, 0xffffffff]),
                GuestAddress(0x1000)
            )
            .unwrap(),
            8
        );
        assert_eq!(compiled.native.source_map.len(), 2);
        assert_eq!(compiled.native.source_map[0], (GuestAddress(0x1000), 0));
        assert_eq!(compiled.native.source_map[1], (GuestAddress(0x1004), 1));
    }
    let compiled =
        crate::testing::block(&bytes(&[0xd503201f, 0xd503201f]), GuestAddress(0x1000)).unwrap();
    assert_eq!(compiled.source_bytes, 8);
    assert_eq!(
        compiled.native.source_map,
        [(GuestAddress(0x1000), 0), (GuestAddress(0x1004), 1)]
    );
    assert!(
        crate::testing::block(&bytes(&[0xffffffff, 0xd65f03c0]), GuestAddress(0x1000)).is_err()
    );
}

#[test]
fn block_boundaries_are_bounded_aligned_and_checked() {
    for (input, base) in [
        (vec![], 0),
        (vec![0; 3], 0),
        (bytes(&[0xd503201f]), 1),
        (bytes(&[0xd503201f]), u64::MAX - 3),
        (bytes(&vec![0xd503201f; MAX_BLOCK_INSTRUCTIONS + 1]), 0),
    ] {
        assert!(crate::testing::block(&input, GuestAddress(base)).is_err());
        assert!(source_bytes(&input, GuestAddress(base)).is_err());
    }
    let compiled = crate::testing::block(
        &bytes(&vec![0xd503201f; MAX_BLOCK_INSTRUCTIONS]),
        GuestAddress(0x1000),
    )
    .unwrap();
    assert_eq!(compiled.source_bytes, MAX_BLOCK_INSTRUCTIONS * 4);
    assert_eq!(compiled.native.source_map.len(), MAX_BLOCK_INSTRUCTIONS);
}

#[test]
fn synchronous_trap_immediates_and_preferred_returns_are_exact() {
    for (encoding, kind, return_pc) in [
        (0xd4000001, TrapKind::SupervisorCall, GuestAddress(0x2004)),
        (
            0xd4200000,
            TrapKind::SoftwareBreakpoint,
            GuestAddress(0x2000),
        ),
    ] {
        for immediate in 0..=u16::MAX {
            let word = encoding | (u32::from(immediate) << 5);
            let trap = SynchronousTrap {
                kind,
                immediate,
                instruction_pc: GuestAddress(0x2000),
                exception_return_pc: return_pc,
            };
            assert_eq!(decode_trap(GuestAddress(0x2000), word).unwrap(), Some(trap));
            assert_eq!(
                decode_terminator(GuestAddress(0x2000), word).unwrap(),
                Some(Terminator::Trap(trap))
            );
        }
    }
    for pc in [1, 2, 3, u64::MAX] {
        assert!(decode_trap(GuestAddress(pc), 0xd4000001).is_err());
        assert!(decode_trap(GuestAddress(pc), 0xd4200000).is_err());
    }
    assert!(decode_trap(GuestAddress(u64::MAX - 3), 0xd4000001).is_err());
    assert_eq!(
        decode_trap(GuestAddress(u64::MAX - 3), 0xd4200000).unwrap(),
        Some(SynchronousTrap {
            kind: TrapKind::SoftwareBreakpoint,
            immediate: 0,
            instruction_pc: GuestAddress(u64::MAX - 3),
            exception_return_pc: GuestAddress(u64::MAX - 3),
        })
    );
}

#[test]
fn trap_masks_reject_every_fixed_bit_neighbor_and_other_exception_classes() {
    for encoding in [0xd4000001, 0xd4200000] {
        for bit in 0..32 {
            if (0xffe0001fu32 & (1 << bit)) != 0 {
                let word = encoding ^ (1 << bit);
                assert_eq!(decode_trap(GuestAddress(0x2000), word).unwrap(), None);
            }
        }
    }
    for word in [
        0xd4000002, // HVC
        0xd4000003, // SMC
        0xd4400000, // HLT
        0xd4a00001, // DCPS1
        0xd4a00002, // DCPS2
        0xd4a00003, // DCPS3
        0xd4000000, // reserved exception encoding
        0xd4200001, // reserved BRK neighbor
    ] {
        assert_eq!(decode_trap(GuestAddress(0x2000), word).unwrap(), None);
        assert!(crate::testing::block(&bytes(&[word]), GuestAddress(0x2000)).is_err());
    }
}

#[test]
fn trap_is_a_consumed_boundary_with_its_own_source_map_and_no_leaf_fallback() {
    for word in [0xd4000001, 0xd4200000, 0xd41fffe1, 0xd43fffe0] {
        let source = bytes(&[0xd503201f, word, 0xffffffff, 0xd65f03c0]);
        let compiled = crate::testing::block(&source, GuestAddress(0x2ffc)).unwrap();
        assert_eq!(compiled.source_bytes, 8);
        assert_eq!(source_bytes(&source, GuestAddress(0x2ffc)).unwrap(), 8);
        assert_eq!(
            compiled.native.source_map,
            [(GuestAddress(0x2ffc), 0), (GuestAddress(0x3000), 1)]
        );
        assert_eq!(
            compiled.native.x86_64_code.len(),
            1 + crate::x86_64::chain::DIRECT_EXIT_BYTES
        );
        assert_eq!(compiled.native.x86_64_code[0], 0x90);
        assert!(crate::testing::leaf(&bytes(&[word, 0xd65f03c0]), GuestAddress(0x3000)).is_err());
    }
    // Original block input-window policy still forbids wrapping extents,
    // even though BRK's standalone preferred return PC is representable.
    for word in [0xd4000001, 0xd4200000] {
        assert!(crate::testing::block(&bytes(&[word]), GuestAddress(u64::MAX - 3)).is_err());
    }
}

fn apply_recovery(
    table: &[FaultRecovery],
    offset: usize,
    gprs: &HostGprSnapshot,
    state: &mut crate::GuestState,
) -> Result<Option<RecoverySettlement>, FaultRecoveryError> {
    FaultRecovery::apply(
        table,
        offset,
        HostFaultSnapshot {
            gprs,
            spills: &HostSpillSnapshot::new(),
        },
        state,
    )
}

/// A self-loop's second copy of its body reaches memory through the masked
/// bases its first copy checked, which its back edge carries; its locals
/// stay in their hosts, so the copy touches no frame slot of a local.
#[test]
fn self_loops_check_their_bases_once_per_entry() {
    let check = crate::x86_64::RangeCheck::FIVE_LEVEL.pinned_code();
    let count = |code: &[u8]| {
        code.windows(check.len())
            .filter(|bytes| *bytes == check)
            .count()
    };
    let compile = |self_loops| {
        // memcmp's byte loop: ldrb w8,[x0],#1; ldrb w9,[x1],#1;
        // subs w8,w8,w9; b.ne +3; subs x2,x2,#1; b.ne -5
        let words = [
            0x3840_1408_u32,
            0x3840_1429,
            0x6b09_0108,
            0x5400_0061,
            0xf100_0442,
            0x54ff_ff61,
        ];
        let source: Vec<u8> = words.iter().copied().flat_map(u32::to_le_bytes).collect();
        let options = crate::Options::new(
            BlockProfile::new(Features::SYSTEM_MODE),
            crate::TargetCpu::detect(),
        )
        .with_memory_checks(crate::x86_64::MemoryChecks::system(true, false, true, true))
        .with_native_system(crate::x86_64::NativeSystem {
            self_loops,
            ..crate::x86_64::NativeSystem::NONE
        });
        crate::compile(&source, GuestAddress(0x1000), &options)
            .unwrap()
            .native
    };
    let (linked, looped) = (compile(false), compile(true));
    assert_eq!(count(&linked.x86_64_code), 2);
    // Both copies are there, and only the first checks.
    assert!(looped.x86_64_code.len() > linked.x86_64_code.len() * 3 / 2);
    assert_eq!(count(&looped.x86_64_code), 2);
    assert_eq!(looped.source_map.len(), linked.source_map.len());
}

/// The loop shapes beyond a branch to the block's start: a body with a DC
/// ZVA in it, one left only by a branch it runs past (an unconditional
/// branch back), and bodies with a branch skipping forward inside them, to
/// an instruction of the body or to the back edge.
#[test]
fn self_loops_take_dc_zva_unconditional_back_branches_and_joins() {
    let compile = |words: &[u32], self_loops: bool| {
        let source: Vec<u8> = words.iter().copied().flat_map(u32::to_le_bytes).collect();
        let options = crate::Options::new(
            BlockProfile::new(Features::SYSTEM_MODE),
            crate::TargetCpu::detect(),
        )
        .with_memory_checks(crate::x86_64::MemoryChecks::system(true, false, true, true))
        .with_native_system(crate::x86_64::NativeSystem {
            self_loops,
            zero_block: Some(64),
            ..crate::x86_64::NativeSystem::NONE
        });
        crate::compile(&source, GuestAddress(0x1000), &options).unwrap()
    };
    let bodies: [&[u32]; 4] = [
        // dc zva,x3; add x3,x3,#64; subs x2,x2,#64; b.hi -3
        &[0xd50b_7423, 0x9101_0063, 0xf101_0042, 0x54ff_ffa8],
        // ldrb w8,[x0],#1; cbz w8,+2; b -2
        &[0x3840_1408, 0x3400_0048, 0x17ff_fffe],
        // ldrb w8,[x0],#1; cmp w8,w9; b.ne +2; add x10,x10,#1;
        // subs x2,x2,#1; b.ne -5
        &[
            0x3840_1408,
            0x6b09_011f,
            0x5400_0041,
            0x9100_054a,
            0xf100_0442,
            0x54ff_ff61,
        ],
        // ldr x8,[x0],#8; sub x2,x2,#1; tbz x8,#0,+2; add x10,x10,x8;
        // cbnz x2,-4
        &[
            0xf840_8408,
            0xd100_0442,
            0x3600_0048,
            0x8b08_014a,
            0xb5ff_ff82,
        ],
    ];
    for words in bodies {
        let (linked, looped) = (compile(words, false), compile(words, true));
        assert!(!linked.self_loop, "{words:x?}");
        assert!(looped.self_loop, "{words:x?}");
        // The whole body is one block either way but for DC ZVA, which
        // ends a block without the loops.
        assert_eq!(looped.source_bytes, words.len() * 4);
        let first = if words[0] == 0xd50b_7423 {
            4
        } else {
            words.len() * 4
        };
        assert_eq!(linked.source_bytes, first);
    }
}

#[test]
fn flag_setters_leave_their_capture_to_the_exit_of_the_branch_after_them() {
    let compile = |words: &[u32], self_loops: bool, capture_every_nzcv: bool| {
        let source: Vec<u8> = words.iter().copied().flat_map(u32::to_le_bytes).collect();
        let options = crate::Options::new(
            BlockProfile::new(Features::SYSTEM_MODE),
            crate::TargetCpu::detect(),
        )
        .with_memory_checks(crate::x86_64::MemoryChecks::system(true, false, true, true))
        .with_native_system(crate::x86_64::NativeSystem {
            self_loops,
            capture_every_nzcv,
            ..crate::x86_64::NativeSystem::NONE
        });
        crate::compile(&source, GuestAddress(0x1000), &options).unwrap()
    };
    // The bytes of row `row`, up to the next row.
    let row = |block: &crate::block::CompiledBlock, row: usize| {
        block.native.source_map[row + 1].1 - block.native.source_map[row].1
    };
    // ldrb w8,[x0],#1; ldrb w9,[x1],#1; subs w8,w8,w9; b.ne +3;
    // subs x2,x2,#1; b.ne -5: the next SUBS defines the flags again
    // before anything needs them.
    let memcmp = [
        0x3840_1408,
        0x3840_1429,
        0x6b09_0108,
        0x5400_0061,
        0xf100_0442,
        0x54ff_ff61,
    ];
    // subs x2,x2,#1; b.lo +4; ldrb w8,[x0],#1; cmp w8,w9; b.ne -4: the
    // load's fault needs the SUBS's flags.
    let checked = [
        0xf100_0442,
        0x5400_0083,
        0x3840_1408,
        0x6b09_011f,
        0x54ff_ff81,
    ];
    for self_loops in [false, true] {
        let (deferred, every) = (
            compile(&memcmp, self_loops, false),
            compile(&memcmp, self_loops, true),
        );
        assert_eq!(deferred.self_loop, self_loops);
        // LAHF, SETO and the word's store move to the branch's exit.
        assert!(
            row(&deferred, 2) + 8 <= row(&every, 2),
            "loops {self_loops}: {} and {}",
            row(&deferred, 2),
            row(&every, 2)
        );
        let (deferred, every) = (
            compile(&checked, self_loops, false),
            compile(&checked, self_loops, true),
        );
        assert_eq!(
            deferred.native.x86_64_code, every.native.x86_64_code,
            "loops {self_loops}"
        );
    }
}
