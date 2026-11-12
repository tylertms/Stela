use super::super::chain::{DIRECT_EXIT_BYTES, DYNAMIC_EXIT_BYTES};
use super::*;
use crate::block::{SynchronousTrap, TrapKind};
use alloc::vec;
use alloc::vec::Vec;

#[test]
fn ordinary_scaled_add_uses_a_settled_right_operand() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan},
    };

    // AND X14,X12,X6; ADD X18,X2,X14,LSL #2 from zlib inflate.
    let producer = 0x8a06_018e_u32;
    let add = 0x8b0e_0852_u32;
    let pc = GuestAddress(0x100004108);
    let compile = |words: &[u32], pc, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(BlockProfile::ORDINARY, target))
            .unwrap()
            .compile()
            .unwrap()
            .native
    };
    let target = TargetCpu::Baseline;
    for target in [target, TargetCpu::Sse41, TargetCpu::X86_64_V2.with_bmi2()] {
        for size_bit in [0, 1 << 31] {
            for amount in 1..=3 {
                for destination in [2, 14, 18] {
                    let add = (add & !(1 << 31 | 63 << 10 | 31))
                        | size_bit
                        | (amount << 10)
                        | destination;
                    let grouped = compile(&[producer, add], pc, target);
                    let second = grouped.source_map[1];
                    assert_eq!(
                        grouped.source_map,
                        [(pc, 0), (GuestAddress(pc.0 + 4), second.1)]
                    );
                    assert!(
                        grouped.x86_64_code[..second.1]
                            .ends_with(&[0x48, 0x89, 0x87, 0x70, 0, 0, 0])
                    );
                    let row = &grouped.x86_64_code[second.1..];
                    let lea = [0x8d, 0x04, 1 | ((amount as u8) << 6)];
                    let load_size = if size_bit == 0 { 6 } else { 8 };
                    assert_eq!(&row[load_size..load_size + lea.len()], &lea);
                    assert!(row[load_size + lea.len()..].starts_with(&[
                        0x48,
                        0x89,
                        0x87,
                        (destination * 8) as u8,
                        0,
                        0,
                        0
                    ]));
                }
            }
        }
    }

    // LDRH W9,[X10,X26,LSL #1]; ADD X8,X8,X9,LSL #2 from SQLite.
    let from_load = compile(
        &[0x787a_7949, 0x8b09_0908],
        GuestAddress(0x100026f8c),
        target,
    );
    let uses_held = |row: &[u8]| {
        row.starts_with(&[0x48, 0x8b, 0x8f]) && row[7..].starts_with(&[0x48, 0x8d, 0x04, 0x81])
    };
    assert!(uses_held(
        &from_load.x86_64_code[from_load.source_map[1].1..]
    ));
    let independent = compile(&[add], GuestAddress(pc.0 + 4), target);
    assert!(!uses_held(&independent.x86_64_code));
    // The zlib load-pair compare leaves NZCV in RAX. The following ADD
    // needs the second load's committed X7 value from the guest frame.
    let after_compare = compile(
        &[0x3940_0086, 0x3940_0167, 0x6b07_00df, 0x8b07_0852],
        GuestAddress(0x100003e14),
        target,
    );
    let cmp = after_compare.source_map[2].1;
    let next = after_compare.source_map[3].1;
    assert!(after_compare.x86_64_code[cmp..next].starts_with(&[0x39, 0xc1]));
    assert!(!uses_held(&after_compare.x86_64_code[next..]));
    let fast = TargetCpu::X86_64_V2.with_bmi2().with_fast_pdep();
    let pair = [0x3940_0086, 0x3940_0167, 0x6b07_00df];
    let branch = 0x5400_0dc1;
    let paired_pc = GuestAddress(0x100003e14);
    let separated = compile(
        &[pair[0], pair[1], pair[2], 0x8b07_0852, branch],
        paired_pc,
        fast,
    );
    let code = &separated.x86_64_code;
    let map = &separated.source_map;
    assert!(code[map[2].1..map[3].1].starts_with(&[0x39, 0xc1]));
    assert!(!uses_held(&code[map[3].1..map[4].1]));
    assert_eq!(code[map[4].1], 0xf6); // Test saved NZCV after the intervening ADD.

    let with_producer = compile(
        &[pair[0], pair[1], pair[2], producer, add, branch],
        paired_pc,
        fast,
    );
    let code = &with_producer.x86_64_code;
    let map = &with_producer.source_map;
    assert!(uses_held(&code[map[4].1..map[5].1]));
    assert_eq!(code[map[5].1], 0xf6);
    for fallback in [
        compile(&[producer, add], pc, target.with_data_independent_timing()),
        compile(&[producer, add | (1 << 22)], pc, target), // LSR
        compile(&[producer, add | (1 << 29)], pc, target), // ADDS
        compile(&[producer, (add & !(63 << 10)) | (4 << 10)], pc, target),
        compile(&[producer, (add & !(31 << 5)) | (14 << 5)], pc, target),
    ] {
        assert!(!uses_held(
            &fallback.x86_64_code[fallback.source_map[1].1..]
        ));
    }
}

#[test]
fn ordinary_adjacent_nzcv_uses_native_condition_on_fast_extract_targets() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan},
    };

    // CMP W13,W1; B.HS from the zlib deflate match search.
    let pc = GuestAddress(0x100003fe0);
    let words = [0x6b01_01bfu32, 0x5400_0062];
    let compile = |words: &[u32], pc, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let options = Options::new(BlockProfile::ORDINARY, target);
        CompilationPlan::new(&source, pc, &options)
            .unwrap()
            .compile()
            .unwrap()
            .native
    };
    let extract = |code: &[u8]| {
        code.windows(5)
            .any(|window| window == [0xc4, 0xe2, 0x7a, 0xf5, 0xc1])
    };
    let fast = TargetCpu::X86_64_V2.with_bmi2().with_fast_pdep();
    let pair = compile(&words, pc, fast);
    let branch = pair.source_map[1];
    assert_eq!(
        pair.source_map,
        [(pc, 0), (GuestAddress(pc.0 + 4), branch.1)]
    );
    assert!(extract(&pair.x86_64_code[..branch.1]));
    // Subtraction's CMC gives Arm carry, so HS branches with host JB.
    assert!(pair.x86_64_code[branch.1..].starts_with(&[0x0f, 0x82]));

    let hi = compile(&[words[0], words[1] & !15 | 8], pc, fast);
    // HI needs both !borrow and !zero. Restore x86 borrow for JA.
    assert!(hi.x86_64_code[hi.source_map[1].1..].starts_with(&[0xf5, 0x0f, 0x87]));
    for target in [
        TargetCpu::Baseline,
        TargetCpu::X86_64_V2.with_bmi2(),
        TargetCpu::X86_64_V2.with_fast_pdep(),
        fast.with_data_independent_timing(),
    ] {
        assert!(!extract(&compile(&words, pc, target).x86_64_code));
    }
    assert!(!extract(&compile(&words[..1], pc, fast).x86_64_code));
    let independent = compile(&words[1..], branch.0, fast);
    assert_eq!(independent.source_map, [(branch.0, 0)]);
    assert_eq!(independent.x86_64_code[0], 0xf6); // TEST the saved NZCV byte.
}

#[test]
fn adjacent_vector_pairs_forward_both_sources_and_keep_each_access() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::MemoryChecks,
    };

    // The signed OpenSSL RSA trace reaches LDP Q0,Q1,[X4]; STP Q0,Q1,[X8].
    let words = [0xad40_0480u32, 0xad00_0500];
    let pc = GuestAddress(0xe5c6c);
    let compile = |words: &[u32], pc, profile, checks, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let options = Options::new(profile, target).with_memory_checks(checks);
        CompilationPlan::new(&source, pc, &options)
            .unwrap()
            .compile()
            .unwrap()
    };
    let target = TargetCpu::Baseline;
    let checks = MemoryChecks::DEFAULT;
    let next = GuestAddress(pc.0 + 4);
    let reload = [
        0xf3, 0x0f, 0x6f, 0x87, 0x08, 0x01, 0, 0, 0xf3, 0x0f, 0x6f, 0x8f, 0x18, 0x01, 0, 0,
    ];
    let has_reload = |native: &NativeBlock| {
        native
            .x86_64_code
            .windows(reload.len())
            .any(|bytes| bytes == reload)
    };
    for profile in [
        BlockProfile::ORDINARY,
        BlockProfile::new(Features::SYSTEM_MODE),
    ] {
        let grouped = compile(&words, pc, profile, checks, target);
        let fallback = compile(
            &words,
            pc,
            profile,
            checks,
            target.with_data_independent_timing(),
        );
        assert_eq!(grouped.native.source_map.len(), 2);
        assert_eq!(grouped.native.source_map[1].0, next);
        // A system-mode block keeps both vectors in host registers
        // (`vector_cache`), whose stores before the second access replace
        // the commits.
        if profile == BlockProfile::ORDINARY {
            assert_eq!(grouped.native.source_map, fallback.native.source_map);
            assert_eq!(
                grouped.native.x86_64_code.len() + reload.len(),
                fallback.native.x86_64_code.len()
            );
        }
        assert!(!has_reload(&grouped.native));
        assert!(has_reload(&fallback.native));
        assert!(has_reload(
            &compile(&words[1..], next, profile, checks, target).native
        ));
        if profile == BlockProfile::ORDINARY {
            continue;
        }
        let accesses: Vec<_> = grouped
            .fault_recovery
            .iter()
            .filter(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
            .collect();
        assert_eq!(accesses.len(), 4);
        for (i, row) in accesses.into_iter().enumerate() {
            let guest = if i < 2 { pc } else { next };
            assert_eq!(row.retirement.instruction_pc(), guest);
            assert_eq!(row.retirement.retired_before() as usize, i / 2);
            let code = &grouped.native.x86_64_code[row.start..row.end];
            assert_eq!(&code[..3], &[0xf3, 0x0f, if i < 2 { 0x6f } else { 0x7f }]);
            assert_eq!(code.len(), 4 + i % 2);
        }
        assert!(has_reload(
            &compile(&words, pc, profile, checks.with_translation(0x1000), target).native
        ));
    }
    let ordinary = BlockProfile::ORDINARY;
    let different_pair = [words[0] ^ (2 << 10), words[1]];
    let separated = [words[0], 0x4f00_e402, words[1]]; // MOVI V2.16B,#0 uses XMM0.
    for fallback in [&different_pair[..], &separated[..]] {
        assert!(has_reload(
            &compile(fallback, pc, ordinary, checks, target).native
        ));
    }
}

#[test]
fn vector_copy_keeps_pair_across_scalar_rows() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::MemoryChecks,
    };

    // Paired Q load, counter SUBS, source ADD, paired Q store,
    // destination ADD, and conditional backedge from real copy loops.
    let windows = [
        (
            0x10000b620,
            [
                0xad7f_8520u32,
                0xf100_818c,
                0x9100_8129,
                0xad3f_8540,
                0x9100_814a,
                0x54ff_ff61,
            ],
        ),
        (
            0x10000003c,
            [
                0xad7f_8540,
                0xf100_816b,
                0x9100_814a,
                0xad3f_8520,
                0x9100_8129,
                0x54ff_ff61,
            ],
        ),
        (
            0x100008150,
            [
                0xad7f_8821,
                0xf100_12d6,
                0x9100_8021,
                0xad3f_8801,
                0x9100_8000,
                0x54ff_ff61,
            ],
        ),
        (
            0x100008dd0,
            [
                0xad7f_8500,
                0xf100_1318,
                0x9100_8108,
                0xad3f_8520,
                0x9100_8129,
                0x54ff_ff61,
            ],
        ),
        (
            0x100008f60,
            [
                0xad7f_8440,
                0xf100_12f7,
                0x9100_8042,
                0xad3f_8660,
                0x9100_8273,
                0x54ff_ff61,
            ],
        ),
    ];
    let compile = |words: &[u32], pc, profile, target, checks| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(
            &source,
            pc,
            &Options::new(profile, target).with_memory_checks(checks),
        )
        .unwrap()
        .compile()
        .unwrap()
    };
    let reloads = |native: &NativeBlock, row: usize| {
        native.x86_64_code[native.source_map[row].1..native.source_map[row + 1].1]
            .windows(4)
            .filter(|bytes| matches!(*bytes, [0xf3, 0x0f, 0x6f, 0x87 | 0x8f]))
            .count()
    };
    let target = TargetCpu::Baseline;
    let checks = MemoryChecks::DEFAULT;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    for (address, words) in windows {
        let pc = GuestAddress(address);
        let store = GuestAddress(address + 12);
        for profile in [BlockProfile::ORDINARY, system] {
            let grouped = compile(&words, pc, profile, target, checks);
            assert_eq!(grouped.native.source_map.len(), words.len());
            for (index, (guest, _)) in grouped.native.source_map.iter().enumerate() {
                assert_eq!(*guest, GuestAddress(address + 4 * index as u64));
            }
            assert_eq!(reloads(&grouped.native, 3), 0);
            let independent = compile(&words[3..], store, profile, target, checks);
            assert_eq!(reloads(&independent.native, 0), 2);
            let dit = compile(
                &words,
                pc,
                profile,
                target.with_data_independent_timing(),
                checks,
            );
            assert_eq!(reloads(&dit.native, 3), 2);
            if profile != system {
                let advance = grouped.native.source_map[4].1;
                assert!(
                    grouped.native.x86_64_code[advance..].starts_with(&[0x48, 0x8d, 0x40, 0x30])
                );
                continue;
            }
            let accesses: Vec<_> = grouped
                .fault_recovery
                .iter()
                .filter(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
                .collect();
            assert_eq!(accesses.len(), 4);
            for (index, row) in accesses.into_iter().enumerate() {
                let (expected_pc, retired, opcode) = if index < 2 {
                    (pc, 0, 0x6f)
                } else {
                    (store, 3, 0x7f)
                };
                assert_eq!(row.retirement.instruction_pc(), expected_pc);
                assert_eq!(row.retirement.retired_before(), retired);
                assert_eq!(
                    &grouped.native.x86_64_code[row.start..row.start + 3],
                    &[0xf3, 0x0f, opcode]
                );
            }
            let translated = compile(&words, pc, profile, target, checks.with_translation(0x1000));
            assert_eq!(reloads(&translated.native, 3), 2);
        }
    }
}

#[test]
fn vector_pair_store_forwards_its_offset_address_to_the_next_base_add() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::MemoryChecks,
    };

    // STP Q0,Q1,[X10,#-16]; ADD X10,X10,#32 from zlib.
    let pc = GuestAddress(0x10000b62c);
    let words = [0xad3f_8540u32, 0x9100_814a];
    let target = TargetCpu::Baseline;
    let ordinary = BlockProfile::ORDINARY;
    let checks = MemoryChecks::DEFAULT;
    let compile = |words: &[u32], pc, profile, target, checks| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(
            &source,
            pc,
            &Options::new(profile, target).with_memory_checks(checks),
        )
        .unwrap()
        .compile()
        .unwrap()
    };
    let grouped = compile(&words, pc, ordinary, target, checks);
    let independent = compile(
        &words[1..],
        GuestAddress(pc.0 + 4),
        ordinary,
        target,
        checks,
    );
    let fallback = compile(
        &words,
        pc,
        ordinary,
        target.with_data_independent_timing(),
        checks,
    );
    let row = grouped.native.source_map[1].1;
    assert_eq!(
        grouped.native.source_map,
        [(pc, 0), (GuestAddress(pc.0 + 4), row)]
    );
    assert_eq!(grouped.native.source_map, fallback.native.source_map);
    assert_eq!(
        &grouped.native.x86_64_code[..row],
        &fallback.native.x86_64_code[..row]
    );
    let frame_load = [0x48, 0x8b, 0x87, 0x50, 0, 0, 0];
    let commit = [0x48, 0x89, 0x87, 0x50, 0, 0, 0];
    let forwarded = [0x48, 0x8d, 0x40, 0x30];
    assert!(grouped.native.x86_64_code[row..].starts_with(&forwarded));
    assert!(fallback.native.x86_64_code[row..].starts_with(&frame_load));
    assert!(independent.native.x86_64_code.starts_with(&frame_load));
    assert_eq!(
        grouped.native.x86_64_code.len() + frame_load.len(),
        fallback.native.x86_64_code.len()
    );

    for (offset, immediate, shifted) in [
        (-1024i32, 4095u32, true),
        (1008, 0, false),
        (32, 32, false),
        (0, 32, false),
    ] {
        let store = (words[0] & !(0x7f << 15)) | (((offset / 16) as u32 & 0x7f) << 15);
        let add = (words[1] & !(0x1fff << 10)) | (immediate << 10) | (u32::from(shifted) << 22);
        let block = compile(&[store, add], pc, ordinary, target, checks);
        let mut expected = Code::new();
        let adjustment = ((immediate << if shifted { 12 } else { 0 }) as i32) - offset;
        super::super::address_add(&mut expected, adjustment, super::super::AddressMode::Offset);
        assert!(
            block.native.x86_64_code[block.native.source_map[1].1..]
                .starts_with(&[&expected[..], &commit].concat())
        );
    }

    // Aliased numeric Q/X register indices are separate architectural banks.
    let same_number = [words[0] | 10, words[1]];
    let aliased = compile(&same_number, pc, ordinary, target, checks);
    assert!(aliased.native.x86_64_code[aliased.native.source_map[1].1..].starts_with(&forwarded));
    let gap = [words[0], 0xaa0b_03eau32, words[1]]; // MOV X10,X11 supersedes the store base.
    let gap = compile(&gap, pc, ordinary, target, checks);
    assert!(!gap.native.x86_64_code[gap.native.source_map[2].1..].starts_with(&forwarded));
    for (store, add) in [
        (words[0], words[1] ^ 1),                          // Different destination.
        (words[0], words[1] ^ (1 << 5)),                   // Different input.
        (words[0], words[1] ^ (1 << 31)),                  // W32.
        (words[0], words[1] | (1 << 29)),                  // ADDS.
        (words[0] ^ (1 << 23), words[1]),                  // Writeback.
        (words[0] | (31 << 5), words[1] | (31 << 5) | 31), // SP.
    ] {
        let block = compile(&[store, add], pc, ordinary, target, checks);
        assert!(!block.native.x86_64_code[block.native.source_map[1].1..].starts_with(&forwarded));
    }
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    for checks in [checks, checks.with_translation(0x1000)] {
        let block = compile(&words, pc, system, target, checks);
        assert!(!block.native.x86_64_code[block.native.source_map[1].1..].starts_with(&forwarded));
        let stores: Vec<_> = block
            .fault_recovery
            .iter()
            .filter(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
            .collect();
        assert_eq!(stores.len(), 2);
        for row in stores {
            assert_eq!(row.retirement.instruction_pc(), pc);
            assert_eq!(row.retirement.retired_before(), 0);
            assert_eq!(
                &block.native.x86_64_code[row.start..row.start + 3],
                &[0xf3, 0x0f, 0x7f]
            );
        }
    }
}

#[test]
fn ordinary_compare_reads_both_settled_load_results() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan},
    };

    // LDRB W6,[X4]; LDRB W7,[X11]; CMP W6,W7; B.NE from zlib.
    let words = [0x3940_0086u32, 0x3940_0167, 0x6b07_00df, 0x5400_0dc1];
    let pc = GuestAddress(0x100003e14);
    let compile = |words: &[u32], pc, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(BlockProfile::ORDINARY, target))
            .unwrap()
            .compile()
            .unwrap()
            .native
    };
    let baseline = TargetCpu::Baseline;
    let fast = TargetCpu::X86_64_V2.with_bmi2().with_fast_pdep();
    let old = compile(&words, pc, baseline);
    let live = compile(&words, pc, fast);
    assert_eq!(&old.source_map[..3], &live.source_map[..3]);
    let compare_at = live.source_map[2].1;
    let branch_at = live.source_map[3].1;
    assert_eq!(
        &old.x86_64_code[..compare_at],
        &live.x86_64_code[..compare_at]
    );
    assert!(live.x86_64_code[compare_at..branch_at].ends_with(&[0x89, 0x87, 0, 1, 0, 0]));
    assert!(live.x86_64_code[branch_at..].starts_with(&[0x0f, 0x85]));
    let extract = |code: &[u8]| {
        code.windows(5)
            .any(|window| window == [0xc4, 0xe2, 0x7a, 0xf5, 0xc1])
    };
    assert!(extract(&live.x86_64_code[compare_at..branch_at]));
    for target in [
        baseline,
        TargetCpu::X86_64_V2.with_bmi2(),
        TargetCpu::X86_64_V2.with_fast_pdep(),
        fast.with_data_independent_timing(),
    ] {
        assert!(!extract(&compile(&words, pc, target).x86_64_code));
    }
    assert!(!extract(&compile(&words[..3], pc, fast).x86_64_code));
    let branch = compile(&words[3..], live.source_map[3].0, fast);
    assert_eq!(branch.x86_64_code[0], 0xf6);
    for target in [baseline, fast] {
        let grouped = compile(&words, pc, target);
        let map = &grouped.source_map;
        assert_eq!(
            map.iter().map(|row| row.0).collect::<Vec<_>>(),
            [
                pc,
                GuestAddress(pc.0 + 4),
                GuestAddress(pc.0 + 8),
                GuestAddress(pc.0 + 12),
            ]
        );
        let code = &grouped.x86_64_code;
        assert!(code[..map[1].1].ends_with(&[0x48, 0x89, 0x8f, 0x30, 0, 0, 0]));
        assert!(code[map[1].1..map[2].1].ends_with(&[0x48, 0x89, 0x87, 0x38, 0, 0, 0]));
        assert!(code[map[2].1..].starts_with(&[0x39, 0xc1]));
        let independent = compile(&words[1..], map[1].0, target);
        assert!(!independent.x86_64_code[independent.source_map[1].1..].starts_with(&[0x39, 0xc1]));
    }

    for fallback in [
        compile(&words, pc, baseline.with_data_independent_timing()),
        compile(&[words[0], 0x3940_0087, words[2], words[3]], pc, baseline), // Shared base.
        compile(&[words[0], 0x3940_00c7, words[2], words[3]], pc, baseline), // Dependent base.
        compile(
            &[words[0], words[1], words[2] | (1 << 10), words[3]],
            pc,
            baseline,
        ),
    ] {
        assert!(!fallback.x86_64_code[fallback.source_map[2].1..].starts_with(&[0x39, 0xc1]));
    }
}

#[test]
fn ordinary_pair_compare_reuses_both_committed_load_results() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };

    // LDP X8,X10,[X1,#8]; CMP X10,X8; B.EQ from yyjson.
    let words = [0xa940_a828, 0xeb08_015f, 0x5400_1b60];
    let pc = GuestAddress(0x1000001ec);
    let ordinary = BlockProfile::ORDINARY;
    let baseline = TargetCpu::Baseline;
    let fast = TargetCpu::X86_64_V2.with_bmi2().with_fast_pdep();
    let compile = |words: &[u32], pc, profile, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let staged = [0x49, 0x39, 0xc8]; // CMP R8,RCX.
    for target in [baseline, fast] {
        let grouped = compile(&words, pc, ordinary, target).native;
        let map = &grouped.source_map;
        assert_eq!(
            map.iter().map(|row| row.0).collect::<Vec<_>>(),
            [pc, GuestAddress(pc.0 + 4), GuestAddress(pc.0 + 8)]
        );
        let (compare, branch) = (map[1].1, map[2].1);
        let code = &grouped.x86_64_code;
        let pair = compile(&words[..1], pc, ordinary, target).native;
        assert_eq!(&code[..compare], &pair.x86_64_code[..compare]);
        assert!(code[..compare].ends_with(&[0x4c, 0x89, 0x87, 0x50, 0, 0, 0])); // Commit X10.
        assert!(code[compare..branch].starts_with(&staged));
        if target == fast {
            assert!(code[branch..].starts_with(&[0x0f, 0x84])); // JE.
        }
        let independent = compile(&words[1..], map[1].0, ordinary, target).native;
        assert_eq!(independent.source_map[0], (map[1].0, 0));
        assert!(independent.x86_64_code.starts_with(&[
            0x48, 0x8b, 0x87, 0x50, 0, 0, 0, // MOV RAX,[frame+X10].
            0x48, 0x3b, 0x87, 0x40, 0, 0, 0, // CMP RAX,[frame+X8].
        ]));
        assert_eq!(independent.source_map[1].1, branch - compare + 11);
        let branch_entry = compile(&words[2..], map[2].0, ordinary, target).native;
        assert_eq!(branch_entry.source_map[0], (map[2].0, 0));
        assert_eq!(branch_entry.x86_64_code[0], 0xf6); // Frame NZCV.
    }

    // W8,W10 and reversed operand order each use the appropriate CMP.
    for (pair, compare, expected) in [
        (0x2940_a828, 0x6b08_015f, [0x41, 0x39, 0xc8]),
        (words[0], 0xeb0a_011f, [0x4c, 0x39, 0xc1]),
        ((words[0] & !(31 << 5)) | (10 << 5), words[1], staged), // Base aliases X10.
    ] {
        let block = compile(&[pair, compare, words[2]], pc, ordinary, baseline).native;
        assert!(block.x86_64_code[block.source_map[1].1..].starts_with(&expected));
    }

    for (pair, compare, target) in [
        (words[0], words[1], fast.with_data_independent_timing()),
        (words[0], words[1] | (1 << 10), baseline), // CMP X10,X8,LSL #1.
        (words[0], words[1] & !(1 << 31), baseline), // CMP W10,W8.
        (words[0] ^ (3 << 23), words[1], baseline), // Post-index LDP.
    ] {
        let fallback = compile(&[pair, compare, words[2]], pc, ordinary, target).native;
        assert!(!fallback.x86_64_code[fallback.source_map[1].1..].starts_with(&staged));
    }

    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let checked = compile(&words, pc, system, fast);
    let compare = checked.native.source_map[1].1;
    assert!(!checked.native.x86_64_code[compare..].starts_with(&staged));
    let accesses: Vec<_> = checked
        .fault_recovery
        .iter()
        .filter(|site| site.site.kind() == RecoverySiteKind::GuestAccess)
        .collect();
    // The X pair's 16-byte access where its address is 16-byte aligned
    // (FEAT_LSE2) and its two element accesses otherwise.
    assert_eq!(accesses.len(), 3);
    for access in accesses {
        assert_eq!(access.retirement.instruction_pc(), pc);
        assert_eq!(access.retirement.retired_before(), 0);
        assert!(access.start < access.end && access.end <= compare);
    }
}

#[test]
fn extended_compare_reads_its_left_operand_in_place() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };

    let pc = GuestAddress(0xffff_fe00_3a41_bdc8);
    let compare = 0xeb2a_013fu32; // CMP X9,W10,UXTB
    let branch = 0x5400_0601; // B.NE
    let target = TargetCpu::Baseline;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let compile = |words: &[u32], pc, profile, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let suffix = |native: &NativeBlock| native.source_map[1].1;
    let has = |bytes: &[u8], needle: &[u8]| bytes.windows(needle.len()).any(|row| row == needle);
    for profile in [BlockProfile::ORDINARY, system] {
        let (left_load, direct_cmp, staged_right, direct_wcmp) = if profile == system {
            (
                &[0x48, 0x8b, 0x47, 0x48][..],
                &[0x48, 0x39, 0x4f, 0x48][..],
                &[0x0f, 0xb6, 0x4f, 0x50][..],
                &[0x39, 0x4f, 0x50][..],
            )
        } else {
            (
                &[0x48, 0x8b, 0x87, 0x48, 0, 0, 0][..],
                &[0x48, 0x39, 0x8f, 0x48, 0, 0, 0][..],
                &[0x0f, 0xb6, 0x8f, 0x50, 0, 0, 0][..],
                &[0x39, 0x8f, 0x50, 0, 0, 0][..],
            )
        };
        let block = compile(&[compare, branch], pc, profile, target);
        let branch_pc = GuestAddress(pc.0 + 4);
        assert_eq!(
            block.native.source_map,
            [(pc, 0), (branch_pc, suffix(&block.native))]
        );
        assert!(
            block
                .native
                .x86_64_code
                .starts_with(&[staged_right, direct_cmp].concat())
        );
        let fallback = compile(
            &[compare, branch],
            pc,
            profile,
            target.with_data_independent_timing(),
        );
        assert!(fallback.native.x86_64_code.starts_with(left_load));
        let independent = compile(&[branch], branch_pc, profile, target);
        assert_eq!(independent.native.source_map, [(branch_pc, 0)]);
        assert!(!has(&independent.native.x86_64_code, direct_cmp));

        // Keep the cheaper RAX-only path when the left input is already held.
        let held = compile(&[0xf940_0049, compare, branch], pc, profile, target);
        let compare_row =
            &held.native.x86_64_code[held.native.source_map[1].1..held.native.source_map[2].1];
        assert!(compare_row.starts_with(staged_right));
        assert!(!has(compare_row, left_load) && !has(compare_row, direct_cmp));

        let wcmp = compile(&[0x6b2c_015f, 0x5400_0420], pc, profile, target);
        assert!(has(
            &wcmp.native.x86_64_code[..suffix(&wcmp.native)],
            direct_wcmp
        ));
    }

    // X8 and SP use their pinned registers, and XZR supplies a zero RHS.
    let pin_left = (compare & !(31 << 5)) | (8 << 5);
    let sp = compare | (31 << 5);
    let zero = compare | (31 << 16);
    for (word, expected) in [
        (pin_left, &[0x48, 0x39, 0xce][..]),
        (sp, &[0x48, 0x39, 0xcb]),
        (zero, &[0x31, 0xc9, 0x48, 0x39, 0x4f]),
    ] {
        let block = compile(&[word, branch], pc, system, target);
        assert!(has(
            &block.native.x86_64_code[..suffix(&block.native)],
            expected
        ));
    }

    let fault = compile(&[compare, 0xf940_004b, branch], pc, system, target);
    let access = fault
        .fault_recovery
        .iter()
        .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
        .unwrap();
    assert_eq!(access.retirement.instruction_pc(), GuestAddress(pc.0 + 4));
    assert_eq!(access.retirement.retired_before(), 1);
    assert!(access.start >= fault.native.source_map[1].1);
}

#[test]
fn pinned_indexed_address_uses_the_base_without_copying_it() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::MemoryChecks,
    };

    let pc = GuestAddress(0xffff_fe00_37ec_f448);
    let pointer = 0xf940_5808u32; // LDR X8,[X0,#0xb0]
    let load = 0xb861_4900; // LDR W0,[X8,W1,UXTW]
    let store = 0xb821_4902; // STR W2,[X8,W1,UXTW]
    let profile = BlockProfile::new(Features::SYSTEM_MODE);
    let direct = MemoryChecks::DEFAULT;
    let compile = |words: &[u32], pc, checks| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let options = Options::new(profile, TargetCpu::Baseline).with_memory_checks(checks);
        CompilationPlan::new(&source, pc, &options)
            .unwrap()
            .compile()
            .unwrap()
    };

    let uxtw = [0x8b, 0xcd, 0x48, 0x8d, 0x04, 0x0e]; // MOV ECX,EBP; LEA RAX,[RSI+RCX]
    for word in [load, store] {
        let block = compile(&[pointer, word], pc, direct);
        let (indexed_pc, start) = block.native.source_map[1];
        assert_eq!(indexed_pc, GuestAddress(pc.0 + 4));
        assert!(block.native.x86_64_code[start..].starts_with(&uxtw));
        let standalone = compile(&[word], indexed_pc, direct);
        assert!(standalone.native.x86_64_code.starts_with(&uxtw));
        let accesses: Vec<_> = block
            .fault_recovery
            .iter()
            .filter(|entry| entry.site.kind() == RecoverySiteKind::GuestAccess)
            .collect();
        assert_eq!(accesses.len(), 2);
        assert!(accesses[0].end <= start && accesses[1].start >= start + uxtw.len());
        assert_eq!(accesses[0].retirement.instruction_pc(), pc);
        assert_eq!(accesses[0].retirement.retired_before(), 0);
        assert_eq!(accesses[1].retirement.instruction_pc(), indexed_pc);
        assert_eq!(accesses[1].retirement.retired_before(), 1);
    }
    // A signed W index, the pinned base as its own W index, and a pinned X
    // index. X16 lives in R12, whose SIB index needs the REX.X extension.
    for (word, expected) in [
        (
            load | (1 << 15),
            &[0x48, 0x63, 0xcd, 0x48, 0x8d, 0x04, 0x0e][..],
        ),
        (
            (load & !(31 << 16)) | (8 << 16),
            &[0x8b, 0xce, 0x48, 0x8d, 0x04, 0x0e],
        ),
        (
            (load & !(31 << 16)) | (16 << 16) | (1 << 13),
            &[0x4a, 0x8d, 0x04, 0x26],
        ),
    ] {
        assert!(
            compile(&[word], pc, direct)
                .native
                .x86_64_code
                .starts_with(expected)
        );
    }
    for checks in [
        direct.with_translation(0x1000),
        MemoryChecks::system(true, true, true, false),
        MemoryChecks::system(true, false, true, true),
    ] {
        assert!(
            !compile(&[load], pc, checks)
                .native
                .x86_64_code
                .starts_with(&uxtw)
        );
    }
}

#[test]
fn inplace_aes_round_pair_keeps_source_rows_and_cpu_fallbacks() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::isa::SimdOpcode,
    };
    let pc = GuestAddress(0xb6210);
    let ordinary = BlockProfile::ORDINARY;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let targets = [TargetCpu::Aes, TargetCpu::Sse41.with_aes()];
    let compile = |words: &[u32], pc, profile, target| {
        let bytes: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&bytes, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let count = |block: &NativeBlock, opcode: SimdOpcode| {
        block
            .x86_64_code
            .windows(opcode.encoding().len())
            .filter(|bytes| *bytes == opcode.encoding())
            .count()
    };
    for (words, round, last) in [
        (
            [0x4e28_4a00u32, 0x4e28_6800], // OpenSSL AESE V0,V16; AESMC V0,V0.
            SimdOpcode::AesEncryptRound,
            SimdOpcode::AesEncryptLast,
        ),
        (
            [0x4e28_5bffu32, 0x4e28_7bff], // AESD V31,V31; AESIMC V31,V31.
            SimdOpcode::AesDecryptRound,
            SimdOpcode::AesDecryptLast,
        ),
    ] {
        for profile in [ordinary, system] {
            for target in targets {
                let grouped = compile(&words, pc, profile, target);
                let split = compile(&words, pc, profile, target.with_data_independent_timing());
                assert!(grouped.native.x86_64_code.len() < split.native.x86_64_code.len());
                assert_eq!(count(&grouped.native, round), 1);
                assert_eq!(count(&grouped.native, last), 0);
                assert_eq!(count(&split.native, last), 1);
                assert_eq!(grouped.native.source_map.len(), 2);
                assert_eq!(grouped.native.source_map[0], (pc, 0));
                let second = grouped.native.source_map[1];
                assert_eq!(second.0, GuestAddress(pc.0 + 4));
                assert!(second.1 > 0);
                assert!(grouped.fault_recovery.is_empty());
                if profile == system {
                    let followed = [words[0], words[1], 0xf940_0289]; // LDR X9,[X20].
                    let compiled = compile(&followed, pc, profile, target);
                    let fault = compiled
                        .fault_recovery
                        .iter()
                        .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
                        .unwrap();
                    assert_eq!(fault.retirement.instruction_pc(), GuestAddress(pc.0 + 8));
                    assert_eq!(fault.retirement.retired_before(), 2);
                    assert!(fault.start >= compiled.native.source_map[2].1);
                    assert!(fault.bindings.is_empty());
                }
                let clipped = compile(&words[..1], pc, profile, target);
                assert_eq!(count(&clipped.native, last), 1);
                let mix = compile(&words[1..], second.0, profile, target);
                assert_eq!(mix.native.source_map, [(second.0, 0)]);
                let other_destination = [words[0], words[1] ^ 1];
                let split = compile(&other_destination, pc, profile, target);
                assert_eq!(count(&split.native, last), 1);
            }
            for target in [TargetCpu::Baseline, TargetCpu::Ssse3, TargetCpu::Sse41] {
                let fallback = compile(&words, pc, profile, target);
                assert_eq!(count(&fallback.native, round), 0);
                assert_eq!(count(&fallback.native, last), 0);
            }
        }
    }
}

#[test]
fn settled_x_index_is_reused_by_single_and_shared_accesses() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::MemoryChecks,
    };

    // LZ4 AND X23,X7,#0x3ffc; LDR W7,[X8,X23]; STR W17,[X8,X23].
    let words = [0x927e_2cf7u32, 0xb877_6907, 0xb837_6911];
    let pc = GuestAddress(0x100008010);
    let target = TargetCpu::Baseline;
    let direct = MemoryChecks::DEFAULT;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let compile = |words: &[u32], pc, profile, target, checks| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(
            &source,
            pc,
            &Options::new(profile, target).with_memory_checks(checks),
        )
        .unwrap()
        .compile()
        .unwrap()
    };
    let frame_add = [0x48, 0x03, 0x87, 0x40, 0, 0, 0];
    let pinned_add = [0x48, 0x03, 0xc6];
    let committed = [0x48, 0x89, 0x87, 0xb8, 0, 0, 0];
    for profile in [BlockProfile::ORDINARY, system] {
        let add = if profile == system {
            &pinned_add[..]
        } else {
            &frame_add[..]
        };
        for source in [&words[..2], &words[..]] {
            let block = compile(source, pc, profile, target, direct);
            let (load_pc, start) = block.native.source_map[1];
            assert_eq!(load_pc, GuestAddress(pc.0 + 4));
            assert_eq!(block.native.source_map.len(), source.len());
            assert!(block.native.x86_64_code[..start].ends_with(&committed));
            assert!(block.native.x86_64_code[start..].starts_with(add));
            let separate = compile(&source[1..], load_pc, profile, target, direct);
            assert!(!separate.native.x86_64_code.starts_with(add));
            if profile != system {
                continue;
            }
            let accesses: Vec<_> = block
                .fault_recovery
                .iter()
                .filter(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
                .collect();
            assert_eq!(accesses.len(), source.len() - 1);
            for (index, site) in accesses.into_iter().enumerate() {
                assert_eq!(
                    site.retirement.instruction_pc(),
                    GuestAddress(pc.0 + 4 * (index as u64 + 1))
                );
                assert_eq!(site.retirement.retired_before(), index as u16 + 1);
                assert!(site.start >= block.native.source_map[index + 1].1);
            }
        }
    }

    // An independent store can use the same fact, and an SXTX index at
    // scale one is the same full-width addition as LSL #0.
    let only_store = [words[0], words[2] | (1 << 15)];
    let store = compile(&only_store, pc, system, target, direct);
    assert!(store.native.x86_64_code[store.native.source_map[1].1..].starts_with(&pinned_add));

    for (words, target, checks) in [
        ([words[0], words[1] | (1 << 12)], target, direct), // Scaled index.
        (
            [words[0], (words[1] & !(31 << 5)) | (23 << 5)],
            target,
            direct,
        ), // Base aliases index.
        (
            [words[0], words[1]],
            target.with_data_independent_timing(),
            direct,
        ),
        (
            [words[0], words[1]],
            target,
            direct.with_translation(0x1000),
        ),
        (
            [words[0], words[1]],
            target,
            MemoryChecks::system(true, true, true, false),
        ),
    ] {
        let block = compile(&words, pc, system, target, checks);
        assert!(!block.native.x86_64_code[block.native.source_map[1].1..].starts_with(&pinned_add));
    }

    // Desktop UBFIZ X9,X10,#3,#1; LDR X9,[X12,X9] uses two frame
    // operands. The first row must settle X9 even though the load writes it.
    let desktop = [0xd37d_0149, 0xf869_6989];
    let block = compile(&desktop, GuestAddress(0x18f5_a17ec), system, target, direct);
    let row = block.native.source_map[1].1;
    assert!(block.native.x86_64_code[..row].ends_with(&[0x48, 0x89, 0x47, 0x48]));
    assert!(block.native.x86_64_code[row..].starts_with(&[0x48, 0x03, 0x47, 0x60]));
}

#[test]
fn post_index_loads_feed_subtraction_after_settling_both_accesses() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };
    let compile = |words: &[u32], pc, profile, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let ordinary = BlockProfile::ORDINARY;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let target = TargetCpu::Baseline;
    let dit = target.with_data_independent_timing();
    let sqlite = [0x3840_1408, 0x3840_1429, 0x6b09_0108, 0x5400_0081];
    let lz4 = [0xf840_8627, 0xf940_0314, 0xeb14_00ff, 0x5400_06c1];
    for (pc, words, subtract) in [
        // LDRB W8,[X0],#1; LDRB W9,[X1],#1; SUBS W8,W8,W9; B.NE.
        (GuestAddress(0x1000002cc), sqlite, [0x44, 0x29, 0xc1]),
        // LDR X7,[X17],#8; LDR X20,[X24]; CMP X7,X20; B.NE.
        (GuestAddress(0x1000081cc), lz4, [0x48, 0x39, 0xc1]),
    ] {
        let grouped = compile(&words, pc, ordinary, target);
        let fallback = compile(&words, pc, ordinary, dit);
        let code = &grouped.native.x86_64_code;
        let map = &grouped.native.source_map;
        assert_eq!(map.len(), words.len());
        assert!(
            map.windows(2)
                .all(|rows| rows[1].0.0 == rows[0].0.0 + 4 && rows[0].1 < rows[1].1)
        );
        assert!(code[map[2].1..].starts_with(&subtract));
        assert!(
            !fallback.native.x86_64_code[fallback.native.source_map[2].1..].starts_with(&subtract)
        );
        for row in 1..words.len() {
            let entry = compile(&words[row..], map[row].0, ordinary, target);
            assert_eq!(entry.native.source_map[0], (map[row].0, 0));
            assert!(!entry.native.x86_64_code.starts_with(&subtract));
        }
        // Checked system code retains both original access/retirement sites.
        let checked = compile(&words, pc, system, target);
        let checked_map = &checked.native.source_map;
        assert!(!checked.native.x86_64_code[checked_map[2].1..].starts_with(&subtract));
        let accesses: Vec<_> = checked
            .fault_recovery
            .iter()
            .filter(|site| site.site.kind() == RecoverySiteKind::GuestAccess)
            .collect();
        assert_eq!(accesses.len(), 2);
        for (row, site) in accesses.into_iter().enumerate() {
            assert_eq!(site.retirement.instruction_pc(), checked_map[row].0);
            assert_eq!(site.retirement.retired_before(), row as u16);
            assert!(site.start >= checked_map[row].1 && site.end <= checked_map[row + 1].1);
        }
    }

    // The first writeback may feed the second address. Leave that case alone.
    let shared_base = [sqlite[0], sqlite[1] & !(31 << 5), sqlite[2], sqlite[3]];
    let alias = compile(&shared_base, GuestAddress(0x1000002cc), ordinary, target);
    assert!(
        !alias.native.x86_64_code[alias.native.source_map[2].1..].starts_with(&[0x44, 0x29, 0xc1])
    );

    // LDR X7,[X17],#8; LDR X20,[X7],#8. The second writeback replaces
    // the first load's X7, so the compare must read the settled X7.
    let writeback_alias = [lz4[0], 0xf840_84f4, lz4[2], lz4[3]];
    let pc = GuestAddress(0x1000081cc);
    let alias = compile(&writeback_alias, pc, ordinary, target);
    let ungrouped = compile(&writeback_alias, pc, ordinary, dit);
    assert_eq!(alias.native.source_map, ungrouped.native.source_map);
    assert_eq!(alias.native.x86_64_code, ungrouped.native.x86_64_code);
}

#[test]
fn indexed_load_store_pair_shares_checked_address_with_distinct_fault_rows() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
        x86_64::MemoryChecks,
    };
    let words = [0xb877_6907u32, 0xb837_6911]; // LDR W7,[X8,X23]; STR W17,[X8,X23]
    let pc = GuestAddress(0x100008014);
    let compile = |words: &[u32], pc, profile, target, checks| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let options = Options::new(profile, target).with_memory_checks(checks);
        CompilationPlan::new(&source, pc, &options)
            .unwrap()
            .compile()
            .unwrap()
    };
    let target = TargetCpu::Baseline;
    let direct = MemoryChecks::DEFAULT;
    let ordinary = BlockProfile::ORDINARY;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let dit = target.with_data_independent_timing();
    for profile in [ordinary, system] {
        let grouped = compile(&words, pc, profile, target, direct);
        let old = compile(&words, pc, profile, dit, direct);
        assert!(grouped.native.x86_64_code.len() < old.native.x86_64_code.len());
        let second = grouped.native.source_map[1];
        assert_eq!(second.0, GuestAddress(pc.0 + 4));
        let standalone = compile(&words[1..], second.0, profile, target, direct);
        assert_eq!(standalone.native.source_map, [(second.0, 0)]);
        assert!(
            !grouped.native.x86_64_code[second.1..]
                .starts_with(&standalone.native.x86_64_code[..3])
        );
        if profile == system {
            let rows = &grouped.fault_recovery;
            let accesses: Vec<_> = rows
                .iter()
                .filter(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
                .collect();
            assert_eq!(accesses.len(), 2);
            assert_eq!(accesses[0].retirement.instruction_pc(), pc);
            assert_eq!(accesses[0].retirement.retired_before(), 0);
            assert!(accesses[0].end <= second.1);
            assert_eq!(accesses[1].retirement.instruction_pc(), second.0);
            assert_eq!(accesses[1].retirement.retired_before(), 1);
            assert!(accesses[1].start >= second.1);
            let failures: Vec<_> = rows
                .iter()
                .filter(|row| row.site.kind() == RecoverySiteKind::GuestCheck)
                .collect();
            assert_eq!(failures.len(), 1);
            assert_eq!(failures[0].retirement.instruction_pc(), pc);
            let mut mask = Code::pinned();
            super::super::unreachable_if(&mut mask.asm(), Cond::A);
            let preceding = &grouped.native.x86_64_code[..accesses[0].start];
            assert!(preceding.ends_with(&mask[..]));
        }
    }

    // Replacing the index with the load result invalidates the shared
    // address. The separate store can use the newly committed index.
    let alias = [words[0] & !31 | 23, words[1]];
    let grouped = compile(&alias, pc, ordinary, target, direct);
    let fallback = compile(&alias, pc, ordinary, dit, direct);
    let (store_pc, start) = grouped.native.source_map[1];
    let separate = compile(&alias[1..], store_pc, ordinary, target, direct);
    let add_base = [0x48, 0x03, 0x87, 0x40, 0, 0, 0];
    let load_base = [0x48, 0x8b, 0x87, 0x40, 0, 0, 0];
    let commit_index = [0x48, 0x89, 0x87, 0xb8, 0, 0, 0];
    assert_eq!(store_pc, GuestAddress(pc.0 + 4));
    assert!(grouped.native.x86_64_code[..start].ends_with(&commit_index));
    assert!(grouped.native.x86_64_code[start..].starts_with(&add_base));
    assert!(fallback.native.x86_64_code[fallback.native.source_map[1].1..].starts_with(&load_base));
    assert!(separate.native.x86_64_code.starts_with(&load_base));
    assert_eq!(
        grouped.native.x86_64_code.len() + load_base.len(),
        fallback.native.x86_64_code.len()
    );
    for checks in [
        MemoryChecks::system(true, false, true, true),
        direct.with_translation(0x1000),
    ] {
        let grouped = compile(&words, pc, system, target, checks);
        let fallback = compile(&words, pc, system, dit, checks);
        assert_eq!(grouped.native.x86_64_code, fallback.native.x86_64_code);
    }
}

#[test]
fn pinned_carry_run_commits_flags_once_before_the_next_guest_fault() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };

    let start = GuestAddress(0xef290);
    let words = [
        0xba0e_0317u32, // ADCS X23,X24,X14
        0xba0f_0338,    // ADCS X24,X25,X15
        0xba10_0359,    // ADCS X25,X26,X16
        0xba11_039a,    // ADCS X26,X28,X17
        0xf940_0349,    // LDR X9,[X26]
    ];
    let compile = |words: &[u32], pc, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let profile = BlockProfile::new(Features::SYSTEM_MODE);
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let target = TargetCpu::Baseline;
    let grouped = compile(&words, start, target);
    let dit = compile(&words, start, target.with_data_independent_timing());
    let store = nzcv_store(true);
    let stores_flags = |native: &NativeBlock, row: usize| {
        let offset = native.source_map[row].1;
        let end = native
            .source_map
            .get(row + 1)
            .map_or(native.x86_64_code.len(), |row| row.1);
        native.x86_64_code[offset..end]
            .windows(store.len())
            .any(|bytes| bytes == store)
    };
    assert_eq!(grouped.native.source_map.len(), words.len());
    for row in 0..4 {
        assert_eq!(
            grouped.native.source_map[row].0,
            GuestAddress(start.0 + row as u64 * 4)
        );
        assert_eq!(stores_flags(&grouped.native, row), row == 3);
        assert!(stores_flags(&dit.native, row));
    }
    let access = grouped
        .fault_recovery
        .iter()
        .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
        .unwrap();
    assert_eq!(
        access.retirement.instruction_pc(),
        GuestAddress(start.0 + 16)
    );
    assert_eq!(access.retirement.retired_before(), 4);
    assert!(access.start >= grouped.native.source_map[4].1);
    assert!(access.bindings.is_empty());

    // Prefix clipping materializes the last flag result actually reached.
    for len in 1..=4 {
        let clipped = compile(&words[..len], start, target);
        assert_eq!(clipped.native.source_map.len(), len);
        assert!(stores_flags(&clipped.native, len - 1));
    }
    let independent = compile(&words[3..], GuestAddress(start.0 + 12), target);
    assert!(stores_flags(&independent.native, 0));

    let bounded = compile(&[words[0]; 9], start, target);
    for row in 0..9 {
        assert_eq!(stores_flags(&bounded.native, row), row >= 7);
    }
}

#[cfg(all(target_arch = "x86_64", any(target_os = "linux", target_os = "macos")))]
#[test]
fn pinned_carry_run_matches_dit_for_mixed_widths_and_borrow() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features},
        context::ExecutionContext,
        x86_64::{asm::vector_extensions::execution::Native, pinned::entered},
    };
    let compile = |words: &[u32], target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let profile = BlockProfile::new(Features::SYSTEM_MODE);
        let body = CompilationPlan::new(
            &source,
            GuestAddress(0x100000),
            &Options::new(profile, target),
        )
        .unwrap()
        .compile()
        .unwrap();
        Native::new(&entered(&body.native.x86_64_code))
    };
    let target = TargetCpu::Baseline;
    let dit = target.with_data_independent_timing();
    for final_subtract in [0, 1 << 30] {
        let words = [
            0xfa0e_02e8u32,               // SBCS X8,X23,X14 (read incoming guest carry)
            0x7a08_0288,                  // SBCS W8,W20,W8 (right aliases pinned destination)
            0xba08_0210,                  // ADCS X16,X16,X8 (consume zero-extended W8)
            0x3a1f_0201 | final_subtract, // ADCS/SBCS W1,W16,WZR
        ];
        let grouped = compile(&words, target);
        let fallback = compile(&words, dit);
        for source in [0, 1, u64::MAX, 0x8000_0000_8000_0000] {
            for nzcv in [0, 0x2000_0000, 0xf000_0000] {
                let mut original = crate::GuestState {
                    sp: 0x1234,
                    nzcv,
                    ..Default::default()
                };
                for (index, register) in original.x.iter_mut().enumerate() {
                    *register = 0xfedc_ba98_7654_3210u64.wrapping_mul(index as u64 + 1);
                }
                original.x[23] = source;
                let mut result = original.clone();
                grouped.run(&mut result, &mut ExecutionContext::default());
                fallback.run(&mut original, &mut ExecutionContext::default());
                assert_eq!(result, original);
            }
        }
    }
}

#[test]
fn pinned_carry_follows_live_flags_and_keeps_the_next_fault_boundary() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };
    let start = GuestAddress(0xeee3c);
    let words = [
        0xab06_0273u32, // ADDS X19,X19,X6
        0xba07_0294,    // ADCS X20,X20,X7
        0xf940_0289,    // LDR X9,[X20]
    ];
    let compile = |words: &[u32], pc, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let profile = BlockProfile::new(Features::SYSTEM_MODE);
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let target = TargetCpu::Baseline;
    let grouped = compile(&words, start, target);
    let dit = compile(&words, start, target.with_data_independent_timing());
    let second = GuestAddress(start.0 + 4);
    let independent = compile(&words[1..], second, target);
    let mut scratch = Code::pinned();
    super::super::flag_word::carry(&mut scratch.asm(), 29);
    let reads_carry = |native: &NativeBlock, row: usize| {
        let start = native.source_map[row].1;
        let end = native.source_map[row + 1].1;
        native.x86_64_code[start..end]
            .windows(scratch.len())
            .any(|bytes| bytes == &scratch[..])
    };
    assert!(!reads_carry(&grouped.native, 1));
    assert!(reads_carry(&dit.native, 1));
    assert!(reads_carry(&independent.native, 0));
    assert_eq!(grouped.native.source_map.len(), words.len());
    assert_eq!(grouped.native.source_map[1].0, second);
    assert_eq!(grouped.native.source_map[2].0, GuestAddress(start.0 + 8));
    let access = grouped
        .fault_recovery
        .iter()
        .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
        .unwrap();
    assert_eq!(
        access.retirement.instruction_pc(),
        grouped.native.source_map[2].0
    );
    assert_eq!(access.retirement.retired_before(), 2);
    assert!(access.start >= grouped.native.source_map[2].1);
    assert!(access.bindings.is_empty());
}

#[cfg(all(target_arch = "x86_64", any(target_os = "linux", target_os = "macos")))]
#[test]
fn pinned_carry_matches_fallback_for_widths_aliases_and_zero_registers() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features},
        context::ExecutionContext,
        x86_64::{asm::vector_extensions::execution::Native, pinned::entered},
    };
    let compile = |words: [u32; 2], target| {
        let source: Vec<_> = words.into_iter().flat_map(u32::to_le_bytes).collect();
        let profile = BlockProfile::new(Features::SYSTEM_MODE);
        let options = Options::new(profile, target);
        let body = CompilationPlan::new(&source, GuestAddress(0x100000), &options)
            .unwrap()
            .compile()
            .unwrap();
        Native::new(&entered(&body.native.x86_64_code))
    };
    let target = TargetCpu::Baseline;
    let dit = target.with_data_independent_timing();
    for wide in [0, 1 << 31] {
        for producer in [0x2b03_004f, 0x6b03_004f] {
            for kind in [0, 1 << 29, 1 << 30, 3 << 29] {
                for (destination, left, right) in [
                    (8, 20, 8),
                    (19, 19, 19),
                    (1, 31, 0),
                    (0, 0, 31),
                    (31, 2, 8),
                    (7, 31, 31),
                    (17, 16, 1),
                ] {
                    let consumer = 0x1a00_0000 | kind | right << 16 | left << 5 | destination;
                    let words = [wide | producer, wide | consumer];
                    let grouped = compile(words, target);
                    let fallback = compile(words, dit);
                    for pair in [
                        [0, 1],
                        [1, 0],
                        [u64::MAX, 1],
                        [0x8000_0000_8000_0000, u64::MAX],
                    ] {
                        let mut original = crate::GuestState {
                            sp: 0x1234,
                            nzcv: 0x9000_0000,
                            ..Default::default()
                        };
                        for (index, register) in original.x.iter_mut().enumerate() {
                            *register = 0xfedc_ba98_7654_3210u64.wrapping_mul(index as u64 + 1);
                        }
                        original.x[2..4].copy_from_slice(&pair);
                        let mut result = original.clone();
                        grouped.run(&mut result, &mut ExecutionContext::default());
                        fallback.run(&mut original, &mut ExecutionContext::default());
                        assert_eq!(
                            result, original,
                            "encodings {words:08x?}, inputs {pair:016x?}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn a_pinned_run_access_recovers_the_locals_it_wrote() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoveryBinding, RecoverySiteKind},
    };

    let words = [
        0x9100_0449u32, // ADD X9,X2,#1
        0x9100_086a,    // ADD X10,X3,#2
        0x8b0a_012b,    // ADD X11,X9,X10
        0xcb0b_012c,    // SUB X12,X9,X11
        0x3940_0d2d,    // LDRB W13,[X9,#3]
    ];
    let start = GuestAddress(0x1000045d4);
    let compile = |words: &[u32], start, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let profile = BlockProfile::new(Features::SYSTEM_MODE);
        CompilationPlan::new(&source, start, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let target = TargetCpu::Baseline;
    let dit = target.with_data_independent_timing();
    let run = compile(&words, start, target);
    let without = compile(&words, start, dit);
    assert_eq!(run.native.source_map.len(), words.len());
    for (index, (pc, _)) in run.native.source_map.iter().enumerate() {
        assert_eq!(pc.0, start.0 + index as u64 * 4);
    }
    assert!(
        run.native
            .source_map
            .windows(2)
            .all(|rows| rows[0].1 < rows[1].1)
    );
    assert!(run.native.x86_64_code.len() < without.native.x86_64_code.len());
    // X9 is the run's local in R9: the access finds the value the run
    // wrote there, which the frame does not hold yet.
    let access = run
        .fault_recovery
        .iter()
        .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
        .unwrap();
    assert_eq!(
        access.retirement.instruction_pc(),
        run.native.source_map[4].0
    );
    assert_eq!(access.retirement.retired_before(), 4);
    assert!(access.start >= run.native.source_map[4].1);
    assert_eq!(access.bindings, [RecoveryBinding::gpr(9, 9)]);
    let without_access = without
        .fault_recovery
        .iter()
        .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
        .unwrap();
    assert!(without_access.bindings.is_empty());

    // Execute an X/W mix through the real pinned ABI with stale high input
    // bits, unchanged flags and unmodified permanently pinned values.
    #[cfg(all(target_arch = "x86_64", any(target_os = "linux", target_os = "macos")))]
    {
        use crate::x86_64::asm::vector_extensions::execution::Native;
        let mixed = [words[0] & !(1 << 31), words[1], words[2], words[3]];
        let native = compile(&mixed, start, target).native;
        let original = compile(&mixed, start, dit).native;
        assert!(native.x86_64_code.len() < original.x86_64_code.len());
        let body = Native::new(&super::super::pinned::entered(&native.x86_64_code));
        let mut initial = crate::GuestState {
            nzcv: 0x9000_0000,
            sp: 0x1234,
            ..Default::default()
        };
        for (index, register) in initial.x.iter_mut().enumerate() {
            *register = 0xfedc_ba98_7654_3210u64.wrapping_mul(index as u64 + 1);
        }
        let mut expected = initial.clone();
        expected.x[9] = initial.x[2].wrapping_add(1) as u32 as u64;
        expected.x[10] = initial.x[3].wrapping_add(2);
        expected.x[11] = expected.x[9].wrapping_add(expected.x[10]);
        expected.x[12] = expected.x[9].wrapping_sub(expected.x[11]);
        body.run(
            &mut initial,
            &mut crate::context::ExecutionContext::default(),
        );
        assert_eq!(initial, expected);
    }
}

#[test]
fn bitfields_and_pc_relative_addresses_feed_the_next_frame_reader() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };
    let compile = |words: &[u32], pc, profile, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let target = TargetCpu::Baseline;
    let ordinary = BlockProfile::ORDINARY;
    let system = BlockProfile::new(Features::SYSTEM_MODE);
    let lz4 = GuestAddress(0x10000800c);
    let lz4_words = [0xd372_fce7, 0x927e_2cf7]; // LSR X7,X7,#50; AND X23,X7,#0x3ffc
    let mut scratch = Code::default();
    let load_x7 = super::super::register_load(&mut scratch, Register::X(7), false).unwrap();
    let grouped = compile(&lz4_words, lz4, ordinary, target);
    let second = grouped.native.source_map[1];
    assert_eq!(second.0, GuestAddress(lz4.0 + 4));
    assert!(!grouped.native.x86_64_code[second.1..].starts_with(load_x7));
    let independent = compile(&lz4_words[1..], second.0, ordinary, target);
    assert!(independent.native.x86_64_code.starts_with(load_x7));
    let dit = compile(
        &lz4_words,
        lz4,
        ordinary,
        target.with_data_independent_timing(),
    );
    assert!(dit.native.x86_64_code[dit.native.source_map[1].1..].starts_with(load_x7));

    let sqlite = GuestAddress(0x10000fee0);
    let sqlite_words = [0x106f_9c89, 0x7868_7929]; // ADR X9,...; LDRH W9,[X9,X8,LSL #1]
    for profile in [ordinary, system] {
        let load_x9 = super::super::register_load(
            &mut scratch,
            Register::X(9),
            profile.has(Features::SYSTEM_MODE),
        )
        .unwrap();
        let grouped = compile(&sqlite_words, sqlite, profile, target);
        let second = grouped.native.source_map[1];
        assert_eq!(second.0, GuestAddress(sqlite.0 + 4));
        assert!(!grouped.native.x86_64_code[second.1..].starts_with(load_x9));
        let independent = compile(&sqlite_words[1..], second.0, profile, target);
        assert!(independent.native.x86_64_code.starts_with(load_x9));
        if profile.has(Features::SYSTEM_MODE) {
            let access = grouped
                .fault_recovery
                .iter()
                .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
                .unwrap();
            assert_eq!(access.retirement.instruction_pc(), second.0);
            assert_eq!(access.retirement.retired_before(), 1);
            assert!(access.start >= second.1);
        }
    }
}

#[test]
fn vector_store_reuses_load_across_integer_rows_and_keeps_recovery() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features, RecoverySiteKind},
    };
    let words = [
        0x3dc0_0280u32, // LDR Q0,[X20]
        0x8b02_0288,    // ADD X8,X20,X2
        0x9240_0ce7,    // AND X7,X7,#0xf
        0x8b02_0133,    // ADD X19,X9,X2
        0x7100_3cff,    // CMP W7,#15
        0x3d80_0120,    // STR Q0,[X9]
        0x7840_2514,    // LDRH W20,[X8],#2
        0xcb14_0262,    // SUB X2,X19,X20
        0x5400_00a0,    // B.EQ
    ];
    let start = GuestAddress(0x100008e04);
    let store = GuestAddress(start.0 + 20);
    let compile = |words: &[u32], pc, profile, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(&source, pc, &Options::new(profile, target))
            .unwrap()
            .compile()
            .unwrap()
    };
    let reloads = |block: &NativeBlock, index: usize| {
        let offset = block.source_map[index].1;
        let end = block
            .source_map
            .get(index + 1)
            .map_or(block.x86_64_code.len(), |row| row.1);
        block.x86_64_code[offset..end]
            .windows(8)
            .any(|bytes| bytes == [0xf3, 0x0f, 0x6f, 0x87, 8, 1, 0, 0])
    };
    let baseline = TargetCpu::Baseline;
    for profile in [
        BlockProfile::ORDINARY,
        BlockProfile::new(Features::SYSTEM_MODE),
    ] {
        let grouped = compile(&words, start, profile, baseline);
        assert_eq!(grouped.native.source_map.len(), words.len());
        assert_eq!(grouped.native.source_map[5].0, store);
        assert!(!reloads(&grouped.native, 5));

        let dit = compile(
            &words,
            start,
            profile,
            baseline.with_data_independent_timing(),
        );
        assert!(reloads(&dit.native, 5));
        let standalone = compile(&words[5..], store, profile, baseline);
        assert!(reloads(&standalone.native, 0));
        if profile.has(Features::SYSTEM_MODE) {
            let access = grouped
                .fault_recovery
                .iter()
                .find(|entry| {
                    entry.site.kind() == RecoverySiteKind::GuestAccess
                        && entry.retirement.instruction_pc() == store
                })
                .unwrap();
            assert_eq!(access.retirement.retired_before(), 5);
            assert!(access.start >= grouped.native.source_map[5].1);
            assert!(access.end <= grouped.native.source_map[6].1);
        }
    }

    // A vector writer between the original load and store replaces XMM0.
    let clobbered = compile(
        &[words[0], 0x4f00_e401, words[2], words[5]],
        start,
        BlockProfile::ORDINARY,
        baseline,
    );
    assert!(reloads(&clobbered.native, 3));
}

#[test]
fn reverse_dword_runs_share_one_mask_with_independent_source_rows() {
    use crate::{Lifter, TargetCpu, aarch64::Decoder};

    let entry = GuestAddress(0x259f5c);
    let end = GuestAddress(entry.0 + 16);
    let instructions: Vec<_> = [0x6e20_0884, 0x6e20_08a5, 0x6e20_08c6, 0x6e20_08e7]
        .into_iter()
        .enumerate()
        .map(|(index, word)| {
            Decoder
                .lift(GuestAddress(entry.0 + index as u64 * 4), word)
                .unwrap()
        })
        .collect();
    let ordinary =
        |source: &[Instruction], target| emit_ordinary_for_target(source, None, end, target);
    let grouped = ordinary(&instructions, TargetCpu::Ssse3);
    let baseline = ordinary(&instructions, TargetCpu::Baseline);
    let dit = ordinary(
        &instructions,
        TargetCpu::Ssse3.with_data_independent_timing(),
    );
    let shuffles = |code: &NativeBlock| {
        code.x86_64_code
            .windows(3)
            .filter(|word| *word == [0x0f, 0x38, 0x00])
            .count()
    };
    assert_eq!(shuffles(&grouped), 4);
    assert_eq!(shuffles(&baseline), 0);
    assert_eq!(shuffles(&dit), 8);
    assert!(grouped.x86_64_code.len() < baseline.x86_64_code.len());
    assert!(baseline.x86_64_code.len() < dit.x86_64_code.len());
    for code in [&grouped, &baseline, &dit] {
        assert_eq!(code.source_map.len(), 4);
        assert!(code.source_map.windows(2).all(|rows| rows[0].1 < rows[1].1));
        for (row, instruction) in code.source_map.iter().zip(&instructions) {
            assert_eq!(row.0, instruction.pc);
        }
    }
    let singleton = ordinary(&instructions[1..2], TargetCpu::Ssse3);
    assert_eq!(singleton.source_map, [(instructions[1].pc, 0)]);
    assert_eq!(shuffles(&singleton), 0);
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn reverse_dword_runs_commit_aliased_vectors_in_guest_order() {
    use crate::{GuestState, Lifter, TargetCpu, aarch64::Decoder, x86_64::native};

    let entry = GuestAddress(0x1000);
    let end = GuestAddress(entry.0 + 16);
    let operations = [(4, 4), (4, 4), (4, 5), (6, 4)];
    let instructions: Vec<_> = operations
        .iter()
        .enumerate()
        .map(|(index, &(destination, source))| {
            Decoder
                .lift(
                    GuestAddress(entry.0 + index as u64 * 4),
                    0x6e20_0800 | (source << 5) | destination,
                )
                .unwrap()
        })
        .collect();
    for target in [TargetCpu::Baseline, TargetCpu::Ssse3] {
        if target.supports(TargetCpu::Ssse3) && !std::arch::is_x86_feature_detected!("ssse3") {
            continue;
        }
        let code = emit_ordinary_for_target(&instructions, None, end, target);
        let mut state = GuestState {
            v: core::array::from_fn(|i| {
                let value = (i as u64).wrapping_mul(0x0e7c_61ab_da86_a105);
                [value, value.rotate_left(29)]
            }),
            nzcv: 0xb000_0000,
            fpsr: 0x800_0010,
            ..GuestState::default()
        };
        let mut expected = state.clone();
        for (destination, source) in operations {
            expected.v[destination as usize] = expected.v[source as usize].map(|half| {
                u64::from((half as u32).swap_bytes())
                    | (u64::from(((half >> 32) as u32).swap_bytes()) << 32)
            });
        }
        assert_eq!(native::run(&code.x86_64_code, &mut state), end.0);
        assert_eq!(state, expected);
    }
}

#[test]
fn hosted_result_chain_keeps_both_zlib_compare_inputs() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan},
    };

    let compile = |words: &[u32], entry, target| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        CompilationPlan::new(
            &source,
            GuestAddress(entry),
            &Options::new(BlockProfile::ORDINARY, target),
        )
        .unwrap()
        .compile()
        .unwrap()
        .native
    };
    let entry = 0x100003df0;
    // MOV W5,W1; ADD X4,X8,X5; ADD X6,X4,W12,SXTW;
    // LDRB W7,[X6]; CMP W7,W3; B.NE.
    let address = [
        0x2a01_03e5,
        0x8b05_0104,
        0x8b2c_c086,
        0x3940_00c7,
        0x6b03_00ff,
        0x5400_0ea1,
    ];
    let native = compile(&address, entry, TargetCpu::Baseline);
    let bytes = &native.x86_64_code;
    assert_eq!(native.source_map.len(), address.len());
    for (index, (pc, _)) in native.source_map.iter().enumerate() {
        assert_eq!(*pc, GuestAddress(entry + index as u64 * 4));
    }
    assert!(bytes[native.source_map[1].1..].starts_with(&[0x48, 0x03, 0x87, 0x40]));
    assert!(bytes[native.source_map[3].1..].starts_with(&[0x0f, 0xb6, 0x00]));
    assert!(bytes[native.source_map[4].1..].starts_with(&[0x3b, 0x87, 0x18]));
    let compare = compile(&address[4..], entry + 16, TargetCpu::Baseline);
    assert!(compare.x86_64_code.starts_with(&[0x48, 0x8b, 0x87, 0x38]));

    // AND W1,W1,W14; LDRH W1,[X9,W1,UXTW #1]; CMP W13,W1; B.HS.
    let indexed = [0x0a0e_0021, 0x7861_5921, 0x6b01_01bf, 0x5400_0062];
    let entry = 0x100003fd8;
    let native = compile(&indexed, entry, TargetCpu::Baseline);
    let dit = compile(
        &indexed,
        entry,
        TargetCpu::Baseline.with_data_independent_timing(),
    );
    assert_eq!(native.source_map.len(), indexed.len());
    let held_compare = [0x39, 0x87, 0x68]; // CMP [frame+X13],EAX.
    assert!(native.x86_64_code[native.source_map[2].1..].starts_with(&held_compare));
    assert!(!dit.x86_64_code[dit.source_map[2].1..].starts_with(&held_compare));
}

#[test]
fn system_address_pair_keeps_source_rows_and_the_following_fault_boundary() {
    use crate::{Lifter, TargetCpu, aarch64::Decoder, block::RecoverySiteKind};

    let entry = GuestAddress(0x1030);
    let add_pc = GuestAddress(entry.0 + 4);
    let load_pc = GuestAddress(entry.0 + 8);
    let fallthrough = GuestAddress(entry.0 + 12);
    let checks = super::super::MemoryChecks::DEFAULT;
    for (register, opcode) in [(0, [0x49, 0xbf]), (20, [0x48, 0xb8])] {
        let instructions = [
            Decoder.lift(entry, 0xb000_0000 | register).unwrap(),
            Decoder
                .lift(add_pc, 0x9140_2000 | register << 5 | register)
                .unwrap(),
            Decoder.lift(load_pc, 0xf940_0002 | register << 5).unwrap(),
        ]; // adrp; add #8, lsl #12; ldr x2,[register]
        let system = |input: &[Instruction], next, target| {
            emit_system_mode(
                input,
                None,
                next,
                target,
                checks,
                super::super::NativeSystem::NONE,
            )
        };
        let (native, recovery) = system(&instructions, fallthrough, TargetCpu::Baseline);
        let (dit, _) = system(
            &instructions,
            fallthrough,
            TargetCpu::Baseline.with_data_independent_timing(),
        );
        let expected_end = if register == 0 { 11 } else { 18 };
        assert_eq!(
            native.source_map,
            [(entry, 0), (add_pc, 1), (load_pc, expected_end)]
        );
        assert_eq!(native.x86_64_code[0], 0x90);
        assert_eq!(native.x86_64_code[1..3], opcode);
        assert_eq!(native.x86_64_code[3..11], 0xa000_u64.to_le_bytes());
        assert!(dit.source_map[1].1 > 1);
        assert!(native.x86_64_code.len() < dit.x86_64_code.len());
        assert!(!recovery.is_empty());
        assert!(recovery.iter().all(|row| row.start >= expected_end));
        let access = recovery
            .iter()
            .find(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
            .unwrap();
        assert_eq!(access.retirement.instruction_pc(), load_pc);
        assert_eq!(access.retirement.retired_before(), 2);

        let (truncated, _) = system(&instructions[..1], add_pc, TargetCpu::Baseline);
        assert_eq!(truncated.source_map, [(entry, 0)]);
        assert_eq!(truncated.x86_64_code[2..10], 0x2000_u64.to_le_bytes());
        let (standalone, _) = system(&instructions[1..2], load_pc, TargetCpu::Baseline);
        assert_eq!(standalone.source_map, [(add_pc, 0)]);
        assert_ne!(standalone.x86_64_code[0], 0x90);
    }
}

#[test]
fn system_address_pair_requires_an_adjacent_flagless_x64_overwrite() {
    use crate::{Lifter, TargetCpu, aarch64::Decoder};

    let entry = GuestAddress(0x1000);
    let first = Decoder.lift(entry, 0xb000_0000).unwrap();
    for (pc, word) in [
        (0x1008, 0x9100_2000), // Gap.
        (0x1004, 0x1100_2000), // W destination.
        (0x1004, 0xb100_2000), // Flag-setting ADD.
        (0x1004, 0xd100_2000), // SUB.
        (0x1004, 0x9100_2001), // Different destination.
        (0x1004, 0x9100_2020), // Different source.
    ] {
        let second = Decoder.lift(GuestAddress(pc), word).unwrap();
        let (native, _) = emit_system_mode(
            &[first.clone(), second],
            None,
            GuestAddress(pc + 4),
            TargetCpu::Baseline,
            super::super::MemoryChecks::DEFAULT,
            super::super::NativeSystem::NONE,
        );
        assert_eq!(native.source_map, [(entry, 0), (GuestAddress(pc), 10)]);
        assert_eq!(native.x86_64_code[2..10], 0x2000_u64.to_le_bytes());
    }
}

/// A reload removed before a check takes the check's jump along.
#[test]
fn removed_instructions_move_the_check_failures_after_them() {
    use super::super::asm::{Cond, Gpr, Mem, Size};
    let mut code = Code::new();
    code.check_failure(Cond::E);
    let start = code.len();
    code.asm()
        .mov_load(Size::Qword, Gpr::Rax, Mem::base(Gpr::Rdi, 0));
    let end = code.len();
    code.check_failure(Cond::Ne);
    code.remove_instruction(start..end);
    code.place_failures();
    assert_eq!(
        code[..],
        [
            0x0f, 0x84, 6, 0, 0, 0, 0x0f, 0x85, 2, 0, 0, 0, 0x0f, 0x0b, 0x0f, 0x0b
        ],
    );
}

/// A conditional exit's branch skips its fallthrough exit, and each of
/// its instructions counts.
#[test]
fn conditional_exits_branch_over_the_fallthrough_exit() {
    let mut code = Code::new();
    conditional_exit(
        &mut code,
        Some(Cond::Ne),
        GuestAddress(0x2000),
        GuestAddress(0x1004),
    );
    let fallthrough = super::super::chain::direct_exit(GuestAddress(0x1004), ExitKind::Fallthrough);
    let taken = super::super::chain::direct_exit(GuestAddress(0x2000), ExitKind::Direct);
    assert_eq!(code[..6], [0x0f, 0x85, 24, 0, 0, 0]);
    assert_eq!(code[6..30], fallthrough);
    assert_eq!(code[30..], taken);
    assert_eq!(code.instructions(), 23);
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn grouped_sha256_schedule_removes_only_the_intermediate_frame_round_trip() {
    use crate::{
        GuestState, Lifter, TargetCpu,
        aarch64::Decoder,
        x86_64::{native, pinned},
    };

    let pc = GuestAddress(0x1000);
    let words = [
        0x5e28_28a4_u32,
        0x4ea0_1c02,
        0x5e10_4020,
        0x5e10_5041,
        0x5e07_60c4,
    ];
    let instructions: Vec<_> = words
        .into_iter()
        .enumerate()
        .map(|(i, word)| {
            Decoder
                .lift(GuestAddress(pc.0 + i as u64 * 4), word)
                .unwrap()
        })
        .collect();
    let checks = super::super::MemoryChecks::DEFAULT;
    for target in [
        TargetCpu::Baseline,
        TargetCpu::Ssse3,
        TargetCpu::Sha,
        TargetCpu::Ssse3.with_sha(),
    ] {
        let system = |input: &[Instruction]| {
            let pc = GuestAddress(input.last().unwrap().pc.0 + 4);
            let terminal = (
                pc,
                Terminator::Register {
                    source: Register::X(30),
                    kind: ExitKind::Return,
                },
            );
            let next = GuestAddress(pc.0 + 4);
            let (block, faults) = emit_system_mode(
                input,
                Some(&terminal),
                next,
                target,
                checks,
                super::super::NativeSystem::NONE,
            );
            assert!(faults.is_empty());
            (block, terminal, next)
        };
        let body_bytes = |input: &[Instruction]| system(input).0.source_map.last().unwrap().1;
        let separate_bytes = body_bytes(&instructions[..1])
            + body_bytes(&instructions[1..4])
            + body_bytes(&instructions[4..]);
        assert_eq!(body_bytes(&instructions) + 16, separate_bytes);

        if (target.supports(TargetCpu::Sha) && !std::arch::is_x86_feature_detected!("sha"))
            || (target.supports(TargetCpu::Ssse3) && !std::arch::is_x86_feature_detected!("ssse3"))
        {
            continue;
        }
        for input in [
            &instructions[..],
            &instructions[..4],
            &instructions[1..],
            &instructions[4..],
        ] {
            let (grouped, terminal, next) = system(input);
            let plain = emit_for_target(input, Some(&terminal), next, target, checks);
            let rows =
                |code: &NativeBlock| code.source_map.iter().map(|row| row.0).collect::<Vec<_>>();
            assert_eq!(rows(&grouped), rows(&plain));
            assert!(
                grouped
                    .source_map
                    .windows(2)
                    .all(|pair| pair[0].1 < pair[1].1)
            );
            let entered = pinned::entered(&grouped.x86_64_code);
            let mut state = GuestState {
                x: core::array::from_fn(|index| u64::MAX ^ index as u64),
                v: core::array::from_fn(|index| {
                    let word = (index as u64).wrapping_mul(0x1234_5678_8765_4321);
                    [word, !word.rotate_left(11)]
                }),
                nzcv: 0xa000_0000,
                fpsr: 0x800_0000,
                ..GuestState::default()
            };
            let mut expected = state.clone();
            let exit = native::run(&plain.x86_64_code, &mut expected);
            assert_eq!(native::run(&entered, &mut state), exit);
            assert_eq!(state, expected);
        }
    }
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn grouped_sha256_rounds_preserve_three_rows_and_both_state_halves() {
    use crate::{
        GuestState, Lifter, TargetCpu,
        aarch64::Decoder,
        x86_64::{native, pinned},
    };

    let pc = GuestAddress(0x1000);
    let checks = super::super::MemoryChecks::DEFAULT;
    for (a, b, temporary, wk) in [(0, 1, 2, 16), (31, 30, 29, 29), (6, 7, 8, 7)] {
        let instructions: Vec<_> = [
            0x4ea0_1c00 | temporary | a << 5 | a << 16,
            0x5e00_4000 | a | b << 5 | wk << 16,
            0x5e00_5000 | b | temporary << 5 | wk << 16,
        ]
        .into_iter()
        .enumerate()
        .map(|(index, word)| {
            Decoder
                .lift(GuestAddress(pc.0 + index as u64 * 4), word)
                .unwrap()
        })
        .collect();
        for target in [TargetCpu::Baseline, TargetCpu::Sha] {
            for (input, separate_hashes) in [
                (&instructions[..], 2),
                (&instructions[..2], 1),
                (&instructions[1..], 2),
                (&instructions[2..], 1),
            ] {
                let return_pc = GuestAddress(input.last().unwrap().pc.0 + 4);
                let terminal = (
                    return_pc,
                    Terminator::Register {
                        source: Register::X(30),
                        kind: ExitKind::Return,
                    },
                );
                let next = GuestAddress(return_pc.0 + 4);
                let (grouped, faults) = emit_system_mode(
                    input,
                    Some(&terminal),
                    next,
                    target,
                    checks,
                    super::super::NativeSystem::NONE,
                );
                let plain = emit_for_target(input, Some(&terminal), next, target, checks);
                let rows = |block: &NativeBlock| {
                    block.source_map.iter().map(|row| row.0).collect::<Vec<_>>()
                };
                assert_eq!(rows(&grouped), rows(&plain));
                assert!(
                    grouped
                        .source_map
                        .windows(2)
                        .all(|rows| rows[0].1 < rows[1].1)
                );
                assert!(faults.is_empty());
                let rounds = |code: &[u8]| {
                    code.windows(3)
                        .filter(|bytes| *bytes == [0x0f, 0x38, 0xcb])
                        .count()
                };
                if target.supports(TargetCpu::Sha) {
                    let grouped_rounds = if input.len() == 3 {
                        2
                    } else {
                        2 * separate_hashes
                    };
                    assert_eq!(rounds(&grouped.x86_64_code), grouped_rounds);
                    assert_eq!(rounds(&plain.x86_64_code), 2 * separate_hashes);
                } else {
                    assert_eq!(rounds(&grouped.x86_64_code), 0);
                    assert_eq!(rounds(&plain.x86_64_code), 0);
                }
                if input.len() == 3 {
                    assert!(grouped.source_map[3].1 < plain.source_map[3].1);
                }
                if target.supports(TargetCpu::Sha) && !std::arch::is_x86_feature_detected!("sha") {
                    continue;
                }
                let entered = pinned::entered(&grouped.x86_64_code);
                for seed in [0, 1, u64::MAX] {
                    let mut state = GuestState {
                        x: core::array::from_fn(|index| seed ^ index as u64),
                        v: core::array::from_fn(|index| {
                            let x = (index as u64).wrapping_mul(0x1234_5678_8765_4321);
                            [x ^ seed, x.rotate_left(17) ^ !seed]
                        }),
                        nzcv: 0xa000_0000,
                        fpsr: 0x800_0000,
                        ..GuestState::default()
                    };
                    let mut expected = state.clone();
                    let exit = native::run(&plain.x86_64_code, &mut expected);
                    assert_eq!(native::run(&entered, &mut state), exit);
                    assert_eq!(state, expected);
                }
            }
        }
    }
}

#[cfg(feature = "std")]
#[test]
fn ordinary_add_sub_zero_branch_reuses_flags_and_keeps_both_source_rows() {
    use crate::{Lifter, TargetCpu, aarch64::Decoder};

    // The W15 subtraction and CBNZ are the exact zlib-deflate tail words.
    let entry = GuestAddress(0x100003fe8);
    let branch_pc = GuestAddress(entry.0 + 4);
    let fallthrough = GuestAddress(entry.0 + 8);
    for (producer, branch_width) in [(0, 0), (0, 1), (1, 1)] {
        for (subtract, nonzero, register) in [(1, 1, 15), (0, 0, 8)] {
            let alu = 0x1100_0400 | producer << 31 | subtract << 30 | register << 5 | register;
            let branch = 0x34ff_f020 | branch_width << 31 | nonzero << 24 | register;
            let instructions = [Decoder.lift(entry, alu).unwrap()];
            let terminal = (
                branch_pc,
                Terminator::Branch(Decoder.lift(branch_pc, branch).unwrap().operation),
            );
            let baseline = emit(&instructions, Some(&terminal), fallthrough);
            let candidate = emit_ordinary_for_target(
                &instructions,
                Some(&terminal),
                fallthrough,
                TargetCpu::Baseline,
            );
            let at = baseline.source_map[1].1;
            assert_eq!(candidate.source_map, [(entry, 0), (branch_pc, at)]);
            assert_eq!(candidate.x86_64_code[..at], baseline.x86_64_code[..at]);
            let test_len = if branch_width == 0 { 2 } else { 3 };
            assert_eq!(
                candidate.x86_64_code[at..],
                baseline.x86_64_code[at + test_len..]
            );
            assert_eq!(
                candidate.x86_64_code[at..at + 2],
                [0x0f, 0x84 + nonzero as u8]
            );

            let dit = TargetCpu::Baseline.with_data_independent_timing();
            assert_eq!(
                emit_ordinary_for_target(&instructions, Some(&terminal), fallthrough, dit)
                    .x86_64_code,
                baseline.x86_64_code,
            );
            let interior =
                emit_ordinary_for_target(&[], Some(&terminal), fallthrough, TargetCpu::Baseline);
            assert_eq!(interior.source_map, [(branch_pc, 0)]);
            assert_eq!(
                interior.x86_64_code,
                emit(&[], Some(&terminal), fallthrough).x86_64_code
            );
            let truncated =
                emit_ordinary_for_target(&instructions, None, branch_pc, TargetCpu::Baseline);
            assert_eq!(truncated.source_map, [(entry, 0)]);
            assert_eq!(
                truncated.x86_64_code,
                emit(&instructions, None, branch_pc).x86_64_code
            );
        }
    }
}

#[test]
fn alu_zero_branch_refuses_incompatible_width_register_and_adjacency() {
    use crate::{Lifter, aarch64::Decoder};

    let entry = GuestAddress(0x1000);
    for (alu, branch, branch_pc) in [
        (0xd100_0400, 0x3500_0040, 0x1004), // SUB X0; CBNZ W0.
        (0x5100_0400, 0x3500_0041, 0x1004), // Branch tests W1.
        (0x5100_0420, 0x3500_0040, 0x1004), // Producer reads W1.
        (0x7100_0400, 0x3500_0040, 0x1004), // SUBS.
        (0x5100_0400, 0x3500_0040, 0x1008), // Nonadjacent.
    ] {
        let instructions = [Decoder.lift(entry, alu).unwrap()];
        let branch_pc = GuestAddress(branch_pc);
        let terminal = (
            branch_pc,
            Terminator::Branch(Decoder.lift(branch_pc, branch).unwrap().operation),
        );
        assert_eq!(alu_zero_branch(&instructions, Some(&terminal)), None);
    }
}

#[test]
fn hosted_trailing_zero_pair_preserves_instruction_boundaries() {
    use crate::{Lifter, TargetCpu, aarch64::Decoder};

    let entry = GuestAddress(0x1000);
    for words in [[0xdac0_0020_u32, 0xdac0_1000], [0x5ac0_0020, 0x5ac0_1000]] {
        let instructions = [
            Decoder.lift(entry, words[0]).unwrap(),
            Decoder.lift(GuestAddress(entry.0 + 4), words[1]).unwrap(),
        ];
        let end = GuestAddress(entry.0 + 8);
        for target in [TargetCpu::Baseline, TargetCpu::Baseline.with_bmi1()] {
            let ordinary = emit_ordinary_for_target(&instructions, None, end, target);
            let separate = emit_for_target(
                &instructions,
                None,
                end,
                target,
                super::super::MemoryChecks::DEFAULT,
            );
            assert_eq!(ordinary.source_map, [(entry, 0), (instructions[1].pc, 1)]);
            assert_eq!(ordinary.x86_64_code[0], 0x90);
            assert!(ordinary.x86_64_code.len() < separate.x86_64_code.len());
            let tzcnt = if words[0] >> 31 == 1 {
                &[0xf3, 0x48, 0x0f, 0xbc][..]
            } else {
                &[0xf3, 0x0f, 0xbc][..]
            };
            assert_eq!(
                ordinary
                    .x86_64_code
                    .windows(tzcnt.len())
                    .any(|window| window == tzcnt),
                target.supports(TargetCpu::Bmi1)
            );
        }
        let dit = TargetCpu::Baseline.with_data_independent_timing();
        assert_eq!(
            emit_ordinary_for_target(&instructions, None, end, dit).x86_64_code,
            emit_for_target(
                &instructions,
                None,
                end,
                dit,
                super::super::MemoryChecks::DEFAULT,
            )
            .x86_64_code
        );
        let (system, _) = emit_system_mode(
            &instructions,
            None,
            end,
            TargetCpu::Baseline,
            super::super::MemoryChecks::DEFAULT,
            super::super::NativeSystem::NONE,
        );
        assert!(system.source_map[1].1 > 1);
    }

    // A different width, source or destination leaves the original pair.
    let first = Decoder.lift(entry, 0xdac0_0020).unwrap();
    for word in [0x5ac0_1000, 0xdac0_1020, 0xdac0_1001] {
        let second = Decoder.lift(GuestAddress(entry.0 + 4), word).unwrap();
        let instructions = [first.clone(), second];
        assert_eq!(
            emit_ordinary_for_target(
                &instructions,
                None,
                GuestAddress(entry.0 + 8),
                TargetCpu::Baseline
            )
            .x86_64_code,
            emit(&instructions, None, GuestAddress(entry.0 + 8)).x86_64_code
        );
    }
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn hosted_trailing_zero_pair_matches_both_scalar_widths_and_aliases() {
    use crate::{GuestState, Lifter, TargetCpu, aarch64::Decoder, x86_64::native};

    let entry = GuestAddress(0x1000);
    let end = GuestAddress(entry.0 + 8);
    for (reverse, count) in [(0xdac0_0000, 0xdac0_1000), (0x5ac0_0000, 0x5ac0_1000)] {
        for source in [0, 1, 31] {
            let instructions = [
                Decoder.lift(entry, reverse | source << 5).unwrap(),
                Decoder.lift(GuestAddress(entry.0 + 4), count).unwrap(),
            ];
            for target in [TargetCpu::Baseline, TargetCpu::Baseline.with_bmi1()] {
                if !TargetCpu::detect().supports(target) {
                    continue;
                }
                let block = emit_ordinary_for_target(&instructions, None, end, target);
                assert_eq!(block.source_map[1].1, 1);
                for input in [0, 1, 0x1234_5678_0000_0000, 1 << 31, 1 << 63, u64::MAX] {
                    let mut state = GuestState::default();
                    state.x[0] = input;
                    state.x[1] = input;
                    state.nzcv = 0xa000_0000;
                    let mut expected = state.clone();
                    let input = if source == 31 { 0 } else { input };
                    expected.x[0] = u64::from(if reverse >> 31 == 1 {
                        input.trailing_zeros()
                    } else {
                        (input as u32).trailing_zeros()
                    });
                    assert_eq!(native::run(&block.x86_64_code, &mut state), end.0);
                    assert_eq!(state, expected);
                }
            }
        }
    }
}

/// add x10, x10, #4; cmp x10, #0x600; b.ne: the CMP takes X10 from
/// RAX where the ADD stored it, and the branch tests the NZCV word the
/// CMP left in EAX, so neither reloads from the frame.
#[test]
fn values_left_in_rax_are_not_reloaded() {
    use crate::{Lifter, aarch64::Decoder};
    const LOAD_X10: [u8; 7] = [0x48, 0x8b, 0x87, 0x50, 0, 0, 0];
    const LOAD_NZCV: [u8; 6] = [0x8b, 0x87, 0, 1, 0, 0];
    let pc = GuestAddress(0x1000);
    let instructions: Vec<_> = [0x9100_114a_u32, 0xf118_015f]
        .into_iter()
        .enumerate()
        .map(|(index, word)| {
            Decoder
                .lift(GuestAddress(pc.0 + 4 * index as u64), word)
                .unwrap()
        })
        .collect();
    let branch = GuestAddress(pc.0 + 8);
    let terminal = (
        branch,
        Terminator::Branch(Operation::BranchCondition {
            target: pc,
            condition: 1,
        }),
    );
    let native = emit(&instructions, Some(&terminal), GuestAddress(branch.0 + 4));
    let code = &native.x86_64_code;
    let count = |pattern: &[u8]| {
        code.windows(pattern.len())
            .filter(|w| *w == pattern)
            .count()
    };
    assert_eq!(count(&LOAD_X10), 1);
    assert_eq!(count(&LOAD_NZCV), 0);
    let compare = native.source_map[1].1;
    assert_eq!(&code[compare..compare + 3], &[0x48, 0x81, 0xf8]); // CMP RAX,imm32.
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn pinned_compare_reuses_the_preceding_unshifted_right_operand() {
    use crate::{GuestState, Lifter, TargetCpu, aarch64::Decoder, x86_64::native};

    let pc = GuestAddress(0x1000);
    let compare = GuestAddress(pc.0 + 4);
    let branch = GuestAddress(pc.0 + 8);
    let next = GuestAddress(pc.0 + 12);
    let terminal = (
        branch,
        Terminator::Branch(Operation::BranchCondition {
            target: pc,
            condition: 3, // LO: borrow.
        }),
    );
    for (sf, mask) in [(0, u64::from(u32::MAX)), (1 << 31, u64::MAX)] {
        for left in [1, 9] {
            let instructions = [
                Decoder.lift(pc, sf | 0x1100_0694).unwrap(), // ADD R20,R20,#1
                Decoder.lift(compare, sf | 0x6b14_001f | left << 5).unwrap(), // CMP Rleft,R20
            ];
            let system = |input: &[Instruction], target| {
                emit_system_mode(
                    input,
                    Some(&terminal),
                    next,
                    target,
                    super::super::MemoryChecks::DEFAULT,
                    super::super::NativeSystem::NONE,
                )
                .0
            };
            let native = system(&instructions, TargetCpu::Baseline);
            let dit = system(
                &instructions,
                TargetCpu::Baseline.with_data_independent_timing(),
            );
            let standalone = system(&instructions[1..], TargetCpu::Baseline);
            let code = &native.x86_64_code;
            let at = native.source_map[1].1;
            let opcode = if sf == 0 { at } else { at + 1 };
            let cmp = if left == 1 {
                &[0x39, 0xc5][..] // CMP EBP/RBP,EAX/RAX.
            } else {
                &[0x39, 0x47, 72][..] // CMP [frame+X9],EAX/RAX.
            };
            assert_eq!(&code[opcode..opcode + cmp.len()], cmp);
            assert!(
                !dit.x86_64_code[dit.source_map[1].1..].starts_with(&code[at..opcode + cmp.len()])
            );
            assert!(
                !standalone
                    .x86_64_code
                    .starts_with(&code[at..opcode + cmp.len()])
            );
            assert_eq!(
                native
                    .source_map
                    .iter()
                    .map(|row| row.0)
                    .collect::<Vec<_>>(),
                [pc, compare, branch]
            );

            let entered = super::super::pinned::entered(code);
            let sign = (mask >> 1) + 1;
            for (a, before) in [
                (1, 0),
                (0, 0),
                (mask, 0),
                (sign - 1, mask - 1),
                (0, sign - 1),
            ] {
                let mut state = GuestState::default();
                state.x[left as usize] = a;
                state.x[20] = before | !mask;
                let mut expected = state.clone();
                let b = before.wrapping_add(1) & mask;
                let difference = a.wrapping_sub(b) & mask;
                expected.x[20] = b;
                expected.nzcv = (u32::from(difference & sign != 0) << 31)
                    | (u32::from(difference == 0) << 30)
                    | (u32::from(a >= b) << 29)
                    | (u32::from((a ^ b) & (a ^ difference) & sign != 0) << 28);
                assert_eq!(
                    native::run(&entered, &mut state),
                    if a < b { pc.0 } else { next.0 }
                );
                assert_eq!(state, expected);
            }
        }
    }
}

#[test]
fn ordinary_integer_operations_read_frame_operands_directly() {
    use crate::{Lifter, aarch64::Decoder};
    let pc = GuestAddress(0x1000);
    // MOV W5,W1; AND W1,W1,W14; CMP W13,W1; CMP W13,#7; ADD X6,X4,W12,SXTW.
    let instructions: Vec<_> = [
        0x2a01_03e5_u32,
        0x0a0e_0021,
        0x6b01_01bf,
        0x7100_1dbf,
        0x8b2c_c086,
    ]
    .into_iter()
    .enumerate()
    .map(|(index, word)| {
        Decoder
            .lift(GuestAddress(pc.0 + 4 * index as u64), word)
            .unwrap()
    })
    .collect();
    let native = emit(&instructions, None, GuestAddress(0x1014));
    let code = &native.x86_64_code;
    let map = &native.source_map;
    assert_eq!(
        &code[..map[1].1],
        &[0x8b, 0x87, 8, 0, 0, 0, 0x48, 0x89, 0x87, 40, 0, 0, 0],
    ); // MOV EAX,[RDI+8]; MOV [RDI+40],RAX.
    assert_eq!(
        &code[map[1].1 + 7..map[1].1 + 13],
        &[0x23, 0x87, 112, 0, 0, 0]
    );
    // CMP [RDI+104],EAX using the preceding AND result.
    assert_eq!(&code[map[2].1..map[2].1 + 6], &[0x39, 0x87, 104, 0, 0, 0]);
    let standalone = emit(&instructions[2..3], None, GuestAddress(0x100c));
    // CMP EAX,[RDI+8] without a producer.
    assert_eq!(&standalone.x86_64_code[7..13], &[0x3b, 0x87, 8, 0, 0, 0]);
    assert_eq!(
        &code[map[3].1..map[3].1 + 7],
        &[0x83, 0xbf, 104, 0, 0, 0, 7]
    );
    assert_eq!(
        &code[map[4].1 + 7..map[4].1 + 14],
        &[0x48, 0x63, 0x8f, 96, 0, 0, 0],
    ); // MOVSXD RCX,[RDI+96].
}

#[test]
fn pinned_scalar_loads_forward_through_the_block() {
    use crate::{Lifter, aarch64::Decoder, block::RecoverySiteKind};
    const FRAME_LOAD: [u8; 4] = [0x48, 0x8b, 0x47, 0x50];
    let pc = GuestAddress(0x1000);
    let second = GuestAddress(0x1004);
    let branch = GuestAddress(0x1008);
    // LDR X10,[X10]; LDR X10,[X10,X2]; CBZ X10,entry.
    let instructions = [
        Decoder.lift(pc, 0xf940_014a).unwrap(),
        Decoder.lift(second, 0xf862_694a).unwrap(),
    ];
    let terminal = (
        branch,
        Terminator::Branch(Operation::BranchZero {
            source: Register::X(10),
            width: crate::aarch64::Width::X64,
            nonzero: false,
            target: pc,
        }),
    );
    let (native, recovery) = emit_system_mode(
        &instructions,
        Some(&terminal),
        GuestAddress(0x100c),
        crate::TargetCpu::Baseline,
        super::super::MemoryChecks::DEFAULT,
        super::super::NativeSystem::NONE,
    );
    let code = &native.x86_64_code;
    assert_eq!(native.source_map.len(), 3);
    let [(pc1, at1), (pc2, at2), (pc3, at3)] = native.source_map.as_slice() else {
        unreachable!()
    };
    assert_eq!([*pc1, *pc2, *pc3], [pc, second, branch]);
    assert!(at1 < at2 && at2 < at3);
    assert_eq!(
        code.windows(FRAME_LOAD.len())
            .filter(|w| *w == FRAME_LOAD)
            .count(),
        1
    );
    assert_eq!(&code[*at3..*at3 + 3], &[0x48, 0x85, 0xc0]); // TEST RAX,RAX
    let accesses: Vec<_> = recovery
        .iter()
        .filter(|row| row.site.kind() == RecoverySiteKind::GuestAccess)
        .collect();
    assert_eq!(accesses.len(), 2);
    for (index, access) in accesses.into_iter().enumerate() {
        let (pc, start) = native.source_map[index];
        assert!(start <= access.start && access.end <= native.source_map[index + 1].1);
        let position = access.retirement.position().unwrap();
        assert_eq!(position.pc, pc);
        assert_eq!(usize::from(position.retired_instructions), index);
    }
}

/// ldr x0, [sp, #8]; ldr x1, [sp, #16]; sub sp, sp, #8; ldr x2, [sp]:
/// the second access relies on the first one's SP alignment check, and
/// the third checks SP again after its move by 8.
#[test]
fn sp_alignment_is_checked_once_while_it_holds() {
    use crate::{Lifter, aarch64::Decoder};
    let pc = GuestAddress(0x1000);
    let instructions: Vec<_> = [0xf940_07e0_u32, 0xf940_0be1, 0xd100_23ff, 0xf940_03e2]
        .into_iter()
        .enumerate()
        .map(|(index, word)| {
            Decoder
                .lift(GuestAddress(pc.0 + 4 * index as u64), word)
                .unwrap()
        })
        .collect();
    let code = emit(&instructions, None, GuestAddress(pc.0 + 16)).x86_64_code;
    let check = super::super::STACK_ALIGNMENT_CHECK;
    let checks = code.windows(check.len()).filter(|w| *w == check).count();
    assert_eq!(checks, 2);
}

/// A vector FMLA whose lanes hold exact zeros completes in the block, as
/// FADD and FMUL do: a zero input or result is exact on both sides, so only
/// infinities, NaNs, subnormals and tiny results need the helper.
#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn vector_fused_multiply_add_keeps_zero_lanes_in_the_block() {
    use crate::{GuestState, Lifter, aarch64::Decoder};
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
        fn mprotect(address: *mut c_void, bytes: usize, protection: i32) -> i32;
        fn munmap(address: *mut c_void, bytes: usize) -> i32;
    }
    #[cfg(target_os = "macos")]
    const MAP_ANONYMOUS: i32 = 0x1000;
    #[cfg(target_os = "linux")]
    const MAP_ANONYMOUS: i32 = 0x20;
    const PAGE: usize = 4096;
    const IXC: u32 = 1 << 4;
    #[repr(C)]
    #[derive(Debug, PartialEq, Eq)]
    struct Exit {
        pc: u64,
        kind: u64,
    }
    let lanes = |values: [f32; 4]| {
        let bits = values.map(|value| u64::from(value.to_bits()));
        [bits[0] | bits[1] << 32, bits[2] | bits[3] << 32]
    };
    let pc = GuestAddress(0x1000);
    // fmla v26.4s, v21.4s, v22.4s
    let instruction = Decoder.lift(pc, 0x4e36_ceba).unwrap();
    let fallthrough = GuestAddress(pc.0 + 4);
    let left = [0.0, 1.5, -0.0, 2.0];
    let right = [3.0, 0.0, 4.0, 3.0];
    let addend = [2.5, -1.25, 0.0, 0.0];
    let expected = lanes(core::array::from_fn(|lane| {
        f32::mul_add(left[lane], right[lane], addend[lane])
    }));
    for target in [crate::TargetCpu::Baseline, crate::TargetCpu::detect()] {
        let code = emit_for_target(
            core::slice::from_ref(&instruction),
            None,
            fallthrough,
            target,
            super::super::MemoryChecks::DEFAULT,
        )
        .x86_64_code;
        let bytes = code.len().next_multiple_of(PAGE);
        // The full path, and the sticky-inexact path.
        for fpsr in [0, IXC] {
            let mut state = GuestState::default();
            state.v[21] = lanes(left);
            state.v[22] = lanes(right);
            state.v[26] = lanes(addend);
            state.fpsr = fpsr;
            // SAFETY: private anonymous memory, RX after the copy and
            // unmapped after the call; the block writes only `state` and
            // its own stack slots.
            let exit = unsafe {
                let page = mmap(core::ptr::null_mut(), bytes, 3, 2 | MAP_ANONYMOUS, -1, 0);
                assert_ne!(page as usize, usize::MAX);
                core::ptr::copy_nonoverlapping(code.as_ptr(), page.cast(), code.len());
                assert_eq!(mprotect(page, bytes, 5), 0);
                let entry: unsafe extern "sysv64" fn(*mut GuestState) -> Exit =
                    core::mem::transmute(page);
                let exit = entry(&mut state);
                assert_eq!(munmap(page, bytes), 0);
                exit
            };
            let case = (target, fpsr);
            assert_eq!(
                exit,
                Exit {
                    pc: fallthrough.0,
                    kind: ExitKind::Fallthrough as u64,
                },
                "{case:?}"
            );
            assert_eq!(state.v[26], expected, "{case:?}");
            // Every lane is exact: no new status.
            assert_eq!(state.fpsr, fpsr, "{case:?}");
        }
    }
}

/// A block whose numerical slow paths are out of line, with runs that
/// check FPCR and FPSR once and inputs taken from XMM0 where the
/// previous instruction left them, runs on the host exactly as its
/// instructions do one at a time with the inline layout: in every
/// sticky-path state and outside it, through zero-product resumes and
/// helper exits in mid-run.
#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn out_of_line_slow_paths_run_as_the_inline_layout() {
    use crate::{GuestState, Lifter, aarch64::Decoder};
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
        fn mprotect(address: *mut c_void, bytes: usize, protection: i32) -> i32;
        fn munmap(address: *mut c_void, bytes: usize) -> i32;
    }
    #[cfg(target_os = "macos")]
    const MAP_ANONYMOUS: i32 = 0x1000;
    #[cfg(target_os = "linux")]
    const MAP_ANONYMOUS: i32 = 0x20;
    const PAGE: usize = 4096;
    #[repr(C)]
    #[derive(Debug, PartialEq, Eq)]
    struct Exit {
        pc: u64,
        kind: u64,
    }
    /// Guest memory at X1.
    type Memory = [u64; 16];
    fn run(code: &[u8], state: &mut GuestState, memory: &mut Memory) -> Exit {
        let bytes = code.len().next_multiple_of(PAGE);
        state.x[1] = memory.as_mut_ptr() as u64;
        // SAFETY: private anonymous memory, RX after the copy, unmapped
        // after the call; the block touches only `state`, `memory`
        // through X1 and its own stack slots.
        let exit = unsafe {
            let page = mmap(core::ptr::null_mut(), bytes, 3, 2 | MAP_ANONYMOUS, -1, 0);
            assert_ne!(page as usize, usize::MAX);
            core::ptr::copy_nonoverlapping(code.as_ptr(), page.cast(), code.len());
            assert_eq!(mprotect(page, bytes, 5), 0);
            let entry: unsafe extern "sysv64" fn(*mut GuestState) -> Exit =
                core::mem::transmute(page);
            let exit = entry(state);
            assert_eq!(munmap(page, bytes), 0);
            exit
        };
        state.x[1] = 0;
        exit
    }
    const WORDS: [u32; 45] = [
        0xbd40_0021, // ldr s1,[x1]
        0x1e21_0802, // fmul s2,s0,s1
        0x1e20_2843, // fadd s3,s2,s0
        0x1e20_08e4, // fmul s4,s7,s0
        0x1e24_3865, // fsub s5,s3,s4
        0xbd00_0425, // str s5,[x1,#4]
        0xfd40_0430, // ldr d16,[x1,#8]
        0x1f50_4a26, // fmadd d6,d17,d16,d18
        0x3d80_1826, // str q6,[x1,#96]
        0x1e61_c0d3, // fsqrt d19,d6
        0x3dc0_0435, // ldr q21,[x1,#16]
        0x4e36_d6b4, // fadd v20.4s,v21.4s,v22.4s
        0x6e79_df17, // fmul v23.2d,v24.2d,v25.2d
        0x9100_0400, // add x0,x0,#1
        0x1e21_08a8, // fmul s8,s5,s1
        0x1e20_1909, // fdiv s9,s8,s0
        0x1e71_8a0a, // fnmul d10,d16,d17
        0x4e36_ceba, // fmla v26.4s,v21.4s,v22.4s
        0x3d80_083a, // str q26,[x1,#32]
        0xfd00_182a, // str d10,[x1,#48]
        0x6f00_e41b, // movi v27.2d,#0
        0x0e35_d77c, // fadd v28.2s,v27.2s,v21.2s
        0x3d80_103c, // str q28,[x1,#64]
        0x1e3c_2b9e, // fadd s30,s28,s28
        0x3d80_143e, // str q30,[x1,#80]
        0x3dc0_042c, // ldr q12,[x1,#16]
        0x6e31_a98d, // uminv b13,v12.16b
        0x1e26_01a2, // fmov w2,s13
        0x6e70_a98e, // umaxv h14,v12.8h
        0x0e02_3dc3, // umov w3,v14.h[0]
        0x6e36_8d8f, // cmeq v15.16b,v12.16b,v22.16b
        0x4e01_2de4, // smov x4,v15.b[0]
        0x4eb6_85f0, // add v16.4s,v15.4s,v22.4s
        0x4e08_3e05, // umov x5,v16.d[0]
        0x3dc0_0431, // ldr q17,[x1,#16]
        0x0e01_3e26, // umov w6,v17.b[0]
        0x3dc0_0432, // ldr q18,[x1,#16]
        0x0e02_3e47, // umov w7,v18.h[0]
        0x3dc0_0433, // ldr q19,[x1,#16]
        0x9e66_0268, // fmov x8,d19
        0x3dc0_0435, // ldr q21,[x1,#16]
        0x6e36_8d94, // cmeq v20.16b,v12.16b,v22.16b
        0x1e26_0289, // fmov w9,s20
        0x4eb6_8697, // add v23.4s,v20.4s,v22.4s
        0x3d80_1c37, // str q23,[x1,#112]
    ];
    const IXC: u32 = 1 << 4;
    const NEP: u32 = 1 << 2;
    const ROUND_UP: u32 = 1 << 22;
    const ROUND_TO_ZERO: u32 = 3 << 22;
    const FZ: u32 = 1 << 24;
    const DN: u32 = 1 << 25;
    const UPPER_GARBAGE: u64 = 0xa5a5_5a5a_c3c3_3c3c;
    let pc = GuestAddress(0x1000);
    let instructions: Vec<_> = WORDS
        .iter()
        .enumerate()
        .map(|(index, &word)| {
            Decoder
                .lift(GuestAddress(pc.0 + 4 * index as u64), word)
                .unwrap()
        })
        .collect();
    let fallthrough = GuestAddress(pc.0 + 4 * WORDS.len() as u64);
    let single = |s: f32| u64::from(s.to_bits()) | UPPER_GARBAGE << 32;
    let lanes = |values: [f32; 4]| {
        let bits = values.map(|value| u64::from(value.to_bits()));
        [bits[0] | bits[1] << 32, bits[2] | bits[3] << 32]
    };
    let initial = |s7: f32, s1: f32, d18: f64| {
        let mut state = GuestState::default();
        for (index, vector) in state.v.iter_mut().enumerate() {
            *vector = [UPPER_GARBAGE.rotate_left(index as u32), UPPER_GARBAGE];
        }
        state.v[0][0] = single(1.1);
        state.v[7][0] = single(s7);
        for (index, value) in [(17, 2.7f64), (18, d18)] {
            state.v[index][0] = value.to_bits();
        }
        state.v[22] = lanes([0.7, 5.5, -1.9, 7e3]);
        state.v[24] = [1.7f64.to_bits(), (-0.3f64).to_bits()];
        state.v[25] = [2.9f64.to_bits(), 1e10f64.to_bits()];
        let mut memory: Memory = [UPPER_GARBAGE; 16];
        memory[0] = single(s1);
        memory[1] = 1.3f64.to_bits();
        [memory[2], memory[3]] = lanes([1.5, -2.25, 3.1, 1e-3]);
        (state, memory)
    };
    for target in [crate::TargetCpu::Baseline, crate::TargetCpu::detect()] {
        let block = emit_for_target(
            &instructions,
            None,
            fallthrough,
            target,
            super::super::MemoryChecks::DEFAULT,
        );
        // Normal values complete; an exact zero product resumes; a subnormal one takes the
        // helper mid-run, as does the root of a negative or an infinite
        // addend; an infinite input takes it at once.
        for (s7, s1, d18) in [
            (2.0, 3.3, 0.9),
            (0.0, 3.3, 0.9),
            (-0.0, 3.3, 0.9),
            (1e-40, 3.3, 0.9),
            (0.0, 3.3, -100.0),
            (0.0, 3.3, f64::INFINITY),
            (0.0, f32::INFINITY, 0.9),
        ] {
            for (fpcr, fpsr) in [
                (0, IXC),
                (0, 0),
                (ROUND_TO_ZERO, IXC),
                (ROUND_UP, 0),
                (NEP, IXC),
                (DN, IXC),
                (FZ, IXC),
            ] {
                let (mut state, mut memory) = initial(s7, s1, d18);
                state.fpcr = fpcr;
                state.fpsr = fpsr;
                let (mut expected, mut expected_memory) = (state.clone(), memory);
                let mut expected_exit = None;
                // The state and memory before each instruction.
                let mut before = vec![];
                for instruction in &instructions {
                    before.push((instruction.pc.0, expected.clone(), expected_memory));
                    let next = GuestAddress(instruction.pc.0 + 4);
                    let mut code = Code::new();
                    match &instruction.operation {
                        Operation::FloatingPoint(operand) => super::super::simd::emit_numerical(
                            &mut code,
                            operand,
                            instruction.pc,
                            target,
                        ),
                        _ => super::super::emit_instruction(
                            &mut code,
                            instruction,
                            &mut vec![],
                            target,
                            super::super::MemoryChecks::DEFAULT,
                        ),
                    }
                    direct_exit(&mut code, next, ExitKind::Fallthrough);
                    let exit = run(&code, &mut expected, &mut expected_memory);
                    if exit.pc != next.0 || exit.kind != ExitKind::Fallthrough as u64 {
                        expected_exit = Some(exit);
                        break;
                    }
                }
                let expected_exit = expected_exit.unwrap_or(Exit {
                    pc: fallthrough.0,
                    kind: ExitKind::Fallthrough as u64,
                });
                let exit = run(&block.x86_64_code, &mut state, &mut memory);
                let case = (target, s7, s1, d18, fpcr, fpsr);
                // With FPSR.IXC clear, the first full path of a run may
                // set it; the run's later instructions then take full
                // paths too, where one at a time they take sticky ones.
                // A full path sends a zero input to the helper, where
                // a sticky product resumes.
                if exit != expected_exit && fpcr == 0 && fpsr & IXC == 0 {
                    assert_eq!(exit.kind, ExitKind::FloatingPoint as u64, "{case:?}");
                    assert!(exit.pc < expected_exit.pc, "{case:?}");
                    let (_, at_exit, memory_at_exit) =
                        before.iter().find(|(pc, ..)| *pc == exit.pc).unwrap();
                    assert_eq!(&state, at_exit, "{case:?}");
                    assert_eq!(&memory, memory_at_exit, "{case:?}");
                    continue;
                }
                assert_eq!(exit, expected_exit, "{case:?}");
                assert_eq!(state, expected, "{case:?}");
                assert_eq!(memory, expected_memory, "{case:?}");
            }
        }
    }
}

#[test]
fn clock_exit_requests_runtime_service_without_counter_or_guest_writes() {
    use crate::block::{ClockInstruction, ClockOperation};
    for operation in [
        ClockOperation::InstructionSynchronization,
        ClockOperation::ReadVirtualCounter { destination: 0 },
        ClockOperation::ReadVirtualCounter { destination: 31 },
    ] {
        let pc = GuestAddress(0x123456789000);
        let request = ClockInstruction {
            operation,
            instruction_pc: pc,
            next_pc: GuestAddress(pc.0 + 4),
        };
        let native = emit(
            &[],
            Some(&(pc, Terminator::Clock(request))),
            request.next_pc,
        );
        let mut expected = vec![0x48, 0xb8];
        expected.extend(pc.0.to_le_bytes());
        expected.extend([
            0xba, 7, 0, 0, 0, 0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
        ]);
        // No RDTSC, host helper address, guessed counter, destination write
        // or placeholder ISB: only the checked runtime service may complete.
        assert_eq!(native.x86_64_code, expected);
        assert_eq!(native.source_map, [(pc, 0)]);
    }
}

#[test]
fn pac_helper_exit_has_no_guest_writes_or_embedded_service_address() {
    use crate::block::{PointerAuthentication, decode_pointer_authentication};
    let fixed_words = [
        0xd503211f, 0xd503215f, 0xd503219f, 0xd50321df, 0xd503231f, 0xd503233f, 0xd503235f,
        0xd503237f, 0xd503239f, 0xd50323bf, 0xd50323df, 0xd50323ff, 0xd50320ff, 0xd65f0bff,
        0xd65f0fff, 0xdac143f1, 0xdac147f1,
    ];
    let integer_words = (0..8).flat_map(|op| {
        [
            0xdac10000 | (op << 10) | (16 << 5) | 17,
            0xdac123e0 | (op << 10) | 17,
        ]
    });
    let words: Vec<_> = fixed_words.into_iter().chain(integer_words).collect();
    assert_eq!(words.len(), 33);
    for word in words {
        let operation = decode_pointer_authentication(GuestAddress(0x1000), word)
            .unwrap()
            .unwrap()
            .operation;
        let pc = GuestAddress(0x123456789000);
        let request = PointerAuthentication {
            operation,
            instruction_pc: pc,
            next_pc: GuestAddress(pc.0 + 4),
        };
        let native = emit(
            &[],
            Some(&(pc, Terminator::PointerAuthentication(request))),
            request.next_pc,
        );
        let mut expected = vec![0x48, 0xb8];
        expected.extend(pc.0.to_le_bytes());
        expected.extend([
            0xba, 6, 0, 0, 0, 0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
        ]);
        // Only the SysV return registers RAX/RDX are touched, including for
        // RETAB: the runtime must keep signed LR while authenticating target.
        assert_eq!(native.x86_64_code, expected);
        assert_eq!(native.source_map, [(pc, 0)]);
    }
}

#[test]
fn ordinary_native_prefix_is_preserved_before_pac_boundary() {
    use crate::{
        Lifter,
        aarch64::Decoder,
        block::{PointerAuthentication, PointerAuthenticationOperation},
    };
    let base = GuestAddress(0x4000);
    let instruction = Decoder.lift(base, 0xd2800020).unwrap();
    let pc = GuestAddress(base.0 + 4);
    let ordinary = emit(core::slice::from_ref(&instruction), None, pc);
    let request = PointerAuthentication {
        operation: PointerAuthenticationOperation::SignIbSp,
        instruction_pc: pc,
        next_pc: GuestAddress(pc.0 + 4),
    };
    let native = emit(
        &[instruction],
        Some(&(pc, Terminator::PointerAuthentication(request))),
        request.next_pc,
    );
    assert_eq!(
        &native.x86_64_code[..ordinary.x86_64_code.len() - DIRECT_EXIT_BYTES],
        &ordinary.x86_64_code[..ordinary.x86_64_code.len() - DIRECT_EXIT_BYTES]
    );
    assert_eq!(native.source_map.len(), 2);
    assert_eq!(native.source_map[1].0, pc);
    assert_eq!(
        &native.x86_64_code[native.x86_64_code.len() - DYNAMIC_EXIT_BYTES..],
        &[
            0xba, 6, 0, 0, 0, 0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc
        ]
    );
}

#[test]
fn trap_exit_uses_instruction_pc_and_only_native_return_scratch_registers() {
    for kind in [TrapKind::SupervisorCall, TrapKind::SoftwareBreakpoint] {
        let pc = GuestAddress(0x123456789000);
        let trap = SynchronousTrap {
            kind,
            immediate: 0x80,
            instruction_pc: pc,
            exception_return_pc: if kind == TrapKind::SupervisorCall {
                GuestAddress(pc.0 + 4)
            } else {
                pc
            },
        };
        let native = emit(
            &[],
            Some(&(pc, Terminator::Trap(trap))),
            GuestAddress(pc.0 + 4),
        );
        let mut expected = vec![0x48, 0xb8]; // movabs rax,instruction_pc
        expected.extend(pc.0.to_le_bytes());
        expected.extend([
            0xba, 5, 0, 0, 0, 0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc,
        ]); // mov edx,Trap; ret
        assert_eq!(native.x86_64_code, expected);
        assert_eq!(native.source_map, [(pc, 0)]);
    }
}

/// Region census of real code, run by hand:
/// `STELA_CENSUS=PATH:FILE_OFFSET:BYTES:BASE` names a raw image, the
/// executable range to sweep and the guest address of file offset 0
/// (all hexadecimal). It sweeps maximal straight-line runs the decoder
/// lifts, replays the region selection of [`emit_selected`] on each,
/// with its cost gates, and prints the summed allocator statistics, the
/// share of instructions each emission path takes, and the categories
/// of the instructions that end a region or run outside one.
/// `STELA_CENSUS_SYSTEM=1` replays system mode, whose accesses check
/// the guest range (`guest_range`), instead of the process profiles.
#[test]
#[ignore = "manual: opt-in"]
fn region_census_of_real_code() {
    use crate::Lifter;
    use std::collections::BTreeMap;
    let spec = std::env::var("STELA_CENSUS").expect("STELA_CENSUS=PATH:OFFSET:BYTES:BASE");
    let system = std::env::var_os("STELA_CENSUS_SYSTEM").is_some();
    let parts: Vec<&str> = spec.split(':').collect();
    let hex = |s: &str| u64::from_str_radix(s.trim_start_matches("0x"), 16).unwrap();
    let image = std::fs::read(parts[0]).unwrap();
    let (offset, bytes, base) = (hex(parts[1]), hex(parts[2]), hex(parts[3]));
    let text = &image[offset as usize..(offset + bytes) as usize];
    let category = |op: &Operation| {
        let name = std::format!("{op:?}");
        let end = name.find(['(', ' ', '{']).unwrap_or(name.len());
        std::string::String::from(&name[..end])
    };
    let mut stats = super::super::region::AllocationStats::default();
    let add = |total: &mut super::super::region::AllocationStats,
               s: &super::super::region::AllocationStats| {
        total.state_loads += s.state_loads;
        total.pressure_stores += s.pressure_stores;
        total.exit_stores += s.exit_stores;
        total.lifetime_stores += s.lifetime_stores;
        total.commits += s.commits;
        total.native_instructions += s.native_instructions;
        total.load_boundary_stores += s.load_boundary_stores;
        total.clean_loads += s.clean_loads;
        total.pair_loads += s.pair_loads;
        total.stores += s.stores;
        total.flag_loads += s.flag_loads;
        total.flag_stores += s.flag_stores;
    };
    let (mut allocated, mut clean, mut rejected, mut plain, mut runs) =
        (0usize, 0usize, 0usize, 0usize, 0usize);
    let (mut region_bytes, mut plain_bytes) = (0usize, 0usize);
    let mut outside: BTreeMap<std::string::String, usize> = BTreeMap::new();
    let mut breakers: BTreeMap<std::string::String, usize> = BTreeMap::new();
    let mut run: Vec<Instruction> = vec![];
    let mut flush = |run: &mut Vec<Instruction>| {
        if run.is_empty() {
            return;
        }
        runs += 1;
        let mut ranges = [false; MAX_BLOCK_INSTRUCTIONS];
        if system {
            super::super::guest_range::checks(run, &mut ranges[..run.len()]);
        }
        let checks_at = |index: usize| super::super::MemoryChecks {
            guest_range: ranges[index],
            ..super::super::MemoryChecks::DEFAULT
        };
        let mut cursor = 0;
        while cursor < run.len() {
            let rest = &run[cursor..];
            // Replays the system-mode selection with its cost gates: a
            // rejected chunk is emitted as baseline and skipped whole.
            let baseline = |count: usize| {
                let mut code = Code::new();
                let mut fixups = vec![];
                for (index, instruction) in rest[..count].iter().enumerate() {
                    super::super::emit_instruction(
                        &mut code,
                        instruction,
                        &mut fixups,
                        crate::TargetCpu::Baseline,
                        checks_at(cursor + index),
                    );
                }
                (code, fixups.is_empty())
            };
            let allocated_region = super::super::region::emit_allocated_prefix(rest)
                .ok()
                .flatten()
                .filter(|region| {
                    let (code, no_fixups) = baseline(region.consumed_instructions);
                    no_fixups && super::super::region::admits_allocated(region, &code)
                });
            let clean_region = || {
                super::super::region::emit_clean_load_prefix_with(
                    rest,
                    super::super::MemoryChecks::DEFAULT,
                    system,
                    system.then(|| &ranges[cursor..]),
                )
                .ok()
                .filter(|region| region.consumed_instructions != 0)
            };
            if let Some(region) = allocated_region {
                allocated += region.consumed_instructions;
                region_bytes += region.code.len();
                add(&mut stats, &region.stats);
                cursor += region.consumed_instructions;
            } else if let Some(region) = clean_region() {
                let (code, _) = baseline(region.consumed_instructions);
                if super::super::region::admits_ordinary(&region, &code) {
                    clean += region.consumed_instructions;
                    region_bytes += region.code.len();
                    add(&mut stats, &region.stats);
                } else {
                    rejected += region.consumed_instructions;
                    plain_bytes += code.len();
                }
                cursor += region.consumed_instructions;
            } else {
                let instruction = &run[cursor];
                let detail = match &instruction.operation {
                    Operation::Memory {
                        transfer,
                        bytes,
                        mode,
                        base,
                        ..
                    }
                    | Operation::MemoryPair {
                        transfer,
                        bytes,
                        mode,
                        base,
                        ..
                    } => std::format!(
                        "{} {transfer:?} {bytes} {mode:?} {}",
                        category(&instruction.operation),
                        if *base == Register::Sp { "sp" } else { "x" }
                    ),
                    operation => category(operation),
                };
                *outside.entry(detail).or_default() += 1;
                let mut code = Code::new();
                let mut fixups = vec![];
                if let Operation::FloatingPoint(operand) = &instruction.operation {
                    super::super::simd::emit_numerical(
                        &mut code,
                        operand,
                        instruction.pc,
                        crate::TargetCpu::Baseline,
                    );
                } else if super::super::validate_target_with(
                    instruction,
                    crate::TargetCpu::Baseline,
                    checks_at(cursor),
                )
                .is_ok()
                {
                    super::super::emit_instruction(
                        &mut code,
                        instruction,
                        &mut fixups,
                        crate::TargetCpu::Baseline,
                        checks_at(cursor),
                    );
                }
                plain_bytes += code.len();
                plain += 1;
                cursor += 1;
                continue;
            }
            if let Some(next) = run.get(cursor) {
                *breakers.entry(category(&next.operation)).or_default() += 1;
            }
        }
        run.clear();
    };
    for (index, word) in text.as_chunks::<4>().0.iter().enumerate() {
        let pc = GuestAddress(base + offset + 4 * index as u64);
        match crate::aarch64::Decoder.lift(pc, u32::from_le_bytes(*word)) {
            Ok(instruction)
                if !category(&instruction.operation).starts_with("Branch")
                    && category(&instruction.operation) != "Return" =>
            {
                run.push(instruction);
                if run.len() == crate::block::MAX_BLOCK_INSTRUCTIONS {
                    flush(&mut run);
                }
            }
            _ => flush(&mut run),
        }
    }
    flush(&mut run);
    let top = |map: &BTreeMap<std::string::String, usize>| {
        let mut v: Vec<_> = map.iter().collect();
        v.sort_by(|a, b| b.1.cmp(a.1));
        v.into_iter()
            .take(40)
            .map(|(k, n)| std::format!("{k} {n}"))
            .collect::<Vec<_>>()
    };
    std::println!(
        "runs {runs} allocated {allocated} clean {clean} rejected {rejected} plain {plain}"
    );
    std::println!("region bytes {region_bytes} plain bytes {plain_bytes}");
    std::println!("{stats:?}");
    std::println!("outside {:?}", top(&outside));
    std::println!("breakers {:?}", top(&breakers));
}

#[test]
fn frame_compare_and_test_branches_do_not_load_a_scratch_register() {
    use crate::aarch64::Width;
    let target = GuestAddress(0x2000);
    for (operation, expected, condition) in [
        (
            Operation::BranchZero {
                target,
                source: Register::X(20),
                width: Width::W32,
                nonzero: false,
            },
            &[0x83, 0xbf, 160, 0, 0, 0, 0][..],
            Cond::E,
        ),
        (
            Operation::BranchZero {
                target,
                source: Register::X(20),
                width: Width::X64,
                nonzero: true,
            },
            &[0x48, 0x83, 0xbf, 160, 0, 0, 0, 0][..],
            Cond::Ne,
        ),
        (
            Operation::BranchBit {
                target,
                source: Register::X(20),
                bit: 63,
                nonzero: true,
            },
            &[0xf6, 0x87, 167, 0, 0, 0, 0x80][..],
            Cond::Ne,
        ),
    ] {
        let mut code = Code::pinned();
        assert_eq!(
            branch_condition(&mut code, &operation, super::super::Rax::Unknown),
            Some(condition)
        );
        assert_eq!(&code[..], expected);
        assert_eq!(code.instructions(), 1);
    }
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn pinned_select_consumes_the_preceding_compare_flags() {
    use crate::{
        GuestState,
        block::{BlockProfile, Features},
        x86_64::{native, pinned},
    };
    // CMP X0, #1; CSEL X1, X2, X3, HI; RET.
    let source: Vec<_> = [0xf100_041fu32, 0x9a83_8041, 0xd65f_03c0]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let code = crate::testing::block_with(
        &source,
        GuestAddress(0x1000),
        BlockProfile::new(Features::SYSTEM_MODE),
    )
    .unwrap()
    .native;
    let select = &code.x86_64_code[code.source_map[1].1..code.source_map[2].1];
    // The compare's flags stay in the host's, whose borrow HI reads as it
    // is: no CMC before the select.
    assert_ne!(select[0], 0xf5, "{select:02x?}");
    let entry = pinned::entered(&code.x86_64_code);
    for (operand, selected, nzcv) in [
        (0, 33, 0x8000_0000),
        (1, 33, 0x6000_0000),
        (2, 22, 0x2000_0000),
    ] {
        let mut state = GuestState::default();
        state.x[0] = operand;
        state.x[2] = 22;
        state.x[3] = 33;
        state.x[30] = 0x2000;
        assert_eq!(native::run(&entry, &mut state), 0x2000);
        assert_eq!(state.x[1], selected);
        assert_eq!(state.nzcv, nzcv);
    }
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn pinned_conditional_increment_consumes_the_preceding_compare_flags() {
    use crate::{
        GuestState,
        block::{BlockProfile, Features},
        x86_64::{native, pinned},
    };
    // CMP X0, #1; CSET W2, EQ; RET.
    let cset = 0x1a9f_17e2;
    // CMP X0, #1; CSINC W8, W2, W8, HI; RET.
    let csinc = 0x1a88_8448;
    for (select, prefix) in [
        (cset, &[0x0f, 0x94, 0xc0][..]), // SETE AL
        (
            csinc,
            &[0x8b, 0xd6, 0x8d, 0x52, 0x01, 0x8b, 0x77, 0x10, 0x0f, 0x46],
        ), // MOV EDX, LEA, MOV ESI, CMOVBE
    ] {
        let source: Vec<_> = [0xf100_041fu32, select, 0xd65f_03c0]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let code = crate::testing::block_with(
            &source,
            GuestAddress(0x1000),
            BlockProfile::new(Features::SYSTEM_MODE),
        )
        .unwrap()
        .native;
        let selected = &code.x86_64_code[code.source_map[1].1..code.source_map[2].1];
        assert!(selected.starts_with(prefix), "{selected:02x?}");
        let entry = pinned::entered(&code.x86_64_code);
        for (compared, equal, higher, nzcv) in [
            (0, 0, false, 0x8000_0000),
            (1, 1, false, 0x6000_0000),
            (2, 0, true, 0x2000_0000),
        ] {
            let mut state = GuestState::default();
            state.x[0] = compared;
            state.x[2] = 0xaabb_ccdd_0000_0016;
            state.x[8] = u64::MAX;
            state.x[30] = 0x2000;
            let mut expected = state.x;
            if select == cset {
                expected[2] = equal;
            } else {
                expected[8] = if higher { 22 } else { 0 };
            }
            assert_eq!(native::run(&entry, &mut state), 0x2000);
            assert_eq!(state.x, expected);
            assert_eq!(state.nzcv, nzcv);
        }
    }
}

#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn pinned_conditional_compare_consumes_the_preceding_flags() {
    use crate::{
        GuestState, Lifter, TargetCpu,
        aarch64::{Decoder, conditional_select::decode_compare},
        x86_64::{native, pinned},
    };

    let pc = GuestAddress(0x1000);
    let compare_pc = GuestAddress(pc.0 + 4);
    let end = GuestAddress(pc.0 + 8);
    for wide in [0, 1] {
        for producer_subtract in [0, 1] {
            // CMP or CMN of X0/W0 and X4/W4.
            let producer = 0x2b04_001f | wide << 31 | producer_subtract << 30;
            for subtract in [0, 1] {
                for right in [2 << 16, (1 << 11) | (31 << 16)] {
                    for condition in 0..16 {
                        // CCMP or CCMN of X1/W1 and X2/W2 or immediate 31.
                        let compare =
                            0x3a40_0025 | wide << 31 | subtract << 30 | right | condition << 12;
                        let instructions = [
                            Decoder.lift(pc, producer).unwrap(),
                            Decoder.lift(compare_pc, compare).unwrap(),
                        ];
                        let (system, _) = emit_system_mode(
                            &instructions,
                            None,
                            end,
                            TargetCpu::Baseline,
                            super::super::MemoryChecks::DEFAULT,
                            super::super::NativeSystem::NONE,
                        );
                        assert_eq!(system.source_map[0], (pc, 0));
                        let (at, offset) = system.source_map[1];
                        assert_eq!(at, compare_pc);
                        let (single, _) = emit_system_mode(
                            &instructions[1..],
                            None,
                            end,
                            TargetCpu::Baseline,
                            super::super::MemoryChecks::DEFAULT,
                            super::super::NativeSystem::NONE,
                        );
                        let bytes = &system.x86_64_code[offset..];
                        if condition < 14 {
                            assert!(bytes.len() < single.x86_64_code.len(), "{compare:08x}");
                            let predicate_at = usize::from(matches!(condition, 8 | 9));
                            assert_eq!(bytes[predicate_at..predicate_at + 2], [0x41, 0x0f]);
                        } else {
                            assert_eq!(bytes, single.x86_64_code);
                        }

                        let dit = TargetCpu::Baseline.with_data_independent_timing();
                        let (system_dit, _) = emit_system_mode(
                            &instructions,
                            None,
                            end,
                            dit,
                            super::super::MemoryChecks::DEFAULT,
                            super::super::NativeSystem::NONE,
                        );
                        let mut baseline = Code::pinned();
                        baseline.set_constant_time(true);
                        super::super::conditional_select::emit_compare(
                            &mut baseline,
                            &decode_compare(compare).unwrap(),
                            false,
                        );
                        assert!(
                            system_dit.x86_64_code[system_dit.source_map[1].1..]
                                .starts_with(&baseline)
                        );

                        let frame = emit(&instructions, None, end);
                        let entered = pinned::entered(&system.x86_64_code);
                        for (left, right) in [
                            (0, 0),
                            (1, 0),
                            (0, 1),
                            (1 << 31, 1),
                            (1 << 63, 1),
                            (u64::MAX, 1),
                            (u64::MAX, u64::MAX),
                        ] {
                            let mut initial = GuestState::default();
                            initial.x[0] = left;
                            initial.x[1] = left ^ 1;
                            initial.x[2] = right;
                            initial.x[4] = right;
                            initial.nzcv = 0xf000_0000;
                            let mut expected = initial.clone();
                            assert_eq!(native::run(&frame.x86_64_code, &mut expected), end.0);
                            assert_eq!(native::run(&entered, &mut initial), end.0);
                            assert_eq!(initial, expected, "{producer:08x} {compare:08x}");
                        }
                    }
                }
            }
        }
    }
}

/// Under FEAT_CSV3 masking, accesses that reuse an earlier access's masked
/// address (`masked_base`) through writebacks, moved copies of the base,
/// SIMD accesses and a side exit run as the block that leaves those checks
/// to `guest_range` does.
#[cfg(all(target_arch = "x86_64", any(target_os = "linux", target_os = "macos")))]
#[test]
fn masked_base_reuse_matches_the_unmasked_block() {
    use crate::{
        Options, TargetCpu,
        block::{BlockProfile, CompilationPlan, Features},
        context::ExecutionContext,
        x86_64::{MemoryChecks, asm::vector_extensions::execution::Native, pinned::entered},
    };
    let words = [
        0xa9be_7bfdu32, // STP X29,X30,[SP,#-32]!
        0x9100_03fd,    // MOV X29,SP
        0xf900_0be0,    // STR X0,[SP,#16]
        0xf940_0ba1,    // LDR X1,[X29,#16]
        0x9100_23a2,    // ADD X2,X29,#8
        0xf900_0041,    // STR X1,[X2]
        0xa8c2_13e3,    // LDP X3,X4,[SP],#32
        0xf85f_03e5,    // LDUR X5,[SP,#-16]
        0x1100_04c6,    // ADD W6,W6,#1
        0x3cde_0fe1,    // LDR Q1,[SP,#-32]!
        0x3d80_07e1,    // STR Q1,[SP,#16]
        0xb400_0049,    // CBZ X9,#8
        0xf940_0bea,    // LDR X10,[SP,#16]
        0xa901_17aa,    // STP X10,X5,[X29,#16]
        0xf85f_8c4b,    // LDR X11,[X2,#-8]!
        0xf840_844c,    // LDR X12,[X2],#8
    ];
    let compile = |every_access_masked| {
        let source: Vec<_> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        let options = Options::new(
            BlockProfile::new(Features::SYSTEM_MODE),
            TargetCpu::Baseline,
        )
        .with_memory_checks(MemoryChecks::system(true, false, true, every_access_masked));
        CompilationPlan::new(&source, GuestAddress(0x1000), &options)
            .unwrap()
            .compile()
            .unwrap()
            .native
            .x86_64_code
    };
    let masked = compile(true);
    let unmasked = compile(false);
    const KEEP_R11: [u8; 3] = [0x49, 0x89, 0xc3]; // mov r11,rax
    assert!(masked.windows(3).any(|bytes| bytes == KEEP_R11));
    let masked = Native::new(&entered(&masked));
    let unmasked = Native::new(&entered(&unmasked));
    let mut memory = [0u128; 32];
    let stack = memory.as_mut_ptr() as u64 + 256;
    for x9 in [0, 0x99] {
        let mut original = crate::GuestState {
            sp: stack,
            ..Default::default()
        };
        for (index, register) in original.x.iter_mut().enumerate() {
            *register = 0x0123_4567_89ab_cdefu64.wrapping_mul(index as u64 + 3);
        }
        original.x[9] = x9;
        let mut runs = Vec::new();
        for native in [&masked, &unmasked] {
            for (index, slot) in memory.iter_mut().enumerate() {
                *slot = 0x1111_2222_3333_4444_5555_6666_7777_8888u128.wrapping_mul(index as u128);
            }
            let mut state = original.clone();
            native.run(&mut state, &mut ExecutionContext::default());
            runs.push((state, memory));
        }
        assert_eq!(runs[0], runs[1], "X9 = {x9:#x}");
        assert_eq!(runs[0].0.sp, stack - 32);
    }
}

/// A flag setter whose NZCV the next one overwrites before any read, fault
/// or exit keeps no capture in system mode, nor does one whose readers take
/// it from the host flags before that; every other one keeps its own.
#[test]
fn dead_nzcv_captures_are_dropped_in_system_mode() {
    use crate::{Lifter, TargetCpu, aarch64::Decoder};
    const CAPTURE: [u8; 4] = [0x9f, 0x0f, 0x90, 0xc0]; // lahf; seto al
    const ADDS_X0: u32 = 0xb100_0400;
    const ADDS_X1: u32 = 0xb100_0421;
    const SUBS_X2: u32 = 0xf100_0c42;
    const ADD_X3: u32 = 0x9100_1463;
    const MOVZ_X4: u32 = 0xd280_00e4;
    const CSEL_X7: u32 = 0x9a81_1007;
    const LDR_X9: u32 = 0xf940_0149;
    const TST_X0: u32 = 0xf240_001f;
    for (words, captures) in [
        (&[ADDS_X0, ADDS_X1][..], 1),
        (&[ADDS_X0, ADD_X3, MOVZ_X4, SUBS_X2], 1),
        (&[ADDS_X0, ADDS_X1, ADD_X3], 1),
        (&[ADDS_X0, TST_X0, SUBS_X2, ADDS_X1], 1),
        (&[ADDS_X0, CSEL_X7, ADDS_X1], 1),
        (&[ADDS_X0, LDR_X9, ADDS_X1], 2),
        (&[ADDS_X0], 1),
    ] {
        let instructions: Vec<_> = words
            .iter()
            .enumerate()
            .map(|(index, &word)| {
                Decoder
                    .lift(GuestAddress(0x1000 + 4 * index as u64), word)
                    .unwrap()
            })
            .collect();
        let end = GuestAddress(0x1000 + 4 * words.len() as u64);
        let (system, _) = emit_system_mode(
            &instructions,
            None,
            end,
            TargetCpu::Baseline,
            super::super::MemoryChecks::DEFAULT,
            super::super::NativeSystem::NONE,
        );
        let code = &system.x86_64_code;
        let found = code
            .windows(CAPTURE.len())
            .filter(|w| *w == CAPTURE)
            .count();
        assert_eq!(found, captures, "{words:08x?}");
        // Every instruction keeps a native range of its own.
        for pair in system.source_map.windows(2) {
            assert!(pair[0].1 < pair[1].1, "{words:08x?}");
        }
    }
}

/// System-mode blocks whose dead captures went leave the state the frame
/// emitter's instruction-by-instruction code leaves, NZCV included.
#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn dead_nzcv_captures_leave_the_frame_emitters_state() {
    use crate::{
        GuestState, Lifter, TargetCpu,
        aarch64::Decoder,
        x86_64::{native, pinned},
    };
    const WORDS: [u32; 10] = [
        0xb100_0400, // adds x0,x0,#1
        0xb100_0421, // adds x1,x1,#1
        0xf100_0c42, // subs x2,x2,#3
        0xeb01_001f, // cmp x0,x1
        0x9100_1463, // add x3,x3,#5
        0xd280_00e4, // movz x4,#7
        0xea06_00a5, // ands x5,x5,x6
        0x9a81_1007, // csel x7,x0,x1,ne
        0xba01_0008, // adcs x8,x0,x1
        0xf240_001f, // tst x0,#1
    ];
    let inputs = [
        0,
        1,
        2,
        0x7fff_ffff_ffff_ffff,
        u64::MAX,
        0x8000_0000_0000_0000,
    ];
    let mut cases = 0;
    // Every sequence of three words, and some longer ones.
    let mut sequences: Vec<Vec<u32>> = Vec::new();
    for a in WORDS {
        for b in WORDS {
            for c in WORDS {
                sequences.push(vec![a, b, c]);
            }
        }
    }
    sequences.push(WORDS.to_vec());
    sequences.push(WORDS.iter().rev().copied().collect());
    for words in sequences {
        let instructions: Vec<_> = words
            .iter()
            .enumerate()
            .map(|(index, &word)| {
                Decoder
                    .lift(GuestAddress(0x1000 + 4 * index as u64), word)
                    .unwrap()
            })
            .collect();
        let end = GuestAddress(0x1000 + 4 * words.len() as u64);
        let (system, _) = emit_system_mode(
            &instructions,
            None,
            end,
            TargetCpu::Baseline,
            super::super::MemoryChecks::DEFAULT,
            super::super::NativeSystem::NONE,
        );
        let system = pinned::entered(&system.x86_64_code);
        let frame = emit_for_target(
            &instructions,
            None,
            end,
            TargetCpu::Baseline,
            super::super::MemoryChecks::DEFAULT,
        )
        .x86_64_code;
        for (index, &value) in inputs.iter().enumerate() {
            let mut expected = GuestState {
                nzcv: (index as u32 % 16) << 28,
                ..GuestState::default()
            };
            for (register, x) in expected.x.iter_mut().enumerate() {
                *x = value.wrapping_add(register as u64 * 0x1111);
            }
            let mut actual = expected.clone();
            assert_eq!(native::run(&frame, &mut expected), end.0);
            assert_eq!(native::run(&system, &mut actual), end.0);
            assert_eq!(actual, expected, "{words:08x?} {value:x}");
            cases += 1;
        }
    }
    assert!(cases > 6000);
}
