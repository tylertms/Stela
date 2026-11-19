use super::*;

#[test]
fn simd_and_fp_decoders_claim_only_words_with_bit_26() {
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    for _ in 0..200_000 {
        state = state.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
        let word = (state >> 32) as u32 & !SIMD_FP;
        assert!(simd::decode(word).is_none(), "{word:#010x}");
        assert!(fp_register::decode(word).is_none(), "{word:#010x}");
        assert!(
            FloatingPointOperation::decode(word).is_none(),
            "{word:#010x}"
        );
    }
}

/// Whether a decoder [`Decoder::lift`] tries before [`lift_integer`] takes
/// `word`.
fn taken_before_integer(pc: GuestAddress, word: u32) -> bool {
    let simd_fp = word & SIMD_FP != 0;
    word == 0xd65f03c0
        || word == 0xd503201f
        || system::synchronization::decode(word).is_some()
        || exclusive::decode(word).is_some()
        || constrained::decode(word).is_some()
        || pc_relative::decode(pc, word).is_some()
        || bitfield::decode(word).is_some()
        || conditional_select::decode(word).is_some()
        || conditional_select::decode_compare(word).is_some()
        || crc::decode(word).is_some()
        || checked_pointer::decode(word).is_some()
        || flags::decode(word).is_some()
        || (simd_fp && fp_register::decode(word).is_some())
        || integer_unary::decode(word).is_some()
        || integer_minmax::decode(word).is_some()
        || tag_arithmetic::decode(word).is_some()
        || integer_math::decode(word).is_some()
        || (simd_fp && FloatingPointOperation::decode(word).is_some())
        || ordered::decode(word).is_some()
        || atomic::decode(word).is_some()
        || non_temporal::decode(word).is_some()
        || unprivileged::decode(word).is_some()
        || prefetch::decode(pc, word).is_some()
        || literal::decode(pc, word).is_some()
        || (simd_fp && simd::decode(word).is_some())
        || register_offset::decode(word).is_some()
}

/// The words [`skips_decoders`] sends to [`lift_integer`] directly are
/// ones no earlier decoder takes, so the lift is the one it would make in
/// order. Every word of these classes was checked once; this samples them.
#[test]
fn direct_classes_skip_no_decoder() {
    let classes = DIRECT.into_iter().chain([
        (0x3f00_0000, 0x3900_0000),
        (0x3e00_0000, 0x2800_0000),
        (0x3f20_0000, 0x3800_0000),
    ]);
    let pc = GuestAddress(0x1000);
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    for (mask, class) in classes {
        let mut direct = 0;
        for _ in 0..100_000 {
            state = state.wrapping_mul(0x5851_f42d_4c95_7f2d).wrapping_add(1);
            let word = (state >> 32) as u32 & !mask | class;
            if skips_decoders(pc, word) {
                direct += 1;
                assert!(!taken_before_integer(pc, word), "{word:#010x}");
            }
        }
        assert!(direct > 50_000, "{mask:#010x} {class:#010x}");
    }
}

#[test]
fn synchronization_admits_only_context_independent_effects() {
    const YIELD_WORD: u32 = 0xd503_203f;
    const CSDB_WORD: u32 = 0xd503_229f;
    const SB_WORD: u32 = 0xd503_30ff;
    const NOP_WORD: u32 = 0xd503_201f;
    const CRM_SHIFT: u32 = 8;
    const CRM_VALUES: u32 = 16;
    let pc = GuestAddress(0x1000);
    for word in [YIELD_WORD, CSDB_WORD] {
        assert!(matches!(
            Decoder.lift(pc, word).unwrap().operation,
            Operation::Synchronization(_)
        ));
    }
    for crm in 0..CRM_VALUES {
        assert_eq!(
            Decoder
                .lift(pc, SB_WORD | crm << CRM_SHIFT)
                .unwrap()
                .operation,
            Operation::Synchronization(system::synchronization::Operation::SpeculationBarrier)
        );
    }
    assert_eq!(
        Decoder.lift(pc, NOP_WORD).unwrap().operation,
        Operation::Nop
    );
    // Existing PAuth block dispatch owns its hints before the ordinary decoder.
    for (mnemonic, word) in [
        ("PACIASP", 0xd503_233f),
        ("PACIBSP", 0xd503_237f),
        ("DSB", 0xd503_3f9f),
        ("ISB", 0xd503_3fdf),
        ("WFE", 0xd503_205f),
        ("WFI", 0xd503_207f),
        ("SEV", 0xd503_209f),
        ("SEVL", 0xd503_20bf),
        ("BTI", 0xd503_241f),
        ("ESB", 0xd503_221f),
        ("TSB", 0xd503_225f),
        ("reserved HINT", 0xd503_301f),
        ("SSBB", 0xd503_309f),
        ("PSSBB", 0xd503_349f),
    ] {
        assert!(
            Decoder.lift(pc, word).is_err(),
            "unexpected admission: {mnemonic} {word:#x}"
        );
    }
}

#[test]
fn consistent_branches_preserve_conditional_branch_operand_semantics() {
    const CONSISTENT_BRANCH_HINT: u32 = 1 << 4;
    const BRANCH_IMMEDIATE_MASK: u32 = (1 << 19) - 1;
    const BRANCH_IMMEDIATE_SHIFT: u32 = 5;
    const INSTRUCTION_BYTES: i32 = 4;
    let pc = GuestAddress(0x20_0000);
    for offset in [-(1 << 18), -3, -1, 0, 1, 3, (1 << 18) - 1] {
        let immediate = ((offset as u32) & BRANCH_IMMEDIATE_MASK) << BRANCH_IMMEDIATE_SHIFT;
        for condition in 0..16 {
            let word = CONDITIONAL_BRANCH_CLASS | immediate | condition;
            let expected = Operation::BranchCondition {
                target: GuestAddress(
                    pc.0.checked_add_signed(i64::from(offset * INSTRUCTION_BYTES))
                        .unwrap(),
                ),
                condition: condition as u8,
            };
            for hint in [0, CONSISTENT_BRANCH_HINT] {
                assert_eq!(Decoder.lift(pc, word | hint).unwrap().operation, expected);
            }
        }
    }
}
#[test]
fn extended_arithmetic_validates_options_shifts_and_sp_zero_roles() {
    let extensions = [
        RegisterExtend::Uxtb,
        RegisterExtend::Uxth,
        RegisterExtend::Uxtw,
        RegisterExtend::Uxtx,
        RegisterExtend::Sxtb,
        RegisterExtend::Sxth,
        RegisterExtend::Sxtw,
        RegisterExtend::Sxtx,
    ];
    for width in [Width::W32, Width::X64] {
        for subtract in [false, true] {
            for flags in [false, true] {
                for (option, extend) in extensions.into_iter().enumerate() {
                    for amount in 0..8 {
                        let word = (u32::from(width == Width::X64) << 31)
                            | (u32::from(subtract) << 30)
                            | (u32::from(flags) << 29)
                            | 0x0b3f03ff
                            | ((option as u32) << 13)
                            | (amount << 10);
                        let decoded = Decoder.lift(GuestAddress(0), word);
                        if amount > 4 {
                            assert!(decoded.is_err());
                            continue;
                        }
                        assert_eq!(
                            decoded.unwrap().operation,
                            Operation::AddSubExtended {
                                width,
                                subtract,
                                flags,
                                destination: if flags { Register::Zero } else { Register::Sp },
                                left: Register::Sp,
                                right: Register::Zero,
                                extend,
                                amount: amount as u8,
                            }
                        );
                        for reserved in [1 << 22, 1 << 23] {
                            assert!(Decoder.lift(GuestAddress(0), word | reserved).is_err());
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn carry_operands_use_zero_and_require_the_exact_instruction_encoding() {
    for width in [Width::W32, Width::X64] {
        for subtract in [false, true] {
            for flags in [false, true] {
                let word = (u32::from(width == Width::X64) << 31)
                    | (u32::from(subtract) << 30)
                    | (u32::from(flags) << 29)
                    | 0x1a1f03ff;
                assert_eq!(
                    Decoder.lift(GuestAddress(0), word).unwrap().operation,
                    Operation::AddSubCarry {
                        width,
                        subtract,
                        flags,
                        destination: Register::Zero,
                        left: Register::Zero,
                        right: Register::Zero,
                    }
                );
                // Bits 15:10 are fixed in the carry family. Neighboring
                // encodings may belong to other allocated families, such as CPA.
                for reserved in 1..64 {
                    let neighbor = word | (reserved << 10);
                    assert!(
                        !matches!(
                            Decoder
                                .lift(GuestAddress(0), neighbor)
                                .map(|instruction| instruction.operation),
                            Ok(Operation::AddSubCarry { .. })
                        ),
                        "neighbor {neighbor:#010x} must not decode as carry"
                    );
                }
            }
        }
    }
    for word in [
        0xd500401f, // CFINV
        0xba008400, // RMIF
        0x3a00080d, // SETF8
        0x3a00480d, // SETF16
    ] {
        assert!(
            matches!(
                Decoder.lift(GuestAddress(0), word).unwrap().operation,
                Operation::Flags(_)
            ),
            "{word:#x}"
        );
    }
}

#[test]
fn logical_immediate_masks_match_all_rotated_runs_and_reserved_encodings() {
    for (width, bits) in [(Width::W32, 32_i32), (Width::X64, 64)] {
        let mut expected = alloc::collections::BTreeMap::new();
        for element_bits in [2, 4, 8, 16, 32, 64] {
            if element_bits > bits {
                continue;
            }
            for ones in 1..element_bits {
                for rotation in 0..64 {
                    // Construct the reference bit-by-bit instead of using
                    // the decoder's shift/rotate/replication arithmetic.
                    let mut mask = 0u64;
                    for position in 0..bits {
                        if (position + rotation) % element_bits < ones {
                            mask |= 1 << position;
                        }
                    }
                    let n = u32::from(element_bits == 64);
                    let size = ((-2 * element_bits) | (ones - 1)) as u32 & 63;
                    expected.insert((n, rotation as u32, size), mask);
                }
            }
        }
        for n in 0..=1 {
            for rotation in 0..64 {
                for size in 0..64 {
                    let key = (n, rotation, size);
                    assert_eq!(
                        logical_mask(width, n, rotation, size),
                        expected.get(&key).copied(),
                        "{width:?} N={n} immr={rotation} imms={size}"
                    );
                    for opcode in 0..4 {
                        let word = (u32::from(width == Width::X64) << 31)
                            | (opcode << 29)
                            | 0x12000000
                            | (n << 22)
                            | (rotation << 16)
                            | (size << 10);
                        assert_eq!(
                            Decoder.lift(GuestAddress(0), word).is_ok(),
                            expected.contains_key(&key),
                            "word {word:#x}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn logical_immediate_distinguishes_sp_destination_and_zero_source() {
    for (word, destination, flags) in [
        (0x92401fff, Register::Sp, false),  // and sp,xzr,#0xff
        (0x32001fff, Register::Sp, false),  // orr wsp,wzr,#0xff
        (0xd2401fff, Register::Sp, false),  // eor sp,xzr,#0xff
        (0xf2401fff, Register::Zero, true), // tst xzr,#0xff
    ] {
        assert!(matches!(
            Decoder.lift(GuestAddress(0), word).unwrap().operation,
            Operation::LogicalImmediate {
                destination: actual_destination,
                source: Register::Zero,
                flags: actual_flags,
                immediate: 255,
                ..
            } if actual_destination == destination && actual_flags == flags
        ));
    }
}

#[test]
fn constrained_memory_operands_are_undefined_before_ordinary_decoders() {
    use constrained::ConstrainedUnpredictable::{
        LoadWritebackOverlap, PairLoadDestinationOverlap, StoreWritebackOverlap,
    };
    let pc = GuestAddress(0x20_0000);
    for (word, reason) in [
        (0xa8400000, PairLoadDestinationOverlap), // LDNP x0,x0,[x0]
        (0xa8407fff, PairLoadDestinationOverlap), // LDNP xzr,xzr,[sp]
        (0x6d400000, PairLoadDestinationOverlap), // LDP d0,d0,[x0]
        (0xa9400000, PairLoadDestinationOverlap), // LDP x0,x0,[x0]
        (0xa9407fff, PairLoadDestinationOverlap), // LDP xzr,xzr,[sp]
        (0xa8c00000, LoadWritebackOverlap),       // WB overlap precedes duplicate destinations
        (0xa8c10400, LoadWritebackOverlap),       // LDP post-index overlaps first destination
        (0xa9c10420, LoadWritebackOverlap),       // LDP pre-index overlaps second destination
        (0xa8810400, StoreWritebackOverlap),      // STP post-index overlaps first source
        (0xa9810420, StoreWritebackOverlap),      // STP pre-index overlaps second source
        (0xf8400c00, LoadWritebackOverlap),       // pre-index LDR base/target overlap
        (0xf8400400, LoadWritebackOverlap),       // post-index LDR base/target overlap
        (0xf8000c00, StoreWritebackOverlap),      // pre-index STR base/target overlap
        (0xf8000400, StoreWritebackOverlap),      // post-index STR base/target overlap
    ] {
        assert_eq!(constrained::decode(word), Some(reason), "{word:#x}");
        assert!(Decoder.lift(pc, word).is_err(), "{word:#x}");
    }
}

#[test]
fn pair_reserved_neighbors_and_ordinary_register_roles_are_preserved() {
    for word in [
        0xe9400400, // unallocated integer pair opcode
        0x69000400, // STGP allocation-tag store must not become plain STP
    ] {
        assert_eq!(constrained::decode(word), None);
        assert!(Decoder.lift(GuestAddress(0), word).is_err(), "{word:#x}");
    }
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0xa9400400).unwrap().operation,
        Operation::MemoryPair {
            transfer: MemoryTransfer::Load { .. },
            ..
        }
    )); // offset LDP can overwrite base
    for word in [0xa8400400, 0xa8000400] {
        assert!(matches!(
            Decoder.lift(GuestAddress(0), word).unwrap().operation,
            Operation::MemoryNonTemporal(_)
        ));
    }
    for word in [0xa9000000, 0xa9bf7fff] {
        assert_eq!(constrained::decode(word), None);
        assert!(matches!(
            Decoder.lift(GuestAddress(0), word).unwrap().operation,
            Operation::MemoryPair {
                transfer: MemoryTransfer::Store,
                ..
            }
        ));
    } // Duplicate store sources are valid; SP and ZR are independent roles.
    assert_eq!(constrained::decode(0x6cc00400), None);
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0x6cc00400).unwrap().operation,
        Operation::Simd(simd::Operand::MemoryPair { load: true, .. })
    )); // LDP d0,d1,[x0],#0 has independent vector and base banks.
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0xad0183e0).unwrap().operation,
        Operation::Simd(simd::Operand::MemoryPair { load: false, .. })
    )); // original Xcode STP q0,q0,[sp,#48]
}

#[test]
fn memory_reserved_neighbors_and_ordinary_writeback_roles_are_preserved() {
    for word in [
        0xb9c00000, // unallocated signed word -> W32
        0xf9c00000, // unallocated signed doubleword
        0x3cc00800, // SIMD unprivileged/reserved mode
    ] {
        assert!(Decoder.lift(GuestAddress(0), word).is_err(), "{word:#x}");
    }
    for word in [0xf9800000, 0xf8800000] {
        assert!(matches!(
            Decoder.lift(GuestAddress(0), word).unwrap().operation,
            Operation::Prefetch(_)
        ));
    }
    for word in [0xf8400800, 0xf8000800] {
        assert!(matches!(
            Decoder.lift(GuestAddress(0), word).unwrap().operation,
            Operation::MemoryUnprivileged(_)
        ));
    }
    // The same register as base and destination is defined without writeback.
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0xf9400000).unwrap().operation,
        Operation::Memory { .. }
    ));
    // Rt=31 is ZR, Rn=31 is SP; they do not overlap.
    assert_eq!(constrained::decode(0xf8400fff), None);
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0xf8400fff).unwrap().operation,
        Operation::Memory {
            data: Register::Zero,
            base: Register::Sp,
            ..
        }
    ));
    // SIMD Rt0 is V0, independent of the X0 address/writeback register.
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0x3c400c00).unwrap().operation,
        Operation::Simd(simd::Operand::Memory { .. })
    ));
}

#[test]
fn sp_and_zero_are_decoded_by_instruction_role() {
    let add = Decoder.lift(GuestAddress(0), 0x910003ff).unwrap(); // add sp,sp,#0
    assert!(matches!(
        add.operation,
        Operation::AddSub {
            destination: Register::Sp,
            source: Register::Sp,
            ..
        }
    ));
    let cmp = Decoder.lift(GuestAddress(0), 0xf10003ff).unwrap(); // subs xzr,sp,#0
    assert!(matches!(
        cmp.operation,
        Operation::AddSub {
            destination: Register::Zero,
            source: Register::Sp,
            flags: true,
            ..
        }
    ));
    assert!(Decoder.lift(GuestAddress(0), 0x52c00000).is_err()); // undefined W32 MOVZ shift 32
    // ADDG occupies a distinct class; its SP roles must not become ADD-immediate.
    assert!(matches!(
        Decoder.lift(GuestAddress(0), 0x918003ff).unwrap().operation,
        Operation::TagArithmetic(tag_arithmetic::Operand {
            destination: Register::Sp,
            kind: tag_arithmetic::Kind::Offset {
                source: Register::Sp,
                ..
            },
        })
    ));
    assert!(Decoder.lift(GuestAddress(0), 0x0b008000).is_err()); // W32 shift 32
    assert!(Decoder.lift(GuestAddress(0), 0x0bc00000).is_err()); // reserved arithmetic ROR
    assert!(Decoder.lift(GuestAddress(0), 0x0a008000).is_err()); // W32 logical shift 32
    assert!(Decoder.lift(GuestAddress(0), 0x94000000).is_err()); // BL requires call semantics
    assert!(Decoder.lift(GuestAddress(0), 0x17ffffff).is_err()); // PC-relative underflow
}
