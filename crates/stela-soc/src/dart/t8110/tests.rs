use super::super::super::interrupt::InterruptLatch;
use super::*;
use stela_contracts::dma::test_support::Ram;
use stela_contracts::refuse;

const ROOT: u64 = 0x1_0000_0000;
const LEAVES: u64 = ROOT + PAGE;
const FRAME: u64 = ROOT + 4 * PAGE;

fn table_entry(address: u64) -> u64 {
    (address >> ENTRY_ADDRESS_SHIFT) & ENTRY_ADDRESS | ENTRY_VALID
}

fn setup() -> (Dart, Ram, InterruptLatch) {
    setup_version(VERSION_1_0)
}

fn setup_version(version: u16) -> (Dart, Ram, InterruptLatch) {
    let mut dart = Dart::for_version(version, 16, Some(AicInput::new(0, 0x301)));
    let mut ram = Ram::new(ROOT, 8 * PAGE as usize);
    let mut latch = InterruptLatch::new();
    let mut access = Access::new(0, 0, &mut latch);
    // IOVA 0x200_4000: first-level entry 1, page entry 1.
    ram.put_u64(ROOT + 8, table_entry(LEAVES));
    ram.put_u64(
        LEAVES + 8,
        table_entry(FRAME) | 0xfff << 40 | ENTRY_NO_WRITE,
    );
    for (offset, value) in [
        (ENABLE_STREAMS, 0xffff),
        (TCR + 4, TCR_TRANSLATE),
        (
            TTBR + 4,
            ((ROOT >> PAGE_SHIFT) << TTBR_ADDRESS_SHIFT) as u32 | TTBR_VALID,
        ),
        (TCR + 8, TCR_BYPASS_DART | TCR_BYPASS_DAPF),
    ] {
        dart.write(offset, Width::Word, u64::from(value), &mut access)
            .unwrap();
    }
    (dart, ram, latch)
}

#[test]
fn resume_restores_transaction_limits() {
    let mut d = Dart::new(16, None);
    let mut sink = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut sink);
    for (index, offset) in TLIMITS.into_iter().enumerate() {
        assert_eq!(d.read(offset, Width::Word, &mut a), Ok(0));
        d.write(offset, Width::Word, 0x1234_5678 + index as u64, &mut a)
            .unwrap();
        let saved = d.read(offset, Width::Word, &mut a).unwrap();
        d.set_power(false, false, &mut a);
        d.set_power(true, false, &mut a);
        assert_eq!(d.read(offset, Width::Word, &mut a), Ok(0));
        d.write(offset, Width::Word, saved, &mut a).unwrap();
        assert_eq!(d.read(offset, Width::Word, &mut a), Ok(saved));
        d.write(offset, Width::Word, 0, &mut a).unwrap();
        assert_eq!(d.read(offset, Width::Word, &mut a), Ok(0));
        assert!(d.read(offset, Width::Double, &mut a).is_err());
    }
}

#[test]
fn sptm_power_cycle_programs_the_interrupt_words_and_no_counter_interrupt_pends() {
    let mut d = Dart::new(16, None);
    let mut sink = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut sink);
    // Power-down: clear all, disable the counter interrupts.
    d.write(PERF_INTERRUPT_STATUS, Width::Word, 0xffff_ffff, &mut a)
        .unwrap();
    assert_eq!(d.read(PERF_INTERRUPT_STATUS, Width::Word, &mut a), Ok(0));
    d.write(PERF_INTERRUPT_ENABLE, Width::Word, 0, &mut a)
        .unwrap();
    d.set_power(false, false, &mut a);
    d.set_power(true, false, &mut a);
    // Power-up: enable all, clear all, restore the in-use streams.
    d.write(PERF_INTERRUPT_ENABLE, Width::Word, 0xffff_ffff, &mut a)
        .unwrap();
    assert_eq!(
        d.read(PERF_INTERRUPT_ENABLE, Width::Word, &mut a),
        Ok(u64::from(PERF_INTERRUPTS))
    );
    d.write(STREAM_INTERRUPTS + 4, Width::Word, 0x5, &mut a)
        .unwrap();
    assert_eq!(d.read(STREAM_INTERRUPTS + 4, Width::Word, &mut a), Ok(5));
    // Every invalidation SPTM issues completes; OP 3 is undefined.
    for (offset, value) in [
        (TLB_RANGE[0], 0x40),
        (TLB_RANGE[1], 0x80),
        (TLB_CMD, 1 << 14 | 1 << 8 | 3),
        (TLB_CMD, 0x7100 | 3),
        (TLB_PARAMETER, 7),
        (TLB_CMD, 2 << 8),
        (TLB_CMD, 1 << 15 | 4 << 8),
        (TLB_CMD, 1 << 15 | 1 << 13),
        (TLB_SYNC, 0),
    ] {
        d.write(offset, Width::Word, value, &mut a).unwrap();
    }
    assert_eq!(d.read(TLB_RANGE[1], Width::Word, &mut a), Ok(0x80));
    assert_eq!(d.read(TLB_CMD, Width::Word, &mut a), Ok(0));
    assert_eq!(d.read(TLB_SYNC, Width::Word, &mut a), Ok(0));
    assert_eq!(d.read(TLB_SYNC_STATUS, Width::Word, &mut a), Ok(0));
    assert!(d.write(TLB_CMD, Width::Word, 3 << 8, &mut a).is_err());
    assert!(d.write(TLB_SYNC, Width::Word, 1, &mut a).is_err());
    // The counters themselves are absent.
    assert!(d.read(0x760, Width::Word, &mut a).is_err());
    // Power-down disables each stream and reads the word back.
    d.write(ENABLE_STREAMS + 4, Width::Word, 0x3, &mut a)
        .unwrap();
    d.write(DISABLE_STREAMS + 4, Width::Word, 0x1, &mut a)
        .unwrap();
    assert_eq!(d.read(DISABLE_STREAMS + 4, Width::Word, &mut a), Ok(2));
    assert_eq!(d.read(UNPROTECT, Width::Word, &mut a), Ok(0));
    d.set_power(false, false, &mut a);
    d.set_power(true, false, &mut a);
    assert_eq!(d.read(STREAM_INTERRUPTS + 4, Width::Word, &mut a), Ok(0));
}

#[test]
fn power_gates_lose_ordinary_state_but_retention_and_secure_zone_locks_survive() {
    for retention in [false, true] {
        let (mut d, mut ram, mut latch) = setup();
        let mut a = Access::new(0, 0, &mut latch);
        d.write(PROTECT, Width::Word, u64::from(PROTECT_TZ | 1), &mut a)
            .unwrap();
        d.write(PROTECT_LOCK, Width::Word, u64::from(PROTECT_TZ | 1), &mut a)
            .unwrap();
        d.set_power(false, retention, &mut a);
        assert!(d.read(PARAMS1, Width::Word, &mut a).is_err());
        assert_eq!(
            d.translate(&mut ram, 2, ROOT, ROOT, false, &mut a),
            Err(DmaError::Unsupported(refuse!("DART power domain is off")))
        );
        d.set_power(true, retention, &mut a);
        assert_eq!(
            d.read(TCR + 4, Width::Word, &mut a),
            Ok(if retention { 1 } else { 0 })
        );
        assert_eq!(
            d.read(PROTECT_LOCK, Width::Word, &mut a),
            Ok(u64::from(PROTECT_TZ) | u64::from(retention))
        );
        for offset in [TZ_CONFIG, TZ_SELECT, TZ_FIRST, TZ_END - 8] {
            assert_eq!(d.read(offset, Width::Word, &mut a), Ok(0));
            assert!(d.write(offset, Width::Word, 1, &mut a).is_err());
        }
        assert!(d.read(TZ_FIRST + 4, Width::Word, &mut a).is_err());
        if !retention {
            d.write(
                TCR + 8,
                Width::Word,
                u64::from(TCR_BYPASS_DART | TCR_BYPASS_DAPF),
                &mut a,
            )
            .unwrap();
            d.write(ENABLE_STREAMS, Width::Word, 4, &mut a).unwrap();
            assert_eq!(
                d.translate(&mut ram, 2, ROOT, ROOT, false, &mut a),
                Ok(ROOT)
            );
        }
    }
}

#[test]
fn powerup_restores_disabled_protected_write_policy_without_unlocking_tables() {
    let (mut dart, _, mut latch) = setup();
    let mut access = Access::new(0, 0, &mut latch);
    dart.write(PROTECT, Width::Word, 1, &mut access).unwrap();
    dart.write(PROTECT_LOCK, Width::Word, 1, &mut access)
        .unwrap();
    dart.write(PROTECTED_WRITE, Width::Word, 0, &mut access)
        .unwrap();
    assert_eq!(dart.read(PROTECTED_WRITE, Width::Word, &mut access), Ok(0));
    assert_eq!(dart.read(PROTECT_LOCK, Width::Word, &mut access), Ok(1));
    assert!(
        dart.write(PROTECTED_WRITE, Width::Word, 2, &mut access)
            .is_err()
    );
    assert!(
        dart.read(PROTECTED_WRITE, Width::Double, &mut access)
            .is_err()
    );
}

#[test]
fn instances_keep_separate_registers_and_or_their_shared_fault_wire() {
    let irq = AicInput::new(0, 1035);
    let mut group = Dart::group(16, Some(irq), 2);
    let mut latch = InterruptLatch::new();
    let mut ram = Ram::new(0, 0x4000);
    {
        let mut access = Access::new(0, 0, &mut latch);
        for (instance, sid) in [(0, 0), (1, 1)] {
            assert!(
                group[instance]
                    .translate(&mut ram, sid, 0, 3, false, &mut access)
                    .is_err()
            );
        }
        group[0].write(TCR, Width::Word, 2, &mut access).unwrap();
        assert_eq!(group[1].read(TCR, Width::Word, &mut access), Ok(0));
    }
    assert!(latch.is_asserted(Wire::Aic(irq)));
    {
        let mut access = Access::new(0, 0, &mut latch);
        group[0]
            .write(ERROR, Width::Word, u64::from(ERROR_FLAG), &mut access)
            .unwrap();
    }
    assert!(latch.is_asserted(Wire::Aic(irq)));
    {
        let mut access = Access::new(0, 0, &mut latch);
        group[1]
            .write(ERROR, Width::Word, u64::from(ERROR_FLAG), &mut access)
            .unwrap();
    }
    assert!(!latch.is_asserted(Wire::Aic(irq)));
}

#[test]
fn streams_translate_through_two_levels_or_bypass() {
    let (mut dart, mut ram, mut latch) = setup();
    let mut access = Access::new(0, 0, &mut latch);
    assert_eq!(
        dart.read(PARAMS1, Width::Word, &mut access),
        Ok(1 << 31 | 0xe << 24)
    );
    assert_eq!(dart.read(PARAMS2, Width::Word, &mut access), Ok(1));
    assert_eq!(
        dart.read(PARAMS3, Width::Word, &mut access),
        Ok(42 << 24 | 36 << 16 | 0x100)
    );
    assert_eq!(dart.read(PARAMS4, Width::Word, &mut access), Ok(16));
    let iova = 0x200_4000 + 0x123;
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova + 3, false, &mut access),
        Ok(FRAME + 0x123)
    );
    assert_eq!(
        dart.translate(&mut ram, 2, 0x8_1234_5678, 0x8_1234_5678, true, &mut access),
        Ok(0x8_1234_5678)
    );
}

#[test]
fn faults_latch_the_first_error_and_raise_the_interrupt_until_cleared() {
    let (mut dart, mut ram, mut latch) = setup();
    let mut access = Access::new(0, 0, &mut latch);
    let iova = 0x200_4000;
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, true, &mut access),
        Err(fault(Fault::Write))
    );
    assert_eq!(
        dart.translate(&mut ram, 1, 0x400_0000, 0x400_0000, false, &mut access),
        Err(fault(Fault::NoPmd))
    );
    // Page entry 2 allows bytes 0x10..0x1f: a read past them violates
    // its region.
    ram.put_u64(LEAVES + 16, table_entry(FRAME) | 4 << 52 | 7 << 40);
    let narrow = 0x200_8000;
    assert_eq!(
        dart.translate(&mut ram, 1, narrow + 0x10, narrow + 0x1f, true, &mut access),
        Ok(FRAME + 0x10)
    );
    for (first, last) in [(0xc, 0xf), (0x1c, 0x20)] {
        assert_eq!(
            dart.translate(
                &mut ram,
                1,
                narrow + first,
                narrow + last,
                false,
                &mut access
            ),
            Err(fault(Fault::Region))
        );
    }
    let read = |dart: &mut Dart, offset, access: &mut Access<'_>| {
        dart.read(offset, Width::Word, access).unwrap() as u32
    };
    let error = read(&mut dart, ERROR, &mut access);
    assert_eq!(
        error,
        ERROR_FLAG | ERROR_WRITE | 1 << ERROR_STREAM_SHIFT | Fault::Write as u32
    );
    assert_eq!(read(&mut dart, ERROR_ADDRESS_LOW, &mut access), iova as u32);
    assert_eq!(read(&mut dart, ERROR_STREAMS, &mut access), 0b10);
    assert!(dart.irq_level);
    dart.write(ERROR, Width::Word, u64::from(error), &mut access)
        .unwrap();
    assert_eq!(read(&mut dart, ERROR, &mut access), 0);
    assert!(!dart.irq_level);
    // A masked code latches without signalling.
    dart.write(ERROR_MASK, Width::Word, Fault::NoTtbr as u64, &mut access)
        .unwrap();
    let _ = dart.translate(&mut ram, 3, 0, 0, false, &mut access);
    assert_eq!(read(&mut dart, ERROR, &mut access) & ERROR_CODE, 1);
    assert!(!dart.irq_level);
}

#[test]
fn sptm_code_only_acknowledgement_clears_the_fault_and_shared_irq() {
    let irq = AicInput::new(0, 0x301);
    let (mut dart, mut ram, mut sink) = setup();
    let iova = 0x200_4000;
    {
        let mut a = Access::new(0, 0, &mut sink);
        assert_eq!(
            dart.translate(&mut ram, 1, iova, iova, true, &mut a),
            Err(fault(Fault::Write))
        );
        // A later read failure keeps the first write's address/direction.
        assert!(dart.translate(&mut ram, 3, 0, 0, false, &mut a).is_err());
        assert_eq!(dart.read(ERROR_ADDRESS_LOW, Width::Word, &mut a), Ok(iova));
        assert_ne!(
            dart.read(ERROR, Width::Word, &mut a).unwrap() & u64::from(ERROR_WRITE),
            0
        );
        dart.write(ERROR, Width::Word, Fault::Read as u64, &mut a)
            .unwrap();
    }
    assert!(sink.is_asserted(Wire::Aic(irq)));
    {
        let mut a = Access::new(0, 0, &mut sink);
        // The original SPTM writes the code, without ERROR_FLAG.
        dart.write(ERROR, Width::Word, Fault::Write as u64, &mut a)
            .unwrap();
        assert_eq!(dart.read(ERROR, Width::Word, &mut a), Ok(0));
        // Per-stream fault evidence is independently W1C.
        assert_eq!(dart.read(ERROR_STREAMS, Width::Word, &mut a), Ok(0xa));
        dart.write(ERROR_STREAMS, Width::Word, 2, &mut a).unwrap();
        assert_eq!(dart.read(ERROR_STREAMS, Width::Word, &mut a), Ok(8));
    }
    assert!(!sink.is_asserted(Wire::Aic(irq)));
    {
        let mut a = Access::new(0, 0, &mut sink);
        assert!(dart.translate(&mut ram, 3, 0, 0, false, &mut a).is_err());
        assert_eq!(
            dart.read(ERROR, Width::Word, &mut a).unwrap() & u64::from(ERROR_WRITE),
            0
        );
    }
    assert!(sink.is_asserted(Wire::Aic(irq)));
}

#[test]
fn ttbr_uses_only_the_address_field_and_stream_words_never_alias_unaligned_offsets() {
    let (mut dart, mut ram, mut sink) = setup();
    let mut a = Access::new(0, 0, &mut sink);
    let ttbr = dart.read(TTBR + 4, Width::Word, &mut a).unwrap();
    // R_TTBR.ADDR is [29:2]; bits31:30 and bit1 are not address bits.
    dart.write(TTBR + 4, Width::Word, ttbr | 0xc000_0002, &mut a)
        .unwrap();
    let iova = 0x200_4000;
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, false, &mut a),
        Ok(FRAME)
    );
    for offset in [TCR + 5, TTBR + 5, TCR + 4 * 16, TTBR + 4 * 16] {
        assert!(dart.read(offset, Width::Word, &mut a).is_err());
        assert!(dart.write(offset, Width::Word, 0, &mut a).is_err());
    }
    assert_eq!(
        dart.read(TCR + 4, Width::Word, &mut a),
        Ok(u64::from(TCR_TRANSLATE))
    );
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, false, &mut a),
        Ok(FRAME)
    );
}

#[test]
fn a_remapped_stream_takes_its_target_context() {
    let (mut dart, mut ram, mut sink) = setup();
    let mut a = Access::new(0, 0, &mut sink);
    let remap = |target: u32| TCR_REMAP_ENABLE | target << TCR_REMAP_SHIFT;
    dart.write(TCR, Width::Word, u64::from(remap(1)), &mut a)
        .unwrap();
    assert_eq!(
        dart.translate(&mut ram, 0, 0x200_4010, 0x200_4013, false, &mut a),
        Ok(FRAME + 0x10)
    );
    // A fault's recorded SID is unrecovered for a remapped source.
    assert_eq!(
        dart.translate(&mut ram, 0, 0x200_4010, 0x200_4013, true, &mut a),
        Err(DmaError::Unsupported(refuse!(
            "DART remapped write fault: source-versus-context SID provenance"
        )))
    );
    assert_eq!(dart.read(ERROR, Width::Word, &mut a), Ok(0));
    // The context's own enable gates the routed stream.
    dart.write(DISABLE_STREAMS, Width::Word, 1 << 1, &mut a)
        .unwrap();
    assert!(matches!(
        dart.translate(&mut ram, 0, 0x200_4010, 0x200_4013, false, &mut a),
        Err(DmaError::Unsupported(_))
    ));
    dart.write(ENABLE_STREAMS, Width::Word, 1 << 1, &mut a)
        .unwrap();
    for (source_tcr, target) in [
        (remap(1) | TCR_BYPASS_DART, 1),
        (remap(3), 3),
        (remap(16), 16),
    ] {
        dart.write(TCR, Width::Word, u64::from(source_tcr), &mut a)
            .unwrap();
        dart.write(TCR + 12, Width::Word, u64::from(remap(1)), &mut a)
            .unwrap();
        assert!(
            matches!(
                dart.translate(&mut ram, 0, 0x200_4010, 0x200_4013, false, &mut a),
                Err(DmaError::Unsupported(_))
            ),
            "target {target}"
        );
    }
}

#[test]
fn unimplemented_and_invalid_translation_modes_never_bypass_permissions() {
    let (mut dart, mut ram, mut sink) = setup();
    let mut a = Access::new(0, 0, &mut sink);
    for (mode, reason) in [
        (
            TCR_TRANSLATE | TCR_BYPASS_DART,
            "DART simultaneous translation and bypass",
        ),
        (
            TCR_TRANSLATE | TCR_REMAP_ENABLE | 3 << 8,
            "DART remapped stream with its own translation, bypass or table",
        ),
        (TCR_TRANSLATE | TCR_FOUR_LEVEL, "four-level DART tables"),
    ] {
        dart.write(TCR + 4, Width::Word, u64::from(mode), &mut a)
            .unwrap();
        assert_eq!(
            dart.translate(&mut ram, 1, 0, 0, true, &mut a),
            Err(DmaError::Unsupported(reason.into()))
        );
        assert_eq!(dart.read(ERROR, Width::Word, &mut a), Ok(0));
    }
}

#[test]
fn dma_completes_prior_pages_but_stops_before_a_protected_page() {
    let (mut dart, mut ram, mut sink) = setup();
    ram.put_u64(LEAVES + 8, table_entry(FRAME) | 0xfff << 40);
    ram.put_u64(
        LEAVES + 16,
        table_entry(FRAME + PAGE) | 0xfff << 40 | ENTRY_NO_WRITE,
    );
    let mut a = Access::new(0, 0, &mut sink);
    let first = 0x200_8000 - 4;
    let mut port = stela_contracts::dma::Port {
        iommu: &mut dart,
        stream: 1,
        memory: &mut ram,
        access: &mut a,
    };
    assert_eq!(
        port.write(first, &[1, 2, 3, 4, 5, 6, 7, 8]),
        Err(fault(Fault::Write))
    );
    let mut bytes = [0; 8];
    assert!(ram.read(FRAME + PAGE - 4, &mut bytes));
    assert_eq!(bytes, [1, 2, 3, 4, 0, 0, 0, 0]);
    assert_eq!(
        dart.read(ERROR_ADDRESS_LOW, Width::Word, &mut a),
        Ok(0x200_8000)
    );
}

#[test]
fn quiesce_and_reset_are_distinct_from_kept_power() {
    let (mut dart, mut ram, mut sink) = setup();
    let mut a = Access::new(0, 0, &mut sink);
    dart.write(PROTECT, Width::Word, 1, &mut a).unwrap();
    dart.write(PROTECT_LOCK, Width::Word, 1, &mut a).unwrap();
    dart.set_quiesced(true);
    assert_eq!(dart.read(TCR + 4, Width::Word, &mut a), Ok(1));
    assert_eq!(
        dart.translate(&mut ram, 2, ROOT, ROOT, false, &mut a),
        Err(DmaError::Unsupported(refuse!("DART DMA is quiesced")))
    );
    dart.set_power(false, true, &mut a);
    dart.set_power(true, true, &mut a);
    assert_eq!(dart.read(PROTECT_LOCK, Width::Word, &mut a), Ok(1));
    // Reset assertion clears state even when the supply it.
    dart.set_reset(true, &mut a);
    assert!(dart.read(TCR + 4, Width::Word, &mut a).is_err());
    assert!(dart.write(TCR, Width::Word, 1, &mut a).is_err());
    assert_eq!(
        dart.translate(&mut ram, 2, ROOT, ROOT, false, &mut a),
        Err(DmaError::Unsupported(refuse!("DART reset is asserted")))
    );
    dart.set_reset(false, &mut a);
    assert_eq!(dart.read(TCR + 4, Width::Word, &mut a), Ok(0));
    assert_eq!(dart.read(PROTECT_LOCK, Width::Word, &mut a), Ok(0));
    assert_eq!(
        dart.translate(&mut ram, 2, ROOT, ROOT, false, &mut a),
        Err(DmaError::Unsupported(refuse!("DART DMA is quiesced")))
    );
    dart.set_quiesced(false);
    // Releasing reset does not restore a software-owned table or enable.
    assert_eq!(
        dart.translate(&mut ram, 2, ROOT, ROOT, false, &mut a),
        Err(fault(Fault::NoTtbr))
    );
}

#[test]
fn cacheless_invalidation_observes_new_tables_and_permissions() {
    let (mut dart, mut ram, mut sink) = setup();
    let mut a = Access::new(0, 0, &mut sink);
    let iova = 0x200_4000;
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, false, &mut a),
        Ok(FRAME)
    );
    ram.put_u64(
        LEAVES + 8,
        table_entry(FRAME + PAGE) | 0xfff << 40 | ENTRY_NO_READ,
    );
    dart.write(
        TLB_CMD,
        Width::Word,
        u64::from(TLB_OP_FLUSH_STREAM | 1),
        &mut a,
    )
    .unwrap();
    assert_eq!(dart.read(TLB_CMD, Width::Word, &mut a), Ok(0));
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, false, &mut a),
        Err(fault(Fault::Read))
    );
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, true, &mut a),
        Ok(FRAME + PAGE)
    );
    ram.put_u64(LEAVES + 8, 0);
    dart.write(TLB_CMD, Width::Word, 0, &mut a).unwrap();
    assert_eq!(
        dart.translate(&mut ram, 1, iova, iova, true, &mut a),
        Err(fault(Fault::NoPte))
    );
}

#[test]
fn smmu_view_shares_trad_power_and_reset_without_creating_capture_entries() {
    let mut sink = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut sink);
    let mut plain = Dart::new(16, None);
    assert!(plain.read(SMMU_OFFSET, Width::Word, &mut a).is_err());
    assert!(plain.enable_smmu());
    assert!(!plain.enable_smmu());
    let mut dart = Dart::new(16, None);
    assert!(dart.enable_smmu());
    for model in [&mut plain, &mut dart] {
        assert_eq!(model.read(SMMU_OFFSET, Width::Word, &mut a), Ok(0));
        assert!(
            model
                .write(SMMU_OFFSET + 0x80, Width::Word, 0, &mut a)
                .is_err()
        );
        assert!(
            model
                .read(SMMU_OFFSET + 0x2000, Width::Word, &mut a)
                .is_err()
        );
        model.set_power(false, true, &mut a);
        assert!(model.read(SMMU_OFFSET, Width::Word, &mut a).is_err());
        model.set_power(true, true, &mut a);
        model.set_reset(true, &mut a);
        assert!(model.read(SMMU_OFFSET, Width::Word, &mut a).is_err());
        model.set_reset(false, &mut a);
        assert_eq!(model.read(SMMU_OFFSET, Width::Word, &mut a), Ok(0));
    }
}

#[test]
fn the_apf_holds_its_slices_until_sptm_locks_them() {
    let mut latch = InterruptLatch::new();
    let mut access = Access::new(0, 0, &mut latch);
    let mut dart = Dart::new(16, None).with_apf(16);
    let slice = |s: u64, register: u64| DAPF_OFFSET + s * SLICE + register;
    for (offset, value) in [
        (slice(1, SLICE_START), 0x0001_c000),
        (slice(1, SLICE_START + 4), 0x2),
        (slice(1, SLICE_END), 0x0005_1000),
        (slice(1, SLICE_END + 4), 0x2),
        (slice(1, SLICE_STREAMS), 0x1),
        (slice(1, SLICE_CONTROL), 0x103),
    ] {
        dart.write(offset, Width::Word, value, &mut access).unwrap();
        assert_eq!(dart.read(offset, Width::Word, &mut access), Ok(value));
    }
    assert!(
        dart.write(slice(1, SLICE_CONTROL), Width::Word, 0x4, &mut access)
            .is_err()
    );
    assert!(dart.read(slice(1, 0x04), Width::Word, &mut access).is_err());
    assert!(dart.read(slice(16, 0), Width::Word, &mut access).is_err());
    // SPTM locks the filter; its slices then keep their values.
    dart.write(PROTECT, Width::Word, 4, &mut access).unwrap();
    dart.write(PROTECT_LOCK, Width::Word, 4, &mut access)
        .unwrap();
    dart.write(UNPROTECT, Width::Word, 4, &mut access).unwrap();
    assert_eq!(dart.read(PROTECT, Width::Word, &mut access), Ok(4));
    assert_eq!(dart.read(PROTECT_LOCK, Width::Word, &mut access), Ok(4));
    dart.write(slice(1, SLICE_STREAMS), Width::Word, 0, &mut access)
        .unwrap();
    assert_eq!(
        dart.read(slice(1, SLICE_STREAMS), Width::Word, &mut access),
        Ok(1)
    );
    // A transfer that the filter would check refuses.
    let mut ram = Ram::new(ROOT, PAGE as usize);
    dart.write(ENABLE_STREAMS, Width::Word, 1, &mut access)
        .unwrap();
    dart.write(TCR, Width::Word, u64::from(TCR_BYPASS_DART), &mut access)
        .unwrap();
    assert_eq!(
        dart.translate(&mut ram, 0, ROOT, ROOT, false, &mut access),
        Err(DmaError::Unsupported(refuse!(
            "DART address permission filtering"
        )))
    );
    // A DART without a filter has no APF domain.
    let mut plain = Dart::new(16, None);
    assert!(plain.write(PROTECT, Width::Word, 4, &mut access).is_err());
    assert!(plain.read(DAPF_OFFSET, Width::Word, &mut access).is_err());
}

/// Original programming inputs are checked independently of active APF
/// transfer semantics. This fixture preserves that refusal.
#[test]
#[ignore = "oracle: STELA_TEST_CPU_DT_DIR"]
fn original_pmp_apf_programming_keeps_all_slices_without_granting_dma() {
    let bytes = std::fs::read(
        std::path::PathBuf::from(std::env::var("STELA_TEST_CPU_DT_DIR").unwrap())
            .join("DeviceTree.j473ap.adt"),
    )
    .unwrap();
    assert_eq!(
        stela_formats::sha256(&bytes),
        "a99fa2260f292236fbad210fcee380fc69f101e591125bce347819e84e68034c"
    );
    let root = stela_contracts::adt::Node::root(&bytes).unwrap();
    let node = root.child("arm-io").unwrap().child("dart-pmp").unwrap();
    assert_eq!(node.first_u32("n-apfs-instance-0"), Some(16));
    assert_eq!(node.first_u32("sid"), Some(0));
    assert!(node.property("apf-bypass-0").is_none());
    assert!(node.property("bypass-0").is_none());
    let records = node.property("dapf-instance-0").unwrap();
    assert_eq!(records.len(), 16 * 52);
    let mut dart = Dart::new(16, None).with_apf(16);
    let mut latch = InterruptLatch::new();
    let mut access = Access::new(0, 0, &mut latch);
    for (index, raw) in records.as_chunks::<52>().0.iter().enumerate() {
        // Original AP _apfSetupInstance 9f172f0..17328: raw record
        // bytes50/49/48 populate bits1:0,5:4,9:8 respectively.
        // These positions establish encoding, not permission meanings.
        let control =
            u32::from(raw[50] & 3) | (u32::from(raw[49] & 3) << 4) | (u32::from(raw[48] & 3) << 8);
        assert_eq!(&raw[48..52], &[0, 3, 1, 0]);
        assert_eq!(control, 0x31);
        let base = DAPF_OFFSET + index as u64 * SLICE;
        for (offset, value) in [
            (SLICE_START, 0),
            (SLICE_START + 4, 4),
            (SLICE_END, 8),
            (SLICE_END + 4, 12),
        ] {
            let value = u64::from(u32::from_le_bytes(
                raw[value..value + 4].try_into().unwrap(),
            ));
            dart.write(base + offset, Width::Word, value, &mut access)
                .unwrap();
            assert_eq!(
                dart.read(base + offset, Width::Word, &mut access),
                Ok(value)
            );
        }
        for word in 0..8 {
            let value = u64::from(u32::from_le_bytes(
                raw[16 + word * 4..20 + word * 4].try_into().unwrap(),
            ));
            assert_eq!(value, u64::from(word == 0));
            let offset = base + SLICE_STREAMS + word as u64 * 4;
            dart.write(offset, Width::Word, value, &mut access).unwrap();
            assert_eq!(dart.read(offset, Width::Word, &mut access), Ok(value));
        }
        dart.write(
            base + SLICE_CONTROL,
            Width::Word,
            u64::from(control),
            &mut access,
        )
        .unwrap();
        assert_eq!(
            dart.read(base + SLICE_CONTROL, Width::Word, &mut access),
            Ok(0x31)
        );
    }
    for register in [PROTECT, PROTECT_LOCK] {
        dart.write(register, Width::Word, u64::from(PROTECT_APF), &mut access)
            .unwrap();
    }
    dart.write(DAPF_OFFSET + SLICE_CONTROL, Width::Word, 0, &mut access)
        .unwrap();
    assert_eq!(
        dart.read(DAPF_OFFSET + SLICE_CONTROL, Width::Word, &mut access),
        Ok(0x31)
    );
    // Authored TCR state: the source has no APF bypass property, but
    // this fixture does not claim to have executed original setup.
    dart.write(ENABLE_STREAMS, Width::Word, 1, &mut access)
        .unwrap();
    dart.write(TCR, Width::Word, u64::from(TCR_TRANSLATE), &mut access)
        .unwrap();
    let mut ram = Ram::new(ROOT, PAGE as usize);
    for write in [false, true] {
        assert_eq!(
            dart.translate(&mut ram, 0, 0x4000, 0x4003, write, &mut access),
            Err(DmaError::Unsupported(refuse!(
                "DART address permission filtering"
            )))
        );
    }
}

#[test]
fn protection_freezes_stream_tables_and_the_lock_keeps_it() {
    let (mut dart, _, mut latch) = setup();
    let mut access = Access::new(0, 0, &mut latch);
    for (offset, value) in [
        (PROTECT, 1),
        (PROTECT_LOCK, 1),
        (UNPROTECT, 1),
        (TCR + 4, 0),
    ] {
        dart.write(offset, Width::Word, value, &mut access).unwrap();
    }
    assert_eq!(dart.read(PROTECT, Width::Word, &mut access), Ok(1));
    assert_eq!(
        dart.read(TCR + 4, Width::Word, &mut access),
        Ok(u64::from(TCR_TRANSLATE))
    );
    // Unknown registers and four-level tables are refused.
    assert!(dart.read(0x10, Width::Word, &mut access).is_err());
    assert!(
        dart.write(TCR + 4 * 16, Width::Word, 0, &mut access)
            .is_err()
    );
}

/// Revision 2.2 DVAs: the base of the two-level window DVA [46:36] 0x10.
const HIGH: u64 = 1 << 40;
const UPPER: u64 = ROOT + 2 * PAGE;

#[test]
fn revision_2_2_identifies_its_register_file_and_refuses_unrecovered_registers() {
    let mut d = Dart::for_version(VERSION_2_2, 64, None);
    let mut sink = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut sink);
    assert_eq!(
        d.read(PARAMS3, Width::Word, &mut a),
        Ok(42 << 24 | 42 << 16 | 0x0202)
    );
    assert_eq!(d.read(PARAMS4, Width::Word, &mut a), Ok(64));
    assert_eq!(d.read(CAPTURE_COUNT, Width::Word, &mut a), Ok(0));
    assert!(d.write(CAPTURE_COUNT, Width::Word, 0, &mut a).is_err());
    // SPTM's power-up clears the moved mask and reads it back.
    d.write(ERROR_MASK_2_2, Width::Word, 0, &mut a).unwrap();
    assert_eq!(d.read(ERROR_MASK_2_2, Width::Word, &mut a), Ok(0));
    assert!(d.write(ERROR_MASK_2_2, Width::Word, 1, &mut a).is_err());
    for offset in [
        ERROR,
        ERROR_MASK,
        ERROR_ADDRESS_LOW,
        ERROR_ADDRESS_HIGH,
        ERROR_STREAMS,
    ] {
        assert!(d.read(offset, Width::Word, &mut a).is_err());
        assert!(d.write(offset, Width::Word, 0, &mut a).is_err());
    }
    // Two pending words for 64 streams. The unknown bitmaps, absent
    // streams' records and the window beyond the register file refuse.
    assert_eq!(d.read(PENDING + 4, Width::Word, &mut a), Ok(0));
    for offset in [
        PENDING + 8,
        0x4020,
        0x4040,
        RECORDS + 64 * RECORD,
        REGISTER_BYTES_2_2,
        DAPF_OFFSET - 4,
    ] {
        assert!(d.read(offset, Width::Word, &mut a).is_err(), "{offset:#x}");
        assert!(
            d.write(offset, Width::Word, 0, &mut a).is_err(),
            "{offset:#x}"
        );
    }
    // The layout both revisions share is unchanged.
    let tcr = u64::from(TCR_TRANSLATE | TCR_FOUR_LEVEL);
    d.write(TCR + 4 * 63, Width::Word, tcr, &mut a).unwrap();
    assert_eq!(d.read(TCR + 4 * 63, Width::Word, &mut a), Ok(tcr));
    // The paired UMMS view's revision 2 interface is unrecovered.
    assert!(d.enable_smmu());
    assert!(d.read(SMMU_OFFSET, Width::Word, &mut a).is_err());
    assert!(d.write(SMMU_OFFSET + 0x80, Width::Word, 0, &mut a).is_err());
    // Revision 1.0 has none of these registers.
    let mut old = Dart::new(64, None);
    assert_eq!(
        old.read(PARAMS3, Width::Word, &mut a),
        Ok(42 << 24 | 36 << 16 | 0x0100)
    );
    for offset in [CAPTURE_COUNT, PENDING, ERROR_MASK_2_2, RECORDS + 8] {
        assert!(old.read(offset, Width::Word, &mut a).is_err());
    }
}

#[test]
fn revision_2_0_walks_a_wide_dva_through_three_tables() {
    let (mut dart, mut ram, mut sink) = setup_version(VERSION_2_0);
    let mut access = Access::new(0, 0, &mut sink);
    ram.put_u64(UPPER + 0x10 * 8, table_entry(ROOT));
    let ttbr = ((UPPER >> PAGE_SHIFT) << TTBR_ADDRESS_SHIFT) as u32 | TTBR_VALID;
    dart.write(
        TCR + 12,
        Width::Word,
        u64::from(TCR_TRANSLATE | TCR_FOUR_LEVEL),
        &mut access,
    )
    .unwrap();
    dart.write(TTBR + 12, Width::Word, u64::from(ttbr), &mut access)
        .unwrap();
    let dva = HIGH + 0x200_4010;
    assert_eq!(
        dart.translate(&mut ram, 3, dva, dva + 3, false, &mut access),
        Ok(FRAME + 0x10)
    );
}

#[test]
fn revision_2_2_walks_both_two_level_windows_and_three_four_level_tables() {
    let (mut d, mut ram, mut sink) = setup_version(VERSION_2_2);
    let mut a = Access::new(0, 0, &mut sink);
    for base in [0, HIGH] {
        assert_eq!(
            d.translate(
                &mut ram,
                1,
                base + 0x200_4010,
                base + 0x200_4013,
                false,
                &mut a
            ),
            Ok(FRAME + 0x10)
        );
    }
    let other = (1 << 36) + 0x200_4010;
    assert_eq!(
        d.translate(&mut ram, 1, other, other, false, &mut a),
        Err(DmaError::Unsupported(refuse!(
            "DART two-level DVA bits [46:36] outside the recovered 0 and 0x10 windows"
        )))
    );
    // FOUR_LEVEL: the TTBR's table maps DVA [46:36] to the same tables.
    ram.put_u64(UPPER + 0x10 * 8, table_entry(ROOT));
    for (offset, value) in [
        (TCR + 12, TCR_TRANSLATE | TCR_FOUR_LEVEL),
        (
            TTBR + 12,
            ((UPPER >> PAGE_SHIFT) << TTBR_ADDRESS_SHIFT) as u32 | TTBR_VALID,
        ),
    ] {
        d.write(offset, Width::Word, u64::from(value), &mut a)
            .unwrap();
    }
    assert_eq!(
        d.translate(
            &mut ram,
            3,
            HIGH + 0x200_4010,
            HIGH + 0x200_4013,
            false,
            &mut a
        ),
        Ok(FRAME + 0x10)
    );
    // DVA [46:36] 0 has no first-level entry: NO_PGD at level 1.
    assert_eq!(
        d.translate(&mut ram, 3, 0x200_4010, 0x200_4010, false, &mut a),
        Err(DmaError::Fault(Fault::NoPgd as u32))
    );
    let record = d.faults.as_ref().unwrap().record(3).unwrap().unwrap();
    assert_eq!(
        (record.code, record.level, record.write, record.address),
        (Code::Invalid, Level::Cte, false, 0x200_4010)
    );
    // Beyond the 42-bit VA space the TTBR is invalid.
    assert_eq!(
        d.translate(&mut ram, 1, 1 << 42, 1 << 42, false, &mut a),
        Err(DmaError::Fault(Fault::NoTtbr as u32))
    );
}

#[test]
fn revision_2_2_captures_per_stream_faults_until_their_pending_bits_clear() {
    let (mut d, mut ram, mut latch) = setup_version(VERSION_2_2);
    let irq = Wire::Aic(AicInput::new(0, 0x301));
    let dva = HIGH + 0x200_4010;
    let record = RECORDS + RECORD;
    {
        let mut a = Access::new(0, 0, &mut latch);
        // A write to the read-only page and an untranslated stream.
        assert_eq!(
            d.translate(&mut ram, 1, dva, dva + 3, true, &mut a),
            Err(DmaError::Fault(Fault::Write as u32))
        );
        assert_eq!(
            d.translate(&mut ram, 4, 0, 0, false, &mut a),
            Err(DmaError::Fault(Fault::NoTtbr as u32))
        );
        // A second fault of a pending stream is unchecked.
        assert_eq!(
            d.translate(&mut ram, 1, dva, dva + 3, true, &mut a),
            Err(DmaError::Unsupported(refuse!(
                "per-SID DART secondary fault capture is unsupported"
            )))
        );
        assert_eq!(d.read(PENDING, Width::Word, &mut a), Ok(1 << 4 | 1 << 1));
        assert_eq!(d.read(record + 8, Width::Word, &mut a), Ok(0x200_4010));
        assert_eq!(d.read(record + 12, Width::Word, &mut a), Ok(HIGH >> 32));
        // The complete status header and auxiliary data are unchecked.
        for offset in [record, record + 4, record + 0x18] {
            assert!(d.read(offset, Width::Word, &mut a).is_err());
        }
        assert!(d.write(record + 8, Width::Word, 0, &mut a).is_err());
        let faults = d.faults.as_ref().unwrap();
        assert_eq!(faults.record(1).unwrap().unwrap().class(), 1);
        assert_eq!(faults.record(4).unwrap().unwrap().class(), 0);
    }
    assert!(latch.is_asserted(irq));
    {
        let mut a = Access::new(0, 0, &mut latch);
        // Bits of absent streams cannot acknowledge anything.
        assert!(
            d.write(PENDING, Width::Word, 1 << 16 | 1 << 1, &mut a)
                .is_err()
        );
        d.write(PENDING, Width::Word, 1 << 1, &mut a).unwrap();
        assert!(d.read(record + 8, Width::Word, &mut a).is_err());
    }
    assert!(latch.is_asserted(irq));
    {
        let mut a = Access::new(0, 0, &mut latch);
        d.write(PENDING, Width::Word, 1 << 4, &mut a).unwrap();
    }
    assert!(!latch.is_asserted(irq));
    {
        // An acknowledged stream captures its next fault.
        let mut a = Access::new(0, 0, &mut latch);
        assert_eq!(
            d.translate(&mut ram, 1, dva, dva, true, &mut a),
            Err(DmaError::Fault(Fault::Write as u32))
        );
    }
    assert!(latch.is_asserted(irq));
    {
        // A power gate without retention clears the bank and the wire.
        let mut a = Access::new(0, 0, &mut latch);
        d.set_power(false, false, &mut a);
        d.set_power(true, false, &mut a);
        assert_eq!(d.read(PENDING, Width::Word, &mut a), Ok(0));
    }
    assert!(!latch.is_asserted(irq));
}

#[test]
fn revision_2_2_keeps_cacheless_invalidation_remap_and_its_protection_domains() {
    let (mut d, mut ram, mut sink) = setup_version(VERSION_2_2);
    let mut a = Access::new(0, 0, &mut sink);
    let dva = HIGH + 0x200_4010;
    assert_eq!(
        d.translate(&mut ram, 1, dva, dva, false, &mut a),
        Ok(FRAME + 0x10)
    );
    // A changed entry is visible at once; every invalidation iBoot and
    // SPTM issue completes (stream, ranged stream, powerdown flush).
    ram.put_u64(LEAVES + 8, table_entry(FRAME + PAGE) | 0xfff << 40);
    for command in [0x8101, 0xf101, 0x7101, 0xa000] {
        d.write(TLB_CMD, Width::Word, command, &mut a).unwrap();
    }
    d.write(TLB_SYNC, Width::Word, 0, &mut a).unwrap();
    assert_eq!(
        d.translate(&mut ram, 1, dva, dva, true, &mut a),
        Ok(FRAME + PAGE + 0x10)
    );
    // A remapped source takes its target's context; the SID a fault of
    // it records is unrecovered.
    let remap = TCR_REMAP_ENABLE | 1 << TCR_REMAP_SHIFT;
    d.write(TCR, Width::Word, u64::from(remap), &mut a).unwrap();
    assert_eq!(
        d.translate(&mut ram, 0, dva, dva, true, &mut a),
        Ok(FRAME + PAGE + 0x10)
    );
    ram.put_u64(
        LEAVES + 8,
        table_entry(FRAME) | 0xfff << 40 | ENTRY_NO_WRITE,
    );
    assert_eq!(
        d.translate(&mut ram, 0, dva, dva, true, &mut a),
        Err(DmaError::Unsupported(refuse!(
            "DART remapped write fault: source-versus-context SID provenance"
        )))
    );
    assert_eq!(d.read(PENDING, Width::Word, &mut a), Ok(0));
    // SPTM protects and locks domains 5 to 8 with TTBR_TCR.
    d.write(PROTECT, Width::Word, 0x1e1, &mut a).unwrap();
    d.write(PROTECT_LOCK, Width::Word, 0x1e1, &mut a).unwrap();
    assert_eq!(d.read(PROTECT_LOCK, Width::Word, &mut a), Ok(0x1e1));
    d.write(UNPROTECT, Width::Word, 0x1e1, &mut a).unwrap();
    assert_eq!(d.read(PROTECT, Width::Word, &mut a), Ok(0x1e1));
    d.write(TCR + 4, Width::Word, 0, &mut a).unwrap();
    assert_eq!(
        d.read(TCR + 4, Width::Word, &mut a),
        Ok(u64::from(TCR_TRANSLATE))
    );
    assert!(d.write(PROTECT, Width::Word, 1 << 9, &mut a).is_err());
    let mut old = Dart::new(16, None);
    assert!(old.write(PROTECT, Width::Word, 1 << 5, &mut a).is_err());
}
