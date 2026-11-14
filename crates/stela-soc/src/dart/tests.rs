use super::*;
use alloc::vec;
use stela_contracts::refuse;
use {
    stela_contracts::dma::Port, stela_contracts::dma::test_support::Ram,
    stela_contracts::interrupt::InterruptLatch, stela_contracts::interrupt::Wire,
};

#[test]
fn explicit_profiles_keep_the_shared_bus_master_type_and_table_effects() {
    const ROOT: u64 = 0x8_0000_0000;
    const PAGE: u64 = 0x4000;
    for profile in [Profile::T8020, Profile::T6000, Profile::T8110] {
        let mut model = Dart::for_profile(profile, 16, None).unwrap();
        let mut sink = InterruptLatch::new();
        let mut a = Access::new(0, 0, &mut sink);
        let mut ram = Ram::new(ROOT, (4 * PAGE) as usize);
        let entry = |address: u64| -> u64 {
            if profile == Profile::T8020 {
                address | 1
            } else {
                address >> 4 | 1
            }
        };
        ram.put_u64(ROOT + 8, entry(ROOT + PAGE));
        ram.put_u64(ROOT + PAGE + 8, entry(ROOT + 2 * PAGE) | 0xfff << 40);
        let (enable, tcr, ttbr, ttbr_value) = if profile == Profile::T8110 {
            (0xc00, 0x1004, 0x1404, ROOT >> 14 << 2 | 1)
        } else {
            (0xfc, 0x104, 0x210, ROOT >> 12 | 1 << 31)
        };
        for (offset, value) in [
            (enable, 2),
            (tcr, if profile == Profile::T8110 { 1 } else { 1 << 7 }),
            (ttbr, ttbr_value),
        ] {
            model.write(offset, Width::Word, value, &mut a).unwrap();
        }
        let mut port = Port {
            iommu: &mut model,
            stream: 1,
            memory: &mut ram,
            access: &mut a,
        };
        port.write(0x200_4007, &[1, 3, 5, 7]).unwrap();
        let mut actual = [0; 4];
        assert!(ram.read(ROOT + 2 * PAGE + 7, &mut actual));
        assert_eq!(actual, [1, 3, 5, 7]);
        let invalid_address = if profile == Profile::T8020 {
            1 << 36
        } else {
            1 << 38
        };
        let leaf_permission = if profile == Profile::T8020 {
            1 << 7
        } else {
            1 << 2
        };
        // Exercise both descriptor levels through a real DMA master.
        // Rejection must precede the data write and cannot invent a
        // physical fault classification or silently mask an address.
        for (table_bits, page_bits, reason) in [
            (1 << 9, 0, "DART descriptor contains unmodeled attributes"),
            (0, 1 << 9, "DART descriptor contains unmodeled attributes"),
            (
                leaf_permission,
                0,
                "DART descriptor contains unmodeled attributes",
            ),
            (
                invalid_address,
                0,
                "DART descriptor target exceeds its native profile",
            ),
            (
                0,
                invalid_address,
                "DART descriptor target exceeds its native profile",
            ),
        ] {
            ram.put_u64(ROOT + 8, entry(ROOT + PAGE) | table_bits);
            ram.put_u64(
                ROOT + PAGE + 8,
                entry(ROOT + 2 * PAGE) | 0xfff << 40 | page_bits,
            );
            let mut port = Port {
                iommu: &mut model,
                stream: 1,
                memory: &mut ram,
                access: &mut a,
            };
            assert_eq!(
                port.write(0x200_4007, &[9, 9, 9, 9]),
                Err(DmaError::Unsupported(reason.into()))
            );
            assert!(ram.read(ROOT + 2 * PAGE + 7, &mut actual));
            assert_eq!(actual, [1, 3, 5, 7]);
            let error = if profile == Profile::T8110 {
                0x100
            } else {
                0x40
            };
            assert_eq!(model.read(error, Width::Word, &mut a), Ok(0));
        }
    }
    assert!(Dart::for_profile(Profile::T8020, 64, None).is_err());
    assert!(Dart::for_profile(Profile::T6000, 0, None).is_err());
    assert_eq!(Dart::new(16, None).name(), "dart-t8110");
}

#[test]
fn grouped_generation_dispatch_preserves_independent_faults_on_the_shared_wire() {
    let irq = AicInput::new(0, 19);
    for profile in [Profile::T8020, Profile::T6000, Profile::T8110] {
        let mut models = Dart::group_for_profile(profile, 16, Some(irq), 2).unwrap();
        let mut sink = InterruptLatch::new();
        let mut ram = Ram::new(0, 0x4000);
        {
            let mut a = Access::new(0, 0, &mut sink);
            for model in &mut models {
                assert!(model.translate(&mut ram, 0, 0, 0, false, &mut a).is_err());
            }
        }
        assert!(sink.is_asserted(Wire::Aic(irq)));
        let error = if profile == Profile::T8110 {
            0x100
        } else {
            0x40
        };
        {
            let mut a = Access::new(0, 0, &mut sink);
            let captured = models[0].read(error, Width::Word, &mut a).unwrap();
            models[0]
                .write(error, Width::Word, captured, &mut a)
                .unwrap();
        }
        assert!(sink.is_asserted(Wire::Aic(irq)));
        {
            let mut a = Access::new(0, 0, &mut sink);
            let captured = models[1].read(error, Width::Word, &mut a).unwrap();
            models[1]
                .write(error, Width::Word, captured, &mut a)
                .unwrap();
        }
        assert!(!sink.is_asserted(Wire::Aic(irq)));
    }
}

#[test]
fn paired_capture_view_is_available_only_for_its_implemented_generation() {
    for profile in [
        Profile::T8020,
        Profile::T6000,
        Profile::T8110,
        Profile::T8110Rev22,
    ] {
        let mut model = Dart::for_profile(profile, 16, None).unwrap();
        assert!(!model.has_smmu());
        let supported = profile == Profile::T6000 || profile.is_t8110();
        assert_eq!(model.supports_smmu(), supported);
        assert_eq!(model.can_attach_smmu(), supported);
        // Capability queries have no attachment side effect.
        assert!(!model.has_smmu());
        if supported {
            model.enable_smmu().unwrap();
            assert_eq!(
                model.enable_smmu(),
                Err(refuse!("DART already has a paired SMMU instance"))
            );
        } else {
            assert_eq!(
                model.enable_smmu(),
                Err(refuse!(
                    "legacy DART paired SMMU capture format is unsupported"
                ))
            );
        }
        assert_eq!(model.has_smmu(), supported);
        assert!(!model.can_attach_smmu());
        let constructed = Dart::for_profile(profile, 16, None).unwrap().with_smmu();
        assert_eq!(constructed.is_ok(), supported);
    }
}

#[test]
fn construction_profile_survives_grouping_and_optional_views() {
    assert_eq!(Dart::new(16, None).profile(), Profile::T8110);
    for dart in Dart::group(16, None, 2) {
        assert_eq!(dart.profile(), Profile::T8110);
    }
    for profile in [
        Profile::T8020,
        Profile::T6000,
        Profile::T8110,
        Profile::T8110Rev22,
    ] {
        assert_eq!(
            Dart::for_profile(profile, 16, None)
                .unwrap()
                .with_apf(1)
                .profile(),
            profile
        );
        for dart in Dart::group_for_profile(profile, 16, None, 2).unwrap() {
            assert_eq!(dart.profile(), profile);
            if dart.supports_smmu() {
                assert_eq!(dart.with_smmu().unwrap().profile(), profile);
            }
        }
    }
}

#[test]
fn grouped_profile_requires_an_translation_instance() {
    for profile in [Profile::T8020, Profile::T6000, Profile::T8110] {
        assert!(matches!(
            Dart::group_for_profile(profile, 16, None, 0),
            Err(e) if e == refuse!("a DART group needs a translation instance")
        ));
    }
}

#[test]
fn mapper_context_and_declared_incoming_stream_remain_distinct() {
    for profile in [Profile::T8020, Profile::T6000, Profile::T8110] {
        assert_eq!(profile.mapper_input_stream(16, 0, &[(5, 0)]), Ok(5));
        assert_eq!(profile.mapper_input_stream(16, 4, &[]), Ok(4));
        assert_eq!(profile.mapper_input_stream(16, 1, &[(5, 0)]), Ok(1));
        assert_eq!(profile.mapper_input_stream(16, 5, &[(7, 5), (6, 5)]), Ok(6));
        for (streams, context, remaps) in [
            (0, 0, vec![]),
            (profile.stream_limit() + 1, 0, vec![]),
            (16, 16, vec![]),
            (16, 5, vec![(5, 0)]),
            (16, 0, vec![(5, 0), (5, 1)]),
            (16, 0, vec![(5, 0), (0, 1)]),
            (16, 0, vec![(0, 1), (5, 0)]),
            (16, 0, vec![(5, 16)]),
            (16, 0, vec![(5, 5)]),
        ] {
            assert!(
                profile
                    .mapper_input_stream(streams, context, &remaps)
                    .is_err()
            );
        }
    }
}

#[test]
fn separate_logical_groups_and_generations_share_one_physical_irq_owner() {
    let input = AicInput::new(0, 873);
    let shared = SharedIrq::new(input);
    let mut legacy = Dart::for_profile(Profile::T6000, 16, Some(input)).unwrap();
    let mut modern = Dart::for_profile(Profile::T8110, 16, Some(input)).unwrap();
    legacy.bind_shared_irq(&shared).unwrap();
    modern.bind_shared_irq(&shared).unwrap();
    let mut sink = InterruptLatch::new();
    let mut ram = Ram::new(0, 0x4000);
    {
        let mut access = Access::new(0, 0, &mut sink);
        for model in [&mut legacy, &mut modern] {
            assert!(matches!(
                model.translate(&mut ram, 0, 0, 0, false, &mut access),
                Err(DmaError::Fault(_))
            ));
            assert!(model.bind_shared_irq(&shared).is_err());
        }
        legacy.write(0x40, Width::Word, 1, &mut access).unwrap();
    }
    assert!(sink.is_asserted(stela_contracts::interrupt::Wire::Aic(input)));
    {
        let mut access = Access::new(0, 0, &mut sink);
        modern.set_reset(true, &mut access);
    }
    assert!(!sink.is_asserted(stela_contracts::interrupt::Wire::Aic(input)));
    assert!(
        legacy
            .bind_shared_irq(&SharedIrq::new(AicInput::new(0, 1)))
            .is_err()
    );
    assert!(
        Dart::for_profile(Profile::T6000, 16, None)
            .unwrap()
            .bind_shared_irq(&shared)
            .is_err()
    );
}

#[test]
fn an_active_original_group_cannot_be_split_by_rebinding_its_idle_peer() {
    let input = AicInput::new(0, 873);
    let mut models = Dart::group_for_profile(Profile::T6000, 16, Some(input), 2).unwrap();
    let mut sink = InterruptLatch::new();
    let mut ram = Ram::new(0, 0x4000);
    let mut access = Access::new(0, 0, &mut sink);
    assert!(matches!(
        models[0].translate(&mut ram, 0, 0, 0, false, &mut access),
        Err(DmaError::Fault(_))
    ));
    assert!(models[1].bind_shared_irq(&SharedIrq::new(input)).is_err());
}

#[test]
fn native_owner_capacity_is_available_without_mmio_or_power() {
    for profile in [
        Profile::T8020,
        Profile::T6000,
        Profile::T8110,
        Profile::T8110Rev22,
    ] {
        for count in [1, profile.stream_limit()] {
            let mut model = Dart::for_profile(profile, count, None).unwrap();
            let mut latch = InterruptLatch::new();
            let mut access = Access::new(0, 0, &mut latch);
            model.set_clock(false);
            model.set_power(false, true, &mut access);
            assert_eq!(model.stream_capacity(), count);
            model.set_reset(true, &mut access);
            assert_eq!(model.stream_capacity(), count);
        }
    }
}

#[test]
fn incoming_streams_outside_the_profile_never_truncate_into_fault_sids() {
    for profile in [Profile::T8020, Profile::T6000, Profile::T8110] {
        let mut model = Dart::for_profile(profile, 1, None).unwrap();
        let mut sink = InterruptLatch::new();
        let mut a = Access::new(0, 0, &mut sink);
        let mut ram = Ram::new(0, 0x4000);
        for stream in [1, 16, 256, usize::MAX] {
            assert_eq!(
                model.translate(&mut ram, stream, 0, 0, false, &mut a),
                Err(DmaError::Unsupported(refuse!(
                    "DART incoming stream is outside the native profile"
                )))
            );
            let error = if profile == Profile::T8110 {
                0x100
            } else {
                0x40
            };
            assert_eq!(model.read(error, Width::Word, &mut a), Ok(0));
        }
    }
}

#[test]
fn functional_clock_gating_keeps_configuration_and_blocks_both_access_paths() {
    for profile in [
        Profile::T8020,
        Profile::T6000,
        Profile::T8110,
        Profile::T8110Rev22,
    ] {
        let mut model = Dart::for_profile(profile, 16, None).unwrap();
        let mut sink = InterruptLatch::new();
        let mut a = Access::new(0, 0, &mut sink);
        let mut ram = Ram::new(0, 0x4000);
        let (tcr, bypass) = if profile.is_t8110() {
            (0x1000, 6)
        } else {
            (0x100, 1 << 8 | 1 << 12)
        };
        let enable = if profile.is_t8110() { 0xc00 } else { 0xfc };
        model.write(enable, Width::Word, 1, &mut a).unwrap();
        model.write(tcr, Width::Word, bypass, &mut a).unwrap();
        assert_eq!(model.translate(&mut ram, 0, 0, 0, false, &mut a), Ok(0));
        model.set_clock(false);
        assert!(model.read(tcr, Width::Word, &mut a).is_err());
        assert!(model.write(tcr, Width::Word, 0, &mut a).is_err());
        assert_eq!(
            model.translate(&mut ram, 0, 0, 0, false, &mut a),
            Err(DmaError::Unsupported(refuse!(
                "DART functional clock is gated"
            )))
        );
        model.set_clock(true);
        assert_eq!(model.read(tcr, Width::Word, &mut a), Ok(bypass));
        assert_eq!(model.translate(&mut ram, 0, 0, 0, false, &mut a), Ok(0));
        // Clock gating itself cannot clear an independent reset line.
        model.set_reset(true, &mut a);
        model.set_clock(false);
        model.set_clock(true);
        assert!(model.read(tcr, Width::Word, &mut a).is_err());
        model.set_reset(false, &mut a);
        assert_eq!(model.read(tcr, Width::Word, &mut a), Ok(0));
    }
}

#[test]
fn compatible_selection_never_aliases_a_different_native_family() {
    for (bytes, expected) in [
        (b"dart,t8020\0".as_slice(), Some(Profile::T8020)),
        (b"dart,t6000\0".as_slice(), Some(Profile::T6000)),
        (b"generic\0dart,t8110\0".as_slice(), Some(Profile::T8110)),
        (b"dart,gen3\0".as_slice(), None),
        (b"dart,gen3\0dart,t8110\0".as_slice(), None),
        (b"dart,t8020\0dart,t6000\0".as_slice(), None),
        (b"dart,unknown\0dart,t8020\0".as_slice(), None),
    ] {
        assert_eq!(Profile::from_compatible(bytes), expected);
    }
    assert_eq!(Profile::T8020.stream_limit(), 16);
    assert_eq!(Profile::T6000.stream_limit(), 16);
    assert_eq!(Profile::T8110.stream_limit(), 256);
    assert_eq!(Profile::T8110Rev22.stream_limit(), 256);
}

#[test]
fn wide_ranged_flush_on_a_small_t8110_window_selects_revision_2_0() {
    let p = Profile::T8110;
    assert_eq!(
        p.for_device_window(0x4000, true, 1 << 40),
        Some(Profile::T8110Rev20)
    );
    assert_eq!(p.for_device_window(0x4000, false, 1 << 40), Some(p));
    assert_eq!(p.for_device_window(0x4000, true, 0x4000), Some(p));
    assert_eq!(
        p.for_device_window(0xc000, true, 1 << 40),
        Some(Profile::T8110Rev22)
    );

    let mut sink = InterruptLatch::new();
    let mut access = Access::new(0, 0, &mut sink);
    let mut dart = Dart::for_profile(Profile::T8110Rev20, 16, None).unwrap();
    assert_eq!(
        dart.read(8, Width::Word, &mut access),
        Ok(42 << 24 | 42 << 16 | 0x0200)
    );
    assert!(dart.read(0x4060, Width::Word, &mut access).is_err());
}

#[test]
fn a_t8110_translation_window_selects_its_register_file_revision() {
    for (profile, bytes, expected) in [
        (Profile::T8110, 0x4000, Some(Profile::T8110)),
        (Profile::T8110, 0xc000, Some(Profile::T8110Rev22)),
        (Profile::T8110, 0x20000, Some(Profile::T8110Rev22)),
        (Profile::T8110Rev22, 0x4000, Some(Profile::T8110)),
        (Profile::T8110, 0x8000, None),
        (Profile::T8110, 0xd000, None),
        (Profile::T8110, 0x24000, None),
        (Profile::T8110, 0, None),
        (Profile::T6000, 0x4000, Some(Profile::T6000)),
        (Profile::T6000, 0xc000, None),
        (Profile::T8020, 0x20000, None),
    ] {
        assert_eq!(
            profile.for_translation_window(bytes),
            expected,
            "{profile:?} {bytes:#x}"
        );
    }
    assert!(Profile::T8110Rev22.is_t8110() && !Profile::T6000.is_t8110());
    // The internal filter and capture views lie above every window.
    assert!(!Profile::T8110Rev22.admits_window(DAPF_OFFSET + DAPF_BYTES));
    assert!(Profile::T8110Rev22.admits_window(DAPF_OFFSET));
    // Revision 2.2 constructs its own register file, alone or grouped.
    let mut sink = InterruptLatch::new();
    let mut a = Access::new(0, 0, &mut sink);
    let mut models = Dart::group_for_profile(Profile::T8110Rev22, 16, None, 2).unwrap();
    models.push(Dart::for_profile(Profile::T8110Rev22, 16, None).unwrap());
    for model in &mut models {
        assert_eq!(model.read(8, Width::Word, &mut a), Ok(0x2a2a_0202));
        assert!(model.read(0x100, Width::Word, &mut a).is_err());
    }
}
