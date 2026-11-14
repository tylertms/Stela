use super::*;

fn processor(number: u32, package: u32, core: u32, thread: u32) -> Processor {
    Processor {
        processor_number: number,
        apic_id: u64::from(number) + 1000,
        flags: ENABLED | HEALTHY,
        topology_version: 2,
        location: Location {
            package,
            core,
            ..Location::default()
        },
        thread,
    }
}

fn classified_snapshot(vendor: [u32; 3], maximum: u32, class: Cpuid) -> Snapshot {
    let mut snapshot = Snapshot::default();
    snapshot.records[0] = Cpuid {
        leaf: 0,
        eax: maximum,
        ebx: vendor[0],
        edx: vendor[1],
        ecx: vendor[2],
        ..Cpuid::default()
    };
    snapshot.records[1] = Cpuid {
        leaf: 0x8000_0000,
        eax: 0x8000_0026,
        ..Cpuid::default()
    };
    snapshot.records[2] = class;
    snapshot.count = 3;
    snapshot
}

#[test]
fn guest_assignment_fits_source_and_preserves_explicit_requests() {
    for available in [1, 8, 64, 128] {
        assert_eq!(
            guest_assignment_count(None, available, 8),
            Ok(available.min(8))
        );
    }
    assert_eq!(guest_assignment_count(Some(4), 64, 8), Ok(4));
    for (request, available, source) in [
        (Some(9), 64, 8),
        (Some(4), 2, 8),
        (Some(0), 64, 8),
        (None, 64, 0),
        (None, 0, 8),
    ] {
        assert!(guest_assignment_count(request, available, source).is_err());
    }
}

#[test]
fn intel_core_classes_require_a_supported_known_type() {
    use stela_compiler::TargetCpu;
    for (kind, expected) in [
        (0x20, CoreClass::Efficiency),
        (0x40, CoreClass::Performance),
        (0, CoreClass::Unknown),
        (0x30, CoreClass::Unknown),
    ] {
        let class = Cpuid {
            leaf: 0x1a,
            eax: kind << 24 | 0x1234,
            ..Cpuid::default()
        };
        let mut snapshot = classified_snapshot(TargetCpu::INTEL_VENDOR, 0x1a, class);
        assert_eq!(snapshot.core_class(), expected);
        snapshot.records[0].eax = 0x19;
        assert_eq!(snapshot.core_class(), CoreClass::Unknown);
        snapshot.records[0].eax = 0x1a;
        snapshot.count = 2;
        assert_eq!(snapshot.core_class(), CoreClass::Unknown);
    }
}

#[test]
fn amd_core_classes_require_a_valid_heterogeneous_level() {
    use stela_compiler::TargetCpu;
    for (kind, expected) in [
        (0, CoreClass::Performance),
        (1, CoreClass::Efficiency),
        (2, CoreClass::LowPower),
        (3, CoreClass::Unknown),
    ] {
        let class = Cpuid {
            leaf: 0x8000_0026,
            eax: 1 << 30,
            ebx: kind << 28 | 2,
            ecx: 1 << 8,
            ..Cpuid::default()
        };
        let snapshot = classified_snapshot(TargetCpu::AMD_VENDOR, 7, class);
        assert_eq!(snapshot.core_class(), expected);
        for changed in [
            Cpuid { eax: 0, ..class },
            Cpuid {
                ebx: kind << 28,
                ..class
            },
            Cpuid { ecx: 0, ..class },
        ] {
            let mut invalid = snapshot;
            invalid.records[2] = changed;
            assert_eq!(invalid.core_class(), CoreClass::Unknown);
        }
        let mut invalid = snapshot;
        invalid.records[1].eax = 0x8000_0025;
        assert_eq!(invalid.core_class(), CoreClass::Unknown);
        invalid = snapshot;
        invalid.count = 2;
        assert_eq!(invalid.core_class(), CoreClass::Unknown);
    }
}

#[test]
fn unknown_vendor_or_malformed_snapshot_has_no_core_class() {
    let class = Cpuid {
        leaf: 0x1a,
        eax: 0x40 << 24,
        ..Cpuid::default()
    };
    let snapshot = classified_snapshot([0; 3], 0x1a, class);
    assert_eq!(snapshot.core_class(), CoreClass::Unknown);
    let mut malformed = snapshot;
    malformed.count = MAX_CPUID as u32 + 1;
    assert_eq!(malformed.core_class(), CoreClass::Unknown);
    assert_eq!(Snapshot::default().core_class(), CoreClass::Unknown);
}

#[test]
fn placement_reserves_matches_before_cross_class_fallback() {
    use CoreClass::{Efficiency as E, Performance as P, Unknown as U};
    assert_eq!(
        placement(&[P, E, E, P], &[P, P, P, E], 0),
        Ok(vec![0, 3, 2, 1])
    );
    assert_eq!(
        placement(&[E, P, E, P], &[P, E, P, E], 1),
        Ok(vec![1, 0, 3, 2])
    );
    assert_eq!(
        placement(&[P, E, E, P], &[P, P, U, E], 0),
        Ok(vec![0, 3, 2, 1])
    );
}

#[test]
fn placement_prefers_efficiency_before_low_power() {
    use CoreClass::{Efficiency as E, LowPower as L, Performance as P};
    assert_eq!(
        placement(&[P, E, E, P], &[P, L, E, P], 0),
        Ok(vec![0, 2, 1, 3])
    );
    assert_eq!(
        placement(&[P, E, L, P], &[P, L, E, P], 0),
        Ok(vec![0, 2, 1, 3])
    );
}

#[test]
fn homogeneous_or_unknown_hosts_keep_bsp_first_order() {
    use CoreClass::{Efficiency as E, Performance as P, Unknown as U};
    for class in [P, E, U] {
        for boot in 0..4 {
            let result = placement(&[E, P, E, P], &[class; 4], boot).unwrap();
            assert_eq!(result[boot], 0);
            assert_eq!(
                result
                    .iter()
                    .copied()
                    .filter(|&worker| worker != 0)
                    .collect::<Vec<_>>(),
                vec![1, 2, 3]
            );
        }
    }
}

#[test]
fn placement_is_a_permutation_with_the_boot_worker_fixed() {
    use CoreClass::{Efficiency, LowPower, Performance, Unknown};
    let classes = [Unknown, Performance, Efficiency, LowPower];
    for guest_bits in 0..64 {
        let guest = core::array::from_fn::<_, 3, _>(|i| classes[guest_bits >> (i * 2) & 3]);
        for host_bits in 0..64 {
            let host = core::array::from_fn::<_, 3, _>(|i| classes[host_bits >> (i * 2) & 3]);
            for boot in 0..3 {
                let mut result = placement(&guest, &host, boot).unwrap();
                assert_eq!(result[boot], 0);
                result.sort_unstable();
                assert_eq!(result, vec![0, 1, 2]);
            }
        }
    }
}

#[test]
fn placement_searches_all_kept_workers_when_guest_has_fewer_slots() {
    use CoreClass::{Efficiency as E, Performance as P, Unknown as U};
    assert_eq!(placement(&[P, E], &[P, P, P, E], 0), Ok(vec![0, 3]));
    assert_eq!(placement(&[E, P], &[P, P, P, E], 1), Ok(vec![3, 0]));
    assert_eq!(placement(&[E], &[P, P, P, E], 0), Ok(vec![0]));
    assert_eq!(placement(&[P, E], &[U, U, U, U], 0), Ok(vec![0, 1]));
    let classes = [U, P, E, CoreClass::LowPower];
    for guest_bits in 0..16 {
        let guest = core::array::from_fn::<_, 2, _>(|i| classes[guest_bits >> (i * 2) & 3]);
        for host_bits in 0..256 {
            let host = core::array::from_fn::<_, 4, _>(|i| classes[host_bits >> (i * 2) & 3]);
            for boot in 0..2 {
                let result = placement(&guest, &host, boot).unwrap();
                assert_eq!(result[boot], 0);
                assert_ne!(result[0], result[1]);
                assert!(result.iter().all(|worker| *worker < host.len()));
            }
        }
    }
}

#[test]
fn placement_rejects_invalid_rosters() {
    use CoreClass::Unknown as U;
    assert_eq!(placement(&[], &[], 0), Err(PlacementError::Cardinality));
    assert_eq!(
        placement(&[U, U], &[U], 0),
        Err(PlacementError::Cardinality)
    );
    assert_eq!(placement(&[U], &[U], 1), Err(PlacementError::Bootstrap));
    assert_eq!(
        placement(&[U; MAX_CORES + 1], &[U; MAX_CORES + 1], 0),
        Err(PlacementError::Capacity)
    );
}

#[test]
fn default_assignment_uses_all_cores_and_only_explicit_counts_narrow_it() {
    for available in 1..=MAX_CORES {
        assert_eq!(assignment_count(None, available), Ok(available));
        for selected in 1..=available {
            assert_eq!(assignment_count(Some(selected), available), Ok(selected));
        }
        assert!(assignment_count(Some(0), available).is_err());
        assert!(assignment_count(Some(available + 1), available).is_err());
    }
    assert!(assignment_count(None, 0).is_err());
}

#[test]
fn default_assignment_caps_large_hosts_and_explicit_over_requests_refuse() {
    for available in [MAX_CORES + 1, 96, 192, 512] {
        assert_eq!(assignment_count(None, available), Ok(MAX_CORES));
        assert_eq!(assignment_count(Some(8), available), Ok(8));
        assert_eq!(assignment_count(Some(MAX_CORES), available), Ok(MAX_CORES));
        assert!(assignment_count(Some(MAX_CORES + 1), available).is_err());
    }
}

/// `packages` x `cores` x `threads` logical processors, BSP on the last
/// thread of the last core of package 0.
fn large_host(packages: u32, cores: u32, threads: u32) -> Vec<Processor> {
    let mut source = Vec::new();
    for package in 0..packages {
        for core in 0..cores {
            for thread in 0..threads {
                let number = source.len() as u32;
                source.push(processor(number, package, core, thread));
            }
        }
    }
    let bsp = (cores * threads - 1) as usize;
    source[bsp].flags |= BSP;
    source
}

#[test]
fn large_hosts_select_every_physical_core_before_siblings_and_retire_the_rest() {
    // Dual-socket 96-core SMT host: 384 logical processors, 192 cores.
    let source = large_host(2, 96, 2);
    let all = select(&source).unwrap();
    assert_eq!(all.len(), 384);
    assert_eq!(physical_cores(&source), 192);
    assert_eq!(all[0], 191); // BSP first, even on thread 1
    assert!(all[1..192].iter().all(|&index| source[index].thread == 0));
    // Siblings follow in location order; the BSP's sibling is thread 0.
    assert!(
        all[192..]
            .iter()
            .all(|&index| source[index].thread == 1 || index == 190)
    );
    assert!(all[192..].contains(&190));
    let assigned = assignment_count(None, all.len()).unwrap();
    assert_eq!(assigned, MAX_CORES);
    // The assigned prefix stays on package 0 before using package 1.
    assert!(
        all[..assigned]
            .iter()
            .all(|&index| source[index].location.package == 0)
    );
    let selected: Vec<_> = all[..assigned].iter().map(|&index| index as u32).collect();
    let retired: Vec<_> = enabled_unselected(&source, &selected).unwrap().collect();
    assert_eq!(retired.len(), source.len() - MAX_CORES);
    for &index in &all[assigned..] {
        assert!(retired.contains(&index)); // unassigned processors get INIT
    }
    assert!(!retired.contains(&191));
}

#[test]
fn processor_inventory_has_no_fixed_logical_ceiling() {
    let source = large_host(4, 80, 2); // 640 logical processors
    assert_eq!(select(&source).map(|all| all.len()), Ok(640));
    let source = large_host(1, 1, 300); // one core, 300 threads
    let all = select(&source).unwrap();
    assert_eq!((all.len(), all[0]), (300, 299));
    assert_eq!(physical_cores(&source), 1);
}

/// PEs a host gives a guest whose profile allows `cap` CPUs.
fn pes(source: &[Processor], cap: usize) -> Vec<usize> {
    let all = select(source).unwrap();
    let count = guest_assignment_count(None, all.len(), cap).unwrap();
    all[..count].to_vec()
}

#[test]
fn smt_siblings_run_pes_after_every_physical_core() {
    // 2 cores / 4 threads, BSP on core 0 thread 0.
    let mut laptop = large_host(1, 2, 2);
    laptop[3].flags &= !BSP;
    laptop[0].flags |= BSP;
    assert_eq!(pes(&laptop, 8), [0, 2, 1, 3]);
    assert_eq!(pes(&laptop, 2), [0, 2]);
    // 8 cores / 16 threads: a cap of 8 takes the 8 leaders, 24 takes all 16.
    let mut desktop = large_host(1, 8, 2);
    desktop[15].flags &= !BSP;
    desktop[0].flags |= BSP;
    let leaders: Vec<_> = (0..8).map(|core| core * 2).collect();
    assert_eq!(pes(&desktop, 8), leaders);
    let siblings: Vec<_> = (0..8).map(|core| core * 2 + 1).collect();
    assert_eq!(pes(&desktop, 24), [leaders.clone(), siblings].concat());
    // A retired or unhealthy sibling is skipped.
    desktop[3].flags = HEALTHY;
    desktop[5].flags = ENABLED;
    let all = pes(&desktop, 24);
    assert_eq!(all.len(), 14);
    assert!(!all.contains(&3) && !all.contains(&5));
    assert_eq!(all[..8], leaders[..]);
    // A sibling assignment is a valid INIT boundary.
    let selected: Vec<_> = all.iter().map(|&index| index as u32).collect();
    let retired: Vec<_> = enabled_unselected(&desktop, &selected).unwrap().collect();
    assert_eq!(retired, [5]);
}

#[test]
fn bsp_leads_and_each_core_is_ordered_by_thread() {
    let mut cores = [
        processor(3, 0, 0, 1),
        processor(8, 0, 0, 0),
        processor(12, 0, 1, 1),
        processor(17, 0, 1, 0),
        processor(22, 1, 0, 0),
    ];
    cores[0].flags |= BSP;
    assert_eq!(select(&cores), Ok(alloc::vec![0, 3, 4, 1, 2]));
    assert_eq!(physical_cores(&cores), 3);
    // Extended die/module/tile coordinates distinguish physical cores.
    cores[4].location.package = 0;
    cores[4].location.die = 1;
    assert_eq!(select(&cores), Ok(alloc::vec![0, 3, 4, 1, 2]));
}

#[test]
fn unselected_enabled_threads_and_unrequested_cores_need_init_without_identity_assumptions() {
    let mut source = [
        processor(41, 3, 9, 0),
        processor(77, 3, 9, 3),
        processor(12, 3, 2, 0),
        processor(93, 3, 2, 1),
        processor(3, 6, 1, 0),
        processor(66, 6, 1, 1),
        processor(51, 6, 8, 0),
        processor(24, 6, 8, 1),
    ];
    source[1].flags |= BSP; // BSP is neither first nor thread zero.
    source[6].flags = HEALTHY; // disabled: no INIT target
    source[7].flags = ENABLED; // unhealthy but still enabled: INIT target
    let all = select(&source).unwrap();
    assert_eq!(all, [1, 2, 4, 3, 0, 5]);
    for requested in 1..=3 {
        let selected: Vec<_> = all[..requested].iter().map(|&index| index as u32).collect();
        let actual: Vec<_> = enabled_unselected(&source, &selected).unwrap().collect();
        let expected: Vec<_> = (0..source.len())
            .filter(|index| *index != 6 && !all[..requested].contains(index))
            .collect();
        assert_eq!(actual, expected);
        assert!(actual.contains(&0)); // BSP's enabled sibling
        assert!(actual.contains(&7)); // enabled, unhealthy AP
        assert!(!actual.contains(&1)); // never INIT the BSP
    }
}

#[test]
fn malformed_quiescence_assignment_refuses_before_any_targets_are_exposed() {
    let mut source = [
        processor(9, 0, 0, 0),
        processor(5, 0, 0, 1),
        processor(3, 0, 1, 0),
    ];
    source[0].flags |= BSP;
    // A sibling of the BSP (index 1) is a valid assignment.
    assert!(enabled_unselected(&source, &[0, 1]).is_ok());
    for selected in [&[][..], &[1][..], &[0, 0][..], &[0, 1, 1][..], &[0, 3][..]] {
        assert!(matches!(
            enabled_unselected(&source, selected),
            Err(Refusal::InvalidAssignment)
        ));
    }
    source[2].flags = ENABLED;
    assert!(matches!(
        enabled_unselected(&source, &[0, 2]),
        Err(Refusal::InvalidAssignment)
    ));
    source[2].apic_id = source[0].apic_id;
    assert!(matches!(
        enabled_unselected(&source, &[0]),
        Err(Refusal::DuplicateApicId)
    ));
}

#[test]
fn disabled_and_unhealthy_aps_cannot_supply_guest_capacity() {
    let mut cores = [
        processor(0, 0, 0, 0),
        processor(1, 0, 1, 0),
        processor(2, 0, 2, 0),
    ];
    cores[0].flags |= BSP;
    cores[1].flags = HEALTHY;
    cores[2].flags = ENABLED;
    assert_eq!(select(&cores), Ok(alloc::vec![0]));
    // The health bit is advisory for the bootstrap processor alone.
    cores[0].flags = BSP | ENABLED;
    assert_eq!(select(&cores), Ok(alloc::vec![0]));
    cores[0].flags = BSP | HEALTHY;
    assert_eq!(select(&cores), Err(Refusal::BootstrapUnavailable));
}

#[test]
fn invalid_and_ambiguous_source_identity_refuses_instead_of_oversubscribing() {
    let mut cores = [processor(0, 0, 0, 0), processor(1, 0, 1, 0)];
    assert_eq!(select(&cores), Err(Refusal::NoBootstrapProcessor));
    cores[0].flags |= BSP;
    cores[1].flags |= BSP;
    assert_eq!(select(&cores), Err(Refusal::MultipleBootstrapProcessors));
    cores[1].flags &= !BSP;
    cores[1].apic_id = cores[0].apic_id;
    assert_eq!(select(&cores), Err(Refusal::DuplicateApicId));
    cores[1].apic_id = 5;
    cores[1].processor_number = 0;
    assert_eq!(select(&cores), Err(Refusal::DuplicateProcessorNumber));
    cores[1].processor_number = 1;
    cores[1].topology_version = 1;
    assert_eq!(select(&cores), Err(Refusal::MixedTopologyVersions));
    cores[1].topology_version = 0;
    assert_eq!(select(&cores), Err(Refusal::InvalidTopology));
}

#[test]
fn extended_apic_identity_is_not_truncated_to_the_legacy_smt_field() {
    let mut snapshot = Snapshot::default();
    snapshot.records[0] = Cpuid {
        leaf: 1,
        ebx: 7 << 24,
        ..Cpuid::default()
    };
    snapshot.records[1] = Cpuid {
        leaf: 0xb,
        ebx: 2,
        ecx: 1 << 8,
        edx: 0x1234,
        ..Cpuid::default()
    };
    snapshot.records[2] = Cpuid {
        leaf: 0x1f,
        ..Cpuid::default()
    };
    snapshot.records[3] = Cpuid {
        leaf: 0,
        eax: 0xb,
        ..Cpuid::default()
    };
    snapshot.count = 4;
    let mut core = processor(0, 0, 0, 0);
    core.apic_id = 0x1234;
    assert!(snapshot.matches(core));
    core.apic_id = 7;
    assert!(!snapshot.matches(core));
    snapshot.records[2] = Cpuid {
        leaf: 0x1f,
        ebx: 2,
        ecx: 1 << 8,
        edx: 0x5678,
        ..Cpuid::default()
    };
    snapshot.records[3].eax = 0x1f;
    assert_eq!(snapshot.apic_id(), Some(0x5678));
    snapshot.count = (MAX_CPUID + 1) as u32;
    assert_eq!(snapshot.apic_id(), None);
}
fn native_snapshot() -> Snapshot {
    let mut snapshot = Snapshot {
        count: 3,
        ..Snapshot::default()
    };
    snapshot.records[0] = Cpuid {
        leaf: 1,
        ecx: u32::MAX,
        edx: u32::MAX,
        ..Cpuid::default()
    };
    snapshot.records[1] = Cpuid {
        leaf: 0x8000_0001,
        ecx: 1,
        edx: (1 << 20) | (1 << 29),
        ..Cpuid::default()
    };
    snapshot.records[2] = Cpuid {
        leaf: 7,
        ecx: 1 << 16,
        ..Cpuid::default()
    };
    snapshot.pat_valid = true;
    snapshot.pat = 0x0007_0406_0007_0406;
    snapshot.apic_base = 1 << 11;
    snapshot.msr_count = 2;
    snapshot.msrs[0] = Msr {
        index: 0xfe,
        value: 8,
    };
    snapshot.msrs[1] = Msr {
        index: 0x2ff,
        value: 0xc06,
    };
    snapshot
}

fn target_snapshot(vendor: [u32; 3]) -> Snapshot {
    let mut snapshot = native_snapshot();
    snapshot.records[3] = Cpuid {
        leaf: 0,
        eax: 7,
        ebx: vendor[0],
        edx: vendor[1],
        ecx: vendor[2],
        ..Cpuid::default()
    };
    snapshot.records[2].ebx = 1 << 29;
    snapshot.count = 4;
    snapshot
}

#[test]
fn identity_selection_checks_observed_maxima_low16_counts_and_amd_vendor() {
    use stela_compiler::TargetCpu;
    let source = |vendor: [u32; 3]| {
        let mut snapshot = target_snapshot(vendor);
        snapshot.records[0].ebx = 7 << 24;
        snapshot.records[1].ecx |= 1 << 22;
        snapshot.records[3].eax = 0x1f;
        snapshot.records[4] = Cpuid {
            leaf: 0x1f,
            ..Cpuid::default()
        };
        snapshot.records[5] = Cpuid {
            leaf: 0xb,
            ..Cpuid::default()
        };
        snapshot.records[6] = Cpuid {
            leaf: 0x8000_0000,
            eax: 0x8000_001e,
            ..Cpuid::default()
        };
        snapshot.records[7] = Cpuid {
            leaf: 0x8000_001e,
            eax: 0x1234,
            ..Cpuid::default()
        };
        snapshot.count = 8;
        snapshot
    };
    for vendor in [TargetCpu::AMD_VENDOR, TargetCpu::INTEL_VENDOR, [0; 3]] {
        let mut snapshot = source(vendor);
        assert_eq!(
            snapshot.apic_id(),
            Some(if vendor == TargetCpu::AMD_VENDOR {
                0x1234
            } else {
                7
            })
        );
        snapshot.records[6].eax = 0x8000_001d;
        assert_eq!(snapshot.apic_id(), Some(7));
    }
    let mut snapshot = source(TargetCpu::INTEL_VENDOR);
    snapshot.records[4] = Cpuid {
        leaf: 0x1f,
        ebx: 1 << 16,
        ecx: 1 << 8,
        edx: 0x5678,
        ..Cpuid::default()
    };
    // Only the architectural low16 count matters; high garbage is not a
    // valid topology source and cannot replace the real legacy identity.
    assert_eq!(snapshot.apic_id(), Some(7));
    snapshot.records[5] = Cpuid {
        leaf: 0xb,
        ebx: 2,
        ecx: 1 << 8,
        edx: 0x1234,
        ..Cpuid::default()
    };
    assert_eq!(snapshot.apic_id(), Some(0x1234));
    snapshot.records[4].ebx = 2;
    assert_eq!(snapshot.apic_id(), Some(0x5678));
    snapshot.records[3].eax = 0xb;
    assert_eq!(snapshot.apic_id(), Some(0x1234));
    snapshot.records[3].eax = 1;
    assert_eq!(snapshot.apic_id(), Some(7));
    snapshot.records[3].eax = 0x1f;
    snapshot.records[4].leaf = 0x1e;
    assert_eq!(snapshot.apic_id(), None); // incomplete declared observation
    let mut amd = source(TargetCpu::AMD_VENDOR);
    amd.records[7].leaf = 0x8000_001d;
    assert_eq!(amd.apic_id(), None);
    let mut amd = source(TargetCpu::AMD_VENDOR);
    amd.records[6].leaf = 0x8000_001d;
    assert_eq!(amd.apic_id(), None); // unknown maximum cannot authorize legacy fallback
    let mut amd = source(TargetCpu::AMD_VENDOR);
    amd.records[1].leaf = 0x8000_0002;
    assert_eq!(amd.apic_id(), None); // missing topology feature observation
}

#[test]
fn common_target_keeps_only_features_present_on_every_assigned_worker() {
    use stela_compiler::TargetCpu;
    let rich = target_snapshot(TargetCpu::INTEL_VENDOR);
    let mut weaker = target_snapshot(TargetCpu::AMD_VENDOR);
    weaker.records[0].ecx &= !((1 << 1) | (1 << 25) | (1 << 28));
    weaker.records[2].ebx = 0;
    let expected = TargetCpu::Sse42
        .with_popcnt()
        .with_cx8()
        .with_cx16()
        .with_movbe();
    assert_eq!(common_native_target(&[&rich, &weaker]), Ok(expected));
    assert_eq!(common_native_target(&[&weaker, &rich]), Ok(expected));
    // Removing a worker recomputes a future cold-registration ceiling;
    // it never changes an existing RAM domain or grants a live entry.
    let solo = common_native_target(&[&rich]).unwrap();
    assert!(solo.supports(TargetCpu::Aes));
    assert!(solo.supports(TargetCpu::Sha));
    assert!(solo.supports(TargetCpu::Pclmulqdq));
    assert!(solo.supports(TargetCpu::AtomicLoad16));
    for excluded in [
        TargetCpu::Bmi1,
        TargetCpu::Fma,
        TargetCpu::F16c,
        TargetCpu::Avx2,
        TargetCpu::Avx512f,
        TargetCpu::Doit,
        TargetCpu::LineAtomic,
    ] {
        assert!(!solo.supports(excluded));
    }
}

/// FMA3, F16C and AVX2 need AVX hardware whose YMM state every installed
/// worker's XCR0 enables, as the interrupt entries save it.
#[test]
fn common_target_keeps_vex_vector_recipes_only_with_every_worker_ymm_state() {
    use stela_compiler::TargetCpu;
    const AVX2: u32 = 1 << 5;
    let mut rich = target_snapshot(TargetCpu::AMD_VENDOR);
    rich.records[2].ebx |= AVX2;
    rich.xcr0 = 0x7;
    rich.xcr0_valid = true;
    let vector = TargetCpu::Baseline.with_fma().with_f16c().with_avx2();
    assert!(common_native_target(&[&rich]).unwrap().supports(vector));
    assert!(
        !common_native_target(&[&rich])
            .unwrap()
            .supports(TargetCpu::Avx512f)
    );
    let mut legacy_state = rich;
    legacy_state.xcr0 = 0x3;
    let mut unobserved = rich;
    unobserved.xcr0_valid = false;
    let mut without_avx = rich;
    without_avx.records[0].ecx &= !(1 << 28);
    for worker in [legacy_state, unobserved, without_avx] {
        let common = common_native_target(&[&rich, &worker]).unwrap();
        for excluded in [TargetCpu::Fma, TargetCpu::F16c, TargetCpu::Avx2] {
            assert!(!common.supports(excluded));
        }
    }
    let mut without_avx2 = rich;
    without_avx2.records[2].ebx &= !AVX2;
    let common = common_native_target(&[&rich, &without_avx2]).unwrap();
    assert!(common.supports(TargetCpu::Baseline.with_fma().with_f16c()));
    assert!(!common.supports(TargetCpu::Avx2));
}

/// BMI1 and BMI2 need VEX decoding, which an Intel worker without AVX
/// lacks, and fast PDEP a Zen 3 or later AMD core on every worker.
#[test]
fn common_target_keeps_scalar_gpr_extensions_every_worker_decodes() {
    use stela_compiler::TargetCpu;
    const BMI: u32 = (1 << 3) | (1 << 8);
    let with_family = |vendor, family: u32| {
        let mut snapshot = target_snapshot(vendor);
        snapshot.records[2].ebx |= BMI;
        snapshot.records[0].eax = (0xf << 8) | (family - 0xf) << 20;
        snapshot
    };
    let zen4 = with_family(TargetCpu::AMD_VENDOR, 0x19);
    let zen2 = with_family(TargetCpu::AMD_VENDOR, 0x17);
    let intel = with_family(TargetCpu::INTEL_VENDOR, 0x10);
    let fast = common_native_target(&[&zen4, &intel]).unwrap();
    for kept in [
        TargetCpu::Bmi1,
        TargetCpu::Bmi2,
        TargetCpu::FastPdep,
        TargetCpu::Movbe,
    ] {
        assert!(fast.supports(kept));
    }
    let slow = common_native_target(&[&zen4, &zen2]).unwrap();
    assert!(slow.supports(TargetCpu::Bmi2) && !slow.supports(TargetCpu::FastPdep));
    let mut without_avx = intel;
    without_avx.records[0].ecx &= !(1 << 28);
    let none = common_native_target(&[&zen4, &without_avx]).unwrap();
    assert!(!none.supports(TargetCpu::Bmi1) && !none.supports(TargetCpu::Bmi2));
}

#[test]
fn common_target_obeys_leaf_maxima_vendor_atomicity_and_complete_sse_tiers() {
    use stela_compiler::TargetCpu;
    let rich = target_snapshot(TargetCpu::INTEL_VENDOR);
    let mut unknown = target_snapshot([0; 3]);
    unknown.records[3].eax = 1;
    // The stale structured record is outside the declared maximum.
    let target = common_native_target(&[&rich, &unknown]).unwrap();
    assert!(!target.supports(TargetCpu::Sha));
    assert!(!target.supports(TargetCpu::AtomicLoad16));
    assert!(target.supports(TargetCpu::Aes));
    for (removed, expected) in [
        (20, TargetCpu::Sse41),
        (19, TargetCpu::Ssse3),
        (9, TargetCpu::Baseline),
    ] {
        let mut worker = rich;
        worker.records[0].ecx &= !(1 << removed);
        let target = common_native_target(&[&rich, &worker]).unwrap();
        assert!(target.supports(expected));
        assert!(!target.supports(TargetCpu::Sse42));
        if removed == 9 {
            assert!(!target.supports(TargetCpu::Ssse3));
        }
        if removed == 19 {
            assert!(!target.supports(TargetCpu::Sse41));
        }
    }
    let mut missing = rich;
    missing.records[2].leaf = 6;
    assert!(
        !common_native_target(&[&rich, &missing])
            .unwrap()
            .supports(TargetCpu::Sha)
    );
    missing.count = MAX_CPUID as u32 + 1;
    assert_eq!(
        common_native_target(&[&missing]),
        Err(CapabilityRefusal::InvalidSnapshot)
    );
    assert_eq!(
        common_native_target(&[]),
        Err(CapabilityRefusal::InvalidSnapshot)
    );
    missing = rich;
    missing.records[3].eax = 0;
    assert_eq!(
        common_native_target(&[&missing]),
        Err(CapabilityRefusal::InvalidSnapshot)
    );
}

#[test]
fn common_target_keeps_legacy_extensions_independently_and_checks_both_leaf_maxima() {
    use stela_compiler::TargetCpu;
    let mut rich = target_snapshot(TargetCpu::INTEL_VENDOR);
    rich.records[0].ecx &= !((1 << 19) | (1 << 20) | (1 << 28));
    rich.records[1].ecx |= 1 << 5;
    rich.records[2].ecx |= 1 << 8;
    rich.records[4] = Cpuid {
        leaf: 0x8000_0000,
        eax: 0x8000_0001,
        ..Cpuid::default()
    };
    rich.count = 5;
    let mut older = rich;
    for (maximum, feature_lost, feature_kept) in [
        (3, TargetCpu::Gfni, TargetCpu::Lzcnt),
        (4, TargetCpu::Lzcnt, TargetCpu::Gfni),
    ] {
        older.records[maximum].eax -= 1;
        let common = common_native_target(&[&rich, &older]).unwrap();
        assert!(common.supports(TargetCpu::Ssse3));
        assert!(!common.supports(TargetCpu::Sse41));
        assert!(common.supports(feature_kept));
        assert!(!common.supports(feature_lost));
        assert!(!common.supports(TargetCpu::Fma));
        older = rich;
    }
    let common = common_native_target(&[&rich, &older]).unwrap();
    assert!(common.supports(TargetCpu::Gfni.with_lzcnt()));
    older.count -= 1;
    assert!(
        !common_native_target(&[&rich, &older])
            .unwrap()
            .supports(TargetCpu::Lzcnt)
    );
}

#[test]
fn core_floor_and_la57_are_required_independently_of_bsp() {
    let baseline = native_snapshot();
    assert_eq!(baseline.qualify_native(true), Ok(()));
    for bit in [0, 9, 13, 19, 20, 23] {
        let mut changed = baseline;
        changed.records[0].ecx &= !(1 << bit);
        assert!(matches!(
            changed.qualify_native(false),
            Err(CapabilityRefusal::NativeFloor(_))
        ));
    }
    for bit in [0, 4, 5, 6, 8, 9, 12, 16, 24, 25, 26] {
        let mut changed = baseline;
        changed.records[0].edx &= !(1 << bit);
        assert!(matches!(
            changed.qualify_native(false),
            Err(CapabilityRefusal::NativeFloor(_))
        ));
    }
    let mut changed = baseline;
    changed.records[2].ecx = 0;
    assert_eq!(changed.qualify_native(false), Ok(()));
    assert_eq!(
        changed.qualify_native(true),
        Err(CapabilityRefusal::FiveLevelPaging)
    );
    changed.msr_count = (MAX_MSRS + 1) as u32;
    assert_eq!(
        changed.qualify_native(false),
        Err(CapabilityRefusal::InvalidSnapshot)
    );
}

#[test]
fn mtrr_pat_equality_does_not_conflate_clock_adjustment() {
    let baseline = native_snapshot();
    let mut changed = baseline;
    assert_eq!(baseline.same_memory_types(&changed), Ok(()));
    changed.msrs[2] = Msr {
        index: 0x3b,
        value: 123,
    };
    changed.msr_count = 3;
    assert_eq!(baseline.same_memory_types(&changed), Ok(()));
    changed.msrs[1].value ^= 1;
    assert_eq!(
        baseline.same_memory_types(&changed),
        Err(CapabilityRefusal::MemoryTypes)
    );
    changed = baseline;
    changed.pat ^= 1 << 8;
    assert_eq!(
        baseline.same_memory_types(&changed),
        Err(CapabilityRefusal::MemoryTypes)
    );
    changed = baseline;
    changed.msr_count = 1;
    assert_eq!(
        baseline.same_memory_types(&changed),
        Err(CapabilityRefusal::MemoryTypes)
    );
}

/// The legacy hole's DRAM routing is each processor's own on AMD: two
/// processors that differ only there have the same installed memory types.
#[test]
fn amd_legacy_hole_routing_is_not_part_of_the_installed_memory_types() {
    let mut baseline = native_snapshot();
    baseline.msrs[baseline.msr_count as usize] = Msr {
        index: 0x26e,
        value: 0x1010_1010_1010_1010,
    };
    baseline.msrs[baseline.msr_count as usize + 1] = Msr {
        index: 0x258,
        value: 0x1e1e_1e1e_1e1e_1e1e,
    };
    baseline.msr_count += 2;
    let shadow = baseline.msr_count as usize - 2;
    let mut other = baseline;
    other.msrs[shadow].value = 0x1818_1818_1818_1818;
    // Masked reads say nothing of the routing: the values must be equal.
    assert_eq!(
        baseline.same_memory_types(&other),
        Err(CapabilityRefusal::MemoryTypes)
    );
    baseline.amd_fixed_raw = true;
    other.amd_fixed_raw = true;
    assert_eq!(baseline.same_memory_types(&other), Ok(()));
    other.msrs[shadow].value ^= 1; // the type is installed
    assert_eq!(
        baseline.same_memory_types(&other),
        Err(CapabilityRefusal::MemoryTypes)
    );
    other.msrs[shadow].value ^= 1;
    other.msrs[shadow + 1].value ^= 0x08; // conventional memory
    assert_eq!(
        baseline.same_memory_types(&other),
        Err(CapabilityRefusal::MemoryTypes)
    );
}

#[test]
fn duplicate_physical_thread_refuses() {
    let mut cores = [processor(0, 0, 0, 0), processor(1, 0, 0, 0)];
    cores[0].flags |= BSP;
    assert_eq!(select(&cores), Err(Refusal::DuplicateThreadLocation));
}

/// Firmware that lists every processor at one package, core and thread
/// with distinct APIC IDs: the IDs place them, by CPUID leaf 0xb's shifts
/// where the processor has them and as one core each where it does not.
#[test]
fn processors_the_firmware_puts_at_one_location_are_placed_by_apic_id() {
    let listed = || {
        let mut cores: Vec<_> = (0..6).map(|number| processor(number, 0, 0, 0)).collect();
        cores[0].flags |= BSP;
        for (core, apic) in cores.iter_mut().zip([0u64, 1, 2, 3, 0x10, 0x11]) {
            core.apic_id = apic;
        }
        cores
    };
    let mut cores = listed();
    assert!(locations_collide(&cores));
    // One thread bit, four bits of thread and core: two packages.
    place_by_apic_id(&mut cores, Some((1, 4)));
    assert!(!locations_collide(&cores));
    let placed: Vec<_> = cores
        .iter()
        .map(|core| (core.location.package, core.location.core, core.thread))
        .collect();
    assert_eq!(
        placed,
        [
            (0, 0, 0),
            (0, 0, 1),
            (0, 1, 0),
            (0, 1, 1),
            (1, 0, 0),
            (1, 0, 1)
        ]
    );
    assert_eq!(physical_cores(&cores), 3);
    // The BSP, then one thread of each other core, then the second threads.
    assert_eq!(select(&cores), Ok(vec![0, 2, 4, 1, 3, 5]));

    for shifts in [None, Some((5, 4)), Some((0, 32))] {
        let mut cores = listed();
        place_by_apic_id(&mut cores, shifts);
        assert_eq!(physical_cores(&cores), 6, "{shifts:?}");
        assert_eq!(select(&cores), Ok(vec![0, 1, 2, 3, 4, 5]));
    }
    // A duplicate APIC ID is not a location fault and is not placed over.
    let mut cores = listed();
    cores[1].apic_id = 0;
    assert!(!locations_collide(&cores));
    assert_eq!(select(&cores), Err(Refusal::DuplicateApicId));
}

#[test]
fn an_amd_processor_without_leaf_0xb_gives_its_shifts_from_the_extended_leaves() {
    // Ryzen 5 2400G: 4 cores of 2 threads, ApicIdCoreIdSize 4, NC 7.
    assert_eq!(amd_apic_id_shifts(4 << 12 | 7, 1 << 8), (1, 4));
    // No ApicIdCoreIdSize: the processor count, 8, gives 3 bits.
    assert_eq!(amd_apic_id_shifts(7, 1 << 8), (1, 3));
    // One thread a core, 6 cores: no thread bit, 3 bits of core.
    assert_eq!(amd_apic_id_shifts(5, 0), (0, 3));
    // The two threads of a core then share it and its package.
    let mut cores: Vec<_> = (0..8).map(|n| processor(n, 0, 0, 0)).collect();
    for (index, core) in cores.iter_mut().enumerate() {
        core.apic_id = index as u64;
    }
    place_by_apic_id(&mut cores, Some(amd_apic_id_shifts(4 << 12 | 7, 1 << 8)));
    assert_eq!(physical_cores(&cores), 4);
    assert!(cores.iter().all(|core| core.location.package == 0));
}

#[test]
fn a_missing_floor_names_each_feature() {
    let baseline = native_snapshot();
    let refusal = |changed: Snapshot| match changed.qualify_native(false) {
        Err(CapabilityRefusal::NativeFloor(missing)) => missing,
        other => panic!("{other:?}"),
    };
    // A Core 2 without SSE4.2 and POPCNT.
    let mut changed = baseline;
    changed.records[0].ecx &= !((1 << 20) | (1 << 23));
    let missing = refusal(changed);
    assert_eq!(missing.names().collect::<Vec<_>>(), ["SSE4.2", "POPCNT"]);
    assert_eq!(
        missing.refusal().to_string(),
        "this PC's CPU lacks SSE4.2 and POPCNT; Stela needs an x86-64-v2 CPU (Intel Nehalem, \
         AMD Bulldozer or newer)"
    );
    for (field, bit, name) in [(0, 0, "SSE3"), (0, 9, "SSSE3"), (0, 13, "CX16")] {
        let mut changed = baseline;
        changed.records[field].ecx &= !(1 << bit);
        assert_eq!(refusal(changed).names().collect::<Vec<_>>(), [name]);
    }
    let mut changed = baseline;
    changed.pat_valid = false;
    changed.records[0].edx &= !(1 << 25);
    assert_eq!(
        refusal(changed).refusal().to_string(),
        "this PC's CPU lacks SSE and a valid PAT setting; Stela needs an x86-64-v2 CPU (Intel Nehalem, AMD \
         Bulldozer or newer)"
    );
}

#[test]
fn an_unhealthy_bootstrap_processor_still_leads_and_unhealthy_aps_run_no_pe() {
    let mut processors = [
        processor(0, 0, 0, 0),
        processor(1, 0, 1, 0),
        processor(2, 0, 2, 0),
    ];
    processors[0].flags = BSP | ENABLED;
    processors[2].flags = ENABLED;
    assert_eq!(select(&processors), Ok(alloc::vec![0, 1]));
    processors[0].flags = BSP;
    assert_eq!(select(&processors), Err(Refusal::BootstrapUnavailable));
}

#[test]
fn an_ap_that_cannot_be_observed_is_left_out_and_its_sibling_takes_its_place() {
    // Two cores of two threads; the first core's second thread fails.
    let mut processors = [
        processor(0, 0, 0, 0),
        processor(1, 0, 0, 1),
        processor(2, 0, 1, 0),
        processor(3, 0, 1, 1),
    ];
    processors[0].flags |= BSP;
    let mut calls = alloc::vec::Vec::new();
    let selected = select_observed(
        &mut processors,
        |slot, index, _| {
            calls.push((slot, index));
            Ok::<_, Refusal>(index != 2)
        },
        |refusal| refusal,
    )
    .unwrap();
    assert_eq!(selected, [0, 3, 1]);
    // Each processor is observed once; the one left out is unavailable now,
    // so the inventory selects the same order again.
    assert_eq!(calls, [(0, 0), (1, 2), (2, 1), (3, 3)]);
    assert!(!processors[2].available());
    assert_eq!(select(&processors), Ok(selected));
    // The bootstrap processor cannot be left out, and errors stop.
    let mut fresh = [processor(0, 0, 0, 0), processor(1, 0, 1, 0)];
    fresh[0].flags |= BSP;
    assert_eq!(
        select_observed(&mut fresh, |_, index, _| Ok(index != 0), |refusal| refusal),
        Err(Refusal::BootstrapUnavailable)
    );
    assert_eq!(
        select_observed(
            &mut fresh,
            |_, _, _| Err(Refusal::InvalidAssignment),
            |refusal| refusal
        ),
        Err(Refusal::InvalidAssignment)
    );
}
