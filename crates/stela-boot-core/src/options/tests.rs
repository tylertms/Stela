use super::*;

#[test]
fn audio_hpet_handoff_requires_an_audio_diagnostic() {
    assert_eq!(
        System::parse("--system --audio-own-hpet").unwrap_err(),
        Error::Constraint(Constraint::AudioHpetDependency)
    );
}
#[test]
fn native_pci_ioapic_handoff_round_trips_and_refuses_diagnostic_paths() {
    let text = "--system --native-pci-own-ioapic";
    let options = System::parse(text).unwrap();
    assert!(options.native_pci_own_ioapic);
    assert_eq!(System::parse(&options.to_string()), Ok(options));
    assert_eq!(
        System::parse("--system --native-pci-own-ioapic --native-pci-own-ioapic").unwrap_err(),
        Error::DuplicateNativePciIoapic
    );
    assert_eq!(
        System::parse("--system --native-pci-own-ioapic --audio-check duplex").unwrap_err(),
        Error::Constraint(Constraint::NativePciIoapicExclusive)
    );
}

use alloc::{format, string::ToString, vec};

#[test]
fn diagnostic_options_are_typed() {
    for text in ["--system", "--system --wifi --audio-own-hpet"] {
        assert_eq!(System::parse(text).unwrap().diagnostic_option(), None);
    }
    for (text, flag) in [
        (
            "--system --native-cores 2 --native-bootstrap-check",
            "--native-bootstrap-check",
        ),
        (
            "--system --native-cores 2 --native-shared-check",
            "--native-shared-check",
        ),
    ] {
        assert_eq!(System::parse(text).unwrap().diagnostic_option(), Some(flag));
    }
}

#[test]
fn host_check_lines_round_trip_and_refuse_bad_values() {
    let line = "--host-check \
                --nor-file 11111111-2222-3333-4444-555555555555:/stela/nor.bin --deadline 30 \
                --key-seconds 5";
    let options = HostCheck::parse(line).unwrap();
    assert_eq!(options.deadline, Some(30));
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    assert_eq!(HostCheck::parse("--host-check"), Ok(HostCheck::default()));
    for text in [
        "--system",
        "--host-check --native-dma-check",
        "--host-check --deadline 0",
        "--host-check --nor-file x",
        "--host-check --unknown",
    ] {
        assert!(HostCheck::parse(text).is_err(), "{text}");
    }
}

const DISK: &str = "74EXN0LI:11111111-2222-3333-4444-555555555555";

fn round_trip(text: &str) -> System {
    let options = System::parse(text).unwrap();
    let line = options.to_string();
    assert_eq!(System::parse(&line), Ok(options.clone()), "{line}");
    options
}

#[test]
fn single_user_needs_a_preboot_selection() {
    let selected = round_trip(&format!("--system --preboot {DISK} {SINGLE_USER}"));
    assert!(selected.single_user);
    assert!(System::parse(&format!("--system {SINGLE_USER}")).is_err());
}

#[test]
fn a_bundle_repair_is_a_preboot_macos_boot() {
    use stela_loader::boot_selection::BUNDLE_REPAIR_OPTION;
    let repair = round_trip(&format!("--system --preboot {DISK} {BUNDLE_REPAIR_OPTION}"));
    assert!(repair.bundle_repair);
    assert!(!round_trip(&format!("--system --preboot {DISK}")).bundle_repair);
    assert_eq!(
        System::parse(&format!("--system {BUNDLE_REPAIR_OPTION}")),
        Err(Error::Constraint(Constraint::PrebootSelection))
    );
    assert_eq!(
        System::parse(&format!(
            "--system --preboot {DISK} {BUNDLE_REPAIR_OPTION} --upgrade"
        )),
        Err(Error::Constraint(Constraint::BundleRepairSelection))
    );
}

#[test]
fn the_installed_macos_never_sees_the_agx_model_and_other_boots_keep_it_without_a_gpu() {
    use stela_loader::boot_selection::{
        BUNDLE_REPAIR_OPTION, PAIRED_RECOVERY_OPTION, UPGRADE_OPTION,
    };
    // A single-user or repair boot continues without a GPU: no Metal device
    // rather than the model, whose work never completes.
    for flag in ["", SINGLE_USER, BUNDLE_REPAIR_OPTION] {
        let installed = round_trip(&format!("--system --preboot {DISK} {flag}"));
        assert!(installed.hides_agx(false), "{flag}");
        assert!(installed.hides_agx(true), "{flag}");
    }
    // The installer's restore, recoveryOS and a staged update open no host
    // GPU and keep the model.
    for line in [
        String::from("--system"),
        format!("--system --preboot {DISK} {PAIRED_RECOVERY_OPTION}"),
        format!("--system --preboot {DISK} {UPGRADE_OPTION}"),
    ] {
        let other = round_trip(&line);
        assert!(!other.hides_agx(false), "{line}");
        assert!(other.hides_agx(true), "{line}");
    }
}

#[test]
fn scanout_mirror_round_trips() {
    assert!(!round_trip("--system").scanout_mirror);
    assert!(round_trip(&format!("--system {SCANOUT_MIRROR}")).scanout_mirror);
}

#[test]
fn maintenance_writes_require_an_explicit_preboot_selection() {
    let ordinary = round_trip(&format!("--system --preboot {DISK}"));
    assert!(!ordinary.maintenance_write);
    assert!(!ordinary.to_string().contains(MAINTENANCE_WRITE));
    let maintenance = round_trip(&format!("--system --preboot {DISK} {MAINTENANCE_WRITE}"));
    assert!(maintenance.maintenance_write);
    assert_eq!(
        System::parse(&format!("--system {MAINTENANCE_WRITE}")),
        Err(Error::Constraint(Constraint::PrebootSelection))
    );
}

#[test]
fn setup_lines_parse_and_serialize_identically() {
    let preboot = format!(
        "--system --storage {DISK} --nor {DISK} --preboot {DISK} --ecid 0x0000000000001234 \
         --external-disks --guest-network"
    );
    let options = round_trip(&preboot);
    assert!(options.bundle && options.external_disks);
    assert_eq!(options.ecid, Some(0x1234));
    assert_eq!(options.to_string(), preboot);
    let bundle = format!("--system --storage {DISK} --nor {DISK} --bundle \\EFI\\stela\\r");
    assert_eq!(round_trip(&bundle).to_string(), bundle);
}

#[test]
fn chosen_disk_round_trips_and_the_mount_refuses_it_unreplaced() {
    let line = "--system --storage @disk --nor @disk --preboot @disk";
    let options = round_trip(line);
    assert!(options.storage.as_ref().is_some_and(is_chosen));
    assert_eq!(options.to_string(), line);
    assert!(System::parse_resolved(line).is_err());
    let resolved = line.replace("@disk", DISK);
    assert!(System::parse_resolved(&resolved).is_ok());
}

#[test]
fn every_option_survives_a_round_trip() {
    let options = round_trip(&format!(
        "--system --storage {DISK} --nor {DISK} --bundle b --install p.json \
         --install-source 11111111-2222-3333-4444-555555555555 --verify-on-read \
         --usb-storage A --usb-storage B --input --idle-skip --sep-observe --native-cores 3 \
         --native-artifacts --serialized --profile --monitor-trace --display-trace \
         --network-check http://h/p --audio-check transport-duplex --audio-own-hpet --dram-base 0x10000000000 \
         --load 0x10000004000 --entry 0x10000008000 --x0 7 --budget 99 --break 0x10 \
         --trace 0x20 --watch 0x30,0x40 --dump x21-0x8,0x10 --dump 0x50,0x60"
    ));
    assert_eq!(options.usb_storage, ["A", "B"]);
    assert_eq!(options.audio_check, Some(AudioCheck::TransportDuplex));
    assert!(options.audio_own_hpet);
    assert!(options.display_trace);
    assert!(!System::default().display_trace);
    assert!(!options.parallel);
    assert_eq!(options.dumps[0], (Base::Register(21, -8), 0x10));
    let restore = round_trip(&format!(
        "--system --bundle b --storage {DISK} --restore-study --restore-source {DISK} \
         --restore-manifest m.plist --restore-identity 2"
    ));
    assert_eq!(restore.restore_manifest_path.as_deref(), Some("m.plist"));
}

#[test]
fn usb4_approvals_repeat_and_keep_their_canonical_spelling() {
    let first = "446ab8f0-5aff-47c9-8962-5870641e4d61";
    let second = "e254fadb-773c-43c3-b00d-4170323c36f8";
    let options = round_trip(&format!(
        "--system --usb4-approve {} --usb4-approve {second}",
        first.to_uppercase()
    ));
    assert_eq!(options.usb4_approved.len(), 2);
    assert_eq!(
        options.to_string(),
        format!("--system --usb4-approve {first} --usb4-approve {second}")
    );
    for (text, error) in [
        ("--system --usb4-approve", Error::Usb4Approve),
        ("--system --usb4-approve nonsense", Error::Usb4Approve),
    ] {
        assert_eq!(System::parse(text).err(), Some(error), "{text}");
    }
    assert_eq!(
        System::parse(&format!(
            "--system --usb4-approve {first} --usb4-approve {first}"
        ))
        .err(),
        Some(Error::DuplicateUsb4Approve)
    );
}

#[test]
fn wifi_names_escape_spaces_and_the_key_path_defaults() {
    let options = round_trip("--system --wifi Home%20Net%25 --wifi-country gb");
    assert_eq!(options.ssid.as_deref(), Some("Home Net%"));
    assert_eq!(options.wifi_key.as_deref(), Some(DEFAULT_WIFI_KEY));
    assert_eq!(options.wifi_country.as_deref(), Some("GB"));
    assert_eq!(
        options.to_string(),
        "--system --wifi Home%20Net%25 --wifi-country GB"
    );
    let keyed = round_trip("--system --wifi caf\u{e9} --wifi-key \\EFI\\k");
    assert_eq!(keyed.ssid.as_deref(), Some("caf\u{e9}"));
    assert_eq!(keyed.wifi_key.as_deref(), Some("\\EFI\\k"));
    for text in [
        "--system --wifi-key \\k",
        "--system --wifi-country GB",
        "--system --wifi %",
        "--system --wifi %4",
        "--system --wifi %FF",
        "--system --wifi 0123456789abcdef0123456789abcdef0",
        "--system --wifi a --wifi b",
        "--system --wifi a --wifi-country GBR",
        "--system --wifi a --wifi-country 12",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
}

#[test]
fn static_ipv4_and_dns_round_trip_and_stay_in_their_subnet() {
    let text = "--system --ipv4 192.168.1.20/24 --ipv4-gateway 192.168.1.1 --dns 1.1.1.1,2606:4700:4700::1111";
    let options = round_trip(text);
    assert_eq!(
        options.ipv4,
        Some(StaticIpv4 {
            address: core::net::Ipv4Addr::new(192, 168, 1, 20),
            prefix: 24
        })
    );
    assert_eq!(options.dns.len(), 2);
    assert_eq!(options.to_string(), text);
    assert_eq!(
        round_trip("--system --ipv4 10.0.0.1/31").to_string(),
        "--system --ipv4 10.0.0.1/31"
    );
    for text in [
        "--system --ipv4 192.168.1.20",
        "--system --ipv4 192.168.1.20/33",
        "--system --ipv4 192.168.1.0/24",
        "--system --ipv4 192.168.1.255/24",
        "--system --ipv4 224.0.0.1/4",
        "--system --ipv4 10.0.0.2/8 --ipv4 10.0.0.3/8",
        "--system --ipv4-gateway 192.168.1.1",
        "--system --ipv4 192.168.1.20/24 --ipv4-gateway 192.168.2.1",
        "--system --ipv4 192.168.1.20/24 --ipv4-gateway 192.168.1.20",
        "--system --dns 1.1.1.1,1.1.1.1",
        "--system --dns 1.1.1.1,1.0.0.1,9.9.9.9,8.8.8.8",
        "--system --dns 0.0.0.0",
        "--system --dns 1.1.1.1 --dns 9.9.9.9",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
}

#[test]
fn native_checks_imply_serialized_execution_without_writing_it() {
    let options = round_trip("--system --native-cores 2 --native-rounds");
    assert!(!options.parallel);
    assert!(!options.to_string().contains("--serialized"));
    assert!(System::parse("--system").unwrap().parallel);
}

#[test]
fn native_core_counts_refuse_duplicates_zero_and_over_capacity() {
    for text in [
        "--system --native-cores 1 --native-cores 1",
        "--system --native-cores 0",
        "--system --native-cores 65",
        "--system --native-cores x",
        "--system --native-cores",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
    assert_eq!(
        System::parse("--system --native-cores 64")
            .unwrap()
            .native_cores,
        Some(64)
    );
}

#[test]
fn guest_pes_bound_the_executors_that_run_them() {
    for text in [
        "--system --guest-pes",
        "--system --guest-pes 0",
        "--system --guest-pes 8 --guest-pes 8",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
    let options = System::parse(&format!("--system --preboot {DISK} --guest-pes 8")).unwrap();
    assert_eq!(options.guest_pes, Some(8));
    assert!(options.to_string().contains("--guest-pes 8"));
    assert_eq!(options.assigned_native_cores(64, None), Ok(8));
    assert_eq!(options.assigned_native_cores(2, None), Ok(2));
    assert_eq!(options.assigned_native_cores(64, Some(4)), Ok(4));
}

#[test]
fn preboot_checks_the_live_count_before_starting_aps() {
    let preboot = format!("--system --preboot {DISK} --guest-pes 8");
    let options = System::parse(&preboot).unwrap();
    assert_eq!(options.validate_native_cores(64, 8, 8), Ok(()));
    assert_eq!(options.validate_native_cores(2, 2, 8), Ok(()));
    assert!(options.validate_native_cores(64, 8, 6).is_err());
    assert!(options.validate_native_cores(64, 8, 10).is_err());
    assert!(options.validate_native_cores(2, 2, 10).is_err());
    let options = System::parse(&format!("{preboot} --native-cores 4")).unwrap();
    assert_eq!(options.validate_native_cores(64, 4, 8), Ok(()));
    assert!(options.validate_native_cores(64, 4, 2).is_err());
    let options = System::parse(&format!("{preboot} --serialized")).unwrap();
    assert_eq!(options.validate_native_cores(64, 1, 10), Ok(()));
}

#[test]
fn exact_roster_distinguishes_verified_disk_authored_and_diagnostic_sources() {
    let assigned = |options: &str, available, prepared| {
        System::parse(options)
            .unwrap()
            .assigned_native_cores(available, prepared)
    };
    let preboot = format!("--system --preboot {DISK}");
    assert!(assigned(&preboot, 64, None).is_err());
    assert_eq!(
        assigned(&format!("{preboot} --native-cores 3"), 64, None),
        Ok(3)
    );
    assert_eq!(
        assigned(&format!("{preboot} --serialized"), 64, None),
        Ok(1)
    );
    assert_eq!(assigned("--system", 64, None), Ok(1));
    assert_eq!(assigned("--system", 64, Some(8)), Ok(8));
    assert_eq!(assigned("--system --native-cores 3", 64, Some(8)), Ok(3));
    assert!(assigned("--system --native-cores 9", 64, Some(8)).is_err());
    assert!(assigned("--system --native-cores 3", 2, Some(8)).is_err());
    assert!(assigned("--system --native-cores 3", 64, None).is_err());
    assert_eq!(assigned("--system --serialized", 64, Some(8)), Ok(1));
    assert!(assigned("--system --serialized --native-cores 4", 64, Some(8)).is_err());
    assert_eq!(
        assigned("--system --native-shared-check --native-cores 4", 64, None),
        Ok(4)
    );
    assert!(assigned("--system --native-bootstrap-check", 64, None).is_err());
}

#[test]
fn invalid_lines_refuse() {
    for text in [
        "--storage x",
        "--system --unknown 1",
        "--system --install p.json",
        "--system --ecid 0",
        "--system --safe-boot",
        "--system --network-check ftp://h/p",
        "--system --dram-base 0x1000",
        "--system --storage-ram 1 --storage-ram 2",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
}

#[test]
fn numbers_take_decimal_and_separated_hex() {
    assert_eq!(number("0x1_0000"), Ok(0x10000));
    assert_eq!(number("42"), Ok(42));
    assert!(number("0xg").is_err());
    assert_eq!(
        vec![Base::parse("x3+0x10").unwrap()],
        [Base::Register(3, 16)]
    );
}

mod constraints;
mod grammar;
