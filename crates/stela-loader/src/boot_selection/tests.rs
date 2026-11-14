use super::*;
use stela_soc_data::nor::nvram::empty_bank;

const PARTITION: &str = "A1B2C3D4-0000-1111-2222-333344445555";
const GROUP: &str = "A304C51B-D22F-4CDD-829D-3E30CB5BD87B";

fn variables(entries: &[(Partition, &[u8], &[u8])]) -> Variables {
    let mut variables = Variables::parse(&empty_bank()).unwrap();
    for (partition, name, value) in entries {
        variables.set(*partition, name, value).unwrap();
    }
    variables
}

fn volume(kind: VolumeType) -> Vec<u8> {
    format!("{}:{PARTITION}:{GROUP}", kind.text()).into_bytes()
}

#[test]
fn uuids_round_trip_their_text_and_types_are_on_disk_gpt_bytes() {
    let uuid = parse_uuid(GROUP.as_bytes()).unwrap();
    assert_eq!(uuid_text(&uuid), GROUP);
    assert_eq!(
        parse_uuid(GROUP.to_ascii_lowercase().as_bytes()),
        Some(uuid)
    );
    assert_eq!(parse_uuid(b"A304C51B-D22F-4CDD-829D-3E30CB5BD87"), None);
    assert_eq!(parse_uuid(b"A304C51B+D22F-4CDD-829D-3E30CB5BD87B"), None);
    // The APFS partition type, 7C3457EF-0000-11AA-AA11-00306543ECAC,
    // as a GPT entry stores it.
    let apfs =
        stela_formats::payload::gpt::Guid::parse("7C3457EF-0000-11AA-AA11-00306543ECAC").unwrap();
    assert_eq!(VolumeType::Apfs.guid_bytes(), apfs.0);
}

#[test]
fn boot_volumes_parse_and_encode_as_llb_reads_them() {
    let text = volume(VolumeType::Apfs);
    let parsed = BootVolume::parse(&text).unwrap();
    assert_eq!(parsed.kind, VolumeType::Apfs);
    assert_eq!(uuid_text(&parsed.volume_group), GROUP);
    assert_eq!(parsed.encode(), text);
    let mut terminated = text.clone();
    terminated.push(0);
    assert_eq!(BootVolume::parse(&terminated).unwrap(), parsed);
    assert!(BootVolume::parse(format!("{GROUP}:{PARTITION}:{GROUP}").as_bytes()).is_err());
    assert!(BootVolume::parse(&text[..text.len() - 1]).is_err());
    assert!(BootVolume::parse(b"x:y").is_err());
}

#[test]
fn no_variables_boot_the_only_macos() {
    let selection = select(&variables(&[])).unwrap();
    assert_eq!(selection.target, Target::MacOs { safe: false });
    assert_eq!(selection.volume, None);
    assert!(!selection.consumes());
}

#[test]
fn one_time_boot_commands_are_consumed() {
    let other = format!("{}:{PARTITION}:{PARTITION}", VolumeType::Apfs.text()).into_bytes();
    let mut vars = variables(&[
        (Partition::System, BOOT_VOLUME, &volume(VolumeType::Apfs)),
        (Partition::System, ALT_BOOT_VOLUME, &other),
        (Partition::Common, BOOT_COMMAND, b"fsboot"),
        (Partition::Common, ONE_TIME_BOOT_COMMAND, b"recover"),
        (Partition::Common, b"iboot-failure-reason", b"x"),
    ]);
    let selection = select(&vars).unwrap();
    assert_eq!(selection.target, Target::PairedRecovery);
    assert_eq!(
        uuid_text(&selection.volume.unwrap().volume_group),
        PARTITION
    );
    assert!(consume(&mut vars, &selection).unwrap());
    assert_eq!(lookup(&vars, ALT_BOOT_VOLUME), None);
    assert_eq!(lookup(&vars, ONE_TIME_BOOT_COMMAND), None);
    assert_eq!(lookup(&vars, b"iboot-failure-reason"), None);
    // The persistent choices stay, and decide the next boot.
    let next = select(&vars).unwrap();
    assert_eq!(next.target, Target::MacOs { safe: false });
    assert_eq!(uuid_text(&next.volume.unwrap().volume_group), GROUP);
    assert!(!consume(&mut vars, &next).unwrap());
    let saved = Variables::parse(&vars.encode().unwrap()).unwrap();
    assert_eq!(select(&saved).unwrap(), next);
}

#[test]
fn recovery_commands_and_recovery_boot_mode_choose_recoveryos() {
    for (command, target) in [
        (&b"recover-system"[..], Target::SystemRecovery),
        (b"recover-fallback", Target::SystemRecovery),
        (b"recover-once", Target::PairedRecovery),
        (b"fsboot-safe", Target::MacOs { safe: true }),
    ] {
        let selection = select(&variables(&[(Partition::System, BOOT_COMMAND, command)]));
        assert_eq!(selection.unwrap().target, target);
    }
    let mode = variables(&[
        (Partition::Common, BOOT_COMMAND, b"fsboot"),
        (Partition::System, RECOVERY_BOOT_MODE, b"unused"),
    ]);
    let selection = select(&mode).unwrap();
    assert_eq!(selection.target, Target::PairedRecovery);
    assert!(!selection.consumes());
    let system = variables(&[
        (Partition::System, RECOVERY_BOOT_MODE, b"ssv-seal-broken"),
        (Partition::System, BOOT_COMMAND, b"recover-system"),
    ]);
    assert_eq!(select(&system).unwrap().target, Target::SystemRecovery);
}

#[test]
fn unmodeled_boots_and_bad_values_are_refused() {
    for command in [&b"field-diags-paired"[..], b"nope"] {
        assert!(select(&variables(&[(Partition::Common, BOOT_COMMAND, command)])).is_err());
    }
    assert!(
        select(&variables(&[(
            Partition::Common,
            ONE_TIME_BOOT_COMMAND,
            b"upgrade"
        )]))
        .is_err()
    );
    assert!(select(&variables(&[(Partition::Common, AUTO_BOOT, b"maybe")])).is_err());
    let bad_once = variables(&[
        (Partition::Common, AUTO_BOOT, b"false"),
        (Partition::Common, AUTO_BOOT_ONCE, b"maybe"),
    ]);
    assert!(select(&bad_once).is_err());
    let once = variables(&[
        (Partition::Common, AUTO_BOOT, b"false"),
        (Partition::Common, AUTO_BOOT_ONCE, b"true"),
    ]);
    let selection = select(&once).unwrap();
    assert_eq!(selection.consumed, [AUTO_BOOT_ONCE]);
    let recovery = variables(&[(
        Partition::System,
        BOOT_VOLUME,
        &volume(VolumeType::Recovery),
    )]);
    assert!(select(&recovery).is_err());
    let mut chosen = variables(&[]);
    let target = BootVolume::parse(&volume(VolumeType::Apfs)).unwrap();
    choose(&mut chosen, &target, true).unwrap();
    assert_eq!(select(&chosen).unwrap().volume, Some(target));
    assert_eq!(select(&chosen).unwrap().consumed, [ALT_BOOT_VOLUME]);
}

#[test]
fn an_upgrade_boots_once_and_a_retry_falls_back_to_the_paired_recoveryos() {
    let mut vars = variables(&[
        (Partition::System, BOOT_VOLUME, &volume(VolumeType::Apfs)),
        (Partition::Common, BOOT_COMMAND, b"upgrade"),
        (Partition::Common, ONE_TIME_BOOT_COMMAND, b"recover"),
    ]);
    let selection = select(&vars).unwrap();
    assert_eq!(selection.target, Target::Upgrade);
    assert_eq!(selection.consumed, [BOOT_COMMAND]);
    assert!(consume(&mut vars, &selection).unwrap());
    // A crash during the upgrade boots by the remaining variables.
    assert_eq!(select(&vars).unwrap().target, Target::PairedRecovery);

    let mut retry = variables(&[
        (Partition::Common, BOOT_COMMAND, b"upgrade"),
        (Partition::Common, UPGRADE_RETRY, b"true"),
        (Partition::System, IBOOT_PRECOMMITTED, b"true"),
    ]);
    let selection = select(&retry).unwrap();
    assert_eq!(selection.target, Target::Upgrade);
    assert_eq!(selection.notes.len(), 1);
    assert!(consume(&mut retry, &selection).unwrap());
    assert_eq!(lookup(&retry, UPGRADE_RETRY), None);
    assert_eq!(lookup(&retry, IBOOT_PRECOMMITTED), None);
    assert_eq!(lookup(&retry, BOOT_COMMAND), Some(&b"recover"[..]));
    assert_eq!(select(&retry).unwrap().target, Target::PairedRecovery);

    let mut failed = variables(&[]);
    record_upgrade_failure(&mut failed, "mount upgrade partition").unwrap();
    assert_eq!(lookup(&failed, OTA_RESULT), Some(UPGRADE_FAILURE));
    assert_eq!(
        lookup(&failed, OTA_FAILURE_REASON),
        Some(&b"mount upgrade partition"[..])
    );
    // post-upgrade is a macOS boot of the fsboot class.
    let post = variables(&[(Partition::Common, ONE_TIME_BOOT_COMMAND, b"post-upgrade")]);
    assert_eq!(select(&post).unwrap().target, Target::MacOs { safe: false });
}

#[test]
fn upgrades_and_paired_recoveryos_on_disk_keep_the_preboot_disk() {
    let vars = variables(&[
        (Partition::System, BOOT_VOLUME, &volume(VolumeType::Apfs)),
        (Partition::System, BOOT_COMMAND, b"upgrade"),
    ]);
    let (options, log) = route(CONFIG, Some(&select(&vars).unwrap())).unwrap();
    assert!(options.contains("--preboot S:G") && options.ends_with(UPGRADE_OPTION));
    assert!(options.contains(BOOT_VOLUME_OPTION));
    assert!(log.starts_with("macOS update"));
    let mut paired = select(&variables(&[(Partition::System, BOOT_COMMAND, b"recover")])).unwrap();
    paired.paired_on_disk = true;
    let (options, log) = route(CONFIG, Some(&paired)).unwrap();
    assert!(options.ends_with(PAIRED_RECOVERY_OPTION) && !options.contains(BUNDLE_OPTION));
    assert!(log.starts_with("paired recoveryOS"));
    assert!(route("--system --upgrade", None).is_err());
    assert!(route("--system --paired-recovery", None).is_err());
}

#[test]
fn an_interrupted_restore_holds_the_machine_in_recovery_mode() {
    // restored clears auto-boot while it writes the disk.
    for vars in [
        variables(&[(Partition::System, AUTO_BOOT, b"false")]),
        variables(&[
            (Partition::System, AUTO_BOOT, b"false"),
            (Partition::System, AUTO_BOOT_ONCE, b"false"),
            (Partition::System, BOOT_COMMAND, b"recover"),
        ]),
        variables(&[(Partition::Common, BOOT_COMMAND, b"device-recovery")]),
    ] {
        let selection = select(&vars).unwrap();
        assert_eq!(selection.target, Target::RecoveryMode);
        // Nothing is consumed: the restore's completion sets auto-boot.
        assert!(!selection.consumes());
        let error = route(CONFIG, Some(&selection)).unwrap_err().to_string();
        assert!(error.starts_with(RECOVERY_MODE_REFUSAL), "{error}");
        assert!(error.contains("Install macOS"), "{error}");
    }
    // auto-boot-once lets one boot through, which consumes it.
    let once = variables(&[
        (Partition::System, AUTO_BOOT, b"false"),
        (Partition::System, AUTO_BOOT_ONCE, b"true"),
    ]);
    assert_eq!(select(&once).unwrap().target, Target::MacOs { safe: false });
}

#[test]
fn recoveryos_prefers_the_disk_s_paired_copy_then_its_system_recovery() {
    let paired = select(&variables(&[(Partition::System, BOOT_COMMAND, b"recover")])).unwrap();
    let system = select(&variables(&[(
        Partition::System,
        BOOT_COMMAND,
        b"recover-system",
    )]))
    .unwrap();
    let routed = |selection: &Selection, paired_on_disk: bool, system_on_disk: bool| {
        let mut selection = selection.clone();
        selection.paired_on_disk = paired_on_disk;
        selection.system_on_disk = system_on_disk;
        route(CONFIG, Some(&selection)).unwrap()
    };
    let (options, log) = routed(&paired, true, true);
    assert!(options.ends_with(PAIRED_RECOVERY_OPTION), "{options}");
    assert!(log.starts_with("paired recoveryOS"));
    // No paired copy: System Recovery from the disk.
    let (options, log) = routed(&paired, false, true);
    assert!(options.contains("--preboot S:G") && options.ends_with(SYSTEM_RECOVERY_OPTION));
    assert!(log.contains("holds no paired recoveryOS"), "{log}");
    // recover-system never boots the paired copy.
    let (options, _) = routed(&system, true, true);
    assert!(options.ends_with(SYSTEM_RECOVERY_OPTION), "{options}");
    // Neither on the disk: refused, with no recoveryOS to fall back to.
    let mut neither = system.clone();
    neither.paired_on_disk = true;
    let error = route(CONFIG, Some(&neither)).unwrap_err().to_string();
    assert!(error.starts_with(NO_RECOVERY_REFUSAL), "{error}");
    // A missing-boot-media boot is System Recovery too.
    let mut blank = select(&variables(&[])).unwrap();
    blank.missing_boot_media();
    assert!(
        routed(&blank, false, true)
            .0
            .ends_with(SYSTEM_RECOVERY_OPTION)
    );
    // The engine is told by NVRAM alone.
    assert!(route("--system --system-recovery", None).is_err());
    // A recoveryOS boot volume names the RecoveryOSContainer to boot.
    let named = variables(&[
        (
            Partition::System,
            BOOT_VOLUME,
            &volume(VolumeType::Recovery),
        ),
        (Partition::System, BOOT_COMMAND, b"recover-system"),
    ]);
    let mut named = select(&named).unwrap();
    named.system_on_disk = true;
    let (options, _) = route(CONFIG, Some(&named)).unwrap();
    assert!(options.contains(&format!(
        "{BOOT_VOLUME_OPTION} {}",
        String::from_utf8(volume(VolumeType::Recovery)).unwrap()
    )));
}

const CONFIG: &str = "--system --storage S:G --nor S:N --preboot S:G";

#[test]
fn macos_boots_keep_the_preboot_disk_and_name_the_volume() {
    let vars = variables(&[
        (Partition::System, BOOT_VOLUME, &volume(VolumeType::Apfs)),
        (Partition::System, BOOT_COMMAND, b"fsboot-safe"),
    ]);
    let (options, log) = route(CONFIG, Some(&select(&vars).unwrap())).unwrap();
    let expected = format!(
        "--system --storage S:G --nor S:N --preboot S:G \
             --boot-volume {} --boot-command 9 --safe-boot",
        String::from_utf8(volume(VolumeType::Apfs)).unwrap()
    );
    assert_eq!(options, expected);
    assert!(log.starts_with("macOS in safe mode"));
    // Without NVRAM the options pass unchanged.
    let (plain, _) = route(CONFIG, None).unwrap();
    assert_eq!(plain, CONFIG);
}

#[test]
fn disk_routes_preserve_command_origins_after_consuming_one_shots() {
    for (command, origin) in [
        (b"fsboot".as_slice(), 1),
        (b"fsboot-safe".as_slice(), 9),
        (b"post-upgrade".as_slice(), 14),
        (b"recover".as_slice(), 11),
        (b"recover-system".as_slice(), 4),
        (b"recover-fallback".as_slice(), 4),
    ] {
        let mut vars = variables(&[(Partition::System, ONE_TIME_BOOT_COMMAND, command)]);
        let mut selection = select(&vars).unwrap();
        selection.paired_on_disk = true;
        selection.system_on_disk = true;
        assert!(consume(&mut vars, &selection).unwrap());
        assert!(lookup(&vars, ONE_TIME_BOOT_COMMAND).is_none());
        let (options, _) = route(CONFIG, Some(&selection)).unwrap();
        assert!(
            options.contains(&format!("{BOOT_COMMAND_OPTION} {origin}")),
            "{command:?}: {options}"
        );
    }
    let update = select(&variables(&[(Partition::System, BOOT_COMMAND, b"upgrade")])).unwrap();
    assert!(
        route(CONFIG, Some(&update))
            .unwrap()
            .0
            .contains("--boot-command 3")
    );
    let mut fallback =
        select(&variables(&[(Partition::System, BOOT_COMMAND, b"recover")])).unwrap();
    fallback.system_on_disk = true;
    assert!(
        route(CONFIG, Some(&fallback))
            .unwrap()
            .0
            .contains("--boot-command 4")
    );
    // An arbitrary config value cannot replace the origin derived by
    // boot selection. Unknown bytes are separately refused by BootCommand.
    assert!(route("--system --preboot S:G --boot-command 7", Some(&update)).is_err());
}

#[test]
fn recovery_boots_of_a_disk_without_recoveryos_are_refused() {
    let vars = variables(&[(Partition::Common, BOOT_COMMAND, b"recover-system")]);
    let selection = select(&vars).unwrap();
    let error = route(CONFIG, Some(&selection)).unwrap_err().to_string();
    assert!(error.starts_with(NO_RECOVERY_REFUSAL), "{error}");
    assert!(error.contains("recover-system"), "{error}");
    let paired = select(&variables(&[(Partition::System, RECOVERY_BOOT_MODE, b"x")])).unwrap();
    assert!(route(CONFIG, Some(&paired)).is_err());
    // A disk without an installed system or System Recovery is refused
    // too.
    let mut empty = select(&variables(&[])).unwrap();
    empty.missing_boot_media();
    let error = route(CONFIG, Some(&empty)).unwrap_err().to_string();
    assert!(error.contains("missing-boot-media"), "{error}");
    empty.system_on_disk = true;
    let (options, log) = route(CONFIG, Some(&empty)).unwrap();
    assert!(options.ends_with(SYSTEM_RECOVERY_OPTION), "{options}");
    assert!(log.contains("missing-boot-media"), "{log}");
    assert!(route("--system --safe-boot", None).is_err());
}

#[test]
fn a_pending_bundle_repair_boots_macos_once_for_it_and_never_again_by_itself() {
    let mut vars = variables(&[(Partition::System, BUNDLE_REPAIR, BUNDLE_REPAIR_PENDING)]);
    let selection = select(&vars).unwrap();
    assert_eq!(selection.target, Target::MacOs { safe: false });
    assert!(selection.bundle_repair);
    let (options, log) = route(CONFIG, Some(&selection)).unwrap();
    assert!(
        options.ends_with(" --boot-command 1 --bundle-repair"),
        "{options}"
    );
    assert!(log.contains("repair the Metal plugin"), "{log}");
    // Recorded as attempted before the boot: a power loss in the middle
    // leaves no request to repeat.
    assert!(consume(&mut vars, &selection).unwrap());
    assert_eq!(lookup(&vars, BUNDLE_REPAIR), Some(BUNDLE_REPAIR_ATTEMPTED));
    // By this version, so a later Stela may try once more.
    assert_eq!(
        lookup(&vars, BUNDLE_REPAIR_VERSION),
        Some(BUNDLE_REPAIR_VERSION_VALUE)
    );
    let next = select(&vars).unwrap();
    assert!(!next.bundle_repair);
    assert!(!next.consumes());
    let (plain, _) = route(CONFIG, Some(&next)).unwrap();
    assert!(!plain.contains(BUNDLE_REPAIR_OPTION), "{plain}");
}

#[test]
fn a_retry_boots_macos_for_the_repair_once_more_and_then_never_by_itself() {
    let mut vars = variables(&[(Partition::System, BUNDLE_REPAIR, BUNDLE_REPAIR_RETRY)]);
    let selection = select(&vars).unwrap();
    assert_eq!(selection.target, Target::MacOs { safe: false });
    assert!(selection.bundle_repair);
    let (options, _) = route(CONFIG, Some(&selection)).unwrap();
    assert!(options.ends_with(BUNDLE_REPAIR_OPTION), "{options}");
    assert!(consume(&mut vars, &selection).unwrap());
    assert_eq!(lookup(&vars, BUNDLE_REPAIR), Some(BUNDLE_REPAIR_RETRIED));
    let next = select(&vars).unwrap();
    assert!(!next.bundle_repair && !next.consumes());
}

#[test]
fn taking_a_bundle_repair_removes_the_last_attempts_result() {
    for record in [
        BUNDLE_REPAIR_PENDING,
        BUNDLE_REPAIR_RETRY,
        BUNDLE_REPAIR_INSTALLED,
    ] {
        let mut vars = variables(&[
            (Partition::System, BUNDLE_REPAIR, record),
            (Partition::System, BUNDLE_REPAIR_RESULT, b"step 3 exited"),
        ]);
        let selection = select(&vars).unwrap();
        assert!(selection.consumed.contains(&BUNDLE_REPAIR_RESULT));
        assert!(consume(&mut vars, &selection).unwrap());
        assert_eq!(lookup(&vars, BUNDLE_REPAIR_RESULT), None);
        // A boot that takes nothing keeps it for the stop to name.
        let mut empty = select(&variables(&[
            (Partition::System, BUNDLE_REPAIR, record),
            (Partition::System, BUNDLE_REPAIR_RESULT, b"step 3 exited"),
        ]))
        .unwrap();
        empty.missing_boot_media();
        assert!(!empty.consumes());
    }
    let vars = variables(&[
        (Partition::System, BUNDLE_REPAIR, BUNDLE_REPAIR_ATTEMPTED),
        (Partition::System, BUNDLE_REPAIR_RESULT, b"step 3 exited"),
    ]);
    assert!(!select(&vars).unwrap().consumes());
}

#[test]
fn an_installs_bundle_repair_boots_macos_once_for_it_and_leaves_no_record() {
    let mut vars = variables(&[(Partition::System, BUNDLE_REPAIR, BUNDLE_REPAIR_INSTALLED)]);
    let selection = select(&vars).unwrap();
    assert_eq!(selection.target, Target::MacOs { safe: false });
    assert!(selection.bundle_repair);
    let (options, _) = route(CONFIG, Some(&selection)).unwrap();
    assert!(options.ends_with(BUNDLE_REPAIR_OPTION), "{options}");
    // Removed before the boot: a repair that fails leaves the next normal
    // boot free to ask for the usual one.
    assert!(consume(&mut vars, &selection).unwrap());
    assert_eq!(lookup(&vars, BUNDLE_REPAIR), None);
    let mut older = variables(&[
        (Partition::System, BUNDLE_REPAIR, BUNDLE_REPAIR_INSTALLED),
        (Partition::System, BUNDLE_REPAIR_VERSION, b"0.0.1"),
    ]);
    let selection = select(&older).unwrap();
    consume(&mut older, &selection).unwrap();
    assert_eq!(lookup(&older, BUNDLE_REPAIR_VERSION), None);
    assert!(!select(&vars).unwrap().bundle_repair);
    // A recoveryOS boot and a disk without a system leave the request.
    let mut empty = select(&variables(&[(
        Partition::System,
        BUNDLE_REPAIR,
        BUNDLE_REPAIR_INSTALLED,
    )]))
    .unwrap();
    empty.missing_boot_media();
    assert!(!empty.bundle_repair && !empty.consumes());
}

#[test]
fn a_bundle_repair_waits_for_a_macos_boot_and_stela_cfg_cannot_ask_for_one() {
    let vars = variables(&[
        (Partition::System, BUNDLE_REPAIR, BUNDLE_REPAIR_PENDING),
        (Partition::System, ONE_TIME_BOOT_COMMAND, b"recover"),
    ]);
    let selection = select(&vars).unwrap();
    assert_eq!(selection.target, Target::PairedRecovery);
    assert!(!selection.bundle_repair);
    assert!(
        !selection
            .assigned
            .iter()
            .any(|(name, _)| *name == BUNDLE_REPAIR)
    );
    // A disk without an installed system boots System Recovery, which
    // leaves the request too.
    let mut empty = select(&variables(&[(
        Partition::System,
        BUNDLE_REPAIR,
        BUNDLE_REPAIR_PENDING,
    )]))
    .unwrap();
    empty.missing_boot_media();
    assert!(!empty.bundle_repair && empty.assigned.is_empty());
    // A NUL-terminated value is the same request; others are none.
    let terminated = variables(&[(Partition::Common, BUNDLE_REPAIR, b"pending\0")]);
    assert!(select(&terminated).unwrap().bundle_repair);
    let other = variables(&[(Partition::System, BUNDLE_REPAIR, b"done")]);
    assert!(!select(&other).unwrap().bundle_repair);
    assert!(route(&format!("{CONFIG} {BUNDLE_REPAIR_OPTION}"), None).is_err());
}
