//! `status` and `repair`.
//!
//! Status reads everything a setup consists of and changes nothing: the
//! record, the installed `stela.efi` against the recorded version and this
//! `stela`'s mount interface, the USB's boot loader,
//! `stela.cfg` and the startup choices against the record, every restore
//! bundle file against its `bundle.json` and the active ones'
//! revision against the one this `stela` prepares, the install medium's
//! manifest and pieces, and for each install its
//! partitions (still the recorded ones), the systems on its Apple region,
//! its NVRAM (what the next boot will be, and a failed update's
//! `ota-result`) and whether its SEP state is provisioned, and the firmware
//! the newest inventory in the USB's boot log asks for and the USB lacks.
//! Each problem names the command that fixes it, and any problem fails the
//! command.
//!
//! Repair puts back what the record says: `stela.cfg` and the startup
//! choices, and, from the active build's restore image (`--ipsw` or a
//! download), every damaged or missing restore bundle and a damaged install
//! medium; with an image it prepares every restore bundle of the active
//! build again, with each machine's own ECID, and gives an install without
//! one its ECID (`machines::machine_ecid`). It removes the bundles and media
//! of other builds and generations that an interrupted update left. A
//! missing or foreign `stela.efi` is `update`'s to replace: no copy of
//! it is kept. With `--initialize-nor` it replaces machine state `stela.efi`
//! would refuse (`installed::reinitialize`), blanking a foreign SEP area
//! only for a machine that never booted. It adds the pinned firmware the
//! last boot asked for and the USB lacks (`firmware::requested`), keeping
//! what the USB holds, and names files without a pinned source. An
//! automatic Metal plugin repair that ran and did not install the plugin
//! becomes a request again (`installed::bundle_repair_again`), so it runs
//! at the install's next start.
use super::{
    CHOICES_PATH, Environment, LOADER_PATH, RECORD, Record, Reported, Request, apply_install,
    bundle, check_bundle_revision, describe_install, ecid::Ecid, fix, host::Host, host::bytes_text,
    installed, interrupted_text, machines, plan_install, publish, read_record, revision_mismatch,
    sources, state, update, write_record,
};
use anyhow::{Context, Result, bail, ensure};
use std::{
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
};
use stela_boot_core::chosen_disk::{Search, Stale};
use stela_loader::{boot_selection, mount_interface, recovery};
use {
    stela_formats::apfs, stela_formats::apfs::Container,
    stela_formats::apfs::volume::APFS_VOL_ROLE_UPDATE, stela_formats::payload::ReadAt,
    stela_formats::preboot, stela_soc_data::nor::CAPACITY,
};

/// A host partition as an APFS source.
struct Partition<'a> {
    host: &'a dyn Host,
    identifier: String,
    bytes: u64,
}

impl ReadAt for Partition<'_> {
    type Error = String;

    fn len(&self) -> u64 {
        self.bytes
    }

    fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> Result<(), String> {
        let bytes = self
            .host
            .read(&self.identifier, offset, buffer.len())
            .map_err(|error| format!("{error:#}"))?;
        buffer.copy_from_slice(&bytes);
        Ok(())
    }
}

/// The installed systems on an Apple region: each container's boot
/// directories, whether it has a staged update, and the boot directories
/// of its paired recoveryOS and System Recovery (`stela_loader::recovery`).
fn systems(host: &dyn Host, identifier: &str, bytes: u64) -> Vec<String> {
    let mut disk = Partition {
        host,
        identifier: identifier.into(),
        bytes,
    };
    let partitions = match apfs::container_partitions(&mut disk) {
        Ok(partitions) => partitions,
        Err(_) => return Vec::new(),
    };
    let mut found = Vec::new();
    for (offset, _) in partitions {
        let Ok(mut container) = Container::open(&mut disk, offset) else {
            continue;
        };
        let update = container.volume_with_role(APFS_VOL_ROLE_UPDATE).is_ok();
        match preboot::candidates(&mut container) {
            Ok((_, sets)) => {
                for set in sets {
                    found.push(format!(
                        "container at {offset:#x}: {}{}",
                        set.directory(),
                        if update {
                            "; an update is staged on its Update volume"
                        } else {
                            ""
                        }
                    ));
                }
            }
            Err(error) => found.push(format!(
                "container at {offset:#x}: no boot objects ({error})"
            )),
        }
    }
    // The recoveryOS copies stela.efi boots when NVRAM asks for one.
    for kind in [recovery::Kind::Paired, recovery::Kind::System] {
        match recovery::find(&mut disk, kind, None) {
            Ok(copies) => {
                for copy in copies {
                    for set in &copy.sets {
                        found.push(format!(
                            "{} at {:#x}: {}",
                            kind.label(),
                            copy.offset,
                            set.directory()
                        ));
                    }
                }
            }
            Err(error) => found.push(format!("{}: unreadable ({error:#})", kind.label())),
        }
    }
    found
}

/// The report and its problems.
struct Report {
    text: String,
    problems: Vec<String>,
    /// The USB, as the fixes name it.
    usb: PathBuf,
}

impl Report {
    /// The command that fixes a problem (`super::fix`).
    fn fix(&self, words: &str) -> String {
        fix(&self.usb, words)
    }

    fn line(&mut self, label: &str, value: impl std::fmt::Display) {
        let _ = writeln!(self.text, "  {label:<14} {value}");
    }

    fn problem(&mut self, problem: String) {
        self.problems.push(problem);
    }
}

/// The translation store's packs, and what `repair` would fix.
fn translations_status(record: &Record, report: &mut Report) {
    let Some(store) = &record.translations else {
        report.line("translations", "none");
        return;
    };
    report.line("translations", store.describe());
    for pack in store.packs() {
        report.line(
            "  pack",
            format!("{} ({})", pack.path.display(), bytes_text(pack.bytes)),
        );
    }
    // The first boot renames the spare into its pack by design: a note, with
    // the repair that prepares the next one.
    let taken = store.spare_taken().then(|| store.spare_repair());
    for repair in store.repairs() {
        let fix = report.fix("repair");
        let text = format!("translation store: {repair}: {fix}");
        match taken.as_ref() == Some(&repair) {
            true => report.line(
                "note",
                format!("{text} (the first boot took the spare for its pack, as it should)"),
            ),
            false => report.problem(text),
        }
    }
}

fn efi_status(environment: &Environment<'_>, record: &Record, report: &mut Report) {
    let tool = record.esp.join(LOADER_PATH);
    match fs::read(&tool) {
        Ok(image) => {
            let id = update::version_id(&image);
            let interface = mount_interface::find(&image);
            report.line(
                "stela.efi",
                format!(
                    "version {id}{}",
                    interface
                        .map(|i| format!(", reads boot-bundle schema {}", i.bundle_schema))
                        .unwrap_or_default()
                ),
            );
            let fix = report.fix("update");
            if record.stela_version.as_deref() != Some(id.as_str()) {
                report.problem(format!(
                    "{} is not the version setup installed ({}): {fix}",
                    tool.display(),
                    record.stela_version.as_deref().unwrap_or("unknown")
                ));
            }
            if let Some(reason) = mount_interface::mismatch(environment.interface, interface) {
                report.problem(format!(
                    "the installed stela.efi does not read what this stela writes ({reason}): \
                     {fix}"
                ));
            }
        }
        Err(_) => {
            let fix = report.fix("update");
            report.problem(format!("{} is missing: {fix}", tool.display()))
        }
    }
    let repair = report.fix("repair");
    let options = fs::read_to_string(record.esp.join("EFI/stela/stela.cfg")).unwrap_or_default();
    match record.options_text() {
        Ok(expected) if options != expected => {
            report.problem(format!("stela.cfg differs from the setup record: {repair}"))
        }
        Ok(_) => {}
        Err(error) => report.problem(format!(
            "stela.cfg cannot be written from the setup record: {error:#}"
        )),
    }
    let choices = fs::read_to_string(record.esp.join(CHOICES_PATH)).ok();
    match record.choices_text() {
        Ok(expected) if choices != expected => report.problem(format!(
            "the startup choices (choices.cfg) differ from the setup record: {repair}"
        )),
        Ok(_) => {}
        Err(error) => report.problem(format!("the startup choices: {error:#}")),
    }
}

/// The firmware the newest inventory in the USB's boot log asks for and the
/// USB lacks: files `repair` adds are problems, the others are named.
fn firmware_status(record: &Record, report: &mut Report) {
    let Ok(tool) = fs::read(record.esp.join(LOADER_PATH)) else {
        return;
    };
    if stela_loader::host_firmware::reads_bundle(&tool) {
        match super::firmware::bundle_status(&record.volume) {
            Ok(complete) => report.line("firmware", format!("bundle {complete}")),
            Err(missing) => {
                report.problem(format!("{missing}: {} adds the rest", report.fix("repair")))
            }
        }
        // Every file a boot asks for that a recipe pins is in the bundle.
        return;
    }
    let requested = match super::firmware::requested(&record.esp, &tool) {
        Ok(requested) => requested,
        Err(error) => {
            report.problem(format!("the stick's host firmware: {error:#}"));
            return;
        }
    };
    if let Some(why) = &requested.unreadable {
        report.line(
            "firmware",
            format!("the boot log's newest firmware request is unreadable: {why}"),
        );
    }
    for request in &requested.unpinned {
        report.line(
            "firmware",
            format!(
                "the last boot asked for {}, which Stela has no pinned source for",
                super::firmware::describe(request)
            ),
        );
    }
    let repair = report.fix("repair");
    for request in &requested.pinned {
        report.problem(format!(
            "the last boot asked for firmware {}, which the stick lacks: {repair} adds it",
            super::firmware::describe(request)
        ));
    }
}

/// The command that puts back the active build's restore files: `repair`,
/// which finds or downloads what it needs of the image itself
/// (`sources`), or first `update stela` when the bundles are of another
/// revision than this stela prepares, which repair refuses to mix.
fn image_fix(environment: &Environment<'_>, record: &Record, report: &Report) -> String {
    match revision_mismatch(environment, record) {
        Some(_) => report.fix("update"),
        None => report.fix("repair"),
    }
}

fn bundle_status(environment: &Environment<'_>, record: &Record, report: &mut Report, full: bool) {
    let repair = report.fix("repair");
    let image = image_fix(environment, record, report);
    for kept in &record.bundles {
        let directory = kept.directory(&record.volume);
        let role = if kept.build == record.build && kept.generation() == record.generation() {
            "active"
        } else {
            "left by an interrupted update"
        };
        let problems = bundle::verify(&directory, &kept.manifest_sha256);
        report.line(
            kept.kind.label(),
            format!(
                "{} ({role}, {}{}): {}",
                directory.display(),
                kept.generation(),
                kept.ecid
                    .map(|ecid| format!(", ECID {ecid}"))
                    .unwrap_or_default(),
                if problems.is_empty() {
                    "intact"
                } else {
                    "DAMAGED"
                }
            ),
        );
        if role != "active" {
            report.problem(format!(
                "{} is left by an interrupted update: run that update again, or {repair} \
                 removes it",
                directory.display()
            ));
            continue;
        }
        for problem in problems {
            report.problem(format!(
                "{problem}: {image} puts it right (--ipsw FILE names a macOS {} image to use)",
                kept.build
            ));
        }
    }
    for directory in record.build_directories() {
        for (_, what) in bundle::leftovers(&directory) {
            report.problem(format!("{what}: {repair} puts it right"));
        }
    }
    if let Some(reason) = revision_mismatch(environment, record) {
        report.problem(reason);
    }
    for (kind, ecid) in record.missing_restore_bundles() {
        report.problem(format!(
            "the active build {} has no {} bundle for {}, so its {} choice is missing: {image}",
            record.build,
            kind.label(),
            ecid.map_or_else(|| "the setup".into(), |e| format!("ECID {e}")),
            super::restore_choice_name(kind)
        ));
    }
    if let Some(next) = &record.replacing {
        report.problem(format!(
            "the switch to macOS {next} stopped before its copy was complete, so this stick cannot \
             install macOS until it is: {} finishes it",
            report.fix("repair")
        ));
    } else if record.installs_media() && record.active_medium().is_none() {
        report.problem(format!(
            "the active build {} has no install medium, so its Install macOS choice is \
             missing: {image}",
            record.build
        ));
    }
    for medium in &record.media {
        let mut problems = medium.problems();
        // A piece an install found damaged since the medium was last
        // written or checked whole: installs read the active build's
        // medium alone, so the log's piece is one of its own.
        if medium.build == record.build && !medium.remote && problems.is_empty() {
            for piece in
                super::medium::reported_damage(&record.esp, record.medium_checked.unwrap_or(0))
            {
                problems.push(format!(
                    "{} failed its block check when an install read it (the stick's boot log \
                     says so)",
                    medium.directory().join(&piece).display()
                ));
            }
            // `--full`: every piece read back and checked now.
            if full && problems.is_empty() {
                match medium.damaged_pieces() {
                    Ok(damaged) => problems.extend(damaged.into_iter().map(|piece| {
                        format!(
                            "{} differs from what was written (read back now)",
                            medium.directory().join(piece).display()
                        )
                    })),
                    Err(error) => problems.push(format!("its pieces cannot be read: {error:#}")),
                }
            }
        }
        if medium.build != record.build {
            report.line(
                "leftover",
                format!("{} (macOS {})", medium.directory().display(), medium.build),
            );
            report.problem(format!(
                "{} is left by an interrupted update: run that update again, or {repair} \
                 removes it",
                medium.directory().display()
            ));
            continue;
        }
        report.line(
            "install medium",
            format!(
                "{} (macOS {}, {}): {}",
                medium.directory().display(),
                medium.build,
                // A streamed image is on Apple's servers, not a partition.
                match medium.remote {
                    true => "Online, downloads during installation".into(),
                    false => format!("Offline, on partition {}", medium.partition_guid),
                },
                match (problems.is_empty(), medium.remote) {
                    (false, _) => "DAMAGED",
                    (true, true) => "intact",
                    // Its bytes are read only by an install, or by
                    // `repair --full`.
                    (true, false) => "complete; every block is checked as the install reads it",
                }
            ),
        );
        for problem in problems {
            report.problem(format!(
                "{problem}: {image} puts it right (--ipsw FILE names a macOS {} image to use)",
                medium.build
            ));
        }
    }
}

/// What this computer holds of the partition Install macOS chose
/// (`disk.cfg`, `SERIAL:GUID`), as `stela.efi` searches for it
/// (`chosen_disk::Search`); `None` when the record cannot be read or this
/// computer cannot list its disks.
fn chosen_search(host: &dyn Host, chosen: &str) -> Option<Result<String, Search>> {
    let (serial, guid) = chosen.trim().rsplit_once(':')?;
    let disks = host.disks().ok().filter(|disks| !disks.is_empty())?;
    let with_serial: Vec<_> = disks
        .iter()
        .filter(|d| host.serial(&d.identifier).ok().flatten().as_deref() == Some(serial))
        .collect();
    let disk = match with_serial[..] {
        [] => return Some(Err(Search::NoDisk)),
        [disk] => disk,
        _ => return Some(Err(Search::SeveralDisks)),
    };
    if disk.partitions.is_empty() {
        return Some(Err(Search::NoTable));
    }
    let matching: Vec<_> = disk
        .partitions
        .iter()
        .filter(|p| {
            host.partition(&p.identifier)
                .is_ok_and(|info| info.guid.eq_ignore_ascii_case(guid))
        })
        .collect();
    Some(match matching[..] {
        [] => Err(Search::NoPartition),
        [partition] if partition.content == "EFI" => Err(Search::EspPartition),
        [partition] => Ok(partition.identifier.clone()),
        _ => Err(Search::SeveralPartitions),
    })
}

/// The partition Install macOS chose (`EFI/stela/disk.cfg`), when it is on
/// this computer.
fn chosen_partition(host: &dyn Host, record: &Record) -> Option<String> {
    let chosen = fs::read_to_string(record.esp.join("EFI/stela/disk.cfg")).ok()?;
    chosen_search(host, &chosen)?.ok()
}

/// The machine state of `install`, which a chosen disk has only where this
/// computer holds that disk; `None` then.
fn install_state_area(
    host: &dyn Host,
    record: &Record,
    install: &machines::Install,
) -> Result<Option<installed::StateArea>> {
    if !install.disk.is_chosen() {
        return installed::state_area(host, &install.disk, None).map(Some);
    }
    chosen_partition(host, record)
        .map(|partition| installed::chosen_state_area(host, &partition))
        .transpose()
}

/// The automatic Metal plugin repair's record and the last attempt's
/// result in `variables`, for status.
fn metal_plugin_status(report: &mut Report, variables: &stela_soc_data::nor::nvram::Variables) {
    use stela_boot_core::bundle_repair::{self, Record};
    let reported = bundle_repair::result(variables);
    match Record::read(variables) {
        Record::Attempted | Record::Retried | Record::Other => {
            let fix = report.fix("repair");
            report.line(
                "Metal plugin",
                format!(
                    "the automatic repair ran and did not install it; {fix} lets it run again \
                     at the next start"
                ),
            );
            if let Some(reported) = reported {
                report.line("", format!("its boot reported: {reported}"));
            }
        }
        Record::Pending | Record::Installed | Record::Retry => report.line(
            "Metal plugin",
            "the automatic repair runs at the next start",
        ),
        Record::None => {}
    }
}

/// The partition Install macOS chose, when it chose one: one that is not
/// on this computer is a note, not a problem. This may not be the PC that
/// runs macOS, and on that PC a stale record only makes Install macOS ask
/// for a disk again.
fn chosen_status(host: &dyn Host, record: &Record, report: &mut Report) {
    let Ok(chosen) = fs::read_to_string(record.esp.join("EFI/stela/disk.cfg")) else {
        return;
    };
    match chosen_search(host, &chosen) {
        Some(Ok(partition)) => {
            report.line("", "the chosen disk is in this computer");
            // Its NVRAM, for the Metal plugin repair this computer can
            // let run again; the rest of its state is the PC's.
            let nvram = installed::chosen_state_area(host, &partition)
                .and_then(|area| area.read(host, 0, CAPACITY));
            if let Ok(bytes) = nvram
                && let state::Nvram::Valid(variables) = state::nvram(&bytes)
            {
                metal_plugin_status(report, &variables);
            }
        }
        Some(Err(search)) => {
            if let Some(stale) = Stale::of(search) {
                report.line(
                    "",
                    format!(
                        "note: {}: if this is the PC that runs macOS, Install macOS will ask \
                         for a disk again",
                        stale.case()
                    ),
                );
            }
        }
        None => {}
    }
}

/// `--install NAME` when the setup has several installs, for a fix that
/// acts on one.
fn install_words(record: &Record, install: &machines::Install) -> String {
    match record.installs.len() {
        1 => String::new(),
        _ => format!(" --install {}", super::quoted(&install.name)),
    }
}

fn install_status(
    environment: &Environment<'_>,
    record: &Record,
    install: &machines::Install,
    report: &mut Report,
) {
    let host = environment.host;
    let default = record.default().is_some_and(|d| d.name == install.name);
    let _ = writeln!(
        report.text,
        "Install \"{}\"{}:",
        install.name,
        if default { " (default)" } else { "" }
    );
    match install.ecid {
        Some(ecid) => report.line("ECID", ecid),
        None => {
            let fix = report.fix("repair");
            report.problem(format!(
                "install \"{}\" has no ECID (an older setup made it): {fix}, with --ecid or \
                 --new-ecid when its SEP state is provisioned",
                install.name
            ))
        }
    }
    let disk = &install.disk;
    // A portable installer's disk is chosen in stela.efi, out of this
    // computer's reach.
    if disk.is_chosen() {
        report.line("Mac's SSD", "a PC disk chosen at Install macOS");
        chosen_status(host, record, report);
        return;
    }
    match installed::confirm(host, disk, installed::Role::AppleRegion, None) {
        Ok(region) => {
            let bytes = host.partition(&region).map(|p| p.bytes).unwrap_or(0);
            // The guest's SSD ends where a machine state in its tail starts.
            let bytes = disk.machine_state_offset.unwrap_or(bytes).min(bytes);
            let systems = systems(host, &region, bytes);
            report.line(
                "Mac's SSD",
                format!(
                    "{region} (serial {}, partition {}): {}",
                    disk.apple_region_serial,
                    disk.apple_region_guid,
                    if systems.is_empty() {
                        "no installed macOS yet; choose Install macOS"
                    } else {
                        "macOS installed"
                    }
                ),
            );
            if region != disk.apple_region {
                report.line(
                    "",
                    format!("was {}; `repair` records the new name", disk.apple_region),
                );
            }
            for system in systems {
                report.line("", system);
            }
        }
        Err(error) => report.problem(format!(
            "install \"{}\"'s SSD partition: {error:#}",
            install.name
        )),
    }
    if let Ok(Some(region)) = installed::foreign(host, disk) {
        let fix = report.fix("repair --erase-target");
        report.problem(format!(
            "{}: {fix} erases it",
            foreign_problem(&install.name, &region.identifier)
        ));
    }
    let area = match installed::state_area(host, disk, None) {
        Ok(area) => area,
        Err(error) => {
            report.problem(format!(
                "install \"{}\"'s machine state: {error:#}",
                install.name
            ));
            return;
        }
    };
    report.line(
        "machine state",
        format!("{} ({})", disk.state_description(), bytes_text(area.bytes)),
    );
    let bytes = match area.read(
        host,
        0,
        CAPACITY + stela_formats::machine_state::SEP_BYTES as usize,
    ) {
        Ok(bytes) => bytes,
        Err(error) => {
            report.problem(format!("{} cannot be read: {error:#}", area.describe()));
            return;
        }
    };
    let (nor_bytes, sep_area) = bytes.split_at(CAPACITY);
    // An interrupted `--target` can leave a machine that never booted with
    // foreign state, which `repair --initialize-nor` replaces.
    let never_booted = installed::never_booted(host, &area)
        .ok()
        .flatten()
        .unwrap_or(false);
    let nor = area.describe();
    let which = install_words(record, install);
    match state::nvram(nor_bytes) {
        state::Nvram::Blank => report.line("NVRAM", "blank"),
        state::Nvram::Valid(variables) => {
            report.line("next boot", state::next_boot(&variables));
            metal_plugin_status(report, &variables);
            for name in [
                boot_selection::OTA_RESULT,
                boot_selection::OTA_FAILURE_REASON,
            ] {
                if let Some(value) = boot_selection::lookup(&variables, name) {
                    report.line(
                        "update",
                        format!(
                            "{}: {}",
                            String::from_utf8_lossy(name),
                            String::from_utf8_lossy(value)
                        ),
                    );
                }
            }
        }
        state::Nvram::Invalid(error) if never_booted => {
            let fix = report.fix("repair --initialize-nor");
            report.problem(format!("{nor}'s NVRAM is unreadable ({error}): {fix}"))
        }
        state::Nvram::Invalid(error) => {
            let fix = report.fix(&format!("nvram reset{which}"));
            report.problem(format!("{nor}'s NVRAM is unreadable ({error}): {fix}"))
        }
    }
    match state::sep(sep_area) {
        Ok(true) => report.line("SEP state", "provisioned"),
        Ok(false) => report.line("SEP state", "blank"),
        Err(error) if never_booted => {
            let fix = report.fix("repair --initialize-nor");
            report.problem(format!(
                "{nor}'s SEP area: {error}; the machine never booted, so it holds no keys: {fix} \
                 blanks it"
            ))
        }
        Err(error) => {
            let lost = installed::lost_keys(install, &report.usb, record.installs.len() > 1);
            report.problem(format!("{nor}'s SEP area: {error}. {lost}"))
        }
    }
}

/// An install's SSD partition holds another file system.
fn foreign_problem(install: &str, region: &str) -> String {
    format!(
        "install \"{install}\"'s SSD partition {region} holds another file system. The PC's own \
         OS mounts it and can overwrite Stela's machine state in it"
    )
}

/// `status`.
pub(super) fn status(environment: &Environment<'_>, usb: &Path, full: bool) -> Result<String> {
    let record = read_record(environment, usb)?;
    let mut report = Report {
        text: String::new(),
        problems: Vec::new(),
        usb: usb.to_path_buf(),
    };
    let _ = writeln!(
        report.text,
        "Stela setup on {} ({}):",
        record.esp.display(),
        record.esp.join(RECORD).display()
    );
    if !record.complete {
        report.problem(interrupted_text(&record, usb));
    }
    report.line("model", &record.model);
    report.line("Stela", super::release::describe(&record));
    report.line("macOS build", &record.build);
    report.line(
        "Wi-Fi",
        match &record.wifi {
            Some(network) => format!(
                "\"{}\"{}",
                network.ssid,
                super::wifi::rules(network.country.as_deref())
            ),
            None => "none: an Online install needs Ethernet".into(),
        },
    );
    efi_status(environment, &record, &mut report);
    firmware_status(&record, &mut report);
    bundle_status(environment, &record, &mut report, full);
    if record.installs_media() {
        let restore_bundles: Vec<PathBuf> = record
            .bundles
            .iter()
            .filter(|kept| kept.kind == bundle::Kind::Restore && kept.build == record.build)
            .map(|kept| kept.directory(&record.volume))
            .collect();
        let remote = record.active_medium().is_some_and(|medium| medium.remote);
        report.line(
            "install",
            super::create::network_status(remote, &restore_bundles),
        );
    }
    translations_status(&record, &mut report);
    for install in &record.installs {
        install_status(environment, &record, install, &mut report);
    }
    // An interrupted first setup has not written the rest yet: running it
    // again writes it, and the other commands refuse until it has.
    if !record.complete && report.problems.len() > 1 {
        let rest = report.problems.len() - 1;
        report.problems.truncate(1);
        report.problems[0].push_str(&format!(
            ". Running the setup again also writes the {rest} other part{} this report finds missing \
             or different",
            if rest == 1 { "" } else { "s" }
        ));
    }
    let mut text = report.text;
    if report.problems.is_empty() {
        text.push_str("No problems found.\n");
        return Ok(text);
    }
    let count = report.problems.len();
    let _ = match count {
        1 => writeln!(text, "1 problem:"),
        _ => writeln!(text, "{count} problems:"),
    };
    for problem in &report.problems {
        let _ = writeln!(text, "  - {problem}");
    }
    Err(Reported {
        report: text,
        summary: match count {
            1 => "1 problem; the report above names its fix".into(),
            _ => format!("{count} problems; the report above names each fix"),
        },
    }
    .into())
}

/// The report line for the device skips the command ends once it has
/// done the rest (`boot_log::clear_device_skips`, [`end_device_skips`]):
/// empty when the USB's log names none. Nothing is written here.
pub(super) fn device_skips(esp: &Path, dry_run: bool) -> Result<String> {
    let log = esp.join(crate::efi::BOOT_LOG);
    let named = crate::boot_log::clear_device_skips(&log, "", true)?;
    Ok(match (named.is_empty(), dry_run) {
        (true, _) => String::new(),
        (false, true) => format!(
            "  device skips   {} would be started again\n",
            named.join(", ")
        ),
        (false, false) => format!(
            "  device skips   {} will be started again at the next boot\n",
            named.join(", ")
        ),
    })
}

/// End the device skips the USB's log at `esp` names, as `by`: the last
/// write of a command that succeeded, so one that stops keeps them and the
/// device that froze the PC stays skipped.
pub(super) fn end_device_skips(esp: &Path, by: &str) -> Result<()> {
    crate::boot_log::clear_device_skips(&esp.join(crate::efi::BOOT_LOG), by, false)?;
    Ok(())
}

/// `repair`: the device skips end once everything else is done.
pub(super) fn repair(environment: &Environment<'_>, command: &Request) -> Result<String> {
    let text = repair_steps(environment, command)?;
    if !command.dry_run {
        let record = read_record(environment, &command.usb)?;
        end_device_skips(&record.esp, "stela repair")?;
    }
    Ok(text)
}

fn repair_steps(environment: &Environment<'_>, command: &Request) -> Result<String> {
    let usb = &command.usb;
    let mut record = read_record(environment, usb)?;
    update::ready(&record, usb)?;
    if !command.dry_run {
        super::release::allow_older(
            environment,
            &record,
            command.yes,
            "repair it with this older stela",
        )?;
    }
    // A macOS switch that stopped part way never comes here: `repair`
    // finishes it as an update (`Request::dispatch`).
    debug_assert!(record.replacing.is_none());
    let esp = record.esp.clone();
    let mut text = format!("Stela repair on {}:\n", esp.display());
    text.push_str(&device_skips(&esp, command.dry_run)?);
    let Ok(tool) = fs::read(esp.join(LOADER_PATH)) else {
        bail!("{} has no stela.efi: {}", esp.display(), fix(usb, "update"));
    };
    if let Some(reason) =
        mount_interface::mismatch(environment.interface, mount_interface::find(&tool))
    {
        bail!(
            "the setup's stela.efi does not read what this stela writes ({reason}): run {} first",
            fix(usb, "update")
        );
    }
    update::check_firmware_support(&esp, &tool)?;
    // The firmware the last boot asked for and the USB lacks.
    let requested = super::firmware::requested(&esp, &tool)?;
    record.install_image = command.install_image.or(record.install_image);
    // With --ipsw, or when a machine has no ECID yet (a setup older than
    // per-machine identities), every restore bundle is prepared again and
    // machines without an ECID get one; else the damaged and missing
    // bundles and medium are put back. What that needs of the restore image,
    // and where it comes from, is decided before anything is downloaded.
    let rebuild = command.ipsw.is_some()
        || (record.installs_media() && record.installs.iter().any(|i| i.ecid.is_none()));
    let has_media = record.installs_media();
    // The full medium's bytes, read whole when asked or when an install
    // found a piece damaged since it was last checked: a damaged piece is
    // written again like a missing one, and removed only after everything
    // that can refuse the repair.
    let mut checked = None;
    if let Some(medium) = record
        .active_medium()
        .filter(|m| !m.remote && m.problems().is_empty())
        .cloned()
    {
        let reported =
            super::medium::reported_damage(&record.esp, record.medium_checked.unwrap_or(0));
        if command.full || !reported.is_empty() {
            let damaged = medium.damaged_pieces()?;
            match damaged.is_empty() {
                true => writeln!(
                    text,
                    "  install medium every piece read and checked: intact{}",
                    match reported.is_empty() {
                        true => String::new(),
                        false => format!(
                            " (the boot log named {}, which reads back correctly now)",
                            reported.join(", ")
                        ),
                    }
                )?,
                false => writeln!(
                    text,
                    "  install medium {} damaged: {}",
                    damaged.join(", "),
                    match command.dry_run {
                        true => "would be removed and written again",
                        false => "removed and written again",
                    }
                )?,
            }
            if let Some(active) = record.media.iter_mut().find(|m| m.build == medium.build) {
                active.damaged = damaged;
            }
            checked = Some(super::medium::log_head(&record.esp));
        }
    } else if command.full {
        writeln!(
            text,
            "  install medium {}",
            match record.active_medium() {
                Some(m) if m.remote => "streams from Apple: no piece is on the stick to check",
                Some(_) => "incomplete: the missing or short pieces are written again",
                None => "none",
            }
        )?;
    }
    let damaged_bundles = has_media
        && super::restore_bundles(&record.install_machines())
            .into_iter()
            .any(|(kind, ecid)| !bundle_intact(&record, kind, ecid));
    let bundles = rebuild || damaged_bundles;
    let need = has_media
        .then(|| sources::need(&record, &record.build, bundles))
        .flatten();
    let (build, model, active) = (
        record.build.clone(),
        record.model.clone(),
        record.active_medium().cloned(),
    );
    let lookup = sources::Lookup {
        need: need.unwrap_or(sources::Need::BootFiles),
        build: &build,
        model: &model,
        ipsw: command.ipsw.as_deref(),
        download_dir: command.download_dir.as_deref(),
        medium: active.as_ref(),
    };
    let source = match need {
        None => None,
        Some(need) => {
            if bundles {
                check_bundle_revision(environment, &record)?;
            }
            let source = sources::locate(environment, &lookup).map_err(|error| {
                anyhow::anyhow!("repair needs the macOS {build} restore image; but {error:#}")
            })?;
            if need.whole() {
                sources::check_room(environment, &record, &build, &source)?;
            }
            if !command.dry_run {
                super::consent_download(
                    environment,
                    command.yes,
                    false,
                    sources::download_bytes(&source),
                    &format!("what this stick lacks of macOS {build}"),
                )?;
            }
            Some(source)
        }
    };
    // Every missing bundle file goes into the cache before the stick
    // changes; a failed download leaves it as it was.
    if !command.dry_run {
        super::firmware::prefetch_bundle(&tool, Some(&record.volume))?;
    }
    let mut notes = Vec::new();
    // Disk names change with plug order and reboots: each install's
    // partitions are found again by their GUIDs and serials, and recorded
    // under their new names.
    for install in &mut record.installs {
        if install.disk.is_chosen() {
            continue;
        }
        for (role, recorded) in [
            (
                installed::Role::AppleRegion,
                install.disk.apple_region.clone(),
            ),
            (installed::Role::Nor, install.disk.nor.clone()),
        ] {
            let Ok(found) = installed::confirm(environment.host, &install.disk, role, None) else {
                continue;
            };
            if found == recorded {
                continue;
            }
            writeln!(
                text,
                "  disks          install \"{}\": {recorded} is now {found}",
                install.name
            )?;
            match role {
                installed::Role::AppleRegion => install.disk.apple_region = found.clone(),
                installed::Role::Nor => install.disk.nor = found,
            }
            // A machine state in the region's tail is named by the region.
            if install.disk.machine_state_offset.is_some() {
                install.disk.nor = install.disk.apple_region.clone();
            }
        }
    }
    let mut assigned: Vec<(String, Ecid)> = Vec::new();
    if rebuild {
        for index in 0..record.installs.len() {
            if record.installs[index].ecid.is_some() {
                continue;
            }
            let install = record.installs[index].clone();
            let area = installed::state_area(environment.host, &install.disk, None)
                .and_then(|area| {
                    area.read(
                        environment.host,
                        CAPACITY as u64,
                        stela_formats::machine_state::SEP_ROOTS_BYTES,
                    )
                })
                .with_context(|| {
                    format!(
                        "install \"{}\" needs an ECID, which depends on its machine state; \
                         connect its disk",
                        install.name
                    )
                })?;
            let provisioned = state::sep(&area).unwrap_or(false);
            let ecid = machines::machine_ecid(
                machines::EcidOptions {
                    given: command.ecid,
                    recorded: None,
                    sep_provisioned: provisioned,
                    new_ecid: command.new_ecid,
                },
                &mut notes,
            )?;
            record.installs[index].ecid = Some(ecid);
            assigned.push((install.name.clone(), ecid));
        }
    }
    for (name, ecid) in &assigned {
        writeln!(text, "  ECID           install \"{name}\" is now {ecid}")?;
    }
    // What interrupted copies left, before any bundle is judged; a dry run
    // judges a set-aside bundle where it would be put back.
    for directory in record.build_directories() {
        for (destination, what) in bundle::leftovers(&directory) {
            if command.dry_run {
                writeln!(text, "  bundle         would put right: {what}")?;
                continue;
            }
            for done in bundle::recover(&destination)? {
                writeln!(text, "  bundle         {done}")?;
            }
        }
    }
    let stale = record.drop_stale_bundles();
    for old in &stale {
        writeln!(
            text,
            "  remove         {}",
            old.directory(&record.volume).display()
        )?;
    }
    // What an interrupted update left beside the active build and
    // generation.
    let leftovers = record.has_leftovers();
    if leftovers {
        writeln!(
            text,
            "  remove         the files an interrupted update left beside macOS {}'s",
            record.build
        )?;
    }
    // The install's restore bundles and medium: all again with an image.
    // A setup that has written media needs the active build's: it is put
    // back when damaged or missing.
    let damaged_medium = has_media && !sources::medium_ready(&record, &record.build);
    let mut restore: Vec<(bundle::Kind, Option<Ecid>)> = Vec::new();
    if has_media {
        for (kind, ecid) in super::restore_bundles(&record.install_machines()) {
            if rebuild || !bundle_intact(&record, kind, ecid) {
                restore.push((kind, ecid));
            }
        }
    }
    let install = match (&source, restore.is_empty() && !damaged_medium) {
        (_, true) => None,
        (Some(source), false) => {
            writeln!(
                text,
                "  restore image  {}",
                sources::describe(source, &lookup)
            )?;
            if command.dry_run && !source.is_local() {
                // Nothing is read or downloaded to prepare the plan.
                writeln!(
                    text,
                    "  install        {} restore bundles of macOS {build}{}",
                    restore.len(),
                    if damaged_medium {
                        " and its install medium"
                    } else {
                        ""
                    }
                )?;
                None
            } else {
                let obtained = sources::obtain(environment, source, &lookup)?;
                let environment = Environment {
                    download: obtained.download.as_ref(),
                    ..*environment
                };
                let plan = plan_install(
                    &environment,
                    &record,
                    super::create::InstallOptions {
                        build: &record.build,
                        ipsw: &obtained.path,
                        bundles: &restore,
                        additional_bytes: 0,
                    },
                )?;
                describe_install(&record, &plan, &mut text)?;
                Some((plan, obtained))
            }
        }
        (None, false) => bail!("repair found nothing to prepare its restore files from"),
    };
    writeln!(
        text,
        "  boot loader    stela.efi, stela.cfg and the startup choices as recorded"
    )?;
    for line in requested.lines() {
        writeln!(text, "  firmware       {line}")?;
    }
    // Machine state stela.efi would refuse: replaced with
    // --initialize-nor, else named.
    // A disk that is not connected keeps its machine state as it is; the
    // USB's own files are repaired all the same.
    let mut reinitialize = Vec::new();
    for install in &record.installs {
        let lost = installed::lost_keys(install, usb, record.installs.len() > 1);
        let fresh = match installed::reinitialize(environment.host, &install.disk, &lost) {
            Ok(Some(fresh)) => fresh,
            Ok(None) => continue,
            Err(error) => {
                notes.push(unreachable_note(&install.name, &error));
                continue;
            }
        };
        match (&fresh.refusal, command.initialize_nor) {
            (Some(refusal), true) => {
                return Err(Reported {
                    report: text,
                    summary: format!("install \"{}\": {refusal}", install.name),
                }
                .into());
            }
            (Some(refusal), false) => {
                notes.push(format!("install \"{}\": {refusal}", install.name))
            }
            (None, true) => {
                for change in &fresh.changes {
                    writeln!(text, "  disks          {change}")?;
                }
                reinitialize.push(fresh);
            }
            (None, false) => notes.push(format!(
                "install \"{}\"'s machine state would be refused by stela.efi; {} replaces it",
                install.name,
                fix(usb, "repair --initialize-nor")
            )),
        }
    }
    // An SSD partition holding another file system: erased with
    // --erase-target, else named.
    let mut foreign = Vec::new();
    for install in &record.installs {
        let region = match installed::foreign(environment.host, &install.disk) {
            Ok(Some(region)) => region,
            Ok(None) => continue,
            Err(error) => {
                // Named once, as the machine-state check above names it.
                let note = unreachable_note(&install.name, &error);
                if !notes.contains(&note) {
                    notes.push(note);
                }
                continue;
            }
        };
        if command.erase_target {
            writeln!(
                text,
                "  disks          erase the file system on {}, install \"{}\"'s SSD partition",
                region.identifier, install.name
            )?;
            foreign.push((region, install.disk.machine_state_offset));
        } else {
            notes.push(format!(
                "{}: {} erases it",
                foreign_problem(&install.name, &region.identifier),
                fix(usb, "repair --erase-target")
            ));
        }
    }
    // An automatic Metal plugin repair that ran and did not install the
    // plugin runs again at the install's next start, as the stop it left
    // asks; a PC without a serial console has no other way. A chosen disk's
    // is reached where this computer holds that disk: the PC itself.
    let mut bundle_repairs = Vec::new();
    for install in &record.installs {
        let again = install_state_area(environment.host, &record, install).and_then(|area| {
            area.map(|area| installed::bundle_repair_again(environment.host, area))
                .transpose()
                .map(Option::flatten)
        });
        match again {
            Ok(Some(again)) => {
                writeln!(
                    text,
                    "  Metal plugin   install \"{}\": the automatic repair {} at the next start",
                    install.name,
                    match command.dry_run {
                        true => "would run again",
                        false => "runs again",
                    }
                )?;
                bundle_repairs.push(again);
            }
            Ok(None) => {}
            Err(error) => {
                let note = unreachable_note(&install.name, &error);
                if !notes.contains(&note) {
                    notes.push(note);
                }
            }
        }
    }
    for note in &notes {
        writeln!(text, "  note           {note}")?;
    }
    let store_repairs = record
        .translations
        .as_ref()
        .map(super::translations::Store::repairs)
        .unwrap_or_default();
    for repair in &store_repairs {
        writeln!(text, "  translations   {repair}")?;
    }
    if command.dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    // Nothing refuses the repair from here: the damaged pieces go, and the
    // install below writes them again as missing ones.
    if let Some(medium) = record.media.iter_mut().find(|m| !m.damaged.is_empty()) {
        for piece in std::mem::take(&mut medium.damaged) {
            let path = medium.directory().join(&piece);
            fs::remove_file(&path).with_context(|| format!("cannot remove {}", path.display()))?;
        }
    }
    if let Some(store) = &record.translations
        && !store_repairs.is_empty()
    {
        store.repair()?;
    }
    for (region, state_offset) in &foreign {
        installed::erase_foreign(environment.host, region, *state_offset)?;
    }
    for fresh in &reinitialize {
        fresh.area.write(environment.host, 0, &fresh.bytes)?;
        ensure!(
            fresh.area.read(environment.host, 0, fresh.bytes.len())? == fresh.bytes,
            "{} did not read back as written",
            fresh.area.describe()
        );
    }
    for again in &bundle_repairs {
        again.write(environment.host)?;
    }
    if let Some((plan, obtained)) = install {
        let environment = Environment {
            download: obtained.download.as_ref(),
            ..*environment
        };
        apply_install(&environment, &mut record, &obtained.path, plan)?;
    }
    // A check that found the medium intact; one that removed pieces had
    // them written again above, which records its own.
    if checked.is_some() && record.medium_checked < checked {
        record.medium_checked = checked;
    }
    publish(&mut record)?;
    write_record(&esp, &record)?;
    super::remove_bundles(&record, &stale)?;
    if leftovers {
        update::prune(&mut record)?;
        write_record(&esp, &record)?;
    }
    // The complete firmware bundle, which an older USB lacks.
    for note in super::firmware::provision_bundle(environment.host, &record.volume, &esp, &tool)? {
        writeln!(text, "  firmware       {note}")?;
    }
    if !stela_loader::host_firmware::reads_bundle(&tool) {
        requested.provision(&esp, &tool)?;
    }
    writeln!(text, "Done. Check it with {}", fix(usb, "status"))?;
    Ok(text)
}

/// Whether `record`'s restore bundle of `kind` for `ecid` reads back as
/// written, judged where an interrupted copy set it aside.
fn bundle_intact(record: &Record, kind: bundle::Kind, ecid: Option<Ecid>) -> bool {
    record.restore_bundle(kind, ecid).is_some_and(|b| {
        bundle::verify(
            &bundle::in_place(&b.directory(&record.volume)),
            &b.manifest_sha256,
        )
        .is_empty()
    })
}

/// Why an install's disk was left alone.
fn unreachable_note(install: &str, error: &anyhow::Error) -> String {
    format!("install \"{install}\"'s disk is left as it is: {error:#}")
}
