//! Creating a setup: the plan of what it writes, its description, and
//! writing it.
use super::{
    CHOICES_PATH, CHOOSER_SECONDS, Environment, INSTALL_CHOICE, InstallImage, LOADER_PATH, MARGIN,
    RECORD, Request, assets,
    bundle::{self, Kind},
    ecid::Ecid,
    firmware, format,
    host::{self, Host},
    image_is_remote, installed, locate_esp, machines, medium,
    prepare::{
        Prepared, PreparedSetup, copy_bundles, download_dir, prepare_kind, prepare_restore_bundles,
        supported,
    },
    read_efi,
    record::{RECORD_KIND, RECORD_SCHEMA, Record, publish, write_record},
    translations, update, wifi, wizard,
};
use crate::{efi, restore};
use anyhow::{Context, Result, bail, ensure};
use std::{
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
};
use stela_formats::builds;

/// The setup USB's one unmounted partition besides its EFI partition: a
/// raw partition, which becomes the Mac's internal SSD. `None` when the stick
/// has none; several are refused, since setup cannot tell them apart.
pub(super) fn stick_ssd(host: &dyn Host, usb_disk: Option<&str>) -> Result<Option<String>> {
    let Some(usb_disk) = usb_disk else {
        return Ok(None);
    };
    let disks = host.disks()?;
    let Some(disk) = disks.iter().find(|disk| disk.identifier == usb_disk) else {
        return Ok(None);
    };
    let raw: Vec<&str> = disk
        .partitions
        .iter()
        .filter(|p| p.content != "EFI" && p.mount.is_none())
        .map(|p| p.identifier.as_str())
        .collect();
    match raw[..] {
        [] => Ok(None),
        [partition] => Ok(Some(partition.into())),
        _ => bail!(
            "{usb_disk} has several unmounted partitions ({}); name the Mac's internal SSD with \
             --target",
            raw.join(", ")
        ),
    }
}

/// The install medium setup writes.
pub(super) struct MediumPlan {
    pub(super) volume: PathBuf,
    pub(super) guid: String,
    /// It names the IPSW on Apple's servers (`medium::write_remote`).
    pub(super) remote: bool,
    pub(super) ipsw_bytes: u64,
    /// Room it needs: the bytes still to write, less a damaged piece's
    /// that goes first, with the hash files and a margin.
    pub(super) need: u64,
    /// The bytes of the pieces still to write.
    pub(super) write: u64,
    pub(super) expected: medium::Expected,
    /// Directories it creates, deepest first.
    pub(super) created: Vec<PathBuf>,
}

/// What installing macOS needs: each machine's restore bundles and, unless
/// the stick holds the build's intact medium, the install medium.
pub(super) struct InstallPlan {
    /// The macOS build it installs.
    pub(super) build: String,
    pub(super) restore: Vec<Prepared>,
    pub(super) medium: Option<MediumPlan>,
    /// The boot files a later update prepares its bundles from, kept on the
    /// USB when the medium streams the image (`inputs`): their bytes.
    pub(super) inputs: u64,
}

/// The restore bundles of both kinds for each of `ecids`.
pub(super) fn restore_bundles(ecids: &[Option<Ecid>]) -> Vec<(Kind, Option<Ecid>)> {
    bundle::RESTORE_KINDS
        .into_iter()
        .flat_map(|kind| ecids.iter().map(move |&ecid| (kind, ecid)))
        .collect()
}

/// Restore image, selected machine bundles and additional volume bytes.
pub(super) struct InstallOptions<'a> {
    pub(super) build: &'a str,
    pub(super) ipsw: &'a Path,
    pub(super) bundles: &'a [(Kind, Option<Ecid>)],
    pub(super) additional_bytes: u64,
}

/// Plan the install of `build` from `ipsw` for `record`: the restore
/// bundles `bundles` names and the build's install medium; and check the
/// data partition's space: `also` is what else setup writes to it.
pub(super) fn plan_install(
    environment: &Environment<'_>,
    record: &Record,
    options: InstallOptions<'_>,
) -> Result<InstallPlan> {
    let InstallOptions {
        build,
        ipsw,
        bundles,
        additional_bytes: also,
    } = options;
    let host = environment.host;
    let mut restore = Vec::new();
    for kind in bundle::RESTORE_KINDS {
        let ecids: Vec<Option<Ecid>> = bundles
            .iter()
            .filter(|(k, _)| *k == kind)
            .map(|&(_, ecid)| ecid)
            .collect();
        if !ecids.is_empty() {
            let (_, _, prepared) = prepare_kind(
                environment,
                super::prepare::BundleOptions {
                    kind,
                    ipsw,
                    model: &record.model,
                    build: Some(build),
                    ecids: &ecids,
                },
            )?;
            restore.extend(prepared);
        }
    }
    // A medium that stays needs nothing of the image but its boot files.
    let intact = super::sources::medium_ready(record, build);
    let remote = !intact && environment.download.is_none() && record.remote_install(ipsw)?;
    let medium = match intact {
        true => None,
        false if remote => Some(MediumPlan {
            volume: record.volume.clone(),
            guid: String::new(),
            remote,
            ipsw_bytes: 0,
            need: MARGIN,
            write: 0,
            expected: medium::Expected::default(),
            created: missing_directories(&record.volume, &medium::directory(&record.volume, build)),
        }),
        false => {
            let receipt = restore::receipt(ipsw)?;
            let (volume, guid) = medium::locate(host, &record.esp, Some(&record.volume))?;
            let ipsw_bytes = match environment.download {
                Some(download) => download.bytes,
                None => fs::metadata(ipsw)?.len(),
            };
            let need = medium::needed_bytes(&volume, build, ipsw_bytes, environment.piece_bytes);
            let damaged = record
                .medium_of(build)
                .map_or(&[][..], |m| m.damaged.as_slice());
            let write =
                medium::missing_bytes(&volume, build, ipsw_bytes, environment.piece_bytes, damaged);
            let created = missing_directories(&volume, &medium::directory(&volume, build));
            Some(MediumPlan {
                volume,
                guid,
                remote,
                ipsw_bytes,
                need,
                write,
                expected: match environment.download {
                    Some(download) => medium::Expected {
                        sha256: None,
                        sha1: download.image.sha1.clone(),
                    },
                    None => medium::Expected::of(receipt.as_ref()),
                },
                created,
            })
        }
    };
    // The medium and the bundles share the data partition.
    // The boot files a later update prepares from count with the bundles.
    let inputs = super::inputs::needed(environment, record, build, ipsw)?;
    let restore_bytes: u64 = restore.iter().map(|p| p.bytes).sum::<u64>() + inputs;
    let need = medium.as_ref().map_or(MARGIN, |plan| plan.need) + restore_bytes + also;
    let free = host.volume(&record.volume)?.free_bytes;
    ensure!(
        free >= need,
        "{} has {} free; the install medium and boot bundles need {}{}. Free space on it, or \
         use a larger stick.",
        record.volume.display(),
        host::bytes_text(free),
        host::bytes_text(need),
        match &medium {
            Some(plan) if !plan.remote => {
                format!(
                    " (the macOS {build} IPSW is {})",
                    host::bytes_text(plan.ipsw_bytes)
                )
            }
            _ => String::new(),
        }
    );
    Ok(InstallPlan {
        build: build.into(),
        restore,
        medium,
        inputs,
    })
}

/// The plan's lines, after their two spaces.
pub(super) fn describe_install(
    record: &Record,
    plan: &InstallPlan,
    text: &mut String,
) -> Result<()> {
    for bundle in &plan.restore {
        writeln!(
            text,
            "  {:<14} {} ({})",
            bundle.kind.label(),
            bundle::directory_of(
                &record.volume,
                bundle::Identity {
                    kind: bundle.kind,
                    build: &bundle.build,
                    ecid: bundle.ecid,
                    generation: record.generation()
                }
            )
            .display(),
            host::bytes_text(bundle.bytes)
        )?;
    }
    if plan.inputs > 0 {
        writeln!(
            text,
            "  boot files     {} ({}), which later updates prepare the bundles from",
            super::inputs::path(&record.volume, &plan.build).display(),
            host::bytes_text(plan.inputs)
        )?;
    }
    if let Some(medium) = plan.medium.as_ref().filter(|m| m.remote) {
        writeln!(
            text,
            "  installer      Online: {} lists macOS {}; Install macOS downloads it from Apple \
             over the PC's internet (Ethernet, or the Wi-Fi network setup names)",
            medium::directory(&medium.volume, &plan.build).display(),
            plan.build,
        )?;
    } else if let Some(medium) = &plan.medium {
        writeln!(
            text,
            "  installer      Offline: {} holds the macOS {} IPSW, on partition {} ({} to write)",
            medium::directory(&medium.volume, &plan.build).display(),
            plan.build,
            medium.guid,
            host::bytes_text(medium.write)
        )?;
        if medium.expected.sha1.is_none() {
            writeln!(
                text,
                "  note           no download receipt: the IPSW is checked against its \
                 BuildManifest only"
            )?;
        }
    }
    Ok(())
}

/// Copy the restore bundles and write the install medium.
pub(super) fn apply_install(
    environment: &Environment<'_>,
    record: &mut Record,
    ipsw: &Path,
    plan: InstallPlan,
) -> Result<()> {
    copy_bundles(record, &plan.restore)?;
    if plan.inputs > 0 {
        super::inputs::keep(environment, record, &plan.build, ipsw)?;
    }
    write_record(&record.esp.clone(), record)?;
    let build = plan.build;
    let Some(plan) = plan.medium else {
        return Ok(());
    };
    // A medium of the other kind or volume this one replaces, removed once
    // this one is written.
    let previous = record.medium_of(&build).cloned();
    // Recorded before the first piece, so an interrupted copy is undone.
    record.media.retain(|m| m.build != build);
    record.media.push(medium::Medium {
        build: build.clone(),
        volume: plan.volume.clone(),
        partition_guid: plan.guid.clone(),
        manifest_sha256: String::new(),
        remote: plan.remote,
        damaged: Vec::new(),
    });
    let mut created = record.created_directories.clone();
    for directory in plan.created {
        if !created.contains(&directory) {
            created.push(directory);
        }
    }
    created.sort_by_key(|p| std::cmp::Reverse(p.components().count()));
    record.created_directories = created;
    write_record(&record.esp.clone(), record)?;
    if plan.remote {
        let progress = crate::progress::Phase::spinner("Preparing the network install inventory");
        let inventory = (environment.remote_inventory)(ipsw, &record.model)?;
        progress.finish();
        ensure!(
            inventory.manifest.product_build == build,
            "Apple's servers hold macOS build {}, not {build}",
            inventory.manifest.product_build,
        );
        let sha256 = medium::write_remote(&plan.volume, inventory)?;
        if let Some(medium) = record.media.iter_mut().find(|m| m.build == build) {
            medium.manifest_sha256 = sha256;
        }
        write_record(&record.esp.clone(), record)?;
        return remove_replaced(record, &build, previous.as_ref());
    }
    if let Some(download) = environment.download {
        let written = download.write(&plan.volume, &record.model, environment.piece_bytes)?;
        if let Some(medium) = record.media.iter_mut().find(|m| m.build == build) {
            medium.manifest_sha256 = written.manifest_sha256;
        }
        // Every piece was checked or written and read back.
        record.medium_checked = Some(medium::log_head(&record.esp));
        write_record(&record.esp.clone(), record)?;
        return remove_replaced(record, &build, previous.as_ref());
    }
    // The inventory shows the restore image's hashing progress.
    let inventory = (environment.inventory)(ipsw, &record.model)?;
    ensure!(
        inventory.manifest.product_build == build,
        "{} holds macOS build {}, not {build}",
        ipsw.display(),
        inventory.manifest.product_build,
    );
    let written = medium::write(medium::WriteOptions {
        volume: &plan.volume,
        ipsw,
        inventory,
        expected: &plan.expected,
        piece_bytes: environment.piece_bytes,
    })?;
    if let Some(medium) = record.media.iter_mut().find(|m| m.build == build) {
        medium.manifest_sha256 = written.manifest_sha256;
    }
    // Every piece was checked or written and read back.
    record.medium_checked = Some(medium::log_head(&record.esp));
    write_record(&record.esp.clone(), record)?;
    remove_replaced(record, &build, previous.as_ref())
}

/// Remove what the medium of `build` that the one just written replaced
/// left (`medium::remove_replaced`).
pub(super) fn remove_replaced(
    record: &Record,
    build: &str,
    previous: Option<&medium::Medium>,
) -> Result<()> {
    match (previous, record.medium_of(build)) {
        (Some(previous), Some(current)) => medium::remove_replaced(previous, current),
        _ => Ok(()),
    }
}

/// What a resumed first setup keeps of the interrupted one.
struct Resume {
    name: String,
    ecid: Ecid,
    /// Its restore bundles, when every one is intact and of the generation
    /// this stela prepares; else empty, and they are prepared again.
    bundles: Vec<bundle::Bundle>,
    /// Its bundles of another generation, which an older stela prepared:
    /// removed before the new ones are copied.
    stale: Vec<bundle::Bundle>,
    /// Its install medium of the build: `plan_install` keeps it when intact
    /// and `medium::write` keeps each piece whose hash matches.
    media: Vec<medium::Medium>,
    translations: Option<translations::Store>,
    created_directories: Vec<PathBuf>,
}

/// The setup a resume is checked against.
struct ResumeRequest<'a> {
    command: &'a Request,
    ipsw: &'a Path,
    disk: &'a installed::InstalledDisk,
    volume: &'a Path,
}

impl Resume {
    /// Refuse a resume of a setup of another build, model, target, data
    /// volume, install name or bundle generation, naming each difference.
    fn check(
        environment: &Environment<'_>,
        previous: &Record,
        request: ResumeRequest<'_>,
    ) -> Result<Self> {
        let ResumeRequest {
            command,
            ipsw,
            disk,
            volume,
        } = request;
        let [install] = &previous.installs[..] else {
            bail!(
                "the interrupted setup on {} names {} installs; {}",
                previous.esp.display(),
                previous.installs.len(),
                super::START_OVER
            );
        };
        let mut differences = Vec::new();
        let (_, build) = restore::payload::identity(ipsw, command.model())?;
        if build != previous.build {
            differences.push(format!("macOS {} (this one {build})", previous.build));
        }
        if command.model() != previous.model {
            differences.push(format!(
                "model {} (this one {})",
                previous.model,
                command.model()
            ));
        }
        let same_disk = |a: &installed::InstalledDisk, b: &installed::InstalledDisk| {
            a.is_chosen() == b.is_chosen()
                && a.apple_region_guid
                    .eq_ignore_ascii_case(&b.apple_region_guid)
                && a.nor_guid.eq_ignore_ascii_case(&b.nor_guid)
                && a.machine_state_offset == b.machine_state_offset
        };
        if !same_disk(&install.disk, disk) {
            differences.push(format!(
                "target {} (this one {})",
                install.disk.partitions(),
                disk.partitions()
            ));
        }
        if previous.volume != volume {
            differences.push(format!(
                "data volume {} (this one {})",
                previous.volume.display(),
                volume.display()
            ));
        }
        if let Some(name) = command
            .install
            .as_ref()
            .filter(|name| **name != install.name)
        {
            differences.push(format!(
                "install \"{}\" (this one \"{name}\")",
                install.name
            ));
        }
        // Bundles an older stela prepared are prepared again, as damaged
        // ones are; the rest of the setup is kept.
        let generation = environment.generation();
        let Some(ecid) = install.ecid else {
            bail!(
                "the interrupted setup on {} names no ECID; {}",
                previous.esp.display(),
                super::START_OVER
            );
        };
        if !differences.is_empty() {
            bail!(
                "{} holds an interrupted setup that differs from this one: {}. Finish it with {}, \
                 or start over with `stela create` on the stick's whole disk, which erases it",
                previous.esp.display(),
                differences.join(", "),
                super::fix(&command.usb, "repair")
            );
        }
        // Every restore bundle intact and current, or all prepared again.
        let current = previous.generation() == generation;
        let intact = current
            && bundle::RESTORE_KINDS.into_iter().all(|kind| {
                previous
                    .restore_bundle(kind, Some(ecid))
                    .is_some_and(|kept| {
                        bundle::verify(&kept.directory(&previous.volume), &kept.manifest_sha256)
                            .is_empty()
                    })
            });
        let bundles = match intact {
            true => previous
                .bundles
                .iter()
                .filter(|b| b.build == build && b.generation() == generation)
                .cloned()
                .collect(),
            false => Vec::new(),
        };
        Ok(Self {
            name: install.name.clone(),
            ecid,
            bundles,
            stale: previous
                .bundles
                .iter()
                .filter(|b| b.generation() != generation)
                .cloned()
                .collect(),
            media: previous
                .media
                .iter()
                .filter(|m| m.build == build)
                .cloned()
                .collect(),
            translations: previous.translations.clone(),
            created_directories: previous.created_directories.clone(),
        })
    }
}

/// [`SIGNING_NOTE`] in a sentence, as the wizard says it before a create.
pub(super) const SIGNING_SUMMARY: &str = "Apple's signing service receives this machine's ECID \
    and the restore's hashes to sign the restore.";

/// What an install sends to Apple's signing service; the plan says it
/// before setup writes the stick.
pub(super) const SIGNING_NOTE: &str = "Apple's signing service (gs.apple.com) receives this \
    machine's ECID, the restore's nonces and its component hashes, to sign the restore's root \
    tickets, its Cryptex1 ticket and the recoveryOS LocalPolicy: setup asks while it prepares \
    the boot files, and the install asks only for what the stick lacks.";

/// An Offline installer with every signing answer on the stick. The install
/// checks the answers before it changes a disk, and asks Apple only when
/// they do not fit it.
pub(super) const OFFLINE_NOTE: &str = "Offline: macOS and Apple's signing answers are on the \
    stick, so the install needs no internet; it checks the answers first and asks Apple only if \
    they do not fit.";
/// An Offline installer whose signing answers are not all on the stick.
pub(super) const OFFLINE_SIGNING_NOTE: &str = "Offline: macOS is on the stick; the install needs \
    internet only to ask Apple's signing server.";
/// An Online installer.
pub(super) const ONLINE_NOTE: &str = "Online: the install downloads macOS, so the PC needs \
    internet (Ethernet, or the Wi-Fi network setup names).";
/// What is not known yet about the installed system without a network.
pub(super) const FIRST_START_NOTE: &str = "After the install, macOS may ask for a network the \
    first time it starts: whether Setup Assistant's activation finishes without one is not \
    tested yet.";

/// Whether the restore bundle in `directory` holds Apple's answers to every
/// signing request its install makes (`tss::staged`).
fn staged(directory: &Path) -> bool {
    fs::read(directory.join(stela_runtime::restore::tss::staged::FILE))
        .ok()
        .and_then(|bytes| stela_runtime::restore::tss::staged::Set::decode(&bytes).ok())
        .is_some_and(|set| set.complete())
}

/// Whether an install needs no network: macOS on the stick and every restore
/// bundle holding all of its signing answers.
fn offline(remote: bool, restore_bundles: &[PathBuf]) -> bool {
    !remote && !restore_bundles.is_empty() && restore_bundles.iter().all(|b| staged(b))
}

/// The installer and what its install needs of the internet, as the plan
/// and `status` say it: [`OFFLINE_NOTE`] with macOS and every signing
/// answer on the stick, else [`OFFLINE_SIGNING_NOTE`] or [`ONLINE_NOTE`].
pub(super) fn network_status(remote: bool, restore_bundles: &[PathBuf]) -> &'static str {
    match (remote, offline(remote, restore_bundles)) {
        (true, _) => ONLINE_NOTE,
        (false, true) => OFFLINE_NOTE,
        (false, false) => OFFLINE_SIGNING_NOTE,
    }
}

/// What the plan says about the installer and the internet
/// ([`network_status`]), and about macOS's first start without it.
pub(super) fn network_notes(remote: bool, offline: bool) -> Vec<String> {
    match (remote, offline) {
        (true, _) => vec![ONLINE_NOTE.into()],
        (false, true) => vec![OFFLINE_NOTE.into(), FIRST_START_NOTE.into()],
        (false, false) => vec![OFFLINE_SIGNING_NOTE.into()],
    }
}

/// The interactive setup's last word on the internet, one sentence, by
/// the installer note of the plan in `report` ([`network_notes`]).
pub(super) fn network_next_step(report: &str) -> &'static str {
    if report.contains(OFFLINE_NOTE) {
        "Offline: the install needs no internet."
    } else if report.contains(OFFLINE_SIGNING_NOTE) {
        "Offline: the install needs internet only to ask Apple's signing server."
    } else {
        "Online: the install downloads macOS, so connect the PC to the internet by Ethernet or \
         Wi-Fi."
    }
}

/// The checks' result: everything setup will write, and where.
pub(super) struct Plan {
    pub(super) record: Record,
    pub(super) ipsw: PathBuf,
    pub(super) product_version: String,
    /// The prepared restore bundles, in a host directory until setup
    /// copies them.
    pub(super) prepared: Vec<Prepared>,
    pub(super) efi_image: Vec<u8>,
    /// The install's disk, and the NOR image setup writes to it.
    pub(super) installed: installed::Checked,
    /// The install medium.
    pub(super) install: InstallPlan,
    pub(super) notes: Vec<String>,
    /// The key file's passphrase, for `--wifi`.
    pub(super) wifi_passphrase: Option<String>,
    /// The bundles an older stela prepared for the interrupted setup this
    /// one resumes, which its own replace.
    pub(super) stale_bundles: Vec<bundle::Bundle>,
}

pub(super) fn plan(
    environment: &Environment<'_>,
    command: &Request,
    ipsw: PathBuf,
    early: Option<PreparedSetup>,
) -> Result<Plan> {
    let host = environment.host;
    let esp = locate_esp(host, &command.usb)?;
    // An interrupted first setup is resumed; any other is refused.
    let previous = match esp.join(RECORD).exists() {
        false => None,
        true => match super::record::interrupted(host, &esp, &command.usb) {
            Some(previous) => Some(previous),
            None => bail!(
                "{} is already set up ({}). Update it with {} or {}, or {}",
                esp.display(),
                esp.join(RECORD).display(),
                super::fix(&command.usb, "update"),
                super::fix(&command.usb, "update --macos VERSION"),
                super::START_OVER
            ),
        },
    };
    let usb_disk = host.volume(&esp)?.disk;
    // Without --target, a raw partition of the stick is the Mac's internal
    // SSD, and the PC's disks its external drives.
    let stick = match &command.target {
        Some(_) => None,
        None => stick_ssd(host, usb_disk.as_deref())?,
    };
    let target = command.target.clone().or(stick.clone());
    // With no disk named here, Install macOS chooses the PC's disk in
    // stela.efi.
    let installed = match &target {
        None => installed::Checked {
            disk: installed::InstalledDisk::chosen(),
            nor_image: None,
            sep_provisioned: false,
            foreign: None,
            notes: Vec::new(),
        },
        Some(target) => installed::check(
            host,
            &installed::Request {
                apple_region: target,
                nor: command.nor.as_deref(),
                serial: command.disk_serial.as_deref(),
                erase_target: command.erase_target,
                initialize_nor: command.initialize_nor,
                usb_disk: usb_disk.as_deref(),
                external_disks: stick.is_some(),
            },
        )?,
    };
    let mut notes = Vec::new();
    // The bundles, the install medium and the translation store go on the
    // USB's FAT32 data partition, which the firmware reads.
    let (volume, _) = medium::locate(host, &esp, command.install_volume.as_deref())?;
    let resume = match &previous {
        Some(previous) => Some(Resume::check(
            environment,
            previous,
            ResumeRequest {
                command,
                ipsw: &ipsw,
                disk: &installed.disk,
                volume: &volume,
            },
        )?),
        None => None,
    };
    let ecid = machines::machine_ecid(
        machines::EcidOptions {
            given: command.ecid,
            recorded: resume.as_ref().map(|r| r.ecid),
            sep_provisioned: installed.sep_provisioned,
            new_ecid: command.new_ecid,
        },
        &mut notes,
    )?;
    let efi_path = match &command.efi {
        Some(path) => path.clone(),
        None => assets::resolve()?,
    };
    let efi_image = read_efi(environment, &efi_path)?;
    update::check_firmware_support(&esp, &efi_image)?;
    // The restore bundles are prepared on this computer before the stick
    // changes, which also measures them for the space check. A resumed
    // setup keeps its intact ones.
    let (product_version, build, prepared) = match (early, &resume) {
        (Some(prepared), _) => prepared,
        (None, Some(resume)) if !resume.bundles.is_empty() => {
            let (version, build) = restore::payload::identity(&ipsw, command.model())?;
            notes.push("the interrupted setup's boot files are intact and kept".into());
            (version, build, Vec::new())
        }
        (None, _) => prepare_restore_bundles(environment, &ipsw, command.model(), ecid)?,
    };
    let bundles: u64 = prepared.iter().map(|bundle| bundle.bytes).sum();
    let install = machines::Install {
        name: resume.as_ref().map_or_else(
            || {
                command
                    .install
                    .clone()
                    .unwrap_or_else(|| machines::FIRST_NAME.into())
            },
            |resume| resume.name.clone(),
        ),
        ecid: Some(ecid),
        disk: installed.disk.clone(),
    };
    machines::check_name(&install.name)?;
    let mut record = Record {
        schema_version: RECORD_SCHEMA,
        kind: RECORD_KIND.into(),
        build: build.clone(),
        model: command.model().into(),
        esp: esp.clone(),
        bundle_directory: bundle::build_directory(&volume, &build),
        created_directories: missing_directories(
            &volume,
            &bundle::build_directory(&volume, &build),
        ),
        volume: volume.clone(),
        payload_directory: None,
        options: String::new(),
        complete: false,
        installed: None,
        stela_version: Some(update::version_id(&efi_image)),
        release: Some(super::release::CURRENT.into()),
        installs: vec![install],
        default_install: None,
        bundle_schema: environment.interface.bundle_schema,
        bundle_revision: environment.bundle_revision,
        bundles: Vec::new(),
        chooser_timeout: match command.chooser_timeout {
            Some(super::command::Wait::Seconds(seconds)) => Some(seconds),
            _ => None,
        },
        medium: None,
        media: Vec::new(),
        verify_on_read: command.verify_on_read,
        profile: command.profile,
        install_image: command.install_image,
        replacing: None,
        medium_checked: None,
        translations: None,
        wifi: command.wifi.clone().map(|ssid| wifi::Network {
            ssid,
            country: command
                .wifi_country
                .clone()
                .or_else(environment.wifi_country),
        }),
        usb4_approved: command.usb4_approve.clone(),
        addresses: super::addresses::Addresses::requested(command)?,
    };
    // What the interrupted setup wrote and this one keeps: its intact
    // bundles, its medium (kept whole when intact, else piece by piece by
    // its hashes), its translation store and the directories it created.
    let mut stale_bundles = Vec::new();
    if let Some(resume) = resume {
        notes.push(format!(
            "resumes the setup interrupted on {}",
            esp.join(RECORD).display()
        ));
        if let Some(previous) = &previous {
            notes.push(previous.resume_summary());
            if !resume.stale.is_empty() {
                notes.push(format!(
                    "the interrupted setup's boot files are an older stela's ({}); this stela \
                     prepares its own ({}) and removes those",
                    previous.generation(),
                    environment.generation()
                ));
            }
        }
        stale_bundles = resume.stale;
        record.bundles = resume.bundles;
        record.media = resume.media;
        record.translations = resume.translations;
        for directory in resume.created_directories {
            if !record.created_directories.contains(&directory) {
                record.created_directories.push(directory);
            }
        }
        record
            .created_directories
            .sort_by_key(|p| std::cmp::Reverse(p.components().count()));
    }
    // The restore bundles are prepared above: the install plan adds the
    // install medium.
    let install = plan_install(
        environment,
        &record,
        InstallOptions {
            build: &record.build,
            ipsw: &ipsw,
            bundles: &[],
            additional_bytes: bundles,
        },
    )?;
    // The translation store takes what the data partition has left after
    // everything above.
    let medium_bytes = install.medium.as_ref().map_or(0, |m| m.need);
    if record.translations.is_none() {
        let (store, note) = translations::choose(
            host,
            &[record.volume.as_path()],
            |_| medium_bytes + bundles,
            command.translation_store_bytes,
        )?;
        notes.extend(note);
        record.translations = store;
    }
    record.options = record.options_text()?;
    record.choices_text()?;
    // Said up front: whether the install needs the network, and what is
    // not known about the installed system's first start.
    let remote = install.medium.as_ref().is_some_and(|medium| medium.remote);
    let restore_bundles: Vec<PathBuf> = prepared
        .iter()
        .filter(|bundle| bundle.kind == Kind::Restore)
        .map(Prepared::path)
        .chain(
            record
                .bundles
                .iter()
                .filter(|bundle| bundle.kind == Kind::Restore)
                .map(|bundle| bundle.directory(&record.volume)),
        )
        .collect();
    notes.extend(network_notes(remote, offline(remote, &restore_bundles)));
    notes.push(SIGNING_NOTE.into());
    notes.extend(installed.notes.iter().cloned());
    // The USB holds every supported device's firmware, whatever this
    // computer has; its own devices only earn a note.
    notes.extend(firmware::host_notes(environment.host)?);
    Ok(Plan {
        record,
        ipsw,
        product_version,
        prepared,
        efi_image,
        installed,
        install,
        notes,
        wifi_passphrase: command.wifi_passphrase.clone(),
        stale_bundles,
    })
}

pub(super) fn describe(plan: &Plan) -> Result<String> {
    let record = &plan.record;
    let mut text = String::new();
    writeln!(
        text,
        "Stela setup for macOS {} ({}) on {}:",
        plan.product_version, record.build, record.model
    )?;
    writeln!(text, "  restore image  {}", plan.ipsw.display())?;
    writeln!(text, "  EFI partition  {}", record.esp.display())?;
    if stela_loader::host_firmware::reads_bundle(&plan.efi_image) {
        writeln!(text, "  firmware       {}", firmware::bundle_summary())?;
    }
    let install = &record.installs[0];
    writeln!(
        text,
        "  ECID           {}",
        install.ecid.expect("a new setup's install has an ECID")
    )?;
    for prepared in &plan.prepared {
        writeln!(
            text,
            "  {:<14} {} ({})",
            prepared.kind.label(),
            bundle::directory_of(
                &record.volume,
                bundle::Identity {
                    kind: prepared.kind,
                    build: &prepared.build,
                    ecid: prepared.ecid,
                    generation: record.generation()
                }
            )
            .display(),
            host::bytes_text(prepared.bytes)
        )?;
    }
    if let Some(store) = &record.translations {
        writeln!(text, "  translations   {}", store.describe())?;
    }
    writeln!(
        text,
        "  boot loader    {}: stela.efi",
        record.esp.join(LOADER_PATH).display()
    )?;
    let checked = &plan.installed;
    let disk = &checked.disk;
    writeln!(
        text,
        "  stela.cfg      {}",
        record.esp.join("EFI/stela/stela.cfg").display()
    )?;
    writeln!(
        text,
        "  choices.cfg    {}: \"{}\", \"{INSTALL_CHOICE}\" ({CHOOSER_SECONDS} s menu)",
        record.esp.join(CHOICES_PATH).display(),
        install.name
    )?;
    describe_install(record, &plan.install, &mut text)?;
    if let Some(network) = &record.wifi {
        writeln!(text, "  {}", wifi::describe(network, &record.esp))?;
    }
    for line in record.addresses.describe() {
        writeln!(text, "  {line}")?;
    }
    match disk.is_chosen() {
        true => writeln!(
            text,
            "  Mac's SSD      a PC disk chosen at {INSTALL_CHOICE}"
        )?,
        false => writeln!(
            text,
            "  Mac's SSD      {} (serial {}, partition {})",
            disk.apple_region, disk.apple_region_serial, disk.apple_region_guid
        )?,
    }
    writeln!(text, "  {}", state_line(disk))?;
    match checked.nor_image {
        Some(_) => writeln!(
            text,
            "  writes         this stick, machine state on {}",
            disk.state_description()
        )?,
        None => writeln!(text, "  writes         this stick only")?,
    }
    for note in &plan.notes {
        writeln!(text, "  note           {note}")?;
    }
    Ok(text)
}

/// The plan's line for an install's machine state, after its two spaces.
pub(super) fn state_line(disk: &installed::InstalledDisk) -> String {
    format!("{:<14} {}", "machine state", disk.state_description())
}

pub(super) fn setup(environment: &Environment<'_>, mut command: Request) -> Result<String> {
    let host = environment.host;
    // Before anything is erased.
    command.read_wifi_passphrase(environment.ui)?;
    if command.fetch {
        builds::find_in(environment.builds, &command.build)?;
    }
    let mut erase = None;
    if let Some(disk) = host.whole_disk(&command.usb)? {
        let found = host.disks()?.into_iter().find(|d| d.identifier == disk);
        let found = found.with_context(|| format!("this computer lists no disk {disk}"))?;
        ensure!(
            found.erasable(),
            "{disk} ({}) is an internal disk; stela erases only a removable USB stick",
            found.name
        );
        if let Some(why) = found.why_typed() {
            ensure!(
                command
                    .confirm_disk
                    .as_deref()
                    .is_some_and(|typed| found.confirmed_by(typed)),
                "{disk} ({}, {}) {why}; pass --confirm-disk with its model or size to erase it",
                found.name,
                host::size_text(found.bytes),
            );
        }
        // The installer it makes, refused before anything else when it
        // cannot be made on this stick.
        let installer =
            choose_installer(command.install_image, found.bytes, command.ipsw.as_deref())?;
        command.install_image = Some(installer);
        if command.dry_run {
            return Ok(format!(
                "Setup would erase {disk} ({}) into an EFI partition and a FAT32 volume named \
                 {USB_VOLUME}, then make the {} installer on it.\nDry run: nothing was changed.\n",
                found.name,
                installer.name()
            ));
        }
        host.check_erase_access(&disk)?;
        let identity = wizard::Usb::from_disk(&found, host);
        // Without a terminal to ask at, the model or size typed back is the
        // confirmation, so a mistyped disk name never erases another USB.
        let typed_back = command
            .confirm_disk
            .as_deref()
            .is_some_and(|typed| found.confirmed_by(typed));
        if !command.confirmed && !typed_back {
            ensure!(
                environment.ui.interactive(),
                "stela cannot ask before erasing {disk} ({}, {}) without a terminal; pass \
                 --confirm-disk with its model or size to erase it",
                found.name,
                host::size_text(found.bytes)
            );
            confirm_erase(environment, &found)?;
        }
        erase = Some((disk, found.name, found.bytes, identity));
    } else {
        // An interrupted setup the stick holds keeps its own installer: an
        // Offline one goes on from the parts already on the stick. A USB
        // named by one of its mounted volumes is otherwise set up as the
        // same USB erased by setup would be. A finished setup is refused by
        // `plan`.
        let esp = super::locate_esp(host, &command.usb)?;
        if let Some(record) = super::record::interrupted(host, &esp, &command.usb) {
            let kind = record.install_kind();
            ensure!(
                command.install_image.is_none_or(|asked| asked == kind),
                "{} holds an interrupted {} setup, which goes on as it began. Run it again \
                 without --installer, or {}",
                esp.display(),
                kind.name(),
                super::START_OVER
            );
            command.install_image = Some(kind);
        } else if !esp.join(RECORD).exists() {
            let capacity = usb_capacity(host, &command.usb)?;
            command.install_image = Some(choose_installer(
                command.install_image,
                capacity,
                command.ipsw.as_deref(),
            )?);
        }
    }
    // Every new or resumed setup knows its installer from here on.
    let install_image = command.install_image.unwrap_or(InstallImage::Network);
    // An image of the build this computer holds already, downloaded by an
    // earlier command or by another user under sudo, serves as --ipsw.
    if command.ipsw.is_none()
        && command.fetch
        && let Some(path) = super::sources::cached_image(
            environment,
            match install_image {
                InstallImage::Full => super::sources::Need::WholeImage,
                InstallImage::Network => super::sources::Need::BootFiles,
            },
            &command.build,
            command.model(),
            command.download_dir.as_deref(),
        )
    {
        command.ipsw = Some(path);
        command.fetch = false;
    }
    // Downloaded boot archives are kept in one place until setup finishes,
    // so a setup run again after a stop reuses them, and removed once it
    // has. A full image goes directly to the stick after preparation and
    // signing; an explicit cache directory keeps what it holds.
    let temporary = (command.fetch && command.download_dir.is_none() && !command.dry_run)
        .then(|| download_dir().join("setup"));
    if let Some(temporary) = &temporary {
        fs::create_dir_all(temporary)?;
    }
    let mut download = None;
    let ipsw = if let Some(temporary) = &temporary
        && command.install_image == Some(InstallImage::Full)
    {
        let image = restore::select(
            &restore::catalog()?,
            Some(&command.build),
            Some(command.model()),
        )?;
        let boot = temporary.join("boot.ipsw");
        download = Some(medium::download::Download::prepare(
            image,
            command.model(),
            Some(&boot),
        )?);
        boot
    } else {
        if let Some(temporary) = &temporary {
            command.download_dir = Some(temporary.clone());
        }
        command.image()?
    };
    if ipsw.is_file() {
        let (_, build) = restore::payload::identity(&ipsw, command.model())?;
        supported(environment, &ipsw, &build)?;
    }
    let environment = &Environment {
        download: download.as_ref(),
        ..*environment
    };
    if !command.dry_run {
        if command.efi.is_none() {
            command.efi = Some(assets::resolve()?);
        }
        read_efi(environment, command.efi.as_ref().unwrap())?;
        // The firmware bundle goes into the cache before the stick is erased
        // or changed: a failed download leaves it as it was.
        let image = fs::read(command.efi.as_ref().unwrap())?;
        let volume = erase
            .is_none()
            .then(|| medium::locate(host, &super::locate_esp(host, &command.usb).ok()?, None).ok())
            .flatten()
            .map(|(volume, _)| volume);
        firmware::prefetch_bundle(&image, volume.as_deref())?;
    }
    let mut early = None;
    if let Some((disk, _, capacity, _)) = &erase {
        for path in std::iter::once(&ipsw).chain(command.efi.iter()) {
            ensure!(
                host.volume(path)?.disk.as_deref() != Some(disk),
                "{} is on the stick being erased; move installer inputs to another disk first",
                path.display()
            );
        }
        if let Some(path) = &command.efi {
            read_efi(environment, path)?;
        }
        let remote = download.is_none() && image_is_remote(&ipsw, install_image)?;
        // The portable installer knows its identity before the target PC is
        // connected. Prepare and sign once, and measure it before erasing USB.
        if command.target.is_none() {
            let ecid = command.ecid.map_or_else(Ecid::generate, Ok)?;
            command.ecid = Some(ecid);
            let (version, build, prepared) =
                prepare_restore_bundles(environment, &ipsw, command.model(), ecid)?;
            let image_bytes = if remote {
                0
            } else {
                match &download {
                    Some(download) => download.bytes,
                    None => fs::metadata(&ipsw)?.len(),
                }
            };
            let required = prepared.iter().map(|p| p.bytes).sum::<u64>()
                + image_bytes
                + format::ESP_BYTES
                + 4 * MARGIN;
            ensure!(
                *capacity == 0 || *capacity >= required,
                "this installer needs a stick of at least {} (this one holds {}): use a larger stick",
                host::bytes_text(required),
                host::bytes_text(*capacity)
            );
            early = Some((version, build, prepared));
        }
    }
    if let Some((disk, name, _, identity)) = erase {
        identity.verify(host)?;
        eprintln!("Erasing {disk} ({name})");
        host.erase(&disk, USB_VOLUME)?;
        let volume = host
            .disks()?
            .into_iter()
            .find(|d| d.identifier == disk)
            .and_then(|d| d.partitions.into_iter().find_map(|p| p.mount))
            .with_context(|| format!("{disk} has no mounted volume after it was erased"))?;
        command.usb = volume;
    }
    let plan = plan(environment, &command, ipsw, early)?;
    let mut report = describe(&plan)?;
    if command.dry_run {
        report.push_str("Dry run: nothing was changed.\n");
        return Ok(report);
    }
    let disk = plan.installed.disk.clone();
    let volumes = [plan.record.esp.clone(), plan.record.volume.clone()];
    apply(environment, plan)?;
    // Before "Done": a USB taken out right after holds everything.
    for volume in &volumes {
        host.flush(volume)?;
    }
    if let Some(temporary) = &temporary {
        // The boot files are on the stick now.
        let _ = fs::remove_dir_all(temporary);
    }
    let start = match disk {
        disk if disk.is_chosen() => format!(
            "choose \"{INSTALL_CHOICE}\" in Stela's startup choices, pick the PC disk and the \
             space macOS gets, and the install restores macOS onto it; Stela then starts the \
             installed macOS from that disk"
        ),
        disk if disk.external_disks => format!(
            "its internal SSD ({}) is blank, so Stela offers \"{INSTALL_CHOICE}\", which \
             restores macOS onto it; Stela then starts that macOS from the stick, and macOS \
             sees the PC's other disks as external drives",
            disk.apple_region
        ),
        disk => format!(
            "choose \"{INSTALL_CHOICE}\" in Stela's startup choices to install macOS onto \
             {target}, then boot from the stick again, which starts the installed macOS",
            target = disk.apple_region
        ),
    };
    writeln!(
        report,
        "Done. Eject the stick before you unplug it. Boot the PC from this stick (the firmware's \
         boot menu lists it as a UEFI USB device): {start}.\nCheck it with {}",
        super::fix(&command.usb, "status")
    )?;
    Ok(report)
}

/// A new setup's installer on a USB of `capacity` bytes: the one `asked`
/// for, or else Offline where it can be made and Online where it cannot
/// (`super::offline_unavailable`): on a smaller USB, or from a local IPSW
/// with only boot files, which prepare the boot bundles alone.
/// Offline asked for where it cannot be made is refused, before anything
/// is downloaded or changed.
pub(super) fn choose_installer(
    asked: Option<InstallImage>,
    capacity: u64,
    ipsw: Option<&Path>,
) -> Result<InstallImage> {
    let whole = match ipsw {
        Some(ipsw) => restore::payload::holds_system_image(ipsw)?,
        None => true,
    };
    match (asked, super::offline_unavailable(capacity, whole)) {
        (Some(InstallImage::Full), Some(reason)) => bail!(
            "Offline {reason}. Use --installer online, or {}",
            match whole {
                true => "a larger stick",
                false => "an IPSW file with all of macOS",
            }
        ),
        (Some(kind), _) => Ok(kind),
        (None, Some(_)) => Ok(InstallImage::Network),
        (None, None) => Ok(InstallImage::Full),
    }
}

/// The bytes of the whole disk holding the mounted volume `usb`, as the
/// host lists it; 0 when the host names no such disk.
fn usb_capacity(host: &dyn Host, usb: &Path) -> Result<u64> {
    let Some(disk) = host.volume(usb)?.disk else {
        return Ok(0);
    };
    Ok(host
        .disks()?
        .into_iter()
        .find(|listed| listed.identifier == disk)
        .map_or(0, |listed| listed.bytes))
}

/// Ask before erasing `disk` at a terminal: a mistyped disk name would
/// erase another removable disk.
fn confirm_erase(environment: &Environment<'_>, disk: &host::Disk) -> Result<()> {
    let serial = environment
        .host
        .serial(&disk.identifier)
        .ok()
        .flatten()
        .map(|serial| format!(", serial {serial}"))
        .unwrap_or_default();
    let question = format!(
        "Erase {} ({}, {}{serial})? Everything on it is deleted.",
        disk.identifier,
        disk.name,
        host::size_text(disk.bytes)
    );
    // Escape is a no.
    let accepted = match environment.ui.confirm(&question) {
        Err(error) if super::ui::cancelled(&error) => false,
        answer => answer?,
    };
    match accepted {
        true => Ok(()),
        false => Err(super::Declined.into()),
    }
}

/// The FAT32 volume setup erases a USB into.
pub(super) const USB_VOLUME: &str = "STELA";

/// Directories from `root` down to `path` that do not exist yet, deepest
/// first.
pub(super) fn missing_directories(root: &Path, path: &Path) -> Vec<PathBuf> {
    let mut missing: Vec<PathBuf> = path
        .ancestors()
        .take_while(|p| p.starts_with(root) && *p != root)
        .filter(|p| !p.exists())
        .map(Path::to_path_buf)
        .collect();
    missing.sort_by_key(|p| std::cmp::Reverse(p.components().count()));
    missing
}

pub(super) fn apply(environment: &Environment<'_>, plan: Plan) -> Result<()> {
    let mut record = plan.record;
    let esp = record.esp.clone();
    // An older stela's bundles go while the record on the stick still names
    // them, so a stop here leaves nothing it does not name.
    super::record::remove_bundles(&record, &plan.stale_bundles)?;
    // Recorded before the first write, so an interrupted setup can be undone.
    write_record(&esp, &record)?;
    copy_bundles(&mut record, &plan.prepared)?;
    write_record(&esp, &record)?;
    if let Some(store) = &record.translations {
        store.publish()?;
    }
    apply_install(environment, &mut record, &plan.ipsw, plan.install)?;
    let checked = &plan.installed;
    if let Some(region) = &checked.foreign {
        installed::erase_foreign(environment.host, region)?;
    }
    if let Some(nor) = &checked.nor_image {
        installed::initialize_nor(environment.host, &checked.disk, nor)?;
    }
    if let Some(passphrase) = &plan.wifi_passphrase {
        wifi::write_key(&esp, passphrase)?;
    }
    efi::add_mount(&esp, &plan.efi_image, None, Some(&record.options))?;
    publish(&mut record)?;
    record.complete = true;
    write_record(&esp, &record)?;
    for note in firmware::provision_bundle(environment.host, &record.volume, &esp, &plan.efi_image)
        .with_context(|| {
            format!(
                "the stick is set up, but without its complete firmware bundle: add it with {}",
                super::fix(&esp, "repair")
            )
        })?
    {
        eprintln!("Note: {note}");
    }
    Ok(())
}
