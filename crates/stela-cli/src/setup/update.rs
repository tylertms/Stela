//! Updates of an existing setup: a Stela update replaces `stela.efi`, a
//! macOS update replaces the install medium and restore bundles. Each one
//! writes the new files first and removes the old ones last, so one that was
//! interrupted is finished by running it again. The record names the new
//! version or build only once it is active, and `repair` removes what a stop
//! left of the other one (`Record::has_leftovers`). There is no rollback.
//!
//! - Stela: the USB's boot loader is replaced atomically by the new one. A
//!   new `stela.efi` must carry this `stela`'s mount interface. When it reads
//!   another boot-bundle schema than the setup's bundles have, or this
//!   `stela` prepares another bundle revision
//!   (`stela_loader::boot_bundle::REVISION`) than theirs, the update prepares
//!   the active build's restore bundles for every machine again (while the
//!   setup has the build's install medium) at the new schema and revision,
//!   from `--ipsw` or a download; this makes it an update even with the same
//!   `stela.efi`. The cache store's spare is prepared before activating it:
//!   its new translator takes that spare. Repeating an already-installed
//!   update replenishes a consumed spare without replacing EFI or boot
//!   configuration.
//! - macOS: an installed macOS updates itself: Software Update stages the
//!   new system on the container's Update volume and sets `boot-command
//!   upgrade`, which `stela.efi` boots from that volume as iBEC would
//!   (`stela_loader::upgrade`), and every later boot builds from the
//!   Preboot volume's new objects. `update os` brings the setup's Install
//!   macOS to a new build: the build's install medium and each
//!   machine's restore bundles, so the USB needs room for both while it runs.
//!
//! Neither touches the installed disks' partitions.
use super::{
    Environment, LOADER_PATH, Prepared, Record, apply_install, bundle, check_bundle_revision,
    check_space, copy_bundles, describe_install, plan_install, prepare_kind, publish, read_efi,
    restore_bundles, write_record,
};
use crate::efi;
use anyhow::{Context, Result, bail, ensure};
use sha2::{Digest, Sha256};
use std::{fmt::Write as _, fs, path::Path};

pub(super) fn version_id(image: &[u8]) -> String {
    let digest = hex::encode(Sha256::digest(image));
    digest[..16].into()
}

pub(super) fn check_firmware_support(esp: &Path, image: &[u8]) -> Result<()> {
    stela_loader::host_firmware::check_installed_mount(
        &esp.join(stela_loader::host_firmware::DIRECTORY),
        image,
    )
    .context("stela.efi cannot load installed host firmware")?;
    Ok(())
}

/// Make `build` the active build, which Install macOS restores; its
/// restore bundles and medium are the startup choices' from now on.
fn activate_build(record: &mut Record, build: &str) -> Result<()> {
    record.build = build.into();
    record.replacing = None;
    record.bundle_directory = bundle::build_directory(&record.volume, build);
    publish(record)
}

/// Remove the bundles and media of everything but the active build and
/// generation, and the build a stopped macOS switch moves to: its part of
/// a new copy stays for the switch to go on from.
pub(super) fn prune(record: &mut Record) -> Result<()> {
    let generation = record.generation();
    let kept: Vec<String> = std::iter::once(record.build.clone())
        .chain(record.replacing.clone())
        .collect();
    let (active, stale): (Vec<_>, Vec<_>) = record
        .bundles
        .drain(..)
        .partition(|b| kept.contains(&b.build) && b.generation() == generation);
    record.bundles = active;
    super::remove_bundles(record, &stale)?;
    for build in stale.iter().map(|b| &b.build) {
        let directory = bundle::build_directory(&record.volume, build);
        if !kept.contains(build) && directory.is_dir() {
            fs::remove_dir_all(directory)?;
        }
    }
    let (active, stale): (Vec<_>, Vec<_>) = record
        .media
        .drain(..)
        .partition(|m| kept.contains(&m.build));
    record.media = active;
    for medium in stale {
        remove_medium(&medium)?;
    }
    Ok(())
}

/// Delete a medium the record no longer names.
fn remove_medium(medium: &super::medium::Medium) -> Result<()> {
    let directory = medium.removable_directory()?;
    if directory.is_dir() {
        fs::remove_dir_all(&directory)
            .with_context(|| format!("cannot remove {}", directory.display()))?;
    }
    Ok(())
}

/// Refuse a change to an interrupted setup, naming the command that
/// finishes it.
pub(super) fn ready(record: &Record, usb: &Path) -> Result<()> {
    ensure!(record.complete, "{}", super::interrupted_text(record, usb));
    Ok(())
}

/// Image sources and write policy for a Stela update.
pub(super) struct UpdateOptions<'a> {
    /// The stick, as STICK named it.
    pub(super) usb: &'a Path,
    pub(super) image_path: &'a Path,
    pub(super) ipsw: Option<&'a Path>,
    pub(super) download_dir: Option<&'a Path>,
    pub(super) dry_run: bool,
    /// `--yes`: put an older Stela on a stick without asking.
    pub(super) yes: bool,
    /// It reports the device skips the stick's log names, which `update`
    /// ends once the whole command is done; a repair reports its own.
    pub(super) skips: bool,
}

/// Replace `stela.efi` with `image_path`, preparing bundles of its schema
/// and this `stela`'s revision from `ipsw` when the setup's are of another.
pub(super) fn update_stela(
    environment: &Environment<'_>,
    mut record: Record,
    options: UpdateOptions<'_>,
) -> Result<String> {
    let UpdateOptions {
        usb,
        image_path,
        ipsw,
        download_dir,
        dry_run,
        yes,
        skips,
    } = options;
    let image = read_efi(environment, image_path)?;
    check_firmware_support(&record.esp, &image)?;
    let to = version_id(&image);
    let to_generation = environment.generation();
    ready(&record, usb)?;
    // A dry run asks nothing; it only says what would change.
    if !dry_run {
        super::release::allow_older(environment, &record, yes, "put the older Stela on it")?;
    }
    let from_generation = record.generation();
    // Restore bundles of the new generation for every machine of the active
    // build while it has an install medium. Where their restore image comes
    // from is decided before anything is downloaded or changed.
    let mut next = record.clone();
    next.bundle_schema = to_generation.schema;
    next.bundle_revision = to_generation.revision;
    let missing_restore = match from_generation == to_generation {
        true => Vec::new(),
        false => next.missing_restore_bundles(),
    };
    let lookup = super::sources::Lookup {
        need: super::sources::Need::BootFiles,
        build: &record.build,
        model: &record.model,
        ipsw,
        download_dir,
        medium: record.active_medium(),
    };
    let source = match missing_restore.is_empty() {
        true => None,
        false => Some(
            super::sources::locate(environment, &lookup).map_err(|error| {
                anyhow::anyhow!(
                    "this stela and stela.efi {to} boot bundles of {to_generation}, and this \
                     setup's are {from_generation}, so the update prepares them again from the \
                     macOS {} restore image; but {error:#}",
                    record.build
                )
            })?,
        ),
    };
    // Every bundle file goes into the cache before the stick changes, so a
    // failed download leaves the old stela.efi and the old firmware.
    let bundle_incomplete = stela_loader::host_firmware::reads_bundle(&image)
        && super::firmware::bundle_status(&record.volume).is_err();
    if !dry_run && bundle_incomplete {
        super::firmware::prefetch_bundle(&image, Some(&record.volume))?;
    }
    let esp = record.esp.clone();
    // A missing or damaged loader is what an update also puts right.
    let from = fs::read(esp.join(LOADER_PATH)).map_or("none".into(), |image| version_id(&image));
    let mut text = String::new();
    writeln!(text, "Stela update on {}:", esp.display())?;
    if skips {
        text.push_str(&super::doctor::device_skips(&esp, dry_run)?);
    }
    text.push_str(&super::fdr::prepare(&record, true)?);
    // The firmware the last boot asked for and the USB lacks, which every
    // update that changes something adds.
    let requested = super::firmware::requested(&esp, &image)?;
    for line in requested.lines() {
        writeln!(text, "  firmware       {line}")?;
    }
    if from == to && from_generation == to_generation {
        if !dry_run {
            super::fdr::prepare(&record, false)?;
        }
        // An update stopped after its loader went in: the record still names
        // the previous version, or the bundles and media it replaced.
        if record.stela_version.as_deref() != Some(to.as_str())
            || record.release.as_deref() != Some(super::release::CURRENT)
            || record.has_leftovers()
        {
            writeln!(
                text,
                "  stela.efi      version {to} is installed; its update is finished"
            )?;
            writeln!(
                text,
                "  boot loader    stela.cfg and the startup choices as recorded"
            )?;
            if dry_run {
                text.push_str("Dry run: nothing was changed.\n");
                return Ok(text);
            }
            if let Some(store) = &record.translations {
                store.publish()?;
            }
            activate(&mut record, to)?;
            requested.provision(&esp, &image)?;
            // An update whose bundle download stopped it is finished here.
            for note in
                super::firmware::provision_bundle(environment.host, &record.volume, &esp, &image)?
            {
                writeln!(text, "  firmware       {note}")?;
            }
            text.push_str("Done.\n");
            return Ok(text);
        }
        writeln!(text, "  stela.efi      version {to} is already installed")?;
        if bundle_incomplete {
            writeln!(text, "  firmware       complete the firmware bundle")?;
            if !dry_run {
                for note in super::firmware::provision_bundle(
                    environment.host,
                    &record.volume,
                    &esp,
                    &image,
                )? {
                    writeln!(text, "  firmware       {note}")?;
                }
            }
        }
        let spare = record
            .translations
            .as_ref()
            .filter(|store| !store.config_intact() || !store.spare_ready());
        if spare.is_some() {
            writeln!(
                text,
                "  translations   prepare the cache configuration and spare"
            )?;
        }
        if spare.is_some() || !requested.is_empty() {
            if dry_run {
                text.push_str("Dry run: nothing was changed.\n");
            } else {
                if let Some(store) = spare {
                    store.publish()?;
                }
                requested.provision(&esp, &image)?;
            }
        }
        return Ok(text);
    }
    if from == to {
        writeln!(text, "  stela.efi      version {to} stays")?;
    } else {
        writeln!(text, "  stela.efi      version {from} -> {to}")?;
    }
    if let Some(source) = &source {
        writeln!(
            text,
            "  restore image  {}",
            super::sources::describe(source, &lookup)
        )?;
    }
    // A dry run prepares only from an image already on this computer: it
    // reads nothing from the USB and downloads nothing.
    let local = source
        .as_ref()
        .is_some_and(super::sources::Source::is_local);
    if dry_run && !local {
        for &(kind, ecid) in &missing_restore {
            writeln!(
                text,
                "  {:<14} {} ({to_generation}), replaces {from_generation}",
                kind.label(),
                bundle::directory_of(
                    &record.volume,
                    bundle::Identity {
                        kind,
                        build: &record.build,
                        ecid,
                        generation: to_generation
                    }
                )
                .display(),
            )?;
        }
    }
    // The image stays until the bundles and the boot files a later update
    // prepares from are on the USB.
    let mut obtained = None;
    let prepared = if missing_restore.is_empty() || (dry_run && !local) {
        Vec::new()
    } else {
        let source = source.as_ref().expect("missing bundles have a source");
        let ipsw = &obtained
            .insert(super::sources::obtain(environment, source, &lookup)?)
            .path;
        let mut prepared: Vec<Prepared> = Vec::new();
        for kind in bundle::RESTORE_KINDS {
            let ecids: Vec<_> = missing_restore
                .iter()
                .filter(|(k, _)| *k == kind)
                .map(|&(_, ecid)| ecid)
                .collect();
            if !ecids.is_empty() {
                let (_, _, restore) = prepare_kind(
                    environment,
                    super::prepare::BundleOptions {
                        kind,
                        ipsw,
                        model: &record.model,
                        build: Some(&record.build),
                        ecids: &ecids,
                    },
                )?;
                prepared.extend(restore);
            }
        }
        check_space(environment.host, &record.volume, &prepared)?;
        // Room for the boot files kept beside the bundles.
        let inputs = super::inputs::needed(environment, &record, &record.build, ipsw)?;
        if inputs > 0 {
            super::inputs::check_room(
                environment,
                &record,
                inputs + prepared.iter().map(|p| p.bytes).sum::<u64>(),
            )?;
        }
        prepared
    };
    for bundle in &prepared {
        writeln!(
            text,
            "  {:<14} {} ({to_generation}, {}), replaces {from_generation}",
            bundle.kind.label(),
            bundle::directory_of(
                &record.volume,
                bundle::Identity {
                    kind: bundle.kind,
                    build: &bundle.build,
                    ecid: bundle.ecid,
                    generation: to_generation
                }
            )
            .display(),
            super::host::bytes_text(bundle.bytes)
        )?;
    }
    match &record.translations {
        Some(store) => {
            if from == to {
                writeln!(text, "  translations   {}", store.describe())?;
            } else {
                writeln!(
                    text,
                    "  translations   {}; new pack for the new stela.efi",
                    store.describe()
                )?;
            }
            for pack in store.stale() {
                writeln!(text, "  remove         {}", pack.path.display())?;
            }
        }
        None => writeln!(text, "  translations   no store on this stick")?,
    }
    if dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    record.bundle_schema = to_generation.schema;
    record.bundle_revision = to_generation.revision;
    copy_bundles(&mut record, &prepared)?;
    if let Some(obtained) = &obtained {
        super::inputs::keep(environment, &record, &record.build, &obtained.path)?;
    }
    write_record(&esp, &record)?;
    // A new translator must have its bounded spare before its EFI becomes
    // active. Failure leaves the old loader active.
    if let Some(store) = &record.translations {
        store.publish()?;
    }
    super::fdr::prepare(&record, false)?;
    efi::add_mount(&esp, &image, None, None)?;
    activate(&mut record, to)?;
    // An older USB, with a per-PC firmware set or none, gets the complete
    // bundle.
    for note in super::firmware::provision_bundle(environment.host, &record.volume, &esp, &image)? {
        writeln!(text, "  firmware       {note}")?;
    }
    if !stela_loader::host_firmware::reads_bundle(&image) {
        requested.provision(&esp, &image)?;
    }
    text.push_str("Done.\n");
    Ok(text)
}

/// The steps after the new loader is in place: `stela.cfg` and the startup
/// choices for the record, the record naming version `to`, and the bundles,
/// media and packs it replaced removed. The record names the old version
/// until it is written here, so `update stela` run again after a stop
/// anywhere before finishes these steps.
fn activate(record: &mut Record, to: String) -> Result<()> {
    publish(record)?;
    record.stela_version = Some(to);
    record.release = Some(super::release::CURRENT.into());
    write_record(&record.esp.clone(), record)?;
    prune(record)?;
    if let Some(store) = &record.translations {
        store.prune()?;
    }
    write_record(&record.esp.clone(), record)
}

/// Whether the USB has room for `build`'s whole image from `source` beside
/// what it keeps. When it has room only once the active build's copy is
/// gone, the user is asked on a terminal (a dry run says it would ask)
/// and the copy to remove first is returned; without a terminal, or with
/// a no, the update is refused before anything is downloaded.
fn room_for_new_copy(
    environment: &Environment<'_>,
    record: &Record,
    build: &str,
    source: &super::sources::Source,
    (dry_run, yes): (bool, bool),
    text: &mut String,
) -> Result<Option<super::medium::Medium>> {
    let Err(error) = super::sources::check_room(environment, record, build, source) else {
        return Ok(None);
    };
    let from = &record.build;
    let needed = super::medium::needed_bytes(
        &record.volume,
        build,
        source.image_bytes()?,
        environment.piece_bytes,
    );
    let free = environment.host.volume(&record.volume)?.free_bytes;
    let Some((old, old_bytes)) = record
        .active_medium()
        .filter(|medium| !medium.remote)
        .map(|medium| (medium.clone(), medium.bytes_on_volume()))
        .filter(|&(_, bytes)| free.saturating_add(bytes) >= needed)
    else {
        bail!("{error:#} Removing macOS {from}'s copy would not make enough room either.");
    };
    let directory = old.directory();
    let size = super::host::bytes_text(old_bytes);
    if dry_run {
        writeln!(
            text,
            "  remove first   {} ({size}), after asking: the stick has room for one copy of \
             macOS, and cannot install macOS until the new one is complete",
            directory.display()
        )?;
        return Ok(Some(old));
    }
    let question = format!(
        "This stick has room for one copy of macOS only. Switching to macOS {build} first removes \
         macOS {from}'s copy ({}, {size}). Until the new copy is complete, this stick cannot \
         install macOS; a macOS already installed keeps starting, and `stela repair` continues \
         the switch. Remove macOS {from}'s copy and continue?",
        directory.display()
    );
    super::consent(
        environment,
        yes,
        &question,
        &format!(
            "{error:#} The stick has room for one copy of macOS: switching to macOS {build} first \
             removes macOS {from}'s copy ({size})"
        ),
    )?;
    Ok(Some(old))
}

/// Image source and write policy for a macOS update.
pub(super) struct OsOptions<'a> {
    pub(super) usb: &'a Path,
    /// The new build's image; its build is the new one.
    pub(super) ipsw: Option<&'a Path>,
    /// The new build without `ipsw` (`--macos`).
    pub(super) build: &'a str,
    pub(super) download_dir: Option<&'a Path>,
    pub(super) dry_run: bool,
    /// `--yes`: remove the old copy for room without asking.
    pub(super) yes: bool,
    /// The command named the download, as `update --macos` does: without a
    /// terminal it is not asked about. A repair that finishes a stopped
    /// switch names none.
    pub(super) named: bool,
    /// The bytes of a download plain `stela`'s choice named and the person
    /// chose, which is not asked about again.
    pub(super) accepted: Option<u64>,
}

/// Add the new build's install medium and every machine's restore bundles,
/// and make it the build Install macOS restores.
pub(super) fn update_os(
    environment: &Environment<'_>,
    mut record: Record,
    options: OsOptions<'_>,
) -> Result<String> {
    let OsOptions {
        usb,
        ipsw,
        build,
        download_dir,
        dry_run,
        yes,
        named,
        accepted,
    } = options;
    // Everything that can refuse the update comes before any download.
    check_bundle_revision(environment, &record)?;
    let build = match ipsw {
        Some(ipsw) => {
            let (_, build) = crate::restore::payload::identity(ipsw, &record.model)?;
            super::supported(environment, ipsw, &build)?;
            build
        }
        None => {
            let entry = stela_formats::builds::find_in(environment.builds, build)
                .map_err(|error| anyhow::anyhow!("macOS {build}: {error}"))?;
            ensure!(
                entry.build == build,
                "macOS {build} is not a build Stela was tested with; pass its image with --ipsw \
                 to use it"
            );
            build.to_owned()
        }
    };
    ready(&record, usb)?;
    let from = record.build.clone();
    if build == from && record.has_leftovers() {
        // Stopped after the new build became the active one: remove what it
        // replaces.
        let mut text = format!("macOS {build} install files:\n");
        writeln!(
            text,
            "  remove         the restore files of earlier builds and generations"
        )?;
        if dry_run {
            text.push_str("Dry run: nothing was changed.\n");
            return Ok(text);
        }
        prune(&mut record)?;
        write_record(&record.esp.clone(), &record)?;
        text.push_str("Done.\n");
        return Ok(text);
    }
    if build == from {
        return Ok(format!(
            "macOS {build} is already the build this setup installs: nothing to update. {} \
             checks its files.\n",
            super::fix(usb, "status")
        ));
    }
    // The build's install medium and restore bundles, so Install macOS
    // restores the new build: what that needs of its image, where it comes
    // from and whether the USB has room, before anything is downloaded.
    let need =
        super::sources::need(&record, &build, true).unwrap_or(super::sources::Need::BootFiles);
    let lookup = super::sources::Lookup {
        need,
        build: &build,
        model: &record.model,
        ipsw,
        download_dir,
        medium: record.medium_of(&build),
    };
    let source = super::sources::locate(environment, &lookup).map_err(|error| {
        anyhow::anyhow!("updating to macOS {build} needs its restore image; but {error:#}")
    })?;
    let mut text = String::new();
    writeln!(text, "macOS {build} install files:")?;
    writeln!(
        text,
        "  restore image  {}",
        super::sources::describe(&source, &lookup)
    )?;
    // A USB with room for one whole image only replaces the old copy, once
    // the user agrees: it cannot install macOS until the new copy is
    // complete.
    let replace = match need {
        super::sources::Need::WholeImage | super::sources::Need::Pieces => room_for_new_copy(
            environment,
            &record,
            &build,
            &source,
            (dry_run, yes),
            &mut text,
        )?,
        super::sources::Need::BootFiles => None,
    };
    // `--macos` names the download, so only a terminal run asks first.
    if !dry_run {
        let bytes = super::sources::download_bytes(&source);
        super::consent_download(
            environment,
            yes || accepted == Some(bytes),
            named,
            bytes,
            &format!("macOS {build}"),
        )?;
    }
    // A dry run reads and downloads nothing to prepare its plan, and plans
    // no copy that needs the old one's room.
    let obtained = match dry_run && (!source.is_local() || replace.is_some()) {
        true => None,
        false => Some(super::sources::obtain(environment, &source, &lookup)?),
    };
    if let (Some(old), false) = (&replace, dry_run) {
        // The record stops naming the old copy before its files go, and
        // names the build that replaces it until that one is active, so
        // `status` says how an interrupted update is finished.
        record.replacing = Some(build.clone());
        record.media.retain(|medium| medium.build != old.build);
        write_record(&record.esp.clone(), &record)?;
        publish(&mut record)?;
        remove_medium(old)?;
    }
    let install = match &obtained {
        Some(obtained) => {
            let environment = Environment {
                download: obtained.download.as_ref(),
                ..*environment
            };
            let plan = plan_install(
                &environment,
                &record,
                super::create::InstallOptions {
                    build: &build,
                    ipsw: &obtained.path,
                    bundles: &restore_bundles(&record.install_machines()),
                    additional_bytes: 0,
                },
            )?;
            describe_install(&record, &plan, &mut text)?;
            Some(plan)
        }
        None => {
            writeln!(
                text,
                "  install        its install medium and every machine's restore bundle"
            )?;
            None
        }
    };
    if let Some(medium) = record.active_medium() {
        writeln!(
            text,
            "  remove         {} and the {from} restore files",
            medium.directory().display()
        )?;
    }
    if dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    let (Some(install), Some(obtained)) = (install, obtained) else {
        unreachable!("a change has its plan");
    };
    let environment = &Environment {
        download: obtained.download.as_ref(),
        ..*environment
    };
    let ipsw = obtained.path.as_path();
    apply_install(environment, &mut record, ipsw, install)?;
    activate_build(&mut record, &build)?;
    // The record names the new build before the old one's files go, so a
    // stop while they are removed leaves them as leftovers, which running
    // this again or `repair` removes.
    write_record(&record.esp.clone(), &record)?;
    prune(&mut record)?;
    write_record(&record.esp.clone(), &record)?;
    text.push_str(
        "Done. An installed macOS keeps updating itself. To install this build on a new or \
         erased disk, boot the stick and choose Install macOS, which erases that disk.\n",
    );
    Ok(text)
}
