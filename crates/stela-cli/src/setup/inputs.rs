//! The restore image members a setup keeps on its USB, so a later Stela
//! update prepares the boot bundles again without the image: exactly the
//! members `boot prepare` reads for a restore bundle
//! (`boot::restore_inputs_in`, the list prepare's reads are held to), about
//! 0.45 GB of macOS 27.2's 27 GB, in `stela/boot/<build>/inputs.ipsw`.
//!
//! A setup whose install medium streams the image keeps them; a full
//! installer's medium holds the whole image already (`sources`). They are
//! written whenever bundles are prepared and the stick does not hold them yet,
//! and go with their build's directory when an update removes it.
use super::{Environment, MARGIN, bundle, record::Record};
use crate::restore;
use anyhow::{Result, ensure};
use std::{
    fs,
    path::{Path, PathBuf},
};

const FILE: &str = "inputs.ipsw";

/// Where `build`'s inputs are on `volume`.
pub(super) fn path(volume: &Path, build: &str) -> PathBuf {
    bundle::build_directory(volume, build).join(FILE)
}

/// The members of the IPSW `ipsw` that preparing `model`'s bundles reads
/// (`Environment::bundle_inputs`).
pub(super) fn names(
    environment: &Environment<'_>,
    ipsw: &Path,
    model: &str,
) -> Result<Vec<String>> {
    let mut file = fs::File::open(ipsw)?;
    let mut archive = zip::ZipArchive::new(&mut file as &mut dyn restore::ImageSource)?;
    (environment.bundle_inputs)(&mut archive, model)
}

/// Whether `record` keeps inputs: its installs stream the image.
fn wanted(record: &Record) -> bool {
    record.install_kind() == super::InstallImage::Network
}

/// `build`'s inputs on `volume` when they are complete for this stela: the
/// build's archive, holding every member prepare reads for `model`.
pub(super) fn kept(
    environment: &Environment<'_>,
    volume: &Path,
    build: &str,
    model: &str,
) -> Option<PathBuf> {
    let path = path(volume, build);
    // Its manifest names its build; it holds too few members for an
    // inventory.
    if !path.is_file()
        || !restore::archive::manifest(&path, None, false)
            .is_ok_and(|manifest| manifest.product_build == build)
    {
        return None;
    }
    let names = names(environment, &path, model).ok()?;
    let archive = zip::ZipArchive::new(fs::File::open(&path).ok()?).ok()?;
    names
        .iter()
        .all(|name| archive.index_for_name(name).is_some())
        .then_some(path)
}

/// The bytes keeping `ipsw`'s inputs for `build` takes on `record`'s USB:
/// none when the setup keeps none or holds them already.
pub(super) fn needed(
    environment: &Environment<'_>,
    record: &Record,
    build: &str,
    ipsw: &Path,
) -> Result<u64> {
    if !wanted(record) || kept(environment, &record.volume, build, &record.model).is_some() {
        return Ok(0);
    }
    let names = names(environment, ipsw, &record.model)?;
    restore::payload::members_bytes(ipsw, &names)
}

/// Refuse a USB without room for `bytes` of inputs, before it changes.
pub(super) fn check_room(environment: &Environment<'_>, record: &Record, bytes: u64) -> Result<()> {
    let free = environment.host.volume(&record.volume)?.free_bytes;
    ensure!(
        free >= bytes + MARGIN,
        "{} has {} free; the boot files a later update prepares its bundles from need {}",
        record.volume.display(),
        super::host::bytes_text(free),
        super::host::bytes_text(bytes + MARGIN)
    );
    Ok(())
}

/// Keep `ipsw`'s inputs for `build` on `record`'s USB, when [`needed`].
pub(super) fn keep(
    environment: &Environment<'_>,
    record: &Record,
    build: &str,
    ipsw: &Path,
) -> Result<()> {
    if needed(environment, record, build, ipsw)? == 0 {
        return Ok(());
    }
    let names = names(environment, ipsw, &record.model)?;
    let path = path(&record.volume, build);
    fs::create_dir_all(path.parent().expect("a build directory"))?;
    restore::payload::copy_named(ipsw, &names, &path, "Keeping boot files on the stick")?;
    Ok(())
}
