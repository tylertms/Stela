//! Install the UEFI mount on a USB's EFI system partition as its
//! removable-media boot loader, `\EFI\BOOT\BOOTX64.EFI`, which a PC's
//! firmware starts when it boots from the stick; no other boot manager is
//! involved.
//!
//! `install` copies `stela.efi` there and writes the mount's
//! `EFI/stela/stela.cfg` and `EFI/stela/source.bin`. A partition that
//! already boots another loader is refused and left unchanged: the loader
//! there is replaced only when it is a `stela.efi` (it carries a mount
//! interface marker, [`mount_interface::find`]). Files are replaced
//! atomically. `uninstall` removes what `install` wrote.
//!
//! With `--ipsw`, `install` also writes a restore payload for `stela.efi`
//! under `EFI/stela/images/<build>/` on the ESP or another volume of the
//! same USB: FAT32-safe slices of the model's members, or the whole IPSW,
//! and `payload.json` with every member's SHA-256 and the public AEA key
//! responses. `uninstall` leaves payloads alone.
pub use crate::restore::payload::{
    PayloadLayout, SYNTHETIC_BUILD, SYNTHETIC_MODEL, WriteOptions as PayloadOptions,
    synthetic_ipsw, write as write_payload,
};
use anyhow::{Context, Result, bail, ensure};
use clap::{Args, Subcommand};
use serde_json::json;
use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};
use stela_loader::mount_interface;
use stela_runtime::pc::boot_log;

use stela_formats::payload::MAX_SLICE_BYTES;

/// The removable-media boot loader path for x64 (UEFI 2.10, Removable Media
/// Boot Behavior).
pub const LOADER: &str = "EFI/BOOT/BOOTX64.EFI";
const LOADER_DIRECTORY: &str = "EFI/BOOT";
const MOUNT: &str = "EFI/stela";
pub(crate) const OPTIONS: &str = "EFI/stela/stela.cfg";
/// `mount_interface::BOOT_LOG_FILE`.
pub(crate) const BOOT_LOG: &str = "EFI/stela/boot.log";
const SOURCE: &str = "EFI/stela/source.bin";

#[derive(Subcommand)]
pub enum EfiCommand {
    /// Install `stela.efi` as an EFI partition's removable-media boot loader.
    Install(Install),
    /// Remove everything `install` added.
    Uninstall(Uninstall),
}

impl EfiCommand {
    pub fn run(self) -> Result<()> {
        let report = match self {
            Self::Install(command) => install(&command)?,
            Self::Uninstall(command) => uninstall(&command.esp, false)?,
        };
        println!("{report:#}");
        Ok(())
    }
}

#[derive(Args)]
pub struct Install {
    /// The EFI system partition root.
    #[arg(long)]
    pub esp: PathBuf,
    /// The mount application, `stela.efi`.
    #[arg(long)]
    pub image: PathBuf,
    /// Original ARM bytes; omit to keep the installed `source.bin`.
    #[arg(long)]
    pub source: Option<PathBuf>,
    /// Mount options for `stela.cfg`; omit to keep the installed file.
    #[arg(long, allow_hyphen_values = true)]
    pub options: Option<String>,
    /// Write this IPSW's restore payload as well.
    #[arg(long)]
    pub ipsw: Option<PathBuf>,
    /// The Mac model whose identities the payload carries.
    #[arg(long, default_value = crate::setup::PINNED_MODEL, requires = "ipsw")]
    pub model: String,
    /// The volume root for the payload; defaults to `--esp`.
    #[arg(long, requires = "ipsw")]
    pub payload_root: Option<PathBuf>,
    #[arg(long, value_enum, default_value_t = PayloadLayout::Split, requires = "ipsw")]
    pub payload_layout: PayloadLayout,
    /// Largest slice of a split payload; FAT32 allows 4 GiB - 1.
    #[arg(long, default_value_t = MAX_SLICE_BYTES, hide = true)]
    pub slice_bytes: u64,
    /// Leave out the public AEA keys; the restore then needs the network.
    #[arg(long, requires = "ipsw")]
    pub no_aea_keys: bool,
}

#[derive(Args)]
pub struct Uninstall {
    /// The EFI system partition root.
    #[arg(long)]
    esp: PathBuf,
}

/// Replace `path` through a sibling temporary file, so a failed write leaves
/// the old file in place; with `fresh`, only when nothing is there yet.
pub(crate) fn write_atomic(path: &Path, bytes: &[u8], fresh: bool) -> Result<()> {
    let parent = path.parent().context("a file without a parent directory")?;
    (|| -> Result<()> {
        let mut file = tempfile::NamedTempFile::new_in(parent)?;
        file.write_all(bytes)?;
        file.as_file().sync_all()?;
        match fresh {
            true => persist_new(file, path)?,
            false => drop(file.persist(path).map_err(|error| error.error)?),
        };
        Ok(())
    })()
    .with_context(|| format!("cannot write {}", path.display()))?;
    // FAT may refuse to sync a directory; the file itself is already synced.
    let _ = fs::File::open(parent).and_then(|directory| directory.sync_all());
    Ok(())
}

/// Give the synced temporary `file` the name `path`, never replacing a
/// `path` that exists ([`rename_new`]); a refused `file` is removed.
pub(crate) fn persist_new(file: tempfile::NamedTempFile, path: &Path) -> Result<()> {
    let temporary = file.into_temp_path();
    rename_new(&temporary, path)?;
    // It is `path` now: nothing is left to remove.
    let _ = temporary.keep();
    Ok(())
}

/// Give the finished file `from` the name `to`, never replacing a `to` that
/// exists: by a rename that refuses to replace where the file system has
/// one, else, as on FAT and exFAT under macOS (which refuse that rename with
/// ENOTSUP and have no hard links either), by a plain rename once `to` is
/// checked to be absent. A refused `from` stays where it is.
pub(crate) fn rename_new(from: &Path, to: &Path) -> Result<()> {
    rename_new_with(from, to, |from, to| {
        tempfile::TempPath::try_from_path(from)?
            .persist_noclobber(to)
            .map_err(|error| {
                // Kept for the caller, not removed with the handle.
                let _ = error.path.keep();
                error.error
            })
    })
}

fn rename_new_with(
    from: &Path,
    to: &Path,
    no_replace: impl FnOnce(&Path, &Path) -> std::io::Result<()>,
) -> Result<()> {
    match no_replace(from, to) {
        Ok(()) => return Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
            bail!("{} already exists", to.display())
        }
        Err(_) => {}
    }
    match fs::symlink_metadata(to) {
        Ok(_) => bail!("{} already exists", to.display()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => {
            return Err(error).with_context(|| format!("cannot inspect {}", to.display()));
        }
    }
    fs::rename(from, to)
        .with_context(|| format!("cannot rename {} to {}", from.display(), to.display()))
}

/// Whether the partition's boot loader is a `stela.efi`; `None` when it has
/// none.
pub(crate) fn loader_is_stela(esp: &Path) -> Result<Option<bool>> {
    let path = esp.join(LOADER);
    match fs::read(&path) {
        Ok(image) => Ok(Some(mount_interface::find(&image).is_some())),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error).with_context(|| format!("cannot read {}", path.display())),
    }
}

/// Refuse a partition whose boot loader is not Stela's, changing nothing.
pub(crate) fn check_loader(esp: &Path) -> Result<()> {
    ensure!(
        loader_is_stela(esp)? != Some(false),
        "{} already boots another loader ({}). Stela boots the stick itself, as its only boot \
         loader, and does not replace another one: use a stick whose EFI partition has no \
         {LOADER}, or erase the stick first.",
        esp.display(),
        esp.join(LOADER).display()
    );
    Ok(())
}

/// Install or replace the mount, and return a report of what changed.
pub fn install(command: &Install) -> Result<serde_json::Value> {
    let esp = &command.esp;
    let source = match &command.source {
        Some(source) => {
            Some(fs::read(source).with_context(|| format!("cannot read {}", source.display()))?)
        }
        None if esp.join(SOURCE).is_file() => None,
        None => bail!("pass --source: {} has no {SOURCE}", esp.display()),
    };
    // Check the partition before writing the payload, which goes before the
    // loader, so the stick never boots Stela with a half-written payload.
    check_loader(esp)?;
    let payload = match &command.ipsw {
        Some(ipsw) => Some(crate::restore::payload::write(
            &crate::restore::payload::WriteOptions {
                image: ipsw,
                model: &command.model,
                root: command.payload_root.as_deref().unwrap_or(esp),
                layout: command.payload_layout,
                slice_bytes: command.slice_bytes,
                fetch_keys: !command.no_aea_keys,
            },
        )?),
        None => None,
    };
    let image = fs::read(&command.image)
        .with_context(|| format!("cannot read {}", command.image.display()))?;
    let mut report = add_mount(esp, &image, source.as_deref(), command.options.as_deref())?;
    report["payload"] = json!(payload);
    Ok(report)
}

/// Copy the ARM source when given and the options, then the loader, over
/// whatever loader is there: callers check it first. `stela create` uses this
/// without a source: install mode runs none.
pub(crate) fn add_mount(
    esp: &Path,
    image: &[u8],
    source: Option<&[u8]>,
    options: Option<&str>,
) -> Result<serde_json::Value> {
    ensure!(image.starts_with(b"MZ"), "the image is not a PE image");
    ensure!(
        mount_interface::find(image).is_some(),
        "the image is not a stela.efi: it carries no mount interface marker"
    );
    fs::create_dir_all(esp.join(MOUNT))?;
    fs::create_dir_all(esp.join(LOADER_DIRECTORY))?;
    if let Some(source) = source {
        write_atomic(&esp.join(SOURCE), source, false)?;
    }
    if let Some(options) = options {
        write_atomic(&esp.join(OPTIONS), options.as_bytes(), false)?;
    }
    // An existing log keeps its earlier boots. `stela.efi` writes its
    // clusters wherever the volume put them.
    if !esp.join(BOOT_LOG).is_file() {
        write_atomic(
            &esp.join(BOOT_LOG),
            &vec![0; boot_log::LOG_BYTES as usize],
            true,
        )?;
    }
    // The loader goes in last, so the stick never boots it without its files.
    write_atomic(&esp.join(LOADER), image, false)?;
    Ok(json!({
        "loader": esp.join(LOADER),
        "source": esp.join(SOURCE).is_file().then(|| esp.join(SOURCE)),
        "options": esp.join(OPTIONS).is_file().then(|| esp.join(OPTIONS)),
        "boot_log": esp.join(BOOT_LOG),
    }))
}

/// Paths relative to the ESP that `install` writes, for reports.
pub(crate) const INSTALLED: [&str; 4] = [LOADER, OPTIONS, SOURCE, BOOT_LOG];

/// Remove the loader, when it is Stela's, and the files `install` wrote,
/// and return a report.
pub fn uninstall(esp: &Path, owned: bool) -> Result<serde_json::Value> {
    let stela = match owned {
        true => Some(true),
        false => loader_is_stela(esp)?,
    };
    let mut remove = vec![esp.join(OPTIONS), esp.join(SOURCE), esp.join(BOOT_LOG)];
    if stela == Some(true) {
        remove.insert(0, esp.join(LOADER));
    }
    let mut files = Vec::new();
    for file in remove {
        if file.is_file() {
            fs::remove_file(&file)?;
            files.push(file);
        }
    }
    // Directories go only when nothing else is left in them.
    for directory in [esp.join(MOUNT), esp.join(LOADER_DIRECTORY)] {
        if fs::read_dir(&directory).is_ok_and(|mut entries| entries.next().is_none()) {
            fs::remove_dir(&directory)?;
        }
    }
    Ok(json!({
        "files": files,
        "kept_loader": (stela == Some(false)).then(|| esp.join(LOADER)),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What macOS answers for a rename that keeps an existing file on FAT
    /// and exFAT: ENOTSUP.
    fn unsupported(_: &Path, _: &Path) -> std::io::Result<()> {
        Err(std::io::Error::from(std::io::ErrorKind::Unsupported))
    }

    fn temporary(directory: &Path, bytes: &[u8]) -> tempfile::NamedTempFile {
        let mut file = tempfile::NamedTempFile::new_in(directory).unwrap();
        file.write_all(bytes).unwrap();
        file
    }

    #[test]
    fn a_volume_without_a_keeping_rename_publishes_a_new_file_by_renaming() {
        let temp = tempfile::tempdir().unwrap();
        let (part, path) = (temp.path().join("a.part"), temp.path().join("boot.log"));
        fs::write(&part, b"new").unwrap();
        rename_new_with(&part, &path, unsupported).unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"new");
        // Only the published file is left: the part became it.
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
    }

    #[test]
    fn an_existing_file_is_never_replaced_either_way() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("boot.log");
        fs::write(&path, b"earlier boots").unwrap();
        // A refused part stays for its caller, either way.
        let part = temp.path().join("a.part");
        fs::write(&part, b"new").unwrap();
        for result in [
            rename_new_with(&part, &path, unsupported),
            rename_new(&part, &path),
        ] {
            let error = result.unwrap_err().to_string();
            assert!(error.contains("already exists"), "{error}");
            assert_eq!(fs::read(&part).unwrap(), b"new");
        }
        fs::remove_file(&part).unwrap();
        let error = persist_new(temporary(temp.path(), b"new"), &path)
            .unwrap_err()
            .to_string();
        assert!(error.contains("already exists"), "{error}");
        assert_eq!(fs::read(&path).unwrap(), b"earlier boots");
        // The refused temporary files are removed.
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
    }

    /// The real rename, on whatever file system holds the temporary
    /// directory (`TMPDIR` on a FAT or exFAT volume takes the fallback).
    #[test]
    fn a_new_file_is_published_on_the_temporary_directory_s_file_system() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("boot.log");
        persist_new(temporary(temp.path(), b"new"), &path).unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"new");
        let image = temp.path().join("fdr.img4");
        write_atomic(&image, b"image", true).unwrap();
        assert_eq!(fs::read(&image).unwrap(), b"image");
        assert!(write_atomic(&image, b"other", true).is_err());
        assert_eq!(fs::read(&image).unwrap(), b"image");
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 2);
    }

    #[test]
    fn a_failed_write_names_the_file() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("missing").join("boot.log");
        let error = format!("{:#}", write_atomic(&path, b"new", true).unwrap_err());
        assert!(
            error.contains(&format!("cannot write {}", path.display())),
            "{error}"
        );
    }
}
