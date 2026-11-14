//! A machine's state on its machine-state partition: the NOR image (NVRAM)
//! and, after it, the SEP's area (its UID/GID roots and anti-replay store,
//! `stela_soc::machine::sep_state`, which the SEP model owns;
//! setup only copies the area whole and never reads its keys).
//!
//! - `nvram reset` writes a fresh NOR image, as clearing a Mac's NVRAM
//!   does; the SEP's area is kept.
//! - `state backup FILE` copies the NOR image and the SEP's area to a new
//!   file only its owner can read: the area holds the machine's keys. The
//!   file also names the machine (ECID, partition and serial) and carries
//!   the SHA-256 of what it holds.
//! - `state restore FILE` writes such a file back to the machine it came
//!   from, and to no other.
//! - `log` prints the boot log `stela.efi` keeps after the SEP's area
//!   on a partition of at least `boot_log::PARTITION_BYTES`
//!   (`stela_runtime::pc::boot_log`), and writes nothing.
//!
//! Every command finds the install's partition by its recorded GUID and disk
//! serial (`installed::confirm`), so no other partition is written; `--nor`
//! names it only where this computer cannot find it, and must name the same
//! partition. Each write's plan names the partition, its GUID and its disk
//! serial. An install whose machine state lives in its SSD partition's tail
//! is named by that partition, and every command addresses the tail
//! (`installed::state_area`).
use super::{
    Environment, Record, Request, check_installed_efi,
    ecid::Ecid,
    host::{Disk, Host, bytes_text},
    installed, machines,
};
use anyhow::{Context, Result, anyhow, bail, ensure};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    fmt::Write as _,
    fs,
    io::{BufRead, Read, Write},
    path::Path,
};
use stela_loader::boot_selection;
use {
    stela_runtime::pc::boot_log, stela_soc::machine::sep_state, stela_soc_data::nor::CAPACITY,
    stela_soc_data::nor::Image, stela_soc_data::nor::nvram::Variables,
};

const MAGIC: &str = "STELA MACHINE STATE 1";
const STATE_BYTES: usize = CAPACITY + stela_formats::machine_state::SEP_BYTES as usize;

/// What a backup file says of itself.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
struct Header {
    install: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    ecid: Option<Ecid>,
    model: String,
    nor_serial: String,
    nor_guid: String,
    nor_bytes: usize,
    sep_bytes: usize,
    sha256: String,
}

/// The NVRAM a NOR image holds.
pub(super) enum Nvram {
    Blank,
    Valid(Variables),
    Invalid(stela_contracts::Refusal),
}

pub(super) fn nvram(bytes: &[u8]) -> Nvram {
    if bytes.iter().all(|&b| b == 0) {
        return Nvram::Blank;
    }
    let parsed = Image::from_bytes(bytes.to_vec()).and_then(|image| {
        let bank = image.current_bank()?;
        Variables::parse(image.bank(bank)?)
    });
    match parsed {
        Ok(variables) => Nvram::Valid(variables),
        Err(error) => Nvram::Invalid(error),
    }
}

/// What the SEP's area holds.
pub(super) fn sep(area: &[u8]) -> Result<bool, stela_contracts::Refusal> {
    sep_state::blank(&area[..stela_formats::machine_state::SEP_ROOTS_BYTES]).map(|blank| !blank)
}

/// What the next boot of an NVRAM is, as `stela.efi` will choose it.
pub(super) fn next_boot(variables: &Variables) -> String {
    match boot_selection::select(variables) {
        Ok(selection) => {
            let target = match selection.target {
                boot_selection::Target::MacOs { safe: false } => "macOS".into(),
                boot_selection::Target::MacOs { safe: true } => "macOS in safe mode".into(),
                boot_selection::Target::PairedRecovery => "the paired recoveryOS (the Recovery \
                     volume's, else System Recovery)"
                    .into(),
                boot_selection::Target::SystemRecovery => {
                    "System Recovery (the RecoveryOSContainer's)".into()
                }
                boot_selection::Target::Upgrade => {
                    "the macOS update staged on the Update volume".to_string()
                }
                boot_selection::Target::RecoveryMode => "no system: recovery mode, as an \
                     interrupted restore leaves it; stela.efi shows its startup choices for \
                     Install macOS"
                    .into(),
            };
            if selection.reasons.is_empty() {
                target
            } else {
                format!("{target} ({})", selection.reasons.join(", "))
            }
        }
        Err(error) => format!("refused by stela.efi: {error:#}"),
    }
}

/// The install and confirmed machine state a state command acts on.
fn target<'a>(
    environment: &Environment<'_>,
    record: &'a Record,
    command: &Request,
) -> Result<(&'a machines::Install, installed::StateArea)> {
    let install = record.install(command.install.as_deref())?;
    let area = installed::state_area(environment.host, &install.disk, command.nor.as_deref())?;
    Ok((install, area))
}

/// A write's plan line naming the partition it writes.
fn partition_line(install: &machines::Install, area: &installed::StateArea) -> String {
    format!(
        "  partition      {} (partition {}, disk serial {})\n",
        area.describe(),
        install.disk.nor_guid,
        install.disk.nor_serial
    )
}

/// `nvram reset`.
pub(super) fn reset_nvram(
    environment: &Environment<'_>,
    record: &Record,
    command: &Request,
) -> Result<String> {
    check_installed_efi(environment, record)?;
    let (install, area) = target(environment, record, command)?;
    let host = environment.host;
    let current = area.read(host, 0, CAPACITY)?;
    let mut text = format!("Reset the NVRAM of install \"{}\":\n", install.name);
    text.push_str(&partition_line(install, &area));
    match nvram(&current) {
        Nvram::Blank => writeln!(text, "  NVRAM          blank; it is written fresh")?,
        Nvram::Valid(variables) => {
            let count = [
                stela_soc_data::nor::nvram::Partition::System,
                stela_soc_data::nor::nvram::Partition::Common,
            ]
            .iter()
            .map(|p| variables.list(*p).count())
            .sum::<usize>();
            let removed = match count {
                1 => "1 variable is".to_owned(),
                _ => format!("{count} variables are"),
            };
            writeln!(
                text,
                "  NVRAM          {removed} removed, boot-volume and boot-args among any set; \
                 macOS's startup disk choice is made again in macOS"
            )?;
        }
        Nvram::Invalid(error) => writeln!(
            text,
            "  NVRAM          unreadable ({error}); it is written fresh"
        )?,
    }
    writeln!(text, "  SEP state      kept")?;
    if command.dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    let fresh = Image::fresh();
    area.write(host, 0, fresh.bytes())?;
    ensure!(
        area.read(host, 0, CAPACITY)? == fresh.bytes(),
        "{} did not read back as written",
        area.describe()
    );
    text.push_str("Done.\n");
    Ok(text)
}

/// `log`: the boot log `stela.efi` keeps after the SEP's area, when
/// the partition holds one.
/// The USB's own log (`mount_interface::BOOT_LOG_FILE`), which every boot
/// writes from its first line, then the install's on its machine-state
/// partition; either may be missing.
pub(super) fn boot_log(
    environment: &Environment<'_>,
    record: &Record,
    command: &Request,
) -> Result<String> {
    let path = record.esp.join(crate::efi::BOOT_LOG);
    let usb = match fs::read(&path) {
        Ok(region) => boot_log::read(&region)
            .with_context(|| format!("{} has no boot log", path.display()))
            .map(|log| shown(&format!("Boot log on the stick ({})", path.display()), &log)),
        // A folder the host refuses, or an EFI partition it no longer
        // mounts: the same file through the stick's raw partition.
        Err(error) => [record.esp.as_path(), command.usb.as_path()]
            .into_iter()
            .find_map(|usb| usb_disk(environment.host, usb))
            .context("its disk is not known")
            .and_then(|disk| {
                let logs = crate::boot_log::disk_logs(environment.host, &disk)?.usb;
                ensure!(
                    !logs.is_empty(),
                    "{} holds no {} either",
                    crate::boot_log::describe(&disk),
                    crate::efi::BOOT_LOG
                );
                Ok(raw_text(&disk, logs, "Boot log on the stick"))
            })
            .with_context(|| format!("cannot read {}: {error}", path.display())),
    };
    let install = install_log(environment, record, command);
    match (usb, install) {
        (Ok(usb), Ok(install)) => Ok(format!("{usb}\n{install}")),
        (Ok(usb), Err(error)) => Ok(format!("{usb}\n{error:#}\n")),
        (Err(error), Ok(install)) => Ok(format!("{error:#}\n\n{install}")),
        (Err(usb), Err(install)) => Err(anyhow!("{usb:#}\n{install:#}")),
    }
}

/// `log` after a line naming it `title` and saying what it holds.
fn shown(title: &str, log: &boot_log::Log) -> String {
    let mut text = format!("{title}: {}", crate::boot_log::summary(log));
    text.push_str(&String::from_utf8_lossy(&log.stream));
    if !text.ends_with('\n') {
        text.push('\n');
    }
    text
}

/// `logs`, read directly from `disk`, each named `title` with its place.
fn raw_text(disk: &Disk, logs: Vec<(String, boot_log::Log)>, title: &str) -> String {
    let disk = crate::boot_log::describe(disk);
    logs.iter()
        .map(|(place, log)| shown(&format!("{title} ({disk}, {place}, read directly)"), log))
        .collect::<Vec<_>>()
        .join("\n")
}

/// The whole disk `usb` names, or the disk of the volume mounted there.
fn usb_disk(host: &dyn Host, usb: &Path) -> Option<Disk> {
    let identifier = match host.whole_disk(usb) {
        Ok(Some(identifier)) => identifier,
        _ => host.volume(usb).ok()?.disk?,
    };
    host.disks()
        .ok()?
        .into_iter()
        .find(|disk| disk.identifier == identifier)
}

/// `log` for a USB whose setup record cannot be read (`why`): every
/// log on its disk, read directly, which needs neither the record nor a
/// mounted EFI partition.
pub(super) fn raw_boot_log(host: &dyn Host, usb: &Path, why: &anyhow::Error) -> Result<String> {
    let disk = usb_disk(host, usb)
        .with_context(|| format!("{why:#}; nor is the disk of {} known", usb.display()))?;
    let logs = crate::boot_log::disk_logs(host, &disk)
        .with_context(|| format!("{why:#}; nor can its disk be read directly"))?
        .all();
    ensure!(
        !logs.is_empty(),
        "{why:#}; nor does {} hold a Stela boot log",
        crate::boot_log::describe(&disk)
    );
    Ok(format!(
        "The setup on {} cannot be read ({why:#}), so its disk was read directly.\n\n{}",
        usb.display(),
        raw_text(&disk, logs, "Boot log")
    ))
}

fn install_log(
    environment: &Environment<'_>,
    record: &Record,
    command: &Request,
) -> Result<String> {
    let (install, area) = target(environment, record, command)?;
    let nor = area.describe();
    let bytes = area.bytes;
    ensure!(
        bytes >= boot_log::PARTITION_BYTES,
        "{nor} holds {}; the boot log needs {}, so stela.efi boots install \"{}\" without one",
        bytes_text(bytes),
        bytes_text(boot_log::PARTITION_BYTES),
        install.name
    );
    let region = area.read(
        environment.host,
        boot_log::OFFSET,
        boot_log::LOG_BYTES as usize,
    )?;
    let log = boot_log::read(&region)
        .with_context(|| format!("{nor} has no boot log of install \"{}\"", install.name))?;
    let mut text = format!(
        "Boot log of install \"{}\" ({nor}): {}",
        install.name,
        crate::boot_log::summary(&log)
    );
    text.push_str(&String::from_utf8_lossy(&log.stream));
    if !text.ends_with('\n') {
        text.push('\n');
    }
    Ok(text)
}

/// `state backup FILE`.
pub(super) fn backup(
    environment: &Environment<'_>,
    record: &Record,
    command: &Request,
    path: &Path,
) -> Result<String> {
    let (install, area) = target(environment, record, command)?;
    let nor = area.describe();
    ensure!(
        !path.exists(),
        "{} exists; choose a new file for the backup",
        path.display()
    );
    let bytes = area.read(environment.host, 0, STATE_BYTES)?;
    let header = Header {
        install: install.name.clone(),
        ecid: install.ecid,
        model: record.model.clone(),
        nor_serial: install.disk.nor_serial.clone(),
        nor_guid: install.disk.nor_guid.clone(),
        nor_bytes: CAPACITY,
        sep_bytes: stela_formats::machine_state::SEP_BYTES as usize,
        sha256: hex::encode(Sha256::digest(&bytes)),
    };
    let mut text = format!(
        "Back up the machine state of install \"{}\" ({nor}) to {}:\n",
        install.name,
        path.display()
    );
    if let Some(ecid) = install.ecid {
        writeln!(text, "  ECID           {ecid}")?;
    }
    writeln!(
        text,
        "  holds          the NOR image (NVRAM) and the SEP's area, with the machine's keys: \
         keep the file private; {}",
        if cfg!(unix) {
            "it is created readable by its owner only"
        } else {
            "it gets the access of the folder it is created in"
        }
    )?;
    if command.dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    // Windows gives a new file its folder's access rules instead.
    #[cfg(unix)]
    std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
    let mut file = options
        .open(path)
        .with_context(|| format!("cannot create {}", path.display()))?;
    writeln!(file, "{MAGIC}")?;
    writeln!(file, "{}", serde_json::to_string(&header)?)?;
    file.write_all(&bytes)?;
    file.sync_all()?;
    text.push_str("Done.\n");
    Ok(text)
}

fn read_backup(path: &Path) -> Result<(Header, Vec<u8>)> {
    let file = fs::File::open(path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut reader = std::io::BufReader::new(file);
    let mut magic = String::new();
    reader.read_line(&mut magic)?;
    ensure!(
        magic.trim_end() == MAGIC,
        "{} is not a Stela machine-state backup",
        path.display()
    );
    let mut line = String::new();
    reader.read_line(&mut line)?;
    let header: Header = serde_json::from_str(line.trim_end())
        .with_context(|| format!("{} has a damaged header", path.display()))?;
    let mut bytes = Vec::new();
    reader.read_to_end(&mut bytes)?;
    ensure!(
        header.nor_bytes == CAPACITY
            && header.sep_bytes == stela_formats::machine_state::SEP_BYTES as usize
            && bytes.len() == STATE_BYTES,
        "{} holds a machine-state layout this stela does not write",
        path.display()
    );
    ensure!(
        hex::encode(Sha256::digest(&bytes)) == header.sha256,
        "{} is damaged: its contents differ from their SHA-256",
        path.display()
    );
    Ok((header, bytes))
}

/// `state restore FILE`.
pub(super) fn restore(
    environment: &Environment<'_>,
    record: &Record,
    command: &Request,
    path: &Path,
) -> Result<String> {
    check_installed_efi(environment, record)?;
    let (header, bytes) = read_backup(path)?;
    let (install, area) = target(environment, record, command)?;
    let (host, nor) = (environment.host, area.describe());
    ensure!(
        header.nor_guid.eq_ignore_ascii_case(&install.disk.nor_guid),
        "{} is the state of partition {}, not install \"{}\"'s {}; a machine's state is \
         restored only to that machine",
        path.display(),
        header.nor_guid,
        install.name,
        install.disk.nor_guid
    );
    match (header.ecid, install.ecid) {
        (Some(saved), Some(current)) if saved != current => bail!(
            "{} is the state of ECID {saved}, but install \"{}\" is ECID {current}",
            path.display(),
            install.name
        ),
        _ => {}
    }
    let current = area.read(host, 0, STATE_BYTES)?;
    let mut text = format!(
        "Restore the machine state of install \"{}\" from {}:\n",
        install.name,
        path.display()
    );
    text.push_str(&partition_line(install, &area));
    if current == bytes {
        writeln!(text, "  state          already as backed up")?;
        return Ok(text);
    }
    writeln!(
        text,
        "  writes         the NOR image (NVRAM) and the SEP's area as backed up"
    )?;
    writeln!(
        text,
        "  note           this returns the SEP's anti-replay store to the backup's: nonces \
         issued since then are forgotten, which the SEP cannot detect"
    )?;
    if command.dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    area.write(host, 0, &bytes)?;
    ensure!(
        area.read(host, 0, STATE_BYTES)? == bytes,
        "{nor} did not read back as written"
    );
    text.push_str("Done.\n");
    Ok(text)
}
