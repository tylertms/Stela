//! The setup record, `EFI/stela/setup.json`: what setup wrote, read by every
//! later command and written before each change.
use super::{
    CHOICES_PATH, CHOOSER_SECONDS, Environment, InstallImage, LOADER_PATH, RECORD, addresses,
    bundle::{self, Kind},
    ecid::Ecid,
    host::Host,
    image_is_remote, installed, locate_esp, machines, medium, restore_choice_name, translations,
    wifi,
};
use crate::efi;
use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use std::{
    fs,
    path::{Path, PathBuf},
};
use stela_boot_core::options::{System, Uuid};
use stela_loader::mount_interface::{Choice, Choices};

/// What setup writes, recorded on the ESP so `uninstall` can remove it.
/// Records before schema 8 also named a recoveryOS bundle on the stick, and
/// records before schema 9 kept the data partition's files under `EFI/`;
/// this `stela` reads them only to uninstall them.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub(super) struct Record {
    pub(super) schema_version: u32,
    pub(super) kind: String,
    /// The active macOS build, which Install macOS restores.
    pub(super) build: String,
    pub(super) model: String,
    pub(super) esp: PathBuf,
    /// The USB's FAT32 data partition.
    #[serde(alias = "payload_volume")]
    pub(super) volume: PathBuf,
    /// A schema 1 setup's restore payload.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) payload_directory: Option<PathBuf>,
    /// `stela/boot/<build>` of the active build.
    pub(super) bundle_directory: PathBuf,
    /// Directories setup created on the payload volume, deepest first.
    pub(super) created_directories: Vec<PathBuf>,
    /// `stela.cfg`.
    pub(super) options: String,
    pub(super) complete: bool,
    /// Schema 3: the PC's own disk; read into `installs`.
    #[serde(default, skip_serializing)]
    pub(super) installed: Option<installed::InstalledDisk>,
    /// Schema 3: the installed `stela.efi`'s version.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) stela_version: Option<String>,
    /// The Stela version that made or last updated the setup
    /// (`setup::release`); none before 0.4.
    #[serde(
        default,
        rename = "stela_release",
        skip_serializing_if = "Option::is_none"
    )]
    pub(super) release: Option<String>,
    /// Schema 4: the PC disks macOS installs onto, each its own machine.
    #[serde(default)]
    pub(super) installs: Vec<machines::Install>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) default_install: Option<String>,
    /// Schema 4: the boot-bundle schema the active `stela.efi` reads.
    #[serde(default = "first_schema")]
    pub(super) bundle_schema: u32,
    /// Schema 7: the revision of the active bundles, those of the loader
    /// that prepared them (`stela_loader::boot_bundle::REVISION`); 0 before.
    #[serde(default)]
    pub(super) bundle_revision: u32,
    /// Schema 4: every boot bundle setup wrote.
    #[serde(default)]
    pub(super) bundles: Vec<bundle::Bundle>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) chooser_timeout: Option<u32>,
    /// Schema 5: the install medium; read into `media`.
    #[serde(default, skip_serializing)]
    pub(super) medium: Option<medium::Medium>,
    /// Schema 6: the install media, each a full IPSW on the stick's FAT32
    /// data partition: the active build's, and the next one's while
    /// `update os` writes it.
    #[serde(default)]
    pub(super) media: Vec<medium::Medium>,
    /// Schema 5: the Install choice checks the IPSW as it reads it
    /// (`--verify-on-read`).
    #[serde(default)]
    pub(super) verify_on_read: bool,
    /// Every choice times the engine (`--profile`).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub(super) profile: bool,
    /// Where installs read the restore image; streamed when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) install_image: Option<InstallImage>,
    /// The build an `update os` is writing after it removed the active
    /// build's copy for room: until it is active, the stick cannot install
    /// macOS, and running that update again finishes it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) replacing: Option<String>,
    /// The USB boot log's head when the install medium was last written or
    /// checked whole: a damaged piece the log names before it is put right
    /// (`medium::reported_damage`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) medium_checked: Option<u64>,
    /// The translation store `stela.efi` keeps its packs in, if the stick
    /// has room for one (`setup::translations`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) translations: Option<translations::Store>,
    /// The Wi-Fi network every choice joins (`--wifi`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) wifi: Option<wifi::Network>,
    /// The USB4 devices every choice admits without an IOMMU
    /// (`--usb4-approve`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub(super) usb4_approved: Vec<String>,
    /// The network addresses every choice sets by hand (`--ipv4`, `--dns`).
    #[serde(default, skip_serializing_if = "addresses::Addresses::is_automatic")]
    pub(super) addresses: addresses::Addresses,
}

pub(super) fn first_schema() -> u32 {
    1
}

pub(super) const RECORD_KIND: &str = "stela_setup";

/// Schema 8: no recoveryOS bundle; every setup has an install. Schema 9:
/// the data partition's files are under `stela/`, not `EFI/stela/`.
pub(super) const RECORD_SCHEMA: u32 = 9;

impl Record {
    /// Whether installs stream `ipsw`'s build from Apple's servers rather
    /// than read it from the stick, by the setup's installer
    /// ([`Self::install_kind`]). A full installer's erase restore needs the
    /// macOS system image, which an archive of boot files
    /// (`restore::fetch_inputs`) leaves out.
    pub(super) fn remote_install(&self, ipsw: &Path) -> Result<bool> {
        image_is_remote(ipsw, self.install_kind())
    }

    /// The installer this setup was made as: its recorded choice, else (a
    /// record from before the choice was recorded) whole when it has a
    /// medium that holds the image, else streamed.
    pub(super) fn install_kind(&self) -> InstallImage {
        match self.install_image {
            Some(kind) => kind,
            None if self.media.iter().any(|medium| !medium.remote) => InstallImage::Full,
            None => InstallImage::Network,
        }
    }

    /// What resuming this interrupted setup continues, after the medium the
    /// stopped setup left on the stick: "continuing the Offline installer: 3
    /// of 6 parts of macOS already on the stick".
    pub(super) fn resume_summary(&self) -> String {
        if self.install_kind() == InstallImage::Network {
            return "continuing the Online installer".into();
        }
        let found = self
            .medium_of(&self.build)
            .filter(|medium| !medium.remote)
            .map(|medium| medium::download::pieces_on_usb(&medium.directory()));
        match found {
            Some((present, Some(total))) => format!(
                "continuing the Offline installer: {present} of {total} parts of macOS already \
                 on the stick"
            ),
            Some((present, None)) if present > 0 => format!(
                "continuing the Offline installer: {present} parts of macOS already on the stick"
            ),
            _ => "continuing the Offline installer: no part of macOS on the stick yet".into(),
        }
    }

    /// Bring an older schema's record to one `uninstall` reads: its
    /// install medium is listed and its installed disk becomes the first
    /// install.
    pub(super) fn migrate(&mut self) {
        if let Some(medium) = self.medium.take()
            && !self.media.iter().any(|m| m.build == medium.build)
        {
            self.media.push(medium);
        }
        if let Some(disk) = self.installed.take() {
            self.installs.push(machines::Install {
                name: machines::FIRST_NAME.into(),
                ecid: None,
                disk,
            });
        }
    }

    /// The install `stela.cfg` boots.
    pub(super) fn default(&self) -> Option<&machines::Install> {
        self.default_install
            .as_ref()
            .and_then(|name| self.installs.iter().find(|i| &i.name == name))
            .or(self.installs.first())
    }

    /// The install `name` names, or the default one.
    pub(super) fn install(&self, name: Option<&str>) -> Result<&machines::Install> {
        match name {
            Some(name) => self
                .installs
                .iter()
                .find(|i| i.name == name)
                .with_context(|| format!("this setup has no install named {name:?}")),
            None => self.default().with_context(|| {
                format!(
                    "the setup on {} names no installed disk (--target)",
                    self.esp.display()
                )
            }),
        }
    }

    /// The generation of the active bundles.
    pub(super) fn generation(&self) -> bundle::Generation {
        bundle::Generation {
            schema: self.bundle_schema,
            revision: self.bundle_revision,
        }
    }

    pub(super) fn line(&self, mut system: System) -> String {
        // Every bundle of a setup is of its one Mac.
        system.guest_pes = self
            .bundles
            .iter()
            .map(|bundle| bundle.processors)
            .find(|&processors| processors != 0);
        system.profile = self.profile;
        // The key file is `--wifi`'s default one.
        system.ssid = self.wifi.as_ref().map(|network| network.ssid.clone());
        system.wifi_country = self
            .wifi
            .as_ref()
            .and_then(|network| network.country.clone());
        system.usb4_approved = self
            .usb4_approved
            .iter()
            .filter_map(|uuid| Uuid::parse(uuid))
            .collect();
        self.addresses.apply(&mut system);
        format!("{system}\n")
    }

    /// The option line that boots `install`.
    pub(super) fn boot_line(&self, install: &machines::Install) -> Result<String> {
        Ok(self.line(install.disk.options(install.ecid)?))
    }

    /// `stela.cfg`: the default install. Every setup this `stela` reads
    /// has one ([`read_record`]).
    pub(super) fn options_text(&self) -> Result<String> {
        self.default()
            .map(|install| self.boot_line(install))
            .transpose()
            .map(Option::unwrap_or_default)
    }

    /// The active build's restore bundle of `kind` (erase or update) for
    /// `ecid`'s machine, under the AP ticket Apple's signing service
    /// personalized for it. A bundle under the IPSW's global ticket, which
    /// an older setup without the signing opt-in prepared, counts as
    /// missing, so `repair --ipsw` prepares it again.
    pub(super) fn restore_bundle(&self, kind: Kind, ecid: Option<Ecid>) -> Option<&bundle::Bundle> {
        self.bundles.iter().find(|b| {
            b.same_kind(bundle::Identity {
                kind,
                build: &self.build,
                ecid,
                generation: self.generation(),
            }) && b.personalized
        })
    }

    /// The install medium of `build`.
    pub(super) fn medium_of(&self, build: &str) -> Option<&medium::Medium> {
        self.media.iter().find(|m| m.build == build)
    }

    /// The active build's install medium.
    pub(super) fn active_medium(&self) -> Option<&medium::Medium> {
        self.medium_of(&self.build)
    }

    /// Whether the installs restore from install media: the setup has
    /// written a medium or a restore bundle, as every setup does.
    pub(super) fn installs_media(&self) -> bool {
        !self.installs.is_empty() && (!self.media.is_empty() || !self.bundles.is_empty())
    }

    /// The machines of the installs, each once.
    pub(super) fn install_machines(&self) -> Vec<Option<Ecid>> {
        let mut machines = Vec::new();
        for install in &self.installs {
            if !machines.contains(&install.ecid) {
                machines.push(install.ecid);
            }
        }
        machines
    }

    /// The option line of `install`'s Install macOS choice
    /// ([`Kind::Restore`]) or Update macOS choice
    /// ([`Kind::Update`]): its restore bundle of that kind, the install
    /// medium and its partitions. None without the active build's medium or
    /// the machine's personalized bundle.
    pub(super) fn restore_line(
        &self,
        kind: Kind,
        install: &machines::Install,
    ) -> Result<Option<String>> {
        let Some(medium) = self.active_medium() else {
            return Ok(None);
        };
        if self.restore_bundle(kind, install.ecid).is_none() {
            return Ok(None);
        }
        let restore = bundle::firmware_path_of(bundle::Identity {
            kind,
            build: &self.build,
            ecid: install.ecid,
            generation: self.generation(),
        });
        let mut system = install.disk.bundle_options(&restore, install.ecid)?;
        medium.restore_from(&mut system)?;
        system.update_restore = kind == Kind::Update;
        system.verify_on_read = self.verify_on_read;
        system.development_restore = kind == Kind::Restore;
        Ok(Some(self.line(system)))
    }

    /// The Install macOS choice's option line.
    pub(super) fn install_line(&self, install: &machines::Install) -> Result<Option<String>> {
        self.restore_line(Kind::Restore, install)
    }

    /// The name of `install`'s Install macOS or Update macOS
    /// (Stela) choice.
    pub(super) fn restore_choice(&self, kind: Kind, install: &machines::Install) -> String {
        if self.installs.len() == 1 {
            return restore_choice_name(kind).into();
        }
        let several = match kind {
            Kind::Update => "Update macOS",
            _ => "Install macOS",
        };
        format!("{} - {several}", install.name)
    }

    /// The restore bundles the installs' machines need of the active build
    /// and lack: both kinds of every install's while the setup has the
    /// build's install medium.
    pub(super) fn missing_restore_bundles(&self) -> Vec<(Kind, Option<Ecid>)> {
        if self.active_medium().is_none() {
            return Vec::new();
        }
        let mut missing = Vec::new();
        for kind in bundle::RESTORE_KINDS {
            for ecid in self.install_machines() {
                let present = self
                    .restore_bundle(kind, ecid)
                    .is_some_and(|b| b.directory(&self.volume).join(bundle::MANIFEST).is_file());
                if !present {
                    missing.push((kind, ecid));
                }
            }
        }
        missing
    }

    /// The startup choices: each install, default first, and its Install
    /// macOS. None without an installed disk.
    pub(super) fn choices(&self) -> Result<Option<Choices>> {
        let Some(default) = self.default() else {
            return Ok(None);
        };
        let mut entries = Vec::new();
        let others = self.installs.iter().filter(|i| i.name != default.name);
        for install in std::iter::once(default).chain(others) {
            entries.push(Choice {
                name: install.name.clone(),
                options: self.boot_line(install)?.trim().into(),
            });
            for kind in bundle::RESTORE_KINDS {
                if let Some(line) = self.restore_line(kind, install)? {
                    entries.push(Choice {
                        name: self.restore_choice(kind, install),
                        options: line.trim().into(),
                    });
                }
            }
        }
        // The list opens on its own with several installs, and while the
        // USB can install macOS.
        let mut installing = false;
        for install in &self.installs {
            installing |= self.install_line(install)?.is_some();
        }
        let seconds = if self.installs.len() > 1 || installing {
            CHOOSER_SECONDS
        } else {
            0
        };
        Ok(Some(Choices {
            timeout_seconds: self.chooser_timeout.unwrap_or(seconds),
            entries,
        }))
    }

    pub(super) fn choices_text(&self) -> Result<Option<String>> {
        self.choices()?
            .map(|choices| choices.encode().map_err(anyhow::Error::msg))
            .transpose()
    }

    /// Forget the bundles of machines the setup no longer has; returns
    /// them for [`remove_bundles`] once the record no longer names them.
    pub(super) fn drop_stale_bundles(&mut self) -> Vec<bundle::Bundle> {
        let machines = self.install_machines();
        let (kept, stale) = self
            .bundles
            .drain(..)
            .partition(|b| machines.contains(&b.ecid));
        self.bundles = kept;
        stale
    }

    /// Whether the record names bundles or media of another build or
    /// generation than the active one: what an interrupted update left,
    /// which `update::prune` removes.
    pub(super) fn has_leftovers(&self) -> bool {
        let generation = self.generation();
        self.bundles
            .iter()
            .any(|b| b.build != self.build || b.generation() != generation)
            || self.media.iter().any(|m| m.build != self.build)
    }

    /// Every build directory the record or a bundle names.
    pub(super) fn build_directories(&self) -> Vec<PathBuf> {
        let mut builds = vec![&self.build];
        builds.extend(self.bundles.iter().map(|b| &b.build));
        let mut directories = Vec::new();
        for build in builds {
            let directory = bundle::build_directory(&self.volume, build);
            if !directories.contains(&directory) {
                directories.push(directory);
            }
        }
        directories
    }
}

/// Delete the directories of bundles the record no longer names.
pub(super) fn remove_bundles(record: &Record, stale: &[bundle::Bundle]) -> Result<()> {
    for kept in stale {
        let directory = kept.directory(&record.volume);
        if directory.is_dir() {
            fs::remove_dir_all(directory)?;
        }
    }
    Ok(())
}

/// Write `stela.cfg`, the startup choices and the entry for the record.
pub(super) fn publish(record: &mut Record) -> Result<()> {
    if record.bundles.iter().all(|bundle| bundle.processors == 0) {
        let generation = record.generation();
        for kept in &mut record.bundles {
            if kept.build == record.build
                && kept.generation() == generation
                && let Some(processors) = bundle::recorded_processors(
                    &kept.directory(&record.volume),
                    &kept.manifest_sha256,
                )
            {
                kept.processors = processors;
                break;
            }
        }
    }
    record.options = record.options_text()?;
    let choices = record.choices_text()?;
    let tool = record.esp.join(LOADER_PATH);
    efi::add_mount(&record.esp, &fs::read(&tool)?, None, Some(&record.options))?;
    let path = record.esp.join(CHOICES_PATH);
    match choices {
        Some(text) => efi::write_atomic(&path, text.as_bytes(), false)?,
        None if path.exists() => fs::remove_file(&path)?,
        None => {}
    }
    Ok(())
}

/// The record on the ESP of `usb`, which must be a complete setup's.
pub(super) fn read_record(environment: &Environment<'_>, usb: &Path) -> Result<Record> {
    let esp = locate_esp(environment.host, usb)?;
    let path = esp.join(RECORD);
    ensure!(
        path.is_file(),
        "{} holds no Stela setup: make an installer on the stick with `stela create`",
        esp.display()
    );
    let header: serde_json::Value = stela_engine::read_json(&path)?;
    let schema = header["schema_version"].as_u64().unwrap_or(0);
    ensure!(
        header["kind"] == RECORD_KIND && schema <= u64::from(RECORD_SCHEMA),
        "{} is not a setup record this version understands; it may come from a newer stela",
        path.display()
    );
    ensure!(
        schema == u64::from(RECORD_SCHEMA),
        "{} is a schema {schema} setup, which this stela no longer reads: {}. Start over with \
         `stela create` on the stick's whole disk, which erases it",
        path.display(),
        match schema {
            8 => "it keeps its files under EFI/ on the data partition",
            _ => "it booted a recoveryOS bundle from the stick",
        }
    );
    let mut value = header;
    rebase(
        &mut value,
        &esp,
        &current_volume(environment.host, &esp, usb),
    );
    let record: Record = serde_json::from_value(value)
        .with_context(|| format!("cannot read setup record {}", path.display()))?;
    ensure!(
        !record.installs.is_empty(),
        "{} names no install. Start over with `stela create` on the stick's whole disk, which \
         erases it",
        path.display()
    );
    Ok(record)
}

/// Where the data partition of the stick whose EFI partition is `esp` is
/// mounted now: `usb` when it names it, else the FAT32 data partition beside
/// the EFI partition, else the EFI partition itself, as for a setup with one
/// partition.
pub(super) fn current_volume(host: &dyn Host, esp: &Path, usb: &Path) -> PathBuf {
    if usb != esp {
        return usb.to_path_buf();
    }
    medium::locate(host, esp, None).map_or_else(|_| esp.to_path_buf(), |(volume, _)| volume)
}

/// Move every path of the record `value` to where the stick is mounted now:
/// `esp`, and the data partition at `volume` ([`current_volume`]). The record
/// holds the paths of the computer and user that wrote it, and another user,
/// computer or system mounts the same USB somewhere else.
fn rebase(value: &mut serde_json::Value, esp: &Path, volume: &Path) {
    let text = |key: &str| value[key].as_str().map(str::to_owned);
    let (Some(old_esp), Some(old_volume)) = (
        text("esp"),
        text("volume").or_else(|| text("payload_volume")),
    ) else {
        return;
    };
    // A setup with one partition keeps everything on its EFI partition. With
    // two, the data partition's paths stay as they were only when no data
    // partition is mounted now.
    let mut moves = Vec::new();
    if old_volume != old_esp && volume != esp {
        moves.push((old_volume, volume));
    }
    moves.push((old_esp, esp));
    fn walk(value: &mut serde_json::Value, moves: &[(String, &Path)]) {
        match value {
            serde_json::Value::String(text) => {
                for (old, new) in moves {
                    let Some(rest) = text.strip_prefix(old.as_str()) else {
                        continue;
                    };
                    // The whole old path, not another one that starts alike.
                    let separator = |c: char| c == '/' || c == '\\';
                    if !(rest.is_empty() || rest.starts_with(separator) || old.ends_with(separator))
                    {
                        continue;
                    }
                    let mut path = new.to_path_buf();
                    path.extend(rest.split(separator).filter(|part| !part.is_empty()));
                    *text = path.to_string_lossy().into_owned();
                    break;
                }
            }
            serde_json::Value::Array(values) => values.iter_mut().for_each(|v| walk(v, moves)),
            serde_json::Value::Object(fields) => {
                for (key, value) in fields {
                    // The boot options name UEFI paths, which do not move.
                    if key != "options" {
                        walk(value, moves);
                    }
                }
            }
            _ => {}
        }
    }
    walk(value, &moves);
}

/// Any schema's record, for `uninstall`: the bundles of an older
/// schema's record that name no kind or `recovery` are recoveryOS bundles,
/// which this `stela` has no kind for; their build directories are removed
/// with the rest. Before schema 9 the data partition's files were under
/// `EFI/`, which the record's volumes then gain, so every directory derived
/// from them is where that setup wrote it. Its paths follow the stick to where
/// it is mounted now ([`rebase`]): the EFI partition `esp`, and the data
/// partition `usb` names or beside it.
pub(super) fn read_any_record(host: &dyn Host, esp: &Path, usb: &Path) -> Result<Record> {
    let path = &esp.join(RECORD);
    let mut value: serde_json::Value = stela_engine::read_json(path)?;
    let schema = value["schema_version"].as_u64().unwrap_or(0);
    ensure!(
        value["kind"] == RECORD_KIND && (1..=u64::from(RECORD_SCHEMA)).contains(&schema),
        "{} is not a setup record this version understands",
        path.display()
    );
    rebase(&mut value, esp, &current_volume(host, esp, usb));
    if schema < 8
        && let Some(bundles) = value
            .get_mut("bundles")
            .and_then(serde_json::Value::as_array_mut)
    {
        bundles.retain(|bundle| {
            bundle
                .get("kind")
                .and_then(serde_json::Value::as_str)
                .is_some_and(|kind| kind != "recovery")
        });
    }
    let mut record: Record = serde_json::from_value(value)
        .with_context(|| format!("{} cannot be read", path.display()))?;
    record.migrate();
    if schema < 9 {
        record.volume.push("EFI");
        for medium in record.media.iter_mut().chain(record.medium.as_mut()) {
            medium.volume.push("EFI");
        }
        if let Some(store) = &mut record.translations {
            store.volume.push("EFI");
        }
    }
    Ok(record)
}

/// The record on the EFI partition `esp` when it is this schema's and its
/// first setup was interrupted (`complete` is false): what `create::plan`
/// resumes. Its paths follow the stick to where it is mounted now, as
/// [`read_record`]'s do: a USB taken out part way is often mounted somewhere
/// else when setup runs again.
pub(super) fn interrupted(host: &dyn Host, esp: &Path, usb: &Path) -> Option<Record> {
    let mut value: serde_json::Value = stela_engine::read_json(&esp.join(RECORD)).ok()?;
    rebase(&mut value, esp, &current_volume(host, esp, usb));
    serde_json::from_value::<Record>(value)
        .ok()
        .filter(|record| record.schema_version == RECORD_SCHEMA && !record.complete)
}

/// Replace the record through a synced sibling file, so a stop at any point
/// leaves the previous record or this one whole: the record is the journal
/// every interrupted setup and update is resumed from.
///
/// It spells each path on the stick the way a read does ([`rebase`]), however
/// the path was built: `stela/boot` joined whole keeps its `/` where the
/// system writes `\`, and a record just made would differ from the same
/// record read and written again.
pub(super) fn write_record(esp: &Path, record: &Record) -> Result<()> {
    let path = esp.join(RECORD);
    fs::create_dir_all(path.parent().unwrap())?;
    let mut value = serde_json::to_value(record)?;
    rebase(&mut value, &record.esp, &record.volume);
    // Through the record again: its fields keep their order in the file.
    let mut record: Record = serde_json::from_value(value)?;
    record.schema_version = RECORD_SCHEMA;
    let mut bytes = serde_json::to_vec_pretty(&record)?;
    bytes.push(b'\n');
    efi::write_atomic(&path, &bytes, false)
}

#[cfg(test)]
mod rebase_tests {
    use super::*;

    #[test]
    fn a_record_follows_the_usb_to_where_it_is_mounted_now() {
        let mut value = serde_json::json!({
            "esp": "/run/media/root/EFI",
            "volume": "/run/media/root/STELA",
            "bundle_directory": "/run/media/root/STELA/stela/boot/27A1",
            "created_directories": ["/run/media/root/STELA/stela", "/run/media/root/STELATE"],
            "options": "--bundle \\stela\\boot /run/media/root/STELA",
            "media": [{"volume": "/run/media/root/STELA"}],
            "translations": {"volume": "/run/media/root/STELA", "pack_bytes": 1},
        });
        rebase(
            &mut value,
            Path::new("/media/user/EFI"),
            Path::new("/media/user/STELA1"),
        );
        assert_eq!(value["esp"], "/media/user/EFI");
        assert_eq!(value["volume"], "/media/user/STELA1");
        assert_eq!(
            Path::new(value["bundle_directory"].as_str().unwrap()),
            Path::new("/media/user/STELA1/stela/boot/27A1")
        );
        assert_eq!(
            Path::new(value["created_directories"][0].as_str().unwrap()),
            Path::new("/media/user/STELA1/stela")
        );
        // Another path that only starts alike, and the boot options, stay.
        assert_eq!(value["created_directories"][1], "/run/media/root/STELATE");
        assert_eq!(
            value["options"],
            "--bundle \\stela\\boot /run/media/root/STELA"
        );
        assert_eq!(value["media"][0]["volume"], "/media/user/STELA1");
        assert_eq!(value["translations"]["volume"], "/media/user/STELA1");
        // A record written on Windows, read where `/` separates.
        let mut value = serde_json::json!({
            "esp": "E:\\",
            "volume": "F:\\",
            "bundle_directory": "F:\\stela\\boot\\27A1",
        });
        rebase(&mut value, Path::new("/mnt/efi"), Path::new("/mnt/data"));
        assert_eq!(value["esp"], "/mnt/efi");
        assert_eq!(
            Path::new(value["bundle_directory"].as_str().unwrap()),
            Path::new("/mnt/data/stela/boot/27A1")
        );
        // One partition: everything follows the EFI partition.
        let mut value = serde_json::json!({
            "esp": "/a",
            "volume": "/a",
            "bundle_directory": "/a/stela/boot/27A1",
        });
        rebase(&mut value, Path::new("/b"), Path::new("/b"));
        assert_eq!(value["volume"], "/b");
        assert_eq!(
            Path::new(value["bundle_directory"].as_str().unwrap()),
            Path::new("/b/stela/boot/27A1")
        );
    }
}
