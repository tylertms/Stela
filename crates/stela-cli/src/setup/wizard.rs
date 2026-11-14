//! The create wizard: the questions that make an installer on a stick plain
//! `stela` or `stela create` chose, asked through [`Ui`] so tests script
//! them. Only real choices are asked: the installer always (Offline shown
//! unavailable with its reason where it cannot be made), and Wi-Fi for an
//! Online installer. The macOS version is the newest tested one and the
//! Mac the one supported model; Review shows both, and "Change something"
//! changes any item without walking every step again. No disk is written
//! here.
use super::{
    Environment, InstallImage, Request, firmware,
    home::Stick,
    host::{self, Disk, Host, Removal},
    sources,
    ui::{self, Ui},
    wifi,
};
use crate::restore;
use anyhow::{Context, Result, ensure};
use std::path::{Path, PathBuf};

pub(super) mod menu;
pub(super) mod theme;

use menu::Item;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct Usb {
    id: String,
    name: String,
    bytes: u64,
    removal: Removal,
    serial: Option<String>,
}

impl Usb {
    pub(super) fn from_disk(disk: &Disk, host: &dyn Host) -> Self {
        Self {
            id: disk.identifier.clone(),
            name: disk.name.clone(),
            bytes: disk.bytes,
            removal: disk.removal,
            serial: host.serial(&disk.identifier).ok().flatten(),
        }
    }

    pub(super) fn verify(&self, host: &dyn Host) -> Result<()> {
        let disk = host
            .disks()?
            .into_iter()
            .find(|d| d.identifier == self.id)
            .context("the chosen stick was disconnected; run stela again")?;
        ensure!(
            disk.erasable() && *self == Self::from_disk(&disk, host),
            "the chosen stick changed; run stela again and choose it again"
        );
        Ok(())
    }

    fn size(&self) -> String {
        host::size_text(self.bytes)
    }

    fn description(&self) -> String {
        format!(
            "{} · {} · {}{}",
            self.name,
            self.size(),
            host::DISK_PREFIX,
            self.id
        )
    }

    /// Why erasing it needs its model or size typed back: a disk other than
    /// a USB stick, or any disk when stela cannot tell which one this
    /// computer runs from (`host::typed_reason`).
    fn why_typed(&self) -> Option<&'static str> {
        host::typed_reason(self.removal, self.bytes)
    }

    /// Its model or size, typed back, saying `why` it is asked.
    fn typed_back(&self, ui: &dyn Ui, why: &str) -> Result<String> {
        ui.warning(&format!(
            "{} ({}, {}) {why}.",
            self.name,
            self.size(),
            self.id,
        ))?;
        let (name, bytes) = (self.name.clone(), self.bytes);
        let expected = format!("Type {} or {}", self.name, self.size());
        ui.input(
            "Type its model or size to erase it",
            &format!("{} or {}", self.name, self.size()),
            "",
            Box::new(move |typed: &str| {
                if host::names_disk(&name, bytes, typed) {
                    Ok(())
                } else {
                    Err(expected.clone())
                }
            }),
        )
    }

    /// The name and size people recognize, for the erase warning.
    fn short(&self) -> String {
        format!("{} ({})", self.name, self.size())
    }
}

#[derive(serde::Deserialize)]
struct LocalImage {
    #[serde(rename = "ProductVersion")]
    product_version: String,
    #[serde(rename = "ProductBuildVersion")]
    product_build: String,
}

fn local_image(path: &Path) -> Result<LocalImage> {
    // Only the version is read here; preparation checks the image before
    // anything is erased.
    let bytes = restore::archive::read_member(path, "BuildManifest.plist", 64 * 1024 * 1024)?;
    Ok(plist::from_reader(std::io::Cursor::new(bytes))?)
}

/// The macOS image the installer is made from, and what making it
/// downloads.
struct Image {
    version: String,
    /// It holds all of macOS, not only its startup files.
    whole: bool,
    /// Its bytes and its boot files' bytes, when known.
    bytes: Option<u64>,
    boot_bytes: Option<u64>,
    /// An image of the build this computer holds already.
    local: Option<PathBuf>,
}

/// The image `command` names: its `--ipsw`, or the build to download.
fn image(environment: &Environment<'_>, command: &mut Request) -> Result<Image> {
    if let Some(path) = command.ipsw.clone() {
        let manifest = local_image(&path).with_context(|| format!("reading {}", path.display()))?;
        stela_formats::builds::find(&manifest.product_build)
            .with_context(|| path.display().to_string())?;
        command.fetch = false;
        command.build = manifest.product_build.clone();
        return Ok(Image {
            version: manifest.product_version,
            whole: restore::payload::holds_system_image(&path)?,
            bytes: std::fs::metadata(&path).ok().map(|meta| meta.len()),
            boot_bytes: None,
            local: Some(path),
        });
    }
    command.fetch = true;
    let version = environment
        .builds
        .iter()
        .find(|entry| entry.build == command.build)
        .map_or_else(|| command.build.clone(), |entry| entry.version.into());
    let sizes = (environment.image_size)(&command.build, command.model()).ok();
    Ok(Image {
        version,
        whole: true,
        bytes: sizes.map(|(_, whole)| whole),
        boot_bytes: sizes.map(|(boot, _)| boot),
        local: None,
    })
}

impl Image {
    /// What making `kind` downloads now, and its Review line.
    fn download(
        &self,
        environment: &Environment<'_>,
        command: &Request,
        kind: InstallImage,
    ) -> (u64, String) {
        if self.local.is_some() {
            return (0, "none (uses the IPSW named)".into());
        }
        let need = match kind {
            InstallImage::Full => sources::Need::WholeImage,
            InstallImage::Network => sources::Need::BootFiles,
        };
        if sources::cached_image(
            environment,
            need,
            &command.build,
            command.model(),
            command.download_dir.as_deref(),
        )
        .is_some()
        {
            return (
                0,
                format!(
                    "none (uses the macOS {} image already on this computer)",
                    self.version
                ),
            );
        }
        match (kind, self.bytes, self.boot_bytes) {
            (InstallImage::Full, Some(bytes), _) | (InstallImage::Network, _, Some(bytes)) => {
                (bytes, host::size_text(bytes))
            }
            _ => (0, "the size is unknown until Apple's server answers".into()),
        }
    }
}

/// Ask how to make the installer on `stick`, which a yes erases: `false`
/// is a no, and nothing was changed.
pub(super) fn configure(
    environment: &Environment<'_>,
    command: &mut Request,
    stick: &Stick,
) -> Result<bool> {
    let ui = environment.ui;
    ensure!(
        ui.interactive(),
        "making an installer step by step needs a terminal; scripts run `stela create STICK \
         --confirm-disk SIZE`"
    );
    match ask(environment, command, stick) {
        Err(error) if ui::cancelled(&error) => {
            ui.cancel("Nothing was changed.")?;
            Ok(false)
        }
        result => result,
    }
}

/// The parts Review can change.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Part {
    Installer,
    Macos,
    Wifi,
    Addresses,
}

fn ask(environment: &Environment<'_>, command: &mut Request, stick: &Stick) -> Result<bool> {
    let ui = environment.ui;
    let host = environment.host;
    let usb = Usb::from_disk(&stick.disk, host);
    command.usb = PathBuf::from(&usb.id);
    command
        .model
        .get_or_insert_with(|| super::PINNED_MODEL.into());
    let model_name = environment
        .models
        .iter()
        .find(|(model, _)| *model == command.model())
        .map_or(command.model(), |(_, name)| *name)
        .to_owned();
    let mut image = image(environment, command)?;
    let mut parts = vec![Part::Installer];
    loop {
        for part in std::mem::take(&mut parts) {
            match part {
                Part::Installer => {
                    let (rows, initial) =
                        installer_rows(usb.bytes, image.whole, image.bytes, command.install_image);
                    let kind = INSTALLERS[ui.select("Installer", &rows, initial)?];
                    command.install_image = Some(kind);
                    // Only the Online installer's install needs a network.
                    if kind == InstallImage::Network && command.wifi.is_none() {
                        parts.push(Part::Wifi);
                    }
                }
                Part::Macos => {
                    choose_macos(environment, command)?;
                    image = self::image(environment, command)?;
                }
                Part::Wifi => choose_wifi(environment, command)?,
                Part::Addresses => super::addresses::choose(ui, command)?,
            }
        }
        if !parts.is_empty() {
            continue;
        }
        let online = command.install_image == Some(InstallImage::Network);
        if !online {
            (command.wifi, command.wifi_passphrase) = (None, None);
            (command.ipv4, command.ipv4_gateway, command.dns) = (None, None, None);
        }
        let kind = command.install_image.unwrap_or(InstallImage::Network);
        let (download, download_text) = image.download(environment, command, kind);
        let network = match online {
            false => String::new(),
            true => format!(
                "Network    {}, {}\n",
                match &command.wifi {
                    Some(ssid) => format!(
                        "Wi-Fi {ssid}{}",
                        wifi::rules(
                            command
                                .wifi_country
                                .clone()
                                .or_else(environment.wifi_country)
                                .as_deref()
                        )
                    ),
                    None => "Ethernet".into(),
                },
                super::addresses::summary(command)
            ),
        };
        ui.note(
            "Review",
            &format!(
                "Stick      {}\nInstaller  {}\nmacOS      {} ({})\nMac        {model_name}\n\
                 {network}Download   {download_text}\n{}",
                usb.serial.as_ref().map_or_else(
                    || usb.description(),
                    |serial| format!("{} · serial {serial}", usb.description())
                ),
                match online {
                    false => InstallImage::Full.name(),
                    true => InstallImage::Network.name(),
                },
                image.version,
                command.build,
                // Only a scripted --profile turns it on.
                if command.profile {
                    "Profiling  On\n"
                } else {
                    ""
                },
            ),
        )?;
        for note in firmware::host_notes(host)? {
            ui.info(&note)?;
        }
        ui.info(super::create::SIGNING_SUMMARY)?;
        if let Some(record) = &stick.record {
            // The command its state calls for, if any: a stick that needs
            // nothing is kept by not erasing it.
            let keep = match stick.next(true) {
                Some(command) => format!("choose Quit and run `{command}`"),
                None => "choose Quit".into(),
            };
            ui.warning(&format!(
                "This stick holds Stela with macOS {}: a new installer replaces it. To keep it, \
                 {keep}.",
                super::home::macos_name(&record.build)
            ))?;
        }
        match &stick.state {
            super::home::State::Unreadable(reason) => ui.warning(&format!(
                "This stick may hold Stela, but stela cannot read it: {} A new installer \
                 replaces whatever it holds. To keep it, choose Quit.",
                super::home::sentence(reason)
            ))?,
            super::home::State::Newer(version) if stick.record.is_none() => {
                ui.warning(&format!(
                    "This stick holds Stela {version}: a new installer replaces it. To keep it, \
                 choose Quit."
                ))?
            }
            _ => {}
        }
        let contents = stick.disk.contents();
        ui.warning(&match contents.is_empty() {
            true => format!("Everything on {} will be erased.", usb.short()),
            false => format!("Everything on {} will be erased: {contents}.", usb.short()),
        })?;
        let create = match download {
            0 => "Erase and create".to_owned(),
            bytes => format!("Erase and create (downloads {})", host::size_text(bytes)),
        };
        let rows = ui::rows(&[(&create, ""), ("Change something", ""), ("Quit", "")]);
        // Enter takes Quit: no row that erases or downloads is the default.
        match ui.select("Create the installer?", &rows, QUIT)? {
            0 => {
                if let Some(why) = usb.why_typed() {
                    command.confirm_disk = Some(usb.typed_back(ui, why)?);
                }
                usb.verify(host)?;
                command.confirmed = true;
                return Ok(true);
            }
            1 => parts.push(change(environment, command, online)?),
            _ => {
                ui.cancel("Nothing was changed.")?;
                return Ok(false);
            }
        }
    }
}

/// The value of Quit in "Create the installer?".
const QUIT: usize = 2;

/// Which part of the installer to change.
fn change(environment: &Environment<'_>, command: &Request, online: bool) -> Result<Part> {
    let mut rows = vec![(Part::Installer, "Installer")];
    // A local IPSW fixes the version; otherwise each tested build is one.
    if command.ipsw.is_none() && environment.builds.len() > 1 {
        rows.push((Part::Macos, "macOS version"));
    }
    if online {
        rows.push((Part::Wifi, "Wi-Fi"));
        rows.push((Part::Addresses, "Network addresses"));
    }
    let items: Vec<_> = rows.iter().map(|(_, label)| (*label, "")).collect();
    let index = environment
        .ui
        .select("Change what?", &ui::rows(&items), 0)?;
    Ok(rows[index].0)
}

/// The macOS version: each build Stela was tested with.
fn choose_macos(environment: &Environment<'_>, command: &mut Request) -> Result<()> {
    let builds: Vec<_> = environment.builds.iter().collect();
    let labels: Vec<String> = builds
        .iter()
        .map(|entry| format!("macOS {} ({})", entry.version, entry.build))
        .collect();
    let items: Vec<_> = labels
        .iter()
        .enumerate()
        .map(|(value, label)| Item {
            value,
            label: label.clone(),
            hint: if value == 0 {
                "Newest tested".into()
            } else {
                String::new()
            },
            enabled: true,
        })
        .collect();
    let initial = builds
        .iter()
        .position(|entry| entry.build == command.build)
        .unwrap_or(0);
    let index = environment.ui.select("macOS version", &items, initial)?;
    command.build = builds[index].build.into();
    Ok(())
}

/// The installer step's choices, by their menu values.
const INSTALLERS: [InstallImage; 2] = [InstallImage::Full, InstallImage::Network];

/// The installer step on a stick of `usb_bytes`, for an image of
/// `image_bytes` that holds all of macOS (`whole`) or only its startup
/// files: Offline, shown unavailable with its reason where it cannot be
/// made, and Online; and the one preselected, `chosen` before or else
/// Offline where it can be made.
fn installer_rows(
    usb_bytes: u64,
    whole: bool,
    image_bytes: Option<u64>,
    chosen: Option<InstallImage>,
) -> (Vec<Item>, usize) {
    let unavailable = super::offline_unavailable(usb_bytes, whole);
    let offline = match &unavailable {
        Some(reason) => {
            let mut hint = reason.clone();
            hint[..1].make_ascii_uppercase();
            hint
        }
        None => format!(
            "macOS is copied to the stick now{}. The install needs no internet.",
            image_bytes.map_or_else(String::new, |bytes| format!(
                " (about {:.0} GB)",
                bytes as f64 / 1e9
            ))
        ),
    };
    let rows = vec![
        Item {
            value: 0,
            label: InstallImage::Full.name().into(),
            hint: offline,
            enabled: unavailable.is_none(),
        },
        Item {
            value: 1,
            label: InstallImage::Network.name().into(),
            hint: "Small and quick to make. The install downloads macOS, so the PC needs \
                   internet."
                .into(),
            enabled: true,
        },
    ];
    let initial = match (chosen, unavailable) {
        (Some(InstallImage::Network), _) | (_, Some(_)) => 1,
        _ => 0,
    };
    (rows, initial)
}

/// An Online installer's Wi-Fi: a network this computer sees or knows, one
/// typed in, or none.
pub(super) fn choose_wifi(environment: &Environment<'_>, command: &mut Request) -> Result<()> {
    let ui = environment.ui;
    let networks = (environment.wifi_networks)();
    const ETHERNET: usize = usize::MAX;
    const TYPED: usize = usize::MAX - 1;
    let mut rows = vec![Item {
        value: ETHERNET,
        label: "No Wi-Fi (Ethernet)".into(),
        hint: "Connect the PC to Ethernet during installation".into(),
        enabled: true,
    }];
    for (value, ssid) in networks.iter().enumerate() {
        rows.push(Item {
            value,
            label: ssid.clone(),
            hint: String::new(),
            enabled: true,
        });
    }
    rows.push(Item {
        value: TYPED,
        label: "Another network".into(),
        hint: "Type its name".into(),
        enabled: true,
    });
    let initial = command
        .wifi
        .as_ref()
        .and_then(|ssid| networks.iter().position(|known| known == ssid))
        .unwrap_or(if command.wifi.is_some() {
            TYPED
        } else {
            ETHERNET
        });
    let ssid = match ui.select("Wi-Fi during installation", &rows, initial)? {
        ETHERNET => {
            (command.wifi, command.wifi_passphrase) = (None, None);
            return Ok(());
        }
        TYPED => ui.input(
            "Network name",
            "",
            command.wifi.as_deref().unwrap_or_default(),
            Box::new(|name: &str| wifi::ssid(name).map(|_| ())),
        )?,
        index => networks[index].clone(),
    };
    ui.warning("The passphrase is stored in plaintext on the stick.")?;
    command.wifi_passphrase = Some(wifi::ask_passphrase(ui, &ssid)?);
    command.wifi = Some(ssid);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_manifest_names_its_version() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let path = dir.path().join("local.ipsw");
        crate::efi::synthetic_ipsw(&path)?;
        let manifest = local_image(&path)?;
        assert_eq!(manifest.product_build, restore::payload::SYNTHETIC_BUILD);
        assert!(local_image(&dir.path().join("missing.ipsw")).is_err());
        Ok(())
    }

    fn shown(rows: &[Item]) -> Vec<(String, String, bool)> {
        rows.iter()
            .map(|row| (row.label.clone(), row.hint.clone(), row.enabled))
            .collect()
    }

    const ONLINE: &str =
        "Small and quick to make. The install downloads macOS, so the PC needs internet.";

    #[test]
    fn a_large_stick_offers_both_installers_with_offline_first() {
        let (rows, initial) = installer_rows(64_000_000_000, true, Some(17_800_000_000), None);
        assert_eq!(
            shown(&rows),
            [
                (
                    "Offline".into(),
                    "macOS is copied to the stick now (about 18 GB). The install needs no \
                     internet."
                        .into(),
                    true
                ),
                ("Online".into(), ONLINE.into(), true),
            ]
        );
        assert_eq!(INSTALLERS[initial], InstallImage::Full);
        // Change something comes back to Online once it was chosen.
        let (_, initial) = installer_rows(64_000_000_000, true, None, Some(InstallImage::Network));
        assert_eq!(INSTALLERS[initial], InstallImage::Network);
        let (rows, _) = installer_rows(64_000_000_000, true, None, None);
        assert_eq!(
            rows[0].hint,
            "macOS is copied to the stick now. The install needs no internet."
        );
    }

    #[test]
    fn a_small_stick_shows_offline_unavailable_with_its_reason() {
        let (rows, initial) = installer_rows(16_000_000_000, true, Some(17_800_000_000), None);
        assert_eq!(
            shown(&rows),
            [
                (
                    "Offline".into(),
                    "Needs a USB stick of 32 GB or more (this one is 16.0 GB)".into(),
                    false
                ),
                ("Online".into(), ONLINE.into(), true),
            ]
        );
        assert_eq!(INSTALLERS[initial], InstallImage::Network);
        // An earlier Offline choice cannot be preselected there.
        let (_, initial) = installer_rows(16_000_000_000, true, None, Some(InstallImage::Full));
        assert_eq!(INSTALLERS[initial], InstallImage::Network);
    }

    #[test]
    fn a_local_ipsw_without_all_of_macos_shows_offline_unavailable() {
        let (rows, initial) = installer_rows(64_000_000_000, false, Some(1_000_000_000), None);
        assert!(!rows[0].enabled);
        assert_eq!(
            rows[0].hint,
            "Needs an IPSW file with all of macOS; this one has only its startup files"
        );
        assert_eq!(INSTALLERS[initial], InstallImage::Network);
    }
}
