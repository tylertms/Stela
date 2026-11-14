//! The installs of a setup: each a disk of the PC (an Apple region whose
//! tail holds the machine state, or an Apple region and a machine-state
//! partition) that is its own Stela machine with its own ECID,
//! NVRAM and SEP state, as two Macs are. One is the default `stela.cfg`
//! boots; `stela.efi`'s startup choices (`choices.cfg`) list every install
//! and its Install macOS, which restores the USB's install medium onto it
//! with the machine's own restore bundle. They open on their own with
//! several installs or an install medium, and when a key is held otherwise.
//!
//! A machine's ECID is kept for its life: an install whose machine-state
//! partition already holds SEP state takes the ECID given with `--ecid`, and
//! a new one only with `--new-ecid`, since its LocalPolicy and SEP state are
//! bound to the old one. Every install needs its own restore bundle,
//! prepared from the active build's restore image.
use super::{
    Environment, Request, apply_install, bundle, check_bundle_revision, check_installed_efi,
    describe_install, ecid::Ecid, installed, plan_install, publish, read_record, sources,
    write_record,
};
use anyhow::{Result, bail, ensure};
use serde::{Deserialize, Serialize};
use std::{fmt::Write as _, fs};
use stela_loader::mount_interface::MAX_CHOICES;

/// The first install's name unless `--install` gives one.
pub(super) const FIRST_NAME: &str = "macOS";
/// Each install has up to three startup choices: itself, Install macOS and
/// Update macOS.
const MAX_INSTALLS: usize = MAX_CHOICES / 3;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub(super) struct Install {
    pub(super) name: String,
    /// None only for an install a schema 3 setup made.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) ecid: Option<Ecid>,
    #[serde(flatten)]
    pub(super) disk: installed::InstalledDisk,
}

/// A name the startup choices can show, with room for the longest suffix
/// of its choices, " - Install macOS".
pub(super) fn check_name(name: &str) -> Result<()> {
    const SUFFIX: usize = " - Install macOS".len();
    ensure!(
        !name.is_empty()
            && name.len() + SUFFIX <= stela_loader::mount_interface::MAX_NAME
            && name.bytes().all(|b| b.is_ascii_graphic() || b == b' ')
            && name.trim() == name,
        "an install name is 1 to {} printable ASCII characters",
        stela_loader::mount_interface::MAX_NAME - SUFFIX
    );
    Ok(())
}

/// Requested and recorded ECIDs with machine-state replacement conditions.
pub(super) struct EcidOptions {
    pub(super) given: Option<Ecid>,
    pub(super) recorded: Option<Ecid>,
    pub(super) sep_provisioned: bool,
    pub(super) new_ecid: bool,
}

/// The ECID of a machine being set up: the one `given`, else the one
/// `recorded`, else a fresh one, but never a new one for machine state that
/// belongs to an earlier machine unless `new_ecid` says so.
pub(super) fn machine_ecid(options: EcidOptions, notes: &mut Vec<String>) -> Result<Ecid> {
    let EcidOptions {
        given,
        recorded,
        sep_provisioned,
        new_ecid,
    } = options;
    match (given, recorded) {
        (Some(given), Some(recorded)) if given != recorded && !new_ecid => bail!(
            "this machine's ECID is {recorded}, not {given}; changing it orphans its LocalPolicy \
             and SEP state. Pass --new-ecid to change it anyway."
        ),
        (Some(given), _) => Ok(given),
        (None, Some(recorded)) => Ok(recorded),
        (None, None) if sep_provisioned && !new_ecid => bail!(
            "this disk's machine state holds an earlier Stela machine's SEP state, which is \
             bound to that machine's ECID. Pass that ECID with --ecid (`stela status` of \
             its old stick, or its `state backup` file, shows it), or --new-ecid to start a new \
             machine, which orphans its LocalPolicy and SEP state."
        ),
        (None, None) => {
            if sep_provisioned {
                notes.push("--new-ecid: the old SEP state and LocalPolicy are orphaned".into());
            }
            Ecid::generate()
        }
    }
}

/// `--target` (and an optional `--nor`) on an existing setup: add an
/// install.
pub(super) fn add(environment: &Environment<'_>, command: &Request) -> Result<String> {
    let record = read_record(environment, &command.usb)?;
    check_installed_efi(environment, &record)?;
    check_bundle_revision(environment, &record)?;
    super::update::ready(&record, &command.usb)?;
    if let Some(model) = &command.model {
        ensure!(
            *model == record.model,
            "this setup presents {}; every install of a setup presents the same model, so set \
             up {model} on another USB",
            record.model
        );
    }
    ensure!(
        record.installs.len() < MAX_INSTALLS,
        "the startup choices list at most {MAX_INSTALLS} installs"
    );
    let host = environment.host;
    let Some(target) = &command.target else {
        bail!("an install needs --target");
    };
    // An install's Apple region is never converted to keep its machine
    // state in its own tail, before anything of it is read.
    if command.nor.is_none() {
        let guid = host.partition(target)?.guid;
        if let Some(install) = record.installs.iter().find(|install| {
            install.disk.machine_state_offset.is_none()
                && install.disk.apple_region_guid.eq_ignore_ascii_case(&guid)
        }) {
            bail!(
                "{target} is install \"{}\"'s Apple region, whose machine state is on {}; it is \
                 not converted to keep the machine state in its own last bytes, which would \
                 truncate its guest disk and overwrite its backup GPT",
                install.name,
                install.disk.nor
            );
        }
    }
    let usb_disk = host.volume(&record.esp)?.disk;
    let checked = installed::check(
        host,
        &installed::Request {
            apple_region: target,
            nor: command.nor.as_deref(),
            serial: command.disk_serial.as_deref(),
            erase_target: command.erase_target,
            initialize_nor: command.initialize_nor,
            usb_disk: usb_disk.as_deref(),
            external_disks: false,
        },
    )?;
    // An add that was interrupted after the record named its install is
    // finished by the same command: the install keeps its name and ECID.
    let mut unfinished = None;
    for install in &record.installs {
        let guids = [&install.disk.apple_region_guid, &install.disk.nor_guid];
        if !guids.contains(&&checked.disk.apple_region_guid)
            && !guids.contains(&&checked.disk.nor_guid)
        {
            continue;
        }
        let same = install
            .disk
            .apple_region_guid
            .eq_ignore_ascii_case(&checked.disk.apple_region_guid)
            && install
                .disk
                .nor_guid
                .eq_ignore_ascii_case(&checked.disk.nor_guid)
            && command
                .install
                .as_ref()
                .is_none_or(|name| *name == install.name);
        let missing = record
            .missing_restore_bundles()
            .iter()
            .any(|&(_, ecid)| ecid == install.ecid)
            || checked.nor_image.is_some()
            || checked.foreign.is_some();
        ensure!(
            same && missing,
            "{} already belongs to install \"{}\"",
            checked.disk.partitions(),
            install.name
        );
        unfinished = Some(install.clone());
    }
    let mut notes = checked.notes.clone();
    let mut next = record.clone();
    let (name, ecid) = match unfinished {
        Some(install) => {
            let ecid = machine_ecid(
                EcidOptions {
                    given: command.ecid,
                    recorded: install.ecid,
                    sep_provisioned: checked.sep_provisioned,
                    new_ecid: command.new_ecid,
                },
                &mut notes,
            )?;
            notes.push(format!(
                "finishes adding install \"{}\", which was interrupted",
                install.name
            ));
            if let Some(kept) = next.installs.iter_mut().find(|i| i.name == install.name) {
                kept.ecid = Some(ecid);
                kept.disk = checked.disk.clone();
            }
            (install.name, ecid)
        }
        None => {
            let name = match &command.install {
                Some(name) => name.clone(),
                None if record.installs.is_empty() => FIRST_NAME.into(),
                None => (2..)
                    .map(|n| format!("{FIRST_NAME} {n}"))
                    .find(|name| record.installs.iter().all(|i| &i.name != name))
                    .expect("a free name"),
            };
            check_name(&name)?;
            ensure!(
                record.installs.iter().all(|i| i.name != name),
                "this setup already has an install named {name:?}"
            );
            let ecid = machine_ecid(
                EcidOptions {
                    given: command.ecid,
                    recorded: None,
                    sep_provisioned: checked.sep_provisioned,
                    new_ecid: command.new_ecid,
                },
                &mut notes,
            )?;
            ensure!(
                record.installs.iter().all(|i| i.ecid != Some(ecid)),
                "ECID {ecid} already belongs to another install of this setup"
            );
            next.installs.push(Install {
                name: name.clone(),
                ecid: Some(ecid),
                disk: checked.disk.clone(),
            });
            (name, ecid)
        }
    };
    next.verify_on_read = command.verify_on_read;
    next.install_image = command.install_image.or(next.install_image);
    let stale = next.drop_stale_bundles();
    // Every install restores from the medium with its own restore bundle.
    let need = sources::need(&next, &next.build, true).unwrap_or(sources::Need::BootFiles);
    let active = next.active_medium().cloned();
    let lookup = sources::Lookup {
        need,
        build: &record.build,
        model: &record.model,
        ipsw: command.ipsw.as_deref(),
        download_dir: command.download_dir.as_deref(),
        medium: active.as_ref(),
    };
    let source = sources::locate(environment, &lookup).map_err(|error| {
        anyhow::anyhow!(
            "install \"{name}\" (ECID {ecid}) needs a restore bundle of its own, prepared from \
             the macOS {} restore image; but {error:#}",
            record.build
        )
    })?;
    if need.whole() {
        sources::check_room(environment, &next, &next.build, &source)?;
    }
    let restore: Vec<(bundle::Kind, Option<Ecid>)> =
        super::restore_bundles(&next.install_machines())
            .into_iter()
            .filter(|&(kind, ecid)| next.restore_bundle(kind, ecid).is_none())
            .collect();
    // A dry run reads and downloads nothing to prepare its plan.
    let install_plan = match command.dry_run && !source.is_local() {
        true => None,
        false => {
            let obtained = sources::obtain(environment, &source, &lookup)?;
            let environment = Environment {
                download: obtained.download.as_ref(),
                ..*environment
            };
            let plan = plan_install(
                &environment,
                &next,
                super::create::InstallOptions {
                    build: &next.build,
                    ipsw: &obtained.path,
                    bundles: &restore,
                    additional_bytes: 0,
                },
            )?;
            Some((plan, obtained))
        }
    };
    let mut text = String::new();
    writeln!(text, "Add install \"{name}\" to {}:", record.esp.display())?;
    writeln!(text, "  ECID           {ecid}")?;
    let disk = &checked.disk;
    writeln!(
        text,
        "  Mac's SSD      {} (serial {}, partition {})",
        disk.apple_region, disk.apple_region_serial, disk.apple_region_guid
    )?;
    writeln!(text, "  {}", super::state_line(disk))?;
    writeln!(
        text,
        "  restore image  {}",
        sources::describe(&source, &lookup)
    )?;
    match &install_plan {
        Some((plan, _)) => describe_install(&next, plan, &mut text)?,
        None => writeln!(
            text,
            "  install        its restore bundle, prepared from that image"
        )?,
    }
    writeln!(
        text,
        "  choices.cfg    adds \"{name}\" and its Install macOS"
    )?;
    if checked.nor_image.is_some() {
        writeln!(
            text,
            "  writes         machine state on {}",
            disk.state_description()
        )?;
    }
    for kept in &stale {
        writeln!(
            text,
            "  remove         {}",
            kept.directory(&record.volume).display()
        )?;
    }
    for note in &notes {
        writeln!(text, "  note           {note}")?;
    }
    if command.dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    write_record(&next.esp.clone(), &next)?;
    let (plan, obtained) = install_plan.expect("a change has its plan");
    let environment = &Environment {
        download: obtained.download.as_ref(),
        ..*environment
    };
    apply_install(environment, &mut next, &obtained.path, plan)?;
    if let Some(region) = &checked.foreign {
        installed::erase_foreign(host, region, checked.disk.machine_state_offset)?;
    }
    if let Some(image) = &checked.nor_image {
        installed::initialize_nor(host, &checked.disk, image)?;
    }
    publish(&mut next)?;
    write_record(&next.esp.clone(), &next)?;
    super::remove_bundles(&next, &stale)?;
    text.push_str("Done.\n");
    Ok(text)
}

/// Settings of an existing setup: the default install, removing one, the
/// engine profile and the startup choices' timeout.
pub(super) fn configure(environment: &Environment<'_>, command: &Request) -> Result<String> {
    let mut record = read_record(environment, &command.usb)?;
    check_installed_efi(environment, &record)?;
    super::update::ready(&record, &command.usb)?;
    let mut text = format!("Stela setup on {}:\n", record.esp.display());
    let mut removed_bundles = Vec::new();
    if let Some(name) = &command.remove_install {
        let install = record.install(Some(name))?.clone();
        ensure!(
            record.installs.len() > 1,
            "install \"{name}\" is this setup's only one; to remove it, {}",
            super::START_OVER
        );
        record.installs.retain(|i| i.name != *name);
        if record.default_install.as_deref() == Some(name.as_str()) {
            record.default_install = None;
        }
        writeln!(
            text,
            "  remove         install \"{name}\" (disk kept{})",
            install
                .ecid
                .map(|ecid| format!(", ECID {ecid}"))
                .unwrap_or_default()
        )?;
        removed_bundles = record.drop_stale_bundles();
        for kept in &removed_bundles {
            writeln!(
                text,
                "  remove         {}",
                kept.directory(&record.volume).display()
            )?;
        }
    }
    if let Some(name) = &command.default_install {
        record.install(Some(name))?;
        record.default_install = Some(name.clone());
        writeln!(text, "  default        install \"{name}\" boots by default")?;
    }
    if command.profile || command.no_profile {
        record.profile = command.profile;
        writeln!(
            text,
            "  profile        {}",
            match record.profile {
                true => "on: each heartbeat of the boot log times the engine's phases",
                false => "off",
            }
        )?;
    }
    match command.chooser_timeout {
        Some(super::command::Wait::Seconds(seconds)) => {
            record.chooser_timeout = Some(seconds);
            writeln!(text, "  choices        wait {seconds} s")?;
        }
        Some(super::command::Wait::Automatic) => {
            record.chooser_timeout = None;
            writeln!(
                text,
                "  choices        wait {} s with several installs or an install medium, else \
                 open only when a key is held",
                super::CHOOSER_SECONDS
            )?;
        }
        None => {}
    }
    if let Some(ssid) = &command.wifi {
        let network = super::wifi::Network {
            ssid: ssid.clone(),
            country: command
                .wifi_country
                .clone()
                .or_else(environment.wifi_country),
        };
        writeln!(text, "  {}", super::wifi::describe(&network, &record.esp))?;
        record.wifi = Some(network);
    }
    if let Some(ipv4) = command.ipv4 {
        record.addresses.ipv4 = Some(super::addresses::Ipv4::new(ipv4, command.ipv4_gateway)?);
    }
    if command.automatic_ipv4 && record.addresses.ipv4.take().is_some() {
        writeln!(text, "  IPv4           from DHCP")?;
    }
    if let Some(dns) = &command.dns {
        record.addresses.dns = dns.clone();
    }
    if command.automatic_dns && !std::mem::take(&mut record.addresses.dns).is_empty() {
        writeln!(text, "  DNS            from DHCP and the router")?;
    }
    if command.ipv4.is_some() || command.dns.is_some() {
        for line in record.addresses.describe() {
            writeln!(text, "  {line}")?;
        }
    }
    if command.no_wifi && record.wifi.take().is_some() {
        writeln!(
            text,
            "  Wi-Fi          off; remove {}",
            record.esp.join(super::wifi::KEY_PATH).display()
        )?;
    }
    if !command.usb4_approve.is_empty() {
        record.usb4_approved = command.usb4_approve.clone();
        writeln!(
            text,
            "  USB4 PCIe      also without an IOMMU for {}",
            record.usb4_approved.join(", ")
        )?;
    }
    if command.no_usb4_approve && !std::mem::take(&mut record.usb4_approved).is_empty() {
        writeln!(text, "  USB4 PCIe      only with an IOMMU")?;
    }
    record.choices_text()?;
    if command.dry_run {
        text.push_str("Dry run: nothing was changed.\n");
        return Ok(text);
    }
    if let Some(passphrase) = &command.wifi_passphrase {
        super::wifi::write_key(&record.esp, passphrase)?;
    }
    publish(&mut record)?;
    write_record(&record.esp.clone(), &record)?;
    super::remove_bundles(&record, &removed_bundles)?;
    // The bundle already holds every Wi-Fi adapter's files.
    let key = record.esp.join(super::wifi::KEY_PATH);
    if command.no_wifi && key.is_file() {
        fs::remove_file(key)?;
    }
    text.push_str("Done.\n");
    Ok(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ecids_are_kept_and_changed_only_when_asked() {
        let mut notes = Vec::new();
        let (a, b) = (Ecid(0xa), Ecid(0xb));
        assert_eq!(
            machine_ecid(
                EcidOptions {
                    given: None,
                    recorded: Some(a),
                    sep_provisioned: true,
                    new_ecid: false
                },
                &mut notes
            )
            .unwrap(),
            a
        );
        assert_eq!(
            machine_ecid(
                EcidOptions {
                    given: Some(a),
                    recorded: Some(a),
                    sep_provisioned: true,
                    new_ecid: false
                },
                &mut notes
            )
            .unwrap(),
            a
        );
        assert!(
            machine_ecid(
                EcidOptions {
                    given: Some(b),
                    recorded: Some(a),
                    sep_provisioned: false,
                    new_ecid: false
                },
                &mut notes
            )
            .is_err()
        );
        assert_eq!(
            machine_ecid(
                EcidOptions {
                    given: Some(b),
                    recorded: Some(a),
                    sep_provisioned: false,
                    new_ecid: true
                },
                &mut notes
            )
            .unwrap(),
            b
        );
        let error = machine_ecid(
            EcidOptions {
                given: None,
                recorded: None,
                sep_provisioned: true,
                new_ecid: false,
            },
            &mut notes,
        )
        .unwrap_err();
        assert!(error.to_string().contains("--ecid"), "{error}");
        assert_eq!(
            machine_ecid(
                EcidOptions {
                    given: Some(b),
                    recorded: None,
                    sep_provisioned: true,
                    new_ecid: false
                },
                &mut notes
            )
            .unwrap(),
            b
        );
        assert!(notes.is_empty());
        machine_ecid(
            EcidOptions {
                given: None,
                recorded: None,
                sep_provisioned: true,
                new_ecid: true,
            },
            &mut notes,
        )
        .unwrap();
        assert!(notes[0].contains("orphaned"));
        assert!(check_name("macOS 2").is_ok());
        assert!(check_name(&"x".repeat(24)).is_ok());
        for bad in ["", " macOS", "a\tb", &"x".repeat(25)] {
            assert!(check_name(bad).is_err(), "{bad:?}");
        }
    }
}
