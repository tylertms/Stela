//! The PC's own disk for an installed macOS: the partition whose bytes are
//! the Mac's internal SSD (ANS2 backing, `--storage`, and the disk the
//! installed system's Preboot volume is read from, `--preboot`) and the
//! machine state (`--nor`): its NOR flash, where NVRAM persists, then the
//! SEP's state and the boot log. The machine state is either a partition of
//! its own or, when setup is given only `--target`, the Apple region's last
//! bytes (`stela_runtime::pc::machine_state`): the boot options then name
//! the Apple region as `--nor` too, and `stela.efi` gives the guest only the
//! bytes before them, as a Mac keeps NVRAM and the SEP's storage off its
//! internal SSD. An existing guest disk that spans the whole partition is
//! never converted, since the tail would truncate it.
//!
//! Setup checks them and writes neither, except that `--initialize-nor`
//! writes a fresh NOR image over a NOR area that is neither blank nor a NOR
//! image, and blanks a SEP area that is neither blank nor a SEP roots
//! record. Install macOS's restore writes the Apple region, as on a Mac; a
//! region that holds anything but a guest disk
//! or zeros is used only with `--erase-target`, so no host data is lost by
//! accident. Every later Stela or macOS update keeps both partitions.
use super::ecid::Ecid;
use super::host::{Host, PartitionInfo, bytes_text};
use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use std::path::Path;
use stela_boot_core::options::{self, System};
use stela_loader::{
    boot_selection::{BOOT_COMMAND, Command, ONE_TIME_BOOT_COMMAND, RECOVERY_BOOT_MODE, lookup},
    mount_interface::CHOSEN_DISK,
};
use {
    stela_formats::payload::gpt::DEFAULT_MIN_APPLE_BYTES, stela_runtime::pc::board_flash,
    stela_runtime::pc::boot_log, stela_runtime::pc::machine_state,
    stela_runtime::pc::storage::Target, stela_soc::machine::sep_state,
    stela_soc_data::nor::CAPACITY, stela_soc_data::nor::Image, stela_soc_data::nor::SECTOR_SIZE,
    stela_soc_data::nor::nvram::Partition, stela_soc_data::nor::nvram::Variables,
};

use super::signatures::{self, Found, GPT_HEADER_OFFSETS};

/// The bytes a guest disk's GPT header is read from.
const PROBE_BYTES: usize = 8192;

/// The installed disk, as `stela.cfg` names it.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub(super) struct InstalledDisk {
    pub(super) apple_region: String,
    pub(super) apple_region_serial: String,
    pub(super) apple_region_guid: String,
    pub(super) nor: String,
    pub(super) nor_serial: String,
    pub(super) nor_guid: String,
    /// The machine state's byte offset in the Apple region, when it lives in
    /// the region's tail (`pc::machine_state`); the `nor` fields then name
    /// the Apple region. None for a machine-state partition of its own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(super) machine_state_offset: Option<u64>,
    /// The Apple region is a partition of the Stela stick, and every PC disk
    /// is the Mac's external drive (`--external-disks`), which recoveryOS
    /// partitions, formats and installs macOS onto.
    #[serde(default, skip_serializing_if = "core::ops::Not::not")]
    pub(super) external_disks: bool,
}

/// How messages name the partition `stela.efi`'s Install macOS chooses.
const CHOSEN_REGION: &str = "the PC disk chosen in stela.efi";

impl InstalledDisk {
    /// The partition `stela.efi`'s Install macOS chooses on the PC itself
    /// (`mount_interface::CHOSEN_DISK`), with the machine state in its tail.
    /// Setup never reaches it; the PC's other disks are the Mac's external
    /// drives.
    pub(super) fn chosen() -> Self {
        Self {
            apple_region: CHOSEN_REGION.into(),
            apple_region_serial: CHOSEN_DISK.into(),
            apple_region_guid: String::new(),
            nor: CHOSEN_REGION.into(),
            nor_serial: CHOSEN_DISK.into(),
            nor_guid: String::new(),
            machine_state_offset: None,
            external_disks: true,
        }
    }

    pub(super) fn is_chosen(&self) -> bool {
        self.apple_region_serial == CHOSEN_DISK
    }

    /// The partitions the install keeps, for messages.
    pub(super) fn partitions(&self) -> String {
        if self.is_chosen() {
            return self.apple_region.clone();
        }
        match self.machine_state_offset {
            Some(_) => self.apple_region.clone(),
            None => format!("{} and {}", self.apple_region, self.nor),
        }
    }

    /// Where the machine state is, for messages.
    pub(super) fn state_description(&self) -> String {
        if self.is_chosen() {
            return format!("the tail of {CHOSEN_REGION}");
        }
        match self.machine_state_offset {
            Some(offset) => format!(
                "the tail of {} from byte {offset:#x}, outside the Mac's SSD (serial {}, \
                 partition {})",
                self.apple_region, self.apple_region_serial, self.apple_region_guid
            ),
            None => format!(
                "{} (serial {}, partition {})",
                self.nor, self.nor_serial, self.nor_guid
            ),
        }
    }

    /// The engine options that boot it: macOS from its Preboot volume, or
    /// the disk's own recoveryOS as NVRAM chooses (`boot_selection::route`).
    /// The bundle `stela.efi` builds from the disk carries the machine's
    /// `ecid`.
    pub(super) fn options(&self, ecid: Option<Ecid>) -> Result<System> {
        let region = self.region_target()?;
        Ok(System {
            storage: Some(region.clone()),
            nor: Some(self.nor_target()?),
            preboot: Some(region),
            bundle: true,
            ecid: ecid.map(|ecid| ecid.0),
            // The PC's keyboards and pointers drive the Mac's. An install
            // runs without them (`bundle_options`).
            input: true,
            ..self.external()
        })
    }

    /// The options that boot the stick's bundle `bundle`, such as a restore
    /// bundle, with this disk and its NVRAM.
    pub(super) fn bundle_options(&self, bundle: &str, ecid: Option<Ecid>) -> Result<System> {
        Ok(System {
            storage: Some(self.region_target()?),
            nor: Some(self.nor_target()?),
            bundle: true,
            bundle_directory: Some(bundle.into()),
            ecid: ecid.map(|ecid| ecid.0),
            ..self.external()
        })
    }

    /// A stick setup's Mac sees the PC's disks as external drives and has
    /// the network, so recoveryOS can install macOS onto one of them.
    fn external(&self) -> System {
        System {
            external_disks: self.external_disks,
            guest_network: self.external_disks,
            ..System::default()
        }
    }

    fn region_target(&self) -> Result<Target> {
        self.target(&self.apple_region_serial, &self.apple_region_guid)
    }

    fn nor_target(&self) -> Result<Target> {
        self.target(&self.nor_serial, &self.nor_guid)
    }

    /// The partition `serial`/`guid`, or `@disk` for the chosen disk.
    fn target(&self, serial: &str, guid: &str) -> Result<Target> {
        if self.is_chosen() {
            return Ok(options::chosen_disk());
        }
        let partition = guid
            .parse()
            .map_err(anyhow::Error::msg)
            .with_context(|| format!("partition GUID {guid} of disk {serial}"))?;
        Ok(Target {
            serial: serial.into(),
            partition,
        })
    }
}

/// Which of an installed disk's partitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Role {
    AppleRegion,
    Nor,
}

/// Where a command that changes the PC's own disk works.
const ON_THE_PC: &str = "run stela on the PC itself, for example from a Linux live USB, with \
    that disk connected";

/// The partition identifier that is still `disk`'s partition in `role`:
/// `explicit` when the user names one, else the recorded identifier, else
/// whichever partition of this computer's disks carries the recorded GUID,
/// since disk names change with plug order and reboots. It must carry the
/// recorded GUID, sit on a disk with the recorded serial (when this
/// computer reports one) and be unmounted. Every write setup makes to a
/// partition goes through this, so a renumbered disk or another partition
/// is never written.
///
/// A machine's state is on the PC's own disk, which is rarely connected to
/// the computer that makes the stick: each refusal says where the command
/// works.
pub(super) fn confirm(
    host: &dyn Host,
    disk: &InstalledDisk,
    role: Role,
    explicit: Option<&str>,
) -> Result<String> {
    ensure!(
        !disk.is_chosen(),
        "this install's disk is chosen in stela.efi on the PC, so the stick does not record \
         where it is and stela cannot reach it from any computer"
    );
    let (recorded, serial, guid, what) = match role {
        Role::AppleRegion => (
            &disk.apple_region,
            &disk.apple_region_serial,
            &disk.apple_region_guid,
            "SSD partition",
        ),
        Role::Nor => (
            &disk.nor,
            &disk.nor_serial,
            &disk.nor_guid,
            "machine-state partition",
        ),
    };
    let carries = |identifier: &str| {
        host.partition(identifier)
            .is_ok_and(|info| info.guid.eq_ignore_ascii_case(guid))
    };
    let identifier = match explicit {
        Some(explicit) => explicit.to_owned(),
        None if carries(recorded) => recorded.clone(),
        None => match host.disks().ok().and_then(|disks| {
            disks
                .into_iter()
                .flat_map(|disk| disk.partitions)
                .map(|partition| partition.identifier)
                .find(|identifier| carries(identifier))
        }) {
            Some(found) => found,
            None => bail!(
                "this install's {what} (partition {guid}, on the disk with serial {serial:?}) is \
                 not connected to this computer. It is on the PC's own disk: connect that disk \
                 here, or {ON_THE_PC}"
            ),
        },
    };
    let info = host
        .partition(&identifier)
        .with_context(|| format!("cannot inspect {identifier}"))?;
    ensure!(
        info.guid.eq_ignore_ascii_case(guid),
        "{identifier} is not this install's {what} (partition {guid}). Leave out --nor, and \
         setup finds the partition by its GUID"
    );
    if let Some(found) = host.serial(&info.disk)? {
        ensure!(
            found == *serial,
            "{identifier} is on a disk with serial {found:?}, not this install's {serial:?}"
        );
    }
    ensure!(
        info.mount.is_none(),
        "{identifier} is mounted; this install's {what} is a raw partition Stela owns"
    );
    Ok(identifier)
}

/// Where a machine's state is: a partition, and the byte offset and length
/// of its layout (the NOR image, the SEP's area, the boot log) in it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct StateArea {
    pub(super) identifier: String,
    pub(super) offset: u64,
    pub(super) bytes: u64,
}

impl StateArea {
    pub(super) fn read(&self, host: &dyn Host, at: u64, length: usize) -> Result<Vec<u8>> {
        self.check(at, length)?;
        host.read(&self.identifier, self.offset + at, length)
    }

    pub(super) fn write(&self, host: &dyn Host, at: u64, data: &[u8]) -> Result<()> {
        self.check(at, data.len())?;
        host.write(&self.identifier, self.offset + at, data)
    }

    fn check(&self, at: u64, length: usize) -> Result<()> {
        ensure!(
            at.checked_add(length as u64)
                .is_some_and(|end| end <= self.bytes),
            "{length} bytes at byte {at} are outside {}'s machine state ({})",
            self.identifier,
            bytes_text(self.bytes)
        );
        Ok(())
    }

    /// The area, for messages: the partition, or where in it the tail is.
    pub(super) fn describe(&self) -> String {
        if self.offset == 0 {
            self.identifier.clone()
        } else {
            format!("{} from byte {:#x}", self.identifier, self.offset)
        }
    }
}

/// The confirmed machine state of `disk` (`confirm` with [`Role::Nor`]):
/// its partition, or the Apple region's tail, which must still start where
/// the record says, so a resized partition is never written.
pub(super) fn state_area(
    host: &dyn Host,
    disk: &InstalledDisk,
    explicit: Option<&str>,
) -> Result<StateArea> {
    let identifier = confirm(host, disk, Role::Nor, explicit)?;
    let bytes = host.partition(&identifier)?.bytes;
    let Some(recorded) = disk.machine_state_offset else {
        return Ok(StateArea {
            identifier,
            offset: 0,
            bytes,
        });
    };
    let layout = machine_state::split(bytes).map_err(|e| anyhow::anyhow!("{identifier}: {e}"))?;
    ensure!(
        layout.state_offset() == recorded,
        "{identifier} holds {}, so its machine state would start at byte {}, not at the \
         recorded byte {recorded}: the partition was resized, and stela.efi would no longer \
         find its NVRAM and SEP state. Restore its size, or restore a `state backup` file \
         after setting the install up again.",
        bytes_text(bytes),
        layout.state_offset()
    );
    Ok(StateArea {
        identifier,
        offset: recorded,
        bytes: layout.state_bytes,
    })
}

/// What `stela.efi` will find in the Apple region.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Contents {
    Blank,
    /// A guest disk with a partition table: an installed or installing
    /// system, kept.
    GuestDisk,
    /// Anything else, such as a host file system: what it is.
    Other(&'static str),
}

/// The choices the command line makes for the installed disk.
pub(super) struct Request<'a> {
    pub(super) apple_region: &'a str,
    /// A machine-state partition of its own; without one the machine state
    /// lives in the Apple region's tail.
    pub(super) nor: Option<&'a str>,
    pub(super) serial: Option<&'a str>,
    pub(super) erase_target: bool,
    pub(super) initialize_nor: bool,
    /// The setup USB's whole disk, which holds neither partition, unless
    /// the Apple region is the stick's own (`external_disks`).
    pub(super) usb_disk: Option<&'a str>,
    /// The Apple region is a partition of the Stela stick, and the PC's disks
    /// are the Mac's external drives.
    pub(super) external_disks: bool,
}

/// The checked disk, what setup will write to its machine-state partition
/// from its start, and notes.
#[derive(Debug)]
pub(super) struct Checked {
    pub(super) disk: InstalledDisk,
    pub(super) nor_image: Option<Vec<u8>>,
    /// The SEP area holds an earlier machine's state, which is bound to
    /// that machine's ECID.
    pub(super) sep_provisioned: bool,
    /// The region holds another file system, which [`erase_foreign`]
    /// clears: the partition as found, mounted or not.
    pub(super) foreign: Option<PartitionInfo>,
    pub(super) notes: Vec<String>,
}

/// What the first `bytes` of `partition` hold, the bytes a guest disk may
/// span: a known signature at its documented offset, near the start or
/// the end, is that kind of data (`signatures::probe`); only zeros
/// wherever one can sit are blank. A partition that cannot be read is
/// refused, never taken for blank.
fn contents(host: &dyn Host, partition: &str, bytes: u64) -> Result<Contents> {
    let found = signatures::probe(bytes, &mut |offset, length| {
        host.read(partition, offset, length)
    })
    .with_context(|| format!("cannot tell what {partition} holds"))?;
    Ok(match found {
        Found::Blank => Contents::Blank,
        Found::Gpt => Contents::GuestDisk,
        Found::Known(name) => Contents::Other(name),
        Found::Unknown => Contents::Other("data"),
    })
}

/// The bytes a guest disk's GPT spans from the partition's start: past its
/// last usable block and its backup header. None when the probe holds no
/// GPT header.
fn guest_disk_bytes(head: &[u8]) -> Option<u64> {
    GPT_HEADER_OFFSETS.iter().find_map(|&block| {
        let header = head.get(block as usize..block as usize + 56)?;
        if &header[..signatures::GPT_HEADER.len()] != signatures::GPT_HEADER {
            return None;
        }
        let word = |at: usize| u64::from_le_bytes(header[at..at + 8].try_into().unwrap());
        let (backup, last_usable) = (word(32), word(48));
        backup
            .max(last_usable)
            .checked_add(1)
            .and_then(|blocks| blocks.checked_mul(block))
    })
}

/// `identifier` as a partition Stela may own. A mounted one is refused,
/// unless `erasing` says setup erases it anyway, after unmounting it.
fn usable(
    host: &dyn Host,
    identifier: &str,
    role: &str,
    usb_disk: Option<&str>,
    erasing: bool,
) -> Result<PartitionInfo> {
    let info = host.partition(identifier)?;
    ensure!(
        !info.guid.is_empty() && !info.disk.is_empty(),
        "{identifier} is not a GUID partition table partition"
    );
    ensure!(
        info.content != "EFI",
        "{identifier} is an EFI system partition; the {role} needs its own partition"
    );
    ensure!(
        usb_disk != Some(info.disk.as_str()),
        "{identifier} is on the Stela stick; the {role} belongs on the PC's own disk"
    );
    if erasing {
        return Ok(info);
    }
    if let Some(mount) = &info.mount {
        bail!(
            "{identifier} is mounted at {}: the {role} is a raw partition Stela owns. Move \
             anything you keep elsewhere, then pass --erase-target, which unmounts and erases it.",
            mount.display()
        );
    }
    if let Some(container) = &info.container {
        bail!(
            "{identifier} backs APFS container {container}, whose volumes are mounted: the {role} \
             is a raw partition Stela owns. Move anything you keep elsewhere, then pass \
             --erase-target, which unmounts and erases it; the container of the running system \
             cannot be used."
        );
    }
    Ok(info)
}

fn serial(host: &dyn Host, disk: &str, explicit: Option<&str>) -> Result<String> {
    match (host.serial(disk)?, explicit) {
        (Some(found), Some(given)) if found != given => {
            bail!("{disk} reports serial {found:?}, not {given:?}")
        }
        (Some(found), _) => Ok(found),
        (None, Some(given)) => Ok(given.into()),
        (None, None) => bail!(
            "{disk} reports no serial number, which stela.efi finds it by; pass the serial its \
             NVMe or ATA controller reports with --disk-serial"
        ),
    }
}

/// Check the PC and its installed disk.
pub(super) fn check(host: &dyn Host, request: &Request) -> Result<Checked> {
    let mut notes = Vec::new();
    let cpu = host.cpu()?;
    ensure!(
        cpu.entropy,
        "this PC's CPU has neither RDSEED nor RDRAND, which the kernel's seeds need"
    );
    ensure!(
        request.nor != Some(request.apple_region),
        "--target and --nor name the same partition; leave out --nor to keep the machine state \
         in the target's last {}",
        bytes_text(machine_state::MACHINE_STATE_BYTES)
    );
    let region = usable(
        host,
        request.apple_region,
        "Mac's SSD",
        request.usb_disk.filter(|_| !request.external_disks),
        request.erase_target,
    )?;
    // Without `--nor`, the guest's SSD is the region less its tail.
    let split = match request.nor {
        Some(_) => None,
        None => Some(
            machine_state::split(region.bytes)
                .map_err(|e| anyhow::anyhow!("{}: {e}", region.identifier))?,
        ),
    };
    let ssd_bytes = split.map_or(region.bytes, |split| split.namespace_bytes);
    ensure!(
        ssd_bytes >= DEFAULT_MIN_APPLE_BYTES,
        "{} holds {}{}; macOS needs at least {}. Give it a larger partition.",
        region.identifier,
        // Rounded down, so that a shortfall never reads as the minimum.
        bytes_text(ssd_bytes / 100_000_000 * 100_000_000),
        if split.is_some() {
            " before the machine state in its last bytes"
        } else {
            ""
        },
        bytes_text(DEFAULT_MIN_APPLE_BYTES)
    );
    let found = contents(host, &region.identifier, ssd_bytes)?;
    let foreign = matches!(found, Contents::Other(_)).then(|| region.clone());
    match found {
        Contents::GuestDisk => {
            if let Some(split) = split {
                let head = host.read(&region.identifier, 0, PROBE_BYTES)?;
                let spans = guest_disk_bytes(&head);
                ensure!(
                    spans.is_some_and(|bytes| bytes <= split.namespace_bytes),
                    "{} holds a macOS disk that spans {}, past the {} the Mac's SSD keeps when \
                     the machine state lives in the partition's last bytes. Its install is not \
                     converted: that would truncate the disk and overwrite its backup GPT. Keep \
                     its machine state on a partition of its own with --nor.",
                    region.identifier,
                    spans.map_or_else(|| "an unreadable size".into(), bytes_text),
                    bytes_text(split.namespace_bytes)
                );
            }
            notes.push(format!(
                "{} already holds a macOS disk; it is kept and booted",
                region.identifier
            ))
        }
        Contents::Blank => notes.push(format!("{} is blank", region.identifier)),
        Contents::Other(what) if request.erase_target => notes.push(format!(
            "{} holds {what}; --erase-target erases it",
            region.identifier
        )),
        // A partition of the stick itself: `--target` names the PC's disks
        // only, so `--erase-target` cannot reach it.
        Contents::Other(what) if request.external_disks => bail!(
            "{} on the stick holds {what}, not a Stela macOS disk; installing macOS onto it \
             would erase it. Copy off what you keep, then delete or wipe that partition.",
            region.identifier
        ),
        Contents::Other(what) => bail!(
            "{} holds {what}, not a Stela macOS disk; installing macOS onto it would erase it. \
             Pass --erase-target to use it anyway.",
            region.identifier
        ),
    }
    // The machine state: a partition of its own, or the region's tail.
    let (nor, area, label) = match (request.nor, split) {
        (Some(identifier), _) => {
            let nor = usable(host, identifier, "machine state", request.usb_disk, false)?;
            let area = StateArea {
                identifier: nor.identifier.clone(),
                offset: 0,
                bytes: nor.bytes,
            };
            let label = nor.identifier.clone();
            (nor, area, label)
        }
        (None, Some(split)) => {
            let area = StateArea {
                identifier: region.identifier.clone(),
                offset: split.state_offset(),
                bytes: split.state_bytes,
            };
            let label = format!(
                "{}'s machine state (its last {})",
                region.identifier,
                bytes_text(split.state_bytes)
            );
            notes.push(format!(
                "the machine state takes the last {} of {}; the Mac's SSD is the {} before it",
                bytes_text(split.state_bytes),
                region.identifier,
                bytes_text(split.namespace_bytes)
            ));
            (region.clone(), area, label)
        }
        (None, None) => unreachable!("a split without --nor"),
    };
    let state_bytes = CAPACITY + stela_formats::machine_state::SEP_BYTES as usize;
    ensure!(
        area.bytes >= state_bytes as u64,
        "{} holds {}; the machine state needs at least {} (NVRAM and the SEP's state), and {} \
         to keep the boot log and the board devices' firmware too",
        nor.identifier,
        bytes_text(area.bytes),
        bytes_text(state_bytes as u64),
        bytes_text(board_flash::PARTITION_BYTES)
    );
    notes.push(if area.bytes >= boot_log::PARTITION_BYTES {
        format!("{label} keeps the boot log")
    } else {
        format!(
            "{label} has no room for the boot log ({} of {})",
            bytes_text(area.bytes),
            bytes_text(boot_log::PARTITION_BYTES)
        )
    });
    // The board devices' flash follows the log; stela.efi boots without it
    // and says so (`BOARD FLASH off`).
    if area.bytes < board_flash::PARTITION_BYTES {
        notes.push(format!(
            "{label} has no room for the board devices' flash ({} of {}): boots of the \
             installed disk lack the device firmware a restore bundle carries",
            bytes_text(area.bytes),
            bytes_text(board_flash::PARTITION_BYTES)
        ));
    }
    let bytes = area.read(host, 0, state_bytes)?;
    let (nor_bytes, sep_area) = bytes.split_at(CAPACITY);
    let blank = nor_bytes.iter().all(|&b| b == 0);
    let image = (!blank)
        .then(|| Image::from_bytes(nor_bytes.to_vec()))
        .transpose();
    let nor_foreign = match image {
        _ if blank => {
            notes.push(format!(
                "{label} is blank; stela.efi writes a NOR image at first boot"
            ));
            false
        }
        Ok(Some(image)) if image.current_bank().is_ok() => {
            notes.push(format!("{label} holds a NOR image; its NVRAM is kept"));
            false
        }
        _ if request.initialize_nor => {
            notes.push(format!(
                "{label} is not a NOR image; --initialize-nor overwrites it"
            ));
            true
        }
        _ => bail!(
            "{label} is neither blank nor a NOR image, so stela.efi would refuse it. Pass \
             --initialize-nor to write a fresh NOR image over it."
        ),
    };
    let mut sep_provisioned = false;
    let sep_foreign = match sep_state::blank(
        &sep_area[..stela_formats::machine_state::SEP_ROOTS_BYTES],
    ) {
        Ok(true) => false,
        Ok(false) => {
            sep_provisioned = true;
            notes.push(format!("{label} holds SEP state; kept"));
            false
        }
        Err(_) if request.initialize_nor => {
            notes.push(format!(
                "{label}: SEP area is not blank; --initialize-nor blanks it"
            ));
            true
        }
        Err(error) => bail!(
            "{label}: {error}, so stela.efi would refuse it. Pass --initialize-nor to blank the \
             SEP's state."
        ),
    };
    let nor_image = (nor_foreign || sep_foreign).then(|| {
        let mut plan = if nor_foreign {
            Image::fresh().bytes().to_vec()
        } else {
            nor_bytes.to_vec()
        };
        if sep_foreign {
            plan.resize(state_bytes, 0);
        } else {
            plan.extend_from_slice(sep_area);
        }
        plan
    });
    let same_disk = region.disk == nor.disk;
    ensure!(
        request.serial.is_none() || same_disk,
        "--disk-serial names one disk, but --target and --nor are on {} and {}",
        region.disk,
        nor.disk
    );
    let disk = InstalledDisk {
        apple_region_serial: serial(host, &region.disk, request.serial)?,
        apple_region_guid: region.guid.to_ascii_uppercase(),
        apple_region: region.identifier,
        nor_serial: serial(host, &nor.disk, request.serial)?,
        nor_guid: nor.guid.to_ascii_uppercase(),
        nor: nor.identifier,
        machine_state_offset: split.map(|split| split.state_offset()),
        external_disks: request.external_disks,
    };
    Ok(Checked {
        disk,
        nor_image,
        sep_provisioned,
        foreign,
        notes,
    })
}

/// The Apple region of `disk`, when it holds another file system than a
/// guest disk: the PC's own OS mounts it and clears its free space, Stela's
/// machine state included. A disk Install macOS chooses in `stela.efi` has
/// none on this computer.
pub(super) fn foreign(host: &dyn Host, disk: &InstalledDisk) -> Result<Option<PartitionInfo>> {
    if disk.is_chosen() {
        return Ok(None);
    }
    let identifier = confirm(host, disk, Role::AppleRegion, None)?;
    let info = host.partition(&identifier)?;
    // A machine state in the region's tail is Stela's, not the guest's.
    let guest_bytes = disk.machine_state_offset.unwrap_or(info.bytes);
    Ok(match contents(host, &identifier, guest_bytes)? {
        Contents::Other(_) => Some(info),
        Contents::Blank | Contents::GuestDisk => None,
    })
}

/// Clear the other file system of `region`, unmounting it first, which
/// `--erase-target` gave up: zeros over every byte a signature can sit in
/// (`signatures::HEAD_BYTES` and `signatures::TAIL_BYTES`) of the bytes the
/// guest disk spans, up to the machine state at `state_offset` when it
/// lives in the region's tail.
pub(super) fn erase_foreign(
    host: &dyn Host,
    region: &PartitionInfo,
    state_offset: Option<u64>,
) -> Result<()> {
    if region.mount.is_some() || region.container.is_some() {
        host.unmount(region)?;
    }
    let guest = state_offset.unwrap_or(region.bytes);
    let head = guest.min(signatures::HEAD_BYTES);
    let tail = guest.saturating_sub(signatures::TAIL_BYTES).max(head);
    for (offset, end) in [(0, head), (tail, guest)] {
        host.write(
            &region.identifier,
            offset,
            &vec![0; usize::try_from(end - offset)?],
        )
        .with_context(|| format!("erasing the file system on {}", region.identifier))?;
    }
    Ok(())
}

/// Write the machine-state bytes `check` planned: a fresh NOR image, a
/// blank SEP area, or both, at the start of the machine state.
pub(super) fn initialize_nor(host: &dyn Host, disk: &InstalledDisk, image: &[u8]) -> Result<()> {
    let area = state_area(host, disk, None)?;
    area.write(host, 0, image)
        .with_context(|| format!("writing a fresh NOR image to {}", area.describe()))
}

/// Whether `stela.efi` has never booted the machine whose state `area`
/// holds: its boot log's header is all zeros (`stela.efi`'s loader writes
/// it at the start of every boot, before the guest and its SEP run), and
/// its NVRAM holds no variables. A damaged header, as a flush cut short by a
/// power loss leaves it, is no proof: the machine may have booted. None when
/// the area is too small for a log, which leaves it unknown.
pub(super) fn never_booted(host: &dyn Host, area: &StateArea) -> Result<Option<bool>> {
    if area.bytes < boot_log::PARTITION_BYTES {
        return Ok(None);
    }
    let region = area.read(host, boot_log::OFFSET, boot_log::LOG_BYTES as usize)?;
    if boot_log::read(&region) != Err(boot_log::HeaderError::Blank) {
        return Ok(Some(false));
    }
    let nor = area.read(host, 0, CAPACITY)?;
    let variables = Image::from_bytes(nor).and_then(|image| {
        let bank = image.current_bank()?;
        Variables::parse(image.bank(bank)?)
    });
    Ok(Some(match variables {
        Ok(variables) => [Partition::System, Partition::Common]
            .into_iter()
            .all(|partition| variables.list(partition).next().is_none()),
        Err(_) => true,
    }))
}

/// What to do about an install whose SEP area may hold its keys but cannot
/// be read: restore a backup, or give its disk a new machine.
pub(super) fn lost_keys(install: &super::machines::Install, usb: &Path, several: bool) -> String {
    let fix = |words: &str| super::fix(usb, words);
    let name = super::quoted(&install.name);
    let which = if several {
        format!(" --install {name}")
    } else {
        String::new()
    };
    let disk = &install.disk;
    let target = match disk.machine_state_offset {
        Some(_) => format!("--target {}", disk.apple_region),
        None => format!("--target {} --nor {}", disk.apple_region, disk.nor),
    };
    let start_over = if several {
        format!(
            "remove the install with {}, then add the disk again with {}",
            fix(&format!("settings --remove-install {name}")),
            fix(&format!("create {target} --initialize-nor"))
        )
    } else {
        format!(
            "make the stick again with {}, which erases it",
            fix(&format!("create {target} --initialize-nor"))
        )
    };
    format!(
        "The machine may have booted, so it may hold its keys. If you have a backup, restore it \
         with {}. Without one, its keys are lost: {start_over}",
        fix(&format!("state restore FILE{which}"))
    )
}

/// Machine state that `stela.efi` would refuse, as an interrupted
/// `--target` can leave it, and its fresh replacement.
pub(super) struct Reinitialize {
    pub(super) area: StateArea,
    /// The NOR image and SEP area to write from the area's start.
    pub(super) bytes: Vec<u8>,
    pub(super) changes: Vec<String>,
    /// Why it cannot be replaced: a foreign SEP area of a machine that may
    /// have booted can hold its keys.
    pub(super) refusal: Option<String>,
}

/// What `repair --initialize-nor` writes to `disk`'s machine state: a fresh
/// NOR image over one that is neither blank nor a NOR image, and a blank SEP
/// area over a foreign one when the machine never booted. A foreign SEP area
/// of a machine that may have booted can hold its keys: its `refusal` names
/// the way to put it right, `lost_keys` ([`lost_keys`]). None when nothing
/// is foreign.
pub(super) fn reinitialize(
    host: &dyn Host,
    disk: &InstalledDisk,
    lost_keys: &str,
) -> Result<Option<Reinitialize>> {
    if disk.is_chosen() {
        return Ok(None);
    }
    let area = state_area(host, disk, None)?;
    let state_bytes = CAPACITY + stela_formats::machine_state::SEP_BYTES as usize;
    let mut bytes = area.read(host, 0, state_bytes)?;
    let (nor_bytes, sep_area) = bytes.split_at_mut(CAPACITY);
    let mut changes = Vec::new();
    let mut refusal = None;
    let nor_foreign = !nor_bytes.iter().all(|&b| b == 0)
        && Image::from_bytes(nor_bytes.to_vec())
            .and_then(|image| image.current_bank())
            .is_err();
    if nor_foreign {
        nor_bytes.copy_from_slice(Image::fresh().bytes());
        changes.push(format!(
            "{}: a fresh NOR image over one stela.efi would refuse",
            area.describe()
        ));
    }
    if let Err(error) = sep_state::blank(&sep_area[..stela_formats::machine_state::SEP_ROOTS_BYTES])
    {
        if never_booted(host, &area)? == Some(true) {
            sep_area.fill(0);
            changes.push(format!(
                "{}: a blank SEP area; the machine never booted",
                area.describe()
            ));
        } else {
            refusal = Some(format!(
                "{}'s SEP area: {error}. {lost_keys}",
                area.describe()
            ));
        }
    }
    Ok(
        (!changes.is_empty() || refusal.is_some()).then_some(Reinitialize {
            area,
            bytes,
            changes,
            refusal,
        }),
    )
}

/// The NVRAM sectors `repair` writes to an install's machine state so the
/// automatic Metal plugin repair runs again at its next start
/// (`bundle_repair::again`).
pub(super) struct BundleRepairAgain {
    pub(super) area: StateArea,
    /// Each changed sector's offset in the NOR image and its new bytes.
    pub(super) sectors: Vec<(u64, Vec<u8>)>,
}

impl BundleRepairAgain {
    pub(super) fn write(&self, host: &dyn Host) -> Result<()> {
        for (offset, bytes) in &self.sectors {
            self.area.write(host, *offset, bytes)?;
            ensure!(
                self.area.read(host, *offset, bytes.len())? == *bytes,
                "{} did not read back as written",
                self.area.describe()
            );
        }
        Ok(())
    }
}

/// What lets `disk`'s automatic Metal plugin repair run again: an attempt
/// that did not install the plugin becomes a request for the next macOS
/// start, as a boot that finds the plugin missing makes. None when its
/// NVRAM holds no finished attempt, or is blank or unreadable, which
/// status and `--initialize-nor` handle.
pub(super) fn bundle_repair_again(
    host: &dyn Host,
    disk: &InstalledDisk,
) -> Result<Option<BundleRepairAgain>> {
    if disk.is_chosen() {
        return Ok(None);
    }
    let area = state_area(host, disk, None)?;
    let Ok(mut image) = Image::from_bytes(area.read(host, 0, CAPACITY)?) else {
        return Ok(None);
    };
    let Ok(mut variables) = image
        .current_bank()
        .and_then(|bank| Variables::parse(image.bank(bank)?))
    else {
        return Ok(None);
    };
    if !stela_boot_core::bundle_repair::again(&mut variables).map_err(anyhow::Error::msg)? {
        return Ok(None);
    }
    let bank = variables.encode().map_err(anyhow::Error::msg)?;
    let (_, sectors) = image.save_bank(&bank).map_err(anyhow::Error::msg)?;
    let sectors = sectors
        .into_iter()
        .map(|offset| {
            (
                offset as u64,
                image.bytes()[offset..offset + SECTOR_SIZE].to_vec(),
            )
        })
        .collect();
    Ok(Some(BundleRepairAgain { area, sectors }))
}

/// What the next boot is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub(crate) enum NextBoot {
    /// System Recovery: the disk's RecoveryOSContainer.
    Recovery,
    /// macOS, clearing a pending recovery request.
    Macos,
}

/// Set the next boot in the NOR's NVRAM as macOS's `nvram` would: a
/// one-time `recover-system` command, or none (also clearing a persistent
/// recovery command and `recovery-boot-mode`). Returns what changed;
/// nothing is written with `dry_run`.
pub(super) fn next_boot(
    host: &dyn Host,
    disk: &InstalledDisk,
    next: NextBoot,
    dry_run: bool,
) -> Result<Vec<String>> {
    let area = state_area(host, disk, None)?;
    let bytes = area.read(host, 0, CAPACITY)?;
    let blank = bytes.iter().all(|&b| b == 0);
    let mut image = if blank {
        Image::fresh()
    } else {
        Image::from_bytes(bytes).map_err(anyhow::Error::msg)?
    };
    let current = image.current_bank().map_err(anyhow::Error::msg)?;
    let mut variables = Variables::parse(image.bank(current).map_err(anyhow::Error::msg)?)
        .map_err(anyhow::Error::msg)?;
    let mut changes = Vec::new();
    match next {
        NextBoot::Recovery => {
            let value = Command::RecoverSystem.name().as_bytes();
            variables
                .set(Partition::System, ONE_TIME_BOOT_COMMAND, value)
                .map_err(anyhow::Error::msg)?;
            changes.push("set one-time-boot-command=recover-system".into());
        }
        NextBoot::Macos => {
            let recovery_command = lookup(&variables, BOOT_COMMAND)
                .and_then(|v| Command::parse(v).ok())
                .is_some_and(|c| c != Command::Fsboot && c != Command::FsbootSafe);
            let mut names = vec![ONE_TIME_BOOT_COMMAND, RECOVERY_BOOT_MODE];
            if recovery_command {
                names.push(BOOT_COMMAND);
            }
            for name in names {
                for partition in [Partition::System, Partition::Common] {
                    if variables.remove(partition, name).is_some() {
                        changes.push(format!("remove {}", String::from_utf8_lossy(name)));
                    }
                }
            }
        }
    }
    if changes.is_empty() || dry_run {
        return Ok(changes);
    }
    let bank = variables.encode().map_err(anyhow::Error::msg)?;
    let (_, sectors) = image.save_bank(&bank).map_err(anyhow::Error::msg)?;
    if blank {
        area.write(host, 0, image.bytes())?;
    } else {
        for offset in sectors {
            area.write(
                host,
                offset as u64,
                &image.bytes()[offset..offset + SECTOR_SIZE],
            )?;
        }
    }
    Ok(changes)
}
