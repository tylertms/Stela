//! Linux: sysfs gives each block device's disk, size and serial number;
//! `lsblk` lists the partitions with their GPT types, GUIDs and mounts;
//! `udisksctl` mounts and unmounts where it runs, `mount` and `umount`
//! otherwise. Partitions are read and written through `/dev`, and a USB is
//! erased by writing setup's own layout to its whole disk.
use super::{
    Bus, Cpu, Disk, DiskFacts, FileSystem, Host, Partition, PartitionInfo, Removal, Volume,
};
use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Deserializer};
use std::{
    collections::BTreeSet,
    fs,
    path::{Path, PathBuf},
    process::Command,
    sync::Once,
};
use system_disks::{Blocks, SystemDevices};

mod system_disks;

/// How setup gets the rights its raw reads and writes need.
pub(crate) const ADMIN: &str = "sudo";
/// A whole disk as STICK names it.
/// What a disk identifier is preceded by where a user types it.
pub(crate) const DISK_PREFIX: &str = "/dev/";
pub(crate) const DISK_EXAMPLE: &str = "/dev/sdb";

/// The EFI system partition's GPT type, as `lsblk` prints it, and its MBR
/// type.
const ESP_TYPES: [&str; 2] = ["c12a7328-f81f-11d2-ba4b-00a0c93ec93b", "0xef"];
/// `lsblk`'s columns, for [`Device`].
const COLUMNS: &str =
    "NAME,TYPE,SIZE,MODEL,TRAN,RM,RO,MOUNTPOINT,MOUNTPOINTS,PARTTYPE,PARTUUID,PKNAME";
/// The mount points of the running system's volumes, and swap's: with
/// OSTree's physical root, Nix's and Guix's stores, and the media live
/// systems run from (dracut, archiso, casper, live-boot).
const SYSTEM_MOUNTS: [&str; 17] = [
    "/",
    "/sysroot",
    "/boot",
    "/boot/efi",
    "/efi",
    "/usr",
    "/var",
    "/home",
    "/nix",
    "/nix/store",
    "/gnu/store",
    "/run/initramfs/live",
    "/run/archiso/bootmnt",
    "/cdrom",
    "/run/live/medium",
    "/run/rootfsbase",
    "[SWAP]",
];
/// The `BLKRRPART` ioctl: reread the partition table.
const BLKRRPART: u64 = 0x125f;

/// The computer `stela` runs on.
pub(crate) struct System;

#[derive(Deserialize)]
struct Listing {
    blockdevices: Vec<Device>,
}

/// One block device in `lsblk --json --bytes` output.
#[derive(Deserialize)]
struct Device {
    name: String,
    #[serde(rename = "type")]
    kind: String,
    #[serde(default, deserialize_with = "number")]
    size: u64,
    model: Option<String>,
    tran: Option<String>,
    #[serde(default, deserialize_with = "flag")]
    rm: bool,
    #[serde(default, deserialize_with = "flag")]
    ro: bool,
    mountpoint: Option<String>,
    #[serde(default)]
    mountpoints: Option<Vec<Option<String>>>,
    parttype: Option<String>,
    partuuid: Option<String>,
    pkname: Option<String>,
    #[serde(default)]
    children: Vec<Device>,
}

impl Device {
    /// Its mount points: `mountpoints`, or `mountpoint` from an older lsblk.
    fn mounts(&self) -> impl Iterator<Item = &str> {
        self.mountpoint.as_deref().into_iter().chain(
            self.mountpoints
                .iter()
                .flatten()
                .flatten()
                .map(String::as_str),
        )
    }

    /// Whether it, or a device on it, is mounted at `/`.
    fn holds_root(&self) -> bool {
        self.mounts().any(|mount| mount == "/") || self.children.iter().any(Device::holds_root)
    }

    /// Whether it, or a device on it, holds part of the running system: the
    /// root, boot, EFI, `/usr`, `/var` or `/home` volume, or active swap.
    fn holds_system(&self) -> bool {
        self.mounts().any(|mount| SYSTEM_MOUNTS.contains(&mount))
            || self.children.iter().any(Device::holds_system)
    }

    /// Whether it, or a device on it, is one of `names`, by kernel name.
    fn holds_any(&self, names: &BTreeSet<String>) -> bool {
        names.contains(&self.name) || self.children.iter().any(|child| child.holds_any(names))
    }

    /// What decides whether setup may erase this whole disk.
    fn facts(&self) -> DiskFacts {
        DiskFacts {
            bus: match self.tran.as_deref() {
                _ if self.kind == "loop" => Bus::Image,
                Some("usb") => Bus::Usb,
                Some("ieee1394") => Bus::FireWire,
                _ => Bus::Other,
            },
            removable_media: self.rm,
            external: false,
        }
    }

    fn content(&self) -> String {
        match self.parttype.as_deref() {
            Some(kind) if ESP_TYPES.iter().any(|esp| esp.eq_ignore_ascii_case(kind)) => {
                "EFI".into()
            }
            kind => kind.unwrap_or_default().into(),
        }
    }

    fn mount(&self) -> Option<PathBuf> {
        self.mountpoint
            .as_deref()
            .filter(|mount| mount.starts_with('/'))
            .map(PathBuf::from)
    }
}

/// lsblk before util-linux 2.33 prints flags as "0" and "1".
fn flag<'de, D: Deserializer<'de>>(deserializer: D) -> Result<bool, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Flag {
        Bool(bool),
        Text(String),
    }
    Ok(match Option::<Flag>::deserialize(deserializer)? {
        Some(Flag::Bool(value)) => value,
        Some(Flag::Text(text)) => text == "1",
        None => false,
    })
}

/// lsblk before util-linux 2.33 prints sizes as strings.
fn number<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u64, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Number {
        Number(u64),
        Text(String),
    }
    Ok(match Option::<Number>::deserialize(deserializer)? {
        Some(Number::Number(value)) => value,
        Some(Number::Text(text)) => text.parse().map_err(serde::de::Error::custom)?,
        None => 0,
    })
}

/// `lsblk`'s listing of `device`, or of every block device.
fn lsblk(device: Option<&str>) -> Result<Vec<Device>> {
    let legacy = COLUMNS.replace(",MOUNTPOINTS", "");
    let mut failure = String::new();
    // Older util-linux has only the single-mount column. Root detection still
    // has to succeed before either listing can expose erase candidates.
    for columns in [COLUMNS, legacy.as_str()] {
        let mut command = Command::new("lsblk");
        command.args(["--json", "--bytes", "--output", columns]);
        if let Some(device) = device {
            command.arg(format!("/dev/{device}"));
        }
        let output = command.output().context("cannot run lsblk")?;
        if output.status.success() {
            return Ok(serde_json::from_slice::<Listing>(&output.stdout)
                .context("cannot read lsblk's listing")?
                .blockdevices);
        }
        failure = String::from_utf8_lossy(&output.stderr).trim().to_owned();
    }
    anyhow::bail!("lsblk failed: {failure}")
}

/// The block device `name`, which sysfs lists; `partition` says which kind.
fn checked(name: &str, partition: bool) -> Result<()> {
    let sys = Path::new("/sys/class/block").join(name);
    ensure!(
        !name.is_empty()
            && name
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
            && sys.exists()
            && sys.join("partition").exists() == partition,
        "{name} is not a {} on this computer",
        if partition {
            "partition, such as sdb1 or nvme0n1p2,"
        } else {
            "whole disk, such as sdb or nvme0n1,"
        }
    );
    Ok(())
}

fn read_trimmed(path: &Path) -> Option<String> {
    fs::read_to_string(path)
        .ok()
        .map(|text| text.trim().to_owned())
        .filter(|text| !text.is_empty())
}

/// Run `command`, or say what it printed.
fn run(command: &mut Command) -> Result<()> {
    let output = command
        .output()
        .with_context(|| format!("cannot run {:?}", command.get_program()))?;
    ensure!(
        output.status.success(),
        "{:?} failed: {}",
        command.get_program(),
        String::from_utf8_lossy(&output.stderr).trim()
    );
    Ok(())
}

/// sysfs, and the ZFS pool labels `lsblk` reads.
struct Sysfs;

impl Blocks for Sysfs {
    fn by_number(&self, major: u32, minor: u32) -> Option<String> {
        let sys = fs::canonicalize(format!("/sys/dev/block/{major}:{minor}")).ok()?;
        Some(sys.file_name()?.to_str()?.to_owned())
    }

    fn by_node(&self, node: &str) -> Option<String> {
        let node = fs::canonicalize(node).ok()?;
        let name = node.strip_prefix("/dev").ok()?.to_str()?.to_owned();
        Path::new("/sys/class/block")
            .join(&name)
            .exists()
            .then_some(name)
    }

    fn members(&self, name: &str) -> Vec<String> {
        names_in(&Path::new("/sys/class/block").join(name).join("slaves"))
    }

    fn backing_file(&self, name: &str) -> Option<String> {
        read_trimmed(
            &Path::new("/sys/class/block")
                .join(name)
                .join("loop/backing_file"),
        )
    }

    fn btrfs_devices(&self, name: &str) -> Vec<String> {
        names_in(Path::new("/sys/fs/btrfs"))
            .into_iter()
            .map(|uuid| names_in(&Path::new("/sys/fs/btrfs").join(uuid).join("devices")))
            .find(|devices| devices.iter().any(|device| device == name))
            .unwrap_or_default()
    }

    fn zfs_members(&self, pool: &str) -> Vec<String> {
        let Ok(output) = Command::new("lsblk")
            .args(["--json", "--list", "--output", "KNAME,FSTYPE,LABEL"])
            .output()
        else {
            return Vec::new();
        };
        zfs_members(&String::from_utf8_lossy(&output.stdout), pool)
    }
}

/// The names of the entries of `directory`.
fn names_in(directory: &Path) -> Vec<String> {
    fs::read_dir(directory)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| entry.file_name().into_string().ok())
        .collect()
}

/// The devices `lsblk --json --list --output KNAME,FSTYPE,LABEL` labels as
/// members of ZFS pool `pool`.
fn zfs_members(listing: &str, pool: &str) -> Vec<String> {
    #[derive(Deserialize)]
    struct Member {
        kname: String,
        fstype: Option<String>,
        label: Option<String>,
    }
    #[derive(Deserialize)]
    struct Members {
        blockdevices: Vec<Member>,
    }
    serde_json::from_str::<Members>(listing)
        .map(|listing| listing.blockdevices)
        .unwrap_or_default()
        .into_iter()
        .filter(|m| m.fstype.as_deref() == Some("zfs_member") && m.label.as_deref() == Some(pool))
        .map(|m| m.kname)
        .collect()
}

/// The devices the running system stands on, from the mount table and sysfs.
fn system_devices() -> SystemDevices {
    let mountinfo = fs::read_to_string("/proc/self/mountinfo").unwrap_or_default();
    let points: Vec<&str> = SYSTEM_MOUNTS
        .into_iter()
        .filter(|point| point.starts_with('/'))
        .collect();
    system_disks::system_devices(&system_disks::mounts(&mountinfo), &Sysfs, &points)
}

/// Whether setup knows which disks hold `/`: from the mount table and sysfs,
/// or from lsblk's mount points.
fn root_known(devices: &[Device], system: &SystemDevices) -> bool {
    system.unknown_root.is_none() || devices.iter().any(Device::holds_root)
}

/// Setup's view of the whole disks `devices`. A disk holding part of the
/// running system is `system`. When setup cannot tell which disks hold `/`,
/// a USB is erased only once its model or size is typed back, since it may
/// be the disk this computer runs from.
fn whole_disks(devices: Vec<Device>, system: &SystemDevices) -> Vec<Disk> {
    let root_known = root_known(&devices, system);
    devices
        .into_iter()
        // A loop device stands in for a USB, as a disk image does on a
        // Mac; read-only ones hold snaps and images, never a USB.
        .filter(|d| d.kind == "disk" || d.kind == "loop" && !d.ro && d.size > 0)
        .map(|d| Disk {
            removal: match d.facts().removal() {
                Removal::Usb if !root_known => Removal::Unsure,
                removal => removal,
            },
            system: d.holds_system() || d.holds_any(&system.names),
            bytes: d.size,
            name: match d.model.as_deref().map(str::trim) {
                Some(model) if !model.is_empty() => model.to_owned(),
                _ if d.kind == "loop" => "loop device".into(),
                _ => String::new(),
            },
            partitions: d
                .children
                .iter()
                .filter(|p| p.kind == "part")
                .map(|p| Partition {
                    identifier: p.name.clone(),
                    content: p.content(),
                    mount: p.mount(),
                })
                .collect(),
            identifier: d.name,
        })
        .collect()
}

/// Why the whole disk `disk` cannot be opened to erase it: rights only
/// when the system says so, else what it said.
fn erase_access_error(disk: &str, error: std::io::Error) -> anyhow::Error {
    match error.raw_os_error() {
        Some(libc::EACCES | libc::EPERM) => {
            anyhow::Error::new(error).context(format!("USB access requires {ADMIN}"))
        }
        Some(libc::ENOMEDIUM) => {
            anyhow::anyhow!("/dev/{disk} has no medium in it: put the card in, or plug in a stick")
        }
        _ => anyhow::Error::new(error).context(format!("cannot open /dev/{disk} to erase it")),
    }
}

/// The vendor and device IDs of this computer's PCI devices of base class
/// `class`.
fn pci_devices(class: u32) -> Result<Vec<(u16, u16)>> {
    let hex = |device: &Path, name| {
        let text = fs::read_to_string(device.join(name)).ok()?;
        u32::from_str_radix(text.trim().strip_prefix("0x")?, 16).ok()
    };
    let mut devices = Vec::new();
    for device in fs::read_dir("/sys/bus/pci/devices")? {
        let device = device?.path();
        if hex(&device, "class").is_some_and(|found| found >> 16 == class)
            && let (Some(vendor), Some(id)) = (hex(&device, "vendor"), hex(&device, "device"))
        {
            devices.push((vendor as u16, id as u16));
        }
    }
    Ok(devices)
}

impl Host for System {
    fn nvidia_gpus(&self) -> Result<Vec<u16>> {
        // Base class 3 is a display controller; an NVIDIA GPU's audio
        // function is not one.
        Ok(pci_devices(3)?
            .into_iter()
            .filter(|&(vendor, _)| vendor == 0x10de)
            .map(|(_, device)| device)
            .collect())
    }

    fn volume(&self, path: &Path) -> Result<Volume> {
        use std::os::unix::fs::MetadataExt;
        let filesystem;
        let free_bytes;
        {
            use std::os::unix::ffi::OsStrExt;
            let c_path = std::ffi::CString::new(path.as_os_str().as_bytes())?;
            // SAFETY: statfs contains integer fields and fixed byte arrays, for which
            // all-zero initialization is valid before the syscall fills them.
            let mut info: libc::statfs = unsafe { std::mem::zeroed() };
            // SAFETY: c_path owns a NUL-terminated path and info is a writable
            // statfs structure; both remain live through the synchronous syscall.
            if unsafe { libc::statfs(c_path.as_ptr(), &mut info) } != 0 {
                return Err(std::io::Error::last_os_error())
                    .with_context(|| format!("cannot inspect {}", path.display()));
            }
            // Linux superblock magic numbers.
            filesystem = match info.f_type as u64 {
                0x4d44 => FileSystem::Fat,
                0x2011_bab0 => FileSystem::ExFat,
                0x482b | 0x4858 => FileSystem::Hfs,
                0x5346_544e | 0x7366_746e => FileSystem::Ntfs,
                other => FileSystem::Other(format!("file system {other:#x}")),
            };
            free_bytes = info.f_bavail as u64 * info.f_bsize as u64;
        }
        // The block device holding the volume, and the disk holding that.
        let device = fs::metadata(path)?.dev();
        let (major, minor) = (libc::major(device), libc::minor(device));
        let disk = fs::canonicalize(format!("/sys/dev/block/{major}:{minor}"))
            .ok()
            .and_then(|sys| {
                let disk = if sys.join("partition").exists() {
                    sys.parent()?
                } else {
                    &sys
                };
                disk.file_name()?.to_str().map(str::to_owned)
            });
        Ok(Volume {
            filesystem,
            free_bytes,
            disk,
        })
    }

    fn disks(&self) -> Result<Vec<Disk>> {
        let devices = lsblk(None)?;
        let system = system_devices();
        if let Some(reason) = &system.unknown_root
            && !root_known(&devices, &system)
        {
            static NOTED: Once = Once::new();
            NOTED.call_once(|| {
                eprintln!(
                    "Note: setup cannot tell which disk this computer runs from ({reason}). It \
                     erases a USB only once its model or size is typed back."
                );
            });
        }
        Ok(whole_disks(devices, &system))
    }

    fn partition(&self, identifier: &str) -> Result<PartitionInfo> {
        checked(identifier, true)?;
        let device = lsblk(Some(identifier))?
            .into_iter()
            .next()
            .with_context(|| format!("lsblk does not list {identifier}"))?;
        Ok(PartitionInfo {
            identifier: identifier.into(),
            disk: device.pkname.clone().unwrap_or_default(),
            guid: device
                .partuuid
                .as_deref()
                .unwrap_or_default()
                .to_ascii_uppercase(),
            bytes: device.size,
            content: device.content(),
            mount: device.mount(),
            container: None,
        })
    }

    fn serial(&self, disk: &str) -> Result<Option<String>> {
        checked(disk, false)?;
        // A loop device has no hardware, and no serial number.
        let Ok(device) = fs::canonicalize(Path::new("/sys/block").join(disk).join("device")) else {
            return Ok(None);
        };
        // A USB disk: its device's serial number string, which sysfs keeps
        // beside the device's vendor and product IDs.
        if let Some(usb) = device.ancestors().find(|dir| dir.join("idVendor").exists()) {
            return Ok(read_trimmed(&usb.join("serial")));
        }
        // An NVMe namespace: its controller's serial number.
        if let Some(serial) = read_trimmed(&device.join("serial")) {
            return Ok(Some(serial));
        }
        // An ATA disk: its IDENTIFY serial, which udev records.
        let output = Command::new("lsblk")
            .args(["--nodeps", "--noheadings", "--output", "SERIAL"])
            .arg(format!("/dev/{disk}"))
            .output()
            .context("cannot run lsblk")?;
        Ok(
            Some(String::from_utf8_lossy(&output.stdout).trim().to_owned())
                .filter(|serial| output.status.success() && !serial.is_empty()),
        )
    }

    fn read(&self, identifier: &str, offset: u64, length: usize) -> Result<Vec<u8>> {
        use std::os::unix::fs::FileExt;
        checked(identifier, true)?;
        let path = format!("/dev/{identifier}");
        let file = fs::File::open(&path).with_context(|| {
            format!(
                "cannot read {path}; setup reads the installed disk's partitions as root ({ADMIN})"
            )
        })?;
        let mut bytes = vec![0; length];
        file.read_exact_at(&mut bytes, offset)?;
        Ok(bytes)
    }

    fn write(&self, identifier: &str, offset: u64, bytes: &[u8]) -> Result<()> {
        use std::os::unix::fs::FileExt;
        checked(identifier, true)?;
        let path = format!("/dev/{identifier}");
        let file = fs::OpenOptions::new()
            .write(true)
            .open(&path)
            .with_context(|| format!("cannot open {path} for writing ({ADMIN})"))?;
        file.write_all_at(bytes, offset)?;
        file.sync_data()?;
        Ok(())
    }

    fn unmount(&self, partition: &PartitionInfo) -> Result<()> {
        checked(&partition.identifier, true)?;
        let device = format!("/dev/{}", partition.identifier);
        let udisks = run(Command::new("udisksctl").args([
            "unmount",
            "--no-user-interaction",
            "--block-device",
            &device,
        ]));
        if let Err(udisks) = udisks {
            run(Command::new("umount").arg(&device)).with_context(|| {
                format!("udisksctl could not unmount {device} either: {udisks}")
            })?;
        }
        Ok(())
    }

    fn mount(&self, identifier: &str) -> Result<PathBuf> {
        checked(identifier, true)?;
        let device = format!("/dev/{identifier}");
        let udisks = run(Command::new("udisksctl").args([
            "mount",
            "--no-user-interaction",
            "--block-device",
            &device,
        ]));
        if let Err(udisks) = udisks {
            // Without udisks, as root: a mount point of setup's own.
            let mount = Path::new("/run/stela").join(identifier);
            fs::create_dir_all(&mount)
                .with_context(|| format!("cannot create {} ({ADMIN})", mount.display()))?;
            run(Command::new("mount").arg(&device).arg(&mount))
                .with_context(|| format!("udisksctl could not mount {device} either: {udisks}"))?;
        }
        self.partition(identifier)?
            .mount
            .with_context(|| format!("{device} was mounted nowhere"))
    }

    fn flush(&self, path: &Path) -> Result<()> {
        use std::os::unix::io::AsRawFd;
        let directory =
            fs::File::open(path).with_context(|| format!("cannot open {}", path.display()))?;
        // SAFETY: syncfs takes an open descriptor, which `directory` keeps
        // open through the call.
        if unsafe { libc::syncfs(directory.as_raw_fd()) } != 0 {
            return Err(std::io::Error::last_os_error())
                .with_context(|| format!("cannot write out {}", path.display()));
        }
        Ok(())
    }

    fn check_erase_access(&self, disk: &str) -> Result<()> {
        fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(format!("/dev/{disk}"))
            .map_err(|error| erase_access_error(disk, error))?;
        Ok(())
    }

    fn erase(&self, disk: &str, name: &str) -> Result<()> {
        use std::os::unix::{fs::OpenOptionsExt, io::AsRawFd};
        checked(disk, false)?;
        for listed in self.disks()?.iter().filter(|d| d.identifier == disk) {
            for mounted in listed.partitions.iter().filter(|p| p.mount.is_some()) {
                self.unmount(&self.partition(&mounted.identifier)?)?;
            }
        }
        let sys = Path::new("/sys/block").join(disk);
        let sectors: u64 = read_trimmed(&sys.join("size"))
            .and_then(|size| size.parse().ok())
            .with_context(|| format!("sysfs gives no size for {disk}"))?;
        let block_size: u32 = read_trimmed(&sys.join("queue/logical_block_size"))
            .and_then(|size| size.parse().ok())
            .with_context(|| format!("sysfs gives no block size for {disk}"))?;
        // sysfs counts 512-byte sectors whatever the disk's block size.
        let layout = crate::setup::format::layout(sectors * 512, block_size, name, &mut |bytes| {
            getrandom::getrandom(bytes).context("the system's random generator failed")
        })?;
        let path = format!("/dev/{disk}");
        // Exclusively: the kernel refuses while any partition is in use.
        let mut file = fs::OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_EXCL)
            .open(&path)
            .map_err(|error| match error.raw_os_error() {
                Some(libc::EBUSY) => anyhow::anyhow!(
                    "{path} is in use: a volume on it is still mounted or open (another program, \
                     an encrypted volume, a RAID or LVM member); close it and run setup again"
                ),
                _ => anyhow::Error::new(error)
                    .context(format!("cannot open {path} to erase it ({ADMIN})")),
            })?;
        super::write_layout(&mut file, &layout)?;
        // The kernel reads the new partition table; udev, probing the disk
        // after the write, may hold it briefly.
        let mut reread = Err(std::io::Error::other("not tried"));
        for _ in 0..10 {
            // SAFETY: BLKRRPART takes no argument.
            reread = match unsafe { libc::ioctl(file.as_raw_fd(), BLKRRPART as _) } {
                0 => Ok(()),
                _ => Err(std::io::Error::last_os_error()),
            };
            if reread.is_ok() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(300));
        }
        reread.with_context(|| {
            format!(
                "{disk} was erased, but the kernel did not read its new partition table (a loop \
                 device needs partition scanning: losetup -P)"
            )
        })?;
        drop(file);
        let _ = Command::new("udevadm").arg("settle").status();
        // The data volume: the disk's second partition, once udev names it.
        let mut data = None;
        for _ in 0..50 {
            data = fs::read_dir(&sys)?.flatten().find_map(|entry| {
                let number = read_trimmed(&entry.path().join("partition"))?;
                (number == "2").then(|| entry.file_name().to_string_lossy().into_owned())
            });
            if data
                .as_ref()
                .is_some_and(|data| Path::new("/dev").join(data).exists())
            {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        let data = data.with_context(|| format!("{disk} shows no data partition after erasing"))?;
        self.mount(&data)?;
        Ok(())
    }

    fn whole_disk(&self, usb: &Path) -> Result<Option<String>> {
        if usb.is_dir() {
            return Ok(None);
        }
        // `/dev/disk/by-id` and other links name the disk they point to.
        let usb = fs::canonicalize(usb).unwrap_or_else(|_| usb.to_path_buf());
        let Some(name) = usb.to_str() else {
            return Ok(None);
        };
        let name = name.strip_prefix("/dev/").unwrap_or(name);
        Ok(self
            .disks()?
            .into_iter()
            .map(|disk| disk.identifier)
            .find(|identifier| identifier == name))
    }

    fn cpu(&self) -> Result<Cpu> {
        super::cpu()
    }
}

#[cfg(test)]
mod tests {
    use super::super::Removal;
    use super::*;

    #[test]
    fn lsblk_old_and_new_formats_name_esps_and_removable_disks() {
        let new = r#"{"blockdevices": [
            {"name": "sdb", "type": "disk", "size": 64023257088, "model": "Ultra Fit ",
             "tran": "usb", "rm": true, "hotplug": true, "ro": false, "mountpoint": null,
             "parttype": null, "partuuid": null, "pkname": null, "children": [
                {"name": "sdb1", "type": "part", "size": 209715200, "model": null,
                 "tran": null, "rm": true, "hotplug": true, "ro": false,
                 "mountpoint": "/media/me/EFI",
                 "parttype": "C12A7328-F81F-11D2-BA4B-00A0C93EC93B",
                 "partuuid": "0a1b2c3d-0000-4000-8000-000000000001", "pkname": "sdb"},
                {"name": "sdb2", "type": "part", "size": 63000000000, "model": null,
                 "tran": null, "rm": true, "hotplug": true, "ro": false,
                 "mountpoint": "[SWAP]", "parttype": "0657fd6d-a4ab-43c4-84e5-0933c84b4f4f",
                 "partuuid": null, "pkname": "sdb"}]}]}"#;
        let old = r#"{"blockdevices": [
            {"name": "sda", "type": "disk", "size": "500107862016", "model": "SSD",
             "tran": "sata", "rm": "0", "hotplug": "0", "ro": "0", "mountpoint": null,
             "parttype": null, "partuuid": null, "pkname": null, "children": [
                {"name": "sda1", "type": "part", "size": "500000000000", "model": null,
                 "tran": null, "rm": "0", "hotplug": "0", "ro": "0", "mountpoint": "/",
                 "parttype": "0x83", "partuuid": null, "pkname": "sda"}]}]}"#;
        let new: Listing = serde_json::from_str(new).unwrap();
        let usb = &new.blockdevices[0];
        // Its active swap makes it part of the running system.
        assert!(usb.rm && !usb.holds_root() && usb.holds_system());
        assert_eq!(usb.children[0].content(), "EFI");
        assert_eq!(
            usb.children[0].mount(),
            Some(PathBuf::from("/media/me/EFI"))
        );
        // Swap is no mounted volume.
        assert_eq!(usb.children[1].mount(), None);
        let old: Listing = serde_json::from_str(old).unwrap();
        let ssd = &old.blockdevices[0];
        assert_eq!(ssd.size, 500_107_862_016);
        assert!(!ssd.rm && ssd.holds_root() && ssd.holds_system());
        assert_eq!(ssd.children[0].content(), "0x83");
        assert_eq!(usb.facts().removal(), Removal::Usb);
        assert_eq!(ssd.facts().removal(), Removal::Fixed);
    }

    fn known(names: &[&str]) -> SystemDevices {
        SystemDevices {
            names: names.iter().map(|name| name.to_string()).collect(),
            unknown_root: None,
        }
    }

    fn unknown() -> SystemDevices {
        SystemDevices {
            names: BTreeSet::new(),
            unknown_root: Some("/ is tmpfs none".into()),
        }
    }

    #[test]
    fn root_guard_uses_all_mounts() {
        let source = r#"{"blockdevices": [
            {"name":"sda","type":"disk","model":"USB system disk","tran":"usb",
             "mountpoint":"/bind","mountpoints":["/", "/bind"]},
            {"name":"sdb","type":"disk","model":"USB target","tran":"usb",
             "mountpoint":null,"mountpoints":null}]}"#;
        let mut listing: Listing = serde_json::from_str(source).unwrap();
        assert!(root_known(&listing.blockdevices, &unknown()));
        assert!(listing.blockdevices[0].holds_root());
        assert!(!listing.blockdevices[1].holds_root());
        listing.blockdevices[0].mountpoints = None;
        assert!(!root_known(&listing.blockdevices, &unknown()));
        assert!(!root_known(&[], &unknown()));
        assert!(root_known(&[], &known(&[])));
    }

    #[test]
    fn a_disk_under_a_system_mount_is_never_offered() {
        // lsblk lists device-mapper devices under the partitions they stand
        // on, by their mapper names; sysfs names them dm-N.
        let source = r#"{"blockdevices": [
            {"name":"nvme0n1","type":"disk","tran":"nvme","children":[
                {"name":"nvme0n1p3","type":"part","children":[
                    {"name":"luks-root","type":"crypt"}]}]},
            {"name":"sdb","type":"disk","model":"Stick","tran":"usb","rm":true,"size":64000000000,
             "children":[{"name":"sdb1","type":"part"}]},
            {"name":"sdc","type":"disk","model":"Target","tran":"usb","rm":true,"size":64000000000}]}"#;
        let disks = |system: &SystemDevices| {
            let listing: Listing = serde_json::from_str(source).unwrap();
            whole_disks(listing.blockdevices, system)
                .into_iter()
                .map(|d| (d.identifier, d.system, d.removal))
                .collect::<Vec<_>>()
        };
        // A composefs root over /sysroot on the NVMe disk, and a live
        // system's squashfs looped from the stick.
        assert_eq!(
            disks(&known(&["dm-0", "nvme0n1p3", "loop0", "sdb1"])),
            [
                ("nvme0n1".into(), true, Removal::Fixed),
                ("sdb".into(), true, Removal::Usb),
                ("sdc".into(), false, Removal::Usb),
            ]
        );
        // Nothing found under /: every USB needs its model or size typed
        // back, and an internal disk stays fixed.
        assert_eq!(
            disks(&unknown()),
            [
                ("nvme0n1".into(), false, Removal::Fixed),
                ("sdb".into(), false, Removal::Unsure),
                ("sdc".into(), false, Removal::Unsure),
            ]
        );
    }

    #[test]
    fn erase_access_names_rights_only_when_they_are_missing() {
        let text = |code| {
            format!(
                "{:#}",
                erase_access_error("sdb", std::io::Error::from_raw_os_error(code))
            )
        };
        assert!(text(libc::EACCES).starts_with("USB access requires sudo"));
        assert!(text(libc::EPERM).starts_with("USB access requires sudo"));
        let empty = text(libc::ENOMEDIUM);
        assert!(empty.contains("/dev/sdb has no medium"), "{empty}");
        assert!(!empty.contains("sudo"), "{empty}");
        let busy = text(libc::EBUSY);
        assert!(
            busy.starts_with("cannot open /dev/sdb to erase it: "),
            "{busy}"
        );
        assert!(!busy.contains("sudo"), "{busy}");
    }

    #[test]
    fn zfs_members_by_pool_label() {
        let listing = r#"{"blockdevices": [
            {"kname":"nvme0n1","fstype":null,"label":null},
            {"kname":"nvme0n1p4","fstype":"zfs_member","label":"rpool"},
            {"kname":"nvme0n1p3","fstype":"zfs_member","label":"bpool"},
            {"kname":"sdb1","fstype":"vfat","label":"rpool"}]}"#;
        assert_eq!(zfs_members(listing, "rpool"), ["nvme0n1p4"]);
        assert!(zfs_members("not json", "rpool").is_empty());
    }

    #[test]
    fn a_disk_holding_boot_home_or_swap_is_never_offered() {
        let source = r#"{"blockdevices": [
            {"name":"nvme0n1","type":"disk","tran":"nvme","mountpoint":null,"children":[
                {"name":"nvme0n1p2","type":"part","mountpoint":"/"}]},
            {"name":"sdb","type":"disk","tran":"usb","rm":true,"mountpoint":null,"children":[
                {"name":"sdb1","type":"part","mountpoint":"/boot/efi"}]},
            {"name":"sdc","type":"disk","tran":"usb","rm":true,"mountpoint":null,"children":[
                {"name":"sdc1","type":"part","mountpoints":["/home"]}]},
            {"name":"sdd","type":"disk","tran":"usb","rm":true,"mountpoint":null,"children":[
                {"name":"sdd1","type":"part","mountpoint":"/media/me/STELA"}]}]}"#;
        let listing: Listing = serde_json::from_str(source).unwrap();
        let system: Vec<bool> = listing
            .blockdevices
            .iter()
            .map(Device::holds_system)
            .collect();
        assert_eq!(system, [true, true, true, false]);
    }

    #[test]
    fn removal_kind_follows_transport_not_hotplug() {
        let listing = r#"{"blockdevices": [
            {"name": "nvme1n1", "type": "disk", "size": 1000204886016, "model": "WD Blue",
             "tran": "nvme", "rm": false, "hotplug": true, "ro": false, "mountpoint": null,
             "parttype": null, "partuuid": null, "pkname": null},
            {"name": "mmcblk0", "type": "disk", "size": 62537072640, "model": null,
             "tran": "mmc", "rm": false, "hotplug": false, "ro": false, "mountpoint": null,
             "parttype": null, "partuuid": null, "pkname": null},
            {"name": "mmcblk1", "type": "disk", "size": 63864569856, "model": null,
             "tran": "mmc", "rm": true, "hotplug": true, "ro": false, "mountpoint": null,
             "parttype": null, "partuuid": null, "pkname": null},
            {"name": "loop3", "type": "loop", "size": 8589934592, "model": null,
             "tran": null, "rm": false, "hotplug": false, "ro": false, "mountpoint": null,
             "parttype": null, "partuuid": null, "pkname": null}]}"#;
        let listing: Listing = serde_json::from_str(listing).unwrap();
        let removal: Vec<_> = listing
            .blockdevices
            .iter()
            .map(|d| d.facts().removal())
            .collect();
        assert_eq!(
            removal,
            [Removal::Fixed, Removal::Fixed, Removal::Typed, Removal::Usb]
        );
    }
}
