//! macOS: `diskutil` lists and names the disks, mounts, unmounts and erases;
//! `system_profiler` and `ioreg` report serial numbers; partitions are read
//! and written through their raw devices, `/dev/rdiskNsM`.
use super::{
    Cpu, Disk, FileSystem, Host, PartitionInfo, Volume,
    diskutil::{self, array, raw_device, serial_of, text, usb_serial_of, whole_disk},
};
use anyhow::{Context, Result, bail, ensure};
use plist::Value;
use std::{
    path::{Path, PathBuf},
    process::Command,
};

/// How setup gets the rights its raw reads and writes need.
pub(crate) const ADMIN: &str = "sudo";
/// A whole disk as STICK names it.
/// What a disk identifier is preceded by where a user types it.
pub(crate) const DISK_PREFIX: &str = "";
pub(crate) const DISK_EXAMPLE: &str = "disk5";

/// The computer `stela` runs on.
pub(crate) struct System;

impl Host for System {
    fn volume(&self, path: &Path) -> Result<Volume> {
        statfs(path)
    }

    fn disks(&self) -> Result<Vec<Disk>> {
        let list = plist(Command::new("diskutil").args(["list", "-plist"]))?;
        diskutil::disks(&list, &mut |identifier| {
            plist(Command::new("diskutil").args(["info", "-plist", identifier]))
        })
    }

    fn partition(&self, identifier: &str) -> Result<PartitionInfo> {
        ensure!(
            identifier.starts_with("disk") && identifier.bytes().all(|b| b.is_ascii_alphanumeric()),
            "{identifier} is not a partition identifier such as disk1s2"
        );
        let info = plist(Command::new("diskutil").args(["info", "-plist", identifier]))?;
        let size = info
            .as_dictionary()
            .and_then(|d| d.get("Size").or_else(|| d.get("TotalSize")))
            .and_then(Value::as_unsigned_integer)
            .unwrap_or(0);
        let list = plist(Command::new("diskutil").args(["list", "-plist"]))?;
        let container = array(&list, "AllDisksAndPartitions")
            .iter()
            .find(|entry| {
                array(entry, "APFSPhysicalStores")
                    .iter()
                    .any(|store| text(store, "DeviceIdentifier") == identifier)
                    && array(entry, "APFSVolumes")
                        .iter()
                        .any(|volume| !text(volume, "MountPoint").is_empty())
            })
            .map(|entry| text(entry, "DeviceIdentifier"));
        Ok(PartitionInfo {
            identifier: identifier.into(),
            disk: text(&info, "ParentWholeDisk"),
            guid: text(&info, "DiskUUID"),
            bytes: size,
            content: text(&info, "Content"),
            mount: Some(text(&info, "MountPoint"))
                .filter(|m| !m.is_empty())
                .map(PathBuf::from),
            container,
        })
    }

    fn serial(&self, disk: &str) -> Result<Option<String>> {
        let output = Command::new("system_profiler")
            .args(["-json", "SPNVMeDataType", "SPSerialATADataType"])
            .output()
            .context("cannot run system_profiler")?;
        let report: serde_json::Value = serde_json::from_slice(&output.stdout)?;
        if let Some(serial) = serial_of(&report, disk) {
            return Ok(Some(serial));
        }
        // A USB disk: its device's serial number string, which the
        // IORegistry links to the disk as the device's descendant.
        let devices =
            plist(Command::new("ioreg").args(["-a", "-r", "-c", "IOUSBHostDevice", "-l"]))?;
        Ok(usb_serial_of(&devices, disk))
    }

    fn read(&self, identifier: &str, offset: u64, length: usize) -> Result<Vec<u8>> {
        use std::os::unix::fs::FileExt;
        let path = raw_device(identifier)?;
        let file = std::fs::File::open(&path).with_context(|| {
            format!(
                "cannot read {path}; setup reads the installed disk's partitions as root (sudo)"
            )
        })?;
        // A raw device takes whole blocks at block offsets only.
        super::read_blocks(block_size(&file), offset, length, &mut |buffer, at| {
            file.read_exact_at(buffer, at)
        })
        .with_context(|| format!("cannot read {path}"))
    }

    fn write(&self, identifier: &str, offset: u64, bytes: &[u8]) -> Result<()> {
        use std::os::unix::fs::FileExt;
        let path = raw_device(identifier)?;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&path)
            .with_context(|| format!("cannot open {path} for writing (sudo)"))?;
        // A raw device has no file cache to flush: its writes reach the
        // disk unbuffered, and `sync_all`'s F_FULLFSYNC fails on it (ENOTTY).
        super::write_blocks(
            block_size(&file),
            offset,
            bytes,
            &mut |buffer, at| file.read_exact_at(buffer, at),
            &mut |data, at| file.write_all_at(data, at),
        )
        .with_context(|| format!("cannot write {path}"))
    }

    fn unmount(&self, partition: &PartitionInfo) -> Result<()> {
        let mut command = Command::new("diskutil");
        match &partition.container {
            Some(container) => command.args(["unmountDisk", container]),
            None => command.args(["unmount", &partition.identifier]),
        };
        let status = command.status().context("cannot run diskutil")?;
        ensure!(
            status.success(),
            "diskutil could not unmount {}",
            partition
                .container
                .as_deref()
                .unwrap_or(&partition.identifier)
        );
        Ok(())
    }

    fn mount(&self, identifier: &str) -> Result<PathBuf> {
        let status = Command::new("diskutil")
            .args(["mount", identifier])
            .status()
            .context("cannot run diskutil")?;
        ensure!(status.success(), "diskutil could not mount {identifier}");
        let info = plist(Command::new("diskutil").args(["info", "-plist", identifier]))?;
        let mount = text(&info, "MountPoint");
        ensure!(!mount.is_empty(), "diskutil mounted {identifier} nowhere");
        Ok(PathBuf::from(mount))
    }

    fn erase(&self, disk: &str, name: &str) -> Result<()> {
        // diskutil's GPT always starts with a 200 MiB EFI partition.
        let status = Command::new("diskutil")
            .args(["eraseDisk", "FAT32", name, "GPT", disk])
            .status()
            .context("cannot run diskutil")?;
        ensure!(status.success(), "diskutil could not erase {disk}");
        Ok(())
    }

    fn flush(&self, _path: &Path) -> Result<()> {
        // macOS has no per-volume sync; each file setup wrote is already
        // synced with F_FULLFSYNC, and this writes out the rest.
        // SAFETY: sync takes no arguments.
        unsafe { libc::sync() };
        Ok(())
    }

    fn cpu(&self) -> Result<Cpu> {
        super::cpu()
    }

    /// `disk6`, `/dev/disk6` or `/dev/rdisk6` names whole disk `disk6`; a
    /// partition is refused (`diskutil::typed_disk`).
    fn whole_disk(&self, usb: &Path) -> Result<Option<String>> {
        if usb.is_dir() {
            return Ok(None);
        }
        let Some(disk) = usb
            .to_str()
            .map(diskutil::typed_disk)
            .transpose()?
            .flatten()
        else {
            return Ok(None);
        };
        Ok(self
            .disks()?
            .into_iter()
            .map(|listed| listed.identifier)
            .find(|identifier| *identifier == disk))
    }
}

/// The block size of the raw device `file` (`DKIOCGETBLOCKSIZE`), or 4096,
/// which every disk's blocks divide.
fn block_size(file: &std::fs::File) -> u64 {
    use std::os::unix::io::AsRawFd;
    /// `_IOR('d', 24, uint32_t)`.
    const DKIOCGETBLOCKSIZE: libc::c_ulong = 0x4004_6418;
    let mut size: u32 = 0;
    // SAFETY: the request writes one u32 into `size`, which outlives the
    // call, through a descriptor `file` keeps open.
    let done = unsafe { libc::ioctl(file.as_raw_fd(), DKIOCGETBLOCKSIZE, &mut size) };
    match done == 0 && size >= 512 && size.is_power_of_two() {
        true => u64::from(size),
        false => 4096,
    }
}

fn statfs(path: &Path) -> Result<Volume> {
    use std::{ffi::CStr, os::unix::ffi::OsStrExt};
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
    // SAFETY: the successful statfs call NUL-terminated f_fstypename in info.
    let name = unsafe { CStr::from_ptr(info.f_fstypename.as_ptr()) }.to_string_lossy();
    // SAFETY: the successful statfs call NUL-terminated f_mntfromname in info.
    let device = unsafe { CStr::from_ptr(info.f_mntfromname.as_ptr()) }.to_string_lossy();
    let filesystem = match name.as_ref() {
        "msdos" => FileSystem::Fat,
        "exfat" => FileSystem::ExFat,
        "hfs" => FileSystem::Hfs,
        "apfs" => FileSystem::Apfs,
        "ntfs" | "ufsd_NTFS" | "tuxera_ntfs" => FileSystem::Ntfs,
        other => FileSystem::Other(other.into()),
    };
    Ok(Volume {
        filesystem,
        free_bytes: info.f_bavail * info.f_bsize as u64,
        disk: device.strip_prefix("/dev/").map(whole_disk),
    })
}

fn plist(command: &mut Command) -> Result<Value> {
    let output = command
        .output()
        .with_context(|| format!("cannot run {:?}", command.get_program()))?;
    if !output.status.success() {
        bail!(
            "{:?} failed: {}",
            command.get_program(),
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(Value::from_reader(std::io::Cursor::new(output.stdout))?)
}
