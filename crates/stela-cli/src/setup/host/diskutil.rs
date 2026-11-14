//! What macOS's `diskutil`, `system_profiler` and `ioreg` print, read into
//! setup's disks. The macOS backend runs the tools; this module only reads
//! their output, so it is compiled and tested on every host.
use super::{Bus, Disk, DiskFacts, Partition};
use anyhow::{Result, ensure};
use plist::Value;
use std::path::PathBuf;

/// The disks of a `diskutil list -plist` listing, each with the facts its
/// `diskutil info -plist` gives (`info`): every physical disk, and disk
/// images, which stand in for a USB as a stick's copy does in a VM. The
/// synthesized APFS containers are left out: their physical stores are the
/// disks. A disk that holds the running system is marked so, and setup
/// never offers it.
pub(super) fn disks(
    list: &Value,
    info: &mut dyn FnMut(&str) -> Result<Value>,
) -> Result<Vec<Disk>> {
    let system_disks = system_disks(list)?;
    let mut disks = Vec::new();
    for entry in array(list, "AllDisksAndPartitions") {
        let identifier = text(entry, "DeviceIdentifier");
        if !array(entry, "APFSPhysicalStores").is_empty() {
            continue;
        }
        let info = info(&identifier)?;
        let flag = |key| {
            info.as_dictionary()
                .and_then(|d| d.get(key))
                .and_then(Value::as_boolean)
        };
        let image = text(&info, "BusProtocol") == "Disk Image";
        // Other virtual disks are no USB; nor is a read-only image, such as
        // a mounted installer.
        if text(&info, "VirtualOrPhysical") == "Virtual" && !image
            || image && flag("WritableMedia") == Some(false)
        {
            continue;
        }
        let partitions = array(entry, "Partitions")
            .iter()
            .map(|p| Partition {
                identifier: text(p, "DeviceIdentifier"),
                content: text(p, "Content"),
                mount: Some(text(p, "MountPoint"))
                    .filter(|m| !m.is_empty())
                    .map(PathBuf::from),
            })
            .collect();
        disks.push(Disk {
            name: text(&info, "MediaName"),
            bytes: info
                .as_dictionary()
                .and_then(|d| d.get("TotalSize").or_else(|| d.get("Size")))
                .and_then(Value::as_unsigned_integer)
                .unwrap_or(0),
            removal: DiskFacts {
                bus: match text(&info, "BusProtocol").as_str() {
                    "USB" => Bus::Usb,
                    "FireWire" => Bus::FireWire,
                    "Disk Image" => Bus::Image,
                    _ => Bus::Other,
                },
                removable_media: flag("RemovableMedia").unwrap_or(false),
                // A disk diskutil does not place is taken as internal.
                external: flag("Internal") == Some(false),
            }
            .removal(),
            system: system_disks.contains(&identifier),
            partitions,
            identifier,
        });
    }
    Ok(disks)
}

/// The whole disks holding the physical stores of the APFS container that
/// the running macOS is mounted from.
pub(super) fn system_disks(list: &Value) -> Result<Vec<String>> {
    let mut disks = Vec::new();
    for entry in array(list, "AllDisksAndPartitions") {
        let mounted_root = array(entry, "APFSVolumes").iter().any(|volume| {
            matches!(
                text(volume, "MountPoint").as_str(),
                "/" | "/System/Volumes/Data"
            )
        });
        if !mounted_root {
            continue;
        }
        for store in array(entry, "APFSPhysicalStores") {
            let device = text(store, "DeviceIdentifier");
            let valid = device.strip_prefix("disk").is_some_and(|suffix| {
                suffix
                    .split('s')
                    .all(|part| !part.is_empty() && part.bytes().all(|byte| byte.is_ascii_digit()))
            });
            ensure!(
                valid,
                "the running macOS installation has an invalid physical store identifier: {device:?}"
            );
            disks.push(whole_disk(&device));
        }
    }
    ensure!(
        !disks.is_empty(),
        "cannot identify the physical disks holding the running macOS installation"
    );
    Ok(disks)
}

/// `disk5s1` → `disk5`; APFS volumes such as `disk3s1s1` → `disk3`.
pub(super) fn whole_disk(device: &str) -> String {
    match device.strip_prefix("disk").and_then(|rest| rest.find('s')) {
        Some(at) => device[..4 + at].into(),
        None => device.into(),
    }
}

/// The whole disk a typed STICK names, as macOS users type one: `disk6`,
/// `/dev/disk6` or its raw device `/dev/rdisk6`, all `disk6`. `None` when it
/// names no disk, such as a volume's path; a partition (`disk6s1`,
/// `/dev/rdisk6s1`) is refused, naming its whole disk.
pub(super) fn typed_disk(typed: &str) -> Result<Option<String>> {
    let name = typed.strip_prefix("/dev/").unwrap_or(typed);
    let name = name.strip_prefix('r').unwrap_or(name);
    let Some(rest) = name.strip_prefix("disk") else {
        return Ok(None);
    };
    let digits = |text: &str| !text.is_empty() && text.bytes().all(|b| b.is_ascii_digit());
    if digits(rest) {
        return Ok(Some(name.into()));
    }
    if let Some((disk, slice)) = rest.split_once('s')
        && digits(disk)
        && digits(slice)
    {
        anyhow::bail!(
            "{typed} is a partition; name the whole stick, disk{disk}, which stela erases whole"
        );
    }
    Ok(None)
}

/// `/dev/rdiskNsM` for a partition identifier.
pub(super) fn raw_device(identifier: &str) -> Result<String> {
    let partition = identifier.strip_prefix("disk").is_some_and(|rest| {
        rest.split_once('s').is_some_and(|(disk, slice)| {
            !disk.is_empty()
                && !slice.is_empty()
                && disk.bytes().all(|b| b.is_ascii_digit())
                && slice.bytes().all(|b| b.is_ascii_digit())
        })
    });
    ensure!(
        partition,
        "{identifier} is not a partition identifier such as disk1s2"
    );
    Ok(format!("/dev/r{identifier}"))
}

/// The serial `system_profiler -json` reports for the whole disk `disk`.
pub(super) fn serial_of(report: &serde_json::Value, disk: &str) -> Option<String> {
    let groups = report.as_object()?;
    for controllers in groups.values().filter_map(serde_json::Value::as_array) {
        for controller in controllers {
            let devices = controller
                .get("_items")
                .and_then(serde_json::Value::as_array);
            for device in devices.into_iter().flatten() {
                if device.get("bsd_name").and_then(serde_json::Value::as_str) == Some(disk) {
                    return device
                        .get("device_serial")
                        .and_then(serde_json::Value::as_str)
                        .map(|s| s.trim().to_owned())
                        .filter(|s| !s.is_empty());
                }
            }
        }
    }
    None
}

/// The `USB Serial Number` of the USB device in an `ioreg -a -r -c
/// IOUSBHostDevice -l` listing whose descendants include whole disk `disk`.
pub(super) fn usb_serial_of(devices: &Value, disk: &str) -> Option<String> {
    fn holds(entry: &Value, disk: &str) -> bool {
        let Some(entry) = entry.as_dictionary() else {
            return false;
        };
        let whole = entry.get("Whole").and_then(Value::as_boolean) == Some(true);
        if whole && entry.get("BSD Name").and_then(Value::as_string) == Some(disk) {
            return true;
        }
        entry
            .get("IORegistryEntryChildren")
            .and_then(Value::as_array)
            .is_some_and(|children| children.iter().any(|child| holds(child, disk)))
    }
    let devices = match devices.as_array() {
        Some(devices) => devices.as_slice(),
        None => core::slice::from_ref(devices),
    };
    devices
        .iter()
        .find(|device| holds(device, disk))?
        .as_dictionary()?
        .get("USB Serial Number")?
        .as_string()
        .map(|serial| serial.trim().to_owned())
        .filter(|serial| !serial.is_empty())
}

pub(super) fn text(value: &Value, key: &str) -> String {
    value
        .as_dictionary()
        .and_then(|d| d.get(key))
        .and_then(Value::as_string)
        .unwrap_or_default()
        .to_owned()
}

pub(super) fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value
        .as_dictionary()
        .and_then(|d| d.get(key))
        .and_then(Value::as_array)
        .map_or(&[], Vec::as_slice)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::setup::host::Removal;

    fn fixture(name: &str) -> Value {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/diskutil")
            .join(name);
        Value::from_file(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
    }

    /// A Mac with its internal SSD (disk0, whose APFS container disk3 runs
    /// macOS), a USB stick (disk4), an SD card in the built-in reader
    /// (disk5), a Thunderbolt SSD (disk6), a mounted read-only installer
    /// image (disk7) and a writable disk image standing in for a USB
    /// (disk8). The listing and the per-disk facts are written for
    /// this test; no serial numbers or volume names of a real Mac.
    fn listed() -> Vec<Disk> {
        let list = fixture("list.plist");
        disks(&list, &mut |identifier| {
            Ok(fixture(&format!("info-{identifier}.plist")))
        })
        .unwrap()
    }

    #[test]
    fn only_removable_disks_that_do_not_run_macos_are_erasable() {
        let disks = listed();
        let ids: Vec<&str> = disks.iter().map(|d| d.identifier.as_str()).collect();
        // The APFS container and the installer image are not offered.
        assert_eq!(ids, ["disk0", "disk4", "disk5", "disk6", "disk8"]);
        let by = |id: &str| disks.iter().find(|d| d.identifier == id).unwrap();
        let internal = by("disk0");
        assert!(internal.system && internal.removal == Removal::Fixed);
        assert!(!internal.erasable());
        let usb = by("disk4");
        assert_eq!(
            (usb.removal, usb.system, usb.bytes, usb.name.as_str()),
            (Removal::Usb, false, 32_017_047_552, "Sample Flash Disk")
        );
        assert_eq!(usb.partitions.len(), 2);
        assert_eq!(usb.partitions[0].content, "EFI");
        assert_eq!(
            usb.partitions[1].mount.as_deref(),
            Some(std::path::Path::new("/Volumes/STELA"))
        );
        // Removable media and a disk macOS places outside the Mac need their
        // model or size typed back.
        assert_eq!(by("disk5").removal, Removal::Typed);
        assert_eq!(by("disk6").removal, Removal::Typed);
        assert_eq!(by("disk8").removal, Removal::Usb);
        let erasable: Vec<&str> = disks
            .iter()
            .filter(|d| d.erasable())
            .map(|d| d.identifier.as_str())
            .collect();
        assert_eq!(erasable, ["disk4", "disk5", "disk6", "disk8"]);
    }

    #[test]
    fn a_listing_without_the_running_system_offers_nothing() {
        let mut list = fixture("list.plist");
        let entries = list
            .as_dictionary_mut()
            .and_then(|d| d.get_mut("AllDisksAndPartitions"))
            .and_then(Value::as_array_mut)
            .unwrap();
        entries.retain(|entry| array(entry, "APFSPhysicalStores").is_empty());
        assert!(disks(&list, &mut |_| unreachable!()).is_err());
    }

    #[test]
    fn root_store_identifiers_are_validated() {
        let listing = |identifier: &str| {
            let dictionary = |entries: Vec<(&str, Value)>| {
                Value::Dictionary(
                    entries
                        .into_iter()
                        .map(|(key, value)| (key.to_owned(), value))
                        .collect(),
                )
            };
            dictionary(vec![(
                "AllDisksAndPartitions",
                Value::Array(vec![dictionary(vec![
                    (
                        "APFSVolumes",
                        Value::Array(vec![dictionary(vec![("MountPoint", Value::from("/"))])]),
                    ),
                    (
                        "APFSPhysicalStores",
                        Value::Array(vec![dictionary(vec![(
                            "DeviceIdentifier",
                            Value::from(identifier),
                        )])]),
                    ),
                ])]),
            )])
        };
        for malformed in ["", "disk", "disk2s", "disk2s1garbage", "other2"] {
            assert!(system_disks(&listing(malformed)).is_err(), "{malformed}");
        }
        assert_eq!(system_disks(&listing("disk2s1")).unwrap(), ["disk2"]);
        assert!(system_disks(&Value::Dictionary(Default::default())).is_err());
    }

    #[test]
    fn partition_and_apfs_names_map_to_their_whole_disk() {
        assert_eq!(whole_disk("disk5s1"), "disk5");
        assert_eq!(whole_disk("disk3s1s1"), "disk3");
        assert_eq!(whole_disk("disk12"), "disk12");
        assert_eq!(whole_disk("disk12s3"), "disk12");
        assert!(raw_device("disk1s2").is_ok());
        assert!(raw_device("disk1").is_err());
        assert!(raw_device("../disk1s2").is_err());
    }

    #[test]
    fn serials_come_from_the_controller_listing_by_bsd_name() {
        let report = serde_json::json!({
            "SPNVMeDataType": [{"_items": [
                {"bsd_name": "disk0", "device_serial": "  74EXN0LIWOG103  "},
                {"bsd_name": "disk1", "device_serial": ""},
            ]}],
            "SPSerialATADataType": [{"_items": [{"bsd_name": "disk2", "device_serial": "WD-1"}]}],
        });
        assert_eq!(
            serial_of(&report, "disk0").as_deref(),
            Some("74EXN0LIWOG103")
        );
        assert_eq!(serial_of(&report, "disk1"), None);
        assert_eq!(serial_of(&report, "disk2").as_deref(), Some("WD-1"));
        assert_eq!(serial_of(&report, "disk9"), None);
        let media = |name: &str, whole: bool| {
            Value::Dictionary(
                [
                    ("BSD Name".to_owned(), Value::from(name)),
                    ("Whole".to_owned(), Value::from(whole)),
                ]
                .into_iter()
                .collect(),
            )
        };
        let device = |serial: &str, children: Vec<Value>| {
            Value::Dictionary(
                [
                    ("USB Serial Number".to_owned(), Value::from(serial)),
                    (
                        "IORegistryEntryChildren".to_owned(),
                        Value::Array(vec![Value::Dictionary(
                            [("IORegistryEntryChildren".to_owned(), Value::Array(children))]
                                .into_iter()
                                .collect(),
                        )]),
                    ),
                ]
                .into_iter()
                .collect(),
            )
        };
        let devices = Value::Array(vec![
            device("KEYBOARD", Vec::new()),
            device(
                " D036F68A3B1E6921 ",
                vec![media("disk5", true), media("disk5s1", false)],
            ),
        ]);
        assert_eq!(
            usb_serial_of(&devices, "disk5").as_deref(),
            Some("D036F68A3B1E6921")
        );
        assert_eq!(usb_serial_of(&devices, "disk5s1"), None);
        assert_eq!(usb_serial_of(&devices, "disk6"), None);
    }

    #[test]
    fn a_stick_is_named_by_its_disk_or_raw_disk_and_never_by_a_partition() {
        for typed in ["disk6", "/dev/disk6", "/dev/rdisk6", "rdisk6"] {
            assert_eq!(
                typed_disk(typed).unwrap().as_deref(),
                Some("disk6"),
                "{typed}"
            );
        }
        for typed in ["/Volumes/STELA", "STELA", "/dev/diskx", "disk", "/dev/sdb"] {
            assert_eq!(typed_disk(typed).unwrap(), None, "{typed}");
        }
        for typed in ["disk6s1", "/dev/rdisk6s2"] {
            let error = typed_disk(typed).unwrap_err().to_string();
            assert!(error.contains("is a partition"), "{error}");
            assert!(error.contains("disk6"), "{error}");
        }
    }
}
