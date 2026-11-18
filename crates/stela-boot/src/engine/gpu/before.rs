//! What each stack's kernel driver needs of the boot before Linux sees its
//! function: what identifies the device to its firmware files, the files
//! themselves, and what must have settled after the function's reset. The
//! claim's own steps (identity, the firmware's screen, BARs, decoding, the
//! reset and the private domain, MSI) are the same for every stack.
use super::{PAGE, output};
use alloc::{format, string::String, sync::Arc, vec::Vec};
use stela_boot_core::opregion;
use stela_crypto::{Sha256, digest::Digest};
use stela_drivers::{
    bus::{
        mmio::owned::{self, CacheAttribute, Mapping, Protection},
        pci::{ConfigSpace, Function, owned::OwnedConfig},
    },
    firmware::{
        self, Catalog, ErrorKind as FirmwareError, Image, Provenance, Provider, Requirement,
        Resource,
    },
    gpu::{self, Vendor, stack::GpuStack},
    probe::{NativeServices, ServiceError},
};
use stela_linux::catalogue::{FirmwareTarget, PciFirmware};
use stela_loader::host_firmware::requirements::Inventory;

/// What a stack's part found on the function before its reset.
pub(super) struct Identified {
    /// The files the driver may ask for, each under the name it asks by.
    pub(super) firmware: PciFirmware,
    /// Register 0 where it identifies the chip, so it reads the same after
    /// the reset.
    pub(super) identity: Option<u32>,
    /// The chip or platform as people know it.
    pub(super) chip: Option<&'static str>,
    /// Words of Linux's command line (`module.parameter=value`) the driver
    /// needs set for this device before it registers.
    pub(super) parameters: Option<String>,
    /// Other functions the driver's probe searches for and reads the
    /// identity of. Linux is shown them for searches only.
    pub(super) searched: Vec<stela_linux::function::Identity>,
}

/// The function as a stack's part may read it before its reset.
pub(super) struct Observed<'a> {
    pub(super) services: &'a dyn NativeServices,
    pub(super) adapter: &'a gpu::Adapter,
    /// The PCI-to-ISA bridges the probe read.
    pub(super) isa_bridges: &'a [stela_drivers::probe::IsaBridge],
    /// Its configuration, for reads only.
    pub(super) function: &'a mut Function<OwnedConfig>,
    /// The first page of the stack's register BAR, mapped read-only.
    pub(super) registers: &'a Mapping,
}

/// One stack's part of the claim.
pub(super) struct Before {
    /// What the boot can check from the function's PCI identity alone, before
    /// the claim changes anything on the function or takes the firmware's
    /// screen: that the driver's files for the device are pinned and on the
    /// USB. A GPU refused here keeps the firmware's picture.
    pub(super) admits: fn(&gpu::Adapter, &Catalog) -> Result<(), ServiceError>,
    /// Reads what selects the driver's firmware, through the first page of
    /// the stack's register BAR mapped read-only before the reset, and
    /// selects it.
    pub(super) identify: fn(Observed<'_>, Arc<Catalog>) -> Result<Identified, ServiceError>,
    /// Waits, after the reset, for what the driver's probe expects to find
    /// done. The register BAR is the resource.
    pub(super) settle:
        fn(&dyn NativeServices, &owned::Resource, &Identified) -> Result<(), ServiceError>,
}

/// The part for `stack`'s kernel driver, if the boot has one.
pub(super) fn of(stack: &GpuStack) -> Option<&'static Before> {
    match stack.vendor {
        Vendor::Nvidia => Some(&NOUVEAU),
        Vendor::Amd => Some(&AMDGPU),
        Vendor::Intel => Some(&XE),
        _ => None,
    }
}

/// nouveau: the chip is `PMC_BOOT_0`'s, its GSP-RM files are that chip's, and
/// a Turing-to-Ada GPU's own firmware must have booted again after the reset.
static NOUVEAU: Before = Before {
    // Its files are the chip's, which only its registers name.
    admits: |_, _| Ok(()),
    identify: |observed, catalog| {
        let registers = observed.registers;
        let endian = registers.read32(4).map_err(ServiceError::Mapping)?;
        if endian == u32::MAX || endian == 0x01000001 {
            return Err(ServiceError::Refused(
                "the GPU register byte order is unavailable",
            ));
        }
        let boot0 = registers.read32(0).map_err(ServiceError::Mapping)?;
        let chip = gpu::nouveau_chip::Chip::from_boot0(boot0).ok_or(ServiceError::Refused(
            "the GPU's PMC chipset is unsupported",
        ))?;
        let name = chip.name.to_ascii_lowercase();
        let mut inventory = Inventory::default();
        stela_loader::host_firmware::recipes::require_nvidia(&mut inventory, &name)
            .map_err(|_| ServiceError::Refused("the GPU has no pinned firmware recipe"))?;
        Ok(Identified {
            firmware: select_firmware(&inventory, &name, catalog, None)?,
            identity: Some(boot0),
            chip: Some(chip.name),
            parameters: None,
            searched: Vec::new(),
        })
    },
    settle: |services, registers, identified| match identified
        .identity
        .and_then(gpu::nouveau_chip::Chip::from_boot0)
        .filter(|chip| chip.waits_for_firmware_boot())
    {
        Some(chip) => wait_for_firmware_boot(services, registers, chip),
        None => Ok(()),
    },
};

/// amdgpu: the files are those its IP blocks ask for on the PCI device ID's
/// GPU, as pinned. The driver reads which blocks the device has from the
/// device's own discovery table and loads every engine's firmware itself,
/// so nothing boots by itself after the reset and nothing waits here.
static AMDGPU: Before = Before {
    admits: |adapter, catalog| {
        on_usb(
            &amdgpu_files(adapter.device)?,
            &format!("{:04x}", adapter.device),
            catalog,
        )
    },
    identify: |observed, catalog| {
        use stela_loader::host_firmware::recipes;
        let adapter = observed.adapter;
        // Register 0 identifies nothing here; the read proves the window.
        observed
            .registers
            .read32(0)
            .map_err(ServiceError::Mapping)?;
        let inventory = amdgpu_files(adapter.device)?;
        let name = format!("{:04x}", adapter.device);
        Ok(Identified {
            firmware: select_firmware(&inventory, &name, catalog, None)?,
            identity: None,
            chip: recipes::amd_gpu_family(adapter.device),
            parameters: None,
            searched: Vec::new(),
        })
    },
    settle: |_, _, _| Ok(()),
};

/// xe: the platform is the PCI device ID's, as in xe's own table, and its
/// GuC, HuC, GSC and DMC files are that platform's. Nothing of the GPU's
/// boots by itself after a reset: xe loads all of it. The display code's
/// Video BIOS Table is the one file the PC itself holds.
static XE: Before = Before {
    admits: |adapter, catalog| {
        on_usb(
            &xe_files(adapter.device)?,
            &format!("{:04x}", adapter.device),
            catalog,
        )
    },
    identify: |mut observed, catalog| {
        use stela_loader::host_firmware::recipes;
        let adapter = observed.adapter;
        // Register 0 identifies nothing here; the read proves the window.
        observed
            .registers
            .read32(0)
            .map_err(ServiceError::Mapping)?;
        let inventory = xe_files(adapter.device)?;
        let target = inventory
            .requests()
            .first()
            .ok_or(ServiceError::Refused(
                "the GPU has no pinned firmware recipe",
            ))?
            .view()
            .target;
        let table = video_bios_table(&mut observed, target);
        let name = format!("{:04x}", adapter.device);
        Ok(Identified {
            firmware: select_firmware(&inventory, &name, catalog, table.map(|table| (table, VBT)))?,
            identity: None,
            chip: recipes::intel_gpu_platform(adapter.device),
            // The display code asks the firmware service for the table this
            // parameter names before it looks anywhere else (`intel_bios.c`).
            parameters: table.map(|_| format!("xe.vbt_firmware={VBT}")),
            searched: chipset_bridges(&observed),
        })
    },
    settle: |_, _, _| Ok(()),
};

/// The PCI-to-ISA bridges of the GPU's segment, with one line each. The
/// display code names the PCH by the bridge's IDs (`intel_pch.c` searches
/// the class) and takes its panel power sequencer and port map from that;
/// with none it detects no PCH.
fn chipset_bridges(observed: &Observed<'_>) -> Vec<stela_linux::function::Identity> {
    let address = observed.adapter.address;
    let bridges: Vec<_> = observed
        .isa_bridges
        .iter()
        .filter(|bridge| bridge.identity.address.segment == address.segment)
        .collect();
    if bridges.is_empty() {
        output::arguments(format_args!(
            "GPU {address}: no PCI-to-ISA bridge was found, so the display code detects no PCH\n"
        ));
    }
    bridges
        .into_iter()
        .map(|bridge| {
            let (at, class) = (bridge.identity.address, bridge.identity.class);
            output::arguments(format_args!(
                "GPU {address}: PCI-to-ISA bridge {at} {:04x}:{:04x} is shown to xe for its \
                 chipset search\n",
                bridge.identity.vendor, bridge.identity.device
            ));
            stela_linux::function::Identity {
                bus: u32::from(at.bus),
                devfn: u32::from(at.device << 3 | at.function),
                vendor: bridge.identity.vendor,
                device: bridge.identity.device,
                subsystem_vendor: bridge.subsystem_vendor,
                subsystem_device: bridge.subsystem_device,
                class_revision: u32::from_be_bytes([class[0], class[1], class[2], bridge.revision]),
            }
        })
        .collect()
}

/// The name the display code asks for the PC's Video BIOS Table by.
const VBT: &str = "intel/vbt.bin";

/// The Video BIOS Table of the OpRegion the function's `ASLS` register
/// points at, as an image of `target`, and one line saying where it came
/// from or why there is none. Linux reads it through its ACPI OpRegion
/// driver, which Stela's Linux profile does not build; the OpRegion is plain
/// firmware memory, read here through its owner.
fn video_bios_table(
    observed: &mut Observed<'_>,
    target: firmware::Target<'_>,
) -> Option<Image<'static>> {
    let address = observed.adapter.address;
    let absent = |why: &dyn core::fmt::Display| {
        output::arguments(format_args!(
            "GPU {address}: no Video BIOS Table from the OpRegion ({why}): panels are found \
             by probing only\n"
        ));
    };
    let asls = match observed.function.config().read32(opregion::ASLS) {
        Ok(asls) => asls,
        Err(error) => {
            absent(&format_args!("its address could not be read: {error:?}"));
            return None;
        }
    };
    let table = match opregion::read(u64::from(asls), |physical, into| {
        observed.services.read_firmware_memory(physical, into)
    }) {
        Ok(table) => table,
        Err(why) => {
            absent(&why);
            return None;
        }
    };
    let (bytes, place, (major, minor)) = (table.bytes.len(), table.place, table.version);
    let revision = format!("{major}.{minor}");
    let catalog = Catalog::from_handoff(alloc::vec![Resource {
        name: VBT,
        target,
        provenance: Provenance {
            source: "ACPI OpRegion",
            revision: &revision,
            member: "vbt",
            license: "the PC's firmware",
        },
        expected_length: bytes,
        expected_sha256: Sha256::digest(&table.bytes).into(),
        bytes: table.bytes,
    }]);
    // The table is held for the boot, like the stored catalog's images.
    let image = catalog.ok().and_then(|catalog| {
        let held: &'static Catalog = alloc::boxed::Box::leak(alloc::boxed::Box::new(catalog));
        held.load(Requirement {
            name: VBT,
            target,
            minimum_bytes: 1,
        })
        .ok()
    });
    match image {
        Some(_) => output::arguments(format_args!(
            "GPU {address}: Video BIOS Table of {bytes} bytes from the OpRegion's {place} \
             (OpRegion {major}.{minor})\n"
        )),
        None => absent(&"it cannot be held as a firmware image"),
    }
    image
}

/// The files amdgpu asks for on the AMD GPU with PCI device ID `device`.
fn amdgpu_files(device: u16) -> Result<Inventory, ServiceError> {
    let mut inventory = Inventory::default();
    stela_loader::host_firmware::recipes::require_amdgpu(&mut inventory, device).map_err(|_| {
        ServiceError::Refused(
            "Stela pins amdgpu's firmware only for Ryzen 7000 and 9000 desktop graphics, \
                 not for this GPU",
        )
    })?;
    Ok(inventory)
}

/// The files xe and the display code ask for on the Intel GPU with PCI
/// device ID `device`.
fn xe_files(device: u16) -> Result<Inventory, ServiceError> {
    let mut inventory = Inventory::default();
    stela_loader::host_firmware::recipes::require_xe(&mut inventory, device)
        .map_err(|_| ServiceError::Refused("the GPU has no pinned xe firmware recipe"))?;
    Ok(inventory)
}

/// The image `selection` names in `stored`: its exact binding, else the
/// bundle's file by its pinned digest.
fn find<'a>(
    stored: &'a Catalog,
    selection: &stela_loader::host_firmware::recipes::Selection,
) -> Result<Image<'a>, firmware::Error> {
    use stela_loader::host_firmware::bundle;
    let request = &selection.request;
    match stored.load(request.view()) {
        Err(error) if error.kind == FirmwareError::Missing => {
            let name = bundle::name(selection.recipe.image.sha256);
            match stored.load(Requirement {
                name: &name,
                target: bundle::TARGET,
                minimum_bytes: request.minimum_bytes,
            }) {
                Err(stored_error) if stored_error.kind == FirmwareError::Missing => Err(error),
                other => other,
            }
        }
        other => other,
    }
}

/// Whether every file of `inventory` the driver needs is in `catalog`,
/// before anything of the GPU `name` changes: one line naming the files
/// missing and why ([`lacks`]), and the refusal [`select_firmware`] would
/// give for them later.
/// A file the driver starts without (`recipes::optional`) may be missing.
fn on_usb(inventory: &Inventory, name: &str, catalog: &Catalog) -> Result<(), ServiceError> {
    use stela_loader::host_firmware::recipes;
    let plan = recipes::Plan::select(inventory)
        .map_err(|_| ServiceError::Refused("the GPU firmware recipe cannot be selected"))?;
    let missing: Vec<_> = plan
        .selections()
        .iter()
        .filter(|selection| {
            !recipes::optional(&selection.request)
                && find(catalog, selection).is_err_and(|error| error.kind == FirmwareError::Missing)
        })
        .map(|selection| {
            (
                selection.request.name.clone(),
                selection.recipe.image.sha256,
            )
        })
        .collect();
    if missing.is_empty() {
        return Ok(());
    }
    Err(lacks(name, &missing, plan.selections().len()))
}

/// Says which of the `files` files of the GPU `name` are `missing`, each
/// named with its digest, and why, by what the loader recorded of the USB's
/// bundle ([`crate::host_firmware::bundle_held`]); the refusal that says it
/// on the screen.
fn lacks(name: &str, missing: &[(String, &str)], files: usize) -> ServiceError {
    use stela_loader::host_firmware::bundle::{Absent, Held};
    let held: Vec<Held> = missing
        .iter()
        .map(|(_, sha256)| crate::host_firmware::bundle_held(sha256))
        .collect();
    let names = missing
        .iter()
        .map(|(name, _)| name.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    let count = missing.len();
    match Absent::of(held.iter().copied()) {
        Absent::NotOnUsb => {
            output::arguments(format_args!(
                "GPU {name}: the USB's firmware bundle lacks {count} of this chip's {files} files \
                 ({names}); `stela repair` completes it\n"
            ));
            ServiceError::Refused(NO_FIRMWARE)
        }
        Absent::ReadFailed => {
            output::arguments(format_args!(
                "GPU {name}: the USB's firmware bundle holds {count} of this chip's {files} files \
                 ({names}) that were not in memory: reading them before the firmware's exit \
                 failed, as the HOST FIRMWARE BUNDLE LACKS lines say\n"
            ));
            ServiceError::Refused(FIRMWARE_READ_FAILED)
        }
        Absent::AnsweredLate => {
            let why = if held.contains(&Held::NoRoom) {
                "the files read for such a GPU did not fit the firmware handoff"
            } else {
                "the firmware showed a GPU of this family Stela drives or no PCI port with \
                 nothing below it, so none was expected"
            };
            output::arguments(format_args!(
                "GPU {name}: the USB's firmware bundle holds {count} of this chip's {files} files \
                 ({names}), but they were not read: the firmware did not show this GPU before \
                 its exit, so it answered only after that, and {why}\n"
            ));
            ServiceError::Refused(FIRMWARE_NOT_READ)
        }
    }
}

/// The firmware of every request in `inventory`, each file bound under the
/// exact name and target its driver asks for. A driver needs every file of
/// its recipe but those it starts without (`recipes::optional`): one of the
/// others missing fails its start later with a generic error, so the GPU is
/// refused; an optional one missing is said once and its request answers
/// that the file does not exist, as on Linux.
/// `read` is an image the boot read from the PC itself and the name the
/// driver asks for it by, bound beside the files under the same target.
fn select_firmware(
    inventory: &Inventory,
    name: &str,
    catalog: Arc<Catalog>,
    read: Option<(Image<'static>, &str)>,
) -> Result<PciFirmware, ServiceError> {
    use stela_loader::host_firmware::recipes;
    let mut names = Vec::new();
    let request = inventory.requests().first().ok_or(ServiceError::Refused(
        "the GPU has no pinned firmware recipe",
    ))?;
    let target = FirmwareTarget::new(
        request.target.driver.clone(),
        request.target.hardware.clone(),
        request.target.revision.clone(),
        request.target.board.clone(),
    )
    .map_err(|_| ServiceError::Refused("the GPU firmware target is invalid"))?;
    let plan = recipes::Plan::select(inventory)
        .map_err(|_| ServiceError::Refused("the GPU firmware recipe cannot be selected"))?;
    // SAFETY: the reference count is never given back, so the catalog
    // stays allocated for every image borrowed from it for the boot.
    let stored: &'static Catalog = unsafe { &*Arc::into_raw(catalog.clone()) };
    let mut bindings = Vec::new();
    let mut missing = Vec::new();
    for selection in plan.selections() {
        let request = &selection.request;
        let found = find(stored, selection);
        if let Ok(image) = found {
            output::arguments(format_args!(
                "GPU {name}: firmware {} of {} bytes\n",
                request.name,
                image.bytes().len()
            ));
            bindings.push((image, request));
        }
        match found {
            Ok(image) => {
                selection
                    .recipe
                    .image
                    .verify_digest(image.bytes().len(), image.sha256())
                    .map_err(|_| {
                        ServiceError::Refused(
                            "the GPU firmware digest does not match its pinned recipe",
                        )
                    })?;
            }
            Err(error) if error.kind == FirmwareError::Missing && recipes::optional(request) => {
                output::arguments(format_args!(
                    "GPU {name}: firmware {} is not on the USB; its driver starts without it\n",
                    request.name
                ));
            }
            Err(error) if error.kind == FirmwareError::Missing => {
                missing.push((request.name.clone(), selection.recipe.image.sha256));
            }
            Err(error) => {
                output::arguments(format_args!(
                    "GPU {name}: firmware {} refused: {error:?}\n",
                    request.name
                ));
                return Err(ServiceError::Refused(
                    "the GPU firmware does not match its exact catalogue target",
                ));
            }
        }
        names.push(request.name.clone());
    }
    if !missing.is_empty() {
        return Err(lacks(name, &missing, names.len()));
    }
    names.extend(read.iter().map(|(_, name)| String::from(*name)));
    // Each file under the exact name and target the driver asks for.
    let catalog = Catalog::rebind_all(
        bindings
            .iter()
            .map(|(image, request)| {
                let view = request.view();
                (*image, view.name, view.target)
            })
            .chain(
                read.iter()
                    .map(|(image, name)| (*image, *name, request.view().target)),
            ),
    )
    .map_err(|_| ServiceError::Refused("the GPU firmware cannot be bound to its requests"))?;
    PciFirmware::new(target, Arc::new(catalog), names.into_boxed_slice())
        .map_err(|_| ServiceError::Refused("the GPU firmware image names are invalid"))
}

fn read_register(
    services: &dyn NativeServices,
    registers: &owned::Resource,
    offset: usize,
) -> Option<u32> {
    let page = offset & !(PAGE as usize - 1);
    let mut window = services
        .authority()
        .ok()?
        .map(
            registers.clone(),
            page,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )
        .ok()?;
    let value = window.read32(offset - page).ok();
    // SAFETY: the read above was this scoped mapping's only use.
    unsafe { window.retire() }.ok()?;
    value
}

/// After the claim's function-level reset a Turing-to-Ada GPU's own firmware
/// runs devinit again. nouveau reads the VBIOS before it waits for that
/// (`firmware_boot`), so wait here first, by nouveau's bound, and print what
/// the start of the old boot flow depends on: the firmware boot, WPR2 (which
/// FWSEC-FRTS sets) and the VBIOS window the PRAMIN shadow reads.
/// `GPU AD107: firmware boot completed after N polls: gfw=… plm=… wpr2=…-…
/// display_fuse=… vbios_window=…`, or `did not complete` and a refusal.
fn wait_for_firmware_boot(
    services: &dyn NativeServices,
    registers: &owned::Resource,
    chip: &gpu::nouveau_chip::Chip,
) -> Result<(), ServiceError> {
    use gpu::nouveau_chip::firmware_boot::{self, Error as BootError};
    let page = firmware_boot::PRIV_LEVEL_MASK & !(PAGE as usize - 1);
    let mut window = services
        .authority()?
        .map(
            registers.clone(),
            page,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )
        .map_err(ServiceError::Mapping)?;
    let timer = services.minimum_delay()?;
    let result = firmware_boot::wait(
        |offset| {
            window
                .read32(offset - page)
                .map_err(|_| "a register read failed")
        },
        |micros| timer.wait_us(micros).map_err(|_| "the timer failed"),
    );
    // SAFETY: the polls above were this scoped mapping's only use.
    unsafe { window.retire() }.map_err(ServiceError::Mapping)?;
    let read = |offset| read_register(services, registers, offset);
    let hex =
        |value: Option<u32>| value.map_or_else(|| String::from("unread"), |v| format!("{v:#010x}"));
    // nouveau's PRAMIN shadow reads the window only with the display fused
    // on (`shadowramin.c` `pramin_init`: 0x820c04 from GA100, else 0x021c04).
    let fuse = read(match chip.architecture {
        gpu::nouveau_chip::Architecture::Pascal | gpu::nouveau_chip::Architecture::Turing => {
            0x02_1c04
        }
        _ => 0x82_0c04,
    });
    let window = fuse
        .filter(|fuse| fuse & 1 == 0)
        .and_then(|_| read(0x62_5f04));
    let state = format!(
        "wpr2={}-{} display_fuse={} vbios_window={}",
        hex(read(0x1f_a824)),
        hex(read(0x1f_a828)),
        hex(fuse),
        hex(window),
    );
    match result {
        Ok(reading) => {
            output::arguments(format_args!(
                "GPU {}: firmware boot completed after {} polls: gfw={:#010x} plm={:#010x} {state}\n",
                chip.name, reading.polls, reading.progress, reading.priv_level_mask
            ));
            Ok(())
        }
        Err(BootError::Timeout(reading)) => {
            output::arguments(format_args!(
                "GPU {}: firmware boot did not complete after its reset in {} polls of {} us: \
                 gfw={:#010x} (expected low byte {:#04x}) plm={:#010x} (expected bit 0) {state}\n",
                chip.name,
                reading.polls,
                firmware_boot::POLL_MICROS,
                reading.progress,
                firmware_boot::COMPLETED,
                reading.priv_level_mask
            ));
            Err(ServiceError::Refused(
                "the GPU's firmware boot did not complete after its reset",
            ))
        }
        Err(BootError::Access(error)) => {
            output::arguments(format_args!(
                "GPU {}: firmware boot wait stopped: {error}\n",
                chip.name
            ));
            Err(ServiceError::Refused(
                "the GPU's firmware boot registers could not be read",
            ))
        }
    }
}

/// Why a GPU was refused when the USB lacks any of its chip's firmware files.
pub(super) const NO_FIRMWARE: &str = "no pinned firmware for the observed GPU chipset was loaded";
/// Why a GPU was refused when the USB holds its chip's files and reading
/// some of them before the firmware's exit failed.
pub(super) const FIRMWARE_READ_FAILED: &str =
    "the USB holds the GPU's firmware, but reading it before the firmware's exit failed";
/// Why a GPU was refused when the USB holds its chip's files and they were
/// not read, the GPU having answered only after the firmware's exit.
pub(super) const FIRMWARE_NOT_READ: &str = "the USB holds the GPU's firmware, but it was not read: the GPU answered only after the \
     firmware's exit";
