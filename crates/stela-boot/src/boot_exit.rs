//! Leaving the firmware's boot services (`stela_boot_core::boot_exit`): the
//! final memory map's buffer, allocated before anything closes the
//! firmware's log, and the bounded retry, which never resets the PC. On a
//! final refusal the firmware's boot services still run: the boot stops
//! with the reason on the screen and in the firmware's variables
//! (`output::fatal`), which the next start copies into the USB's log.
use crate::{output, paging::PAGE};
use stela_boot_core::boot_exit::{self, Exited, Firmware, Map};
use uefi::{
    Status,
    boot::{self, AllocateType, MemoryType},
    mem::memory_map::{MemoryDescriptor, MemoryMapKey, MemoryMapMeta, MemoryMapRef},
};

/// The final memory map's buffer: LOADER_DATA pages Stela keeps for the
/// rest of the boot.
pub(crate) struct Buffer(&'static mut [u8]);

impl Buffer {
    /// Size the buffer from the map now (`boot_exit::buffer_bytes`) and take
    /// it. Called while boot services run, before `output::leave_boot_services`
    /// closes the firmware's log and timer.
    pub(crate) fn allocate() -> Self {
        let (_, size) = read_map(&mut []);
        let pages = boot_exit::buffer_bytes(size.bytes, size.descriptor_bytes).div_ceil(PAGE);
        let memory = boot::allocate_pages(AllocateType::AnyPages, MemoryType::LOADER_DATA, pages)
            .unwrap_or_else(|error| {
                output::fatal(format_args!(
                    "the firmware has no {pages} pages for its final memory map: {:?}",
                    error.status()
                ))
            });
        // SAFETY: fresh LOADER_DATA pages, owned by this buffer alone and
        // never freed: the final map lives in them for the rest of the boot.
        Self(unsafe { core::slice::from_raw_parts_mut(memory.as_ptr(), pages * PAGE) })
    }
}

/// `GetMemoryMap` into `buffer`; with an empty one, the map's size alone.
fn read_map(buffer: &mut [u8]) -> (Status, Map) {
    let mut map = Map {
        bytes: buffer.len(),
        key: 0,
        descriptor_bytes: 0,
        version: 0,
    };
    let Some(system) = uefi::table::system_table_raw() else {
        return (Status::NOT_READY, map);
    };
    // SAFETY: the firmware's system table; every caller runs before a
    // successful exit, while its boot services table is live.
    let services = unsafe { system.as_ref().boot_services };
    if services.is_null() {
        return (Status::NOT_READY, map);
    }
    // SAFETY: as above; the buffer is this call's and `map.bytes` its length,
    // and page-aligned buffers suit the descriptors.
    let status = unsafe {
        ((*services).get_memory_map)(
            &mut map.bytes,
            buffer.as_mut_ptr().cast::<MemoryDescriptor>(),
            &mut map.key,
            &mut map.descriptor_bytes,
            &mut map.version,
        )
    };
    (status, map)
}

/// The firmware's boot services, called through its table: nothing may
/// allocate between a map and the exit that uses its key.
struct Services;

impl Firmware for Services {
    fn memory_map(&mut self, buffer: &mut [u8]) -> Result<Map, Status> {
        match read_map(buffer) {
            (status, map) if status == Status::SUCCESS => Ok(map),
            (status, _) => Err(status),
        }
    }

    fn exit(&mut self, key: usize) -> Status {
        let Some(system) = uefi::table::system_table_raw() else {
            return Status::NOT_READY;
        };
        // SAFETY: as in `read_map`: no exit has succeeded yet.
        let services = unsafe { system.as_ref().boot_services };
        if services.is_null() {
            return Status::NOT_READY;
        }
        // SAFETY: as above; this image's handle and the key of the map read
        // just before.
        unsafe { ((*services).exit_boot_services)(boot::image_handle().as_ptr(), key) }
    }
}

/// End the firmware's boot services with `buffer`, trying again with a
/// fresh map when the firmware refuses (`boot_exit::exit`), and return the
/// final map. A final refusal stops the boot with its reason.
///
/// # Safety
///
/// As `uefi::boot::exit_boot_services`: no protocol, event or pool
/// allocation outlives the call, and the firmware's log and console are
/// closed (`output::leave_boot_services`).
pub(crate) unsafe fn exit(buffer: Buffer) -> MemoryMapRef<'static> {
    let Buffer(buffer) = buffer;
    let Exited { map, attempts } = boot_exit::exit(&mut Services, buffer)
        .unwrap_or_else(|refused| output::fatal(format_args!("{refused}")));
    let meta = MemoryMapMeta {
        map_size: map.bytes,
        desc_size: map.descriptor_bytes,
        map_key: MemoryMapKey::default(),
        desc_version: map.version,
    };
    let capacity = buffer.len() / map.descriptor_bytes;
    output::arguments(format_args!(
        "FIRMWARE boot services ended at try {attempts} of {}, map {} of {capacity} descriptors\n",
        boot_exit::ATTEMPTS,
        map.bytes / map.descriptor_bytes,
    ));
    // `boot_exit::exit` checked the sizes `new` checks, and the buffer is
    // page-aligned.
    MemoryMapRef::new(buffer, meta).unwrap_or_else(|error| {
        output::fatal(format_args!("the final memory map is unusable: {error}"))
    })
}
