//! Firmware reservations and page-table construction for the engine handoff.
use super::*;
use crate::Result;
use stela_contracts::refuse;

/// [`allocate`] aligned as [`tables::LARGE`], so that its mapping can take
/// large pages.
fn allocate_large(bytes: u64) -> Result<Region> {
    let pages = (bytes + tables::LARGE).div_ceil(PAGE) as usize;
    Ok(Region {
        physical: (firmware::pages(pages)? as u64).next_multiple_of(tables::LARGE),
        bytes,
    })
}

/// The relocated engine image and the host ranges its mappings must reach.
pub(super) struct EngineImage {
    pub image: firmware::Image,
    pub engine: Region,
    pub resident: crate::native_image::ResidentImage,
    pub ram: Vec<(usize, usize)>,
    pub screen: Option<Framebuffer>,
    pub apic: u64,
    pub placement: Placement,
}

/// Firmware-owned allocations kept by the engine and its native workers.
pub(super) struct EngineMemory {
    pub heap: Region,
    pub native_resources: NativeResources,
    pub native_task_allocations: crate::native_smp::resources::TaskAllocations,
    pub stack: Region,
    pub code: Region,
    pub pool: Region,
    pub handoff_page: Region,
    pub trampoline: u64,
    pub root: u64,
    pub loader_tables: crate::paging::loader::Builder,
}

/// Copy and relocate the pristine image into the selected host window.
pub(super) fn prepare_image(snapshot: &Snapshot) -> Result<EngineImage> {
    let image = firmware::Image::loaded()?;
    if image.start != snapshot.start || image.end - image.start != snapshot.bytes {
        return Err(refuse!("image layout changed after the snapshot"));
    }
    let ram = firmware::ram()?;
    let screen = Framebuffer::for_engine()?;
    let apic_base = crate::apic::base_msr();
    // The physmap reaches RAM, the framebuffer and the local APIC.
    let apic = apic_base & crate::apic::BASE_ADDRESS;
    let top = ram
        .iter()
        .map(|r| r.1 as u64)
        .chain(screen.iter().map(|s| (s.base + s.bytes) as u64))
        .chain(
            screen
                .iter()
                .filter_map(|s| s.flip)
                .map(|r| r as u64 + PAGE),
        )
        .chain(core::iter::once(apic + PAGE))
        .max()
        .unwrap_or(0);
    if top > 1 << crate::cpu::physical_bits() {
        return Err(refuse!(
            "the firmware placed memory above the address width AMD memory encryption leaves"
        ));
    }
    let placement = layout::choose(crate::cpu::LA57.present(), top).ok_or(refuse!(
        "host physical addresses lie beyond what any placement's physmap reaches"
    ))?;
    layout::set(placement);
    let resident = crate::native_image::ResidentImage::allocate(snapshot.bytes, layout::image())
        .map_err(|error| refuse!("engine image allocation failed", %error))?;
    let engine = resident.region();
    // SAFETY: the pristine source and the newly owned, zeroed image allocation
    // are distinct and each covers exactly snapshot.bytes.
    unsafe {
        core::ptr::copy_nonoverlapping(
            snapshot.copy.as_ptr(),
            engine.physical as *mut u8,
            snapshot.bytes,
        );
    }
    relocate(
        engine.physical as usize,
        snapshot.bytes,
        layout::image().wrapping_sub(image.start as u64),
    )?;
    Ok(EngineImage {
        image,
        engine,
        resident,
        ram,
        screen,
        apic,
        placement,
    })
}

/// Reserve the engine's working memory and each physical worker's resources.
/// The other cores start from a page below 1 MiB, taken first; firmware
/// that left none free boots on one core.
pub(super) fn reserve(cores: usize, ram: &[(usize, usize)]) -> Result<EngineMemory> {
    let (cores, sipi) = match cores {
        1 => (1, 0),
        _ => match sipi_page() {
            Some(page) => (cores, page),
            None => {
                crate::status::info(&format!(
                    "the firmware left no free memory below 1 MiB to start the other cores from, \
                     so macOS runs on one of this PC's {cores}"
                ));
                (1, 0)
            }
        },
    };
    let free = free_ram()?;
    let sizes = stela_boot_core::host_memory::sizes(free, cores, cores)?;
    output::arguments(format_args!(
        "HOST MEMORY free={}M heap={}M code={}M pool={}M guest={}M\n",
        free >> 20,
        sizes.heap >> 20,
        sizes.code.bytes >> 20,
        sizes.pool >> 20,
        sizes.guest >> 20,
    ));
    // Each claimed core owns a full native part. Check both ends before allocation.
    for slot in [0, cores - 1] {
        let part = crate::native_smp::resources::partition(
            cores,
            cores,
            slot,
            sizes.code.bytes,
            sizes.pool,
        )?;
        crate::native_smp::resources::code_part_bytes(part.code_bytes, sizes.code.parts)?;
    }
    let heap = allocate(sizes.heap)?;
    // The engine allocator uses this existing physmap directly. Prove the
    // complete firmware reservation is covered before publishing the handoff.
    if !ram.iter().any(|&(start, end)| {
        start as u64 <= heap.physical
            && heap
                .physical
                .checked_add(heap.bytes)
                .is_some_and(|top| top <= end as u64)
    }) {
        return Err(refuse!("engine heap is outside the retained RAM physmap"));
    }
    let stacks = allocate(STACK_BYTES * cores as u64)?;
    let invocations = allocate(
        crate::native_smp::resources::invocation_stacks(cores)
            .map_err(|_| refuse!("invalid native invocation-stack geometry"))?
            .physical_bytes(),
    )?;
    use crate::native_smp::resources::TaskAllocations;
    let native_task_allocations = TaskAllocations::reserve(
        crate::native_smp::resources::task_stacks(layout::task_stacks())
            .map_err(|_| refuse!("task stack geometry overflow"))?
            .raw_allocation_bytes(),
    )
    .map_err(|error| {
        output::arguments(format_args!("TASK MEMORY allocation failed: {error}\n"));
        "firmware task allocation unavailable"
    })?;
    let native_resources = NativeResources {
        cores: cores as u32,
        code_parts: sizes.code.parts,
        stacks,
        invocations,
        tasks: native_task_allocations.region(),
        sipi,
        ..NativeResources::default()
    };
    let stack = Region {
        physical: native_resources.stacks.physical,
        bytes: STACK_BYTES,
    };
    let code = allocate_large(sizes.code.bytes)?;
    let pool = allocate(sizes.pool)?;
    // The handoff, then the loader's unstored lines (`Handoff::loader_lines`).
    let handoff_page =
        allocate((size_of::<Handoff>() + stela_boot_core::loader_lines::BYTES) as u64)?;
    let trampoline = low_page(MemoryType::LOADER_CODE)?;
    let loader_tables = crate::paging::loader::Builder::new()?;
    let root = loader_tables.root();
    Ok(EngineMemory {
        heap,
        native_resources,
        native_task_allocations,
        stack,
        code,
        pool,
        handoff_page,
        trampoline,
        root,
        loader_tables,
    })
}

/// The page APs start from (Intel SDM 3A 9.4.4: a SIPI vector names a page
/// below 1 MiB), above the real-mode vector table.
fn sipi_page() -> Option<u64> {
    let page = boot::allocate_pages(
        AllocateType::MaxAddress(0x9f000),
        MemoryType::LOADER_CODE,
        1,
    )
    .ok()?;
    let physical = page.as_ptr() as u64;
    if physical < PAGE {
        // SAFETY: the page just allocated, which nothing references.
        let _ = unsafe { boot::free_pages(page, 1) };
        return None;
    }
    // SAFETY: the allocation is owned by Stela through every AP's lifetime.
    unsafe {
        page.as_ptr().write_bytes(0, PAGE as usize);
    }
    Some(physical)
}

/// Build the identity root and shared engine window over those reservations.
pub(super) fn map(
    image: &EngineImage,
    memory: &mut EngineMemory,
    runtime: &Runtime,
    storage: Option<&stela_drivers::Handoff>,
    native_mmio: Option<&mut crate::native_mmio::Prepared>,
    host_firmware: &crate::host_firmware::Resources,
) -> Result<Storage> {
    let EngineImage {
        image,
        engine,
        ram,
        screen,
        apic,
        ..
    } = image;
    let EngineMemory {
        native_resources,
        code,
        trampoline,
        root,
        loader_tables,
        ..
    } = memory;
    let (root, trampoline, apic) = (*root, *trampoline, *apic);
    let cores = native_resources.cores as usize;
    let frames = loader_tables;
    let mut image_rights: Vec<(u64, u64, u64)> = image
        .sections
        .iter()
        .map(|&(start, end, rights)| (start as u64, end as u64, rights))
        .chain(core::iter::once((trampoline, trampoline + PAGE, PRESENT)))
        .chain(runtime.code())
        .collect();
    if native_resources.sipi != 0 {
        image_rights.push((native_resources.sipi, native_resources.sipi + PAGE, PRESENT));
    }
    identity(frames, root, ram, &image_rights)?;
    // The engine's drivers reach the DMA pages through the physmap, which
    // maps RAM alone: pages outside it would fault in a driver call.
    if let Some(storage) = storage
        && !storage.dma.is_empty()
        && !ram
            .iter()
            .any(|&(start, end)| start as u64 <= storage.dma.start && storage.dma.end <= end as u64)
    {
        return Err(refuse!(
            "host DMA pages lie outside the engine's RAM physmap"
        ));
    }
    if let Some(storage) = storage {
        map_devices(frames, root, ram, &storage.mapped)?;
    }
    let mapped = storage
        .as_ref()
        .map_or(&[][..], |storage| &storage.mapped[..]);
    runtime.map(frames, root, ram, mapped)?;
    let storage = match storage {
        Some(storage) => Storage::new(
            storage,
            crate::storage::captures(host_firmware.wifi_observations()),
        )?,
        None => Storage::absent(),
    };
    // The framebuffer is written, never read back, so its writes combine.
    let video = PRESENT | WRITABLE | NX | WRITE_COMBINING;
    if let Some(screen) = screen {
        let start = screen.base as u64 & !(PAGE - 1);
        let end = (screen.base + screen.bytes) as u64;
        tables::map_range(
            frames,
            root,
            start,
            start,
            end.next_multiple_of(PAGE) - start,
            video,
        )?;
        // The registers that flip between its frames are device registers.
        if let Some(registers) = screen.flip {
            let page = registers as u64 & !(PAGE - 1);
            tables::map_page(frames, root, page, page, PRESENT | WRITABLE | NX | UNCACHED)?;
        }
    }
    // The window: the engine image with section rights, heap, stack, code.
    let bytes = engine.bytes;
    let mut page = 0;
    while page < bytes {
        let rights = image.rights(image.start + page as usize);
        let run = (page..bytes)
            .step_by(PAGE as usize)
            .take_while(|&next| image.rights(image.start + next as usize) == rights)
            .count() as u64
            * PAGE;
        tables::map_range(
            frames,
            root,
            layout::image() + page,
            engine.physical + page,
            run,
            rights | tables::GLOBAL,
        )?;
        page += run;
    }
    let data = PRESENT | WRITABLE | NX;
    for slot in 0..cores {
        tables::map_range(
            frames,
            root,
            layout::stack() + slot as u64 * STACK_STRIDE + PAGE,
            native_resources.stacks.physical + slot as u64 * STACK_BYTES,
            STACK_BYTES,
            data | tables::GLOBAL,
        )?;
    }
    // These real LoaderData allocations outlive every root and continuation.
    // Map usable pages only; both guard pages of each slot remain absent.
    // This happens before AP/root clones and grants no execution capability.
    let geometry = crate::native_smp::resources::invocation_stacks(cores)
        .map_err(|_| refuse!("invalid native invocation-stack geometry"))?;
    for index in 0..geometry.count() {
        let slot = geometry
            .slot(layout::invocation_stacks(), index)
            .map_err(|_| refuse!("guarded stack virtual range overflow"))?;
        tables::map_range(
            frames,
            root,
            slot.usable.start,
            native_resources.invocations.physical
                + geometry
                    .physical_offset(index)
                    .map_err(|_| refuse!("guarded stack physical offset overflow"))?,
            slot.usable.end - slot.usable.start,
            data | tables::GLOBAL,
        )?;
    }
    let geometry = crate::native_smp::resources::task_stacks(layout::task_stacks())
        .map_err(|_| refuse!("task stack virtual range overflow"))?;
    let raw = native_resources.tasks;
    let end = raw
        .physical
        .checked_add(raw.bytes)
        .ok_or(refuse!("task stack reservation wraps"))?;
    if raw.bytes != geometry.raw_allocation_bytes() {
        return Err(refuse!("task stack reservation size changed"));
    }
    for index in 0..geometry.count() {
        let slot = geometry
            .slot(index)
            .map_err(|_| refuse!("task stack slot overflow"))?;
        let backing = geometry
            .backing(raw.physical..end, index)
            .map_err(|_| refuse!("task stack backing alignment overflow"))?;
        tables::map_range(
            frames,
            root,
            slot.usable.start,
            backing.start,
            backing.end - backing.start,
            data | tables::GLOBAL,
        )?;
    }
    tables::map_range(
        frames,
        root,
        layout::code(code.bytes),
        code.physical,
        code.bytes,
        PRESENT | tables::GLOBAL,
    )?;
    // The physmap: all RAM, the framebuffer and the local APIC.
    for &(start, end) in ram {
        let (start, end) = (start as u64, end as u64);
        tables::map_range(
            frames,
            root,
            layout::physical(start),
            start,
            end - start,
            data | tables::GLOBAL,
        )?;
    }
    if let Some(screen) = screen {
        let start = screen.base as u64 & !(PAGE - 1);
        let end = ((screen.base + screen.bytes) as u64).next_multiple_of(PAGE);
        if !ram.iter().any(|r| r.0 as u64 <= start && end <= r.1 as u64) {
            tables::map_range(
                frames,
                root,
                layout::physical(start),
                start,
                end - start,
                video,
            )?;
        }
        if let Some(registers) = screen.flip {
            let page = registers as u64 & !(PAGE - 1);
            tables::map_page(frames, root, layout::physical(page), page, data | UNCACHED)?;
        }
    }
    // The xAPIC page must be strong uncacheable (Intel SDM Vol. 3A 11.4.1);
    // the entry says so rather than relying on the firmware's MTRRs.
    if !ram.iter().any(|r| r.0 as u64 <= apic && apic < r.1 as u64) {
        tables::map_page(frames, root, layout::physical(apic), apic, data | UNCACHED)?;
    }
    // The bounce slot's leaf entries, which the translation helper points
    // at guest frames; until then they map a scratch page.
    for slot in 0..cores {
        for index in 0..2 {
            let scratch = allocate(PAGE)?.physical;
            let address = layout::bounce() + (slot * 2 + index) as u64 * PAGE;
            tables::map_page(frames, root, address, scratch, data)?;
            let entry = tables::entry(frames, root, address, 0, false)?;
            native_resources.bounce[slot][index] =
                entry.ok_or(refuse!("bounce slot unmapped"))? as u64;
        }
    }

    if let Some(prepared) = native_mmio {
        prepared.aperture(frames, root, cores)?;
    }

    Ok(storage)
}
