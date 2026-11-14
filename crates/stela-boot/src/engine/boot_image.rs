//! Load a verified bundle or raw guest image before hardware construction.
use super::*;
use stela_formats::boot_bundle::Nvram;

/// Entry state and bundle provenance kept through machine construction.
pub(super) struct GuestImage {
    pub entry: u64,
    pub x0: u64,
    pub read_only_region: Option<(u64, u64)>,
    pub core_locks: stela_engine::machine::ctrr::CoreLocks,
    pub hidden_devices: Vec<String>,
    pub live_tree: Option<bundle::live_tree::LiveTree>,
    pub source: Option<bundle::source::Intake>,
    pub boot_processor: Option<usize>,
    pub hidden_clpc: Vec<String>,
}

/// Load the image while the caller owns guest RAM and the loader's files.
pub(super) fn load(
    host: &mut impl Host,
    handoff: &Handoff,
    options: &Options,
    built: Option<&[(String, Vec<u8>)]>,
    nvram: Option<Nvram>,
) -> GuestImage {
    // A bundle uses the firmware's framebuffer mode.
    let framebuffer = handoff
        .screen
        .filter(|_| options.bundle)
        .and_then(|screen| {
            display::boot_framebuffer(screen.resolution(), options.dram_base + options.ram_bytes)
        });
    if options.bundle {
        crate::status::begin("Loading macOS into memory");
        let files = &handoff.files[..handoff.file_count];
        let dram = bundle::Dram {
            base: options.dram_base,
            backed: options.ram_bytes,
            installed: options.dram_bytes,
        };
        let loaded = match built {
            Some(built) => bundle::load_built(host, built, dram, framebuffer, nvram),
            None => bundle::load(host, files, dram, framebuffer, nvram),
        };
        let loaded = match loaded {
            Ok(loaded) => loaded,
            Err(error) => {
                output::fatal(format_args!("the boot bundle could not be loaded: {error}"))
            }
        };
        output::arguments(format_args!(
            "BUNDLE regions={} patched={} kept={:?} dram_base={:#x}\n",
            loaded.regions, loaded.patches, loaded.kept, loaded.dram_base
        ));
        let image = GuestImage {
            entry: loaded.pc,
            x0: loaded.x0,
            read_only_region: loaded.read_only_region,
            core_locks: loaded.core_locks,
            hidden_devices: loaded.hidden_devices,
            live_tree: Some(loaded.live_tree),
            source: Some(loaded.source),
            boot_processor: loaded.boot_processor,
            hidden_clpc: loaded.hidden_clpc,
        };
        crate::status::done("");
        image
    } else {
        // SAFETY: the loader keeps this source allocation in the shared physmap.
        let source = unsafe {
            core::slice::from_raw_parts(
                layout::physical(handoff.source.physical) as *const u8,
                handoff.source.bytes as usize,
            )
        };
        if !host.write_ram(options.load, source) {
            output::fatal("guest image does not fit guest DRAM at its load address");
        }
        output::arguments(format_args!(
            "GUEST IMAGE pa={:#x} bytes={} sha256={}\n",
            options.load,
            source.len(),
            stela_formats::sha256(source),
        ));
        GuestImage {
            entry: options.entry,
            x0: options.x0,
            read_only_region: None,
            core_locks: stela_engine::machine::ctrr::CoreLocks::default(),
            hidden_devices: Vec::new(),
            live_tree: None,
            source: None,
            boot_processor: None,
            hidden_clpc: Vec::new(),
        }
    }
}
