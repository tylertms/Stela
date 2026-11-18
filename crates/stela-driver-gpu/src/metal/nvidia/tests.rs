use super::{fixture::nvk, *};
use crate::metal::{
    abi::{
        ArgumentBuffersTier, CounterSamplingPoint, GpuFamily, ReadWriteTextureTier, SparsePageSize,
        TextureType,
    },
    capabilities::{
        ApiFeatures, Capabilities, Cause, FormatCapabilities as C, ShaderFeatures, Size,
    },
    open::PcieLink,
    pixel_format::PixelFormat as P,
};

const GEN4_X16: PcieLink = PcieLink {
    speed: 4,
    width: 16,
};

fn attachment() -> Attachment {
    Attachment {
        link: GEN4_X16,
        external: false,
        location_number: 0,
    }
}

fn with_api(chip: &Chip, api: ApiFeatures) -> Capabilities {
    Capabilities::derive(
        &nvk(chip, 16 << 30),
        &chip.platform(attachment()),
        ShaderFeatures::KNOWN,
        api,
    )
}

/// This build's API paths over a compiler with every shader feature.
fn complete(chip: &Chip) -> Capabilities {
    with_api(chip, ApiFeatures::IMPLEMENTED)
}

#[test]
fn the_chip_table_is_nouveaus_pascal_and_later_list() {
    assert!(
        CHIPS
            .windows(2)
            .all(|pair| pair[0].chipset < pair[1].chipset)
    );
    assert!(CHIPS.iter().all(|chip| chip.chipset >= 0x132));
    let headless: alloc::vec::Vec<_> = CHIPS
        .iter()
        .filter(|chip| !chip.display)
        .map(|chip| chip.name)
        .collect();
    assert_eq!(headless, ["GA100", "GH100", "GB100", "GB102"]);
    // GP106 (GTX 1060) is admitted under Pascal architecture.
    assert_eq!(Chip::find(0x136).map(|chip| chip.name), Some("GP106"));
    assert_eq!(
        Chip::find(0x136).map(|chip| chip.architecture),
        Some(Architecture::Pascal)
    );
    // The GTX 16 series (TU116, TU117) are also present.
    assert_eq!(Chip::find(0x168).map(|chip| chip.name), Some("TU116"));
    assert_eq!(
        Chip::find(0x167).map(|chip| chip.architecture),
        Some(Architecture::Turing)
    );
}

#[test]
fn admission_requires_an_nvk_discrete_gsp_chip() {
    let chip = Chip::find(0x1b3).unwrap();
    let identity = nvk(chip, 16 << 30).identity;
    assert_eq!(Chip::admit(&identity, 0x1b3), Ok(chip));
    let gp106 = Chip::find(0x136).unwrap();
    assert_eq!(Chip::admit(&identity, 0x136), Ok(gp106));
    assert_eq!(
        Chip::admit(&identity, 0x140),
        Err(Refusal::PreTuring { chipset: 0x140 })
    );
    assert_eq!(
        Chip::admit(&identity, 0x169),
        Err(Refusal::UnknownChipset { chipset: 0x169 })
    );

    let mut other = identity.clone();
    other.vendor_id = 0x1002;
    assert_eq!(
        Chip::admit(&other, 0x1b3),
        Err(Refusal::NotNvidia { vendor_id: 0x1002 })
    );
    let mut other = identity.clone();
    other.driver_id = DriverId::MesaVenus;
    assert_eq!(
        Chip::admit(&other, 0x1b3),
        Err(Refusal::NotNvk(DriverId::MesaVenus))
    );
    let mut other = identity;
    other.device_type = PhysicalDeviceType::IntegratedGpu;
    assert_eq!(
        Chip::admit(&other, 0x1b3),
        Err(Refusal::NotDiscrete(PhysicalDeviceType::IntegratedGpu))
    );
}

#[test]
fn link_bandwidth_follows_linux_encoding_rates() {
    // Link Status: speed in bits 0-3, width in bits 4-9.
    assert_eq!(PcieLink::from_link_status(0x0104), GEN4_X16);
    assert_eq!(
        PcieLink { speed: 1, width: 1 }.bytes_per_second(),
        250_000_000
    );
    assert_eq!(
        PcieLink {
            speed: 3,
            width: 16
        }
        .bytes_per_second(),
        15_752_000_000
    );
    assert_eq!(GEN4_X16.bytes_per_second(), 31_506_000_000);
    assert_eq!(
        PcieLink { speed: 6, width: 4 }.bytes_per_second(),
        32_000_000_000
    );
    assert_eq!(
        PcieLink {
            speed: 7,
            width: 16
        }
        .bytes_per_second(),
        0
    );
}

#[test]
fn platform_reports_placement_and_display_engine() {
    let ga100 = Chip::find(0x170).unwrap().platform(attachment());
    assert!(ga100.headless);
    assert_eq!(ga100.architecture, "GA100");
    assert_eq!(ga100.location, DeviceLocation::Slot);
    let egpu = Chip::find(0x194).unwrap().platform(Attachment {
        link: GEN4_X16,
        external: true,
        location_number: 2,
    });
    assert!(!egpu.headless && egpu.removable);
    assert_eq!(
        (egpu.location, egpu.location_number),
        (DeviceLocation::External, 2)
    );
}

#[test]
fn every_turing_and_later_chip_derives_the_same_metal_profile() {
    for chip in &CHIPS {
        let capabilities = complete(chip);
        assert_eq!(
            capabilities.families(),
            [
                GpuFamily::Apple1,
                GpuFamily::Apple2,
                GpuFamily::Apple3,
                GpuFamily::Apple4,
                GpuFamily::Apple5,
                GpuFamily::Apple6,
                GpuFamily::Apple7,
                GpuFamily::Common1,
                GpuFamily::Common2,
                GpuFamily::Common3,
                GpuFamily::Mac1,
                GpuFamily::Mac2,
                GpuFamily::MacCatalyst1,
                GpuFamily::MacCatalyst2,
                GpuFamily::Metal3,
            ],
            "{}",
            chip.name
        );
        assert_eq!(
            capabilities.argument_buffers_tier,
            ArgumentBuffersTier::Tier2
        );
        assert_eq!(
            capabilities.read_write_texture_tier,
            ReadWriteTextureTier::Tier2
        );
        assert_eq!(capabilities.limits.thread_execution_width, 32);
        assert_eq!(capabilities.limits.max_threadgroup_memory_length, 48 << 10);
        assert_eq!(
            capabilities.limits.max_threads_per_threadgroup,
            Size {
                width: 1024,
                height: 1024,
                depth: 64
            }
        );
        assert_eq!(capabilities.limits.max_buffer_length, 1 << 31);
        assert_eq!(capabilities.limits.max_texture_dimension_2d, 32768);
        assert_eq!(capabilities.headless, !chip.display);
    }
}

#[test]
fn nvidia_hardware_reaches_apple7_and_metal3_without_ray_tracing() {
    let capabilities = with_api(Chip::find(0x1b3).unwrap(), ApiFeatures::KNOWN);
    for family in [
        GpuFamily::Mac2,
        GpuFamily::Common3,
        GpuFamily::Apple7,
        GpuFamily::Metal3,
    ] {
        assert!(capabilities.supports_family(family), "{family:?}");
    }
    assert!(!capabilities.supports_family(GpuFamily::Apple8));
    let cause = |name: &str| {
        capabilities
            .gaps()
            .iter()
            .find(|gap| gap.capability == name)
            .map(|gap| gap.cause)
    };
    assert_eq!(cause("GPU family Metal3"), None);
    assert_eq!(cause("ray tracing"), Some(Cause::Driver));
    assert_eq!(
        cause("Apple GPU families from Apple8"),
        Some(Cause::AppleArchitecture)
    );
}

#[test]
fn common3_needs_gpu_encoded_indirect_commands() {
    let chip = Chip::find(0x1b3).unwrap();
    let capabilities = Capabilities::derive(
        &nvk(chip, 16 << 30),
        &chip.platform(attachment()),
        ShaderFeatures::KNOWN.difference(ShaderFeatures::INDIRECT_COMMAND_ENCODING),
        ApiFeatures::IMPLEMENTED,
    );
    assert!(!capabilities.supports_family(GpuFamily::Common3));
    let gap = capabilities
        .gaps()
        .iter()
        .find(|gap| gap.capability == "GPU-encoded indirect commands");
    assert_eq!(gap.map(|gap| gap.cause), Some(Cause::Compiler));
}

#[test]
fn sparse_textures_need_residency_feedback() {
    let chip = Chip::find(0x1b3).unwrap();
    let mut device = nvk(chip, 16 << 30);
    device.features.shader_resource_residency = false;
    let capabilities = Capabilities::derive(
        &device,
        &chip.platform(attachment()),
        ShaderFeatures::KNOWN,
        ApiFeatures::KNOWN,
    );
    assert_eq!(capabilities.sparse_tile_bytes(SparsePageSize::Size64), None);
    let cause = |name: &str| {
        capabilities
            .gaps()
            .iter()
            .find(|gap| gap.capability == name)
            .map(|gap| gap.cause)
    };
    assert_eq!(cause("sparse residency feedback"), Some(Cause::Driver));
    assert_eq!(cause("sparse textures"), Some(Cause::Driver));
}

#[test]
fn missing_families_name_the_layer_that_lacks_them() {
    let capabilities = complete(Chip::find(0x172).unwrap());
    let cause = |name: &str| {
        capabilities
            .gaps()
            .iter()
            .find(|gap| gap.capability == name)
            .map(|gap| gap.cause)
    };
    assert_eq!(cause("tessellation"), None);
    assert_eq!(cause("GPU family Common3"), None);
    assert_eq!(cause("GPU family Metal3"), None);
    assert_eq!(cause("GPU family Apple7"), None);
    assert_eq!(cause("ray tracing"), Some(Cause::Driver));
    // Stencil outputs run the fragment tests in their function instead.
    assert_eq!(cause("stencil feedback"), None);
    assert!(capabilities.fragment_tests);
    assert!(capabilities.stencil_follows_depth);
    assert_eq!(cause("fast resource loading"), None);
    assert_eq!(cause("MetalFX upscaling"), None);
    assert_eq!(cause("raster order groups"), None);
    assert_eq!(cause("framebuffer reads"), None);
    assert!(capabilities.framebuffer_reads);
    assert_eq!(cause("GPU family Metal4"), Some(Cause::AppleArchitecture));
    assert_eq!(cause("ASTC LDR pixel formats"), None);
    assert_eq!(cause("ASTC HDR pixel formats"), None);
    assert!(capabilities.supports_family(GpuFamily::Apple7));
    let names: alloc::vec::Vec<_> = capabilities
        .gaps()
        .iter()
        .map(|gap| gap.capability.as_str())
        .collect();
    let mut unique = names.clone();
    unique.sort_unstable();
    unique.dedup();
    assert_eq!(unique.len(), names.len(), "each gap is reported once");
}

#[test]
fn compute_only_compilers_advertise_no_family() {
    let chip = Chip::find(0x1b3).unwrap();
    let capabilities = Capabilities::derive(
        &nvk(chip, 16 << 30),
        &chip.platform(attachment()),
        ShaderFeatures::COMPUTE | ShaderFeatures::THREADGROUP_MEMORY,
        ApiFeatures::IMPLEMENTED,
    );
    assert!(capabilities.families().is_empty());
    assert!(!capabilities.format(P::Rgba8Unorm).contains(C::READ));
    assert!(capabilities.format(P::Rgba8Unorm).contains(C::BLIT));
    assert!(
        capabilities
            .gaps()
            .iter()
            .any(|gap| gap.capability == "GPU family Common1" && gap.cause == Cause::Compiler)
    );
    assert!(
        capabilities
            .gaps()
            .iter()
            .any(|gap| gap.capability == "tessellation" && gap.cause == Cause::Compiler)
    );
}

#[test]
fn pixel_format_capabilities_follow_the_driver_tables() {
    let capabilities = complete(Chip::find(0x162).unwrap());
    let all_color =
        C::READ | C::FILTER | C::WRITE | C::READ_WRITE | C::COLOR | C::BLEND | C::MSAA | C::RESOLVE;
    assert!(
        capabilities
            .format(P::Rgba8Unorm)
            .contains(all_color | C::SPARSE | C::LINEAR | C::BLIT)
    );
    assert!(
        capabilities
            .format(P::R32Uint)
            .contains(C::ATOMIC | C::COLOR | C::MSAA)
    );
    assert!(
        !capabilities
            .format(P::R32Uint)
            .intersects(C::FILTER | C::BLEND | C::RESOLVE)
    );
    assert_eq!(
        capabilities.format(P::Bc7RgbaUnorm),
        C::READ | C::FILTER | C::SPARSE | C::BLIT | C::HOST_TRANSFER
    );
    // ETC2, EAC, PVRTC and ASTC, which NVIDIA samples no blocks of or none
    // as Apple decodes them, sample decoded.
    for format in [
        P::Etc2Rgb8,
        P::Etc2Rgb8A1Srgb,
        P::EacRgba8,
        P::EacR11Snorm,
        P::EacRg11Unorm,
        P::PvrtcRgb2bpp,
        P::PvrtcRgba4bppSrgb,
        P::Astc4x4Ldr,
        P::Astc12x10Srgb,
        P::Astc4x4Hdr,
        P::Astc12x12Hdr,
    ] {
        assert_eq!(capabilities.format(format), C::READ | C::FILTER | C::BLIT);
    }
    // Extended-range formats render and filter through half floats, which
    // no shader write, buffer, linear or sparse texture shares.
    for format in [P::Bgra10Xr, P::Bgr10Xr, P::Bgr10XrSrgb] {
        let xr = capabilities.format(format);
        assert!(xr.contains(C::READ | C::FILTER | C::COLOR | C::BLEND | C::MSAA | C::RESOLVE));
        assert!(!xr.intersects(
            C::WRITE | C::READ_WRITE | C::ATOMIC | C::SPARSE | C::LINEAR | C::TEXTURE_BUFFER_READ
        ));
    }
    // Formats NVIDIA samples but renders no view of render through their
    // raw texels.
    for format in [P::R8UnormSrgb, P::Abgr4Unorm] {
        assert!(capabilities.format(format).contains(
            C::READ | C::FILTER | C::COLOR | C::BLEND | C::MSAA | C::RESOLVE | C::RAW_RENDER
        ));
    }
    // RG8Unorm_sRGB, which NVIDIA lacks altogether, renders padded to four
    // channels.
    let padded = capabilities.format(P::Rg8UnormSrgb);
    assert!(padded.contains(C::READ | C::FILTER | C::COLOR | C::BLEND | C::MSAA | C::RESOLVE));
    assert!(!padded.intersects(C::RAW_RENDER | C::WRITE | C::LINEAR));
    assert!(!capabilities.format(P::Rgba8Unorm).contains(C::RAW_RENDER));
    // A1BGR5 renders as A1R5G5B5, its texels rotated by a bit, which only
    // the CPU and the texel kernels see.
    let rotated = capabilities.format(P::A1Bgr5Unorm);
    assert!(
        rotated
            .contains(C::READ | C::FILTER | C::COLOR | C::BLEND | C::MSAA | C::RESOLVE | C::BLIT)
    );
    assert!(!rotated.intersects(C::WRITE | C::SPARSE | C::LINEAR | C::TEXTURE_BUFFER_READ));
    assert!(
        capabilities
            .format(P::Rgb8Unorm)
            .contains(C::TEXTURE_BUFFER_READ)
    );
    assert!(!capabilities.format(P::Rgb8Unorm).contains(C::READ));
    assert!(capabilities.format(P::Depth32Float).contains(
        C::DEPTH_STENCIL | C::READ | C::FILTER | C::MSAA | C::RESOLVE | C::COMPARE_SAMPLE
    ));
    assert!(
        !capabilities
            .format(P::X32Stencil8)
            .contains(C::DEPTH_STENCIL)
    );
    assert!(capabilities.depth24_stencil8_supported);
    assert!(capabilities.supports_bc_texture_compression);
    assert!(capabilities.supports_32bit_float_filtering && capabilities.supports_32bit_msaa);
}

#[test]
fn device_queries_follow_nvk_limits() {
    let capabilities = complete(Chip::find(0x194).unwrap());
    assert_eq!(
        capabilities.recommended_max_working_set_size,
        (16 << 30) - (256 << 20)
    );
    assert!(!capabilities.has_unified_memory && !capabilities.low_power);
    assert_eq!(capabilities.max_transfer_rate, 31_506_000_000);
    assert!(capabilities.supports_texture_sample_count(8));
    assert!(!capabilities.supports_texture_sample_count(16));
    assert!(!capabilities.supports_texture_sample_count(3));
    assert!(capabilities.supports_vertex_amplification_count(2));
    assert!(!capabilities.supports_vertex_amplification_count(3));
    assert!(capabilities.supports_counter_sampling(CounterSamplingPoint::AtDrawBoundary));
    assert!(!capabilities.supports_counter_sampling(CounterSamplingPoint::AtTileDispatchBoundary));
    assert_eq!(capabilities.timestamp_frequency, 1_000_000_000);
    assert!(capabilities.programmable_sample_positions);
    assert!(capabilities.supports_mesh_shaders);
    assert!(capabilities.supports_tessellation);
    assert!(capabilities.memoryless_render_targets);
    assert_eq!(capabilities.limits.max_tessellation_factor, 64);
    assert!(capabilities.supports_rasterization_rate_map(1));
    assert!(capabilities.supports_rasterization_rate_map(2));
    assert!(!capabilities.supports_rasterization_rate_map(3));
    assert!(capabilities.supports_feature_set(10002));
    assert_eq!(
        capabilities.supports_feature_set(10005),
        capabilities.supports_family(GpuFamily::Mac2)
    );
    assert!(!capabilities.supports_feature_set(16));
}

#[test]
fn sparse_tiles_use_the_standard_64k_block_shapes() {
    let chip = Chip::find(0x176).unwrap();
    if !ApiFeatures::IMPLEMENTED.contains(ApiFeatures::SPARSE_TEXTURES) {
        assert_eq!(
            complete(chip).sparse_tile_bytes(SparsePageSize::Size64),
            None
        );
    }
    let capabilities = with_api(
        chip,
        ApiFeatures::IMPLEMENTED | ApiFeatures::SPARSE_TEXTURES,
    );
    assert_eq!(
        capabilities.sparse_tile_bytes(SparsePageSize::Size64),
        Some(64 << 10)
    );
    assert_eq!(capabilities.sparse_tile_bytes(SparsePageSize::Size16), None);
    let tile = |format, texture_type, samples| {
        capabilities.sparse_tile_size(texture_type, format, samples)
    };
    assert_eq!(
        tile(P::Rgba8Unorm, TextureType::D2, 1),
        Some(Size {
            width: 128,
            height: 128,
            depth: 1
        })
    );
    assert_eq!(
        tile(P::Rgba32Float, TextureType::D3, 1),
        Some(Size {
            width: 16,
            height: 16,
            depth: 16
        })
    );
    // Vulkan's MSAA block shapes halve the single-sample shape per doubling,
    // width first.
    let planar = |width, height| {
        Some(Size {
            width,
            height,
            depth: 1,
        })
    };
    assert_eq!(
        tile(P::R8Unorm, TextureType::D2Multisample, 2),
        planar(128, 256)
    );
    assert_eq!(
        tile(P::R8Unorm, TextureType::D2Multisample, 4),
        planar(128, 128)
    );
    assert_eq!(
        tile(P::Rgba8Unorm, TextureType::D2MultisampleArray, 8),
        planar(32, 64)
    );
    assert_eq!(
        tile(P::Rgba32Float, TextureType::D2Multisample, 4),
        planar(32, 32)
    );
    assert_eq!(tile(P::R8Unorm, TextureType::D2Multisample, 1), None);
    assert_eq!(tile(P::R8Unorm, TextureType::D2, 4), None);
    assert_eq!(
        tile(P::R8Unorm, TextureType::D2Multisample, 16),
        None,
        "the device has no 16-sample images"
    );
    assert_eq!(
        tile(P::Bc1Rgba, TextureType::D2, 1),
        Some(Size {
            width: 512,
            height: 256,
            depth: 1
        })
    );
    assert_eq!(tile(P::Rgba8Unorm, TextureType::Cube, 1), None);
    assert_eq!(tile(P::Astc4x4Ldr, TextureType::D2, 1), None);
}

#[test]
fn linear_and_texture_buffer_alignments() {
    let capabilities = complete(Chip::find(0x1b2).unwrap());
    assert_eq!(
        capabilities.minimum_linear_texture_alignment(P::Rgba8Unorm),
        32
    );
    assert_eq!(capabilities.minimum_linear_texture_alignment(P::Bc1Rgba), 0);
    assert_eq!(
        capabilities.minimum_texture_buffer_alignment(P::Rgba8Unorm),
        4
    );
    assert_eq!(
        capabilities.minimum_texture_buffer_alignment(P::Rgba32Float),
        16
    );
    assert_eq!(
        capabilities.minimum_texture_buffer_alignment(P::Rgb32Float),
        16
    );
    assert_eq!(
        capabilities.minimum_texture_buffer_alignment(P::Depth32Float),
        0
    );
}
