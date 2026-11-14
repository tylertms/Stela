use super::*;
use crate::metal::{abi::HazardTrackingMode, error::Error, nvidia::fixture::capabilities};

fn options(storage_mode: StorageMode) -> ResourceOptions {
    ResourceOptions {
        cpu_cache_mode: CpuCacheMode::DefaultCache,
        storage_mode,
        hazard_tracking_mode: HazardTrackingMode::Default,
    }
}

fn descriptor(texture_type: TextureType, pixel_format: PixelFormat) -> TextureDescriptor {
    TextureDescriptor {
        texture_type,
        pixel_format,
        width: 256,
        height: 256,
        depth: 1,
        mipmap_level_count: 1,
        sample_count: 1,
        array_length: 1,
        options: options(StorageMode::Private),
        usage: TextureUsage::SHADER_READ,
        swizzle: SwizzleChannels::IDENTITY,
    }
}

#[test]
fn storage_modes_select_discrete_gpu_memory() {
    let shared = memory(options(StorageMode::Shared)).unwrap();
    assert_eq!(
        (shared.required, shared.preferred, shared.mapped),
        (M::HOST_VISIBLE | M::HOST_COHERENT, M::HOST_CACHED, true)
    );
    let write_combined = memory(ResourceOptions {
        cpu_cache_mode: CpuCacheMode::WriteCombined,
        ..options(StorageMode::Managed)
    })
    .unwrap();
    assert_eq!(
        (write_combined.required, write_combined.preferred),
        (M::HOST_VISIBLE, M::DEVICE_LOCAL)
    );
    assert_eq!(
        memory(options(StorageMode::Private)).unwrap().required,
        M::DEVICE_LOCAL
    );
    assert_eq!(
        memory(options(StorageMode::Memoryless)),
        Err(Error::Invalid("memoryless storage holds only textures"))
    );
}

#[test]
fn memoryless_textures_live_in_device_memory_out_of_the_cpu_reach() {
    let mut memoryless = descriptor(TextureType::D2Multisample, PixelFormat::Depth32Float);
    memoryless.sample_count = 4;
    memoryless.usage = TextureUsage::RENDER_TARGET;
    memoryless.options = options(StorageMode::Memoryless);
    let plan = texture(&capabilities(), &memoryless).unwrap();
    assert_eq!(plan.cpu_access, CpuAccess::None);
    assert_eq!(
        (plan.memory.required, plan.memory.mapped),
        (M::DEVICE_LOCAL, false)
    );
}

#[test]
fn buffers_serve_every_use_up_to_the_device_limit() {
    let capabilities = capabilities();
    let plan = buffer(&capabilities, 4096, options(StorageMode::Shared)).unwrap();
    assert_eq!(plan.info.usage, BUFFER_USAGE);
    assert!(
        plan.info
            .usage
            .contains(BufferUsage::SHADER_DEVICE_ADDRESS | BufferUsage::INDIRECT_BUFFER)
    );
    assert!(matches!(
        buffer(&capabilities, 0, options(StorageMode::Shared)),
        Err(Error::Invalid(_))
    ));
    let max = capabilities.limits.max_buffer_length;
    assert!(buffer(&capabilities, max, options(StorageMode::Private)).is_ok());
    assert_eq!(
        buffer(&capabilities, max + 1, options(StorageMode::Private)),
        Err(Error::Unsupported("buffer length"))
    );
}

#[test]
fn cube_arrays_become_cube_compatible_layered_images() {
    let mut cube = descriptor(TextureType::CubeArray, PixelFormat::Rgba16Float);
    cube.array_length = 2;
    cube.mipmap_level_count = 9;
    cube.usage = TextureUsage::SHADER_READ | TextureUsage::RENDER_TARGET;
    let plan = texture(&capabilities(), &cube).unwrap();
    assert_eq!(plan.image.image_type, ImageType::D2);
    assert_eq!(plan.image.array_layers, 12);
    assert!(plan.image.flags.contains(ImageCreate::CUBE_COMPATIBLE));
    assert!(
        plan.image
            .usage
            .contains(ImageUsage::SAMPLED | ImageUsage::COLOR_ATTACHMENT)
    );
    // Framebuffer reads view its raw texels as storage, and kernels write
    // it as Apple GPUs let them without shader-write usage.
    assert!(plan.image.usage.contains(ImageUsage::STORAGE));
    assert!(
        plan.image
            .flags
            .contains(ImageCreate::MUTABLE_FORMAT | ImageCreate::EXTENDED_USAGE)
    );
    assert_eq!(
        plan.image.view_formats,
        [Format::R16G16B16A16Sfloat, Format::R32G32Uint]
    );
    let sampled = plan.views.sampled.unwrap();
    assert_eq!(
        (
            sampled.view_type,
            sampled.range.layer_count,
            sampled.range.level_count
        ),
        (ImageViewType::CubeArray, 12, 9)
    );
    // Reads fetch from the faces.
    assert_eq!(
        plan.views.faces,
        Some(ImageViewInfo {
            view_type: ImageViewType::D2Array,
            usage: ImageUsage::SAMPLED,
            ..sampled
        })
    );
    // Kernels write it through the sampled view, which is also storage.
    assert_eq!(plan.views.storage, plan.views.sampled);
}

#[test]
fn multisample_depth_targets_take_custom_sample_positions() {
    let mut depth = descriptor(TextureType::D2Multisample, PixelFormat::Depth32Float);
    depth.sample_count = 4;
    depth.usage = TextureUsage::RENDER_TARGET;
    let compatible = |capabilities: &Capabilities, depth: &TextureDescriptor| {
        texture(capabilities, depth)
            .unwrap()
            .image
            .flags
            .contains(ImageCreate::SAMPLE_LOCATIONS_COMPATIBLE_DEPTH)
    };
    let mut capabilities = capabilities();
    assert!(capabilities.programmable_sample_positions);
    assert!(compatible(&capabilities, &depth));
    let mut color = depth;
    color.pixel_format = PixelFormat::Rgba8Unorm;
    assert!(!compatible(&capabilities, &color));
    capabilities.programmable_sample_positions = false;
    assert!(!compatible(&capabilities, &depth));
}

#[test]
fn shape_rules_follow_metal() {
    let capabilities = capabilities();
    let invalid = |change: fn(&mut TextureDescriptor)| {
        let mut texture_descriptor = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
        change(&mut texture_descriptor);
        matches!(
            texture(&capabilities, &texture_descriptor),
            Err(Error::Invalid(_))
        )
    };
    assert!(invalid(|d| d.width = 0));
    assert!(invalid(|d| d.mipmap_level_count = 10));
    assert!(invalid(|d| d.array_length = 2));
    assert!(invalid(|d| d.depth = 2));
    assert!(invalid(|d| d.sample_count = 4));
    assert!(invalid(|d| d.texture_type = TextureType::TextureBuffer));
    assert!(invalid(|d| {
        d.texture_type = TextureType::Cube;
        d.height = 128;
    }));
    assert!(invalid(|d| {
        d.texture_type = TextureType::D2Multisample;
        d.sample_count = 4;
        d.mipmap_level_count = 2;
    }));
    assert!(invalid(|d| {
        d.pixel_format = PixelFormat::Depth32Float;
        d.texture_type = TextureType::D3;
    }));
    assert!(invalid(|d| d.pixel_format = PixelFormat::X32Stencil8));

    // Apple's frameworks expect shared textures on any device.
    let mut shared = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    shared.options.storage_mode = StorageMode::Shared;
    assert!(texture(&capabilities, &shared).is_ok());
    shared.texture_type = TextureType::D2Multisample;
    shared.sample_count = 4;
    assert!(matches!(
        texture(&capabilities, &shared),
        Err(Error::Unsupported(_))
    ));

    let mut huge = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    huge.width = 65536;
    assert_eq!(
        texture(&capabilities, &huge),
        Err(Error::Unsupported("texture dimensions"))
    );
    let mut msaa16 = descriptor(TextureType::D2Multisample, PixelFormat::Rgba8Unorm);
    msaa16.sample_count = 16;
    assert_eq!(
        texture(&capabilities, &msaa16),
        Err(Error::Unsupported("texture sample count"))
    );
}

#[test]
fn unsupported_formats_and_uses_are_refused_by_capability() {
    let capabilities = capabilities();
    // ASTC HDR blocks are held decoded to half floats.
    let astc = descriptor(TextureType::D2, PixelFormat::Astc4x4Hdr);
    assert_eq!(
        texture(&capabilities, &astc).unwrap().image.format,
        Format::R16G16B16A16Sfloat
    );
    // Extended-range textures are half-float images; shader writes would
    // skip the clamping their stores do.
    let mut xr = descriptor(TextureType::D2, PixelFormat::Bgra10Xr);
    xr.usage = TextureUsage::SHADER_READ | TextureUsage::RENDER_TARGET;
    let plan = texture(&capabilities, &xr).unwrap();
    assert_eq!(plan.image.format, Format::R16G16B16A16Sfloat);
    xr.usage = TextureUsage::SHADER_WRITE;
    assert_eq!(
        texture(&capabilities, &xr),
        Err(Error::Unsupported("shader writes to this pixel format"))
    );
    let mut bc_target = descriptor(TextureType::D2, PixelFormat::Bc1Rgba);
    bc_target.usage = TextureUsage::RENDER_TARGET;
    assert_eq!(
        texture(&capabilities, &bc_target),
        Err(Error::Unsupported("rendering to this pixel format"))
    );
    let mut float_atomic = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    float_atomic.usage = TextureUsage::SHADER_ATOMIC;
    assert_eq!(
        texture(&capabilities, &float_atomic),
        Err(Error::Unsupported("texture atomics on this pixel format"))
    );
}

/// Metal gives depth and stencil formats no shader writes, and Apple's
/// drivers make a depth texture whose descriptor asks for them, as Setup
/// Assistant's Depth32Float texture does: the bit is dropped, its other uses
/// kept.
#[test]
fn shader_writes_asked_of_a_depth_or_stencil_texture_are_dropped() {
    let capabilities = capabilities();
    for format in [
        PixelFormat::Depth32Float,
        PixelFormat::Depth16Unorm,
        PixelFormat::Depth32FloatStencil8,
        PixelFormat::Stencil8,
    ] {
        let mut depth = descriptor(TextureType::D2, format);
        depth.usage =
            TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE | TextureUsage::RENDER_TARGET;
        let plan = texture(&capabilities, &depth).unwrap();
        assert!(
            !plan.image.usage.contains(ImageUsage::STORAGE),
            "{format:?}"
        );
        assert!(
            plan.image
                .usage
                .contains(ImageUsage::SAMPLED | ImageUsage::DEPTH_STENCIL_ATTACHMENT),
            "{format:?}"
        );
        depth.usage = TextureUsage::SHADER_WRITE;
        let plan = texture(&capabilities, &depth).unwrap();
        assert!(
            !plan.image.usage.contains(ImageUsage::STORAGE),
            "{format:?}"
        );
    }
}

#[test]
fn unknown_usage_admits_every_supported_use() {
    let mut unknown = descriptor(TextureType::D2, PixelFormat::R32Uint);
    unknown.usage = TextureUsage::EMPTY;
    let plan = texture(&capabilities(), &unknown).unwrap();
    assert!(
        plan.image
            .usage
            .contains(ImageUsage::SAMPLED | ImageUsage::STORAGE | ImageUsage::COLOR_ATTACHMENT)
    );
    assert!(plan.image.flags.contains(ImageCreate::MUTABLE_FORMAT));
    assert!(plan.image.view_formats.is_empty());
}

#[test]
fn srgb_textures_may_view_their_linear_pair() {
    let plan = texture(
        &capabilities(),
        &descriptor(TextureType::D2, PixelFormat::Bgra8UnormSrgb),
    )
    .unwrap();
    assert!(plan.image.flags.contains(ImageCreate::MUTABLE_FORMAT));
    assert_eq!(
        plan.image.view_formats,
        [Format::B8G8R8A8Srgb, Format::B8G8R8A8Unorm]
    );
}

#[test]
fn srgb_textures_store_through_their_linear_pair() {
    let mut srgb = descriptor(TextureType::D2, PixelFormat::Rgba8UnormSrgb);
    srgb.usage = TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE;
    let plan = texture(&capabilities(), &srgb).unwrap();
    assert!(plan.image.flags.contains(ImageCreate::EXTENDED_USAGE));
    assert!(plan.image.usage.contains(ImageUsage::STORAGE));
    assert_eq!(plan.views.sampled.unwrap().format, Format::R8G8B8A8Srgb);
    assert_eq!(plan.views.storage.unwrap().format, Format::R8G8B8A8Unorm);
    assert!(plan.views.srgb_storage);
    // A linear texture's views stay its own.
    let mut linear = srgb;
    linear.pixel_format = PixelFormat::Rgba8Unorm;
    let plan = texture(&capabilities(), &linear).unwrap();
    assert!(!plan.views.srgb_storage);
}

/// Shaders write the two-plane extended-range formats as an Apple GPU does,
/// holding their half floats to the codes: their storage views say so, and
/// through the sRGB curve for the sRGB format.
#[test]
fn extended_range_textures_are_written_through_their_codes() {
    for (format, srgb) in [
        (PixelFormat::Rgb10A8_2pXr10, false),
        (PixelFormat::Rgb10A8_2pXr10Srgb, true),
    ] {
        let mut written = descriptor(TextureType::D2, format);
        written.usage = TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE;
        let plan = texture(&capabilities(), &written).unwrap();
        assert_eq!(
            plan.views.storage.unwrap().format,
            Format::R16G16B16A16Sfloat
        );
        assert_eq!(plan.views.extended_storage, Some(srgb));
        assert!(!plan.views.srgb_storage);
        // A render target's kernels keep their writes too, as on Apple GPUs.
        written.usage = TextureUsage::SHADER_READ | TextureUsage::RENDER_TARGET;
        let plan = texture(&capabilities(), &written).unwrap();
        assert_eq!(plan.views.extended_storage, Some(srgb));
    }
    // Other extended-range formats still refuse them.
    let mut other = descriptor(TextureType::D2, PixelFormat::Bgr10Xr);
    other.usage = TextureUsage::SHADER_WRITE;
    assert!(texture(&capabilities(), &other).is_err());
}

#[test]
fn managed_textures_use_host_image_copies() {
    let mut managed = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    managed.options = options(StorageMode::Managed);
    let plan = texture(&capabilities(), &managed).unwrap();
    assert_eq!(plan.cpu_access, CpuAccess::HostImageCopy);
    assert!(plan.image.usage.contains(ImageUsage::HOST_TRANSFER));
    assert_eq!(plan.memory.required, M::DEVICE_LOCAL);
}

#[test]
fn shared_textures_use_host_image_copies() {
    let mut shared = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    shared.options = options(StorageMode::Shared);
    let plan = texture(&capabilities(), &shared).unwrap();
    assert_eq!(plan.cpu_access, CpuAccess::HostImageCopy);
    assert!(plan.image.usage.contains(ImageUsage::HOST_TRANSFER));
}

#[test]
fn swizzles_compose_and_storage_views_stay_identity() {
    let bgra = SwizzleChannels([
        TextureSwizzle::Blue,
        TextureSwizzle::Green,
        TextureSwizzle::Red,
        TextureSwizzle::One,
    ]);
    let s = ComponentSwizzle::Identity;
    assert_eq!(
        compose_swizzle([s, s, s, s], bgra),
        [
            ComponentSwizzle::B,
            s,
            ComponentSwizzle::R,
            ComponentSwizzle::One
        ]
    );
    // An RGB PVRTC realization forces alpha to one before Metal's swizzle.
    let opaque = [s, s, s, ComponentSwizzle::One];
    let alpha_to_red = SwizzleChannels([
        TextureSwizzle::Alpha,
        TextureSwizzle::Green,
        TextureSwizzle::Blue,
        TextureSwizzle::Alpha,
    ]);
    assert_eq!(
        compose_swizzle(opaque, alpha_to_red),
        [ComponentSwizzle::One, s, s, ComponentSwizzle::One]
    );

    let mut swizzled = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    swizzled.usage = TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE;
    swizzled.swizzle = bgra;
    let plan = texture(&capabilities(), &swizzled).unwrap();
    assert_eq!(
        plan.views.sampled.unwrap().components[0],
        ComponentSwizzle::B
    );
    assert_eq!(plan.views.storage.unwrap().components, [s; 4]);
}

#[test]
fn views_follow_metal_format_and_type_compatibility() {
    let capabilities = capabilities();
    let mut parent = descriptor(TextureType::D2Array, PixelFormat::Rgba8Unorm);
    parent.width = 64;
    parent.height = 64;
    parent.array_length = 12;
    parent.mipmap_level_count = 7;
    let view = |pixel_format, texture_type, levels: Range<u64>, slices: Range<u64>| {
        TextureViewDescriptor {
            pixel_format,
            texture_type,
            levels,
            slices,
            swizzle: SwizzleChannels::IDENTITY,
            min_lod: 0.0,
        }
    };
    let cube = texture_view(
        &capabilities,
        &parent,
        &view(PixelFormat::Rgba8UnormSrgb, TextureType::Cube, 2..4, 6..12),
    )
    .unwrap()
    .sampled
    .unwrap();
    assert_eq!(cube.view_type, ImageViewType::Cube);
    assert_eq!(cube.format, Format::R8G8B8A8Srgb);
    assert_eq!((cube.range.base_mip_level, cube.range.level_count), (2, 2));
    assert_eq!(
        (cube.range.base_array_layer, cube.range.layer_count),
        (6, 6)
    );

    let refused = |parent: &TextureDescriptor, descriptor: TextureViewDescriptor| {
        texture_view(&capabilities, parent, &descriptor).is_err()
    };
    assert!(
        refused(
            &parent,
            view(PixelFormat::R32Float, TextureType::D2, 0..1, 0..1)
        ),
        "needs PixelFormatView"
    );
    assert!(refused(
        &parent,
        view(PixelFormat::Rgba8Unorm, TextureType::Cube, 0..1, 0..5)
    ));
    assert!(refused(
        &parent,
        view(PixelFormat::Rgba8Unorm, TextureType::D3, 0..1, 0..1)
    ));
    assert!(refused(
        &parent,
        view(PixelFormat::Rgba8Unorm, TextureType::D2, 0..8, 0..1)
    ));
    assert!(refused(
        &parent,
        view(PixelFormat::Rgba8Unorm, TextureType::D2Array, 0..1, 4..13)
    ));

    let mut reinterpretable = parent;
    reinterpretable.usage |= TextureUsage::PIXEL_FORMAT_VIEW;
    assert!(!refused(
        &reinterpretable,
        view(PixelFormat::R32Float, TextureType::D2, 0..1, 0..1)
    ));
    assert!(!refused(
        &reinterpretable,
        view(PixelFormat::Rg16Float, TextureType::D2, 0..1, 0..1)
    ));
    assert!(
        refused(
            &reinterpretable,
            view(PixelFormat::Rgba16Float, TextureType::D2, 0..1, 0..1)
        ),
        "different texel size"
    );
}

#[test]
fn view_minimum_lods_bound_sampling_in_the_images_levels() {
    let capabilities = capabilities();
    let mut parent = descriptor(TextureType::D2, PixelFormat::Rgba8Unorm);
    parent.width = 16;
    parent.height = 16;
    parent.mipmap_level_count = 5;
    parent.usage = TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE;
    let view = |levels: Range<u64>, min_lod| TextureViewDescriptor {
        pixel_format: PixelFormat::Rgba8Unorm,
        texture_type: TextureType::D2,
        levels,
        slices: 0..1,
        swizzle: SwizzleChannels::IDENTITY,
        min_lod,
    };
    let views = texture_view(&capabilities, &parent, &view(1..4, 1.5)).unwrap();
    // Vulkan counts from the image's first level, Metal from the view's.
    assert_eq!(views.sampled.unwrap().min_lod, 2.5);
    assert_eq!(views.storage.unwrap().min_lod, 0.0);
    assert!(texture_view(&capabilities, &parent, &view(1..4, 2.5)).is_err());
    assert!(texture_view(&capabilities, &parent, &view(1..4, -1.0)).is_err());
    let mut lacking = capabilities.clone();
    lacking.view_min_lod = false;
    assert!(texture_view(&lacking, &parent, &view(1..4, 1.0)).is_err());
    assert!(texture_view(&lacking, &parent, &view(1..4, 0.0)).is_ok());
}

#[test]
fn stencil_views_read_the_stencil_aspect() {
    let capabilities = capabilities();
    let parent = descriptor(TextureType::D2, PixelFormat::Depth32FloatStencil8);
    let stencil = TextureViewDescriptor {
        pixel_format: PixelFormat::X32Stencil8,
        texture_type: TextureType::D2,
        levels: 0..1,
        slices: 0..1,
        swizzle: SwizzleChannels::IDENTITY,
        min_lod: 0.0,
    };
    let view = texture_view(&capabilities, &parent, &stencil)
        .unwrap()
        .sampled
        .unwrap();
    assert_eq!(view.format, Format::D32SfloatS8Uint);
    assert_eq!(view.range.aspects, ImageAspects::STENCIL);
    let depth = texture(&capabilities, &parent)
        .unwrap()
        .views
        .sampled
        .unwrap();
    assert_eq!(depth.range.aspects, ImageAspects::DEPTH);
}

#[test]
fn linear_textures_alias_buffer_rows_with_nvk_alignment() {
    let capabilities = capabilities();
    let mut linear = descriptor(TextureType::D2, PixelFormat::Bgra8Unorm);
    linear.width = 100;
    linear.height = 10;
    linear.options = options(StorageMode::Shared);
    let plan = linear_texture(&capabilities, &linear, 1 << 20, 8192, 416).unwrap();
    assert_eq!(plan.image.tiling, ImageTiling::DrmFormatModifier);
    assert_eq!(plan.image.linear, Some(LinearLayout { row_pitch: 416 }));
    assert_eq!(plan.cpu_access, CpuAccess::Linear);

    let refused = |offset, bytes_per_row, length| {
        linear_texture(&capabilities, &linear, length, offset, bytes_per_row).is_err()
    };
    assert!(refused(8192, 400, 1 << 20), "rows need 32-byte alignment");
    assert!(refused(8192, 384, 1 << 20), "a row is 400 bytes");
    assert!(refused(4100, 416, 1 << 20), "offsets need page alignment");
    assert!(refused(0, 416, 9 * 416 + 399), "the last row must fit");
    assert!(!refused(0, 416, 9 * 416 + 400));
    let mut d1 = linear;
    d1.texture_type = TextureType::D1;
    d1.height = 1;
    assert_eq!(
        linear_texture(&capabilities, &d1, 1 << 20, 0, 416),
        Err(Error::Unsupported("1D textures inside buffers"))
    );
}

#[test]
fn two_plane_textures_are_a_full_plane_and_a_subsampled_pair() {
    let capabilities = capabilities();
    let layout = |offset, bytes_per_row| PlaneLayout {
        offset,
        bytes_per_row,
    };
    for (format, second) in [
        (PixelFormat::Rgb8_444_2p, [101, 11]),
        (PixelFormat::Rgb8_422_2p, [51, 11]),
        (PixelFormat::Rgb8_420_2p, [51, 6]),
    ] {
        let mut surface = descriptor(TextureType::D2, format);
        surface.width = 101;
        surface.height = 11;
        surface.options = options(StorageMode::Shared);
        let [first, pair] = two_plane_texture(
            &capabilities,
            &surface,
            1 << 20,
            [layout(0, 128), layout(1 << 16, 256)],
        )
        .unwrap();
        assert_eq!(first.image.format, Format::R8Unorm, "{format:?}");
        assert_eq!(first.image.extent, [101, 11, 1]);
        assert_eq!(first.image.linear, Some(LinearLayout { row_pitch: 128 }));
        assert_eq!(pair.image.format, Format::R8G8Unorm);
        assert_eq!(pair.image.extent, [second[0], second[1], 1], "{format:?}");
        assert_eq!(pair.image.linear, Some(LinearLayout { row_pitch: 256 }));
        assert!(texture(&capabilities, &surface).is_err());
        assert!(linear_texture(&capabilities, &surface, 1 << 20, 0, 128).is_err());
    }
}

#[test]
fn ten_bit_two_plane_textures_are_planes_of_16_bit_words() {
    let capabilities = capabilities();
    let layout = |offset, bytes_per_row| PlaneLayout {
        offset,
        bytes_per_row,
    };
    for (format, second) in [
        (PixelFormat::Rgb10_444_2p, [101, 11]),
        (PixelFormat::Ycbcr10_422_2p, [51, 11]),
        (PixelFormat::Rgb10_420_2p, [51, 6]),
        (PixelFormat::Ycbcr10_420_2p, [51, 6]),
    ] {
        let mut surface = descriptor(TextureType::D2, format);
        surface.width = 101;
        surface.height = 11;
        surface.options = options(StorageMode::Shared);
        let [first, pair] = two_plane_texture(
            &capabilities,
            &surface,
            1 << 20,
            [layout(0, 256), layout(1 << 16, 512)],
        )
        .unwrap();
        assert_eq!(first.image.format, Format::R16Unorm, "{format:?}");
        assert_eq!(first.image.extent, [101, 11, 1]);
        assert_eq!(pair.image.format, Format::R16G16Unorm);
        assert_eq!(pair.image.extent, [second[0], second[1], 1], "{format:?}");
        assert_eq!(format.describe().block.bytes, 2);
        assert!(
            !capabilities
                .format(format)
                .contains(crate::metal::capabilities::FormatCapabilities::COLOR),
            "{format:?} does not render"
        );
    }
}

#[test]
fn texture_buffers_are_bounded_texel_views() {
    let capabilities = capabilities();
    let view = texture_buffer(
        &capabilities,
        PixelFormat::Rgba32Float,
        64,
        TextureUsage::SHADER_READ,
        4096,
        16,
    )
    .unwrap();
    assert_eq!(
        view,
        BufferViewInfo {
            format: Format::R32G32B32A32Sfloat,
            offset: 16,
            range: 1024
        }
    );
    assert!(
        texture_buffer(
            &capabilities,
            PixelFormat::Rgba32Float,
            64,
            TextureUsage::SHADER_READ,
            4096,
            8
        )
        .is_err()
    );
    assert!(
        texture_buffer(
            &capabilities,
            PixelFormat::Rgba32Float,
            256,
            TextureUsage::SHADER_READ,
            4096,
            16
        )
        .is_err()
    );
    assert_eq!(
        texture_buffer(
            &capabilities,
            PixelFormat::Bc1Rgba,
            4,
            TextureUsage::SHADER_READ,
            4096,
            0
        ),
        Err(Error::Unsupported("texture buffers of this pixel format"))
    );
}
