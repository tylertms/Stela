//! Test snapshots modeled on what NVK (Mesa 26.2.5) reports for Turing and
//! later classes in `nvk_physical_device.c` and `nvk_image.c`.
//!
//! These are fixtures for the derivation code, not a substitute for the
//! runtime query: format support is simplified to NVK's broad rules
//! (renderable uncompressed color and depth formats, BC sampling, no ETC2,
//! ASTC or PVRTC on discrete GPUs, buffer-only three-component formats).
use super::{Architecture, Chip};
use crate::metal::open::{Attachment, PcieLink};
use crate::metal::{
    capabilities::{ApiFeatures, Capabilities, ShaderFeatures},
    pixel_format::{Compression, Kind, PixelFormat},
    vulkan::{
        DriverId, Format, FormatFeatures as F, MemoryHeapFlags, MemoryProperties as M,
        PhysicalDeviceType, ResolveModes, SampleCounts, ShaderStages, SubgroupFeatures,
        physical::{
            Features, FormatSupport, FormatTable, GeneratedCommandsLimits, Identity, Limits,
            Memory, MemoryHeap, MemoryType, MeshLimits, PciBusInfo, PhysicalDevice,
            SampleLocationLimits, SparseLimits,
        },
    },
};
use alloc::{string::String, vec};

const SAMPLE_COUNTS: SampleCounts = SampleCounts::C1
    .union(SampleCounts::C2)
    .union(SampleCounts::C4)
    .union(SampleCounts::C8);
const DESCRIPTORS: u32 = 1 << 20;

/// GB203 with a compiler that lowers every shader feature and this build's
/// API paths: the profile other modules' tests run against.
pub(crate) fn capabilities() -> Capabilities {
    let chip = Chip::find(0x1b3).expect("GB203 is in nouveau's table");
    let attachment = Attachment {
        link: PcieLink {
            speed: 5,
            width: 16,
        },
        external: false,
        location_number: 0,
    };
    Capabilities::derive(
        &nvk(chip, 16 << 30),
        &chip.platform(attachment),
        ShaderFeatures::KNOWN,
        ApiFeatures::IMPLEMENTED,
    )
}

pub(crate) fn nvk(chip: &Chip, vram: u64) -> PhysicalDevice {
    PhysicalDevice {
        identity: Identity {
            api_version: (1 << 22) | (4 << 12),
            driver_id: DriverId::MesaNvk,
            driver_version: 0,
            vendor_id: super::VENDOR_ID,
            device_id: 0x2c05,
            pipeline_cache_uuid: [0x5a; 16],
            device_type: PhysicalDeviceType::DiscreteGpu,
            device_name: String::from("NVIDIA test GPU"),
            conformance_version: match chip.architecture {
                Architecture::Hopper => [0; 4],
                Architecture::Blackwell if !chip.display => [0; 4],
                _ => [1, 4, 3, 0],
            },
            pci_bus: Some(PciBusInfo {
                domain: 0,
                bus: 1,
                device: 0,
                function: 0,
            }),
        },
        limits: limits(),
        features: features(chip),
        memory: Memory {
            heaps: vec![
                MemoryHeap {
                    size: vram,
                    flags: MemoryHeapFlags::DEVICE_LOCAL,
                    budget: Some(vram - (256 << 20)),
                },
                MemoryHeap {
                    size: 16 << 30,
                    flags: MemoryHeapFlags::EMPTY,
                    budget: Some(12 << 30),
                },
            ],
            types: vec![
                MemoryType {
                    properties: M::DEVICE_LOCAL,
                    heap_index: 0,
                },
                MemoryType {
                    properties: M::HOST_VISIBLE | M::HOST_COHERENT,
                    heap_index: 1,
                },
                MemoryType {
                    properties: M::HOST_VISIBLE | M::HOST_COHERENT | M::HOST_CACHED,
                    heap_index: 1,
                },
                MemoryType {
                    properties: M::DEVICE_LOCAL | M::HOST_VISIBLE | M::HOST_COHERENT,
                    heap_index: 0,
                },
            ],
        },
        formats: formats(),
    }
}

fn limits() -> Limits {
    Limits {
        max_image_dimension_1d: 0x8000,
        max_image_dimension_2d: 0x8000,
        max_image_dimension_3d: 0x4000,
        max_image_dimension_cube: 0x8000,
        max_image_array_layers: 2048,
        max_texel_buffer_elements: 128 << 20,
        max_uniform_buffer_range: 65536,
        max_storage_buffer_range: u32::MAX,
        max_push_constants_size: 256,
        max_memory_allocation_count: 4096,
        max_sampler_allocation_count: 4000,
        buffer_image_granularity: 0x400,
        sparse_address_space_size: 1 << 39,
        max_per_stage_update_after_bind_samplers: DESCRIPTORS,
        max_per_stage_update_after_bind_sampled_images: DESCRIPTORS,
        max_per_stage_update_after_bind_storage_images: DESCRIPTORS,
        max_per_stage_update_after_bind_storage_buffers: DESCRIPTORS,
        max_descriptor_set_update_after_bind_samplers: DESCRIPTORS,
        max_descriptor_set_update_after_bind_sampled_images: DESCRIPTORS,
        max_descriptor_set_update_after_bind_storage_images: DESCRIPTORS,
        max_vertex_input_attributes: 32,
        max_vertex_input_bindings: 32,
        max_vertex_input_attribute_offset: 2047,
        max_vertex_input_binding_stride: 2048,
        max_vertex_output_components: 128,
        max_tessellation_generation_level: 64,
        max_tessellation_patch_size: 32,
        max_fragment_input_components: 128,
        max_fragment_output_attachments: 8,
        max_fragment_dual_src_attachments: 1,
        max_compute_shared_memory_size: 48 << 10,
        max_compute_work_group_count: [0x7fff_ffff, 65535, 65535],
        max_compute_work_group_invocations: 1024,
        max_compute_work_group_size: [1024, 1024, 64],
        max_sampler_lod_bias: 15.0,
        max_sampler_anisotropy: 16.0,
        max_viewports: 16,
        max_viewport_dimensions: [32768, 32768],
        min_texel_buffer_offset_alignment: 16,
        min_uniform_buffer_offset_alignment: 64,
        min_storage_buffer_offset_alignment: 16,
        max_framebuffer_width: 0x8000,
        max_framebuffer_height: 0x8000,
        max_framebuffer_layers: 2048,
        framebuffer_color_sample_counts: SAMPLE_COUNTS,
        framebuffer_depth_sample_counts: SAMPLE_COUNTS,
        framebuffer_stencil_sample_counts: SAMPLE_COUNTS,
        framebuffer_no_attachments_sample_counts: SAMPLE_COUNTS,
        max_color_attachments: 8,
        sampled_image_color_sample_counts: SAMPLE_COUNTS,
        sampled_image_integer_sample_counts: SAMPLE_COUNTS,
        sampled_image_depth_sample_counts: SAMPLE_COUNTS,
        storage_image_sample_counts: SAMPLE_COUNTS,
        max_sample_mask_words: 1,
        timestamp_compute_and_graphics: true,
        timestamp_period: 1.0,
        max_clip_distances: 8,
        max_cull_distances: 8,
        point_size_range: [1.0, 2047.94],
        line_width_range: [1.0, 64.0],
        non_coherent_atom_size: 64,
        subgroup_size: 32,
        min_subgroup_size: 32,
        max_subgroup_size: 32,
        subgroup_supported_stages: ShaderStages::KNOWN,
        subgroup_supported_operations: SubgroupFeatures::KNOWN,
        subgroup_quad_operations_in_all_stages: true,
        max_multiview_view_count: 32,
        max_memory_allocation_size: 1 << 31,
        max_buffer_size: 1 << 32,
        storage_texel_buffer_offset_alignment_bytes: 16,
        storage_texel_buffer_offset_single_texel_alignment: true,
        uniform_texel_buffer_offset_alignment_bytes: 16,
        uniform_texel_buffer_offset_single_texel_alignment: true,
        supported_depth_resolve_modes: ResolveModes::KNOWN,
        supported_stencil_resolve_modes: ResolveModes::SAMPLE_ZERO
            | ResolveModes::MIN
            | ResolveModes::MAX,
        independent_resolve_none: true,
        independent_resolve: true,
        max_vertex_attrib_divisor: u32::MAX,
        sample_locations: Some(SampleLocationLimits {
            sample_counts: SAMPLE_COUNTS.difference(SampleCounts::C1),
            max_grid_size: [1, 1],
            coordinate_range: [0.0, 0.9375],
            sub_pixel_bits: 4,
            variable_sample_locations: true,
        }),
        mesh: Some(MeshLimits {
            max_task_work_group_total_count: 1 << 22,
            max_task_work_group_count: [1 << 22, 65535, 65535],
            max_task_work_group_invocations: 128,
            max_task_payload_size: 1 << 14,
            max_mesh_work_group_total_count: 1 << 22,
            max_mesh_work_group_count: [1 << 22, 65535, 65535],
            max_mesh_work_group_invocations: 128,
            max_mesh_output_vertices: 256,
            max_mesh_output_primitives: 256,
        }),
        generated_commands: Some(GeneratedCommandsLimits {
            max_pipelines: u32::MAX,
            max_sequences: 1 << 20,
            max_tokens: 16,
            max_token_offset: 2047,
            max_stride: 1 << 12,
            pipeline_stages: ShaderStages::KNOWN.difference(ShaderStages::GEOMETRY),
        }),
        dma_buf_import: true,
        stencil_export: false,
        sparse: SparseLimits {
            residency_standard_2d_block_shape: true,
            residency_standard_2d_multisample_block_shape: true,
            residency_standard_3d_block_shape: true,
            residency_aligned_mip_size: false,
            residency_non_resident_strict: true,
        },
    }
}

/// NVK keys these on the 3D class, and every Turing and later class has
/// them; only TU11x lacks cooperative matrices.
fn features(chip: &Chip) -> Features {
    Features {
        image_cube_array: true,
        independent_blend: true,
        geometry_shader: true,
        tessellation_shader: true,
        sample_rate_shading: true,
        dual_src_blend: true,
        alpha_to_one: true,
        multi_draw_indirect: true,
        draw_indirect_first_instance: true,
        depth_clamp: true,
        depth_bias_clamp: true,
        fill_mode_non_solid: true,
        wide_lines: true,
        large_points: true,
        multi_viewport: true,
        sampler_anisotropy: true,
        texture_compression_bc: true,
        occlusion_query_precise: true,
        pipeline_statistics_query: true,
        vertex_pipeline_stores_and_atomics: true,
        fragment_stores_and_atomics: true,
        shader_image_gather_extended: true,
        shader_storage_image_multisample: true,
        shader_storage_image_read_without_format: true,
        shader_storage_image_write_without_format: true,
        shader_clip_distance: true,
        shader_cull_distance: true,
        shader_float64: true,
        shader_int64: true,
        shader_int16: true,
        shader_float16: true,
        shader_int8: true,
        shader_resource_residency: true,
        shader_resource_min_lod: true,
        sparse_binding: true,
        sparse_residency_buffer: true,
        sparse_residency_image_2d: true,
        sparse_residency_image_3d: true,
        sparse_residency_samples: SAMPLE_COUNTS.difference(SampleCounts::C1),
        sparse_residency_aliased: true,
        storage_buffer_8bit_access: true,
        storage_buffer_16bit_access: true,
        multiview: true,
        sampler_mirror_clamp_to_edge: true,
        sampler_filter_minmax: true,
        descriptor_indexing: true,
        runtime_descriptor_array: true,
        descriptor_binding_partially_bound: true,
        descriptor_binding_update_unused_while_pending: true,
        descriptor_binding_sampled_image_update_after_bind: true,
        descriptor_binding_storage_image_update_after_bind: true,
        descriptor_binding_uniform_texel_buffer_update_after_bind: true,
        descriptor_binding_storage_texel_buffer_update_after_bind: true,
        shader_sampled_image_array_non_uniform_indexing: true,
        shader_storage_image_array_non_uniform_indexing: true,
        buffer_device_address: true,
        timeline_semaphore: true,
        vulkan_memory_model: true,
        vulkan_memory_model_device_scope: true,
        shader_buffer_int64_atomics: true,
        shader_shared_int64_atomics: true,
        shader_image_int64_atomics: true,
        shader_buffer_float32_atomic_add: true,
        shader_shared_float32_atomic_add: true,
        shader_image_float32_atomics: true,
        shader_demote_to_helper_invocation: true,
        subgroup_size_control: true,
        compute_full_subgroups: true,
        synchronization2: true,
        dynamic_rendering: true,
        dynamic_rendering_local_read: true,
        maintenance4: true,
        maintenance5: true,
        maintenance8: true,
        host_image_copy: true,
        extended_dynamic_state: true,
        extended_dynamic_state2: true,
        extended_dynamic_state3_polygon_mode: true,
        extended_dynamic_state3_depth_clamp_enable: true,
        extended_dynamic_state3_depth_clip_enable: true,
        vertex_input_dynamic_state: true,
        vertex_attribute_instance_rate_divisor: true,
        depth_clip_enable: true,
        bresenham_lines: true,
        rectangular_lines: true,
        image_view_min_lod: true,
        fragment_shader_barycentric: true,
        task_shader: true,
        mesh_shader: true,
        // NVK's fragment shader interlock (Mesa's patch series).
        fragment_shader_pixel_interlock: true,
        unified_image_layouts: true,
        null_descriptor: true,
        dynamic_rendering_unused_attachments: true,
        dynamic_primitive_topology_unrestricted: true,
        host_query_reset: true,
        conditional_rendering: true,
        image_drm_format_modifier: true,
        shader_float16_vector_atomics: true,
        memory_budget: true,
        calibrated_timestamps: true,
        cooperative_matrix: !matches!(chip.architecture, Architecture::Pascal)
            && chip.chipset != 0x167
            && chip.chipset != 0x168,
        ..Features::default()
    }
}

fn formats() -> FormatTable {
    let mut table = FormatTable::default();
    let transfer = F::TRANSFER_SRC | F::TRANSFER_DST | F::BLIT_SRC | F::HOST_IMAGE_TRANSFER;
    for format in PixelFormat::ALL {
        let description = format.describe();
        let Ok(realization) = description.realization else {
            continue;
        };
        let block_bytes = description.block.bytes;
        let support = match (description.kind, description.compression) {
            (_, Compression::Bc) => FormatSupport {
                optimal: F::SAMPLED_IMAGE | F::SAMPLED_IMAGE_FILTER_LINEAR | transfer,
                sample_counts: SampleCounts::C1,
                sparse: SampleCounts::C1,
                ..FormatSupport::default()
            },
            (_, Compression::None)
                if description.is_color() && block_bytes % 3 == 0 && block_bytes != 3 * 4 =>
            {
                // RGB8 and RGB16: vertex and texel buffers only.
                FormatSupport {
                    buffer: F::VERTEX_BUFFER | F::UNIFORM_TEXEL_BUFFER,
                    ..FormatSupport::default()
                }
            }
            (Kind::Color(numeric), Compression::None) => {
                // NIL samples these but neither renders nor stores them.
                let texture_only = matches!(
                    realization.format,
                    Format::R8Srgb | Format::R4G4B4A4UnormPack16 | Format::E5B9G9R9UfloatPack32
                );
                let renderable = block_bytes != 12 && !texture_only;
                let filter = !numeric.is_integer();
                let atomic = matches!(
                    realization.format,
                    Format::R32Uint | Format::R32Sint | Format::R32Sfloat
                );
                let optimal = F::SAMPLED_IMAGE
                    .union(transfer)
                    .with(
                        F::SAMPLED_IMAGE_FILTER_LINEAR | F::SAMPLED_IMAGE_FILTER_MINMAX,
                        filter,
                    )
                    .with(
                        F::STORAGE_IMAGE
                            | F::STORAGE_READ_WITHOUT_FORMAT
                            | F::STORAGE_WRITE_WITHOUT_FORMAT,
                        renderable,
                    )
                    .with(F::COLOR_ATTACHMENT | F::BLIT_DST, renderable)
                    .with(F::COLOR_ATTACHMENT_BLEND, renderable && filter)
                    .with(F::STORAGE_IMAGE_ATOMIC, atomic);
                FormatSupport {
                    optimal,
                    linear: optimal.difference(F::STORAGE_IMAGE_ATOMIC),
                    // NVK renders to and stores linear-modifier images.
                    modifier_linear: optimal.difference(F::STORAGE_IMAGE_ATOMIC),
                    buffer: F::VERTEX_BUFFER | F::UNIFORM_TEXEL_BUFFER | F::STORAGE_TEXEL_BUFFER,
                    sample_counts: if renderable {
                        SAMPLE_COUNTS
                    } else {
                        SampleCounts::C1
                    },
                    sparse: if renderable {
                        SAMPLE_COUNTS
                    } else {
                        SampleCounts::EMPTY
                    },
                }
            }
            // NVK copies depth and stencil only on the GPU: no host image
            // copies (nvk_image.c).
            (_, Compression::None) => FormatSupport {
                optimal: F::SAMPLED_IMAGE
                    | F::SAMPLED_IMAGE_FILTER_LINEAR
                    | F::SAMPLED_IMAGE_DEPTH_COMPARISON
                    | F::DEPTH_STENCIL_ATTACHMENT
                    | transfer.difference(F::HOST_IMAGE_TRANSFER),
                sample_counts: SAMPLE_COUNTS,
                ..FormatSupport::default()
            },
            _ => continue,
        };
        table.insert(realization.format, support);
    }
    // NIL fetches signed 10:10:10:2 vertices but not shared-exponent ones.
    table.insert(
        Format::A2B10G10R10SnormPack32,
        FormatSupport {
            buffer: F::VERTEX_BUFFER,
            ..FormatSupport::default()
        },
    );
    let rgb9e5 = table.get(Format::E5B9G9R9UfloatPack32);
    table.insert(
        Format::E5B9G9R9UfloatPack32,
        FormatSupport {
            buffer: rgb9e5.buffer.difference(F::VERTEX_BUFFER),
            ..rgb9e5
        },
    );
    table
}
