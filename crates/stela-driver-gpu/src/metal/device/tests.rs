use super::{
    pipelines::{
        ComputePipelineRequest, ConstantKey, ConstantValue, FunctionHandle, LinkedFunctions,
        MeshPipelineRequest, PipelineArchives, RenderPipelineRequest, library_digest,
    },
    queue::{AttachmentRequest, CounterSet, RenderPassRequest},
    resources::{CPU_BAND_BYTES, CpuBand, HeapRefusals, Strides},
    *,
};
use crate::metal::{
    abi::{
        ConversionRounding, CpuCacheMode, DataType, DispatchType, FunctionType, HazardTrackingMode,
        HeapType, IndirectCommandType, LoadAction, MultisampleDepthResolveFilter,
        MultisampleStencilResolveFilter, PrimitiveTopologyClass, PrimitiveType, ResourceOptions,
        SparsePageSize, SparseTextureMappingMode, StorageMode, StoreAction, StoreActionOptions,
        SwizzleChannels, TessellationPartitionMode, TextureType, TextureUsage,
        VisibilityResultType,
    },
    capabilities::Size,
    command_buffer::{
        BufferRef,
        blit::Region,
        counters::{COUNTER_ERROR_VALUE, CounterSample},
        device_generated::{GeneratedLayoutKey, GeneratedWork},
        indirect::ExecutionRange,
    },
    heap::{HeapDescriptor, PAGE, SizeAndAlign},
    indirect::{IndirectCommandBufferDescriptor, RecordLayout},
    nvidia::{CHIPS, PcieLink},
    pipeline::{
        ColorAttachmentDescriptor, MAX_COLOR_ATTACHMENTS, RenderTargets, StageThreads,
        TessellationDescriptor,
    },
    pixel_format::{BlockKernel, PixelFormat, extended_range},
    resource::TextureDescriptor,
    sampler::SamplerDescriptor,
    shader::{DYNAMIC_BICUBIC, LinkedFunction, Linking, Specialization, Stage},
    sparse::{MappingUpdate, SparseRegion},
    vulkan::{
        MemoryProperties, PipelineStages, PrimitiveTopology, ShaderStages,
        backend::{Descriptor, Object},
        command::Command,
        object::{Buffer, MemoryRequest, Pipeline, PipelineSource, QueryType, Semaphore},
    },
};
use alloc::{string::String, sync::Arc, vec::Vec};
use core::sync::atomic::AtomicBool;

use super::fixture::{LIBRARY, MockBackend, MockCompiler, TestDevice, device, device_on};
use crate::metal::library::{
    self, Function,
    tests::{fixture, linking},
};

fn options(storage_mode: StorageMode) -> ResourceOptions {
    ResourceOptions {
        cpu_cache_mode: CpuCacheMode::DefaultCache,
        storage_mode,
        hazard_tracking_mode: HazardTrackingMode::Default,
    }
}

fn texture(pixel_format: PixelFormat, usage: TextureUsage) -> TextureDescriptor {
    TextureDescriptor {
        texture_type: TextureType::D2,
        pixel_format,
        width: 128,
        height: 128,
        depth: 1,
        mipmap_level_count: 1,
        sample_count: 1,
        array_length: 1,
        options: options(StorageMode::Private),
        usage,
        swizzle: SwizzleChannels::IDENTITY,
    }
}

/// Submits an empty command buffer and returns its serial.
fn submit(device: &mut TestDevice) -> u64 {
    let commands = device.new_command_buffer();
    device.commit(commands).unwrap()
}

fn complete(device: &mut TestDevice, serial: u64) {
    let timeline = device.timeline;
    device.backend().complete(timeline, serial);
    assert_eq!(device.poll(), Ok(serial));
}

#[test]
fn devices_missing_a_required_feature_are_refused() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.null_descriptor = false;
    let platform = CHIPS[0].platform(crate::metal::nvidia::Attachment {
        link: PcieLink {
            speed: 4,
            width: 16,
        },
        external: false,
        location_number: 0,
    });
    let device = Device::new(backend, MockCompiler::default(), &platform);
    assert!(matches!(device, Err(Error::Unsupported("nullDescriptor"))));
}

/// A guest's poll takes the completion marker's serial as the timeline: no
/// driver query, whether or not the marker shows something new; without a
/// marker the driver is asked as before.
#[test]
fn a_marked_poll_needs_no_driver_query() {
    use core::sync::atomic::{AtomicU32, Ordering};
    let mut backend = MockBackend::new();
    backend.marker = Some(alloc::boxed::Box::new(AtomicU32::new(0)));
    let mut device = device_on(backend);
    let first = submit(&mut device);
    let second = submit(&mut device);
    // A query would fail and lose the device.
    device.backend().semaphore_error = Some(Error::DeviceLost);
    assert_eq!(device.poll_marked(), Ok(0), "nothing marked yet");
    device
        .backend()
        .marker
        .as_ref()
        .unwrap()
        .store(first as u32, Ordering::SeqCst);
    assert_eq!(device.poll_marked(), Ok(first));
    assert_eq!(device.reached_for(0), first);
    device
        .backend()
        .marker
        .as_ref()
        .unwrap()
        .store(second as u32, Ordering::SeqCst);
    assert_eq!(device.poll_marked(), Ok(second));
    assert!(!device.is_lost());
    // The driver was never asked: its error is still waiting.
    assert_eq!(device.poll(), Ok(second), "nothing in flight to ask about");
    let third = submit(&mut device);
    assert_eq!(device.poll(), Err(Error::DeviceLost));
    assert!(third > second);
}

/// A marked poll tells the backend which submissions finished, so it frees
/// their command buffers without the query it would otherwise make once
/// enough wait: every one before the marked serial, which the queue ran
/// first, not the marked one, whose last command wrote the marker.
#[test]
fn a_marked_poll_lets_the_backend_recycle_finished_submissions() {
    use core::sync::atomic::{AtomicU32, Ordering};
    let mut backend = MockBackend::new();
    backend.marker = Some(alloc::boxed::Box::new(AtomicU32::new(0)));
    let mut device = device_on(backend);
    let first = submit(&mut device);
    let second = submit(&mut device);
    device.backend().semaphore_error = Some(Error::DeviceLost);
    assert_eq!(device.poll_marked(), Ok(0));
    assert!(device.backend().passed.is_empty(), "nothing marked yet");
    let mark = |device: &mut TestDevice, serial: u64| {
        device
            .backend()
            .marker
            .as_ref()
            .unwrap()
            .store(serial as u32, Ordering::SeqCst)
    };
    mark(&mut device, first);
    assert_eq!(device.poll_marked(), Ok(first));
    assert!(
        device
            .backend()
            .passed
            .iter()
            .all(|&(_, value)| value < first),
        "the marked submission may still run"
    );
    mark(&mut device, second);
    assert_eq!(device.poll_marked(), Ok(second));
    let passed = device.backend().passed.clone();
    assert_eq!(passed.last().map(|&(_, value)| value), Some(first));
    assert!(
        passed
            .iter()
            .all(|&(semaphore, _)| semaphore == passed[0].0)
    );
    assert!(!device.is_lost(), "no driver query was made");
}

/// While held work completes on the late timeline, a marked poll asks the
/// driver for that timeline alone: the marker still stands for the queue's
/// own, as a late submission writes it only after all submitted before it.
#[test]
fn a_marked_poll_with_late_work_asks_only_for_the_late_timeline() {
    use core::sync::atomic::{AtomicU32, Ordering};
    let mut backend = MockBackend::new();
    backend.marker = Some(alloc::boxed::Box::new(AtomicU32::new(0)));
    let mut device = device_on(backend);
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_wait_for_event(&mut device.resolver(), semaphore, 1)
        .unwrap();
    let held = device.commit_for(2, commands).unwrap();
    let later = submit(&mut device);
    assert!(later > held && device.holds());
    // The later command buffer completes; the held one is released after
    // it, so it completes on the late timeline.
    device.backend().finish();
    let store = |device: &mut TestDevice, serial: u64| {
        device
            .backend()
            .marker
            .as_ref()
            .unwrap()
            .store(serial as u32, Ordering::SeqCst)
    };
    store(&mut device, later);
    device.set_signaled_value(event, 1).unwrap();
    assert!(device.holds(), "the released command buffer completes late");
    device.backend().queried.clear();
    device.poll_marked().unwrap();
    let timeline = device.timeline;
    assert!(
        !device.backend().queried.contains(&timeline),
        "the marker answers for the queue's own timeline"
    );
    assert!(device.reached_for(0) >= later);
    // The held one runs and writes its serial late; the late timeline says
    // it completed.
    device.backend().finish();
    store(&mut device, held);
    device.poll_marked().unwrap();
    assert!(!device.backend().queried.contains(&timeline));
    assert_eq!(device.reached_for(2), device.submitted_serial());
    assert!(!device.is_lost());
}

#[test]
fn destruction_waits_for_submitted_work() {
    let mut device = device();
    let buffer = device
        .new_buffer(4096, options(StorageMode::Shared))
        .unwrap();
    assert_eq!(device.current_allocated_size(), 4096);
    let reference = device.buffer(buffer).unwrap();
    let (memory, _, _) = device.buffer_memory(buffer).unwrap();
    assert_eq!(reference.length, 4096);
    let serial = submit(&mut device);
    device.destroy_buffer(buffer).unwrap();
    assert_eq!(device.current_allocated_size(), 4096);
    assert!(
        device.backend().destroyed.is_empty(),
        "the GPU may still read it"
    );
    complete(&mut device, serial);
    assert_eq!(device.current_allocated_size(), 0);
    assert!(device.backend().destroyed.contains(&Object::Memory(memory)));
    assert!(
        device
            .backend()
            .destroyed
            .contains(&Object::Buffer(reference.buffer))
    );
    assert!(device.buffer(buffer).is_err());

    let idle = device
        .new_buffer(16, options(StorageMode::Private))
        .unwrap();
    let idle_buffer = device.buffer(idle).unwrap().buffer;
    device.destroy_buffer(idle).unwrap();
    assert!(
        device
            .backend()
            .destroyed
            .contains(&Object::Buffer(idle_buffer)),
        "nothing pending: destroyed at once"
    );
    assert!(device.buffer_memory(idle).is_err());
    assert_eq!(device.current_allocated_size(), 0);
}

/// A retirement bounded in time (the boot processor's look) destroys what it
/// can in release order and leaves the rest charged for the next one, even
/// with nothing left in flight.
#[test]
fn a_bounded_retirement_leaves_what_it_has_not_destroyed_for_the_next() {
    use core::sync::atomic::{AtomicU64, Ordering};
    static NOW: AtomicU64 = AtomicU64::new(0);
    let mut device = device();
    device.set_clock(|| NOW.fetch_add(1, Ordering::SeqCst));
    let buffers: Vec<_> = (0..4)
        .map(|_| {
            device
                .new_buffer(4096, options(StorageMode::Shared))
                .unwrap()
        })
        .collect();
    let first = device.buffer(buffers[0]).unwrap().buffer;
    let serial = submit(&mut device);
    for &buffer in &buffers {
        device.destroy_buffer(buffer).unwrap();
    }
    // Each reading of the clock advances it: the first object fits.
    device.bound_destruction(2);
    complete(&mut device, serial);
    device.unbound_destruction();
    assert!(device.destruction_left());
    assert!(device.backend().destroyed.contains(&Object::Buffer(first)));
    let left = |device: &mut TestDevice| {
        let destroyed = &device.backend().destroyed;
        let buffers = destroyed.iter();
        4 - buffers
            .filter(|object| matches!(object, Object::Buffer(_)))
            .count()
    };
    assert!(left(&mut device) > 0);
    assert!(device.current_allocated_size() > 0, "still charged");
    // Nothing is in flight; the next look or call destroys the rest.
    assert_eq!(device.poll(), Ok(serial));
    assert!(!device.destruction_left());
    assert_eq!(left(&mut device), 0);
    assert_eq!(device.current_allocated_size(), 0);
}

#[test]
fn allocations_are_charged_until_their_own_retirement_serial() {
    let mut device = device();
    let first = device
        .new_buffer(4096, options(StorageMode::Shared))
        .unwrap();
    let second = device
        .new_buffer(8192, options(StorageMode::Shared))
        .unwrap();
    let first_serial = submit(&mut device);
    device.destroy_buffer(first).unwrap();
    let second_serial = submit(&mut device);
    device.destroy_buffer(second).unwrap();
    assert_eq!(device.current_allocated_size(), 12288);
    complete(&mut device, first_serial);
    assert_eq!(device.current_allocated_size(), 8192);
    complete(&mut device, second_serial);
    assert_eq!(device.current_allocated_size(), 0);
    // Polling a completed serial must not release the charge a second time.
    assert_eq!(device.poll(), Ok(second_serial));
    assert_eq!(device.current_allocated_size(), 0);
}

#[test]
fn textures_publish_descriptors_and_release_them() {
    let mut device = device();
    let usage = TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE;
    let id = device
        .new_texture(&texture(PixelFormat::Rgba8Unorm, usage))
        .unwrap();
    let resource = device.texture_resource_id(id).unwrap();
    assert!(resource.low() != 0 && resource.high() != 0);
    let written: Vec<_> = device
        .backend()
        .descriptors
        .iter()
        .map(|(index, descriptor)| (*index, descriptor.array()))
        .collect();
    assert!(written.contains(&(resource.low(), HeapArray::SampledImages)));
    assert!(written.contains(&(resource.high_index(), HeapArray::StorageImages)));

    let view = device
        .new_texture_view(
            id,
            &crate::metal::resource::TextureViewDescriptor {
                pixel_format: PixelFormat::Rgba8UnormSrgb,
                texture_type: TextureType::D2,
                levels: 0..1,
                slices: 0..1,
                swizzle: SwizzleChannels::IDENTITY,
                min_lod: 0.0,
            },
        )
        .unwrap_or_else(|error| panic!("{error:?}"));
    assert_eq!(
        device.destroy_texture(id),
        Err(Error::Invalid("views of the texture are still alive"))
    );
    device.destroy_texture(view).unwrap();
    device.destroy_texture(id).unwrap();
    let images = device
        .backend()
        .destroyed
        .iter()
        .filter(|object| matches!(object, Object::Image(_)))
        .count();
    assert_eq!(images, 0, "views share their parent's image, which is kept");
    assert_eq!(device.recycled().textures, 1);
    // Released indices are reused once the (absent) work completes.
    let again = device
        .new_texture(&texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ))
        .unwrap();
    assert!(device.texture_resource_id(again).unwrap().low() != 0);
}

fn destroyed_images(device: &mut TestDevice) -> usize {
    device
        .backend()
        .destroyed
        .iter()
        .filter(|object| matches!(object, Object::Image(_)))
        .count()
}

#[test]
fn destroyed_private_textures_serve_equal_descriptors_once_their_work_completes() {
    let mut device = device();
    let descriptor = texture(PixelFormat::Rgba8Unorm, TextureUsage::RENDER_TARGET);
    let first = device.new_texture(&descriptor).unwrap();
    let resource = device.texture_resource_id(first).unwrap();
    let size = device.current_allocated_size();
    let serial = submit(&mut device);
    device.destroy_texture(first).unwrap();
    assert_eq!(device.current_allocated_size(), 0);

    // Work submitted before the destruction may still use it.
    let second = device.new_texture(&descriptor).unwrap();
    assert!(device.backend().cleared.is_empty());
    assert_ne!(device.texture_resource_id(second).unwrap(), resource);
    complete(&mut device, serial);
    device.destroy_texture(second).unwrap();

    let third = device.new_texture(&descriptor).unwrap();
    assert_eq!(device.texture_resource_id(third).unwrap(), resource);
    assert_eq!(
        device.backend().cleared.len(),
        1,
        "a reused image is zeroed"
    );
    assert_eq!(device.current_allocated_size(), size);
    assert_eq!(destroyed_images(&mut device), 0);
    let recycled = device.recycled();
    assert_eq!(
        (recycled.textures, recycled.hits, recycled.misses),
        (1, 1, 2)
    );

    // Another descriptor allocates.
    let other = texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ);
    device.new_texture(&other).unwrap();
    assert_eq!(device.recycled().hits, 1);
}

#[test]
fn textures_the_cpu_reaches_are_freed_not_recycled() {
    let mut device = device();
    let mut descriptor = texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ);
    descriptor.options = options(StorageMode::Shared);
    let id = device.new_texture(&descriptor).unwrap();
    device.destroy_texture(id).unwrap();
    assert_eq!(destroyed_images(&mut device), 1);
    assert_eq!(device.recycled().textures, 0);
}

#[test]
fn recycling_frees_the_oldest_textures_beyond_its_budget() {
    let mut device = device();
    let descriptor = texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ);
    let textures: Vec<_> = (0..=super::resources::RECYCLED_TEXTURES)
        .map(|_| device.new_texture(&descriptor).unwrap())
        .collect();
    for id in textures {
        device.destroy_texture(id).unwrap();
    }
    assert_eq!(
        device.recycled().textures,
        super::resources::RECYCLED_TEXTURES
    );
    assert_eq!(destroyed_images(&mut device), 1);
}

#[test]
fn storage_textures_have_a_descriptor_per_level() {
    let mut device = device();
    let mut descriptor = texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_WRITE);
    descriptor.mipmap_level_count = 8;
    // The CPU reaches a shared texture, so destruction frees it.
    descriptor.options = options(StorageMode::Shared);
    let id = device.new_texture(&descriptor).unwrap();
    let resource = device.texture_resource_id(id).unwrap();
    assert_eq!(resource.low(), 0, "write-only textures are not sampled");
    assert_eq!(resource.high_count(), 8);
    let storage: Vec<_> = device
        .backend()
        .descriptors
        .iter()
        .filter(|(_, descriptor)| descriptor.array() == HeapArray::StorageImages)
        .map(|(index, _)| *index)
        .collect();
    let first = resource.high_index();
    assert_eq!(storage, (first..first + 8).collect::<Vec<_>>());
    assert_eq!(device.descriptors.in_use(HeapArray::StorageImages), 8);
    let serial = submit(&mut device);
    device.destroy_texture(id).unwrap();
    complete(&mut device, serial);
    assert_eq!(device.descriptors.in_use(HeapArray::StorageImages), 0);
    let views = device
        .backend()
        .destroyed
        .iter()
        .filter(|object| matches!(object, Object::ImageView(_)))
        .count();
    assert_eq!(views, 8, "each level's view is destroyed");
}

#[test]
fn sampled_cubes_are_followed_by_their_faces() {
    let mut device = device();
    // The CPU reaches a shared texture, so destruction frees it.
    let cube = TextureDescriptor {
        texture_type: TextureType::Cube,
        options: options(StorageMode::Shared),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let id = device.new_texture(&cube).unwrap();
    let low = device.texture_resource_id(id).unwrap().low();
    let sampled: Vec<_> = device
        .backend()
        .descriptors
        .iter()
        .filter(|(_, descriptor)| descriptor.array() == HeapArray::SampledImages)
        .map(|(index, _)| *index)
        .collect();
    assert_eq!(sampled, [low, low + 1]);
    assert_eq!(device.descriptors.in_use(HeapArray::SampledImages), 2);
    let serial = submit(&mut device);
    device.destroy_texture(id).unwrap();
    complete(&mut device, serial);
    assert_eq!(device.descriptors.in_use(HeapArray::SampledImages), 0);
}

#[test]
fn texture_buffers_and_linear_textures_alias_their_buffer() {
    let mut device = device();
    let buffer = device
        .new_buffer(1 << 16, options(StorageMode::Shared))
        .unwrap();
    let mut descriptor = texture(
        PixelFormat::R32Float,
        TextureUsage::SHADER_READ | TextureUsage::SHADER_WRITE,
    );
    descriptor.texture_type = TextureType::TextureBuffer;
    descriptor.width = 1024;
    descriptor.height = 1;
    descriptor.options = options(StorageMode::Shared);
    let texel = device
        .new_texture_from_buffer(buffer, &descriptor, 0, 4096)
        .unwrap();
    let id = device.texture_resource_id(texel).unwrap();
    let arrays: Vec<_> = device
        .backend()
        .descriptors
        .iter()
        .map(|(_, descriptor)| descriptor.array())
        .collect();
    assert!(
        arrays.contains(&HeapArray::UniformTexelBuffers)
            && arrays.contains(&HeapArray::StorageTexelBuffers)
    );
    assert!(id.low() != 0 && id.high() != 0);
    assert_eq!(
        device.destroy_buffer(buffer),
        Err(Error::Invalid("textures still alias the buffer"))
    );
    device.destroy_texture(texel).unwrap();
    device.destroy_buffer(buffer).unwrap();
}

#[test]
fn heap_buffers_start_where_a_linear_texture_over_them_binds() {
    let mut device = device();
    let shared = options(StorageMode::Shared);
    let heap = device
        .new_heap(&HeapDescriptor {
            size: 1 << 20,
            options: shared,
            heap_type: HeapType::Automatic,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    let (_, linear) = device.capabilities.linear_texture_alignments();
    let first = device.new_heap_buffer(heap, 100, shared, None).unwrap();
    let second = device.new_heap_buffer(heap, 1 << 14, shared, None).unwrap();
    for buffer in [first, second] {
        let offset = device.buffer_placement(buffer).unwrap().heap_offset;
        assert!(offset.is_multiple_of(linear), "{offset:#x}");
    }
    let mut descriptor = texture(PixelFormat::R32Float, TextureUsage::SHADER_READ);
    descriptor.width = 16;
    descriptor.height = 16;
    descriptor.options = shared;
    device
        .new_texture_from_buffer(second, &descriptor, 0, 256)
        .unwrap();
}

/// A pixel-coordinate sampler, bicubic or not, says so in its ID: shaders
/// divide its coordinates by the base level's size for its normalized
/// Vulkan sampler, which NVIDIA requires on textures with mip levels.
#[test]
fn pixel_coordinate_samplers_flag_their_ids() {
    let mut device = device();
    let pixels = device
        .new_sampler(&SamplerDescriptor {
            normalized_coordinates: false,
            ..SamplerDescriptor::default()
        })
        .unwrap();
    let id = device.sampler_resource_id(pixels).unwrap();
    assert_ne!(id.high() & ResourceId::SAMPLER_PIXELS, 0);
    assert_eq!(id.high() & ResourceId::SAMPLER_CUBIC, 0);
    let normalized = device.new_sampler(&SamplerDescriptor::default()).unwrap();
    let id = device.sampler_resource_id(normalized).unwrap();
    assert_eq!(id.high() & ResourceId::SAMPLER_SHADED, 0);
}

#[test]
fn samplers_are_shared_and_clamped_variants_cached() {
    let mut device = device();
    let descriptor = SamplerDescriptor::default();
    let first = device.new_sampler(&descriptor).unwrap();
    let second = device.new_sampler(&descriptor).unwrap();
    assert_eq!(first, second);
    let id = device.sampler_resource_id(first).unwrap();
    assert_eq!(
        id.low(),
        id.sampler_integer(),
        "no border: the float sampler serves integer textures"
    );
    // Metal's default compares with Never: comparisons through it are zero.
    assert_ne!(id.high() & ResourceId::SAMPLER_NEVER, 0);
    let mut resolver = device.resolver();
    let clamped =
        crate::metal::command_buffer::Resolver::clamped_sampler(&mut resolver, id, 1.0, 4.0)
            .unwrap();
    let again =
        crate::metal::command_buffer::Resolver::clamped_sampler(&mut resolver, id, 1.0, 4.0)
            .unwrap();
    assert_eq!(clamped, again);
    assert_ne!(clamped, id);
    device.destroy_sampler(first).unwrap();
    assert!(
        device.backend().destroyed.is_empty(),
        "one reference remains"
    );
    device.destroy_sampler(second).unwrap();
    let samplers = device
        .backend()
        .destroyed
        .iter()
        .filter(|object| matches!(object, Object::Sampler(_)))
        .count();
    assert_eq!(samplers, 2, "the sampler and its clamped variant");
}

#[test]
fn constexpr_samplers_are_created_once_and_reach_compilation() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "sampling", Some(&[])).unwrap();
    // Indirect command support compiles the pipeline's function at once.
    let request = ComputePipelineRequest {
        compute_function: kernel,
        threadgroup_size_is_multiple_of_thread_execution_width: false,
        max_total_threads_per_threadgroup: 64,
        stage_input: None,
        linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: true,
        archives: Default::default(),
    };
    let created = |device: &mut TestDevice| {
        let descriptors = &device.backend().descriptors;
        descriptors
            .iter()
            .filter(|(_, descriptor)| matches!(descriptor, Descriptor::Sampler(_)))
            .count()
    };
    device.new_compute_pipeline(&request).unwrap();
    let samplers = device.compiler.compiled.last().unwrap().samplers.clone();
    assert_eq!(created(&mut device), 2);
    // Each is the sampler of the descriptor its state encodes.
    let (default, linear) = (0x007b_ff00_0008_0049, 0x007b_ff00_0008_0a49);
    let mut id = |state| {
        let sampler = device
            .new_sampler(&SamplerDescriptor::from_constexpr(state).unwrap())
            .unwrap();
        device.sampler_resource_id(sampler).unwrap().0
    };
    assert_eq!(samplers, [(default, id(default)), (linear, id(linear))]);
    device.new_compute_pipeline(&request).unwrap();
    assert_eq!(device.compiler.compiled.len(), 2);
    assert_eq!(device.compiler.compiled[1].samplers, samplers);
    assert_eq!(
        created(&mut device),
        2,
        "later pipelines reuse the samplers"
    );
}

#[test]
fn automatic_heaps_place_and_alias_resources() {
    let mut device = device();
    let heap = device
        .new_heap(&HeapDescriptor {
            size: 1 << 20,
            options: options(StorageMode::Private),
            heap_type: HeapType::Automatic,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    let a = device
        .new_heap_buffer(heap, 1000, options(StorageMode::Private), None)
        .unwrap();
    let b = device
        .new_heap_buffer(heap, 1000, options(StorageMode::Private), None)
        .unwrap();
    // Each starts where a linear texture over it can bind.
    assert_eq!(device.heap_usage(heap, 1).unwrap().0, 2048);
    assert!(
        device
            .new_heap_buffer(heap, 64, options(StorageMode::Shared), None)
            .is_err(),
        "storage mode differs"
    );
    assert!(
        device
            .new_heap_buffer(heap, 64, options(StorageMode::Private), Some(0))
            .is_err(),
        "automatic heaps place"
    );
    device.make_buffer_aliasable(a).unwrap();
    assert_eq!(device.heap_usage(heap, 1).unwrap().0, 1024);
    // Again changes nothing: the range returned once.
    device.make_buffer_aliasable(a).unwrap();
    assert_eq!(device.heap_usage(heap, 1).unwrap().0, 1024);
    let c = device
        .new_heap_buffer(heap, 1000, options(StorageMode::Private), None)
        .unwrap();
    assert_eq!(
        device.buffer(c).unwrap().address & 0xffff_ffff,
        0,
        "c reuses a's range"
    );
    let texture = device
        .new_heap_texture(
            heap,
            &texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ),
            None,
        )
        .unwrap();
    assert_eq!(
        device.destroy_heap(heap),
        Err(Error::Invalid("resources still live in the heap"))
    );
    assert_eq!(device.current_allocated_size(), 1 << 20);
    let serial = submit(&mut device);
    for buffer in [a, b, c] {
        device.destroy_buffer(buffer).unwrap();
    }
    device.destroy_texture(texture).unwrap();
    assert_eq!(device.heap_usage(heap, 1).unwrap().0, 0);
    assert_eq!(device.current_allocated_size(), 1 << 20);
    device.destroy_heap(heap).unwrap();
    assert_eq!(device.current_allocated_size(), 1 << 20);
    complete(&mut device, serial);
    assert_eq!(device.current_allocated_size(), 0);
}

/// `heapTextureSizeAndAlignWithDescriptor:` answers enough for the texture
/// whatever its storage and whatever usage the answer and the texture each
/// name: a heap image carries every usage its format allows, so NVK lays it
/// out one way. A heap of exactly the answered size takes it, automatic or
/// at offset zero of a placement heap, and a heap of the sum of two answers
/// takes both textures.
#[test]
fn heaps_of_the_answered_size_hold_the_texture() {
    let read = TextureUsage::SHADER_READ;
    let render = TextureUsage::RENDER_TARGET;
    let usages = [read, render, read | render];
    let others = [read | TextureUsage::SHADER_WRITE, TextureUsage::EMPTY];
    let formats = [
        PixelFormat::Bgra8Unorm,
        PixelFormat::Rgba16Float,
        PixelFormat::R8Unorm,
    ];
    let heap = |device: &mut TestDevice, size, storage_mode, heap_type| {
        device
            .new_heap(&HeapDescriptor {
                size,
                options: options(storage_mode),
                heap_type,
                sparse_page_size: SparsePageSize::Size64,
            })
            .unwrap()
    };
    for storage_mode in [StorageMode::Private, StorageMode::Shared] {
        for pixel_format in formats {
            let mut device = device();
            let described = |usage| {
                let mut descriptor = texture(pixel_format, usage);
                (descriptor.width, descriptor.height) = (2824, 1704);
                descriptor.options = options(storage_mode);
                descriptor
            };
            // The image NVK sizes is the same for every usage.
            let image = |usage| {
                crate::metal::resource::heap_texture(&device.capabilities, &described(usage))
                    .unwrap()
                    .image
            };
            let first = image(read);
            for usage in usages.iter().chain(&others) {
                assert_eq!(image(*usage), first, "{pixel_format:?} {usage:?}");
            }
            for asked in usages {
                let answered = device
                    .heap_texture_size_and_align(&described(asked))
                    .unwrap();
                for made in usages {
                    for heap_type in [HeapType::Automatic, HeapType::Placement] {
                        let id = heap(&mut device, answered.size, storage_mode, heap_type);
                        let offset = (heap_type == HeapType::Placement).then_some(0);
                        assert!(
                            device
                                .new_heap_texture(id, &described(made), offset)
                                .is_ok(),
                            "{heap_type:?} heap of {} bytes answered for {asked:?} holds \
                             {pixel_format:?} {made:?} {storage_mode:?}",
                            answered.size
                        );
                    }
                }
            }
        }
    }
    // Core Animation's pair: a small half-float texture and a screen.
    for (asked, made) in [(read, read | render), (read | render, read), (render, read)] {
        let mut device = device();
        let pair = [
            (PixelFormat::R16Float, 188, 87),
            (PixelFormat::Bgra8Unorm, 2824, 1704),
        ];
        let described = |format, (width, height), usage| {
            let mut descriptor = texture(format, usage);
            (descriptor.width, descriptor.height) = (width, height);
            descriptor
        };
        let mut size = 0;
        for (format, width, height) in pair {
            let answered = device
                .heap_texture_size_and_align(&described(format, (width, height), asked))
                .unwrap();
            size += answered.size.next_multiple_of(answered.align);
        }
        let id = heap(&mut device, size, StorageMode::Private, HeapType::Automatic);
        for (format, width, height) in pair {
            assert!(
                device
                    .new_heap_texture(id, &described(format, (width, height), made), None)
                    .is_ok(),
                "a heap of {size} bytes answered for {asked:?} holds {format:?} {made:?}"
            );
        }
    }
}

/// A heap holds as many textures as a Mac's does, whatever the backend's
/// layout of them: 1,024 8x8 R8 textures fill 128 KiB, the heap counts each
/// as 128 bytes, and one made aliasable gives its room to the next. A heap
/// of 100 bytes holds a page, as on a Mac.
#[test]
fn heaps_hold_as_many_textures_as_a_macs() {
    let mut device = device();
    let heap = |device: &mut TestDevice, size| {
        device
            .new_heap(&HeapDescriptor {
                size,
                options: options(StorageMode::Private),
                heap_type: HeapType::Automatic,
                sparse_page_size: SparsePageSize::Size64,
            })
            .unwrap()
    };
    let glyphs = heap(&mut device, 131_072);
    let mut glyph = texture(PixelFormat::R8Unorm, TextureUsage::SHADER_READ);
    (glyph.width, glyph.height) = (8, 8);
    assert_eq!(
        device.heap_texture_size_and_align(&glyph).unwrap(),
        SizeAndAlign {
            size: 128,
            align: 128
        }
    );
    let mut placed = Vec::new();
    while let Ok(texture) = device.new_heap_texture(glyphs, &glyph, None) {
        placed.push(texture);
    }
    assert_eq!(placed.len(), 1024);
    assert_eq!(device.heap_usage(glyphs, 128).unwrap(), (131_072, 0));
    assert_eq!(
        device.texture_placement(placed[1]).unwrap().heap_offset,
        128
    );
    assert_eq!(
        device.texture_placement(placed[1]).unwrap().allocated_size,
        128
    );
    device.make_texture_aliasable(placed[0]).unwrap();
    assert_eq!(
        device.heap_usage(glyphs, 128).unwrap(),
        (131_072 - 128, 128)
    );
    placed.push(device.new_heap_texture(glyphs, &glyph, None).unwrap());
    for texture in placed {
        device.destroy_texture(texture).unwrap();
    }
    assert_eq!(device.heap_usage(glyphs, 128).unwrap(), (0, 131_072));
    device.destroy_heap(glyphs).unwrap();

    let small = heap(&mut device, 100);
    assert_eq!(device.heap_usage(small, 1).unwrap(), (0, PAGE));
    assert_eq!(device.heap_occupancy(small).unwrap(), (PAGE, 0));
    device.destroy_heap(small).unwrap();
}

/// The heaps Core Animation filled on the guest, as a Mac (M1 Pro, macOS
/// 27) fills them: a heap holds its size and no more, so a second 66,560
/// byte R16Float mask does not fit 128 KiB, and the heap answers usedSize
/// and maxAvailableSizeWithAlignment: 128 and 16,384 as the Mac does after
/// each texture.
#[test]
fn heaps_refuse_what_a_macs_refuse() {
    let mut device = device();
    let drawn = TextureUsage::SHADER_READ | TextureUsage::RENDER_TARGET;
    // Heap size, format, width, height, then (used, max at 128, max at
    // 16 KiB) after each texture that fits; the next one does not.
    type Sequence = (u64, PixelFormat, u64, u64, &'static [(u64, u64, u64)]);
    let cases: [Sequence; 6] = [
        (
            131_072,
            PixelFormat::R16Float,
            72,
            151,
            &[(66_560, 64_512, 49_152)],
        ),
        (
            131_072,
            PixelFormat::R16Float,
            113,
            153,
            &[(66_560, 64_512, 49_152)],
        ),
        (
            196_608,
            PixelFormat::R16Float,
            248,
            81,
            &[(66_560, 130_048, 114_688), (133_120, 63_488, 49_152)],
        ),
        (
            65_536,
            PixelFormat::Rgba16Float,
            36,
            35,
            &[(32_896, 32_640, 16_384)],
        ),
        (65_536, PixelFormat::Rgba16Float, 68, 67, &[]),
        (65_536, PixelFormat::Rgba16Float, 122, 67, &[]),
    ];
    for (size, format, width, height, after) in cases {
        let heap = device
            .new_heap(&HeapDescriptor {
                size,
                options: options(StorageMode::Private),
                heap_type: HeapType::Automatic,
                sparse_page_size: SparsePageSize::Size64,
            })
            .unwrap();
        let mut descriptor = texture(format, drawn);
        (descriptor.width, descriptor.height) = (width, height);
        let mut placed = Vec::new();
        for &(used, max, max_page) in after {
            placed.push(device.new_heap_texture(heap, &descriptor, None).unwrap());
            assert_eq!(
                device.heap_usage(heap, 128).unwrap(),
                (used, max),
                "{format:?} {width}x{height} in {size}"
            );
            assert_eq!(device.heap_usage(heap, PAGE).unwrap().1, max_page);
        }
        assert_eq!(
            device.new_heap_texture(heap, &descriptor, None),
            Err(Error::HeapFull),
            "{format:?} {width}x{height} in {size}"
        );
        assert_eq!(device.heap_occupancy(heap).unwrap().0, size);
        for texture in placed {
            device.destroy_texture(texture).unwrap();
        }
        device.destroy_heap(heap).unwrap();
    }
}

/// A full automatic heap answers nil as Metal does, which is no shortage
/// of device memory.
#[test]
fn a_full_heap_is_not_out_of_memory() {
    let mut device = device();
    let descriptor = texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ);
    let answered = device.heap_texture_size_and_align(&descriptor).unwrap();
    let heap = device
        .new_heap(&HeapDescriptor {
            size: answered.size,
            options: options(StorageMode::Private),
            heap_type: HeapType::Automatic,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    device.new_heap_texture(heap, &descriptor, None).unwrap();
    assert_eq!(
        device.new_heap_texture(heap, &descriptor, None),
        Err(Error::HeapFull)
    );
    // The rest of the heap's last page holds a small buffer, as on a Mac.
    let rest = answered.size.next_multiple_of(crate::metal::heap::PAGE) - answered.size;
    assert!(
        device
            .new_heap_buffer(heap, rest, options(StorageMode::Private), None)
            .is_ok()
    );
    assert_eq!(
        device.new_heap_buffer(heap, 64, options(StorageMode::Private), None),
        Err(Error::HeapFull)
    );
    let full = HeapRefusals {
        full: 2,
        fragmented: 0,
    };
    assert_eq!(device.memory().heap_refusals, full);
    // Free bytes enough for a texture twice the size, in two pieces.
    let heap = device
        .new_heap(&HeapDescriptor {
            size: 3 * answered.size,
            options: options(StorageMode::Private),
            heap_type: HeapType::Automatic,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    let placed: Vec<_> = (0..3)
        .map(|_| device.new_heap_texture(heap, &descriptor, None).unwrap())
        .collect();
    device.destroy_texture(placed[0]).unwrap();
    device.destroy_texture(placed[2]).unwrap();
    let mut double = descriptor;
    double.height *= 2;
    assert_eq!(
        device.heap_texture_size_and_align(&double).unwrap().size,
        2 * answered.size
    );
    assert_eq!(
        device.new_heap_texture(heap, &double, None),
        Err(Error::HeapFull)
    );
    assert_eq!(
        device.memory().heap_refusals,
        HeapRefusals {
            fragmented: 1,
            ..full
        }
    );
}

fn color_targets() -> RenderTargets {
    let mut targets = RenderTargets::default();
    targets.color_attachments[0] = ColorAttachmentDescriptor {
        pixel_format: Some(PixelFormat::Rgba8Unorm),
        ..ColorAttachmentDescriptor::default()
    };
    targets
}

#[test]
fn functions_take_bicubic_argument_samplers_once_admitted() {
    let mut device = device();
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let request = bicubic_request(vertex, fragment);
    device.new_render_pipeline(&request).unwrap();
    assert!(
        device
            .compiler
            .compiled
            .iter()
            .all(|stage| !stage.bicubic_arguments)
    );

    // Only the admitted function compiles for bicubic samplers.
    device.admit_bicubic(fragment).unwrap();
    let compiled = device.compiler.compiled.len();
    device.new_render_pipeline(&request).unwrap();
    let stages: Vec<_> = device.compiler.compiled[compiled..]
        .iter()
        .map(|stage| (stage.bicubic_arguments, stage.bicubic_samplers))
        .collect();
    // Bound samplers filter bicubically in the variants draws bind.
    assert_eq!(stages, [(false, 0), (true, 0)]);
}

/// A pipeline made before its process's first pixel-coordinate or bicubic
/// sampler compiles again for its next draw, whose variant reads argument
/// samplers' flags: the order of creation does not change what it samples.
#[test]
fn pipelines_made_before_admission_compile_again_on_next_use() {
    let mut device = device();
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let id = device
        .new_render_pipeline(&bicubic_request(vertex, fragment))
        .unwrap();
    let (state_id, base) = {
        let state = device.render_pipeline_state(id).unwrap();
        (state.id, state.pipeline.unwrap())
    };
    let early = device.render_variant(state_id, [0, 1]).unwrap();
    device.admit_bicubic(vertex).unwrap();
    device.admit_bicubic(fragment).unwrap();
    device.outdate_pipelines(&[vertex, fragment]);
    assert!(device.render_pipeline_state(id).unwrap().outdated);
    let compiled = device.compiler.compiled.len();
    let late = device.render_variant(state_id, [0; 2]).unwrap();
    assert_ne!(late, base);
    assert_ne!(device.render_variant(state_id, [0, 1]).unwrap(), early);
    assert!(
        device.compiler.compiled[compiled..]
            .iter()
            .all(|stage| stage.bicubic_arguments)
    );
    assert_eq!(device.render_variant(state_id, [0; 2]).unwrap(), late);
}

fn bicubic_request(vertex: FunctionId, fragment: FunctionId) -> RenderPipelineRequest {
    RenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: Some(fragment),
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Triangle,
        max_vertex_amplification_count: 1,
        tessellation: Default::default(),
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: false,
        archives: Default::default(),
    }
}

#[test]
fn bicubic_render_variants_compile_once_per_sampler_mask() {
    let mut device = device();
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let id = device
        .new_render_pipeline(&bicubic_request(vertex, fragment))
        .unwrap();
    let state = device.render_pipeline_state(id).unwrap();
    let (state_id, base) = (state.id, state.pipeline.unwrap());
    assert_eq!(state.bicubic, [0; 2]);
    // The pipeline's own mask is the pipeline.
    assert_eq!(device.render_variant(state_id, [0; 2]).unwrap(), base);

    let compiled = device.compiler.compiled.len();
    let variant = device.render_variant(state_id, [0, 1 << 2]).unwrap();
    assert_ne!(variant, base);
    let stages: Vec<_> = device.compiler.compiled[compiled..]
        .iter()
        .map(|stage| stage.bicubic_samplers)
        .collect();
    assert_eq!(stages, [0, 1 << 2]);
    // The variant compiles once; another mask is another variant.
    assert_eq!(
        device.render_variant(state_id, [0, 1 << 2]).unwrap(),
        variant
    );
    assert_eq!(device.compiler.compiled.len(), compiled + 2);
    let other = device.render_variant(state_id, [1, 1 << 2]).unwrap();
    assert!(other != variant && other != base);
    assert_eq!(device.compiler.compiled.len(), compiled + 4);
    device.destroy_render_pipeline(id).unwrap();
}

#[test]
fn indirect_render_pipelines_read_each_samplers_filter_once_admitted() {
    let mut device = device();
    if device
        .backend()
        .physical_device()
        .limits
        .generated_commands
        .is_none()
    {
        return;
    }
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    device.admit_bicubic(fragment).unwrap();
    let request = RenderPipelineRequest {
        support_indirect_command_buffers: true,
        ..bicubic_request(vertex, fragment)
    };
    let id = device.new_render_pipeline(&request).unwrap();
    let stages: Vec<_> = device
        .compiler
        .compiled
        .iter()
        .map(|stage| stage.bicubic_samplers)
        .collect();
    assert_eq!(stages, [0, DYNAMIC_BICUBIC]);
    let state = device.render_pipeline_state(id).unwrap();
    assert_eq!(state.bicubic, [0, DYNAMIC_BICUBIC]);
}

#[test]
fn indirect_pipelines_share_execution_sets_by_state() {
    let mut device = device();
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let request = RenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: Some(fragment),
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Triangle,
        max_vertex_amplification_count: 1,
        tessellation: Default::default(),
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: true,
        archives: Default::default(),
    };
    let first = device.new_render_pipeline(&request).unwrap();
    let second = device.new_render_pipeline(&request).unwrap();
    let mut blended = request.clone();
    blended.targets.color_attachments[0].blending_enabled = true;
    let other = device.new_render_pipeline(&blended).unwrap();
    let plain = device
        .new_render_pipeline(&RenderPipelineRequest {
            support_indirect_command_buffers: false,
            archives: Default::default(),
            ..request.clone()
        })
        .unwrap();

    let handle = |device: &TestDevice, id| device.render_pipeline_resource_id(id).unwrap();
    assert_eq!(handle(&device, first), 1 << 32 | 1);
    assert_eq!(
        handle(&device, second),
        1 << 32 | 2,
        "the same state shares a set"
    );
    assert_eq!(
        handle(&device, other),
        2 << 32 | 1,
        "other state has its own"
    );
    assert_eq!(handle(&device, plain), 0);
    let graphics = &device.backend().graphics;
    let bindable: Vec<bool> = graphics.iter().map(|info| info.indirect_bindable).collect();
    assert_eq!(bindable, [true, true, true, false]);
    let sets = &device.backend().execution_sets;
    assert_eq!(sets.len(), 2);
    let first_set = sets.values().next().unwrap();
    assert_eq!(first_set.len(), 3, "index 0 repeats the initial pipeline");
    assert_eq!(first_set[&0], first_set[&1]);

    device.destroy_render_pipeline(other).unwrap();
    assert!(
        device
            .backend()
            .destroyed
            .iter()
            .any(|object| matches!(object, Object::ExecutionSet(_))),
        "the set goes with its last pipeline"
    );
    device.destroy_render_pipeline(second).unwrap();
    let third = device.new_render_pipeline(&request).unwrap();
    assert_eq!(
        handle(&device, third),
        1 << 32 | 3,
        "indices return only once idle"
    );

    let vulkan = |device: &TestDevice, id| device.render_pipeline_state(id).unwrap().pipeline;
    let (initial, kept) = (vulkan(&device, first), vulkan(&device, third));
    let classes = device.indirect_render_classes();
    assert_eq!(classes.len(), 1);
    assert_eq!(
        (classes[0].index, classes[0].pipeline),
        (0, initial.unwrap())
    );
    assert_eq!(classes[0].topology, PrimitiveTopologyClass::Triangle);
    let destroyed = |device: &mut TestDevice, pipeline: Option<Pipeline>| {
        let pipeline = Object::Pipeline(pipeline.unwrap());
        device.backend().destroyed.contains(&pipeline)
    };
    device.destroy_render_pipeline(first).unwrap();
    assert!(
        !destroyed(&mut device, initial),
        "the class keeps its initial pipeline"
    );
    assert_eq!(
        device.indirect_render_classes()[0].pipeline,
        kept.unwrap(),
        "executions bind a live pipeline"
    );
    device.destroy_render_pipeline(third).unwrap();
    assert!(destroyed(&mut device, initial) && destroyed(&mut device, kept));
    assert!(device.indirect_render_classes().is_empty());
}

#[test]
fn indirect_compute_pipelines_share_the_compute_execution_set() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let request = ComputePipelineRequest {
        compute_function: kernel,
        threadgroup_size_is_multiple_of_thread_execution_width: false,
        max_total_threads_per_threadgroup: 40,
        stage_input: None,
        linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: true,
        archives: Default::default(),
    };
    let first = device.new_compute_pipeline(&request).unwrap();
    let specialization = device.compiler.compiled.last().unwrap().clone();
    assert!(specialization.dynamic_threadgroups);
    assert_eq!(
        specialization.workgroup_size,
        Some([64, 1, 1]),
        "the most threads, in whole SIMD-groups"
    );
    assert!(device.backend().compute.last().unwrap().indirect_bindable);
    let second = device.new_compute_pipeline(&request).unwrap();
    let plain = device
        .new_compute_pipeline(&ComputePipelineRequest {
            support_indirect_command_buffers: false,
            archives: Default::default(),
            ..request.clone()
        })
        .unwrap();

    let handle = |device: &TestDevice, id| device.compute_pipeline_resource_id(id).unwrap();
    assert_eq!(handle(&device, first), u64::from(u32::MAX) << 32 | 1);
    assert_eq!(handle(&device, second), u64::from(u32::MAX) << 32 | 2);
    assert_eq!(handle(&device, plain), 0);
    assert_eq!(device.backend().execution_sets.len(), 1);

    let variant = |device: &TestDevice, id| {
        let state = device.compute_pipeline_state(id).unwrap();
        state.indirect.unwrap().pipeline
    };
    let (initial, kept) = (variant(&device, first), variant(&device, second));
    assert_eq!(device.indirect_compute_set().unwrap().1, initial);
    let destroyed = |device: &mut TestDevice, pipeline| {
        let destroyed = &device.backend().destroyed;
        destroyed.contains(&Object::Pipeline(pipeline))
    };
    device.destroy_compute_pipeline(first).unwrap();
    assert!(
        !destroyed(&mut device, initial),
        "the set keeps its initial pipeline"
    );
    assert_eq!(
        device.indirect_compute_set().unwrap().1,
        kept,
        "executions bind a live pipeline"
    );
    device.destroy_compute_pipeline(second).unwrap();
    assert!(destroyed(&mut device, initial) && destroyed(&mut device, kept));
    assert!(
        device
            .backend()
            .destroyed
            .iter()
            .any(|object| matches!(object, Object::ExecutionSet(_))),
        "the set goes with its last pipeline"
    );
    assert!(device.indirect_compute_set().is_none());
}

#[test]
fn generated_layouts_are_created_once_per_key() {
    let mut device = device();
    let key = GeneratedLayoutKey {
        stages: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
        execution_set: false,
        work: GeneratedWork::Draw {
            indexed: true,
            vertex_buffers: alloc::vec![0],
        },
    };
    let request = MemoryRequest {
        required: MemoryProperties::HOST_VISIBLE,
        preferred: MemoryProperties::EMPTY,
        mapped: true,
    };
    let (layout, requirements) = device.generated_layout(&key, None, 1, request).unwrap();
    assert_eq!((requirements.size, requirements.alignment), (64, 256));
    assert_eq!(
        device.generated_layout(&key, None, 1, request).unwrap().0,
        layout
    );
    let wide = GeneratedLayoutKey {
        work: GeneratedWork::Draw {
            indexed: true,
            vertex_buffers: (0..31).collect(),
        },
        ..key.clone()
    };
    assert!(
        matches!(
            device.generated_layout(&wide, None, 1, request),
            Err(Error::Unsupported(_))
        ),
        "more tokens than the device takes"
    );
    let limit = device.backend().physical_device().limits.generated_commands;
    let beyond = limit.unwrap().max_sequences + 1;
    assert!(
        matches!(
            device.generated_layout(&key, None, beyond, request),
            Err(Error::Unsupported(_))
        ),
        "more sequences than one execution runs"
    );
    assert_eq!(device.backend().layouts, [key.info()]);
}

#[test]
fn amplified_pipelines_compile_amplified_vertex_functions() {
    let mut device = device();
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let request = RenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: Some(fragment),
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Triangle,
        max_vertex_amplification_count: 2,
        tessellation: Default::default(),
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    device.new_render_pipeline(&request).unwrap();
    let amplified: Vec<bool> = device
        .compiler
        .compiled
        .iter()
        .map(|specialization| specialization.amplified)
        .collect();
    assert_eq!(amplified, [true, false]);
}

#[test]
fn functions_take_values_for_their_function_constants() {
    let mut device = device();
    let scale = ConstantValue::at(0, DataType::UInt, alloc::vec![2, 0, 0, 0]);
    let named = |name: &str, data_type, value| ConstantValue {
        key: ConstantKey::Name(String::from(name)),
        data_type,
        value,
    };
    let refused = [
        (Vec::new(), "a required function constant has no value"),
        (
            alloc::vec![ConstantValue::at(0, DataType::UInt, alloc::vec![2, 0])],
            "a function constant value of another type",
        ),
        (
            alloc::vec![ConstantValue::at(0, DataType::Int, alloc::vec![2, 0, 0, 0])],
            "a function constant value of another type",
        ),
        (
            alloc::vec![scale.clone(), scale.clone()],
            "a function constant has two values",
        ),
        (
            alloc::vec![
                scale.clone(),
                named("scale", DataType::UInt, alloc::vec![0; 4])
            ],
            "a function constant has two values",
        ),
    ];
    for (constants, reason) in refused {
        assert_eq!(
            device.new_function(LIBRARY, "constants", Some(&constants)),
            Err(Error::Invalid(reason))
        );
    }
    // tint is optional; values of constants the function lacks are ignored.
    for constants in [
        alloc::vec![scale.clone()],
        alloc::vec![named("scale", DataType::UInt, alloc::vec![2, 0, 0, 0])],
        alloc::vec![
            scale.clone(),
            ConstantValue::at(1, DataType::Half3, alloc::vec![0; 8]),
            ConstantValue::at(9, DataType::Bool, alloc::vec![1]),
            named("missing", DataType::Bool, alloc::vec![1]),
        ],
    ] {
        assert!(
            device
                .new_function(LIBRARY, "constants", Some(&constants))
                .is_ok()
        );
    }
}

#[test]
fn a_function_made_without_values_reflects_but_becomes_no_pipeline() {
    let mut device = device();
    let constants = device.new_function(LIBRARY, "constants", None).unwrap();
    let reflection = device.function_reflection(constants).unwrap();
    assert!(
        reflection
            .constants()
            .iter()
            .any(|constant| constant.required)
    );
    assert!(device.function_bindings(constants).is_ok());
    assert_eq!(
        device.new_compute_pipeline(&archived_compute(constants, PipelineArchives::default())),
        Err(Error::Invalid("a required function constant has no value"))
    );
    // One whose constants are all optional takes their defaults.
    let kernel = device.new_function(LIBRARY, "kernel", None).unwrap();
    let request = archived_compute(kernel, PipelineArchives::default());
    assert!(device.new_compute_pipeline(&request).is_ok());
}

fn archived_compute(function: FunctionId, archives: PipelineArchives) -> ComputePipelineRequest {
    ComputePipelineRequest {
        compute_function: function,
        threadgroup_size_is_multiple_of_thread_execution_width: false,
        max_total_threads_per_threadgroup: 0,
        stage_input: None,
        linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: false,
        archives,
    }
}

fn required(archive: ArchiveId) -> PipelineArchives {
    PipelineArchives {
        searched: alloc::vec![archive],
        required: true,
        added_to: None,
    }
}

fn added_to(archive: ArchiveId) -> PipelineArchives {
    PipelineArchives {
        added_to: Some(archive),
        ..PipelineArchives::default()
    }
}

/// A pipeline cache header of `vendor`, `device` and `uuid`.
fn cache_header(vendor: u32, device: u32, uuid: [u8; 16]) -> Vec<u8> {
    [32, 1, vendor, device]
        .iter()
        .flat_map(|word: &u32| word.to_le_bytes())
        .chain(uuid)
        .collect()
}

#[test]
fn archive_files_hold_pipelines_only_under_this_devices_header() {
    let mut device = device();
    let identity = device.backend().physical_device().identity.clone();
    let (vendor, id, uuid) = (
        identity.vendor_id,
        identity.device_id,
        identity.pipeline_cache_uuid,
    );
    assert!(device.archive_header_matches(&cache_header(vendor, id, uuid)));
    let mut other_uuid = uuid;
    other_uuid[15] ^= 1;
    assert!(!device.archive_header_matches(&cache_header(vendor, id, other_uuid)));
    assert!(!device.archive_header_matches(&cache_header(vendor, id + 1, uuid)));
    assert!(!device.archive_header_matches(&cache_header(0x106b, id, uuid)));
    let mut version_two = cache_header(vendor, id, uuid);
    version_two[4] = 2;
    assert!(!device.archive_header_matches(&version_two));
    let mut short = cache_header(vendor, id, uuid);
    short[0] = 16;
    assert!(!device.archive_header_matches(&short));
    assert!(!device.archive_header_matches(&cache_header(vendor, id, uuid)[..31]));
    assert!(!device.archive_header_matches(&[]));
}

#[test]
fn binary_archives_hold_what_was_added_and_outlive_their_files() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let empty = device.new_archive(&[]).unwrap();
    assert_eq!(
        device.new_compute_pipeline(&archived_compute(kernel, required(empty))),
        Err(Error::NotArchived)
    );
    assert!(
        device.backend().compute.is_empty(),
        "a miss compiles nothing"
    );

    let adding = archived_compute(kernel, added_to(empty));
    let added = device.new_compute_pipeline(&adding).unwrap();
    device.destroy_compute_pipeline(added).unwrap();
    assert_eq!(device.backend().compute.len(), 1);
    let found = device
        .new_compute_pipeline(&archived_compute(kernel, required(empty)))
        .unwrap();
    assert!(matches!(
        device.backend().compute.last().unwrap().source,
        PipelineSource::Archived(_)
    ));
    device.destroy_compute_pipeline(found).unwrap();

    // Its bytes make the same archive, found after one that lacks it.
    let data = device.archive_data(empty).unwrap();
    let loaded = device.new_archive(&data).unwrap();
    let missing = device.new_archive(&[]).unwrap();
    let archives = PipelineArchives {
        searched: alloc::vec![missing, loaded],
        required: true,
        added_to: None,
    };
    assert!(
        device
            .new_compute_pipeline(&archived_compute(kernel, archives))
            .is_ok()
    );
}

#[test]
fn binary_archives_learn_the_shapes_dispatches_compile() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let archive = device.new_archive(&[]).unwrap();
    let searched = PipelineArchives {
        searched: alloc::vec![archive],
        ..PipelineArchives::default()
    };
    let pipeline = device
        .new_compute_pipeline(&archived_compute(kernel, searched))
        .unwrap();
    assert!(device.backend().compute.is_empty(), "shapes compile later");
    assert_eq!(device.archive_data(archive), Ok(alloc::vec![0]));
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    assert_eq!(device.archive_data(archive), Ok(alloc::vec![1]));
}

#[test]
fn pipelines_keep_their_binary_archives() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let archive = device.new_archive(&[]).unwrap();
    let searched = PipelineArchives {
        searched: alloc::vec![archive],
        ..PipelineArchives::default()
    };
    let request = archived_compute(kernel, searched);
    let pipeline = device.new_compute_pipeline(&request).unwrap();
    device.destroy_archive(archive).unwrap();
    assert_eq!(
        device.destroy_archive(archive),
        Err(Error::Invalid("unknown or destroyed object"))
    );
    assert_eq!(
        device.new_compute_pipeline(&request),
        Err(Error::Invalid("unknown or destroyed object"))
    );
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    let cached = |device: &mut TestDevice| {
        device
            .backend()
            .destroyed
            .iter()
            .any(|object| matches!(object, Object::PipelineCache(_)))
    };
    assert!(!cached(&mut device));
    device.destroy_compute_pipeline(pipeline).unwrap();
    let serial = submit(&mut device);
    complete(&mut device, serial);
    assert!(cached(&mut device), "the last pipeline frees the archive");
}

#[test]
fn render_pipelines_come_from_binary_archives() {
    let mut device = device();
    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let archive = device.new_archive(&[]).unwrap();
    let request = |archives| RenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: None,
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Unspecified,
        max_vertex_amplification_count: 1,
        tessellation: TessellationDescriptor::default(),
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: false,
        archives,
    };
    assert_eq!(
        device.new_render_pipeline(&request(required(archive))),
        Err(Error::NotArchived)
    );
    let added = device
        .new_render_pipeline(&request(added_to(archive)))
        .unwrap();
    device.destroy_render_pipeline(added).unwrap();
    device
        .new_render_pipeline(&request(required(archive)))
        .unwrap();
    assert!(matches!(
        device.backend().graphics.last().unwrap().source,
        PipelineSource::Archived(_)
    ));
}

#[test]
fn mesh_pipelines_answer_for_binary_archives_at_creation() {
    let mut device = device();
    let mesh = device.new_function(LIBRARY, "mesh", Some(&[])).unwrap();
    let archive = device.new_archive(&[]).unwrap();
    let request = |archives| MeshPipelineRequest {
        object_function: None,
        mesh_function: mesh,
        fragment_function: None,
        targets: color_targets(),
        max_vertex_amplification_count: 1,
        object_threads: StageThreads::default(),
        mesh_threads: StageThreads::default(),
        payload_memory_length: 0,
        object_linked_functions: LinkedFunctions::default(),
        mesh_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        archives,
    };
    assert_eq!(
        device.new_mesh_render_pipeline(&request(required(archive))),
        Err(Error::NotArchived)
    );
    device
        .new_mesh_render_pipeline(&request(added_to(archive)))
        .unwrap();
    device
        .new_mesh_render_pipeline(&request(required(archive)))
        .unwrap();
}

#[test]
fn a_stitched_function_links_what_it_calls() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let visible = device.new_function(LIBRARY, "visible", Some(&[])).unwrap();
    assert_eq!(
        device.new_stitched_library(&[kernel], &[]),
        Err(Error::Invalid("stitched functions are visible functions"))
    );
    let library = device.new_stitched_library(&[visible], &[]).unwrap();
    let stitched = device.new_function(library, "visible", Some(&[])).unwrap();
    // A second stitch of the first links what the first calls too.
    let library = device.new_stitched_library(&[stitched], &[]).unwrap();
    let outer = device.new_function(library, "visible", Some(&[])).unwrap();
    device.destroy_function(visible).unwrap();
    device.destroy_function(stitched).unwrap();

    let request = ComputePipelineRequest {
        compute_function: kernel,
        threadgroup_size_is_multiple_of_thread_execution_width: false,
        max_total_threads_per_threadgroup: 0,
        stage_input: None,
        linked_functions: LinkedFunctions {
            functions: alloc::vec![outer],
            max_call_stack_depth: 1,
            ..LinkedFunctions::default()
        },
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    let pipeline = device.new_compute_pipeline(&request).unwrap();
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    let linking = &device.compiler.compiled.last().unwrap().linking;
    let libraries = |functions: &[LinkedFunction]| -> Vec<u64> {
        functions.iter().map(|function| function.library).collect()
    };
    assert_eq!(libraries(&linking.functions), [2]);
    assert_eq!(libraries(&linking.private_functions), [1, 0]);

    // A stitched kernel links them as its own calls.
    let stitched_kernel = device.new_function(library, "kernel", Some(&[])).unwrap();
    let request = ComputePipelineRequest {
        compute_function: stitched_kernel,
        linked_functions: LinkedFunctions::default(),
        ..request
    };
    let pipeline = device.new_compute_pipeline(&request).unwrap();
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    let linking = &device.compiler.compiled.last().unwrap().linking;
    assert!(linking.functions.is_empty());
    assert_eq!(libraries(&linking.private_functions), [1, 0]);
    // The libraries it keeps for them preload nothing.
    assert!(linking.libraries.is_empty());
}

#[test]
fn a_specialized_name_is_the_name_pipelines_link_a_function_by() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let visible = device.new_function(LIBRARY, "visible", Some(&[])).unwrap();
    let plain = device.new_function(LIBRARY, "visible", Some(&[])).unwrap();
    device.name_function(visible, "renamed").unwrap();
    // Its own name, or a stage's, renames nothing.
    device.name_function(plain, "visible").unwrap();
    device.name_function(kernel, "renamed_kernel").unwrap();
    // A stitch of it links it under that name too.
    let library = device.new_stitched_library(&[visible], &[]).unwrap();
    let stitched = device.new_function(library, "visible", Some(&[])).unwrap();
    device.destroy_function(visible).unwrap();

    let request = |private_functions| ComputePipelineRequest {
        compute_function: kernel,
        threadgroup_size_is_multiple_of_thread_execution_width: false,
        max_total_threads_per_threadgroup: 0,
        stage_input: None,
        linked_functions: LinkedFunctions {
            private_functions,
            max_call_stack_depth: 1,
            ..LinkedFunctions::default()
        },
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    let pipeline = device
        .new_compute_pipeline(&request(alloc::vec![plain, stitched]))
        .unwrap();
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    let linking = &device.compiler.compiled.last().unwrap().linking;
    let aliases: Vec<_> = linking
        .private_functions
        .iter()
        .map(|function| (function.entry.as_str(), function.alias.as_deref()))
        .collect();
    assert_eq!(
        aliases,
        [
            ("visible", None),
            ("visible", None),
            ("visible", Some("renamed"))
        ]
    );
}

#[test]
fn stages_link_visible_functions_and_name_them_by_handle() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let visible = device.new_function(LIBRARY, "visible", Some(&[])).unwrap();
    let tint = 1.0f32.to_le_bytes().to_vec();
    let tinted = device
        .new_function(
            LIBRARY,
            "tinted",
            Some(&[ConstantValue::at(0, DataType::Float, tint.clone())]),
        )
        .unwrap();
    let dynamic = device.new_library(fixture(false)).unwrap();
    assert_eq!(device.function_type(visible), Ok(FunctionType::Visible));
    assert_eq!(device.function_type(kernel), Ok(FunctionType::Kernel));
    assert!(device.function_interface(visible).is_err());

    let request = |functions: Vec<FunctionId>| ComputePipelineRequest {
        compute_function: kernel,
        threadgroup_size_is_multiple_of_thread_execution_width: false,
        max_total_threads_per_threadgroup: 0,
        stage_input: None,
        linked_functions: LinkedFunctions {
            functions,
            libraries: alloc::vec![dynamic],
            max_call_stack_depth: 4,
            ..LinkedFunctions::default()
        },
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    let pipeline = device
        .new_compute_pipeline(&request(alloc::vec![visible]))
        .unwrap();
    assert_eq!(
        device.compute_function_handle(pipeline, visible),
        Ok(Some(FunctionHandle(1)))
    );
    assert_eq!(device.compute_function_handle(pipeline, kernel), Ok(None));
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    assert_eq!(
        device.compiler.compiled.last().unwrap().linking,
        Linking {
            functions: alloc::vec![LinkedFunction {
                library: 0,
                entry: String::from("visible"),
                constants: Vec::new(),
                alias: None,
            }],
            private_functions: Vec::new(),
            libraries: alloc::vec![1],
            max_call_stack_depth: 4,
        }
    );
    assert_eq!(
        device.new_compute_pipeline(&request(alloc::vec![kernel])),
        Err(Error::Invalid("linked functions are visible functions"))
    );
    // A specialized function links with its values.
    let pipeline = device
        .new_compute_pipeline(&request(alloc::vec![tinted]))
        .unwrap();
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    assert_eq!(
        device.compiler.compiled.last().unwrap().linking.functions,
        [LinkedFunction {
            library: 0,
            entry: String::from("tinted"),
            constants: alloc::vec![(0, tint)],
            alias: None,
        }]
    );

    let vertex = device.new_function(LIBRARY, "vertex", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let render = RenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: Some(fragment),
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Triangle,
        max_vertex_amplification_count: 1,
        tessellation: Default::default(),
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions {
            functions: alloc::vec![visible],
            ..LinkedFunctions::default()
        },
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    let pipeline = device.new_render_pipeline(&render).unwrap();
    let handle =
        |device: &TestDevice, stage| device.render_function_handle(pipeline, stage, visible);
    assert_eq!(
        handle(&device, Stage::Fragment),
        Ok(Some(FunctionHandle(1)))
    );
    assert_eq!(handle(&device, Stage::Vertex), Ok(None));
    let linked: Vec<_> = device.compiler.compiled[device.compiler.compiled.len() - 2..]
        .iter()
        .map(|specialization| specialization.linking.functions.len())
        .collect();
    assert_eq!(linked, [0, 1], "only the fragment stage links");
}

#[test]
fn visible_function_tables_hold_handles() {
    let mut device = device();
    assert!(device.new_visible_function_table(0).is_err());
    let table = device.new_visible_function_table(4).unwrap();
    let (memory, offset, length) = device.buffer_memory(table).unwrap();
    assert_eq!(length, 40);
    device
        .set_visible_functions(table, 2, &[Some(FunctionHandle(3)), None])
        .unwrap();
    assert_eq!(
        device.set_visible_functions(table, 3, &[None, None]),
        Err(Error::Invalid("functions past the table's end"))
    );
    let mut bytes = [0xff; 40];
    device
        .backend()
        .read_memory(memory, offset, &mut bytes)
        .unwrap();
    let words: Vec<u64> = bytes
        .chunks(8)
        .map(|word| u64::from_le_bytes(word.try_into().unwrap()))
        .collect();
    assert_eq!(words, [4, 0, 0, 3, 0], "a count, then no functions");
}

#[test]
fn source_libraries_name_their_functions_and_keep_what_they_load_with() {
    let mut device = device();
    let air = b"BC\xc0\xde\0\0\0\0";
    let dependency = device.new_library(fixture(false)).unwrap();
    let library = device
        .new_source_library(
            air,
            Some("lib/source"),
            &[dependency],
            ConversionRounding::TowardZero,
        )
        .unwrap();
    // A dynamic one lists what it exports too, as Metal's compiler does.
    assert_eq!(
        device.library_function_names(library),
        Ok(&[
            String::from("kernel"),
            String::from("visible"),
            String::from("_Z6helperf")
        ][..])
    );
    assert_eq!(
        device.library_identity(library),
        Ok((true, Some("lib/source")))
    );

    // Its functions link what the options named.
    let kernel = device.new_function(library, "kernel", Some(&[])).unwrap();
    let pipeline = device
        .new_compute_pipeline(&ComputePipelineRequest {
            compute_function: kernel,
            threadgroup_size_is_multiple_of_thread_execution_width: false,
            max_total_threads_per_threadgroup: 0,
            stage_input: None,
            linked_functions: LinkedFunctions::default(),
            support_indirect_command_buffers: false,
            archives: Default::default(),
        })
        .unwrap();
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    assert_eq!(
        device.compiler.compiled.last().unwrap().linking.libraries,
        [1]
    );

    // And they load for as long as it does.
    let kept = device.new_library(fixture(false)).unwrap();
    let keeper = device
        .new_source_library(air, None, &[kept], ConversionRounding::ToNearestEven)
        .unwrap();
    assert_eq!(device.library_identity(keeper), Ok((false, None)));
    device.destroy_library(kept).unwrap();
    assert!(device.compiler.libraries[3].is_some());
    device.destroy_library(keeper).unwrap();
    assert!(device.compiler.libraries[3].is_none());
    assert!(
        device
            .new_source_library(air, None, &[kept], ConversionRounding::ToNearestEven)
            .is_err()
    );
}

#[test]
fn libraries_of_held_bytes_share_them() {
    let mut device = device();
    let data = fixture(false);
    let length = data.len() as u64;
    let digest = library_digest(&data);
    let mut other = data.clone();
    other[0] ^= 1;
    assert_eq!(
        device.share_library(length, &library_digest(&other)),
        Ok(None)
    );
    assert_eq!(
        device.share_library(length + 1, &digest),
        Ok(None),
        "only bytes of the same length share"
    );
    let shared = device.share_library(length, &digest).unwrap().unwrap();
    assert_ne!(shared, LIBRARY);
    assert_eq!(
        device.library_function_names(shared),
        device.library_function_names(LIBRARY)
    );

    // Each lives on without the other.
    device.destroy_library(LIBRARY).unwrap();
    assert!(device.new_function(shared, "kernel", Some(&[])).is_ok());
}

#[test]
fn libraries_name_the_dynamic_libraries_they_link() {
    let mut device = device();
    let data = linking(b"user.metallib\0", &[b"@loader_path/dynamic.metallib\0"]);
    let library = device.new_library(data.clone()).unwrap();
    assert_eq!(
        device.library_linked_names(library),
        Ok(&[String::from("@loader_path/dynamic.metallib")][..])
    );
    // A library of the same bytes links the same.
    let shared = device
        .share_library(data.len() as u64, &library_digest(&data))
        .unwrap()
        .unwrap();
    assert_eq!(
        device.library_linked_names(shared),
        device.library_linked_names(library)
    );
    let plain = device.new_library(fixture(false)).unwrap();
    assert_eq!(device.library_linked_names(plain), Ok(&[][..]));
}

#[test]
fn extern_functions_are_named_apart_and_make_no_function() {
    let mut device = device();
    let air = b"BC\xc0\xde\0\0\0\0";
    let data = library::assemble(
        air,
        &[("kernel", 2), ("declared", Function::EXTERN)],
        0,
        None,
    );
    let library = device.new_library(data).unwrap();
    assert_eq!(
        device.library_function_names(library),
        Ok(&[String::from("kernel")][..])
    );
    assert_eq!(
        device.library_extern_function_names(library),
        Ok(&[String::from("declared")][..])
    );
    assert_eq!(
        device.new_function(library, "declared", Some(&[])),
        Err(Error::Unsupported("the function of an extern declaration"))
    );
}

#[test]
fn pipelines_keep_their_functions_and_libraries() {
    let mut device = device();
    let library = device.new_library(fixture(false)).unwrap();
    assert_eq!(
        device.library_function_names(library),
        Ok(&[String::from("kernel")][..])
    );
    assert_eq!(
        device.new_library(alloc::vec![0; 16]),
        Err(Error::Invalid("Metal library"))
    );
    let kernel = device.new_function(library, "kernel", Some(&[])).unwrap();
    let visible = device.new_function(library, "visible", Some(&[])).unwrap();
    let pipeline = device
        .new_compute_pipeline(&ComputePipelineRequest {
            compute_function: kernel,
            threadgroup_size_is_multiple_of_thread_execution_width: false,
            max_total_threads_per_threadgroup: 0,
            stage_input: None,
            linked_functions: LinkedFunctions {
                functions: alloc::vec![visible],
                ..LinkedFunctions::default()
            },
            support_indirect_command_buffers: false,
            archives: Default::default(),
        })
        .unwrap();
    device.destroy_function(kernel).unwrap();
    device.destroy_function(visible).unwrap();
    device.destroy_library(library).unwrap();
    // Nothing new may use them, nor destroy them twice...
    let destroyed = Error::Invalid("unknown or destroyed object");
    assert_eq!(
        device.new_function(library, "kernel", Some(&[])),
        Err(destroyed)
    );
    assert_eq!(device.destroy_function(kernel), Err(destroyed));
    assert_eq!(device.destroy_library(library), Err(destroyed));
    // ...but the pipeline still compiles from them and names its functions.
    let dispatch = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    device
        .compute_variant(u64::from(pipeline.0), &dispatch)
        .unwrap();
    assert_eq!(
        device.compute_function_handle(pipeline, visible),
        Ok(Some(FunctionHandle(1)))
    );
    assert!(device.compiler.libraries[1].is_some());
    device.destroy_compute_pipeline(pipeline).unwrap();
    assert!(device.compiler.libraries[1].is_none());
    assert_eq!(device.function_type(kernel), Err(destroyed));
}

#[test]
fn render_pipelines_compile_their_functions_once() {
    let mut device = device();
    let vertex = device
        .new_function(
            LIBRARY,
            "vertex",
            Some(&[ConstantValue::at(
                0,
                DataType::UInt,
                alloc::vec![1, 0, 0, 0],
            )]),
        )
        .unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    assert!(device.new_function(LIBRARY, "missing", Some(&[])).is_err());
    let request = RenderPipelineRequest {
        vertex_function: vertex,
        fragment_function: Some(fragment),
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Triangle,
        max_vertex_amplification_count: 1,
        tessellation: Default::default(),
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    let pipeline = device.new_render_pipeline(&request).unwrap();
    assert_eq!(
        device.compiler.compiled[0].constants,
        [(0, alloc::vec![1, 0, 0, 0])]
    );
    for specialization in &device.compiler.compiled {
        let linked: Vec<_> = specialization
            .varyings
            .iter()
            .map(|varying| varying.identifier.as_str())
            .collect();
        assert_eq!(linked, ["user(color)"], "both stages link");
    }
    assert_eq!(
        device.compiler.released.len(),
        2,
        "NIR is released once the pipeline exists"
    );
    let state = device.render_pipeline_state(pipeline).unwrap().clone();
    assert!(state.pipeline.is_some());
    device.destroy_render_pipeline(pipeline).unwrap();
    assert!(
        device
            .backend()
            .destroyed
            .contains(&Object::Pipeline(state.pipeline.unwrap()))
    );

    let bad = RenderPipelineRequest {
        vertex_function: fragment,
        ..request.clone()
    };
    assert!(device.new_render_pipeline(&bad).is_err());
    assert_eq!(
        device.compiler.released.len(),
        4,
        "failed pipelines release their NIR"
    );

    let unlinked = device.new_function(LIBRARY, "unlinked", Some(&[])).unwrap();
    let unlinked = RenderPipelineRequest {
        fragment_function: Some(unlinked),
        ..request
    };
    let compiled = device.compiler.compiled.len();
    assert!(device.new_render_pipeline(&unlinked).is_err());
    assert_eq!(
        device.compiler.compiled.len(),
        compiled,
        "unlinked functions compile nothing"
    );
}

#[test]
fn extended_range_texels_cross_the_cpu_as_metal_bytes() {
    let mut device = device_on(MockBackend::new());
    let descriptor = TextureDescriptor {
        width: 2,
        height: 1,
        options: options(StorageMode::Shared),
        ..texture(PixelFormat::Bgr10Xr, TextureUsage::SHADER_READ)
    };
    let texture = device.new_texture(&descriptor).unwrap();
    let region = Region {
        slice: 0,
        level: 0,
        origin: [0, 0, 0],
        size: [2, 1, 1],
    };
    let tight = Strides {
        bytes_per_row: 8,
        bytes_per_image: 0,
    };
    // Red 1023, green 894, blue 384; then every channel at 0.
    let pack = |red: u32, green: u32, blue: u32| blue | green << 10 | red << 20 | 3 << 30;
    let written: Vec<u8> = [pack(1023, 894, 384), pack(0, 0, 0)]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect();
    device
        .replace_region(texture, &region, &written, tight)
        .unwrap();
    let image = device.texture_image(texture).unwrap();
    let stored = &device.backend.image_bytes[&image.0];
    let half =
        |at: usize| extended_range::f16_value(u16::from_le_bytes([stored[at], stored[at + 1]]));
    assert_eq!(stored.len(), 16, "half floats, four channels a texel");
    assert!((half(0) - 639.0 / 510.0).abs() < 1e-3);
    assert_eq!([half(2), half(4), half(6)], [1.0, 0.0, 1.0]);
    assert!((half(8) + 384.0 / 510.0).abs() < 1e-3);
    let mut read = [0; 8];
    device
        .get_bytes(texture, &region, &mut read, tight)
        .unwrap();
    assert_eq!(read[..], written[..]);
}

#[test]
fn managed_textures_keep_a_cpu_copy_without_host_image_copy() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let descriptor = TextureDescriptor {
        width: 8,
        height: 4,
        mipmap_level_count: 2,
        options: options(StorageMode::Managed),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let texture = device.new_texture(&descriptor).unwrap();
    let region = Region {
        slice: 0,
        level: 0,
        origin: [2, 1, 0],
        size: [2, 2, 1],
    };
    let tight = Strides {
        bytes_per_row: 8,
        bytes_per_image: 0,
    };
    let written: Vec<u8> = (0..16).collect();
    device
        .replace_region(texture, &region, &written, tight)
        .unwrap();
    let mut read = [0; 16];
    device
        .get_bytes(texture, &region, &mut read, tight)
        .unwrap();
    assert_eq!(read[..], written[..], "reads come from the CPU copy");
    // Rows of 8 bytes, 12 apart: the 4 between them are left alone.
    let strided = Strides {
        bytes_per_row: 12,
        ..tight
    };
    let mut read = [0xee; 20];
    device
        .get_bytes(texture, &region, &mut read, strided)
        .unwrap();
    assert_eq!(read[..8], written[..8]);
    assert_eq!(read[8..12], [0xee; 4]);
    assert_eq!(read[12..], written[8..]);
    let refused = [
        (&written[..12], tight, "the bytes end before the region"),
        (
            &written[..],
            Strides {
                bytes_per_row: 4,
                ..tight
            },
            "bytesPerRow is smaller than a row",
        ),
    ];
    for (bytes, strides, reason) in refused {
        assert_eq!(
            device.replace_region(texture, &region, bytes, strides),
            Err(Error::Invalid(reason))
        );
    }

    // Bands of whole rows, each within CPU_BAND_BYTES of the caller's bytes.
    assert_eq!(
        device.cpu_bands(texture, &region, tight).unwrap(),
        [CpuBand {
            region,
            bytes: 0..16
        }]
    );
    let apart = Strides {
        bytes_per_row: CPU_BAND_BYTES / 2 + 1,
        ..tight
    };
    let row = |y, start| CpuBand {
        region: Region {
            origin: [2, y, 0],
            size: [2, 1, 1],
            ..region
        },
        bytes: start..start + 8,
    };
    assert_eq!(
        device.cpu_bands(texture, &region, apart).unwrap(),
        [row(1, 0), row(2, CPU_BAND_BYTES / 2 + 1)]
    );
    let overflowing = Strides {
        bytes_per_row: u64::MAX,
        ..tight
    };
    assert_eq!(
        device.cpu_bands(texture, &region, overflowing),
        Err(Error::Invalid("the bytes end before the region"))
    );

    let level1 = device.texture_cpu_copy(texture, 1, 0).unwrap().unwrap();
    assert_eq!((level1.offset, level1.bytes_per_row), (8 * 4 * 4, 4 * 4));

    submit(&mut device);
    let segment = &device.backend().submissions.last().unwrap()[0];
    let Command::CopyBufferToImage { src, region, .. } = &segment.commands[1] else {
        panic!("{:?}", segment.commands[1]);
    };
    assert_eq!(*src, level1.buffer.buffer);
    assert_eq!(
        (
            region.buffer_offset,
            region.buffer_row_length,
            region.offset
        ),
        ((8 * 4) + 2 * 4, 8, [2, 1, 0])
    );
    assert!(matches!(segment.commands[2], Command::Barrier(_)));
    submit(&mut device);
    let segment = &device.backend().submissions.last().unwrap()[0];
    assert!(
        !segment
            .commands
            .iter()
            .any(|command| matches!(command, Command::CopyBufferToImage { .. })),
        "each write uploads once"
    );
}

#[test]
fn decoded_textures_keep_their_blocks_and_decode_written_subresources() {
    let mut device = device();
    let descriptor = TextureDescriptor {
        width: 8,
        height: 8,
        array_length: 2,
        texture_type: TextureType::D2Array,
        options: options(StorageMode::Shared),
        ..texture(PixelFormat::EacRgba8, TextureUsage::SHADER_READ)
    };
    let texture = device.new_texture(&descriptor).unwrap();
    assert_eq!(
        device.texture_cpu_copy(texture, 0, 0),
        Ok(None),
        "the blocks are no staged copy"
    );
    let blocks = device.texture_blocks(texture).unwrap().unwrap();
    assert_eq!(blocks.length, 2 * 2 * 2 * 16);
    // The second slice's two lower blocks, rows of 32 bytes.
    let region = Region {
        slice: 1,
        level: 0,
        origin: [0, 4, 0],
        size: [8, 4, 1],
    };
    let strides = Strides {
        bytes_per_row: 32,
        bytes_per_image: 0,
    };
    let written: Vec<u8> = (0..32).collect();
    device
        .replace_region(texture, &region, &written, strides)
        .unwrap();
    // Twice: the slice decodes once.
    device
        .replace_region(texture, &region, &written, strides)
        .unwrap();
    let mut read = [0; 32];
    device
        .get_bytes(texture, &region, &mut read, strides)
        .unwrap();
    assert_eq!(read[..], written[..], "reads return the blocks");

    submit(&mut device);
    let segment = &device.backend().submissions.last().unwrap()[0];
    let decodes: Vec<_> = segment
        .commands
        .iter()
        .filter_map(|command| match *command {
            Command::DecodeBlocks { kernel, count, .. } => Some((kernel, count)),
            _ => None,
        })
        .collect();
    assert_eq!(
        decodes,
        [(BlockKernel::Etc, 64)],
        "the written slice decodes whole"
    );
    assert!(
        !segment
            .commands
            .iter()
            .any(|command| matches!(command, Command::CopyBufferToImage { .. })),
        "the image holds texels, not blocks"
    );

    device
        .replace_region(texture, &region, &written, strides)
        .unwrap();
    device.destroy_texture(texture).unwrap();
    submit(&mut device);
    let segment = &device.backend().submissions.last().unwrap()[0];
    assert!(
        !segment
            .commands
            .iter()
            .any(|command| matches!(command, Command::DecodeBlocks { .. })),
        "a destroyed texture's decodes are dropped"
    );
}

#[test]
fn texture_transfers_keep_strided_padding_and_coalesce_contiguous_rows() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let descriptor = TextureDescriptor {
        texture_type: TextureType::D3,
        width: 2,
        height: 2,
        depth: 2,
        options: options(StorageMode::Managed),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let texture = device.new_texture(&descriptor).unwrap();
    let region = Region {
        slice: 0,
        level: 0,
        origin: [0; 3],
        size: [2; 3],
    };
    let strides = Strides {
        bytes_per_row: 12,
        bytes_per_image: 28,
    };
    let ranges = [0..8, 12..20, 28..36, 40..48];
    let pixels = core::array::from_fn::<_, 32, _>(|index| index as u8);
    let (rows, _) = pixels.as_chunks::<8>();
    let mut upload = [0xaa; 48];
    for (range, row) in ranges.iter().zip(rows) {
        upload[range.clone()].copy_from_slice(row);
    }
    let previous = device.backend().writes.len();
    device
        .replace_region(texture, &region, &upload, strides)
        .unwrap();
    let [(_, _, written)] = &device.backend().writes[previous..] else {
        panic!("contiguous texture data should reach the backend in one write");
    };
    assert_eq!(written, &pixels);

    let mut read = [0xee; 49];
    device
        .get_bytes(texture, &region, &mut read, strides)
        .unwrap();
    let mut expected = [0xee; 49];
    for (range, row) in ranges.into_iter().zip(rows) {
        expected[range].copy_from_slice(row);
    }
    assert_eq!(read, expected);
    assert_eq!(
        device.replace_region(
            texture,
            &region,
            &upload,
            Strides {
                bytes_per_row: u64::MAX,
                bytes_per_image: u64::MAX,
            },
        ),
        Err(Error::Invalid("bytesPerImage is smaller than an image"))
    );
}

#[test]
fn post_tessellation_functions_compile_with_their_feeding_stages() {
    let mut device = device();
    let patch = device.new_function(LIBRARY, "patch", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let request = RenderPipelineRequest {
        vertex_function: patch,
        fragment_function: Some(fragment),
        vertex_descriptor: None,
        targets: color_targets(),
        input_primitive_topology: PrimitiveTopologyClass::Unspecified,
        max_vertex_amplification_count: 1,
        tessellation: TessellationDescriptor {
            partition_mode: TessellationPartitionMode::FractionalOdd,
            max_tessellation_factor: 64,
            ..TessellationDescriptor::default()
        },
        vertex_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        support_indirect_command_buffers: false,
        archives: Default::default(),
    };
    let pipeline = device.new_render_pipeline(&request).unwrap();
    let tessellation = &device.compiler.tessellations[0];
    assert_eq!(
        (tessellation.partition, tessellation.max_factor),
        (TessellationPartitionMode::FractionalOdd, 64)
    );
    let state = device.render_pipeline_state(pipeline).unwrap();
    let stages: Vec<_> = state
        .plan
        .info
        .stages
        .iter()
        .map(|(stage, _)| *stage)
        .collect();
    assert_eq!(
        stages,
        [
            ShaderStages::VERTEX,
            ShaderStages::TESSELLATION_CONTROL,
            ShaderStages::TESSELLATION_EVALUATION,
            ShaderStages::FRAGMENT
        ]
    );
    assert_eq!(state.plan.info.topology, PrimitiveTopology::PatchList);
    assert_eq!(
        device.compiler.released.len(),
        4,
        "every stage's NIR is released"
    );

    let odd = RenderPipelineRequest {
        tessellation: TessellationDescriptor {
            max_tessellation_factor: 15,
            ..TessellationDescriptor::default()
        },
        ..request.clone()
    };
    assert!(matches!(
        device.new_render_pipeline(&odd),
        Err(Error::Invalid(_))
    ));
    let too_large = RenderPipelineRequest {
        tessellation: TessellationDescriptor {
            max_tessellation_factor: 128,
            ..TessellationDescriptor::default()
        },
        ..request
    };
    assert_eq!(
        device.new_render_pipeline(&too_large),
        Err(Error::Unsupported("tessellation factor"))
    );
    assert_eq!(
        device.compiler.compiled.len(),
        2,
        "invalid tessellation compiles nothing"
    );
}

#[test]
fn compute_and_mesh_variants_compile_per_shape() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let pipeline = device
        .new_compute_pipeline(&ComputePipelineRequest {
            compute_function: kernel,
            threadgroup_size_is_multiple_of_thread_execution_width: false,
            max_total_threads_per_threadgroup: 0,
            stage_input: None,
            linked_functions: LinkedFunctions::default(),
            support_indirect_command_buffers: false,
            archives: Default::default(),
        })
        .unwrap();
    let state = device.compute_pipeline_state(pipeline).unwrap().clone();
    let buffer = device
        .new_buffer(4096, options(StorageMode::Private))
        .unwrap();
    let buffer = device.buffer(buffer).unwrap();

    let mut commands = device.new_command_buffer();
    commands.begin_compute(DispatchType::Serial).unwrap();
    commands.set_compute_pipeline_state(&state).unwrap();
    commands.set_compute_buffer(Some(buffer), 0, 0).unwrap();
    let size = |width| Size {
        width,
        height: 1,
        depth: 1,
    };
    for threads in [64, 64, 128] {
        commands
            .dispatch_threadgroups(&mut device.resolver(), [1, 1, 1], size(threads))
            .unwrap();
    }
    commands.end_encoding(&mut device.resolver()).unwrap();
    let serial = device.commit(commands).unwrap();
    assert_eq!(
        device.backend().compute.len(),
        2,
        "one variant per threadgroup size"
    );
    assert_eq!(device.backend().compute[0].required_subgroup_size, 32);
    let submission = device.backend().submissions.last().unwrap().clone();
    let signals = &submission.last().unwrap().signals;
    assert_eq!(signals, &[(device.timeline, serial)]);
    // Serial dispatches wait for each other: each barrier after one is
    // marked for the report, then the end.
    let commands: Vec<_> = submission
        .iter()
        .flat_map(|segment| &segment.commands)
        .collect();
    let stamps: Vec<_> = commands
        .iter()
        .enumerate()
        .filter_map(|(at, command)| match command {
            Command::WriteTimestamp { query, .. } => Some((at, *query)),
            _ => None,
        })
        .collect();
    let (&(end_at, end), marks) = stamps.split_last().unwrap();
    assert_eq!(end_at, commands.len() - 1);
    assert_eq!(end as usize, marks.len());
    for (index, &(at, query)) in marks.iter().enumerate() {
        assert_eq!(query as usize, index);
        if query != 0 {
            assert!(matches!(commands[at + 1], Command::Barrier(_)));
            assert!(matches!(commands[at - 1], Command::Dispatch(_)));
        }
    }
    assert!(
        marks.len() >= 3,
        "a mark after each serial dispatch but the last"
    );
    assert!(
        !device.backend().writes.is_empty(),
        "argument tables reach the upload chunk"
    );

    let object = device.new_function(LIBRARY, "object", Some(&[])).unwrap();
    let mesh = device.new_function(LIBRARY, "mesh", Some(&[])).unwrap();
    let fragment = device.new_function(LIBRARY, "fragment", Some(&[])).unwrap();
    let request = MeshPipelineRequest {
        object_function: Some(object),
        mesh_function: mesh,
        fragment_function: Some(fragment),
        targets: color_targets(),
        max_vertex_amplification_count: 1,
        object_threads: StageThreads::default(),
        mesh_threads: StageThreads::default(),
        payload_memory_length: 0,
        object_linked_functions: LinkedFunctions::default(),
        mesh_linked_functions: LinkedFunctions::default(),
        fragment_linked_functions: LinkedFunctions::default(),
        archives: Default::default(),
    };
    let compiled = device.compiler.compiled.len();
    let id = device.new_mesh_render_pipeline(&request).unwrap();
    let first = device
        .mesh_variant(u64::from(id.0), [32, 1, 1], [64, 1, 1], &[])
        .unwrap();
    let linked = [("user(color)", false), ("user(normal)", true)];
    let stages: Vec<_> = device.compiler.compiled[compiled..]
        .iter()
        .map(|specialization| {
            let varyings: Vec<_> = specialization
                .varyings
                .iter()
                .map(|varying| (varying.identifier.as_str(), varying.per_primitive))
                .collect();
            (varyings, specialization.launched_by_grid)
        })
        .collect();
    assert_eq!(
        stages,
        [
            (linked.to_vec(), false),
            (Vec::new(), true),
            (linked.to_vec(), false),
        ],
        "the fragment, then the object function the draw launches and the mesh function"
    );
    let again = device
        .mesh_variant(u64::from(id.0), [32, 1, 1], [64, 1, 1], &[])
        .unwrap();
    assert_eq!(first, again);
    let sized = device
        .mesh_variant(u64::from(id.0), [32, 1, 1], [64, 1, 1], &[(0, 256)])
        .unwrap();
    assert_ne!(first, sized, "threadgroup memory is part of a variant");
    let object = &device.compiler.compiled[device.compiler.compiled.len() - 2];
    assert_eq!(object.threadgroup_lengths, [(0, 256)]);
    assert_eq!(
        device.compiler.compiled.last().unwrap().threadgroup_lengths,
        []
    );
    let stages: Vec<_> = device
        .backend()
        .graphics
        .last()
        .unwrap()
        .stages
        .iter()
        .map(|(stage, _)| *stage)
        .collect();
    use crate::metal::vulkan::ShaderStages as S;
    assert_eq!(stages, [S::TASK, S::MESH, S::FRAGMENT]);
    assert!(
        device
            .mesh_variant(u64::from(id.0), [32, 1, 1], [4096, 1, 1], &[])
            .is_err(),
        "beyond the mesh limit"
    );
    device.destroy_render_pipeline(id).unwrap();
    complete(&mut device, serial);
}

/// A color attachment over a buffer's rows, as an IOSurface's texture is,
/// shares its pass with a memoryless attachment, as Core Animation's passes
/// do: NVK renders the linear image through a tiled shadow, so draws that
/// read the framebuffer access a storage copy the image fills before them
/// and which returns before a draw renders through the blend unit and where
/// the pass ends. An optimal image's draws access its own texels.
#[test]
fn framebuffer_reads_of_a_linear_attachment_run_on_its_copy() {
    let mut device = device();
    assert!(device.capabilities().framebuffer_reads);
    let shared = options(StorageMode::Shared);
    let buffer = device.new_buffer(1 << 16, shared).unwrap();
    let mut descriptor = texture(
        PixelFormat::Rgba8Unorm,
        TextureUsage::RENDER_TARGET | TextureUsage::SHADER_READ,
    );
    descriptor.width = 64;
    descriptor.height = 32;
    descriptor.options = shared;
    let linear = device
        .new_texture_from_buffer(buffer, &descriptor, 0, 256)
        .unwrap();
    descriptor.options = options(StorageMode::Private);
    let optimal = device.new_texture(&descriptor).unwrap();
    descriptor.options = options(StorageMode::Memoryless);
    descriptor.usage = TextureUsage::RENDER_TARGET;
    let memoryless = device.new_texture(&descriptor).unwrap();
    let vertices = device.new_buffer(256, shared).unwrap();
    let vertices = device.buffer(vertices).unwrap();
    let attachment = |texture, load_action, store_action| AttachmentRequest {
        texture,
        level: 0,
        slice: 0,
        depth_plane: 0,
        resolve: None,
        load_action,
        store_action,
        store_action_options: StoreActionOptions::EMPTY,
        clear: [0.0; 4],
        surface: None,
    };
    let copies = |device: &mut TestDevice, target| {
        let mut colors = [None; MAX_COLOR_ATTACHMENTS];
        colors[0] = Some(attachment(target, LoadAction::Load, StoreAction::Store));
        colors[1] = Some(attachment(
            memoryless,
            LoadAction::DontCare,
            StoreAction::DontCare,
        ));
        let request = RenderPassRequest {
            color_attachments: colors,
            depth_attachment: None,
            depth_resolve_filter: MultisampleDepthResolveFilter::Sample0,
            stencil_attachment: None,
            stencil_resolve_filter: MultisampleStencilResolveFilter::Sample0,
            visibility_result_buffer: None,
            visibility_result_type: VisibilityResultType::Reset,
            render_target_array_length: 0,
            render_target_width: 0,
            render_target_height: 0,
            default_raster_sample_count: 0,
            tile_memory: Default::default(),
            sample_positions: Vec::new(),
            rasterization_rate_map: false,
            counter_samples: Vec::new(),
        };
        let mut reads = crate::metal::command_buffer::tests::render_state(1);
        reads.plan.color_formats[1] = None;
        reads.interfaces[1].storage_attachments = 1;
        let mut blended = crate::metal::command_buffer::tests::render_state(1);
        blended.plan.color_formats[1] = None;
        let mut commands = device.new_command_buffer();
        device.begin_render(&mut commands, &request).unwrap();
        commands
            .set_render_buffer(Stage::Vertex, Some(vertices), 0, 2)
            .unwrap();
        for state in [&reads, &blended, &reads] {
            commands.set_render_pipeline_state(state).unwrap();
            commands
                .draw_primitives(&mut device.resolver(), PrimitiveType::Triangle, 0, 3, 1, 0)
                .unwrap();
        }
        commands.end_encoding(&mut device.resolver()).unwrap();
        let serial = device.commit(commands).unwrap();
        let mut order = String::new();
        let mut pairs = Vec::new();
        for command in &device.backend().submissions.last().unwrap()[0].commands {
            match command {
                Command::CopyImage { src, dst, .. } => {
                    order.push('c');
                    pairs.push((*src, *dst));
                }
                Command::BeginRendering(_) => order.push('b'),
                Command::EndRendering => order.push('e'),
                _ => {}
            }
        }
        complete(device, serial);
        (order, pairs)
    };
    let (order, pairs) = copies(&mut device, linear);
    // In, back before the blended draw, in again, back where the pass
    // ends; each between pass instances, after an instance's store has
    // brought what it rendered to the image and before the next loads it.
    assert_eq!(order, "becbecbecbecbe");
    assert_eq!(pairs.len(), 4);
    let (to, from) = (pairs[0], (pairs[0].1, pairs[0].0));
    assert_eq!(pairs, [to, from, to, from]);
    let (order, pairs) = copies(&mut device, optimal);
    assert_eq!((order.as_str(), pairs.len()), ("be", 0));
}

/// Core Animation's compositor passes on macOS 27: a stored color target
/// (loaded, or a cleared two-plane extended-range one) beside memoryless
/// color attachments of other formats that the shader can read, which its
/// tile shaders use as imageblock data and which the pass neither loads
/// nor stores.
#[test]
fn compositor_passes_with_readable_memoryless_attachments_begin() {
    let mut device = device();
    assert!(device.capabilities().framebuffer_reads);
    let usage = TextureUsage::SHADER_READ | TextureUsage::RENDER_TARGET;
    let new = |device: &mut TestDevice, format, height, storage| {
        let descriptor = TextureDescriptor {
            width: 1920,
            height,
            options: options(storage),
            ..texture(format, usage)
        };
        device.new_texture(&descriptor).unwrap()
    };
    let attachment = |texture, load_action, store_action| AttachmentRequest {
        texture,
        level: 0,
        slice: 0,
        depth_plane: 0,
        resolve: None,
        load_action,
        store_action,
        store_action_options: StoreActionOptions::EMPTY,
        clear: [0.0, 0.0, 0.0, 1.0],
        surface: None,
    };
    let request = |colors| RenderPassRequest {
        color_attachments: colors,
        depth_attachment: None,
        depth_resolve_filter: MultisampleDepthResolveFilter::Sample0,
        stencil_attachment: None,
        stencil_resolve_filter: MultisampleStencilResolveFilter::Sample0,
        visibility_result_buffer: None,
        visibility_result_type: VisibilityResultType::Reset,
        render_target_array_length: 0,
        render_target_width: 0,
        render_target_height: 0,
        default_raster_sample_count: 0,
        tile_memory: Default::default(),
        sample_positions: Vec::new(),
        rasterization_rate_map: false,
        counter_samples: Vec::new(),
    };
    let scratch = |texture| {
        Some(attachment(
            texture,
            LoadAction::DontCare,
            StoreAction::DontCare,
        ))
    };
    // The first pass of each IOSurface target makes an image whose memory
    // Linux allocates in a job: the pass defers, which the boot log once
    // noted as a refusal, and begins when made again.
    let lock = crate::metal::deferred::JobLock {
        lock: Arc::new(crate::metal::compiler::CompileLock::new()),
        relax: || {},
    };
    let run = |device: &mut TestDevice, colors| {
        if device.backend().image_gate.is_some() {
            device.backend().defer_pipelines(Some(&lock));
            let mut commands = device.new_command_buffer();
            assert_eq!(
                device.begin_render(&mut commands, &request(colors)).err(),
                Some(crate::metal::compiler::DEFERRED)
            );
            device.discard(commands);
            device.backend().stop_deferring();
            assert!(device.backend().take_job().is_some());
        }
        let mut commands = device.new_command_buffer();
        device
            .begin_render(&mut commands, &request(colors))
            .unwrap();
        commands.end_encoding(&mut device.resolver()).unwrap();
        let serial = device.commit(commands).unwrap();
        complete(device, serial);
    };

    // The targets are textures of their own or an IOSurface's.
    let managed = options(StorageMode::Managed);
    let buffer = device
        .new_buffer(1920 * 1440 * 5 + (1 << 16), managed)
        .unwrap();
    let descriptor = |format, height| TextureDescriptor {
        width: 1920,
        height,
        options: managed,
        ..texture(format, usage)
    };
    let strips = [
        new(
            &mut device,
            PixelFormat::Bgra8Unorm,
            60,
            StorageMode::Managed,
        ),
        device
            .new_texture_from_buffer(buffer, &descriptor(PixelFormat::Bgra8Unorm, 60), 0, 7680)
            .unwrap(),
    ];
    let half = new(
        &mut device,
        PixelFormat::Rgba16Float,
        60,
        StorageMode::Memoryless,
    );
    for (strip, surface) in strips.into_iter().zip([false, true]) {
        if surface {
            device.backend().image_gate = Some(Arc::new(AtomicBool::new(true)));
        }
        let mut colors = [None; MAX_COLOR_ATTACHMENTS];
        colors[0] = Some(AttachmentRequest {
            surface: Some((3, 1)),
            ..attachment(strip, LoadAction::Load, StoreAction::Store)
        });
        colors[1] = scratch(half);
        run(&mut device, colors);
    }

    let plane = |offset, bytes_per_row| crate::metal::resource::PlaneLayout {
        offset,
        bytes_per_row,
    };
    let targets = [
        new(
            &mut device,
            PixelFormat::Rgb10A8_2pXr10,
            1440,
            StorageMode::Managed,
        ),
        device
            .new_extended_plane_texture(
                buffer,
                &descriptor(PixelFormat::Rgb10A8_2pXr10, 1440),
                [plane(0, 7680), plane(1920 * 1440 * 4, 1920)],
            )
            .unwrap(),
    ];
    let half = new(
        &mut device,
        PixelFormat::Rgba16Float,
        1440,
        StorageMode::Memoryless,
    );
    let single = new(
        &mut device,
        PixelFormat::R32Float,
        1440,
        StorageMode::Memoryless,
    );
    for (target, surface) in targets.into_iter().zip([false, true]) {
        if surface {
            device.backend().image_gate = Some(Arc::new(AtomicBool::new(true)));
        }
        let mut colors = [None; MAX_COLOR_ATTACHMENTS];
        colors[0] = Some(AttachmentRequest {
            surface: Some((4, 1)),
            ..attachment(target, LoadAction::Clear, StoreAction::Store)
        });
        colors[1] = scratch(half);
        colors[2] = scratch(single);
        run(&mut device, colors);
        colors[2] = None;
        run(&mut device, colors);
    }
}

#[test]
fn render_passes_resolve_cached_attachment_views() {
    let mut device = device();
    let target = device
        .new_texture(&texture(
            PixelFormat::Rgba8Unorm,
            TextureUsage::RENDER_TARGET,
        ))
        .unwrap();
    let request = RenderPassRequest {
        color_attachments: {
            let mut colors = [None; MAX_COLOR_ATTACHMENTS];
            colors[0] = Some(AttachmentRequest {
                texture: target,
                level: 0,
                slice: 0,
                depth_plane: 0,
                resolve: None,
                load_action: LoadAction::Clear,
                store_action: StoreAction::Store,
                store_action_options: StoreActionOptions::EMPTY,
                clear: [1.0, 0.0, 0.0, 1.0],
                surface: None,
            });
            colors
        },
        depth_attachment: None,
        depth_resolve_filter: MultisampleDepthResolveFilter::Sample0,
        stencil_attachment: None,
        stencil_resolve_filter: MultisampleStencilResolveFilter::Sample0,
        visibility_result_buffer: None,
        visibility_result_type: VisibilityResultType::Reset,
        render_target_array_length: 0,
        render_target_width: 0,
        render_target_height: 0,
        default_raster_sample_count: 0,
        tile_memory: Default::default(),
        sample_positions: Vec::new(),
        rasterization_rate_map: false,
        counter_samples: Vec::new(),
    };
    let mut views = Vec::new();
    for _ in 0..2 {
        let mut commands = device.new_command_buffer();
        device.begin_render(&mut commands, &request).unwrap();
        commands.end_encoding(&mut device.resolver()).unwrap();
        device.commit(commands).unwrap();
        let segment = &device.backend().submissions.last().unwrap()[0];
        let view = segment.commands.iter().find_map(|command| match command {
            Command::BeginRendering(rendering) => rendering.color[0].map(|color| color.view),
            _ => None,
        });
        views.push(view.unwrap());
    }
    assert_eq!(views[0], views[1], "the attachment view is created once");
    let _ = PrimitiveType::Triangle;

    // The profile marks where the work changes kind and names each span.
    device.set_pass_profile(true);
    let buffer = device
        .new_buffer(4096, options(StorageMode::Private))
        .unwrap();
    let buffer = device.buffer(buffer).unwrap();
    let mut commands = device.new_command_buffer();
    commands.begin_blit().unwrap();
    commands
        .fill_buffer(&mut device.resolver(), buffer, 0, 4096, 7)
        .unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    device.begin_render(&mut commands, &request).unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    let serial = device.commit(commands).unwrap();
    let segment = &device.backend().submissions.last().unwrap()[0];
    let order: Vec<_> = segment
        .commands
        .iter()
        .filter_map(|command| match command {
            Command::FillBuffer { .. } => Some("fill"),
            Command::BeginRendering(_) => Some("begin"),
            Command::EndRendering => Some("end"),
            Command::WriteTimestamp { .. } => Some("mark"),
            _ => None,
        })
        .collect();
    assert_eq!(
        order,
        ["mark", "fill", "mark", "begin", "end", "mark", "mark"]
    );
    let pool = device.backend().query_pools.last().unwrap().1;
    assert_eq!(pool.count, pass_times::PROFILE_MARKS + 2);
    device.backend().queries_ready = true;
    complete(&mut device, serial);
    let passes = device.take_pass_times().unwrap();
    assert_eq!(passes.command_buffers, 1);
    assert_eq!(passes.spans, 3);
    assert_eq!(passes.submits.total, 1);
    assert_eq!(passes.passes.total, 1);
    let keys: Vec<_> = passes.entries.keys().map(String::as_str).collect();
    assert_eq!(
        keys,
        [
            "idle:-",
            "render:-@128x128/Rgba8Unorm/optimal",
            "transfer:-"
        ]
    );
    device.set_pass_profile(false);
    assert_eq!(device.take_pass_times(), None);
}

#[test]
fn completion_recycles_upload_chunks() {
    let mut device = device();
    let record = |device: &mut TestDevice| {
        let mut commands = device.new_command_buffer();
        commands.begin_compute(DispatchType::Serial).unwrap();
        commands
            .set_compute_bytes(&mut device.resolver(), &[7; 64], 0)
            .unwrap();
        commands.end_encoding(&mut device.resolver()).unwrap();
        device.commit(commands).unwrap()
    };
    let first = record(&mut device);
    complete(&mut device, first);
    let created = device.backend().next;
    record(&mut device);
    assert_eq!(
        device.backend().next,
        created,
        "the completed chunk is reused"
    );
}

#[test]
fn shared_events_and_counters() {
    let mut device = device();
    let event = device.new_event().unwrap();
    device.set_signaled_value(event, 5).unwrap();
    device.set_signaled_value(event, 3).unwrap();
    assert_eq!(
        device.signaled_value(event),
        Ok(5),
        "timeline values never decrease"
    );
    assert_eq!(device.wait_event(event, 5, 0), Ok(true));
    assert_eq!(device.wait_event(event, 6, 0), Ok(false));

    let counters = device
        .new_counter_sample_buffer(CounterSet::Timestamp, 4)
        .unwrap();
    assert_eq!(device.resolve_counters(counters, 0, 4), Ok(None));
    device.backend().queries_ready = true;
    assert_eq!(
        device.resolve_counters(counters, 1, 2),
        Ok(Some(alloc::vec![0, 10]))
    );
    assert!(device.resolve_counters(counters, 3, 2).is_err());
    let sample = device.counter_sample(counters, 0).unwrap();
    assert!(matches!(sample, CounterSample::Timestamp { query: 0, .. }));
    assert!(device.counter_sample(counters, 4).is_err());
    let mut commands = device.new_command_buffer();
    commands
        .sample_counters(&mut device.resolver(), sample, true)
        .unwrap();
    device.commit(commands).unwrap();
    device.destroy_counter_sample_buffer(counters).unwrap();
    device.destroy_event(event).unwrap();
}

#[test]
fn statistic_samples_start_as_error_values() {
    let mut device = device();
    let counters = device
        .new_counter_sample_buffer(CounterSet::Statistic, 2)
        .unwrap();
    let values = device.resolve_counters(counters, 1, 1).unwrap().unwrap();
    assert_eq!(
        values, [COUNTER_ERROR_VALUE; 8],
        "one MTLCounterResultStatistic"
    );
    let CounterSample::Statistic(first) = device.counter_sample(counters, 0).unwrap() else {
        panic!("statistic buffers hold statistic samples")
    };
    assert_eq!(
        device.counter_sample(counters, 1),
        Ok(CounterSample::Statistic(first + 64))
    );
    let mut commands = device.new_command_buffer();
    commands
        .sample_counters(
            &mut device.resolver(),
            CounterSample::Statistic(first),
            false,
        )
        .unwrap();
    device.commit(commands).unwrap();
    let segment = &device.backend().submissions.last().unwrap()[0];
    assert!(
        segment
            .commands
            .iter()
            .any(|command| matches!(command, Command::AccumulateResults { runs: 7, .. })),
        "commit resolves the sample's seven counted members"
    );
    device.destroy_counter_sample_buffer(counters).unwrap();
}

#[test]
fn sparse_textures_map_heap_pages_at_commit() {
    let mut device = device();
    let heap = device
        .new_heap(&HeapDescriptor {
            size: 4 << 16,
            options: options(StorageMode::Private),
            heap_type: HeapType::Sparse,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    let descriptor = TextureDescriptor {
        width: 256,
        height: 256,
        mipmap_level_count: 4,
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    assert!(
        device.new_heap_texture(heap, &descriptor, Some(0)).is_err(),
        "sparse heaps place textures themselves"
    );
    let sparse = device.new_heap_texture(heap, &descriptor, None).unwrap();
    assert_eq!(device.sparse_tail(sparse), Ok(Some((2, 64 << 10))));
    let image = device.sparse_image(sparse).unwrap();
    let plain = device
        .new_texture(&texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ))
        .unwrap();
    assert_eq!(device.sparse_tail(plain), Ok(None));
    assert!(device.sparse_image(plain).is_err());

    let mut commands = device.new_command_buffer();
    commands.begin_blit().unwrap();
    let scratch = BufferRef {
        buffer: Buffer(99),
        offset: 0,
        address: 99 << 32,
        length: 64,
    };
    commands.copy_buffer(scratch, 0, scratch, 32, 32).unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    commands
        .begin_resource_state(&mut device.resolver())
        .unwrap();
    let tiles = |level, origin, size| MappingUpdate {
        level,
        slice: 0,
        region: SparseRegion { origin, size },
    };
    commands
        .update_texture_mappings(
            image,
            SparseTextureMappingMode::Map,
            alloc::vec![tiles(0, [0; 3], [2, 1, 1]), tiles(3, [0; 3], [1; 3])],
        )
        .unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    device.commit(commands).unwrap();
    let segments = device.backend().submissions.last().unwrap().clone();
    assert_eq!(
        segments.len(),
        2,
        "the encoder's updates bind between segments"
    );
    assert!(segments[0].binds.is_empty());
    assert_eq!(segments[1].binds.len(), 3, "two blocks and the tail");
    assert_eq!(device.heap_usage(heap, 1), Ok((3 << 16, 1 << 16)));

    // A bad region fails the commit before any mapping changes.
    let mut commands = device.new_command_buffer();
    commands
        .begin_resource_state(&mut device.resolver())
        .unwrap();
    commands
        .update_texture_mappings(
            image,
            SparseTextureMappingMode::Unmap,
            alloc::vec![tiles(0, [0; 3], [1; 3]), tiles(0, [2, 0, 0], [1; 3])],
        )
        .unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    assert!(device.commit(commands).is_err());
    assert_eq!(device.heap_usage(heap, 1), Ok((3 << 16, 1 << 16)));

    assert!(
        device.destroy_heap(heap).is_err(),
        "the texture lives in it"
    );
    device.destroy_texture(sparse).unwrap();
    assert_eq!(device.heap_usage(heap, 1), Ok((0, 4 << 16)));
    device.destroy_heap(heap).unwrap();
}

#[test]
fn multisample_sparse_textures_map_msaa_blocks() {
    let mut device = device();
    let heap = device
        .new_heap(&HeapDescriptor {
            size: 4 << 16,
            options: options(StorageMode::Private),
            heap_type: HeapType::Sparse,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    // Four-sample RGBA8 blocks are 64 by 64 texels.
    let descriptor = TextureDescriptor {
        texture_type: TextureType::D2Multisample,
        sample_count: 4,
        width: 128,
        height: 128,
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::RENDER_TARGET)
    };
    let sparse = device.new_heap_texture(heap, &descriptor, None).unwrap();
    assert_eq!(device.sparse_tail(sparse), Ok(Some((1, 0))), "no mip tail");
    let image = device.sparse_image(sparse).unwrap();

    let mut commands = device.new_command_buffer();
    commands
        .begin_resource_state(&mut device.resolver())
        .unwrap();
    commands
        .update_texture_mappings(
            image,
            SparseTextureMappingMode::Map,
            alloc::vec![MappingUpdate {
                level: 0,
                slice: 0,
                region: SparseRegion {
                    origin: [0; 3],
                    size: [2, 2, 1],
                },
            }],
        )
        .unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    device.commit(commands).unwrap();
    let segments = device.backend().submissions.last().unwrap().clone();
    assert_eq!(segments.last().unwrap().binds.len(), 4);
    assert_eq!(device.heap_usage(heap, 1), Ok((4 << 16, 0)));
    device.destroy_texture(sparse).unwrap();
    device.destroy_heap(heap).unwrap();
}

#[test]
fn indirect_mapping_updates_bind_once_their_arguments_are_read() {
    let mut device = device();
    let heap = device
        .new_heap(&HeapDescriptor {
            size: 4 << 16,
            options: options(StorageMode::Private),
            heap_type: HeapType::Sparse,
            sparse_page_size: SparsePageSize::Size64,
        })
        .unwrap();
    // Two by two tiles and no mip tail.
    let descriptor = TextureDescriptor {
        width: 256,
        height: 256,
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let sparse = device.new_heap_texture(heap, &descriptor, None).unwrap();
    let image = device.sparse_image(sparse).unwrap();
    let indirect = BufferRef {
        buffer: Buffer(99),
        offset: 0,
        address: 99 << 32,
        length: 96,
    };
    let encode = |device: &mut TestDevice, offset| {
        let mut commands = device.new_command_buffer();
        commands
            .begin_resource_state(&mut device.resolver())
            .unwrap();
        let encoded = commands.update_texture_mapping_indirect(
            &mut device.resolver(),
            image,
            SparseTextureMappingMode::Map,
            indirect,
            offset,
        );
        commands.end_encoding(&mut device.resolver()).unwrap();
        encoded.map(|()| commands)
    };
    assert!(
        encode(&mut device, 2).is_err(),
        "offsets are 4-byte aligned"
    );
    assert!(
        encode(&mut device, 94).is_err(),
        "the count is in the buffer"
    );

    // Commits an update and plays the GPU: its copy of the arguments lands
    // and signals its step.
    // `MTLMapIndirectBufferFormat`: a count, then its mappings.
    let copied = |device: &mut TestDevice, words: &[u32]| {
        let submitted = device.backend().submissions.len();
        let copy = device.backend().submissions.last().unwrap().clone();
        assert_eq!(copy.len(), 1, "the segments from the update's on are held");
        let at = copy[0]
            .commands
            .iter()
            .find_map(|command| match command {
                Command::CopyBuffer { src, region, .. } if *src == Buffer(99) => {
                    Some(region.dst_offset)
                }
                _ => None,
            })
            .unwrap();
        // The step, before the late timeline value that ends the part.
        let (steps, step) = *copy[0].signals.iter().rev().nth(1).unwrap();
        let backend = device.backend();
        let memory = backend.writes.last().unwrap().0;
        let bytes: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        backend.write_memory(memory, at, &bytes).unwrap();
        (submitted, steps, step)
    };

    let commands = encode(&mut device, 32).unwrap();
    device.commit(commands).unwrap();
    submit(&mut device);
    // Origin (1, 0, 0), size (1, 2, 1), level 0, slice 0: the right column.
    let (submitted, steps, step) = copied(&mut device, &[1, 1, 0, 0, 1, 2, 1, 0, 0]);
    assert_eq!(device.poll(), Ok(0));
    assert_eq!(
        device.backend().submissions.len(),
        submitted,
        "later command buffers wait"
    );
    device.backend().signal_semaphore(steps, step).unwrap();
    assert_eq!(device.poll(), Ok(0));
    let submissions = &device.backend().submissions;
    assert_eq!(submissions.len(), submitted + 2);
    assert_eq!(submissions[submitted][0].binds.len(), 2);
    assert!(submissions[submitted + 1][0].binds.is_empty());
    assert_eq!(device.heap_usage(heap, 1), Ok((2 << 16, 2 << 16)));

    // Of two mappings, one naming no region of the texture maps nothing
    // and the other its tile.
    let commands = encode(&mut device, 0).unwrap();
    device.commit(commands).unwrap();
    let (_, steps, step) = copied(
        &mut device,
        &[2, 0, 0, 0, 1, 1, 1, 5, 0, 0, 0, 0, 1, 1, 1, 0, 0],
    );
    device.backend().signal_semaphore(steps, step).unwrap();
    assert_eq!(device.poll(), Ok(0));
    assert_eq!(
        device.backend().submissions.last().unwrap()[0].binds.len(),
        1
    );
    assert_eq!(device.heap_usage(heap, 1), Ok((3 << 16, 1 << 16)));

    // A count of none maps nothing.
    let commands = encode(&mut device, 0).unwrap();
    let serial = device.commit(commands).unwrap();
    let (_, steps, step) = copied(&mut device, &[0]);
    device.backend().signal_semaphore(steps, step).unwrap();
    assert_eq!(device.poll(), Ok(0));
    assert!(
        device.backend().submissions.last().unwrap()[0]
            .binds
            .is_empty()
    );
    assert_eq!(device.heap_usage(heap, 1), Ok((3 << 16, 1 << 16)));
    complete(&mut device, serial);
    device.destroy_texture(sparse).unwrap();
    device.destroy_heap(heap).unwrap();
}

#[test]
fn indirect_commands_execute_snapshots_of_their_buffer() {
    let mut device = device();
    let kernel = device.new_function(LIBRARY, "kernel", Some(&[])).unwrap();
    let pipeline = device
        .new_compute_pipeline(&ComputePipelineRequest {
            compute_function: kernel,
            threadgroup_size_is_multiple_of_thread_execution_width: false,
            max_total_threads_per_threadgroup: 0,
            stage_input: None,
            linked_functions: LinkedFunctions::default(),
            support_indirect_command_buffers: false,
            archives: Default::default(),
        })
        .unwrap();
    let state = device.compute_pipeline_state(pipeline).unwrap().clone();
    let buffer = device
        .new_buffer(4096, options(StorageMode::Private))
        .unwrap();
    let buffer = device.buffer(buffer).unwrap();
    let descriptor = IndirectCommandBufferDescriptor {
        command_types: IndirectCommandType::CONCURRENT_DISPATCH,
        inherit_pipeline_state: false,
        inherit_buffers: false,
        inherit_cull_mode: true,
        inherit_depth_stencil_state: true,
        inherit_depth_bias: true,
        inherit_depth_clip_mode: true,
        inherit_front_facing_winding: true,
        inherit_triangle_fill_mode: true,
        max_vertex_buffer_bind_count: 0,
        max_fragment_buffer_bind_count: 0,
        max_kernel_buffer_bind_count: 1,
        max_kernel_threadgroup_memory_bind_count: 0,
        max_object_buffer_bind_count: 0,
        max_mesh_buffer_bind_count: 0,
        support_ray_tracing: false,
        support_dynamic_attribute_stride: false,
    };
    let icb = device.new_indirect_command_buffer(&descriptor, 2).unwrap();
    let threads = Size {
        width: 64,
        height: 1,
        depth: 1,
    };
    let contents = device.indirect_command_buffer_mut(icb).unwrap();
    let mut command = contents.compute_command(0).unwrap();
    command.set_compute_pipeline_state(&state).unwrap();
    command.set_kernel_buffer(buffer, 0, 0).unwrap();
    command
        .concurrent_dispatch_threadgroups([3, 1, 1], threads)
        .unwrap();

    let mut commands = device.new_command_buffer();
    commands.begin_compute(DispatchType::Serial).unwrap();
    commands
        .execute_commands(&mut device.resolver(), icb, ExecutionRange::Direct(0..2))
        .unwrap();
    // Encoding after the execution changes later executions only.
    device
        .indirect_command_buffer_mut(icb)
        .unwrap()
        .reset(0..1)
        .unwrap();
    commands
        .execute_commands(&mut device.resolver(), icb, ExecutionRange::Direct(0..2))
        .unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    device.commit(commands).unwrap();
    let segments = device.backend().submissions.last().unwrap().clone();
    let dispatches = segments[0]
        .commands
        .iter()
        .filter(|command| **command == Command::Dispatch([3, 1, 1]))
        .count();
    assert_eq!(dispatches, 1);
    assert_eq!(device.indirect_command_buffer(icb).unwrap().size(), 2);
    device.destroy_indirect_command_buffer(icb).unwrap();
    assert!(device.indirect_command_buffer(icb).is_err());
}

/// Draws under inherited state with one vertex buffer.
fn draws() -> IndirectCommandBufferDescriptor {
    IndirectCommandBufferDescriptor {
        command_types: IndirectCommandType::DRAW,
        inherit_pipeline_state: true,
        inherit_buffers: false,
        inherit_cull_mode: true,
        inherit_depth_stencil_state: true,
        inherit_depth_bias: true,
        inherit_depth_clip_mode: true,
        inherit_front_facing_winding: true,
        inherit_triangle_fill_mode: true,
        max_vertex_buffer_bind_count: 1,
        max_fragment_buffer_bind_count: 0,
        max_kernel_buffer_bind_count: 0,
        max_kernel_threadgroup_memory_bind_count: 0,
        max_object_buffer_bind_count: 0,
        max_mesh_buffer_bind_count: 0,
        support_ray_tracing: false,
        support_dynamic_attribute_stride: false,
    }
}

#[test]
fn functions_that_set_command_barriers_order_resident_buffers() {
    let mut device = device();
    let resident = device.new_indirect_command_buffer(&draws(), 1).unwrap();
    device
        .indirect_command_buffer_resource_id(resident)
        .unwrap();
    let later = device.new_indirect_command_buffer(&draws(), 1).unwrap();
    let barriers =
        |device: &Device<_, _>, id| device.indirect_command_buffer(id).unwrap().has_barriers();
    device.new_function(LIBRARY, "visible", Some(&[])).unwrap();
    assert!(!barriers(&device, resident));
    device.new_function(LIBRARY, "barriers", Some(&[])).unwrap();
    assert!(barriers(&device, resident));
    device.indirect_command_buffer_resource_id(later).unwrap();
    assert!(barriers(&device, later));
}

#[test]
fn handing_out_an_indirect_address_keeps_commands_as_records() {
    let mut device = device();
    let icb = device.new_indirect_command_buffer(&draws(), 2).unwrap();
    device
        .indirect_command_buffer_mut(icb)
        .unwrap()
        .render_command(1)
        .unwrap()
        .draw_primitives(PrimitiveType::Point, 0, 5, 1, 0)
        .unwrap();
    let address = device.indirect_command_buffer_resource_id(icb).unwrap();
    assert_eq!(device.indirect_command_buffer_resource_id(icb), Ok(address));
    let layout = *device
        .indirect_command_buffer(icb)
        .unwrap()
        .records()
        .unwrap();
    let (memory, base, image) = device.backend().writes.last().unwrap().clone();
    assert_eq!(image.len() as u64, layout.bytes());
    let record = layout.record(1) as usize;
    assert_eq!(image[record], RecordLayout::DRAW as u8);

    // CPU encoding reaches the records when the next command buffer commits.
    device
        .indirect_command_buffer_mut(icb)
        .unwrap()
        .render_command(1)
        .unwrap()
        .reset()
        .unwrap();
    let written = device.backend().writes.len();
    let commands = device.new_command_buffer();
    device.commit(commands).unwrap();
    let mut action = [0xff; 4];
    device
        .backend()
        .read_memory(memory, base + layout.record(1), &mut action)
        .unwrap();
    assert_eq!(action, [0; 4]);
    assert_eq!(device.backend().writes.len(), written + 1);

    let mut commands = device.new_command_buffer();
    commands.begin_blit().unwrap();
    commands
        .reset_commands(&mut device.resolver(), icb, 0..2)
        .unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    device.commit(commands).unwrap();
    let segments = device.backend().submissions.last().unwrap().clone();
    let fill = segments[0]
        .commands
        .iter()
        .find_map(|command| match command {
            Command::FillBuffer { offset, size, .. } => Some((*offset, *size)),
            _ => None,
        });
    assert_eq!(fill, Some((layout.record(0), 2 * u64::from(layout.stride))));
    device.destroy_indirect_command_buffer(icb).unwrap();
}

#[test]
fn synchronized_managed_buffers_are_invalidated_on_completion() {
    let mut device = device();
    let managed = device
        .new_buffer(256, options(StorageMode::Managed))
        .unwrap();
    let shared = device
        .new_buffer(256, options(StorageMode::Shared))
        .unwrap();
    assert_eq!(
        device.managed_range(shared),
        Err(Error::Invalid(
            "synchronizeResource: is for managed resources"
        ))
    );
    let range = device.managed_range(managed).unwrap();
    let mut commands = device.new_command_buffer();
    commands.begin_blit().unwrap();
    commands.synchronize_buffer(range).unwrap();
    commands.end_encoding(&mut device.resolver()).unwrap();
    let serial = device.commit(commands).unwrap();
    // The managed buffer outlives its handle until the work completes.
    device.destroy_buffer(managed).unwrap();
    assert!(device.backend().invalidated.is_empty());
    complete(&mut device, serial);
    let invalidated = (range.memory, range.offset, range.length);
    assert_eq!(device.backend().invalidated, [invalidated]);
}

#[test]
fn command_buffers_are_timed_from_their_first_command_to_their_last() {
    let mut device = device();
    let serial = submit(&mut device);
    let segments = device.backend().submissions.last().unwrap().clone();
    let stamp = |command: &Command| match command {
        Command::WriteTimestamp { query, stage, .. } => Some((*query, *stage)),
        _ => None,
    };
    let first = &segments[0].commands;
    assert_eq!(stamp(&first[1]), Some((0, PipelineStages::TOP_OF_PIPE)));
    let last = segments.last().unwrap().commands.last().unwrap();
    assert_eq!(stamp(last), Some((1, PipelineStages::ALL_COMMANDS)));
    assert_eq!(
        device.command_buffer_times(serial),
        Err(Error::Invalid("the command buffer has not completed"))
    );
    device.backend().queries_ready = true;
    complete(&mut device, serial);
    assert_eq!(device.command_buffer_times(serial), Ok([0, 10]));
}

#[test]
fn a_command_buffer_without_written_timestamps_is_told_apart_from_an_evicted_one() {
    let mut device = device();
    // Its timestamps were never written, as when it ends in an error: zeros,
    // reported once.
    let errored = submit(&mut device);
    complete(&mut device, errored);
    assert_eq!(device.command_buffer_times(errored), Ok([0, 0]));
    assert!(
        device
            .take_unsupported()
            .is_some_and(|what| what.contains("ended without them"))
    );
    assert_eq!(device.command_buffer_times(errored), Ok([0, 0]));
    assert_eq!(device.take_unsupported(), None);
    // Past the kept window, a timed one is no longer kept: still refused.
    device.backend().queries_ready = true;
    for _ in 0..super::queue::TIMES_KEPT {
        let serial = submit(&mut device);
        complete(&mut device, serial);
    }
    assert_eq!(
        device.command_buffer_times(errored),
        Err(Error::Invalid(
            "the command buffer's GPU times are no longer kept"
        ))
    );
}

#[test]
fn completion_query_pools_reuse_two_slots_after_retirement() {
    let mut device = device();
    device
        .new_counter_sample_buffer(CounterSet::Timestamp, 17)
        .unwrap();
    let first = submit(&mut device);
    let second = submit(&mut device);
    let backend = device.backend();
    let [
        (public_pool, public_info),
        (first_pool, first_info),
        (second_pool, second_info),
    ] = backend.query_pools.as_slice()
    else {
        panic!("expected one public pool and two in-flight completion pools");
    };
    assert_eq!(public_info.count, 17);
    assert_eq!(first_info.query_type, QueryType::Timestamp);
    assert_eq!(first_info.count, 32);
    assert_eq!(second_info, first_info);
    let (public_pool, first_pool, second_pool) = (*public_pool, *first_pool, *second_pool);
    assert_ne!(first_pool, second_pool);
    assert_eq!(
        backend.query_resets,
        [
            (public_pool, 0, 17),
            (first_pool, 0, 32),
            (second_pool, 0, 32)
        ]
    );
    assert!(backend.query_reads.is_empty());

    backend.queries_ready = true;
    complete(&mut device, first);
    assert_eq!(device.command_buffer_times(first), Ok([0, 10]));
    assert_eq!(device.backend().query_reads, [(first_pool, 0, 2)]);
    assert_eq!(device.backend().query_resets[3..], [(first_pool, 0, 32)]);

    let third = submit(&mut device);
    let backend = device.backend();
    assert_eq!(backend.query_pools.len(), 3);
    assert_eq!(backend.query_resets.len(), 4);
    let submission = backend.submissions.last().unwrap();
    assert!(matches!(
        submission[0].commands[1],
        Command::WriteTimestamp { pool, query: 0, .. } if pool == first_pool
    ));

    complete(&mut device, third);
    assert_eq!(device.command_buffer_times(second), Ok([0, 10]));
    assert_eq!(device.command_buffer_times(third), Ok([0, 10]));
    assert_eq!(
        device.backend().query_reads,
        [(first_pool, 0, 2), (second_pool, 0, 2), (first_pool, 0, 2)]
    );
    assert_eq!(
        device.backend().query_resets[3..],
        [
            (first_pool, 0, 32),
            (second_pool, 0, 32),
            (first_pool, 0, 32)
        ]
    );
}

#[test]
fn failed_completion_preparation_preserves_pending_texture_uploads() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let texture = device
        .new_texture(&TextureDescriptor {
            options: options(StorageMode::Managed),
            ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
        })
        .unwrap();
    device
        .replace_region(
            texture,
            &Region {
                slice: 0,
                level: 0,
                origin: [0; 3],
                size: [1; 3],
            },
            &[1, 2, 3, 4],
            Strides {
                bytes_per_row: 4,
                bytes_per_image: 0,
            },
        )
        .unwrap();
    let encode = |device: &mut TestDevice| {
        let mut commands = device.new_command_buffer();
        commands.begin_compute(DispatchType::Serial).unwrap();
        commands
            .set_compute_bytes(&mut device.resolver(), &[7; 64], 0)
            .unwrap();
        commands.end_encoding(&mut device.resolver()).unwrap();
        commands
    };

    let commands = encode(&mut device);
    let writes = device.backend().writes.len();
    let created = device.backend().next;
    device.backend().query_pool_error = Some(Error::OutOfMemory);
    assert_eq!(device.commit(commands), Err(Error::OutOfMemory));
    assert_eq!(device.backend().writes.len(), writes);
    assert!(device.backend().submissions.is_empty());

    let commands = encode(&mut device);
    device.backend().write_error = Some(Error::OutOfMemory);
    assert_eq!(device.commit(commands), Err(Error::OutOfMemory));
    assert_eq!(device.backend().next, created + 1);
    let pool = device.backend().query_pools[0].0;
    assert_eq!(device.backend().query_resets, [(pool, 0, 32)]);
    assert!(device.backend().submissions.is_empty());

    let commands = encode(&mut device);
    let serial = device.commit(commands).unwrap();
    assert_eq!(serial, 1);
    assert_eq!(device.backend().next, created + 1);
    assert_eq!(device.backend().query_resets, [(pool, 0, 32)]);
    let submitted = &device.backend().submissions[0][0].commands;
    assert!(matches!(submitted[1], Command::CopyBufferToImage { .. }));
    assert!(matches!(
        submitted[3],
        Command::WriteTimestamp { query: 0, .. }
    ));
    complete(&mut device, serial);
    assert_eq!(
        device.backend().query_resets,
        [(pool, 0, 32), (pool, 0, 32)]
    );
}

#[test]
fn uncommitted_command_buffers_return_their_upload_memory() {
    let mut device = device();
    let encode = |device: &mut TestDevice| {
        let mut commands = device.new_command_buffer();
        commands.begin_compute(DispatchType::Serial).unwrap();
        commands
            .set_compute_bytes(&mut device.resolver(), &[7; 64], 0)
            .unwrap();
        commands
    };
    let commands = encode(&mut device);
    let created = device.backend().next;
    assert_eq!(
        device.commit(commands),
        Err(Error::Invalid("another encoder is still encoding"))
    );
    let commands = encode(&mut device);
    device.discard(commands);
    let mut commands = encode(&mut device);
    commands.end_encoding(&mut device.resolver()).unwrap();
    device.commit(commands).unwrap();
    assert_eq!(
        device.backend().next,
        created + 1,
        "each command buffer reused the first one's chunk; only the commit's timestamp pool is new"
    );
}

#[test]
fn imported_memory_stays_until_the_gpu_is_done_with_it() {
    let mut device = device();
    let token = core::num::NonZeroU64::new(9).unwrap();
    let ranges = [stela_contracts::gate::MemoryRange {
        address: 0x8_0000_0000,
        bytes: 8192,
    }];
    let memory = device.import_memory(&ranges, token).unwrap();
    assert_eq!(device.current_allocated_size(), 8192);
    assert_eq!(device.backend().imports.len(), 1);
    assert_eq!(
        device.new_imported_buffer(memory, 8192, options(StorageMode::Private)),
        Err(Error::Invalid("imported buffers are shared or managed"))
    );
    assert_eq!(
        device.new_imported_buffer(memory, 8193, options(StorageMode::Shared)),
        Err(Error::Invalid("the buffer is longer than its memory"))
    );
    let buffer = device
        .new_imported_buffer(memory, 8192, options(StorageMode::Shared))
        .unwrap();
    assert_eq!(device.current_allocated_size(), 8192);
    let serial = submit(&mut device);
    device.destroy_buffer(buffer).unwrap();
    assert_eq!(device.current_allocated_size(), 8192);
    assert_eq!(device.retired(), None);
    assert_eq!(device.backend().imports.len(), 1);
    complete(&mut device, serial);
    assert_eq!(device.current_allocated_size(), 0);
    assert!(device.backend().imports.is_empty());
    assert_eq!(device.retired(), Some(token));
    assert_eq!(device.retired(), None);
}

#[test]
fn imported_heap_allocations_are_counted_once_through_retirement() {
    let mut device = device();
    let token = core::num::NonZeroU64::new(10).unwrap();
    let ranges = [stela_contracts::gate::MemoryRange {
        address: 0x8_0000_0000,
        bytes: 1 << 20,
    }];
    let memory = device.import_memory(&ranges, token).unwrap();
    let heap = device
        .new_imported_heap(
            memory,
            &HeapDescriptor {
                size: 1 << 20,
                options: options(StorageMode::Shared),
                heap_type: HeapType::Automatic,
                sparse_page_size: SparsePageSize::Size64,
            },
        )
        .unwrap();
    let buffer = device
        .new_heap_buffer(heap, 4096, options(StorageMode::Shared), None)
        .unwrap();
    assert_eq!(device.current_allocated_size(), 1 << 20);
    let serial = submit(&mut device);
    device.destroy_buffer(buffer).unwrap();
    device.destroy_heap(heap).unwrap();
    assert_eq!(device.current_allocated_size(), 1 << 20);
    assert_eq!(device.retired(), None);
    complete(&mut device, serial);
    assert_eq!(device.current_allocated_size(), 0);
    assert_eq!(device.retired(), Some(token));
    assert_eq!(device.retired(), None);
}

/// A shared heap of 100 bytes lives in the page the guest maps for it and
/// holds that page, as on a Mac.
#[test]
fn imported_heaps_hold_whole_pages() {
    let mut device = device();
    let token = core::num::NonZeroU64::new(11).unwrap();
    let ranges = [stela_contracts::gate::MemoryRange {
        address: 0x8_0000_0000,
        bytes: PAGE,
    }];
    let memory = device.import_memory(&ranges, token).unwrap();
    let heap = device
        .new_imported_heap(
            memory,
            &HeapDescriptor {
                size: 100,
                options: options(StorageMode::Shared),
                heap_type: HeapType::Automatic,
                sparse_page_size: SparsePageSize::Size64,
            },
        )
        .unwrap();
    assert_eq!(device.heap_occupancy(heap).unwrap(), (PAGE, 0));
    assert_eq!(device.heap_usage(heap, 1).unwrap(), (0, PAGE));
    device.destroy_heap(heap).unwrap();
}

#[test]
fn device_loss_keeps_in_flight_imports_pinned_and_charged() {
    let mut device = device();
    let ranges = [stela_contracts::gate::MemoryRange {
        address: 0x8_0000_0000,
        bytes: 8192,
    }];
    let memory = device
        .import_memory(&ranges, core::num::NonZeroU64::new(11).unwrap())
        .unwrap();
    let buffer = device
        .new_imported_buffer(memory, 8192, options(StorageMode::Shared))
        .unwrap();
    let unclaimed = device
        .import_memory(&ranges, core::num::NonZeroU64::new(12).unwrap())
        .unwrap();
    let serial = submit(&mut device);
    let query_resets = device.backend().query_resets.len();
    device.destroy_buffer(buffer).unwrap();
    device.backend().semaphore_error = Some(Error::DeviceLost);
    assert_eq!(device.poll(), Err(Error::DeviceLost));
    assert!(device.is_lost());
    device.release_import(unclaimed).unwrap();
    // A later backend value cannot establish completion after device loss.
    device.backend().semaphore_error = None;
    let timeline = device.timeline;
    device.backend().complete(timeline, serial);
    assert_eq!(device.poll(), Err(Error::DeviceLost));
    assert_eq!(device.current_allocated_size(), 16384);
    assert_eq!(device.backend().imports.len(), 2);
    assert!(device.backend().destroyed.is_empty());
    assert_eq!(device.backend().query_resets.len(), query_resets);
    assert!(device.backend().query_reads.is_empty());
    assert_eq!(device.retired(), None);
}

/// WindowServer's event patterns never hold its work: a wait on a value an
/// earlier commit of its own signals, on one the CPU signaled before or
/// signals after, and a signal then a wait in one commit.
#[test]
fn window_server_event_patterns_submit_at_once() {
    let mut device = device_on(MockBackend::new());
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let commit = |device: &mut TestDevice, key, signal: Option<u64>, wait: Option<u64>| {
        let mut commands = device.new_command_buffer();
        if let Some(value) = signal {
            commands
                .encode_signal_event(&mut device.resolver(), semaphore, value)
                .unwrap();
        }
        if let Some(value) = wait {
            commands
                .encode_wait_for_event(&mut device.resolver(), semaphore, value)
                .unwrap();
        }
        device.commit_for(key, commands).unwrap()
    };
    // Signal in one commit, wait in the next, both from one connection.
    commit(&mut device, 1, Some(1), None);
    commit(&mut device, 1, None, Some(1));
    // Signal then wait in one commit.
    commit(&mut device, 1, Some(2), Some(2));
    // The CPU signals before the commit that waits.
    device.set_signaled_value(event, 5).unwrap();
    commit(&mut device, 1, None, Some(5));
    assert!(!device.holds());
    // The CPU signals after: held until then, then submitted at once.
    let waiting = commit(&mut device, 2, None, Some(6));
    assert!(device.holds());
    device.set_signaled_value(event, 6).unwrap();
    assert!(!device.holds());
    assert_eq!(device.backend().submissions.len() as u64, waiting);
    assert!(device.backend().unpromised.is_empty());
    device.backend().finish();
    device.poll().unwrap();
    assert_eq!(device.reached_for(1), waiting);
    assert_eq!(device.reached_for(2), waiting);
}

/// One event operation of a command buffer the queue tests commit.
#[derive(Clone, Copy)]
enum EventStep {
    Signal(usize, u64),
    Wait(usize, u64),
}

/// Commits a command buffer of `steps` over `events` for connection `key`
/// on its Metal queue `queue`.
fn commit_events(
    device: &mut TestDevice,
    events: &[Semaphore],
    [key, queue]: [u64; 2],
    steps: &[EventStep],
) -> u64 {
    let mut commands = device.new_command_buffer();
    for &step in steps {
        match step {
            EventStep::Signal(event, value) => commands
                .encode_signal_event(&mut device.resolver(), events[event], value)
                .unwrap(),
            EventStep::Wait(event, value) => commands
                .encode_wait_for_event(&mut device.resolver(), events[event], value)
                .unwrap(),
        }
    }
    device.commit_on(key, queue, commands).unwrap()
}

/// `count` new events and their semaphores.
fn new_events(device: &mut TestDevice, count: usize) -> (Vec<EventId>, Vec<Semaphore>) {
    let ids: Vec<_> = (0..count).map(|_| device.new_event().unwrap()).collect();
    let semaphores = ids
        .iter()
        .map(|&id| device.event_semaphore(id).unwrap())
        .collect();
    (ids, semaphores)
}

/// The index of the first submission that signals `signal`.
fn submitted_at(device: &mut TestDevice, signal: (Semaphore, u64)) -> Option<usize> {
    device.backend().submissions.iter().position(|segments| {
        segments
            .iter()
            .any(|segment| segment.signals.contains(&signal))
    })
}

/// MetalFX's temporal scaler over MPSGraph: the caller's command buffer A
/// signals the graph's B, waits for it, signals D and waits for D, while B,
/// C and D run on the graph's own queue and the CPU signals between them.
/// A's first segment must reach the GPU while its later ones wait, and B
/// must not wait behind A.
#[test]
fn a_command_buffer_runs_its_segments_as_other_queues_promise_its_waits() {
    use EventStep::{Signal, Wait};
    let mut device = device_on(MockBackend::new());
    let (ids, e) = new_events(&mut device, 7);
    let (caller, graph) = ([1, 1], [1, 2]);
    let a = commit_events(
        &mut device,
        &e,
        caller,
        &[Signal(1, 1), Wait(3, 2), Signal(4, 1), Wait(2, 1)],
    );
    // Its first segment is submitted; the rest waits for #3.
    assert!(submitted_at(&mut device, (e[1], 1)).is_some());
    assert!(submitted_at(&mut device, (e[4], 1)).is_none());
    assert!(device.holds());
    let b = commit_events(
        &mut device,
        &e,
        graph,
        &[Wait(1, 1), Signal(5, 2), Signal(6, 2), Signal(3, 2)],
    );
    // B ran, and what it signals released A's middle segment.
    let b_at = submitted_at(&mut device, (e[3], 2)).unwrap();
    let middle = submitted_at(&mut device, (e[4], 1)).unwrap();
    assert!(submitted_at(&mut device, (e[1], 1)).unwrap() < b_at && b_at < middle);
    device.backend().finish();
    device.poll().unwrap();
    // B completed on its own queue while A still waits.
    assert_eq!(device.reached_for(1), a - 1);
    assert_eq!(device.completed_after(1, a - 1), b);
    assert_eq!(device.completed_after(1, b), 0);
    // The CPU raises #6 once it saw #5 and #6 reach 2.
    device.set_signaled_value(ids[6], 3).unwrap();
    let c = commit_events(&mut device, &e, graph, &[Wait(6, 2), Signal(6, 3)]);
    let d = commit_events(
        &mut device,
        &e,
        graph,
        &[Wait(6, 3), Wait(4, 1), Signal(2, 1)],
    );
    let d_at = submitted_at(&mut device, (e[2], 1)).unwrap();
    assert!(middle < d_at);
    // A's last segment followed D.
    let last = device.backend().submissions.len() - 1;
    assert!(d_at < last);
    assert!(
        device.backend().submissions[last][0]
            .waits
            .contains(&(e[2], 1))
    );
    assert!(device.backend().unpromised.is_empty());
    device.backend().finish();
    device.poll().unwrap();
    assert!(!device.holds());
    assert_eq!(device.reached_for(1), d);
    assert!(c < d && d == device.submitted_serial());
}

/// A wait nothing promises holds the rest of its command buffer and its
/// queue's later work, but neither another queue of the same connection nor
/// another connection, and the other queue's completions are answered.
#[test]
fn a_wait_nothing_promises_holds_only_its_queue() {
    use EventStep::{Signal, Wait};
    let mut device = device_on(MockBackend::new());
    let (ids, e) = new_events(&mut device, 2);
    let held = commit_events(&mut device, &e, [1, 1], &[Signal(1, 1), Wait(0, 1)]);
    let behind = commit_events(&mut device, &e, [1, 1], &[]);
    let other_queue = commit_events(&mut device, &e, [1, 2], &[Wait(1, 1)]);
    let other_connection = commit_events(&mut device, &e, [2, 1], &[]);
    device.backend().finish();
    device.poll().unwrap();
    assert_eq!(device.reached_for(1), held - 1);
    assert_eq!(device.completed_after(1, held - 1), other_queue);
    assert_eq!(device.completed_after(1, other_queue), 0);
    assert_eq!(device.reached_for(2), other_connection);
    // Completion waits only for the submitted part of the held command
    // buffer; freeing waits for all of it, which may still read what was
    // released meanwhile.
    assert_eq!(device.completed_serial(), device.submitted_serial());
    assert_eq!(device.freeable(), held - 1);
    assert!(device.backend().unpromised.is_empty());
    device.set_signaled_value(ids[0], 1).unwrap();
    device.backend().finish();
    device.poll().unwrap();
    assert_eq!(device.reached_for(1), device.submitted_serial());
    assert!(behind > held && !device.holds());
}

/// Two queues whose waits each need the other's held segment stay held:
/// nothing promises either value. A third queue and another connection run.
#[test]
fn waits_that_promise_each_other_through_held_segments_stay_held() {
    use EventStep::{Signal, Wait};
    let mut device = device_on(MockBackend::new());
    let (_, e) = new_events(&mut device, 3);
    let first = commit_events(
        &mut device,
        &e,
        [1, 1],
        &[Signal(2, 1), Wait(0, 1), Signal(1, 1)],
    );
    commit_events(&mut device, &e, [1, 2], &[Wait(1, 1), Signal(0, 1)]);
    let third = commit_events(&mut device, &e, [1, 3], &[]);
    let other = commit_events(&mut device, &e, [2, 1], &[]);
    // Only the first command buffer's first segment and the free work ran.
    assert!(submitted_at(&mut device, (e[2], 1)).is_some());
    assert!(submitted_at(&mut device, (e[0], 1)).is_none());
    assert!(submitted_at(&mut device, (e[1], 1)).is_none());
    assert!(device.backend().unpromised.is_empty());
    device.backend().finish();
    device.poll().unwrap();
    assert!(device.holds());
    assert_eq!(device.reached_for(1), first - 1);
    assert_eq!(device.completed_after(1, first - 1), third);
    assert_eq!(device.reached_for(2), other);
    // Retiring passes the held work: its submitted segment completed.
    // Freeing does not: its held segments run later.
    assert_eq!(device.completed_serial(), device.submitted_serial());
    assert_eq!(device.freeable(), first - 1);
}

/// Closing a connection drops the held segments of a command buffer whose
/// first segments ran; it completes once those have.
#[test]
fn closing_drops_held_segments() {
    use EventStep::{Signal, Wait};
    let mut device = device_on(MockBackend::new());
    let (_, e) = new_events(&mut device, 2);
    commit_events(
        &mut device,
        &e,
        [1, 1],
        &[Signal(1, 1), Wait(0, 1), Signal(1, 2)],
    );
    commit_events(&mut device, &e, [1, 1], &[]);
    let submissions = device.backend().submissions.len();
    device.abandon(1);
    let other = commit_events(&mut device, &e, [2, 1], &[]);
    device.backend().finish();
    device.poll().unwrap();
    assert!(!device.holds());
    assert_eq!(device.backend().submissions.len(), submissions + 1);
    assert!(submitted_at(&mut device, (e[1], 2)).is_none());
    assert_eq!(device.reached_for(2), other);
    assert_eq!(device.drained(), device.submitted_serial());
}

/// A lost device's report names the owner of each whole allocation freed
/// lately, at the GPU range freeing it unbound, with the live ranges either
/// side: a GPU MMU fault address names one of them.
#[test]
fn the_memory_book_names_freed_ranges_and_their_neighbours() {
    let mut device = device();
    let token = core::num::NonZeroU64::new(11).unwrap();
    let shared = options(StorageMode::Shared);
    let surface = 64 * 64 * 4;
    let ranges = [stela_contracts::gate::MemoryRange {
        address: 0x8_0000_0000,
        bytes: surface,
    }];
    let made = device.submitted_serial();
    let memory = device.import_memory(&ranges, token).unwrap();
    device.book_holder(super::MemoryOwner::Import(memory), 3);
    let buffer = device.new_imported_buffer(memory, surface, shared).unwrap();
    let mut descriptor = texture(PixelFormat::Bgra8Unorm, TextureUsage::SHADER_READ);
    descriptor.width = 64;
    descriptor.height = 64;
    descriptor.options = shared;
    let texture = device
        .new_texture_from_buffer(buffer, &descriptor, 0, 256)
        .unwrap();
    let ranges = [stela_contracts::gate::MemoryRange {
        address: 0x9_0000_0000,
        bytes: 1 << 20,
    }];
    let heap_memory = device
        .import_memory(&ranges, core::num::NonZeroU64::new(12).unwrap())
        .unwrap();
    let heap = device
        .new_imported_heap(
            heap_memory,
            &HeapDescriptor {
                size: 1 << 20,
                options: shared,
                heap_type: HeapType::Automatic,
                sparse_page_size: SparsePageSize::Size64,
            },
        )
        .unwrap();
    device.book_holder(super::MemoryOwner::Heap(heap), 3);
    device.new_heap_buffer(heap, 4096, shared, None).unwrap();
    assert_eq!(
        device.memory_book_report(),
        ["lost device memory: no whole allocation with a known address was freed"]
    );
    let serial = submit(&mut device);
    device.destroy_texture(texture).unwrap();
    device.destroy_buffer(buffer).unwrap();
    // Not freed while submitted work may use it.
    assert!(device.memory_book_report()[0].contains("no whole allocation"));
    complete(&mut device, serial);
    let report = device.memory_book_report();
    assert_eq!(report.len(), 1, "{report:?}");
    let line = &report[0];
    assert!(
        line.contains(&alloc::format!(
            "({surface} bytes) buffer of {surface} bytes under a 64x64 Bgra8Unorm texture of \
             connection 3, made at serial {made}, released at serial {serial} when serial {serial} \
             completed"
        )),
        "{line}"
    );
    assert!(line.contains("live below: "), "{line}");
    assert!(
        line.contains("live above: ")
            && line.contains("(1048576 bytes) imported heap of 1048576 bytes of connection 3"),
        "{line}"
    );
}

/// A CPU-copy upload rides the next commit of any connection; behind held
/// work it goes ahead on its own. Once it ran, `completed` passes the held
/// command buffer's serial, and the texture's own connection may destroy
/// the texture: its CPU copy stays until the held command buffer is done. Freed at once, the
/// copy's GPU mapping was gone when the event released the command buffer:
/// an MMU fault on the copy's last page, which lost the device.
#[test]
fn uploads_a_held_command_buffer_carries_outlive_their_texture() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let descriptor = TextureDescriptor {
        width: 8,
        height: 4,
        options: options(StorageMode::Managed),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let texture = device.new_texture(&descriptor).unwrap();
    let copy = device.texture_cpu_copy(texture, 0, 0).unwrap().unwrap();
    let source = copy.buffer.buffer;
    let region = Region {
        slice: 0,
        level: 0,
        origin: [0, 0, 0],
        size: [8, 4, 1],
    };
    let strides = Strides {
        bytes_per_row: 32,
        bytes_per_image: 0,
    };
    device
        .replace_region(texture, &region, &[7; 128], strides)
        .unwrap();
    // Another connection's command buffer waits for an event the CPU
    // signals later; the upload rides it.
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_wait_for_event(&mut device.resolver(), semaphore, 1)
        .unwrap();
    let held = device.commit_for(2, commands).unwrap();
    assert!(device.holds());
    let freed =
        |device: &mut TestDevice| device.backend().destroyed.contains(&Object::Buffer(source));
    // The upload, submitted ahead, runs; the writer releases the texture:
    // none of its own work is unfinished.
    device.backend().finish();
    device.destroy_texture(texture).unwrap();
    device.poll().unwrap();
    assert!(device.completed_serial() >= held);
    assert!(
        !freed(&mut device),
        "the held command buffer reads the copy"
    );
    assert_eq!(destroyed_images(&mut device), 0);
    // The event arrives: the command buffer runs, reading the copy.
    device.set_signaled_value(event, 1).unwrap();
    assert!(!device.holds());
    let uploaded = device
        .backend()
        .submissions
        .iter()
        .flatten()
        .any(|segment| {
            segment.commands.iter().any(|command| {
            matches!(command, Command::CopyBufferToImage { src, .. } if *src == source)
        })
        });
    assert!(uploaded);
    assert!(!freed(&mut device));
    device.backend().finish();
    device.poll().unwrap();
    assert!(freed(&mut device), "freed once the command buffer ran");
    assert_eq!(destroyed_images(&mut device), 1);
}

/// A texture destroyed before the next commit takes its CPU-copy upload
/// leaves none for the texture that takes its slot.
#[test]
fn a_destroyed_texture_leaves_no_upload_for_the_next_in_its_slot() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let descriptor = TextureDescriptor {
        width: 8,
        height: 4,
        options: options(StorageMode::Managed),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let first = device.new_texture(&descriptor).unwrap();
    let region = Region {
        slice: 0,
        level: 0,
        origin: [0, 0, 0],
        size: [8, 4, 1],
    };
    let strides = Strides {
        bytes_per_row: 32,
        bytes_per_image: 0,
    };
    device
        .replace_region(first, &region, &[7; 128], strides)
        .unwrap();
    device.destroy_texture(first).unwrap();
    let second = device.new_texture(&descriptor).unwrap();
    assert_eq!(second, first, "the slot is reused");
    submit(&mut device);
    let uploaded = device
        .backend()
        .submissions
        .iter()
        .flatten()
        .any(|segment| {
            segment
                .commands
                .iter()
                .any(|command| matches!(command, Command::CopyBufferToImage { .. }))
        });
    assert!(!uploaded, "the second texture was never written");
}

/// Core Animation commits an unretained-reference command buffer that waits
/// for an event, then releases a resource it names, as a Mac's kernel keeps
/// such memory until the GPU is done. `completed` passes the held command
/// buffer, but its resources stay until it ran.
#[test]
fn what_a_held_command_buffer_names_outlives_its_release() {
    let mut device = device_on(MockBackend::new());
    let buffer = device
        .new_buffer(1 << 16, options(StorageMode::Private))
        .unwrap();
    let vk_buffer = device.buffer(buffer).unwrap().buffer;
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_wait_for_event(&mut device.resolver(), semaphore, 1)
        .unwrap();
    let held = device.commit_for(1, commands).unwrap();
    assert!(device.holds());
    device.destroy_buffer(buffer).unwrap();
    device.poll().unwrap();
    assert!(device.completed_serial() >= held);
    let freed = |device: &mut TestDevice| {
        device
            .backend()
            .destroyed
            .contains(&Object::Buffer(vk_buffer))
    };
    assert!(!freed(&mut device), "the held command buffer may read it");
    device.set_signaled_value(event, 1).unwrap();
    assert!(!freed(&mut device), "submitted, not yet complete");
    device.backend().finish();
    device.poll().unwrap();
    assert!(freed(&mut device));
    // With nothing in flight, a release frees at once.
    let idle = device
        .new_buffer(1 << 16, options(StorageMode::Private))
        .unwrap();
    let idle_buffer = device.buffer(idle).unwrap().buffer;
    device.destroy_buffer(idle).unwrap();
    assert!(
        device
            .backend()
            .destroyed
            .contains(&Object::Buffer(idle_buffer))
    );
}

/// Commits for connection `key` a command buffer held for an event nothing
/// has signaled yet.
fn hold_for_event(device: &mut TestDevice, key: u64) -> (EventId, u64) {
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_wait_for_event(&mut device.resolver(), semaphore, 1)
        .unwrap();
    let held = device.commit_for(key, commands).unwrap();
    assert!(device.holds());
    (event, held)
}

/// Releases what `release` destroys as connection `key` does for an object
/// it got at serial `made`.
fn release_as<T>(
    device: &mut TestDevice,
    key: u64,
    made: u64,
    release: impl FnOnce(&mut TestDevice) -> T,
) -> T {
    device.release_for(pins::Scope::connection(key, made), release)
}

/// One connection's command buffer held for an event nobody signals keeps
/// no other connection's releases (the audit of the held-work fix: all 100
/// stayed pinned under the device-wide rule).
#[test]
fn one_held_command_buffer_pins_no_other_connections_frees() {
    let mut device = device_on(MockBackend::new());
    hold_for_event(&mut device, 1);
    let mut pinned = 0;
    for _ in 0..100 {
        // Connection 2 makes and drops a buffer each frame, with a command
        // buffer of its own that completes.
        let made = device.submitted_serial();
        let buffer = device
            .new_buffer(1 << 16, options(StorageMode::Private))
            .unwrap();
        let vk_buffer = device.buffer(buffer).unwrap().buffer;
        let commands = device.new_command_buffer();
        device.commit_for(2, commands).unwrap();
        release_as(&mut device, 2, made, |device| device.destroy_buffer(buffer)).unwrap();
        device.backend().finish();
        device.poll().unwrap();
        if !device
            .backend()
            .destroyed
            .contains(&Object::Buffer(vk_buffer))
        {
            pinned += 1;
        }
    }
    assert_eq!(pinned, 0, "{pinned} of 100 frees by connection 2 pinned");
    let memory = device.memory();
    assert_eq!((memory.deferred, memory.pinned), (0, 0));
    assert!(device.holds());
}

/// A held command buffer keeps what its own connection held when it was
/// committed and released later, and nothing that connection made since.
#[test]
fn a_held_command_buffer_keeps_only_its_connections_older_objects() {
    let mut device = device_on(MockBackend::new());
    let made = device.submitted_serial();
    let older = device
        .new_buffer(1 << 16, options(StorageMode::Private))
        .unwrap();
    let older_buffer = device.buffer(older).unwrap().buffer;
    let (event, _) = hold_for_event(&mut device, 1);
    let newer_made = device.submitted_serial();
    let newer = device
        .new_buffer(1 << 16, options(StorageMode::Private))
        .unwrap();
    let newer_buffer = device.buffer(newer).unwrap().buffer;
    release_as(&mut device, 1, made, |device| device.destroy_buffer(older)).unwrap();
    release_as(&mut device, 1, newer_made, |device| {
        device.destroy_buffer(newer)
    })
    .unwrap();
    device.poll().unwrap();
    let freed = |device: &mut TestDevice, buffer| {
        device.backend().destroyed.contains(&Object::Buffer(buffer))
    };
    assert!(
        !freed(&mut device, older_buffer),
        "the held work may read it"
    );
    assert!(freed(&mut device, newer_buffer), "made after the commit");
    let memory = device.memory();
    assert_eq!(memory.pinned, 2, "the buffer and its memory");
    assert!(memory.pinned_bytes >= 1 << 16);
    device.set_signaled_value(event, 1).unwrap();
    device.backend().finish();
    device.poll().unwrap();
    assert!(freed(&mut device, older_buffer));
    let memory = device.memory();
    assert_eq!((memory.deferred, memory.pinned), (0, 0));
    assert!(memory.peak_pinned_bytes >= 1 << 16, "{memory:?}");
}

/// A texture whose CPU-copy upload rides another connection's held
/// command buffer stays until that command buffer ran, though its own
/// connection has no unfinished work.
#[test]
fn a_texture_another_connections_held_work_uploads_waits_for_it() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let descriptor = TextureDescriptor {
        width: 8,
        height: 4,
        options: options(StorageMode::Managed),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let made = device.submitted_serial();
    let texture = device.new_texture(&descriptor).unwrap();
    let copy = device.texture_cpu_copy(texture, 0, 0).unwrap().unwrap();
    let source = copy.buffer.buffer;
    let region = Region {
        slice: 0,
        level: 0,
        origin: [0, 0, 0],
        size: [8, 4, 1],
    };
    let strides = Strides {
        bytes_per_row: 32,
        bytes_per_image: 0,
    };
    device
        .replace_region(texture, &region, &[7; 128], strides)
        .unwrap();
    let (event, _) = hold_for_event(&mut device, 2);
    release_as(&mut device, 1, made, |device| {
        device.destroy_texture(texture)
    })
    .unwrap();
    device.poll().unwrap();
    let freed =
        |device: &mut TestDevice| device.backend().destroyed.contains(&Object::Buffer(source));
    assert!(
        !freed(&mut device),
        "connection 2's held work reads the copy"
    );
    device.set_signaled_value(event, 1).unwrap();
    device.backend().finish();
    device.poll().unwrap();
    assert!(freed(&mut device));
}

/// A sampler state equal descriptors share across connections waits for
/// every connection that held it.
#[test]
fn a_shared_sampler_state_waits_for_each_holders_work() {
    let mut device = device_on(MockBackend::new());
    let descriptor = SamplerDescriptor::default();
    let first = device.new_sampler(&descriptor).unwrap();
    let second = device.new_sampler(&descriptor).unwrap();
    let (event, _) = hold_for_event(&mut device, 1);
    release_as(&mut device, 1, 0, |device| device.destroy_sampler(first)).unwrap();
    release_as(&mut device, 2, 0, |device| device.destroy_sampler(second)).unwrap();
    device.poll().unwrap();
    let samplers = |device: &mut TestDevice| {
        device
            .backend()
            .destroyed
            .iter()
            .filter(|object| matches!(object, Object::Sampler(_)))
            .count()
    };
    assert_eq!(
        samplers(&mut device),
        0,
        "connection 1's held work may read it"
    );
    device.set_signaled_value(event, 1).unwrap();
    device.backend().finish();
    device.poll().unwrap();
    assert_eq!(samplers(&mut device), 1);
}

/// Under another connection's hold, 10,000 frames that each make and drop
/// a sampler state and a texture never run out of descriptors: the device
/// keeps 4,000 samplers, which the device-wide rule exhausted.
#[test]
fn descriptors_last_10000_frames_under_another_connections_hold() {
    let mut device = device_on(MockBackend::new());
    hold_for_event(&mut device, 1);
    let before = device.memory().descriptors;
    let mut descriptor = texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ);
    descriptor.options = options(StorageMode::Shared);
    descriptor.width = 16;
    descriptor.height = 16;
    for frame in 0..10_000 {
        let made = device.submitted_serial();
        let sampler = device
            .new_sampler(&SamplerDescriptor::default())
            .unwrap_or_else(|error| panic!("frame {frame}: {error:?}"));
        let texture = device
            .new_texture(&descriptor)
            .unwrap_or_else(|error| panic!("frame {frame}: {error:?}"));
        let commands = device.new_command_buffer();
        device.commit_for(2, commands).unwrap();
        release_as(&mut device, 2, made, |device| {
            device.destroy_sampler(sampler)?;
            device.destroy_texture(texture)
        })
        .unwrap();
        device.backend().finish();
        device.poll().unwrap();
    }
    let memory = device.memory();
    assert_eq!(memory.descriptors, before, "{memory:?}");
    assert_eq!(memory.deferred, 0);
    assert!(device.holds());
}

#[test]
fn an_events_value_needs_no_driver_query_once_it_reached_every_promise() {
    let mut device = device_on(MockBackend::new());
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    // A query would fail.
    device.backend().semaphore_error = Some(Error::DeviceLost);
    assert_eq!(device.signaled_value(event), Ok(0), "nothing signals it");
    device.backend().semaphore_error = None;
    let mut commands = device.new_command_buffer();
    commands
        .encode_signal_event(&mut device.resolver(), semaphore, 3)
        .unwrap();
    device.commit(commands).unwrap();
    // Submitted, not yet run: the driver answers.
    assert_eq!(device.signaled_value(event), Ok(0));
    device.backend().finish();
    assert_eq!(device.signaled_value(event), Ok(3));
    device.backend().semaphore_error = Some(Error::DeviceLost);
    assert_eq!(
        device.signaled_value(event),
        Ok(3),
        "reached what was promised"
    );
    assert_eq!(device.wait_event(event, 3, 0), Ok(true));
    device.backend().semaphore_error = None;
    // The CPU's own signal is known at once.
    device.set_signaled_value(event, 9).unwrap();
    device.backend().semaphore_error = Some(Error::DeviceLost);
    assert_eq!(device.signaled_value(event), Ok(9));
    device.backend().semaphore_error = None;
}

#[test]
fn a_submission_the_driver_refuses_fails_only_its_command_buffer() {
    use alloc::sync::Arc;
    use core::sync::atomic::{AtomicU32, Ordering};
    let mut device = device_on(MockBackend::new());
    let outstanding = Arc::new(AtomicU32::new(7));
    device.count_failures(&outstanding);
    assert_eq!(
        outstanding.load(Ordering::Acquire),
        0,
        "counting starts at none"
    );
    // The driver refuses the first command buffer's submission but makes
    // its signals, as NVK's refusal does.
    device.backend().refuse_next = Some(Error::OutOfMemory);
    let commands = device.new_command_buffer();
    let first = device.commit_for(1, commands).unwrap();
    assert_ne!(
        outstanding.load(Ordering::Acquire),
        0,
        "counted before its signals show"
    );
    let commands = device.new_command_buffer();
    let second = device.commit_for(1, commands).unwrap();
    let commands = device.new_command_buffer();
    let other = device.commit_for(2, commands).unwrap();
    device.backend().finish();
    assert_eq!(device.poll(), Ok(other), "later command buffers complete");
    assert!(!device.is_lost(), "the device is kept");
    assert_eq!(device.submit_failed_for(1), first);
    assert_eq!(device.submit_failed_for(2), 0);
    assert_eq!(
        device.take_submit_failure(1, second),
        None,
        "the next one ran"
    );
    assert_eq!(
        device.take_submit_failure(1, first),
        Some(Error::OutOfMemory),
        "its commit's failure"
    );
    assert_eq!(device.take_submit_failure(1, first), None, "answered once");
    assert_eq!(device.submit_failed_for(1), 0);
    assert_eq!(outstanding.load(Ordering::Acquire), 0, "settled");
    // The device goes on.
    let commands = device.new_command_buffer();
    let next = device.commit_for(1, commands).unwrap();
    device.backend().finish();
    assert_eq!(device.poll(), Ok(next));
}

#[test]
fn a_refusal_no_command_buffer_takes_is_forgotten_once_nothing_is_in_flight() {
    use alloc::sync::Arc;
    use core::sync::atomic::{AtomicU32, Ordering};
    let mut device = device_on(MockBackend::new());
    let outstanding = Arc::new(AtomicU32::new(0));
    device.count_failures(&outstanding);
    let commands = device.new_command_buffer();
    let serial = device.commit_for(1, commands).unwrap();
    // A refusal named by a value no command buffer signals: the device's
    // own work.
    let stray = device.backend().create_semaphore(0).unwrap();
    device
        .backend()
        .refused
        .push((stray, 1, Error::OutOfMemory));
    outstanding.fetch_add(1, Ordering::AcqRel);
    device.backend().finish();
    assert_eq!(device.poll(), Ok(serial));
    assert_eq!(device.submit_failed_for(1), 0);
    assert_eq!(outstanding.load(Ordering::Acquire), 0, "forgotten");
}

#[test]
fn a_closed_connections_refusals_are_settled() {
    use alloc::sync::Arc;
    use core::sync::atomic::{AtomicU32, Ordering};
    let mut device = device_on(MockBackend::new());
    let outstanding = Arc::new(AtomicU32::new(0));
    device.count_failures(&outstanding);
    device.backend().refuse_next = Some(Error::OutOfMemory);
    let commands = device.new_command_buffer();
    let first = device.commit_for(1, commands).unwrap();
    device.backend().finish();
    assert_eq!(device.poll(), Ok(first));
    assert_eq!(device.submit_failed_for(1), first);
    device.abandon(1);
    assert_eq!(device.submit_failed_for(1), 0);
    assert_eq!(outstanding.load(Ordering::Acquire), 0);
}

/// A managed texture of connection 1 written from the CPU while connection
/// 2's first command buffer waits for `event`; connection 2 then commits
/// again, behind its held work. Returns the texture's CPU copy buffer.
fn upload_behind_held_carrier(device: &mut TestDevice, event: EventId) -> (TextureId, Buffer) {
    let descriptor = TextureDescriptor {
        width: 8,
        height: 4,
        options: options(StorageMode::Managed),
        ..texture(PixelFormat::Rgba8Unorm, TextureUsage::SHADER_READ)
    };
    let texture = device.new_texture(&descriptor).unwrap();
    let copy = device.texture_cpu_copy(texture, 0, 0).unwrap().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_wait_for_event(&mut device.resolver(), semaphore, 1)
        .unwrap();
    device.commit_for(2, commands).unwrap();
    assert!(device.holds());
    let region = Region {
        slice: 0,
        level: 0,
        origin: [0, 0, 0],
        size: [8, 4, 1],
    };
    let strides = Strides {
        bytes_per_row: 32,
        bytes_per_image: 0,
    };
    device
        .replace_region(texture, &region, &[7; 128], strides)
        .unwrap();
    let commands = device.new_command_buffer();
    device.commit_for(2, commands).unwrap();
    (texture, copy.buffer.buffer)
}

/// The first submission that copies `source` to an image.
fn upload_submission(device: &mut TestDevice, source: Buffer) -> Option<usize> {
    device.backend().submissions.iter().position(|submission| {
        submission.iter().any(|segment| {
            segment.commands.iter().any(|command| {
                matches!(command, Command::CopyBufferToImage { src, .. } if *src == source)
            })
        })
    })
}

/// A CPU-copy upload that another connection's held command buffer would
/// carry is made even when that connection closes before its event
/// (auditgpu2 finding 1).
#[test]
fn cpu_copy_upload_survives_its_held_carrier_closing() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let event = device.new_event().unwrap();
    let (texture, source) = upload_behind_held_carrier(&mut device, event);
    let commands = device.new_command_buffer();
    device.commit_for(1, commands).unwrap();
    let late = upload_submission(&mut device, source).is_none();
    device.abandon(2);
    device.backend().finish();
    device.poll().unwrap();
    let commands = device.new_command_buffer();
    device.commit_for(1, commands).unwrap();
    device.backend().finish();
    device.poll().unwrap();
    let made = upload_submission(&mut device, source).is_some();
    assert!(!late && made, "late: {late}; ever made: {made}");
    assert!(!device.holds());
    device.destroy_texture(texture).unwrap();
}

/// While the carrier is held, the upload is submitted on its own, waiting
/// for nothing, ahead of the owner's later work; the carrier's own work
/// still runs once its event arrives, and every command buffer completes.
#[test]
fn cpu_copy_upload_precedes_owners_later_work_while_carrier_held() {
    let mut backend = MockBackend::new();
    backend.physical.as_mut().unwrap().features.host_image_copy = false;
    let mut device = device_on(backend);
    let event = device.new_event().unwrap();
    let (texture, source) = upload_behind_held_carrier(&mut device, event);
    let upload = upload_submission(&mut device, source).expect("the upload is submitted");
    let submission = &device.backend().submissions[upload];
    assert_eq!(submission.len(), 1);
    assert!(submission[0].waits.is_empty(), "it waits for nothing");
    assert!(matches!(
        submission[0].commands.first(),
        Some(Command::Barrier(_))
    ));
    assert!(matches!(
        submission[0].commands.last(),
        Some(Command::Barrier(_))
    ));
    let before = device.backend().submissions.len();
    let commands = device.new_command_buffer();
    let owner = device.commit_for(1, commands).unwrap();
    assert!(device.backend().submissions.len() > before);
    assert!(upload < before, "the upload precedes the owner's work");
    // The texture's destruction still waits for the carrier.
    device.destroy_texture(texture).unwrap();
    device.set_signaled_value(event, 1).unwrap();
    assert!(device.held_connections().is_empty());
    device.backend().finish();
    assert_eq!(device.poll(), Ok(owner));
    assert!(!device.holds());
}

/// A GPU signal queued before a higher CPU signal can reach the event
/// after it, and the driver then answers the lower value until it runs;
/// the event's value never goes down (auditgpu2 finding 4).
#[test]
fn event_value_never_goes_below_a_cpu_signal() {
    let mut device = device_on(MockBackend::new());
    let event = device.new_event().unwrap();
    let semaphore = device.event_semaphore(event).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_signal_event(&mut device.resolver(), semaphore, 5)
        .unwrap();
    device.commit(commands).unwrap();
    device.set_signaled_value(event, 10).unwrap();
    assert_eq!(device.signaled_value(event), Ok(10));
    // A later GPU signal is promised, so the value is queried again.
    let mut commands = device.new_command_buffer();
    commands
        .encode_signal_event(&mut device.resolver(), semaphore, 20)
        .unwrap();
    device.commit(commands).unwrap();
    // The reordered point: the driver answers the unrun signal's chain.
    device.backend().semaphores.insert(semaphore.0, 0);
    assert_eq!(device.signaled_value(event), Ok(10));
    device.backend().finish();
    assert_eq!(device.signaled_value(event), Ok(20));
}

/// A flight that signaled an event completes after the event was destroyed;
/// the driver then hands the freed semaphore to a new event. The old value
/// must not answer for the new event before its own signal runs.
#[test]
fn a_destroyed_events_late_completion_does_not_answer_for_its_handles_next_event() {
    let mut backend = MockBackend::new();
    backend.reuse_semaphores = true;
    let mut device = device_on(backend);
    let a = device.new_event().unwrap();
    let first = device.event_semaphore(a).unwrap();
    let mut commands = device.new_command_buffer();
    commands
        .encode_signal_event(&mut device.resolver(), first, 5)
        .unwrap();
    device.commit(commands).unwrap();
    // Destroyed while the signal is still running.
    device.destroy_event(a).unwrap();
    device.backend().finish();
    device.poll().unwrap();
    assert!(
        device
            .backend()
            .destroyed
            .contains(&Object::Semaphore(first)),
        "freed once its work completed"
    );
    let b = device.new_event().unwrap();
    assert_eq!(
        device.event_semaphore(b).unwrap(),
        first,
        "the handle is reused"
    );
    // B's own signal of 3 is submitted, not yet run.
    let mut commands = device.new_command_buffer();
    commands
        .encode_signal_event(&mut device.resolver(), first, 3)
        .unwrap();
    device.commit(commands).unwrap();
    assert_eq!(device.signaled_value(b), Ok(0));
    assert_eq!(device.wait_event(b, 3, 0), Ok(false));
}

/// A waiter that took the completion timeline and left gets it again: the
/// timeline is exported anew, never once the device is lost.
#[test]
fn the_completion_timeline_is_offered_again_until_the_device_is_lost() {
    let mut backend = MockBackend::new();
    backend.exported = Some(7);
    let mut device = device_on(backend);
    assert_eq!(device.take_completion(), Some(7));
    assert_eq!(device.take_completion(), None);
    assert!(device.rearm_completion());
    assert!(device.rearm_completion(), "offered once until taken");
    assert_eq!(device.take_completion(), Some(7));
    device.mark_lost();
    assert!(!device.rearm_completion());
    assert_eq!(device.take_completion(), None);
}
