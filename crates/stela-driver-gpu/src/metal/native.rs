//! [`Backend`] over a Vulkan driver loaded in this process: NVK on NVIDIA,
//! or another Mesa driver the device admits.
//!
//! The platform supplies the driver's `vkGetInstanceProcAddr` and the PCI
//! function to drive. The backend creates one instance and one device with
//! the extensions in [`physical`] and the features the driver offers (less
//! robust access), takes one universal queue, and builds the bindless heap
//! and the single pipeline layout of the [binding ABI](super::binding).
//!
//! Handles cross the boundary as their raw Vulkan values, except memory:
//! buffers and images are placed in larger blocks ([`blocks`]), so a
//! [`Memory`] is the backend's own number for a block place or a whole
//! allocation ([`memory`]). Shader code is
//! NIR, which pipelines pass in Mesa's `VkPipelineShaderStageNirCreateInfoMESA`
//! (as Mesa's own meta shaders do) without a module or SPIR-V. Images are moved to `VK_IMAGE_LAYOUT_GENERAL` once, on the host
//! when they allow host transfers and otherwise at the start of the next
//! submission, and stay there.
mod blocks;
mod deferral;
mod device_generated;
mod memory;
mod objects;
mod physical;
pub mod pipeline_store;
mod premade;
mod record;
mod sparse;
#[cfg(test)]
mod tests;
mod zero;

use super::{
    binding::{HEAP_SET, HeapLayout, NULL_DESCRIPTOR, PushConstants},
    command_buffer::Segment,
    error::{Error, Result, ensure},
    vulkan::{
        SampleCounts,
        backend::{
            Backend, BufferAllocation, Descriptor, MemoryBlocks, Object, Outside, PreparedImport,
            Requirements, requirements,
        },
        command::{BufferImageCopy, Command, Subresource},
        object::{
            Buffer, BufferInfo, BufferView, BufferViewInfo, ComputePipelineInfo, ExecutionSet,
            GraphicsPipelineInfo, Image, ImageInfo, ImageView, ImageViewInfo,
            IndirectCommandsLayout, IndirectCommandsLayoutInfo, Memory, MemoryRequest, Pipeline,
            PipelineCache, PipelineSource, QueryPool, QueryPoolInfo, Sampler, SamplerInfo,
            Semaphore, ShaderCode, SparseImageRequirements,
        },
        physical::{PciBusInfo, PhysicalDevice},
    },
};
use alloc::{
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    format,
    sync::Arc,
    vec::Vec,
};
pub use ash::vk;
use ash::{ext, khr};
use blocks::{Candidate, Needs, Purpose, Resident, Source, Tiling};
use core::{
    any::Any,
    cell::UnsafeCell,
    ffi::{CStr, c_void},
    mem::ManuallyDrop,
    num::NonZeroU64,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use physical::{Extensions, FeatureChain};
use stela_contracts::gate::MemoryRange;

/// The driver's loader entry point, and where its messages go.
#[derive(Clone, Copy)]
pub struct Loader {
    pub get_instance_proc_addr: vk::PFN_vkGetInstanceProcAddr,
    /// Told each error and warning the driver reports, such as why a call
    /// failed with only a generic result.
    pub log: fn(&str),
    /// Puts a queue's submissions on the driver's submit thread, which makes
    /// them where the driver's kernel runs: `vkQueueSubmit2` then returns once
    /// the work is queued. `None` keeps submissions on the caller.
    pub submit_thread: Option<unsafe extern "C" fn(vk::Queue) -> vk::Result>,
    /// Submissions the driver refused while keeping the device. `None` for
    /// a driver that loses the device instead.
    pub refusals: Option<Refusals>,
    /// Waits a moment, as the platform's waits do: a call outside the
    /// service's turn may wait for the boot processor.
    pub relax: fn(),
}

/// The driver's record of submissions it refused while keeping the device:
/// it still made their signals, after their waits.
#[derive(Clone, Copy)]
pub struct Refusals {
    /// Takes the failure of the refused submission that signaled a
    /// semaphore value; `SUCCESS` for every other.
    pub take: unsafe extern "C" fn(vk::Queue, vk::Semaphore, u64) -> vk::Result,
    /// Drops the kept failures nobody will take; answers how many.
    pub forget: unsafe extern "C" fn(vk::Queue) -> u32,
    /// Has the queue raise a count by each failure it keeps, before the
    /// refused submission's signals are made, and lower it by each it drops
    /// unasked.
    pub count: unsafe extern "C" fn(vk::Queue, *mut u32),
}

/// What a device whose CPU reaches only part of its memory (no Resizable
/// BAR) costs: textures the CPU copies to and from (`HOST_TRANSFER` images,
/// Metal's shared and managed textures) must then share the window the
/// CPU reaches, and run out of memory at its size. `None` when the CPU
/// reaches all the device-local memory.
fn small_bar(memory: &super::vulkan::physical::Memory) -> Option<alloc::string::String> {
    use super::vulkan::MemoryHeapFlags;
    use super::vulkan::MemoryProperties as P;
    let local = memory
        .heaps
        .iter()
        .filter(|heap| heap.flags.contains(MemoryHeapFlags::DEVICE_LOCAL))
        .map(|heap| heap.size)
        .max()?;
    let reached = memory
        .types
        .iter()
        .filter(|kind| kind.properties.contains(P::DEVICE_LOCAL | P::HOST_VISIBLE))
        .filter_map(|kind| memory.heaps.get(kind.heap_index as usize))
        .map(|heap| heap.size)
        .max()
        .unwrap_or(0);
    // The CPU reaching less than all of it, with room for the kernel's own.
    (reached < local - local / 8).then(|| {
        format!(
            "GPU memory the CPU reaches is {} MiB of {} MiB (no Resizable BAR): textures the \
             CPU copies share it",
            reached >> 20,
            local >> 20
        )
    })
}

/// Forwards the driver's errors and warnings to a [`Loader::log`] through
/// `VK_EXT_debug_utils`, for drivers that keep them otherwise.
struct Messenger {
    fns: ext::debug_utils::Instance,
    handle: vk::DebugUtilsMessengerEXT,
}

impl Messenger {
    /// # Safety
    ///
    /// `instance` was created from `entry` with `VK_EXT_debug_utils`.
    unsafe fn new(entry: &ash::Entry, instance: &ash::Instance, log: fn(&str)) -> Option<Self> {
        let fns = ext::debug_utils::Instance::new(entry, instance);
        let info = vk::DebugUtilsMessengerCreateInfoEXT::default()
            .message_severity(
                vk::DebugUtilsMessageSeverityFlagsEXT::ERROR
                    | vk::DebugUtilsMessageSeverityFlagsEXT::WARNING,
            )
            .message_type(
                vk::DebugUtilsMessageTypeFlagsEXT::GENERAL
                    | vk::DebugUtilsMessageTypeFlagsEXT::VALIDATION
                    | vk::DebugUtilsMessageTypeFlagsEXT::PERFORMANCE,
            )
            .pfn_user_callback(Some(forward))
            .user_data(log as *mut c_void);
        // SAFETY: the instance enables the extension; `forward` reads the
        // user data as the `fn(&str)` given here, which is 'static.
        let handle = unsafe { fns.create_debug_utils_messenger(&info, None) }.ok()?;
        Some(Self { fns, handle })
    }

    /// # Safety
    ///
    /// Called once, before the instance is destroyed.
    unsafe fn destroy(&self) {
        // SAFETY: the caller's guarantee.
        unsafe { self.fns.destroy_debug_utils_messenger(self.handle, None) };
    }
}

/// Hands a driver message to the `fn(&str)` its messenger was made with.
unsafe extern "system" fn forward(
    _: vk::DebugUtilsMessageSeverityFlagsEXT,
    _: vk::DebugUtilsMessageTypeFlagsEXT,
    data: *const vk::DebugUtilsMessengerCallbackDataEXT<'_>,
    user: *mut c_void,
) -> vk::Bool32 {
    // SAFETY: `Messenger::new` made `user` from a `fn(&str)`, and the driver
    // passes valid callback data or null.
    let (log, data) = unsafe {
        (
            core::mem::transmute::<*mut c_void, fn(&str)>(user),
            data.as_ref(),
        )
    };
    if let Some(data) = data.filter(|data| !data.p_message.is_null()) {
        // SAFETY: the driver's message is a NUL-terminated string for the call.
        if let Ok(message) = unsafe { CStr::from_ptr(data.p_message) }.to_str() {
            log(message.trim_end());
        }
    }
    vk::FALSE
}

/// The backend's own kernels: the NIR of `stela_metal_accumulate_kernel`,
/// `stela_metal_select_kernel`, `stela_metal_resolve_draws_kernel`,
/// `stela_metal_resolve_dispatches_kernel`, `stela_metal_store_texels_kernel`,
/// `stela_metal_load_texels_kernel`, `stela_metal_average_samples_kernel`,
/// `stela_metal_decode_etc_kernel`, `stela_metal_decode_pvrtc_kernel`,
/// `stela_metal_reorder_blocks_kernel` and `stela_metal_decode_astc_kernel`
/// (`compiler/metal_abi.h`).
#[derive(Clone, Copy, Debug)]
pub struct InternalKernels {
    /// Sums query results per
    /// [`AccumulateArguments`](super::binding::AccumulateArguments), one
    /// thread per run in workgroups of `WORKGROUP_SIZE`.
    pub accumulate: ShaderCode,
    /// Flags commands of an execution range per
    /// [`SelectArguments`](super::binding::SelectArguments), one thread per
    /// command in workgroups of `WORKGROUP_SIZE`.
    pub select: ShaderCode,
    /// Resolves GPU-resident render indirect commands per
    /// [`ResolveArguments`](super::binding::ResolveArguments), one thread
    /// per command in workgroups of `WORKGROUP_SIZE`.
    pub resolve_draws: ShaderCode,
    /// Resolves GPU-resident compute indirect commands likewise.
    pub resolve_dispatches: ShaderCode,
    /// Converts a buffer's Metal texels into a texture's stored ones per
    /// [`TexelArguments`](super::binding::TexelArguments), one thread per
    /// texel in workgroups of `WORKGROUP_SIZE`.
    pub store_texels: ShaderCode,
    /// Converts a texture's stored texels into a buffer's Metal ones likewise.
    pub load_texels: ShaderCode,
    /// Resolves an attachment rendered through its raw texels per
    /// [`AverageArguments`](super::binding::AverageArguments), one thread
    /// per texel in workgroups of `WORKGROUP_SIZE`.
    pub average_samples: ShaderCode,
    /// Decodes a buffer's ETC2 and EAC blocks into a texture's image per
    /// [`TexelArguments`](super::binding::TexelArguments), one thread per
    /// texel in workgroups of `WORKGROUP_SIZE`.
    pub decode_etc: ShaderCode,
    /// Decodes a buffer's level of PVRTC blocks likewise.
    pub decode_pvrtc: ShaderCode,
    /// Copies blocks between a buffer's rows and a PVRTC level's Morton
    /// order per [`ReorderArguments`](super::binding::ReorderArguments),
    /// one thread per block in workgroups of `WORKGROUP_SIZE`.
    pub reorder_blocks: ShaderCode,
    /// Decodes a buffer's ASTC LDR blocks into a texture's image as
    /// [`decode_etc`](Self::decode_etc) does.
    pub decode_astc: ShaderCode,
}

/// How the driver takes memory a CPU reaches: its kernel driver maps a dma-buf.
const DMA_BUF: vk::ExternalMemoryHandleTypeFlags = vk::ExternalMemoryHandleTypeFlags::DMA_BUF_EXT;

/// A host mapping of a `VkDeviceMemory`.
/// Memory the platform holds, for the driver to import.
pub trait MemoryExporter: Send + Sync {
    /// `ranges` for the driver, and a mapping of them for the host.
    fn export(&self, ranges: &[MemoryRange]) -> Result<ExportedMemory>;
    /// Undoes an export the driver did not import, which leaves it the
    /// descriptor.
    fn close(&self, exported: ExportedMemory);
    /// Ends the host's mapping of memory the driver freed, which took the
    /// descriptor when it imported.
    fn unmap(&self, exported: ExportedMemory);
}

/// An export: `fd` is a dma-buf of `bytes`, which the host reaches at
/// `pointer`.
#[derive(Clone, Copy, Debug)]
pub struct ExportedMemory {
    pub fd: i32,
    pub pointer: NonNull<u8>,
    pub bytes: u64,
}

#[derive(Clone, Copy, Debug)]
struct Mapping {
    pointer: NonNull<u8>,
    size: u64,
    coherent: bool,
}

/// A mapped range: its host pointer, and its `VkDeviceMemory` and offset
/// there.
#[derive(Clone, Copy, Debug)]
struct Mapped {
    pointer: NonNull<u8>,
    memory: vk::DeviceMemory,
    offset: u64,
    coherent: bool,
}

/// How a buffer or image takes memory.
#[derive(Clone, Copy, Debug)]
struct Resource {
    tiling: Tiling,
    /// The least alignment of its place.
    alignment: u64,
    /// The driver requires or prefers memory of the resource's own.
    own: bool,
}

/// Where a buffer's memory starts at the least: NIL aligns a linear image
/// to at most 256 bytes (`Image::new_linear`), and memory of the buffer's
/// own started at 0, so a linear image over the buffer binds wherever
/// Metal's offsets put it.
const LINEAR_PLACEMENT: u64 = 256;

/// Whether `VkMemoryDedicatedRequirements` asks for memory of the
/// resource's own.
fn own(dedicated: &vk::MemoryDedicatedRequirements<'_>) -> bool {
    dedicated.prefers_dedicated_allocation == vk::TRUE
        || dedicated.requires_dedicated_allocation == vk::TRUE
}

/// The requirements of `image`, and whether the driver requires or prefers
/// memory of its own for it (`VkMemoryDedicatedRequirements`).
fn image_requirements(device: &ash::Device, image: vk::Image) -> (vk::MemoryRequirements, bool) {
    let info = vk::ImageMemoryRequirementsInfo2::default().image(image);
    let mut dedicated = vk::MemoryDedicatedRequirements::default();
    let mut requirements = vk::MemoryRequirements2::default().push_next(&mut dedicated);
    // SAFETY: the image is live and the chain outlives the call.
    unsafe { device.get_image_memory_requirements2(&info, &mut requirements) };
    let needs = requirements.memory_requirements;
    (needs, own(&dedicated))
}

#[derive(Debug)]
enum CommandBuffers {
    One(vk::CommandBuffer),
    Many(Vec<vk::CommandBuffer>),
}

impl CommandBuffers {
    /// The pool belongs to this device and is used only by the caller.
    fn allocate(device: &ash::Device, pool: vk::CommandPool, count: u32) -> Result<Self> {
        let info = vk::CommandBufferAllocateInfo::default()
            .command_pool(pool)
            .level(vk::CommandBufferLevel::PRIMARY)
            .command_buffer_count(count);
        if count == 1 {
            let mut buffer = vk::CommandBuffer::null();
            // SAFETY: one writable handle is sufficient for the requested count.
            unsafe {
                (device.fp_v1_0().allocate_command_buffers)(device.handle(), &info, &mut buffer)
            }
            .result()
            .map_err(failure)?;
            return Ok(Self::One(buffer));
        }
        // SAFETY: ash provides output space for the requested count.
        unsafe { device.allocate_command_buffers(&info) }
            .map(Self::Many)
            .map_err(failure)
    }

    fn as_slice(&self) -> &[vk::CommandBuffer] {
        match self {
            Self::One(buffer) => core::slice::from_ref(buffer),
            Self::Many(buffers) => buffers,
        }
    }
}

/// Unfreed submissions past which a submission asks the timeline what
/// completed rather than waiting for a completion poll to.
const RECLAIM_AFTER: usize = 64;

/// Command buffers of one submission, freed once the queue's timeline
/// passes `value`.
#[derive(Debug)]
struct InFlight {
    buffers: CommandBuffers,
    timeline: vk::Semaphore,
    value: u64,
}

impl InFlight {
    /// Counts finished submissions from the front of the queue. Timeline
    /// values only increase, so one observation covers consecutive
    /// submissions that signal the same semaphore.
    fn completed_prefix(
        flights: &VecDeque<Self>,
        mut observe: impl FnMut(vk::Semaphore) -> Result<u64>,
    ) -> Result<usize> {
        let mut observed = None;
        let mut ready = 0;
        for entry in flights {
            let value = match observed {
                Some((timeline, value)) if timeline == entry.timeline => value,
                _ => {
                    let value = observe(entry.timeline)?;
                    observed = Some((entry.timeline, value));
                    value
                }
            };
            if value < entry.value {
                break;
            }
            ready += 1;
        }
        Ok(ready)
    }
}

/// The bindless descriptor heap: set 0 of every pipeline.
#[derive(Clone, Copy, Debug)]
struct Heap {
    layout: vk::DescriptorSetLayout,
    pool: vk::DescriptorPool,
    set: vk::DescriptorSet,
}

/// Device-level functions of the optional extensions.
struct ExtensionFns {
    host_image_copy: Option<ext::host_image_copy::Device>,
    dynamic_state3: Option<ext::extended_dynamic_state3::Device>,
    sample_locations: Option<ext::sample_locations::Device>,
    /// Depth bias by pointer: a target without floating-point registers, such
    /// as `stela.efi`'s Rust, cannot pass the core command's floats to C.
    depth_bias: Option<ext::depth_bias_control::Device>,
    mesh: Option<ext::mesh_shader::Device>,
    calibrated: Option<khr::calibrated_timestamps::Device>,
    external_memory_fd: Option<khr::external_memory_fd::Device>,
    /// Exports the queue's timeline as a DRM syncobj, which the platform
    /// waits on to learn of completions as they happen.
    external_semaphore_fd: Option<khr::external_semaphore_fd::Device>,
    conditional: ext::conditional_rendering::Device,
    /// With the snapshot's `generated_commands` limits.
    generated: Option<device_generated::Fns>,
    /// Each pipeline's compiled shaders' statistics, which the backend
    /// reports through the loader's log for a shader that spills or is
    /// large: NAK's work and memory grow with a block's instructions.
    executables: Option<Executables>,
}

/// The functions that read a pipeline's shader statistics, and the loader's
/// log they are reported to.
struct Executables {
    fns: khr::pipeline_executable_properties::Device,
    log: fn(&str),
}

pub struct NativeBackend {
    // A failed idle query leaves the driver's code and export owner live.
    _entry: ManuallyDrop<ash::Entry>,
    instance: ash::Instance,
    messenger: Option<Messenger>,
    device: ash::Device,
    fns: ExtensionFns,
    queue: vk::Queue,
    snapshot: PhysicalDevice,
    heap: Heap,
    /// The sampler behind the samplers' null descriptor, which Vulkan's
    /// `nullDescriptor` does not cover.
    null_sampler: vk::Sampler,
    layout: vk::PipelineLayout,
    command_pool: vk::CommandPool,
    in_flight: VecDeque<InFlight>,
    /// Command buffers of submissions without a completion signal of their
    /// own: the next signal covers them, as a signal's first synchronization
    /// scope includes every command earlier in submission order.
    unsignaled: Vec<vk::CommandBuffer>,
    /// Host mappings of whole `VkDeviceMemory` allocations, by handle.
    mappings: BTreeMap<u64, Mapping>,
    /// What each [`Memory`] value names, and the blocks resources share.
    memories: memory::Memories,
    exporter: ManuallyDrop<Arc<dyn MemoryExporter>>,
    /// The driver work calls do outside the service's turn: imports'
    /// exports and allocations, and frees of evicted kept imports.
    outside: Arc<NativeOutside>,
    /// What each imported memory came from, its token and the guest RAM
    /// it views.
    imports: BTreeMap<u64, (ExportedMemory, NonZeroU64, Vec<MemoryRange>)>,
    /// Imports freed lately, oldest first, kept bound: the same guest RAM
    /// imported again takes one back with no host kernel call
    /// ([`ImportCache`]).
    import_cache: ImportCache,
    /// The tokens of imported memory now freed.
    retired: VecDeque<NonZeroU64>,
    /// [`Loader::refusals`].
    refusals: Option<Refusals>,
    /// The count the driver raises by each refusal it keeps
    /// ([`Backend::count_failures`]); kept alive while the queue may.
    outstanding: Option<Arc<AtomicU32>>,
    /// Images still in `VK_IMAGE_LAYOUT_UNDEFINED`, with their aspects.
    new_images: Vec<(vk::Image, vk::ImageAspectFlags)>,
    /// What new resources take of memory used before, which the host or
    /// the next submission zeroes first.
    zeroing: zero::Zeroing,
    /// Orders sparse binds between command buffers: before a segment's
    /// binds the queue signals an odd value, and the binds signal the next.
    bind_timeline: vk::Semaphore,
    bind_value: u64,
    accumulate: vk::Pipeline,
    select: vk::Pipeline,
    resolve_draws: vk::Pipeline,
    resolve_dispatches: vk::Pipeline,
    store_texels: vk::Pipeline,
    load_texels: vk::Pipeline,
    average_samples: vk::Pipeline,
    decode_etc: vk::Pipeline,
    decode_pvrtc: vk::Pipeline,
    reorder_blocks: vk::Pipeline,
    decode_astc: vk::Pipeline,
    /// The cache compiled pipelines read and fill, kept across boots.
    store: pipeline_store::Persisted,
    /// Pipeline creations deferred outside the service's turn.
    deferral: deferral::Deferral,
    /// What jobs allocated outside the turn for new images.
    premade: premade::Premade,
    /// The platform's tick counter, and the phases of `create_image` and
    /// `submit` it timed since the device took them.
    ticks: fn() -> u64,
    split: [u64; 3],
}

impl NativeBackend {
    /// The generated-commands entry points, which the Metal layer uses only
    /// with the snapshot's `generated_commands` limits.
    fn generated(&self) -> &device_generated::Fns {
        self.fns
            .generated
            .as_ref()
            .expect("generated commands without the extension")
    }
}

/// Maps a Vulkan failure to the Metal layer's.
fn failure(result: vk::Result) -> Error {
    match result {
        vk::Result::ERROR_DEVICE_LOST => Error::DeviceLost,
        vk::Result::ERROR_OUT_OF_HOST_MEMORY
        | vk::Result::ERROR_OUT_OF_DEVICE_MEMORY
        | vk::Result::ERROR_TOO_MANY_OBJECTS
        | vk::Result::ERROR_FRAGMENTED_POOL
        | vk::Result::ERROR_OUT_OF_POOL_MEMORY => Error::OutOfMemory,
        vk::Result::PIPELINE_COMPILE_REQUIRED => Error::NotArchived,
        // The rest name the driver's result, so a refusal says what failed.
        vk::Result::ERROR_UNKNOWN => Error::Unsupported("Vulkan driver failure (VK_ERROR_UNKNOWN)"),
        vk::Result::ERROR_INITIALIZATION_FAILED => {
            Error::Unsupported("Vulkan initialization (VK_ERROR_INITIALIZATION_FAILED)")
        }
        vk::Result::ERROR_MEMORY_MAP_FAILED => {
            Error::Unsupported("Vulkan memory map (VK_ERROR_MEMORY_MAP_FAILED)")
        }
        vk::Result::ERROR_FEATURE_NOT_PRESENT => {
            Error::Unsupported("Vulkan feature (VK_ERROR_FEATURE_NOT_PRESENT)")
        }
        vk::Result::ERROR_FORMAT_NOT_SUPPORTED => {
            Error::Unsupported("Vulkan format (VK_ERROR_FORMAT_NOT_SUPPORTED)")
        }
        vk::Result::ERROR_INVALID_EXTERNAL_HANDLE => {
            Error::Unsupported("Vulkan external handle (VK_ERROR_INVALID_EXTERNAL_HANDLE)")
        }
        vk::Result::TIMEOUT => Error::Unsupported("Vulkan wait (VK_TIMEOUT)"),
        // The refusal names the raw result.
        _ => Error::Driver(result.as_raw()),
    }
}

// SAFETY: Vulkan's handles hold to no thread, and the fields behind pointers
// are this backend's alone.
unsafe impl Send for NativeBackend {}

impl NativeBackend {
    /// Creates the instance and device of the GPU at `pci`.
    ///
    /// # Safety
    ///
    /// `loader.get_instance_proc_addr` is a Vulkan driver's entry point, and
    /// `kernels` answers shader modules the compiler made for the device it
    /// is given.
    pub unsafe fn new(
        loader: Loader,
        pci: PciBusInfo,
        exporter: Box<dyn MemoryExporter>,
        kernels: impl FnOnce(vk::PhysicalDevice) -> Result<InternalKernels>,
    ) -> Result<Self> {
        let static_fn = ash::StaticFn {
            get_instance_proc_addr: loader.get_instance_proc_addr,
        };
        // SAFETY: the caller guarantees the entry point.
        let entry = unsafe { ash::Entry::from_static_fn(static_fn) };
        let application = vk::ApplicationInfo::default()
            .application_name(c"Stela Metal")
            .engine_name(c"Stela")
            .api_version(vk::API_VERSION_1_3);
        // SAFETY: the caller guarantees the entry point.
        let debug_utils =
            unsafe { entry.enumerate_instance_extension_properties(None) }.is_ok_and(|offered| {
                offered
                    .iter()
                    .any(|offer| offer.extension_name_as_c_str() == Ok(ext::debug_utils::NAME))
            });
        let names = [ext::debug_utils::NAME.as_ptr()];
        let info = vk::InstanceCreateInfo::default()
            .application_info(&application)
            .enabled_extension_names(if debug_utils { &names } else { &[] });
        // SAFETY: the create info and its application info outlive the call.
        let instance = unsafe { entry.create_instance(&info, None) }.map_err(failure)?;
        // SAFETY: the instance enables the extension when the driver offers it.
        let messenger = debug_utils
            .then(|| unsafe { Messenger::new(&entry, &instance, loader.log) })
            .flatten();
        // SAFETY: the instance was just created and nothing else uses it.
        let destroy_instance = |instance: &ash::Instance| unsafe {
            if let Some(messenger) = &messenger {
                messenger.destroy();
            }
            instance.destroy_instance(None);
        };
        let selected = Self::select(&instance, pci, loader.log);
        let Some((physical, extensions, snapshot, family)) = selected else {
            destroy_instance(&instance);
            return Err(Error::Unsupported("Vulkan device of the GPU"));
        };
        if let Some(missing) = requirements(&snapshot.features)
            .iter()
            .find(|requirement| !requirement.present)
        {
            destroy_instance(&instance);
            return Err(Error::Unsupported(missing.name));
        }
        if !snapshot.limits.dma_buf_import {
            destroy_instance(&instance);
            return Err(Error::Unsupported("VK_EXT_external_memory_dma_buf"));
        }

        let mut features = FeatureChain::new(&extensions, false);
        // SAFETY: `physical` came from this instance; the chain links only
        // structures of supported extensions.
        unsafe { instance.get_physical_device_features2(physical, &mut features.core) };
        features.for_enabling();
        let names = extensions.enabled();
        let priority = [1.0];
        let queues = [vk::DeviceQueueCreateInfo::default()
            .queue_family_index(family)
            .queue_priorities(&priority)];
        let mut info = vk::DeviceCreateInfo::default()
            .queue_create_infos(&queues)
            .enabled_extension_names(&names);
        info.p_next = core::ptr::addr_of!(features.core).cast();
        // SAFETY: every name and chained structure outlives the call.
        let device = match unsafe { instance.create_device(physical, &info, None) } {
            Ok(device) => device,
            Err(error) => {
                destroy_instance(&instance);
                return Err(failure(error));
            }
        };
        let has = |name| extensions.has(name);
        let fns = ExtensionFns {
            host_image_copy: has(ext::host_image_copy::NAME)
                .then(|| ext::host_image_copy::Device::new(&instance, &device)),
            dynamic_state3: has(ext::extended_dynamic_state3::NAME)
                .then(|| ext::extended_dynamic_state3::Device::new(&instance, &device)),
            sample_locations: has(ext::sample_locations::NAME)
                .then(|| ext::sample_locations::Device::new(&instance, &device)),
            depth_bias: has(ext::depth_bias_control::NAME)
                .then(|| ext::depth_bias_control::Device::new(&instance, &device)),
            mesh: has(ext::mesh_shader::NAME)
                .then(|| ext::mesh_shader::Device::new(&instance, &device)),
            calibrated: has(khr::calibrated_timestamps::NAME)
                .then(|| khr::calibrated_timestamps::Device::new(&instance, &device)),
            external_memory_fd: has(khr::external_memory_fd::NAME)
                .then(|| khr::external_memory_fd::Device::new(&instance, &device)),
            external_semaphore_fd: has(khr::external_semaphore_fd::NAME)
                .then(|| khr::external_semaphore_fd::Device::new(&instance, &device)),
            // A requirement: the device was refused without it.
            conditional: ext::conditional_rendering::Device::new(&instance, &device),
            generated: features
                .generated_commands()
                .then(|| device_generated::Fns::load(&instance, &device))
                .flatten(),
            executables: has(khr::pipeline_executable_properties::NAME).then(|| Executables {
                fns: khr::pipeline_executable_properties::Device::new(&instance, &device),
                log: loader.log,
            }),
        };
        let kernels = kernels(physical);
        if let Some(note) = small_bar(&snapshot.memory) {
            (loader.log)(&note);
        }
        let mut snapshot = snapshot;
        if fns.generated.is_none() {
            snapshot.limits.generated_commands = None;
        }
        // SAFETY: the device was created with one queue in `family`.
        let queue = unsafe { device.get_device_queue(family, 0) };
        if let Some(submit_thread) = loader.submit_thread {
            // SAFETY: the queue is the new device's and nothing submitted to
            // it yet. A submission the driver refuses still makes its
            // signals; its command buffer answers the failure
            // ([`Loader::refusals`]).
            let result = unsafe { submit_thread(queue) };
            if result != vk::Result::SUCCESS {
                (loader.log)(&alloc::format!(
                    "submissions stay on the committing processor: the driver's submit thread \
                     did not start ({result:?})"
                ));
            }
        }
        let memories = memory::Memories::new(snapshot.limits.buffer_image_granularity);
        let zeroing = zero::Zeroing::new(snapshot.limits.max_buffer_size, loader.log);
        let exporter: Arc<dyn MemoryExporter> = Arc::from(exporter);
        let outside = Arc::new(NativeOutside::new(
            device.clone(),
            exporter.clone(),
            fns.external_semaphore_fd.is_some(),
            loader.relax,
        ));
        let mut backend = Self {
            _entry: ManuallyDrop::new(entry),
            instance,
            messenger,
            device,
            fns,
            queue,
            snapshot,
            heap: Heap {
                layout: vk::DescriptorSetLayout::null(),
                pool: vk::DescriptorPool::null(),
                set: vk::DescriptorSet::null(),
            },
            null_sampler: vk::Sampler::null(),
            layout: vk::PipelineLayout::null(),
            command_pool: vk::CommandPool::null(),
            in_flight: VecDeque::new(),
            unsignaled: Vec::new(),
            mappings: BTreeMap::new(),
            memories,
            outside,
            exporter: ManuallyDrop::new(exporter),
            imports: BTreeMap::new(),
            import_cache: ImportCache::default(),
            retired: VecDeque::new(),
            refusals: loader.refusals,
            outstanding: None,
            new_images: Vec::new(),
            zeroing,
            bind_timeline: vk::Semaphore::null(),
            bind_value: 0,
            accumulate: vk::Pipeline::null(),
            select: vk::Pipeline::null(),
            resolve_draws: vk::Pipeline::null(),
            resolve_dispatches: vk::Pipeline::null(),
            store_texels: vk::Pipeline::null(),
            load_texels: vk::Pipeline::null(),
            average_samples: vk::Pipeline::null(),
            decode_etc: vk::Pipeline::null(),
            decode_pvrtc: vk::Pipeline::null(),
            reorder_blocks: vk::Pipeline::null(),
            decode_astc: vk::Pipeline::null(),
            store: pipeline_store::Persisted::none(loader.log),
            deferral: deferral::Deferral::default(),
            premade: premade::Premade::default(),
            ticks: super::timing::no_clock,
            split: [0; 3],
        };
        // Dropping a partly built backend destroys what was created.
        let kernels = kernels?;
        backend.create_layout(family)?;
        // SAFETY: the device is live and `snapshot` describes its GPU.
        backend.store = unsafe {
            pipeline_store::Persisted::open(&backend.device, &backend.snapshot.identity, loader.log)
        };
        backend.bind_timeline = handle(backend.create_semaphore(0)?.0);
        let kernel = |code| ComputePipelineInfo {
            code,
            required_subgroup_size: 0,
            full_subgroups: false,
            indirect_bindable: false,
            source: PipelineSource::Compiled,
        };
        backend.accumulate = backend.create_compute(&kernel(kernels.accumulate))?;
        backend.select = backend.create_compute(&kernel(kernels.select))?;
        backend.resolve_draws = backend.create_compute(&kernel(kernels.resolve_draws))?;
        backend.resolve_dispatches = backend.create_compute(&kernel(kernels.resolve_dispatches))?;
        backend.store_texels = backend.create_compute(&kernel(kernels.store_texels))?;
        backend.load_texels = backend.create_compute(&kernel(kernels.load_texels))?;
        backend.average_samples = backend.create_compute(&kernel(kernels.average_samples))?;
        backend.decode_etc = backend.create_compute(&kernel(kernels.decode_etc))?;
        backend.decode_pvrtc = backend.create_compute(&kernel(kernels.decode_pvrtc))?;
        backend.reorder_blocks = backend.create_compute(&kernel(kernels.reorder_blocks))?;
        backend.decode_astc = backend.create_compute(&kernel(kernels.decode_astc))?;
        Ok(backend)
    }

    /// The device at `pci` with a universal queue family, its supported
    /// extensions, its snapshot and the family. When none is, why each
    /// device the driver offers is not goes to `log`.
    fn select(
        instance: &ash::Instance,
        pci: PciBusInfo,
        log: fn(&str),
    ) -> Option<(vk::PhysicalDevice, Extensions, PhysicalDevice, u32)> {
        // SAFETY: the instance is valid.
        let devices = match unsafe { instance.enumerate_physical_devices() } {
            Ok(devices) => devices,
            Err(error) => {
                log(&format!(
                    "Vulkan physical devices not enumerated: {error:?}"
                ));
                return None;
            }
        };
        let mut rejected = Vec::new();
        let selected = devices.iter().enumerate().find_map(|(index, &physical)| {
            // SAFETY: `physical` came from this instance.
            let available =
                match unsafe { instance.enumerate_device_extension_properties(physical) } {
                    Ok(available) => available,
                    Err(error) => {
                        rejected.push(format!(
                            "Vulkan device {index}: extensions not enumerated: {error:?}"
                        ));
                        return None;
                    }
                };
            let extensions = Extensions::of(&available);
            let Some(snapshot) = physical::probe(instance, physical, &extensions) else {
                rejected.push(format!(
                    "Vulkan device {index}: a driver Stela does not admit"
                ));
                return None;
            };
            if snapshot.identity.pci_bus != Some(pci) {
                rejected.push(format!(
                    "Vulkan device {index}: at {:?}, not the GPU's {pci:?}",
                    snapshot.identity.pci_bus
                ));
                return None;
            }
            // SAFETY: as above.
            let families =
                unsafe { instance.get_physical_device_queue_family_properties(physical) };
            let universal = vk::QueueFlags::GRAPHICS | vk::QueueFlags::COMPUTE;
            let candidates = families.iter().enumerate().filter(|(_, family)| {
                family.queue_flags.contains(universal) && family.timestamp_valid_bits > 0
            });
            let sparse = |(_, family): &(usize, &vk::QueueFamilyProperties)| {
                family.queue_flags.contains(vk::QueueFlags::SPARSE_BINDING)
            };
            let Some((family, properties)) = candidates
                .clone()
                .find(sparse)
                .or_else(|| candidates.clone().next())
            else {
                rejected.push(format!(
                    "Vulkan device {index}: no graphics and compute queue family with timestamps \
                     among {}",
                    families.len()
                ));
                return None;
            };
            let mut snapshot = snapshot;
            if !properties
                .queue_flags
                .contains(vk::QueueFlags::SPARSE_BINDING)
            {
                // Sparse binds run on the universal queue.
                log(&format!(
                    "Vulkan device {index}: its universal queue family {family} binds no sparse \
                     memory, so this device has no sparse resources"
                ));
                let features = &mut snapshot.features;
                features.sparse_binding = false;
                features.sparse_residency_buffer = false;
                features.sparse_residency_image_2d = false;
                features.sparse_residency_image_3d = false;
                features.sparse_residency_samples = SampleCounts::EMPTY;
                features.sparse_residency_aliased = false;
            }
            Some((physical, extensions, snapshot, family as u32))
        });
        if selected.is_none() {
            log(&format!(
                "Vulkan offers {} physical devices, none the GPU's {pci:?}",
                devices.len()
            ));
            rejected.iter().for_each(|reason| log(reason));
        }
        selected
    }

    /// The heap's set layout, pool and set with its null descriptors, the
    /// pipeline layout and the command pool.
    fn create_layout(&mut self, family: u32) -> Result<()> {
        let counts = HeapLayout::new(&self.snapshot.limits).counts;
        let bindings = objects::heap_bindings(&counts);
        let flags = [vk::DescriptorBindingFlags::PARTIALLY_BOUND
            | vk::DescriptorBindingFlags::UPDATE_AFTER_BIND
            | vk::DescriptorBindingFlags::UPDATE_UNUSED_WHILE_PENDING; 5];
        let mut binding_flags =
            vk::DescriptorSetLayoutBindingFlagsCreateInfo::default().binding_flags(&flags);
        let info = vk::DescriptorSetLayoutCreateInfo::default()
            .flags(vk::DescriptorSetLayoutCreateFlags::UPDATE_AFTER_BIND_POOL)
            .bindings(&bindings)
            .push_next(&mut binding_flags);
        // SAFETY: the device is valid; the create info outlives the call.
        self.heap.layout =
            unsafe { self.device.create_descriptor_set_layout(&info, None) }.map_err(failure)?;
        let sizes: Vec<vk::DescriptorPoolSize> = bindings
            .iter()
            .map(|binding| vk::DescriptorPoolSize {
                ty: binding.descriptor_type,
                descriptor_count: binding.descriptor_count,
            })
            .collect();
        let info = vk::DescriptorPoolCreateInfo::default()
            .flags(vk::DescriptorPoolCreateFlags::UPDATE_AFTER_BIND)
            .max_sets(1)
            .pool_sizes(&sizes);
        // SAFETY: as above.
        self.heap.pool =
            unsafe { self.device.create_descriptor_pool(&info, None) }.map_err(failure)?;
        let layouts = [self.heap.layout];
        let info = vk::DescriptorSetAllocateInfo::default()
            .descriptor_pool(self.heap.pool)
            .set_layouts(&layouts);
        // SAFETY: the pool has room for its one set.
        self.heap.set = unsafe { self.device.allocate_descriptor_sets(&info) }.map_err(failure)?[0];
        let ranges = [vk::PushConstantRange {
            stage_flags: vk::ShaderStageFlags::ALL,
            offset: 0,
            size: PushConstants::BYTES,
        }];
        let info = vk::PipelineLayoutCreateInfo::default()
            .set_layouts(&layouts)
            .push_constant_ranges(&ranges);
        // SAFETY: as above.
        self.layout =
            unsafe { self.device.create_pipeline_layout(&info, None) }.map_err(failure)?;
        let info = vk::CommandPoolCreateInfo::default()
            .flags(vk::CommandPoolCreateFlags::TRANSIENT)
            .queue_family_index(family);
        // SAFETY: as above.
        self.command_pool =
            unsafe { self.device.create_command_pool(&info, None) }.map_err(failure)?;
        let info = vk::SamplerCreateInfo::default();
        // SAFETY: as above.
        self.null_sampler = unsafe { self.device.create_sampler(&info, None) }.map_err(failure)?;
        let nulls = [
            Descriptor::SampledImage(ImageView(0)),
            Descriptor::StorageImage(ImageView(0)),
            Descriptor::Sampler(Sampler(raw(self.null_sampler))),
            Descriptor::UniformTexelBuffer(BufferView(0)),
            Descriptor::StorageTexelBuffer(BufferView(0)),
        ];
        for descriptor in nulls {
            self.write_descriptor(NULL_DESCRIPTOR, descriptor);
        }
        debug_assert_eq!(HEAP_SET, 0);
        Ok(())
    }

    /// Allocates memory of its own of a type `request` allows among
    /// `types`, trying the preferred properties first. The memory is
    /// zeroed: a heap's resources are not, but none may read what another
    /// client left in the memory before.
    fn allocate(&mut self, size: u64, request: MemoryRequest, types: u32) -> Result<Memory> {
        let candidates = objects::memory_candidates(&self.snapshot.memory, types, request);
        ensure(!candidates.is_empty(), "no memory type suits the resource")?;
        let mut source = memory::Allocator {
            device: &self.device,
            types: &self.snapshot.memory.types,
            mappings: &mut self.mappings,
            zeroing: &mut self.zeroing,
            mapped: request.mapped,
            dedicated: None,
        };
        let mut last = vk::Result::ERROR_OUT_OF_DEVICE_MEMORY;
        for index in candidates {
            match source.allocate(index, size, Purpose::Own) {
                Ok(device_memory) => {
                    let memory = self.memories.insert(memory::Allocation::Whole {
                        memory: device_memory,
                        resource: false,
                    });
                    if let Err(error) = self.zero_range(device_memory, 0, size) {
                        self.free_memory(memory);
                        return Err(error);
                    }
                    return Ok(memory);
                }
                Err(error) => last = error,
            }
        }
        Err(failure(last))
    }

    /// While the call may defer, leaves the job that allocates what an image
    /// of `needs` would allocate in the turn, a new kernel buffer object:
    /// memory of its own, made with the image, or a new block
    /// ([`premade`]). False when it allocates nothing new, when its key's
    /// job failed, or when the call may not defer.
    fn defer_image_memory(
        &mut self,
        key: [u8; 32],
        info: &ImageInfo,
        request: MemoryRequest,
        requirements: vk::MemoryRequirements,
        resource: Resource,
    ) -> bool {
        if !self.deferral.deferring() {
            return false;
        }
        let candidates = self.candidates(requirements, request);
        let needs = Needs {
            size: requirements.size,
            alignment: requirements.alignment.max(resource.alignment),
            tiling: resource.tiling,
            own: resource.own,
        };
        let gone = self.deferral.gone();
        let job = match self
            .memories
            .blocks
            .growth(needs, request.mapped, &candidates)
        {
            blocks::Growth::None => return false,
            // The call, deferred already, runs again: nothing it makes stays.
            _ if self.deferral.deferred() => return true,
            blocks::Growth::Own(_) if self.premade.failed(&key) => return false,
            blocks::Growth::Own(_) => premade::image_job(
                &self.device,
                gone,
                key,
                info.clone(),
                candidates
                    .iter()
                    .map(|candidate| candidate.memory_type)
                    .collect(),
                request.mapped,
            ),
            blocks::Growth::Block(candidate) => {
                if self
                    .premade
                    .failed(&premade::block_key(candidate.memory_type, request.mapped))
                {
                    return false;
                }
                premade::block_job(
                    &self.device,
                    gone,
                    candidate.memory_type,
                    candidate.block_bytes,
                    request.mapped,
                )
            }
        };
        self.deferral.defer(job)
    }

    /// Creates the image a job made for this call ([`premade`]): its memory
    /// is its own, bound already.
    fn premade_image(
        &mut self,
        made: premade::Image,
        info: &ImageInfo,
        started: u64,
    ) -> Result<(Image, Memory, u64)> {
        let image = made.image;
        if let Some(pointer) = made.pointer {
            let mapping = memory::mapping(
                &self.snapshot.memory.types,
                made.memory_type,
                pointer,
                made.size,
            );
            self.mappings.insert(raw(made.memory), mapping);
        }
        self.memories.blocks.adopt_own();
        let memory = self.memories.insert(memory::Allocation::Whole {
            memory: made.memory,
            resource: true,
        });
        let adopted = (self.ticks)();
        let result = self.adopt_image(image, info);
        self.time_split([started, adopted, adopted, (self.ticks)()]);
        self.or_destroy(
            result,
            &[Object::Image(Image(raw(image))), Object::Memory(memory)],
        )?;
        self.zeroing
            .pending
            .images
            .insert(raw(image), zero::Unzeroed::new(info));
        Ok((Image(raw(image)), memory, made.size))
    }

    /// Keeps what a memory job made: a block joins its pool; an image waits
    /// for its call's next run.
    fn adopt_memory(&mut self, key: [u8; 32], mut making: premade::Making) {
        // A job of a device this backend replaced destroys nothing: that
        // device is gone.
        if !Arc::ptr_eq(&making.gone, &self.deferral.gone()) {
            return;
        }
        let (block, evicted) = self.premade.keep(key, making.made.take());
        if let Some(evicted) = evicted {
            premade::destroy(&self.device, premade::Kind::Image(evicted));
        }
        let Some(block) = block else {
            return;
        };
        match block.pointer {
            Some(pointer) => {
                let mapping = memory::mapping(
                    &self.snapshot.memory.types,
                    block.memory_type,
                    pointer,
                    block.size,
                );
                self.mappings.insert(raw(block.memory), mapping);
            }
            None => self.zeroing.cover(
                &self.device,
                block.memory,
                block.memory_type,
                memory::allocation_bytes(block.size, true),
            ),
        }
        self.memories
            .blocks
            .adopt_block(block.memory_type, block.mapped, block.memory, block.size);
    }

    /// The memory types a resource of `requirements` may take for
    /// `request`, in order of preference, with their block sizes.
    fn candidates(
        &self,
        requirements: vk::MemoryRequirements,
        request: MemoryRequest,
    ) -> Vec<Candidate> {
        let memory = &self.snapshot.memory;
        let atom = self.snapshot.limits.non_coherent_atom_size.max(1);
        objects::memory_candidates(memory, requirements.memory_type_bits, request)
            .into_iter()
            .map(|index| {
                let properties = memory.types[index as usize].properties;
                let heap = memory
                    .heaps
                    .get(memory.types[index as usize].heap_index as usize)
                    .map_or(0, |heap| heap.size);
                let device_local =
                    properties.contains(super::vulkan::MemoryProperties::DEVICE_LOCAL);
                let coherent = properties.contains(super::vulkan::MemoryProperties::HOST_COHERENT);
                Candidate {
                    memory_type: index,
                    block_bytes: blocks::block_bytes(heap, device_local),
                    atom: if request.mapped && !coherent { atom } else { 1 },
                }
            })
            .collect()
    }

    /// Memory for a buffer or image of `requirements`: a place in a block,
    /// or memory of its own, dedicated to the resource `dedicated` names if
    /// any, when `own` (the driver requires or prefers that), when it is
    /// large, or when no block can be allocated. Answers the memory, the
    /// `VkDeviceMemory` and offset to bind at, and the bytes it spans there.
    fn allocate_resource(
        &mut self,
        requirements: vk::MemoryRequirements,
        resource: Resource,
        request: MemoryRequest,
        dedicated: Option<vk::MemoryDedicatedAllocateInfo<'static>>,
    ) -> Result<(Memory, vk::DeviceMemory, u64, u64)> {
        let candidates = self.candidates(requirements, request);
        let needs = Needs {
            size: requirements.size,
            alignment: requirements.alignment.max(resource.alignment),
            tiling: resource.tiling,
            own: resource.own,
        };
        let memory = &self.snapshot.memory;
        let mut source = memory::Allocator {
            device: &self.device,
            types: &memory.types,
            mappings: &mut self.mappings,
            zeroing: &mut self.zeroing,
            mapped: request.mapped,
            dedicated,
        };
        let resident =
            self.memories
                .blocks
                .allocate(&mut source, needs, request.mapped, &candidates);
        match resident {
            Ok(Resident::Placed(place)) => {
                let (block, _) = self
                    .memories
                    .blocks
                    .memory(place.block)
                    .ok_or(Error::Invalid("a place outside the blocks"))?;
                let memory = self.memories.insert(memory::Allocation::Placed(place));
                Ok((memory, block, place.offset, place.size))
            }
            Ok(Resident::Own(own)) => {
                let memory = self.memories.insert(memory::Allocation::Whole {
                    memory: own,
                    resource: true,
                });
                Ok((memory, own, 0, requirements.size))
            }
            Err(Some(error)) => Err(failure(error)),
            Err(None) => Err(Error::Invalid("no memory type suits the resource")),
        }
    }

    /// The memory type an import of the dma-buf `fd` takes: the first of
    /// NVK's types for it that prefers coherent memory.
    fn import_memory_type(&self, fd: i32) -> Result<u32> {
        let bits = self
            .fns
            .external_memory_fd
            .as_ref()
            .ok_or(Error::Unsupported("memory import"))
            .and_then(|fns| {
                let mut properties = vk::MemoryFdPropertiesKHR::default();
                // SAFETY: the descriptor is the dma-buf just exported.
                unsafe { fns.get_memory_fd_properties(DMA_BUF, fd, &mut properties) }
                    .map_err(failure)?;
                Ok(properties.memory_type_bits)
            })?;
        let request = MemoryRequest {
            required: super::vulkan::MemoryProperties::EMPTY,
            preferred: super::vulkan::MemoryProperties::HOST_COHERENT,
            mapped: false,
        };
        objects::memory_candidates(&self.snapshot.memory, bits, request)
            .first()
            .copied()
            .ok_or(Error::Unsupported("a memory type of imported memory"))
    }

    /// Takes an imported `VkDeviceMemory` as [`Memory`]: the host reaches it
    /// through the export's mapping (Linux has no pages to map for an
    /// imported buffer), and freeing it retires `token`.
    fn own_import(
        &mut self,
        memory: vk::DeviceMemory,
        exported: ExportedMemory,
        token: NonZeroU64,
        ranges: Vec<MemoryRange>,
    ) -> Memory {
        self.mappings.insert(
            raw(memory),
            Mapping {
                pointer: exported.pointer,
                size: exported.bytes,
                coherent: true,
            },
        );
        self.imports.insert(raw(memory), (exported, token, ranges));
        self.memories.insert(memory::Allocation::Whole {
            memory,
            resource: false,
        })
    }

    /// Frees `memory`: returns a place to its block, or frees a whole
    /// allocation. A freed import retires its token and is kept for the
    /// same guest RAM's next import ([`ImportCache`]).
    fn free_memory(&mut self, memory: Memory) {
        let Some(allocation) = self.memories.remove(memory) else {
            return;
        };
        if let memory::Allocation::Whole { memory, resource } = allocation
            && let Some((exported, token, ranges)) = self.imports.remove(&raw(memory))
        {
            if resource {
                self.memories.blocks.release_own();
            }
            self.mappings.remove(&raw(memory));
            self.zeroing.forget(&self.device, memory);
            self.retired.push_back(token);
            for evicted in self.import_cache.keep(ranges, memory, exported) {
                self.free_after_turn(Freed {
                    memory: evicted.memory,
                    exported: Some(evicted.exported),
                });
            }
            return;
        }
        if let memory::Allocation::Whole { memory, resource } = allocation {
            if resource {
                self.memories.blocks.release_own();
            }
            self.mappings.remove(&raw(memory));
            self.zeroing.forget(&self.device, memory);
            self.free_after_turn(Freed {
                memory,
                exported: None,
            });
            return;
        }
        let mut source = memory::Allocator {
            device: &self.device,
            types: &self.snapshot.memory.types,
            mappings: &mut self.mappings,
            zeroing: &mut self.zeroing,
            mapped: false,
            dedicated: None,
        };
        match allocation {
            memory::Allocation::Placed(place) => {
                self.memories.blocks.release(&mut source, place);
            }
            memory::Allocation::Whole { .. } => unreachable!("freed above"),
        }
    }

    /// Frees a whole allocation the Metal device freed, so nothing the GPU
    /// may still run uses it: after the turn when the outside work takes
    /// it, as the free's host kernel calls wait for the boot processor,
    /// else now.
    fn free_after_turn(&mut self, freed: Freed) {
        if let Some(freed) = self.outside.defer_free(freed) {
            // SAFETY: as above; nothing else holds it.
            unsafe { self.device.free_memory(freed.memory, None) };
            if let Some(exported) = freed.exported {
                self.exporter.unmap(exported);
            }
        }
    }

    /// The `VkDeviceMemory` behind `memory` and the offset of its first byte.
    fn resolve(&self, memory: Memory) -> Result<memory::Resolved> {
        self.memories
            .resolve(memory)
            .ok_or(Error::Invalid("memory that was freed"))
    }

    /// The host pointer to `offset..offset + size` of `memory`.
    fn mapping(&self, memory: Memory, offset: u64, size: u64) -> Result<Mapped> {
        let resolved = self.resolve(memory)?;
        let mapping = self
            .mappings
            .get(&raw(resolved.memory))
            .copied()
            .ok_or(Error::Invalid("memory is not mapped"))?;
        let limit = resolved.size.unwrap_or(mapping.size);
        ensure(
            offset.checked_add(size).is_some_and(|end| end <= limit),
            "range outside the mapped memory",
        )?;
        let at = resolved.offset + offset;
        Ok(Mapped {
            // SAFETY: a place lies within its block's mapping and `at + size`
            // within the place, or within the whole mapping.
            pointer: unsafe { mapping.pointer.add(at as usize) },
            memory: resolved.memory,
            offset: at,
            coherent: mapping.coherent,
        })
    }

    /// Zeroes `size` bytes at `offset` of `memory`, which a new resource
    /// takes: mapped memory at once, other memory at the start of the next
    /// submission. Zeroing at hand-out rather than at free covers every
    /// source alike (reused places, new blocks and VRAM the kernel does not
    /// clear) and never races a read of the freed resource.
    fn zero_range(&mut self, memory: vk::DeviceMemory, offset: u64, size: u64) -> Result<()> {
        let Some(mapping) = self.mappings.get(&raw(memory)).copied() else {
            self.zeroing.fill(memory, offset, size);
            return Ok(());
        };
        ensure(
            offset
                .checked_add(size)
                .is_some_and(|end| end <= mapping.size),
            "range outside the mapped memory",
        )?;
        // SAFETY: the range lies in the mapping, and only the new resource,
        // which nothing uses yet, owns it.
        unsafe {
            mapping
                .pointer
                .add(offset as usize)
                .as_ptr()
                .write_bytes(0, size as usize);
        }
        if mapping.coherent {
            Ok(())
        } else {
            self.flush_range(memory, offset, size)
        }
    }

    /// What a submission starting with `commands` zeroes first: `pending`'s
    /// fills, and clears of the image subresources its leading commands do
    /// not overwrite whole before anything reads them.
    fn zero_work(&self, pending: &zero::Pending, commands: &[Command]) -> record::Zero {
        let fills = self.zeroing.buffer_fills(pending);
        if pending.images.is_empty() {
            return record::Zero {
                fills,
                clears: Vec::new(),
            };
        }
        let mut images = pending.images.clone();
        zero::first_writes(commands, &self.zeroing.views, &mut images);
        let clears = images
            .iter()
            .filter_map(|(&image, entry)| {
                let aspects = objects::aspects(entry.aspects);
                let ranges: Vec<_> = if entry.all_stale() {
                    alloc::vec![objects::whole(aspects)]
                } else {
                    entry
                        .stale_runs()
                        .into_iter()
                        .map(|run| vk::ImageSubresourceRange {
                            aspect_mask: aspects,
                            base_mip_level: run.level,
                            level_count: 1,
                            base_array_layer: run.layers.start,
                            layer_count: run.layers.end - run.layers.start,
                        })
                        .collect()
                };
                (!ranges.is_empty()).then(|| record::Clear {
                    image: handle(image),
                    aspects,
                    ranges,
                })
            })
            .collect();
        record::Zero { fills, clears }
    }

    /// Zeroes on the host the subresources of `sub` in a host-transfer
    /// image that nothing wrote whole yet, before a host copy reaches them.
    fn zero_on_host(&mut self, image: Image, sub: Subresource) -> Result<()> {
        let Some(entry) = self.zeroing.pending.images.get(&image.0) else {
            return Ok(());
        };
        let stale = entry.stale_in(sub);
        if stale.is_empty() {
            return Ok(());
        }
        let aspects = entry.aspects;
        let extents: Vec<_> = stale
            .iter()
            .map(|&(level, _)| entry.level_extent(level))
            .collect();
        let copy = self
            .fns
            .host_image_copy
            .as_ref()
            .ok_or(Error::Unsupported("host image copies"))?;
        let image_handle: vk::Image = handle(image.0);
        let mut zeros = Vec::new();
        for (&(level, layer), extent) in stale.iter().zip(extents) {
            for aspect in [
                super::vulkan::ImageAspects::COLOR,
                super::vulkan::ImageAspects::DEPTH,
                super::vulkan::ImageAspects::STENCIL,
            ] {
                if !aspects.contains(aspect) {
                    continue;
                }
                let aspect = objects::aspects(aspect);
                let subresource =
                    vk::ImageSubresource2KHR::default().image_subresource(vk::ImageSubresource {
                        aspect_mask: aspect,
                        mip_level: level,
                        array_layer: layer,
                    });
                let mut bytes = vk::SubresourceHostMemcpySizeEXT::default();
                let mut layout = vk::SubresourceLayout2KHR::default().push_next(&mut bytes);
                // SAFETY: the image allows host transfers; the chain outlives
                // the call.
                unsafe {
                    copy.get_image_subresource_layout2(image_handle, &subresource, &mut layout)
                };
                zeros.resize(zeros.len().max(bytes.size as usize), 0);
                let regions = [vk::MemoryToImageCopyEXT::default()
                    .host_pointer(zeros.as_ptr().cast())
                    .image_subresource(vk::ImageSubresourceLayers {
                        aspect_mask: aspect,
                        mip_level: level,
                        base_array_layer: layer,
                        layer_count: 1,
                    })
                    .image_extent(vk::Extent3D {
                        width: extent[0],
                        height: extent[1],
                        depth: extent[2],
                    })];
                let info = vk::CopyMemoryToImageInfoEXT::default()
                    .flags(vk::HostImageCopyFlagsEXT::MEMCPY)
                    .dst_image(image_handle)
                    .dst_image_layout(vk::ImageLayout::GENERAL)
                    .regions(&regions);
                // SAFETY: `zeros` holds the subresource's memcpy size; the
                // region is the whole subresource, as a memcpy needs.
                unsafe { copy.copy_memory_to_image(&info) }.map_err(failure)?;
            }
            if let Some(entry) = self.zeroing.pending.images.get_mut(&image.0) {
                entry.written(level, layer);
            }
        }
        Ok(())
    }

    /// The flush or invalidate range of `offset..offset + size` in a
    /// non-coherent mapping, widened to the device's atom size.
    fn atom_range(
        &self,
        memory: vk::DeviceMemory,
        offset: u64,
        size: u64,
    ) -> vk::MappedMemoryRange<'_> {
        let atom = self.snapshot.limits.non_coherent_atom_size.max(1);
        let start = offset / atom * atom;
        let end = (offset + size).next_multiple_of(atom);
        vk::MappedMemoryRange::default()
            .memory(memory)
            .offset(start)
            .size(end - start)
    }

    fn flush_range(&self, memory: vk::DeviceMemory, offset: u64, size: u64) -> Result<()> {
        let range = self.atom_range(memory, offset, size);
        // SAFETY: the memory is mapped; the range is atom aligned.
        unsafe { self.device.flush_mapped_memory_ranges(&[range]) }.map_err(failure)
    }

    fn invalidate_range(&self, memory: vk::DeviceMemory, offset: u64, size: u64) -> Result<()> {
        let range = self.atom_range(memory, offset, size);
        // SAFETY: as in `flush_range`.
        unsafe { self.device.invalidate_mapped_memory_ranges(&[range]) }.map_err(failure)
    }

    /// [`image_requirements`] of `image`.
    fn image_needs(&self, image: vk::Image) -> (vk::MemoryRequirements, bool) {
        image_requirements(&self.device, image)
    }

    /// As [`Self::image_needs`], of a buffer.
    fn buffer_needs(&self, buffer: vk::Buffer) -> (vk::MemoryRequirements, bool) {
        let info = vk::BufferMemoryRequirementsInfo2::default().buffer(buffer);
        let mut dedicated = vk::MemoryDedicatedRequirements::default();
        let mut requirements = vk::MemoryRequirements2::default().push_next(&mut dedicated);
        // SAFETY: the buffer is live and the chain outlives the call.
        unsafe {
            self.device
                .get_buffer_memory_requirements2(&info, &mut requirements)
        };
        let needs = requirements.memory_requirements;
        (needs, own(&dedicated))
    }

    /// Moves a new image to `GENERAL`: on the host when it allows host
    /// transfers, otherwise at the start of the next submission.
    fn adopt_image(&mut self, image: vk::Image, info: &ImageInfo) -> Result<()> {
        let aspects = objects::aspects(info.format.aspects());
        let host = info
            .usage
            .contains(super::vulkan::ImageUsage::HOST_TRANSFER);
        match (&self.fns.host_image_copy, host) {
            (Some(copy), true) => {
                let transition = [vk::HostImageLayoutTransitionInfoEXT::default()
                    .image(image)
                    .old_layout(vk::ImageLayout::UNDEFINED)
                    .new_layout(vk::ImageLayout::GENERAL)
                    .subresource_range(objects::whole(aspects))];
                // SAFETY: the image was created with host transfer usage and
                // no command uses it yet.
                unsafe { copy.transition_image_layout(&transition) }.map_err(failure)
            }
            _ => {
                self.new_images.push((image, aspects));
                Ok(())
            }
        }
    }

    fn create_compute(&mut self, info: &ComputePipelineInfo) -> Result<vk::Pipeline> {
        if let Some(created) = self.deferred_creation(deferral::Wanted::Compute(info)) {
            let pipeline = created?;
            self.report_executables(pipeline);
            return Ok(pipeline);
        }
        let pipeline = self.through_store(info.source, |backend, source, feedback| {
            let info = ComputePipelineInfo { source, ..*info };
            objects::compute_pipeline(&info, backend.layout, |create| {
                let mut chained = vk::PipelineCreationFeedbackCreateInfo::default()
                    .pipeline_creation_feedback(feedback);
                let create = (*create).push_next(&mut chained);
                // SAFETY: the create info, its chain, its stage and the NIR
                // outlive the call.
                unsafe {
                    backend
                        .device
                        .create_compute_pipelines(pipeline_cache(source), &[create], None)
                }
                .map(|pipelines| pipelines[0])
                .map_err(|(_, error)| failure(error))
            })
        })?;
        self.report_executables(pipeline);
        Ok(pipeline)
    }

    /// Creates a pipeline of `source` with `create`, which builds it from
    /// the source it is given and fills the feedback. A compiled pipeline
    /// goes through the pipeline store's cache: found there or added, and
    /// once the store is full, found there or compiled without it.
    fn through_store<T>(
        &mut self,
        source: PipelineSource,
        mut create: impl FnMut(
            &mut Self,
            PipelineSource,
            &mut vk::PipelineCreationFeedback,
        ) -> Result<T>,
    ) -> Result<T> {
        let mut feedback = vk::PipelineCreationFeedback::default();
        let (PipelineSource::Compiled, Some((cache, grows))) = (source, self.store.cache()) else {
            return create(self, source, &mut feedback);
        };
        let cache = PipelineCache(raw(cache));
        let hit = |feedback: &vk::PipelineCreationFeedback| {
            feedback.flags.contains(
                vk::PipelineCreationFeedbackFlags::VALID
                    | vk::PipelineCreationFeedbackFlags::APPLICATION_PIPELINE_CACHE_HIT,
            )
        };
        if !grows {
            return match create(self, PipelineSource::Archived(cache), &mut feedback) {
                Err(Error::NotArchived) => {
                    let created = create(self, PipelineSource::Compiled, &mut feedback)?;
                    pipeline_store::note_lookup(false, feedback.duration);
                    Ok(created)
                }
                created => {
                    let created = created?;
                    pipeline_store::note_lookup(true, feedback.duration);
                    Ok(created)
                }
            };
        }
        let created = create(self, PipelineSource::Cached(cache), &mut feedback)?;
        let hit = hit(&feedback);
        pipeline_store::note_lookup(hit, feedback.duration);
        self.store.created(&self.device, hit);
        Ok(created)
    }

    /// Logs each of `pipeline`'s shaders that spills or runs past
    /// [`LARGE_SHADER`] instructions, with NVK's statistics for it.
    fn report_executables(&self, pipeline: vk::Pipeline) {
        /// Instructions past which a shader is reported.
        const LARGE_SHADER: u64 = 16384;
        let Some(Executables { fns, log }) = &self.fns.executables else {
            return;
        };
        let info = vk::PipelineInfoKHR::default().pipeline(pipeline);
        // SAFETY: the pipeline was just created on this device.
        let Ok(executables) = (unsafe { fns.get_pipeline_executable_properties(&info) }) else {
            return;
        };
        for (index, executable) in executables.iter().enumerate() {
            let info = vk::PipelineExecutableInfoKHR::default()
                .pipeline(pipeline)
                .executable_index(index as u32);
            // SAFETY: the executable is one the pipeline listed.
            let Ok(statistics) = (unsafe { fns.get_pipeline_executable_statistics(&info) }) else {
                continue;
            };
            let value = |statistic: &vk::PipelineExecutableStatisticKHR| match statistic.format {
                vk::PipelineExecutableStatisticFormatKHR::UINT64 => {
                    // SAFETY: the format names the union's member.
                    unsafe { statistic.value.u64 }
                }
                vk::PipelineExecutableStatisticFormatKHR::INT64 => {
                    // SAFETY: as above.
                    unsafe { statistic.value.i64 }.max(0) as u64
                }
                _ => 0,
            };
            let named = |name: &str| {
                statistics
                    .iter()
                    .find(|statistic| {
                        statistic
                            .name_as_c_str()
                            .is_ok_and(|n| n.to_bytes() == name.as_bytes())
                    })
                    .map_or(0, value)
            };
            let instructions = named("Instruction count");
            let spills = named("Spills to memory");
            let fills = named("Fills from memory");
            if instructions <= LARGE_SHADER && spills == 0 && fills == 0 {
                continue;
            }
            let mut line = format!(
                "shader {}:",
                executable
                    .name_as_c_str()
                    .map_or("?", |name| name.to_str().unwrap_or("?"))
            );
            for statistic in &statistics {
                let name = statistic
                    .name_as_c_str()
                    .map_or("?", |name| name.to_str().unwrap_or("?"));
                line.push_str(&format!(" {name}={}", value(statistic)));
            }
            log(&line);
        }
    }

    /// Passes `result` through, destroying `objects` when it failed so a
    /// failed creation leaks nothing.
    fn or_destroy<T>(&mut self, result: Result<T>, objects: &[Object]) -> Result<T> {
        if result.is_err() {
            for &object in objects {
                self.destroy(object);
            }
        }
        result
    }

    /// Frees the command buffers of submissions the queue has finished.
    fn reclaim(&mut self) -> Result<()> {
        // Leave the queue intact if observing a completion fails.
        let ready = InFlight::completed_prefix(&self.in_flight, |timeline| {
            // SAFETY: the timeline semaphore is alive while submissions use it.
            unsafe { self.device.get_semaphore_counter_value(timeline) }.map_err(failure)
        })?;
        self.free_completed(ready);
        Ok(())
    }

    fn free_completed(&mut self, ready: usize) {
        if ready == 0 {
            return;
        }
        let buffers = if ready == 1 {
            self.in_flight
                .pop_front()
                .expect("a ready submission")
                .buffers
        } else {
            let mut completed = self.in_flight.drain(..ready);
            let first = completed.next().expect("a ready submission").buffers;
            let mut buffers = match first {
                CommandBuffers::One(buffer) => {
                    let mut buffers = Vec::with_capacity(ready);
                    buffers.push(buffer);
                    buffers
                }
                CommandBuffers::Many(buffers) => buffers,
            };
            for entry in completed {
                buffers.extend_from_slice(entry.buffers.as_slice());
            }
            CommandBuffers::Many(buffers)
        };
        // SAFETY: the queue finished every command buffer in `buffers`.
        unsafe {
            self.device
                .free_command_buffers(self.command_pool, buffers.as_slice())
        };
    }
}

impl Drop for NativeBackend {
    fn drop(&mut self) {
        // A deferred pipeline job uses the device outside any turn, and so
        // does a call's import after its turn.
        self.deferral.retire();
        let deferred = self.outside.close();
        // An error is not evidence that queued DMA stopped. Keep every native
        // handle, the driver's code and the exporter on that uncertain path.
        // SAFETY: the device belongs to this backend and no caller uses it now.
        if unsafe { self.device.device_wait_idle() }.is_err() {
            return;
        }
        self.store.idle(&self.device);
        // Images jobs made that no call took; the device is idle.
        for image in self.premade.drain() {
            premade::destroy(&self.device, premade::Kind::Image(image));
        }
        let mut source = memory::Allocator {
            device: &self.device,
            types: &self.snapshot.memory.types,
            mappings: &mut self.mappings,
            zeroing: &mut self.zeroing,
            mapped: false,
            dedicated: None,
        };
        self.memories.blocks.release_all(&mut source);
        self.zeroing.destroy(&self.device);
        let kept = self.import_cache.drain().map(|kept| Freed {
            memory: kept.memory,
            exported: Some(kept.exported),
        });
        let Upkeep {
            frees,
            spares,
            doomed,
        } = deferred;
        for semaphore in spares.into_iter().chain(doomed) {
            // SAFETY: the device is idle; spares were never used, and doomed
            // ones were released by the Metal device.
            unsafe { self.device.destroy_semaphore(semaphore, None) };
        }
        for freed in kept.collect::<Vec<_>>().into_iter().chain(frees) {
            // SAFETY: the device is idle and the Metal device freed it.
            unsafe { self.device.free_memory(freed.memory, None) };
            if let Some(exported) = freed.exported {
                self.exporter.unmap(exported);
            }
        }
        // SAFETY: after the device idles, nothing uses any object, and each
        // handle is destroyed once; null handles are ignored by Vulkan.
        unsafe {
            self.store.destroy(&self.device);
            self.deferral.destroy(&self.device);
            self.device.destroy_pipeline(self.accumulate, None);
            self.device.destroy_pipeline(self.resolve_draws, None);
            self.device.destroy_pipeline(self.resolve_dispatches, None);
            self.device.destroy_pipeline(self.select, None);
            self.device.destroy_pipeline(self.store_texels, None);
            self.device.destroy_pipeline(self.load_texels, None);
            self.device.destroy_pipeline(self.average_samples, None);
            self.device.destroy_pipeline(self.decode_etc, None);
            self.device.destroy_pipeline(self.decode_pvrtc, None);
            self.device.destroy_pipeline(self.reorder_blocks, None);
            self.device.destroy_pipeline(self.decode_astc, None);
            self.device.destroy_command_pool(self.command_pool, None);
            self.device.destroy_pipeline_layout(self.layout, None);
            self.device.destroy_sampler(self.null_sampler, None);
            self.device.destroy_semaphore(self.bind_timeline, None);
            self.device.destroy_descriptor_pool(self.heap.pool, None);
            self.device
                .destroy_descriptor_set_layout(self.heap.layout, None);
            self.device.destroy_device(None);
            if let Some(messenger) = &self.messenger {
                messenger.destroy();
            }
            self.instance.destroy_instance(None);
            ManuallyDrop::drop(&mut self.exporter);
            ManuallyDrop::drop(&mut self._entry);
        }
    }
}

/// Imports freed lately, oldest first, each still bound with its host
/// mapping: an import views guest RAM by address, so a freed import of the
/// same ranges views the same pages, and taking it back skips the export,
/// the PRIME import, the VA allocation and both binds (an IOSurface the
/// guest imports again each frame). What it holds is guest RAM the exporter
/// already maps; nothing reads it while kept.
#[derive(Default)]
struct ImportCache {
    kept: VecDeque<KeptImport>,
    bytes: u64,
}

struct KeptImport {
    ranges: Vec<MemoryRange>,
    memory: vk::DeviceMemory,
    exported: ExportedMemory,
}

impl ImportCache {
    /// Imports kept at most, and the guest RAM they view at most.
    const COUNT: usize = 32;
    const BYTES: u64 = 512 << 20;

    /// Keeps a freed import and answers the oldest ones past the bounds,
    /// for the caller to free.
    fn keep(
        &mut self,
        ranges: Vec<MemoryRange>,
        memory: vk::DeviceMemory,
        exported: ExportedMemory,
    ) -> Vec<KeptImport> {
        self.bytes += exported.bytes;
        self.kept.push_back(KeptImport {
            ranges,
            memory,
            exported,
        });
        let mut evicted = Vec::new();
        while self.kept.len() > Self::COUNT || self.bytes > Self::BYTES {
            let Some(oldest) = self.kept.pop_front() else {
                break;
            };
            self.bytes -= oldest.exported.bytes;
            evicted.push(oldest);
        }
        evicted
    }

    /// Whether it keeps an import of exactly `ranges`.
    fn holds(&self, ranges: &[MemoryRange]) -> bool {
        self.kept.iter().any(|kept| kept.ranges == ranges)
    }

    /// Takes back the kept import of exactly `ranges`.
    fn take(&mut self, ranges: &[MemoryRange]) -> Option<KeptImport> {
        let index = self.kept.iter().position(|kept| kept.ranges == ranges)?;
        let kept = self.kept.remove(index)?;
        self.bytes -= kept.exported.bytes;
        Some(kept)
    }

    fn drain(&mut self) -> impl Iterator<Item = KeptImport> + '_ {
        self.bytes = 0;
        self.kept.drain(..)
    }
}

/// An import [`NativeOutside::prepare_import`] made, for
/// [`NativeBackend::adopt_import`].
struct MadeImport {
    memory: vk::DeviceMemory,
    exported: ExportedMemory,
    ranges: Vec<MemoryRange>,
}

// SAFETY: the export's pointer is the host's mapping of guest RAM, which
// any processor may hold; only the backend that adopts it, or the exporter
// when it is discarded, uses it.
unsafe impl Send for MadeImport {}
/// A whole allocation to free after the turn, and the export it viewed.
struct Freed {
    memory: vk::DeviceMemory,
    exported: Option<ExportedMemory>,
}

// SAFETY: as for `MadeImport`; frees pass between processors only through
// `NativeOutside::state`.
unsafe impl Send for Freed {}

/// What [`NativeOutside`] keeps for work after the turn: whole allocations
/// to free, fresh event semaphores made ahead, and semaphores to destroy.
#[derive(Default)]
struct Upkeep {
    frees: Vec<Freed>,
    spares: Vec<vk::Semaphore>,
    doomed: Vec<vk::Semaphore>,
}

/// [`Outside`] of a native backend: the device handle and exporter, which
/// Vulkan lets any thread allocate and free memory and create and destroy
/// semaphores with, the memory type imports take, and the work the backend
/// leaves for after its turn ([`Upkeep`]). A backend closing waits for the
/// calls inside before it destroys anything (`users`, `closed`), relaxing as
/// the platform's waits do, since such a call may wait for the boot
/// processor's Linux.
pub(crate) struct NativeOutside {
    device: ash::Device,
    exporter: Arc<dyn MemoryExporter>,
    /// The memory type imports take, `u32::MAX` until an import found it:
    /// every dma-buf the exporter makes is guest RAM of one kind, and asking
    /// NVK for a descriptor's types imports, maps, unmaps and closes it,
    /// five host kernel calls per import.
    import_type: AtomicU32,
    /// Whether semaphores are made exportable, as the backend makes them.
    exportable: bool,
    users: AtomicU32,
    closed: AtomicBool,
    relax: fn(),
    /// The work left for after the turn, its lock, and its lengths.
    state: UnsafeCell<Upkeep>,
    locked: AtomicBool,
    frees_count: AtomicU32,
    spares_count: AtomicU32,
    doomed_count: AtomicU32,
    /// A service runs the work after its calls' turns.
    enabled: AtomicBool,
}

// SAFETY: `state` is touched only under `locked`; the rest is atomic,
// immutable, or a Vulkan device whose memory allocation and freeing and
// semaphore creation and destruction need no external synchronization.
unsafe impl Sync for NativeOutside {}
// SAFETY: as above; kept imports are `Send`, semaphores plain handles.
unsafe impl Send for NativeOutside {}

impl NativeOutside {
    /// Frees it holds at most; past it the backend frees in its turn.
    const FREES: u32 = 64;
    /// Frees one frontend call runs after its turn at most, and the count
    /// waiting from which it runs any: below it the boot processor's
    /// upkeep runs them, where a host kernel call needs no posting.
    const FREES_A_CALL: usize = 4;
    const FREES_BACKLOG: u32 = Self::FREES / 2;
    /// Fresh event semaphores kept made, and the count below which upkeep
    /// is due.
    const SPARES: u32 = 32;
    const SPARES_LOW: u32 = 16;
    /// Semaphores waiting for destruction at most; past it the backend
    /// destroys in its turn.
    const DOOMED: u32 = 256;

    fn new(
        device: ash::Device,
        exporter: Arc<dyn MemoryExporter>,
        exportable: bool,
        relax: fn(),
    ) -> Self {
        Self {
            device,
            exporter,
            import_type: AtomicU32::new(u32::MAX),
            exportable,
            users: AtomicU32::new(0),
            closed: AtomicBool::new(false),
            relax,
            state: UnsafeCell::new(Upkeep::default()),
            locked: AtomicBool::new(false),
            frees_count: AtomicU32::new(0),
            spares_count: AtomicU32::new(0),
            doomed_count: AtomicU32::new(0),
            enabled: AtomicBool::new(false),
        }
    }

    fn import_type(&self) -> Option<u32> {
        let index = self.import_type.load(Ordering::Acquire);
        (index != u32::MAX).then_some(index)
    }

    /// Has the driver import `exported` as memory type `index`; on failure
    /// the export is closed and the next import asks for its type again.
    fn allocate_import(&self, exported: &ExportedMemory, index: u32) -> Result<vk::DeviceMemory> {
        let mut from_fd = vk::ImportMemoryFdInfoKHR::default()
            .handle_type(DMA_BUF)
            .fd(exported.fd);
        let mut flags =
            vk::MemoryAllocateFlagsInfo::default().flags(vk::MemoryAllocateFlags::DEVICE_ADDRESS);
        let allocate = vk::MemoryAllocateInfo::default()
            .allocation_size(exported.bytes)
            .memory_type_index(index)
            .push_next(&mut flags)
            .push_next(&mut from_fd);
        // SAFETY: the allocate info and its chain outlive the call.
        match unsafe { self.device.allocate_memory(&allocate, None) } {
            Ok(memory) => {
                self.import_type.store(index, Ordering::Release);
                Ok(memory)
            }
            Err(error) => {
                self.exporter.close(*exported);
                self.import_type.store(u32::MAX, Ordering::Release);
                Err(failure(error))
            }
        }
    }

    /// A timeline semaphore at `initial`, exportable when the device's
    /// semaphores can be.
    fn create_semaphore(&self, initial: u64) -> Result<vk::Semaphore> {
        let mut timeline = vk::SemaphoreTypeCreateInfo::default()
            .semaphore_type(vk::SemaphoreType::TIMELINE)
            .initial_value(initial);
        // Every timeline can be exported: NVK's are DRM syncobjs either way.
        let mut export = vk::ExportSemaphoreCreateInfo::default()
            .handle_types(vk::ExternalSemaphoreHandleTypeFlags::OPAQUE_FD);
        let mut create = vk::SemaphoreCreateInfo::default().push_next(&mut timeline);
        if self.exportable {
            create = create.push_next(&mut export);
        }
        // SAFETY: the create info and its chain outlive the call.
        unsafe { self.device.create_semaphore(&create, None) }.map_err(failure)
    }

    /// Runs `body` unless the backend closed; the backend's close waits
    /// for it.
    fn inside<T>(&self, body: impl FnOnce() -> T) -> Option<T> {
        self.users.fetch_add(1, Ordering::SeqCst);
        let result = (!self.closed.load(Ordering::SeqCst)).then(body);
        self.users.fetch_sub(1, Ordering::SeqCst);
        result
    }

    /// Stops calls from entering, waits for those inside, and answers the
    /// work not yet done, for the backend to finish once the device idles.
    fn close(&self) -> Upkeep {
        self.closed.store(true, Ordering::SeqCst);
        while self.users.load(Ordering::SeqCst) != 0 {
            (self.relax)();
        }
        self.with_state(core::mem::take)
    }

    fn with_state<T>(&self, body: impl FnOnce(&mut Upkeep) -> T) -> T {
        while self
            .locked
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }
        // SAFETY: the lock is held.
        let state = unsafe { &mut *self.state.get() };
        let result = body(state);
        self.frees_count
            .store(state.frees.len() as u32, Ordering::Release);
        self.spares_count
            .store(state.spares.len() as u32, Ordering::Release);
        self.doomed_count
            .store(state.doomed.len() as u32, Ordering::Release);
        self.locked.store(false, Ordering::Release);
        result
    }

    /// Takes `kept` to free after the turn, or gives it back when nobody
    /// runs the work ([`Outside::enable`]) or enough wait already.
    fn defer_free(&self, kept: Freed) -> Option<Freed> {
        if !self.enabled.load(Ordering::Acquire) {
            return Some(kept);
        }
        self.with_state(|state| {
            if state.frees.len() as u32 >= Self::FREES {
                Some(kept)
            } else {
                state.frees.push(kept);
                None
            }
        })
    }

    /// A fresh timeline semaphore at zero made ahead, if one is kept.
    fn take_spare(&self) -> Option<vk::Semaphore> {
        if self.spares_count.load(Ordering::Acquire) == 0 {
            return None;
        }
        self.with_state(|state| state.spares.pop())
    }

    /// Takes `semaphore` to destroy after the turn, or gives it back when
    /// nobody runs the work or enough wait already.
    fn defer_destroy(&self, semaphore: vk::Semaphore) -> Option<vk::Semaphore> {
        if !self.enabled.load(Ordering::Acquire) {
            return Some(semaphore);
        }
        self.with_state(|state| {
            if state.doomed.len() as u32 >= Self::DOOMED {
                Some(semaphore)
            } else {
                state.doomed.push(semaphore);
                None
            }
        })
    }

    /// Frees `freed`.
    fn free(&self, freed: Freed) {
        // SAFETY: the Metal device freed it after the GPU finished with it,
        // and no import took it back.
        unsafe { self.device.free_memory(freed.memory, None) };
        if let Some(exported) = freed.exported {
            self.exporter.unmap(exported);
        }
    }
}

/// One piece of upkeep, taken under the lock and done outside it.
enum Chore {
    Free(Freed),
    Destroy(vk::Semaphore),
    Make,
}

impl Outside for NativeOutside {
    fn prepare_import(self: Arc<Self>, ranges: &[MemoryRange]) -> Result<Option<PreparedImport>> {
        let made = self.inside(|| -> Result<Option<MadeImport>> {
            let Some(index) = self.import_type() else {
                // The first import finds the type in the turn.
                return Ok(None);
            };
            let exported = self.exporter.export(ranges)?;
            let memory = self.allocate_import(&exported, index)?;
            Ok(Some(MadeImport {
                memory,
                exported,
                ranges: ranges.to_vec(),
            }))
        });
        let made = made.ok_or(Error::DeviceLost)??;
        Ok(made.map(|made| PreparedImport {
            outside: self as Arc<dyn Outside>,
            made: Some(Box::new(made)),
        }))
    }

    fn run_frees(&self) -> usize {
        // The oldest few: on a guest processor each free waits for the boot
        // processor's Linux (unbind and close), about 1.5-3 ms a host kernel
        // call.
        let frees = self.with_state(|state| {
            let count = state.frees.len().min(Self::FREES_A_CALL);
            state.frees.drain(..count).collect::<Vec<_>>()
        });
        let count = frees.len();
        // Closed, the device's destruction frees what it holds.
        self.inside(|| frees.into_iter().for_each(|freed| self.free(freed)));
        count
    }

    fn frees_pending(&self) -> bool {
        self.frees_count.load(Ordering::Acquire) >= Self::FREES_BACKLOG
    }

    fn upkeep(&self, more: &mut dyn FnMut() -> bool) -> usize {
        let mut done = 0;
        self.inside(|| {
            while more() {
                let chore = self.with_state(|state| {
                    if let Some(freed) = state.frees.pop() {
                        Some(Chore::Free(freed))
                    } else if let Some(semaphore) = state.doomed.pop() {
                        Some(Chore::Destroy(semaphore))
                    } else {
                        ((state.spares.len() as u32) < Self::SPARES).then_some(Chore::Make)
                    }
                });
                match chore {
                    None => break,
                    Some(Chore::Free(freed)) => self.free(freed),
                    Some(Chore::Destroy(semaphore)) => {
                        // SAFETY: the Metal device destroys a semaphore only
                        // after every submission that uses it completed.
                        unsafe { self.device.destroy_semaphore(semaphore, None) };
                    }
                    Some(Chore::Make) => match self.create_semaphore(0) {
                        Ok(semaphore) => self.with_state(|state| state.spares.push(semaphore)),
                        // A turn makes its own, and reports a failure.
                        Err(_) => break,
                    },
                }
                done += 1;
            }
        });
        done
    }

    fn upkeep_due(&self) -> bool {
        self.enabled.load(Ordering::Acquire)
            && (self.frees_count.load(Ordering::Acquire) != 0
                || self.doomed_count.load(Ordering::Acquire) != 0
                || self.spares_count.load(Ordering::Acquire) < Self::SPARES_LOW)
    }

    fn enable(&self) {
        self.enabled.store(true, Ordering::Release);
    }

    fn discard(&self, made: Box<dyn Any + Send>) {
        let Ok(made) = made.downcast::<MadeImport>() else {
            return;
        };
        let MadeImport {
            memory, exported, ..
        } = *made;
        self.inside(|| {
            // SAFETY: no import adopted it, so nothing uses it.
            unsafe { self.device.free_memory(memory, None) };
        });
        // The driver took the descriptor; the host's mapping is the
        // exporter's either way.
        self.exporter.unmap(exported);
    }
}

/// Converts a raw boundary handle to its Vulkan type.
fn handle<T: vk::Handle>(raw: u64) -> T {
    T::from_raw(raw)
}

/// Whether pipelines of `a` and `b` are created with different flags: only
/// an archived one fails when its cache lacks it.
fn create_flags_differ(a: PipelineSource, b: PipelineSource) -> bool {
    matches!(a, PipelineSource::Archived(_)) != matches!(b, PipelineSource::Archived(_))
}

/// The cache a creation from `source` reads.
fn pipeline_cache(source: PipelineSource) -> vk::PipelineCache {
    match source {
        PipelineSource::Compiled => vk::PipelineCache::null(),
        PipelineSource::Cached(cache) | PipelineSource::Archived(cache) => handle(cache.0),
    }
}

fn raw<T: vk::Handle>(handle: T) -> u64 {
    handle.as_raw()
}

impl Backend for NativeBackend {
    fn acknowledge_idle(&mut self) -> Result<()> {
        // SAFETY: this backend exclusively owns the device. The Metal owner
        // closes admission and drains its submission timeline before asking;
        // only a successful Vulkan acknowledgement establishes idleness.
        unsafe { self.device.device_wait_idle() }.map_err(failure)?;
        self.store.idle(&self.device);
        Ok(())
    }

    fn physical_device(&self) -> &PhysicalDevice {
        &self.snapshot
    }

    fn allocate_memory(
        &mut self,
        size: u64,
        request: MemoryRequest,
        memory_types: u32,
    ) -> Result<Memory> {
        self.allocate(size, request, memory_types)
    }

    fn buffer_requirements(&mut self, info: &BufferInfo) -> Result<Requirements> {
        objects::buffer(info, |create| {
            let info = vk::DeviceBufferMemoryRequirements::default().create_info(create);
            let mut requirements = vk::MemoryRequirements2::default();
            // SAFETY: the create info outlives the call.
            unsafe {
                self.device
                    .get_device_buffer_memory_requirements(&info, &mut requirements)
            };
            Ok(objects::requirements(&requirements.memory_requirements))
        })
    }

    fn image_requirements(&mut self, info: &ImageInfo) -> Result<Requirements> {
        objects::image(info, |create| {
            let info = vk::DeviceImageMemoryRequirements::default().create_info(create);
            let mut requirements = vk::MemoryRequirements2::default();
            // SAFETY: the create info outlives the call.
            unsafe {
                self.device
                    .get_device_image_memory_requirements(&info, &mut requirements)
            };
            Ok(objects::requirements(&requirements.memory_requirements))
        })
    }

    fn create_buffer(
        &mut self,
        info: &BufferInfo,
        request: MemoryRequest,
    ) -> Result<BufferAllocation> {
        let buffer = objects::buffer(info, |create| {
            // SAFETY: the create info outlives the call.
            unsafe { self.device.create_buffer(create, None) }.map_err(failure)
        })?;
        let (needs, own) = self.buffer_needs(buffer);
        let resource = Resource {
            tiling: Tiling::Linear,
            alignment: LINEAR_PLACEMENT,
            own,
        };
        // Memory of a buffer's own is dedicated to it only when the driver
        // asks, so a fill buffer can zero all of it.
        let dedicated = own.then(|| vk::MemoryDedicatedAllocateInfo::default().buffer(buffer));
        let (memory, device_memory, offset, size) =
            match self.allocate_resource(needs, resource, request, dedicated) {
                Ok(allocated) => allocated,
                Err(error) => {
                    // SAFETY: nothing uses the new buffer.
                    unsafe { self.device.destroy_buffer(buffer, None) };
                    return Err(error);
                }
            };
        // SAFETY: the memory, or the place at `offset` in it, was allocated
        // for this buffer's requirements.
        let bound = unsafe {
            self.device
                .bind_buffer_memory(buffer, device_memory, offset)
        }
        .map_err(failure)
        .and_then(|()| {
            // Metal zero-fills a new buffer; the whole place is zeroed, so
            // no read past its end finds another resource's bytes.
            if dedicated.is_some() && !request.mapped {
                self.zeroing.pending.buffers.push(buffer);
                Ok(())
            } else {
                self.zero_range(device_memory, offset, size)
            }
        });
        self.or_destroy(
            bound,
            &[Object::Buffer(Buffer(raw(buffer))), Object::Memory(memory)],
        )?;
        Ok(BufferAllocation {
            buffer: Buffer(raw(buffer)),
            memory,
            address: self.address(buffer, info),
            allocated: needs.size,
        })
    }

    fn import_memory(&mut self, ranges: &[MemoryRange], token: NonZeroU64) -> Result<Memory> {
        // The same guest RAM imported again: a freed import of it views the
        // same pages, still bound.
        if let Some(kept) = self.import_cache.take(ranges) {
            self.mappings.insert(
                raw(kept.memory),
                Mapping {
                    pointer: kept.exported.pointer,
                    size: kept.exported.bytes,
                    coherent: true,
                },
            );
            self.imports
                .insert(raw(kept.memory), (kept.exported, token, kept.ranges));
            return Ok(self.memories.insert(memory::Allocation::Whole {
                memory: kept.memory,
                resource: false,
            }));
        }
        let exported = self.exporter.export(ranges)?;
        let imported = match self.outside.import_type() {
            Some(index) => Ok(index),
            None => self.import_memory_type(exported.fd),
        };
        let index = match imported {
            Ok(index) => index,
            Err(error) => {
                self.exporter.close(exported);
                return Err(error);
            }
        };
        let memory = self.outside.allocate_import(&exported, index)?;
        Ok(self.own_import(memory, exported, token, ranges.to_vec()))
    }

    fn keeps_import(&self, ranges: &[MemoryRange]) -> bool {
        self.import_cache.holds(ranges)
    }

    fn outside(&self) -> Option<Arc<dyn Outside>> {
        Some(self.outside.clone())
    }

    fn adopt_import(&mut self, mut prepared: PreparedImport, token: NonZeroU64) -> Result<Memory> {
        let outside: Arc<dyn Outside> = self.outside.clone();
        if !prepared.made_by(&outside) {
            // Another device's: freed as the preparation drops.
            return Err(Error::DeviceLost);
        }
        let made = prepared
            .take()
            .and_then(|made| made.downcast::<MadeImport>().ok())
            .ok_or(Error::Invalid("an import this backend did not make"))?;
        let MadeImport {
            memory,
            exported,
            ranges,
        } = *made;
        Ok(self.own_import(memory, exported, token, ranges))
    }

    fn retired(&mut self) -> Option<NonZeroU64> {
        self.retired.pop_front()
    }

    fn retiring(&self) -> bool {
        !self.retired.is_empty()
    }

    fn create_placed_buffer(
        &mut self,
        info: &BufferInfo,
        memory: Memory,
        offset: u64,
    ) -> Result<(Buffer, u64)> {
        let create = |create: &vk::BufferCreateInfo<'_>| {
            // SAFETY: the create info outlives the call.
            unsafe { self.device.create_buffer(create, None) }.map_err(failure)
        };
        let resolved = self.resolve(memory)?;
        let buffer = if self.imports.contains_key(&raw(resolved.memory)) {
            objects::imported_buffer(info, DMA_BUF, create)
        } else {
            objects::buffer(info, create)
        }?;
        // SAFETY: the Metal heap placed the buffer within `memory`, which
        // starts `resolved.offset` into its `VkDeviceMemory`.
        let bound = unsafe {
            self.device
                .bind_buffer_memory(buffer, resolved.memory, resolved.offset + offset)
        }
        .map_err(failure);
        self.or_destroy(bound, &[Object::Buffer(Buffer(raw(buffer)))])?;
        Ok((Buffer(raw(buffer)), self.address(buffer, info)))
    }

    fn create_image(
        &mut self,
        info: &ImageInfo,
        request: MemoryRequest,
    ) -> Result<(Image, Memory, u64)> {
        let started = (self.ticks)();
        let key = premade::image_key(info, request);
        if let Some(made) = self.premade.take(&key) {
            return self.premade_image(made, info, started);
        }
        let image = objects::image(info, |create| {
            // SAFETY: the create info and its chain outlive the call.
            unsafe { self.device.create_image(create, None) }.map_err(failure)
        })?;
        let (needs, own) = self.image_needs(image);
        let tiling = match info.tiling {
            super::vulkan::ImageTiling::Optimal => Tiling::Optimal,
            _ => Tiling::Linear,
        };
        let resource = Resource {
            tiling,
            alignment: 1,
            own,
        };
        if self.defer_image_memory(key, info, request, needs, resource) {
            // SAFETY: nothing uses the new image; the job makes its own.
            unsafe { self.device.destroy_image(image, None) };
            return Err(super::compiler::DEFERRED);
        }
        let dedicated = vk::MemoryDedicatedAllocateInfo::default().image(image);
        let created = (self.ticks)();
        let (memory, device_memory, offset, _) =
            match self.allocate_resource(needs, resource, request, Some(dedicated)) {
                Ok(allocated) => allocated,
                Err(error) => {
                    // SAFETY: nothing uses the new image.
                    unsafe { self.device.destroy_image(image, None) };
                    return Err(error);
                }
            };
        let allocated = (self.ticks)();
        // SAFETY: the memory, or the place at `offset` in it, was allocated
        // for this image's requirements.
        let bound =
            unsafe { self.device.bind_image_memory(image, device_memory, offset) }.map_err(failure);
        let adopted = bound.and_then(|()| self.adopt_image(image, info));
        self.time_split([started, created, allocated, (self.ticks)()]);
        self.or_destroy(
            adopted,
            &[Object::Image(Image(raw(image))), Object::Memory(memory)],
        )?;
        // Its memory may hold another resource's bytes until a clear, or a
        // write of every subresource, reaches it.
        self.zeroing
            .pending
            .images
            .insert(raw(image), zero::Unzeroed::new(info));
        Ok((Image(raw(image)), memory, needs.size))
    }

    fn create_placed_image(
        &mut self,
        info: &ImageInfo,
        memory: Memory,
        offset: u64,
    ) -> Result<Image> {
        let image = objects::image(info, |create| {
            // SAFETY: the create info and its chain outlive the call.
            unsafe { self.device.create_image(create, None) }.map_err(failure)
        })?;
        let bound = self.resolve(memory).and_then(|resolved| {
            // SAFETY: the Metal heap or buffer placed the image within
            // `memory`, which starts `resolved.offset` into its
            // `VkDeviceMemory`.
            unsafe {
                self.device
                    .bind_image_memory(image, resolved.memory, resolved.offset + offset)
            }
            .map_err(failure)
        });
        let adopted = bound.and_then(|()| self.adopt_image(image, info));
        self.or_destroy(adopted, &[Object::Image(Image(raw(image)))])?;
        Ok(Image(raw(image)))
    }

    fn clear_reused_image(&mut self, image: Image, info: &ImageInfo) {
        self.zeroing
            .pending
            .images
            .insert(image.0, zero::Unzeroed::new(info));
    }

    fn create_sparse_image(
        &mut self,
        info: &ImageInfo,
    ) -> Result<(Image, SparseImageRequirements)> {
        let image = objects::image(info, |create| {
            // SAFETY: the create info and its chain outlive the call.
            unsafe { self.device.create_image(create, None) }.map_err(failure)
        })?;
        // SAFETY: the image was just created.
        let (needs, sparse) = unsafe {
            (
                self.device.get_image_memory_requirements(image),
                self.device.get_image_sparse_memory_requirements(image),
            )
        };
        let color = sparse.iter().find(|requirements| {
            requirements
                .format_properties
                .aspect_mask
                .contains(vk::ImageAspectFlags::COLOR)
        });
        let requirements = color
            .map(|color| {
                let granularity = color.format_properties.image_granularity;
                SparseImageRequirements {
                    memory_types: needs.memory_type_bits,
                    granularity: [granularity.width, granularity.height, granularity.depth],
                    mip_tail_first_lod: color.image_mip_tail_first_lod,
                    mip_tail_size: color.image_mip_tail_size,
                    mip_tail_offset: color.image_mip_tail_offset,
                    mip_tail_stride: color.image_mip_tail_stride,
                    single_mip_tail: color
                        .format_properties
                        .flags
                        .contains(vk::SparseImageFormatFlags::SINGLE_MIPTAIL),
                }
            })
            .ok_or(Error::Unsupported("sparse residency of this image"));
        let adopted = requirements
            .and_then(|requirements| self.adopt_image(image, info).map(|()| requirements));
        let requirements = self.or_destroy(adopted, &[Object::Image(Image(raw(image)))])?;
        Ok((Image(raw(image)), requirements))
    }

    fn create_image_view(&mut self, image: Image, info: &ImageViewInfo) -> Result<ImageView> {
        let view = objects::image_view(handle(image.0), info, |create| {
            // SAFETY: the create info and its chain outlive the call.
            unsafe { self.device.create_image_view(create, None) }
                .map(|view| ImageView(raw(view)))
                .map_err(failure)
        })?;
        // Render passes name their attachments by view.
        self.zeroing.views.insert(
            view.0,
            zero::ViewRange {
                image: image.0,
                range: info.range,
            },
        );
        Ok(view)
    }

    fn create_buffer_view(&mut self, buffer: Buffer, info: &BufferViewInfo) -> Result<BufferView> {
        let create = vk::BufferViewCreateInfo::default()
            .buffer(handle(buffer.0))
            .format(objects::format(info.format))
            .offset(info.offset)
            .range(info.range);
        // SAFETY: the create info outlives the call.
        unsafe { self.device.create_buffer_view(&create, None) }
            .map(|view| BufferView(raw(view)))
            .map_err(failure)
    }

    fn create_sampler(&mut self, info: &SamplerInfo) -> Result<Sampler> {
        objects::sampler(info, |create| {
            // SAFETY: the create info and its chain outlive the call.
            unsafe { self.device.create_sampler(create, None) }
                .map(|sampler| Sampler(raw(sampler)))
                .map_err(failure)
        })
    }

    fn create_graphics_pipeline(&mut self, info: &GraphicsPipelineInfo) -> Result<Pipeline> {
        if let Some(created) = self.deferred_creation(deferral::Wanted::Graphics(info)) {
            let pipeline = created?;
            self.report_executables(pipeline);
            return Ok(Pipeline(raw(pipeline)));
        }
        self.through_store(info.source, |backend, source, feedback| {
            // Only a full store's lookup changes the creation flags.
            let replaced;
            let info = if create_flags_differ(info.source, source) {
                replaced = GraphicsPipelineInfo {
                    source,
                    ..info.clone()
                };
                &replaced
            } else {
                info
            };
            objects::graphics_pipeline(info, backend.layout, |create| {
                let mut chained = vk::PipelineCreationFeedbackCreateInfo::default()
                    .pipeline_creation_feedback(feedback);
                let create = (*create).push_next(&mut chained);
                // SAFETY: the create info, every structure it points to and
                // the NIR outlive the call.
                unsafe {
                    backend.device.create_graphics_pipelines(
                        pipeline_cache(source),
                        &[create],
                        None,
                    )
                }
                .map(|pipelines| pipelines[0])
                .map_err(|(_, error)| failure(error))
            })
        })
        .map(|pipeline| {
            self.report_executables(pipeline);
            Pipeline(raw(pipeline))
        })
    }

    fn create_compute_pipeline(&mut self, info: &ComputePipelineInfo) -> Result<Pipeline> {
        self.create_compute(info)
            .map(|pipeline| Pipeline(raw(pipeline)))
    }

    fn defer_pipelines(&mut self, deferring: Option<&crate::metal::deferred::JobLock>) {
        self.deferral.set(deferring);
    }

    fn stop_deferring(&mut self) {
        self.deferral.stop();
    }

    fn deferred(&self) -> bool {
        self.deferral.deferred()
    }

    fn take_job(&mut self) -> Option<crate::metal::deferred::Job> {
        self.deferral.take()
    }

    fn adopt_job(&mut self, done: crate::metal::deferred::Done) {
        let crate::metal::deferred::Done { key, owner, made } = done;
        match made.downcast::<premade::Making>() {
            Ok(making) => self.adopt_memory(key, *making),
            Err(made) => self.adopt_pipeline(crate::metal::deferred::Done { key, owner, made }),
        }
    }

    fn create_pipeline_cache(&mut self, data: &[u8]) -> Result<PipelineCache> {
        let create = vk::PipelineCacheCreateInfo::default().initial_data(data);
        // SAFETY: the create info and the data outlive the call, and the
        // driver ignores data whose header is not its own.
        unsafe { self.device.create_pipeline_cache(&create, None) }
            .map(|cache| PipelineCache(raw(cache)))
            .map_err(failure)
    }

    fn pipeline_cache_data(&mut self, cache: PipelineCache) -> Result<Vec<u8>> {
        // SAFETY: the cache is alive and the service creates no pipeline
        // meanwhile.
        unsafe { self.device.get_pipeline_cache_data(handle(cache.0)) }.map_err(failure)
    }

    fn create_query_pool(&mut self, info: &QueryPoolInfo) -> Result<QueryPool> {
        let create = vk::QueryPoolCreateInfo::default()
            .query_type(vk::QueryType::from_raw(info.query_type.raw() as i32))
            .query_count(info.count)
            .pipeline_statistics(vk::QueryPipelineStatisticFlags::from_raw(
                info.statistics.bits(),
            ));
        // SAFETY: the create info outlives the call.
        let pool = unsafe { self.device.create_query_pool(&create, None) }.map_err(failure)?;
        // Pools start unavailable; Metal's first use expects reset queries.
        // SAFETY: `hostQueryReset` is enabled and nothing uses the pool.
        unsafe { self.device.reset_query_pool(pool, 0, info.count) };
        Ok(QueryPool(raw(pool)))
    }

    fn create_semaphore(&mut self, initial: u64) -> Result<Semaphore> {
        // One made ahead after a turn is the same as a new one: a timeline
        // at zero, exportable alike. Creating one in the turn is a host
        // kernel call a guest processor waits for the boot processor for.
        let spare = if initial == 0 {
            self.outside.take_spare()
        } else {
            None
        };
        let made = match spare {
            Some(spare) => spare,
            None => self.outside.create_semaphore(initial)?,
        };
        Ok(Semaphore(raw(made)))
    }

    fn export_semaphore(&mut self, semaphore: Semaphore) -> Option<i32> {
        let fns = self.fns.external_semaphore_fd.as_ref()?;
        let info = vk::SemaphoreGetFdInfoKHR::default()
            .semaphore(handle(semaphore.0))
            .handle_type(vk::ExternalSemaphoreHandleTypeFlags::OPAQUE_FD);
        // SAFETY: the semaphore is this device's, created exportable.
        unsafe { fns.get_semaphore_fd(&info) }.ok()
    }

    fn create_indirect_commands_layout(
        &mut self,
        info: &IndirectCommandsLayoutInfo,
    ) -> Result<IndirectCommandsLayout> {
        self.generated()
            .create_layout(info, self.layout)
            .map(IndirectCommandsLayout)
            .map_err(failure)
    }

    fn create_execution_set(&mut self, initial: Pipeline, capacity: u32) -> Result<ExecutionSet> {
        self.generated()
            .create_set(handle(initial.0), capacity)
            .map(ExecutionSet)
            .map_err(failure)
    }

    fn write_execution_set(&mut self, set: ExecutionSet, index: u32, pipeline: Pipeline) {
        // SAFETY: the Metal device writes only indices no pending
        // submission uses.
        unsafe { self.generated().write_set(set.0, index, handle(pipeline.0)) }
    }

    fn generated_commands_requirements(
        &mut self,
        layout: IndirectCommandsLayout,
        set: Option<ExecutionSet>,
        sequences: u32,
        request: MemoryRequest,
    ) -> Result<Requirements> {
        let needs = self
            .generated()
            .memory_requirements(layout.0, set.map(|set| set.0), sequences);
        // Every type the request could allocate from must hold the memory.
        let candidates = objects::memory_candidates(&self.snapshot.memory, u32::MAX, request);
        if candidates
            .iter()
            .any(|&index| needs.memory_type_bits & 1 << index == 0)
        {
            return Err(Error::Unsupported(
                "generated commands preprocessing in host-visible memory",
            ));
        }
        Ok(Requirements {
            size: needs.size,
            alignment: needs.alignment,
            memory_types: needs.memory_type_bits,
        })
    }

    fn destroy(&mut self, object: Object) {
        if let Object::Memory(memory) = object {
            self.free_memory(memory);
            return;
        }
        // SAFETY: the Metal device destroys an object only after every
        // submission that could use it completed, and only once.
        unsafe {
            match object {
                Object::Buffer(buffer) => {
                    let buffer: vk::Buffer = handle(buffer.0);
                    self.zeroing
                        .pending
                        .buffers
                        .retain(|&pending| pending != buffer);
                    self.device.destroy_buffer(buffer, None);
                }
                Object::Image(image) => {
                    self.zeroing.pending.images.remove(&image.0);
                    let image: vk::Image = handle(image.0);
                    self.new_images.retain(|(new, _)| *new != image);
                    self.device.destroy_image(image, None);
                }
                Object::ImageView(view) => {
                    self.zeroing.views.remove(&view.0);
                    self.device.destroy_image_view(handle(view.0), None);
                }
                Object::BufferView(view) => {
                    self.device.destroy_buffer_view(handle(view.0), None);
                }
                Object::Sampler(sampler) => self.device.destroy_sampler(handle(sampler.0), None),
                Object::Pipeline(pipeline) => {
                    self.device.destroy_pipeline(handle(pipeline.0), None);
                }
                Object::PipelineCache(cache) => {
                    self.device.destroy_pipeline_cache(handle(cache.0), None);
                }
                Object::QueryPool(pool) => self.device.destroy_query_pool(handle(pool.0), None),
                Object::Semaphore(semaphore) => {
                    // After the turn when the outside work takes it, as the
                    // destruction is a host kernel call.
                    if let Some(semaphore) = self.outside.defer_destroy(handle(semaphore.0)) {
                        self.device.destroy_semaphore(semaphore, None);
                    }
                }
                Object::Memory(_) => {}
                Object::IndirectCommandsLayout(layout) => self.generated().destroy_layout(layout.0),
                Object::ExecutionSet(set) => self.generated().destroy_set(set.0),
            }
        }
    }

    fn write_descriptor(&mut self, index: u32, descriptor: Descriptor) {
        objects::descriptor_write(self.heap.set, index, descriptor, |write| {
            // SAFETY: the heap is update-after-bind and update-unused-while-
            // pending, so a slot no pending command uses may be rewritten.
            unsafe { self.device.update_descriptor_sets(&[*write], &[]) }
        });
    }

    fn maps_words(&self) -> bool {
        true
    }

    fn host_word(&mut self, memory: Memory, offset: u64) -> Option<NonNull<AtomicU32>> {
        let mapped = self.mapping(memory, offset, 4).ok()?;
        let aligned =
            (mapped.pointer.as_ptr() as usize).is_multiple_of(core::mem::align_of::<AtomicU32>());
        (mapped.coherent && aligned).then(|| mapped.pointer.cast())
    }

    fn host_bytes(&mut self, memory: Memory, offset: u64, length: u64) -> Option<NonNull<u8>> {
        let mapped = self.mapping(memory, offset, length).ok()?;
        mapped.coherent.then_some(mapped.pointer)
    }

    fn write_memory(&mut self, memory: Memory, offset: u64, bytes: &[u8]) -> Result<()> {
        let mapped = self.mapping(memory, offset, bytes.len() as u64)?;
        // SAFETY: `mapped` covers the range and the slice is disjoint from
        // device memory.
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), mapped.pointer.as_ptr(), bytes.len());
        }
        if mapped.coherent {
            Ok(())
        } else {
            self.flush_range(mapped.memory, mapped.offset, bytes.len() as u64)
        }
    }

    fn read_memory(&mut self, memory: Memory, offset: u64, bytes: &mut [u8]) -> Result<()> {
        let mapped = self.mapping(memory, offset, bytes.len() as u64)?;
        if !mapped.coherent {
            self.invalidate_range(mapped.memory, mapped.offset, bytes.len() as u64)?;
        }
        // SAFETY: as in `write_memory`.
        unsafe {
            core::ptr::copy_nonoverlapping(
                mapped.pointer.as_ptr(),
                bytes.as_mut_ptr(),
                bytes.len(),
            );
        }
        Ok(())
    }

    fn flush_memory(&mut self, memory: Memory, offset: u64, size: u64) -> Result<()> {
        let resolved = self.resolve(memory)?;
        self.flush_range(resolved.memory, resolved.offset + offset, size)
    }

    fn invalidate_memory(&mut self, memory: Memory, offset: u64, size: u64) -> Result<()> {
        let resolved = self.resolve(memory)?;
        self.invalidate_range(resolved.memory, resolved.offset + offset, size)
    }

    fn copy_to_image(
        &mut self,
        image: Image,
        region: &BufferImageCopy,
        bytes: &[u8],
    ) -> Result<()> {
        let (sub, offset, extent) = (region.subresource, region.offset, region.extent);
        let whole = self
            .zeroing
            .pending
            .images
            .get(&image.0)
            .is_none_or(|entry| entry.whole(sub, offset, extent));
        if !whole {
            self.zero_on_host(image, sub)?;
        }
        let copy = self
            .fns
            .host_image_copy
            .as_ref()
            .ok_or(Error::Unsupported("host image copies"))?;
        let regions = [objects::memory_to_image(region, bytes)];
        let info = vk::CopyMemoryToImageInfoEXT::default()
            .dst_image(handle(image.0))
            .dst_image_layout(vk::ImageLayout::GENERAL)
            .regions(&regions);
        // SAFETY: the Metal layer sized `bytes` to the region, tightly
        // packed, and the image allows host transfers.
        unsafe { copy.copy_memory_to_image(&info) }.map_err(failure)?;
        if let Some(entry) = self.zeroing.pending.images.get_mut(&image.0) {
            entry.write(sub, offset, extent);
        }
        Ok(())
    }

    fn copy_from_image(
        &mut self,
        image: Image,
        region: &BufferImageCopy,
        bytes: &mut [u8],
    ) -> Result<()> {
        self.zero_on_host(image, region.subresource)?;
        let copy = self
            .fns
            .host_image_copy
            .as_ref()
            .ok_or(Error::Unsupported("host image copies"))?;
        let regions = [objects::image_to_memory(region, bytes)];
        let info = vk::CopyImageToMemoryInfoEXT::default()
            .src_image(handle(image.0))
            .src_image_layout(vk::ImageLayout::GENERAL)
            .regions(&regions);
        // SAFETY: as in `copy_to_image`.
        unsafe { copy.copy_image_to_memory(&info) }.map_err(failure)
    }

    fn reset_queries(&mut self, pool: QueryPool, first: u32, count: u32) {
        // SAFETY: `hostQueryReset` is enabled; no pending command uses the
        // queries the Metal device resets.
        unsafe { self.device.reset_query_pool(handle(pool.0), first, count) };
    }

    fn query_results(&mut self, pool: QueryPool, first: u32, results: &mut [u64]) -> Result<bool> {
        // SAFETY: the pool holds `results.len()` queries from `first`.
        let queried = unsafe {
            self.device.get_query_pool_results(
                handle(pool.0),
                first,
                results,
                vk::QueryResultFlags::TYPE_64,
            )
        };
        match queried {
            Ok(()) => Ok(true),
            Err(vk::Result::NOT_READY) => Ok(false),
            Err(error) => Err(failure(error)),
        }
    }

    fn submit_failure(&mut self, semaphore: Semaphore, value: u64) -> Option<Error> {
        let refusals = self.refusals?;
        // SAFETY: the queue and semaphore are alive; the driver only reads
        // its own record of refused submissions.
        let result = unsafe { (refusals.take)(self.queue, handle(semaphore.0), value) };
        (result != vk::Result::SUCCESS).then(|| failure(result))
    }

    fn settle_failures(&mut self, mut settled: u32, forget: bool) {
        let (Some(refusals), Some(outstanding)) = (self.refusals, &self.outstanding) else {
            return;
        };
        if forget {
            // SAFETY: the queue is alive.
            settled += unsafe { (refusals.forget)(self.queue) };
        }
        if settled > 0 {
            outstanding.fetch_sub(settled, Ordering::AcqRel);
        }
    }

    fn count_failures(&mut self, outstanding: &Arc<AtomicU32>) {
        let Some(refusals) = self.refusals else {
            return;
        };
        outstanding.store(0, Ordering::Release);
        // SAFETY: the queue is alive, and the count outlives it: the
        // backend keeps it.
        unsafe { (refusals.count)(self.queue, outstanding.as_ptr()) };
        self.outstanding = Some(outstanding.clone());
    }

    fn semaphore_value(&mut self, semaphore: Semaphore) -> Result<u64> {
        let timeline = handle(semaphore.0);
        // SAFETY: the semaphore is alive.
        let value =
            unsafe { self.device.get_semaphore_counter_value(timeline) }.map_err(failure)?;
        // The Metal completion poll already observed this timeline. Free the
        // prefix it proves complete so the next submission need not query it
        // again just to free these command buffers.
        self.timeline_passed(semaphore, value);
        Ok(value)
    }

    fn timeline_passed(&mut self, semaphore: Semaphore, value: u64) {
        let timeline = handle(semaphore.0);
        let ready = self
            .in_flight
            .iter()
            .take_while(|entry| entry.timeline == timeline && entry.value <= value)
            .count();
        self.free_completed(ready);
    }

    fn signal_semaphore(&mut self, semaphore: Semaphore, value: u64) -> Result<()> {
        let info = vk::SemaphoreSignalInfo::default()
            .semaphore(handle(semaphore.0))
            .value(value);
        // SAFETY: the Metal device signals only increasing values.
        unsafe { self.device.signal_semaphore(&info) }.map_err(failure)
    }

    fn wait_semaphore(&mut self, semaphore: Semaphore, value: u64, timeout: u64) -> Result<bool> {
        let semaphores = [handle(semaphore.0)];
        let values = [value];
        let info = vk::SemaphoreWaitInfo::default()
            .semaphores(&semaphores)
            .values(&values);
        // SAFETY: the semaphore is alive.
        match unsafe { self.device.wait_semaphores(&info, timeout) } {
            Ok(()) => Ok(true),
            Err(vk::Result::TIMEOUT) => Ok(false),
            Err(error) => Err(failure(error)),
        }
    }

    fn submit(&mut self, segments: &[Segment]) -> Result<()> {
        let started = (self.ticks)();
        // The device's completion poll frees what it finds complete
        // (`semaphore_value`). Asking the timeline again here would cost
        // every submission a driver call, which on a host whose driver
        // calls cross to another processor waits for that one; only a queue
        // nobody polls grows to the bound.
        if self.in_flight.len() >= RECLAIM_AFTER {
            self.reclaim()?;
        }
        if segments.is_empty() {
            return Ok(());
        }
        let reclaimed = (self.ticks)();
        let buffers =
            CommandBuffers::allocate(&self.device, self.command_pool, segments.len() as u32)?;
        let handles = buffers.as_slice();
        let new_images = core::mem::take(&mut self.new_images);
        let pending = core::mem::take(&mut self.zeroing.pending);
        let zero = self.zero_work(&pending, &segments[0].commands);
        let none = record::Zero::default();
        let recorded =
            handles
                .iter()
                .zip(segments)
                .enumerate()
                .try_for_each(|(index, (&buffer, segment))| {
                    let (adopt, zero) = if index == 0 {
                        (&new_images[..], &zero)
                    } else {
                        (&[][..], &none)
                    };
                    record::segment(self, buffer, adopt, zero, segment)
                });
        let recorded_at = (self.ticks)();
        let submitted = recorded.and_then(|()| self.queue_segments(segments, handles));
        self.time_split([started, reclaimed, recorded_at, (self.ticks)()]);
        if let Err(error) = submitted {
            // Some segments may have reached the queue. A failed idle query
            // leaves their buffers owned by the command pool until device
            // teardown can prove idleness.
            let _ = free_after_idle(&self.device, self.queue, self.command_pool, handles);
            self.new_images = new_images;
            self.zeroing.pending = pending;
            return Err(error);
        }
        // A commit appends the queue's completion signal last.
        let completion = segments.last().and_then(|segment| segment.signals.last());
        match completion {
            Some(&(timeline, value)) => {
                let buffers = if self.unsignaled.is_empty() {
                    buffers
                } else {
                    let mut covered = core::mem::take(&mut self.unsignaled);
                    covered.extend_from_slice(buffers.as_slice());
                    CommandBuffers::Many(covered)
                };
                self.in_flight.push_back(InFlight {
                    buffers,
                    timeline: handle(timeline.0),
                    value,
                });
            }
            // Free with the next submission that signals (its signal waits for
            // all commands submitted before it), or with the pool, rather than
            // waiting here for the queue to idle.
            None => self.unsignaled.extend_from_slice(handles),
        }
        Ok(())
    }

    fn calibrated_timestamps(&mut self) -> Result<(u64, u64)> {
        let calibrated = self
            .fns
            .calibrated
            .as_ref()
            .ok_or(Error::Unsupported("calibrated timestamps"))?;
        let domains = [
            vk::CalibratedTimestampInfoKHR::default().time_domain(vk::TimeDomainKHR::DEVICE),
            vk::CalibratedTimestampInfoKHR::default()
                .time_domain(vk::TimeDomainKHR::CLOCK_MONOTONIC),
        ];
        // Both domains are valid for Linux-derived drivers.
        // SAFETY: the caller's domains, on the device these were loaded for.
        let (stamps, _) =
            unsafe { calibrated.get_calibrated_timestamps(&domains) }.map_err(failure)?;
        Ok((stamps[1], stamps[0]))
    }

    fn set_clock(&mut self, ticks: fn() -> u64) {
        self.ticks = ticks;
    }

    fn take_split(&mut self) -> [u64; 3] {
        core::mem::take(&mut self.split)
    }

    fn memory_blocks(&self) -> MemoryBlocks {
        self.memories.blocks.report()
    }

    fn whole_memory(&self, memory: Memory) -> bool {
        self.memories
            .resolve(memory)
            .is_some_and(|resolved| resolved.size.is_none())
    }
}

/// Free possibly submitted buffers only after the driver acknowledges idle.
fn free_after_idle(
    device: &ash::Device,
    queue: vk::Queue,
    pool: vk::CommandPool,
    buffers: &[vk::CommandBuffer],
) -> Result<()> {
    // SAFETY: the backend exclusively owns this queue, pool and buffers.
    unsafe {
        device.queue_wait_idle(queue).map_err(failure)?;
        device.free_command_buffers(pool, buffers);
    }
    Ok(())
}

/// Submits `pending` in one `vkQueueSubmit2` and empties it.
fn flush(
    device: &ash::Device,
    queue: vk::Queue,
    pending: &mut Vec<vk::SubmitInfo2<'_>>,
) -> Result<()> {
    if pending.is_empty() {
        return Ok(());
    }
    // SAFETY: every command buffer is recorded and every array outlives the
    // call; the queue is used only by this backend.
    let submitted = unsafe { device.queue_submit2(queue, pending, vk::Fence::null()) };
    pending.clear();
    submitted.map_err(failure)
}

/// Keep the Vulkan packet for a single ordinary segment on the stack.
fn submit_single_segment(
    device: &ash::Device,
    queue: vk::Queue,
    segments: &[Segment],
    buffers: &[vk::CommandBuffer],
) -> Option<Result<()>> {
    let ([segment], [buffer]) = (segments, buffers) else {
        return None;
    };
    if !segment.binds.is_empty() || segment.waits.len() > 1 || segment.signals.len() > 1 {
        return None;
    }
    let wait = segment.waits.first().copied().map(objects::semaphore);
    let signal = segment.signals.first().copied().map(objects::semaphore);
    let commands = [vk::CommandBufferSubmitInfo::default().command_buffer(*buffer)];
    let submits = [vk::SubmitInfo2::default()
        .wait_semaphore_infos(wait.as_slice())
        .command_buffer_infos(&commands)
        .signal_semaphore_infos(signal.as_slice())];
    // SAFETY: the backend owns the queue and recorded buffer. All packet
    // arrays stay alive for the call, which consumes their contents.
    Some(unsafe { device.queue_submit2(queue, &submits, vk::Fence::null()) }.map_err(failure))
}

impl NativeBackend {
    /// Adds the ticks between each of a call's phase marks to its split.
    fn time_split(&mut self, marks: [u64; 4]) {
        for (slot, pair) in self.split.iter_mut().zip(marks.windows(2)) {
            *slot = slot.saturating_add(pair[1].wrapping_sub(pair[0]));
        }
    }

    /// Submits the recorded segments in order. A segment with sparse binds
    /// splits the submission: the queue signals the bind timeline once
    /// everything before is done, the binds wait for that and the
    /// segment's own waits, and the segment's commands wait for the binds.
    fn queue_segments(
        &mut self,
        segments: &[Segment],
        buffers: &[vk::CommandBuffer],
    ) -> Result<()> {
        if let Some(submitted) = submit_single_segment(&self.device, self.queue, segments, buffers)
        {
            return submitted;
        }
        let bound: Vec<Option<u64>> = segments
            .iter()
            .map(|segment| {
                (!segment.binds.is_empty()).then(|| {
                    self.bind_value += 2;
                    self.bind_value
                })
            })
            .collect();
        let timeline = |value| {
            [vk::SemaphoreSubmitInfo::default()
                .semaphore(self.bind_timeline)
                .value(value)
                .stage_mask(vk::PipelineStageFlags2::ALL_COMMANDS)]
        };
        let ready: Vec<_> = bound
            .iter()
            .map(|value| timeline(value.map_or(0, |value| value - 1)))
            .collect();
        let waits: Vec<Vec<vk::SemaphoreSubmitInfo>> = segments
            .iter()
            .zip(&bound)
            .map(|(segment, bound)| match bound {
                Some(value) => timeline(*value).to_vec(),
                None => objects::semaphores(&segment.waits),
            })
            .collect();
        let signals: Vec<Vec<vk::SemaphoreSubmitInfo>> = segments
            .iter()
            .map(|segment| objects::semaphores(&segment.signals))
            .collect();
        let commands: Vec<[vk::CommandBufferSubmitInfo; 1]> = buffers
            .iter()
            .map(|&buffer| [vk::CommandBufferSubmitInfo::default().command_buffer(buffer)])
            .collect();
        let mut pending = Vec::new();
        for (index, segment) in segments.iter().enumerate() {
            if let Some(value) = bound[index] {
                pending.push(vk::SubmitInfo2::default().signal_semaphore_infos(&ready[index]));
                flush(&self.device, self.queue, &mut pending)?;
                let mut waits: Vec<_> = segment
                    .waits
                    .iter()
                    .map(|&(semaphore, value)| (handle(semaphore.0), value))
                    .collect();
                waits.push((self.bind_timeline, value - 1));
                let memories = &self.memories;
                let resolve = |memory| {
                    memories
                        .resolve(memory)
                        .map_or((vk::DeviceMemory::null(), 0), |resolved| {
                            (resolved.memory, resolved.offset)
                        })
                };
                sparse::SparseBatch::new(&segment.binds, resolve).submit(
                    &self.device,
                    self.queue,
                    &waits,
                    (self.bind_timeline, value),
                )?;
            }
            pending.push(
                vk::SubmitInfo2::default()
                    .wait_semaphore_infos(&waits[index])
                    .command_buffer_infos(&commands[index])
                    .signal_semaphore_infos(&signals[index]),
            );
        }
        flush(&self.device, self.queue, &mut pending)
    }

    /// `vkGetBufferDeviceAddress` when the buffer allows it.
    fn address(&self, buffer: vk::Buffer, info: &BufferInfo) -> u64 {
        if !info
            .usage
            .contains(super::vulkan::BufferUsage::SHADER_DEVICE_ADDRESS)
        {
            return 0;
        }
        let info = vk::BufferDeviceAddressInfo::default().buffer(buffer);
        // SAFETY: the buffer is bound and has device-address usage.
        unsafe { self.device.get_buffer_device_address(&info) }
    }
}
