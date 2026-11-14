//! The Vulkan device the Metal layer drives.
//!
//! The native process implements [`Backend`] over the vendor's Mesa Vulkan
//! driver (NVK for NVIDIA): one `VkDevice` with the extensions and features
//! [`requirements`] lists, one universal queue, the bindless descriptor set
//! of [`binding`](crate::metal::binding) and the single pipeline layout
//! every pipeline uses. Shader code arrives as NIR the compiler built, which
//! the backend hands to the driver's internal NIR entry instead of SPIR-V.
//! Each method is one Vulkan call or a fixed sequence of them; failures map
//! to [`Error::OutOfMemory`], [`Error::DeviceLost`] or, for a result with no
//! Metal answer, [`Error::Driver`] with its raw `VkResult`.
//!
//! [`Error::OutOfMemory`]: crate::metal::error::Error::OutOfMemory
//! [`Error::DeviceLost`]: crate::metal::error::Error::DeviceLost
//! [`Error::Driver`]: crate::metal::error::Error::Driver
use super::{
    object::{
        Buffer, BufferInfo, BufferView, BufferViewInfo, ComputePipelineInfo, ExecutionSet,
        GraphicsPipelineInfo, Image, ImageInfo, ImageView, ImageViewInfo, IndirectCommandsLayout,
        IndirectCommandsLayoutInfo, Memory, MemoryRequest, Pipeline, PipelineCache, QueryPool,
        QueryPoolInfo, Sampler, SamplerInfo, Semaphore, SparseImageRequirements,
    },
    physical::{Features, PhysicalDevice},
};
use crate::metal::{
    binding::HeapArray,
    command_buffer::Segment,
    error::{Error, Result},
};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::{any::Any, num::NonZeroU64};
use stela_contracts::gate::MemoryRange;

/// Driver work that needs no service state, which a call does after its
/// turn so that its host kernel calls, which wait for the boot processor,
/// hold no other connection's call. It is callable from any processor, one
/// caller at a time (the service's outside lock), never while that caller
/// holds the turn; a closed backend answers [`Error::DeviceLost`] and does
/// nothing else.
pub trait Outside: Send + Sync {
    /// Exports `ranges` and has the driver import them. `None` when it
    /// cannot yet: the import is made in the turn.
    fn prepare_import(self: Arc<Self>, ranges: &[MemoryRange]) -> Result<Option<PreparedImport>>;
    /// Frees memory the backend let go of in its turn, at most a few
    /// allocations a call so that one call's exit stays short; answers how
    /// many.
    fn run_frees(&self) -> usize;
    /// Whether so much waits to be freed that a frontend call should run
    /// [`Self::run_frees`]; below that [`Self::upkeep`] frees it.
    fn frees_pending(&self) -> bool;
    /// The work the backend left for after its turns, one piece at a time
    /// while `more` answers true: frees, destruction of released
    /// semaphores, and new semaphores made ahead for the turns to take.
    /// The platform runs it where a host kernel call costs least (the boot
    /// processor, which runs Linux itself). Answers the pieces done.
    fn upkeep(&self, _more: &mut dyn FnMut() -> bool) -> usize {
        0
    }
    /// Whether [`Self::upkeep`] has work.
    fn upkeep_due(&self) -> bool {
        false
    }
    /// Told that a service runs [`Self::run_frees`] after its calls'
    /// turns; until then the backend frees in its turn.
    fn enable(&self);
    /// Frees a preparation no import adopted.
    fn discard(&self, prepared: Box<dyn Any + Send>);
}

/// An import [`Outside::prepare_import`] made, for
/// [`Backend::adopt_import`]. Dropped unadopted, its memory is freed.
pub struct PreparedImport {
    pub(crate) outside: Arc<dyn Outside>,
    pub(crate) made: Option<Box<dyn Any + Send>>,
}

impl PreparedImport {
    /// Whether `outside` made it.
    pub fn made_by(&self, outside: &Arc<dyn Outside>) -> bool {
        core::ptr::addr_eq(Arc::as_ptr(&self.outside), Arc::as_ptr(outside))
    }

    /// What the backend made, which the backend now owns.
    pub fn take(&mut self) -> Option<Box<dyn Any + Send>> {
        self.made.take()
    }
}

impl Drop for PreparedImport {
    fn drop(&mut self) {
        if let Some(made) = self.made.take() {
            self.outside.discard(made);
        }
    }
}

/// `VkMemoryRequirements`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Requirements {
    pub size: u64,
    pub alignment: u64,
    pub memory_types: u32,
}

/// A buffer with its own memory.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BufferAllocation {
    pub buffer: Buffer,
    pub memory: Memory,
    /// `vkGetBufferDeviceAddress`.
    pub address: u64,
    /// Bytes of memory allocated, for Metal's allocated-size reports.
    pub allocated: u64,
}

/// The memory blocks a backend places buffers and images in, and the
/// resources in memory of their own.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MemoryBlocks {
    pub blocks: usize,
    pub block_bytes: u64,
    /// Resources placed in blocks, and the bytes they take.
    pub placed: usize,
    pub placed_bytes: u64,
    /// Buffers and images in memory of their own.
    pub own: usize,
}

/// A descriptor written into the bindless heap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Descriptor {
    SampledImage(ImageView),
    StorageImage(ImageView),
    Sampler(Sampler),
    UniformTexelBuffer(BufferView),
    StorageTexelBuffer(BufferView),
}

impl Descriptor {
    pub const fn array(&self) -> HeapArray {
        match self {
            Self::SampledImage(_) => HeapArray::SampledImages,
            Self::StorageImage(_) => HeapArray::StorageImages,
            Self::Sampler(_) => HeapArray::Samplers,
            Self::UniformTexelBuffer(_) => HeapArray::UniformTexelBuffers,
            Self::StorageTexelBuffer(_) => HeapArray::StorageTexelBuffers,
        }
    }
}

/// Any object the backend destroys.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Object {
    Buffer(Buffer),
    Image(Image),
    ImageView(ImageView),
    BufferView(BufferView),
    Sampler(Sampler),
    Pipeline(Pipeline),
    PipelineCache(PipelineCache),
    QueryPool(QueryPool),
    Semaphore(Semaphore),
    Memory(Memory),
    IndirectCommandsLayout(IndirectCommandsLayout),
    ExecutionSet(ExecutionSet),
}

pub trait Backend {
    fn physical_device(&self) -> &PhysicalDevice;

    /// Acknowledges that no queued GPU operation can still access this
    /// backend's resources. The owner has closed admission and observed all
    /// accepted submission serials complete before calling. Failure provides
    /// no evidence of idleness; resources and native roots must stay owned.
    /// This does not acknowledge PCI reset/removal or DMA-domain invalidation.
    fn acknowledge_idle(&mut self) -> Result<()> {
        Err(crate::metal::error::Error::Unsupported(
            "GPU idle acknowledgement",
        ))
    }

    /// Memory of its own, zeroed before the host or the GPU reaches it.
    fn allocate_memory(
        &mut self,
        size: u64,
        request: MemoryRequest,
        memory_types: u32,
    ) -> Result<Memory>;
    fn buffer_requirements(&mut self, info: &BufferInfo) -> Result<Requirements>;
    fn image_requirements(&mut self, info: &ImageInfo) -> Result<Requirements>;
    /// A buffer with memory the backend chooses, mapped when the request
    /// says so: a place in a larger allocation, or memory of its own. The
    /// memory starts at least at a 256-byte boundary, where a linear image
    /// over the buffer can bind. All of it reads zero, as Metal's new
    /// buffers do.
    fn create_buffer(
        &mut self,
        info: &BufferInfo,
        request: MemoryRequest,
    ) -> Result<BufferAllocation>;
    /// The wired guest RAM of `ranges`, which the GPU reaches where it is.
    /// Destroying the memory lets [`Self::retired`] answer `token`.
    fn import_memory(&mut self, ranges: &[MemoryRange], token: NonZeroU64) -> Result<Memory>;
    /// Whether an import of exactly `ranges` takes back memory kept from
    /// an earlier one, with no driver call.
    fn keeps_import(&self, _ranges: &[MemoryRange]) -> bool {
        false
    }
    /// The driver work the backend does outside the service's turn
    /// ([`Outside`]); `None` keeps all of it in the turn.
    fn outside(&self) -> Option<alloc::sync::Arc<dyn Outside>> {
        None
    }
    /// [`Self::import_memory`] of memory [`Outside::prepare_import`] made.
    /// Memory another device made is refused, and freed when the refused
    /// preparation drops.
    fn adopt_import(&mut self, _prepared: PreparedImport, _token: NonZeroU64) -> Result<Memory> {
        Err(Error::Unsupported(
            "an import made outside the service's turn",
        ))
    }
    /// The token of imported memory now destroyed.
    fn retired(&mut self) -> Option<NonZeroU64>;
    /// Whether [`Self::retired`] has an answer.
    fn retiring(&self) -> bool;
    /// A buffer bound at `offset` in `memory`; returns its device address.
    fn create_placed_buffer(
        &mut self,
        info: &BufferInfo,
        memory: Memory,
        offset: u64,
    ) -> Result<(Buffer, u64)>;
    /// An image with memory the backend chooses, as [`Self::create_buffer`]
    /// does, its memory and the bytes it needs. Every subresource reads zero
    /// until written.
    fn create_image(
        &mut self,
        info: &ImageInfo,
        request: MemoryRequest,
    ) -> Result<(Image, Memory, u64)>;
    fn create_placed_image(
        &mut self,
        info: &ImageInfo,
        memory: Memory,
        offset: u64,
    ) -> Result<Image>;
    /// Zeroes every subresource of `image`, which `info` created and an
    /// earlier texture used, as [`Self::create_image`] zeroes a new one:
    /// before the next submission's commands, unless they overwrite it
    /// whole first. The earlier texture's work has completed.
    fn clear_reused_image(&mut self, image: Image, info: &ImageInfo);
    /// A sparse-residency color image with no memory bound; segments bind
    /// its blocks.
    fn create_sparse_image(&mut self, info: &ImageInfo)
    -> Result<(Image, SparseImageRequirements)>;
    fn create_image_view(&mut self, image: Image, info: &ImageViewInfo) -> Result<ImageView>;
    fn create_buffer_view(&mut self, buffer: Buffer, info: &BufferViewInfo) -> Result<BufferView>;
    fn create_sampler(&mut self, info: &SamplerInfo) -> Result<Sampler>;
    fn create_graphics_pipeline(&mut self, info: &GraphicsPipelineInfo) -> Result<Pipeline>;
    fn create_compute_pipeline(&mut self, info: &ComputePipelineInfo) -> Result<Pipeline>;
    /// While `deferring` names the lock jobs run under, a pipeline creation
    /// that would compile and that no cache holds fails with
    /// [`DEFERRED`](crate::metal::compiler::DEFERRED) and leaves a job that
    /// compiles it outside the service's turn ([`crate::metal::deferred`]).
    fn defer_pipelines(&mut self, _deferring: Option<&crate::metal::deferred::JobLock>) {}
    /// Creates in the call from here on: the call changed what a second run
    /// would change again.
    fn stop_deferring(&mut self) {}
    /// Whether the call deferred a pipeline creation.
    fn deferred(&self) -> bool {
        false
    }
    /// The pipeline creation the call deferred.
    fn take_job(&mut self) -> Option<crate::metal::deferred::Job> {
        None
    }
    /// Keeps what a job this backend deferred made.
    fn adopt_job(&mut self, _done: crate::metal::deferred::Done) {}
    /// A pipeline cache of `data`, an earlier one's bytes. Bytes of another
    /// device or driver, or of nothing, leave it empty.
    fn create_pipeline_cache(&mut self, data: &[u8]) -> Result<PipelineCache>;
    /// The bytes of `cache`.
    fn pipeline_cache_data(&mut self, cache: PipelineCache) -> Result<Vec<u8>>;
    fn create_query_pool(&mut self, info: &QueryPoolInfo) -> Result<QueryPool>;
    fn create_semaphore(&mut self, initial: u64) -> Result<Semaphore>;
    /// A file descriptor of `semaphore` the platform can wait on outside the
    /// device (a DRM syncobj), which it owns; `None` when the driver has
    /// none to give, and completions are then found by polling.
    fn export_semaphore(&mut self, semaphore: Semaphore) -> Option<i32>;
    /// `vkCreateIndirectCommandsLayoutEXT`; only with
    /// [`Limits::generated_commands`].
    ///
    /// [`Limits::generated_commands`]: super::physical::Limits::generated_commands
    fn create_indirect_commands_layout(
        &mut self,
        info: &IndirectCommandsLayoutInfo,
    ) -> Result<IndirectCommandsLayout>;
    /// `vkCreateIndirectExecutionSetEXT` of up to `capacity` pipelines, with
    /// `initial`, an indirect-bindable pipeline, at index 0.
    fn create_execution_set(&mut self, initial: Pipeline, capacity: u32) -> Result<ExecutionSet>;
    /// `vkUpdateIndirectExecutionSetPipelineEXT` of one index, which no
    /// pending submission uses.
    fn write_execution_set(&mut self, set: ExecutionSet, index: u32, pipeline: Pipeline);
    /// The size and alignment of the preprocess memory `sequences` of
    /// `layout` need (`vkGetGeneratedCommandsMemoryRequirementsEXT`), in
    /// memory of `request`; refused when no memory `request` allows can
    /// hold it.
    fn generated_commands_requirements(
        &mut self,
        layout: IndirectCommandsLayout,
        set: Option<ExecutionSet>,
        sequences: u32,
        request: MemoryRequest,
    ) -> Result<Requirements>;
    fn destroy(&mut self, object: Object);

    /// `vkUpdateDescriptorSets` into heap slot `index`.
    fn write_descriptor(&mut self, index: u32, descriptor: Descriptor);

    /// Whether [`Self::host_word`] gives the host's view of mapped memory,
    /// so the device keeps a completion marker.
    fn maps_words(&self) -> bool {
        false
    }
    /// The host's view of the 32-bit word at `offset` of mapped,
    /// host-coherent `memory`, valid until the memory is freed and readable
    /// from any processor: the device's completion marker. `None` when the
    /// memory is not mapped coherently there.
    fn host_word(
        &mut self,
        _memory: Memory,
        _offset: u64,
    ) -> Option<core::ptr::NonNull<core::sync::atomic::AtomicU32>> {
        None
    }
    /// The host's view of `length` bytes at `offset` of mapped,
    /// host-coherent `memory`, valid until the memory is freed and readable
    /// from any processor once the GPU's writes to it completed: where the
    /// CPU reads what a GPU copy wrote, outside the device's lock. `None`
    /// when the memory is not mapped coherently there.
    fn host_bytes(
        &mut self,
        _memory: Memory,
        _offset: u64,
        _length: u64,
    ) -> Option<core::ptr::NonNull<u8>> {
        None
    }
    /// Copies into mapped memory and flushes it.
    fn write_memory(&mut self, memory: Memory, offset: u64, bytes: &[u8]) -> Result<()>;
    /// Invalidates mapped memory and copies out of it.
    fn read_memory(&mut self, memory: Memory, offset: u64, bytes: &mut [u8]) -> Result<()>;
    /// `vkFlushMappedMemoryRanges` after CPU writes through another mapping.
    fn flush_memory(&mut self, memory: Memory, offset: u64, size: u64) -> Result<()>;
    /// `vkInvalidateMappedMemoryRanges` before CPU reads.
    fn invalidate_memory(&mut self, memory: Memory, offset: u64, size: u64) -> Result<()>;
    /// `vkCopyMemoryToImage` of tightly packed rows into one subresource
    /// region (`hostImageCopy`).
    fn copy_to_image(
        &mut self,
        image: Image,
        region: &super::command::BufferImageCopy,
        bytes: &[u8],
    ) -> Result<()>;
    /// `vkCopyImageToMemory`.
    fn copy_from_image(
        &mut self,
        image: Image,
        region: &super::command::BufferImageCopy,
        bytes: &mut [u8],
    ) -> Result<()>;

    /// `vkResetQueryPool` from the host.
    fn reset_queries(&mut self, pool: QueryPool, first: u32, count: u32);
    /// 64-bit results; `false` while any is still unavailable.
    fn query_results(&mut self, pool: QueryPool, first: u32, results: &mut [u64]) -> Result<bool>;

    fn semaphore_value(&mut self, semaphore: Semaphore) -> Result<u64>;
    /// Every submission signaling `semaphore` at `value` or less is known
    /// done without asking the driver (the completion poll read it
    /// elsewhere): the backend may recycle what those held, as after a
    /// [`Self::semaphore_value`] that answered `value`.
    fn timeline_passed(&mut self, _semaphore: Semaphore, _value: u64) {}
    /// The failure of a submission the driver refused while keeping the
    /// device, named by `value` on `semaphore`, which it still signaled
    /// after its waits; taken once. `None` for every other submission.
    fn submit_failure(&mut self, _semaphore: Semaphore, _value: u64) -> Option<Error> {
        None
    }
    /// Lowers the count [`Self::count_failures`] gave by `settled` taken
    /// failures that were answered and, with `forget`, drops kept ones
    /// nobody will take.
    fn settle_failures(&mut self, _settled: u32, _forget: bool) {}
    /// Has the backend count refused submissions not yet settled in
    /// `outstanding`, which it resets: raised before a refused
    /// submission's signals are made, so whoever sees them made sees it
    /// raised, from any processor. Untouched by a backend that refuses
    /// none.
    fn count_failures(&mut self, _outstanding: &alloc::sync::Arc<core::sync::atomic::AtomicU32>) {}
    fn signal_semaphore(&mut self, semaphore: Semaphore, value: u64) -> Result<()>;
    /// Waits until `semaphore` reaches `value` or `timeout` nanoseconds pass.
    fn wait_semaphore(&mut self, semaphore: Semaphore, value: u64, timeout: u64) -> Result<bool>;

    /// Records each segment into a command buffer and submits them in
    /// order. A segment's sparse binds wait for everything submitted before
    /// it, and its commands wait for the binds.
    fn submit(&mut self, segments: &[Segment]) -> Result<()>;

    /// `vkGetCalibratedTimestampsKHR`: (host nanoseconds, GPU ticks).
    fn calibrated_timestamps(&mut self) -> Result<(u64, u64)>;

    /// Gives the backend the platform's tick counter, which times the
    /// phases of its own calls that [`Self::take_split`] answers.
    fn set_clock(&mut self, _ticks: fn() -> u64) {}

    /// The ticks spent since the last take in phases inside the backend's
    /// own calls, each summed into its slot: [`Self::create_image`]'s image
    /// creation, allocation and binding, and [`Self::submit`]'s reclaim,
    /// recording and queue submission. Zeros for a backend that does not
    /// time them; the device takes before and after the call it splits.
    fn take_split(&mut self) -> [u64; 3] {
        [0; 3]
    }

    /// The blocks [`Self::create_buffer`] and [`Self::create_image`] place
    /// resources in; zeros for a backend that gives each its own memory.
    fn memory_blocks(&self) -> MemoryBlocks {
        MemoryBlocks::default()
    }

    /// Whether `memory` is a whole allocation, bound at GPU addresses of
    /// its own that freeing it unbinds, rather than a place in a block.
    fn whole_memory(&self, _memory: Memory) -> bool {
        true
    }
}

/// A Vulkan feature or extension the Metal layer needs from every device.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Requirement {
    pub name: &'static str,
    pub present: bool,
}

/// The features the binding ABI, dynamic state model and synchronization
/// rely on; a device missing any is refused rather than half-supported.
pub fn requirements(features: &Features) -> [Requirement; 20] {
    let requirement = |name, present| Requirement { name, present };
    [
        requirement("bufferDeviceAddress", features.buffer_device_address),
        requirement("timelineSemaphore", features.timeline_semaphore),
        requirement("synchronization2", features.synchronization2),
        requirement("shaderFloat16", features.shader_float16),
        requirement("dynamicRendering", features.dynamic_rendering),
        requirement(
            "descriptorIndexing",
            features.descriptor_indexing && features.runtime_descriptor_array,
        ),
        requirement(
            "descriptorBindingPartiallyBound",
            features.descriptor_binding_partially_bound,
        ),
        requirement(
            "descriptorBindingUpdateAfterBind",
            features.descriptor_binding_sampled_image_update_after_bind
                && features.descriptor_binding_storage_image_update_after_bind
                && features.descriptor_binding_uniform_texel_buffer_update_after_bind
                && features.descriptor_binding_storage_texel_buffer_update_after_bind
                && features.descriptor_binding_update_unused_while_pending,
        ),
        requirement(
            "shaderImageArrayNonUniformIndexing",
            features.shader_sampled_image_array_non_uniform_indexing
                && features.shader_storage_image_array_non_uniform_indexing,
        ),
        requirement(
            "extendedDynamicState",
            features.extended_dynamic_state && features.extended_dynamic_state2,
        ),
        requirement("unifiedImageLayouts", features.unified_image_layouts),
        requirement(
            "maintenance5",
            features.maintenance4 && features.maintenance5,
        ),
        requirement("vulkanMemoryModel", features.vulkan_memory_model),
        requirement(
            "storageImageWithoutFormat",
            features.shader_storage_image_read_without_format
                && features.shader_storage_image_write_without_format,
        ),
        requirement("independentBlend", features.independent_blend),
        requirement("nullDescriptor", features.null_descriptor),
        requirement("conditionalRendering", features.conditional_rendering),
        requirement(
            "dynamicRenderingUnusedAttachments",
            features.dynamic_rendering_unused_attachments,
        ),
        requirement(
            "dynamicPrimitiveTopologyUnrestricted",
            features.dynamic_primitive_topology_unrestricted,
        ),
        requirement("hostQueryReset", features.host_query_reset),
    ]
}
