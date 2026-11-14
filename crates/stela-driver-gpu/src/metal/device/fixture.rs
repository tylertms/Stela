//! A device over a recording Vulkan backend and a compiler that reflects
//! functions by name, for the tests of the device and the layers above it.

use super::{Device, LibraryId};
use crate::metal::deferred::{Done, Job, JobLock, Made, Owner};
use crate::metal::{
    abi::{DataType, FunctionType, TextureType},
    capabilities::ShaderFeatures,
    command_buffer::Segment,
    error::{Error, NO_FUNCTION, Result},
    library::tests::fixture,
    nvidia::{CHIPS, Chip, PcieLink, fixture::nvk},
    reflection::{
        ArrayType, Binding, BindingAccess, BindingKind, PointerType, StructMember, StructType,
        TextureReferenceType, Type,
    },
    shader::{
        Access, BufferUse, CompiledShader, Compiler, FunctionConstant, Interface, LinkedFunction,
        Patch, PatchDomain, Provenance, Reflection, Scalar, Specialization, Stage, Tessellation,
        TessellationStages, Varying, tests::interface,
    },
    stitching::StitchedGraph,
    vulkan::{
        backend::{
            Backend, BufferAllocation, Descriptor, Object, Outside, PreparedImport, Requirements,
        },
        command::BufferImageCopy,
        object::{
            Buffer, BufferInfo, BufferView, BufferViewInfo, ComputePipelineInfo, ExecutionSet,
            GraphicsPipelineInfo, Image, ImageInfo, ImageView, ImageViewInfo,
            IndirectCommandsLayout, IndirectCommandsLayoutInfo, Memory, MemoryRequest, Pipeline,
            PipelineCache, PipelineSource, QueryPool, QueryPoolInfo, Sampler, SamplerInfo,
            Semaphore, ShaderCode, SparseImageRequirements,
        },
        physical::PhysicalDevice,
    },
};
use alloc::sync::Arc;
use alloc::{
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    string::String,
    vec::Vec,
};
use core::{
    num::NonZeroU64,
    sync::atomic::{AtomicBool, Ordering},
};
use stela_contracts::gate::MemoryRange;

/// A deferred job that runs until `gate` opens, as a long compilation.
fn gated_job(key: u8, owner: Owner, gate: Arc<AtomicBool>) -> Job {
    Job::new([key; 32], owner, move || {
        while !gate.load(Ordering::Acquire) {
            core::hint::spin_loop();
        }
        Box::new(()) as Made
    })
}

/// [`Outside`] of the mock: a preparation waits while `hold` is set, as the
/// driver's import waits for the boot processor, and counts what it did.
#[derive(Default)]
pub(crate) struct MockOutside {
    pub(crate) hold: core::sync::atomic::AtomicBool,
    pub(crate) preparing: core::sync::atomic::AtomicBool,
    pub(crate) prepared: core::sync::atomic::AtomicUsize,
    pub(crate) discarded: core::sync::atomic::AtomicUsize,
    /// Frees waiting, and how many times they ran.
    pub(crate) pending: core::sync::atomic::AtomicUsize,
    pub(crate) ran: core::sync::atomic::AtomicUsize,
    pub(crate) enabled: core::sync::atomic::AtomicBool,
    /// Pieces of upkeep waiting, and those done.
    pub(crate) chores: core::sync::atomic::AtomicUsize,
    pub(crate) upkept: core::sync::atomic::AtomicUsize,
}

impl MockOutside {
    pub(crate) fn frees_pending_now(&self) -> bool {
        self.pending.load(core::sync::atomic::Ordering::SeqCst) != 0
    }
}

impl Outside for MockOutside {
    fn prepare_import(
        self: alloc::sync::Arc<Self>,
        ranges: &[MemoryRange],
    ) -> Result<Option<PreparedImport>> {
        use core::sync::atomic::Ordering;
        self.preparing.store(true, Ordering::SeqCst);
        while self.hold.load(Ordering::SeqCst) {
            core::hint::spin_loop();
        }
        self.preparing.store(false, Ordering::SeqCst);
        self.prepared.fetch_add(1, Ordering::SeqCst);
        Ok(Some(PreparedImport {
            outside: self as alloc::sync::Arc<dyn Outside>,
            made: Some(Box::new(ranges.to_vec())),
        }))
    }
    fn run_frees(&self) -> usize {
        use core::sync::atomic::Ordering;
        self.ran.fetch_add(1, Ordering::SeqCst);
        self.pending.swap(0, Ordering::SeqCst)
    }
    fn frees_pending(&self) -> bool {
        self.pending.load(core::sync::atomic::Ordering::SeqCst) != 0
    }
    fn upkeep(&self, more: &mut dyn FnMut() -> bool) -> usize {
        use core::sync::atomic::Ordering;
        let mut done = 0;
        while self.chores.load(Ordering::SeqCst) != 0 && more() {
            self.chores.fetch_sub(1, Ordering::SeqCst);
            self.upkept.fetch_add(1, Ordering::SeqCst);
            done += 1;
        }
        done
    }
    fn upkeep_due(&self) -> bool {
        self.chores.load(core::sync::atomic::Ordering::SeqCst) != 0
    }
    fn enable(&self) {
        self.enabled
            .store(true, core::sync::atomic::Ordering::SeqCst);
    }
    fn discard(&self, _: Box<dyn core::any::Any + Send>) {
        self.discarded
            .fetch_add(1, core::sync::atomic::Ordering::SeqCst);
    }
}

#[derive(Default)]
pub(crate) struct MockBackend {
    pub(crate) idle_calls: usize,
    pub(crate) idle_error: Option<Error>,
    pub(crate) semaphore_error: Option<Error>,
    /// Timeline values the device told the backend passed without a query,
    /// in order.
    pub(crate) passed: Vec<(Semaphore, u64)>,
    /// Semaphores whose value the driver was asked for, in order.
    pub(crate) queried: Vec<Semaphore>,
    /// Freed semaphores' handles, which `create_semaphore` hands out again
    /// when `reuse_semaphores` is set.
    pub(crate) reuse_semaphores: bool,
    pub(crate) freed_semaphores: Vec<u64>,
    /// The next submission is refused with this failure while the device
    /// is kept: its signals are still made, as NVK's refusal makes them.
    pub(crate) refuse_next: Option<Error>,
    /// Submissions the driver refused, by each value they signaled, as
    /// `submit_failure` takes them.
    pub(crate) refused: Vec<(Semaphore, u64, Error)>,
    /// The count `count_failures` gave, raised by each refused record.
    pub(crate) outstanding: Option<alloc::sync::Arc<core::sync::atomic::AtomicU32>>,
    pub(crate) physical: Option<PhysicalDevice>,
    pub(crate) next: u64,
    pub(crate) destroyed: Vec<Object>,
    /// Images zeroed for reuse, in order.
    pub(crate) cleared: Vec<Image>,
    pub(crate) descriptors: Vec<(u32, Descriptor)>,
    pub(crate) graphics: Vec<GraphicsPipelineInfo>,
    pub(crate) compute: Vec<ComputePipelineInfo>,
    /// The pipelines each pipeline cache holds.
    pub(crate) caches: BTreeMap<u64, u8>,
    /// While a call may defer, the next compute pipeline creation defers
    /// to a job that runs until this opens.
    pub(crate) pipeline_gate: Option<Arc<AtomicBool>>,
    /// While a call may defer, the next image creation leaves its memory to
    /// a job that runs until this opens, as the driver's kernel allocation
    /// waits for Linux.
    pub(crate) image_gate: Option<Arc<AtomicBool>>,
    deferring: Option<JobLock>,
    job: Option<Job>,
    /// Pipeline jobs adopted.
    pub(crate) adopted: usize,
    pub(crate) submissions: Vec<Vec<Segment>>,
    pub(crate) writes: Vec<(Memory, u64, Vec<u8>)>,
    pub(crate) write_error: Option<Error>,
    pub(crate) invalidated: Vec<(Memory, u64, u64)>,
    /// Host memory each imported memory is, by memory.
    pub(crate) imports: BTreeMap<u64, (Vec<MemoryRange>, NonZeroU64)>,
    /// Driver work outside the service's turn, when a test gives it.
    pub(crate) outside: Option<alloc::sync::Arc<MockOutside>>,
    /// The word a test's GPU writes as the completion marker, when the
    /// test gives one: every mapped word answers it.
    pub(crate) marker: Option<Box<core::sync::atomic::AtomicU32>>,
    /// The descriptor each export of a semaphore answers, when a test gives
    /// one.
    pub(crate) exported: Option<i32>,
    pub(crate) retired: VecDeque<NonZeroU64>,
    pub(crate) semaphores: BTreeMap<u64, u64>,
    /// The highest value each semaphore's submitted signals reach.
    pub(crate) promised: BTreeMap<u64, u64>,
    /// Waits submitted on a value neither reached nor promised by work
    /// submitted before: a driver holds the queue there, every submission
    /// behind it included, until a signal arrives.
    pub(crate) unpromised: Vec<(Semaphore, u64)>,
    pub(crate) queries_ready: bool,
    pub(crate) query_pool_error: Option<Error>,
    pub(crate) query_pools: Vec<(QueryPool, QueryPoolInfo)>,
    pub(crate) query_resets: Vec<(QueryPool, u32, u32)>,
    pub(crate) query_reads: Vec<(QueryPool, u32, usize)>,
    pub(crate) layouts: Vec<IndirectCommandsLayoutInfo>,
    /// Each execution set's pipelines by index.
    pub(crate) execution_sets: BTreeMap<u64, BTreeMap<u32, Pipeline>>,
    /// The bytes of each host image copy into an image, which copies out
    /// of it read back.
    pub(crate) image_bytes: BTreeMap<u64, Vec<u8>>,
    /// The host's view of each memory mapped through `host_bytes`, which a
    /// submitted copy of an image to its buffer fills from `image_bytes`,
    /// as the GPU would.
    pub(crate) host: BTreeMap<u64, Box<[u8]>>,
}

impl MockBackend {
    pub(crate) fn new() -> Self {
        Self {
            physical: Some(nvk(&CHIPS[0], 8 << 30)),
            ..Self::default()
        }
    }

    fn id(&mut self) -> u64 {
        self.next += 1;
        self.next
    }

    /// Finds or keeps a pipeline in the cache of `source`.
    fn source(&mut self, source: PipelineSource) -> Result<()> {
        match source {
            PipelineSource::Compiled => {}
            PipelineSource::Cached(cache) => *self.caches.get_mut(&cache.0).unwrap() += 1,
            PipelineSource::Archived(cache) if self.caches[&cache.0] == 0 => {
                return Err(Error::NotArchived);
            }
            PipelineSource::Archived(_) => {}
        }
        Ok(())
    }

    /// The GPU runs everything submitted: each signal reaches its value.
    pub(crate) fn finish(&mut self) {
        for (&semaphore, &promised) in &self.promised {
            let value = self.semaphores.entry(semaphore).or_default();
            *value = (*value).max(promised);
        }
    }

    /// Completes everything submitted so far.
    pub(crate) fn complete(&mut self, timeline: Semaphore, serial: u64) {
        self.semaphores.insert(timeline.0, serial);
    }
}

impl Backend for MockBackend {
    fn acknowledge_idle(&mut self) -> Result<()> {
        self.idle_calls += 1;
        self.idle_error.map_or(Ok(()), Err)
    }

    fn physical_device(&self) -> &PhysicalDevice {
        self.physical.as_ref().unwrap()
    }
    fn allocate_memory(&mut self, _: u64, _: MemoryRequest, _: u32) -> Result<Memory> {
        Ok(Memory(self.id()))
    }
    fn buffer_requirements(&mut self, info: &BufferInfo) -> Result<Requirements> {
        Ok(Requirements {
            size: info.size,
            alignment: 256,
            memory_types: u32::MAX,
        })
    }
    fn image_requirements(&mut self, info: &ImageInfo) -> Result<Requirements> {
        let size = u64::from(info.extent[0]) * u64::from(info.extent[1]) * 4;
        Ok(Requirements {
            size,
            alignment: 65536,
            memory_types: u32::MAX,
        })
    }
    fn create_buffer(&mut self, info: &BufferInfo, _: MemoryRequest) -> Result<BufferAllocation> {
        let id = self.id();
        Ok(BufferAllocation {
            buffer: Buffer(id),
            memory: Memory(id),
            address: id << 32,
            allocated: info.size,
        })
    }
    fn import_memory(&mut self, ranges: &[MemoryRange], token: NonZeroU64) -> Result<Memory> {
        let id = self.id();
        self.imports.insert(id, (ranges.to_vec(), token));
        Ok(Memory(id))
    }
    fn outside(&self) -> Option<alloc::sync::Arc<dyn Outside>> {
        self.outside
            .clone()
            .map(|outside| outside as alloc::sync::Arc<dyn Outside>)
    }
    fn adopt_import(&mut self, mut prepared: PreparedImport, token: NonZeroU64) -> Result<Memory> {
        let ranges = prepared
            .take()
            .and_then(|made| made.downcast::<Vec<MemoryRange>>().ok())
            .ok_or(Error::Invalid("an import this backend did not make"))?;
        let id = self.id();
        self.imports.insert(id, (*ranges, token));
        Ok(Memory(id))
    }
    fn retired(&mut self) -> Option<NonZeroU64> {
        self.retired.pop_front()
    }
    fn retiring(&self) -> bool {
        !self.retired.is_empty()
    }
    fn create_placed_buffer(
        &mut self,
        _: &BufferInfo,
        _: Memory,
        offset: u64,
    ) -> Result<(Buffer, u64)> {
        let id = self.id();
        Ok((Buffer(id), (id << 32) + offset))
    }
    fn create_image(
        &mut self,
        info: &ImageInfo,
        request: MemoryRequest,
    ) -> Result<(Image, Memory, u64)> {
        if let Some(lock) = &self.deferring
            && let Some(gate) = self.image_gate.take()
        {
            lock.lock.set_deferred(true);
            self.job = Some(gated_job(3, Owner::Backend, gate));
            return Err(crate::metal::compiler::DEFERRED);
        }
        let requirements = self.image_requirements(info)?;
        let memory = self.allocate_memory(requirements.size, request, u32::MAX)?;
        Ok((Image(self.id()), memory, requirements.size))
    }
    fn create_placed_image(&mut self, _: &ImageInfo, _: Memory, _: u64) -> Result<Image> {
        Ok(Image(self.id()))
    }
    fn clear_reused_image(&mut self, image: Image, _: &ImageInfo) {
        self.cleared.push(image);
    }
    /// RGBA8's standard block at the image's sample count, with each
    /// layer's tail of one block from level 2.
    fn create_sparse_image(
        &mut self,
        info: &ImageInfo,
    ) -> Result<(Image, SparseImageRequirements)> {
        let halvings = info.samples.bits().trailing_zeros();
        let requirements = SparseImageRequirements {
            memory_types: u32::MAX,
            granularity: [128 >> halvings.div_ceil(2), 128 >> (halvings / 2), 1],
            mip_tail_first_lod: 2,
            mip_tail_size: 64 << 10,
            mip_tail_offset: 1 << 20,
            mip_tail_stride: 64 << 10,
            single_mip_tail: false,
        };
        Ok((Image(self.id()), requirements))
    }
    fn create_image_view(&mut self, _: Image, _: &ImageViewInfo) -> Result<ImageView> {
        Ok(ImageView(self.id()))
    }
    fn create_buffer_view(&mut self, _: Buffer, _: &BufferViewInfo) -> Result<BufferView> {
        Ok(BufferView(self.id()))
    }
    fn create_sampler(&mut self, _: &SamplerInfo) -> Result<Sampler> {
        Ok(Sampler(self.id()))
    }
    fn defer_pipelines(&mut self, deferring: Option<&JobLock>) {
        self.deferring = deferring.cloned();
    }
    fn stop_deferring(&mut self) {
        self.deferring = None;
    }
    fn deferred(&self) -> bool {
        self.job.is_some()
    }
    fn take_job(&mut self) -> Option<Job> {
        self.job.take()
    }
    fn adopt_job(&mut self, _: Done) {
        self.adopted += 1;
    }
    fn create_pipeline_cache(&mut self, data: &[u8]) -> Result<PipelineCache> {
        let id = self.id();
        self.caches.insert(id, data.first().copied().unwrap_or(0));
        Ok(PipelineCache(id))
    }
    fn pipeline_cache_data(&mut self, cache: PipelineCache) -> Result<Vec<u8>> {
        Ok(alloc::vec![self.caches[&cache.0]])
    }
    fn create_graphics_pipeline(&mut self, info: &GraphicsPipelineInfo) -> Result<Pipeline> {
        self.source(info.source)?;
        self.graphics.push(info.clone());
        Ok(Pipeline(self.id()))
    }
    fn create_compute_pipeline(&mut self, info: &ComputePipelineInfo) -> Result<Pipeline> {
        if let Some(lock) = &self.deferring
            && info.source == PipelineSource::Compiled
            && let Some(gate) = self.pipeline_gate.take()
        {
            lock.lock.set_deferred(true);
            self.job = Some(gated_job(2, Owner::Backend, gate));
            return Err(crate::metal::compiler::DEFERRED);
        }
        self.source(info.source)?;
        self.compute.push(*info);
        Ok(Pipeline(self.id()))
    }
    fn create_query_pool(&mut self, info: &QueryPoolInfo) -> Result<QueryPool> {
        if let Some(error) = self.query_pool_error.take() {
            return Err(error);
        }
        let pool = QueryPool(self.id());
        self.query_pools.push((pool, *info));
        Ok(pool)
    }
    fn create_semaphore(&mut self, initial: u64) -> Result<Semaphore> {
        // A driver may hand out a freed semaphore's handle again.
        let id = match self.reuse_semaphores {
            true => self.freed_semaphores.pop().unwrap_or_else(|| self.id()),
            false => self.id(),
        };
        self.semaphores.insert(id, initial);
        Ok(Semaphore(id))
    }
    fn export_semaphore(&mut self, _: Semaphore) -> Option<i32> {
        self.exported
    }
    fn create_indirect_commands_layout(
        &mut self,
        info: &IndirectCommandsLayoutInfo,
    ) -> Result<IndirectCommandsLayout> {
        self.layouts.push(info.clone());
        Ok(IndirectCommandsLayout(self.id()))
    }
    fn create_execution_set(&mut self, initial: Pipeline, _: u32) -> Result<ExecutionSet> {
        let id = self.id();
        self.execution_sets
            .insert(id, BTreeMap::from([(0, initial)]));
        Ok(ExecutionSet(id))
    }
    fn write_execution_set(&mut self, set: ExecutionSet, index: u32, pipeline: Pipeline) {
        self.execution_sets
            .get_mut(&set.0)
            .unwrap()
            .insert(index, pipeline);
    }
    fn generated_commands_requirements(
        &mut self,
        _: IndirectCommandsLayout,
        _: Option<ExecutionSet>,
        sequences: u32,
        _: MemoryRequest,
    ) -> Result<Requirements> {
        Ok(Requirements {
            size: u64::from(sequences) * 64,
            alignment: 256,
            memory_types: u32::MAX,
        })
    }
    fn destroy(&mut self, object: Object) {
        if let Object::Semaphore(Semaphore(id)) = object {
            self.freed_semaphores.push(id);
        }
        if let Object::Memory(Memory(id)) = object
            && let Some((_, token)) = self.imports.remove(&id)
        {
            self.retired.push_back(token);
        }
        self.destroyed.push(object);
    }
    fn write_descriptor(&mut self, index: u32, descriptor: Descriptor) {
        self.descriptors.push((index, descriptor));
    }
    fn maps_words(&self) -> bool {
        self.marker.is_some()
    }
    fn host_word(
        &mut self,
        _: Memory,
        _: u64,
    ) -> Option<core::ptr::NonNull<core::sync::atomic::AtomicU32>> {
        self.marker.as_deref().map(core::ptr::NonNull::from)
    }
    fn host_bytes(
        &mut self,
        memory: Memory,
        offset: u64,
        length: u64,
    ) -> Option<core::ptr::NonNull<u8>> {
        let end = usize::try_from(offset.checked_add(length)?).ok()?;
        let bytes = self
            .host
            .entry(memory.0)
            .or_insert_with(|| alloc::vec![0; end].into_boxed_slice());
        let place = bytes.get_mut(offset as usize..end)?;
        core::ptr::NonNull::new(place.as_mut_ptr())
    }
    fn write_memory(&mut self, memory: Memory, offset: u64, bytes: &[u8]) -> Result<()> {
        if let Some(error) = self.write_error.take() {
            return Err(error);
        }
        self.writes.push((memory, offset, bytes.to_vec()));
        Ok(())
    }
    /// Reads back the latest write to each byte.
    fn read_memory(&mut self, memory: Memory, offset: u64, bytes: &mut [u8]) -> Result<()> {
        for (written, at, data) in &self.writes {
            for (index, byte) in bytes.iter_mut().enumerate() {
                let address = offset + index as u64;
                if *written == memory && (*at..*at + data.len() as u64).contains(&address) {
                    *byte = data[(address - at) as usize];
                }
            }
        }
        Ok(())
    }
    fn flush_memory(&mut self, _: Memory, _: u64, _: u64) -> Result<()> {
        Ok(())
    }
    fn invalidate_memory(&mut self, memory: Memory, offset: u64, size: u64) -> Result<()> {
        self.invalidated.push((memory, offset, size));
        Ok(())
    }
    fn copy_to_image(&mut self, image: Image, _: &BufferImageCopy, bytes: &[u8]) -> Result<()> {
        self.image_bytes.insert(image.0, bytes.to_vec());
        Ok(())
    }
    fn copy_from_image(
        &mut self,
        image: Image,
        _: &BufferImageCopy,
        bytes: &mut [u8],
    ) -> Result<()> {
        if let Some(written) = self.image_bytes.get(&image.0) {
            let count = written.len().min(bytes.len());
            bytes[..count].copy_from_slice(&written[..count]);
        }
        Ok(())
    }
    fn reset_queries(&mut self, pool: QueryPool, first: u32, count: u32) {
        self.query_resets.push((pool, first, count));
    }
    fn query_results(&mut self, pool: QueryPool, first: u32, results: &mut [u64]) -> Result<bool> {
        self.query_reads.push((pool, first, results.len()));
        results
            .iter_mut()
            .enumerate()
            .for_each(|(index, result)| *result = index as u64 * 10);
        Ok(self.queries_ready)
    }
    fn submit_failure(&mut self, semaphore: Semaphore, value: u64) -> Option<Error> {
        let index = self
            .refused
            .iter()
            .position(|&(refused, at, _)| refused == semaphore && at == value)?;
        Some(self.refused.remove(index).2)
    }
    fn settle_failures(&mut self, mut settled: u32, forget: bool) {
        if forget {
            settled += self.refused.len() as u32;
            self.refused.clear();
        }
        if let Some(outstanding) = &self.outstanding {
            outstanding.fetch_sub(settled, core::sync::atomic::Ordering::AcqRel);
        }
    }
    fn count_failures(&mut self, outstanding: &alloc::sync::Arc<core::sync::atomic::AtomicU32>) {
        outstanding.store(0, core::sync::atomic::Ordering::Release);
        self.outstanding = Some(outstanding.clone());
    }
    fn semaphore_value(&mut self, semaphore: Semaphore) -> Result<u64> {
        self.queried.push(semaphore);
        if let Some(error) = self.semaphore_error.take() {
            return Err(error);
        }
        Ok(self.semaphores[&semaphore.0])
    }
    fn timeline_passed(&mut self, semaphore: Semaphore, value: u64) {
        self.passed.push((semaphore, value));
    }
    fn signal_semaphore(&mut self, semaphore: Semaphore, value: u64) -> Result<()> {
        self.semaphores.insert(semaphore.0, value);
        Ok(())
    }
    fn wait_semaphore(&mut self, semaphore: Semaphore, value: u64, _: u64) -> Result<bool> {
        Ok(self.semaphores[&semaphore.0] >= value)
    }
    fn submit(&mut self, segments: &[Segment]) -> Result<()> {
        for segment in segments {
            for &(semaphore, value) in &segment.waits {
                let reached = self.semaphores.get(&semaphore.0).copied().unwrap_or(0);
                let promised = self.promised.get(&semaphore.0).copied().unwrap_or(0);
                if reached.max(promised) < value {
                    self.unpromised.push((semaphore, value));
                }
            }
            for &(semaphore, value) in &segment.signals {
                let promised = self.promised.entry(semaphore.0).or_default();
                *promised = (*promised).max(value);
            }
        }
        if let Some(error) = self.refuse_next.take() {
            for segment in segments {
                for &(semaphore, value) in &segment.signals {
                    self.refused.push((semaphore, value, error));
                    if let Some(outstanding) = &self.outstanding {
                        outstanding.fetch_add(1, core::sync::atomic::Ordering::AcqRel);
                    }
                }
            }
        }
        // A buffer and its memory share an id: a copy of an image the host
        // wrote to a mapped buffer lands where the host reads it.
        for command in segments.iter().flat_map(|segment| &segment.commands) {
            if let crate::metal::vulkan::command::Command::CopyImageToBuffer { src, dst, region } =
                command
                && let (Some(written), Some(host)) =
                    (self.image_bytes.get(&src.0), self.host.get_mut(&dst.0))
                && let Some(place) = host.get_mut(region.buffer_offset as usize..)
            {
                let count = written.len().min(place.len());
                place[..count].copy_from_slice(&written[..count]);
            }
        }
        self.submissions.push(segments.to_vec());
        Ok(())
    }
    fn calibrated_timestamps(&mut self) -> Result<(u64, u64)> {
        Ok((1, 2))
    }
}

/// Reflects functions by name: `"kernel"`, `"vertex"`, `"fragment"`,
/// `"object"`, `"mesh"`, the post-tessellation `"patch"`, `"unlinked"`, a
/// fragment function reading a varying nothing outputs, `"constants"`, a
/// kernel with a required and an optional function constant, and the
/// visible functions `"visible"`, `"barriers"`, whose module sets indirect
/// commands' barriers, and `"tinted"`, whose module has a function
/// constant. Library `n` is the `n`-th added, `None` once removed.
#[derive(Default)]
pub(crate) struct MockCompiler {
    /// Each library's bytes; a shared library holds the same ones, as the
    /// AIR compiler's does.
    pub(crate) libraries: Vec<Option<alloc::sync::Arc<Vec<u8>>>>,
    pub(crate) compiled: Vec<Specialization>,
    pub(crate) tessellations: Vec<Tessellation>,
    pub(crate) released: Vec<ShaderCode>,
    /// The next compilation defers, as the AIR compiler's does when the
    /// service runs the call again.
    pub(crate) defer_next: bool,
    /// A compilation deferred, until the test takes it.
    pub(crate) deferred: bool,
    /// While a call may defer, the deferred compilation leaves a job that
    /// runs until this opens.
    pub(crate) job_gate: Option<Arc<AtomicBool>>,
    deferring: bool,
    job: Option<Job>,
    /// Jobs adopted.
    pub(crate) adopted: usize,
}

/// Every function's bindings: uniforms holding a matrix and an array of
/// structures, an argument buffer of a texture and a buffer, and a sampler.
pub(crate) fn bound() -> Vec<Binding> {
    let member = |name: &str, offset, ty, argument_index| StructMember {
        name: String::from(name),
        offset,
        ty,
        argument_index,
    };
    let inner = StructType {
        members: alloc::vec![
            member("uv", 0, Type::Data(DataType::Float2), 0),
            member("h", 8, Type::Data(DataType::Half), 0),
        ],
    };
    let uniforms = StructType {
        members: alloc::vec![
            member("mvp", 0, Type::Data(DataType::Float4x4), 0),
            member(
                "inner",
                64,
                Type::Array(ArrayType {
                    element: Box::new(Type::Struct(inner)),
                    length: 2,
                    stride: 16,
                    argument_index_stride: 0,
                }),
                0,
            ),
        ],
    };
    let texture = TextureReferenceType {
        data_type: DataType::Float,
        texture_type: TextureType::D2,
        access: BindingAccess::ReadOnly,
        depth: false,
    };
    let arguments = StructType {
        members: alloc::vec![
            member("t", 0, Type::Texture(texture), 0),
            member(
                "d",
                8,
                Type::Pointer(PointerType {
                    element: Box::new(Type::Data(DataType::Float)),
                    access: BindingAccess::ReadWrite,
                    alignment: 4,
                    data_size: 4,
                    element_is_argument_buffer: false,
                }),
                1,
            ),
        ],
    };
    let buffer = |name: &str, index, element, alignment, data_size, argument_buffer| Binding {
        name: String::from(name),
        index,
        access: BindingAccess::ReadOnly,
        used: true,
        kind: BindingKind::Buffer(PointerType {
            element: Box::new(element),
            access: BindingAccess::ReadOnly,
            alignment,
            data_size,
            element_is_argument_buffer: argument_buffer,
        }),
    };
    alloc::vec![
        buffer("u", 0, Type::Struct(uniforms), 16, 96, false),
        buffer("args", 1, Type::Struct(arguments), 8, 16, true),
        Binding {
            name: String::from("s"),
            index: 0,
            access: BindingAccess::ReadOnly,
            used: false,
            kind: BindingKind::Sampler { array_length: 1 },
        },
    ]
}

fn reflected(entry: &str) -> Result<Interface> {
    let stage = match entry {
        "kernel" => Stage::Kernel,
        "vertex" | "patch" => Stage::Vertex,
        "fragment" | "unlinked" | "filtering" => Stage::Fragment,
        "object" => Stage::Object,
        "mesh" => Stage::Mesh,
        "constants" | "sampling" => Stage::Kernel,
        _ => return Err(NO_FUNCTION),
    };
    let mut interface = interface(stage);
    if entry == "patch" {
        interface.patch = Some(Patch {
            domain: PatchDomain::Quad,
            control_points: Some(4),
        });
    }
    let varying = |identifier: &str, per_primitive| Varying {
        identifier: String::from(identifier),
        scalar: Scalar::Float,
        components: 4,
        per_primitive,
    };
    interface.varyings = match entry {
        "vertex" | "patch" | "fragment" | "filtering" => {
            alloc::vec![varying("user(color)", false)]
        }
        "mesh" => alloc::vec![varying("user(color)", false), varying("user(normal)", true)],
        "unlinked" => alloc::vec![varying("user(missing)", false)],
        _ => Vec::new(),
    };
    // A fragment function sampling through the sampler at index 1.
    if entry == "filtering" {
        interface.samplers = alloc::vec![1];
    }
    if entry == "constants" {
        let constant = |name: &str, data_type, index, required| FunctionConstant {
            name: String::from(name),
            data_type,
            index,
            required,
        };
        interface.constants = alloc::vec![
            constant("scale", DataType::UInt, 0, true),
            constant("tint", DataType::Half3, 1, false),
        ];
    }
    if stage == Stage::Kernel {
        interface.buffers = alloc::vec![BufferUse {
            index: 0,
            access: Access::READ | Access::WRITE,
            argument_buffer: false
        }];
    }
    Ok(interface)
}

impl Compiler for MockCompiler {
    fn features(&self) -> ShaderFeatures {
        ShaderFeatures::KNOWN
    }
    fn deferred(&self) -> bool {
        self.deferred
    }
    fn defer_compiles(&mut self, deferring: bool) {
        self.deferring = deferring;
    }
    fn take_job(&mut self) -> Option<Job> {
        self.deferred = false;
        self.job.take()
    }
    fn adopt_job(&mut self, _: Done) {
        self.adopted += 1;
    }
    fn add_library(&mut self, data: Vec<u8>) -> Result<u64> {
        self.libraries.push(Some(alloc::sync::Arc::new(data)));
        Ok(self.libraries.len() as u64 - 1)
    }
    fn share_library(&mut self, library: u64) -> Result<u64> {
        let data = self.libraries[library as usize]
            .clone()
            .ok_or(Error::Invalid("unknown library"))?;
        self.libraries.push(Some(data));
        Ok(self.libraries.len() as u64 - 1)
    }
    fn functions(&mut self, _: &[u8]) -> Result<Vec<(String, FunctionType)>> {
        Ok(alloc::vec![
            (String::from("kernel"), FunctionType::Kernel),
            (String::from("visible"), FunctionType::Visible),
        ])
    }
    fn exports(&mut self, _: &[u8]) -> Result<Vec<String>> {
        Ok(alloc::vec![String::from("_Z6helperf")])
    }
    fn stitch(&mut self, _: &[LinkedFunction], _: &[StitchedGraph]) -> Result<Vec<u8>> {
        Ok(b"BC\xc0\xde\0\0\0\0".to_vec())
    }
    fn library_data(&self, library: u64) -> Result<&[u8]> {
        self.libraries[library as usize]
            .as_deref()
            .map(Vec::as_slice)
            .ok_or(Error::Invalid("unknown library"))
    }
    fn remove_library(&mut self, library: u64) -> Result<()> {
        self.libraries[library as usize]
            .take()
            .map(drop)
            .ok_or(Error::Invalid("unknown library"))
    }
    fn constants(&mut self, library: u64, entry: &str) -> Result<Vec<FunctionConstant>> {
        let reflection = self.reflect(library, entry, &Specialization::default())?;
        Ok(reflection.constants().to_vec())
    }
    fn reflect(&mut self, _: u64, entry: &str, _: &Specialization) -> Result<Reflection> {
        match entry {
            "visible" => Ok(Reflection::Visible {
                constants: Vec::new(),
                command_barriers: false,
            }),
            "barriers" => Ok(Reflection::Visible {
                constants: Vec::new(),
                command_barriers: true,
            }),
            "tinted" => Ok(Reflection::Visible {
                constants: alloc::vec![FunctionConstant {
                    name: String::from("tint"),
                    data_type: DataType::Float,
                    index: 0,
                    required: false,
                }],
                command_barriers: false,
            }),
            _ => reflected(entry).map(|interface| Reflection::Stage(Box::new(interface))),
        }
    }
    fn bindings(
        &mut self,
        _: u64,
        entry: &str,
        _: Option<&Specialization>,
    ) -> Result<Vec<Binding>> {
        reflected(entry).map(|_| bound())
    }
    fn sampler_states(&mut self, _: u64, entry: &str, _: &Specialization) -> Result<Vec<u64>> {
        // The default state, and filter::linear with the default's state.
        Ok(match entry {
            "sampling" => alloc::vec![0x007b_ff00_0008_0049, 0x007b_ff00_0008_0a49],
            _ => Vec::new(),
        })
    }
    fn compile(
        &mut self,
        _: u64,
        entry: &str,
        specialization: &Specialization,
    ) -> Result<CompiledShader> {
        if core::mem::take(&mut self.defer_next) {
            self.deferred = true;
            if self.deferring
                && let Some(gate) = self.job_gate.take()
            {
                self.job = Some(gated_job(1, Owner::Compiler, gate));
            }
            return Err(crate::metal::compiler::DEFERRED);
        }
        self.compiled.push(specialization.clone());
        Ok(CompiledShader {
            code: ShaderCode(self.compiled.len() as u64),
            interface: reflected(entry)?,
            provenance: Provenance::default(),
        })
    }
    fn compile_tessellation(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
        tessellation: &Tessellation,
    ) -> Result<TessellationStages> {
        self.tessellations.push(tessellation.clone());
        let evaluation = self.compile(library, entry, specialization)?;
        let base = evaluation.code.0;
        Ok(TessellationStages {
            vertex: ShaderCode(base + 100),
            control: ShaderCode(base + 200),
            evaluation,
        })
    }
    fn release(&mut self, code: ShaderCode) {
        self.released.push(code);
    }
}

pub(crate) type TestDevice = Device<MockBackend, MockCompiler>;

/// The library [`device`] opens with, whose functions [`MockCompiler`]
/// reflects.
pub(crate) const LIBRARY: LibraryId = LibraryId(0);

/// A device of the first chip, holding [`LIBRARY`].
pub(crate) fn device() -> TestDevice {
    device_on(MockBackend::new())
}

/// A device of the first chip over `backend`, holding [`LIBRARY`].
pub(crate) fn device_on(backend: MockBackend) -> TestDevice {
    let chip: &Chip = &CHIPS[0];
    let platform = chip.platform(crate::metal::nvidia::Attachment {
        link: PcieLink {
            speed: 4,
            width: 16,
        },
        external: false,
        location_number: 0,
    });
    let mut device = Device::new(backend, MockCompiler::default(), &platform).unwrap();
    assert_eq!(device.new_library(fixture(false)), Ok(LIBRARY));
    device
}
