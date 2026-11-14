//! `MTLDevice`: Metal objects over a Vulkan [`Backend`].
//!
//! The device owns every Metal object as a slab entry named by a typed id,
//! with the Vulkan objects behind it. Destroying an object releases its
//! descriptors and Vulkan objects only after the last submitted command
//! buffer that could use it completes. Resources are in [`resources`],
//! pipelines in [`pipelines`], indirect command buffers in [`indirect`] and
//! command submission in [`queue`].
use super::{
    binding::{DescriptorHeap, HeapArray, HeapLayout, ResourceId},
    capabilities::{ApiFeatures, Capabilities, Platform},
    error::{Error, Result},
    shader::{Compiler, Refusal},
    timing::{CallPhases, Recorder},
    vulkan::{
        BufferCreate, BufferUsage, MemoryProperties,
        backend::{Backend, Descriptor, Object, requirements},
        object::{Buffer, BufferInfo, Memory, MemoryRequest, Semaphore},
    },
};
use alloc::{collections::VecDeque, vec::Vec};
use core::{ptr::NonNull, sync::atomic::AtomicU32};

/// A word the GPU writes after each submission's work, which the host reads
/// from any processor ([`Device::completion_word`]).
struct CompletionMarker {
    buffer: Buffer,
    word: NonNull<AtomicU32>,
}

/// The marker's memory: host-coherent, preferably cached, as the host polls
/// it and the GPU writes one word a submission.
const MARKER_MEMORY: MemoryRequest = MemoryRequest {
    required: MemoryProperties::HOST_VISIBLE.union(MemoryProperties::HOST_COHERENT),
    preferred: MemoryProperties::HOST_CACHED,
    mapped: true,
};

mod execution_sets;
#[cfg(test)]
pub(crate) mod fixture;
pub mod gpu_times;
pub mod indirect;
pub mod lifecycle;
mod memory_book;
pub use memory_book::MemoryOwner;
pub mod pass_times;
pub mod pins;
pub mod pipelines;
pub mod queue;
pub mod resources;

/// A slab of objects addressed by index; freed slots are reused.
#[derive(Debug)]
struct Slab<T> {
    entries: Vec<Option<T>>,
    free: Vec<u32>,
}

impl<T> Default for Slab<T> {
    fn default() -> Self {
        Self {
            entries: Vec::new(),
            free: Vec::new(),
        }
    }
}

impl<T> Slab<T> {
    fn insert(&mut self, value: T) -> u32 {
        match self.free.pop() {
            Some(index) => {
                self.entries[index as usize] = Some(value);
                index
            }
            None => {
                self.entries.push(Some(value));
                self.entries.len() as u32 - 1
            }
        }
    }

    fn get(&self, index: u32) -> Result<&T> {
        self.entries
            .get(index as usize)
            .and_then(Option::as_ref)
            .ok_or(Error::Invalid("unknown or destroyed object"))
    }

    fn get_mut(&mut self, index: u32) -> Result<&mut T> {
        self.entries
            .get_mut(index as usize)
            .and_then(Option::as_mut)
            .ok_or(Error::Invalid("unknown or destroyed object"))
    }

    fn iter(&self) -> impl Iterator<Item = &T> {
        self.entries.iter().flatten()
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.entries.iter_mut().flatten()
    }

    fn remove(&mut self, index: u32) -> Result<T> {
        let value = self.entries.get_mut(index as usize).and_then(Option::take);
        let value = value.ok_or(Error::Invalid("unknown or destroyed object"))?;
        self.free.push(index);
        Ok(value)
    }
}

macro_rules! ids {
    ($($(#[$meta:meta])* $name:ident;)*) => {$(
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name(pub u32);
    )*};
}

ids! {
    BufferId;
    TextureId;
    SamplerId;
    HeapId;
    MemoryId;
    LibraryId;
    /// `MTLBinaryArchive`.
    ArchiveId;
    FunctionId;
    RenderPipelineId;
    ComputePipelineId;
    EventId;
    CounterBufferId;
}

pub struct Device<B: Backend, C: Compiler> {
    backend: B,
    compiler: C,
    capabilities: Capabilities,
    descriptors: DescriptorHeap,
    /// The universal queue's timeline: command buffer `n` signals `n`.
    timeline: Semaphore,
    /// The completion marker, when the backend maps one.
    marker: Option<CompletionMarker>,
    /// A descriptor of the timeline for the platform to wait on, until it
    /// takes it.
    completion: Option<i32>,
    submitted: u64,
    completed: u64,
    /// A submission failed; nothing pending will complete.
    lost: bool,
    quiesce: Option<lifecycle::QuiesceStatus>,
    /// Objects waiting for the command buffers that may read them, with the
    /// allocation bytes charged until their backend memory is destroyed.
    /// Entries follow release order.
    deferred: VecDeque<(pins::Release, Object, u64)>,
    /// Ticks the next retirement may spend destroying what it frees, zero
    /// for no bound ([`Self::bound_destruction`]), and whether one left
    /// freeable objects for the next.
    destruction_budget: u64,
    destruction_left: bool,
    /// Who may read what is released now ([`Self::release_for`]).
    scope: pins::Scope,
    /// The most deferred bytes that only held work kept ([`DeviceMemory`]).
    peak_pinned: u64,
    allocated: u64,
    resources: resources::Resources,
    pipelines: pipelines::Pipelines,
    execution_sets: execution_sets::ExecutionSets,
    indirect: Slab<indirect::IndirectObject>,
    /// A function's module may set or copy indirect commands' barriers, so
    /// any GPU-resident buffer's commands may have them.
    command_barriers: bool,
    /// A render pipeline runs the fragment tests in its fragment function
    /// ([`RenderPipelinePlan::tests_fragments`]), so passes keep storage
    /// copies of their depth and stencil attachments.
    ///
    /// [`RenderPipelinePlan::tests_fragments`]: super::pipeline::RenderPipelinePlan::tests_fragments
    tests_fragments: bool,
    queue: queue::Queue,
    /// What the device left undone with no call to refuse, such as work it
    /// submits later, each reported once.
    unsupported: Vec<&'static str>,
    /// What it left undone that no call has reported yet, oldest first.
    unreported: Vec<&'static str>,
    /// The phases of the call it serves now, for the platform's log.
    timing: Recorder,
    /// Who owned the GPU addresses of whole allocations, for a lost
    /// device's report.
    book: memory_book::MemoryBook,
}

/// What a device holds at one moment.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DeviceMemory {
    /// `currentAllocatedSize`, with what waits for submitted work.
    pub allocated: u64,
    /// Objects destroyed once submitted work completes, and their bytes.
    pub deferred: usize,
    pub deferred_bytes: u64,
    /// The deferred objects and bytes that only held work keeps: released
    /// after everything running then completed. The peak is since the
    /// device opened.
    pub pinned: usize,
    pub pinned_bytes: u64,
    pub peak_pinned_bytes: u64,
    /// Submissions the queue has not finished.
    pub in_flight: usize,
    pub submitted: u64,
    pub completed: u64,
    /// Destroyed private textures kept for reuse, and how often creations
    /// reused one.
    pub recycled: resources::RecycleReport,
    /// Resources automatic heaps answered nil for, by cause.
    pub heap_refusals: resources::HeapRefusals,
    /// The backend's memory blocks and the resources placed in them.
    pub blocks: crate::metal::vulkan::backend::MemoryBlocks,
    /// Each descriptor array's use, in [`HeapArray`] order.
    pub descriptors: [DescriptorUse; 5],
}

/// One descriptor array's indices: in use, released but waiting for the
/// GPU, and in all.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DescriptorUse {
    pub in_use: u32,
    pub retiring: u32,
    pub count: u32,
}

impl core::fmt::Display for DescriptorUse {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}+{}/{}", self.in_use, self.retiring, self.count)
    }
}

impl<B: Backend, C: Compiler> Device<B, C> {
    /// The serials of the last command buffer submitted and the last seen
    /// complete.
    pub fn progress(&self) -> (u64, u64) {
        (self.submitted, self.completed)
    }

    /// Gives the device and its backend the platform's tick counter, which
    /// times the phases of the calls a [`Recorder`] names.
    pub fn set_clock(&mut self, ticks: fn() -> u64) {
        self.timing.set_clock(ticks);
        self.backend.set_clock(ticks);
    }

    /// Bounds the destruction of released objects in the retirements until
    /// [`Self::unbound_destruction`] to `ticks` each (by the platform's
    /// clock, [`Self::set_clock`]): a retirement past it leaves what it has
    /// not destroyed for the next ([`Self::destruction_left`]), which any
    /// look or call makes. Each object is still destroyed only once every
    /// command buffer that may read it is done, as before, only later.
    pub fn bound_destruction(&mut self, ticks: u64) {
        self.destruction_budget = ticks;
    }

    /// Ends [`Self::bound_destruction`].
    pub fn unbound_destruction(&mut self) {
        self.destruction_budget = 0;
    }

    /// Whether a bounded retirement left freeable objects undestroyed.
    pub fn destruction_left(&self) -> bool {
        self.destruction_left
    }

    /// The tick counter the platform gave.
    pub fn clock(&self) -> fn() -> u64 {
        self.timing.clock()
    }

    /// The phases of the call served now.
    pub fn timing(&mut self) -> &mut Recorder {
        &mut self.timing
    }

    /// The phases of the call that just ended, when one was timed.
    pub fn take_phases(&mut self) -> Option<CallPhases> {
        self.timing.take()
    }

    /// What the device holds now.
    pub fn memory(&self) -> DeviceMemory {
        let (pinned, pinned_bytes) = self.pinned();
        DeviceMemory {
            allocated: self.allocated,
            deferred: self.deferred.len(),
            deferred_bytes: self.deferred.iter().map(|&(_, _, bytes)| bytes).sum(),
            pinned,
            pinned_bytes,
            peak_pinned_bytes: self.peak_pinned.max(pinned_bytes),
            in_flight: self.queue.in_flight_count(),
            submitted: self.submitted,
            completed: self.completed,
            recycled: self.recycled(),
            heap_refusals: self.heap_refusals(),
            blocks: self.backend.memory_blocks(),
            descriptors: core::array::from_fn(|index| {
                let array = HeapArray::ALL[index];
                DescriptorUse {
                    in_use: self.descriptors.in_use(array),
                    retiring: self.descriptors.retiring(array),
                    count: self.descriptors.layout().count(array),
                }
            }),
        }
    }

    /// Reports `what` the device leaves undone, once.
    fn unsupported(&mut self, what: &'static str) {
        if !self.unsupported.contains(&what) {
            self.unsupported.push(what);
            self.unreported.push(what);
        }
    }

    /// The oldest thing the device left undone that no call's status
    /// reported; each is answered once.
    pub fn take_unsupported(&mut self) -> Option<&'static str> {
        (!self.unreported.is_empty()).then(|| self.unreported.remove(0))
    }

    /// The compiler's detail of the last function it refused, once.
    pub fn take_refusal(&mut self) -> Option<Refusal> {
        self.compiler.take_refusal()
    }

    /// Opens the device, refusing Vulkan devices that lack a feature the
    /// binding ABI or command model relies on.
    pub fn new(mut backend: B, compiler: C, platform: &Platform) -> Result<Self> {
        let physical = backend.physical_device();
        if let Some(missing) = requirements(&physical.features)
            .iter()
            .find(|requirement| !requirement.present)
        {
            return Err(Error::Unsupported(missing.name));
        }
        let capabilities = Capabilities::derive(
            physical,
            platform,
            compiler.features(),
            ApiFeatures::IMPLEMENTED,
        );
        let layout = HeapLayout::new(&physical.limits);
        let timeline = backend.create_semaphore(0)?;
        let completion = backend.export_semaphore(timeline);
        let marker = completion_marker(&mut backend);
        Ok(Self {
            backend,
            compiler,
            capabilities,
            descriptors: DescriptorHeap::new(layout),
            timeline,
            marker,
            completion,
            submitted: 0,
            completed: 0,
            lost: false,
            quiesce: None,
            deferred: VecDeque::new(),
            destruction_budget: 0,
            destruction_left: false,
            scope: pins::Scope::DEVICE,
            peak_pinned: 0,
            allocated: 0,
            resources: resources::Resources::default(),
            pipelines: pipelines::Pipelines::default(),
            execution_sets: execution_sets::ExecutionSets::default(),
            indirect: Slab::default(),
            command_barriers: false,
            tests_fragments: false,
            queue: queue::Queue::default(),
            unsupported: Vec::new(),
            unreported: Vec::new(),
            timing: Recorder::default(),
            book: memory_book::MemoryBook::default(),
        })
    }

    pub fn capabilities(&self) -> &Capabilities {
        &self.capabilities
    }

    pub fn backend(&mut self) -> &mut B {
        &mut self.backend
    }

    /// A lost device cannot resume under its old object identities. Keep
    /// in-flight imports pinned until the native owner proves DMA has stopped.
    pub fn mark_lost(&mut self) {
        self.lost = true;
    }

    pub fn is_lost(&self) -> bool {
        self.lost
    }

    /// The newest serial a poll saw completed, without asking the GPU.
    pub fn completed_serial(&self) -> u64 {
        self.completed
    }

    /// The newest serial submitted.
    pub fn submitted_serial(&self) -> u64 {
        self.submitted
    }

    /// The low 32 bits of the newest serial whose submission's work the GPU
    /// finished, as the GPU wrote them after that work with every earlier
    /// write made visible to the host: readable from any processor without
    /// the driver, valid while this device lives. `None` without a marker or
    /// once lost.
    pub fn completion_word(&self) -> Option<NonNull<AtomicU32>> {
        self.marker
            .as_ref()
            .filter(|_| !self.lost)
            .map(|marker| marker.word)
    }

    /// The marker buffer the commands writing serial `serial` write, for the
    /// submission that signals the timeline's `serial`.
    fn marker_buffer(&self) -> Option<Buffer> {
        self.marker.as_ref().map(|marker| marker.buffer)
    }

    /// A descriptor of the queue's timeline, which signals each serial as it
    /// completes, for the platform to wait on and own: once per device.
    pub fn take_completion(&mut self) -> Option<i32> {
        self.completion.take()
    }

    /// Exports the queue's timeline again for [`Self::take_completion`],
    /// as the platform's waiter that took it left: whether one is offered.
    /// None once lost.
    pub fn rearm_completion(&mut self) -> bool {
        if self.lost {
            return false;
        }
        if self.completion.is_none() {
            self.completion = self.backend.export_semaphore(self.timeline);
        }
        self.completion.is_some()
    }

    /// `currentAllocatedSize`, including allocations pending GPU completion.
    pub fn current_allocated_size(&self) -> u64 {
        self.allocated
    }

    /// Writes `descriptor` into a fresh heap index.
    fn descriptor(&mut self, descriptor: Descriptor) -> Result<u32> {
        let index = self.descriptors.allocate(descriptor.array())?;
        self.backend.write_descriptor(index, descriptor);
        Ok(index)
    }

    /// Runs `release` with `scope` naming who may read what it releases,
    /// instead of every command buffer: a connection's own objects are
    /// read only by its command buffers committed since it made them
    /// ([`pins`]).
    pub fn release_for<T>(
        &mut self,
        scope: pins::Scope,
        release: impl FnOnce(&mut Self) -> T,
    ) -> T {
        let outer = core::mem::replace(&mut self.scope, scope);
        let result = release(self);
        self.scope = outer;
        result
    }

    /// A release now, by the current scope.
    fn release_now(&self) -> pins::Release {
        pins::Release {
            serial: self.submitted,
            scope: self.scope.clone(),
        }
    }

    /// Whether no command buffer in flight may read what `release` frees.
    /// One seen complete but not yet retired still counts: retiring reads
    /// its mapped ranges.
    fn frees(&self, release: &pins::Release) -> bool {
        !self.queue.reads(release)
    }

    /// The deferred objects and bytes that only held work keeps.
    fn pinned(&self) -> (usize, u64) {
        self.deferred
            .iter()
            .filter(|(release, ..)| release.serial <= self.completed)
            .fold((0, 0), |(count, bytes), &(_, _, more)| {
                (count + 1, bytes + more)
            })
    }

    /// Destroys `object` once no command buffer that may read it is in
    /// flight: by default, every one submitted so far.
    fn defer(&mut self, object: Object) {
        self.defer_allocation(object, 0);
    }

    fn defer_allocation(&mut self, object: Object, allocated: u64) {
        if let Object::Memory(memory) = object {
            self.book.release(memory, self.submitted);
        }
        let release = self.release_now();
        if !self.lost && self.frees(&release) {
            self.destroy_now(object);
            self.allocated -= allocated;
        } else {
            self.deferred.push_back((release, object, allocated));
        }
    }

    /// Frees the descriptors of `id` once no command buffer that may read
    /// them is in flight: the run of `low_length` from its low half and its
    /// high half's run.
    fn release_id(&mut self, id: ResourceId, arrays: [HeapArray; 2], low_length: u32) {
        let runs = [(id.low(), low_length), (id.high_index(), id.high_count())];
        for ((first, length), array) in runs.into_iter().zip(arrays) {
            if first != super::binding::NULL_DESCRIPTOR {
                let release = self.release_now();
                self.descriptors
                    .release_run_for(array, first, length, release);
            }
        }
    }

    /// Frees one descriptor once no command buffer that may read it is in
    /// flight.
    fn release_descriptor(&mut self, array: HeapArray, index: u32) {
        let release = self.release_now();
        self.descriptors.release_run_for(array, index, 1, release);
    }

    fn release_memory(&mut self, memory: Memory, bytes: u64) {
        self.defer_allocation(Object::Memory(memory), bytes);
    }

    /// Destroys `object` now that no submitted work uses it; freed memory
    /// leaves the book as of the completed serial.
    fn destroy_now(&mut self, object: Object) {
        if let Object::Memory(memory) = object {
            self.book.free(memory, self.completed);
        }
        self.backend.destroy(object);
    }

    /// Notes `memory`, a whole allocation of `bytes` that `what` owns, in
    /// the book; memory placed in a block is not, as freeing it unmaps
    /// nothing.
    fn book_memory(
        &mut self,
        memory: Memory,
        bytes: u64,
        what: impl FnOnce() -> alloc::string::String,
    ) {
        if self.backend.whole_memory(memory) {
            self.book.note(memory, bytes, what(), self.submitted);
        }
    }

    /// The newest frees of whole allocations and the live ranges either
    /// side, which name what the address of a GPU MMU fault held.
    pub fn memory_book_report(&self) -> Vec<alloc::string::String> {
        self.book.report(self.submitted)
    }
}

#[cfg(test)]
pub(crate) mod tests;

/// The completion marker: a small buffer of host-coherent memory the host
/// maps, zero until the first submission writes it. `None` when the backend
/// gives no such view; completion is then only the driver's.
fn completion_marker<B: Backend>(backend: &mut B) -> Option<CompletionMarker> {
    if !backend.maps_words() {
        return None;
    }
    let info = BufferInfo {
        size: 256,
        usage: BufferUsage::TRANSFER_DST,
        flags: BufferCreate::EMPTY,
    };
    let allocation = backend.create_buffer(&info, MARKER_MEMORY).ok()?;
    match backend.host_word(allocation.memory, 0) {
        Some(word) => Some(CompletionMarker {
            buffer: allocation.buffer,
            word,
        }),
        None => {
            backend.destroy(Object::Buffer(allocation.buffer));
            backend.destroy(Object::Memory(allocation.memory));
            None
        }
    }
}
