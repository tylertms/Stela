//! Buffers, textures, samplers and heaps.
use super::{BufferId, Device, HeapId, MemoryId, MemoryOwner, SamplerId, Slab, TextureId};
use crate::metal::{
    abi::{
        CompareFunction, CpuCacheMode, HazardTrackingMode, HeapType, ResourceOptions,
        SamplerMipFilter, StorageMode, TextureType, TextureUsage,
    },
    binding::{HeapArray, NULL_DESCRIPTOR, ResourceId, TexelArguments},
    capabilities::FormatCapabilities,
    command_buffer::{
        BufferRef, MappedRange,
        blit::{BufferLayout, Region},
    },
    error::{Error, Result, ensure, require},
    hazard::FULL,
    heap::{self, HeapAllocator, HeapDescriptor, SizeAndAlign},
    pass::{AttachmentView, TestedCopy},
    pixel_format::{Description, Encoding},
    resource::{
        self, CpuAccess, ShaderViews, SubresourceLayout, TextureDescriptor, TexturePlan,
        TextureViewDescriptor,
    },
    sampler::{self, SamplerDescriptor},
    shader::Compiler,
    sparse::{self, MappingRequest, SparseLayout, SparsePages, SparseTexture},
    timing::{Detail, Phase, Timed},
    vulkan::{
        BufferCreate, BufferUsage, Format, ImageAspects, ImageCreate, ImageTiling, ImageType,
        ImageUsage, ImageViewType, MemoryProperties, SampleCounts,
        backend::{Backend, BufferAllocation, Descriptor, Object, Outside, PreparedImport},
        command::{BufferImageCopy, Command, SparseBind},
        object::{
            Buffer, BufferInfo, BufferView, Image, ImageInfo, ImageView, ImageViewInfo, Memory,
            MemoryRequest, Sampler, SubresourceRange,
        },
    },
};
use alloc::{
    collections::{BTreeMap, VecDeque},
    format,
    sync::Arc,
    vec::Vec,
};
use core::{num::NonZeroU64, ops::Range};
use stela_contracts::gate::MemoryRange;

mod extended_planes;
mod read_back;
pub(super) use extended_planes::PlaneActions;
pub use extended_planes::PlaneChanges;
pub use read_back::ReadBack;

const TEXTURE_ARRAYS: [HeapArray; 2] = [HeapArray::SampledImages, HeapArray::StorageImages];
const TEXEL_BUFFER_ARRAYS: [HeapArray; 2] = [
    HeapArray::UniformTexelBuffers,
    HeapArray::StorageTexelBuffers,
];

/// What the device keeps of destroyed private textures for reuse: at most
/// this many bytes and textures, each no larger than
/// [`RECYCLED_TEXTURE_BYTES`]. Applications create and destroy the same
/// intermediate textures every frame; each allocation and free is a kernel
/// call into the GPU's driver, where a reused texture is none.
const RECYCLED_BYTES: u64 = 256 << 20;
pub(super) const RECYCLED_TEXTURES: usize = 128;
const RECYCLED_TEXTURE_BYTES: u64 = 64 << 20;

/// A destroyed private texture kept whole, with its image, memory, views and
/// descriptors, for the next `newTexture` of an equal descriptor once the
/// work submitted before its destruction completes.
#[derive(Debug)]
struct Recycled {
    /// Its destruction: reused or freed once nothing it names reads it.
    ready: super::pins::Release,
    bytes: u64,
    texture: TextureObject,
}

/// `allocatedSize` and `heapOffset` of a buffer or texture: zero for those
/// sharing another resource's memory or mapping sparse pages.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Placement {
    pub allocated_size: u64,
    pub heap_offset: u64,
}

/// Where a resource's memory came from.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Backing {
    /// Its own allocation of this many bytes.
    Owned(u64),
    /// Placed in a heap at `offset`, `size` bytes as the heap counts them
    /// (`heapOffset`, `allocatedSize`); automatic heaps return that range on
    /// destruction unless the resource was made aliasable. Where it lives:
    /// the range `real` of the heap's memory its automatic heap reserved,
    /// or at `offset` in it (`real` none, `own` zero), or memory of its
    /// own of `own` bytes when its layout does not fit there.
    Heap {
        heap: HeapId,
        offset: u64,
        size: u64,
        aliasable: bool,
        real: Option<(u64, u64)>,
        own: u64,
    },
    /// Aliases a buffer's memory (linear textures and texture buffers).
    Buffer(BufferId),
    /// Maps pages of a sparse heap.
    Sparse(HeapId),
}

impl Backing {
    fn placement(self) -> Placement {
        match self {
            Self::Owned(bytes) => Placement {
                allocated_size: bytes,
                heap_offset: 0,
            },
            Self::Heap { offset, size, .. } => Placement {
                allocated_size: size,
                heap_offset: offset,
            },
            Self::Buffer(_) | Self::Sparse(_) => Placement::default(),
        }
    }
}

#[derive(Debug)]
struct BufferObject {
    buffer: Buffer,
    memory: Memory,
    memory_offset: u64,
    address: u64,
    length: u64,
    options: ResourceOptions,
    backing: Backing,
    /// Textures over this buffer, which keep it alive.
    dependents: u32,
}

#[derive(Debug)]
struct TextureObject {
    image: Option<Image>,
    descriptor: TextureDescriptor,
    id: ResourceId,
    views: Vec<ImageView>,
    texel_views: Vec<BufferView>,
    attachment_views: Vec<(AttachmentView<()>, ImageView)>,
    /// Raw texel storage views of attached subresources, with their storage
    /// descriptors, which framebuffer reads access.
    attachment_storage: Vec<(AttachmentView<()>, ImageView, u32)>,
    /// Copies of attached depth or stencil subresources, which fragment
    /// functions testing fragments themselves access as storage.
    tested_copies: Vec<TestedCopyObject>,
    cpu_access: CpuAccess,
    backing: Backing,
    /// The texture's own allocation, when `backing` is `Owned`.
    memory: Option<Memory>,
    /// The texture this one views; views keep their parent alive.
    parent: Option<TextureId>,
    dependents: u32,
    /// A managed texture's CPU copy, when the CPU cannot reach the image,
    /// or a decoded format's blocks ([`CpuAccess::Blocks`]), followed by
    /// the decode kernels' arguments for each subresource.
    cpu_copy: Option<BufferAllocation>,
    /// A two-plane texture's second plane's image; `image` is its first's.
    second_plane: Option<Image>,
}

/// A depth or stencil subresource's storage copy ([`TestedCopy`]): what it
/// copies, and its image, memory, view and storage descriptor.
#[derive(Debug)]
struct TestedCopyObject {
    view: AttachmentView<()>,
    aspect: ImageAspects,
    copy: TestedCopy,
    memory: Memory,
    bytes: u64,
    image_view: ImageView,
    descriptor: u32,
}

#[derive(Debug)]
struct SamplerObject {
    descriptor: SamplerDescriptor,
    samplers: Vec<Sampler>,
    id: ResourceId,
    /// LOD-clamped variants: (min, max, sampler id).
    clamped: Vec<(f32, f32, ResourceId)>,
    references: u32,
    /// Who may read it for the holders that released it: equal descriptors
    /// share one state across connections.
    released: Option<super::pins::Scope>,
}

#[derive(Debug)]
struct HeapObject {
    descriptor: HeapDescriptor,
    memory: Memory,
    /// The bytes of `memory`: a device heap's size rounded up to whole
    /// [`heap::PAGE`]s, as a Mac rounds it.
    bytes: u64,
    /// What an automatic heap holds by its answered sizes.
    allocator: Option<HeapAllocator>,
    /// Where in `memory` an automatic heap's resources live, by the
    /// backend's own sizes and alignments.
    real: Option<HeapAllocator>,
    /// Its memory is the guest's, which the CPU reaches at the offsets its
    /// buffers answer: a buffer lives nowhere else.
    imported: bool,
    sparse: Option<SparseHeap>,
    resources: u32,
}

/// A sparse heap's pages and the memory types its textures must accept.
#[derive(Debug)]
struct SparseHeap {
    pages: SparsePages,
    memory_types: u32,
}

/// Imported guest memory no buffer or heap has taken yet.
#[derive(Clone, Copy, Debug)]
struct ImportObject {
    memory: Memory,
    bytes: u64,
}

#[derive(Debug, Default)]
pub(super) struct Resources {
    imports: Slab<ImportObject>,
    buffers: Slab<BufferObject>,
    textures: Slab<TextureObject>,
    samplers: Slab<SamplerObject>,
    heaps: Slab<HeapObject>,
    /// CPU-copy regions of managed textures that the next commit uploads.
    cpu_copy_uploads: Vec<(TextureId, BufferImageCopy)>,
    /// Decodes of subresources whose blocks the CPU wrote, by texture,
    /// level and slice, that the next commit runs.
    block_decodes: Vec<((TextureId, u64, u64), Command)>,
    /// Sparse textures by image, with their heaps.
    sparse: BTreeMap<Image, (HeapId, SparseTexture)>,
    /// The samplers of constexpr sampler states by state. Any pipeline
    /// compiled later may name one, so each lives as long as the device.
    constexpr_samplers: BTreeMap<u64, ResourceId>,
    /// IOSurface textures of extended-range planes, by texture: where the
    /// planes lie in their buffers and what their working image holds
    /// ([`extended_planes`]).
    extended_planes: BTreeMap<u32, extended_planes::PlaneState>,
    /// The times textures encoded into each IOSurface's planes, by surface
    /// ID ([`extended_planes`]).
    extended_generations: BTreeMap<u32, u64>,
    /// Destroyed private textures kept for reuse, oldest first, and their
    /// bytes.
    recycled: VecDeque<Recycled>,
    recycled_bytes: u64,
    /// Private texture creations a recycled texture served, and those
    /// that allocated.
    recycle_hits: u64,
    recycle_misses: u64,
    heap_refusals: HeapRefusals,
}

/// What the device keeps of destroyed private textures, and how often
/// creations reused one.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RecycleReport {
    pub textures: usize,
    pub bytes: u64,
    pub hits: u64,
    pub misses: u64,
}

/// Resources automatic heaps answered nil for, as Metal does when a heap
/// has no room: `full` when its free bytes are fewer than the resource's
/// size, `fragmented` when they are enough but in pieces too small.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HeapRefusals {
    pub full: u64,
    pub fragmented: u64,
}

impl<B: Backend, C: Compiler> Device<B, C> {
    /// `newBufferWithLength:options:`.
    pub fn new_buffer(&mut self, length: u64, options: ResourceOptions) -> Result<BufferId> {
        let plan = resource::buffer(&self.capabilities, length, options)?;
        let allocation = self.backend.create_buffer(&plan.info, plan.memory)?;
        Ok(self.own_buffer(allocation, length, options))
    }

    /// The wired guest RAM of `ranges`, which the GPU reaches where it is. The
    /// kernel keeps it wired until [`Self::retired`] answers `token`.
    pub fn import_memory(&mut self, ranges: &[MemoryRange], token: NonZeroU64) -> Result<MemoryId> {
        self.import_memory_with(ranges, token, None)
    }

    /// [`Self::import_memory`], of memory the backend made outside the
    /// service's turn when `prepared` holds it.
    pub fn import_memory_with(
        &mut self,
        ranges: &[MemoryRange],
        token: NonZeroU64,
        prepared: Option<PreparedImport>,
    ) -> Result<MemoryId> {
        self.ensure_admission()?;
        let bytes = ranges.iter().map(|range| range.bytes).sum();
        let memory = match prepared {
            Some(prepared) => self.backend.adopt_import(prepared, token)?,
            None => self.backend.import_memory(ranges, token)?,
        };
        self.allocated += bytes;
        self.book_memory(memory, bytes, || {
            format!("imported memory of {bytes} bytes")
        });
        Ok(MemoryId(
            self.resources
                .imports
                .insert(ImportObject { memory, bytes }),
        ))
    }

    /// The backend's work outside the turn for an import of `ranges`:
    /// `None` when the backend keeps an import of them, taken back in the
    /// turn with no driver call, or does all of it in the turn.
    pub fn import_outside(&self, ranges: &[MemoryRange]) -> Option<Arc<dyn Outside>> {
        if self.backend.keeps_import(ranges) {
            return None;
        }
        self.backend.outside()
    }

    /// The backend's work outside the turn, whatever it is.
    pub fn outside(&self) -> Option<Arc<dyn Outside>> {
        self.backend.outside()
    }

    /// Imported memory no object took.
    /// The bytes an object allocated itself, which `currentAllocatedSize`
    /// counts for the process holding it: imported memory, and buffers,
    /// textures and heaps of their own memory; none for one placed in a
    /// heap or over a buffer, which its heap or buffer counts.
    pub fn import_bytes(&self, id: MemoryId) -> u64 {
        self.resources
            .imports
            .get(id.0)
            .map_or(0, |import| import.bytes)
    }

    pub fn buffer_bytes(&self, id: BufferId) -> u64 {
        self.resources
            .buffers
            .get(id.0)
            .map_or(0, |buffer| owned_bytes(buffer.backing))
    }

    pub fn texture_bytes(&self, id: TextureId) -> u64 {
        self.resources
            .textures
            .get(id.0)
            .map_or(0, |texture| owned_bytes(texture.backing))
    }

    pub fn heap_bytes(&self, id: HeapId) -> u64 {
        self.resources.heaps.get(id.0).map_or(0, |heap| heap.bytes)
    }

    pub fn release_import(&mut self, id: MemoryId) -> Result<()> {
        let import = self.resources.imports.remove(id.0)?;
        self.release_memory(import.memory, import.bytes);
        Ok(())
    }

    /// The token of imported memory the GPU no longer reaches.
    pub fn retired(&mut self) -> Option<NonZeroU64> {
        self.backend.retired()
    }

    /// Whether [`Self::retired`] has an answer.
    pub fn retiring(&self) -> bool {
        self.backend.retiring()
    }

    /// A buffer over imported memory, which it keeps; the frontend's mapping
    /// is its `contents`. `newBufferWithBytesNoCopy:…` imports the app's
    /// pages, and the frontend runs the deallocator once the last command
    /// buffer that used the buffer completes.
    pub fn new_imported_buffer(
        &mut self,
        id: MemoryId,
        length: u64,
        options: ResourceOptions,
    ) -> Result<BufferId> {
        ensure(
            matches!(
                options.storage_mode,
                StorageMode::Shared | StorageMode::Managed
            ),
            "imported buffers are shared or managed",
        )?;
        let ImportObject { memory, bytes } = *self.resources.imports.get(id.0)?;
        ensure(length <= bytes, "the buffer is longer than its memory")?;
        let plan = resource::buffer(&self.capabilities, length, options)?;
        let (buffer, address) = self.backend.create_placed_buffer(&plan.info, memory, 0)?;
        self.resources.imports.remove(id.0)?;
        self.allocated -= bytes;
        let buffer = self.own_buffer(
            BufferAllocation {
                buffer,
                memory,
                address,
                allocated: bytes,
            },
            length,
            options,
        );
        self.book
            .describe(memory, format!("imported buffer of {length} bytes"));
        Ok(buffer)
    }

    fn own_buffer(
        &mut self,
        allocation: BufferAllocation,
        length: u64,
        options: ResourceOptions,
    ) -> BufferId {
        self.allocated += allocation.allocated;
        self.book_memory(allocation.memory, allocation.allocated, || {
            format!("buffer of {length} bytes ({:?})", options.storage_mode)
        });
        if allocation.address != 0 {
            self.book
                .learn_address(allocation.memory, allocation.address);
        }
        BufferId(self.resources.buffers.insert(BufferObject {
            buffer: allocation.buffer,
            memory: allocation.memory,
            memory_offset: 0,
            address: allocation.address,
            length,
            options,
            backing: Backing::Owned(allocation.allocated),
            dependents: 0,
        }))
    }

    /// A shared buffer of the device's own that holds `bytes`.
    pub(super) fn new_shared_buffer(&mut self, bytes: &[u8]) -> Result<BufferId> {
        let options = ResourceOptions {
            cpu_cache_mode: CpuCacheMode::DefaultCache,
            storage_mode: StorageMode::Shared,
            hazard_tracking_mode: HazardTrackingMode::Default,
        };
        let buffer = self.new_buffer(bytes.len() as u64, options)?;
        let (memory, offset, _) = self.buffer_memory(buffer)?;
        if let Err(error) = self.backend.write_memory(memory, offset, bytes) {
            self.destroy_buffer(buffer)?;
            return Err(error);
        }
        Ok(buffer)
    }

    pub fn buffer(&self, id: BufferId) -> Result<BufferRef> {
        let buffer = self.resources.buffers.get(id.0)?;
        Ok(BufferRef {
            buffer: buffer.buffer,
            offset: 0,
            address: buffer.address,
            length: buffer.length,
        })
    }

    pub fn buffer_placement(&self, id: BufferId) -> Result<Placement> {
        Ok(self.resources.buffers.get(id.0)?.backing.placement())
    }

    pub fn texture_placement(&self, id: TextureId) -> Result<Placement> {
        Ok(self.resources.textures.get(id.0)?.backing.placement())
    }

    /// The mapped memory behind a buffer: its memory, offset and length.
    pub fn buffer_memory(&self, id: BufferId) -> Result<(Memory, u64, u64)> {
        let buffer = self.resources.buffers.get(id.0)?;
        ensure(
            buffer.options.storage_mode != StorageMode::Private,
            "private buffers have no contents",
        )?;
        Ok((buffer.memory, buffer.memory_offset, buffer.length))
    }

    /// `didModifyRange:` of a managed buffer.
    pub fn did_modify_range(&mut self, id: BufferId, offset: u64, length: u64) -> Result<()> {
        let buffer = self.resources.buffers.get(id.0)?;
        ensure(
            buffer.options.storage_mode == StorageMode::Managed,
            "didModifyRange: is for managed buffers",
        )?;
        ensure(
            offset
                .checked_add(length)
                .is_some_and(|end| end <= buffer.length),
            "range extends past the buffer",
        )?;
        let (memory, base) = (buffer.memory, buffer.memory_offset);
        self.backend.flush_memory(memory, base + offset, length)
    }

    /// The mapping `synchronizeResource:` makes current for a managed buffer.
    pub fn managed_range(&self, id: BufferId) -> Result<MappedRange> {
        let buffer = self.resources.buffers.get(id.0)?;
        ensure(
            buffer.options.storage_mode == StorageMode::Managed,
            "synchronizeResource: is for managed resources",
        )?;
        Ok(MappedRange {
            memory: buffer.memory,
            offset: buffer.memory_offset,
            length: buffer.length,
        })
    }

    pub fn destroy_buffer(&mut self, id: BufferId) -> Result<()> {
        ensure(
            self.resources.buffers.get(id.0)?.dependents == 0,
            "textures still alias the buffer",
        )?;
        let buffer = self.resources.buffers.remove(id.0)?;
        self.defer(Object::Buffer(buffer.buffer));
        self.release_backing(buffer.backing, Some(buffer.memory))
    }

    /// `newTextureWithDescriptor:`.
    pub fn new_texture(&mut self, descriptor: &TextureDescriptor) -> Result<TextureId> {
        let start = self.timing.now();
        let plan = resource::texture(&self.capabilities, descriptor)?;
        let start = self.timing.since(Phase::TexturePlan, start);
        let reused = self.reuse_texture(descriptor, &plan);
        let start = self.timing.since(Phase::TextureRecycle, start);
        self.time_texture(descriptor, 0, reused.is_some());
        if let Some(texture) = reused {
            return Ok(texture);
        }
        self.backend.take_split();
        let (image, memory, bytes) = match self.backend.create_image(&plan.image, plan.memory) {
            // Recycled textures hold memory nothing else may use.
            Err(Error::OutOfMemory) if !self.resources.recycled.is_empty() => {
                self.trim_recycled(0, 0)?;
                self.backend.create_image(&plan.image, plan.memory)?
            }
            created => created?,
        };
        // The backend splits its call; what it does not is the image's.
        let whole = self.timing.now().wrapping_sub(start);
        let [_, allocate, bind] = self.backend.take_split();
        self.timing.add(Phase::TextureAllocate, allocate);
        self.timing.add(Phase::TextureBind, bind);
        self.timing
            .add(Phase::TextureCreate, whole.saturating_sub(allocate + bind));
        self.time_texture(descriptor, bytes, false);
        self.allocated += bytes;
        self.finish_texture(
            image,
            descriptor,
            &plan,
            Backing::Owned(bytes),
            Some(memory),
        )
    }

    /// What a timed `newTextureWithDescriptor:` makes.
    fn time_texture(&mut self, descriptor: &TextureDescriptor, bytes: u64, recycled: bool) {
        let detail = Detail::Texture {
            format: descriptor.pixel_format,
            size: [descriptor.width, descriptor.height, descriptor.depth],
            slices: descriptor.slice_count(),
            levels: descriptor.mipmap_level_count,
            bytes,
            recycled,
        };
        self.timing.detail(Timed::NewTexture, detail);
    }

    fn finish_texture(
        &mut self,
        image: Image,
        descriptor: &TextureDescriptor,
        plan: &TexturePlan,
        backing: Backing,
        memory: Option<Memory>,
    ) -> Result<TextureId> {
        let start = self.timing.now();
        let cpu_copy = match plan.cpu_access {
            CpuAccess::Staged => self.new_cpu_copy(descriptor, false).map(Some),
            CpuAccess::Blocks => self.new_cpu_copy(descriptor, true).map(Some),
            _ => Ok(None),
        };
        self.timing.since(Phase::TextureCpuCopy, start);
        let created = cpu_copy.and_then(|cpu_copy| match self.shader_views(image, &plan.views) {
            Ok((id, views)) => Ok((id, views, cpu_copy)),
            Err(error) => {
                if let Some(copy) = cpu_copy {
                    self.destroy_cpu_copy(copy);
                }
                Err(error)
            }
        });
        let (id, views, cpu_copy) = match created {
            Ok(created) => created,
            Err(error) => {
                self.backend.destroy(Object::Image(image));
                self.release_backing(backing, memory)?;
                return Err(error);
            }
        };
        let texture = TextureObject {
            image: Some(image),
            descriptor: *descriptor,
            id,
            views,
            texel_views: Vec::new(),
            attachment_views: Vec::new(),
            attachment_storage: Vec::new(),
            tested_copies: Vec::new(),
            cpu_access: plan.cpu_access,
            backing,
            memory,
            parent: None,
            dependents: 0,
            cpu_copy,
            second_plane: None,
        };
        Ok(TextureId(self.resources.textures.insert(texture)))
    }

    /// The host-visible buffer behind a managed texture's CPU copy, or
    /// behind a decoded format's `blocks` and their decode arguments, which
    /// the decode kernels read by address.
    fn new_cpu_copy(
        &mut self,
        descriptor: &TextureDescriptor,
        blocks: bool,
    ) -> Result<BufferAllocation> {
        let (size, usage) = if blocks {
            let arguments = descriptor.mipmap_level_count * descriptor.slice_count();
            (
                decode_arguments_offset(descriptor) + arguments * DECODE_ARGUMENTS_STRIDE,
                BufferUsage::SHADER_DEVICE_ADDRESS,
            )
        } else {
            (descriptor.cpu_copy_bytes(), BufferUsage::EMPTY)
        };
        let info = BufferInfo {
            size,
            usage: BufferUsage::TRANSFER_SRC | BufferUsage::TRANSFER_DST | usage,
            flags: BufferCreate::EMPTY,
        };
        let request = MemoryRequest {
            required: MemoryProperties::HOST_VISIBLE | MemoryProperties::HOST_COHERENT,
            preferred: MemoryProperties::HOST_CACHED,
            mapped: true,
        };
        let allocation = self.backend.create_buffer(&info, request)?;
        self.allocated += allocation.allocated;
        Ok(allocation)
    }

    fn destroy_cpu_copy(&mut self, copy: BufferAllocation) {
        self.defer(Object::Buffer(copy.buffer));
        self.release_memory(copy.memory, copy.allocated);
    }

    /// Creates the sampled view, a cube's faces and a storage view of each
    /// level, which a storage image holds one of, with their descriptors.
    /// On failure nothing
    /// it created remains.
    fn shader_views(
        &mut self,
        image: Image,
        views: &ShaderViews,
    ) -> Result<(ResourceId, Vec<ImageView>)> {
        let levels = views.storage.iter().flat_map(|info| {
            (0..info.range.level_count).map(|level| ImageViewInfo {
                range: SubresourceRange {
                    base_mip_level: info.range.base_mip_level + level,
                    level_count: 1,
                    ..info.range
                },
                ..*info
            })
        });
        let infos: Vec<_> = views
            .sampled
            .into_iter()
            .chain(views.faces)
            .chain(levels)
            .collect();
        let start = self.timing.now();
        let mut created = Vec::with_capacity(infos.len());
        let mut descriptors = Ok(ResourceId::NULL);
        for info in &infos {
            match self.backend.create_image_view(image, info) {
                Ok(view) => created.push(view),
                Err(error) => {
                    descriptors = Err(error);
                    break;
                }
            }
        }
        let start = self.timing.since(Phase::TextureViews, start);
        if descriptors.is_ok() {
            let sampled = usize::from(views.sampled.is_some()) + usize::from(views.faces.is_some());
            let (sampled, storage) = created.split_at(sampled);
            descriptors = self.view_descriptors(sampled, storage).map(|id| {
                id.with_srgb_storage(views.srgb_storage)
                    .with_extended_storage(views.extended_storage)
                    .with_filter_class(views.filter_class)
            });
        }
        self.timing.since(Phase::TextureDescriptors, start);
        match descriptors {
            Ok(id) => Ok((id, created)),
            Err(error) => {
                for view in created {
                    self.backend.destroy(Object::ImageView(view));
                }
                Err(error)
            }
        }
    }

    /// Writes the descriptors of consecutive sampled views, a view and a
    /// cube's faces, and of consecutive storage views, one per level.
    fn view_descriptors(
        &mut self,
        sampled_views: &[ImageView],
        storage: &[ImageView],
    ) -> Result<ResourceId> {
        let sampled_run = sampled_views.len() as u32;
        let sampled = match sampled_run {
            0 => NULL_DESCRIPTOR,
            run => self
                .descriptors
                .allocate_run(HeapArray::SampledImages, run)?,
        };
        for (index, view) in (sampled..).zip(sampled_views) {
            self.backend
                .write_descriptor(index, Descriptor::SampledImage(*view));
        }
        if storage.is_empty() {
            return Ok(ResourceId::new(sampled, NULL_DESCRIPTOR));
        }
        let levels = storage.len() as u32;
        let first = match self
            .descriptors
            .allocate_run(HeapArray::StorageImages, levels)
        {
            Ok(first) => first,
            Err(error) => {
                self.release_id(
                    ResourceId::new(sampled, NULL_DESCRIPTOR),
                    TEXTURE_ARRAYS,
                    sampled_run,
                );
                return Err(error);
            }
        };
        for (level, view) in (first..).zip(storage) {
            self.backend
                .write_descriptor(level, Descriptor::StorageImage(*view));
        }
        Ok(ResourceId::texture(sampled, first, levels))
    }

    /// `gpuResourceID` of a texture.
    pub fn texture_resource_id(&self, id: TextureId) -> Result<ResourceId> {
        Ok(self.resources.textures.get(id.0)?.id)
    }

    pub fn texture_descriptor(&self, id: TextureId) -> Result<&TextureDescriptor> {
        Ok(&self.resources.textures.get(id.0)?.descriptor)
    }

    /// A two-plane texture's second plane's image; `None` for a texture of
    /// one plane.
    pub fn second_plane(&self, id: TextureId) -> Result<Option<Image>> {
        Ok(self.resources.textures.get(id.0)?.second_plane)
    }

    /// Where texture `id`'s texels are, which the profile's pass report
    /// names: a view's are its parent's.
    pub(super) fn target_memory(&self, id: TextureId) -> super::pass_times::TargetMemory {
        use super::pass_times::TargetMemory;
        let textures = &self.resources.textures;
        let Ok(texture) = textures.get(id.0) else {
            return TargetMemory::Optimal;
        };
        let root = texture.parent.unwrap_or(id);
        if self.has_extended_planes(root) {
            return TargetMemory::Planes;
        }
        let backing = textures.get(root.0).map(|texture| texture.backing);
        let Ok(Backing::Buffer(buffer)) = backing else {
            return TargetMemory::Optimal;
        };
        let Ok(buffer) = self.resources.buffers.get(buffer.0) else {
            return TargetMemory::Optimal;
        };
        match (buffer.options.storage_mode, buffer.options.cpu_cache_mode) {
            (StorageMode::Private, _) => TargetMemory::LinearPrivate,
            (_, CpuCacheMode::WriteCombined) => TargetMemory::LinearWriteCombined,
            _ => TargetMemory::LinearShared,
        }
    }

    /// The texture's image, for blits.
    pub fn texture_image(&self, id: TextureId) -> Result<Image> {
        let texture = self.resources.textures.get(id.0)?;
        if texture.second_plane.is_some() {
            return Err(Error::Unsupported("blits of two-plane textures"));
        }
        texture
            .image
            .ok_or(Error::Invalid("texture buffers are not images"))
    }

    /// `newTextureViewWithPixelFormat:textureType:levels:slices:swizzle:`.
    pub fn new_texture_view(
        &mut self,
        parent: TextureId,
        view: &TextureViewDescriptor,
    ) -> Result<TextureId> {
        let texture = self.resources.textures.get(parent.0)?;
        if texture.second_plane.is_some() {
            return Err(Error::Unsupported("views of two-plane textures"));
        }
        let image = texture
            .image
            .ok_or(Error::Invalid("texture buffers have no views"))?;
        let mut descriptor = texture.descriptor;
        let root = texture.parent.unwrap_or(parent);
        let views = resource::texture_view(&self.capabilities, &descriptor, view)?;
        let (id, created) = self.shader_views(image, &views)?;
        descriptor.pixel_format = view.pixel_format;
        descriptor.texture_type = view.texture_type;
        descriptor.swizzle = view.swizzle;
        self.resources.textures.get_mut(root.0)?.dependents += 1;
        let view = TextureObject {
            image: Some(image),
            descriptor,
            id,
            views: created,
            texel_views: Vec::new(),
            attachment_views: Vec::new(),
            attachment_storage: Vec::new(),
            tested_copies: Vec::new(),
            cpu_access: CpuAccess::None,
            backing: Backing::Owned(0),
            memory: None,
            parent: Some(root),
            dependents: 0,
            cpu_copy: None,
            second_plane: None,
        };
        Ok(TextureId(self.resources.textures.insert(view)))
    }

    /// `newTextureWithDescriptor:offset:bytesPerRow:` on a buffer: a linear
    /// texture, or a texture buffer for `MTLTextureTypeTextureBuffer`.
    pub fn new_texture_from_buffer(
        &mut self,
        buffer_id: BufferId,
        descriptor: &TextureDescriptor,
        offset: u64,
        bytes_per_row: u64,
    ) -> Result<TextureId> {
        let buffer = self.resources.buffers.get(buffer_id.0)?;
        let (vk_buffer, memory, memory_offset, length) = (
            buffer.buffer,
            buffer.memory,
            buffer.memory_offset,
            buffer.length,
        );
        let texture = if descriptor.texture_type == TextureType::TextureBuffer {
            let info = resource::texture_buffer(
                &self.capabilities,
                descriptor.pixel_format,
                descriptor.width,
                descriptor.usage,
                length,
                offset,
            )?;
            let view = self.backend.create_buffer_view(vk_buffer, &info)?;
            let usage = descriptor.usage;
            let read = if usage.is_empty()
                || usage.contains(crate::metal::abi::TextureUsage::SHADER_READ)
            {
                self.descriptor(Descriptor::UniformTexelBuffer(view))?
            } else {
                super::super::binding::NULL_DESCRIPTOR
            };
            let write = if usage.contains(crate::metal::abi::TextureUsage::SHADER_WRITE) {
                self.descriptor(Descriptor::StorageTexelBuffer(view))?
            } else {
                super::super::binding::NULL_DESCRIPTOR
            };
            TextureObject {
                image: None,
                descriptor: *descriptor,
                id: ResourceId::new(read, write),
                views: Vec::new(),
                texel_views: alloc::vec![view],
                attachment_views: Vec::new(),
                attachment_storage: Vec::new(),
                tested_copies: Vec::new(),
                cpu_access: CpuAccess::Linear,
                backing: Backing::Buffer(buffer_id),
                memory: None,
                parent: None,
                dependents: 0,
                cpu_copy: None,
                second_plane: None,
            }
        } else {
            let plan = resource::linear_texture(
                &self.capabilities,
                descriptor,
                length,
                offset,
                bytes_per_row,
            )?;
            let (_, linear) = self.capabilities.linear_texture_alignments();
            let place = memory_offset
                .checked_add(offset)
                .filter(|place| place.is_multiple_of(linear))
                .ok_or(Error::Invalid(
                    "the buffer's memory is not aligned for a texture",
                ))?;
            let image = self
                .backend
                .create_placed_image(&plan.image, memory, place)?;
            let (id, views) = match self.shader_views(image, &plan.views) {
                Ok(created) => created,
                Err(error) => {
                    self.backend.destroy(Object::Image(image));
                    return Err(error);
                }
            };
            TextureObject {
                image: Some(image),
                descriptor: *descriptor,
                id,
                views,
                texel_views: Vec::new(),
                attachment_views: Vec::new(),
                attachment_storage: Vec::new(),
                tested_copies: Vec::new(),
                cpu_access: CpuAccess::Linear,
                backing: Backing::Buffer(buffer_id),
                memory: None,
                parent: None,
                dependents: 0,
                cpu_copy: None,
                second_plane: None,
            }
        };
        self.resources.buffers.get_mut(buffer_id.0)?.dependents += 1;
        self.book_texture_over(buffer_id, descriptor);
        Ok(TextureId(self.resources.textures.insert(texture)))
    }

    /// Names the texture `descriptor` made over buffer `id` as what owns
    /// the buffer's memory, when the buffer has memory of its own (an
    /// IOSurface's or an app's imported pages).
    fn book_texture_over(&mut self, id: BufferId, descriptor: &TextureDescriptor) {
        let Ok(buffer) = self.resources.buffers.get(id.0) else {
            return;
        };
        if buffer.memory_offset != 0 || !matches!(buffer.backing, Backing::Owned(_)) {
            return;
        }
        let memory = buffer.memory;
        let length = buffer.length;
        self.book.describe(
            memory,
            format!(
                "buffer of {length} bytes under a {}x{} {:?} texture",
                descriptor.width, descriptor.height, descriptor.pixel_format
            ),
        );
    }

    /// Notes `connection` as the first holder of `owner`'s memory in the
    /// book.
    pub fn book_holder(&mut self, owner: MemoryOwner, connection: u64) {
        let memory = match owner {
            MemoryOwner::Import(id) => self
                .resources
                .imports
                .get(id.0)
                .ok()
                .map(|import| import.memory),
            MemoryOwner::Buffer(id) => self
                .resources
                .buffers
                .get(id.0)
                .ok()
                .map(|buffer| buffer.memory),
            MemoryOwner::Texture(id) => {
                self.resources
                    .textures
                    .get(id.0)
                    .ok()
                    .and_then(|texture| match texture.backing {
                        Backing::Buffer(buffer) => self
                            .resources
                            .buffers
                            .get(buffer.0)
                            .ok()
                            .map(|buffer| buffer.memory),
                        _ => texture.memory,
                    })
            }
            MemoryOwner::Heap(id) => self.resources.heaps.get(id.0).ok().map(|heap| heap.memory),
        };
        if let Some(memory) = memory {
            self.book.hold(memory, connection);
        }
    }

    /// `newTextureWithDescriptor:iosurface:plane:` of a two-plane format
    /// ([`PixelFormat::planes`]) on the buffer over the surface: each
    /// plane's linear image, whose sampled descriptors are consecutive, the
    /// second's following the first's, as shaders combine them
    /// ([`ResourceId::with_planes`]), YCbCr in the range of the surface's
    /// pixel format `surface_format` ([`video_range`]). Usages beyond
    /// reading are reported: rendering to and writing these textures is not
    /// implemented.
    pub fn new_two_plane_texture(
        &mut self,
        buffer_id: BufferId,
        descriptor: &TextureDescriptor,
        planes: [resource::PlaneLayout; 2],
        surface_format: u32,
    ) -> Result<TextureId> {
        let ratio = descriptor
            .pixel_format
            .planes()
            .ok_or(Error::Invalid("the format has one plane"))?;
        if !descriptor.swizzle.is_identity() {
            return Err(Error::Unsupported("swizzled two-plane textures"));
        }
        let renders = self
            .capabilities
            .format(descriptor.pixel_format)
            .contains(crate::metal::capabilities::FormatCapabilities::COLOR);
        let supported = if renders {
            TextureUsage::SHADER_READ | TextureUsage::RENDER_TARGET
        } else {
            TextureUsage::SHADER_READ
        };
        let unsupported = descriptor.usage.difference(supported);
        if !descriptor.usage.is_empty() && !unsupported.is_empty() {
            self.unsupported(if renders {
                "two-plane textures' usages beyond reading and rendering"
            } else {
                "two-plane textures' usages beyond reading; rendering to subsampled ones"
            });
        }
        let buffer = self.resources.buffers.get(buffer_id.0)?;
        let (memory, memory_offset, length) = (buffer.memory, buffer.memory_offset, buffer.length);
        let plans = resource::two_plane_texture(&self.capabilities, descriptor, length, planes)?;
        let (_, linear) = self.capabilities.linear_texture_alignments();
        let mut images = Vec::with_capacity(2);
        let mut views = Vec::with_capacity(2);
        let created = (|| -> Result<ResourceId> {
            for (plan, layout) in plans.iter().zip(planes) {
                let place = memory_offset
                    .checked_add(layout.offset)
                    .filter(|place| place.is_multiple_of(linear))
                    .ok_or(Error::Invalid(
                        "the surface's plane is not aligned for a texture",
                    ))?;
                images.push(
                    self.backend
                        .create_placed_image(&plan.image, memory, place)?,
                );
                let image = images[images.len() - 1];
                let info = plan
                    .views
                    .sampled
                    .ok_or(Error::Invalid("a plane has no sampled view"))?;
                views.push(self.backend.create_image_view(image, &info)?);
            }
            let first = self.descriptors.allocate_run(HeapArray::SampledImages, 2)?;
            for (index, view) in (first..).zip(&views) {
                self.backend
                    .write_descriptor(index, Descriptor::SampledImage(*view));
            }
            let format = descriptor.pixel_format;
            let flag = |set: bool, flag: u32| if set { flag } else { 0 };
            let flags = flag(format.ycbcr(), ResourceId::YCBCR)
                | flag(format.ten_bit_planes(), ResourceId::TEN_BIT)
                | flag(
                    format.ycbcr() && video_range(surface_format),
                    ResourceId::VIDEO_RANGE,
                );
            Ok(ResourceId::new(first, NULL_DESCRIPTOR)
                .with_planes(ratio, flags)
                .with_filter_class(descriptor.pixel_format.filter_class()))
        })();
        let id = match created {
            Ok(id) => id,
            Err(error) => {
                for view in views {
                    self.backend.destroy(Object::ImageView(view));
                }
                for image in images {
                    self.backend.destroy(Object::Image(image));
                }
                return Err(error);
            }
        };
        let texture = TextureObject {
            image: Some(images[0]),
            descriptor: *descriptor,
            id,
            views,
            texel_views: Vec::new(),
            attachment_views: Vec::new(),
            attachment_storage: Vec::new(),
            tested_copies: Vec::new(),
            cpu_access: CpuAccess::Linear,
            backing: Backing::Buffer(buffer_id),
            memory: None,
            parent: None,
            dependents: 0,
            cpu_copy: None,
            second_plane: Some(images[1]),
        };
        self.resources.buffers.get_mut(buffer_id.0)?.dependents += 1;
        self.book_texture_over(buffer_id, descriptor);
        Ok(TextureId(self.resources.textures.insert(texture)))
    }

    /// The view a render pass attaches, created on first use and kept with
    /// the texture.
    pub fn attachment_view(
        &mut self,
        id: TextureId,
        view: AttachmentView<()>,
    ) -> Result<ImageView> {
        let texture = self.resources.textures.get(id.0)?;
        if texture.second_plane.is_some()
            && !self
                .capabilities
                .format(texture.descriptor.pixel_format)
                .contains(crate::metal::capabilities::FormatCapabilities::COLOR)
        {
            return Err(Error::Unsupported(
                "rendering to subsampled two-plane textures",
            ));
        }
        if let Some(&(_, cached)) = texture
            .attachment_views
            .iter()
            .find(|(key, _)| *key == view)
        {
            return Ok(cached);
        }
        if self.has_extended_planes(id) {
            return self.extended_attachment_view(id, view);
        }
        let image = texture
            .image
            .ok_or(Error::Invalid("texture buffers are not attachments"))?;
        let pixel_format = texture.descriptor.pixel_format;
        // A format rendering through its raw texels attaches them.
        let format = if self
            .capabilities
            .format(pixel_format)
            .contains(FormatCapabilities::RAW_RENDER)
        {
            resource::raw_texel_format(pixel_format)
                .ok_or(Error::Unsupported("rendering to this pixel format"))?
        } else {
            resource::vulkan_format(pixel_format)?
        };
        let info = ImageViewInfo {
            view_type: view.view_type,
            format,
            components: [crate::metal::vulkan::ComponentSwizzle::Identity; 4],
            range: view.range,
            usage: if texture.descriptor.pixel_format.describe().is_color() {
                crate::metal::vulkan::ImageUsage::COLOR_ATTACHMENT
            } else {
                crate::metal::vulkan::ImageUsage::DEPTH_STENCIL_ATTACHMENT
            },
            min_lod: 0.0,
        };
        let created = self.backend.create_image_view(image, &info)?;
        self.resources
            .textures
            .get_mut(id.0)?
            .attachment_views
            .push((view, created));
        Ok(created)
    }

    /// The `gpuResourceID`-shaped identifier through which framebuffer
    /// reads access a pass's color attachment: a storage view of the raw
    /// texels of its attached level and layers, created on first use and
    /// kept with the texture.
    pub fn attachment_storage(
        &mut self,
        id: TextureId,
        view: AttachmentView<()>,
    ) -> Result<ResourceId> {
        if self.has_extended_planes(id) {
            return Ok(self.extended_working(id, view)?.id);
        }
        let texture = self.resources.textures.get(id.0)?;
        if let Some(&(_, _, descriptor)) = texture
            .attachment_storage
            .iter()
            .find(|(key, ..)| *key == view)
        {
            return Ok(ResourceId::texture(NULL_DESCRIPTOR, descriptor, 1));
        }
        let image = texture
            .image
            .ok_or(Error::Invalid("texture buffers are not attachments"))?;
        let format = resource::raw_texel_format(texture.descriptor.pixel_format)
            .ok_or(Error::Unsupported("framebuffer reads of the pixel format"))?;
        let info = |format| ImageViewInfo {
            view_type: ImageViewType::D2Array,
            format,
            components: [crate::metal::vulkan::ComponentSwizzle::Identity; 4],
            range: view.range,
            usage: crate::metal::vulkan::ImageUsage::STORAGE,
            min_lod: 0.0,
        };
        if let Some(second) = texture.second_plane {
            return self.two_plane_storage(
                id,
                view,
                [image, second],
                [
                    &info(format),
                    &info(
                        texture
                            .descriptor
                            .pixel_format
                            .plane_formats()
                            .and_then(|[_, second]| resource::raw_texel_format(second))
                            .ok_or(Error::Invalid("a second plane has raw texels"))?,
                    ),
                ],
            );
        }
        let created = self.backend.create_image_view(image, &info(format))?;
        let descriptor = match self.descriptor(Descriptor::StorageImage(created)) {
            Ok(descriptor) => descriptor,
            Err(error) => {
                self.backend.destroy(Object::ImageView(created));
                return Err(error);
            }
        };
        self.resources
            .textures
            .get_mut(id.0)?
            .attachment_storage
            .push((view, created, descriptor));
        Ok(ResourceId::texture(NULL_DESCRIPTOR, descriptor, 1))
    }

    /// [`Self::attachment_storage`] of a two-plane texture: storage views
    /// of both planes' raw texels in consecutive descriptors, the second's
    /// following the first's, as fragment functions write them.
    fn two_plane_storage(
        &mut self,
        id: TextureId,
        view: AttachmentView<()>,
        images: [Image; 2],
        infos: [&ImageViewInfo; 2],
    ) -> Result<ResourceId> {
        let mut created = Vec::with_capacity(2);
        for (image, info) in images.into_iter().zip(infos) {
            match self.backend.create_image_view(image, info) {
                Ok(view) => created.push(view),
                Err(error) => {
                    for view in created {
                        self.backend.destroy(Object::ImageView(view));
                    }
                    return Err(error);
                }
            }
        }
        let first = match self.descriptors.allocate_run(HeapArray::StorageImages, 2) {
            Ok(first) => first,
            Err(error) => {
                for view in created {
                    self.backend.destroy(Object::ImageView(view));
                }
                return Err(error);
            }
        };
        let texture = self.resources.textures.get_mut(id.0)?;
        for (index, view_handle) in (first..).zip(&created) {
            texture.attachment_storage.push((view, *view_handle, index));
        }
        for (index, view_handle) in (first..).zip(created) {
            self.backend
                .write_descriptor(index, Descriptor::StorageImage(view_handle));
        }
        Ok(ResourceId::texture(NULL_DESCRIPTOR, first, 1))
    }

    /// The storage copy of an attached depth (or, with `stencil`, stencil)
    /// subresource, created on first use and kept with the texture: an
    /// image of the aspect's raw texels, a 16- or 32-bit word of depth or a
    /// byte of stencil, with the view's layers and the texture's samples,
    /// which image copies fill from the attachment and return to it.
    pub fn tested_copy(
        &mut self,
        id: TextureId,
        view: AttachmentView<()>,
        stencil: bool,
    ) -> Result<TestedCopy> {
        let texture = self.resources.textures.get(id.0)?;
        let aspect = if stencil {
            ImageAspects::STENCIL
        } else {
            ImageAspects::DEPTH
        };
        if let Some(object) = texture
            .tested_copies
            .iter()
            .find(|object| object.view == view && object.aspect == aspect)
        {
            return Ok(object.copy);
        }
        let realized = texture
            .descriptor
            .pixel_format
            .describe()
            .realization
            .map_err(|_| Error::Unsupported("depth and stencil copies of the pixel format"))?
            .format;
        let (format, encoding) = if stencil {
            (Format::R8Uint, 0)
        } else {
            match realized {
                Format::D16Unorm => (Format::R16Uint, 1),
                Format::D24UnormS8Uint => (Format::R32Uint, 2),
                Format::D32Sfloat | Format::D32SfloatS8Uint => (Format::R32Uint, 3),
                _ => {
                    return Err(Error::Unsupported(
                        "depth and stencil copies of the pixel format",
                    ));
                }
            }
        };
        self.storage_copy(id, view, aspect, format, encoding)
    }

    /// Whether framebuffer reads of the texture as a color attachment go
    /// through its storage copy ([`color_copy`](Self::color_copy)) though a
    /// raw view reads its texels: a one-plane linear image over a buffer's
    /// rows, an IOSurface's. A backend may render such an image through a
    /// tiled shadow while a pass instance lasts, as NVK does unless every
    /// attachment of the instance is linear: a storage view of the image
    /// then reads what the instance began with, and the shadow's store
    /// where it ends overwrites the storage writes.
    pub fn reads_attachment_through_copy(&self, id: TextureId) -> Result<bool> {
        let mut texture = self.resources.textures.get(id.0)?;
        let mut root = id;
        while let Some(parent) = texture.parent {
            root = parent;
            texture = self.resources.textures.get(parent.0)?;
        }
        Ok(matches!(texture.backing, Backing::Buffer(_))
            && texture.image.is_some()
            && texture.second_plane.is_none()
            && !self.has_extended_planes(id)
            && !self.has_extended_planes(root))
    }

    /// The storage copy of an attached color subresource that has no raw
    /// texel storage view of its own, an A8Unorm texture's or a 3D
    /// texture's depth planes, or whose reads go through a copy
    /// ([`reads_attachment_through_copy`](Self::reads_attachment_through_copy)),
    /// created on first use and kept with the texture: an image of its raw
    /// texels with the view's layers (or planes) and the texture's samples,
    /// which image copies fill from the attachment and return to it, as
    /// [`tested_copy`](Self::tested_copy)'s.
    pub fn color_copy(&mut self, id: TextureId, view: AttachmentView<()>) -> Result<TestedCopy> {
        let texture = self.resources.textures.get(id.0)?;
        if let Some(object) = texture
            .tested_copies
            .iter()
            .find(|object| object.view == view && object.aspect == ImageAspects::COLOR)
        {
            return Ok(object.copy);
        }
        let format = resource::raw_texel_bytes_format(texture.descriptor.pixel_format)
            .ok_or(Error::Unsupported("framebuffer reads of the pixel format"))?;
        self.storage_copy(id, view, ImageAspects::COLOR, format, 0)
    }

    /// Creates and keeps the copy [`tested_copy`](Self::tested_copy) and
    /// [`color_copy`](Self::color_copy) describe.
    fn storage_copy(
        &mut self,
        id: TextureId,
        view: AttachmentView<()>,
        aspect: ImageAspects,
        format: Format,
        encoding: u32,
    ) -> Result<TestedCopy> {
        let texture = self.resources.textures.get(id.0)?;
        let attachment = texture
            .image
            .ok_or(Error::Invalid("texture buffers are not attachments"))?;
        let descriptor = texture.descriptor;
        let planes = descriptor.texture_type == crate::metal::abi::TextureType::D3;
        let level = view.range.base_mip_level;
        let extent = [
            (descriptor.width >> level).max(1) as u32,
            (descriptor.height >> level).max(1) as u32,
        ];
        let layers = view.range.layer_count;
        let info = ImageInfo {
            flags: ImageCreate::EMPTY,
            image_type: ImageType::D2,
            format,
            extent: [extent[0], extent[1], 1],
            mip_levels: 1,
            array_layers: layers,
            samples: SampleCounts::single(descriptor.sample_count.max(1) as u32)
                .ok_or(Error::Invalid("texture sample count"))?,
            tiling: ImageTiling::Optimal,
            usage: ImageUsage::STORAGE | ImageUsage::TRANSFER_SRC | ImageUsage::TRANSFER_DST,
            view_formats: Vec::new(),
            linear: None,
        };
        let request = MemoryRequest {
            required: MemoryProperties::DEVICE_LOCAL,
            preferred: MemoryProperties::EMPTY,
            mapped: false,
        };
        let (image, memory, bytes) = self.backend.create_image(&info, request)?;
        self.allocated += bytes;
        let view_info = ImageViewInfo {
            view_type: ImageViewType::D2Array,
            format,
            components: [crate::metal::vulkan::ComponentSwizzle::Identity; 4],
            range: SubresourceRange {
                aspects: ImageAspects::COLOR,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: layers,
            },
            usage: ImageUsage::STORAGE,
            min_lod: 0.0,
        };
        let created = match self.backend.create_image_view(image, &view_info) {
            Ok(image_view) => match self.descriptor(Descriptor::StorageImage(image_view)) {
                Ok(descriptor) => Ok((image_view, descriptor)),
                Err(error) => {
                    self.backend.destroy(Object::ImageView(image_view));
                    Err(error)
                }
            },
            Err(error) => Err(error),
        };
        let (image_view, storage) = match created {
            Ok(created) => created,
            Err(error) => {
                self.backend.destroy(Object::Image(image));
                self.release_memory(memory, bytes);
                return Err(error);
            }
        };
        let copy = TestedCopy {
            id: ResourceId::texture(NULL_DESCRIPTOR, storage, 1),
            image,
            attachment,
            aspect,
            level,
            base_layer: view.range.base_array_layer,
            layers,
            extent,
            encoding,
            planes,
        };
        self.resources
            .textures
            .get_mut(id.0)?
            .tested_copies
            .push(TestedCopyObject {
                view,
                aspect,
                copy,
                memory,
                bytes,
                image_view,
                descriptor: storage,
            });
        Ok(copy)
    }

    /// For a buffer copy of `level` and `slice` of a texture whose image
    /// stores another encoding than its format's bytes, the raw storage view
    /// of that subresource the texel kernels convert through, created on
    /// first use and kept with the texture. Null for an exact encoding, which
    /// a transfer copies, and for 1D and 3D textures, which have no such view.
    pub fn texel_storage(&mut self, id: TextureId, level: u64, slice: u64) -> Result<ResourceId> {
        let descriptor = *self.texture_descriptor(id)?;
        if descriptor.pixel_format.describe().encoding().is_exact()
            || !resource::has_texel_storage(descriptor.texture_type)
        {
            return Ok(ResourceId::NULL);
        }
        ensure(
            level < descriptor.mipmap_level_count && slice < descriptor.slice_count(),
            "subresource outside the texture",
        )?;
        let view = AttachmentView {
            texture: (),
            view_type: ImageViewType::D2Array,
            range: SubresourceRange {
                aspects: ImageAspects::COLOR,
                base_mip_level: level as u32,
                level_count: 1,
                base_array_layer: slice as u32,
                layer_count: 1,
            },
        };
        self.attachment_storage(id, view)
    }

    /// `region` as bands that each hold whole block rows of one image, whose
    /// bytes span at most [`CPU_BAND_BYTES`] of the caller's layout (one row
    /// at least). A transfer that moves a band at a time holds a bounded
    /// amount of memory, whatever the texture's size.
    pub fn cpu_bands(
        &self,
        id: TextureId,
        region: &Region,
        strides: Strides,
    ) -> Result<Vec<CpuBand>> {
        let (region, strides) = self.block_run(id, region, strides)?;
        let region = &region;
        let descriptor = &self.resources.textures.get(id.0)?.descriptor;
        let shape = RowShape::of(descriptor, region, strides)?;
        let per_band = CPU_BAND_BYTES
            .checked_div(strides.bytes_per_row)
            .map_or(shape.rows, |rows| rows.max(1));
        let block_height = u64::from(descriptor.pixel_format.describe().block.height);
        let mut bands = Vec::new();
        for image in 0..shape.images {
            let mut row = 0;
            while row < shape.rows {
                let count = per_band.min(shape.rows - row);
                let start = shape.start(strides, image, row)?;
                let end = shape.start(strides, image, row + count - 1)? + shape.row_bytes;
                let texel_row = row * block_height;
                bands.push(CpuBand {
                    region: Region {
                        origin: [
                            region.origin[0],
                            region.origin[1] + texel_row,
                            region.origin[2] + image,
                        ],
                        size: [
                            region.size[0],
                            (count * block_height).min(region.size[1] - texel_row),
                            1,
                        ],
                        ..*region
                    },
                    bytes: start..end,
                });
                row += count;
            }
        }
        Ok(bands)
    }

    /// `replaceRegion:mipmapLevel:slice:withBytes:bytesPerRow:bytesPerImage:`,
    /// for textures the CPU can write.
    pub fn replace_region(
        &mut self,
        id: TextureId,
        region: &Region,
        bytes: &[u8],
        strides: Strides,
    ) -> Result<()> {
        let (region, strides) = self.block_run(id, region, strides)?;
        let region = &region;
        let (copy, rows) = self.cpu_region(id, region, strides, bytes.len())?;
        if rows.tight() {
            return self.write_region(id, &copy, &bytes[..rows.packed_bytes]);
        }
        let mut packed = Vec::with_capacity(rows.packed_bytes);
        for row in rows.ranges() {
            packed.extend_from_slice(&bytes[row]);
        }
        self.write_region(id, &copy, &packed)
    }

    /// `getBytes:bytesPerRow:bytesPerImage:fromRegion:mipmapLevel:slice:`.
    pub fn get_bytes(
        &mut self,
        id: TextureId,
        region: &Region,
        bytes: &mut [u8],
        strides: Strides,
    ) -> Result<()> {
        let (region, strides) = self.block_run(id, region, strides)?;
        let region = &region;
        let (copy, rows) = self.cpu_region(id, region, strides, bytes.len())?;
        let mut packed = alloc::vec![0; rows.packed_bytes];
        self.read_region(id, &copy, &mut packed)?;
        let start = self.timing.now();
        if rows.tight() {
            bytes[..rows.packed_bytes].copy_from_slice(&packed);
        } else {
            let mut packed = packed.as_slice();
            for row in rows.ranges() {
                let (head, rest) = packed.split_at(row.len());
                bytes[row].copy_from_slice(head);
                packed = rest;
            }
        }
        self.timing.since(Phase::BytesUnpack, start);
        Ok(())
    }

    /// A CPU transfer's region and the caller's strides as
    /// [`Region::block_run`] has them: a run of blocks is packed.
    fn block_run(
        &self,
        id: TextureId,
        region: &Region,
        strides: Strides,
    ) -> Result<(Region, Strides)> {
        let descriptor = &self.resources.textures.get(id.0)?.descriptor;
        let run = region.block_run(descriptor)?;
        if run == *region {
            return Ok((run, strides));
        }
        let block = descriptor.pixel_format.describe().block;
        let packed = Strides {
            bytes_per_row: run.size[0].div_ceil(u64::from(block.width)) * u64::from(block.bytes),
            bytes_per_image: 0,
        };
        Ok((run, packed))
    }

    /// The copy of a color texture's `region` and the ranges of its rows in
    /// `length` bytes laid out by `strides`.
    fn cpu_region(
        &self,
        id: TextureId,
        region: &Region,
        strides: Strides,
        length: usize,
    ) -> Result<(BufferImageCopy, CpuRows)> {
        let descriptor = &self.resources.textures.get(id.0)?.descriptor;
        let shape = RowShape::of(descriptor, region, strides)?;
        let (subresource, offset, extent) = region.resolve(descriptor, ImageAspects::COLOR)?;
        let end = if shape.images == 0 || shape.rows == 0 {
            0
        } else {
            shape.start(strides, shape.images - 1, shape.rows - 1)? + shape.row_bytes
        };
        ensure(end <= length as u64, "the bytes end before the region")?;
        // The validated non-overlapping layout contains all the packed bytes.
        let rows = CpuRows {
            strides,
            row_bytes: shape.row_bytes,
            rows: shape.rows,
            images: shape.images,
            packed_bytes: (shape.row_bytes * shape.rows * shape.images) as usize,
        };
        let copy = BufferImageCopy {
            buffer_offset: 0,
            buffer_row_length: 0,
            buffer_image_height: 0,
            subresource,
            offset,
            extent,
        };
        Ok((copy, rows))
    }

    /// Writes `region` from tightly packed bytes.
    fn write_region(
        &mut self,
        id: TextureId,
        region: &BufferImageCopy,
        bytes: &[u8],
    ) -> Result<()> {
        let texture = self.resources.textures.get(id.0)?;
        self.cpu_access(texture)?;
        let image = texture.image.ok_or(Error::Invalid(
            "texture buffers are written through their buffer",
        ))?;
        let Some(copy) = &texture.cpu_copy else {
            let description = texture.descriptor.pixel_format.describe();
            return match description.encoding() {
                Encoding::Exact => self.backend.copy_to_image(image, region, bytes),
                encoding => {
                    let mut stored = alloc::vec![0; stored_len(description, bytes.len())];
                    // A two-plane extended-range texel keeps its alpha.
                    if matches!(encoding, Encoding::ExtendedRange(codes) if codes.planes) {
                        self.backend.copy_from_image(image, region, &mut stored)?;
                    }
                    encoding.store(bytes, &mut stored);
                    self.backend.copy_to_image(image, region, &stored)
                }
            };
        };
        let (memory, rows) = (
            copy.memory,
            cpu_copy_rows(&texture.descriptor, region, bytes.len())?,
        );
        let first = rows.first;
        for (offset, packed) in rows {
            self.backend.write_memory(memory, offset, &bytes[packed])?;
        }
        let upload = BufferImageCopy {
            buffer_offset: first,
            ..*region
        };
        if texture.cpu_access == CpuAccess::Blocks {
            let subresource = region.subresource;
            let level = u64::from(subresource.mip_level);
            let first = u64::from(subresource.base_array_layer);
            for slice in first..first + u64::from(subresource.layer_count) {
                let key = (id, level, slice);
                if !self.resources.block_decodes.iter().any(|(k, _)| *k == key) {
                    let decode = self.decode_subresource(id, level, slice)?;
                    self.resources.block_decodes.push((key, decode));
                }
            }
            return Ok(());
        }
        let upload = cpu_copy_region(&texture.descriptor, upload);
        self.resources.cpu_copy_uploads.push((id, upload));
        Ok(())
    }

    /// Reads `region` into tightly packed bytes.
    fn read_region(
        &mut self,
        id: TextureId,
        region: &BufferImageCopy,
        bytes: &mut [u8],
    ) -> Result<()> {
        let texture = self.resources.textures.get(id.0)?;
        self.cpu_access(texture)?;
        let image = texture.image.ok_or(Error::Invalid(
            "texture buffers are read through their buffer",
        ))?;
        let start = self.timing.now();
        let Some(copy) = &texture.cpu_copy else {
            let description = texture.descriptor.pixel_format.describe();
            return match description.encoding() {
                Encoding::Exact => {
                    let copied = self.backend.copy_from_image(image, region, bytes);
                    self.timing.since(Phase::BytesCopy, start);
                    copied
                }
                encoding => {
                    let mut stored = alloc::vec![0; stored_len(description, bytes.len())];
                    self.backend.copy_from_image(image, region, &mut stored)?;
                    let start = self.timing.since(Phase::BytesCopy, start);
                    encoding.load(&stored, bytes);
                    self.timing.since(Phase::BytesDecode, start);
                    Ok(())
                }
            };
        };
        let (memory, rows) = (
            copy.memory,
            cpu_copy_rows(&texture.descriptor, region, bytes.len())?,
        );
        for (offset, packed) in rows {
            self.backend
                .read_memory(memory, offset, &mut bytes[packed])?;
        }
        self.timing.since(Phase::BytesCopy, start);
        Ok(())
    }

    /// The CPU copy of a managed texture's `level` and `slice`, for
    /// `synchronizeTexture`; `None` when the CPU reaches the image directly.
    pub fn texture_cpu_copy(
        &self,
        id: TextureId,
        level: u64,
        slice: u64,
    ) -> Result<Option<BufferLayout>> {
        let texture = self.resources.textures.get(id.0)?;
        if texture.cpu_access != CpuAccess::Staged {
            return Ok(None);
        }
        self.cpu_copy_layout(texture, level, slice)
    }

    /// A decoded format's blocks in the CPU copy's layout, which buffer and
    /// texture copies read and write in place of its image; `None` for a
    /// texture whose image holds its format's blocks or texels.
    pub fn texture_blocks(&self, id: TextureId) -> Result<Option<BufferRef>> {
        let texture = self.resources.textures.get(id.0)?;
        Ok(texture
            .cpu_copy
            .as_ref()
            .filter(|_| texture.cpu_access == CpuAccess::Blocks)
            .map(|copy| BufferRef {
                buffer: copy.buffer,
                offset: 0,
                address: copy.address,
                length: texture.descriptor.cpu_copy_bytes(),
            }))
    }

    fn cpu_copy_layout(
        &self,
        texture: &TextureObject,
        level: u64,
        slice: u64,
    ) -> Result<Option<BufferLayout>> {
        let Some(copy) = &texture.cpu_copy else {
            return Ok(None);
        };
        let descriptor = &texture.descriptor;
        ensure(
            level < descriptor.mipmap_level_count && slice < descriptor.slice_count(),
            "subresource outside the texture",
        )?;
        let layout = descriptor.subresource_layout(level, slice);
        Ok(Some(BufferLayout {
            buffer: BufferRef {
                buffer: copy.buffer,
                offset: 0,
                address: copy.address,
                length: descriptor.cpu_copy_bytes(),
            },
            offset: layout.offset,
            bytes_per_row: layout.bytes_per_row,
            bytes_per_image: layout.bytes_per_image,
        }))
    }

    /// The textures the next commit's CPU-copy uploads and decodes write.
    pub(super) fn cpu_copy_textures(&self) -> Vec<TextureId> {
        let uploads = self.resources.cpu_copy_uploads.iter().map(|&(id, _)| id);
        let decodes = self.resources.block_decodes.iter().map(|&((id, ..), _)| id);
        let mut textures: Vec<TextureId> = uploads.chain(decodes).collect();
        textures.sort_unstable();
        textures.dedup();
        textures
    }

    /// Commands that bring images up to date with CPU-copy writes since the
    /// last commit, ending in a barrier; empty when there are none. A
    /// decoded format's written subresources decode whole from its blocks.
    pub(super) fn take_cpu_copy_uploads(&mut self) -> Vec<Command> {
        let uploads = core::mem::take(&mut self.resources.cpu_copy_uploads);
        let decodes = core::mem::take(&mut self.resources.block_decodes);
        let textures = &self.resources.textures;
        let mut commands: Vec<Command> = uploads
            .into_iter()
            .filter_map(|(id, region)| {
                // A texture destroyed since its write needs no upload.
                let texture = textures.get(id.0).ok()?;
                Some(Command::CopyBufferToImage {
                    src: texture.cpu_copy.as_ref()?.buffer,
                    dst: texture.image?,
                    region,
                })
            })
            .collect();
        commands.extend(
            decodes
                .into_iter()
                .filter(|((id, ..), _)| textures.get(id.0).is_ok())
                .map(|(_, decode)| decode),
        );
        if !commands.is_empty() {
            commands.push(Command::Barrier(FULL));
        }
        commands
    }

    /// The decode of a decoded format's whole subresource from its blocks,
    /// whose arguments the blocks' buffer keeps after them.
    fn decode_subresource(&mut self, id: TextureId, level: u64, slice: u64) -> Result<Command> {
        let storage = self.texel_storage(id, level, slice)?;
        let texture = self.resources.textures.get(id.0)?;
        let descriptor = texture.descriptor;
        let copy = texture
            .cpu_copy
            .as_ref()
            .ok_or(Error::Invalid("decoded textures keep their blocks"))?;
        let (memory, address) = (copy.memory, copy.address);
        let (kernel, encoding) = descriptor
            .pixel_format
            .describe()
            .decoder()
            .ok_or(Error::Invalid("the texture holds no decoded blocks"))?;
        let layout = descriptor.subresource_layout(level, slice);
        let [width, height, _] = descriptor.level_extent(level);
        let narrow = |value: u64| {
            u32::try_from(value).map_err(|_| Error::Unsupported("decodes of 4 GiB of texels"))
        };
        let arguments = TexelArguments {
            buffer: address + layout.offset,
            count: narrow(width * height)?,
            encoding,
            image: storage.0,
            width: narrow(width)?,
            height: narrow(height)?,
            bytes_per_row: narrow(layout.bytes_per_row)?,
            bytes_per_image: narrow(layout.bytes_per_image)?,
            origin: [0; 3],
            ..TexelArguments::default()
        };
        let offset = decode_arguments_offset(&descriptor)
            + (level * descriptor.slice_count() + slice) * DECODE_ARGUMENTS_STRIDE;
        self.backend
            .write_memory(memory, offset, &arguments.encode())?;
        Ok(Command::DecodeBlocks {
            kernel,
            arguments: address + offset,
            count: arguments.count,
        })
    }

    fn cpu_access(&self, texture: &TextureObject) -> Result<()> {
        if texture.second_plane.is_some() {
            // The CPU reaches the IOSurface's planes itself.
            return Err(Error::Unsupported("CPU copies of two-plane textures"));
        }
        ensure(
            texture.descriptor.options.storage_mode != StorageMode::Private,
            "private textures have no CPU access",
        )?;
        match texture.cpu_access {
            CpuAccess::HostImageCopy
            | CpuAccess::Linear
            | CpuAccess::Staged
            | CpuAccess::Blocks => Ok(()),
            CpuAccess::None => Err(Error::Invalid("the texture has no CPU access")),
        }
    }

    pub fn destroy_texture(&mut self, id: TextureId) -> Result<()> {
        ensure(
            self.resources.textures.get(id.0)?.dependents == 0,
            "views of the texture are still alive",
        )?;
        let texture = self.resources.textures.remove(id.0)?;
        self.forget_extended_planes(id);
        // Its decodes would write a view destroyed with it, and its uploads
        // would go to whatever texture takes its slot next.
        self.resources
            .block_decodes
            .retain(|((decoded, ..), _)| *decoded != id);
        self.resources
            .cpu_copy_uploads
            .retain(|(uploaded, _)| *uploaded != id);
        // Command buffers of any connection that carry its uploads read it.
        let scope = self
            .queue
            .carriers(id)
            .fold(self.scope.clone(), |scope, key| super::pins::Scope {
                reach: scope.reach.with(key),
                ..scope
            });
        self.release_for(scope, |device| match recyclable(&texture) {
            Some(bytes) if !device.lost => device.recycle_texture(texture, bytes),
            _ => device.free_texture(texture),
        })
    }

    /// What the device keeps of destroyed private textures.
    pub fn recycled(&self) -> RecycleReport {
        RecycleReport {
            textures: self.resources.recycled.len(),
            bytes: self.resources.recycled_bytes,
            hits: self.resources.recycle_hits,
            misses: self.resources.recycle_misses,
        }
    }

    /// A recycled texture of `descriptor`, zeroed as a new one, if the
    /// device keeps one no submitted work still uses.
    fn reuse_texture(
        &mut self,
        descriptor: &TextureDescriptor,
        plan: &TexturePlan,
    ) -> Option<TextureId> {
        if plan.cpu_access != CpuAccess::None || descriptor.pixel_format.describe().is_compressed()
        {
            return None;
        }
        // Not while a command buffer that may name it is in flight, held
        // ones included.
        let Some(index) =
            self.resources.recycled.iter().position(|entry| {
                entry.texture.descriptor == *descriptor && self.frees(&entry.ready)
            })
        else {
            self.resources.recycle_misses += 1;
            return None;
        };
        let entry = self.resources.recycled.remove(index)?;
        self.resources.recycled_bytes -= entry.bytes;
        self.resources.recycle_hits += 1;
        self.allocated += entry.bytes;
        let image = entry
            .texture
            .image
            .expect("a recycled texture owns its image");
        self.backend.clear_reused_image(image, &plan.image);
        Some(TextureId(self.resources.textures.insert(entry.texture)))
    }

    /// Keeps a destroyed texture of `bytes` for reuse, freeing the oldest
    /// kept ones beyond the budget.
    fn recycle_texture(&mut self, texture: TextureObject, bytes: u64) -> Result<()> {
        self.allocated -= bytes;
        self.resources.recycled_bytes += bytes;
        self.resources.recycled.push_back(Recycled {
            ready: self.release_now(),
            bytes,
            texture,
        });
        self.trim_recycled(RECYCLED_BYTES, RECYCLED_TEXTURES)
    }

    /// Frees the oldest recycled textures until at most `bytes` and
    /// `textures` remain.
    fn trim_recycled(&mut self, bytes: u64, textures: usize) -> Result<()> {
        while self.resources.recycled_bytes > bytes || self.resources.recycled.len() > textures {
            let Some(entry) = self.resources.recycled.pop_front() else {
                break;
            };
            self.resources.recycled_bytes -= entry.bytes;
            // Its bytes left `allocated` when it was recycled.
            self.allocated += entry.bytes;
            // Read by whoever could read it when it was destroyed.
            let scope = entry.ready.scope;
            self.release_for(scope, |device| device.free_texture(entry.texture))?;
        }
        Ok(())
    }

    /// Frees what a destroyed texture holds once submitted work completes.
    fn free_texture(&mut self, texture: TextureObject) -> Result<()> {
        let arrays = if texture.texel_views.is_empty() {
            TEXTURE_ARRAYS
        } else {
            TEXEL_BUFFER_ARRAYS
        };
        let low_length = if texture.second_plane.is_some() {
            2
        } else {
            resource::sampled_descriptors(resource::view_type(texture.descriptor.texture_type))
        };
        self.release_id(texture.id, arrays, low_length);
        for view in texture
            .views
            .iter()
            .chain(texture.attachment_views.iter().map(|(_, view)| view))
            .chain(texture.attachment_storage.iter().map(|(_, view, _)| view))
        {
            self.defer(Object::ImageView(*view));
        }
        for &(_, _, descriptor) in &texture.attachment_storage {
            self.release_descriptor(HeapArray::StorageImages, descriptor);
        }
        for copy in texture.tested_copies {
            self.defer(Object::ImageView(copy.image_view));
            self.release_descriptor(HeapArray::StorageImages, copy.descriptor);
            self.defer(Object::Image(copy.copy.image));
            self.release_memory(copy.memory, copy.bytes);
        }
        for view in texture.texel_views {
            self.defer(Object::BufferView(view));
        }
        match texture.parent {
            Some(parent) => self.resources.textures.get_mut(parent.0)?.dependents -= 1,
            None => {
                if let Some(image) = texture.image {
                    self.release_sparse(image)?;
                    self.defer(Object::Image(image));
                }
                if let Some(image) = texture.second_plane {
                    self.defer(Object::Image(image));
                }
                if let Some(copy) = texture.cpu_copy {
                    self.destroy_cpu_copy(copy);
                }
                self.release_backing(texture.backing, texture.memory)?;
            }
        }
        Ok(())
    }

    fn release_backing(&mut self, backing: Backing, memory: Option<Memory>) -> Result<()> {
        match (backing, memory) {
            (Backing::Owned(bytes), Some(memory)) => self.release_memory(memory, bytes),
            (Backing::Owned(_), None) => {}
            (
                Backing::Heap {
                    heap,
                    offset,
                    size,
                    aliasable,
                    real,
                    own,
                },
                memory,
            ) => {
                let heap = self.resources.heaps.get_mut(heap.0)?;
                heap.resources -= 1;
                if let (Some(allocator), false) = (&mut heap.allocator, aliasable) {
                    allocator.free(offset, size)?;
                }
                if let (Some(allocator), Some((at, length))) = (&mut heap.real, real) {
                    allocator.free(at, length)?;
                }
                if let (Some(memory), true) = (memory, own != 0) {
                    self.release_memory(memory, own);
                }
            }
            (Backing::Buffer(buffer), _) => {
                self.resources.buffers.get_mut(buffer.0)?.dependents -= 1
            }
            (Backing::Sparse(heap), _) => self.resources.heaps.get_mut(heap.0)?.resources -= 1,
        }
        Ok(())
    }

    /// `newSamplerStateWithDescriptor:`; equal descriptors share one state.
    pub fn new_sampler(&mut self, descriptor: &SamplerDescriptor) -> Result<SamplerId> {
        let existing = self.resources.samplers.entries.iter().position(|entry| {
            entry
                .as_ref()
                .is_some_and(|sampler| sampler.descriptor == *descriptor)
        });
        if let Some(index) = existing {
            self.resources.samplers.get_mut(index as u32)?.references += 1;
            return Ok(SamplerId(index as u32));
        }
        let (id, samplers) = self.create_sampler(descriptor)?;
        let object = SamplerObject {
            descriptor: *descriptor,
            samplers,
            id,
            clamped: Vec::new(),
            references: 1,
            released: None,
        };
        Ok(SamplerId(self.resources.samplers.insert(object)))
    }

    fn create_sampler(
        &mut self,
        descriptor: &SamplerDescriptor,
    ) -> Result<(ResourceId, Vec<Sampler>)> {
        let plan = sampler::sampler(&self.capabilities, descriptor)?;
        if descriptor.bicubic()
            && descriptor.mip_filter != SamplerMipFilter::NotMipmapped
            && (descriptor.lod_min_clamp > 0.0 || descriptor.lod_max_clamp < 16.0)
        {
            // Shaders select the bicubic level from the sample's LOD alone.
            self.unsupported("bicubic sampling of mip levels outside the sampler's LOD clamps");
        }
        let float = self.backend.create_sampler(&plan.float)?;
        let mut samplers = alloc::vec![float];
        let float_index = self.descriptor(Descriptor::Sampler(float))?;
        let integer_index = match plan.integer {
            Some(info) => {
                let integer = self.backend.create_sampler(&info)?;
                samplers.push(integer);
                self.descriptor(Descriptor::Sampler(integer))?
            }
            None => float_index,
        };
        Ok((
            ResourceId::sampler(float_index, integer_index, sampler_flags(descriptor)),
            samplers,
        ))
    }

    /// The sampler of a constexpr sampler state.
    pub(super) fn constexpr_sampler(&mut self, state: u64) -> Result<ResourceId> {
        if let Some(&id) = self.resources.constexpr_samplers.get(&state) {
            return Ok(id);
        }
        let sampler = self.new_sampler(&SamplerDescriptor::from_constexpr(state)?)?;
        let id = self.sampler_resource_id(sampler)?;
        self.resources.constexpr_samplers.insert(state, id);
        Ok(id)
    }

    /// `gpuResourceID` of a sampler.
    pub fn sampler_resource_id(&self, id: SamplerId) -> Result<ResourceId> {
        Ok(self.resources.samplers.get(id.0)?.id)
    }

    /// A sampler like the one `id` names with its LOD range clamped.
    pub(super) fn clamped_sampler(
        &mut self,
        id: ResourceId,
        lod_min: f32,
        lod_max: f32,
    ) -> Result<ResourceId> {
        ensure(
            lod_min >= 0.0 && lod_min <= lod_max,
            "LOD clamps are ordered and nonnegative",
        )?;
        let found = self
            .resources
            .samplers
            .entries
            .iter()
            .position(|entry| entry.as_ref().is_some_and(|sampler| sampler.id == id));
        let index = found.ok_or(Error::Invalid("unknown sampler"))? as u32;
        let sampler = self.resources.samplers.get(index)?;
        if let Some(&(_, _, clamped)) = sampler
            .clamped
            .iter()
            .find(|(min, max, _)| *min == lod_min && *max == lod_max)
        {
            return Ok(clamped);
        }
        let descriptor = SamplerDescriptor {
            lod_min_clamp: sampler.descriptor.lod_min_clamp.max(lod_min),
            lod_max_clamp: sampler.descriptor.lod_max_clamp.min(lod_max),
            ..sampler.descriptor
        };
        let (clamped, samplers) = self.create_sampler(&descriptor)?;
        let sampler = self.resources.samplers.get_mut(index)?;
        sampler.samplers.extend(samplers);
        sampler.clamped.push((lod_min, lod_max, clamped));
        Ok(clamped)
    }

    pub fn destroy_sampler(&mut self, id: SamplerId) -> Result<()> {
        let sampler = self.resources.samplers.get_mut(id.0)?;
        sampler.references -= 1;
        let released = match &sampler.released {
            Some(released) => released.union(&self.scope),
            None => self.scope.clone(),
        };
        if sampler.references > 0 {
            sampler.released = Some(released);
            return Ok(());
        }
        let sampler = self.resources.samplers.remove(id.0)?;
        self.release_for(released, |device| device.free_sampler(sampler));
        Ok(())
    }

    /// Frees a sampler state no holder keeps.
    fn free_sampler(&mut self, sampler: SamplerObject) {
        let ids = core::iter::once(sampler.id).chain(sampler.clamped.iter().map(|&(_, _, id)| id));
        for id in ids.collect::<Vec<_>>() {
            let low = ResourceId::new(id.low(), super::super::binding::NULL_DESCRIPTOR);
            self.release_id(low, [HeapArray::Samplers; 2], 1);
            if id.sampler_integer() != id.low() {
                self.release_id(
                    ResourceId::new(id.sampler_integer(), super::super::binding::NULL_DESCRIPTOR),
                    [HeapArray::Samplers; 2],
                    1,
                );
            }
        }
        for sampler in sampler.samplers {
            self.defer(Object::Sampler(sampler));
        }
    }

    /// `newHeapWithDescriptor:`.
    /// `newHeapWithDescriptor:` of a heap the CPU reaches: over imported
    /// memory, which it keeps.
    pub fn new_imported_heap(
        &mut self,
        id: MemoryId,
        descriptor: &HeapDescriptor,
    ) -> Result<HeapId> {
        let plan = heap::heap(
            &self.capabilities,
            descriptor,
            crate::metal::capabilities::ApiFeatures::IMPLEMENTED,
        )?;
        ensure(
            descriptor.heap_type != HeapType::Sparse,
            "sparse heaps are the device's",
        )?;
        let ImportObject { memory, bytes } = *self.resources.imports.get(id.0)?;
        // Its memory is whole pages, as the Mac rounds a heap.
        ensure(
            bytes == plan.size.next_multiple_of(heap::PAGE),
            "the heap is not the size of its memory",
        )?;
        self.resources.imports.remove(id.0)?;
        let heap = self.own_heap(descriptor, memory, None, bytes);
        self.resources.heaps.get_mut(heap.0)?.imported = true;
        self.book
            .describe(memory, format!("imported heap of {bytes} bytes"));
        Ok(heap)
    }

    pub fn new_heap(&mut self, descriptor: &HeapDescriptor) -> Result<HeapId> {
        let plan = heap::heap(
            &self.capabilities,
            descriptor,
            crate::metal::capabilities::ApiFeatures::IMPLEMENTED,
        )?;
        let sparse = match descriptor.heap_type {
            HeapType::Sparse => {
                let probe = sparse::probe(&self.capabilities, descriptor.options)?;
                let page = self
                    .capabilities
                    .sparse_tile_bytes(descriptor.sparse_page_size)
                    .ok_or(Error::Unsupported("sparse page size"))?;
                Some(SparseHeap {
                    pages: SparsePages::new(plan.size, page),
                    memory_types: self.backend.image_requirements(&probe)?.memory_types,
                })
            }
            HeapType::Automatic | HeapType::Placement => None,
        };
        let memory_types = sparse.as_ref().map_or(u32::MAX, |heap| heap.memory_types);
        // A Mac holds whole pages of resources whatever size was asked.
        let bytes = match sparse {
            Some(_) => plan.size,
            None => plan.size.next_multiple_of(heap::PAGE),
        };
        let memory = self
            .backend
            .allocate_memory(bytes, plan.memory, memory_types)?;
        self.allocated += bytes;
        self.book_memory(memory, bytes, || {
            format!("{:?} heap of {bytes} bytes", descriptor.heap_type)
        });
        Ok(self.own_heap(descriptor, memory, sparse, bytes))
    }

    fn own_heap(
        &mut self,
        descriptor: &HeapDescriptor,
        memory: Memory,
        sparse: Option<SparseHeap>,
        size: u64,
    ) -> HeapId {
        let automatic = descriptor.heap_type == HeapType::Automatic;
        HeapId(self.resources.heaps.insert(HeapObject {
            descriptor: *descriptor,
            memory,
            bytes: size,
            allocator: automatic.then(|| HeapAllocator::new(size)),
            real: automatic.then(|| HeapAllocator::new(size)),
            imported: false,
            sparse,
            resources: 0,
        }))
    }

    /// `heapBufferSizeAndAlignWithLength:options:`.
    pub fn heap_buffer_size_and_align(
        &mut self,
        length: u64,
        options: ResourceOptions,
    ) -> Result<SizeAndAlign> {
        let plan = resource::buffer(&self.capabilities, length, options)?;
        let requirements = self.backend.buffer_requirements(&plan.info)?;
        // A linear texture may later be made over any buffer, at an offset
        // Metal aligns only within the buffer; the image binds at the buffer's
        // place plus that offset, so the buffer starts where an image can.
        let (_, linear) = self.capabilities.linear_texture_alignments();
        SizeAndAlign::from_requirements(requirements.size, requirements.alignment.max(linear))
            .ok_or(Error::Invalid("requirements"))
    }

    /// `heapTextureSizeAndAlignWithDescriptor:`.
    pub fn heap_texture_size_and_align(
        &mut self,
        descriptor: &TextureDescriptor,
    ) -> Result<SizeAndAlign> {
        let plan = resource::heap_texture(&self.capabilities, descriptor)?;
        let layout = self.heap_texture_requirements(&plan)?;
        Ok(heap::apple_texture(descriptor).unwrap_or(layout))
    }

    fn heap_texture_requirements(&mut self, plan: &TexturePlan) -> Result<SizeAndAlign> {
        let requirements = self.backend.image_requirements(&plan.image)?;
        SizeAndAlign::from_requirements(requirements.size, requirements.alignment)
            .ok_or(Error::Invalid("requirements"))
    }

    /// Reserves heap memory: first fit in automatic heaps, the given offset
    /// in placement heaps.
    fn place(
        &mut self,
        id: HeapId,
        options: ResourceOptions,
        requirement: SizeAndAlign,
        offset: Option<u64>,
    ) -> Result<(Memory, u64)> {
        let heap = self.resources.heaps.get_mut(id.0)?;
        ensure(
            options.storage_mode == heap.descriptor.options.storage_mode,
            "heap resources share the heap's storage mode",
        )?;
        let offset = match (&mut heap.allocator, offset) {
            (Some(allocator), None) => {
                let Some(offset) = allocator.allocate(requirement.size, requirement.align) else {
                    let free = allocator.size() - allocator.used();
                    let refusals = &mut self.resources.heap_refusals;
                    if free >= requirement.size {
                        refusals.fragmented += 1;
                    } else {
                        refusals.full += 1;
                    }
                    return Err(Error::HeapFull);
                };
                offset
            }
            (None, Some(offset)) => {
                heap::placement(heap.bytes, offset, requirement)?;
                offset
            }
            (Some(_), Some(_)) => return Err(Error::Invalid("automatic heaps choose offsets")),
            (None, None) => return Err(Error::Invalid("placement heaps need an offset")),
        };
        heap.resources += 1;
        Ok((heap.memory, offset))
    }

    /// `newBufferWithLength:options:` (and `…offset:`) on a heap. An
    /// automatic heap's buffer lives at its offset in the heap's memory
    /// unless a texture's own layout took part of that range, and then in
    /// memory of its own.
    pub fn new_heap_buffer(
        &mut self,
        id: HeapId,
        length: u64,
        options: ResourceOptions,
        offset: Option<u64>,
    ) -> Result<BufferId> {
        let requirement = self.heap_buffer_size_and_align(length, options)?;
        let (memory, offset) = self.place(id, options, requirement, offset)?;
        let plan = resource::buffer(&self.capabilities, length, options)?;
        let heap = self.resources.heaps.get_mut(id.0)?;
        let real = match &mut heap.real {
            Some(real) => real
                .take(offset, requirement.size)
                .then_some((offset, requirement.size)),
            None => Some((offset, 0)),
        };
        if real.is_none() && heap.imported {
            // A texture's own layout holds that range of the CPU's memory.
            heap.resources -= 1;
            if let Some(allocator) = &mut heap.allocator {
                allocator.free(offset, requirement.size)?;
            }
            self.resources.heap_refusals.full += 1;
            return Err(Error::HeapFull);
        }
        let mut backing = Backing::Heap {
            heap: id,
            offset,
            size: requirement.size,
            aliasable: false,
            real: real.filter(|&(_, length)| length != 0),
            own: 0,
        };
        let created = match real {
            Some(_) => self
                .backend
                .create_placed_buffer(&plan.info, memory, offset)
                .map(|(buffer, address)| (buffer, memory, offset, address)),
            None => self
                .backend
                .create_buffer(&plan.info, plan.memory)
                .map(|own| {
                    self.allocated += own.allocated;
                    if let Backing::Heap { own: bytes, .. } = &mut backing {
                        *bytes = own.allocated;
                    }
                    (own.buffer, own.memory, 0, own.address)
                }),
        };
        let (buffer, memory, memory_offset, address) = match created {
            Ok(created) => created,
            Err(error) => {
                self.release_backing(backing, None)?;
                return Err(error);
            }
        };
        if address != 0 && address >= memory_offset {
            self.book.learn_address(memory, address - memory_offset);
        }
        Ok(BufferId(self.resources.buffers.insert(BufferObject {
            buffer,
            memory,
            memory_offset,
            address,
            length,
            options,
            backing,
            dependents: 0,
        })))
    }

    /// `newTextureWithDescriptor:` (and `…offset:`) on a heap. The heap
    /// counts the texture by the size and alignment a Mac answers
    /// ([`heap::apple_texture`]), so as many fit, at the same offsets, as
    /// there. The image lives where its own layout fits: a free range of an
    /// automatic heap's memory, a placement heap's offset when the layout
    /// is no larger than the answer, or else memory of its own.
    pub fn new_heap_texture(
        &mut self,
        id: HeapId,
        descriptor: &TextureDescriptor,
        offset: Option<u64>,
    ) -> Result<TextureId> {
        if self.resources.heaps.get(id.0)?.sparse.is_some() {
            ensure(offset.is_none(), "sparse heaps place textures themselves")?;
            return self.new_sparse_texture(id, descriptor);
        }
        let plan = resource::heap_texture(&self.capabilities, descriptor)?;
        let layout = self.heap_texture_requirements(&plan)?;
        let answer = heap::apple_texture(descriptor).unwrap_or(layout);
        let (memory, offset) = self.place(id, descriptor.options, answer, offset)?;
        let heap = self.resources.heaps.get_mut(id.0)?;
        let real = match &mut heap.real {
            Some(real) => real
                .allocate(layout.size, layout.align)
                .map(|at| Some((at, layout.size))),
            None => (offset.is_multiple_of(layout.align)
                && layout.size <= answer.size
                && offset + layout.size <= heap.bytes)
                .then_some(None),
        };
        let mut backing = Backing::Heap {
            heap: id,
            offset,
            size: answer.size,
            aliasable: false,
            real: real.flatten(),
            own: 0,
        };
        let created =
            match real {
                Some(range) => {
                    let at = range.map_or(offset, |(at, _)| at);
                    self.backend
                        .create_placed_image(&plan.image, memory, at)
                        .map(|image| (image, None))
                }
                None => self.backend.create_image(&plan.image, plan.memory).map(
                    |(image, own, bytes)| {
                        self.allocated += bytes;
                        if let Backing::Heap { own: owned, .. } = &mut backing {
                            *owned = bytes;
                        }
                        (image, Some(own))
                    },
                ),
            };
        let (image, own) = match created {
            Ok(created) => created,
            Err(error) => {
                self.release_backing(backing, None)?;
                return Err(error);
            }
        };
        self.finish_texture(image, descriptor, &plan, backing, own)
    }

    /// A texture on a sparse heap, with every tile unmapped.
    fn new_sparse_texture(
        &mut self,
        id: HeapId,
        descriptor: &TextureDescriptor,
    ) -> Result<TextureId> {
        let (plan, tile) = sparse::texture(&self.capabilities, descriptor)?;
        let heap = self.resources.heaps.get(id.0)?;
        ensure(
            descriptor.options.storage_mode == heap.descriptor.options.storage_mode,
            "heap resources share the heap's storage mode",
        )?;
        let memory_types = heap.sparse.as_ref().map_or(0, |sparse| sparse.memory_types);
        let page = self
            .capabilities
            .sparse_tile_bytes(heap.descriptor.sparse_page_size)
            .ok_or(Error::Unsupported("sparse page size"))?;
        let (image, requirements) = self.backend.create_sparse_image(&plan.image)?;
        let layout = require(
            requirements.memory_types & memory_types == memory_types,
            "sparse textures in this heap's memory",
        )
        .and_then(|()| SparseLayout::new(descriptor, tile, &requirements, page));
        let layout = match layout {
            Ok(layout) => layout,
            Err(error) => {
                self.backend.destroy(Object::Image(image));
                return Err(error);
            }
        };
        self.resources.heaps.get_mut(id.0)?.resources += 1;
        let texture = self.finish_texture(image, descriptor, &plan, Backing::Sparse(id), None)?;
        self.resources
            .sparse
            .insert(image, (id, SparseTexture::new(image, layout)));
        Ok(texture)
    }

    /// The image a resource state encoder names for a sparse texture.
    pub fn sparse_image(&self, id: TextureId) -> Result<Image> {
        let image = self.resources.textures.get(id.0)?.image;
        image
            .filter(|image| self.resources.sparse.contains_key(image))
            .ok_or(Error::Invalid("the texture is not sparse"))
    }

    /// `firstMipmapInTail` and `tailSizeInBytes`; `None` unless the texture
    /// is sparse (`isSparse`).
    pub fn sparse_tail(&self, id: TextureId) -> Result<Option<(u64, u64)>> {
        let image = self.resources.textures.get(id.0)?.image;
        Ok(image
            .and_then(|image| self.resources.sparse.get(&image))
            .map(|(_, texture)| (texture.layout.first_mip_in_tail, texture.layout.tail_bytes)))
    }

    pub(super) fn sparse_texture(&self, image: Image) -> Result<&(HeapId, SparseTexture)> {
        self.resources
            .sparse
            .get(&image)
            .ok_or(Error::Invalid("the texture is not sparse"))
    }

    /// Fails as [`Self::sparse_binds`] would, without changing anything, so
    /// a commit either applies all its mapping updates or none.
    pub(super) fn check_mapping(&self, request: &MappingRequest) -> Result<()> {
        match request {
            MappingRequest::Update { image, updates, .. } => {
                let (_, texture) = self.sparse_texture(*image)?;
                updates
                    .iter()
                    .try_for_each(|update| texture.check_update(update))
            }
            MappingRequest::Move {
                source,
                destination,
                size,
            } => {
                let (heap, from) = self.sparse_texture(source.image)?;
                let (other, to) = self.sparse_texture(destination.image)?;
                ensure(heap == other, "moved mappings stay in one heap")?;
                ensure(
                    from.check_move(source, *size)? == to.check_move(destination, *size)?,
                    "moves cover the same tiles in both textures",
                )
            }
        }
    }

    /// The sparse binds of a resource state encoder's request, applied to
    /// the device's tile and page state.
    pub(super) fn sparse_binds(&mut self, request: &MappingRequest) -> Result<Vec<SparseBind>> {
        let mut binds = Vec::new();
        let resources = &mut self.resources;
        match request {
            MappingRequest::Update {
                image,
                mode,
                updates,
            } => {
                let (heap, texture) = resources
                    .sparse
                    .get_mut(image)
                    .ok_or(Error::Invalid("the texture is not sparse"))?;
                let heap = resources.heaps.get_mut(heap.0)?;
                let pages = &mut heap.sparse.as_mut().expect("sparse heap").pages;
                for update in updates {
                    texture.update(*mode, update, pages, heap.memory, &mut binds)?;
                }
            }
            MappingRequest::Move {
                source,
                destination,
                size,
            } => {
                let (heap, texture) = resources
                    .sparse
                    .get_mut(&source.image)
                    .ok_or(Error::Invalid("the texture is not sparse"))?;
                let heap = resources.heaps.get_mut(heap.0)?;
                let moved = texture.take_region(source, *size, heap.memory, &mut binds)?;
                let (_, texture) = resources
                    .sparse
                    .get_mut(&destination.image)
                    .ok_or(Error::Invalid("the texture is not sparse"))?;
                let pages = &mut heap.sparse.as_mut().expect("sparse heap").pages;
                texture.place_region(destination, *size, moved, pages, heap.memory, &mut binds)?;
            }
        }
        Ok(binds)
    }

    /// Returns a destroyed sparse texture's pages to its heap.
    fn release_sparse(&mut self, image: Image) -> Result<()> {
        if let Some((heap, mut texture)) = self.resources.sparse.remove(&image) {
            let heap = self.resources.heaps.get_mut(heap.0)?;
            texture.release(&mut heap.sparse.as_mut().expect("sparse heap").pages);
        }
        Ok(())
    }

    /// `makeAliasable` of a heap buffer: its range returns to the heap now,
    /// once.
    pub fn make_buffer_aliasable(&mut self, id: BufferId) -> Result<()> {
        let buffer = self.resources.buffers.get_mut(id.0)?;
        match make_aliasable(&mut buffer.backing)? {
            Some(range) => self.free_heap_range(range),
            None => Ok(()),
        }
    }

    /// `makeAliasable` of a heap texture.
    pub fn make_texture_aliasable(&mut self, id: TextureId) -> Result<()> {
        let texture = self.resources.textures.get_mut(id.0)?;
        match make_aliasable(&mut texture.backing)? {
            Some(range) => self.free_heap_range(range),
            None => Ok(()),
        }
    }

    fn free_heap_range(&mut self, (heap, (offset, size), real): AliasedRanges) -> Result<()> {
        let heap = self.resources.heaps.get_mut(heap.0)?;
        if let Some(allocator) = &mut heap.allocator {
            allocator.free(offset, size)?;
        }
        if let (Some(allocator), Some((at, length))) = (&mut heap.real, real) {
            allocator.free(at, length)?;
        }
        Ok(())
    }

    /// The resources automatic heaps answered nil for so far.
    pub fn heap_refusals(&self) -> HeapRefusals {
        self.resources.heap_refusals
    }

    /// A heap's size, whole pages as a Mac answers `size` and
    /// `currentAllocatedSize`, and how many resources live in it.
    pub fn heap_occupancy(&self, id: HeapId) -> Result<(u64, u32)> {
        let heap = self.resources.heaps.get(id.0)?;
        Ok((heap.bytes, heap.resources))
    }

    /// `usedSize` and `maxAvailableSizeWithAlignment:` of a heap.
    pub fn heap_usage(&self, id: HeapId, alignment: u64) -> Result<(u64, u64)> {
        let heap = self.resources.heaps.get(id.0)?;
        Ok(match (&heap.allocator, &heap.sparse) {
            (Some(allocator), _) => (allocator.used(), allocator.max_available(alignment)),
            (None, Some(sparse)) => (sparse.pages.used_bytes(), sparse.pages.free_bytes()),
            (None, None) => (heap.bytes, heap.bytes),
        })
    }

    pub fn destroy_heap(&mut self, id: HeapId) -> Result<()> {
        ensure(
            self.resources.heaps.get(id.0)?.resources == 0,
            "resources still live in the heap",
        )?;
        let heap = self.resources.heaps.remove(id.0)?;
        self.release_memory(heap.memory, heap.bytes);
        Ok(())
    }
}

/// A heap, the range it counts a resource in, and the range of its memory
/// the resource reserved.
type AliasedRanges = (HeapId, (u64, u64), Option<(u64, u64)>);

fn owned_bytes(backing: Backing) -> u64 {
    match backing {
        Backing::Owned(bytes) => bytes,
        _ => 0,
    }
}

/// The heap ranges a resource gives back as it becomes aliasable, as the
/// heap counts it and of the heap's memory, which later resources may then
/// alias; none for one already aliasable, which Metal leaves as it is.
fn make_aliasable(backing: &mut Backing) -> Result<Option<AliasedRanges>> {
    match backing {
        Backing::Heap {
            heap,
            offset,
            size,
            aliasable,
            real,
            ..
        } => {
            let first = !*aliasable;
            *aliasable = true;
            Ok(first.then(|| (*heap, (*offset, *size), real.take())))
        }
        _ => Err(Error::Invalid("only heap resources become aliasable")),
    }
}

/// Validated row layout in the caller's bytes.
struct CpuRows {
    strides: Strides,
    row_bytes: u64,
    rows: u64,
    images: u64,
    packed_bytes: usize,
}

impl CpuRows {
    fn tight(&self) -> bool {
        self.packed_bytes == 0
            || (self.rows <= 1 || self.strides.bytes_per_row == self.row_bytes)
                && (self.images <= 1 || self.strides.bytes_per_image == self.row_bytes * self.rows)
    }

    fn ranges(&self) -> impl Iterator<Item = Range<usize>> + '_ {
        (0..self.images).flat_map(move |image| {
            (0..self.rows).map(move |row| {
                let start = image * self.strides.bytes_per_image + row * self.strides.bytes_per_row;
                start as usize..(start + self.row_bytes) as usize
            })
        })
    }
}

/// The bytes of a destroyed texture the device may keep for reuse: a
/// private texture with an allocation of its own, no CPU copy or texel
/// views, of a format a clear can zero.
fn recyclable(texture: &TextureObject) -> Option<u64> {
    match texture.backing {
        Backing::Owned(bytes)
            if (1..=RECYCLED_TEXTURE_BYTES).contains(&bytes)
                && texture.parent.is_none()
                && texture.image.is_some()
                && texture.memory.is_some()
                && texture.cpu_access == CpuAccess::None
                && texture.cpu_copy.is_none()
                && texture.texel_views.is_empty()
                && !texture.descriptor.pixel_format.describe().is_compressed() =>
        {
            Some(bytes)
        }
        _ => None,
    }
}

/// Contiguous spans `region` covers in a texture's CPU copy, as (copy offset,
/// range in tightly packed bytes of `length`); `first` is the region's
/// first byte in the copy.
#[derive(Debug)]
struct CopyRows {
    first: u64,
    bytes_per_row: u64,
    bytes_per_image: u64,
    row_bytes: usize,
    rows: u64,
    images: u64,
    rows_per_copy: u64,
    next: u64,
}

impl Iterator for CopyRows {
    type Item = (u64, core::ops::Range<usize>);

    fn next(&mut self) -> Option<Self::Item> {
        if self.next == self.rows * self.images {
            return None;
        }
        let (image, row) = (self.next / self.rows, self.next % self.rows);
        let offset = self.first + image * self.bytes_per_image + row * self.bytes_per_row;
        let start = self.next as usize * self.row_bytes;
        self.next += self.rows_per_copy;
        Some((
            offset,
            start..start + self.rows_per_copy as usize * self.row_bytes,
        ))
    }
}

/// Validates a tightly packed region of `length` bytes against the texture
/// and locates its rows in the CPU copy.
/// Bytes of the image's texels holding `metal_len` bytes of Metal's.
fn stored_len(description: &Description, metal_len: usize) -> usize {
    metal_len / description.block.bytes as usize * description.stored_block_bytes() as usize
}

/// Bytes of each subresource's decode arguments after a decoded format's
/// blocks.
const DECODE_ARGUMENTS_STRIDE: u64 = 64;

/// Where a decoded format's decode arguments start in its blocks' buffer.
fn decode_arguments_offset(descriptor: &TextureDescriptor) -> u64 {
    descriptor
        .cpu_copy_bytes()
        .next_multiple_of(TexelArguments::ALIGNMENT)
}

fn cpu_copy_rows(
    descriptor: &TextureDescriptor,
    region: &BufferImageCopy,
    length: usize,
) -> Result<CopyRows> {
    let subresource = &region.subresource;
    let (level, slice) = (
        u64::from(subresource.mip_level),
        u64::from(subresource.base_array_layer),
    );
    ensure(
        subresource.layer_count == 1
            && level < descriptor.mipmap_level_count
            && slice < descriptor.slice_count(),
        "one subresource of the texture",
    )?;
    let SubresourceLayout {
        offset,
        bytes_per_row,
        bytes_per_image,
        extent,
    } = descriptor.subresource_layout(level, slice);
    let block = descriptor.pixel_format.describe().block;
    let block_size = [u64::from(block.width), u64::from(block.height), 1];
    let origin = region.offset.map(|offset| offset as u64);
    let size = region.extent.map(u64::from);
    for axis in 0..3 {
        let end = origin[axis] + size[axis];
        ensure(end <= extent[axis], "region outside the texture level")?;
        ensure(
            origin[axis].is_multiple_of(block_size[axis])
                && (size[axis].is_multiple_of(block_size[axis]) || end == extent[axis]),
            "compressed regions cover whole blocks",
        )?;
    }
    let row_bytes = size[0].div_ceil(block_size[0]) * u64::from(block.bytes);
    let rows = size[1].div_ceil(block_size[1]);
    ensure(
        row_bytes * rows * size[2] == length as u64,
        "the bytes are the region, tightly packed",
    )?;
    let rows_per_copy = if bytes_per_row != row_bytes {
        1
    } else if bytes_per_image != bytes_per_row * rows {
        rows
    } else {
        rows * size[2]
    };
    Ok(CopyRows {
        first: offset
            + origin[2] * bytes_per_image
            + origin[1] / block_size[1] * bytes_per_row
            + origin[0] / block_size[0] * u64::from(block.bytes),
        bytes_per_row,
        bytes_per_image,
        row_bytes: row_bytes as usize,
        rows,
        images: size[2],
        rows_per_copy,
        next: 0,
    })
}

/// The most bytes of the caller's layout that one band of a CPU texture
/// transfer spans ([`Device::cpu_bands`]).
pub const CPU_BAND_BYTES: u64 = 4 << 20;

/// Whole block rows of one image of a CPU texture transfer: `region`, whose
/// texels lie at `bytes` of the caller's layout, in the transfer's strides.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CpuBand {
    pub region: Region,
    pub bytes: Range<u64>,
}

/// The block rows of a color region in CPU memory.
struct RowShape {
    row_bytes: u64,
    rows: u64,
    images: u64,
}

impl RowShape {
    /// Validates `strides` for `region` of a color texture. Every row's
    /// start, at most the last one's, fits in `u64`.
    fn of(descriptor: &TextureDescriptor, region: &Region, strides: Strides) -> Result<Self> {
        let description = descriptor.pixel_format.describe();
        ensure(
            description.is_color(),
            "the CPU reaches depth and stencil textures through blits",
        )?;
        region.resolve(descriptor, ImageAspects::COLOR)?;
        let block = description.block;
        let shape = Self {
            row_bytes: region.size[0].div_ceil(u64::from(block.width)) * u64::from(block.bytes),
            rows: region.size[1].div_ceil(u64::from(block.height)),
            images: region.size[2],
        };
        ensure(
            strides.bytes_per_row >= shape.row_bytes,
            "bytesPerRow is smaller than a row",
        )?;
        ensure(
            shape.images <= 1
                || strides
                    .bytes_per_row
                    .checked_mul(shape.rows)
                    .is_some_and(|image| strides.bytes_per_image >= image),
            "bytesPerImage is smaller than an image",
        )?;
        if shape.rows > 0 && shape.images > 0 {
            shape.start(strides, shape.images - 1, shape.rows - 1)?;
        }
        Ok(shape)
    }

    /// Where `row` of `image` starts in the caller's layout.
    fn start(&self, strides: Strides, image: u64, row: u64) -> Result<u64> {
        image
            .checked_mul(strides.bytes_per_image)
            .zip(row.checked_mul(strides.bytes_per_row))
            .and_then(|(image, row)| image.checked_add(row))
            .and_then(|start| start.checked_add(self.row_bytes).map(|_| start))
            .ok_or(Error::Invalid("the bytes end before the region"))
    }
}

/// The row strides of texels in CPU memory: `bytesPerRow`, and
/// `bytesPerImage`, which only regions deeper than one image read.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Strides {
    pub bytes_per_row: u64,
    pub bytes_per_image: u64,
}

/// `region` reading its texels from the CPU copy, whose rows span the whole
/// level.
fn cpu_copy_region(descriptor: &TextureDescriptor, region: BufferImageCopy) -> BufferImageCopy {
    let extent = descriptor.level_extent(region.subresource.mip_level.into());
    let block = descriptor.pixel_format.describe().block;
    BufferImageCopy {
        buffer_row_length: extent[0].next_multiple_of(block.width.into()) as u32,
        buffer_image_height: extent[1].next_multiple_of(block.height.into()) as u32,
        ..region
    }
}

/// A sampler ID's flags (`ResourceId::SAMPLER_*`).
fn sampler_flags(descriptor: &SamplerDescriptor) -> u32 {
    let mut flags = 0;
    if descriptor.compare_function == CompareFunction::Never {
        flags |= ResourceId::SAMPLER_NEVER;
    }
    if descriptor.bicubic() {
        flags |= ResourceId::SAMPLER_CUBIC;
        if descriptor.mip_filter != SamplerMipFilter::NotMipmapped {
            flags |= ResourceId::SAMPLER_MIPPED;
        }
    }
    if !descriptor.normalized_coordinates {
        flags |= ResourceId::SAMPLER_PIXELS;
    }
    flags
}

/// Whether an IOSurface pixel format code names video-range YCbCr, as
/// CoreVideo's codes do: `'420v'`, `'x420'`, `'422v'`, `'x444'` and their
/// lossless `'&'` forms are video range; full-range codes (`'420f'`,
/// `'xf20'`, `'444f'`) carry an `f`, and zero, no surface, is full range.
pub fn video_range(surface_format: u32) -> bool {
    surface_format != 0 && !surface_format.to_be_bytes().contains(&b'f')
}

#[cfg(test)]
mod surface_range_tests {
    use super::video_range;

    #[test]
    fn surface_codes_name_their_range() {
        let code = |s: &[u8; 4]| u32::from_be_bytes(*s);
        for video in [b"420v", b"x420", b"422v", b"x422", b"x444", b"&8v0"] {
            assert!(video_range(code(video)));
        }
        for full in [b"420f", b"xf20", b"444f", b"xf44", b"&8f0"] {
            assert!(!video_range(code(full)));
        }
        assert!(!video_range(0));
    }
}
