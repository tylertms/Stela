//! Command buffers, submission, completion, events and counters.
//!
//! Every `MTLCommandQueue` submits to the device's one universal queue, each
//! in commit order, which satisfies Metal's per-queue ordering. Command
//! buffer `n` signals `n` on the queue's timeline semaphore when it is
//! submitted in serial order; one submitted after a later serial signals the
//! next value of a second, late timeline instead, since a timeline only
//! rises. [`Device::poll`] retires each command buffer once its timeline
//! passes it.
//!
//! A command buffer's segments from its first event wait that is neither
//! reached nor signaled by submitted work are held, not submitted. The
//! driver would hold such a wait, and everything submitted behind it, from
//! the GPU until a signal arrived, for every connection: an event that never
//! comes would stop the whole device. The segments before it are submitted,
//! so what they signal promises its values to other work. The held segments
//! delay only the later command buffers of their Metal queue, as Apple's GPUs
//! delay only that queue, and are submitted once a CPU signal or a submitted
//! signal promises the value. Signals of held segments promise nothing, so
//! waits that form a cycle through held segments stay held.
//!
//! An indirect sparse mapping update splits its command buffer: the segments
//! from the update's on are held until the GPU has copied its arguments,
//! when the device reads them, makes the binds and submits the segments.
//! Its queue's command buffers committed meanwhile are held behind them;
//! polling and waiting submit held work as it becomes ready.
use super::gpu_times::{GpuClock, GpuTimes};
use super::pass_times::{
    self, CommandBufferCounts, PROFILE_MARKS, PassTimes, SpanKind, SpanWork, TargetInfo,
};
use super::{
    BufferId, ComputePipelineId, CounterBufferId, Device, EventId, RenderPipelineId, Slab,
    TextureId,
};
use crate::metal::{
    abi::{
        LoadAction, MultisampleDepthResolveFilter, MultisampleStencilResolveFilter, StoreAction,
        StoreActionOptions, VisibilityResultType,
    },
    binding::ResourceId,
    command_buffer::{
        BoundPipeline, BufferRef, CommandBuffer, ComputePipelineState, DynamicFeatures,
        MappedRange, RenderPipelineState, Resolver, Segment, Workload,
        counters::{self, CounterRange, CounterSample, STATISTIC_SAMPLE_BYTES},
        device_generated::{GeneratedLayoutKey, IndirectRenderClass},
        render::PassBoundary,
    },
    error::{Error, Result, ensure, require},
    indirect::{IndirectCommandBuffer, IndirectCommandBufferId, RecordLayout},
    pass::{
        self, AttachedTexture, AttachmentDescriptor, AttachmentView, RenderPassDescriptor,
        RenderPassPlan, RenderingAttachment, Target,
    },
    pipeline::MAX_COLOR_ATTACHMENTS,
    resource::TextureDescriptor,
    shader::{Compiler, Specialization},
    sparse::{
        MAP_INDIRECT_ARGUMENTS_BYTES, MAP_INDIRECT_MAPPINGS, Mapping, MappingRequest, MappingUpdate,
    },
    timing::{Detail, Phase},
    upload::{Chunk, ChunkSource},
    vulkan::{
        BufferCreate, BufferUsage, MemoryProperties, PipelineStages, PipelineStatistics,
        backend::{Backend, BufferAllocation, Object, Requirements},
        command::Command,
        object::{
            BufferInfo, ExecutionSet, ImageView, IndirectCommandsLayout, Memory, MemoryRequest,
            Pipeline, QueryPool, QueryPoolInfo, QueryType, Semaphore,
        },
    },
};
use alloc::{
    collections::{BTreeMap, VecDeque},
    format,
    string::String,
    sync::Arc,
    vec::Vec,
};

/// Queries per encoder attachment pool.
const POOL_QUERIES: u32 = 256;

/// Timestamps for one command buffer: its start, [`PASS_MARKS`] marks at
/// most, and its end.
const COMPLETION_QUERIES: u32 = PASS_MARKS + 2;

/// Marks a command buffer's timestamps make where the GPU already waits:
/// after each pass and before each barrier that follows dispatches, so the
/// platform's report can say which part of a slow one took the time.
const PASS_MARKS: u32 = 30;

/// Completed command buffers whose GPU times are kept.
pub(super) const TIMES_KEPT: usize = 1024;

/// Render pass targets the profile remembers by attachment view.
const TARGETS_KEPT: usize = 1024;

/// A `MTLCounterSet` the device samples.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CounterSet {
    Timestamp,
    Statistic,
}

/// A counter sample buffer's samples.
#[derive(Clone, Copy, Debug)]
enum CounterStorage {
    Timestamps(QueryPool),
    /// `MTLCounterResultStatistic`s in host-visible memory, initially
    /// `MTLCounterErrorValue`.
    Statistics(BufferAllocation),
}

#[derive(Clone, Copy, Debug)]
struct CounterBuffer {
    storage: CounterStorage,
    count: u32,
}

/// What a submitted command buffer holds until it completes.
#[derive(Debug)]
struct InFlight {
    serial: u64,
    /// The connection that committed it.
    key: u64,
    track: Track,
    chunks: Vec<Chunk>,
    pools: Vec<(QueryType, QueryPool)>,
    invalidations: Vec<MappedRange>,
    /// The dedicated pool that times the command buffer, and its marks.
    times: Option<(QueryPool, Marks)>,
    pipelines: Vec<BoundPipeline>,
    workload: Workload,
    /// The platform's ticks when it was committed.
    committed: u64,
    /// `vkQueueSubmit2` calls its work took, and the Vulkan command
    /// buffers they submitted.
    submits: u64,
    segments: u64,
    /// It carries the device's own commands for objects it does not own:
    /// CPU-copy uploads and block decodes of any connection's textures,
    /// which a lost device's report names.
    carries: bool,
    /// The textures whose uploads or decodes it carries: their destruction
    /// waits for it too ([`super::pins`]).
    carried: Vec<TextureId>,
    /// The event values its submitted segments signal, reached once it is
    /// done.
    signals: Vec<(Semaphore, u64)>,
    /// Every value its submissions signal, completion's included: the
    /// driver names a submission it refused by one of them.
    named: Vec<(Semaphore, u64)>,
}

/// Where a committed command buffer's completion shows.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Track {
    /// Held: not yet submitted.
    Held,
    /// Its first segments submitted, the rest held for an event or for the
    /// GPU to copy an indirect mapping update's arguments. The submitted
    /// segments end by signaling this value of the late timeline.
    Started(u64),
    /// The queue's timeline reaching its serial.
    Timeline,
    /// The late timeline reaching this value.
    Late(u64),
    /// Seen complete, or dropped unsubmitted with its connection.
    Done,
}

/// What a command buffer's timestamps after its start mark: for each span
/// up to the next mark, the indices in its `pipelines` it bound and what it
/// drew.
#[derive(Debug, Default)]
struct Marks {
    spans: Vec<Vec<u16>>,
    shapes: Vec<SpanShape>,
    /// The pool's queries: [`COMPLETION_QUERIES`], or more under the
    /// profile ([`pass_times`]), whose spans it reports.
    queries: u32,
    profiled: bool,
    counts: CommandBufferCounts,
}

impl Marks {
    /// Ends the span here: the next timestamp is its end.
    fn mark(
        &mut self,
        pool: QueryPool,
        bound: &mut Vec<u16>,
        shape: &mut SpanShape,
        commands: &mut Vec<Command>,
    ) {
        self.spans.push(core::mem::take(bound));
        self.shapes.push(core::mem::take(shape));
        commands.push(Command::WriteTimestamp {
            pool,
            query: self.spans.len() as u32,
            stage: PipelineStages::ALL_COMMANDS,
        });
    }
}

/// What one span between marks drew: the last pass's extent, its draws and
/// dispatches, and the most vertices times instances of one direct draw;
/// for the profile, its kind of work, the last pass's first attachment and
/// the texels it copied or converted.
#[derive(Clone, Copy, Debug, Default)]
struct SpanShape {
    extent: [u32; 2],
    draws: u32,
    dispatches: u32,
    largest: u64,
    kind: SpanKind,
    target: Option<ImageView>,
    texels: u64,
}

impl SpanShape {
    fn add(&mut self, command: &Command) {
        // A span with a pass is the pass's; else its first work names it.
        let kind = SpanKind::of(command);
        if kind != SpanKind::None && (kind == SpanKind::Render || self.kind == SpanKind::None) {
            self.kind = kind;
        }
        if let Some(texels) = pass_times::copied_texels(command) {
            self.texels += texels;
        }
        match command {
            Command::BeginRendering(rendering) => {
                self.extent = rendering.extent;
                self.target = rendering
                    .color
                    .iter()
                    .flatten()
                    .next()
                    .or(rendering.depth.as_ref())
                    .or(rendering.stencil.as_ref())
                    .map(|attachment| attachment.view);
            }
            Command::Draw {
                vertex_count: count,
                instance_count,
                ..
            }
            | Command::DrawIndexed {
                index_count: count,
                instance_count,
                ..
            } => {
                self.draws += 1;
                self.largest = self
                    .largest
                    .max(u64::from(*count) * u64::from(*instance_count));
            }
            Command::DrawIndirect { .. }
            | Command::DrawIndexedIndirect { .. }
            | Command::DrawMeshTasks(_)
            | Command::DrawMeshTasksIndirect { .. } => self.draws += 1,
            Command::Dispatch(_) | Command::DispatchIndirect { .. } => self.dispatches += 1,
            _ => {}
        }
    }

    /// `[WxH draws=… dispatches=… largest=…]`, which the report shows after
    /// the span's entry points.
    fn describe(&self) -> String {
        format!(
            "[{}x{} draws={} dispatches={} largest={}]",
            self.extent[0], self.extent[1], self.draws, self.dispatches, self.largest
        )
    }
}

#[derive(Debug)]
struct FreeQueryPool {
    query_type: QueryType,
    count: u32,
    pool: QueryPool,
}

/// Segments of a committed command buffer that the host submits: either all
/// of them, or those from an indirect mapping update's on.
#[derive(Debug)]
struct Held {
    serial: u64,
    /// The connection that committed it.
    key: u64,
    /// The connection's Metal queue it was committed on, whose later work
    /// waits behind it.
    queue: u64,
    /// The step value the copy of the update's arguments signals; zero
    /// when only the held work ahead delays the segments.
    step: u64,
    segments: Vec<Segment>,
    /// Mapping operations by index into `segments`.
    mappings: Vec<(usize, Mapping)>,
}

/// One commit's prepared work. Only indirect mapping updates split it.
enum HeldBatch {
    One(Held),
    Split(Vec<Held>),
}

impl HeldBatch {
    fn as_mut_slice(&mut self) -> &mut [Held] {
        match self {
            Self::One(held) => core::slice::from_mut(held),
            Self::Split(held) => held,
        }
    }

    fn enqueue(self, queue: &mut VecDeque<Held>) {
        match self {
            Self::One(held) => queue.push_back(held),
            Self::Split(held) => queue.extend(held),
        }
    }
}

#[derive(Debug, Default)]
pub(super) struct Queue {
    /// Upload chunks with their memory; free ones are reused.
    chunks: Vec<(Chunk, Memory)>,
    free_chunks: Vec<Chunk>,
    free_pools: Vec<FreeQueryPool>,
    in_flight: VecDeque<InFlight>,
    events: Slab<Semaphore>,
    counters: Slab<CounterBuffer>,
    /// Committed work not yet submitted, in commit order.
    held: VecDeque<Held>,
    /// Signals each indirect mapping update's step once its arguments are
    /// copied; created with the first.
    steps: Option<Semaphore>,
    step: u64,
    /// Completes command buffers submitted after a later serial, which the
    /// timeline already passed; created with the first.
    late: Option<Semaphore>,
    late_value: u64,
    /// The newest serial submitted to signal the timeline.
    timeline_serial: u64,
    /// The highest value submitted work or the CPU signals on each
    /// semaphore: a wait on no more than it is safe to submit.
    promised: Vec<(Semaphore, u64)>,
    /// The highest value each event's semaphore is known to have reached,
    /// read from the driver or signaled by the CPU: once it is what was
    /// promised, the event's value is known without asking the driver.
    reached_events: Vec<(Semaphore, u64)>,
    /// (connection, serial, failure, the driver's records of it) of
    /// completed command buffers whose submission the driver refused while
    /// keeping the device, until the connection asks
    /// ([`Device::take_submit_failure`]).
    refused: Vec<(u64, u64, Error, u32)>,
    /// Command buffers seen complete, in any order.
    completions: u64,
    /// (connection, serial) of command buffers retired while an older one
    /// of their connection is not complete, until the connection reaches
    /// them: another Metal queue's held work never delays their completion.
    early: Vec<(u64, u64)>,
    /// GPU start and end ticks of the latest completed command buffers, by
    /// serial, oldest first; none for one whose timestamps were not written
    /// (it ended in an error).
    times: VecDeque<(u64, Option<[u64; 2]>)>,
    /// Where retired command buffers' time went, for the platform's report.
    clock: GpuClock,
    /// Per-pass timing without `--profile`'s global switch, for tests.
    profile: bool,
    /// The profile's spans and counts since the last report.
    passes: PassTimes,
    /// The profile's render pass targets by attachment view, newest last.
    targets: BTreeMap<ImageView, (u64, TargetInfo)>,
    /// Targets noted so far, which orders them by age.
    noted: u64,
}

impl Queue {
    pub(super) fn in_flight_count(&self) -> usize {
        self.in_flight.len()
    }

    /// Whether a command buffer in flight may read what `release` frees,
    /// including one seen complete and not yet retired.
    pub(super) fn reads(&self, release: &super::pins::Release) -> bool {
        self.in_flight
            .iter()
            .any(|flight| release.read_by(flight.key, flight.serial))
    }

    /// The connections of unfinished command buffers that carry uploads or
    /// decodes of `texture`.
    pub(super) fn carriers(&self, texture: TextureId) -> impl Iterator<Item = u64> + '_ {
        self.in_flight
            .iter()
            .filter(move |flight| flight.track != Track::Done && flight.carried.contains(&texture))
            .map(|flight| flight.key)
    }

    /// The connections with command buffers held for an event, oldest
    /// first, without repeats.
    pub(super) fn holders(&self) -> Vec<u64> {
        let mut keys = Vec::new();
        for flight in &self.in_flight {
            if matches!(flight.track, Track::Held | Track::Started(_))
                && !keys.contains(&flight.key)
            {
                keys.push(flight.key);
            }
        }
        keys
    }
}

/// One attachment of a [`RenderPassRequest`].
#[derive(Clone, Copy, Debug)]
pub struct AttachmentRequest<Clear> {
    pub texture: TextureId,
    pub level: u64,
    pub slice: u64,
    pub depth_plane: u64,
    /// (texture, level, slice, depth plane).
    pub resolve: Option<(TextureId, u64, u64, u64)>,
    pub load_action: LoadAction,
    pub store_action: StoreAction,
    pub store_action_options: StoreActionOptions,
    pub clear: Clear,
    /// The IOSurface the texture lies in and its seed, which changes as a
    /// CPU writes it, as the guest saw them encoding the pass; `None` when
    /// unknown.
    pub surface: Option<(u32, u32)>,
}

/// `MTLRenderPassDescriptor` with the device's object ids.
#[derive(Clone, Debug)]
pub struct RenderPassRequest {
    pub color_attachments: [Option<AttachmentRequest<[f64; 4]>>; MAX_COLOR_ATTACHMENTS],
    pub depth_attachment: Option<AttachmentRequest<f64>>,
    pub depth_resolve_filter: MultisampleDepthResolveFilter,
    pub stencil_attachment: Option<AttachmentRequest<u32>>,
    pub stencil_resolve_filter: MultisampleStencilResolveFilter,
    pub visibility_result_buffer: Option<BufferId>,
    pub visibility_result_type: VisibilityResultType,
    pub render_target_array_length: u64,
    pub render_target_width: u64,
    pub render_target_height: u64,
    pub default_raster_sample_count: u64,
    pub tile_memory: crate::metal::pass::TileMemory,
    pub sample_positions: Vec<[f32; 2]>,
    pub rasterization_rate_map: bool,
    /// `sampleBufferAttachments`: (sample buffer, sample index, boundary).
    pub counter_samples: Vec<(CounterBufferId, u32, PassBoundary)>,
}

impl<B: Backend, C: Compiler> Device<B, C> {
    /// `commandBuffer` of any queue.
    pub fn new_command_buffer(&self) -> CommandBuffer {
        CommandBuffer::new(DynamicFeatures::new(&self.capabilities))
    }

    /// The services encoding calls need.
    pub fn resolver(&mut self) -> DeviceResolver<'_, B, C> {
        DeviceResolver { device: self }
    }

    /// `renderCommandEncoderWithDescriptor:`.
    pub fn begin_render(
        &mut self,
        commands: &mut CommandBuffer,
        request: &RenderPassRequest,
    ) -> Result<()> {
        // Copy the descriptors so the pass can borrow them while views are
        // created.
        let textures = self.pass_textures(request)?;
        let lookup = |id: TextureId| {
            textures
                .iter()
                .find(|(known, _)| *known == id)
                .map(|(_, descriptor)| descriptor)
        };
        let target = |id: TextureId, level, slice, depth_plane| -> Result<Target<'_, TextureId>> {
            let descriptor = lookup(id).ok_or(Error::Invalid("unknown texture"))?;
            Ok(Target {
                texture: AttachedTexture {
                    texture: id,
                    descriptor,
                },
                level,
                slice,
                depth_plane,
            })
        };
        fn attachment<'a, Clear: Copy>(
            request: &AttachmentRequest<Clear>,
            target: &impl Fn(TextureId, u64, u64, u64) -> Result<Target<'a, TextureId>>,
        ) -> Result<AttachmentDescriptor<'a, TextureId, Clear>> {
            Ok(AttachmentDescriptor {
                target: target(
                    request.texture,
                    request.level,
                    request.slice,
                    request.depth_plane,
                )?,
                resolve: request
                    .resolve
                    .map(|(texture, level, slice, plane)| target(texture, level, slice, plane))
                    .transpose()?,
                load_action: request.load_action,
                store_action: request.store_action,
                store_action_options: request.store_action_options,
                clear: request.clear,
            })
        }
        let mut color_attachments = [None; MAX_COLOR_ATTACHMENTS];
        for (slot, color) in color_attachments.iter_mut().zip(&request.color_attachments) {
            *slot = color
                .as_ref()
                .map(|color| attachment(color, &target))
                .transpose()?;
        }
        let descriptor = RenderPassDescriptor {
            color_attachments,
            depth_attachment: request
                .depth_attachment
                .as_ref()
                .map(|depth| attachment(depth, &target))
                .transpose()?,
            depth_resolve_filter: request.depth_resolve_filter,
            stencil_attachment: request
                .stencil_attachment
                .as_ref()
                .map(|stencil| attachment(stencil, &target))
                .transpose()?,
            stencil_resolve_filter: request.stencil_resolve_filter,
            visibility_results: request.visibility_result_buffer.is_some(),
            render_target_array_length: request.render_target_array_length,
            render_target_width: request.render_target_width,
            render_target_height: request.render_target_height,
            default_raster_sample_count: request.default_raster_sample_count,
            tile_memory: request.tile_memory,
            sample_positions: request.sample_positions.clone(),
            rasterization_rate_map: request.rasterization_rate_map,
        };
        let mut plan = pass::render_pass(&self.capabilities, &descriptor)?;
        // A two-plane attachment clears its first plane as the pass begins
        // and its second, whole, before.
        for (index, color) in descriptor.color_attachments.iter().enumerate() {
            let Some(color) = color.filter(|color| color.load_action == LoadAction::Clear) else {
                continue;
            };
            let texture = color.target.texture;
            // An extended-range texture's planes clear through its half
            // floats (below).
            if texture.descriptor.pixel_format.extended_planes().is_some() {
                continue;
            }
            let Some(image) = self.second_plane(texture.texture)? else {
                continue;
            };
            let format = texture.descriptor.pixel_format;
            if plan.extent
                != [
                    texture.descriptor.width as u32,
                    texture.descriptor.height as u32,
                ]
            {
                return Err(Error::Unsupported(
                    "clearing part of a two-plane attachment",
                ));
            }
            let value = crate::metal::pixel_format::second_plane_clear(format, color.clear)
                .ok_or(Error::Invalid("the attachment has one plane"))?;
            plan.inputs.second_plane_clears[index] = Some((image, pass::ClearValue::Float(value)));
        }
        // A two-plane extended-range attachment renders as half floats,
        // which the pass converts where it begins and ends.
        for (index, color) in descriptor.color_attachments.iter().enumerate() {
            let (Some(color), Some(Some(attached))) = (color, plan.color.get(index)) else {
                continue;
            };
            if let Some((_, resolve)) = attached.resolve
                && self
                    .texture_descriptor(resolve.texture)?
                    .pixel_format
                    .extended_planes()
                    .is_some()
            {
                return Err(Error::Unsupported(
                    "multisample resolves into two-plane extended-range textures",
                ));
            }
            let view = AttachmentView {
                texture: (),
                view_type: attached.view.view_type,
                range: attached.view.range,
            };
            let stores = !attached.memoryless
                && attached.store_op == crate::metal::vulkan::AttachmentStoreOp::Store;
            let actions = super::resources::PlaneActions {
                loads: color.load_action == LoadAction::Load,
                stores,
                settled: color.store_action == StoreAction::Store,
                surface: request.color_attachments[index]
                    .as_ref()
                    .and_then(|color| color.surface),
            };
            plan.inputs.extended_planes[index] = self.extended_pass_planes(
                attached.view.texture,
                view,
                plan.extent,
                actions,
                &mut commands.planes,
            )?;
        }
        // Framebuffer reads access each color attachment's raw texels, of
        // the attachment or, where no view of it reads them or the backend
        // may render it through a shadow (a linear image), of its copy.
        if self.capabilities.framebuffer_reads {
            for (index, color) in plan.color.iter().enumerate() {
                let Some(color) = color else { continue };
                let texture = self.texture_descriptor(color.view.texture)?;
                if !texture
                    .usage
                    .contains(crate::metal::abi::TextureUsage::RENDER_TARGET)
                {
                    continue;
                }
                let view = AttachmentView {
                    texture: (),
                    view_type: color.view.view_type,
                    range: color.view.range,
                };
                let format = texture.pixel_format;
                let viewed = texture.texture_type != crate::metal::abi::TextureType::D3
                    && crate::metal::resource::raw_texel_format(format).is_some();
                let texture = color.view.texture;
                let viewed = viewed && !self.reads_attachment_through_copy(texture)?;
                if viewed {
                    plan.inputs.textures[index] = self.attachment_storage(texture, view)?;
                } else if crate::metal::resource::raw_texel_bytes_format(format).is_some() {
                    let copy = self.color_copy(texture, view)?;
                    plan.inputs.textures[index] = copy.id;
                    plan.inputs.color_copies[index] = Some(copy);
                }
            }
        }
        // The average samples kernel writes an averaged attachment's resolve
        // texture through its raw texels.
        for (average, color) in plan.inputs.averages.iter_mut().zip(&plan.color) {
            let Some((_, resolve)) = color
                .as_ref()
                .filter(|color| color.averaged)
                .and_then(|color| color.resolve)
            else {
                continue;
            };
            let view = AttachmentView {
                texture: (),
                view_type: resolve.view_type,
                range: resolve.range,
            };
            *average = self.attachment_storage(resolve.texture, view)?;
        }
        // Fragment functions that run the fragment tests themselves access
        // copies of the depth and stencil attachments.
        if self.tests_fragments {
            let copy = |attachment: &Option<crate::metal::pass::RenderingAttachment<TextureId>>| {
                attachment.as_ref().map(|attachment| {
                    (
                        attachment.view.texture,
                        AttachmentView {
                            texture: (),
                            view_type: attachment.view.view_type,
                            range: attachment.view.range,
                        },
                    )
                })
            };
            if let Some((texture, view)) = copy(&plan.depth) {
                plan.inputs.depth_copy = Some(self.tested_copy(texture, view, false)?);
            }
            if let Some((texture, view)) = copy(&plan.stencil) {
                plan.inputs.stencil_copy = Some(self.tested_copy(texture, view, true)?);
            }
        }
        // A stencil resolve following the depth resolve reads copies of the
        // samples and writes a copy of the resolve texture's stencil.
        if let (Some(depth), Some(stencil)) = (&plan.depth, &plan.stencil)
            && let Some((_, resolve)) = stencil.resolve.filter(|_| stencil.follows_depth)
        {
            let view = |view: &AttachmentView<TextureId>| AttachmentView {
                texture: (),
                view_type: view.view_type,
                range: view.range,
            };
            let followed = crate::metal::pass::FollowedStencil {
                depth: self.tested_copy(depth.view.texture, view(&depth.view), false)?,
                stencil: self.tested_copy(stencil.view.texture, view(&stencil.view), true)?,
                resolve: self.tested_copy(resolve.texture, view(&resolve), true)?,
            };
            plan.inputs.follows_depth = Some(followed);
        }
        // The profile names a pass by its first attachment.
        fn first<T: Copy>(plan: &RenderPassPlan<T>) -> Option<T> {
            plan.color
                .iter()
                .flatten()
                .next()
                .or(plan.depth.as_ref())
                .or(plan.stencil.as_ref())
                .map(|attachment| attachment.view.texture)
        }
        let texture = first(&plan).filter(|_| self.passes_profiled());
        let plan = self.resolve_views(plan)?;
        if let (Some(texture), Some(view)) = (texture, first(&plan)) {
            self.note_target(view, texture);
        }
        let visibility = request
            .visibility_result_buffer
            .map(|buffer| Ok((self.buffer(buffer)?, request.visibility_result_type)))
            .transpose()?;
        let samples = request
            .counter_samples
            .iter()
            .map(|&(counters, index, boundary)| {
                Ok((self.counter_sample(counters, index)?, boundary))
            })
            .collect::<Result<Vec<_>>>()?;
        commands.begin_render(&mut self.resolver(), plan, visibility, samples)
    }

    fn pass_textures(
        &self,
        request: &RenderPassRequest,
    ) -> Result<Vec<(TextureId, TextureDescriptor)>> {
        let colors = request
            .color_attachments
            .iter()
            .flatten()
            .flat_map(|color| [Some(color.texture), color.resolve.map(|r| r.0)]);
        let depth = request
            .depth_attachment
            .iter()
            .flat_map(|depth| [Some(depth.texture), depth.resolve.map(|r| r.0)]);
        let stencil = request
            .stencil_attachment
            .iter()
            .flat_map(|stencil| [Some(stencil.texture), stencil.resolve.map(|r| r.0)]);
        colors
            .chain(depth)
            .chain(stencil)
            .flatten()
            .map(|id| Ok((id, *self.texture_descriptor(id)?)))
            .collect()
    }

    /// Replaces texture ids in a pass plan with attachment views.
    fn resolve_views(
        &mut self,
        plan: RenderPassPlan<TextureId>,
    ) -> Result<RenderPassPlan<ImageView>> {
        let mut view = |view: AttachmentView<TextureId>| -> Result<AttachmentView<ImageView>> {
            let key = AttachmentView {
                texture: (),
                view_type: view.view_type,
                range: view.range,
            };
            let image_view = self.attachment_view(view.texture, key)?;
            Ok(AttachmentView {
                texture: image_view,
                view_type: view.view_type,
                range: view.range,
            })
        };
        let mut attachment =
            |attachment: RenderingAttachment<TextureId>| -> Result<RenderingAttachment<ImageView>> {
                Ok(RenderingAttachment {
                    view: view(attachment.view)?,
                    resolve: attachment
                        .resolve
                        .map(|(mode, resolve)| Ok::<_, Error>((mode, view(resolve)?)))
                        .transpose()?,
                    load_op: attachment.load_op,
                    store_op: attachment.store_op,
                    clear: attachment.clear,
                    store_action: attachment.store_action,
                    memoryless: attachment.memoryless,
                    averaged: attachment.averaged,
                    follows_depth: attachment.follows_depth,
                })
            };
        let color = plan
            .color
            .into_iter()
            .map(|color| color.map(&mut attachment).transpose())
            .collect::<Result<_>>()?;
        let depth = plan.depth.map(&mut attachment).transpose()?;
        let stencil = plan.stencil.map(&mut attachment).transpose()?;
        Ok(RenderPassPlan {
            extent: plan.extent,
            tile: plan.tile,
            layer_count: plan.layer_count,
            color,
            depth,
            stencil,
            sample_count: plan.sample_count,
            color_formats: plan.color_formats,
            depth_format: plan.depth_format,
            stencil_format: plan.stencil_format,
            visibility_results: plan.visibility_results,
            sample_locations: plan.sample_locations,
            inputs: plan.inputs,
        })
    }

    /// `commit`: submits the command buffer and returns its serial. A
    /// command buffer that cannot be submitted is discarded.
    pub fn commit(&mut self, commands: CommandBuffer) -> Result<u64> {
        self.commit_for(0, commands)
    }

    /// Commits `commands` for connection `key` on its one Metal queue.
    pub fn commit_for(&mut self, key: u64, commands: CommandBuffer) -> Result<u64> {
        self.commit_on(key, 0, commands)
    }

    /// Commits `commands` for connection `key` on its Metal queue `queue`,
    /// behind that queue's earlier work: held work of another queue or
    /// connection never delays it.
    pub fn commit_on(&mut self, key: u64, queue: u64, mut commands: CommandBuffer) -> Result<u64> {
        let start = self.timing.now();
        self.ensure_admission()?;
        let serial = self.submitted + 1;
        let completion = (self.timeline, serial);
        let finished = self
            .flush_indirect_writes()
            .and_then(|()| commands.finish(&mut self.resolver(), completion));
        // Applied once the recording reaches the queue.
        let planes = core::mem::take(&mut commands.planes);
        let recording = commands.into_recording();
        if let Err(error) = finished.and_then(|()| self.check_mappings(&recording.mappings)) {
            self.recycle(recording.uploads, recording.query_pools);
            return Err(error);
        }
        let start = self.timing.since(Phase::CommitFinish, start);
        if let Some(Detail::Commit { uploads, .. }) = self.timing.detail_mut() {
            *uploads = recording
                .uploads
                .iter()
                .map(|(_, bytes)| bytes.len() as u64)
                .sum();
        }
        let mut times = None;
        let prepared = (|| {
            let mut held =
                self.hold(serial, [key, queue], recording.segments, recording.mappings)?;
            times = self.time(held.as_mut_slice())?;
            if let Some(marker) = self.marker_buffer() {
                for work in held.as_mut_slice() {
                    mark_completion(&mut work.segments, completion, marker, serial);
                }
            }
            let mut chunks = Vec::with_capacity(recording.uploads.len());
            for (chunk, bytes) in &recording.uploads {
                let memory = self.chunk_memory(chunk.buffer)?;
                self.backend.write_memory(memory, 0, bytes)?;
                chunks.push(*chunk);
            }
            Ok((held, chunks))
        })();
        let (mut held, chunks) = match prepared {
            Ok(prepared) => prepared,
            Err(error) => {
                self.recycle(recording.uploads, recording.query_pools);
                if let Some((pool, marks)) = times {
                    // The recording never reached the queue, so the pool
                    // remains reset and can be returned without a GPU call.
                    self.queue.free_pools.push(FreeQueryPool {
                        query_type: QueryType::Timestamp,
                        count: marks.queries,
                        pool,
                    });
                }
                if error == Error::DeviceLost {
                    self.mark_lost();
                }
                return Err(error);
            }
        };
        // CPU-copy writes reach the images first, after the opening barrier
        // and outside the command buffer's GPU times. Keep pending copies
        // available for the next commit until preparation succeeds.
        let carried = self.cpu_copy_textures();
        let mut uploads = self.take_cpu_copy_uploads();
        let carries = !uploads.is_empty();
        // They ride the command buffer only when it reaches the queue now.
        // Behind held work they go ahead in a submission of their own that
        // waits for nothing: any connection's later work then follows them,
        // and closing the carrier's connection cannot drop them.
        let first = &mut held.as_mut_slice()[0];
        let leading = if carries && !self.submits_now(first) {
            uploads.insert(0, Command::Barrier(crate::metal::hazard::FULL));
            let segment = Segment {
                commands: uploads,
                ..Segment::default()
            };
            Some(Held {
                serial,
                key,
                queue,
                step: 0,
                segments: alloc::vec![segment],
                mappings: Vec::new(),
            })
        } else {
            first.segments[0].commands.splice(1..1, uploads);
            None
        };
        let start = self.timing.since(Phase::CommitUpload, start);
        self.submitted = serial;
        self.apply_plane_changes(planes);
        self.queue.in_flight.push_back(InFlight {
            serial,
            key,
            track: Track::Held,
            chunks,
            pools: recording.query_pools,
            invalidations: recording.invalidations,
            times,
            pipelines: recording.pipelines,
            workload: recording.workload,
            committed: self.timing.now(),
            submits: 0,
            segments: 0,
            carries,
            carried,
            signals: Vec::new(),
            named: Vec::new(),
        });
        if held.as_mut_slice().iter().any(|held| {
            held.segments
                .iter()
                .any(|segment| !segment.waits.is_empty())
        }) {
            self.queue.clock.waits();
        }
        held.enqueue(&mut self.queue.held);
        self.backend.take_split();
        // The uploads' submission marks the command buffer started; a
        // failed one never signals, so the loss is recorded.
        let advanced = match leading {
            Some(leading) => self.submit_held(leading).inspect_err(|_| self.lost = true),
            None => Ok(()),
        }
        .and_then(|()| self.advance());
        // The backend splits its submissions; the rest is submitting.
        let whole = self.timing.now().wrapping_sub(start);
        let [reclaim, record, _] = self.backend.take_split();
        self.timing.add(Phase::CommitReclaim, reclaim);
        self.timing.add(Phase::CommitRecord, record);
        self.timing
            .add(Phase::CommitSubmit, whole.saturating_sub(reclaim + record));
        advanced?;
        Ok(serial)
    }

    /// Brackets a command buffer's work with timestamps for `GPUStartTime`
    /// and `GPUEndTime`, and marks the ends of its first passes and
    /// dispatches for the platform's report; `None` without timestamps.
    fn time(&mut self, held: &mut [Held]) -> Result<Option<(QueryPool, Marks)>> {
        if self.capabilities.timestamp_frequency == 0 {
            return Ok(None);
        }
        let profiled = self.passes_profiled();
        let limit = if profiled { PROFILE_MARKS } else { PASS_MARKS };
        let queries = if profiled {
            PROFILE_MARKS + 2
        } else {
            COMPLETION_QUERIES
        };
        let pool = self.take_query_pool(QueryType::Timestamp, queries)?;
        held[0].segments[0].commands.insert(
            1,
            Command::WriteTimestamp {
                pool,
                query: 0,
                stage: PipelineStages::TOP_OF_PIPE,
            },
        );
        let mut marks = Marks {
            queries,
            profiled,
            ..Marks::default()
        };
        // The pipelines a span binds, by the order the command buffer first
        // bound each: the order its `pipelines` lists them in.
        let mut seen: Vec<Pipeline> = Vec::new();
        let mut bound: Vec<u16> = Vec::new();
        // Marks go where the GPU already waits: after a pass, and before the
        // barrier that follows dispatches, so a mark never stops dispatches
        // the encoder lets overlap.
        let mut dispatched = false;
        let mut shape = SpanShape::default();
        // Under the profile a mark also goes wherever the work changes
        // kind outside a pass, which may stop work that would overlap.
        let mut in_pass = false;
        for segment in held.iter_mut().flat_map(|held| held.segments.iter_mut()) {
            let commands = core::mem::take(&mut segment.commands);
            segment.commands.reserve(commands.len());
            for command in commands {
                let room = marks.spans.len() < limit as usize;
                let kind = SpanKind::of(&command);
                let changes = profiled
                    && !in_pass
                    && kind != SpanKind::None
                    && shape.kind != SpanKind::None
                    && shape.kind != kind;
                if room && (dispatched && matches!(command, Command::Barrier(_)) || changes) {
                    dispatched = false;
                    marks.mark(pool, &mut bound, &mut shape, &mut segment.commands);
                }
                dispatched |= matches!(
                    command,
                    Command::Dispatch(_) | Command::DispatchIndirect { .. }
                );
                let pass_ends = matches!(command, Command::EndRendering);
                match command {
                    Command::BeginRendering(_) => {
                        in_pass = true;
                        marks.counts.passes += 1;
                    }
                    Command::EndRendering => in_pass = false,
                    _ => {}
                }
                if let Some(texels) = pass_times::copied_texels(&command) {
                    marks.counts.copies += 1;
                    marks.counts.copied_texels += texels;
                }
                shape.add(&command);
                if let Command::BindPipeline { pipeline, .. } = &command {
                    let index = seen
                        .iter()
                        .position(|known| known == pipeline)
                        .unwrap_or_else(|| {
                            seen.push(*pipeline);
                            seen.len() - 1
                        });
                    let index = u16::try_from(index).unwrap_or(u16::MAX);
                    if !bound.contains(&index) {
                        bound.push(index);
                    }
                }
                segment.commands.push(command);
                if pass_ends && marks.spans.len() < limit as usize {
                    marks.mark(pool, &mut bound, &mut shape, &mut segment.commands);
                }
            }
        }
        marks.counts.out_of_marks = marks.spans.len() >= limit as usize;
        // The rest of the work to the end.
        marks.spans.push(bound);
        marks.shapes.push(shape);
        let last = held.last_mut().and_then(|held| held.segments.last_mut());
        last.expect("a segment")
            .commands
            .push(Command::WriteTimestamp {
                pool,
                query: marks.spans.len() as u32,
                stage: PipelineStages::ALL_COMMANDS,
            });
        Ok(Some((pool, marks)))
    }

    fn take_query_pool(&mut self, query_type: QueryType, count: u32) -> Result<QueryPool> {
        let free = &mut self.queue.free_pools;
        if let Some(index) = free
            .iter()
            .position(|entry| entry.query_type == query_type && entry.count == count)
        {
            return Ok(free.swap_remove(index).pool);
        }
        let statistics = if query_type == QueryType::PipelineStatistics {
            counters::STATISTICS
        } else {
            PipelineStatistics::EMPTY
        };
        let info = QueryPoolInfo {
            query_type,
            count,
            statistics,
        };
        let pool = self.backend.create_query_pool(&info)?;
        self.backend.reset_queries(pool, 0, count);
        Ok(pool)
    }

    fn retire_query_pool(&mut self, pool: QueryPool, query_type: QueryType, count: u32) {
        self.backend.reset_queries(pool, 0, count);
        self.queue.free_pools.push(FreeQueryPool {
            query_type,
            count,
            pool,
        });
    }

    /// `GPUStartTime` and `GPUEndTime` of the completed command buffer with
    /// `serial`, in timestamp ticks; the latest [`TIMES_KEPT`] are kept.
    /// One whose timestamps were never written is answered zeros, reported
    /// once: what a Mac answers for it is not recorded.
    pub fn command_buffer_times(&mut self, serial: u64) -> Result<[u64; 2]> {
        let running = self
            .queue
            .in_flight
            .iter()
            .any(|flight| flight.serial == serial);
        ensure(
            serial != 0 && serial <= self.submitted && !running,
            "the command buffer has not completed",
        )?;
        require(self.capabilities.timestamp_frequency != 0, "GPU times")?;
        let times = &self.queue.times;
        let found = times.binary_search_by_key(&serial, |&(serial, _)| serial);
        match found.map(|index| times[index].1) {
            Ok(Some(times)) => Ok(times),
            Ok(None) => {
                self.unsupported(
                    "GPU times of a command buffer that ended without them, answered zeros",
                );
                Ok([0, 0])
            }
            Err(_) => Err(Error::Invalid(
                "the command buffer's GPU times are no longer kept",
            )),
        }
    }

    /// Drops a command buffer that will not be committed.
    pub fn discard(&mut self, commands: CommandBuffer) {
        let recording = commands.into_recording();
        self.recycle(recording.uploads, recording.query_pools);
    }

    /// Takes back the upload chunks and query pools of a recording that was
    /// never submitted, so its queries are still reset.
    fn recycle(&mut self, uploads: Vec<(Chunk, Vec<u8>)>, pools: Vec<(QueryType, QueryPool)>) {
        let chunks = uploads.into_iter().map(|(chunk, _)| chunk);
        self.queue.free_chunks.extend(chunks);
        self.queue
            .free_pools
            .extend(pools.into_iter().map(|(query_type, pool)| FreeQueryPool {
                query_type,
                count: POOL_QUERIES,
                pool,
            }));
    }

    fn check_mappings(&self, mappings: &[(usize, Mapping)]) -> Result<()> {
        for (_, mapping) in mappings {
            match mapping {
                Mapping::Request(request) => self.check_mapping(request)?,
                // The region is checked once the host reads it.
                Mapping::Indirect { image, .. } => {
                    self.sparse_texture(*image)?;
                }
            }
        }
        Ok(())
    }

    /// Splits a command buffer before each segment whose mapping operations
    /// include an indirect update; the segment before it, which copies the
    /// arguments, signals the next step.
    fn hold(
        &mut self,
        serial: u64,
        [key, queue]: [u64; 2],
        mut segments: Vec<Segment>,
        mut mappings: Vec<(usize, Mapping)>,
    ) -> Result<HeldBatch> {
        let mut starts: Vec<_> = mappings
            .iter()
            .filter(|(_, mapping)| matches!(mapping, Mapping::Indirect { .. }))
            .map(|(segment, _)| *segment)
            .collect();
        starts.dedup();
        if starts.is_empty() {
            return Ok(HeldBatch::One(Held {
                serial,
                key,
                queue,
                step: 0,
                segments,
                mappings,
            }));
        }
        let rebase = |mappings: Vec<(usize, Mapping)>, base: usize| {
            mappings
                .into_iter()
                .map(|(segment, mapping)| (segment - base, mapping))
                .collect()
        };
        let mut held = Vec::with_capacity(starts.len() + 1);
        let (mut base, mut step) = (0, 0);
        for start in starts {
            let rest = segments.split_off(start - base);
            let steps = match self.queue.steps {
                Some(steps) => steps,
                None => *self.queue.steps.insert(self.backend.create_semaphore(0)?),
            };
            self.queue.step += 1;
            let copy = segments.last_mut().expect("the arguments' copy");
            copy.signals.push((steps, self.queue.step));
            let later = mappings.split_off(mappings.partition_point(|(at, _)| *at < start));
            held.push(Held {
                serial,
                key,
                queue,
                step,
                segments,
                mappings: rebase(mappings, base),
            });
            (base, step, segments, mappings) = (start, self.queue.step, rest, later);
        }
        held.push(Held {
            serial,
            key,
            queue,
            step,
            segments,
            mappings: rebase(mappings, base),
        });
        Ok(HeldBatch::Split(held))
    }

    /// Submits held work that is ready, each Metal queue's in order. A
    /// command buffer's segments before its first wait nothing promises yet
    /// are submitted; the rest, and work whose arguments are unread, hold
    /// only their queue's later work.
    fn advance(&mut self) -> Result<()> {
        let mut blocked: Vec<[u64; 2]> = Vec::new();
        let mut index = 0;
        while index < self.queue.held.len() {
            if self.lost {
                return Err(Error::DeviceLost);
            }
            let order = [self.queue.held[index].key, self.queue.held[index].queue];
            let ready = if blocked.contains(&order) {
                0
            } else {
                self.ready_segments(index)?
            };
            if ready == 0 {
                blocked.push(order);
                index += 1;
                continue;
            }
            let whole = ready == self.queue.held[index].segments.len();
            let held = if whole {
                self.queue.held.remove(index).expect("held work")
            } else {
                // The rest stays at its place, to be looked at again with
                // what this part promises.
                let rest = &mut self.queue.held[index];
                let tail = rest.segments.split_off(ready);
                let segments = core::mem::replace(&mut rest.segments, tail);
                let split = rest.mappings.partition_point(|(at, _)| *at < ready);
                let mut later = rest.mappings.split_off(split);
                for (at, _) in &mut later {
                    *at -= ready;
                }
                let mappings = core::mem::replace(&mut rest.mappings, later);
                let step = core::mem::take(&mut rest.step);
                Held {
                    serial: rest.serial,
                    key: rest.key,
                    queue: rest.queue,
                    step,
                    segments,
                    mappings,
                }
            };
            // A failed submission never signals; record the loss so waits end.
            self.submit_held(held).inspect_err(|_| self.lost = true)?;
            // What it signals may have readied work passed over.
            if !blocked.is_empty() {
                blocked.clear();
                index = 0;
            }
        }
        Ok(())
    }

    /// Whether [`Self::advance`] submits `held`'s first segment as soon as
    /// it is queued: no earlier work of its Metal queue is held, and its
    /// first segment's waits are promised.
    fn submits_now(&self, held: &Held) -> bool {
        let order = [held.key, held.queue];
        !self.lost
            && held.step == 0
            && !self
                .queue
                .held
                .iter()
                .any(|earlier| [earlier.key, earlier.queue] == order)
            && held.segments[0]
                .waits
                .iter()
                .all(|&(semaphore, value)| self.promised(semaphore) >= value)
    }

    /// How many of held work `index`'s first segments can be submitted:
    /// none while its arguments are being copied, else those before the
    /// first wait nothing promises.
    fn ready_segments(&mut self, index: usize) -> Result<usize> {
        let step = self.queue.held[index].step;
        if step > 0 {
            let steps = self.queue.steps.expect("steps have their semaphore");
            if self.backend.semaphore_value(steps)? < step {
                return Ok(0);
            }
        }
        let held = &self.queue.held[index];
        // Only submitted work and the CPU's signals raise an event, and both
        // promise their values here: a held wait needs no driver query.
        Ok(held
            .segments
            .iter()
            .position(|segment| {
                segment
                    .waits
                    .iter()
                    .any(|&(semaphore, value)| self.promised(semaphore) < value)
            })
            .unwrap_or(held.segments.len()))
    }

    /// The highest value submitted work or the CPU signals on `semaphore`.
    fn promised(&self, semaphore: Semaphore) -> u64 {
        self.queue
            .promised
            .iter()
            .find(|&&(promised, _)| promised == semaphore)
            .map_or(0, |&(_, value)| value)
    }

    fn promise(&mut self, semaphore: Semaphore, value: u64) {
        match self
            .queue
            .promised
            .iter_mut()
            .find(|(promised, _)| *promised == semaphore)
        {
            Some((_, promised)) => *promised = (*promised).max(value),
            None => self.queue.promised.push((semaphore, value)),
        }
    }

    /// Points held work's completion at the timeline, or at the late
    /// timeline when a later serial already signaled the timeline, and
    /// notes what its segments signal. Work that does not complete its
    /// command buffer signals the next late value last, which says when
    /// the submitted part is done.
    fn track(&mut self, held: &mut Held) -> Result<()> {
        let timeline = self.timeline;
        let completes = held
            .segments
            .iter()
            .any(|segment| segment.signals.contains(&(timeline, held.serial)));
        let late = |device: &mut Self| -> Result<(Semaphore, u64)> {
            let late = match device.queue.late {
                Some(late) => late,
                None => *device
                    .queue
                    .late
                    .insert(device.backend.create_semaphore(0)?),
            };
            device.queue.late_value += 1;
            Ok((late, device.queue.late_value))
        };
        let track = if !completes {
            let (late, value) = late(self)?;
            let last = held.segments.last_mut().expect("a segment");
            last.signals.push((late, value));
            Track::Started(value)
        } else if held.serial > self.queue.timeline_serial {
            self.queue.timeline_serial = held.serial;
            Track::Timeline
        } else {
            let (late, value) = late(self)?;
            for segment in &mut held.segments {
                for signal in &mut segment.signals {
                    if *signal == (timeline, held.serial) {
                        *signal = (late, value);
                    }
                }
            }
            Track::Late(value)
        };
        if let Some(flight) = self
            .queue
            .in_flight
            .iter_mut()
            .find(|flight| flight.serial == held.serial)
        {
            flight.track = track;
        }
        let late = self.queue.late;
        let named = held
            .segments
            .iter()
            .flat_map(|segment| &segment.signals)
            .copied();
        if let Some(flight) = self
            .queue
            .in_flight
            .iter_mut()
            .find(|flight| flight.serial == held.serial)
        {
            flight.named.extend(named);
        }
        let signals: Vec<_> = held
            .segments
            .iter()
            .flat_map(|segment| &segment.signals)
            .copied()
            .filter(|&(semaphore, _)| semaphore != timeline && Some(semaphore) != late)
            .collect();
        for &(semaphore, value) in &signals {
            self.promise(semaphore, value);
        }
        if let Some(flight) = self
            .queue
            .in_flight
            .iter_mut()
            .find(|flight| flight.serial == held.serial)
        {
            flight.signals.extend(signals);
        }
        Ok(())
    }

    /// Makes held segments' binds, reading indirect regions from the copies
    /// of their arguments, and submits the segments. Metal leaves regions
    /// outside a texture undefined, as it does textures destroyed while in
    /// use, and counts past the buffer's end; none maps anything. A count
    /// past [`MAP_INDIRECT_MAPPINGS`] maps that many and is reported.
    fn submit_held(&mut self, mut held: Held) -> Result<()> {
        for (segment, mapping) in core::mem::take(&mut held.mappings) {
            let request = match mapping {
                Mapping::Request(request) => request,
                Mapping::Indirect {
                    image,
                    mode,
                    arguments,
                    capacity,
                } => {
                    let memory = self.chunk_memory(arguments.buffer)?;
                    let mut count = [0; 4];
                    self.backend
                        .read_memory(memory, arguments.offset, &mut count)?;
                    let count = u64::from(u32::from_le_bytes(count));
                    if count > capacity && capacity == MAP_INDIRECT_MAPPINGS {
                        self.unsupported(
                            "indirect sparse mapping updates of more than 4096 regions",
                        );
                    }
                    let mut bytes = alloc::vec![0; (count.min(capacity) as usize) * MAP_INDIRECT_ARGUMENTS_BYTES];
                    self.backend
                        .read_memory(memory, arguments.offset + 4, &mut bytes)?;
                    // A region outside the texture maps nothing of its own.
                    let updates = bytes
                        .as_chunks::<MAP_INDIRECT_ARGUMENTS_BYTES>()
                        .0
                        .iter()
                        .map(MappingUpdate::from_indirect)
                        .filter(|update| {
                            self.check_mapping(&MappingRequest::Update {
                                image,
                                mode,
                                updates: alloc::vec![*update],
                            })
                            .is_ok()
                        })
                        .collect();
                    MappingRequest::Update {
                        image,
                        mode,
                        updates,
                    }
                }
            };
            if self.check_mapping(&request).is_ok() {
                let binds = self.sparse_binds(&request)?;
                held.segments[segment].binds.extend(binds);
            }
        }
        self.track(&mut held)?;
        if let Some(flight) = self
            .queue
            .in_flight
            .iter_mut()
            .find(|flight| flight.serial == held.serial)
        {
            flight.submits += 1;
            flight.segments += held.segments.len() as u64;
        }
        // NVK names a refused segment's failure by the next signal it
        // submits, so a submission must end with one: a refusal then always
        // lands on this command buffer, never on the next one's.
        debug_assert!(
            held.segments
                .last()
                .is_some_and(|segment| !segment.signals.is_empty()),
            "a submission ends with a signal"
        );
        self.backend.submit(&held.segments)
    }

    /// Waits for the held work through `serial` whose arguments are being
    /// copied to be submitted; `false` when a step's wait times out. Work
    /// waiting for an event stays held: only a signal readies it.
    fn submit_held_through(&mut self, serial: u64, timeout: u64) -> Result<bool> {
        self.advance()?;
        let steps = self.queue.steps;
        while let Some(step) = self
            .queue
            .held
            .iter()
            .find(|held| held.serial <= serial && held.step > 0)
            .map(|held| held.step)
        {
            let steps = steps.expect("steps have their semaphore");
            if self.backend.semaphore_value(steps)? >= step {
                // Copied, but held for an event or work ahead of it.
                break;
            }
            if !self.backend.wait_semaphore(steps, step, timeout)? {
                return Ok(false);
            }
            self.advance()?;
        }
        Ok(true)
    }

    fn chunk_memory(&self, buffer: crate::metal::vulkan::object::Buffer) -> Result<Memory> {
        let found = self
            .queue
            .chunks
            .iter()
            .find(|(chunk, _)| chunk.buffer == buffer);
        found
            .map(|(_, memory)| *memory)
            .ok_or(Error::Invalid("unknown upload chunk"))
    }

    /// The work in flight, oldest first: each command buffer's workload and
    /// its pipelines by their functions' entry points, which name what a lost
    /// device was running. A pipeline released meanwhile shows as `?`, or as
    /// whatever reused its slot.
    pub fn work_in_flight(&self) -> String {
        let mut work = String::new();
        for in_flight in &self.queue.in_flight {
            if !work.is_empty() {
                work.push_str("; ");
            }
            let load = in_flight.workload;
            work.push_str(&format!(
                "command buffer {} ({} draws, largest {}, {} indirect, {} passes, {} dispatches, \
                 {} transfers, {} waited queries, {} conditional, {} internal):",
                in_flight.serial,
                load.direct,
                load.largest,
                load.indirect,
                load.passes,
                load.dispatches,
                load.transfers,
                load.waited_queries,
                load.conditional,
                load.internal
            ));
            // How long it waited, and whether it read other textures' CPU
            // copies, say whether it ran on memory freed meanwhile.
            work.push_str(&format!(
                " {:?} {} ticks after commit{};",
                in_flight.track,
                self.timing.now().wrapping_sub(in_flight.committed),
                if in_flight.carries {
                    ", carrying CPU-copy uploads or decodes"
                } else {
                    ""
                }
            ));
            if let Some(widest) = load.widest {
                work.push_str(&format!(" widest dispatch {widest};"));
            }
            for &pipeline in &in_flight.pipelines {
                let entries = self.pipelines.entries(pipeline);
                let entries =
                    entries.map_or_else(|_| String::from("?"), |entries| entries.join("+"));
                work.push(' ');
                work.push_str(&entries);
            }
        }
        work
    }

    /// Submits ready held work, retires completed command buffers and
    /// returns the completed serial.
    pub fn poll(&mut self) -> Result<u64> {
        let result = self.poll_inner(None, false);
        if result == Err(Error::DeviceLost) {
            self.mark_lost();
        }
        result
    }

    /// [`Self::poll`] with the queue's timeline known to have reached
    /// `timeline`, as the completion waiter read it: no driver query (a host
    /// kernel call) unless work completes on the late timeline. The answer
    /// may lag the GPU, never lead it.
    pub fn poll_reached(&mut self, timeline: u64) -> Result<u64> {
        let result = self.poll_inner(Some(timeline), false);
        if result == Err(Error::DeviceLost) {
            self.mark_lost();
        }
        result
    }

    /// What connection `key` sees complete after a [`Self::poll`]: every
    /// serial before its oldest command buffer not yet complete, or every
    /// serial submitted. Another connection's held work never holds it.
    pub fn reached_for(&self, key: u64) -> u64 {
        self.queue
            .in_flight
            .iter()
            .find(|flight| flight.key == key && flight.track != Track::Done)
            .map_or(self.submitted, |flight| flight.serial - 1)
    }

    /// Every serial before the oldest command buffer not yet complete,
    /// held or running, after a [`Self::poll`].
    pub fn drained(&self) -> u64 {
        self.queue
            .in_flight
            .iter()
            .find(|flight| flight.track != Track::Done)
            .map_or(self.submitted, |flight| flight.serial - 1)
    }

    /// Whether command buffer `serial` was submitted and completed, after
    /// a [`Self::poll`], whatever completed before it.
    pub fn serial_complete(&self, serial: u64) -> bool {
        serial <= self.submitted
            && !self
                .queue
                .in_flight
                .iter()
                .any(|flight| flight.serial == serial && flight.track != Track::Done)
    }

    /// Whether some connection's completion differs from the device's:
    /// work is held, or completes on the late timeline.
    pub fn holds(&self) -> bool {
        self.queue.in_flight.iter().any(|flight| {
            matches!(
                flight.track,
                Track::Held | Track::Started(_) | Track::Late(_)
            )
        })
    }

    /// The connections with command buffers held for an event, oldest
    /// first.
    pub fn held_connections(&self) -> Vec<u64> {
        self.queue.holders()
    }

    /// Command buffers seen complete so far, in whatever order they
    /// completed.
    pub fn completions(&self) -> u64 {
        self.queue.completions
    }

    /// The oldest of connection `key`'s command buffers after `serial` that
    /// completed while an older one of its own has not, after a
    /// [`Self::poll`]; zero for none. Another Metal queue's held work
    /// delays [`Self::reached_for`], not these.
    /// The oldest of connection `key`'s completed command buffers whose
    /// submission the driver refused, not yet taken; zero for none.
    pub fn submit_failed_for(&self, key: u64) -> u64 {
        self.queue
            .refused
            .iter()
            .filter(|&&(refused, ..)| refused == key)
            .map(|&(_, serial, ..)| serial)
            .min()
            .unwrap_or(0)
    }

    /// Takes the failure of connection `key`'s command buffer `serial`,
    /// which completed without running all of its commands; `None` for one
    /// that ran them all.
    pub fn take_submit_failure(&mut self, key: u64, serial: u64) -> Option<Error> {
        let index = self
            .queue
            .refused
            .iter()
            .position(|&(refused, at, ..)| refused == key && at == serial)?;
        let (_, _, error, records) = self.queue.refused.remove(index);
        self.backend.settle_failures(records, false);
        Some(error)
    }

    /// [`Backend::count_failures`]: `outstanding` counts the refusals not
    /// yet answered, readable without the device.
    pub fn count_failures(
        &mut self,
        outstanding: &alloc::sync::Arc<core::sync::atomic::AtomicU32>,
    ) {
        self.backend.count_failures(outstanding);
    }

    pub fn completed_after(&self, key: u64, serial: u64) -> u64 {
        self.queue
            .early
            .iter()
            .filter(|&&(early, completed)| early == key && completed > serial)
            .map(|&(_, completed)| completed)
            .min()
            .unwrap_or(0)
    }

    /// Drops connection `key`'s held segments, which no longer run: it
    /// closed. Segments already submitted still finish.
    pub fn abandon(&mut self, key: u64) {
        let mut dropped = Vec::new();
        self.queue.held.retain(|held| {
            if held.key == key {
                dropped.push(held.serial);
            }
            held.key != key
        });
        for flight in &mut self.queue.in_flight {
            if !dropped.contains(&flight.serial) {
                continue;
            }
            flight.track = match flight.track {
                // Done once its submitted segments are.
                Track::Started(value) => Track::Late(value),
                _ => Track::Done,
            };
        }
        self.queue.early.retain(|&(early, _)| early != key);
        let mut settled = 0;
        self.queue.refused.retain(|&(refused, _, _, records)| {
            if refused == key {
                settled += records;
            }
            refused != key
        });
        if settled > 0 {
            self.backend.settle_failures(settled, false);
        }
    }

    /// [`Self::poll`] taking the completion marker's serial as the queue's
    /// timeline, and asking only for the late timeline while work completes
    /// there: a guest's
    /// poll then needs no driver query (a host kernel call a guest
    /// processor waits for the boot processor to serve) even while held
    /// work keeps completion from being published, and even when nothing
    /// new completed. The marker never leads the timeline, so the answer
    /// may lag, never lead. A loss is still noticed by the platform's look,
    /// which asks the driver whenever the marker shows nothing new.
    pub fn poll_marked(&mut self) -> Result<u64> {
        // Work completing on the late timeline still asks for that
        // timeline alone: the marker stands for the queue's own timeline
        // whatever was submitted late, as every late submission writes it
        // only after everything submitted before it.
        match self.marker_serial() {
            Some(marked) => {
                let result = self.poll_inner(Some(marked.max(self.completed)), true);
                if result == Err(Error::DeviceLost) {
                    self.mark_lost();
                }
                result
            }
            None => self.poll(),
        }
    }

    /// The serial the completion marker shows, widened from its 32 bits
    /// against the newest serial submitted.
    fn marker_serial(&self) -> Option<u64> {
        let word = self.completion_word()?;
        // SAFETY: the word is this device's mapped marker, alive with it.
        let low = u64::from(unsafe { word.as_ref() }.load(core::sync::atomic::Ordering::Acquire));
        let value = (self.submitted & !0xffff_ffff) | low;
        if value > self.submitted {
            value.checked_sub(1 << 32)
        } else {
            Some(value)
        }
    }

    fn poll_inner(&mut self, known: Option<u64>, trusted: bool) -> Result<u64> {
        self.advance()?;
        if self.lost {
            return Err(Error::DeviceLost);
        }
        // Nothing outstanding can have completed, and this runs on every pump
        // of the boot processor: asking would enter the driver for nothing.
        // What a bounded retirement left is destroyed now.
        if self.queue.in_flight.is_empty() {
            if self.destruction_left {
                self.retire(self.completed)?;
            }
            return Ok(self.completed);
        }
        let lost = |error: &Error| *error == Error::DeviceLost;
        let timeline = match known {
            // A value that shows no progress may hide a loss, which only the
            // driver's answer says.
            Some(timeline) if trusted || (!self.late_in_flight() && timeline > self.completed) => {
                // The backend recycles what finished submissions held, as
                // after a query; else it asks the driver itself once enough
                // wait. The marker shows the submission at `timeline` writing
                // it, not finished: the queue runs its submissions in order,
                // so every earlier one is.
                let passed = if trusted {
                    timeline.saturating_sub(1)
                } else {
                    timeline
                };
                if passed != 0 {
                    self.backend.timeline_passed(self.timeline, passed);
                }
                timeline
            }
            _ => {
                let timeline = self.backend.semaphore_value(self.timeline);
                timeline.inspect_err(|error| self.lost |= lost(error))?
            }
        };
        let late = match self.queue.late {
            Some(late) if self.late_in_flight() => {
                let value = self.backend.semaphore_value(late);
                value.inspect_err(|error| self.lost |= lost(error))?
            }
            _ => 0,
        };
        let mut reached_events = Vec::new();
        let mut done = Vec::new();
        for flight in &mut self.queue.in_flight {
            match flight.track {
                Track::Timeline if flight.serial <= timeline => flight.track = Track::Done,
                Track::Late(value) if value <= late => flight.track = Track::Done,
                _ => continue,
            }
            // What it signaled is reached: an event read needs no query.
            reached_events.append(&mut flight.signals);
            done.push((
                flight.key,
                flight.serial,
                core::mem::take(&mut flight.named),
            ));
        }
        for (semaphore, value) in reached_events {
            self.note_reached_event(semaphore, value);
        }
        // A submission the driver refused still signaled, after its waits
        // and what of it ran: its command buffer fails, the device goes on.
        for (key, serial, named) in done {
            let mut refusal = None;
            let mut records = 0;
            for (semaphore, value) in named {
                if let Some(error) = self.backend.submit_failure(semaphore, value) {
                    refusal = refusal.or(Some(error));
                    records += 1;
                }
            }
            if let Some(error) = refusal {
                self.queue.refused.push((key, serial, error, records));
            }
        }
        // Everything through the first command buffer whose submitted work
        // still runs; held segments run nothing yet. So `completed` may pass
        // a held serial. Deferred
        // destruction does not: it waits for `freeable`, as a held command
        // buffer still names what was released after it was committed.
        let completed = self
            .queue
            .in_flight
            .iter()
            .find(|flight| match flight.track {
                Track::Started(value) => value > late,
                Track::Timeline | Track::Late(_) => true,
                Track::Held | Track::Done => false,
            })
            .map_or(self.submitted, |flight| flight.serial - 1);
        self.retire(completed)?;
        // With nothing in flight, a refusal no command buffer took names
        // only the device's own work: nobody will ask for it.
        if self.queue.in_flight.is_empty() {
            self.backend.settle_failures(0, true);
        }
        // Out of order completions the connection now reaches in order.
        let mut early = core::mem::take(&mut self.queue.early);
        early.retain(|&(key, serial)| serial > self.reached_for(key));
        self.queue.early = early;
        Ok(self.completed)
    }

    /// Every serial through which deferred destruction may free what was
    /// released: [`Self::completed_serial`], but never past the oldest
    /// command buffer not yet done, held ones included. `completed` passes
    /// a held command buffer, yet a held command buffer names what it will
    /// read once its event arrives: a resource its app released after
    /// committing it (an unretained-reference command buffer, which Core
    /// Animation commits and then drops the IOSurface texture of; a Mac's
    /// kernel keeps such memory until the GPU is done), and the CPU-copy
    /// uploads and decodes of any connection's textures that ride the next
    /// commit. Freed when `completed` passed it, its memory was unbound when
    /// the event released the command buffer: a GPU MMU fault that lost the
    /// device. A command buffer dropped unsubmitted with its connection is
    /// done. This is the device-wide rule, which execution set indices and
    /// releases of [`super::pins::Scope::DEVICE`] follow; a connection's
    /// own releases wait only for its command buffers ([`super::pins`]).
    pub(super) fn freeable(&self) -> u64 {
        self.queue
            .in_flight
            .iter()
            .find(|flight| flight.track != Track::Done)
            .map_or(self.completed, |flight| {
                self.completed.min(flight.serial - 1)
            })
    }

    fn late_in_flight(&self) -> bool {
        self.queue
            .in_flight
            .iter()
            .any(|flight| matches!(flight.track, Track::Started(_) | Track::Late(_)))
    }

    /// Recycles what completed work held, keeps its GPU times and destroys
    /// what waited for everything through `completed`; a failed
    /// invalidation or read is answered once the bookkeeping is done.
    fn retire(&mut self, completed: u64) -> Result<()> {
        let done = self
            .queue
            .in_flight
            .iter()
            .any(|flight| flight.track == Track::Done);
        if !done && completed <= self.completed && !self.destruction_left {
            return Ok(());
        }
        let seen = self.timing.now();
        self.calibrate_gpu_clock();
        let mut outcome = Ok(());
        let mut flights = VecDeque::new();
        for flight in core::mem::take(&mut self.queue.in_flight) {
            if flight.track == Track::Done {
                flights.push_back(flight);
            } else {
                self.queue.in_flight.push_back(flight);
            }
        }
        self.queue.completions += flights.len() as u64;
        for flight in &flights {
            let older = self
                .queue
                .in_flight
                .iter()
                .any(|kept| kept.key == flight.key && kept.serial < flight.serial);
            if older {
                self.queue.early.push((flight.key, flight.serial));
            }
        }
        while let Some(flight) = flights.pop_front() {
            // Before deferred destruction, which may free the memory.
            for range in flight.invalidations {
                outcome = outcome.and(self.backend.invalidate_memory(
                    range.memory,
                    range.offset,
                    range.length,
                ));
            }
            // Before the pool is reset.
            if let Some((pool, marks)) = flight.times {
                let mut stamps = alloc::vec![0; marks.spans.len() + 1];
                let read = &mut stamps[..];
                match self.backend.query_results(pool, 0, read) {
                    Ok(true) => {
                        let times = [read[0], read[read.len() - 1]];
                        self.keep_times(flight.serial, Some(times));
                        let frequency = u128::from(self.capabilities.timestamp_frequency);
                        let ns =
                            |ticks: u64| (u128::from(ticks) * 1_000_000_000 / frequency) as u64;
                        let pipelines = &self.pipelines;
                        let named = |index: usize| {
                            flight
                                .pipelines
                                .get(index)
                                .map_or_else(Vec::new, |&pipeline| {
                                    pipelines.entries(pipeline).unwrap_or_default()
                                })
                        };
                        if marks.profiled {
                            let counts = CommandBufferCounts {
                                submits: flight.submits,
                                segments: flight.segments,
                                ..marks.counts
                            };
                            self.queue.passes.command_buffer(counts);
                            for ((bound, shape), pair) in
                                marks.spans.iter().zip(&marks.shapes).zip(read.windows(2))
                            {
                                let entries = bound.iter().flat_map(|&index| named(index.into()));
                                let names = super::gpu_times::names(entries);
                                let target = shape.target.and_then(|view| {
                                    self.queue.targets.get(&view).map(|&(_, target)| target)
                                });
                                let key = pass_times::span_key(
                                    shape.kind,
                                    &names,
                                    shape.extent,
                                    target.filter(|_| shape.kind == SpanKind::Render),
                                );
                                self.queue.passes.span(
                                    key,
                                    SpanWork {
                                        ns: ns(pair[1]).saturating_sub(ns(pair[0])),
                                        draws: shape.draws.into(),
                                        dispatches: shape.dispatches.into(),
                                        texels: shape.texels,
                                    },
                                );
                            }
                        }
                        let spans = marks
                            .spans
                            .iter()
                            .zip(&marks.shapes)
                            .zip(read.windows(2))
                            .map(|((bound, shape), pair)| {
                                let entries = bound.iter().flat_map(|&index| named(index.into()));
                                let mut entries: Vec<String> = entries.collect();
                                entries.push(shape.describe());
                                (ns(pair[1]).saturating_sub(ns(pair[0])), entries)
                            });
                        self.queue.clock.retire(
                            flight.serial,
                            flight.committed,
                            seen,
                            [ns(times[0]), ns(times[1])],
                            || (0..flight.pipelines.len()).flat_map(named).collect(),
                            spans,
                        );
                    }
                    Ok(false) => self.keep_times(flight.serial, None),
                    Err(error) => {
                        self.keep_times(flight.serial, None);
                        outcome = outcome.and(Err(error));
                    }
                }
                self.retire_query_pool(pool, QueryType::Timestamp, marks.queries);
            }
            self.queue.free_chunks.extend(flight.chunks);
            for (query_type, pool) in flight.pools {
                self.retire_query_pool(pool, query_type, POOL_QUERIES);
            }
        }
        // A held command buffer that finished frees what waited for it even
        // when `completed` passed it while it was held.
        self.completed = self.completed.max(completed);
        // Each release waits only for the command buffers that may read it
        // (`pins`): another connection's held work does not keep it.
        // A bounded retirement (the boot processor's look, whose guest
        // waits meanwhile) stops destroying once its budget is spent and
        // leaves the rest, in order, for the next.
        let (budget, began) = (self.destruction_budget, self.timing.now());
        let mut pinned = 0;
        self.destruction_left = false;
        let mut kept = VecDeque::with_capacity(self.deferred.len());
        for (release, object, allocated) in core::mem::take(&mut self.deferred) {
            let frees = !self.queue.reads(&release);
            let spent = budget != 0 && self.timing.now().wrapping_sub(began) >= budget;
            if frees && !spent {
                self.destroy_now(object);
                self.allocated -= allocated;
                continue;
            }
            if frees {
                self.destruction_left = true;
            } else if release.serial <= self.completed {
                // Everything running at its release is done: held work
                // keeps it.
                pinned += allocated;
            }
            kept.push_back((release, object, allocated));
        }
        self.deferred = kept;
        self.peak_pinned = self.peak_pinned.max(pinned);
        self.descriptors
            .retire_with(|release| !self.queue.reads(release));
        self.execution_sets.retire(self.freeable());
        outcome
    }

    /// Reads the GPU's clock against the platform's when a reading is due
    /// and timed work is retiring; a device that cannot is not asked again.
    fn calibrate_gpu_clock(&mut self) {
        let timed = self
            .queue
            .in_flight
            .front()
            .is_some_and(|flight| flight.times.is_some());
        if !timed || !self.queue.clock.due() {
            return;
        }
        let before = self.timing.now();
        match self.backend.calibrated_timestamps() {
            Ok((_, gpu)) => {
                let after = self.timing.now();
                let frequency = u128::from(self.capabilities.timestamp_frequency);
                let gpu_ns = (u128::from(gpu) * 1_000_000_000 / frequency) as u64;
                self.queue.clock.calibrate(before, after, gpu_ns);
            }
            Err(_) => self.queue.clock.disable(),
        }
    }

    /// Where retired command buffers' time went since the last call.
    pub fn take_gpu_times(&mut self) -> GpuTimes {
        self.queue.clock.take()
    }

    /// Times each pass apart and reports [`PassTimes`], as `--profile`
    /// does for every device ([`pass_times::set_pass_profile`]).
    pub fn set_pass_profile(&mut self, on: bool) {
        self.queue.profile = on;
    }

    /// Whether command buffers time each pass apart.
    pub fn passes_profiled(&self) -> bool {
        self.queue.profile || pass_times::pass_profile()
    }

    /// The profile's spans and counts since the last call; `None` when
    /// passes are not profiled or the device has no timestamps.
    pub fn take_pass_times(&mut self) -> Option<PassTimes> {
        (self.passes_profiled() && self.capabilities.timestamp_frequency != 0)
            .then(|| self.queue.passes.take())
    }

    /// Notes what a pass attaching `view` of `texture` renders into, for
    /// the profile's report.
    fn note_target(&mut self, view: ImageView, texture: TextureId) {
        let Ok(descriptor) = self.texture_descriptor(texture) else {
            return;
        };
        let target = TargetInfo {
            format: descriptor.pixel_format,
            memory: self.target_memory(texture),
        };
        let queue = &mut self.queue;
        queue.noted += 1;
        queue.targets.insert(view, (queue.noted, target));
        if queue.targets.len() > TARGETS_KEPT {
            // The oldest note goes: its view is likely gone.
            let oldest = queue
                .targets
                .iter()
                .min_by_key(|(_, (noted, _))| *noted)
                .map(|(view, _)| *view);
            if let Some(oldest) = oldest {
                queue.targets.remove(&oldest);
            }
        }
    }

    fn keep_times(&mut self, serial: u64, times: Option<[u64; 2]>) {
        let kept = &mut self.queue.times;
        if kept.len() == TIMES_KEPT {
            kept.pop_front();
        }
        // Held work completes after later serials; the times stay in order.
        let at = kept.partition_point(|&(kept, _)| kept < serial);
        kept.insert(at, (serial, times));
    }

    /// `newEvent` and `newSharedEvent`: both are timeline semaphores.
    pub fn new_event(&mut self) -> Result<EventId> {
        let semaphore = self.backend.create_semaphore(0)?;
        Ok(EventId(self.queue.events.insert(semaphore)))
    }

    /// The semaphore command buffers signal and wait on.
    pub fn event_semaphore(&self, id: EventId) -> Result<Semaphore> {
        self.queue.events.get(id.0).copied()
    }

    /// `signaledValue` of a shared event.
    pub fn signaled_value(&mut self, id: EventId) -> Result<u64> {
        let semaphore = self.event_semaphore(id)?;
        self.event_value(semaphore)
    }

    /// The value of event semaphore `semaphore`. Only submitted work and
    /// the CPU's signals raise it, and both promise their values first, so
    /// it never passes what was promised: once it is known to have reached
    /// that, the answer needs no driver query (a host kernel call).
    fn event_value(&mut self, semaphore: Semaphore) -> Result<u64> {
        if self.lost {
            return Err(Error::DeviceLost);
        }
        let promised = self.promised(semaphore);
        if self.reached_event(semaphore) >= promised {
            return Ok(promised);
        }
        let value = self.backend.semaphore_value(semaphore)?;
        self.note_reached_event(semaphore, value);
        // A GPU signal queued before a higher CPU signal can reach the
        // event after it, and the driver then answers the lower value until
        // that work runs: the value never goes down.
        Ok(self.reached_event(semaphore))
    }

    fn reached_event(&self, semaphore: Semaphore) -> u64 {
        self.queue
            .reached_events
            .iter()
            .find(|&&(reached, _)| reached == semaphore)
            .map_or(0, |&(_, value)| value)
    }

    /// Notes that event semaphore `semaphore` reached `value`. Only a live
    /// event's: a destroyed event's semaphore has no promise, and a flight
    /// that signaled it may complete after it was destroyed; noted then,
    /// the value would answer for the next event that gets the freed handle
    /// before its own signal ran.
    fn note_reached_event(&mut self, semaphore: Semaphore, value: u64) {
        if !self
            .queue
            .promised
            .iter()
            .any(|&(promised, _)| promised == semaphore)
        {
            return;
        }
        match self
            .queue
            .reached_events
            .iter_mut()
            .find(|(reached, _)| *reached == semaphore)
        {
            Some((_, reached)) => *reached = (*reached).max(value),
            None => self.queue.reached_events.push((semaphore, value)),
        }
    }

    /// `setSignaledValue:` from the CPU; timeline values only increase.
    pub fn set_signaled_value(&mut self, id: EventId, value: u64) -> Result<()> {
        let semaphore = self.event_semaphore(id)?;
        let current = self.event_value(semaphore)?;
        if value > current {
            self.backend.signal_semaphore(semaphore, value)?;
            // Promised first: only a promised event's value is noted.
            self.promise(semaphore, value);
            self.note_reached_event(semaphore, value);
            // Work held for the value goes to the queue now.
            self.advance()?;
        }
        Ok(())
    }

    /// `waitUntilSignaledValue:timeoutMS:`. Held work may signal the event,
    /// so it is submitted first; `timeout` bounds each wait.
    pub fn wait_event(&mut self, id: EventId, value: u64, timeout: u64) -> Result<bool> {
        if self.lost {
            return Err(Error::DeviceLost);
        }
        let semaphore = self.event_semaphore(id)?;
        if self.event_value(semaphore)? >= value {
            return Ok(true);
        }
        if !self.submit_held_through(self.submitted, timeout)? {
            return Ok(false);
        }
        let reached = self.backend.wait_semaphore(semaphore, value, timeout)?;
        if reached {
            self.note_reached_event(semaphore, value);
        }
        Ok(reached)
    }

    pub fn destroy_event(&mut self, id: EventId) -> Result<()> {
        let semaphore = self.queue.events.remove(id.0)?;
        self.queue
            .promised
            .retain(|&(promised, _)| promised != semaphore);
        self.queue
            .reached_events
            .retain(|&(reached, _)| reached != semaphore);
        self.defer(Object::Semaphore(semaphore));
        Ok(())
    }

    /// `newCounterSampleBufferWithDescriptor:`.
    pub fn new_counter_sample_buffer(
        &mut self,
        set: CounterSet,
        count: u32,
    ) -> Result<CounterBufferId> {
        ensure(count > 0, "counter sample buffers hold samples")?;
        let storage = match set {
            CounterSet::Timestamp => {
                require(
                    self.capabilities.timestamp_frequency != 0,
                    "timestamp counters",
                )?;
                let info = QueryPoolInfo {
                    query_type: QueryType::Timestamp,
                    count,
                    statistics: PipelineStatistics::EMPTY,
                };
                let pool = self.backend.create_query_pool(&info)?;
                self.backend.reset_queries(pool, 0, count);
                CounterStorage::Timestamps(pool)
            }
            CounterSet::Statistic => {
                require(self.capabilities.statistic_counters, "statistic counters")?;
                let size = u64::from(count) * STATISTIC_SAMPLE_BYTES;
                let info = BufferInfo {
                    size,
                    usage: BufferUsage::TRANSFER_SRC
                        | BufferUsage::TRANSFER_DST
                        | BufferUsage::STORAGE_BUFFER
                        | BufferUsage::SHADER_DEVICE_ADDRESS,
                    flags: BufferCreate::EMPTY,
                };
                let request = MemoryRequest {
                    required: MemoryProperties::HOST_VISIBLE | MemoryProperties::HOST_COHERENT,
                    preferred: MemoryProperties::HOST_CACHED,
                    mapped: true,
                };
                let allocation = self.backend.create_buffer(&info, request)?;
                let unsampled = alloc::vec![0xff; size as usize];
                if let Err(error) = self.backend.write_memory(allocation.memory, 0, &unsampled) {
                    self.backend.destroy(Object::Buffer(allocation.buffer));
                    self.backend.destroy(Object::Memory(allocation.memory));
                    return Err(error);
                }
                self.allocated += allocation.allocated;
                CounterStorage::Statistics(allocation)
            }
        };
        let buffer = CounterBuffer { storage, count };
        Ok(CounterBufferId(self.queue.counters.insert(buffer)))
    }

    fn counter_buffer(&self, id: CounterBufferId) -> Result<CounterBuffer> {
        self.queue.counters.get(id.0).copied()
    }

    /// Where sample `index` of `id` lands.
    pub fn counter_sample(&self, id: CounterBufferId, index: u32) -> Result<CounterSample> {
        let buffer = self.counter_buffer(id)?;
        ensure(
            index < buffer.count,
            "sample index beyond the counter sample buffer",
        )?;
        Ok(match buffer.storage {
            CounterStorage::Timestamps(pool) => CounterSample::Timestamp { pool, query: index },
            CounterStorage::Statistics(allocation) => CounterSample::Statistic(
                allocation.address + u64::from(index) * STATISTIC_SAMPLE_BYTES,
            ),
        })
    }

    /// The samples `resolveCounters:inRange:…` copies.
    pub fn counter_range(
        &self,
        id: CounterBufferId,
        first: u32,
        count: u32,
    ) -> Result<CounterRange> {
        let buffer = self.counter_buffer(id)?;
        ensure(
            first
                .checked_add(count)
                .is_some_and(|end| end <= buffer.count),
            "range beyond the counter sample buffer",
        )?;
        Ok(match buffer.storage {
            CounterStorage::Timestamps(pool) => CounterRange::Timestamps { pool, first, count },
            CounterStorage::Statistics(allocation) => CounterRange::Statistics(BufferRef {
                buffer: allocation.buffer,
                offset: u64::from(first) * STATISTIC_SAMPLE_BYTES,
                address: allocation.address + u64::from(first) * STATISTIC_SAMPLE_BYTES,
                length: u64::from(count) * STATISTIC_SAMPLE_BYTES,
            }),
        })
    }

    /// `resolveCounterRange:` on the CPU, as 64-bit values. Timestamps are
    /// `None` while a sample is pending; statistics read as their command
    /// buffers last wrote them, which Metal resolves only after completion.
    pub fn resolve_counters(
        &mut self,
        id: CounterBufferId,
        first: u32,
        count: u32,
    ) -> Result<Option<Vec<u64>>> {
        let buffer = self.counter_buffer(id)?;
        ensure(
            first
                .checked_add(count)
                .is_some_and(|end| end <= buffer.count),
            "range beyond the counter sample buffer",
        )?;
        match buffer.storage {
            CounterStorage::Timestamps(pool) => {
                let mut results = alloc::vec![0; count as usize];
                Ok(self
                    .backend
                    .query_results(pool, first, &mut results)?
                    .then_some(results))
            }
            CounterStorage::Statistics(allocation) => {
                let mut bytes =
                    alloc::vec![0; (u64::from(count) * STATISTIC_SAMPLE_BYTES) as usize];
                let offset = u64::from(first) * STATISTIC_SAMPLE_BYTES;
                self.backend
                    .read_memory(allocation.memory, offset, &mut bytes)?;
                let values = bytes
                    .as_chunks::<8>()
                    .0
                    .iter()
                    .map(|&value| u64::from_le_bytes(value))
                    .collect();
                Ok(Some(values))
            }
        }
    }

    pub fn destroy_counter_sample_buffer(&mut self, id: CounterBufferId) -> Result<()> {
        let buffer = self.queue.counters.remove(id.0)?;
        match buffer.storage {
            CounterStorage::Timestamps(pool) => self.defer(Object::QueryPool(pool)),
            CounterStorage::Statistics(allocation) => {
                self.defer(Object::Buffer(allocation.buffer));
                self.release_memory(allocation.memory, allocation.allocated);
            }
        }
        Ok(())
    }

    /// `sampleTimestamps:gpuTimestamp:`.
    pub fn sample_timestamps(&mut self) -> Result<(u64, u64)> {
        self.backend.calibrated_timestamps()
    }
}

/// [`Resolver`] over a device.
pub struct DeviceResolver<'a, B: Backend, C: Compiler> {
    device: &'a mut Device<B, C>,
}

/// Upload chunks: host-visible memory the GPU reads and copies from,
/// preferably device-local (resizable BAR). Internal kernels also write
/// scratch there, such as the select kernel's conditional rendering flags.
const CHUNK_USAGE: BufferUsage = BufferUsage::SHADER_DEVICE_ADDRESS
    .union(BufferUsage::STORAGE_BUFFER)
    .union(BufferUsage::UNIFORM_BUFFER)
    .union(BufferUsage::VERTEX_BUFFER)
    .union(BufferUsage::INDEX_BUFFER)
    .union(BufferUsage::INDIRECT_BUFFER)
    .union(BufferUsage::CONDITIONAL_RENDERING)
    .union(BufferUsage::TRANSFER_SRC)
    .union(BufferUsage::TRANSFER_DST);

/// Upload chunks' memory.
const CHUNK_MEMORY: MemoryRequest = MemoryRequest {
    required: MemoryProperties::HOST_VISIBLE.union(MemoryProperties::HOST_COHERENT),
    preferred: MemoryProperties::DEVICE_LOCAL,
    mapped: true,
};

impl<B: Backend, C: Compiler> ChunkSource for DeviceResolver<'_, B, C> {
    fn acquire(&mut self, bytes: u64) -> Result<Chunk> {
        let queue = &mut self.device.queue;
        if let Some(index) = queue
            .free_chunks
            .iter()
            .position(|chunk| chunk.size >= bytes)
        {
            return Ok(queue.free_chunks.swap_remove(index));
        }
        // Generated commands preprocess into chunks too.
        let generated = self
            .device
            .backend
            .physical_device()
            .limits
            .generated_commands;
        let usage = CHUNK_USAGE.with(BufferUsage::PREPROCESS_BUFFER, generated.is_some());
        let info = BufferInfo {
            size: bytes,
            usage,
            flags: BufferCreate::EMPTY,
        };
        let allocation = self.device.backend.create_buffer(&info, CHUNK_MEMORY)?;
        let chunk = Chunk {
            buffer: allocation.buffer,
            address: allocation.address,
            size: bytes,
        };
        self.device.queue.chunks.push((chunk, allocation.memory));
        Ok(chunk)
    }
}

impl<B: Backend, C: Compiler> Resolver for DeviceResolver<'_, B, C> {
    fn compute_pipeline(
        &mut self,
        state: &ComputePipelineState,
        specialization: &Specialization,
    ) -> Result<Pipeline> {
        self.device.compute_variant(state.id, specialization)
    }

    fn clamped_sampler(
        &mut self,
        sampler: ResourceId,
        lod_min: f32,
        lod_max: f32,
    ) -> Result<ResourceId> {
        self.device.clamped_sampler(sampler, lod_min, lod_max)
    }

    fn bicubic_render_pipeline(
        &mut self,
        state: &RenderPipelineState,
        bicubic: [u32; 2],
    ) -> Result<Pipeline> {
        self.device.render_variant(state.id, bicubic)
    }

    fn mesh_pipeline(
        &mut self,
        state: &RenderPipelineState,
        object: [u32; 3],
        mesh: [u32; 3],
        object_lengths: &[(u32, u64)],
    ) -> Result<Pipeline> {
        self.device
            .mesh_variant(state.id, object, mesh, object_lengths)
    }

    fn query_pool(&mut self, query_type: QueryType) -> Result<(QueryPool, u32)> {
        let pool = self.device.take_query_pool(query_type, POOL_QUERIES)?;
        Ok((pool, POOL_QUERIES))
    }

    fn render_pipeline_state(&mut self, id: u64) -> Result<RenderPipelineState> {
        let id = u32::try_from(id).map_err(|_| Error::Invalid("unknown pipeline"))?;
        self.device
            .render_pipeline_state(RenderPipelineId(id))
            .cloned()
    }

    fn compute_pipeline_state(&mut self, id: u64) -> Result<ComputePipelineState> {
        let id = u32::try_from(id).map_err(|_| Error::Invalid("unknown pipeline"))?;
        self.device
            .compute_pipeline_state(ComputePipelineId(id))
            .cloned()
    }

    fn indirect_commands(
        &mut self,
        id: IndirectCommandBufferId,
    ) -> Result<Arc<IndirectCommandBuffer>> {
        self.device.indirect_snapshot(id)
    }

    fn indirect_records(
        &mut self,
        id: IndirectCommandBufferId,
    ) -> Result<(RecordLayout, BufferRef)> {
        self.device.indirect_records(id)
    }

    fn indirect_render_classes(&mut self) -> Vec<IndirectRenderClass> {
        self.device.indirect_render_classes()
    }

    fn indirect_compute_set(&mut self) -> Option<(ExecutionSet, Pipeline)> {
        self.device.indirect_compute_set()
    }

    fn generated_layout(
        &mut self,
        key: &GeneratedLayoutKey,
        set: Option<ExecutionSet>,
        sequences: u32,
    ) -> Result<(IndirectCommandsLayout, Requirements)> {
        self.device
            .generated_layout(key, set, sequences, CHUNK_MEMORY)
    }

    fn update_indirect_commands(
        &mut self,
        id: IndirectCommandBufferId,
        update: &mut dyn FnMut(&mut IndirectCommandBuffer) -> Result<()>,
    ) -> Result<()> {
        // A commit running again would apply the update twice.
        self.device.stop_deferring();
        update(self.device.indirect_command_buffer_mut(id)?)
    }
}

/// Has the segment that signals `completion` write `serial`'s low 32 bits to
/// the completion marker `marker` after all its work: a barrier makes every
/// earlier write in submission order available to the host first, and one
/// after the write flushes the word itself. So a host that reads the serial
/// there may treat every timeline serial through it as complete, as the
/// timeline's own value would say, without asking the driver. A held
/// command buffer whose completion later moves to the late timeline still
/// writes only after everything submitted before it, so a lower value it
/// writes late claims nothing that is not done.
fn mark_completion(
    segments: &mut [Segment],
    completion: (Semaphore, u64),
    marker: crate::metal::vulkan::object::Buffer,
    serial: u64,
) {
    use crate::metal::vulkan::{Access, PipelineStages, command::Barrier};
    for segment in segments
        .iter_mut()
        .filter(|segment| segment.signals.contains(&completion))
    {
        segment.commands.extend([
            Command::Barrier(Barrier {
                src_stages: PipelineStages::ALL_COMMANDS,
                src_access: Access::MEMORY_WRITE,
                dst_stages: PipelineStages::ALL_COMMANDS.union(PipelineStages::HOST),
                dst_access: Access::HOST_READ.union(Access::TRANSFER_WRITE),
            }),
            Command::FillBuffer {
                buffer: marker,
                offset: 0,
                size: 4,
                data: serial as u32,
            },
            Command::Barrier(Barrier {
                src_stages: PipelineStages::ALL_COMMANDS,
                src_access: Access::TRANSFER_WRITE,
                dst_stages: PipelineStages::HOST,
                dst_access: Access::HOST_READ,
            }),
        ]);
    }
}
