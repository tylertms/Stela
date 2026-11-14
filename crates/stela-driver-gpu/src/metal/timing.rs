//! Where the time inside the slowest guest calls goes: the phases of
//! `NewTexture`, `GetBytes`, `NewRenderPipeline`, `Commit` and
//! `NewFunction`, in the
//! platform's ticks.
//!
//! The device has no clock of its own. The shared service gives it the
//! platform's ([`Recorder::set_clock`]), which the device passes on to its
//! backend for the phases inside a Vulkan call. A call names its operation as
//! it starts ([`Recorder::begin`]); a phase of another operation, such as a
//! texture a commit's replay creates, is not counted. [`Totals`] sums the
//! calls the gate served between two reports, from any processor.
use crate::metal::pixel_format::PixelFormat;
use core::{
    fmt,
    sync::atomic::{AtomicU64, Ordering},
};

/// The most phases an operation has.
pub const PHASES: usize = 8;

/// Slow calls of one operation a report period shows a line for; the
/// report counts the rest.
pub const SLOW_SHOWN: u64 = 4;

/// A clock that never moves, until the platform gives one.
pub fn no_clock() -> u64 {
    0
}

/// An operation timed by phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Timed {
    NewTexture,
    GetBytes,
    NewRenderPipeline,
    Commit,
    NewFunction,
}

impl Timed {
    pub const ALL: [Self; 5] = [
        Self::NewTexture,
        Self::GetBytes,
        Self::NewRenderPipeline,
        Self::Commit,
        Self::NewFunction,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            Self::NewTexture => "new_texture",
            Self::GetBytes => "get_bytes",
            Self::NewRenderPipeline => "new_render_pipeline",
            Self::Commit => "commit",
            Self::NewFunction => "new_function",
        }
    }

    /// Its phases, in the order a line shows them.
    pub const fn phases(self) -> &'static [Phase] {
        use Phase::*;
        match self {
            Self::NewTexture => &[
                TexturePlan,
                TextureRecycle,
                TextureCreate,
                TextureAllocate,
                TextureBind,
                TextureViews,
                TextureDescriptors,
                TextureCpuCopy,
            ],
            Self::GetBytes => &[BytesRead, BytesCopy, BytesDecode, BytesUnpack, BytesWrite],
            Self::NewRenderPipeline => &[
                PipelineSpecialize,
                PipelineVertex,
                PipelineFragment,
                PipelinePlan,
                PipelineCreate,
            ],
            Self::Commit => &[
                CommitRead,
                CommitReplay,
                CommitFinish,
                CommitUpload,
                CommitReclaim,
                CommitRecord,
                CommitSubmit,
                CommitCompile,
            ],
            Self::NewFunction => &[FunctionReflect],
        }
    }
}

/// A phase of a [`Timed`] operation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Phase {
    /// Deriving the Vulkan image and views from the descriptor.
    TexturePlan,
    /// Looking for a destroyed texture to reuse.
    TextureRecycle,
    /// `vkCreateImage` and its memory requirements.
    TextureCreate,
    /// `vkAllocateMemory`, and its mapping when the CPU reaches it.
    TextureAllocate,
    /// `vkBindImageMemory` and the move to `GENERAL`.
    TextureBind,
    /// `vkCreateImageView` of each view.
    TextureViews,
    /// Writing the views' descriptors.
    TextureDescriptors,
    /// The host-visible buffer behind a managed or decoded texture.
    TextureCpuCopy,
    /// Reading the caller's span, whose bytes outside the rows stay.
    BytesRead,
    /// `vkCopyImageToMemory`, or the read of the CPU copy.
    BytesCopy,
    /// Converting a stored format to the caller's.
    BytesDecode,
    /// Moving the packed rows to the caller's strides.
    BytesUnpack,
    /// Writing the caller's span.
    BytesWrite,
    /// Function constants and constexpr samplers.
    PipelineSpecialize,
    /// AIR to NIR and the NIR passes of the vertex function, with the
    /// stages that feed a tessellated one.
    PipelineVertex,
    /// AIR to NIR and the NIR passes of the fragment function.
    PipelineFragment,
    /// Deriving the Vulkan pipeline's state.
    PipelinePlan,
    /// `vkCreateGraphicsPipelines`: NAK's compile of the NIR.
    PipelineCreate,
    /// Reading the command stream.
    CommitRead,
    /// Replaying the stream into a command buffer.
    CommitReplay,
    /// Ending the command buffer: its last pass and checks.
    CommitFinish,
    /// Copying uploads into their chunks and queueing CPU-copy writes.
    CommitUpload,
    /// Freeing completed command buffers before recording.
    CommitReclaim,
    /// Recording the Vulkan command buffers.
    CommitRecord,
    /// `vkQueueSubmit2`, and the sparse binds and held work around it.
    CommitSubmit,
    /// AIR to NIR of the pipeline variants the replay needed for a
    /// dispatch's or draw's shape; part of the replay's time.
    CommitCompile,
    /// The AIR compiler's reflection of the function with its constants,
    /// or its cache's answer.
    FunctionReflect,
}

impl Phase {
    /// Its operation and its slot among that operation's phases.
    fn place(self) -> (Timed, usize) {
        Timed::ALL
            .into_iter()
            .find_map(|operation| {
                let slot = operation.phases().iter().position(|phase| *phase == self);
                slot.map(|slot| (operation, slot))
            })
            .expect("a phase is among its operation's")
    }

    /// Whether the phase's time is part of another phase's, which counts
    /// it already.
    pub const fn nested(self) -> bool {
        matches!(self, Self::CommitCompile)
    }

    /// Whether every turn this phase held past the platform's limit is
    /// reported, rather than the first for its operation: a CPU copy through
    /// a texture's memory holds every process's calls for as long as it
    /// runs, so each one is a stall of the display. Each report needs a
    /// turn that long, which bounds how many there are.
    pub const fn reported_each_time(self) -> bool {
        matches!(self, Self::BytesCopy)
    }

    pub const fn name(self) -> &'static str {
        use Phase::*;
        match self {
            TexturePlan | PipelinePlan => "plan",
            TextureRecycle => "recycle",
            TextureCreate | PipelineCreate => "create",
            TextureAllocate => "alloc",
            TextureBind => "bind",
            TextureViews => "views",
            TextureDescriptors => "descriptors",
            TextureCpuCopy => "cpu_copy",
            BytesRead | CommitRead => "read",
            BytesCopy => "copy",
            BytesDecode => "decode",
            BytesUnpack => "unpack",
            BytesWrite => "write",
            PipelineSpecialize => "specialize",
            PipelineVertex => "vertex",
            PipelineFragment => "fragment",
            CommitReplay => "replay",
            CommitFinish => "finish",
            CommitUpload => "upload",
            CommitReclaim => "reclaim",
            CommitRecord => "record",
            CommitSubmit => "submit",
            CommitCompile => "compile",
            FunctionReflect => "reflect",
        }
    }
}

/// A function's entry point, cut to what a log line needs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Name {
    bytes: [u8; 40],
    length: u8,
}

impl Name {
    pub fn new(text: &str) -> Self {
        let mut length = text.len().min(40);
        while !text.is_char_boundary(length) {
            length -= 1;
        }
        let mut bytes = [0; 40];
        bytes[..length].copy_from_slice(&text.as_bytes()[..length]);
        Self {
            bytes,
            length: length as u8,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..usize::from(self.length)]).unwrap_or("?")
    }
}

/// What a slow call worked on.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Detail {
    #[default]
    None,
    Texture {
        format: PixelFormat,
        size: [u64; 3],
        slices: u64,
        levels: u64,
        /// The image's memory, zero for a reused texture.
        bytes: u64,
        recycled: bool,
    },
    Bytes {
        format: PixelFormat,
        width: u64,
        rows: u64,
        bands: u64,
        bytes: u64,
    },
    Pipeline {
        vertex: Name,
        fragment: Option<Name>,
    },
    Function {
        entry: Name,
    },
    Commit {
        stream: u64,
        uploads: u64,
    },
}

impl fmt::Display for Detail {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::None => Ok(()),
            Self::Texture {
                format,
                size: [width, height, depth],
                slices,
                levels,
                bytes,
                recycled,
            } => write!(
                formatter,
                " format={format:?} size={width}x{height}x{depth} slices={slices} levels={levels} bytes={bytes} recycled={recycled}"
            ),
            Self::Bytes {
                format,
                width,
                rows,
                bands,
                bytes,
            } => write!(
                formatter,
                " format={format:?} width={width} rows={rows} bands={bands} bytes={bytes}"
            ),
            Self::Pipeline { vertex, fragment } => write!(
                formatter,
                " vertex={} fragment={}",
                vertex.as_str(),
                fragment.as_ref().map_or("-", Name::as_str)
            ),
            Self::Function { entry } => write!(formatter, " entry={}", entry.as_str()),
            Self::Commit { stream, uploads } => {
                write!(formatter, " stream={stream} uploads={uploads}")
            }
        }
    }
}

/// One call's phases, in ticks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CallPhases {
    pub operation: Timed,
    /// From the call's start to its end.
    pub total: u64,
    /// In [`Timed::phases`] order.
    pub phases: [u64; PHASES],
    pub detail: Detail,
}

impl CallPhases {
    pub fn new(operation: Timed) -> Self {
        Self {
            operation,
            total: 0,
            phases: [0; PHASES],
            detail: Detail::None,
        }
    }

    /// What no phase accounts for.
    pub fn other(&self) -> u64 {
        other(self.operation, self.total, &self.phases)
    }

    /// The phase that took longest, and its ticks.
    pub fn longest_phase(&self) -> (&'static str, u64) {
        let (phase, ticks) = self.longest();
        (phase.map_or("other", Phase::name), ticks)
    }

    /// Whether a long turn of this call is reported each time
    /// ([`Phase::reported_each_time`]): by its longest phase.
    pub fn reported_each_time(&self) -> bool {
        self.longest().0.is_some_and(Phase::reported_each_time)
    }

    /// The phase that took longest, `None` for the time no phase accounts
    /// for, and its ticks.
    fn longest(&self) -> (Option<Phase>, u64) {
        let phases = self.operation.phases().iter().zip(self.phases);
        phases
            .map(|(&phase, ticks)| (Some(phase), ticks))
            .chain([(None, self.other())])
            .fold(
                (None, 0),
                |longest, phase| {
                    if phase.1 > longest.1 { phase } else { longest }
                },
            )
    }

    /// `GPU SLOW operation took_us=… most=phase/…us phase_us=… … detail`,
    /// showing the phases that took any time.
    pub fn slow_line(&self, tsc_hz: u64) -> impl fmt::Display + '_ {
        Line(move |formatter: &mut fmt::Formatter<'_>| {
            let (most, ticks) = self.longest_phase();
            write!(
                formatter,
                "GPU SLOW {} took_us={} most={most}/{}us",
                self.operation.name(),
                micros(self.total, tsc_hz),
                micros(ticks, tsc_hz),
            )?;
            for (phase, ticks) in self.operation.phases().iter().zip(self.phases) {
                if ticks != 0 {
                    write!(formatter, " {}_us={}", phase.name(), micros(ticks, tsc_hz))?;
                }
            }
            write!(
                formatter,
                " other_us={}{}",
                micros(self.other(), tsc_hz),
                self.detail
            )
        })
    }
}

fn other(operation: Timed, total: u64, phases: &[u64; PHASES]) -> u64 {
    total.saturating_sub(
        operation
            .phases()
            .iter()
            .zip(phases)
            .filter(|(phase, _)| !phase.nested())
            .fold(0, |sum, (_, ticks)| sum.saturating_add(*ticks)),
    )
}

/// `ticks` of a `tsc_hz` clock in microseconds.
pub fn micros(ticks: u64, tsc_hz: u64) -> u128 {
    u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1))
}

/// A line written by its closure.
struct Line<F>(F);

impl<F: Fn(&mut fmt::Formatter<'_>) -> fmt::Result> fmt::Display for Line<F> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        (self.0)(formatter)
    }
}

/// The phases of the call the device serves now, which a device owns.
pub struct Recorder {
    ticks: fn() -> u64,
    started: u64,
    call: Option<CallPhases>,
}

impl Default for Recorder {
    fn default() -> Self {
        Self {
            ticks: no_clock,
            started: 0,
            call: None,
        }
    }
}

impl Recorder {
    pub fn set_clock(&mut self, ticks: fn() -> u64) {
        self.ticks = ticks;
    }

    pub fn clock(&self) -> fn() -> u64 {
        self.ticks
    }

    pub fn now(&self) -> u64 {
        (self.ticks)()
    }

    /// Starts timing a call of `operation`, in place of any other.
    pub fn begin(&mut self, operation: Timed) {
        self.started = self.now();
        self.call = Some(CallPhases::new(operation));
    }

    /// Whether the call timed now is one of `operation`.
    pub fn timing(&self, operation: Timed) -> bool {
        self.call.is_some_and(|call| call.operation == operation)
    }

    /// Counts `ticks` to `phase`, when the call timed now is of its
    /// operation.
    pub fn add(&mut self, phase: Phase, ticks: u64) {
        let (operation, slot) = phase.place();
        if let Some(call) = self
            .call
            .as_mut()
            .filter(|call| call.operation == operation)
        {
            call.phases[slot] = call.phases[slot].saturating_add(ticks);
        }
    }

    /// Counts the ticks from `start` until now to `phase`; answers now, the
    /// next phase's start.
    pub fn since(&mut self, phase: Phase, start: u64) -> u64 {
        let now = self.now();
        self.add(phase, now.wrapping_sub(start));
        now
    }

    /// What the call timed now works on, when it is one of `operation`.
    pub fn detail(&mut self, operation: Timed, detail: Detail) {
        if let Some(call) = self
            .call
            .as_mut()
            .filter(|call| call.operation == operation)
        {
            call.detail = detail;
        }
    }

    /// What the call timed now works on, to complete.
    pub fn detail_mut(&mut self) -> Option<&mut Detail> {
        self.call.as_mut().map(|call| &mut call.detail)
    }

    /// The phases of the call timed now, which ends.
    pub fn take(&mut self) -> Option<CallPhases> {
        let mut call = self.call.take()?;
        call.total = self.now().wrapping_sub(self.started);
        Some(call)
    }
}

/// One operation's calls since the last report.
struct Sums {
    count: AtomicU64,
    total: AtomicU64,
    longest: AtomicU64,
    phases: [AtomicU64; PHASES],
    /// Calls at or over the slow threshold, and the lines shown of them.
    slow: AtomicU64,
    shown: AtomicU64,
}

/// The timed calls since the last report, summed by operation.
pub struct Totals {
    operations: [Sums; Timed::ALL.len()],
}

impl Default for Totals {
    fn default() -> Self {
        Self::new()
    }
}

/// One operation's calls in a report period, in ticks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Taken {
    pub operation: Timed,
    pub count: u64,
    pub total: u64,
    pub longest: u64,
    pub phases: [u64; PHASES],
    pub slow: u64,
}

impl Totals {
    pub const fn new() -> Self {
        Self {
            operations: [const {
                Sums {
                    count: AtomicU64::new(0),
                    total: AtomicU64::new(0),
                    longest: AtomicU64::new(0),
                    phases: [const { AtomicU64::new(0) }; PHASES],
                    slow: AtomicU64::new(0),
                    shown: AtomicU64::new(0),
                }
            }; Timed::ALL.len()],
        }
    }

    /// Adds `call`; answers whether to show its slow line: it took `slow`
    /// ticks or more (zero for never) and fewer than [`SLOW_SHOWN`] of its
    /// operation's were shown since the last report.
    pub fn add(&self, call: &CallPhases, slow: u64) -> bool {
        let sums = &self.operations[call.operation as usize];
        sums.count.fetch_add(1, Ordering::Relaxed);
        sums.total.fetch_add(call.total, Ordering::Relaxed);
        sums.longest.fetch_max(call.total, Ordering::Relaxed);
        for (sum, ticks) in sums.phases.iter().zip(call.phases) {
            if ticks != 0 {
                sum.fetch_add(ticks, Ordering::Relaxed);
            }
        }
        if slow == 0 || call.total < slow {
            return false;
        }
        sums.slow.fetch_add(1, Ordering::Relaxed);
        sums.shown.fetch_add(1, Ordering::Relaxed) < SLOW_SHOWN
    }

    /// Each operation's calls since the last call, those with any.
    pub fn take(&self) -> impl Iterator<Item = Taken> + '_ {
        Timed::ALL
            .into_iter()
            .zip(&self.operations)
            .filter_map(|(operation, sums)| {
                let count = sums.count.swap(0, Ordering::Relaxed);
                sums.shown.store(0, Ordering::Relaxed);
                (count != 0).then(|| Taken {
                    operation,
                    count,
                    total: sums.total.swap(0, Ordering::Relaxed),
                    longest: sums.longest.swap(0, Ordering::Relaxed),
                    phases: core::array::from_fn(|slot| {
                        sums.phases[slot].swap(0, Ordering::Relaxed)
                    }),
                    slow: sums.slow.swap(0, Ordering::Relaxed),
                })
            })
    }
}

impl Taken {
    /// What no phase accounts for.
    pub fn other(&self) -> u64 {
        other(self.operation, self.total, &self.phases)
    }

    /// `GPU PHASES operation count=… total_us=… phase_us=… … other_us=…
    /// longest_us=… slow=…`.
    pub fn line(&self, tsc_hz: u64) -> impl fmt::Display + '_ {
        Line(move |formatter: &mut fmt::Formatter<'_>| {
            write!(
                formatter,
                "GPU PHASES {} count={} total_us={}",
                self.operation.name(),
                self.count,
                micros(self.total, tsc_hz),
            )?;
            for (phase, ticks) in self.operation.phases().iter().zip(self.phases) {
                write!(formatter, " {}_us={}", phase.name(), micros(ticks, tsc_hz))?;
            }
            write!(
                formatter,
                " other_us={} longest_us={} slow={}",
                micros(self.other(), tsc_hz),
                micros(self.longest, tsc_hz),
                self.slow
            )
        })
    }
}

/// The time every gate exit should stay under: a processor in an exit takes
/// no guest interrupt, so a longer one is a dropped frame on the screen it
/// serves, and XNU panics at half a second.
pub const EXIT_TARGET_MICROS: u64 = 5_000;

/// The gate's exits since the last report, from any processor: the longest,
/// how many passed [`EXIT_TARGET_MICROS`], and the calls answered `Pending`
/// at the wait bound.
pub struct ExitWatch {
    exits: AtomicU64,
    over: AtomicU64,
    longest: AtomicU64,
    operation: AtomicU64,
    gave_up: AtomicU64,
}

impl ExitWatch {
    pub const fn new() -> Self {
        Self {
            exits: AtomicU64::new(0),
            over: AtomicU64::new(0),
            longest: AtomicU64::new(0),
            operation: AtomicU64::new(0),
            gave_up: AtomicU64::new(0),
        }
    }

    /// Counts one exit of `ticks` that served `operation`.
    pub fn note(&self, ticks: u64, operation: u32, gave_up: bool, tsc_hz: u64) {
        self.exits.fetch_add(1, Ordering::Relaxed);
        if gave_up {
            self.gave_up.fetch_add(1, Ordering::Relaxed);
        }
        if micros(ticks, tsc_hz) >= u128::from(EXIT_TARGET_MICROS) {
            self.over.fetch_add(1, Ordering::Relaxed);
        }
        if self.longest.fetch_max(ticks, Ordering::Relaxed) < ticks {
            self.operation
                .store(u64::from(operation), Ordering::Relaxed);
        }
    }

    /// What was counted since the last call, which starts the next count;
    /// `None` without exits.
    pub fn take(&self) -> Option<Exits> {
        let exits = self.exits.swap(0, Ordering::Relaxed);
        let taken = Exits {
            exits,
            over: self.over.swap(0, Ordering::Relaxed),
            longest: self.longest.swap(0, Ordering::Relaxed),
            operation: self.operation.load(Ordering::Relaxed) as u32,
            gave_up: self.gave_up.swap(0, Ordering::Relaxed),
        };
        (exits != 0).then_some(taken)
    }
}

impl Default for ExitWatch {
    fn default() -> Self {
        Self::new()
    }
}

/// The exits of one report period ([`ExitWatch::take`]).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Exits {
    pub exits: u64,
    pub over: u64,
    pub longest: u64,
    pub operation: u32,
    pub gave_up: u64,
}

impl Exits {
    /// Whether every exit stayed under [`EXIT_TARGET_MICROS`].
    pub fn within_target(&self) -> bool {
        self.over == 0
    }

    /// `GPU EXIT longest_us=…@op over=…/exits target_us=… check=ok|over
    /// gave_up=…`: `check=over` marks a period with an exit past the target.
    pub fn line(&self, tsc_hz: u64) -> impl fmt::Display + '_ {
        Line(move |formatter: &mut fmt::Formatter<'_>| {
            write!(
                formatter,
                "GPU EXIT longest_us={}@{:#x} over={}/{} target_us={EXIT_TARGET_MICROS} check={} \
                 gave_up={}",
                micros(self.longest, tsc_hz),
                self.operation,
                self.over,
                self.exits,
                if self.within_target() { "ok" } else { "over" },
                self.gave_up,
            )
        })
    }
}

#[cfg(test)]
mod tests;
