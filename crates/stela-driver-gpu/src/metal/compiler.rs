//! [`Compiler`] over the AIR compiler service, `compiler/metal_compiler.h`.
//!
//! The platform links the service with LLVM and Mesa and supplies its entry
//! points with the Vulkan driver's NIR options. The compiler keeps each
//! library it is given as the original MTLB bytes; reflecting or compiling a
//! function selects it by name and checks its AIR against the library's
//! digest first. A function's [`Interface`] is the service's reflection, and
//! its [`Provenance`] is that digest with the module's `llvm.ident`.
//!
//! The service refuses AIR it does not lower with a typed kind and a detail:
//! the call fails with `Error::Unsupported` naming the kind, and
//! [`AirCompiler::refusal`] keeps the detail for the frontend's `NSError`.
//! Kernel stage-in has no AIR form the frontend lowers, so it is refused
//! before it runs. A post-tessellation vertex function compiles with its
//! pipeline's tessellation and stage-in layout into the evaluation stage,
//! and the service synthesizes the stages ahead of it.
//!
//! Once the platform keeps compilations ([`AirCompiler::keep_compilations`]),
//! reflections and compiled NIR are kept for the device's boot by what they
//! read, so a function or pipeline made again skips the service; a miss may
//! defer its compilation to run outside the service's lock
//! ([`AirCompiler::set_deferring`], `service/shared.rs`).
//!
//! A stage's [`Linking`] reaches the service as the AIR modules of its
//! functions, with their function constant values, and of every function
//! of its dynamic libraries, without, each module once and none that is the
//! function's own, with the names of the functions visible function tables
//! call.
mod bindings;
#[cfg(test)]
mod cache_tests;
pub mod ffi;
mod library_cache;
#[cfg(feature = "metal-linked")]
mod linked;
mod nir_cache;
#[cfg(test)]
mod tests;

use super::{
    abi::{DataType, FunctionType, IndexType, TessellationControlPointIndexType, TextureType},
    capabilities::ShaderFeatures,
    deferred::{
        Done, Job, Made, Owner,
        kept::{Kept, Taken},
    },
    error::{Error, Result, ensure},
    library::Air,
    reflection::Binding,
    shader::{
        Access, AttributeUse, BufferUse, ColorOutput, CompiledShader, Compiler, DeclaredAttribute,
        FunctionConstant, Interface, LinkedFunction, Linking, MAX_PATCH_CONTROL_POINTS,
        NumericClass, Patch, PatchDomain, PreparedFunction, Provenance, Reflection, Refusal,
        Scalar, Specialization, Stage, Tessellation, TessellationStages, TextureUse,
        ThreadgroupUse, Varying,
    },
    stitching::{StitchedGraph, StitchedNode},
    vertex::{Attribute, STRIDE_DYNAMIC, StageInputDescriptor, VertexDescriptor, VertexFormat},
    vulkan::{object::ShaderCode, physical::PhysicalDevice},
};
use alloc::{
    borrow::Cow,
    boxed::Box,
    collections::BTreeMap,
    ffi::CString,
    string::{String, ToString},
    vec::Vec,
};
use core::{
    ffi::{c_char, c_void},
    ptr::{self, NonNull},
    slice, str,
};
use triomphe::Arc;

use library_cache::{LoadedLibrary, ModuleSource, Selected};
pub use nir_cache::{CompileGuard, CompileLock};
use nir_cache::{NirCache, ReflectionCache, ReflectionKey, Returned};

/// The shader stages the AIR frontend lowers completely; other constructs
/// are refused with a typed diagnostic.
const FEATURES: ShaderFeatures = ShaderFeatures::COMPUTE
    .union(ShaderFeatures::VERTEX_FRAGMENT)
    .union(ShaderFeatures::MESH)
    .union(ShaderFeatures::TEXTURE_SAMPLE)
    .union(ShaderFeatures::TEXTURE_READ_WRITE)
    .union(ShaderFeatures::TEXTURE_ATOMICS)
    .union(ShaderFeatures::DEPTH_COMPARE_SAMPLE)
    .union(ShaderFeatures::QUERY_LOD)
    .union(ShaderFeatures::BUFFER_ATOMICS)
    .union(ShaderFeatures::THREADGROUP_MEMORY)
    .union(ShaderFeatures::SIMDGROUP)
    .union(ShaderFeatures::QUADGROUP)
    .union(ShaderFeatures::ARGUMENT_BUFFERS)
    .union(ShaderFeatures::INDIRECT_COMMAND_ENCODING)
    .union(ShaderFeatures::LAYERED_RENDERING)
    .union(ShaderFeatures::VERTEX_AMPLIFICATION)
    .union(ShaderFeatures::TESSELLATION)
    .union(ShaderFeatures::BARYCENTRICS)
    .union(ShaderFeatures::PULL_MODEL_INTERPOLATION)
    .union(ShaderFeatures::SPARSE_FEEDBACK)
    .union(ShaderFeatures::FUNCTION_POINTERS)
    .union(ShaderFeatures::DYNAMIC_LIBRARIES)
    .union(ShaderFeatures::RASTER_ORDER_GROUPS)
    .union(ShaderFeatures::IMAGEBLOCKS)
    .union(ShaderFeatures::SIMDGROUP_MATRIX);

/// LLVM instructions one compilation may lower, legalization included. Far
/// beyond the functions Metal compiles, it bounds the work of hostile input.
const MAX_INSTRUCTIONS: u32 = 1 << 20;
/// NVIDIA's local memory per thread, where thread memory lowers to.
const MAX_PRIVATE_BYTES: u32 = 512 << 10;

/// The reflection does not describe a function the frontend lowers.
const MALFORMED: Error = Error::Invalid("the compiler's reflection");

/// The service's entry points and the NIR options it compiles with.
#[derive(Clone, Copy, Debug)]
pub struct Service {
    /// `stela_metal_reflect`.
    pub reflect: unsafe extern "C" fn(
        *const c_void,
        usize,
        *const c_char,
        *const ffi::ConstantValue,
        u32,
        *mut ffi::Reflection,
        *mut ffi::AirError,
    ) -> bool,
    /// `stela_metal_reflect_bindings`.
    pub reflect_bindings: unsafe extern "C" fn(
        *const c_void,
        usize,
        *const c_char,
        *const ffi::ConstantValue,
        u32,
        *mut ffi::Bindings,
        *mut ffi::AirError,
    ) -> bool,
    /// `stela_metal_sampler_states`.
    pub sampler_states:
        unsafe extern "C" fn(*const c_void, usize, *mut u64, *mut u32, *mut ffi::AirError) -> bool,
    /// `stela_metal_functions`.
    pub functions: unsafe extern "C" fn(
        *const c_void,
        usize,
        ffi::VisitFunction,
        *mut c_void,
        *mut ffi::AirError,
    ) -> bool,
    /// `stela_metal_exports`.
    pub exports: unsafe extern "C" fn(
        *const c_void,
        usize,
        ffi::VisitFunction,
        *mut c_void,
        *mut ffi::AirError,
    ) -> bool,
    /// `stela_metal_stitch`.
    pub stitch: unsafe extern "C" fn(
        *const ffi::StitchSource,
        u32,
        *const ffi::StitchGraph,
        u32,
        ffi::VisitBytes,
        *mut c_void,
        *mut ffi::AirError,
    ) -> bool,
    /// `stela_metal_compile_reflected`. Custom callbacks receive the full
    /// record. Stela's linked callbacks also accept a compilation projection.
    pub compile: unsafe extern "C" fn(
        *const c_void,
        usize,
        *const c_char,
        *const ffi::Target,
        *const ffi::Specialization,
        *const ffi::Reflection,
        *mut ffi::AirError,
    ) -> *mut c_void,
    /// `stela_metal_tessellation_stages`.
    pub tessellation_stages:
        unsafe extern "C" fn(*const ffi::Target, *const ffi::Tessellation, *mut *mut c_void),
    /// `stela_metal_accumulate_kernel`.
    pub accumulate_kernel: KernelBuilder,
    /// `stela_metal_select_kernel`.
    pub select_kernel: KernelBuilder,
    /// `stela_metal_resolve_draws_kernel`.
    pub resolve_draws_kernel: KernelBuilder,
    /// `stela_metal_resolve_dispatches_kernel`.
    pub resolve_dispatches_kernel: KernelBuilder,
    /// `stela_metal_store_texels_kernel`.
    pub store_texels_kernel: KernelBuilder,
    /// `stela_metal_load_texels_kernel`.
    pub load_texels_kernel: KernelBuilder,
    /// `stela_metal_average_samples_kernel`.
    pub average_samples_kernel: KernelBuilder,
    /// `stela_metal_decode_etc_kernel`.
    pub decode_etc_kernel: KernelBuilder,
    /// `stela_metal_decode_pvrtc_kernel`.
    pub decode_pvrtc_kernel: KernelBuilder,
    /// `stela_metal_reorder_blocks_kernel`.
    pub reorder_blocks_kernel: KernelBuilder,
    /// `stela_metal_decode_astc_kernel`.
    pub decode_astc_kernel: KernelBuilder,
    /// `stela_metal_release`.
    pub release: unsafe extern "C" fn(*mut c_void),
    /// `stela_metal_buffer_reads`.
    pub buffer_reads: unsafe extern "C" fn(*const c_void) -> u32,
    /// The Vulkan driver's `nir_shader_compiler_options`.
    pub nir_options: NonNull<c_void>,
}

/// One of the device's internal kernels' builders (`metal_abi.h`).
pub type KernelBuilder = unsafe extern "C" fn(*const c_void) -> *mut c_void;

impl Service {
    /// Builds the device's internal kernels.
    ///
    /// # Safety
    ///
    /// As [`AirCompiler::new`]: the entry points are the service's and the
    /// options the driver's.
    pub unsafe fn internal_kernels(&self) -> Result<InternalKernelCode> {
        let mut kernels = InternalKernelCode {
            release: self.release,
            code: [ShaderCode(0); 11],
        };
        let builders = [
            self.accumulate_kernel,
            self.select_kernel,
            self.resolve_draws_kernel,
            self.resolve_dispatches_kernel,
            self.store_texels_kernel,
            self.load_texels_kernel,
            self.average_samples_kernel,
            self.decode_etc_kernel,
            self.decode_pvrtc_kernel,
            self.reorder_blocks_kernel,
            self.decode_astc_kernel,
        ];
        for (code, build) in kernels.code.iter_mut().zip(builders) {
            // SAFETY: the caller guarantees the builder and the options it
            // reads.
            let shader = unsafe { build(self.nir_options.as_ptr()) };
            if shader.is_null() {
                return Err(Error::OutOfMemory);
            }
            *code = ShaderCode(shader as usize as u64);
        }
        Ok(kernels)
    }
}

/// The internal kernels' NIR, released when dropped: a pipeline keeps its
/// own copy of its shader.
pub struct InternalKernelCode {
    release: unsafe extern "C" fn(*mut c_void),
    code: [ShaderCode; 11],
}

impl InternalKernelCode {
    /// The kernels in [`InternalKernels`](super::native::InternalKernels)'
    /// order.
    pub fn code(&self) -> [ShaderCode; 11] {
        self.code
    }
}

impl Drop for InternalKernelCode {
    fn drop(&mut self) {
        for code in self.code.into_iter().filter(|code| code.0 != 0) {
            // SAFETY: the service built the shader, which is released once.
            unsafe { release_nir(self.release, code) }
        }
    }
}

/// How many NIR shaders the compilers released: while it stands, a
/// [`ShaderCode`] still names the shader it named, since only a release
/// frees an address another shader may take ([`nir_releases`]).
static NIR_RELEASES: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// The count of NIR releases, which a holder of a [`ShaderCode`] compares
/// later to know that the code still names the same shader.
pub fn nir_releases() -> u64 {
    NIR_RELEASES.load(core::sync::atomic::Ordering::Acquire)
}

/// Releases `code` through the service's `release`, counting it first.
///
/// # Safety
/// `code` is NIR the service built, released once, which nothing uses.
unsafe fn release_nir(release: unsafe extern "C" fn(*mut c_void), code: ShaderCode) {
    NIR_RELEASES.fetch_add(1, core::sync::atomic::Ordering::AcqRel);
    // SAFETY: the caller's.
    unsafe { release(code.0 as usize as *mut c_void) }
}

/// Loaded libraries by id: ids that share bytes hold one copy.
type Libraries = BTreeMap<u64, Arc<LoadedLibrary>>;

pub struct AirCompiler {
    service: Service,
    target: ffi::Target,
    project_reflection: bool,
    reuse_sampler_states: bool,
    /// MTLB libraries by id; ids are never reused.
    libraries: Libraries,
    next_library: u64,
    refusal: Option<Refusal>,
    /// Reflections and compiled NIR, once the platform keeps them
    /// ([`AirCompiler::keep_compilations`]).
    reflections: Option<ReflectionCache>,
    nir: Option<NirCache>,
    /// A compilation the cache misses defers rather than runs
    /// ([`AirCompiler::set_deferring`]), and the one deferred.
    deferring: bool,
    deferred: Option<CompileJob>,
    /// A reflection the cache missed, deferred as a compilation is.
    reflecting: Option<ReflectJob>,
    /// A stitched library deferred as a compilation is, what such jobs
    /// made for their calls' next runs, and the functions of the AIR the
    /// last one taken stitched, for its library's listing.
    stitching: Option<StitchJob>,
    stitched: Kept<Option<StitchedAir>>,
    listed: Option<(Vec<u8>, Listing)>,
    /// A call that would wait in its turn for a job's use of LLVM waits
    /// for this job, which runs after it, instead.
    waiting: Waiting,
    /// Keys whose compilation outside the lock failed: they compile in the
    /// call, whose failure is then the call's.
    failed: alloc::collections::BTreeSet<[u8; 32]>,
    lock: alloc::sync::Arc<CompileLock>,
    /// What a holder of the compile lock runs while another holds it.
    relax: fn(),
    /// Reflections and compilations the service ran.
    reflected: u64,
    compiled: u64,
    deferrals: u64,
    /// NIR an earlier boot compiled, serialized, by key, and where this
    /// boot's compilations go for the next ([`AirCompiler::persist`]).
    stored: BTreeMap<[u8; 32], &'static [u8]>,
    persist: Option<Persist>,
    /// Where this boot's serialized compilations are found again: an
    /// evicted compilation deserializes instead of lowering AIR again.
    recall: Option<Recall>,
    /// Stored NIR deserialized in place of a compilation, and this boot's
    /// recalled.
    restored: u64,
    recalled: u64,
}

/// Where a compilation's serialized NIR goes for the next boot, by key.
pub type Persist = fn(&[u8; 32], &[u8]);

/// The serialized NIR of a key that [`Persist`] took this boot, if kept.
pub type Recall = fn(&[u8; 32]) -> Option<Vec<u8>>;

/// Stitched libraries jobs made that wait for their calls' next runs; past
/// this the oldest is dropped, and its call stitches again.
const STITCHED: usize = 16;

/// The NIR budget of a device's compiled shaders.
pub const NIR_BUDGET: usize = 96 << 20;

// SAFETY: the target's NIR options are the driver's, read from any thread.
unsafe impl Send for AirCompiler {}

impl Drop for AirCompiler {
    /// Releases the NIR the cache holds and no one borrows, as the compiler
    /// of a device lost and replaced goes.
    fn drop(&mut self) {
        let codes = self.nir.as_mut().map(NirCache::take_unlent);
        for code in codes.into_iter().flatten() {
            // SAFETY: a cached shader no one holds; the service outlives
            // the compiler.
            unsafe { release_nir(self.service.release, code) };
        }
    }
}

impl AirCompiler {
    /// A compiler for the Vulkan device `device` describes.
    ///
    /// # Safety
    ///
    /// `service` holds the compiler service's entry points and the NIR
    /// options of that device's driver, which outlive the compiler and every
    /// shader it compiles.
    pub unsafe fn new(service: Service, device: &PhysicalDevice) -> Self {
        let limits = &device.limits;
        let mesh = limits.mesh;
        let varying_components = limits
            .max_vertex_output_components
            .min(limits.max_fragment_input_components);
        let target = ffi::Target {
            nir: service.nir_options.as_ptr(),
            max_instructions: MAX_INSTRUCTIONS,
            // NAK sizes shared memory in 16 bits.
            max_threadgroup_bytes: limits.max_compute_shared_memory_size.min(u16::MAX.into()),
            max_private_bytes: MAX_PRIVATE_BYTES,
            // The driver reads constant data through a uniform buffer.
            max_constant_bytes: limits.max_uniform_buffer_range,
            max_varying_slots: (varying_components / 4).min(ffi::VARYINGS as u32),
            max_payload_bytes: mesh.map_or(0, |mesh| mesh.max_task_payload_size),
            max_mesh_vertices: mesh.map_or(0, |mesh| mesh.max_mesh_output_vertices),
            max_mesh_primitives: mesh.map_or(0, |mesh| mesh.max_mesh_output_primitives),
            stencil_export: limits.stencil_export.into(),
            raster_order_groups: device.features.fragment_shader_pixel_interlock.into(),
            fragment_tests: super::capabilities::fragment_tests(device).into(),
        };
        // The boot image mounts the linked AIR compiler. Hosted all-feature
        // builds also expose `Service::linked`, but only pull in its external
        // symbols if the caller constructs that service.
        #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
        let project_reflection = service.accepts_compilation_projection();
        #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
        let project_reflection = false;
        #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
        let reuse_sampler_states = service.has_linked_sampler_states();
        #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
        let reuse_sampler_states = false;
        Self {
            service,
            target,
            project_reflection,
            reuse_sampler_states,
            libraries: BTreeMap::new(),
            next_library: 0,
            refusal: None,
            reflections: None,
            nir: None,
            deferring: false,
            deferred: None,
            reflecting: None,
            stitching: None,
            stitched: Kept::new(STITCHED),
            listed: None,
            waiting: Waiting::default(),
            failed: alloc::collections::BTreeSet::new(),
            lock: alloc::sync::Arc::new(CompileLock::new()),
            relax: core::hint::spin_loop,
            reflected: 0,
            compiled: 0,
            deferrals: 0,
            stored: BTreeMap::new(),
            persist: None,
            recall: None,
            restored: 0,
            recalled: 0,
        }
    }

    /// Takes `stored`, NIR an earlier boot's compilations serialized by key,
    /// and gives this boot's compilations to `persist`, which `recall` finds
    /// again: a compilation either holds deserializes instead of lowering
    /// AIR again, so one the NIR cache evicted is not compiled twice.
    pub fn persist(
        &mut self,
        stored: impl IntoIterator<Item = ([u8; 32], &'static [u8])>,
        persist: Persist,
        recall: Recall,
    ) {
        if self.nir.is_none() || !nir_serialization() {
            return;
        }
        self.stored.extend(stored);
        self.persist = Some(persist);
        self.recall = Some(recall);
    }

    /// The serialized NIR of `key`: an earlier boot's, else this boot's.
    fn serialized(&self, key: &[u8; 32]) -> Option<(Cow<'static, [u8]>, bool)> {
        if let Some(&blob) = self.stored.get(key) {
            return Some((Cow::Borrowed(blob), false));
        }
        let blob = (self.recall?)(key)?;
        Some((Cow::Owned(blob), true))
    }

    /// The serialized NIR of `job`, deserialized, as its compilation would
    /// make.
    fn restore(&mut self, job: &CompileJob) -> Option<(CompiledShader, usize)> {
        let (blob, recalled) = self.serialized(&job.key?)?;
        let shader = deserialize(&blob, self.target.nir)?;
        if recalled {
            self.recalled += 1;
        } else {
            self.restored += 1;
        }
        Some(job.finish(shader))
    }

    /// Uses `lock`, the compile lock of the compiler this one replaces, as a
    /// device reopened after a loss does: jobs the platform runs hold it.
    pub fn share_compile_lock(&mut self, lock: alloc::sync::Arc<CompileLock>, relax: fn()) {
        self.lock = lock;
        self.relax = relax;
    }

    /// Gives a compilation's serialized NIR to the next boot, unless it
    /// came from there.
    fn keep_serialized(&self, key: &[u8; 32], blob: Option<&[u8]>) {
        if let (Some(persist), Some(blob)) = (self.persist, blob)
            && !self.stored.contains_key(key)
        {
            persist(key, blob);
        }
    }

    /// Keeps reflections and up to `nir_budget` bytes of compiled NIR for
    /// the device's boot, keyed by what they read, so a function or pipeline
    /// made again, by this process or another, skips the service.
    pub fn keep_compilations(&mut self, nir_budget: usize) {
        self.reflections
            .get_or_insert_with(ReflectionCache::default);
        self.nir.get_or_insert_with(|| NirCache::new(nir_budget));
    }

    /// The lock every AIR compilation holds, and what a waiter for it runs.
    pub fn compile_lock(&mut self, relax: fn()) -> alloc::sync::Arc<CompileLock> {
        self.relax = relax;
        self.lock.clone()
    }

    /// Whether a compilation the NIR cache misses defers: the first is kept
    /// for [`Self::take_deferred`] and the call fails, to run again once it
    /// is compiled outside the service's lock ([`Self::adopt`]).
    pub fn set_deferring(&mut self, deferring: bool) {
        self.deferring = deferring && self.nir.is_some();
        self.lock.set_deferred(false);
    }

    /// Whether the call may still leave a job: it defers, and has left
    /// none yet.
    fn may_wait(&self) -> bool {
        self.deferring
            && self.deferred.is_none()
            && self.reflecting.is_none()
            && self.stitching.is_none()
    }

    /// The compilation the call deferred.
    pub fn take_deferred(&mut self) -> Option<CompileJob> {
        self.lock.set_deferred(false);
        self.deferred.take()
    }

    /// Keeps what a deferred compilation made, for the call's next run.
    pub fn adopt(&mut self, finished: Finished) {
        self.compiled += 1;
        let Finished {
            key,
            result,
            serialized,
        } = finished;
        self.keep_serialized(&key, serialized.as_deref());
        let Some(nir) = self.nir.as_mut() else {
            if let Ok((shader, _)) = result {
                // SAFETY: the job's NIR, which nothing else holds.
                unsafe { release_nir(self.service.release, shader.code) };
            }
            return;
        };
        match result {
            Ok((shader, bytes)) => {
                let code = shader.code;
                if nir.insert(key, shader, bytes, false) == Returned::Release {
                    // SAFETY: as above.
                    unsafe { release_nir(self.service.release, code) };
                }
            }
            Err(()) => {
                self.failed.insert(key);
            }
        }
        self.release_doomed();
    }

    /// Holds the compile lock for a use of LLVM in a call's turn: LLVM is
    /// built without threads, and a job may be using it. A call that may
    /// defer does not wait for the job: it fails with `DEFERRED` and leaves
    /// a job that does nothing, which runs once the lock is free.
    fn llvm<'a>(
        lock: &'a CompileLock,
        waiting: &mut Waiting,
        may_defer: bool,
        relax: fn(),
    ) -> Result<CompileGuard<'a>> {
        if let Some(held) = lock.try_hold() {
            return Ok(held);
        }
        if may_defer {
            // A retry in the same turn must not wait for LLVM while holding
            // the service lock. Keep the existing wake-up job and its key.
            if waiting.job.is_none() {
                waiting.count += 1;
                let mut key = [0xff; 32];
                key[..8].copy_from_slice(&waiting.count.to_le_bytes());
                waiting.job = Some(Job::new(key, Owner::Compiler, || Box::new(()) as Made));
            }
            lock.set_deferred(true);
            return Err(DEFERRED);
        }
        Ok(lock.hold(relax))
    }

    /// Keeps what a deferred reflection made, for the call's next run.
    fn adopt_reflection(&mut self, reflected: Reflected) {
        let Reflected {
            digest,
            key,
            result,
        } = reflected;
        match (result, self.reflections.as_mut()) {
            (Some((reflection, provenance)), Some(cache)) => {
                self.reflected += 1;
                cache.insert(key, reflection, provenance);
            }
            // Its call reflects in its turn, whose failure is then its own.
            _ => {
                self.failed.insert(digest);
            }
        }
    }

    /// Ends a call: what it lent or adopted may be evicted again.
    pub fn end_call(&mut self) {
        self.deferring = false;
        self.reflecting = None;
        self.stitching = None;
        self.listed = None;
        self.waiting.job = None;
        if let Some(nir) = self.nir.as_mut() {
            nir.unpin();
        }
        self.release_doomed();
    }

    fn release_doomed(&mut self) {
        let doomed = self.nir.as_mut().map(NirCache::take_doomed);
        for code in doomed.into_iter().flatten() {
            // SAFETY: an evicted shader no one holds.
            unsafe { release_nir(self.service.release, code) };
        }
    }

    /// `GPU COMPILER reflected=… reflections=… nir_hits=… nir_misses=…
    /// compiled=… restored=…/stored deferred=… nir=entries/MiB evicted=…
    /// recalled=…`, since the device opened: reflections and compilations
    /// the service ran, cache hits and misses, stored NIR deserialized of
    /// what the store held, calls that compiled outside the lock, the cache,
    /// and evicted compilations of this boot deserialized again.
    pub fn report(&self) -> String {
        let (hits, misses, entries, bytes, evicted) =
            self.nir.as_ref().map_or((0, 0, 0, 0, 0), |nir| {
                (
                    nir.hits,
                    nir.misses,
                    nir.entries(),
                    nir.bytes(),
                    nir.evictions,
                )
            });
        alloc::format!(
            "GPU COMPILER reflected={} reflections={} nir_hits={hits} nir_misses={misses} \
             compiled={} restored={}/{} deferred={} nir={entries}/{}M evicted={evicted} \
             recalled={}",
            self.reflected,
            self.reflections.as_ref().map_or(0, ReflectionCache::len),
            self.compiled,
            self.restored,
            self.stored.len(),
            self.deferrals,
            bytes >> 20,
            self.recalled,
        )
    }

    /// The reflection of `entry` of `library` with function constant
    /// `values`, or as declared, from the cache when it keeps one.
    fn reflect_cached(
        &mut self,
        library: u64,
        entry: &str,
        values: Option<&[(u32, Vec<u8>)]>,
    ) -> Result<(Reflection, Provenance)> {
        let selected = select(&self.libraries, library, entry)?;
        let key = self.reflections.as_ref().map(|_| {
            (
                selected.module.slice_hash,
                String::from(entry),
                values.map(<[_]>::to_vec),
            )
        });
        if let (Some(cache), Some(key)) = (&self.reflections, &key)
            && let Some(hit) = cache.get(key)
        {
            return Ok(hit);
        }
        // A large module's reflection, which parses and specializes its
        // AIR, can take seconds: it runs outside the turn like a compile.
        if let Some(key) = &key
            && self.may_wait()
        {
            let digest = reflection_digest(key);
            if !self.failed.remove(&digest) {
                let library = self
                    .libraries
                    .get(&library)
                    .ok_or(Error::Invalid("unknown library"))?
                    .clone();
                self.deferrals += 1;
                self.reflecting = Some(ReflectJob {
                    service: self.service,
                    library,
                    digest,
                    key: key.clone(),
                });
                self.lock.set_deferred(true);
                return Err(DEFERRED);
            }
        }
        let raw = values.map(constant_values);
        // LLVM, built without threads, runs on one processor at a time: a
        // job holds this lock while it runs.
        let reflection = {
            let lock = self.lock.clone();
            let may_wait = self.may_wait();
            let _held = Self::llvm(&lock, &mut self.waiting, may_wait, self.relax)?;
            let selected = select(&self.libraries, library, entry)?;
            selected.reflect_raw(&self.service, raw.as_deref(), &mut self.refusal)?
        };
        let selected = select(&self.libraries, library, entry)?;
        self.reflected += 1;
        let decoded = selected.decode_reflection(&reflection)?;
        if let (Some(cache), Some(key)) = (&mut self.reflections, key) {
            cache.insert(key, decoded.0.clone(), decoded.1.clone());
        }
        Ok(decoded)
    }

    /// The detail of the last call the service refused.
    pub fn refusal(&self) -> Option<&Refusal> {
        self.refusal.as_ref()
    }

    /// A new id for `data`.
    fn keep_library(&mut self, data: Arc<LoadedLibrary>) -> u64 {
        let id = self.next_library;
        self.next_library += 1;
        self.libraries.insert(id, data);
        id
    }
}

impl Compiler for AirCompiler {
    fn take_refusal(&mut self) -> Option<Refusal> {
        self.refusal.take()
    }

    fn stop_deferring(&mut self) {
        self.deferring = false;
    }

    fn deferred(&self) -> bool {
        self.deferred.is_some()
            || self.reflecting.is_some()
            || self.stitching.is_some()
            || self.waiting.job.is_some()
    }

    fn defer_compiles(&mut self, deferring: bool) {
        self.set_deferring(deferring);
    }

    fn take_job(&mut self) -> Option<Job> {
        if let Some(waiting) = self.waiting.job.take() {
            self.lock.set_deferred(false);
            return Some(waiting);
        }
        if let Some(reflect) = self.reflecting.take() {
            self.lock.set_deferred(false);
            return Some(Job::new(reflect.digest, Owner::Compiler, move || {
                Box::new(reflect.run()) as Made
            }));
        }
        if let Some(stitch) = self.stitching.take() {
            self.lock.set_deferred(false);
            return Some(Job::new(stitch.key, Owner::Compiler, move || {
                Box::new(stitch.run()) as Made
            }));
        }
        let job = self.take_deferred()?;
        let key = job.key.expect("a deferred compilation has its key");
        Some(Job::new(key, Owner::Compiler, move || {
            Box::new(job.complete()) as Made
        }))
    }

    fn adopt_job(&mut self, done: Done) {
        match done.made.downcast::<Finished>() {
            Ok(finished) => self.adopt(*finished),
            Err(made) => match made.downcast::<Reflected>() {
                Ok(reflected) => self.adopt_reflection(*reflected),
                Err(made) => {
                    if let Ok(stitched) = made.downcast::<Stitched>() {
                        let Stitched { key, made } = *stitched;
                        // One pushed out stitches again in its call.
                        drop(self.stitched.keep(key, 0, made));
                    }
                }
            },
        }
    }

    fn features(&self) -> ShaderFeatures {
        FEATURES
    }

    fn add_library(&mut self, data: Vec<u8>) -> Result<u64> {
        Ok(self.keep_library(Arc::new(LoadedLibrary::new(data)?)))
    }

    fn share_library(&mut self, library: u64) -> Result<u64> {
        let loaded = self
            .libraries
            .get(&library)
            .ok_or(Error::Invalid("unknown library"))?;
        Ok(self.keep_library(loaded.clone()))
    }

    fn functions(&mut self, air: &[u8]) -> Result<Vec<(String, FunctionType)>> {
        // A stitched library's job listed its functions with its AIR.
        if let Some((listed, functions)) = self.listed.take()
            && listed == air
        {
            return Ok(functions);
        }
        let mut error = ffi::AirError::NONE;
        // LLVM, built without threads, runs on one processor at a time.
        let lock = self.lock.clone();
        let may_wait = self.may_wait();
        let _held = Self::llvm(&lock, &mut self.waiting, may_wait, self.relax)?;
        list_functions(&self.service, air, &mut error)?
            .ok_or_else(|| refuse(&mut self.refusal, "the library", &error))
    }

    fn exports(&mut self, air: &[u8]) -> Result<Vec<String>> {
        unsafe extern "C" fn visit(
            data: *mut c_void,
            name: *const c_char,
            name_bytes: usize,
            _: u32,
        ) -> bool {
            // SAFETY: the data is the list below, and the service's name is
            // `name_bytes` long for the call.
            let (list, name) = unsafe {
                (
                    &mut *data.cast::<Vec<String>>(),
                    slice::from_raw_parts(name.cast::<u8>(), name_bytes),
                )
            };
            let Ok(name) = str::from_utf8(name) else {
                return false;
            };
            list.push(String::from(name));
            true
        }
        let air = Air::parse(air).map_err(|_| Error::Invalid("AIR"))?;
        let mut exports = Vec::new();
        let mut error = ffi::AirError::NONE;
        // LLVM, built without threads, runs on one processor at a time.
        let lock = self.lock.clone();
        let may_wait = self.may_wait();
        let _held = Self::llvm(&lock, &mut self.waiting, may_wait, self.relax)?;
        // SAFETY: the bitcode and the list outlive the call, which writes
        // the list through `visit` and the error.
        let listed = unsafe {
            (self.service.exports)(
                air.bitcode.as_ptr().cast(),
                air.bitcode.len(),
                visit,
                ptr::from_mut(&mut exports).cast(),
                &mut error,
            )
        };
        if !listed {
            return Err(refuse(&mut self.refusal, "the library", &error));
        }
        Ok(exports)
    }

    fn stitch(&mut self, sources: &[LinkedFunction], graphs: &[StitchedGraph]) -> Result<Vec<u8>> {
        let key = {
            let selected = sources
                .iter()
                .map(|function| select(&self.libraries, function.library, &function.entry))
                .collect::<Result<Vec<_>>>()?;
            stitch_digest(&selected, sources, graphs)
        };
        // What a job stitched for the call's last run; a refusal it met
        // the turn repeats, so the call reports it.
        let refused = match self.stitched.take(&key, 0) {
            Taken::Fresh(Some(StitchedAir { air, functions })) => {
                self.listed = Some((air.clone(), functions));
                return Ok(air);
            }
            Taken::Fresh(None) => true,
            // The key names its inputs' contents: no generation is older.
            Taken::Stale(_) | Taken::Missing => false,
        };
        // Stitching and listing its functions run LLVM for milliseconds:
        // outside the turn, like a compile.
        if !refused && self.may_wait() {
            let sources = sources
                .iter()
                .map(|function| {
                    let library = self
                        .libraries
                        .get(&function.library)
                        .ok_or(Error::Invalid("unknown library"))?;
                    Ok((
                        library.clone(),
                        function.entry.clone(),
                        function.constants.clone(),
                    ))
                })
                .collect::<Result<Vec<_>>>()?;
            self.deferrals += 1;
            self.stitching = Some(StitchJob {
                service: self.service,
                sources,
                graphs: graphs.to_vec(),
                key,
            });
            self.lock.set_deferred(true);
            return Err(DEFERRED);
        }
        let values: Vec<_> = sources
            .iter()
            .map(|function| constant_values(&function.constants))
            .collect();
        let mut error = ffi::AirError::NONE;
        // LLVM, built without threads, runs on one processor at a time.
        let lock = self.lock.clone();
        let may_wait = self.may_wait();
        let _held = Self::llvm(&lock, &mut self.waiting, may_wait, self.relax)?;
        let selected = sources
            .iter()
            .map(|function| select(&self.libraries, function.library, &function.entry))
            .collect::<Result<Vec<_>>>()?;
        stitch_raw(&self.service, &selected, &values, graphs, &mut error)?
            .ok_or_else(|| refuse(&mut self.refusal, "stitched functions", &error))
    }
    fn library_data(&self, library: u64) -> Result<&[u8]> {
        self.libraries
            .get(&library)
            .map(|loaded| loaded.bytes())
            .ok_or(Error::Invalid("unknown library"))
    }

    fn remove_library(&mut self, library: u64) -> Result<()> {
        self.libraries
            .remove(&library)
            .map(drop)
            .ok_or(Error::Invalid("unknown library"))
    }

    fn reflect(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
    ) -> Result<Reflection> {
        let (reflection, _) =
            self.reflect_cached(library, entry, Some(&specialization.constants))?;
        Ok(reflection)
    }

    fn prepare_function(
        &mut self,
        library: u64,
        entry: &str,
        constants: Option<Vec<(u32, Vec<u8>)>>,
    ) -> Result<PreparedFunction> {
        let (reflection, provenance) = self.reflect_cached(library, entry, constants.as_deref())?;
        let specialization = constants.map(|constants| Specialization {
            constants,
            ..Specialization::default()
        });
        Ok(PreparedFunction {
            library,
            specialized: specialization.is_some(),
            constants: specialization.map_or_else(Vec::new, |known| known.constants),
            reflection,
            provenance,
            reflection_service: self
                .project_reflection
                .then_some(self.service.reflect as usize)
                .and_then(core::num::NonZeroUsize::new),
        })
    }

    fn constants(&mut self, library: u64, entry: &str) -> Result<Vec<FunctionConstant>> {
        let (reflection, _) = self.reflect_cached(library, entry, None)?;
        Ok(reflection.constants().to_vec())
    }

    fn bindings(
        &mut self,
        library: u64,
        entry: &str,
        specialization: Option<&Specialization>,
    ) -> Result<Vec<Binding>> {
        let selected = select(&self.libraries, library, entry)?;
        let mut bindings = ffi::Bindings::zeroed();
        let mut error = ffi::AirError::NONE;
        let air = selected.module.bitcode;
        let values = specialization.map(|known| constant_values(&known.constants));
        let count = u32::try_from(values.as_ref().map_or(0, Vec::len)).map_err(|_| MALFORMED)?;
        // LLVM, built without threads, runs on one processor at a time.
        let lock = self.lock.clone();
        let may_wait = self.may_wait();
        let _held = Self::llvm(&lock, &mut self.waiting, may_wait, self.relax)?;
        // SAFETY: the AIR, entry and values outlive the call, which writes
        // only the bindings and the error.
        let reflected = unsafe {
            (self.service.reflect_bindings)(
                air.as_ptr().cast(),
                air.len(),
                selected.entry.as_ptr(),
                values
                    .as_ref()
                    .map_or(core::ptr::null(), |values| values.as_ptr()),
                count,
                &mut *bindings,
                &mut error,
            )
        };
        if !reflected {
            return Err(refuse(&mut self.refusal, entry, &error));
        }
        bindings::bindings(&bindings)
    }

    fn sampler_states(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
    ) -> Result<Vec<u64>> {
        let selected = select(&self.libraries, library, entry)?;
        let own = (
            selected.module.slice_hash,
            specialization.constants.as_slice(),
        );
        let (linked, _) = linked_modules(&self.libraries, &specialization.linking, own)?;
        let mut states = Vec::new();
        for module in
            core::iter::once(selected.module).chain(linked.iter().map(|module| module.source))
        {
            if self.reuse_sampler_states
                && let Some(cached) = module.samplers.get()
            {
                states.extend_from_slice(cached);
                continue;
            }
            let mut listed = [0; ffi::SAMPLER_STATES];
            let mut count = 0;
            let mut error = ffi::AirError::NONE;
            // LLVM, built without threads, runs on one processor at a time.
            let lock = self.lock.clone();
            let may_wait = self.may_wait();
            let _held = Self::llvm(&lock, &mut self.waiting, may_wait, self.relax)?;
            // SAFETY: the bitcode outlives the call, which writes at most
            // SAMPLER_STATES states, the count and the error.
            let valid = unsafe {
                (self.service.sampler_states)(
                    module.bitcode.as_ptr().cast(),
                    module.bitcode.len(),
                    listed.as_mut_ptr(),
                    &mut count,
                    &mut error,
                )
            };
            if !valid {
                return Err(refuse(&mut self.refusal, entry, &error));
            }
            let listed = listed.get(..count as usize).ok_or(MALFORMED)?;
            if self.reuse_sampler_states {
                let _ = module.samplers.set(Box::from(listed));
            }
            states.extend_from_slice(listed);
        }
        states.sort_unstable();
        states.dedup();
        Ok(states)
    }

    fn compile(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
    ) -> Result<CompiledShader> {
        self.compile_entry(library, entry, specialization, None)
    }

    fn compile_tessellation(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
        tessellation: &Tessellation,
    ) -> Result<TessellationStages> {
        self.compile_patch(library, entry, specialization, tessellation, None)
    }

    fn compile_prepared(
        &mut self,
        function: &PreparedFunction,
        specialization: &Specialization,
    ) -> Result<CompiledShader> {
        self.compile_entry(
            function.library,
            function.entry(),
            specialization,
            Some(function),
        )
    }

    fn compile_prepared_tessellation(
        &mut self,
        function: &PreparedFunction,
        specialization: &Specialization,
        tessellation: &Tessellation,
    ) -> Result<TessellationStages> {
        self.compile_patch(
            function.library,
            function.entry(),
            specialization,
            tessellation,
            Some(function),
        )
    }

    fn release(&mut self, code: ShaderCode) {
        // A shader the cache keeps, or still lends, stays.
        if self
            .nir
            .as_mut()
            .is_some_and(|nir| nir.release(code) == Returned::Kept)
        {
            return;
        }
        // SAFETY: the code is NIR this compiler's service compiled, which the
        // device releases once.
        unsafe { release_nir(self.service.release, code) }
    }
}

impl AirCompiler {
    fn compile_entry(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
        prepared: Option<&PreparedFunction>,
    ) -> Result<CompiledShader> {
        let stage_in = specialization
            .stage_in
            .as_ref()
            .map(ffi::StageIn::kernel)
            .transpose()?;
        self.compile_function(
            library,
            entry,
            specialization,
            None,
            stage_in.as_deref(),
            prepared,
        )
    }

    fn compile_patch(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
        tessellation: &Tessellation,
        prepared: Option<&PreparedFunction>,
    ) -> Result<TessellationStages> {
        if specialization.stage_in.is_some() {
            return Err(Error::Invalid("kernel stage-in of a vertex function"));
        }
        let properties = ffi::Tessellation::new(tessellation);
        let stage_in = tessellation
            .vertex_descriptor
            .as_ref()
            .map(ffi::StageIn::vertex)
            .transpose()?;
        let evaluation = self.compile_function(
            library,
            entry,
            specialization,
            Some(&properties),
            stage_in.as_deref(),
            prepared,
        )?;
        let mut stages = [ptr::null_mut(); 2];
        // SAFETY: the target and tessellation outlive the call, which writes
        // only the two stages.
        unsafe {
            (self.service.tessellation_stages)(&self.target, &properties, stages.as_mut_ptr());
        }
        let [vertex, control] = stages.map(|stage| ShaderCode(stage as usize as u64));
        Ok(TessellationStages {
            vertex,
            control,
            evaluation,
        })
    }

    /// Compiles `entry` of `library`: a post-tessellation vertex function
    /// with its pipeline's tessellation, and a kernel or post-tessellation
    /// vertex function with the stage-in layout it fetches by. The NIR
    /// cache answers a compilation it holds; a miss runs the service, or
    /// defers while the service runs compilations outside its lock.
    fn compile_function(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
        tessellation: Option<&ffi::Tessellation>,
        stage_in: Option<&ffi::StageIn>,
        prepared: Option<&PreparedFunction>,
    ) -> Result<CompiledShader> {
        let job = self.plan(
            library,
            entry,
            specialization,
            tessellation,
            stage_in,
            prepared,
        )?;
        if let (Some(nir), Some(key)) = (self.nir.as_mut(), job.key)
            && let Some(shader) = nir.lend(&key)
        {
            return Ok(shader);
        }
        if let Some((shader, bytes)) = self.restore(&job) {
            let key = job.key.expect("restored by its key");
            let nir = self.nir.as_mut().expect("restoring keeps compilations");
            nir.insert(key, shader.clone(), bytes, true);
            self.release_doomed();
            return Ok(shader);
        }
        if let Some(key) = job.key
            && self.deferring
            && self.deferred.is_none()
            && !self.failed.contains(&key)
        {
            self.deferrals += 1;
            self.deferred = Some(job);
            self.lock.set_deferred(true);
            return Err(DEFERRED);
        }
        if let Some(key) = job.key {
            self.failed.remove(&key);
        }
        let result = {
            let _held = self.lock.hold(self.relax);
            job.run()
        };
        self.compiled += 1;
        match result {
            Ok((shader, bytes)) => {
                if let Some(key) = job.key
                    && self.persist.is_some()
                {
                    self.keep_serialized(&key, serialize(shader.code).as_deref());
                }
                if let (Some(nir), Some(key)) = (self.nir.as_mut(), job.key) {
                    // Lent to the caller, whose release leaves it held.
                    if nir.insert(key, shader.clone(), bytes, true) == Returned::Kept {
                        self.release_doomed();
                    }
                }
                Ok(shader)
            }
            Err(error) => Err(refuse(&mut self.refusal, entry, &error)),
        }
    }

    /// What compiling `entry` of `library` reads, owned, so the service can
    /// run it with or without this compiler ([`CompileJob::run`]).
    fn plan(
        &mut self,
        library: u64,
        entry: &str,
        specialization: &Specialization,
        tessellation: Option<&ffi::Tessellation>,
        stage_in: Option<&ffi::StageIn>,
        prepared: Option<&PreparedFunction>,
    ) -> Result<CompileJob> {
        let selected = select(&self.libraries, library, entry)?;
        let constants = constant_values(&specialization.constants);
        // Reflection uses only the module, entry and function constant values.
        // Linkage, layouts and dispatch sizes enter later at compilation.
        let prepared = prepared.filter(|function| {
            self.project_reflection
                && function.library == library
                && function.entry() == entry
                && function.provenance.slice_hash == selected.module.slice_hash
                && function.constants == specialization.constants
                && function
                    .reflection_service
                    .is_some_and(|service| service.get() == self.service.reflect as usize)
        });
        let (reflected, interface, provenance) = if let Some(function) = prepared {
            let Reflection::Stage(interface) = &function.reflection else {
                return Err(Error::Invalid("a visible function is not a pipeline stage"));
            };
            (
                compilation_reflection(interface)?,
                interface.as_ref().clone(),
                function.provenance.clone(),
            )
        } else {
            let reflected =
                selected.reflect_raw(&self.service, Some(&constants), &mut self.refusal)?;
            let (reflection, provenance) = selected.decode_reflection(&reflected)?;
            let Reflection::Stage(interface) = reflection else {
                return Err(Error::Invalid("a visible function is not a pipeline stage"));
            };
            (reflected, *interface, provenance)
        };
        let linking = &specialization.linking;
        let own = (
            selected.module.slice_hash,
            specialization.constants.as_slice(),
        );
        let (linked, copies) = linked_modules(&self.libraries, linking, own)?;
        let modules: Vec<JobModule> = linked
            .iter()
            .map(|module| {
                let constants = module.constants.map(<[_]>::to_vec);
                let count = constants.as_ref().map_or(Ok(0), |values| {
                    u32::try_from(values.len()).map_err(|_| MALFORMED)
                })?;
                let aliases = module
                    .aliases
                    .iter()
                    .map(|&(definition, name)| {
                        Ok((
                            CString::new(definition).map_err(|_| MALFORMED)?,
                            CString::new(name).map_err(|_| MALFORMED)?,
                        ))
                    })
                    .collect::<Result<Vec<_>>>()?;
                Ok(JobModule {
                    bitcode: module.source.bitcode.as_ptr(),
                    size: module.source.bitcode.len(),
                    digest: module.source.slice_hash,
                    constants,
                    count,
                    copy: module.copy,
                    aliases,
                })
            })
            .collect::<Result<_>>()?;
        // Functions of another copy of their module carry its suffix.
        let names: Vec<String> = linking
            .functions
            .iter()
            .zip(&copies)
            .map(|(function, &copy)| alloc::format!("{}{}", function.entry, copy_suffix(copy)))
            .collect();
        let lengths = specialization
            .threadgroup_lengths
            .iter()
            .map(|&(index, bytes)| {
                let bytes = u32::try_from(bytes)
                    .map_err(|_| Error::Invalid("threadgroup memory length"))?;
                Ok(ffi::ThreadgroupLength { index, bytes })
            })
            .collect::<Result<Vec<_>>>()?;
        let samplers: Vec<_> = specialization
            .samplers
            .iter()
            .map(|&(state, id)| ffi::ConstexprSampler { state, id })
            .collect();
        let attachments = specialization.storage_attachments.map(|attachment| {
            attachment.map_or(ffi::Attachment::default(), |attachment| ffi::Attachment {
                format: attachment.format,
                bytes: attachment.bytes,
                blend: attachment.blend.into(),
                write_mask: attachment.write_mask,
                rgb_op: attachment.operations[0],
                alpha_op: attachment.operations[1],
                source_rgb: attachment.factors[0],
                destination_rgb: attachment.factors[1],
                source_alpha: attachment.factors[2],
                destination_alpha: attachment.factors[3],
                planes: attachment.planes,
            })
        });
        let count = |len: usize| u32::try_from(len).map_err(|_| MALFORMED);
        for len in [
            lengths.len(),
            specialization.varyings.len(),
            constants.len(),
            modules.len(),
            names.len(),
            samplers.len(),
            attachments.len(),
        ] {
            count(len)?;
        }
        // Everything the compilation reads, by content: the cache's key.
        let key = (self.nir.is_some() || self.deferring).then(|| {
            let mut normal = specialization.clone();
            normal.linking = Linking {
                max_call_stack_depth: linking.max_call_stack_depth,
                ..Linking::default()
            };
            let mut hasher = stela_crypto::Sha256::new();
            use stela_crypto::digest::Digest;
            hasher.update(selected.module.slice_hash);
            hasher.update(entry.as_bytes());
            hasher.update([0]);
            hasher.update(alloc::format!("{normal:?}").as_bytes());
            hasher.update(alloc::format!("{tessellation:?}{stage_in:?}").as_bytes());
            for module in &modules {
                hasher.update(module.digest);
                hasher.update(
                    alloc::format!(
                        "{:?}/{}/{:?}",
                        module.constants,
                        module.copy,
                        module.aliases
                    )
                    .as_bytes(),
                );
            }
            hasher.update(alloc::format!("{names:?}").as_bytes());
            let digest: [u8; 32] = hasher.finalize().into();
            digest
        });
        // The bitcode lives in the libraries the job holds.
        let mut held: Vec<Arc<LoadedLibrary>> = Vec::new();
        let ids = core::iter::once(library)
            .chain(
                linking
                    .functions
                    .iter()
                    .chain(&linking.private_functions)
                    .map(|function| function.library),
            )
            .chain(linking.libraries.iter().copied());
        for id in ids {
            if let Some(loaded) = self.libraries.get(&id)
                && !held.iter().any(|known| Arc::ptr_eq(known, loaded))
            {
                held.push(loaded.clone());
            }
        }
        Ok(CompileJob {
            service: self.service,
            target: self.target,
            _held: held,
            air: selected.module.bitcode.as_ptr(),
            air_size: selected.module.bitcode.len(),
            entry: CString::from(selected.entry),
            name: String::from(entry),
            specialization: specialization.clone(),
            tessellation: tessellation.copied(),
            stage_in: stage_in.map(|stage_in| Box::new(*stage_in)),
            modules,
            names,
            lengths,
            samplers,
            attachments,
            reflected,
            interface,
            provenance,
            key,
            serializing: self.persist.is_some(),
        })
    }
}

/// What a compilation the cache missed fails with while it defers: the call
/// runs again once the compilation is done outside the service's lock.
pub const DEFERRED: Error = Error::Unsupported("a compilation deferred outside the service's lock");

/// A linked module of a [`CompileJob`].
struct JobModule {
    bitcode: *const u8,
    size: usize,
    digest: [u8; 32],
    constants: Option<Vec<(u32, Vec<u8>)>>,
    count: u32,
    copy: u32,
    /// `(definition, name)` pairs (`struct stela_air_alias`).
    aliases: Vec<(CString, CString)>,
}

/// A compilation's inputs, owned: it runs under the service's lock or, for a
/// deferred one, outside it on a stack of its own, while other calls are
/// served. It reads only the libraries' immutable bytes, which it holds, and
/// the driver's NIR options; the service's calls read their arguments only.
pub struct CompileJob {
    service: Service,
    target: ffi::Target,
    _held: Vec<Arc<LoadedLibrary>>,
    air: *const u8,
    air_size: usize,
    entry: CString,
    name: String,
    specialization: Specialization,
    tessellation: Option<ffi::Tessellation>,
    stage_in: Option<Box<ffi::StageIn>>,
    modules: Vec<JobModule>,
    names: Vec<String>,
    lengths: Vec<ffi::ThreadgroupLength>,
    samplers: Vec<ffi::ConstexprSampler>,
    attachments: [ffi::Attachment; crate::metal::pipeline::MAX_COLOR_ATTACHMENTS],
    reflected: Box<ffi::Reflection>,
    interface: Interface,
    provenance: Provenance,
    key: Option<[u8; 32]>,
    /// The next boot keeps its NIR.
    serializing: bool,
}

// SAFETY: its pointers are the held libraries' immutable bytes and the
// driver's NIR options, which any thread reads; nothing else holds them.
unsafe impl Send for CompileJob {}

/// A job that waits for the compile lock, for a call whose turn would.
#[derive(Default)]
struct Waiting {
    job: Option<Job>,
    count: u64,
}

/// A reflection the cache missed, owned, to run outside the service's lock:
/// the library's immutable bytes and the function's constant values.
struct ReflectJob {
    service: Service,
    library: Arc<LoadedLibrary>,
    digest: [u8; 32],
    key: ReflectionKey,
}

// SAFETY: as for a compile job.
unsafe impl Send for ReflectJob {}

/// What a [`ReflectJob`] made: the reflection, or `None` when the service
/// refused it.
struct Reflected {
    digest: [u8; 32],
    key: ReflectionKey,
    result: Option<(Reflection, Provenance)>,
}

// SAFETY: plain values the job made, which only the adopting turn reads.
unsafe impl Send for Reflected {}

impl ReflectJob {
    fn run(self) -> Reflected {
        let (_, entry, values) = &self.key;
        let result = self.library.select(entry).ok().and_then(|selected| {
            let raw = values.as_deref().map(constant_values);
            let mut refusal = None;
            let reflection = selected
                .reflect_raw(&self.service, raw.as_deref(), &mut refusal)
                .ok()?;
            selected.decode_reflection(&reflection).ok()
        });
        Reflected {
            digest: self.digest,
            key: self.key,
            result,
        }
    }
}

/// The digest a deferred reflection is known by among jobs.
fn reflection_digest(key: &ReflectionKey) -> [u8; 32] {
    use stela_crypto::{Sha256, digest::Digest};
    let (slice, entry, values) = key;
    let mut hasher = Sha256::new();
    hasher.update(b"reflection");
    hasher.update(slice);
    hasher.update((entry.len() as u64).to_le_bytes());
    hasher.update(entry.as_bytes());
    for (index, value) in values.iter().flatten() {
        hasher.update(index.to_le_bytes());
        hasher.update((value.len() as u64).to_le_bytes());
        hasher.update(value);
    }
    hasher.update([u8::from(values.is_some())]);
    hasher.finalize().into()
}

/// What a deferred [`CompileJob`] made, for [`AirCompiler::adopt`]: the
/// shader with its NIR's bytes, or a failure the call's next run repeats.
pub struct Finished {
    key: [u8; 32],
    result: core::result::Result<(CompiledShader, usize), ()>,
    /// Its NIR, serialized for the next boot, when that keeps it.
    serialized: Option<Vec<u8>>,
}

impl CompileJob {
    /// Compiles outside the compiler, under its held lock, and serializes
    /// the NIR for the next boot, when that keeps it. The caller holds the
    /// lock for as long as it runs on the one compile stack.
    pub fn run_outside(self, _held: &CompileGuard<'_>) -> Finished {
        self.complete()
    }

    /// [`Self::run_outside`], for a [`Job`] whose runner holds the lock.
    fn complete(self) -> Finished {
        let serializing = self.serializing;
        let result = self.run();
        let serialized = match &result {
            Ok((shader, _)) if serializing => serialize(shader.code),
            _ => None,
        };
        Finished {
            key: self.key.expect("a deferred compilation has its key"),
            result: result.map_err(|_| ()),
            serialized,
        }
    }

    /// The function's name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Runs the service: the shader and its NIR's bytes, or the refusal.
    fn run(&self) -> core::result::Result<(CompiledShader, usize), Box<ffi::AirError>> {
        let specialization = &self.specialization;
        let constants = constant_values(&specialization.constants);
        let values: Vec<_> = self
            .modules
            .iter()
            .map(|module| module.constants.as_deref().map(constant_values))
            .collect();
        let aliases: Vec<Vec<ffi::Alias>> = self
            .modules
            .iter()
            .map(|module| {
                module
                    .aliases
                    .iter()
                    .map(|(definition, name)| ffi::Alias {
                        definition: definition.as_ptr(),
                        name: name.as_ptr(),
                    })
                    .collect()
            })
            .collect();
        let modules: Vec<_> = self
            .modules
            .iter()
            .zip(&values)
            .zip(&aliases)
            .map(|((module, values), aliases)| ffi::Module {
                bitcode: module.bitcode.cast(),
                size: module.size,
                constants: values
                    .as_ref()
                    .map_or(ptr::null(), |values| values.as_ptr()),
                constant_count: module.count,
                copy: module.copy,
                aliases: aliases.as_ptr(),
                alias_count: aliases.len() as u32,
            })
            .collect();
        let functions: Vec<_> = self
            .names
            .iter()
            .map(|name| ffi::LinkedFunction {
                name: name.as_ptr().cast(),
                name_bytes: name.len(),
            })
            .collect();
        let varyings: Vec<_> = specialization
            .varyings
            .iter()
            .map(|varying| ffi::LinkedVarying {
                identifier: varying.identifier.as_ptr().cast(),
                identifier_bytes: varying.identifier.len(),
                per_primitive: varying.per_primitive.into(),
            })
            .collect();
        let storage = specialization
            .storage_attachments
            .iter()
            .any(Option::is_some);
        // Each length was checked when planned.
        let count = |len: usize| len as u32;
        let raw = ffi::Specialization {
            workgroup_size: specialization.workgroup_size.unwrap_or_default(),
            threadgroup_length_count: count(self.lengths.len()),
            threadgroup_lengths: self.lengths.as_ptr(),
            varyings: varyings.as_ptr(),
            varying_count: count(varyings.len()),
            amplified: specialization.amplified.into(),
            launched_by_grid: specialization.launched_by_grid.into(),
            dynamic_threadgroups: specialization.dynamic_threadgroups.into(),
            constant_count: count(constants.len()),
            constants: constants.as_ptr(),
            tessellation: self
                .tessellation
                .as_ref()
                .map_or(ptr::null(), ptr::from_ref),
            stage_in: self.stage_in.as_deref().map_or(ptr::null(), ptr::from_ref),
            modules: modules.as_ptr(),
            linked_functions: functions.as_ptr(),
            module_count: count(modules.len()),
            linked_function_count: count(functions.len()),
            max_call_stack_depth: specialization.linking.max_call_stack_depth,
            sampler_count: count(self.samplers.len()),
            samplers: self.samplers.as_ptr(),
            attachments: self.attachments.as_ptr(),
            attachment_count: if storage {
                count(self.attachments.len())
            } else {
                0
            },
            samples: specialization.samples.max(1),
            round_toward_zero: specialization.round_toward_zero.into(),
            extended_range: specialization.extended_range,
            vertex_formats: vertex_formats(&specialization.vertex_words),
            imageblock_size: specialization.imageblock_size.unwrap_or_default(),
            tile: specialization.tile.into(),
            fragment_tests: [
                (specialization.self_tests, ffi::SELF_TESTS),
                (specialization.alpha_to_coverage, ffi::ALPHA_TO_COVERAGE),
                (specialization.alpha_to_one, ffi::ALPHA_TO_ONE),
            ]
            .into_iter()
            .filter(|&(set, _)| set)
            .fold(0, |bits, (_, bit)| bits | bit),
            bicubic: specialization.bicubic_samplers & !ffi::BICUBIC_ARGUMENTS
                | if specialization.bicubic_arguments {
                    ffi::BICUBIC_ARGUMENTS
                } else {
                    0
                },
        };
        let mut error = ffi::AirError::NONE;
        // SAFETY: the AIR, entry, target and specialization, with the arrays,
        // identifiers, values, layouts, modules and names it points to,
        // outlive the call, which writes only the error. The libraries the
        // AIR and modules are in are held. The compilation fields come from
        // this service's reflection of the same input and constants.
        let shader = unsafe {
            (self.service.compile)(
                self.air.cast(),
                self.air_size,
                self.entry.as_ptr(),
                &self.target,
                &raw,
                &*self.reflected,
                &mut error,
            )
        };
        if shader.is_null() {
            return Err(Box::new(error));
        }
        Ok(self.finish(shader))
    }

    /// The compiled shader of `shader`, this job's NIR, with its NIR's bytes.
    fn finish(&self, shader: *mut c_void) -> (CompiledShader, usize) {
        let specialization = &self.specialization;
        let mut interface = self.interface.clone();
        // SAFETY: the compiler's live NIR, which the call only reads.
        interface.buffer_reads = unsafe { (self.service.buffer_reads)(shader) };
        interface.storage_attachments = specialization
            .storage_attachments
            .iter()
            .enumerate()
            .filter(|(_, attachment)| attachment.is_some())
            .fold(0, |mask, (index, _)| mask | 1 << index);
        (
            CompiledShader {
                code: ShaderCode(shader as usize as u64),
                interface,
                provenance: self.provenance.clone(),
            },
            nir_bytes(shader),
        )
    }
}

/// Mesa's `struct blob` and `struct blob_reader`.
#[cfg(all(feature = "metal-linked", target_os = "uefi"))]
#[repr(C)]
struct Blob {
    data: *mut u8,
    allocated: usize,
    size: usize,
    fixed_allocation: bool,
    out_of_memory: bool,
}

#[cfg(all(feature = "metal-linked", target_os = "uefi"))]
#[repr(C)]
struct BlobReader {
    data: *const u8,
    end: *const u8,
    current: *const u8,
    overrun: bool,
}

#[cfg(all(feature = "metal-linked", target_os = "uefi"))]
unsafe extern "C" {
    fn blob_init(blob: *mut Blob);
    fn blob_reader_init(reader: *mut BlobReader, data: *const c_void, size: usize);
    fn nir_serialize(blob: *mut Blob, shader: *const c_void, strip: bool);
    fn nir_deserialize(
        context: *mut c_void,
        options: *const c_void,
        reader: *mut BlobReader,
    ) -> *mut c_void;
    fn free(pointer: *mut c_void);
}

/// Whether this image serializes NIR: the linked Mesa does.
const fn nir_serialization() -> bool {
    cfg!(all(feature = "metal-linked", target_os = "uefi"))
}

/// `code`'s NIR serialized whole, names kept, so the NIR deserialized from it
/// is the same and the driver's pipeline cache, keyed by its digest, finds
/// what it compiled from it.
fn serialize(code: ShaderCode) -> Option<Vec<u8>> {
    #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
    {
        let mut blob = Blob {
            data: ptr::null_mut(),
            allocated: 0,
            size: 0,
            fixed_allocation: false,
            out_of_memory: false,
        };
        // SAFETY: a blob of Mesa's layout, and the service's live NIR, which
        // serializing only reads. The blob's memory is Mesa's to free.
        unsafe {
            blob_init(&mut blob);
            nir_serialize(&mut blob, code.0 as usize as *const c_void, false);
        }
        let bytes = (!blob.out_of_memory && !blob.data.is_null()).then(|| {
            // SAFETY: the blob's written bytes.
            unsafe { slice::from_raw_parts(blob.data, blob.size) }.to_vec()
        });
        // SAFETY: `blob_finish` of a growable blob.
        unsafe { free(blob.data.cast()) };
        bytes
    }
    #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
    {
        let _ = code;
        None
    }
}

/// The NIR `blob` serialized, compiled for the driver's `options`.
fn deserialize(blob: &[u8], options: *const c_void) -> Option<*mut c_void> {
    #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
    {
        let mut reader = BlobReader {
            data: ptr::null(),
            end: ptr::null(),
            current: ptr::null(),
            overrun: false,
        };
        // SAFETY: the stored bytes outlive the read, and the options are the
        // driver's, as the compiler's own NIR is made with.
        let shader = unsafe {
            blob_reader_init(&mut reader, blob.as_ptr().cast(), blob.len());
            nir_deserialize(ptr::null_mut(), options, &mut reader)
        };
        if shader.is_null() {
            return None;
        }
        if reader.overrun || reader.current != reader.end {
            // SAFETY: the deserialized shader, a ralloc context of its own.
            unsafe { free_nir(shader) };
            return None;
        }
        Some(shader)
    }
    #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
    {
        let _ = (blob, options);
        None
    }
}

/// A copy of `code`'s NIR that the caller owns until [`free_nir_copy`], as
/// a deferred pipeline creation needs once its call has returned the
/// original; `None` where this image links no Mesa.
pub fn clone_nir(code: ShaderCode) -> Option<ShaderCode> {
    #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
    {
        unsafe extern "C" {
            fn nir_shader_clone(context: *mut c_void, shader: *const c_void) -> *mut c_void;
        }
        // SAFETY: the compiler's live NIR, which cloning only reads.
        let copy = unsafe { nir_shader_clone(ptr::null_mut(), code.0 as usize as *const c_void) };
        (!copy.is_null()).then_some(ShaderCode(copy as usize as u64))
    }
    #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
    {
        let _ = code;
        None
    }
}

/// Frees a copy [`clone_nir`] made.
pub fn free_nir_copy(code: ShaderCode) {
    #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
    // SAFETY: the copy, a ralloc context of its own that nothing else holds.
    unsafe {
        free_nir(code.0 as usize as *mut c_void);
    }
    #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
    let _ = code;
}

/// Frees NIR deserialized here.
#[cfg(all(feature = "metal-linked", target_os = "uefi"))]
unsafe fn free_nir(shader: *mut c_void) {
    unsafe extern "C" {
        fn ralloc_free(context: *mut c_void);
    }
    // SAFETY: the caller's shader, which nothing else holds.
    unsafe { ralloc_free(shader) }
}

/// The bytes `shader`'s NIR takes, for the cache's budget.
fn nir_bytes(shader: *mut c_void) -> usize {
    #[cfg(all(feature = "metal-linked", target_os = "uefi"))]
    {
        unsafe extern "C" {
            /// Mesa's `ralloc_total_size`: a context and its children.
            fn ralloc_total_size(context: *const c_void) -> usize;
        }
        // SAFETY: the service's NIR, a ralloc context of its own.
        unsafe { ralloc_total_size(shader) }
    }
    #[cfg(not(all(feature = "metal-linked", target_os = "uefi")))]
    {
        // A hosted service's shaders are not NIR this image can measure.
        let _ = shader;
        64 << 10
    }
}

impl ffi::Tessellation {
    fn new(tessellation: &Tessellation) -> Self {
        Self {
            quad: tessellation.patch.domain == PatchDomain::Quad,
            partition: tessellation.partition.raw() as u32,
            max_factor: tessellation.max_factor,
            step: tessellation.step_function.raw() as u32,
            winding: tessellation.winding.raw() as u32,
            control_point_index_bytes: match tessellation.control_point_index {
                TessellationControlPointIndexType::None => 0,
                TessellationControlPointIndexType::UInt16 => 2,
                TessellationControlPointIndexType::UInt32 => 4,
            },
            scales: tessellation.factor_scale,
        }
    }
}

impl ffi::StageIn {
    /// The layout of a validated vertex descriptor.
    fn vertex(descriptor: &VertexDescriptor) -> Result<Box<Self>> {
        let layouts = descriptor.layouts.iter().map(|layout| {
            let step = layout.step_function.raw();
            (layout.buffer_index, layout.stride, step, layout.step_rate)
        });
        Self::new(&descriptor.attributes, layouts, 0, 0)
    }

    /// The layout of a validated stage-input descriptor.
    fn kernel(descriptor: &StageInputDescriptor) -> Result<Box<Self>> {
        let layouts = descriptor.layouts.iter().map(|layout| {
            let step = layout.step_function.raw();
            (layout.buffer_index, layout.stride, step, layout.step_rate)
        });
        let index_bytes = match descriptor.index_type {
            IndexType::UInt16 => 2,
            IndexType::UInt32 => 4,
        };
        Self::new(
            &descriptor.attributes,
            layouts,
            index_bytes,
            descriptor.index_buffer_index,
        )
    }

    /// Attributes, and each layout's buffer, stride and raw step function
    /// and rate.
    fn new(
        attributes: &[Attribute],
        layouts: impl Iterator<Item = (u32, u64, u64, u64)>,
        index_bytes: u32,
        index_buffer: u32,
    ) -> Result<Box<Self>> {
        const INVALID: Error = Error::Invalid("stage-in descriptor");
        let mut stage_in = Box::new(Self {
            attributes: [ffi::StageInAttribute::default(); ffi::ATTRIBUTES],
            layouts: Default::default(),
            index_bytes,
            index_buffer,
        });
        for attribute in attributes {
            *stage_in
                .attributes
                .get_mut(attribute.index as usize)
                .ok_or(INVALID)? = ffi::StageInAttribute {
                format: attribute.format.raw() as u32,
                offset: u32::try_from(attribute.offset).map_err(|_| INVALID)?,
                buffer: attribute.buffer_index,
            };
        }
        for (buffer, stride, step, rate) in layouts {
            *stage_in.layouts.get_mut(buffer as usize).ok_or(INVALID)? = ffi::StageInLayout {
                stride: match stride {
                    STRIDE_DYNAMIC => ffi::STRIDE_DYNAMIC,
                    stride => u32::try_from(stride)
                        .ok()
                        .filter(|stride| *stride != ffi::STRIDE_DYNAMIC)
                        .ok_or(INVALID)?,
                },
                step: u32::try_from(step).map_err(|_| INVALID)?,
                rate: u32::try_from(rate).map_err(|_| INVALID)?,
            };
        }
        Ok(stage_in)
    }
}

/// Function constant values as the service takes them, borrowing their
/// bytes.
fn constant_values(constants: &[(u32, Vec<u8>)]) -> Vec<ffi::ConstantValue> {
    constants
        .iter()
        .map(|(index, value)| ffi::ConstantValue {
            index: *index,
            bytes: value.as_ptr().cast(),
            size: value.len(),
        })
        .collect()
}

/// An AIR module a function links, with the function constant values of
/// the functions linked from it, or none for a dynamic library's, and its
/// copy number: each set of values a module links with is its own copy.
struct LinkedModule<'a> {
    source: ModuleSource<'a>,
    constants: Option<&'a [(u32, Vec<u8>)]>,
    copy: u32,
    /// `(definition, name)`: calls of `name` reach the copy's `definition`.
    aliases: Vec<(&'a str, &'a str)>,
}

/// The suffix of the names a module's `copy` defines (`.stela.copy<copy>`,
/// `struct stela_air_module`), empty for the first.
fn copy_suffix(copy: u32) -> String {
    match copy {
        0 => String::new(),
        copy => alloc::format!(".stela.copy{copy}"),
    }
}

/// Whether two sets of function constant values, each with one value per
/// index, specialize a module alike.
fn same_values(a: Option<&[(u32, Vec<u8>)]>, b: Option<&[(u32, Vec<u8>)]>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => a.len() == b.len() && a.iter().all(|value| b.contains(value)),
        _ => true,
    }
}

/// The AIR modules `linking` adds to the function whose module has `own`
/// digest and values, each once for each set of values it links with, and
/// the copy of the module each of `linking.functions` comes from. The
/// function's own module is copy 0 of its digest; a dynamic library's
/// modules take the values of a function linked from them.
fn linked_modules<'a>(
    libraries: &'a Libraries,
    linking: &'a Linking,
    own: ([u8; 32], &'a [(u32, Vec<u8>)]),
) -> Result<(Vec<LinkedModule<'a>>, Vec<u32>)> {
    let mut modules: Vec<LinkedModule<'a>> = Vec::new();
    let mut add = |source: ModuleSource<'a>, constants, alias: Option<(&'a str, &'a str)>| -> u32 {
        // An alias needs a copy to name, so the function's own module
        // links again for it.
        if alias.is_none() && source.slice_hash == own.0 && same_values(Some(own.1), constants) {
            return 0;
        }
        let copies = || {
            modules
                .iter()
                .filter(|module| module.source.slice_hash == source.slice_hash)
        };
        if let Some(module) = copies()
            .find(|module| same_values(module.constants, constants))
            .map(|module| module.copy)
        {
            let module = modules
                .iter_mut()
                .find(|known| known.source.slice_hash == source.slice_hash && known.copy == module)
                .expect("the copy found");
            module.constants = module.constants.or(constants);
            if let Some(alias) = alias
                && !module.aliases.contains(&alias)
            {
                module.aliases.push(alias);
            }
            return module.copy;
        }
        // The function's own module is copy 0 of its digest.
        let first = u32::from(source.slice_hash != own.0);
        let copy = copies()
            .map(|module| module.copy + 1)
            .max()
            .unwrap_or(1 - first);
        modules.push(LinkedModule {
            source,
            constants,
            copy,
            aliases: alias.into_iter().collect(),
        });
        copy
    };
    let mut copies = Vec::with_capacity(linking.functions.len());
    for (index, function) in linking
        .functions
        .iter()
        .chain(&linking.private_functions)
        .enumerate()
    {
        let selected = select(libraries, function.library, &function.entry)?;
        let alias = function
            .alias
            .as_deref()
            .map(|name| (function.entry.as_str(), name));
        let copy = add(selected.module, Some(function.constants.as_slice()), alias);
        if index < linking.functions.len() {
            copies.push(copy);
        }
    }
    for library in &linking.libraries {
        let library = libraries
            .get(library)
            .ok_or(Error::Invalid("unknown library"))?;
        ensure(library.dynamic, "a preloaded library that is not dynamic")?;
        for module in library.modules() {
            add(module?, None, None);
        }
    }
    Ok((modules, copies))
}

/// Selects `entry` of `library`.
fn select<'a>(libraries: &'a Libraries, library: u64, entry: &str) -> Result<Selected<'a>> {
    libraries
        .get(&library)
        .ok_or(Error::Invalid("unknown library"))?
        .select(entry)
}

impl Selected<'_> {
    /// The service's reflection of the function specialized with the
    /// function constant values `values`, or as it is declared without.
    fn reflect_raw(
        &self,
        service: &Service,
        values: Option<&[ffi::ConstantValue]>,
        refusal: &mut Option<Refusal>,
    ) -> Result<Box<ffi::Reflection>> {
        let mut reflection = ffi::Reflection::zeroed();
        let mut error = ffi::AirError::NONE;
        let air = self.module.bitcode;
        let (values, count) = match values {
            Some(values) => (
                values.as_ptr(),
                u32::try_from(values.len()).map_err(|_| MALFORMED)?,
            ),
            None => (core::ptr::null(), 0),
        };
        // SAFETY: the AIR, entry and values outlive the call, which writes
        // only the reflection and the error.
        let reflected = unsafe {
            (service.reflect)(
                air.as_ptr().cast(),
                air.len(),
                self.entry.as_ptr(),
                values,
                count,
                &mut *reflection,
                &mut error,
            )
        };
        if !reflected {
            return Err(refuse(refusal, self.name, &error));
        }
        Ok(reflection)
    }

    fn decode_reflection(&self, reflection: &ffi::Reflection) -> Result<(Reflection, Provenance)> {
        let provenance = Provenance {
            build_id: name(reflection, reflection.ident, reflection.ident_bytes)?.to_vec(),
            slice_hash: self.module.slice_hash,
            source_map: None,
            entry: self.name.to_string(),
        };
        let reflection = if u64::from(reflection.stage) == FunctionType::Visible.raw() {
            Reflection::Visible {
                constants: constants(reflection)?,
                command_barriers: reflection.command_barriers != 0,
            }
        } else {
            Reflection::Stage(Box::new(interface(reflection)?))
        };
        Ok((reflection, provenance))
    }
}

/// Records the service's refusal and names what it refused.
fn refuse(refusal: &mut Option<Refusal>, entry: &str, error: &ffi::AirError) -> Error {
    use ffi::kind;
    *refusal = Some(Refusal {
        entry: entry.to_string(),
        instruction: error.instruction,
        detail: error.detail(),
    });
    match error.kind {
        kind::INPUT => Error::Invalid("compiler request"),
        kind::ALLOCATION => Error::OutOfMemory,
        unsupported => Error::Unsupported(match unsupported {
            kind::LLVM_VERSION => "AIR from a newer LLVM",
            kind::BITCODE => "AIR bitcode",
            kind::VERIFY => "AIR that LLVM rejects",
            kind::TARGET => "AIR target",
            kind::METADATA => "AIR metadata",
            kind::ARGUMENT => "AIR function arguments",
            kind::CONTROL_FLOW => "AIR control flow",
            kind::INSTRUCTION => "AIR instructions",
            kind::TYPE => "AIR types",
            kind::MEMORY => "AIR memory access",
            kind::LIMIT => "AIR beyond the compiler's limits",
            _ => "AIR",
        }),
    }
}

/// The first `count` of a reflection's rows.
fn rows<T>(rows: &[T], count: u32) -> Result<&[T]> {
    rows.get(..count as usize).ok_or(MALFORMED)
}

/// `bytes` of the reflection's names from `offset`.
fn name(reflection: &ffi::Reflection, offset: u32, bytes: u32) -> Result<&[u8]> {
    let end = offset.checked_add(bytes).ok_or(MALFORMED)?;
    if end > reflection.names_bytes {
        return Err(MALFORMED);
    }
    reflection
        .names
        .get(offset as usize..end as usize)
        .ok_or(MALFORMED)
}

fn access(bits: u32) -> Result<Access> {
    u8::try_from(bits)
        .ok()
        .and_then(Access::from_bits)
        .ok_or(MALFORMED)
}

fn scalar(primitive: u32) -> Result<Scalar> {
    use ffi::primitive::*;
    Ok(match primitive {
        CHAR => Scalar::Char,
        UCHAR => Scalar::Uchar,
        SHORT => Scalar::Short,
        USHORT => Scalar::Ushort,
        INT => Scalar::Int,
        UINT => Scalar::Uint,
        FLOAT => Scalar::Float,
        HALF => Scalar::Half,
        _ => return Err(MALFORMED),
    })
}

/// The compilation ABI reads only these fields. Rebuild them from the
/// validated interface to avoid retaining a full ABI record for each function.
fn compilation_reflection(interface: &Interface) -> Result<Box<ffi::Reflection>> {
    let mut reflection = ffi::Reflection::zeroed();
    reflection.stage = interface.stage.raw() as u32;
    reflection.patch = match interface.patch.map(|patch| patch.domain) {
        None => 0,
        Some(PatchDomain::Triangle) => 1,
        Some(PatchDomain::Quad) => 2,
    };
    reflection.threadgroup_bytes = interface.static_threadgroup_bytes;
    for (index, varying) in interface.varyings.iter().enumerate() {
        let identifier = varying.identifier.as_bytes();
        let offset = reflection.names_bytes as usize;
        let end = offset.checked_add(identifier.len()).ok_or(MALFORMED)?;
        reflection
            .names
            .get_mut(offset..end)
            .ok_or(MALFORMED)?
            .copy_from_slice(identifier);
        *reflection.varyings.get_mut(index).ok_or(MALFORMED)? = ffi::Varying {
            identifier: offset as u32,
            identifier_bytes: identifier.len() as u32,
            primitive: match varying.scalar {
                Scalar::Char => ffi::primitive::CHAR,
                Scalar::Uchar => ffi::primitive::UCHAR,
                Scalar::Short => ffi::primitive::SHORT,
                Scalar::Ushort => ffi::primitive::USHORT,
                Scalar::Int => ffi::primitive::INT,
                Scalar::Uint => ffi::primitive::UINT,
                Scalar::Float => ffi::primitive::FLOAT,
                Scalar::Half => ffi::primitive::HALF,
            },
            components: varying.components.into(),
            per_primitive: varying.per_primitive.into(),
        };
        reflection.names_bytes = end as u32;
    }
    reflection.varying_count = interface.varyings.len() as u32;
    Ok(reflection)
}

fn numeric(primitive: u32) -> Result<NumericClass> {
    Ok(match scalar(primitive)? {
        Scalar::Float | Scalar::Half => NumericClass::Float,
        Scalar::Int | Scalar::Short | Scalar::Char => NumericClass::Sint,
        Scalar::Uint | Scalar::Ushort | Scalar::Uchar => NumericClass::Uint,
    })
}

fn components(count: u32) -> Result<u8> {
    u8::try_from(count)
        .ok()
        .filter(|count| (1..=4).contains(count))
        .ok_or(MALFORMED)
}

/// The `MTLDataType` of a function constant's MSL type spelling, a scalar
/// or vector as `float`, `uint3` or `bool2`.
fn data_type(spelling: &[u8]) -> Result<DataType> {
    DataType::from_spelling(spelling)
        .filter(|data_type| data_type.lanes().is_some())
        .ok_or(Error::Unsupported("function constant type"))
}

/// `enum stela_air_patch` and the control points the function declares.
fn patch(raw: u32, control_points: u32) -> Result<Option<Patch>> {
    let domain = match raw {
        0 => return (control_points == 0).then_some(None).ok_or(MALFORMED),
        1 => PatchDomain::Triangle,
        2 => PatchDomain::Quad,
        _ => return Err(MALFORMED),
    };
    let control_points = match control_points {
        0 => None,
        1..=MAX_PATCH_CONTROL_POINTS => Some(control_points),
        _ => return Err(MALFORMED),
    };
    Ok(Some(Patch {
        domain,
        control_points,
    }))
}

/// `enum stela_air_texture_type`.
fn texture_type(raw: u32) -> Result<TextureType> {
    const TYPES: [TextureType; 10] = [
        TextureType::D1,
        TextureType::D1Array,
        TextureType::D2,
        TextureType::D2Array,
        TextureType::D2Multisample,
        TextureType::D2MultisampleArray,
        TextureType::D3,
        TextureType::Cube,
        TextureType::CubeArray,
        TextureType::TextureBuffer,
    ];
    let index = raw.checked_sub(1).ok_or(MALFORMED)?;
    TYPES.get(index as usize).copied().ok_or(MALFORMED)
}

fn constants(r: &ffi::Reflection) -> Result<Vec<FunctionConstant>> {
    rows(&r.function_constants, r.function_constant_count)?
        .iter()
        .map(|constant| {
            let constant_name = name(r, constant.name, constant.name_bytes)?;
            Ok(FunctionConstant {
                name: String::from(str::from_utf8(constant_name).map_err(|_| MALFORMED)?),
                data_type: data_type(name(r, constant.type_name, constant.type_bytes)?)?,
                index: constant.index,
                required: constant.required != 0,
            })
        })
        .collect()
}

fn interface(reflection: &ffi::Reflection) -> Result<Interface> {
    let r = reflection;
    let stage = Stage::from_raw(r.stage.into()).ok_or(MALFORMED)?;
    let patch = patch(r.patch, r.patch_control_points)?;
    let buffers = rows(&r.buffers, r.buffer_count)?
        .iter()
        .map(|buffer| {
            Ok(BufferUse {
                index: buffer.index,
                access: access(buffer.access)?,
                argument_buffer: buffer.argument_buffer != 0,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let textures = rows(&r.textures, r.texture_count)?
        .iter()
        .map(|texture| {
            Ok(TextureUse {
                index: texture.index,
                access: access(texture.access)?,
                texture_type: texture_type(texture.texture_type)?,
                numeric: numeric(texture.primitive)?,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let threadgroup = rows(&r.threadgroup, r.threadgroup_count)?
        .iter()
        .map(|threadgroup| ThreadgroupUse {
            index: threadgroup.index,
            declared_bytes: threadgroup.bytes.into(),
        })
        .collect::<Vec<_>>();
    let declared_attributes = rows(&r.attributes, r.attribute_count)?
        .iter()
        .map(|attribute| {
            let name = name(r, attribute.name, attribute.name_bytes)?;
            let scalar = scalar(attribute.primitive)?.data_type();
            let components = components(attribute.components)?;
            Ok(DeclaredAttribute {
                name: String::from(str::from_utf8(name).map_err(|_| MALFORMED)?),
                index: attribute.index,
                data_type: DataType::vector(scalar, components.into()).ok_or(MALFORMED)?,
                active: attribute.active != 0,
                patch_control_point: attribute.patch_control_point != 0,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let attributes = rows(&r.attributes, r.attribute_count)?
        .iter()
        .filter(|attribute| attribute.active != 0)
        .map(|attribute| {
            Ok(AttributeUse {
                index: attribute.index,
                numeric: numeric(attribute.primitive)?,
                components: components(attribute.components)?,
                patch_control_point: attribute.patch_control_point != 0,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let varyings = rows(&r.varyings, r.varying_count)?
        .iter()
        .map(|varying| {
            let identifier = name(r, varying.identifier, varying.identifier_bytes)?;
            Ok(Varying {
                identifier: String::from(str::from_utf8(identifier).map_err(|_| MALFORMED)?),
                scalar: scalar(varying.primitive)?,
                components: components(varying.components)?,
                per_primitive: varying.per_primitive != 0,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let color_outputs = rows(&r.color_outputs, r.color_output_count)?
        .iter()
        .map(|output| {
            Ok(ColorOutput {
                index: output.index,
                numeric: numeric(output.primitive)?,
                dual_source: output.dual_source != 0,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let constants = constants(r)?;
    let features = match stage {
        Stage::Kernel => ShaderFeatures::COMPUTE,
        Stage::Vertex | Stage::Fragment => ShaderFeatures::VERTEX_FRAGMENT,
        Stage::Object | Stage::Mesh => ShaderFeatures::MESH,
    }
    .with(
        ShaderFeatures::TEXTURE_SAMPLE,
        textures
            .iter()
            .any(|texture| texture.access.contains(Access::SAMPLE)),
    )
    .with(
        ShaderFeatures::TEXTURE_READ_WRITE,
        textures.iter().any(|texture| texture.access.writes()),
    )
    .with(
        ShaderFeatures::THREADGROUP_MEMORY,
        !threadgroup.is_empty() || r.threadgroup_bytes != 0,
    )
    .with(
        ShaderFeatures::ARGUMENT_BUFFERS,
        buffers.iter().any(|buffer| buffer.argument_buffer),
    )
    .with(ShaderFeatures::VERTEX_AMPLIFICATION, r.amplification != 0)
    .with(ShaderFeatures::TESSELLATION, patch.is_some())
    .with(ShaderFeatures::BARYCENTRICS, r.barycentrics != 0)
    .with(
        ShaderFeatures::PULL_MODEL_INTERPOLATION,
        r.interpolants != 0,
    )
    .with(ShaderFeatures::FUNCTION_POINTERS, r.function_tables != 0)
    .with(ShaderFeatures::INDIRECT_COMMAND_ENCODING, r.commands != 0);
    let interface = Interface {
        stage,
        patch,
        buffers,
        textures,
        samplers: rows(&r.samplers, r.sampler_count)?.to_vec(),
        threadgroup,
        attributes,
        declared_attributes,
        varyings,
        color_outputs,
        writes_depth: r.writes_depth != 0,
        writes_stencil: r.writes_stencil != 0,
        writes_sample_mask: r.writes_sample_mask != 0,
        discards: r.discards != 0,
        // The frontend refuses metadata it does not lower, this among it.
        early_fragment_tests: false,
        static_threadgroup_bytes: r.threadgroup_bytes,
        imageblock_bytes: r.imageblock_bytes,
        max_total_threads: (r.max_total_threads != 0).then_some(r.max_total_threads),
        amplification: r.amplification != 0,
        command_barriers: r.command_barriers != 0,
        attachment_reads: r.attachment_reads,
        storage_attachments: 0,
        buffer_reads: 0,
        constants,
        features,
    };
    interface.validate()?;
    Ok(interface)
}

/// The raw `MTLVertexFormat` of each vertex input fetched as words, zero for
/// the rest.
fn vertex_formats(words: &[Option<VertexFormat>]) -> [u8; 32] {
    let mut raw = [0; 32];
    for (slot, format) in raw.iter_mut().zip(words) {
        // Every format's value is below 64.
        *slot = format.map_or(0, |format| format.raw() as u8);
    }
    raw
}

/// The AIR module of `graphs` stitched from the `selected` functions with
/// their constant `values`, or `None` when the service refused, as `error`
/// says. The caller holds the compile lock.
fn stitch_raw(
    service: &Service,
    selected: &[Selected],
    values: &[Vec<ffi::ConstantValue>],
    graphs: &[StitchedGraph],
    error: &mut ffi::AirError,
) -> Result<Option<Vec<u8>>> {
    unsafe extern "C" fn visit(data: *mut c_void, bytes: *const c_void, size: usize) {
        // SAFETY: the data is the module below, and the service's bytes
        // are `size` long for the call.
        let (module, bytes) = unsafe {
            (
                &mut *data.cast::<Vec<u8>>(),
                slice::from_raw_parts(bytes.cast::<u8>(), size),
            )
        };
        module.extend_from_slice(bytes);
    }
    let count = |len: usize| u32::try_from(len).map_err(|_| MALFORMED);
    let sources = selected
        .iter()
        .zip(values)
        .map(|(selected, values)| {
            Ok(ffi::StitchSource {
                bitcode: selected.module.bitcode.as_ptr().cast(),
                size: selected.module.bitcode.len(),
                name: selected.entry.as_ptr(),
                constants: values.as_ptr(),
                constant_count: count(values.len())?,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let names = graphs
        .iter()
        .map(|graph| CString::new(graph.name.as_str()).map_err(|_| MALFORMED))
        .collect::<Result<Vec<_>>>()?;
    let nodes = graphs
        .iter()
        .map(|graph| {
            graph
                .nodes
                .iter()
                .map(|node| {
                    Ok(match node {
                        StitchedNode::Input(index) => ffi::StitchNode {
                            function: ffi::STITCH_INPUT,
                            input: *index,
                            ..ffi::StitchNode::NONE
                        },
                        &StitchedNode::Builtin(builtin) => ffi::StitchNode {
                            function: ffi::STITCH_BUILTIN,
                            input: builtin as u32,
                            ..ffi::StitchNode::NONE
                        },
                        &StitchedNode::BufferAddress {
                            index,
                            offset,
                            dereference,
                        } => ffi::StitchNode {
                            function: ffi::STITCH_BUFFER_ADDRESS,
                            input: index,
                            dereference,
                            offset,
                            ..ffi::StitchNode::NONE
                        },
                        StitchedNode::EarlyReturn { condition, value } => ffi::StitchNode {
                            function: ffi::STITCH_EARLY_RETURN,
                            input: *condition,
                            arguments: value.as_slice().as_ptr(),
                            argument_count: u32::from(value.is_some()),
                            ..ffi::StitchNode::NONE
                        },
                        &StitchedNode::Texture(index) => ffi::StitchNode {
                            function: ffi::STITCH_TEXTURE,
                            input: index,
                            ..ffi::StitchNode::NONE
                        },
                        &StitchedNode::Sampler(index) => ffi::StitchNode {
                            function: ffi::STITCH_SAMPLER,
                            input: index,
                            ..ffi::StitchNode::NONE
                        },
                        &StitchedNode::Threadgroup(index) => ffi::StitchNode {
                            function: ffi::STITCH_THREADGROUP,
                            input: index,
                            ..ffi::StitchNode::NONE
                        },
                        StitchedNode::Call {
                            function,
                            arguments,
                        } => ffi::StitchNode {
                            function: *function,
                            arguments: arguments.as_ptr(),
                            argument_count: count(arguments.len())?,
                            ..ffi::StitchNode::NONE
                        },
                    })
                })
                .collect::<Result<Vec<_>>>()
        })
        .collect::<Result<Vec<_>>>()?;
    let raw = graphs
        .iter()
        .zip(&names)
        .zip(&nodes)
        .map(|((graph, name), nodes)| {
            Ok(ffi::StitchGraph {
                name: name.as_ptr(),
                nodes: nodes.as_ptr(),
                node_count: count(nodes.len())?,
                output: graph.output.unwrap_or(u32::MAX),
                kernel: graph.kernel,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let mut module = Vec::new();
    // SAFETY: the sources and graphs, with what they point to, and the
    // module outlive the call, which writes the module through `visit`
    // and the error.
    let stitched = unsafe {
        (service.stitch)(
            sources.as_ptr(),
            count(sources.len())?,
            raw.as_ptr(),
            count(raw.len())?,
            visit,
            ptr::from_mut(&mut module).cast(),
            error,
        )
    };
    Ok(stitched.then_some(module))
}

/// The functions the AIR module `air` defines, or `None` when the service
/// refused, as `error` says. The caller holds the compile lock.
fn list_functions(
    service: &Service,
    air: &[u8],
    error: &mut ffi::AirError,
) -> Result<Option<Listing>> {
    unsafe extern "C" fn visit(
        data: *mut c_void,
        name: *const c_char,
        name_bytes: usize,
        stage: u32,
    ) -> bool {
        // SAFETY: the data is the list below, and the service's name is
        // `name_bytes` long for the call.
        let (list, name) = unsafe {
            (
                &mut *data.cast::<Listing>(),
                slice::from_raw_parts(name.cast::<u8>(), name_bytes),
            )
        };
        let (Ok(name), Some(stage)) = (str::from_utf8(name), FunctionType::from_raw(stage.into()))
        else {
            return false;
        };
        list.push((String::from(name), stage));
        true
    }
    let air = Air::parse(air).map_err(|_| Error::Invalid("AIR"))?;
    let mut functions = Vec::new();
    // SAFETY: the bitcode and the list outlive the call, which writes the
    // list through `visit` and the error.
    let listed = unsafe {
        (service.functions)(
            air.bitcode.as_ptr().cast(),
            air.bitcode.len(),
            visit,
            ptr::from_mut(&mut functions).cast(),
            error,
        )
    };
    Ok(listed.then_some(functions))
}

/// The digest a stitched library is known by among jobs and in
/// [`AirCompiler::stitched`]: its functions' modules, entries and constant
/// values, and its graphs.
fn stitch_digest(
    selected: &[Selected],
    sources: &[LinkedFunction],
    graphs: &[StitchedGraph],
) -> [u8; 32] {
    use stela_crypto::{Sha256, digest::Digest};
    let mut hasher = Sha256::new();
    hasher.update(b"stitch");
    hasher.update((selected.len() as u64).to_le_bytes());
    for (selected, source) in selected.iter().zip(sources) {
        hasher.update(selected.module.slice_hash);
        hasher.update((source.entry.len() as u64).to_le_bytes());
        hasher.update(source.entry.as_bytes());
        hasher.update((source.constants.len() as u64).to_le_bytes());
        for (index, value) in &source.constants {
            hasher.update(index.to_le_bytes());
            hasher.update((value.len() as u64).to_le_bytes());
            hasher.update(value);
        }
    }
    hasher.update(crate::metal::stitching::encode(graphs));
    hasher.finalize().into()
}

/// The functions an AIR module defines, with their types.
type Listing = Vec<(String, FunctionType)>;

/// A function a [`StitchJob`] stitches: its library, entry and constant
/// values.
type StitchSource = (Arc<LoadedLibrary>, String, Vec<(u32, Vec<u8>)>);

/// A stitched library the call's turn left, owned, to run outside the
/// service's lock: its functions' libraries, entries and constant values,
/// and its graphs.
struct StitchJob {
    service: Service,
    sources: Vec<StitchSource>,
    graphs: Vec<StitchedGraph>,
    key: [u8; 32],
}

// SAFETY: as for a compile job.
unsafe impl Send for StitchJob {}

/// What a [`StitchJob`] made: the library's AIR and its functions, or
/// `None` when the service refused, which the call's turn then repeats.
struct Stitched {
    key: [u8; 32],
    made: Option<StitchedAir>,
}

/// A stitched library's AIR, with the functions it defines.
struct StitchedAir {
    air: Vec<u8>,
    functions: Listing,
}

impl StitchJob {
    fn run(self) -> Stitched {
        let made = (|| {
            let selected = self
                .sources
                .iter()
                .map(|(library, entry, _)| library.select(entry))
                .collect::<Result<Vec<_>>>()
                .ok()?;
            let values: Vec<_> = self
                .sources
                .iter()
                .map(|(_, _, constants)| constant_values(constants))
                .collect();
            let mut error = ffi::AirError::NONE;
            let air =
                stitch_raw(&self.service, &selected, &values, &self.graphs, &mut error).ok()??;
            let functions = list_functions(&self.service, &air, &mut error).ok()??;
            Some(StitchedAir { air, functions })
        })();
        Stitched {
            key: self.key,
            made,
        }
    }
}
