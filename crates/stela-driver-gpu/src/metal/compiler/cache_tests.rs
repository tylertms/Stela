//! The compiler's caches and deferred compilations, over a service that
//! counts its calls.
extern crate std;

use super::*;
use crate::metal::{
    library::tests::fixture,
    nvidia::{CHIPS, fixture::nvk},
};
use alloc::vec;
use core::cell::Cell;

std::thread_local! {
    /// Reflections, compilations and releases the service made.
    static CALLS: Cell<(u32, u32, u32)> = const { Cell::new((0, 0, 0)) };
}

fn calls() -> (u32, u32, u32) {
    CALLS.get()
}

std::thread_local! {
    /// For the test that runs jobs on another thread: the compile lock and
    /// whether a thread is inside the service, which LLVM, built without
    /// threads, needs to be one at a time and only under the lock.
    static EXCLUSIVE: core::cell::RefCell<Option<(alloc::sync::Arc<CompileLock>, alloc::sync::Arc<core::sync::atomic::AtomicBool>)>> =
        const { core::cell::RefCell::new(None) };
}

/// Enters the service as LLVM would: alone, under the lock.
fn enter_service() {
    EXCLUSIVE.with_borrow(|exclusive| {
        let Some((lock, inside)) = exclusive else {
            return;
        };
        assert!(
            lock.is_held(),
            "the service entered without the compile lock"
        );
        assert!(
            !inside.swap(true, core::sync::atomic::Ordering::AcqRel),
            "two processors in the service at once"
        );
        for _ in 0..2000 {
            core::hint::spin_loop();
        }
        inside.store(false, core::sync::atomic::Ordering::Release);
    });
}

unsafe extern "C" fn reflect(
    _: *const c_void,
    _: usize,
    _: *const c_char,
    _: *const ffi::ConstantValue,
    _: u32,
    reflection: *mut ffi::Reflection,
    _: *mut ffi::AirError,
) -> bool {
    enter_service();
    let (reflected, compiled, released) = CALLS.get();
    CALLS.set((reflected + 1, compiled, released));
    let mut answer = ffi::Reflection::ZERO;
    // A kernel.
    answer.stage = 3;
    // SAFETY: the compiler passes its reflection record.
    unsafe { *reflection = answer };
    true
}

unsafe extern "C" fn reflect_bindings(
    _: *const c_void,
    _: usize,
    _: *const c_char,
    _: *const ffi::ConstantValue,
    _: u32,
    _: *mut ffi::Bindings,
    _: *mut ffi::AirError,
) -> bool {
    false
}

unsafe extern "C" fn sampler_states(
    _: *const c_void,
    _: usize,
    _: *mut u64,
    count: *mut u32,
    _: *mut ffi::AirError,
) -> bool {
    // SAFETY: the compiler passes its count.
    unsafe { *count = 0 };
    true
}

std::thread_local! {
    /// Stitched modules and function listings the service made.
    static STITCHES: Cell<(u32, u32)> = const { Cell::new((0, 0)) };
}

unsafe extern "C" fn functions(
    _: *const c_void,
    _: usize,
    _: ffi::VisitFunction,
    _: *mut c_void,
    _: *mut ffi::AirError,
) -> bool {
    let (stitched, listed) = STITCHES.get();
    STITCHES.set((stitched, listed + 1));
    true
}

/// The module every stitch makes: bare bitcode.
const STITCHED_AIR: &[u8] = b"BC\xc0\xde";

unsafe extern "C" fn stitch(
    _: *const ffi::StitchSource,
    _: u32,
    _: *const ffi::StitchGraph,
    _: u32,
    visit: ffi::VisitBytes,
    data: *mut c_void,
    _: *mut ffi::AirError,
) -> bool {
    enter_service();
    let (stitched, listed) = STITCHES.get();
    STITCHES.set((stitched + 1, listed));
    // SAFETY: the callback takes the bytes for the call.
    unsafe { visit(data, STITCHED_AIR.as_ptr().cast(), STITCHED_AIR.len()) };
    true
}

/// Each compilation's NIR is at its own address.
unsafe extern "C" fn compile(
    _: *const c_void,
    _: usize,
    _: *const c_char,
    _: *const ffi::Target,
    _: *const ffi::Specialization,
    _: *const ffi::Reflection,
    _: *mut ffi::AirError,
) -> *mut c_void {
    let (reflected, compiled, released) = CALLS.get();
    CALLS.set((reflected, compiled + 1, released));
    ptr::without_provenance_mut(0x1000 * (compiled as usize + 1))
}

unsafe extern "C" fn tessellation_stages(
    _: *const ffi::Target,
    _: *const ffi::Tessellation,
    _: *mut *mut c_void,
) {
}

unsafe extern "C" fn kernel(_: *const c_void) -> *mut c_void {
    ptr::null_mut()
}

unsafe extern "C" fn release(_: *mut c_void) {
    let (reflected, compiled, released) = CALLS.get();
    CALLS.set((reflected, compiled, released + 1));
}

unsafe extern "C" fn buffer_reads(_: *const c_void) -> u32 {
    0
}

fn caching() -> (AirCompiler, u64) {
    let service = Service {
        reflect,
        reflect_bindings,
        sampler_states,
        functions,
        exports: functions,
        stitch,
        compile,
        tessellation_stages,
        accumulate_kernel: kernel,
        select_kernel: kernel,
        resolve_draws_kernel: kernel,
        resolve_dispatches_kernel: kernel,
        store_texels_kernel: kernel,
        load_texels_kernel: kernel,
        average_samples_kernel: kernel,
        decode_etc_kernel: kernel,
        decode_pvrtc_kernel: kernel,
        reorder_blocks_kernel: kernel,
        decode_astc_kernel: kernel,
        release,
        buffer_reads,
        nir_options: NonNull::dangling(),
    };
    // SAFETY: the fake service never reads the options.
    let mut compiler = unsafe { AirCompiler::new(service, &nvk(&CHIPS[0], 8 << 30)) };
    compiler.keep_compilations(NIR_BUDGET);
    let library = compiler.add_library(fixture(false)).unwrap();
    (compiler, library)
}

#[test]
fn a_function_made_again_reflects_nothing() {
    let (mut compiler, library) = caching();
    // A function without values, and one with: its declaration and the
    // specialization reflect once each.
    compiler.constants(library, "kernel").unwrap();
    compiler.prepare_function(library, "kernel", None).unwrap();
    let values = vec![(0, vec![1])];
    compiler
        .prepare_function(library, "kernel", Some(values.clone()))
        .unwrap();
    assert_eq!(calls().0, 2);
    // Another library of the same bytes, as another process loads it.
    let again = compiler.add_library(fixture(false)).unwrap();
    compiler.constants(again, "kernel").unwrap();
    compiler
        .prepare_function(again, "kernel", Some(values))
        .unwrap();
    assert_eq!(calls().0, 2);
}

#[test]
fn a_compilation_made_again_is_lent_from_the_cache() {
    let (mut compiler, library) = caching();
    let specialization = Specialization {
        workgroup_size: Some([64, 1, 1]),
        ..Specialization::default()
    };
    let first = compiler
        .compile(library, "kernel", &specialization)
        .unwrap();
    compiler.release(first.code);
    let again = compiler.add_library(fixture(false)).unwrap();
    let second = compiler.compile(again, "kernel", &specialization).unwrap();
    assert_eq!(first, second);
    compiler.release(second.code);
    // Another shape compiles.
    let other = Specialization {
        workgroup_size: Some([32, 1, 1]),
        ..specialization
    };
    let third = compiler.compile(library, "kernel", &other).unwrap();
    assert_ne!(third.code, first.code);
    compiler.release(third.code);
    // The service compiled twice and released nothing the cache holds.
    assert_eq!((calls().1, calls().2), (2, 0));
}

#[test]
fn a_deferred_compilation_runs_outside_and_the_call_finds_it() {
    let (mut compiler, library) = caching();
    let lock = compiler.compile_lock(core::hint::spin_loop);
    compiler.set_deferring(true);
    let specialization = Specialization::default();
    assert_eq!(
        compiler.compile(library, "kernel", &specialization),
        Err(DEFERRED)
    );
    assert!(lock.deferred());
    let job = compiler.take_deferred().unwrap();
    assert!(!lock.deferred());
    assert_eq!(job.name(), "kernel");
    assert_eq!(calls().1, 0);
    let finished = job.run_outside(&lock.hold(core::hint::spin_loop));
    assert_eq!(calls().1, 1);
    compiler.adopt(finished);
    // The call's next run finds the shader without compiling.
    let shader = compiler
        .compile(library, "kernel", &specialization)
        .unwrap();
    compiler.release(shader.code);
    compiler.end_call();
    assert_eq!((calls().1, calls().2), (1, 0));
    assert!(compiler.report().contains("nir_hits=1"));
}

#[test]
fn a_deferred_reflection_runs_outside_and_the_call_finds_it() {
    let (mut compiler, library) = caching();
    let lock = compiler.compile_lock(core::hint::spin_loop);
    let reflected = calls().0;
    compiler.set_deferring(true);
    let constants = Some(vec![(0, vec![1, 0, 0, 0])]);
    assert_eq!(
        compiler
            .prepare_function(library, "kernel", constants.clone())
            .err(),
        Some(DEFERRED)
    );
    assert!(lock.deferred() && Compiler::deferred(&compiler));
    let job = compiler.take_job().unwrap();
    compiler.end_call();
    assert_eq!(calls().0, reflected);
    let done = job.run(&lock.hold(core::hint::spin_loop));
    assert_eq!(calls().0, reflected + 1);
    compiler.adopt_job(done);
    // The call's next run finds the reflection without the service.
    compiler.set_deferring(true);
    let prepared = compiler
        .prepare_function(library, "kernel", constants)
        .unwrap();
    assert_eq!(prepared.constants, vec![(0, vec![1, 0, 0, 0])]);
    assert!(compiler.take_job().is_none());
    assert_eq!(calls().0, reflected + 1);
}

#[test]
fn a_deferred_stitch_runs_outside_and_the_call_finds_it_with_its_functions() {
    let (mut compiler, library) = caching();
    let lock = compiler.compile_lock(core::hint::spin_loop);
    let sources = [LinkedFunction {
        library,
        entry: String::from("kernel"),
        constants: vec![(0, vec![1, 0, 0, 0])],
        alias: None,
    }];
    let graphs = [StitchedGraph {
        name: "stitched".into(),
        nodes: vec![
            StitchedNode::Input(0),
            StitchedNode::Call {
                function: 0,
                arguments: vec![0],
            },
        ],
        output: Some(1),
        kernel: false,
    }];
    compiler.set_deferring(true);
    assert_eq!(compiler.stitch(&sources, &graphs), Err(DEFERRED));
    assert!(lock.deferred() && Compiler::deferred(&compiler));
    let job = compiler.take_job().unwrap();
    compiler.end_call();
    assert_eq!(STITCHES.get(), (0, 0), "nothing ran in the turn");
    let done = job.run(&lock.hold(core::hint::spin_loop));
    assert_eq!(STITCHES.get(), (1, 1), "the job stitched and listed");
    compiler.adopt_job(done);

    // The call's next run takes the module and its listing.
    compiler.set_deferring(true);
    assert_eq!(compiler.stitch(&sources, &graphs).unwrap(), STITCHED_AIR);
    assert_eq!(compiler.functions(STITCHED_AIR), Ok(Vec::new()));
    assert!(compiler.take_job().is_none());
    compiler.end_call();
    assert_eq!(STITCHES.get(), (1, 1));

    // Without deferring, the turn stitches and lists itself.
    compiler.set_deferring(false);
    assert_eq!(compiler.stitch(&sources, &graphs).unwrap(), STITCHED_AIR);
    assert_eq!(compiler.functions(STITCHED_AIR), Ok(Vec::new()));
    assert_eq!(STITCHES.get(), (2, 2));
}

#[test]
fn a_call_that_stopped_deferring_compiles_in_its_turn() {
    let (mut compiler, library) = caching();
    let lock = compiler.compile_lock(core::hint::spin_loop);
    compiler.set_deferring(true);
    // A commit that changed an indirect command buffer cannot run again.
    compiler.stop_deferring();
    let shader = compiler
        .compile(library, "kernel", &Specialization::default())
        .unwrap();
    compiler.release(shader.code);
    assert!(!lock.deferred() && compiler.take_deferred().is_none());
    assert_eq!(calls().1, 1);
}

#[test]
fn without_kept_compilations_nothing_defers_or_stays() {
    let (mut compiler, library) = caching();
    compiler.nir = None;
    compiler.set_deferring(true);
    let shader = compiler
        .compile(library, "kernel", &Specialization::default())
        .unwrap();
    compiler.release(shader.code);
    assert_eq!((calls().1, calls().2), (1, 1));
}

#[test]
fn reflections_in_turns_and_in_jobs_take_the_service_one_at_a_time() {
    use core::sync::atomic::AtomicBool;
    use std::sync::mpsc;
    let (mut compiler, library) = caching();
    let lock = compiler.compile_lock(std::thread::yield_now);
    let inside = alloc::sync::Arc::new(AtomicBool::new(false));
    let exclusive = (lock.clone(), inside.clone());
    EXCLUSIVE.set(Some(exclusive.clone()));
    let (jobs, queue) = mpsc::channel::<Job>();
    let (finished, done) = mpsc::channel::<Done>();
    let worker = std::thread::spawn(move || {
        EXCLUSIVE.set(Some(exclusive));
        let lock = EXCLUSIVE.with_borrow(|exclusive| exclusive.as_ref().unwrap().0.clone());
        for job in queue {
            finished
                .send(job.run(&lock.hold(std::thread::yield_now)))
                .unwrap();
        }
    });
    let constants = |value: u32| Some(vec![(0, value.to_le_bytes().to_vec())]);
    for round in 0..200 {
        // A call that may defer leaves its reflection as a job.
        compiler.set_deferring(true);
        let deferred = compiler.prepare_function(library, "kernel", constants(round));
        assert_eq!(deferred.err(), Some(DEFERRED));
        jobs.send(compiler.take_job().unwrap()).unwrap();
        compiler.end_call();
        // A call that may not reflects in its turn, while the job may run.
        compiler.set_deferring(false);
        compiler
            .prepare_function(library, "kernel", constants(1000 + round))
            .unwrap();
        compiler.end_call();
        for done in done.try_iter() {
            compiler.adopt_job(done);
        }
    }
    drop(jobs);
    worker.join().unwrap();
    for done in done.try_iter() {
        compiler.adopt_job(done);
    }
    EXCLUSIVE.set(None);
    // Each job's reflection is found without the service.
    compiler.set_deferring(true);
    for round in 0..200 {
        compiler
            .prepare_function(library, "kernel", constants(round))
            .unwrap();
    }
    assert!(compiler.take_job().is_none());
}

#[test]
fn a_failed_try_leaves_the_holders_lock_held() {
    let lock = CompileLock::new();
    let held = lock.hold(core::hint::spin_loop);
    assert!(lock.try_hold().is_none());
    assert!(lock.is_held());
    drop(held);
    assert!(lock.try_hold().is_some());
    assert!(!lock.is_held());
}

#[test]
fn an_evicted_compilation_is_found_again_in_what_this_boot_serialized() {
    fn recall(key: &[u8; 32]) -> Option<Vec<u8>> {
        (key == &[2; 32]).then(|| vec![2, 2])
    }
    static STORED: [u8; 1] = [1];
    let (mut compiler, _) = caching();
    // As `persist` sets them where the image serializes NIR.
    compiler.stored.insert([1; 32], &STORED);
    compiler.recall = Some(recall);
    // An earlier boot's comes first, then this boot's, else none.
    let earlier = compiler.serialized(&[1; 32]).unwrap();
    assert_eq!((&*earlier.0, earlier.1), (&[1u8][..], false));
    let recalled = compiler.serialized(&[2; 32]).unwrap();
    assert_eq!((&*recalled.0, recalled.1), (&[2u8, 2][..], true));
    assert!(compiler.serialized(&[3; 32]).is_none());
    assert!(compiler.report().ends_with("recalled=0"));
}

#[test]
fn repeated_llvm_deferral_never_waits_inside_a_service_turn() {
    fn must_not_wait() {
        panic!("a deferred service turn waited for LLVM");
    }
    let lock = CompileLock::new();
    let held = lock.hold(core::hint::spin_loop);
    let mut waiting = Waiting::default();
    for _ in 0..3 {
        assert!(
            matches!(AirCompiler::llvm(&lock, &mut waiting, true, must_not_wait), Err(error) if error == DEFERRED)
        );
        assert_eq!(waiting.count, 1);
        assert!(waiting.job.is_some());
        assert!(lock.is_held());
    }
    drop(held);
    assert!(AirCompiler::llvm(&lock, &mut waiting, true, must_not_wait).is_ok());
}
