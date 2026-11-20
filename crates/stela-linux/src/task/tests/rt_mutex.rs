//! FIFO inheritance, chain propagation, cancellation and exit normalization
//! through original C RT mutexes and the real shared scheduling sidecar.
use super::*;

struct ServiceRoot {
    fixture: Arc<MutexFixture>,
    domain: Arc<service::Domain<16>>,
}

unsafe extern "C" fn register_service(context: *mut c_void) {
    // SAFETY: this stable box is kept by rt_mutex_child through service join.
    let root = unsafe { &*context.cast::<ServiceRoot>() };
    let _binding = root.fixture.binding(4);
    // SAFETY: the executor remains pinned on this checked test CPU.
    root.domain
        .bind_current(unsafe { root.fixture.runtime() })
        .unwrap();
}

/// The service here binds and returns, so nothing can answer a tick: this clock
/// stays put, however long a loaded host takes over the scenarios.
struct FrozenClock {
    cpu: Arc<dyn Cpu>,
}
// SAFETY: a constant is monotonic; no native IRQ attestation.
unsafe impl service::Clock for FrozenClock {
    fn cpu(&self) -> &Arc<dyn Cpu> {
        &self.cpu
    }
    fn now_ns(&self) -> Result<u64, WorkError> {
        self.cpu.check_cpu()?;
        Ok(0)
    }
}

#[test]
#[ignore = "isolation: child"]
fn rt_mutex_child() {
    // SAFETY: an isolated single-test child owns the canonical allocator and
    // all exact-layout native allocations until their checked retirement.
    assert!(crate::test_heap::register());
    let dropped = Arc::new(AtomicU64::new(0));
    for scenario in 11..=14 {
        let cpu = CpuModel::new();
        let f = MutexFixture::new(cpu.clone(), scenario);
        // SAFETY: this child is the sole producer on the CPU the model checks.
        let queue = unsafe { Queue::<16>::new(cpu.clone(), f.clone()) }.unwrap();
        let executor =
            Executor::new_hosted_service_for_test(queue, stacks_count(&dropped, THREAD, 6))
                .unwrap();
        let runtime = executor.runtime();
        f.runtime.store(
            ptr::from_ref(runtime.get_ref()).cast_mut(),
            Ordering::Release,
        );
        let clock: Arc<dyn service::Clock> = Arc::new(FrozenClock { cpu: cpu.clone() });
        let idle = Arc::new(ServiceIdle {
            clock: clock.clone(),
            owner: thread::current(),
            waits: AtomicU64::new(0),
        });
        let domain = service::Domain::new(clock, idle).unwrap();
        runtime.attach_service_domain(domain.clone()).unwrap();
        let mut root = Box::new(ServiceRoot {
            fixture: f.clone(),
            domain: domain.clone(),
        });
        // SAFETY: no C producer/storage is installed by this scheduler-only
        // service. The box, domain and bindings outlive its joined callback.
        let service_ticket =
            unsafe { runtime.start_service(register_service, ptr::from_mut(&mut *root).cast()) }
                .unwrap();
        runtime.drain(10000).unwrap();
        assert!(runtime.queue().completed(service_ticket).unwrap());
        // SAFETY: all role contexts and original C locks/waiters stay live
        // through every genuine suspension until the foreground joins peers.
        unsafe { runtime.call_foreground(mutex_role, f.context(0), 100000) }.unwrap();
        runtime.drain(10000).unwrap();
        assert_eq!(
            f.completed.load(Ordering::Relaxed),
            if scenario == 12 {
                4
            } else if scenario == 11 {
                3
            } else {
                2
            }
        );
        for slot in &f.targets {
            if let Some(target) = slot.lock().unwrap().take() {
                assert_eq!(target.wake.wake(), Err(WorkError::Closed));
                drop(target);
            }
        }
        domain.stop_producers().unwrap();
        // SAFETY: the reserved service completed, and the original C roles
        // joined with no PI donor, waiter, timer, IRQ or native producer left.
        unsafe { domain.retire() }.unwrap();
        runtime.queue().close().unwrap();
        for slot in &f.bindings {
            if let Some(binding) = slot.lock().unwrap().take() {
                // SAFETY: every real job completed and all targets are closed;
                // C verified empty PI trees and reset rank before completion.
                drop(unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap());
                // SAFETY: detach and the joins above prove foreign quiescence.
                unsafe { binding.release_quiescent() }
                    .unwrap_or_else(|_| panic!("RT binding was retained"));
            }
        }
        // SAFETY: all contexts, producers and C bindings have been retired.
        unsafe { runtime.retire() }.unwrap();
        drop(executor);
        assert_eq!(
            dropped.load(Ordering::Relaxed),
            u64::from(scenario - 10) * 6
        );
        assert_eq!(quarantined(), 0);
        assert!(!crate::test_heap::span_occupied());
    }
}
