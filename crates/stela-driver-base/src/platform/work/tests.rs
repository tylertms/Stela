//! Queue admission, ticket identity and hosted continuation fixtures.
extern crate std;
use super::*;
use std::thread;
struct CpuModel {
    thread: thread::ThreadId,
    flags: AtomicU64,
    irq: AtomicBool,
    time: AtomicU64,
}
impl CpuModel {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            thread: thread::current().id(),
            flags: AtomicU64::new(0x202),
            irq: AtomicBool::new(false),
            time: AtomicU64::new(0),
        })
    }
}
// SAFETY: the private hosted fixture checks its creating ThreadId before queue
// access. Atomics model IF/IRQ state and ticks advance during calls. Context
// switches use OS-owned guarded stacks through the hosted test constructor.
unsafe impl Cpu for CpuModel {
    fn check_cpu(&self) -> Result<(), WorkError> {
        if thread::current().id() == self.thread {
            Ok(())
        } else {
            Err(WorkError::WrongCpu)
        }
    }
    fn in_interrupt(&self) -> bool {
        self.irq.load(Ordering::Acquire)
    }
    fn save_disable(&self) -> Result<u64, WorkError> {
        self.check_cpu()?;
        Ok(self.flags.swap(2, Ordering::AcqRel))
    }
    unsafe fn restore(&self, flags: u64) {
        self.flags.store(flags, Ordering::Release);
    }
    fn ticks(&self) -> u64 {
        self.time.fetch_add(1, Ordering::Relaxed)
    }
    fn fatal(&self, e: WorkError) -> ! {
        panic!("fatal context {e:?}")
    }
}
struct Owner(Arc<AtomicU64>);
impl Drop for Owner {
    fn drop(&mut self) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}
unsafe extern "C" fn noop(_: *mut c_void) {}
#[test]
fn admission_bounds_and_flag_restore() {
    let cpu = CpuModel::new();
    let drops = Arc::new(AtomicU64::new(0));
    // SAFETY: CpuModel binds this queue to the current host thread. noop is static
    // code and ignores its null context; the owner covers its full lifetime.
    let queue = unsafe { Queue::<2>::new(cpu.clone(), Arc::new(Owner(drops.clone()))) }.unwrap();
    cpu.irq.store(true, Ordering::Release);
    // SAFETY: noop is static and never dereferences its null context. Queue Arcs
    // and CpuModel remain held on this host thread for the admitted jobs.
    let a = unsafe { queue.execute(Kind::Notify, noop, core::ptr::null_mut()) }.unwrap();
    // SAFETY: noop is static and never dereferences its null context. Queue Arcs
    // and CpuModel remain held on this host thread for the admitted jobs.
    let b = unsafe { queue.execute(Kind::General, noop, core::ptr::null_mut()) }.unwrap();
    assert_eq!(
        // SAFETY: noop is static and never dereferences its null context. Queue Arcs
        // and CpuModel remain held on this host thread for the admitted jobs.
        unsafe { queue.execute(Kind::General, noop, core::ptr::null_mut()) },
        Err(WorkError::Full)
    );
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
    cpu.flags.store(2, Ordering::Release);
    queue.close().unwrap();
    assert_eq!(cpu.flags.load(Ordering::Acquire), 2);
    assert!(!queue.completed(a).unwrap());
    assert!(!queue.completed(b).unwrap());
    assert_eq!(
        // SAFETY: noop is static and never dereferences its null context. Queue Arcs
        // and CpuModel remain held on this host thread for the admitted jobs.
        unsafe { queue.execute(Kind::General, noop, core::ptr::null_mut()) },
        Err(WorkError::Closed)
    );
    cpu.irq.store(false, Ordering::Release);
    // SAFETY: No asynchronous producers or active executor exist in this queue-only
    // fixture. Pending jobs keep their owners; an empty closed queue can retire.
    assert_eq!(unsafe { queue.retire() }, Err(WorkError::Pending));
    let foreign = queue.clone();
    assert_eq!(
        thread::spawn(move || foreign.close()).join().unwrap(),
        Err(WorkError::WrongCpu)
    );
    let count = quarantined_owners();
    drop(queue);
    assert!(quarantined_owners() >= count.saturating_add(1));
    assert_eq!(drops.load(Ordering::Relaxed), 0);
}
#[test]
fn ticket_identity_and_sequence_exhaustion() {
    let cpu = CpuModel::new();
    let owner: RetainedOwner = Arc::new(());
    // SAFETY: CpuModel binds this queue to the current host thread. noop is static
    // code and ignores its null context; the owner covers its full lifetime.
    let a = unsafe { Queue::<2>::new(cpu.clone(), owner.clone()) }.unwrap();
    // SAFETY: CpuModel binds this queue to the current host thread. noop is static
    // code and ignores its null context; the owner covers its full lifetime.
    let b = unsafe { Queue::<2>::new(cpu, owner) }.unwrap();
    // SAFETY: noop is static and never dereferences its null context. Queue Arcs
    // and CpuModel remain held on this host thread for the admitted jobs.
    let ticket = unsafe { a.execute(Kind::General, noop, core::ptr::null_mut()) }.unwrap();
    assert_eq!(b.completed(ticket), Err(WorkError::InvalidTicket));
    {
        let mut g = b.lock().unwrap();
        g.state().next = u64::MAX;
    }
    assert_eq!(
        // SAFETY: noop is static and never dereferences its null context. Queue Arcs
        // and CpuModel remain held on this host thread for the admitted jobs.
        unsafe { b.execute(Kind::General, noop, core::ptr::null_mut()) },
        Err(WorkError::Exhausted)
    );
    assert!(b.lock().unwrap().state().jobs.iter().all(Option::is_none));
    b.close().unwrap();
    // SAFETY: No asynchronous producers or active executor exist in this queue-only
    // fixture. Pending jobs keep their owners; an empty closed queue can retire.
    unsafe { b.retire() }.unwrap();
    drop((a, b));
}

// Guard-paged stacks come from the host's POSIX mmap.
#[cfg(all(target_arch = "x86_64", unix))]
mod switched;

#[cfg(all(target_arch = "x86_64", unix))]
mod native_policy;

#[cfg(all(target_arch = "x86_64", unix))]
mod general_credits;

#[test]
fn a_corrupt_context_names_the_line_that_refused() {
    use std::format;
    let site = core::panic::Location::caller();
    assert_eq!(
        named_site(WorkError::CorruptContext, Some(site)),
        Some(site)
    );
    assert_eq!(named_site(WorkError::Busy, Some(site)), None);
    assert_eq!(named_site(WorkError::CorruptContext, None), None);
    // Other tests refuse concurrently: some site is recorded, maybe theirs.
    assert_eq!(WorkError::corrupt(), WorkError::CorruptContext);
    let recorded = corrupt_site().expect("a refusal records its site");
    let suffix = |text: std::string::String, account: &str| {
        let rest = text.strip_prefix(account).expect("the account comes first");
        assert!(rest.starts_with(" at ") && rest.contains(".rs:"), "{text}");
    };
    suffix(format!("{:?}", WorkError::CorruptContext), "CorruptContext");
    suffix(
        format!("{}", WorkError::CorruptContext),
        "a work context is corrupt",
    );
    assert!(recorded.line() > 0);
    // Every other refusal reads as before.
    assert_eq!(format!("{:?}", WorkError::Busy), "Busy");
    assert_eq!(format!("{}", WorkError::TimedOut), "the work timed out");
}

fn cancellation_error<T>(result: Result<(), (T, WorkError)>) -> WorkError {
    match result {
        Err((owner, error)) => {
            drop(owner);
            error
        }
        Ok(()) => panic!("expected exact native cancellation refusal"),
    }
}

#[cfg(all(target_arch = "x86_64", unix))]
mod accounting;
