//! The ACPICA fixture platform other drivers' ACPI tests share.
extern crate std;
use super::*;
use std::{
    alloc::{Layout, alloc, dealloc},
    collections::{BTreeMap, VecDeque},
    sync::{
        Arc, Mutex,
        atomic::{AtomicU32, AtomicU64, AtomicUsize},
    },
    time::{Duration, Instant},
};

pub static TEST_SERIAL: Mutex<()> = Mutex::new(());

pub(crate) struct Semaphore {
    pub(crate) units: AtomicU32,
    pub(crate) max: u32,
}
#[derive(Default)]
pub(crate) struct Counts {
    pub(crate) allocations: AtomicUsize,
    pub(crate) mappings: AtomicUsize,
    pub(crate) destroyed: AtomicBool,
    pub(crate) memory: AtomicU64,
    pub(crate) memory_accesses: AtomicUsize,
    pub(crate) port_writes: AtomicUsize,
    pub(crate) preflight_status: AtomicU32,
    pub(crate) flush_status: AtomicU32,
    pub(crate) flush_calls: AtomicUsize,
    pub(crate) ledger: Mutex<BTreeMap<usize, usize>>,
}
pub(crate) struct TestPlatform {
    pub(crate) tables: Vec<(u64, Vec<u8>)>,
    pub(crate) leases: Mutex<BTreeMap<(usize, u64), usize>>,
    pub(crate) work: Mutex<VecDeque<(Work, usize)>>,
    pub(crate) counts: Arc<Counts>,
    pub(crate) origin: Instant,
    pub(crate) quiescing: AtomicBool,
    pub(crate) fail_quiesce: AtomicBool,
}
pub(crate) unsafe fn platform<'a>(p: *mut c_void) -> &'a TestPlatform {
    // SAFETY: callers pass the provider context, a live `TestPlatform` until `destroy`.
    unsafe { &*p.cast::<TestPlatform>() }
}
pub(crate) unsafe extern "C" fn allocate(p: *mut c_void, bytes: u64) -> *mut c_void {
    let Ok(bytes) = usize::try_from(bytes) else {
        return ptr::null_mut();
    };
    let Some(total) = bytes.checked_add(16) else {
        return ptr::null_mut();
    };
    let Ok(layout) = Layout::from_size_align(total, 16) else {
        return ptr::null_mut();
    };
    // SAFETY: `layout` has a nonzero size.
    let raw = unsafe { alloc(layout) };
    if raw.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: `raw` is a fresh 16-aligned block of at least 16 bytes.
    unsafe {
        raw.cast::<usize>().write(total);
    }
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .allocations
        .fetch_add(1, Ordering::SeqCst);
    // SAFETY: the block spans `16 + bytes` bytes.
    let result = unsafe { raw.add(16).cast() };
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .ledger
        .lock()
        .unwrap()
        .insert(result as usize, bytes);
    result
}
pub(crate) unsafe extern "C" fn free(p: *mut c_void, allocation: *mut c_void) {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .ledger
        .lock()
        .unwrap()
        .remove(&(allocation as usize))
        .unwrap();
    // SAFETY: `allocation` came from `allocate`, 16 bytes past its block's start.
    let raw = unsafe { allocation.cast::<u8>().sub(16) };
    // SAFETY: `allocate` stored the block's size in its first word.
    let total = unsafe { raw.cast::<usize>().read() };
    // SAFETY: `raw` came from `alloc` with this size and alignment and is freed once.
    unsafe { dealloc(raw, Layout::from_size_align(total, 16).unwrap()) };
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .allocations
        .fetch_sub(1, Ordering::SeqCst);
}
pub(crate) unsafe extern "C" fn map(p: *mut c_void, address: u64, length: u64) -> *mut c_void {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    let platform = unsafe { platform(p) };
    let Some(end) = address.checked_add(length) else {
        return ptr::null_mut();
    };
    let mapped = if address >= 0x9000 && end <= 0x9008 {
        // SAFETY: the address check keeps the offset within the eight-byte memory word.
        unsafe {
            platform
                .counts
                .memory
                .as_ptr()
                .cast::<u8>()
                .add((address - 0x9000) as usize)
        }
        .cast()
    } else {
        let Some((base, bytes)) = platform
            .tables
            .iter()
            .find(|(base, bytes)| *base <= address && end <= *base + bytes.len() as u64)
        else {
            return ptr::null_mut();
        };
        // SAFETY: the table lookup keeps the mapped range within `bytes`.
        (unsafe { bytes.as_ptr().add((address - base) as usize) }) as *mut c_void
    };
    *platform
        .leases
        .lock()
        .unwrap()
        .entry((mapped as usize, length))
        .or_default() += 1;
    platform.counts.mappings.fetch_add(1, Ordering::SeqCst);
    mapped
}
pub(crate) unsafe extern "C" fn unmap(p: *mut c_void, pointer: *mut c_void, length: u64) {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    let platform = unsafe { platform(p) };
    let mut leases = platform.leases.lock().unwrap();
    let entry = leases
        .get_mut(&(pointer as usize, length))
        .expect("exact mapping lease");
    *entry -= 1;
    if *entry == 0 {
        leases.remove(&(pointer as usize, length));
    }
    platform.counts.mappings.fetch_sub(1, Ordering::SeqCst);
}
pub(crate) unsafe extern "C" fn create_lock(_: *mut c_void, out: *mut *mut c_void) -> Status {
    // SAFETY: ACPICA passes a valid out pointer.
    unsafe { out.write(Box::into_raw(Box::new(AtomicBool::new(false))).cast()) };
    0
}
pub(crate) unsafe extern "C" fn delete_lock(_: *mut c_void, lock: *mut c_void) {
    // SAFETY: `lock` came from `create_lock` and ACPICA deletes it once.
    unsafe {
        drop(Box::from_raw(lock.cast::<AtomicBool>()));
    }
}
pub(crate) unsafe extern "C" fn acquire_lock(_: *mut c_void, lock: *mut c_void) -> u64 {
    // SAFETY: `lock` came from `create_lock` and lives until `delete_lock`.
    let lock = unsafe { &*lock.cast::<AtomicBool>() };
    while lock
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        core::hint::spin_loop();
    }
    0 // Hosted fixture has no hardware interrupts; never advertises SCI capability.
}
pub(crate) unsafe extern "C" fn release_lock(_: *mut c_void, lock: *mut c_void, flags: u64) {
    assert_eq!(flags, 0);
    // SAFETY: `lock` came from `create_lock` and lives until `delete_lock`.
    unsafe { &*lock.cast::<AtomicBool>() }.store(false, Ordering::Release);
}
pub(crate) unsafe extern "C" fn create_semaphore(
    _: *mut c_void,
    max: u32,
    initial: u32,
    out: *mut *mut c_void,
) -> Status {
    if initial > max || max == 0 {
        return BAD_PARAMETER.0;
    }
    // SAFETY: ACPICA passes a valid out pointer.
    unsafe {
        out.write(
            Box::into_raw(Box::new(Semaphore {
                units: AtomicU32::new(initial),
                max,
            }))
            .cast(),
        )
    };
    0
}
pub(crate) unsafe extern "C" fn delete_semaphore(_: *mut c_void, s: *mut c_void) -> Status {
    // SAFETY: `s` came from `create_semaphore` and ACPICA deletes it once.
    unsafe {
        drop(Box::from_raw(s.cast::<Semaphore>()));
    }
    0
}
pub(crate) unsafe extern "C" fn wait_semaphore(
    _: *mut c_void,
    s: *mut c_void,
    units: u32,
    timeout: u16,
) -> Status {
    // SAFETY: `s` came from `create_semaphore` and lives until `delete_semaphore`.
    let s = unsafe { &*s.cast::<Semaphore>() };
    let start = Instant::now();
    if units == 0 || units > s.max {
        return BAD_PARAMETER.0;
    }
    loop {
        let current = s.units.load(Ordering::Acquire);
        if current >= units
            && s.units
                .compare_exchange_weak(
                    current,
                    current - units,
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
        {
            return 0;
        }
        if timeout != u16::MAX && start.elapsed() >= Duration::from_millis(u64::from(timeout)) {
            return 0x11;
        }
        std::thread::yield_now();
    }
}
pub(crate) unsafe extern "C" fn signal_semaphore(
    _: *mut c_void,
    s: *mut c_void,
    units: u32,
) -> Status {
    // SAFETY: `s` came from `create_semaphore` and lives until `delete_semaphore`.
    let s = unsafe { &*s.cast::<Semaphore>() };
    let mut current = s.units.load(Ordering::Acquire);
    loop {
        let Some(next) = current.checked_add(units).filter(|n| *n <= s.max) else {
            return LIMIT.0;
        };
        match s
            .units
            .compare_exchange_weak(current, next, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => return 0,
            Err(n) => current = n,
        }
    }
}
pub(crate) unsafe extern "C" fn install_interrupt(
    _: *mut c_void,
    _: u32,
    _: Interrupt,
    _: *mut c_void,
) -> Status {
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn remove_interrupt(_: *mut c_void, _: u32, _: Interrupt) -> Status {
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn thread_id(_: *mut c_void) -> u64 {
    std::thread_local! { static ID: u64 = { static NEXT: AtomicUsize = AtomicUsize::new(1); NEXT.fetch_add(1,Ordering::Relaxed) as u64 }; }
    ID.with(|id| *id)
}
pub(crate) unsafe extern "C" fn execute(
    p: *mut c_void,
    _: u32,
    callback: Work,
    context: *mut c_void,
) -> Status {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    if unsafe { platform(p) }.quiescing.load(Ordering::Acquire) {
        return 6;
    }
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .work
        .lock()
        .unwrap()
        .push_back((callback, context as usize));
    0
}
pub(crate) unsafe extern "C" fn drain_work(p: *mut c_void) -> Status {
    // SAFETY: this fixture keeps its TestPlatform until core shutdown.
    let state = unsafe { platform(p) };
    state.counts.flush_calls.fetch_add(1, Ordering::SeqCst);
    let failure = state.counts.flush_status.load(Ordering::Acquire);
    if failure != 0 {
        return failure;
    }
    loop {
        // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until
        // `destroy`.
        let next = unsafe { platform(p) }.work.lock().unwrap().pop_front();
        let Some((callback, context)) = next else {
            return 0;
        };
        // SAFETY: ACPICA queued this callback with its own context.
        unsafe { callback(context as *mut c_void) };
    }
}
pub(crate) unsafe extern "C" fn sleep(_: *mut c_void, ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}
pub(crate) unsafe extern "C" fn stall(_: *mut c_void, us: u32) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_micros(us.into()) {
        core::hint::spin_loop();
    }
}
pub(crate) unsafe extern "C" fn read_port(_: *mut c_void, _: u64, _: *mut u32, _: u32) -> Status {
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn write_port(p: *mut c_void, _: u64, _: u32, _: u32) -> Status {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .port_writes
        .fetch_add(1, Ordering::SeqCst);
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn read_memory(
    p: *mut c_void,
    address: u64,
    value: *mut u64,
    width: u32,
) -> Status {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .memory_accesses
        .fetch_add(1, Ordering::SeqCst);
    if address != 0x9000 || width != 32 {
        return SUPPORT.0;
    }
    // SAFETY: ACPICA passes a valid out pointer and the provider context, a live
    // `TestPlatform`.
    unsafe { value.write(platform(p).counts.memory.load(Ordering::Acquire)) };
    0
}
pub(crate) unsafe extern "C" fn write_memory(
    p: *mut c_void,
    address: u64,
    value: u64,
    width: u32,
) -> Status {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .memory_accesses
        .fetch_add(1, Ordering::SeqCst);
    if address != 0x9000 || width != 32 {
        return SUPPORT.0;
    }
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .counts
        .memory
        .store(value, Ordering::Release);
    0
}
pub(crate) unsafe extern "C" fn read_pci(
    _: *mut c_void,
    _: *mut PciId,
    _: u32,
    _: *mut u64,
    _: u32,
) -> Status {
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn write_pci(
    _: *mut c_void,
    _: *mut PciId,
    _: u32,
    _: u64,
    _: u32,
) -> Status {
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn timer(p: *mut c_void) -> u64 {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }.origin.elapsed().as_nanos() as u64 / 100
}
pub(crate) unsafe extern "C" fn signal(_: *mut c_void, _: u32, _: *mut c_void) -> Status {
    SUPPORT.0
}
pub(crate) unsafe extern "C" fn log(_: *mut c_void, text: *const c_char, length: u64) {
    // SAFETY: ACPICA passes `length` valid bytes at `text`.
    let bytes = unsafe { core::slice::from_raw_parts(text.cast::<u8>(), length as usize) };
    std::eprint!("{}", std::string::String::from_utf8_lossy(bytes));
}
pub(crate) unsafe extern "C" fn flush_cache(_: *mut c_void) {
    panic!("no hardware cache flush permitted by fixture");
}
pub(crate) unsafe extern "C" fn quiesce(p: *mut c_void) -> Status {
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    if unsafe { platform(p) }.fail_quiesce.load(Ordering::Acquire) {
        return SUPPORT.0;
    }
    // SAFETY: ACPICA passes back the provider context, a live `TestPlatform` until `destroy`.
    unsafe { platform(p) }
        .quiescing
        .store(true, Ordering::Release);
    // SAFETY: `p` is this provider's context.
    unsafe { drain_work(p) }
}
pub(crate) unsafe extern "C" fn preflight_wait_events(p: *mut c_void) -> Status {
    // SAFETY: the singleton fixture owns this live provider context.
    unsafe { platform(p) }
        .counts
        .preflight_status
        .load(Ordering::Acquire)
}
pub(crate) unsafe extern "C" fn destroy(p: *mut c_void) {
    // SAFETY: `p` is the boxed `TestPlatform` the fixture leaked into the provider; ACPICA
    // destroys it once.
    let p = unsafe { Box::from_raw(p.cast::<TestPlatform>()) };
    p.counts.destroyed.store(true, Ordering::SeqCst);
}

pub(crate) unsafe extern "C" fn fatal(_: *mut c_void, status: Status) -> ! {
    std::eprintln!("native ACPI callback completion fatal: {status:#x}");
    // SAFETY: this fatal callback is entered by the same live singleton's C
    // bridge after it poisons namespace access, before any provider teardown.
    assert_eq!(unsafe { stela_acpi_namespace_valid() }, 0);
    std::process::exit(86)
}
pub(crate) fn checksum(bytes: &mut [u8], offset: usize) {
    bytes[offset] = 0;
    bytes[offset] = 0u8.wrapping_sub(bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)));
}
pub(crate) fn table(signature: &[u8; 4], length: usize, revision: u8) -> Vec<u8> {
    let mut bytes = vec![0; length];
    bytes[..4].copy_from_slice(signature);
    bytes[4..8].copy_from_slice(&(length as u32).to_le_bytes());
    bytes[8] = revision;
    bytes[10..16].copy_from_slice(b"STELA ");
    bytes[16..24].copy_from_slice(b"AMLTEST ");
    bytes
}
pub(crate) fn fixture_dsdt(dsdt: Vec<u8>) -> (Services, Arc<Counts>) {
    fixture_tables(dsdt, vec![])
}
/// The fixture platform with `tables` listed in the XSDT after the FADT.
pub(crate) fn fixture_tables(dsdt: Vec<u8>, extra: Vec<Vec<u8>>) -> (Services, Arc<Counts>) {
    let counts = Arc::new(Counts::default());
    let mut rsdp = vec![0; 36];
    rsdp[..8].copy_from_slice(b"RSD PTR ");
    rsdp[9..15].copy_from_slice(b"STELA ");
    rsdp[15] = 2;
    rsdp[20..24].copy_from_slice(&36u32.to_le_bytes());
    rsdp[24..32].copy_from_slice(&0x2000u64.to_le_bytes());
    checksum(&mut rsdp[..20], 8);
    checksum(&mut rsdp, 32);
    let mut xsdt = table(b"XSDT", 44 + 8 * extra.len(), 1);
    xsdt[36..44].copy_from_slice(&0x3000u64.to_le_bytes());
    for index in 0..extra.len() {
        let address = 0x5000 + 0x1000 * index as u64;
        xsdt[44 + 8 * index..52 + 8 * index].copy_from_slice(&address.to_le_bytes());
    }
    checksum(&mut xsdt, 9);
    let mut fadt = table(b"FACP", 276, 6);
    fadt[56..60].copy_from_slice(&0x1000u32.to_le_bytes());
    fadt[64..68].copy_from_slice(&0x1004u32.to_le_bytes());
    fadt[88] = 4;
    fadt[89] = 2;
    fadt[46..48].copy_from_slice(&9u16.to_le_bytes());
    fadt[140..148].copy_from_slice(&0x4000u64.to_le_bytes());
    checksum(&mut fadt, 9);
    let context = Box::into_raw(Box::new(TestPlatform {
        tables: [
            (0x1000, rsdp),
            (0x2000, xsdt),
            (0x3000, fadt),
            (0x4000, dsdt),
        ]
        .into_iter()
        .chain((0x5000..).step_by(0x1000).zip(extra))
        .collect(),
        leases: Mutex::new(BTreeMap::new()),
        work: Mutex::new(VecDeque::new()),
        counts: counts.clone(),
        origin: Instant::now(),
        quiescing: AtomicBool::new(false),
        fail_quiesce: AtomicBool::new(false),
    }))
    .cast();
    (
        Services {
            abi_version: SERVICES_ABI_VERSION,
            abi_bytes: SERVICES_ABI_BYTES,
            context,
            root: 0x1000,
            capabilities: 0,
            allocate,
            free,
            map,
            unmap,
            create_lock,
            delete_lock,
            acquire_lock,
            release_lock,
            create_semaphore,
            delete_semaphore,
            wait_semaphore,
            signal_semaphore,
            install_interrupt,
            remove_interrupt,
            thread_id,
            execute,
            drain_work,
            sleep,
            stall,
            read_port,
            write_port,
            read_memory,
            write_memory,
            read_pci,
            write_pci,
            timer,
            signal,
            log,
            flush_cache,
            quiesce,
            destroy,
            fatal,
            preflight_wait_events,
        },
        counts,
    )
}

/// Reuse the ACPICA fixture with a reduced-hardware FADT. Hardware
/// mode then executes C activation and AML _INI without inventing
/// legacy SCI/PM hardware. Extra AML methods are part of the real DSDT.
pub fn reduced_hardware_fixture(extra_aml: &[u8]) -> Services {
    let mut dsdt = include_bytes!("fixtures/namespace.aml").to_vec();
    dsdt.extend_from_slice(extra_aml);
    let length = dsdt.len() as u32;
    dsdt[4..8].copy_from_slice(&length.to_le_bytes());
    checksum(&mut dsdt, 9);
    let (mut services, _) = fixture_dsdt(dsdt);
    // SAFETY: fixture owns this unpublished provider; ACPICA has not started.
    let provider = unsafe { &mut *services.context.cast::<TestPlatform>() };
    let fadt = &mut provider
        .tables
        .iter_mut()
        .find(|(p, _)| *p == 0x3000)
        .unwrap()
        .1;
    fadt[112..116].copy_from_slice(&(1u32 << 20).to_le_bytes());
    checksum(fadt, 9);
    services.capabilities = FULL_CAPABILITIES;
    services
}
