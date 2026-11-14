extern crate std;

use super::{Memory, MockBackend, MockCompiler, Stream, commit, connect, service};
use crate::metal::service::shared::{LIBRARY_PIECE_BYTES, UPKEEP_PIECES};
use crate::metal::{
    capabilities::block::Block,
    service::{
        GuestMemory, Served, ServiceHost, SharedService,
        wire::{self, GateCall, NegotiateRequest, NegotiateResponse, Operation, Span, Status},
    },
};
use core::{
    mem::size_of,
    num::NonZeroU64,
    sync::atomic::{AtomicU32, AtomicUsize, Ordering},
};
use stela_contracts::gate::{CallerFault, CallerMemory, GateService};

impl CallerMemory for Memory {
    fn read(&mut self, address: u64, buffer: &mut [u8]) -> Result<(), CallerFault> {
        let span = Span {
            address,
            length: buffer.len() as u64,
        };
        // Copied in place: a read allocates nothing of its length.
        let range = self
            .range(span)
            .map_err(|_| CallerFault::Denied { address })?;
        self.longest = self.longest.max(span.length);
        buffer.copy_from_slice(&self.bytes[range]);
        Ok(())
    }

    fn write(&mut self, address: u64, data: &[u8]) -> Result<(), CallerFault> {
        let span = Span {
            address,
            length: data.len() as u64,
        };
        GuestMemory::write(self, span, data).map_err(|_| CallerFault::Denied { address })
    }
}

/// Maps a [`GateCall`] for `operation` with `request`, room for a
/// response of `response` bytes and a 64-byte reason.
fn gate_call(memory: &mut Memory, operation: Operation, request: &[u8], response: usize) -> u64 {
    let call = GateCall {
        operation: operation.raw(),
        status: u32::MAX,
        request: memory.put(request),
        response: memory.put(&alloc::vec![0; response]),
        reason: memory.put(&[0; 64]),
        reason_length: 0,
    };
    memory.records(&[call]).address
}

fn completed(memory: &mut Memory, record: u64) -> GateCall {
    let span = Span {
        address: record,
        length: size_of::<GateCall>() as u64,
    };
    wire::read(&GuestMemory::read(memory, span).unwrap()).unwrap()
}

#[test]
fn a_gate_call_completes_when_the_request_and_response_overlap() {
    let mut service = service();
    let connection = service.connect().unwrap();
    let mut memory = Memory::default();
    let mut buffer = [0xa5; size_of::<NegotiateResponse>()];
    let negotiate = NegotiateRequest {
        major: wire::MAJOR,
        minor: wire::MINOR,
    };
    let request_bytes = size_of::<NegotiateRequest>();
    wire::write(&negotiate, &mut buffer[..request_bytes]).unwrap();
    let response = memory.put(&buffer);
    let reason = memory.put(&[0; 64]);
    let record = memory
        .records(&[GateCall {
            operation: Operation::Negotiate.raw(),
            status: u32::MAX,
            request: Span {
                address: response.address,
                length: request_bytes as u64,
            },
            response,
            reason,
            reason_length: 0,
        }])
        .address;
    service.serve(connection, record, &mut memory).unwrap();
    let call = completed(&mut memory, record);
    assert_eq!((call.status, call.reason_length), (Status::Ok.raw(), 0));
    let response: NegotiateResponse =
        wire::read(&GuestMemory::read(&mut memory, call.response).unwrap()).unwrap();
    assert_eq!(response.major, wire::MAJOR);
    assert_eq!(response.reserved, 0);
}

#[test]
fn a_failed_gate_call_reports_its_status_and_reason() {
    let mut service = service();
    let connection = service.connect().unwrap();
    let mut memory = Memory::default();
    let oversized = gate_call(
        &mut memory,
        Operation::Negotiate,
        &[0; size_of::<Block>() + 1],
        16,
    );
    service.serve(connection, oversized, &mut memory).unwrap();
    let call = completed(&mut memory, oversized);
    assert_eq!(call.status, Status::Malformed.raw());
    let reason = GuestMemory::read(&mut memory, call.reason).unwrap();
    assert_eq!(
        &reason[..call.reason_length as usize],
        alloc::format!("record length {}", size_of::<Block>() + 1).as_bytes()
    );
    // The platform's line names the operation too.
    assert_eq!(
        core::mem::take(&mut service.refused),
        [alloc::format!(
            "record length {} in operation {}",
            size_of::<Block>() + 1,
            Operation::Negotiate.raw()
        )]
    );
    assert_eq!(
        service.serve(connection, 0x10, &mut memory),
        Err(CallerFault::Denied { address: 0x10 })
    );
}

#[test]
fn the_capability_block_crosses_the_gate() {
    let mut service = service();
    let connection = connect(&mut service);
    let mut memory = Memory::default();
    let record = gate_call(
        &mut memory,
        Operation::Capabilities,
        &[],
        size_of::<Block>(),
    );
    service.serve(connection, record, &mut memory).unwrap();
    assert_eq!(completed(&mut memory, record).status, Status::Ok.raw());
}

#[test]
fn a_span_the_caller_lost_asks_it_to_call_again() {
    let mut service = service();
    let connection = service.connect().unwrap();
    let mut memory = Memory::default();
    let reason = memory.put(&[0; 64]);
    let response = memory.put(&[0; size_of::<NegotiateResponse>()]);
    let record = memory
        .records(&[GateCall {
            operation: Operation::Negotiate.raw(),
            status: u32::MAX,
            request: Span {
                address: 0x10,
                length: size_of::<NegotiateRequest>() as u64,
            },
            response,
            reason,
            reason_length: 0,
        }])
        .address;
    service.serve(connection, record, &mut memory).unwrap();
    assert_eq!(
        completed(&mut memory, record).status,
        Status::Unmapped.raw()
    );
}

#[test]
fn the_gate_signals_completed_work_once() {
    let mut service = service();
    let connection = connect(&mut service);
    assert!(!service.signaled());
    commit(&mut service, connection, &Stream::default()).unwrap();
    assert!(!service.signaled());
    let backend = service.device().backend();
    let timeline = *backend.semaphores.keys().next().unwrap();
    backend.semaphores.insert(timeline, 1);
    assert!(service.signaled());
    assert!(!service.signaled());
}

type Shared = SharedService<MockBackend, MockCompiler>;

fn commit_shared(shared: &Shared, connection: NonZeroU64) {
    commit(
        &mut shared.try_turn().unwrap(),
        connection,
        &Stream::default(),
    )
    .unwrap();
}

const HOST: ServiceHost = ServiceHost {
    relax: || panic!("nothing waits"),
    relax_within: || panic!("nothing waits"),
    stack: 0x4000,
    run: |_, call| call(),
    report: |_| {},
    note: |_| {},
    ticks: || 0,
    served: |_| {},
    wake: || panic!("no turn kept an interrupt from looking"),
    bundle: |_| panic!("no kext reports"),
};

/// The GPU finishes what was committed.
fn complete(shared: &Shared) {
    let mut turn = shared.try_turn().unwrap();
    let backend = turn.device().backend();
    let timeline = *backend.semaphores.keys().next().unwrap();
    backend.semaphores.insert(timeline, 1);
}

#[test]
fn the_shared_service_finds_completed_work_only_after_an_interrupt() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    commit_shared(&shared, connection);
    assert!(!shared.signaled());
    complete(&shared);
    assert!(!shared.signaled(), "no interrupt said so");
    shared.interrupted();
    assert!(shared.signaled());
    assert!(!shared.signaled());
}

/// A `Poll` call's answered serial.
fn polled(memory: &mut Memory, record: u64) -> u64 {
    let call = completed(memory, record);
    assert_eq!(call.status, Status::Ok.raw());
    let bytes = GuestMemory::read(memory, call.response).unwrap();
    let wire::PollResponse { value, .. } = wire::read(&bytes).unwrap();
    value
}

#[test]
fn a_poll_while_another_call_holds_the_service_answers_what_completed() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    commit_shared(&shared, connection);
    complete(&shared);
    shared.interrupted();
    // The service answers the connection's first poll itself.
    let first = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, first, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, first), 1);
    // `HOST.relax` panics, so a poll that waited for this holder would fail.
    let held = shared.try_turn().unwrap();
    let busy = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, busy, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, busy), 1);
    drop(held);
}

#[test]
fn only_a_connection_the_service_answered_polls_while_it_is_held() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let record = gate_call(&mut memory, Operation::Poll, &[], 16);
    let held = shared.try_turn().unwrap();
    assert!(!shared.poll_published(connection, record, &mut memory));
    assert_eq!(
        completed(&mut memory, record).status,
        u32::MAX,
        "left unanswered"
    );
    drop(held);
    GateService::serve(&shared, connection, record, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, record), 0);
    let held = shared.try_turn().unwrap();
    let again = gate_call(&mut memory, Operation::Commit, &[], 8);
    assert!(
        !shared.poll_published(connection, again, &mut memory),
        "only polls are answered without a turn"
    );
    drop(held);
}

#[test]
fn a_busy_poll_with_a_nonempty_request_takes_the_normal_validation_path() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let first = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, first, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, first), 0);

    let malformed = gate_call(&mut memory, Operation::Poll, &[0], 16);
    let held = shared.try_turn().unwrap();
    assert!(!shared.poll_published(connection, malformed, &mut memory));
    assert_eq!(completed(&mut memory, malformed).status, u32::MAX);
    drop(held);

    GateService::serve(&shared, connection, malformed, &mut memory).unwrap();
    assert_eq!(
        completed(&mut memory, malformed).status,
        Status::Malformed.raw()
    );
}

#[test]
fn a_busy_poll_falls_back_after_the_device_loss_notification() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let first = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, first, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, first), 0);

    shared.try_turn().unwrap().device().mark_lost();
    assert!(shared.signaled());
    let busy = gate_call(&mut memory, Operation::Poll, &[], 16);
    let held = shared.try_turn().unwrap();
    assert!(!shared.poll_published(connection, busy, &mut memory));
    drop(held);

    GateService::serve(&shared, connection, busy, &mut memory).unwrap();
    assert_eq!(
        completed(&mut memory, busy).status,
        Status::DeviceLost.raw()
    );
}

#[test]
fn a_poll_that_cannot_write_its_status_does_not_enroll_the_connection() {
    struct DenyStatus<'a> {
        memory: &'a mut Memory,
        record: u64,
    }
    impl CallerMemory for DenyStatus<'_> {
        fn read(&mut self, address: u64, buffer: &mut [u8]) -> Result<(), CallerFault> {
            CallerMemory::read(self.memory, address, buffer)
        }

        fn write(&mut self, address: u64, data: &[u8]) -> Result<(), CallerFault> {
            if address == self.record {
                return Err(CallerFault::Denied { address });
            }
            CallerMemory::write(self.memory, address, data)
        }
    }

    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let record = gate_call(&mut memory, Operation::Poll, &[], 16);
    let mut caller = DenyStatus {
        memory: &mut memory,
        record,
    };
    assert_eq!(
        GateService::serve(&shared, connection, record, &mut caller),
        Err(CallerFault::Denied { address: record })
    );

    let held = shared.try_turn().unwrap();
    let busy = gate_call(&mut memory, Operation::Poll, &[], 16);
    assert!(!shared.poll_published(connection, busy, &mut memory));
    drop(held);
}

#[test]
fn every_call_runs_where_the_platform_says() {
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    let host = ServiceHost {
        run: |stack, call| {
            assert_eq!(stack, HOST.stack);
            RUNS.fetch_add(1, Ordering::Relaxed);
            call();
        },
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    let connection = shared.connect().unwrap();
    assert_eq!(RUNS.load(Ordering::Relaxed), 1);
    shared.disconnect(connection);
    assert_eq!(RUNS.load(Ordering::Relaxed), 2);
}

#[test]
fn an_interrupt_during_a_call_wakes_the_platform_to_look_once_it_leaves() {
    static WAKES: AtomicUsize = AtomicUsize::new(0);
    let host = ServiceHost {
        wake: || {
            WAKES.fetch_add(1, Ordering::Relaxed);
        },
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    let connection = connect(&mut shared.try_turn().unwrap());
    commit_shared(&shared, connection);
    complete(&shared);
    let turn = shared.try_turn().unwrap();
    shared.interrupted();
    assert_eq!(WAKES.load(Ordering::Relaxed), 0, "the call still holds it");
    drop(turn);
    // The holder does not ask the device itself: that query would run on
    // the guest call's processor.
    assert_eq!(WAKES.load(Ordering::Relaxed), 1);
    assert!(!shared.signaled());
    shared.interrupted();
    assert!(shared.signaled());
    assert_eq!(WAKES.load(Ordering::Relaxed), 1, "the look cleared it");
}

#[test]
fn interrupt_completion_queries_use_the_services_stack() {
    use core::sync::atomic::{AtomicUsize, Ordering};
    static RUNS: AtomicUsize = AtomicUsize::new(0);
    let host = ServiceHost {
        run: |stack, call| {
            assert_eq!(stack, HOST.stack);
            RUNS.fetch_add(1, Ordering::Relaxed);
            call();
        },
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    assert_eq!(RUNS.load(Ordering::Relaxed), 0);
    shared.interrupted();
    assert_eq!(RUNS.load(Ordering::Relaxed), 1);
}

#[test]
fn the_kexts_plugin_report_goes_to_the_platform() {
    use stela_contracts::gate::BundleReport;
    static REPORTS: AtomicUsize = AtomicUsize::new(0);
    let host = ServiceHost {
        bundle: |report| {
            assert_eq!(report, BundleReport::Missing);
            REPORTS.fetch_add(1, Ordering::Relaxed);
        },
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    shared.bundle(BundleReport::Missing);
    assert_eq!(REPORTS.load(Ordering::Relaxed), 1);
}

#[test]
fn a_calls_device_loss_signals_waiters_without_an_interrupt() {
    let shared = SharedService::new(service(), HOST);
    {
        let mut turn = shared.try_turn().unwrap();
        turn.device().mark_lost();
    }
    assert!(shared.signaled());
    drop(shared.try_turn().unwrap());
    assert!(!shared.signaled());
}

#[test]
fn device_loss_is_latched_and_wakes_waiters_only_once() {
    use super::{
        call,
        wire::{Empty, Operation, Status},
    };
    let mut service = service();
    let connection = connect(&mut service);
    let token = NonZeroU64::new(17).unwrap();
    service
        .import(
            connection,
            &[stela_contracts::gate::MemoryRange {
                address: 0x8_0000_0000,
                bytes: 4096,
            }],
            token,
        )
        .unwrap();
    commit(&mut service, connection, &Stream::default()).unwrap();
    service.device().backend().semaphore_error = Some(crate::metal::error::Error::DeviceLost);
    assert!(service.signaled());
    assert!(service.device().is_lost());
    assert!(!service.signaled());
    // The backend's injected error was one-shot. Later successful queries
    // must not make this device's old identities usable again.
    assert_eq!(service.connect().unwrap_err().status, Status::DeviceLost);
    let mut memory = Memory::default();
    let response: Result<wire::PollResponse, _> = call(
        &mut service,
        connection,
        Operation::Poll,
        &Empty {},
        &mut memory,
    );
    assert_eq!(response.unwrap_err().status, Status::DeviceLost);
    let result = commit(&mut service, connection, &Stream::default());
    assert_eq!(result.unwrap_err().status, Status::DeviceLost);
    service.disconnect(connection).unwrap();
    assert_eq!(
        service.retired(),
        None,
        "loss is not DMA retirement acknowledgement"
    );
}

#[test]
fn the_platform_times_each_call_by_its_operation_and_status() {
    use core::sync::atomic::AtomicU64;
    static TICKS: AtomicU64 = AtomicU64::new(0);
    static SERVED: [AtomicU64; 3] = [const { AtomicU64::new(0) }; 3];
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    fn served(served: Served) {
        // Each call reads the ticks as it rings, enters and leaves.
        assert_eq!((served.waited, served.held), (10, 10));
        let slot = &SERVED[COUNT.fetch_add(1, Ordering::Relaxed)];
        slot.store(
            u64::from(served.operation) << 32 | u64::from(served.status.raw()),
            Ordering::Relaxed,
        );
    }
    let host = ServiceHost {
        ticks: || TICKS.fetch_add(10, Ordering::Relaxed),
        served,
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    let connection = shared.connect().unwrap();
    let mut memory = Memory::default();
    let record = gate_call(
        &mut memory,
        Operation::Capabilities,
        &[],
        size_of::<Block>(),
    );
    GateService::serve(&shared, connection, record, &mut memory).unwrap();
    let poll = gate_call(&mut memory, Operation::Poll, &[], 16);
    let lost = GateCall {
        request: Span {
            address: 0x10,
            length: 8,
        },
        ..completed(&mut memory, poll)
    };
    let lost = memory.records(&[lost]).address;
    GateService::serve(&shared, connection, lost, &mut memory).unwrap();
    assert_eq!(
        GateService::serve(&shared, connection, 0x10, &mut memory),
        Err(CallerFault::Denied { address: 0x10 })
    );
    let seen = SERVED.each_ref().map(|slot| slot.load(Ordering::Relaxed));
    let expect =
        |operation: u32, status: Status| u64::from(operation) << 32 | u64::from(status.raw());
    assert_eq!(
        seen,
        [
            // An unnegotiated connection's call.
            expect(Operation::Capabilities.raw(), Status::Invalid),
            expect(Operation::Poll.raw(), Status::Unmapped),
            // A record out of reach.
            expect(0, Status::Closed),
        ]
    );
    assert_eq!(COUNT.load(Ordering::Relaxed), 3);
}

#[test]
fn a_poll_on_a_free_service_answers_what_was_published_without_asking_the_device() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let first = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, first, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, first), 0);
    commit_shared(&shared, connection);
    complete(&shared);
    // A query would fail: a poll that asked the device would answer the loss.
    shared
        .try_turn()
        .unwrap()
        .device()
        .backend()
        .semaphore_error = Some(crate::metal::error::Error::DeviceLost);
    let free = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, free, &mut memory).unwrap();
    assert_eq!(
        polled(&mut memory, free),
        0,
        "nothing published completion yet"
    );
    shared
        .try_turn()
        .unwrap()
        .device()
        .backend()
        .semaphore_error = None;
    // The interrupt's look publishes it before the kext wakes the frontend.
    shared.interrupted();
    assert!(shared.signaled());
    let woken = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, woken, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, woken), 1);
}

#[test]
fn the_kexts_import_is_timed_as_a_call_of_its_own() {
    use crate::metal::service::{IMPORT_AGAIN_OPERATION, IMPORT_OPERATION};
    use core::sync::atomic::AtomicU64;
    static TICKS: AtomicU64 = AtomicU64::new(0);
    static SERVED: AtomicU64 = AtomicU64::new(0);
    fn served(served: Served) {
        assert_eq!((served.waited, served.held), (10, 10));
        SERVED.store(
            u64::from(served.operation) << 32 | u64::from(served.status.raw()),
            Ordering::Relaxed,
        );
    }
    let shared = SharedService::new(
        service(),
        ServiceHost {
            ticks: || TICKS.fetch_add(10, Ordering::Relaxed),
            served,
            ..HOST
        },
    );
    let connection = shared.connect().unwrap();
    let range = stela_contracts::gate::MemoryRange {
        address: 0x8_0000_0000,
        bytes: 4096,
    };
    let expect = |operation: u32| u64::from(operation) << 32 | u64::from(Status::Ok.raw());
    let token = NonZeroU64::new(9).unwrap();
    assert!(GateService::import(&shared, connection, &[range], token).is_ok());
    assert_eq!(SERVED.load(Ordering::Relaxed), expect(IMPORT_OPERATION));
    // The same pages again, as a surface's for its next texture.
    let token = NonZeroU64::new(10).unwrap();
    assert!(GateService::import(&shared, connection, &[range], token).is_ok());
    assert_eq!(
        SERVED.load(Ordering::Relaxed),
        expect(IMPORT_AGAIN_OPERATION)
    );
    let other = stela_contracts::gate::MemoryRange {
        bytes: 8192,
        ..range
    };
    let token = NonZeroU64::new(11).unwrap();
    assert!(GateService::import(&shared, connection, &[other], token).is_ok());
    assert_eq!(SERVED.load(Ordering::Relaxed), expect(IMPORT_OPERATION));
}

/// `cargo test ... poll_paths_cost -- --ignored --nocapture`: the service's
/// own cost of a poll answered from what was published and of one that takes
/// the turn and asks the device. The mock's timeline query is free; on the
/// PC that query is a host kernel call the boot CPU serves (`GPU POSTED`).
#[test]
#[ignore = "manual: opt-in"]
fn poll_paths_cost() {
    extern crate std;
    const CALLS: u32 = 200_000;
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let record = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, record, &mut memory).unwrap();
    // Work in flight, so a turn's poll asks the device.
    commit_shared(&shared, connection);
    let time = |memory: &mut Memory| {
        let started = std::time::Instant::now();
        for _ in 0..CALLS {
            GateService::serve(&shared, connection, record, memory).unwrap();
        }
        started.elapsed().as_nanos() / u128::from(CALLS)
    };
    let published = time(&mut memory);
    // What a poll did before: a turn, and the service's dispatch in it.
    let turn = {
        let started = std::time::Instant::now();
        for _ in 0..CALLS {
            let mut turn = shared.try_turn().unwrap();
            turn.serve(connection, record, &mut memory).unwrap();
        }
        started.elapsed().as_nanos() / u128::from(CALLS)
    };
    std::println!("poll ns per call: published={published} turn={turn}");
}

/// A look after the waiter's reading takes that reading, with no driver
/// query; a reading that shows nothing new still asks the driver, which is
/// how a loss is noticed.
#[test]
fn a_look_takes_the_waiters_reading_instead_of_asking_the_driver() {
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    commit_shared(&shared, connection);
    complete(&shared);
    // A driver query would answer a loss.
    shared
        .try_turn()
        .unwrap()
        .device()
        .backend()
        .semaphore_error = Some(crate::metal::error::Error::DeviceLost);
    shared.completed_through(0, 1);
    shared.interrupted();
    {
        let mut turn = shared.try_turn().unwrap();
        assert!(!turn.device().is_lost(), "no query");
        assert_eq!(turn.device().completed_serial(), 1);
    }
    // Work outstanding, and a reading that shows nothing new.
    commit_shared(&shared, connection);
    shared.interrupted();
    assert!(
        shared.try_turn().unwrap().device().is_lost(),
        "nothing new: the driver was asked"
    );
}

#[test]
fn a_call_that_waits_past_the_bound_for_the_turn_runs_nothing_and_is_answered_pending() {
    use core::sync::atomic::{AtomicBool, AtomicU64};
    static TICKS: AtomicU64 = AtomicU64::new(0);
    static GAVE_UP: AtomicBool = AtomicBool::new(false);
    let host = ServiceHost {
        // A bounded wait never does the unbounded wait's work.
        relax: || panic!("a bounded wait relaxed without a bound"),
        relax_within: || true,
        ticks: || TICKS.fetch_add(1, Ordering::Relaxed),
        served: |served| {
            if served.gave_up {
                assert_eq!(served.status, Status::Pending);
                assert_eq!(served.held, 0, "it never held the service");
                GAVE_UP.store(true, Ordering::Relaxed);
            }
        },
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    shared.set_wait_bound(5);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    // Another call holds the service past the bound.
    let held = shared.try_turn().unwrap();
    let waiting = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
    GateService::serve(&shared, connection, waiting, &mut memory).unwrap();
    let call = completed(&mut memory, waiting);
    assert_eq!(call.status, Status::Pending.raw());
    assert_eq!(call.reason_length, 0);
    let response = GuestMemory::read(&mut memory, call.response).unwrap();
    assert!(response.iter().all(|&byte| byte == 0), "nothing answered");
    assert!(GAVE_UP.load(Ordering::Relaxed));
    // Made again once the holder has left, it runs.
    drop(held);
    let again = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
    GateService::serve(&shared, connection, again, &mut memory).unwrap();
    assert_eq!(completed(&mut memory, again).status, Status::Ok.raw());
}

/// The platform ends a bounded wait at once when the holder waits for its
/// processor: the call is answered `Pending` before the bound passes, after
/// a single relax.
#[test]
fn a_bounded_wait_the_platform_ends_is_answered_pending_before_the_bound() {
    use core::sync::atomic::AtomicU64;
    static RELAXED: AtomicU64 = AtomicU64::new(0);
    let host = ServiceHost {
        relax: || panic!("a bounded wait relaxed without a bound"),
        relax_within: || {
            RELAXED.fetch_add(1, Ordering::Relaxed);
            false
        },
        ticks: || 0,
        ..HOST
    };
    let shared = SharedService::new(service(), host);
    shared.set_wait_bound(u64::MAX);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let held = shared.try_turn().unwrap();
    let waiting = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
    GateService::serve(&shared, connection, waiting, &mut memory).unwrap();
    assert_eq!(
        completed(&mut memory, waiting).status,
        Status::Pending.raw()
    );
    assert_eq!(RELAXED.load(Ordering::Relaxed), 1);
    drop(held);
    let again = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
    GateService::serve(&shared, connection, again, &mut memory).unwrap();
    assert_eq!(completed(&mut memory, again).status, Status::Ok.raw());
}

#[test]
fn without_a_bound_a_call_waits_for_the_holder() {
    use core::sync::atomic::AtomicU64;
    static TICKS: AtomicU64 = AtomicU64::new(0);
    let host = ServiceHost {
        relax: || {},
        ticks: || TICKS.fetch_add(1_000_000, Ordering::Relaxed),
        ..HOST
    };
    let shared = std::sync::Arc::new(SharedService::new(service(), host));
    let connection = connect(&mut shared.try_turn().unwrap());
    let held = shared.try_turn().unwrap();
    let waiter = {
        let shared = shared.clone();
        std::thread::spawn(move || {
            let mut memory = Memory::default();
            let record = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
            GateService::serve(&*shared, connection, record, &mut memory).unwrap();
            completed(&mut memory, record).status
        })
    };
    std::thread::sleep(std::time::Duration::from_millis(20));
    assert!(
        !waiter.is_finished(),
        "it waits however long the holder holds"
    );
    drop(held);
    assert_eq!(waiter.join().unwrap(), Status::Ok.raw());
}

/// Waits, boundedly, until `ready` holds.
fn wait_until(ready: impl Fn() -> bool) {
    let start = std::time::Instant::now();
    while !ready() {
        assert!(
            start.elapsed() < std::time::Duration::from_secs(10),
            "timed out"
        );
        std::thread::yield_now();
    }
}

/// While an import's export and driver import wait (for the boot
/// processor's Linux, on the real platform), another connection's calls take
/// turns: the import holds none.
#[test]
fn an_imports_driver_work_holds_no_other_connections_call() {
    use crate::metal::device::fixture::MockOutside;
    use alloc::sync::Arc;
    let outside = Arc::new(MockOutside::default());
    let mut service = service();
    service.device().backend().outside = Some(outside.clone());
    let shared = Arc::new(SharedService::new(
        service,
        ServiceHost {
            relax: std::thread::yield_now,
            ..HOST
        },
    ));
    shared.set_outside_stack(0x8000);
    let kext = shared.connect().unwrap();
    let app = connect(&mut shared.try_turn().unwrap());
    outside.hold.store(true, Ordering::SeqCst);
    let importer = {
        let shared = shared.clone();
        std::thread::spawn(move || {
            let range = stela_contracts::gate::MemoryRange {
                address: 0x8_0000_0000,
                bytes: 4096,
            };
            GateService::import(&*shared, kext, &[range], NonZeroU64::new(5).unwrap())
        })
    };
    wait_until(|| outside.preparing.load(Ordering::SeqCst));
    // The other connection commits and polls while the import waits, and
    // leaves frees waiting rather than wait for the outside lock.
    outside.pending.store(2, Ordering::SeqCst);
    commit_shared(&shared, app);
    let mut memory = Memory::default();
    let poll = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&*shared, app, poll, &mut memory).unwrap();
    assert_eq!(completed(&mut memory, poll).status, Status::Ok.raw());
    assert_eq!(outside.ran.load(Ordering::SeqCst), 0, "the import holds it");
    assert!(outside.frees_pending_now());
    outside.hold.store(false, Ordering::SeqCst);
    let imported = importer.join().unwrap();
    assert!(imported.is_ok());
    assert_eq!(outside.prepared.load(Ordering::SeqCst), 1);
    assert_eq!(outside.discarded.load(Ordering::SeqCst), 0);
    let mut turn = shared.try_turn().unwrap();
    assert_eq!(
        turn.device().backend().imports.len(),
        1,
        "adopted in the turn"
    );
}

/// An import whose connection closed while its driver work ran is refused,
/// and the memory made for it is freed.
#[test]
fn an_import_made_outside_for_a_closed_connection_is_freed() {
    use crate::metal::device::fixture::MockOutside;
    use alloc::sync::Arc;
    let outside = Arc::new(MockOutside::default());
    let mut service = service();
    service.device().backend().outside = Some(outside.clone());
    let shared = Arc::new(SharedService::new(
        service,
        ServiceHost {
            relax: std::thread::yield_now,
            report: |_| {},
            ..HOST
        },
    ));
    shared.set_outside_stack(0x8000);
    let kext = shared.connect().unwrap();
    outside.hold.store(true, Ordering::SeqCst);
    let importer = {
        let shared = shared.clone();
        std::thread::spawn(move || {
            let range = stela_contracts::gate::MemoryRange {
                address: 0x8_0000_0000,
                bytes: 4096,
            };
            GateService::import(&*shared, kext, &[range], NonZeroU64::new(5).unwrap())
        })
    };
    wait_until(|| outside.preparing.load(Ordering::SeqCst));
    GateService::disconnect(&*shared, kext);
    outside.hold.store(false, Ordering::SeqCst);
    assert!(importer.join().unwrap().is_err());
    assert_eq!(outside.discarded.load(Ordering::SeqCst), 1);
    let mut turn = shared.try_turn().unwrap();
    assert!(turn.device().backend().imports.is_empty());
}

/// Frees a call's turn let go of run after the turn, and only when some
/// wait; the backend learns a service runs them.
#[test]
fn frees_run_after_the_calls_turn() {
    use crate::metal::device::fixture::MockOutside;
    use alloc::sync::Arc;
    let outside = Arc::new(MockOutside::default());
    let mut service = service();
    service.device().backend().outside = Some(outside.clone());
    let shared = SharedService::new(service, HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    // A call that takes a turn (a poll may be answered without one).
    let poll = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
    GateService::serve(&shared, connection, poll, &mut memory).unwrap();
    assert_eq!(outside.ran.load(Ordering::SeqCst), 0, "no outside stack");
    shared.set_outside_stack(0x8000);
    GateService::serve(&shared, connection, poll, &mut memory).unwrap();
    assert!(outside.enabled.load(Ordering::SeqCst));
    assert_eq!(outside.ran.load(Ordering::SeqCst), 0, "nothing waits");
    outside.pending.store(2, Ordering::SeqCst);
    GateService::serve(&shared, connection, poll, &mut memory).unwrap();
    assert_eq!(outside.ran.load(Ordering::SeqCst), 1);
    assert!(!outside.frees_pending_now());
    assert!(shared.try_turn().is_some(), "no turn kept");
}

/// Work a call's turn leaves for after it (frees, released semaphores,
/// semaphores made ahead) asks the platform once for an upkeep, which runs
/// it holding no turn, a bounded piece at a time.
#[test]
fn upkeep_runs_where_the_platform_asks_holding_no_turn_and_bounded() {
    use crate::metal::device::fixture::MockOutside;
    use alloc::sync::Arc;
    use core::sync::atomic::AtomicUsize;
    static WAKES: AtomicUsize = AtomicUsize::new(0);
    let outside = Arc::new(MockOutside::default());
    let mut service = service();
    service.device().backend().outside = Some(outside.clone());
    let shared = SharedService::new(
        service,
        ServiceHost {
            wake: || {
                WAKES.fetch_add(1, Ordering::SeqCst);
            },
            ..HOST
        },
    );
    let connection = connect(&mut shared.try_turn().unwrap());
    assert_eq!(shared.upkeep(), 0, "no call saw the outside work yet");
    shared.set_outside_stack(0x8000);
    outside.chores.store(UPKEEP_PIECES + 4, Ordering::SeqCst);
    let mut memory = Memory::default();
    let call = gate_call(&mut memory, Operation::AllocatedSize, &[], 8);
    GateService::serve(&shared, connection, call, &mut memory).unwrap();
    GateService::serve(&shared, connection, call, &mut memory).unwrap();
    assert_eq!(WAKES.load(Ordering::SeqCst), 1, "asked once");
    assert_eq!(outside.upkept.load(Ordering::SeqCst), 0, "not in the call");
    // The platform's upkeep while a call holds the turn.
    let held = shared.try_turn().unwrap();
    assert_eq!(shared.upkeep(), UPKEEP_PIECES);
    assert_eq!(WAKES.load(Ordering::SeqCst), 2, "what is left asks again");
    assert_eq!(shared.upkeep(), 4);
    assert_eq!(shared.upkeep(), 0);
    drop(held);
    assert_eq!(WAKES.load(Ordering::SeqCst), 2, "nothing left to ask for");
}

/// The platform's upkeep does nothing while another caller holds the
/// outside lock; that caller asks again once it leaves.
#[test]
fn upkeep_leaves_the_outside_lock_to_its_holder() {
    use crate::metal::device::fixture::MockOutside;
    use alloc::sync::Arc;
    use core::sync::atomic::AtomicUsize;
    static WAKES: AtomicUsize = AtomicUsize::new(0);
    let outside = Arc::new(MockOutside::default());
    let mut service = service();
    service.device().backend().outside = Some(outside.clone());
    let shared = Arc::new(SharedService::new(
        service,
        ServiceHost {
            relax: std::thread::yield_now,
            wake: || {
                WAKES.fetch_add(1, Ordering::SeqCst);
            },
            ..HOST
        },
    ));
    shared.set_outside_stack(0x8000);
    let kext = shared.connect().unwrap();
    outside.hold.store(true, Ordering::SeqCst);
    let importer = {
        let shared = shared.clone();
        std::thread::spawn(move || {
            let range = stela_contracts::gate::MemoryRange {
                address: 0x8_0000_0000,
                bytes: 4096,
            };
            GateService::import(&*shared, kext, &[range], NonZeroU64::new(5).unwrap())
        })
    };
    wait_until(|| outside.preparing.load(Ordering::SeqCst));
    outside.chores.store(2, Ordering::SeqCst);
    assert_eq!(shared.upkeep(), 0, "the import holds the outside lock");
    outside.hold.store(false, Ordering::SeqCst);
    assert!(importer.join().unwrap().is_ok());
    assert_eq!(WAKES.load(Ordering::SeqCst), 1, "the import asked after it");
    assert_eq!(shared.upkeep(), 2);
}

/// With a completion marker, the serial the GPU wrote after a submission's
/// work is published by whichever processor next advances the gate or polls:
/// no driver query, no interrupt, no turn. A lost device's marker is read
/// no more.
#[test]
fn the_completion_marker_publishes_without_the_driver_or_an_interrupt() {
    let mut backend = MockBackend::new();
    backend.marker = Some(alloc::boxed::Box::new(AtomicU32::new(0)));
    let shared = SharedService::new(super::service_on(backend), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let first = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, first, &mut memory).unwrap();
    commit_shared(&shared, connection);
    let marker = {
        let mut turn = shared.try_turn().unwrap();
        let backend = turn.device().backend();
        // The submission ends by writing its serial to the marker.
        let segments = backend.submissions.last().unwrap();
        let fill = segments.iter().flat_map(|segment| &segment.commands).rev();
        assert!(fill.clone().any(|command| matches!(
            command,
            crate::metal::vulkan::command::Command::FillBuffer {
                data: 1,
                size: 4,
                ..
            }
        )));
        core::ptr::from_ref(&**backend.marker.as_ref().unwrap())
    };
    assert!(GateService::awaits_completion(&shared));
    assert!(!GateService::signaled(&shared), "the GPU wrote nothing yet");
    // A query would fail: only the marker can say it completed.
    shared
        .try_turn()
        .unwrap()
        .device()
        .backend()
        .semaphore_error = Some(crate::metal::error::Error::DeviceLost);
    // SAFETY: the backend's box lives as long as the service.
    unsafe { &*marker }.store(1, Ordering::SeqCst);
    assert!(GateService::signaled(&shared), "seen from the marker");
    assert!(!GateService::awaits_completion(&shared));
    let woken = gate_call(&mut memory, Operation::Poll, &[], 16);
    GateService::serve(&shared, connection, woken, &mut memory).unwrap();
    assert_eq!(polled(&mut memory, woken), 1);
    // A lost device's marker is cleared before any turn can replace it.
    shared.try_turn().unwrap().device().mark_lost();
    drop(shared.try_turn());
    assert!(shared.marker_value().is_none());
}

/// A `NewLibrary` call for the library at `data`.
fn library_call(memory: &mut Memory, data: Span) -> u64 {
    let request = wire::NewLibraryRequest { data };
    gate_call(
        memory,
        Operation::NewLibrary,
        wire::bytes(&request),
        size_of::<wire::Handle>(),
    )
}

#[test]
fn a_library_is_read_a_piece_at_each_run_and_its_turn_reads_none() {
    const PIECE: usize = 64;
    let shared = SharedService::new(service(), HOST);
    shared.set_library_piece(PIECE);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    // Not the fixture's library, which the device holds from its start.
    let bytes = crate::metal::library::tests::padded(256, 0x11);
    let data = memory.put(&bytes);
    let record = library_call(&mut memory, data);
    memory.longest = 0;
    // `HOST.relax` panics: a run that waited for this holder would fail.
    let held = shared.try_turn().unwrap();
    for _ in 1..bytes.len().div_ceil(PIECE) {
        GateService::serve(&shared, connection, record, &mut memory).unwrap();
        let call = completed(&mut memory, record);
        assert_eq!(
            (call.status, call.reason_length),
            (Status::Pending.raw(), 0)
        );
    }
    drop(held);
    // The device holds no library of these bytes: they are read again, kept.
    GateService::serve(&shared, connection, record, &mut memory).unwrap();
    let call = completed(&mut memory, record);
    assert_eq!(
        (call.status, call.reason_length),
        (Status::Pending.raw(), 0)
    );
    for _ in 1..bytes.len().div_ceil(PIECE) {
        GateService::serve(&shared, connection, record, &mut memory).unwrap();
        assert_eq!(completed(&mut memory, record).status, Status::Pending.raw());
    }
    GateService::serve(&shared, connection, record, &mut memory).unwrap();
    let call = completed(&mut memory, record);
    assert_eq!(call.status, Status::Ok.raw());
    let handle: wire::Handle =
        wire::read(&GuestMemory::read(&mut memory, call.response).unwrap()).unwrap();
    assert_ne!(handle.handle, 0);
    assert!(
        memory.longest <= PIECE as u64,
        "no read took more than a piece: {}",
        memory.longest
    );

    // A connection's library of bytes the device holds shares them.
    let other = connect(&mut shared.try_turn().unwrap());
    let data = memory.put(&bytes);
    let again = library_call(&mut memory, data);
    shared.set_library_piece(bytes.len());
    GateService::serve(&shared, other, again, &mut memory).unwrap();
    assert_eq!(completed(&mut memory, again).status, Status::Ok.raw());

    // A span out of the caller's reach asks it to map the span.
    let lost = library_call(
        &mut memory,
        Span {
            address: 0x7000_0000_0000,
            length: bytes.len() as u64,
        },
    );
    GateService::serve(&shared, connection, lost, &mut memory).unwrap();
    let call = completed(&mut memory, lost);
    assert_eq!(
        (call.status, call.reason_length),
        (Status::Unmapped.raw(), 0)
    );
}

/// Serves the `NewLibrary` call at `record` until it is answered other than
/// `Pending`, with at most `runs` runs.
fn serve_library(
    shared: &SharedService<MockBackend, MockCompiler>,
    connection: NonZeroU64,
    record: u64,
    memory: &mut Memory,
    runs: usize,
) -> GateCall {
    for _ in 0..runs {
        GateService::serve(shared, connection, record, memory).unwrap();
        let call = completed(memory, record);
        if call.status != Status::Pending.raw() {
            return call;
        }
    }
    panic!("the library call still pending after {runs} runs");
}

/// A library another process holds is shared by its digest with no
/// allocation of its size: a heap whose largest free block is far smaller
/// than the library still serves it. A library the device does not hold
/// needs one block of its size for the kept copy; without one the call is
/// answered out of memory.
#[test]
fn a_held_library_is_shared_without_a_block_of_its_size() {
    const LIBRARY: usize = 8 << 20;
    const LARGEST_BLOCK: usize = 2 << 20;
    let shared = SharedService::new(service(), HOST);
    let first = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let bytes = crate::metal::library::tests::padded(LIBRARY, 0x5a);
    let data = memory.put(&bytes);
    let record = library_call(&mut memory, data);
    let runs = 2 * bytes.len().div_ceil(LIBRARY_PIECE_BYTES) + 4;
    assert_eq!(
        serve_library(&shared, first, record, &mut memory, runs).status,
        Status::Ok.raw()
    );

    let second = connect(&mut shared.try_turn().unwrap());
    let again = library_call(&mut memory, data);
    let (call, refused) = super::heap::without_blocks_of(LARGEST_BLOCK, || {
        serve_library(&shared, second, again, &mut memory, runs)
    });
    assert_eq!(call.status, Status::Ok.raw());
    assert_eq!(refused, 0, "no allocation of a large block was asked for");

    let other = crate::metal::library::tests::padded(LIBRARY, 0xa5);
    let data = memory.put(&other);
    let new = library_call(&mut memory, data);
    let (call, refused) = super::heap::without_blocks_of(LARGEST_BLOCK, || {
        serve_library(&shared, second, new, &mut memory, runs)
    });
    assert_eq!(call.status, Status::OutOfMemory.raw());
    assert_ne!(refused, 0);
}

/// A texture only host image copies reach is read by a GPU copy into host
/// memory: the call's turn starts each band's copy and answers `Pending`,
/// and the call made again copies the band to its caller from the published
/// completion, holding no turn, while another call holds the service.
#[test]
fn a_texture_read_waits_for_its_gpu_copy_and_copies_out_holding_no_turn() {
    use crate::metal::{
        abi::StorageMode,
        device::resources::CPU_BAND_BYTES,
        service::wire::{Empty, RegionRecord, TextureBytesRequest, TextureResponse},
        vulkan::backend::Object,
    };
    let shared = SharedService::new(service(), HOST);
    let connection = connect(&mut shared.try_turn().unwrap());
    let mut memory = Memory::default();
    let texture: TextureResponse = super::call(
        &mut shared.try_turn().unwrap(),
        connection,
        Operation::NewTexture,
        &super::texture(StorageMode::Shared),
        &mut memory,
    )
    .unwrap();
    // Two rows of eight texels, written whole. The test's GPU copies an
    // image from its start, so the rows are alike; each band's copy names
    // its own row.
    let written: alloc::vec::Vec<u8> = (0..64).map(|byte| byte % 32).collect();
    let region = RegionRecord {
        slice: 0,
        level: 0,
        origin: [0; 3],
        size: [8, 2, 1],
    };
    let replace = TextureBytesRequest {
        texture: texture.handle,
        region,
        bytes: memory.put(&written),
        bytes_per_row: 32,
        bytes_per_image: 0,
    };
    let _: Empty = super::call(
        &mut shared.try_turn().unwrap(),
        connection,
        Operation::ReplaceRegion,
        &replace,
        &mut memory,
    )
    .unwrap();
    // Rows a band apart: each is a band of its own.
    let apart = CPU_BAND_BYTES as usize;
    let read = TextureBytesRequest {
        bytes: memory.put(&alloc::vec![0xee; apart + 32]),
        bytes_per_row: CPU_BAND_BYTES,
        ..replace
    };
    let record = gate_call(&mut memory, Operation::GetBytes, wire::bytes(&read), 0);
    let serve = |memory: &mut Memory| {
        GateService::serve(&shared, connection, record, memory).unwrap();
        completed(memory, record).status
    };
    // `HOST.relax` panics: a call that waited for the holder would fail.
    let serve_held = |memory: &mut Memory| {
        let held = shared.try_turn().unwrap();
        let status = serve(memory);
        drop(held);
        status
    };
    let finish = || {
        shared.try_turn().unwrap().device().backend().finish();
        shared.interrupted();
    };
    // The first band's turn starts its copy.
    assert_eq!(serve(&mut memory), Status::Pending.raw());
    assert_eq!(serve_held(&mut memory), Status::Pending.raw(), "copy runs");
    finish();
    // Copied out, and the next band's copy starts in the call's turn.
    assert_eq!(serve(&mut memory), Status::Pending.raw());
    let bytes = GuestMemory::read(&mut memory, read.bytes).unwrap();
    assert_eq!(bytes[..32], written[..32]);
    assert_eq!(serve_held(&mut memory), Status::Pending.raw(), "copy runs");
    finish();
    // The last band is copied out holding no turn.
    assert_eq!(serve_held(&mut memory), Status::Ok.raw());
    let rows: alloc::vec::Vec<(i32, [u32; 3])> = shared
        .try_turn()
        .unwrap()
        .device()
        .backend()
        .submissions
        .iter()
        .flatten()
        .flat_map(|segment| &segment.commands)
        .filter_map(|command| match command {
            crate::metal::vulkan::command::Command::CopyImageToBuffer { region, .. } => {
                Some((region.offset[1], region.extent))
            }
            _ => None,
        })
        .collect();
    assert_eq!(rows, [(0, [8, 1, 1]), (1, [8, 1, 1])]);
    let bytes = GuestMemory::read(&mut memory, read.bytes).unwrap();
    assert_eq!(bytes[..32], written[..32]);
    assert_eq!(bytes[32..apart], alloc::vec![0xee; apart - 32][..]);
    assert_eq!(bytes[apart..], written[32..]);
    // The memory copied out of is freed by the next turn.
    GateService::disconnect(&shared, connection);
    let freed = shared
        .try_turn()
        .unwrap()
        .device()
        .backend()
        .destroyed
        .iter()
        .filter(|object| matches!(object, Object::Buffer(_)))
        .count();
    assert_eq!(freed, 2, "both bands' host memory");
}
