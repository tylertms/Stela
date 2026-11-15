use super::super::{
    plist,
    store::tests::fixture,
    stream::tests::{Memory, receive},
};
use super::*;
use crate::restore::plist::Value;
use alloc::{collections::BTreeMap, vec};
struct Harness {
    streams: BTreeMap<u16, Memory>,
    opens: Vec<u16>,
    closed: Vec<u16>,
    ready: bool,
    failed_write: bool,
    /// A port whose writes the guest never takes.
    stalled: Option<u16>,
}
impl Ports for Harness {
    type Handle = u16;
    fn open(&mut self, p: u16) -> Poll<Result<u16>> {
        if !self.ready {
            return Poll::Pending;
        }
        if self.opens.contains(&p) {
            return Poll::Ready(Err(Error::Invalid("duplicate open")));
        }
        self.opens.push(p);
        Poll::Ready(Ok(p))
    }
    fn read(&mut self, h: u16, b: &mut [u8]) -> Poll<Result<usize>> {
        let s = self.streams.get_mut(&h).unwrap();
        // An open mux receive window with no data is Pending, not EOF.
        if s.at == s.input.len() {
            Poll::Pending
        } else {
            s.read(b)
        }
    }
    fn write(&mut self, h: u16, b: &[u8]) -> Poll<Result<usize>> {
        if self.failed_write {
            return Poll::Ready(Err(Error::Transport("injected write failure".into())));
        }
        if self.stalled == Some(h) {
            return Poll::Pending;
        }
        self.streams.get_mut(&h).unwrap().write(b)
    }
    fn close(&mut self, h: u16) -> Poll<Result<()>> {
        assert!(!self.closed.contains(&h));
        self.closed.push(h);
        Poll::Ready(Ok(()))
    }
}
fn append(h: &mut Harness, v: Value) {
    let body = plist::encode_binary(&v, Limits::default()).unwrap();
    let s = h.streams.get_mut(&PORT).unwrap();
    s.input.extend((body.len() as u32).to_be_bytes());
    s.input.extend(body);
}
fn setup() -> (
    Driver<stela_formats::payload::test_support::Memory, u16>,
    Harness,
) {
    setup_with(b"payload")
}
fn setup_with(
    payload: &[u8],
) -> (
    Driver<stela_formats::payload::test_support::Memory, u16>,
    Harness,
) {
    let d = Driver::new(
        Answers::new(fixture(payload, false)),
        Route::StelaFirmwareStudy,
    )
    .unwrap();
    (d, harness())
}
/// The guest side up to its HardwareInfo reply.
fn harness() -> Harness {
    let mut h = Harness {
        streams: BTreeMap::from([
            (PORT, Memory::new(vec![], 3)),
            (2345, Memory::new(vec![], 1)),
        ]),
        opens: vec![],
        closed: vec![],
        ready: false,
        failed_write: false,
        stalled: None,
    };
    append(
        &mut h,
        Value::dict([
            ("Type", Value::from("com.apple.mobile.restored")),
            ("Result", Value::from("Success")),
            ("RestoreProtocolVersion", Value::Integer(15)),
        ]),
    );
    append(
        &mut h,
        Value::dict([(
            "HardwareInfo",
            Value::dict([
                ("ChipID", Value::Integer(0x8112)),
                ("BoardID", Value::Integer(0x24)),
                ("UniqueChipID", Value::Integer(1234)),
            ]),
        )]),
    );
    h
}
fn restoring<R: ReadAt>(d: &mut Driver<R, u16>, h: &mut Harness) {
    for _ in 0..100 {
        assert!(d.poll(h).is_pending());
    }
    assert!(h.opens.is_empty());
    h.ready = true;
    for _ in 0..100_000 {
        assert!(d.poll(h).is_pending());
        if d.session.phase() == Phase::Restoring {
            return;
        }
    }
    panic!("handshake did not complete");
}
fn request(kind: &str, port: bool) -> Value {
    let mut v = Value::dict([
        ("MsgType", Value::from("AsyncDataRequestMsg")),
        ("DataType", Value::from(kind)),
        ("Arguments", Value::dict([("ImageName", Value::from("OS"))])),
    ]);
    if let Value::Dict(m) = &mut v
        && port
    {
        m.insert("DataPort".into(), Value::Integer(2345));
    }
    v
}
#[test]
fn transcript_short_io_concurrent_control_jobs_and_final_ack_write() {
    let (mut d, mut h) = setup();
    restoring(&mut d, &mut h);
    append(&mut h, request("SourceBootObjectV5", true));
    append(&mut h, request("ReceiptManifest", false));
    append(&mut h, request("FDRTrustData", false));
    append(
        &mut h,
        Value::dict([
            ("MsgType", Value::from("ProgressMsg")),
            ("Operation", Value::Integer(13)),
            ("Progress", Value::Integer(50)),
        ]),
    );
    for _ in 0..100_000 {
        assert!(d.poll_prefetched(&mut h).is_pending());
        d.work();
        d.prefetch(1, || false);
        if d.session.ledger().len() == 6 && d.session.outstanding() == 0 {
            break;
        }
    }
    assert_eq!(d.session.ledger().len(), 6);
    assert_eq!(d.session.outstanding(), 0);
    // A removed job must retain its host-send completion in telemetry; this
    // still does not complete the restore before the final guest status.
    let files: Vec<_> = d.file_progress().collect();
    assert_eq!(files.len(), 1);
    assert_eq!(
        (files[0].sent, files[0].total, files[0].complete),
        (7, 7, true)
    );
    assert_eq!(d.session.phase(), Phase::Restoring);
    assert_eq!(h.opens, vec![PORT, 2345]);
    assert_eq!(h.closed, vec![2345]);
    let s = h.streams.get_mut(&2345).unwrap();
    s.input = core::mem::take(&mut s.output);
    s.at = 0;
    let mut f = Framed::new(Endian::Big, Limits::default());
    assert_eq!(
        receive(&mut f, s)
            .unwrap()
            .required("FileData")
            .unwrap()
            .data()
            .unwrap(),
        b"payload"
    );
    assert!(
        receive(&mut f, s)
            .unwrap()
            .required("FileDataDone")
            .unwrap()
            .boolean()
            .unwrap()
    );
    append(
        &mut h,
        Value::dict([
            ("MsgType", Value::from("StatusMsg")),
            ("Status", Value::Integer(0)),
        ]),
    );
    let mut final_seen = false;
    let mut complete = false;
    for _ in 0..100_000 {
        match d.poll(&mut h) {
            Poll::Ready(v) => {
                v.unwrap();
                complete = true;
                break;
            }
            Poll::Pending => {}
        }
        final_seen |= d.session.phase() == Phase::FinalAck;
    }
    assert!(final_seen && complete);
    assert_eq!(h.closed, vec![2345, PORT]);
    // Decode every control response to catch interleaved frames.
    let s = h.streams.get_mut(&PORT).unwrap();
    s.input = core::mem::take(&mut s.output);
    s.at = 0;
    let mut responses = Vec::new();
    while s.at < s.input.len() {
        responses.push(receive(&mut f, s).unwrap());
    }
    assert_eq!(responses.len(), 6);
    let opts = responses[2].required("RestoreOptions").unwrap();
    assert!(!opts.required("FlashNOR").unwrap().boolean().unwrap());
    let data = opts.required("SupportedDataTypes").unwrap();
    // The whole vocabulary is advertised; the study route refuses a
    // NORData request when it arrives.
    assert!(!data.required("NORData").unwrap().boolean().unwrap());
    assert!(!data.required("FileDataDone").unwrap().boolean().unwrap());
    let messages = opts.required("SupportedMessageTypes").unwrap();
    for name in [
        "ProvisioningInfo",
        "BBUpdateStatusMsg",
        "ReceivedFinalStatusMsg",
    ] {
        assert!(!messages.required(name).unwrap().boolean().unwrap());
    }
    // An AEA image's content key comes through an asynchronous URLAsset.
    let asynchronous = opts.required("SupportedAsyncDataTypes").unwrap();
    for name in ["RecoveryOSASRImage", "SystemImageData", "URLAsset"] {
        assert!(asynchronous.required(name).unwrap().boolean().unwrap());
    }
    assert_eq!(
        responses[5].required("MsgType").unwrap().string().unwrap(),
        "ReceivedFinalStatusMsg"
    );
}
#[test]
fn file_data_on_the_control_stream_drains_before_a_competing_reply() {
    let payload = vec![5; 5 * 8192 + 1];
    let (mut d, mut h) = setup_with(&payload);
    restoring(&mut d, &mut h);
    let s = h.streams.get_mut(&PORT).unwrap();
    let start = s.output.len();
    s.chunk = 1931;
    append(&mut h, request("SourceBootObjectV4", false));
    append(&mut h, request("ReceiptManifest", false));
    for _ in 0..1_000 {
        assert!(d.poll_prefetched(&mut h).is_pending());
        d.work();
        d.prefetch(1, || false);
        if d.session.outstanding() == 0 && d.session.ledger().len() == 4 {
            break;
        }
    }
    assert_eq!(d.session.ledger().len(), 4);
    assert_eq!(d.session.outstanding(), 0);
    let s = h.streams.get_mut(&PORT).unwrap();
    let mut stream = Memory::new(s.output[start..].to_vec(), 1931);
    let mut frame = Framed::new(Endian::Big, Limits::default());
    for expected in payload.chunks(8192) {
        let actual = receive(&mut frame, &mut stream).unwrap();
        assert_eq!(
            actual.required("FileData").unwrap().data().unwrap(),
            expected
        );
    }
    let done = receive(&mut frame, &mut stream).unwrap();
    assert!(done.required("FileDataDone").unwrap().boolean().unwrap());
    let reply = receive(&mut frame, &mut stream).unwrap();
    assert!(reply.get("ReceiptManifest").is_some());
    assert_eq!(stream.at, stream.input.len());
}

#[test]
fn blocked_data_keeps_control_live_and_limits_open_handles() {
    let (_, mut h) = setup();
    let payload = vec![42; 8192];
    let mut d = Driver::new(
        Answers::new(fixture(&payload, false)),
        Route::StelaFirmwareStudy,
    )
    .unwrap();
    restoring(&mut d, &mut h);
    for port in 2345..2352 {
        h.streams.insert(port, Memory::new(vec![], 1));
        let mut r = request("SourceBootObjectV5", false);
        if let Value::Dict(m) = &mut r {
            m.insert("DataPort".into(), Value::Integer(port as u64));
        }
        append(&mut h, r);
    }
    append(
        &mut h,
        Value::dict([
            ("MsgType", Value::from("ProgressMsg")),
            ("Operation", Value::Integer(13)),
            ("Progress", Value::Integer(50)),
        ]),
    );
    for _ in 0..100_000 {
        assert!(d.poll(&mut h).is_pending());
        if d.session.ledger().len() == 10 {
            break;
        }
    }
    assert_eq!(d.session.ledger().len(), 10);
    assert_eq!(d.session.outstanding(), 7);
    assert_eq!(h.opens.len(), DATA_STREAMS + 1);
    assert!(h.closed.is_empty());
    assert_eq!(d.cancel(&mut h), Poll::Ready(Ok(())));
    assert_eq!(h.closed.len(), DATA_STREAMS + 1);
    assert!(matches!(d.poll(&mut h), Poll::Ready(Err(_))));
}
#[test]
fn refusal_keeps_request_never_completes_and_cancels_once() {
    let (mut d, mut h) = setup();
    restoring(&mut d, &mut h);
    let r = request("NORData", false);
    append(&mut h, r.clone());
    let error = (0..100_000)
        .find_map(|_| match d.poll(&mut h) {
            Poll::Ready(Err(e)) => Some(e),
            _ => None,
        })
        .expect("refusal did not terminate");
    assert!(matches!(error,Error::Refused{ref request,..} if **request==r));
    let bytes = h.streams[&PORT].output.len();
    assert_eq!(d.poll(&mut h), Poll::Ready(Err(error)));
    assert_eq!(h.streams[&PORT].output.len(), bytes);
    assert_eq!(d.cancel(&mut h), Poll::Ready(Ok(())));
    assert_eq!(d.cancel(&mut h), Poll::Ready(Ok(())));
    assert_eq!(h.closed, vec![PORT]);
}
#[test]
fn write_failure_is_sticky_and_no_final_ack_can_follow() {
    let (mut d, mut h) = setup();
    h.ready = true;
    assert!(d.poll(&mut h).is_pending());
    h.failed_write = true;
    assert!(matches!(
        d.poll(&mut h),
        Poll::Ready(Err(Error::Transport(_)))
    ));
    h.failed_write = false;
    assert!(matches!(d.poll(&mut h), Poll::Ready(Err(_))));
    assert!(h.streams[&PORT].output.is_empty());
    assert_ne!(d.session.phase(), Phase::Complete);
}

/// A network source bounded once the guest runs: its next `stalls` reads
/// run out of the time they may wait.
struct Stalling {
    source: stela_formats::payload::test_support::Memory,
    stalls: usize,
    reads: usize,
    blocked: bool,
}
impl ReadAt for Stalling {
    type Error = &'static str;
    fn len(&self) -> u64 {
        self.source.len()
    }
    fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> core::result::Result<(), Self::Error> {
        self.reads += 1;
        if self.stalls > 0 {
            self.stalls -= 1;
            self.blocked = true;
            return Err("the read's time ran out");
        }
        self.source.read_at(offset, buffer)
    }
    fn blocked(&mut self) -> bool {
        core::mem::take(&mut self.blocked)
    }
}
/// A driver over a scanned (unrecorded) inventory, which `prepare` reads.
fn stalling_driver() -> Driver<Stalling, u16> {
    use super::super::store::{
        Store,
        tests::{ERASE, parts},
    };
    let (source, manifest) = parts(b"payload", true, &[], &[], ERASE);
    let reader = Stalling {
        source,
        stalls: 0,
        reads: 0,
        blocked: false,
    };
    let store = Store::open(reader, manifest, 0).unwrap();
    Driver::new(Answers::new(store), Route::StelaFirmwareStudy).unwrap()
}
/// The control messages the driver wrote, decoded.
fn sent(h: &mut Harness) -> Vec<Value> {
    let s = h.streams.get_mut(&PORT).unwrap();
    let mut out = Memory::new(core::mem::take(&mut s.output), 1 << 20);
    let mut f = Framed::new(Endian::Big, Limits::default());
    let mut messages = Vec::new();
    while out.at < out.input.len() {
        messages.push(receive(&mut f, &mut out).unwrap());
    }
    messages
}
fn starts(messages: &[Value]) -> usize {
    messages
        .iter()
        .filter(|m| m.get("RestoreOptions").is_some())
        .count()
}

#[test]
fn a_pending_first_read_starts_the_restore_once() {
    let mut d = stalling_driver();
    let mut h = harness();
    // Reads are bounded from here, and the store's first ones run out of
    // time.
    d.answers.store.reader_mut().stalls = 3;
    restoring(&mut d, &mut h);
    assert!(d.failed().is_none(), "{:?}", d.failed());
    assert_eq!(d.answers.store.reader_mut().stalls, 0);
    let messages = sent(&mut h);
    // The type query, the HardwareInfo query and one StartRestore.
    assert_eq!(starts(&messages), 1, "{messages:?}");
    assert_eq!(messages.len(), 3, "{messages:?}");
}

#[test]
fn a_store_prepared_before_the_guest_reads_nothing_at_the_start() {
    let mut d = stalling_driver();
    let mut h = harness();
    // Prepared while reads are unbounded, before the guest starts.
    d.answers.store.prepare().unwrap();
    let reader = d.answers.store.reader_mut();
    let before = reader.reads;
    reader.stalls = usize::MAX;
    restoring(&mut d, &mut h);
    assert!(d.failed().is_none(), "{:?}", d.failed());
    assert_eq!(d.answers.store.reader_mut().reads, before);
    assert_eq!(starts(&sent(&mut h)), 1);
}

/// A read ahead whose time is over after one job leaves the others to the
/// next call, which reads for them first: a job whose guest takes nothing
/// never keeps another from moving on.
#[test]
fn a_read_ahead_over_its_time_reads_for_each_job_in_turn() {
    let payload = vec![7; 3 * 8192];
    let (mut d, mut h) = setup_with(&payload);
    h.streams.insert(2346, Memory::new(vec![], 1));
    h.stalled = Some(2345);
    restoring(&mut d, &mut h);
    for port in [2345u64, 2346] {
        let mut r = request("SourceBootObjectV5", false);
        if let Value::Dict(m) = &mut r {
            m.insert("DataPort".into(), Value::Integer(port));
        }
        append(&mut h, r);
    }
    for _ in 0..100_000 {
        assert!(d.poll_prefetched(&mut h).is_pending());
        d.work();
        d.prefetch(1, || true);
        if h.closed.contains(&2346) {
            break;
        }
    }
    assert!(!h.closed.contains(&2345));
    let s = h.streams.get_mut(&2346).unwrap();
    s.input = core::mem::take(&mut s.output);
    s.at = 0;
    let mut f = Framed::new(Endian::Big, Limits::default());
    let mut sent = Vec::new();
    loop {
        let message = receive(&mut f, s).unwrap();
        match message.get("FileData") {
            Some(data) => sent.extend_from_slice(data.data().unwrap()),
            None => break,
        }
    }
    assert_eq!(sent, payload);
}
