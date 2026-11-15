use super::super::{
    plist,
    store::tests::fixture,
    stream::tests::{Memory, receive},
};
use super::*;
use stela_contracts::refuse;
fn ready() -> (
    Session,
    Answers<stela_formats::payload::test_support::Memory>,
) {
    let (session, answers, _) = ready_with(fixture(b"payload", false));
    (session, answers)
}
type Fixture = super::super::store::Store<stela_formats::payload::test_support::Memory>;
/// A session past StartRestore, and the StartRestore request it sent.
fn ready_with(
    store: Fixture,
) -> (
    Session,
    Answers<stela_formats::payload::test_support::Memory>,
    Value,
) {
    ready_with_fdr(store, None)
}
fn ready_with_fdr(
    store: Fixture,
    diagnostic: Option<super::super::fdr::Diagnostic>,
) -> (
    Session,
    Answers<stela_formats::payload::test_support::Memory>,
    Value,
) {
    let mut s = Session::new();
    let mut a = Answers::new(store);
    a.fdr_diagnostic = diagnostic;
    a.store.prepare().unwrap();
    assert_eq!(
        s.query_type()
            .unwrap()
            .required("Request")
            .unwrap()
            .string()
            .unwrap(),
        "QueryType"
    );
    s.receive(
        Value::dict([
            ("Type", Value::from("com.apple.mobile.restored")),
            ("Result", Value::from("Success")),
            ("RestoreProtocolVersion", Value::Integer(15)),
        ]),
        &mut a,
    )
    .unwrap();
    s.query_value("HardwareInfo").unwrap();
    s.receive(
        Value::dict([(
            "HardwareInfo",
            Value::dict([
                ("ChipID", Value::Integer(0x8112)),
                ("BoardID", Value::Integer(0x24)),
                // An ECID with bit 63 set arrives Signed.
                ("UniqueChipID", Value::Signed(-494_042_649_752_996_581)),
                ("ApNonce", Value::Data(vec![1, 2, 3])),
            ]),
        )]),
        &mut a,
    )
    .unwrap();
    let start = s.start(&a).unwrap();
    assert_eq!(
        start
            .required("RestoreProtocolVersion")
            .unwrap()
            .uint()
            .unwrap(),
        15
    );
    assert!(
        start
            .required("RestoreOptions")
            .unwrap()
            .get("recoveryOSPartitionSize")
            .is_none()
    );
    s.started().unwrap();
    (s, a, start)
}
/// The options StartRestore sent for `store`'s identity.
fn start_options(store: Fixture) -> Value {
    let (_, _, start) = ready_with(store);
    start.required("RestoreOptions").unwrap().clone()
}
#[test]
fn an_update_identity_keeps_the_partitions_an_erase_identity_lays_out() {
    use super::super::store::{
        ERASE_BEHAVIOR, Store, UPDATE_BEHAVIOR,
        tests::{ERASE, UPDATE, parts},
    };
    for (behavior, creates) in [(ERASE, true), (UPDATE, false)] {
        let (reader, manifest) = parts(b"payload", false, &[], &[], behavior);
        let options = start_options(Store::open(reader, manifest, 0).unwrap());
        let flag = |key: &str| options.required(key).unwrap().boolean().unwrap();
        assert_eq!(flag("CreateFilesystemPartitions"), creates);
        assert_eq!(
            options
                .required("AuthInstallRestoreBehavior")
                .unwrap()
                .string()
                .unwrap(),
            if creates {
                ERASE_BEHAVIOR
            } else {
                UPDATE_BEHAVIOR
            }
        );
        assert_eq!(
            options
                .required("AuthInstallRestoreVariant")
                .unwrap()
                .string()
                .unwrap(),
            behavior.0
        );
        // Both restore the system and the paired recoveryOS.
        assert!(flag("ShouldRestoreSystemImage") && flag("InstallRecoveryOS"));
        let variant = |key: &str| options.required(key).unwrap().string().unwrap();
        assert_eq!(variant("AuthInstallRecoveryOSVariant"), "macOS Customer");
        assert_eq!(variant("MacOSVariant"), "macOS Customer");
    }
}
fn request(kind: &str) -> Value {
    Value::dict([
        ("MsgType", Value::from("AsyncDataRequestMsg")),
        ("DataType", Value::from(kind)),
        ("DataPort", Value::Integer(2345)),
        ("Arguments", Value::dict([])),
    ])
}
#[test]
fn a_signed_amr_error_is_a_reported_failure() {
    let (mut session, mut answers) = ready();
    let status = Value::dict([
        ("MsgType", Value::from("StatusMsg")),
        ("Status", Value::Integer(0)),
        ("AMRError", Value::Signed(-1)),
    ]);
    let Err(Error::Refused { reason, .. }) = session.receive(status, &mut answers) else {
        panic!("a failed status must refuse")
    };
    assert_eq!(reason, refuse!("device final status is failure"));
    let (mut session, mut answers) = ready();
    let status = Value::dict([
        ("MsgType", Value::from("StatusMsg")),
        ("Status", Value::Signed(-1)),
    ]);
    let Err(Error::Refused { reason, .. }) = session.receive(status, &mut answers) else {
        panic!("a negative status must refuse as a failure")
    };
    assert_eq!(reason, refuse!("device final status is failure"));
}
#[test]
fn progress_is_a_percentage_or_minus_one_with_the_operation_weights() {
    let progress = |value: Value, weights: Option<Value>| {
        let (mut session, mut answers) = ready();
        let mut message = Value::dict([
            ("MsgType", Value::from("ProgressMsg")),
            ("Operation", Value::Integer(30)),
            ("Progress", value),
        ]);
        if let (Value::Dict(fields), Some(weights)) = (&mut message, weights) {
            fields.insert("OperationWeights".into(), weights);
        }
        session.receive(message, &mut answers).is_ok()
    };
    // 26A434 restored's announcement of the progress bar's weights.
    let weights = Value::dict([
        (
            "operation_ids",
            Value::dict([("id_18", Value::Integer(18))]),
        ),
        ("operation_progresses", Value::dict([])),
        (
            "operation_weights",
            Value::dict([("id_18", Value::Integer(200))]),
        ),
        ("weight_count", Value::Integer(98)),
    ]);
    assert!(progress(Value::Signed(-1), Some(weights)));
    assert!(progress(Value::Integer(100), None));
    assert!(!progress(Value::Integer(101), None));
    assert!(!progress(Value::Signed(-2), None));
    let malformed = Value::dict([("weight_count", Value::Integer(1))]);
    assert!(!progress(Value::Signed(-1), Some(malformed)));
}
#[test]
fn device_reports_continue_and_baseband_or_provisioning_messages_refuse() {
    for kind in [
        "RestoredCrash",
        "CrashLog",
        "AsyncWait",
        "FDRSubmit",
        "RestoreProtocol",
    ] {
        let (mut session, mut answers) = ready();
        let message = Value::dict([("MsgType", Value::from(kind))]);
        assert!(matches!(
            session.receive(message, &mut answers),
            Ok(Action::Event(_))
        ));
        assert_eq!(session.phase(), Phase::Restoring);
    }
    for kind in ["BBUpdateStatusMsg", "ProvisioningInfo", "ProvisioningAck"] {
        let (mut session, mut answers) = ready();
        let message = Value::dict([("MsgType", Value::from(kind))]);
        assert!(session.receive(message, &mut answers).is_err());
        assert_eq!(session.phase(), Phase::Failed);
    }
}
#[test]
fn synthetic_public_protocol_transcript_keeps_control_live_and_requires_ack_write() {
    let (mut session, mut answers) = ready();
    let mut r = request("SourceBootObjectV5");
    let Value::Dict(d) = &mut r else {
        unreachable!()
    };
    d.insert(
        "Arguments".into(),
        Value::dict([("ImageName", Value::from("OS"))]),
    );
    let Action::Transfer(mut job) = session.receive(r, &mut answers).unwrap() else {
        panic!("missing job")
    };
    assert_eq!(job.port, Some(2345));
    session
        .receive(
            Value::dict([
                ("MsgType", Value::from("ProgressMsg")),
                ("Operation", Value::Integer(13)),
                ("Progress", Value::Integer(50)),
            ]),
            &mut answers,
        )
        .unwrap();
    let mut stream = Memory::new(Vec::new(), 1);
    let mut complete = false;
    for _ in 0..5000 {
        if let Poll::Ready(v) = job.poll(&mut stream, &mut answers.store) {
            v.unwrap();
            complete = true;
            break;
        }
    }
    assert!(complete);
    session.finish_job(job.id, Ok(())).unwrap();
    stream.input = stream.output;
    stream.output = Vec::new();
    let mut frame = Framed::new(Endian::Big, Limits::default());
    assert_eq!(
        receive(&mut frame, &mut stream)
            .unwrap()
            .required("FileData")
            .unwrap()
            .data()
            .unwrap(),
        b"payload"
    );
    assert!(
        receive(&mut frame, &mut stream)
            .unwrap()
            .required("FileDataDone")
            .unwrap()
            .boolean()
            .unwrap()
    );
    session
        .receive(
            Value::dict([
                ("MsgType", Value::from("CheckpointMsg")),
                ("CHECKPOINT_ID", Value::Integer(7)),
                ("CHECKPOINT_NAME", Value::from("test")),
                ("CHECKPOINT_RESULT", Value::Signed(-1)),
                ("CHECKPOINT_COMPLETE", Value::Boolean(true)),
            ]),
            &mut answers,
        )
        .unwrap();
    let Action::FinalAck(ack) = session
        .receive(
            Value::dict([
                ("MsgType", Value::from("StatusMsg")),
                ("Status", Value::Integer(0)),
            ]),
            &mut answers,
        )
        .unwrap()
    else {
        panic!("no final ack")
    };
    assert_eq!(
        ack.required("MsgType").unwrap().string().unwrap(),
        "ReceivedFinalStatusMsg"
    );
    assert_eq!(session.phase(), Phase::FinalAck);
    session.final_ack_sent().unwrap();
    assert_eq!(session.phase(), Phase::Complete);
    assert!(session.ledger().len() >= 6);
}
/// FileData frames stay FILE_CHUNK bytes while the member is read in
/// MAX_READ_BYTES runs, across a run boundary, from a deflated member.
#[test]
fn a_deflated_member_streams_whole_in_file_chunks_across_read_runs() {
    let data: Vec<u8> = (0..MAX_READ_BYTES + 3 * FILE_CHUNK + 5)
        .map(|i| (i * 7 / 3) as u8)
        .collect();
    let mut answers = Answers::new(fixture(&data, true));
    answers.store.prepare().unwrap();
    let request = Value::dict([
        ("DataType", Value::from("SourceBootObjectV4")),
        ("Arguments", Value::dict([("ImageName", Value::from("OS"))])),
    ]);
    let mut job = Job::new(
        1,
        Some(2345),
        request.clone(),
        answers.plan(&request).unwrap(),
    );
    let mut stream = Memory::new(Vec::new(), 1 << 20);
    // The native poll cannot inflate or verify source data while its
    // caller holds the guest's devices.
    assert!(
        job.poll_with_source(&mut stream, &mut answers.store, true)
            .is_pending()
    );
    assert!(stream.output.is_empty());

    job.prefetch(&mut answers.store, 1).unwrap();
    for _ in 0..2 * MAX_READ_BYTES / FILE_CHUNK + 2 {
        assert!(
            job.poll_with_source(&mut stream, &mut answers.store, true)
                .is_pending()
        );
        if !job.sending() && job.offset == MAX_READ_BYTES as u64 {
            break;
        }
    }
    assert_eq!(job.offset, MAX_READ_BYTES as u64);
    assert!(!job.sending());
    let sent = stream.output.len();
    assert!(
        job.poll_with_source(&mut stream, &mut answers.store, true)
            .is_pending()
    );
    assert_eq!(stream.output.len(), sent);

    job.prefetch(&mut answers.store, 1).unwrap();
    for _ in 0..16 {
        if let Poll::Ready(result) = job.poll_with_source(&mut stream, &mut answers.store, true) {
            result.unwrap();
            break;
        }
    }
    assert!(job.done);
    stream.input = core::mem::take(&mut stream.output);
    let mut frame = Framed::new(Endian::Big, Limits::default());
    let mut got = Vec::new();
    loop {
        let v = receive(&mut frame, &mut stream).unwrap();
        if let Some(done) = v.get("FileDataDone") {
            assert!(done.boolean().unwrap());
            break;
        }
        let chunk = v.required("FileData").unwrap().data().unwrap();
        assert!(chunk.len() <= FILE_CHUNK);
        got.extend_from_slice(chunk);
    }
    assert_eq!(got, data);
}
#[test]
fn an_optional_missing_global_manifest_sends_only_file_data_done() {
    let mut answers = Answers::new(fixture(b"payload", false));
    answers.store.prepare().unwrap();
    let request = Value::dict([
        ("DataType", Value::from("SourceBootObjectV4")),
        (
            "Arguments",
            Value::dict([
                ("ImageName", Value::from("__GlobalManifest__")),
                ("Variant", Value::from("macOS Customer")),
                ("GlobalManifestPrefix", Value::from("centauri/centauri")),
                ("GlobalManifestOptional", Value::Boolean(true)),
            ]),
        ),
    ]);
    let mut job = Job::new(
        1,
        Some(2345),
        request.clone(),
        answers.plan(&request).unwrap(),
    );
    let mut stream = Memory::new(Vec::new(), 1 << 20);
    while job.poll(&mut stream, &mut answers.store).is_pending() {}
    stream.input = core::mem::take(&mut stream.output);
    let mut frame = Framed::new(Endian::Big, Limits::default());
    let done = receive(&mut frame, &mut stream).unwrap();
    assert!(done.get("FileData").is_none());
    assert!(done.required("FileDataDone").unwrap().boolean().unwrap());
}
/// A stream that takes every byte lets one poll move a whole file of
/// several FileData frames, and its end.
#[test]
fn one_poll_moves_every_frame_a_ready_stream_takes() {
    use stela_soc::usb::mux::{Mux, StreamId, test_support::connected};

    struct MuxSink {
        mux: Mux,
        stream: StreamId,
        bytes: Vec<u8>,
        writes: Vec<usize>,
    }
    impl ByteStream for MuxSink {
        fn read(&mut self, _: &mut [u8]) -> Poll<Result<usize>> {
            Poll::Pending
        }
        fn write(&mut self, b: &[u8]) -> Poll<Result<usize>> {
            let n = self.mux.write(self.stream, b).map_err(Error::Transport)?;
            if n == 0 {
                return Poll::Pending;
            }
            let packet = self.mux.take_packet().expect("mux write emitted a packet");
            // Mux v2 header and fixed TCP header.
            assert_eq!(packet.len(), 36 + n);
            self.bytes.extend_from_slice(&packet[36..]);
            self.writes.push(n);
            Poll::Ready(Ok(n))
        }
    }
    let data = vec![5; FILE_BATCH_BYTES + 1];
    let mut answers = Answers::new(fixture(&data, false));
    answers.store.prepare().unwrap();
    let request = Value::dict([
        ("DataType", Value::from("SourceBootObjectV4")),
        ("Arguments", Value::dict([("ImageName", Value::from("OS"))])),
    ]);
    let plan = answers.plan(&request).unwrap();
    let mut job = Job::new(1, Some(2345), request, plan);
    let (mux, stream) = connected();
    let mut stream = MuxSink {
        mux,
        stream,
        bytes: Vec::new(),
        writes: Vec::new(),
    };
    assert!(matches!(
        job.poll(&mut stream, &mut answers.store),
        Poll::Ready(Ok(()))
    ));
    let mut expected = Vec::new();
    for chunk in data.chunks(FILE_CHUNK) {
        let frame = plist::encode_binary(
            &Value::dict([("FileData", Value::Data(chunk.to_vec()))]),
            Limits::default(),
        )
        .unwrap();
        expected.extend((frame.len() as u32).to_be_bytes());
        expected.extend(frame);
    }
    let done = plist::encode_binary(
        &Value::dict([("FileDataDone", Value::Boolean(true))]),
        Limits::default(),
    )
    .unwrap();
    expected.extend((done.len() as u32).to_be_bytes());
    expected.extend(done);
    assert_eq!(stream.bytes, expected);
    assert_eq!(stream.writes.len(), 3);
    assert_eq!(stream.writes[0], FILE_BATCH_BYTES + 4 * 104);
}

#[test]
fn blocked_file_data_keeps_its_offset_and_a_partial_write_refusal_is_terminal() {
    let data = vec![5; MAX_READ_BYTES + 1];
    let mut answers = Answers::new(fixture(&data, false));
    answers.store.prepare().unwrap();
    let request = Value::dict([
        ("DataType", Value::from("SourceBootObjectV4")),
        ("Arguments", Value::dict([("ImageName", Value::from("OS"))])),
    ]);
    let plan = answers.plan(&request).unwrap();
    let mut job = Job::new(1, Some(2345), request, plan);
    let mut stream = Memory::new(Vec::new(), 31);
    job.prefetch(&mut answers.store, 1).unwrap();
    let offset = (MAX_READ_BYTES - FILE_BATCH_BYTES) as u64;
    job.offset = offset;
    assert!(
        job.poll_with_source(&mut stream, &mut answers.store, true)
            .is_pending()
    );
    assert_eq!((job.offset, stream.output.len()), (offset, 0));
    job.prefetch(&mut answers.store, 1).unwrap();
    assert_eq!(job.window.at, 0);
    assert!(
        job.poll_with_source(&mut stream, &mut answers.store, true)
            .is_pending()
    );
    assert_eq!((job.offset, stream.output.len()), (offset, 31));
    let last_frame = 3 * (FILE_CHUNK + 104);
    stream.chunk = last_frame - 31;
    assert!(
        job.poll_with_source(&mut stream, &mut answers.store, true)
            .is_pending()
    );
    assert_eq!((job.offset, stream.output.len()), (offset, last_frame));
    job.prefetch(&mut answers.store, 1).unwrap();
    assert_eq!(job.window.at, MAX_READ_BYTES as u64);
    stream.chunk = 0;
    assert!(matches!(
        job.poll_with_source(&mut stream, &mut answers.store, true),
        Poll::Ready(Err(_))
    ));
    stream.chunk = FILE_BATCH_BYTES;
    assert!(matches!(
        job.poll_with_source(&mut stream, &mut answers.store, true),
        Poll::Ready(Err(_))
    ));
    assert_eq!((job.offset, stream.output.len()), (offset, last_frame));
}
#[test]
fn unknown_updater_recovery_policy_and_failure_status_keep_full_context() {
    for kind in [
        "RecoveryOSLocalPolicy",
        "FirmwareUpdaterData",
        "UnknownRequiredType",
    ] {
        let (mut s, mut a) = ready();
        let mut r = request(kind);
        let Value::Dict(d) = &mut r else {
            unreachable!()
        };
        d.insert(
            "Arguments".into(),
            Value::dict([
                ("MessageArgUpdaterName", Value::from("UnqualifiedUpdater")),
                ("Nonce", Value::Data(vec![9])),
            ]),
        );
        let Err(Error::Refused { request, .. }) = s.receive(r.clone(), &mut a) else {
            panic!("missing refusal")
        };
        assert_eq!(*request, r);
        assert_eq!(s.phase(), Phase::Failed);
    }
    let (mut s, mut a) = ready();
    assert!(
        s.receive(
            Value::dict([
                ("MsgType", Value::from("StatusMsg")),
                ("Status", Value::Integer(0)),
                ("AMRError", Value::Integer(5))
            ]),
            &mut a
        )
        .is_err()
    );
    assert!(s.final_ack_sent().is_err());
}
#[test]
fn no_final_success_with_outstanding_or_failed_jobs() {
    let (mut s, mut a) = ready();
    let Action::Transfer(job) = s.receive(request("FDRTrustData"), &mut a).unwrap() else {
        panic!()
    };
    assert_eq!(s.outstanding(), 1);
    assert!(
        s.finish_job(job.id, Err(Error::Transport("lost stream".into())))
            .is_err()
    );
    assert_eq!(s.phase(), Phase::Failed);
    assert!(
        s.receive(
            Value::dict([
                ("MsgType", Value::from("StatusMsg")),
                ("Status", Value::Integer(0))
            ]),
            &mut a
        )
        .is_err()
    );
    let (mut s, mut a) = ready();
    s.receive(request("ReceiptManifest"), &mut a).unwrap();
    assert!(
        s.receive(
            Value::dict([
                ("MsgType", Value::from("StatusMsg")),
                ("Status", Value::Integer(0))
            ]),
            &mut a
        )
        .is_err()
    );
}

#[test]
fn fdr_commit_never_acknowledges_unpersisted_records() {
    let (mut session, mut answers) = ready();
    answers.fdr_diagnostic = Some(fdr_fixture(&answers.store, Value::dict([])));
    let mut commit = request("FDRMemoryCommit");
    let Value::Dict(entries) = &mut commit else {
        unreachable!()
    };
    entries.insert(
        "Arguments".into(),
        Value::dict([(
            "FDRMemoryStoreData",
            Value::dict([("appv-own", Value::Data(vec![4, 5, 6]))]),
        )]),
    );
    let Err(Error::Refused { reason, request }) = session.receive(commit.clone(), &mut answers)
    else {
        panic!("commit was acknowledged without persistence")
    };
    assert!(reason.to_string().contains("durable commit"));
    assert_eq!(*request, commit);
    assert_eq!(session.phase(), Phase::Failed);
}

#[test]
fn fdr_import_is_opt_in_and_preserves_the_original_reply_schema() {
    let store = fixture(b"payload", false);
    let records = Value::dict([("appv-own", Value::Data(vec![1, 2, 3]))]);
    let diagnostic = fdr_fixture(&store, records.clone());
    let (_, mut answers, start) = ready_with_fdr(store, Some(diagnostic));
    assert_eq!(
        start
            .required("RestoreOptions")
            .unwrap()
            .required("FDRMemoryStorePath")
            .unwrap()
            .string()
            .unwrap(),
        super::super::fdr::FILE
    );
    let Plan::Reply(reply) = answers
        .plan(&Value::dict([("DataType", Value::from("FDRTrustData"))]))
        .unwrap()
    else {
        panic!("not an FDR reply")
    };
    assert_eq!(reply, Value::dict([("FDRMemoryStoreData", records)]));
    let (_, mut ordinary, start) = ready_with(fixture(b"payload", false));
    assert!(
        start
            .required("RestoreOptions")
            .unwrap()
            .get("FDRMemoryStorePath")
            .is_none()
    );
    let Plan::Reply(reply) = ordinary
        .plan(&Value::dict([("DataType", Value::from("FDRTrustData"))]))
        .unwrap()
    else {
        panic!("not an FDR reply")
    };
    assert_eq!(reply, Value::dict([]));
}

fn fdr_fixture(store: &Fixture, records: Value) -> super::super::fdr::Diagnostic {
    let input = Value::dict([
        ("DiagnosticVersion", Value::Integer(1)),
        ("ECID", Value::Signed(-494_042_649_752_996_581)),
        ("ChipID", Value::Integer(0x8112)),
        ("BoardID", Value::Integer(0x24)),
        (
            "Build",
            Value::from(store.manifest().product_build.as_str()),
        ),
        ("Model", Value::from(store.manifest().model.as_str())),
        ("FDRMemoryStoreData", records.clone()),
    ]);
    let bytes = super::super::plist::encode_binary(&input, Limits::default()).unwrap();
    super::super::fdr::Diagnostic::decode(&bytes).unwrap()
}

#[test]
fn fdr_commit_is_durable_before_acknowledgement_and_keeps_binding() {
    use crate::pc::fdr_store::{self, Journal};
    use alloc::rc::Rc;
    use core::cell::RefCell;
    use stela_drivers::block::RamDisk;
    let store = fixture(b"payload", false);
    let diagnostic = fdr_fixture(&store, Value::dict([]));
    let seed = diagnostic.with_records(Value::dict([])).unwrap();
    let mut disk = RamDisk::new("FDR", (fdr_store::FILE_BYTES / 512) as u64, 512);
    disk.identity.features.write_cache = true;
    disk.bytes = fdr_store::initial_image(&seed).unwrap();
    let mut journal = Journal::open(&mut disk).unwrap();
    let disk = Rc::new(RefCell::new(disk));
    let (mut session, mut answers, _) = ready_with_fdr(store, Some(diagnostic));
    let writer = Rc::clone(&disk);
    answers.fdr_commit = Some(Box::new(move |bytes| {
        journal
            .commit(&mut *writer.borrow_mut(), bytes)
            .map_err(|error| Error::Transport(refuse!("FDR commit: {error}", ?error)))
    }));
    let records = Value::dict([("class-own", Value::Data(vec![9, 8, 7]))]);
    let commit = Value::dict([
        ("MsgType", Value::from("DataRequestMsg")),
        ("DataType", Value::from("FDRMemoryCommit")),
        (
            "Arguments",
            Value::dict([("FDRMemoryStoreData", records.clone())]),
        ),
    ]);
    assert!(matches!(
        session.receive(commit, &mut answers).unwrap(),
        Action::Transfer(_)
    ));
    let reopened = Journal::open(&mut *disk.borrow_mut()).unwrap();
    let stored = super::super::fdr::Diagnostic::decode(reopened.snapshot().unwrap()).unwrap();
    assert!(stored.same_identity(answers.fdr_diagnostic.as_ref().unwrap()));
    assert_eq!(
        stored.reply(),
        Value::dict([("FDRMemoryStoreData", records)])
    );
    assert_eq!(disk.borrow().flushes, 2);
    answers.fdr_commit = Some(Box::new(|_| {
        Err(Error::Transport("injected persistence failure".into()))
    }));
    let request = Value::dict([
        ("DataType", Value::from("FDRMemoryCommit")),
        (
            "Arguments",
            Value::dict([("FDRMemoryStoreData", Value::dict([]))]),
        ),
    ]);
    assert!(answers.plan(&request).is_err());
    assert!(stored.same_identity(answers.fdr_diagnostic.as_ref().unwrap()));
    assert_eq!(
        stored.reply(),
        answers.fdr_diagnostic.as_ref().unwrap().reply()
    );
}

#[test]
fn file_progress_excludes_queued_bytes_and_framing_and_survives_segment_changes() {
    let mut store = fixture(b"payload", false);
    let mut job = Job::new(
        99,
        None,
        Value::dict([
            ("DataType", Value::from("SourceBootObjectV4")),
            (
                "Arguments",
                Value::dict([("ImageName", Value::from("Cryptex1,RosettaOS"))]),
            ),
        ]),
        Plan::File(vec![
            Segment::Bytes(vec![1; 17]),
            Segment::Bytes(vec![2; 31]),
        ]),
    );
    let mut sink = Memory::new(Vec::new(), 1);
    let initial = job.file_progress().unwrap();
    assert_eq!(
        (initial.sent, initial.total, initial.complete),
        (0, 48, false)
    );
    assert_eq!(initial.name, "Cryptex1,RosettaOS");
    assert!(job.poll(&mut sink, &mut store).is_pending());
    assert_eq!(job.file_progress().unwrap().sent, 0);
    let mut previous = 0;
    let mut complete = false;
    for _ in 0..2000 {
        let result = job.poll(&mut sink, &mut store);
        let progress = job.file_progress().unwrap();
        assert!(progress.sent >= previous && progress.sent <= 48);
        previous = progress.sent;
        if let Poll::Ready(result) = result {
            result.unwrap();
            assert!(progress.complete);
            assert_eq!(progress.sent, 48);
            complete = true;
            break;
        }
    }
    assert!(complete);
    let mut wire = Memory::new(sink.output, usize::MAX);
    let mut frame = Framed::new(Endian::Big, Limits::default());
    assert_eq!(
        receive(&mut frame, &mut wire)
            .unwrap()
            .required("FileData")
            .unwrap()
            .data()
            .unwrap()
            .len(),
        17
    );
    assert_eq!(
        receive(&mut frame, &mut wire)
            .unwrap()
            .required("FileData")
            .unwrap()
            .data()
            .unwrap()
            .len(),
        31
    );
    assert!(
        receive(&mut frame, &mut wire)
            .unwrap()
            .required("FileDataDone")
            .unwrap()
            .boolean()
            .unwrap()
    );
}

#[test]
fn file_progress_keeps_64_bit_sizes_without_reading_the_member() {
    let mut job = Job::new(
        1,
        None,
        Value::dict([]),
        Plan::File(vec![
            Segment::Bytes(vec![0; 17]),
            Segment::Member {
                path: "large.dmg".into(),
                offset: 0,
                bytes: 6_484_393_984,
            },
        ]),
    );
    job.index = 1;
    job.offset = 5_000_000_000;
    job.file_pending = FILE_BATCH_BYTES;
    let progress = job.file_progress().unwrap();
    assert_eq!(progress.total, 6_484_394_001);
    assert_eq!(progress.sent, 5_000_000_017);
    assert_eq!(progress.member, "large.dmg");
    assert!(!progress.complete);
}
