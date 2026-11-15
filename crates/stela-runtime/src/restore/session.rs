//! Restored protocol 15 session and independently scheduled transfer jobs.
//! No fixed request ordering is assumed. A final status is successful only
//! after all jobs have completed and ReceivedFinalStatusMsg has been written.
use super::{
    Error, Result,
    answers::{Answers, Plan, Segment},
    asr,
    plist::{Limits, Value},
    route::Route,
    store::{MAX_READ_BYTES, Store},
    stream::{ByteStream, Endian, Framed, Output},
};
use alloc::{boxed::Box, collections::BTreeMap, string::String, vec::Vec};
use core::task::Poll;
use stela_contracts::refuse;
use stela_formats::payload::ReadAt;
pub const PORT: u16 = 62078;
pub const PROTOCOL: u64 = 15;
const MAX_JOBS: usize = 16;
const MAX_LEDGER_BYTES: usize = 16 * 1024 * 1024;
const FILE_CHUNK: usize = 8192;
// Four framed chunks fit a single 48 KiB mux write with room for headers.
const FILE_BATCH_BYTES: usize = 4 * FILE_CHUNK;
const _: () = assert!(MAX_READ_BYTES.is_multiple_of(FILE_BATCH_BYTES));
/// Every message type the host understands, with the value the public host
/// sends for it (idevicerestore 4b3e847e `restore_supported_message_types`,
/// "extracted from ac2"). Restored's `is_host_compatible` compares this
/// dictionary with the messages it can send and fails the restore with
/// AMRError 10 when one it marks required is missing.
const MESSAGE_TYPES: [(&str, bool); 18] = [
    ("AsyncDataRequestMsg", true),
    ("AsyncWait", true),
    ("BBUpdateStatusMsg", false),
    ("CheckpointMsg", true),
    ("CrashLog", true),
    ("DataRequestMsg", false),
    ("FDRSubmit", true),
    ("MsgType", false),
    ("PreviousRestoreLogMsg", false),
    ("ProgressMsg", false),
    ("ProvisioningAck", false),
    ("ProvisioningInfo", false),
    ("ProvisioningStatusMsg", false),
    ("ReceivedFinalStatusMsg", false),
    ("RestoreAttestation", true),
    ("RestoreProtocol", true),
    ("RestoredCrash", true),
    ("StatusMsg", false),
];
/// The data types the host serves for an `AsyncDataRequestMsg`, on the
/// request's own DataPort. 26A434 `restored_external` takes a type as
/// supported this way only when this dictionary names it (R 0x100022dd4);
/// without the dictionary, only the system image and the recoveryOS image.
/// Its AEA transfer asks for an image's content key through `URLAsset` only
/// then (R 0x10005d6e8); otherwise it fails the image with error 1106.
const ASYNC_DATA_TYPES: [&str; 3] = ["RecoveryOSASRImage", "SystemImageData", "URLAsset"];
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    New,
    QueryType,
    Ready,
    QueryValue,
    Starting,
    Restoring,
    FinalAck,
    Complete,
    Failed,
}
pub enum Action {
    None,
    Event(Value),
    Transfer(Box<Job>),
    FinalAck(Value),
}
/// A fresh per-session ledger. The owner may persist it with its input build
/// ID and source hashes; no previous session's nonces/checkpoints are reused.
pub struct Session {
    phase: Phase,
    route: Route,
    query: Option<String>,
    hardware: Option<Value>,
    ledger: Vec<Value>,
    ledger_bytes: usize,
    next_job: u64,
    outstanding: BTreeMap<u64, Value>,
}
impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}
impl Session {
    pub fn new() -> Self {
        Self {
            phase: Phase::New,
            route: Route::Stock,
            query: None,
            hardware: None,
            ledger: Vec::new(),
            ledger_bytes: 0,
            next_job: 1,
            outstanding: BTreeMap::new(),
        }
    }
    pub fn phase(&self) -> Phase {
        self.phase
    }
    pub fn ledger(&self) -> &[Value] {
        &self.ledger
    }
    pub fn outstanding(&self) -> usize {
        self.outstanding.len()
    }
    pub fn query_type(&mut self) -> Result<Value> {
        if self.phase != Phase::New {
            return Err(Error::Invalid("QueryType phase"));
        }
        self.phase = Phase::QueryType;
        Ok(Value::dict([("Request", Value::from("QueryType"))]))
    }
    pub fn query_value(&mut self, key: &str) -> Result<Value> {
        if self.phase != Phase::Ready || key.is_empty() || key.len() > 256 {
            return Err(Error::Invalid("QueryValue phase/key"));
        }
        self.query = Some(key.into());
        self.phase = Phase::QueryValue;
        Ok(Value::dict([
            ("Request", Value::from("QueryValue")),
            ("QueryKey", Value::from(key)),
        ]))
    }
    pub fn start<R: ReadAt>(&mut self, answers: &Answers<R>) -> Result<Value> {
        self.start_for(answers, Route::Stock)
    }
    pub fn start_for<R: ReadAt>(&mut self, answers: &Answers<R>, route: Route) -> Result<Value> {
        if self.phase != Phase::Ready {
            return Err(Error::Invalid("StartRestore phase"));
        }
        if !answers.store.prepared() {
            return Err(Error::Invalid(
                "payload must be prepared before StartRestore",
            ));
        }
        let hw = self
            .hardware
            .as_ref()
            .ok_or(Error::Invalid("HardwareInfo not queried"))?;
        let identity = answers.store.identity();
        for (field, key) in [("ChipID", "ApChipID"), ("BoardID", "ApBoardID")] {
            if hw.required(field)?.uint()? != identity.number(key)? {
                return Err(super::refuse("hardware/BuildManifest mismatch", hw));
            }
        }
        hw.required("UniqueChipID")?.bits()?;
        if let Some(diagnostic) = &answers.fdr_diagnostic {
            let manifest = answers.store.manifest();
            diagnostic.check(hw, &manifest.product_build, &manifest.model)?;
        }
        let info = identity.info()?;
        let behavior = info.required("RestoreBehavior")?;
        let mut opts = BTreeMap::new();
        for key in [
            "AutoBootDelay",
            "FlashNOR",
            "FormatForAPFS",
            "InstallRecoveryOS",
            "RecoveryOSUnpack",
            "ShouldRestoreSystemImage",
        ] {
            opts.insert(
                key.into(),
                if key == "AutoBootDelay" {
                    Value::Integer(0)
                } else {
                    Value::Boolean(true)
                },
            );
        }
        // An erase restore lays out the disk anew; an update restore keeps
        // its partitions and the user's data. restored_update sets this
        // itself (false, R_update 0x10001a39c..0x10001a3c4), and
        // restored_external takes the host's value (R 0x100027008), so the
        // host says what the identity does.
        opts.insert(
            "CreateFilesystemPartitions".into(),
            Value::Boolean(behavior.string()? == super::store::ERASE_BEHAVIOR),
        );
        opts.insert("AllowUntetheredRestore".into(), Value::Boolean(false));
        opts.insert("AuthInstallRestoreBehavior".into(), behavior.clone());
        opts.insert(
            "AuthInstallRestoreVariant".into(),
            info.required("Variant")?.clone(),
        );
        // A macOS identity names the variant of its paired recoveryOS.
        // restored requests the system image only when the host sends it
        // (R 0x100027018), as idevicerestore does with this value.
        if let Some(v) = info.get("MacOSVariant") {
            opts.insert("MacOSVariant".into(), v.clone());
            opts.insert("AuthInstallRecoveryOSVariant".into(), v.clone());
        }
        if let Some(v) = info.get("MinimumSystemPartition") {
            opts.insert("SystemPartitionSize".into(), v.clone());
        }
        opts.insert("SupportedDataTypes".into(), answers.supported());
        opts.insert(
            "SupportedMessageTypes".into(),
            Value::Dict(
                MESSAGE_TYPES
                    .into_iter()
                    .map(|(name, value)| (name.into(), Value::Boolean(value)))
                    .collect(),
            ),
        );
        opts.insert(
            "SupportedAsyncDataTypes".into(),
            Value::Dict(
                ASYNC_DATA_TYPES
                    .into_iter()
                    .map(|name| (name.into(), Value::Boolean(true)))
                    .collect(),
            ),
        );
        let mut options = Value::Dict(opts);
        if answers.fdr_diagnostic.is_some() {
            // R70dc0..70df4 selects Memory by presence; R70ed0 imports
            // the host dictionary. This names a host diagnostic, not a
            // target filesystem path or an installed Hardware store.
            let Value::Dict(entries) = &mut options else {
                unreachable!()
            };
            entries.insert("FDRMemoryStorePath".into(), Value::from(super::fdr::FILE));
        }
        route.options(&mut options)?;
        self.route = route;
        self.phase = Phase::Starting;
        Ok(Value::dict([
            ("Request", Value::from("StartRestore")),
            ("RestoreProtocolVersion", Value::Integer(PROTOCOL)),
            ("RestoreOptions", options),
        ]))
    }
    /// Call only after the entire StartRestore frame has been sent.
    pub fn started(&mut self) -> Result<()> {
        if self.phase != Phase::Starting {
            return Err(Error::Invalid("StartRestore write completion phase"));
        }
        self.phase = Phase::Restoring;
        Ok(())
    }
    pub fn receive<R: ReadAt>(
        &mut self,
        message: Value,
        answers: &mut Answers<R>,
    ) -> Result<Action> {
        let result = self.receive_inner(&message, answers);
        if result.is_err() && !matches!(result, Err(Error::Pending)) {
            self.phase = Phase::Failed;
        }
        result.map_err(|e| match e {
            Error::Refused { .. } | Error::Pending => e,
            _ => super::refuse(refuse!("restore message: {e}", ?e), &message),
        })
    }
    fn receive_inner<R: ReadAt>(&mut self, m: &Value, answers: &mut Answers<R>) -> Result<Action> {
        if matches!(
            self.phase,
            Phase::Failed | Phase::Complete | Phase::FinalAck | Phase::New | Phase::Starting
        ) {
            return Err(Error::Invalid("message out of session phase"));
        }
        let size = super::plist::encode_binary(m, Limits::default())?.len();
        if size > MAX_LEDGER_BYTES - self.ledger_bytes {
            return Err(Error::Limit("session ledger; persist before exhaustion"));
        }
        self.ledger.push(m.clone());
        self.ledger_bytes += size;
        if self.phase == Phase::QueryType {
            if m.required("Type")?.string()? != "com.apple.mobile.restored"
                || m.required("RestoreProtocolVersion")?.uint()? != PROTOCOL
                || m.required("Result")?.string()? != "Success"
            {
                return Err(Error::Invalid("unsupported restore service/protocol"));
            }
            self.phase = Phase::Ready;
            return Ok(Action::None);
        }
        if self.phase == Phase::QueryValue {
            let key = self
                .query
                .as_ref()
                .ok_or(Error::Invalid("QueryValue key absent"))?;
            let value = m.required(key)?.clone();
            if key == "HardwareInfo" {
                self.hardware = Some(value.clone());
            }
            self.query = None;
            self.phase = Phase::Ready;
            return Ok(Action::Event(value));
        }
        if self.phase != Phase::Restoring {
            return Err(Error::Invalid("unsolicited message before StartRestore"));
        }
        match m.required("MsgType")?.string()? {
            "DataRequestMsg" | "AsyncDataRequestMsg" => {
                if self.outstanding.len() == MAX_JOBS {
                    return Err(Error::Limit("outstanding restore jobs"));
                }
                let kind = m.required("DataType")?.string()?;
                if !self.route.allows(kind) {
                    return Err(super::refuse(
                        "Stela firmware option study: required request remains outside qualified global coverage",
                        m,
                    ));
                }
                let plan = if kind == "NORData" && !self.route.flashes_nor() {
                    answers.sep_firmware(m)
                } else {
                    answers.plan(m)
                };
                // A pending answer is asked again with the same message, which
                // the ledger records then.
                if matches!(plan, Err(Error::Pending)) {
                    self.ledger.pop();
                    self.ledger_bytes -= size;
                }
                let plan = plan?;
                let port = m
                    .get("DataPort")
                    .map(Value::uint)
                    .transpose()?
                    .map(|n| {
                        u16::try_from(n)
                            .ok()
                            .filter(|p| *p != 0)
                            .ok_or(Error::Invalid("DataPort outside 1..=65535"))
                    })
                    .transpose()?;
                let port = if matches!(plan, Plan::Asr(_)) {
                    Some(port.unwrap_or(asr::PORT))
                } else {
                    port
                };
                if matches!(plan, Plan::Raw(_)) && port.is_none() {
                    return Err(Error::Invalid("raw Bootability transfer requires DataPort"));
                }
                let id = self.next_job;
                self.next_job = self
                    .next_job
                    .checked_add(1)
                    .ok_or(Error::Limit("job identifier"))?;
                self.outstanding.insert(id, m.clone());
                Ok(Action::Transfer(Box::new(Job::new(
                    id,
                    port,
                    m.clone(),
                    plan,
                ))))
            }
            "RestoreAttestation" => {
                if self.outstanding.len() == MAX_JOBS {
                    return Err(Error::Limit("outstanding restore jobs"));
                }
                let id = self.next_job;
                self.next_job += 1;
                self.outstanding.insert(id, m.clone());
                Ok(Action::Transfer(Box::new(Job::new(
                    id,
                    None,
                    m.clone(),
                    Plan::Reply(Value::dict([(
                        "RestoreShouldAttest",
                        Value::Boolean(false),
                    )])),
                ))))
            }
            // `Progress` is a percentage, or -1 for an operation without one,
            // such as the `OperationWeights` announcement of the progress
            // bar's weights (idevicerestore logs the operation alone).
            "ProgressMsg" => {
                m.required("Operation")?.uint()?;
                match m.required("Progress")? {
                    Value::Signed(-1) => {}
                    progress if progress.uint()? <= 100 => {}
                    _ => return Err(Error::Invalid("restore progress outside 0..=100 or -1")),
                }
                if let Some(weights) = m.get("OperationWeights") {
                    weights.required("weight_count")?.uint()?;
                    for table in ["operation_ids", "operation_weights", "operation_progresses"] {
                        let Value::Dict(entries) = weights.required(table)? else {
                            return Err(Error::Invalid(
                                "operation weight table is not a dictionary",
                            ));
                        };
                        for value in entries.values() {
                            value.uint()?;
                        }
                    }
                }
                Ok(Action::Event(m.clone()))
            }
            "CheckpointMsg" => {
                m.required("CHECKPOINT_ID")?.uint()?;
                m.required("CHECKPOINT_NAME")?.string()?;
                let result = m.required("CHECKPOINT_RESULT")?;
                if !matches!(result, Value::Integer(_) | Value::Signed(_)) {
                    return Err(Error::Invalid("checkpoint result integer"));
                }
                m.required("CHECKPOINT_COMPLETE")?.boolean()?;
                Ok(Action::Event(m.clone()))
            }
            // Reports the public host logs and the device continues past.
            "PreviousRestoreLogMsg"
            | "RestoredCrash"
            | "CrashLog"
            | "AsyncWait"
            | "FDRSubmit"
            | "RestoreProtocol" => Ok(Action::Event(m.clone())),
            // J473 has no baseband, and Stela does no factory provisioning:
            // the host understands these messages but cannot act on them.
            "BBUpdateStatusMsg" => Err(super::refuse(
                "baseband update status on a board without a baseband",
                m,
            )),
            "ProvisioningInfo" | "ProvisioningAck" | "ProvisioningStatusMsg" => Err(super::refuse(
                "factory provisioning is not provided by this host",
                m,
            )),
            "StatusMsg" => {
                // A failure may come as a negative status.
                let status = match m.required("Status")? {
                    Value::Signed(status) => *status,
                    status => i64::try_from(status.uint()?).unwrap_or(i64::MAX),
                };
                // Restored reports unconverted ramrod errors as AMRError -1.
                let failed = match m.get("AMRError") {
                    None | Some(Value::Integer(0)) => false,
                    Some(Value::Integer(_) | Value::Signed(_)) => true,
                    Some(_) => return Err(Error::Invalid("AMRError is not an integer")),
                };
                if status != 0 || failed {
                    return Err(super::refuse("device final status is failure", m));
                }
                if !self.outstanding.is_empty() {
                    return Err(Error::Invalid("final status with unfinished data jobs"));
                }
                self.phase = Phase::FinalAck;
                Ok(Action::FinalAck(Value::dict([(
                    "MsgType",
                    Value::from("ReceivedFinalStatusMsg"),
                )])))
            }
            _ => Err(super::refuse("unknown required restore message", m)),
        }
    }
    /// Transfer failures are terminal even if a later StatusMsg says zero.
    pub fn finish_job(&mut self, id: u64, result: Result<()>) -> Result<()> {
        if self.phase != Phase::Restoring {
            return Err(Error::Invalid("job completion outside restoring phase"));
        }
        let request = self
            .outstanding
            .remove(&id)
            .ok_or(Error::Invalid("unknown/already completed restore job"))?;
        if let Err(e) = result {
            self.phase = Phase::Failed;
            return Err(super::refuse(refuse!("job failed: {e}", ?e), &request));
        }
        Ok(())
    }
    /// Host write completion, not a guest message. Reboot/disconnect is allowed
    /// only after the final acknowledgement has left the stream.
    pub fn final_ack_sent(&mut self) -> Result<()> {
        if self.phase != Phase::FinalAck {
            return Err(Error::Invalid("final acknowledgement phase"));
        }
        self.phase = Phase::Complete;
        Ok(())
    }
}
#[derive(Default)]
struct MemberWindow {
    bytes: Vec<u8>,
    at: u64,
    segment: Option<usize>,
}

impl MemberWindow {
    fn get(&self, segment: usize, at: u64, len: usize) -> Option<&[u8]> {
        if self.segment != Some(segment) {
            return None;
        }
        let start = usize::try_from(at.checked_sub(self.at)?).ok()?;
        self.bytes.get(start..start.checked_add(len)?)
    }

    fn fill<R: ReadAt>(
        &mut self,
        store: &mut Store<R>,
        segment: usize,
        path: &str,
        at: u64,
        len: u64,
    ) -> Result<()> {
        // Read in large verified runs and send them in FILE_CHUNK pieces.
        // A deflated member's random read can replay its inflater.
        self.bytes = store.read_at(path, at, len.min(MAX_READ_BYTES as u64) as usize)?;
        self.at = at;
        self.segment = Some(segment);
        Ok(())
    }
}

fn next_segment<'a>(
    segments: &'a [Segment],
    index: &mut usize,
    offset: &mut u64,
) -> Option<&'a Segment> {
    while let Some(segment) = segments.get(*index) {
        if *offset != segment.len() {
            return Some(segment);
        }
        *index += 1;
        *offset = 0;
    }
    None
}

/// A host-side payload snapshot. Sent counts fully drained FileData batches,
/// excludes framing overhead, and does not imply guest verification or install success.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileProgress {
    pub id: u64,
    pub name: String,
    pub member: String,
    pub sent: u64,
    pub total: u64,
    pub complete: bool,
}

/// The scheduler owns the stream and close/reconnect policy. `port=None` uses
/// the control stream; serialize its outbound frames with all other writers.
/// DataPort jobs can run concurrently with progress/control reads.
pub struct Job {
    pub id: u64,
    pub port: Option<u16>,
    pub request: Value,
    plan: Plan,
    frame: Framed,
    raw: Output,
    index: usize,
    offset: u64,
    /// Payload bytes staged as FileData, committed when the batch drains.
    file_pending: usize,
    /// The stream position at which the last staged frame can be sent.
    file_tail: u64,
    /// Verified member bytes ready to frame.
    window: MemberWindow,
    done: bool,
    failed: bool,
    reply_queued: bool,
}
impl Job {
    fn new(id: u64, port: Option<u16>, request: Value, plan: Plan) -> Self {
        Self {
            id,
            port,
            request,
            plan,
            frame: Framed::new(Endian::Big, Limits::default()),
            raw: Output::default(),
            index: 0,
            offset: 0,
            file_pending: 0,
            file_tail: 0,
            window: MemberWindow::default(),
            done: false,
            failed: false,
            reply_queued: false,
        }
    }
    pub fn poll<R: ReadAt>(
        &mut self,
        s: &mut impl ByteStream,
        store: &mut Store<R>,
    ) -> Poll<Result<()>> {
        self.poll_with_source(s, store, false)
    }

    pub(super) fn poll_with_source<R: ReadAt>(
        &mut self,
        s: &mut impl ByteStream,
        store: &mut Store<R>,
        read_ahead_only: bool,
    ) -> Poll<Result<()>> {
        // Step on while each step queues or sends: the stream takes only
        // what the guest has room for (its receive window), so the guest's
        // own buffers end the turn.
        let r = loop {
            let (before, queued) = (self.written(), self.sending());
            let r = self.step(s, store, read_ahead_only);
            let moved = self.written() != before || !queued && self.sending();
            if r.is_ready() || !moved {
                break r;
            }
        };
        // A source that ran out of the time it may wait reads on later.
        if matches!(r, Poll::Ready(Err(Error::Pending))) {
            return Poll::Pending;
        }
        if matches!(r, Poll::Ready(Err(_))) {
            self.failed = true;
        }
        r.map(|r| r.map_err(|e| super::refuse(refuse!("transfer failed: {e}", ?e), &self.request)))
    }
    /// Read the next verified member window, or up to `chunks` ASR chunks.
    pub fn prefetch<R: ReadAt>(&mut self, store: &mut Store<R>, chunks: usize) -> Result<()> {
        let limit = self.member_chunk_limit();
        match &mut self.plan {
            Plan::Asr(sender) => sender.prefetch(store, chunks)?,
            Plan::File(segments) | Plan::Raw(segments) | Plan::Message(segments)
                if chunks != 0 && !self.done && !self.failed =>
            {
                // Keep the existing read-ahead point at the last logical
                // frame, including when a next-window check can fail.
                if self.file_pending != 0 && self.frame.written() < self.file_tail {
                    return Ok(());
                }
                let mut index = self.index;
                let mut cursor = self.offset + self.file_pending as u64;
                if let Some(Segment::Member {
                    path,
                    offset,
                    bytes,
                }) = next_segment(segments, &mut index, &mut cursor)
                {
                    let at = offset + cursor;
                    let remaining = bytes - cursor;
                    let n = remaining.min(limit as u64) as usize;
                    if self.window.get(index, at, n).is_none() {
                        self.window.fill(store, index, path, at, remaining)?;
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }
    /// The job's ASR sender, if it sends the image.
    pub fn asr(&self) -> Option<&super::asr::Sender> {
        match &self.plan {
            Plan::Asr(sender) => Some(sender),
            _ => None,
        }
    }
    /// Diagnostic only: never advances an offset or reads the source.
    pub fn file_progress(&self) -> Option<FileProgress> {
        let Plan::File(segments) = &self.plan else {
            return None;
        };
        let total = segments
            .iter()
            .fold(0u64, |sum, s| sum.saturating_add(s.len()));
        let sent = segments
            .iter()
            .take(self.index)
            .fold(0u64, |sum, s| sum.saturating_add(s.len()))
            .saturating_add(self.offset)
            .min(total);
        let name = self
            .request
            .get("Arguments")
            .and_then(|v| v.get("ImageName"))
            .or_else(|| self.request.get("DataType"))
            .and_then(|v| v.string().ok())
            .unwrap_or("file");
        let member = segments
            .iter()
            .find_map(|s| match s {
                Segment::Member { path, .. } => Some(path.as_str()),
                _ => None,
            })
            .unwrap_or("generated");
        Some(FileProgress {
            id: self.id,
            name: name.into(),
            member: member.into(),
            sent,
            total,
            complete: self.done && !self.failed,
        })
    }
    fn written(&self) -> u64 {
        self.frame.written() + self.raw.written()
    }
    fn sending(&self) -> bool {
        self.frame.sending() || self.raw.is_pending()
    }
    fn member_chunk_limit(&self) -> usize {
        if matches!(&self.plan, Plan::File(_)) {
            FILE_BATCH_BYTES
        } else {
            FILE_CHUNK
        }
    }
    fn step<R: ReadAt>(
        &mut self,
        s: &mut impl ByteStream,
        store: &mut Store<R>,
        read_ahead_only: bool,
    ) -> Poll<Result<()>> {
        if self.failed {
            return Poll::Ready(Err(Error::Invalid("restore job already failed")));
        }
        if self.done {
            return Poll::Ready(Ok(()));
        }
        if self.frame.sending() {
            return match self.frame.poll_send(s) {
                Poll::Ready(Ok(())) => {
                    self.offset += core::mem::take(&mut self.file_pending) as u64;
                    Poll::Pending
                }
                r => r,
            };
        }
        if self.raw.is_pending() {
            return match self.raw.poll(s) {
                Poll::Ready(Ok(())) => Poll::Pending,
                r => r,
            };
        }
        let limit = self.member_chunk_limit();
        match &mut self.plan {
            Plan::Reply(v) => {
                if self.reply_queued {
                    self.done = true;
                    return Poll::Ready(Ok(()));
                }
                self.frame.queue(v, true)?;
                self.reply_queued = true;
            }
            Plan::Asr(sender) => {
                let r = sender.poll(s, store);
                if matches!(r, Poll::Ready(Ok(()))) {
                    self.done = true;
                }
                return r;
            }
            Plan::File(_) | Plan::Raw(_) | Plan::Message(_) => {
                let raw = !matches!(&self.plan, Plan::File(_));
                let segments = match &self.plan {
                    Plan::File(s) | Plan::Raw(s) | Plan::Message(s) => s,
                    _ => unreachable!(),
                };
                let Some(segment) = next_segment(segments, &mut self.index, &mut self.offset)
                else {
                    if !raw && !self.reply_queued {
                        self.frame
                            .queue(&Value::dict([("FileDataDone", Value::Boolean(true))]), true)?;
                        self.reply_queued = true;
                    } else {
                        self.done = true;
                        return Poll::Ready(Ok(()));
                    }
                    return Poll::Pending;
                };
                let n = (segment.len() - self.offset).min(limit as u64) as usize;
                let bytes = match segment {
                    Segment::Bytes(bytes) => &bytes[self.offset as usize..self.offset as usize + n],
                    Segment::Member {
                        path,
                        offset,
                        bytes,
                    } => {
                        let at = offset + self.offset;
                        if self.window.get(self.index, at, n).is_none() {
                            if read_ahead_only {
                                return Poll::Pending;
                            }
                            self.window
                                .fill(store, self.index, path, at, bytes - self.offset)?;
                        }
                        self.window
                            .get(self.index, at, n)
                            .ok_or(Error::Invalid("short restore member read"))?
                    }
                };
                if raw {
                    self.offset += n as u64;
                    self.raw.queue(bytes.to_vec(), FILE_CHUNK)?;
                } else {
                    let tail = self.frame.queue_file_data(bytes.chunks(FILE_CHUNK))?;
                    self.file_pending = n;
                    self.file_tail = self.frame.written() + tail as u64;
                }
            }
        }
        Poll::Pending
    }
}
#[cfg(test)]
mod tests;
