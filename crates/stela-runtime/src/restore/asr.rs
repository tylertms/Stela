//! ASR negotiation and raw transfer (pinned idevicerestore asr.c). No length
//! prefix surrounds XML; checksum SHA-1 covers each 128-KiB payload chunk,
//! including the final short chunk. It is not an Image4 trust check.
use super::{
    Error, Result,
    plist::{self, Limits, Value},
    store::Store,
    stream::{ByteStream, Output, XmlStream},
};
use alloc::{collections::VecDeque, string::String, vec::Vec};
use core::task::Poll;
use stela_crypto::{Sha1, digest::Digest};
use stela_formats::payload::ReadAt;
pub const PORT: u16 = 12345;
pub const CHUNK_BYTES: usize = 128 * 1024;
const SHA1_BYTES: usize = 20;
const PACKET_BYTES: u64 = 1450;
const FEC_STRIDE: u64 = 40;
const PACKETS_PER_FEC: u64 = 25;
/// Payload chunks [`Sender::prefetch`] keeps read ahead.
const AHEAD_CHUNKS: usize = 16;
pub trait ImageSource {
    fn size(&self, component: &str) -> Result<u64>;
    /// Returns exactly len bytes, or fails. Must preserve source identity.
    fn read_at(&mut self, component: &str, offset: u64, len: usize) -> Result<Vec<u8>>;

    /// Read with an owned buffer available for reuse. The result follows
    /// the same verification and length rules as [`Self::read_at`].
    fn read_at_reusing(
        &mut self,
        component: &str,
        offset: u64,
        len: usize,
        mut buffer: Vec<u8>,
    ) -> Result<Vec<u8>> {
        let data = self.read_at(component, offset, len)?;
        if data.len() != len {
            return Err(Error::Invalid("short ASR payload read"));
        }
        buffer.clear();
        buffer.extend_from_slice(&data);
        Ok(buffer)
    }
}
impl<R: ReadAt> ImageSource for Store<R> {
    fn size(&self, path: &str) -> Result<u64> {
        self.member_len(path)
    }
    fn read_at(&mut self, path: &str, offset: u64, len: usize) -> Result<Vec<u8>> {
        self.read_at(path, offset, len)
    }
    fn read_at_reusing(
        &mut self,
        path: &str,
        offset: u64,
        len: usize,
        buffer: Vec<u8>,
    ) -> Result<Vec<u8>> {
        Store::read_at_reusing(self, path, offset, len, buffer)
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Initiate,
    Validation,
    Oob,
    Payload,
    Complete,
    Failed,
}
pub struct Sender {
    image: String,
    size: u64,
    phase: Phase,
    checksum: bool,
    offset: u64,
    oob_remaining: u64,
    output: Output,
    xml: XmlStream,
    /// Payload chunks read ahead, each with its image bytes and its
    /// checksum, and the image offset the next starts at. Once a caller
    /// reads ahead, [`Self::poll`] sends only chunks read ahead.
    ahead: VecDeque<(usize, Vec<u8>)>,
    /// Full-size buffers already consumed by the stream, reused by prefetch.
    recycled: Vec<Vec<u8>>,
    ahead_at: u64,
    reads_ahead: bool,
    /// Polls that ended with the guest's window open and no chunk read
    /// ahead (the host was behind), or with its window full.
    pub starved: u64,
    pub full: u64,
}
impl Sender {
    pub fn new(image: String, source: &impl ImageSource) -> Result<Self> {
        let size = source.size(&image)?;
        if size == 0 {
            return Err(Error::Invalid("empty ASR image"));
        }
        Ok(Self {
            image,
            size,
            phase: Phase::Initiate,
            checksum: false,
            offset: 0,
            oob_remaining: 0,
            output: Output::default(),
            xml: XmlStream::new(Limits::default()),
            ahead: VecDeque::new(),
            recycled: Vec::new(),
            ahead_at: 0,
            reads_ahead: false,
            starved: 0,
            full: 0,
        })
    }
    pub fn phase(&self) -> Phase {
        self.phase
    }
    /// The payload bytes sent and the image's size.
    pub fn progress(&self) -> (u64, u64) {
        (self.offset, self.size)
    }
    /// Read up to `chunks` payload chunks ahead in one read of the source,
    /// while fewer than [`AHEAD_CHUNKS`] wait: a caller that must not hold
    /// the stream's owner while it reads the source does so here, between
    /// polls.
    pub fn prefetch(&mut self, source: &mut impl ImageSource, chunks: usize) -> Result<()> {
        self.reads_ahead = true;
        let room = AHEAD_CHUNKS.saturating_sub(self.ahead.len()).min(chunks);
        if self.phase != Phase::Payload || room == 0 || self.ahead_at == self.size {
            return Ok(());
        }
        let n = (self.size - self.ahead_at).min((room * CHUNK_BYTES) as u64) as usize;
        let one_chunk = n <= CHUNK_BYTES;
        let read = if one_chunk {
            let buffer = self.chunk_buffer(n);
            source.read_at_reusing(&self.image, self.ahead_at, n, buffer)
        } else {
            source.read_at(&self.image, self.ahead_at, n)
        };
        let bytes = read.inspect_err(|_| self.recycled.clear())?;
        if bytes.len() != n {
            self.recycled.clear();
            return Err(Error::Invalid("short ASR payload read"));
        }
        if one_chunk {
            self.queue_chunk(bytes);
        } else {
            for chunk in bytes.chunks(CHUNK_BYTES) {
                let mut queued = self.chunk_buffer(chunk.len());
                queued.extend_from_slice(chunk);
                self.queue_chunk(queued);
            }
        }
        self.ahead_at += n as u64;
        Ok(())
    }
    fn chunk_buffer(&mut self, bytes: usize) -> Vec<u8> {
        self.recycled
            .pop()
            .unwrap_or_else(|| Vec::with_capacity(bytes + SHA1_BYTES))
    }
    fn queue_chunk(&mut self, mut bytes: Vec<u8>) {
        let len = bytes.len();
        if self.checksum {
            let digest = Sha1::digest(&bytes);
            bytes.extend_from_slice(&digest);
        }
        self.ahead.push_back((len, bytes));
    }
    /// The `n` payload bytes at `offset`, with their checksum if asked.
    fn payload_chunk(
        &self,
        source: &mut impl ImageSource,
        offset: u64,
        n: usize,
    ) -> Result<Vec<u8>> {
        let mut bytes = source.read_at(&self.image, offset, n)?;
        if bytes.len() != n {
            return Err(Error::Invalid("short ASR payload read"));
        }
        if self.checksum {
            let digest = Sha1::digest(&bytes);
            bytes.extend_from_slice(&digest);
        }
        Ok(bytes)
    }
    pub fn poll(
        &mut self,
        stream: &mut impl ByteStream,
        source: &mut impl ImageSource,
    ) -> Poll<Result<()>> {
        // Step on while each step queues or sends: the stream takes only
        // what the guest has room for (its receive window), so the guest's
        // own buffers end the turn.
        let r = loop {
            let (before, queued) = (self.output.written(), self.output.is_pending());
            let r = self.step(stream, source);
            let moved = self.output.written() != before || !queued && self.output.is_pending();
            if r.is_ready() || !moved {
                break r;
            }
        };
        // A source that only ran out of the time it may wait has changed
        // nothing here: the next poll takes the same step again.
        if matches!(r, Poll::Ready(Err(ref e)) if !matches!(e, Error::Pending)) {
            self.phase = Phase::Failed;
            self.recycled.clear();
        }
        if self.phase == Phase::Payload {
            if self.output.is_pending() {
                self.full += 1;
            } else if self.ahead.is_empty() && self.offset < self.size {
                self.starved += 1;
            }
        }
        r
    }
    fn step(
        &mut self,
        stream: &mut impl ByteStream,
        source: &mut impl ImageSource,
    ) -> Poll<Result<()>> {
        if self.phase == Phase::Failed {
            return Poll::Ready(Err(Error::Invalid("ASR already failed")));
        }
        if self.phase == Phase::Complete {
            return Poll::Ready(Ok(()));
        }
        if source.size(&self.image)? != self.size {
            return Poll::Ready(Err(Error::Invalid("ASR image identity/size changed")));
        }
        if self.output.is_pending() {
            return match self.output.poll(stream) {
                Poll::Ready(Ok(())) => {
                    if self.phase == Phase::Payload && self.reads_ahead {
                        let buffer = self.output.take_completed_buffer();
                        if buffer.capacity() == CHUNK_BYTES + SHA1_BYTES
                            && self.recycled.len() < AHEAD_CHUNKS
                        {
                            self.recycled.push(buffer);
                        }
                    }
                    Poll::Pending
                }
                r => r,
            };
        }
        if self.phase == Phase::Oob {
            if self.oob_remaining == 0 {
                self.phase = Phase::Validation;
                return Poll::Pending;
            }
            let n = self.oob_remaining.min(CHUNK_BYTES as u64) as usize;
            let bytes = source.read_at(&self.image, self.offset, n)?;
            if bytes.len() != n {
                return Poll::Ready(Err(Error::Invalid("short ASR OOB read")));
            }
            self.output.queue(bytes, CHUNK_BYTES)?;
            self.offset += n as u64;
            self.oob_remaining -= n as u64;
            return Poll::Pending;
        }
        if self.phase == Phase::Payload {
            if self.offset == self.size {
                self.recycled.clear();
                self.phase = Phase::Complete;
                return Poll::Ready(Ok(()));
            }
            let (n, bytes) = match self.ahead.pop_front() {
                Some(chunk) => chunk,
                None if self.reads_ahead => return Poll::Pending,
                None => {
                    let n = (self.size - self.offset).min(CHUNK_BYTES as u64) as usize;
                    (n, self.payload_chunk(source, self.offset, n)?)
                }
            };
            self.output.queue(bytes, CHUNK_BYTES + SHA1_BYTES)?;
            self.offset += n as u64;
            return Poll::Pending;
        }
        let msg = match self.xml.poll(stream) {
            Poll::Pending => return Poll::Pending,
            Poll::Ready(v) => v?,
        };
        let command = msg.required("Command")?.string()?;
        match command {
            "Initiate" => {
                self.checksum = msg
                    .get("Checksum Chunks")
                    .map(Value::boolean)
                    .transpose()?
                    .unwrap_or(false);
                let mut packet = Value::dict([
                    ("Version", Value::Integer(1)),
                    ("Stream ID", Value::Integer(1)),
                    (
                        "Payload",
                        Value::dict([
                            ("Port", Value::Integer(1)),
                            ("Size", Value::Integer(self.size)),
                        ]),
                    ),
                    ("FEC Slice Stride", Value::Integer(FEC_STRIDE)),
                    ("Packet Payload Size", Value::Integer(PACKET_BYTES)),
                    ("Packets Per FEC", Value::Integer(PACKETS_PER_FEC)),
                ]);
                if self.checksum {
                    let Value::Dict(d) = &mut packet else {
                        unreachable!()
                    };
                    d.insert(
                        "Checksum Chunk Size".into(),
                        Value::Integer(CHUNK_BYTES as u64),
                    );
                }
                self.output.queue(
                    plist::encode_xml(&packet, Limits::default())?,
                    Limits::default().bytes,
                )?;
                self.phase = Phase::Validation;
            }
            // The ramdisk's asr also names an `OOB Ranges` request form,
            // answered per range with `OOB Chunk`/`OOB Error`. The pinned
            // public host serves only one Offset/Length range, and the
            // multi-range reply layout is not recovered: refuse it whole so
            // a run records the request rather than misframing a reply.
            "OOBData" if msg.get("OOB Ranges").is_some() => {
                return Poll::Ready(Err(super::refuse(
                    "ASR multi-range OOB request (OOB Ranges) reply format is not recovered",
                    &msg,
                )));
            }
            "OOBData" if self.phase == Phase::Validation => {
                let offset = msg.required("OOB Offset")?.uint()?;
                let len = msg.required("OOB Length")?.uint()?;
                if offset.checked_add(len).is_none_or(|end| end > self.size) {
                    return Poll::Ready(Err(super::refuse(
                        "ASR OOB range overflow/past image",
                        &msg,
                    )));
                }
                self.offset = offset;
                self.oob_remaining = len;
                self.phase = Phase::Oob;
            }
            "Payload" if self.phase == Phase::Validation => {
                self.phase = Phase::Payload;
                self.offset = 0;
                self.ahead.clear();
                self.ahead_at = 0;
            }
            _ => {
                return Poll::Ready(Err(super::refuse(
                    "ASR command out of phase or unsupported",
                    &msg,
                )));
            }
        }
        Poll::Pending
    }
}
#[cfg(test)]
mod tests {
    use super::super::{store::tests::fixture, stream::tests::Memory};
    use super::*;
    fn xml(v: Value) -> Vec<u8> {
        plist::encode_xml(&v, Limits::default()).unwrap()
    }
    #[test]
    fn repeated_initiate_oob_and_checksummed_payload_over_partial_stream() {
        let data: Vec<_> = (0..CHUNK_BYTES + 17).map(|i| (i * 13) as u8).collect();
        let mut source = fixture(&data, false);
        let mut input = xml(Value::dict([("Command", Value::from("Initiate"))]));
        input.extend(xml(Value::dict([
            ("Command", Value::from("Initiate")),
            ("Checksum Chunks", Value::Boolean(true)),
        ])));
        input.extend(xml(Value::dict([
            ("Command", Value::from("OOBData")),
            ("OOB Offset", Value::Integer(7)),
            ("OOB Length", Value::Integer(9)),
        ])));
        input.extend(xml(Value::dict([("Command", Value::from("Payload"))])));
        let mut stream = Memory::new(input, 37);
        let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
        for _ in 0..100_000 {
            if let Poll::Ready(v) = sender.poll(&mut stream, &mut source) {
                v.unwrap();
                break;
            }
        }
        assert_eq!(sender.phase(), Phase::Complete);
        let output = stream.output;
        let end = output
            .windows(8)
            .enumerate()
            .filter(|(_, w)| *w == b"</plist>")
            .nth(1)
            .unwrap()
            .0
            + 9;
        assert_eq!(&output[end..end + 9], &data[7..16]);
        let p = &output[end + 9..];
        assert_eq!(&p[..CHUNK_BYTES], &data[..CHUNK_BYTES]);
        assert_eq!(
            &p[CHUNK_BYTES..CHUNK_BYTES + 20],
            Sha1::digest(&data[..CHUNK_BYTES]).as_slice()
        );
        assert_eq!(&p[CHUNK_BYTES + 20..CHUNK_BYTES + 37], &data[CHUNK_BYTES..]);
        assert_eq!(
            &p[CHUNK_BYTES + 37..],
            Sha1::digest(&data[CHUNK_BYTES..]).as_slice()
        );
    }
    /// A stream whose every read and write completes at once.
    struct Ready {
        input: Vec<u8>,
        output: Vec<u8>,
    }
    impl ByteStream for Ready {
        fn read(&mut self, b: &mut [u8]) -> Poll<Result<usize>> {
            let n = b.len().min(self.input.len());
            if n == 0 {
                return Poll::Pending;
            }
            b[..n].copy_from_slice(&self.input[..n]);
            self.input.drain(..n);
            Poll::Ready(Ok(n))
        }
        fn write(&mut self, b: &[u8]) -> Poll<Result<usize>> {
            self.output.extend_from_slice(b);
            Poll::Ready(Ok(b.len()))
        }
    }
    /// Over a stream that takes every byte, the payload's chunks leave in
    /// one poll rather than one a poll.
    #[test]
    fn a_ready_stream_takes_the_payload_in_one_poll() {
        let data = vec![9; 3 * CHUNK_BYTES];
        let mut source = fixture(&data, false);
        let mut input = xml(Value::dict([("Command", Value::from("Initiate"))]));
        input.extend(xml(Value::dict([("Command", Value::from("Payload"))])));
        let mut stream = Ready {
            input,
            output: Vec::new(),
        };
        let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
        // The commands arrive a byte a poll, as nothing reads ahead.
        for _ in 0..100_000 {
            if sender.phase() == Phase::Payload {
                break;
            }
            assert!(sender.poll(&mut stream, &mut source).is_pending());
        }
        assert_eq!(sender.phase(), Phase::Payload);
        assert!(sender.poll(&mut stream, &mut source).is_ready());
        assert_eq!(sender.phase(), Phase::Complete);
        assert!(stream.output.ends_with(&data));
    }
    /// A sender that reads ahead sends only chunks read ahead, so a poll
    /// never reads the source while it holds the stream.
    #[test]
    fn a_sender_that_reads_ahead_sends_only_chunks_read_ahead() {
        let data: Vec<_> = (0..3 * CHUNK_BYTES).map(|i| (i * 7) as u8).collect();
        let mut source = fixture(&data, false);
        let mut input = xml(Value::dict([("Command", Value::from("Initiate"))]));
        input.extend(xml(Value::dict([("Command", Value::from("Payload"))])));
        let mut stream = Ready {
            input,
            output: Vec::new(),
        };
        let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
        sender.prefetch(&mut source, 1).unwrap();
        while sender.phase() != Phase::Payload {
            assert!(sender.poll(&mut stream, &mut source).is_pending());
        }
        assert!(sender.poll(&mut stream, &mut source).is_pending());
        assert_eq!(sender.progress().0, 0);
        assert!(sender.starved > 0, "nothing was read ahead yet");
        sender.prefetch(&mut source, 2).unwrap();
        let last_sent = sender.ahead[1].1.as_ptr();
        assert!(sender.poll(&mut stream, &mut source).is_pending());
        assert_eq!(sender.progress().0, 2 * CHUNK_BYTES as u64);
        assert_eq!(sender.recycled.last().unwrap().as_ptr(), last_sent);
        sender.prefetch(&mut source, 2).unwrap();
        assert_eq!(sender.ahead[0].1.as_ptr(), last_sent);
        assert!(sender.poll(&mut stream, &mut source).is_ready());
        assert_eq!(sender.phase(), Phase::Complete);
        assert!(sender.recycled.is_empty());
        assert!(stream.output.ends_with(&data));
    }
    /// A source whose every other read runs out of the time it may wait, as
    /// a network source's does.
    struct Late<S>(S, bool);
    impl<S: ImageSource> ImageSource for Late<S> {
        fn size(&self, component: &str) -> Result<u64> {
            self.0.size(component)
        }
        fn read_at(&mut self, component: &str, offset: u64, len: usize) -> Result<Vec<u8>> {
            self.1 = !self.1;
            if self.1 {
                return Err(Error::Pending);
            }
            self.0.read_at(component, offset, len)
        }
    }
    #[test]
    fn a_source_that_runs_out_of_wait_time_reads_on_at_the_next_poll() {
        let data: Vec<_> = (0..2 * CHUNK_BYTES + 5).map(|i| (i * 7) as u8).collect();
        let mut source = Late(fixture(&data, false), false);
        let mut input = xml(Value::dict([("Command", Value::from("Initiate"))]));
        input.extend(xml(Value::dict([
            ("Command", Value::from("OOBData")),
            ("OOB Offset", Value::Integer(3)),
            ("OOB Length", Value::Integer(CHUNK_BYTES as u64 + 9)),
        ])));
        input.extend(xml(Value::dict([("Command", Value::from("Payload"))])));
        let mut stream = Memory::new(input, 30000);
        let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
        let mut waits = 0;
        let mut done = false;
        for _ in 0..100_000 {
            match sender.poll(&mut stream, &mut source) {
                Poll::Pending => {}
                Poll::Ready(Err(Error::Pending)) => {
                    assert_ne!(sender.phase(), Phase::Failed);
                    waits += 1;
                }
                Poll::Ready(r) => {
                    r.unwrap();
                    done = true;
                    break;
                }
            }
        }
        assert!(done && waits >= 4);
        assert_eq!(sender.phase(), Phase::Complete);
        // The OOB range and the payload each went out once, in order.
        let mut expected = data[3..3 + CHUNK_BYTES + 9].to_vec();
        expected.extend_from_slice(&data);
        assert!(stream.output.ends_with(&expected));
    }
    #[test]
    fn large_oob_is_chunked_and_overflow_refuses_with_original_request() {
        let data = vec![0x53; super::super::store::MAX_READ_BYTES + 3];
        for overflow in [false, true] {
            let mut source = fixture(&data, false);
            let mut input = xml(Value::dict([("Command", Value::from("Initiate"))]));
            input.extend(xml(Value::dict([
                ("Command", Value::from("OOBData")),
                (
                    "OOB Offset",
                    Value::Integer(if overflow { u64::MAX } else { 0 }),
                ),
                ("OOB Length", Value::Integer(data.len() as u64)),
            ])));
            if !overflow {
                input.extend(xml(Value::dict([("Command", Value::from("Payload"))])));
            }
            let mut stream = Memory::new(input, 30000);
            let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
            let mut result = None;
            for _ in 0..100_000 {
                if let Poll::Ready(r) = sender.poll(&mut stream, &mut source) {
                    result = Some(r);
                    break;
                }
            }
            if overflow {
                let Error::Refused { request, .. } = result.unwrap().unwrap_err() else {
                    panic!("missing OOB context")
                };
                assert_eq!(
                    request.required("OOB Offset").unwrap().uint().unwrap(),
                    u64::MAX
                );
            } else {
                result.unwrap().unwrap();
                let at = stream
                    .output
                    .windows(8)
                    .position(|w| w == b"</plist>")
                    .unwrap()
                    + 9;
                assert_eq!(&stream.output[at..at + data.len()], data);
                assert_eq!(&stream.output[at + data.len()..], data);
            }
        }
    }
    #[test]
    fn multi_range_oob_refuses_with_the_whole_request() {
        let mut source = fixture(b"hello world", false);
        let mut input = xml(Value::dict([("Command", Value::from("Initiate"))]));
        input.extend(xml(Value::dict([
            ("Command", Value::from("OOBData")),
            (
                "OOB Ranges",
                Value::Array(alloc::vec![Value::dict([
                    ("OOB Offset", Value::Integer(0)),
                    ("OOB Length", Value::Integer(5)),
                ])]),
            ),
        ])));
        let mut stream = Memory::new(input, 13);
        let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
        let mut result = None;
        for _ in 0..100_000 {
            if let Poll::Ready(r) = sender.poll(&mut stream, &mut source) {
                result = Some(r);
                break;
            }
        }
        let Error::Refused { reason, request } = result.unwrap().unwrap_err() else {
            panic!("missing OOB Ranges context")
        };
        assert!(reason.to_string().contains("OOB Ranges"));
        assert!(request.get("OOB Ranges").is_some());
        assert_eq!(sender.phase(), Phase::Failed);
        // Nothing but the Initiate reply was sent.
        let end = stream
            .output
            .windows(8)
            .position(|w| w == b"</plist>")
            .unwrap()
            + 9;
        assert!(stream.output[end..].iter().all(u8::is_ascii_whitespace));
    }

    /// Remote range reads may yield while their network work continues.
    /// Exercise both OOB validation and checksummed payload across yields.
    #[test]
    fn deferred_image_reads_resume_without_poisoning_asr_or_repeating_bytes() {
        struct Deferred {
            bytes: Vec<u8>,
            wait: bool,
            pending: usize,
        }
        impl ImageSource for Deferred {
            fn size(&self, _: &str) -> Result<u64> {
                Ok(self.bytes.len() as u64)
            }
            fn read_at(&mut self, _: &str, offset: u64, len: usize) -> Result<Vec<u8>> {
                self.wait = !self.wait;
                if self.wait {
                    self.pending += 1;
                    return Err(Error::Pending);
                }
                Ok(self.bytes[offset as usize..offset as usize + len].to_vec())
            }
        }
        let data: Vec<_> = (0..CHUNK_BYTES + 17).map(|i| (i * 13) as u8).collect();
        let mut source = Deferred {
            bytes: data.clone(),
            wait: false,
            pending: 0,
        };
        let mut input = xml(Value::dict([
            ("Command", Value::from("Initiate")),
            ("Checksum Chunks", Value::Boolean(true)),
        ]));
        input.extend(xml(Value::dict([
            ("Command", Value::from("OOBData")),
            ("OOB Offset", Value::Integer(0)),
            ("OOB Length", Value::Integer(data.len() as u64)),
        ])));
        input.extend(xml(Value::dict([("Command", Value::from("Payload"))])));
        let mut stream = Memory::new(input, 30000);
        let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
        for _ in 0..100_000 {
            if let Poll::Ready(result) = sender.poll(&mut stream, &mut source) {
                result.unwrap();
                break;
            }
            assert_ne!(sender.phase(), Phase::Failed);
        }
        assert_eq!(sender.phase(), Phase::Complete);
        assert_eq!(source.pending, 4);
        let start = stream
            .output
            .windows(8)
            .position(|w| w == b"</plist>")
            .unwrap()
            + 9;
        let mut expected = data.clone(); // OOB bytes, without checksums.
        for chunk in data.chunks(CHUNK_BYTES) {
            expected.extend_from_slice(chunk);
            expected.extend_from_slice(&Sha1::digest(chunk));
        }
        assert_eq!(&stream.output[start..], expected);
    }

    #[test]
    fn malformed_ranges_and_commands_are_terminal() {
        for v in [
            Value::dict([("Command", Value::from("Payload"))]),
            Value::dict([
                ("Command", Value::from("Initiate")),
                ("Checksum Chunks", Value::Integer(1)),
            ]),
        ] {
            let mut source = fixture(b"hello", false);
            let mut stream = Memory::new(xml(v), 1);
            let mut sender = Sender::new("os.dmg".into(), &source).unwrap();
            for _ in 0..2000 {
                if let Poll::Ready(v) = sender.poll(&mut stream, &mut source) {
                    assert!(v.is_err());
                    break;
                }
            }
            assert_eq!(sender.phase(), Phase::Failed);
            assert!(matches!(
                sender.poll(&mut stream, &mut source),
                Poll::Ready(Err(_))
            ));
        }
    }
}
