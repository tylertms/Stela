//! Nonblocking owner of one restored session. Borrow a port provider only for
//! a poll; the engine can run the guest between polls without a device borrow
//! or a host pointer in guest RAM. Protocol evidence is in `session`/`stream`.
use super::{
    Error, Result,
    answers::Answers,
    plist::Limits,
    plist::Value,
    route::Route,
    session::{Action, FileProgress, Job, PORT, Phase, Session},
    stream::{ByteStream, Endian, Framed},
};
use alloc::{boxed::Box, vec::Vec};
use core::task::Poll;
use stela_contracts::refuse;
use stela_formats::payload::ReadAt;

/// Handles identify owned streams, including ones still connecting. A Pending
/// open must not allocate a stream. A successful close releases the handle.
/// Zero-byte reads mean EOF, not an empty receive window.
pub trait Ports {
    type Handle: Copy + Eq;
    fn open(&mut self, port: u16) -> Poll<Result<Self::Handle>>;
    fn read(&mut self, handle: Self::Handle, bytes: &mut [u8]) -> Poll<Result<usize>>;
    fn write(&mut self, handle: Self::Handle, bytes: &[u8]) -> Poll<Result<usize>>;
    fn close(&mut self, handle: Self::Handle) -> Poll<Result<()>>;
}
struct Stream<'a, P: Ports> {
    ports: &'a mut P,
    handle: P::Handle,
}
impl<P: Ports> ByteStream for Stream<'_, P> {
    fn read(&mut self, b: &mut [u8]) -> Poll<Result<usize>> {
        self.ports.read(self.handle, b)
    }
    fn write(&mut self, b: &[u8]) -> Poll<Result<usize>> {
        self.ports.write(self.handle, b)
    }
}
struct Transfer<H> {
    job: Box<Job>,
    handle: Option<H>,
    closing: bool,
}
/// Leave capacity for the control stream and an independently owned proxy.
const DATA_STREAMS: usize = 6;
#[derive(Clone, Copy)]
enum Handshake {
    Type,
    Hardware,
    Started,
}
pub struct Driver<R, H> {
    pub answers: Answers<R>,
    pub session: Session,
    route: Route,
    control: Option<H>,
    frame: Framed,
    handshake: Handshake,
    jobs: Vec<Transfer<H>>,
    /// A control message received and not yet answered ([`Self::work`]).
    received: Option<Value>,
    /// The handshake reached StartRestore and the store is not prepared
    /// yet: [`Self::work`] prepares it and sends the message, without
    /// answering the HardwareInfo reply a second time.
    starting: bool,
    failure: Option<Error>,
    complete: bool,
    last_file: Option<FileProgress>,
    /// The job [`Self::prefetch`] reads for first: the one a call left
    /// unread when its time was over.
    prefetch_next: usize,
}
impl<R: ReadAt, H: Copy + Eq> Driver<R, H> {
    /// The complete inventory is verified (`Store::prepare`) just before
    /// StartRestore is sent, so a session that stops earlier never pays for
    /// it. A caller that runs a guest meanwhile excludes the poll's host
    /// time from the guest's clock.
    pub fn new(answers: Answers<R>, route: Route) -> Result<Self> {
        Ok(Self {
            answers,
            session: Session::new(),
            route,
            control: None,
            frame: Framed::new(Endian::Big, Limits::default()),
            handshake: Handshake::Type,
            jobs: Vec::new(),
            received: None,
            starting: false,
            failure: None,
            complete: false,
            last_file: None,
            prefetch_next: 0,
        })
    }
    /// One bounded framing operation and at most one operation per scheduled
    /// job, answering a control message as it arrives. There is no polling
    /// loop waiting for guest execution or mux I/O. Failures are terminal and
    /// preserve the session ledger and full request.
    pub fn poll(&mut self, ports: &mut impl Ports<Handle = H>) -> Poll<Result<()>> {
        let result = self.poll_ports(ports);
        self.work();
        result
    }

    /// [`Self::poll`] without answering: a control message received waits
    /// for [`Self::work`], for a caller that must not hold the ports while
    /// the answers sign with Apple's TSS or read the image.
    pub fn poll_ports(&mut self, ports: &mut impl Ports<Handle = H>) -> Poll<Result<()>> {
        self.poll_inner(ports, false)
    }

    /// Poll without refilling a member transfer's window. The caller reads
    /// those windows with [`Self::prefetch`] between polls, outside its
    /// borrow of the ports.
    pub fn poll_prefetched(&mut self, ports: &mut impl Ports<Handle = H>) -> Poll<Result<()>> {
        self.poll_inner(ports, true)
    }

    fn poll_inner(
        &mut self,
        ports: &mut impl Ports<Handle = H>,
        read_ahead_only: bool,
    ) -> Poll<Result<()>> {
        if let Some(error) = &self.failure {
            return Poll::Ready(Err(error.clone()));
        }
        if self.complete {
            return Poll::Ready(Ok(()));
        }
        let result = self.step(ports, read_ahead_only);
        if let Poll::Ready(Err(error)) = &result {
            self.failure = Some(error.clone());
        }
        result
    }

    /// Answer the control message the last poll received, queueing the
    /// reply for the next one. A failure fails the session at its next poll;
    /// an answer waiting for a signing exchange keeps the message for the
    /// next call, which continues the exchange.
    pub fn work(&mut self) {
        if self.failure.is_some() || self.complete {
            return;
        }
        if self.starting {
            // A read that is not answered yet keeps the start for the next
            // call; no message is taken before StartRestore is queued.
            match self.start() {
                Ok(()) => self.starting = false,
                Err(Error::Pending) => {}
                Err(error) => self.failure = Some(error),
            }
            return;
        }
        if let Some(message) = self.received.take() {
            match self.answer(&message) {
                Ok(()) => {}
                Err(Error::Pending) => self.received = Some(message),
                Err(error) => self.failure = Some(error),
            }
        }
    }

    /// Prepare the store and queue StartRestore. `Error::Pending` leaves
    /// nothing queued and may be repeated.
    fn start(&mut self) -> Result<()> {
        self.answers.store.prepare()?;
        let message = self.session.start_for(&self.answers, self.route)?;
        self.frame.queue(&message, false)?;
        Ok(())
    }

    fn answer(&mut self, message: &Value) -> Result<()> {
        match self.session.receive(message.clone(), &mut self.answers)? {
            Action::None | Action::Event(_) => {}
            Action::Transfer(job) => self.jobs.push(Transfer {
                job,
                handle: None,
                closing: false,
            }),
            Action::FinalAck(value) => self.frame.queue(&value, false)?,
        }
        if self.session.phase() == Phase::Ready {
            let message = match self.handshake {
                Handshake::Type => {
                    self.handshake = Handshake::Hardware;
                    self.session.query_value("HardwareInfo")?
                }
                Handshake::Hardware => {
                    self.handshake = Handshake::Started;
                    self.starting = true;
                    return match self.start() {
                        Ok(()) => {
                            self.starting = false;
                            Ok(())
                        }
                        // The reply is answered; `work` continues the start.
                        Err(Error::Pending) => Ok(()),
                        Err(error) => Err(error),
                    };
                }
                Handshake::Started => {
                    return Err(Error::Invalid("unexpected ready session"));
                }
            };
            self.frame.queue(&message, false)?;
        }
        Ok(())
    }
    fn step(
        &mut self,
        ports: &mut impl Ports<Handle = H>,
        read_ahead_only: bool,
    ) -> Poll<Result<()>> {
        if self.control.is_none() {
            self.control = Some(match ports.open(PORT) {
                Poll::Pending => return Poll::Pending,
                Poll::Ready(v) => v?,
            });
            self.frame.queue(&self.session.query_type()?, false)?;
            return Poll::Pending;
        }
        let control = self.control.unwrap();
        if self.session.phase() == Phase::Complete {
            match ports.close(control) {
                Poll::Pending => return Poll::Pending,
                Poll::Ready(v) => v?,
            };
            self.control = None;
            self.complete = true;
            return Poll::Ready(Ok(()));
        }
        if self.frame.sending() {
            match self.frame.poll_send(&mut Stream {
                ports,
                handle: control,
            }) {
                Poll::Ready(Ok(())) => match self.session.phase() {
                    Phase::Starting => self.session.started()?,
                    Phase::FinalAck => self.session.final_ack_sent()?,
                    _ => {}
                },
                Poll::Ready(Err(e)) => return Poll::Ready(Err(e)),
                Poll::Pending => {}
            }
        }
        // Read control even while an ASR/DataPort write is backpressured;
        // one message at a time, answered by `work`.
        if !self.frame.sending()
            && self.received.is_none()
            && self.session.phase() != Phase::Complete
            && let Poll::Ready(message) = self.frame.poll_receive(&mut Stream {
                ports,
                handle: control,
            })
        {
            self.received = Some(message?);
        }
        let mut active = self.jobs.iter().filter(|j| j.handle.is_some()).count();
        let mut control_used = self.frame.sending();
        let mut at = 0;
        while at < self.jobs.len() {
            let t = &mut self.jobs[at];
            let handle = if let Some(port) = t.job.port {
                if t.handle.is_none() {
                    if active == DATA_STREAMS {
                        at += 1;
                        continue;
                    }
                    match ports.open(port) {
                        Poll::Pending => {
                            at += 1;
                            continue;
                        }
                        Poll::Ready(Ok(h)) => {
                            t.handle = Some(h);
                            active += 1;
                        }
                        Poll::Ready(Err(e)) => {
                            self.session.finish_job(t.job.id, Err(e))?;
                            unreachable!()
                        }
                    }
                }
                t.handle.unwrap()
            } else {
                // Keep frames of concurrent control jobs from interleaving.
                if control_used {
                    at += 1;
                    continue;
                }
                control_used = true;
                control
            };
            if !t.closing {
                match t.job.poll_with_source(
                    &mut Stream { ports, handle },
                    &mut self.answers.store,
                    read_ahead_only,
                ) {
                    Poll::Pending => {
                        at += 1;
                        continue;
                    }
                    Poll::Ready(Err(e)) => {
                        self.session.finish_job(t.job.id, Err(e))?;
                        unreachable!()
                    }
                    Poll::Ready(Ok(())) => t.closing = true,
                }
            }
            if t.handle.is_some() {
                match ports.close(handle) {
                    Poll::Pending => {
                        at += 1;
                        continue;
                    }
                    Poll::Ready(Err(e)) => {
                        self.session.finish_job(t.job.id, Err(e))?;
                        unreachable!()
                    }
                    Poll::Ready(Ok(())) => {
                        active -= 1;
                    }
                }
            }
            if let Some(progress) = t.job.file_progress() {
                self.last_file = Some(progress);
            }
            self.session.finish_job(t.job.id, Ok(()))?;
            self.jobs.remove(at);
            if self.jobs.is_empty() {
                self.answers.store.release_cursors();
            }
        }
        Poll::Pending
    }
    /// Read ahead, up to `chunks` reads a job, what the jobs send next: a
    /// caller that must not hold the ports while it reads the source calls
    /// this between polls, and the polls then send only what it read. A
    /// failed read fails the session at its next poll.
    ///
    /// A caller whose processor waits meanwhile bounds the call: once
    /// `over` says its time has passed, the jobs not yet read for wait for
    /// the next call, which reads for them first. One job is always read
    /// for, so each call moves the restore on.
    pub fn prefetch(&mut self, chunks: usize, mut over: impl FnMut() -> bool) {
        if self.failure.is_some() || self.complete {
            return;
        }
        let count = self.jobs.len();
        let first = self.prefetch_next % count.max(1);
        for step in 0..count {
            let at = (first + step) % count;
            if step > 0 && over() {
                self.prefetch_next = at;
                return;
            }
            let transfer = &mut self.jobs[at];
            // A waiting DataPort job has no receiver yet. Keep the ASR
            // sender's existing prefetch activation before its handshake.
            if transfer.closing
                || (transfer.job.port.is_some()
                    && transfer.handle.is_none()
                    && transfer.job.asr().is_none())
            {
                continue;
            }
            match transfer.job.prefetch(&mut self.answers.store, chunks) {
                Ok(()) => {}
                // The source ran out of the time it may wait: the next
                // prefetch reads on from what arrived meanwhile.
                Err(Error::Pending) => return,
                Err(e) => {
                    self.failure = Some(super::refuse(
                        refuse!("transfer failed: {e}", ?e),
                        &transfer.job.request,
                    ));
                    return;
                }
            }
        }
    }
    /// At most MAX_JOBS active files plus the most recently completed file.
    pub fn file_progress(&self) -> impl Iterator<Item = FileProgress> + '_ {
        self.jobs
            .iter()
            .filter_map(|t| t.job.file_progress())
            .chain(self.last_file.iter().cloned())
    }
    /// The ASR sender of the job that sends the image, while one does.
    pub fn asr(&self) -> Option<&super::asr::Sender> {
        self.jobs.iter().find_map(|transfer| transfer.job.asr())
    }
    /// Close all owned streams after a terminal refusal, without emitting any
    /// further protocol bytes. Caller keeps polling while the guest advances.
    pub fn cancel(&mut self, ports: &mut impl Ports<Handle = H>) -> Poll<Result<()>> {
        // Cancellation is terminal, even while close is backpressured.
        self.failure
            .get_or_insert(Error::Invalid("restore session cancelled"));
        for t in &mut self.jobs {
            if let Some(h) = t.handle {
                match ports.close(h) {
                    Poll::Pending => return Poll::Pending,
                    Poll::Ready(v) => v?,
                };
                t.handle = None;
            }
        }
        if let Some(h) = self.control {
            match ports.close(h) {
                Poll::Pending => return Poll::Pending,
                Poll::Ready(v) => v?,
            };
            self.control = None;
        }
        Poll::Ready(Ok(()))
    }
    pub fn failed(&self) -> Option<&Error> {
        self.failure.as_ref()
    }
}

#[cfg(test)]
mod tests;
