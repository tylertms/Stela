//! Shared deadline arithmetic and capture lifecycle, without execution authority.
use super::{CLOSED, IRQ_WORK_TICK_NS, WorkError};
use core::sync::atomic::{AtomicU64, Ordering};

static NEXT_DOMAIN: AtomicU64 = AtomicU64::new(1);

pub(in crate::platform::work) struct Capture {
    domain: u64,
    serial: u64,
    epoch: u64,
    pub(in crate::platform::work) raised: bool,
    pub(in crate::platform::work) tick: bool,
    pub(in crate::platform::work) timer: bool,
}

pub(in crate::platform::work) struct DeadlineCore {
    id: u64,
    last_ns: u64,
    tick: Option<u64>,
    timer: Option<u64>,
    delivered: u64,
    serial: u64,
    running: bool,
    retired: bool,
}

impl DeadlineCore {
    pub(in crate::platform::work) fn new(now: u64) -> Result<Self, WorkError> {
        let id = NEXT_DOMAIN
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .map_err(|_| WorkError::Exhausted)?;
        Ok(Self {
            id,
            last_ns: now,
            tick: None,
            timer: None,
            delivered: 0,
            serial: 0,
            running: false,
            retired: false,
        })
    }

    pub(in crate::platform::work) fn observe(&mut self, now: u64) -> Result<(), WorkError> {
        if now < self.last_ns {
            return Err(WorkError::corrupt());
        }
        self.last_ns = now;
        Ok(())
    }

    pub(in crate::platform::work) fn can_start_ticks(&self) -> bool {
        self.tick.is_none() && !self.running
    }

    pub(in crate::platform::work) fn start_ticks(&mut self, now: u64) -> Result<(), WorkError> {
        if !self.can_start_ticks() {
            return Err(WorkError::Busy);
        }
        self.tick = Some(
            now.checked_add(IRQ_WORK_TICK_NS)
                .ok_or(WorkError::Exhausted)?,
        );
        Ok(())
    }

    pub(in crate::platform::work) fn set_timer(&mut self, deadline: Option<u64>) {
        self.timer = deadline;
    }

    pub(in crate::platform::work) fn next_deadline(&self) -> Option<u64> {
        match (self.tick, self.timer) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    pub(in crate::platform::work) fn due(&self, now: u64, epoch: u64) -> bool {
        epoch != self.delivered
            || self.tick.is_some_and(|deadline| deadline <= now)
            || self.timer.is_some_and(|deadline| deadline <= now)
    }

    pub(in crate::platform::work) fn running(&self) -> bool {
        self.running
    }

    pub(in crate::platform::work) fn begin(
        &mut self,
        now: u64,
        epoch: u64,
    ) -> Result<Option<Capture>, WorkError> {
        if self.running {
            return Err(WorkError::Busy);
        }
        let raised = epoch != self.delivered;
        let tick = self.tick.is_some_and(|deadline| deadline <= now);
        let timer = self.timer.is_some_and(|deadline| deadline <= now);
        if !(raised || tick || timer) {
            return Ok(None);
        }
        let serial = self.serial.checked_add(1).ok_or(WorkError::Exhausted)?;
        self.serial = serial;
        self.running = true;
        Ok(Some(Capture {
            domain: self.id,
            serial,
            epoch,
            raised,
            tick,
            timer,
        }))
    }

    pub(in crate::platform::work) fn validate(&self, capture: &Capture) -> Result<(), WorkError> {
        if !self.running || capture.domain != self.id || capture.serial != self.serial {
            return Err(WorkError::InvalidTicket);
        }
        Ok(())
    }

    pub(in crate::platform::work) fn finish(
        &mut self,
        capture: Capture,
        now: u64,
        next_timer: Option<u64>,
    ) -> Result<(), WorkError> {
        self.validate(&capture)?;
        let tick = if capture.tick {
            let previous = self.tick.ok_or_else(|| WorkError::corrupt())?;
            let phases = now
                .checked_sub(previous)
                .ok_or_else(|| WorkError::corrupt())?
                / IRQ_WORK_TICK_NS
                + 1;
            Some(
                previous
                    .checked_add(
                        phases
                            .checked_mul(IRQ_WORK_TICK_NS)
                            .ok_or(WorkError::Exhausted)?,
                    )
                    .ok_or(WorkError::Exhausted)?,
            )
        } else {
            self.tick
        };
        self.tick = tick;
        self.timer = next_timer;
        self.delivered = capture.epoch;
        self.running = false;
        Ok(())
    }

    pub(in crate::platform::work) fn retire(&mut self, epoch: u64) -> Result<(), WorkError> {
        if epoch & CLOSED == 0
            || epoch & !CLOSED != self.delivered
            || self.running
            || self.timer.is_some()
        {
            return Err(WorkError::Busy);
        }
        self.tick = None;
        self.timer = None;
        self.retired = true;
        Ok(())
    }

    pub(in crate::platform::work) fn retired(&self) -> bool {
        self.retired
    }
}
