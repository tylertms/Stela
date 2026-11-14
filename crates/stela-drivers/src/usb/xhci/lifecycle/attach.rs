// SPDX-License-Identifier: GPL-2.0-only
//! USB2 downstream attachment waits across host service calls. Port reset
//! and enumeration still require the same stable, observed connection.
use super::*;
use crate::bus::pci::power::{Error as ClockError, Interval};

pub(super) struct Pending {
    pub(super) port: u8,
    debounce: enumerate::Debounce,
    next: Deadline,
    end: Deadline,
    interval: Box<dyn Interval>,
}

impl Pending {
    pub(super) fn start(
        c: &mut Controller,
        id: SlotId,
        port: u8,
    ) -> Result<Option<Self>, BlockError> {
        c.check_minimum()?;
        let end = c
            .clock
            .try_deadline(DEBOUNCE_TIMEOUT_US)
            .map_err(|_| BlockError::Unsupported("xHCI debounce clock unavailable"))?;
        c.observe_current_ancestors_for(id, true)?;
        let (connected, changed) = c.sample_usb2_hub_port(id, port)?;
        let mut debounce = enumerate::Debounce::default();
        debounce.observe(connected, changed);
        let interval = c.minimum.begin_interval(DEBOUNCE_STEP_US);
        let Some(interval) = c.minimum_status(interval)? else {
            return Ok(None);
        };
        Ok(Some(Self {
            port,
            debounce,
            next: c.clock.deadline(DEBOUNCE_STEP_US),
            end,
            interval,
        }))
    }

    fn restart_interval(&mut self, c: &mut Controller) -> Result<(), BlockError> {
        let interval = c.minimum.begin_interval(DEBOUNCE_STEP_US);
        let Some(interval) = c.minimum_status(interval)? else {
            let failure = c.minimum.source_result(Err(ClockError::Clock));
            return c.minimum_status(failure);
        };
        self.interval = interval;
        self.next = c.clock.deadline(DEBOUNCE_STEP_US);
        Ok(())
    }

    pub(super) fn notice(
        &mut self,
        c: &mut Controller,
        connected: bool,
        changed: bool,
    ) -> Result<(), BlockError> {
        if self.debounce.restart_on_change(connected, changed) {
            self.restart_interval(c)?;
        }
        Ok(())
    }

    fn due(&self, now: u64) -> bool {
        self.next.passed_at(now) || self.end.passed_at(now)
    }

    fn sample(&mut self, c: &mut Controller, id: SlotId) -> Result<Option<bool>, BlockError> {
        let sampled = if self.next.passed() {
            let elapsed = c.minimum.interval_elapsed(self.interval.as_ref());
            if c.minimum_status(elapsed)? {
                let (connected, changed) = c.sample_usb2_hub_port(id, self.port)?;
                if let Some(stable) = self.debounce.observe(connected, changed) {
                    return Ok(Some(stable));
                }
                true
            } else {
                false
            }
        } else {
            false
        };
        if self.end.passed() {
            return Err(BlockError::Unsupported("USB port connection never settled"));
        }
        if sampled {
            self.restart_interval(c)?;
        } else {
            self.next = c.clock.deadline(DEBOUNCE_STEP_US);
        }
        Ok(None)
    }
}

impl Host {
    /// Work due even without a controller event or the fallback hub poll.
    pub fn attachment_due(&self) -> bool {
        if !self.sleep.awake() {
            return false;
        }
        if self.settling.iter().any(storage::settle::Settling::due) {
            return true;
        }
        self.hubs.iter().filter(|hub| !hub.retired).any(|hub| {
            !hub.attachments.is_empty() && {
                let now = Clock::ticks();
                hub.attachments.iter().any(|pending| pending.due(now))
            }
        })
    }

    pub(super) fn poll_hub_attachments(
        &mut self,
        index: usize,
        dma: &mut dyn DmaAllocator,
        changes: &mut Changes,
    ) {
        let hub = &self.hubs[index];
        if hub.retired || hub.attachments.is_empty() {
            return;
        }
        let id = hub.device.id;
        let now = Clock::ticks();
        let mut at = 0;
        while let Some(pending) = self.hubs[index].attachments.get_mut(at) {
            if !pending.due(now) {
                at += 1;
                continue;
            }
            let result = pending.sample(&mut self.controller.borrow_mut(), id);
            if matches!(result, Ok(None)) {
                at += 1;
                continue;
            }
            let port = self.hubs[index].attachments.remove(at).port;
            match result {
                Ok(Some(true)) => self.connect_hub_port(index, port, true, dma, changes),
                Ok(_) => {}
                Err(error) => changes.added.failures.push((
                    format!("{}.{port} port lifecycle", self.hubs[index].device.location),
                    error,
                )),
            }
        }
    }
}
