// SPDX-License-Identifier: GPL-2.0-only
//! Submitted ordinary TDs remain owned until a matching terminal completion or
//! acknowledged endpoint cancellation. Ring occupancy is an address property,
//! not the count of tickets a caller happened to poll.
use super::*;
use core::sync::atomic::AtomicU64;

static NEXT_TICKET: AtomicU64 = AtomicU64::new(1);

struct Active {
    id: u64,
    trb: u64,
    length: usize,
    done: Option<Result<usize, BlockError>>,
}

#[derive(Default)]
pub(in crate::usb::xhci) struct Queue {
    active: Vec<Active>,
    recovery: bool,
    failure: Option<BlockError>,
    clear_tt: bool,
    /// Linux `xhci_virt_ep.err_count`: transaction errors since the last
    /// success; cancellation does not clear it.
    errors: u32,
    /// The endpoint's recovery Reset Endpoint while it runs, for ordinary
    /// and stream endpoints alike.
    reset: Option<Reset>,
}

/// A Reset Endpoint that a transfer's recovery issued, stepped by the
/// owner's polls instead of waited for (`Controller::step_recovery_reset`).
#[derive(Clone, Copy)]
pub(in crate::usb::xhci) struct Reset {
    /// The started command; none until no other command is outstanding.
    pub(in crate::usb::xhci) command: Option<u64>,
    /// [`TSP`] for a soft reset.
    pub(in crate::usb::xhci) flags: u32,
    pub(in crate::usb::xhci) rings: Rings,
    pub(in crate::usb::xhci) failed: Failed,
}

/// What a recovery Reset Endpoint rings once it completes.
#[derive(Clone, Copy)]
pub(in crate::usb::xhci) enum Rings {
    /// The endpoint's ring.
    Endpoint,
    /// Each stream with a TD then active.
    Streams,
    /// Nothing: the endpoint stays stopped until it is rung again.
    None,
}

/// Where a recovery Reset Endpoint that failed leaves its error.
#[derive(Clone, Copy)]
pub(in crate::usb::xhci) enum Failed {
    /// The TD the event names stays halted on the ring, fenced as failed.
    Td(Event),
    /// The stream's active TD fails.
    Stream(u16),
    /// Only the poll that stepped it reports the error.
    Endpoint,
}

impl Queue {
    pub(in crate::usb::xhci) fn idle(&self) -> bool {
        self.active.is_empty() && self.readiness().is_ok() && !self.clear_tt && self.reset.is_none()
    }

    pub(in crate::usb::xhci) fn resetting(&self) -> bool {
        self.reset.is_some()
    }

    pub(in crate::usb::xhci) fn reset(&mut self) -> Option<&mut Reset> {
        self.reset.as_mut()
    }

    pub(in crate::usb::xhci) fn begin_reset(&mut self, reset: Reset) {
        self.reset = Some(reset);
    }

    pub(in crate::usb::xhci) fn take_reset(&mut self) -> Option<Reset> {
        self.reset.take()
    }

    pub(in crate::usb::xhci) fn readiness(&self) -> Result<(), BlockError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        if self.recovery {
            return Err(BlockError::Unsupported(
                "xHCI transfer requires cancellation",
            ));
        }
        Ok(())
    }

    pub(in crate::usb::xhci) fn reserve(
        &mut self,
        trb: u64,
        length: usize,
    ) -> Result<u64, BlockError> {
        if self.recovery {
            return Err(BlockError::Unsupported(
                "xHCI transfer requires cancellation",
            ));
        }
        if self.active.len() >= RING_TRBS - 2 || self.active.iter().any(|td| td.trb == trb) {
            return Err(BlockError::Unsupported("xHCI transfer ring full"));
        }
        self.active
            .try_reserve(1)
            .map_err(|_| BlockError::NoDmaMemory)?;
        let id = NEXT_TICKET
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .map_err(|_| BlockError::Unsupported("xHCI transfer ticket space exhausted"))?;
        self.active.push(Active {
            id,
            trb,
            length,
            done: None,
        });
        Ok(id)
    }

    pub(in crate::usb::xhci) fn identity(
        &self,
        trb: u64,
        length: usize,
    ) -> Result<Option<u64>, BlockError> {
        let Some(td) = self.active.iter().find(|td| td.trb == trb) else {
            return Ok(None);
        };
        if td.length != length {
            return Err(BlockError::Unsupported(
                "xHCI completion length differs from submitted TD",
            ));
        }
        Ok(Some(td.id))
    }

    pub(in crate::usb::xhci) fn contains(&self, id: u64, trb: u64) -> bool {
        self.active.iter().any(|td| td.id == id && td.trb == trb)
    }

    pub(in crate::usb::xhci) fn unfinished(&self, id: u64, trb: u64) -> bool {
        self.readiness().is_ok()
            && self
                .active
                .iter()
                .any(|td| td.id == id && td.trb == trb && td.done.is_none())
    }

    pub(in crate::usb::xhci) fn event(&mut self, event: Event) -> Result<(), BlockError> {
        let Some(td) = self.active.iter_mut().find(|td| td.trb == event.parameter) else {
            self.recovery = true;
            let error = BlockError::Unsupported("xHCI event does not name a submitted TD");
            self.failure = Some(error.clone());
            return Err(error);
        };
        if event.control & EVENT_DATA != 0 || event.residual() > td.length || td.done.is_some() {
            self.recovery = true;
            let error = BlockError::Unsupported("invalid xHCI transfer event");
            self.failure = Some(error.clone());
            return Err(error);
        }
        let done = match event.code() {
            SUCCESS | SHORT_PACKET => {
                self.errors = 0;
                Ok(td.length - event.residual())
            }
            status => {
                // Stopped (including length-invalid and short-packet stopped)
                // and every error keep DMA until Stop/Reset/SetDequeue ACK.
                self.recovery = true;
                self.clear_tt |= matches!(
                    status,
                    BABBLE_DETECTED | USB_TRANSACTION_ERROR | STALL | SPLIT_TRANSACTION_ERROR
                );
                Err(BlockError::Device { status })
            }
        };
        td.done = Some(done);
        Ok(())
    }

    /// Linux process_bulk_intr_td: a transaction error on an endpoint that
    /// is not behind a TT is retried in place while it has failed at most
    /// MAX_SOFT_RETRY times before. Any other event is left to [`Self::event`].
    pub(in crate::usb::xhci) fn soft_retry(
        &mut self,
        event: &Event,
        no_soft_retry: bool,
        tt_info: u32,
    ) -> bool {
        let valid = self.active.iter().any(|td| {
            td.trb == event.parameter
                && td.done.is_none()
                && event.control & EVENT_DATA == 0
                && event.residual() <= td.length
        });
        if !valid || self.recovery || event.code() != USB_TRANSACTION_ERROR {
            return false;
        }
        retry_in_place(&mut self.errors, no_soft_retry, tt_info)
    }

    pub(in crate::usb::xhci) fn cancellation_snapshot(
        &self,
        id: u64,
        trb: u64,
    ) -> Option<(usize, Option<Result<usize, BlockError>>)> {
        let td = self.active.iter().find(|td| td.id == id && td.trb == trb)?;
        Some((
            td.length,
            self.failure.clone().map(Err).or_else(|| td.done.clone()),
        ))
    }

    pub(in crate::usb::xhci) fn completed(&mut self, id: u64) -> Option<Result<usize, BlockError>> {
        if let Some(error) = &self.failure {
            return Some(Err(error.clone()));
        }
        let index = self.active.iter().position(|td| td.id == id)?;
        let done = self.active[index].done.clone()?;
        if done.is_ok() {
            self.active.remove(index);
        }
        Some(done)
    }

    /// EP0 protocol STALL does not leave a functional TT halt to clear.
    pub(in crate::usb::xhci) fn control_failure(&mut self, code: u32) {
        self.clear_tt |= matches!(
            code,
            BABBLE_DETECTED | USB_TRANSACTION_ERROR | SPLIT_TRANSACTION_ERROR
        );
    }

    pub(in crate::usb::xhci) fn needs_tt_clear(&self) -> bool {
        self.clear_tt
    }

    pub(in crate::usb::xhci) fn cancelled(&mut self) {
        self.active.clear();
        self.recovery = false;
        self.failure = None;
        self.clear_tt = false;
        self.reset = None;
    }
}
