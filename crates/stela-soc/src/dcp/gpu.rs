//! How far the GPU has got with the work submitted to it, which the display
//! compares with when each swap was submitted.
//!
//! The host's GPU service publishes the serial of the last command buffer
//! submitted and of the last seen complete. A swap submitted after command
//! buffer `n` was submitted may name surfaces that work draws, so a present
//! before `n` completes can show a frame still being drawn.

use core::sync::atomic::{AtomicU64, Ordering};

static SUBMITTED: AtomicU64 = AtomicU64::new(0);
static COMPLETED: AtomicU64 = AtomicU64::new(0);

/// Note the GPU's last submitted and last completed command buffer serials.
pub fn note(submitted: u64, completed: u64) {
    SUBMITTED.store(submitted, Ordering::Release);
    COMPLETED.store(completed, Ordering::Release);
}

/// The serial of the last command buffer submitted.
pub fn submitted() -> u64 {
    SUBMITTED.load(Ordering::Acquire)
}

/// Note the GPU's last completed command buffer serial, from a waiter that
/// saw the queue's timeline reach it: the display's look at a swap between
/// GPU calls then sees the completion without a call of its own.
pub fn note_completed(completed: u64) {
    COMPLETED.fetch_max(completed, Ordering::Release);
}

/// The serial of the last command buffer seen complete.
pub fn completed() -> u64 {
    COMPLETED.load(Ordering::Acquire)
}
