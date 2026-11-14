//! Interrupts, doorbells and TLB shootdowns between host CPUs, and timed
//! sleeps.

use super::*;

pub fn timer_interrupt() {
    crate::native_clock::interrupt();
    local::interrupt();
    state().apic.eoi();
}

pub(crate) fn root_irq_active() -> bool {
    local::irq_active()
}

/// An idle-worker doorbell only. Stop requests live in generation-bound
/// mailboxes; delivery cannot authorize native-frame recovery or settlement.
/// Whether an interrupt came for this core since [`clear_interrupted`].
pub(crate) fn interrupted() -> bool {
    local::interrupted()
}

pub(crate) fn clear_interrupted() {
    local::clear_interrupt();
}

pub(crate) fn wake_interrupt() {
    state().apic.eoi();
    DOORBELLS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    // Release publication follows EOI; startup evidence cannot race
    // ahead of retirement of the directed interrupt at this physical LAPIC.
    local::doorbell();
}

/// A host device's completion signal: a doorbell, counted apart too.
pub(crate) fn signal_interrupt() {
    SIGNALS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    wake_interrupt();
}

/// The doorbells any core has taken.
static DOORBELLS: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);
/// The completion signals host devices sent.
static SIGNALS: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// The completion signals host devices sent ([`completion_signal`]), which
/// unlike [`doorbells`] no other core's doorbell moves.
pub(crate) fn device_signals() -> u64 {
    SIGNALS.load(core::sync::atomic::Ordering::Relaxed)
}

/// The doorbells any core has taken, a host disk's completion signals among
/// them: until this moves, a disk that signals has nothing new to look at.
pub(crate) fn doorbells() -> u64 {
    DOORBELLS.load(core::sync::atomic::Ordering::Relaxed)
}

/// The message a host disk sends as it completes a command: the signal
/// vector, a doorbell counted apart, fixed and edge-triggered, to the
/// processor with APIC ID `id`,
/// which runs the engine's devices and looks for the disk's completions.
/// `None` where the ID needs more than a compatibility-format MSI address
/// holds.
/// Whether this host processor is inside an interrupt or exception handler.
pub(crate) fn in_interrupt() -> bool {
    local::irq_active()
}

/// Sleep until an interrupt, at most `micros`: a completion signal, the
/// engine's own timer or this one-shot. The engine's armed one-shot count
/// is put back less the time slept; a periodic or TSC-deadline timer, or a
/// timer not yet measured, leaves the caller to poll.
pub(crate) fn sleep_for(micros: u64) {
    if micros == 0 || crate::native_clock::host_sleep(micros) {
        return;
    }
    const MASKED: u32 = 1 << 16;
    const MODE: u32 = 3 << 17;
    let state = state();
    let lvt = state.apic.read(apic::LVT_TIMER);
    if state.apic_hz == 0 || lvt & MODE != 0 {
        return;
    }
    let remaining = state.apic.read(apic::CURRENT);
    let armed = lvt & MASKED == 0 && remaining != 0;
    let ticks = (u128::from(micros) * u128::from(state.apic_hz) / 1_000_000)
        .clamp(1, u128::from(u32::MAX)) as u32;
    let ticks = if armed { ticks.min(remaining) } else { ticks };
    state.apic.write(apic::LVT_TIMER, u32::from(TIMER_VECTOR));
    state.apic.write(apic::INITIAL, ticks);
    // SAFETY: interrupts are disabled and the host IDT takes the timer.
    unsafe { crate::native_idle::wait(|| Some(u64::from(ticks) * 1_000_000 / state.apic_hz)) };
    let slept = ticks - state.apic.read(apic::CURRENT);
    state.apic.write(apic::LVT_TIMER, lvt);
    // Ours again only if it was ours before: the hand-back count read now
    // is the one the host's own write would have seen.
    state.timer_lvt = state
        .timer_lvt
        .filter(|_| lvt == u32::from(TIMER_VECTOR))
        .filter(|&seen| {
            seen == crate::native_clock::TIMER_HANDOVERS.load(core::sync::atomic::Ordering::Acquire)
        });
    // A count that ran out during the sleep fires at once.
    state.apic.write(
        apic::INITIAL,
        if armed { (remaining - slept).max(1) } else { 0 },
    );
}

pub(crate) fn completion_signal(id: u64) -> Option<stela_drivers::bus::irq::MsiMessage> {
    (id < 255).then(|| stela_drivers::bus::irq::MsiMessage {
        address: 0xfee0_0000 | id << 12,
        data: u32::from(super::super::interrupts::SIGNAL_VECTOR),
    })
}

/// Doorbells a local APIC did not deliver since the last report: each may
/// leave its processor asleep until its next wake.
pub(super) static UNDELIVERED: core::sync::atomic::AtomicU64 =
    core::sync::atomic::AtomicU64::new(0);

/// Ring the doorbell of the processor whose APIC ID is `destination`,
/// through this processor's own local APIC.
pub(crate) fn ring(destination: u32) {
    let state = state();
    // SAFETY: interrupts are off in the engine, so nothing else on this
    // processor writes its ICR meanwhile; every installed host handles the
    // doorbell vector.
    let sent = unsafe {
        state.apic.notify(
            destination,
            super::super::interrupts::NATIVE_WAKE_VECTOR,
            state.tsc_hz,
        )
    };
    if sent.is_err() {
        UNDELIVERED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

/// Send an NMI to the processor whose APIC ID is `destination`, through
/// this processor's own local APIC: the step watchdog's, to the boot
/// processor stuck in a Linux step (`step_watchdog::look`).
pub(crate) fn nmi(destination: u32) {
    let state = state();
    // SAFETY: interrupts are off in the engine, so nothing else on this
    // processor writes its ICR meanwhile; every installed host's NMI gate
    // stops the boot with the step watchdog's reason.
    if unsafe { state.apic.notify_nmi(destination, state.tsc_hz) }.is_err() {
        UNDELIVERED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}

/// The doorbells left undelivered since the last call.
pub(crate) fn undelivered_doorbells() -> u64 {
    UNDELIVERED.swap(0, core::sync::atomic::Ordering::Relaxed)
}

/// Join this processor to the aperture shootdown at `slot`.
pub(crate) fn join_shootdown(slot: usize) {
    local::shootdown()
        .0
        .store(slot + 1, core::sync::atomic::Ordering::Relaxed);
}

/// This processor's aperture shootdown slot and the generation it flushed.
pub(crate) fn shootdown() -> (Option<usize>, &'static core::sync::atomic::AtomicU64) {
    let (slot, flushed) = local::shootdown();
    (
        slot.load(core::sync::atomic::Ordering::Relaxed)
            .checked_sub(1),
        flushed,
    )
}

/// The aperture shootdown's interrupt: flush, acknowledge, EOI.
pub(crate) fn shootdown_interrupt() {
    crate::native_mmio::shootdown::service();
    state().apic.eoi();
}

pub(crate) fn doorbell_counter() -> &'static core::sync::atomic::AtomicU64 {
    local::doorbell_counter()
}

pub fn stray_interrupt(_vector: u8) {
    let state = state();
    state.stray += 1;
    state.apic.eoi();
}

/// A true APIC spurious event sets no ISR bit. A delayed ordinary fixed
/// interrupt may use the newly reserved spurious vector and still needs EOI.
/// Common entry keeps IF off, the original per-CPU APIC and this IDT/GS record
/// live; no callback/source allocation or attribution is involved.
pub fn spurious_interrupt() {
    let state = state();
    let isr = state.apic.read(0x170);
    if isr == u32::MAX {
        crate::output::fatal("native spurious-vector ISR is unreadable");
    }
    if isr & (1 << 31) != 0 {
        state.apic.eoi();
    }
}
