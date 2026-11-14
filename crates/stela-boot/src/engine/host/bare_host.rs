//! The engine's Host on bare metal: guest RAM, native code, shadow tables, time
//! and waiting.

use super::*;
use stela_contracts::refuse;

/// The engine's handle; mutable native state belongs to this host processor.
pub struct BareHost<'a> {
    pub(super) _owner: core::marker::PhantomData<&'a mut HostOwner>,
    pub(super) _pinned: core::marker::PhantomData<*mut ()>,
}

impl Host for BareHost<'_> {
    #[cfg(feature = "native-profile")]
    fn native_timing_work_start(
        &mut self,
        work: stela_engine::machine::engine::NativeTimingWork,
    ) -> Option<u64> {
        state().native_profile.work_start(work, profile_timestamp)
    }
    #[cfg(feature = "native-profile")]
    fn native_timing_work_end(
        &mut self,
        work: stela_engine::machine::engine::NativeTimingWork,
        started: u64,
    ) {
        state()
            .native_profile
            .work_end(work, started, profile_timestamp());
    }
    #[cfg(feature = "native-profile")]
    fn native_timing(&mut self, phase: stela_engine::machine::engine::NativeTimingPhase) {
        let profile = &mut state().native_profile;
        if phase == stela_engine::machine::engine::NativeTimingPhase::Begin
            && profile.needs_calibration()
        {
            profile.calibrate(profile_timestamp);
        }
        profile.event(phase, profile_timestamp);
    }
    #[cfg(feature = "native-profile")]
    fn native_instruction_sample(
        &mut self,
        sample: stela_engine::machine::engine::NativeInstructionSample<'_>,
    ) {
        state().native_profile.instructions(sample);
    }
    fn native_worker(&self) -> Option<stela_engine::machine::engine::publication::WorkerId> {
        local::native_worker()
    }

    fn native_cpu_lease(&self) -> Option<stela_engine::NativeCpuLease> {
        local::native_cpu_lease()
    }

    fn read_ram(&mut self, physical: u64, buffer: &mut [u8]) -> bool {
        // The engine reads guest RAM only once its translation allowed it.
        super::super::speculation::fence();
        if let Some(ram) = &state().registered_ram {
            // SAFETY: the runtime passes private staging while the adopted
            // canonical reservation/mapping and source controls remain live.
            return unsafe { ram.read_physical(physical, buffer) }.is_ok();
        }
        state().copy(physical, buffer.len(), |host, done, chunk| {
            // SAFETY: guest RAM through the physmap and a distinct buffer.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    host as *const u8,
                    buffer[done..].as_mut_ptr(),
                    chunk,
                )
            };
        })
    }

    fn read_ram_descriptor(&mut self, physical: u64) -> Option<u64> {
        super::super::speculation::fence();
        if let Some(ram) = &state().registered_ram {
            return ram.read_descriptor(physical).ok();
        }
        let (host, bytes) = state().run(physical)?;
        if bytes < 8 || !host.is_multiple_of(8) {
            return None;
        }
        // SAFETY: this aligned descriptor fits in one live RAM run and
        // the physical window maps the entire run. No Rust reference to
        // guest memory is formed.
        Some(unsafe { (layout::physical(host) as *const u64).read_volatile() })
    }

    fn write_ram(&mut self, physical: u64, data: &[u8]) -> bool {
        if let Some(ram) = &state().registered_ram {
            // SAFETY: private copyout staging; the canonical owner performs
            // every reached host write under the same physical writer gates.
            return unsafe { ram.publish_physical(physical, data) }.is_ok();
        }
        state().copy(physical, data.len(), |host, done, chunk| {
            // SAFETY: guest RAM through the physmap and a distinct buffer.
            unsafe {
                core::ptr::copy_nonoverlapping(data[done..].as_ptr(), host as *mut u8, chunk)
            };
        })
    }

    fn compare_exchange_ram(
        &mut self,
        physical: u64,
        expected: &mut [u8],
        new: &[u8],
    ) -> Option<bool> {
        let ram = state().registered_ram.as_ref()?;
        // SAFETY: the adopted canonical reservation and mapping remain live;
        // every other core's writes reach the same bytes.
        unsafe { ram.compare_exchange_physical(physical, expected, new) }.ok()
    }

    fn check_ram_range(
        &mut self,
        physical: u64,
        bytes: usize,
        write: bool,
    ) -> Result<(), stela_engine::machine::engine::DmaError> {
        let ram = state().registered_ram.as_ref().ok_or(
            stela_engine::machine::engine::DmaError::Unsupported(refuse!(
                "canonical DMA RAM owner is not installed"
            )),
        )?;
        ram.check_dma_range(physical, bytes, write)
            .map_err(|_| stela_engine::machine::engine::DmaError::Memory)
    }
    fn read_ram_progress(
        &mut self,
        physical: u64,
        buffer: &mut [u8],
    ) -> Result<(), stela_engine::machine::engine::TransferFault> {
        super::super::speculation::fence();
        let Some(ram) = &state().registered_ram else {
            return self.read_ram(physical, buffer).then_some(()).ok_or(
                stela_engine::machine::engine::TransferFault {
                    error: stela_engine::machine::engine::DmaError::Memory,
                    completed: 0,
                },
            );
        };
        // SAFETY: private DMA staging, same canonical reader/owner contract.
        unsafe { ram.read_physical_progress(physical, buffer) }.map_err(|fault| {
            stela_engine::machine::engine::TransferFault {
                error: stela_engine::machine::engine::DmaError::Memory,
                completed: fault.completed,
            }
        })
    }
    fn write_ram_progress(
        &mut self,
        physical: u64,
        data: &[u8],
    ) -> Result<(), stela_engine::machine::engine::TransferFault> {
        let Some(ram) = &state().registered_ram else {
            return self.write_ram(physical, data).then_some(()).ok_or(
                stela_engine::machine::engine::TransferFault {
                    error: stela_engine::machine::engine::DmaError::Memory,
                    completed: 0,
                },
            );
        };
        // SAFETY: private staging, every writer participates in this same domain.
        unsafe { ram.publish_physical(physical, data) }.map_err(|fault| {
            stela_engine::machine::engine::TransferFault {
                error: stela_engine::machine::engine::DmaError::Memory,
                completed: fault.completed,
            }
        })
    }
    fn install(&mut self, code: &[u8]) -> Result<NativeCode, InstallError> {
        let state = state();
        let part_bytes = state.code_part_bytes;
        let part_offset = state.code_unit * part_bytes;
        let physical = state.code_physical + part_offset;
        let extent = state.installed_extents[state.code_unit as usize]
            .append(part_bytes, code.len(), |start| {
                // SAFETY: the ledger reserves this nonoverlapping extent in
                // the owned arena. The writable physmap reaches it.
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        code.as_ptr(),
                        layout::physical(physical + start) as *mut u8,
                        code.len(),
                    );
                }
                serialize_code();
            })
            .ok_or(InstallError::Full)?;
        state.code_peak = state.code_peak.max(part_offset + extent.end);
        #[cfg(feature = "native-profile")]
        state.native_profile.copied(code.len());
        Ok(NativeCode(
            (state.code_virtual + part_offset + extent.start) as usize,
        ))
    }

    fn install_immutable(&mut self, code: &Arc<Vec<u8>>) -> Result<NativeCode, InstallError> {
        if let Some(installed) = state().installed_immutable.lookup(code) {
            #[cfg(feature = "native-profile")]
            state().native_profile.installed(true);
            return Ok(installed);
        }
        #[cfg(feature = "native-profile")]
        state().native_profile.installed(false);
        let installed = self.install(code.as_slice())?;
        state().installed_immutable.record(code, installed);
        Ok(installed)
    }

    fn read_native(&mut self, code: NativeCode, bytes: &mut [u8]) -> bool {
        let state = state();
        if state.native || !state.owns_installed(code, bytes.len()) {
            return false;
        }
        let offset = code.0 as u64 - state.code_virtual;
        // SAFETY: successful install metadata proves this exact initialized,
        // unreclaimed extent. The IF0 CPU owner keeps the arena's
        // backing/root and physmap; no native frame/executable borrow is live.
        unsafe {
            core::ptr::copy_nonoverlapping(
                layout::physical(state.code_physical + offset) as *const u8,
                bytes.as_mut_ptr(),
                bytes.len(),
            );
        }
        true
    }

    fn patch(&mut self, code: NativeCode, bytes: &[u8]) {
        let state = state();
        state.installed_immutable.clear();
        assert!(
            state.owns_installed(code, bytes.len()),
            "a patch outside a retained initialized install"
        );
        let offset = code.0 as u64 - state.code_virtual;
        assert!(
            offset + bytes.len() as u64 <= state.code_bytes,
            "a patch outside the code arena"
        );
        // SAFETY: installed arena bytes through the writable physmap alias.
        unsafe {
            core::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                layout::physical(state.code_physical + offset) as *mut u8,
                bytes.len(),
            );
        }
        serialize_code();
    }

    fn patch_instruction(
        &mut self,
        code: NativeCode,
        instruction: [u8; stela_compiler::x86_64::chain::PATCH_BYTES],
    ) {
        use stela_compiler::x86_64::chain;
        let state = state();
        state.installed_immutable.clear();
        let point = code.0 as u64;
        assert!(
            chain::patchable(point) && state.owns_installed(code, chain::PATCH_BYTES),
            "a patch point outside a retained initialized install"
        );
        // The aligned word holding the point lies in the arena: installs
        // are 16-byte aligned.
        let word = (point & !7) - state.code_virtual;
        // SAFETY: an aligned arena word through the writable physmap alias;
        // this processor is the arena's only writer.
        let word = unsafe {
            core::sync::atomic::AtomicU64::from_ptr(
                layout::physical(state.code_physical + word) as *mut u64
            )
        };
        let current = word.load(core::sync::atomic::Ordering::Relaxed);
        // One store: a core running the point runs the old or the new
        // instruction whole.
        word.store(
            chain::patch_word(current, point, instruction),
            core::sync::atomic::Ordering::Release,
        );
        serialize_code();
    }

    fn discard_code(&mut self) {
        let state = state();
        state.installed_immutable.clear();
        state.code_unit = 0;
        for extents in &mut state.installed_extents {
            extents.clear();
        }
    }

    fn code_window(&self) -> Option<stela_engine::machine::engine::CodeWindow> {
        let state = state();
        // Every executor maps the whole reservation executable at one
        // address in the window root, and the physmap reaches it writable.
        (state.window_bytes != 0).then(|| stela_engine::machine::engine::CodeWindow {
            exec: state.window_virtual as usize,
            write: layout::physical(state.window_physical) as usize,
            bytes: state.window_bytes as usize,
        })
    }

    fn serialize_code(&mut self) {
        serialize_code();
    }

    fn code_partitions(&self) -> Option<stela_engine::machine::engine::CodePartitions> {
        let state = state();
        Some(stela_engine::machine::engine::CodePartitions {
            base: state.code_virtual as usize,
            bytes_per_part: state.code_part_bytes as usize,
            parts: state.code_parts as usize,
        })
    }

    fn reclaim_code(&mut self) -> Option<core::ops::Range<usize>> {
        let state = state();
        state.installed_immutable.clear();
        let unit = state.code_part_bytes;
        state.code_unit = (state.code_unit + 1) % state.code_parts;
        state.installed_extents[state.code_unit as usize].clear();
        let start = (state.code_virtual + state.code_unit * unit) as usize;
        Some(start..start + unit as usize)
    }

    fn release_oldest_code(&mut self) -> Option<core::ops::Range<usize>> {
        let state = state();
        // The ring fills parts in turn: the first holding code after the
        // one being filled is the oldest. With only that one, what it holds
        // is all this machine runs, and the ring bounds its growth: pressure
        // from the rest of the heap would only make it compile the same
        // blocks again.
        let Some(part) = (1..state.code_parts)
            .map(|step| (state.code_unit + step) % state.code_parts)
            .find(|&part| !state.installed_extents[part as usize].is_empty())
        else {
            return Some(0..0);
        };
        state.installed_immutable.clear();
        // This reclaim is for heap pressure, so give back the ledger too.
        state.installed_extents[part as usize] = CodeExtents::default();
        let unit = state.code_part_bytes;
        let start = (state.code_virtual + part * unit) as usize;
        Some(start..start + unit as usize)
    }

    fn memory_low(&self) -> bool {
        crate::ALLOCATOR.low()
    }

    unsafe fn execute(&mut self, code: NativeCode, guest: &mut GuestState) -> NativeExit {
        assert!(!state().native, "nested native block entry");
        state().native = true;
        state().load_pan();
        let entered = rdtsc();
        // SAFETY: the engine passes installed system-mode block code.
        let exit = unsafe { stela_enter_block(guest, code.0) };
        state().follow_native();
        state().follow_pan(guest.pan != 0);
        native_return(code, exit, entered)
    }

    unsafe fn execute_context(
        &mut self,
        code: NativeCode,
        guest: &mut GuestState,
        context: &mut stela_compiler::context::ExecutionContext,
    ) -> Option<NativeExit> {
        assert!(!state().native, "nested native context entry");
        #[cfg(feature = "native-profile")]
        let measured = state().native_profile.work_start(
            stela_engine::machine::engine::NativeTimingWork::NativeCall,
            profile_timestamp,
        );
        state().native = true;
        state().load_pan();
        let entered = rdtsc();
        // SAFETY: caller holds the installed context block, live frame
        // and service owners. The current GS HostOwner keeps recovery state.
        let exit = unsafe { stela_enter_context_block(guest, context, code.0) };
        let returned = native_return(code, exit, entered);
        #[cfg(feature = "native-profile")]
        if let Some(started) = measured {
            state().native_profile.work_end(
                stela_engine::machine::engine::NativeTimingWork::NativeCall,
                started,
                profile_timestamp(),
            );
        }
        Some(returned)
    }

    fn context_entry(&mut self, code: NativeCode) -> Option<usize> {
        // Installed code is callable where it lies, through the executable arena alias.
        Some(code.0)
    }

    fn share_lower_globals(&mut self, span: Arc<GlobalSpan>) {
        let state = state();
        state.lower_globals = Some(span);
        state.reset_lower();
    }

    fn set_regime(&mut self, regime: &Regime) {
        let state = state();
        state.follow_native();
        let mut changed_roots = None;
        if let Some(current) = state.regime
            && current.controls != regime.controls
        {
            let (from, to) = (current.controls, regime.controls);
            // What the lower half's own inputs, E0PD1 and SPRR's
            // permission registers leave of the controls: the lower half has
            // its own reset and the others their bank, and they compose (a
            // context switch writes TTBR0 and the thread's SPRR_PERM_EL0
            // together).
            let (lower_tcr, user_tcr) = match to.regime {
                mmu::Regime::El10 | mmu::Regime::El20 => (LOWER_TCR, USER_TCR),
                _ => (0, 0),
            };
            let own = lower_tcr | user_tcr;
            let rest = mmu::Controls {
                ttbr0: to.ttbr0,
                tcr: from.tcr & !own | to.tcr & own,
                // The fields that keep an index writable after the locks
                // change no access.
                sprr: mmu::Sprr {
                    el0: to.sprr.el0,
                    el1: to.sprr.el1,
                    fields: to.sprr.fields,
                    ..from.sprr
                },
                ..from
            };
            let lower = from.ttbr0 != to.ttbr0 || (from.tcr ^ to.tcr) & lower_tcr != 0;
            // TTBR0 translates only the lower half, but it can name the
            // ASID the upper half's non-global entries were made in.
            let asid = mmu::current_asid(&from) == mmu::current_asid(&to);
            if rest == to && (!lower || asid || !state.extents[UPPER].private()) {
                if lower {
                    let key = |c: &mmu::Controls| (c.ttbr0, c.tcr & lower_tcr);
                    changed_roots = Some(state.switch_lower(key(&from), key(&to)));
                }
                let rights = |c: &mmu::Controls| (c.sprr.el0, c.sprr.el1, c.tcr & user_tcr);
                let (from, to) = (rights(&from), rights(&to));
                let bank = from != to;
                if bank {
                    state.switch_bank(from, to);
                    changed_roots = Some([true; ROOTS]);
                }
                // The helper's software TLB keys neither TTBR0 nor the bank.
                if lower || bank {
                    state.tlb.clear();
                }
            } else {
                let ttbr0 = mmu::Controls {
                    ttbr0: to.ttbr0,
                    ..from
                };
                let sprr = mmu::Controls {
                    sprr: to.sprr,
                    ..from
                };
                let cause = if sprr == to {
                    Cause::Sprr
                } else if ttbr0 == to {
                    Cause::Table
                } else {
                    Cause::Controls
                };
                if matches!(cause, Cause::Controls) && state.legacy_translations_dirty {
                    let fields = differing(&from, &to);
                    for (count, differs) in state.control_fields.iter_mut().zip(fields) {
                        *count += u64::from(differs);
                    }
                    state.last_controls = Some((from, to));
                }
                state.reset_shadow(cause);
            }
        }
        // The tag-checked roots hold one pattern of checked halves.
        if regime.tag_checked() && regime.tag_checks != state.tag_checks {
            if state.tag_checks.contains(&true) {
                state.reset_shadow(Cause::Tags);
            }
            state.tag_checks = regime.tag_checks;
        }
        state.regime = Some(*regime);
        // EL0, EL1, EL1 with PAN, and the GXF guarded level (GL1).
        let view = view_of(regime);
        let root = state.root_of(view);
        let switch_view = root != state.root_of(state.current);
        state.current = view;
        state.view_switches += u64::from(switch_view);
        // Lower-half and bank changes, and a switch of the active view,
        // can coincide. Load the final view after applying all of them.
        let mut load = switch_view;
        if let Some(changed) = changed_roots {
            for (stale, changed) in state.contexts.stale.iter_mut().zip(changed) {
                *stale |= changed;
            }
            load |= changed[root];
        }
        if load {
            state.load_view(view);
        }
    }

    fn isolate_predictions(&mut self) {
        super::super::speculation::isolate();
    }

    fn speculation_barrier(&self) {
        super::super::speculation::fence();
    }

    fn pan_in_flags(&self) -> bool {
        state().contexts.pan_flag
    }

    /// A view of one regime's controls, watchpoints and tag checks whose
    /// root exists: CR3 in its PCID with the no-flush bit while the context
    /// holds nothing stale (the record checks the flag), plain CR3 (which
    /// flushes) without PCIDE; RFLAGS.AC for PAN where SMAP stands for it.
    fn native_view(&mut self, from: &Regime, to: &Regime) -> Option<NativeView> {
        use stela_compiler::x86_64::transition::{AC_CLEAR, AC_KEEP, AC_SET};
        let state = state();
        if from.controls != to.controls
            || from.watch != to.watch
            || from.tag_checks != to.tag_checks
            || state.regime != Some(*to)
        {
            return None;
        }
        let view = view_of(to);
        let root = state.root_of(view);
        let base = *state.roots.get(root)?;
        if base == 0 {
            return None;
        }
        let cr3 = if state.contexts.tagged {
            base | view_context(root) | crate::engine::storage::KEEP_ENTRIES
        } else {
            base
        };
        let ac = if !state.contexts.pan_flag {
            AC_KEEP
        } else if view % VIEWS == PAN_VIEW {
            AC_CLEAR
        } else {
            AC_SET
        };
        Some(NativeView {
            cr3,
            root_at: &state.roots[root] as *const u64 as u64,
            root: base,
            stale_at: &state.contexts.stale[root] as *const bool as u64,
            view_at: &state.native_view as *const u64 as u64,
            view: native_view_word(view, to.requester),
            ac,
        })
    }

    fn invalidate_translations(&mut self) {
        state().reset_shadow(Cause::Invalidated);
    }

    fn invalidate_reach(&mut self, reach: Reach, everywhere: Everywhere) {
        let state = state();
        if !state.kept.is_empty() {
            let bounds = state.half_range(LOWER);
            let entry_bits = 12 + 9 * layout::top_level();
            let pool = &state.pool;
            // Kept halves share the current controls' TCR outside the lower
            // half's fields; without a regime the ASIDs are unknown.
            let tcr = state
                .regime
                .map_or(mmu::shadow::UNKNOWN_ASIDS, |regime| regime.controls.tcr);
            let dropped = match everywhere {
                Everywhere::NonGlobal(asid) => state.kept.drop_asid(asid, tcr),
                _ => state.kept.invalidate(
                    everywhere,
                    tcr,
                    bounds,
                    entry_bits,
                    |entry, first, last| tables::unmap_under(pool, entry, first, last),
                ),
            };
            state.kept_dropped += dropped as u64;
        }
        if reach == Reach::All {
            return match everywhere {
                Everywhere::NonGlobal(_) => state.drop_non_global(),
                _ => state.reset_shadow(Cause::Maintained),
            };
        }
        state.tlb.clear();
        // Most maintenance addresses reach a single extent, without a
        // large overlapping leaf or a second half.
        let mut first_range = None;
        let mut extra_ranges = Vec::new();
        let mut pages = 0u64;
        let mut whole = false;
        for half in [LOWER, UPPER] {
            let bounds = state.half_range(half);
            if !state.extents[half].visit_ranges(reach, bounds, |first, last| {
                pages = pages.saturating_add(((last >> 12) - (first >> 12)) + 1);
                if first_range.is_none() {
                    first_range = Some((first, last));
                } else {
                    extra_ranges.push((first, last));
                }
            }) {
                state.clear(half);
                whole = true;
            }
        }
        const { assert!(INVLPG_PAGES <= u16::BITS as u64) };
        let precise = !whole && pages <= INVLPG_PAGES;
        let current = state.root_of(state.current);
        let mut reload_current = false;
        for (first, last) in first_range.into_iter().chain(extra_ranges) {
            let base = first & !(PAGE - 1);
            for (view, root) in state.roots.into_iter().enumerate() {
                let mut removed = 0u16;
                tables::unmap(&state.pool, root, first, last, |address| {
                    let bit = if precise { (address - base) / PAGE } else { 0 };
                    removed |= 1 << bit;
                });
                if whole || removed == 0 {
                    continue;
                }
                if !precise {
                    state.contexts.stale[view] = true;
                    reload_current |= view == current;
                    continue;
                }
                while removed != 0 {
                    let page = base + u64::from(removed.trailing_zeros()) * PAGE;
                    removed &= removed - 1;
                    if view == current {
                        invlpg(page);
                    } else {
                        state.contexts.forget_inactive(page, view);
                    }
                }
            }
            // A bank has no PCID while it is inactive. Switching to it
            // flushes the views before they can use its entries again.
            for root in state.banks.iter().flat_map(|bank| bank.roots) {
                tables::unmap(&state.pool, root, first, last, |_| {});
            }
        }
        if whole {
            state.reload();
        } else if reload_current {
            state.load_view(state.current);
        }
        if whole {
            state.resets += 1;
        } else {
            state.scoped += 1;
        }
    }

    fn profile_ticks(&mut self) -> u64 {
        rdtsc()
    }

    fn profile_boundary(
        &mut self,
        phase: stela_engine::machine::engine::step_timing::Phase,
        now: u64,
    ) {
        pmu::boundary(phase, now);
    }

    fn counter(&mut self) -> u64 {
        fresh_counter()
    }

    fn tsc_counter(&self) -> Option<(u64, u64)> {
        let state = state();
        Some((state.native_tsc_base(), state.counter_per_tsc))
    }

    fn arm_timer(&mut self, deadline: Option<u64>) {
        let now = self.counter();
        let state = state();
        let Some(deadline) = deadline else {
            if crate::native_clock::arm_guest(None) {
                state.timer_lvt = None;
            } else {
                state.apic.write(apic::INITIAL, 0);
            }
            return;
        };
        let guest = deadline.saturating_sub(now).max(1);
        let ticks = u128::from(guest) * u128::from(state.apic_hz) / u128::from(COUNTER_HZ);
        let ticks = ticks.clamp(1, u128::from(u32::MAX)) as u32;
        if crate::native_clock::arm_guest(Some(ticks)) {
            // The native clock owns the timer and may set its own mode.
            state.timer_lvt = None;
        } else {
            // One APIC write per arm where the LVT already holds ours: each
            // is a WRMSR (x2APIC) or uncached store, and a VM exit in a VM.
            let handovers =
                crate::native_clock::TIMER_HANDOVERS.load(core::sync::atomic::Ordering::Acquire);
            if state.timer_lvt != Some(handovers) {
                state.apic.write(apic::LVT_TIMER, u32::from(TIMER_VECTOR));
                state.timer_lvt = Some(handovers);
            }
            state.apic.write(apic::INITIAL, ticks);
        }
    }

    fn wait_for_interrupt(&mut self) {
        crate::native_mmio::shootdown::service();
        local::clear_interrupt();
        // The wait sends queued serial output until an interrupt comes,
        // opening a window for one after each burst.
        while crate::output::send_queued() {
            // SAFETY: STI holds interrupts off for one more instruction, so
            // only the NOP's boundary takes a pending one.
            unsafe { asm!("sti", "nop", "cli", options(nomem, nostack)) };
            if local::interrupted() {
                return;
            }
        }
        // The wait may last until the armed one-shot count runs out.
        const MASKED: u32 = 1 << 16;
        let budget = || {
            let state = state();
            let remaining = state.apic.read(apic::CURRENT);
            (state.apic_hz != 0 && state.apic.read(apic::LVT_TIMER) & MASKED == 0 && remaining != 0)
                .then(|| u64::from(remaining) * 1_000_000 / state.apic_hz)
        };
        let ring = state().ring;
        // SAFETY: interrupts are disabled and the host IDT takes the wake;
        // the ring is this processor's.
        unsafe { crate::native_idle::wait_with(budget, ring) };
    }

    fn relax(&mut self) {
        crate::native_mmio::shootdown::service();
        core::hint::spin_loop();
    }

    fn halts_on_doorbell(&self) -> bool {
        crate::native_idle::waits_on_doorbell(state().ring)
    }

    fn poll_interrupt(&mut self) -> bool {
        crate::native_mmio::shootdown::service();
        crate::output::send_queued();
        // SAFETY: as in the wait above: only the NOP's boundary takes a
        // pending interrupt.
        unsafe { asm!("sti", "nop", "cli", options(nomem, nostack)) };
        let interrupted = local::interrupted();
        if interrupted {
            local::clear_interrupt();
        }
        interrupted
    }

    fn host_work_due(&mut self) -> bool {
        super::super::storage::signalled()
    }

    fn report(&mut self, event: &stela_engine::machine::Event) {
        super::super::report(event);
    }

    fn placement(&self) -> Placement {
        state().placement
    }

    fn translation_helper(&self) -> Option<u64> {
        state().bounce.as_ref().map(|_| translated::slot())
    }

    fn translations(&mut self) -> Option<&mut dyn stela_engine::machine::Translations> {
        super::super::translations::store()
            .map(|store| store as &mut dyn stela_engine::machine::Translations)
    }
}
