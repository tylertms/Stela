//! Native faults: recovery footers, abandoned blocks, page faults and native
//! exceptions.

use super::*;
use stela_compiler::x86_64::flag_word;

// Native code owns the FP registers: the host is soft-float, translated code
// keeps no guest state in them across an exit and IRQs use their IST images.
// recovery_rsp holds the host's registers, which return and recovery pop.
global_asm!(
    r#"
.global stela_enter_block
stela_enter_block:
    xor r10d, r10d
    jmp stela_enter_owned
.global stela_enter_context_block
stela_enter_context_block:
    mov r10d, 1
stela_enter_owned:
    push rbp
    push rbx
    push r12
    push r13
    push r14
    push r15
    push rdi
    push rsi
    push rdx
    cmp qword ptr gs:[{recovery}], 0
    jne {refuse}
    cmp qword ptr gs:[{helper}], 0
    jne {refuse}
    ldmxcsr [rip + {default_mxcsr}]
    mov gs:[{recovery}], rsp
    test r10d, r10d
    jne stela_enter_owned_context
    // Preserve the legacy generated-block entry alignment (RSP%16=0 in the
    // block); the separate context ABI remains SysV (RSP%16=8 in the block).
    sub rsp, 8
    mov rax, rsi
    // The pinned guest registers (`x86_64::pinned`) live in these while
    // native code runs, with the flag word (`x86_64::flag_word`).
    mov ecx, [rdi + {nzcv}]
    shr ecx, {arm_shift}
    imul ecx, ecx, {pack}
    and ecx, {word_mask}
    mov [rdi + {nzcv}], ecx
    mov rbx, [rdi + {sp}]
    mov rsi, [rdi + 64]
    mov r14, [rdi + 152]
    mov r15, [rdi + 0]
    mov rbp, [rdi + 8]
    mov r12, [rdi + 128]
    mov r13, [rdi + 136]
    call rax
    mov [rdi + {sp}], rbx
    mov [rdi + 64], rsi
    mov [rdi + 152], r14
    mov [rdi + 0], r15
    mov [rdi + 8], rbp
    mov [rdi + 128], r12
    mov [rdi + 136], r13
    mov ecx, [rdi + {nzcv}]
    and ecx, {word_mask}
    imul ecx, ecx, {unpack}
    and ecx, {arm_mask}
    mov [rdi + {nzcv}], ecx
    jmp stela_native_owned_return
stela_enter_owned_context:
    call rdx
stela_native_owned_return:
    cmp qword ptr gs:[{helper}], 0
    jne {refuse}
    jmp stela_native_restore_host
.global stela_native_recover
stela_native_recover:
    xor eax, eax
    mov edx, 0xffff
stela_native_restore_host:
    // An abandoned helper has no surviving borrowed state or image.
    mov rsp, gs:[{recovery}]
    test rsp, rsp
    jz {refuse}
    mov qword ptr gs:[{helper}], 0
    mov qword ptr gs:[{recovery}], 0
    add rsp, 24
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
    ret
"#,
    sp = const core::mem::offset_of!(GuestState, sp),
    nzcv = const core::mem::offset_of!(GuestState, nzcv),
    arm_shift = const flag_word::ARM_SHIFT,
    pack = const flag_word::PACK_MULTIPLIER,
    unpack = const flag_word::UNPACK_MULTIPLIER,
    word_mask = const flag_word::WORD_MASK,
    arm_mask = const flag_word::ARM_MASK,
    recovery = const local::RECOVERY_RSP_OFFSET,
    helper = const local::HELPER_RSP_OFFSET,
    refuse = sym super::super::xstate::stela_xstate_boundary_refuse,
    default_mxcsr = sym super::super::xstate::DEFAULT_MXCSR,
);

unsafe extern "sysv64" {
    pub(super) fn stela_enter_block(state: *mut GuestState, code: usize) -> BlockExit;
    pub(super) fn stela_enter_context_block(
        state: *mut GuestState,
        context: *mut stela_compiler::context::ExecutionContext,
        code: usize,
    ) -> BlockExit;
    pub(super) fn stela_native_recover();
}

/// Whether `rip` lies in native code this processor runs: its private arena
/// or the board's code window.
pub(super) fn in_code(state: &State, rip: u64) -> bool {
    state.native
        && ((state.code_virtual..state.code_virtual + state.code_bytes).contains(&rip)
            || (state.window_virtual..state.window_virtual + state.window_bytes).contains(&rip))
}

/// End the native block at `frame` and report `cause` to the engine.
pub(super) fn abandon(
    state: &mut State,
    frame: &mut Frame,
    address: u64,
    write: bool,
    cause: FaultCause,
    saved: &super::super::interrupts::SavedState<'_>,
) {
    state.recorded = Some(Recorded {
        rip: frame.rip,
        origin: stela_engine::machine::NativeFaultOrigin::Exception,
        trap: u16::try_from(frame.vector).unwrap_or(u16::MAX),
        error_code: frame.error,
        address,
        write,
        cause,
        registers: saved
            .registers()
            .unwrap_or_else(|_| {
                crate::output::fatal("native fault has no valid owned interrupt register image")
            })
            .with_vectors(saved.vectors()),
    });
    frame.rip = stela_native_recover as *const () as u64;
}

/// The guest's translation of the access at `address`, through the same
/// software walk for shadow faults and the translation helper: the
/// translation and the host physical address of its RAM page, or why the
/// engine must take the access (Arm checks watchpoints after translation,
/// `AArch64_TranslateAddress`, then Tag Checks).
pub(super) fn resolve(
    state: &mut State,
    address: u64,
    write: bool,
    unprivileged: bool,
) -> Result<(mmu::Translation, u64), FaultCause> {
    let regime = state.regime.as_ref().ok_or(FaultCause::NonCanonical)?;
    let access = if write { Access::Write } else { Access::Read };
    let requester = mmu::Requester {
        unprivileged: regime.requester.unprivileged || unprivileged,
        ..regime.requester
    };
    let virtual_page = address & !(PAGE - 1);
    if let Some(physical) = state.tlb.device_physical(address, requester, write) {
        // The cached translation already passed both stages and data
        // permissions. Watchpoint passes can change without a new walk.
        return Err(if regime.watch.blocks(virtual_page) {
            FaultCause::Watched
        } else {
            FaultCause::Device { physical }
        });
    }
    let translation = match mmu::translate(
        &regime.controls,
        address,
        access,
        requester,
        &mut GuestTables::new(state),
    ) {
        Outcome::Mapped(translation) => translation,
        Outcome::Fault(fault) => return Err(FaultCause::Guest(fault)),
        Outcome::Unsupported(unsupported) => return Err(FaultCause::Translation(unsupported)),
    };
    if regime.watch.blocks(virtual_page) {
        return Err(FaultCause::Watched);
    }
    if regime.tag_blocks(address, translation.attributes) {
        return Err(FaultCause::TagChecked);
    }
    let page = translation.physical & !(PAGE - 1);
    let Some(host) = state.host_physical(page) else {
        // This walker makes no AF or dirty descriptor updates.
        state.tlb.record_device(address, requester, translation);
        return Err(FaultCause::Device {
            physical: translation.physical,
        });
    };
    Ok((translation, host))
}

/// #PF: fill the shadow from the guest's tables or end the block precisely.
pub(crate) fn page_fault(
    frame: &mut Frame,
    address: u64,
    saved: &super::super::interrupts::SavedState<'_>,
) {
    let entered = rdtsc();
    let write = frame.error & 2 != 0;
    #[cfg(feature = "nvk")]
    if super::super::gpu::client::serve_fault(address, write) {
        return;
    }
    let state = state();
    if !in_code(state, frame.rip) {
        super::super::interrupts::fatal(frame);
    }
    state.follow_native();
    state.follow_pan(frame.rflags & RFLAGS_AC == 0);
    let Some(tag_checks) = state.regime.as_ref().map(|regime| regime.tag_checks) else {
        super::super::interrupts::fatal(frame);
    };
    // Only the native ranges are shadow; the rest is the engine's. An
    // access past a check an earlier one stood for may reach the guard
    // (`mmu::placement`): its own check finds where it goes.
    if !state.placement.native(address) {
        state.guard_faults += u64::from(state.placement.in_guard(address));
        return abandon(
            state,
            frame,
            address,
            write,
            FaultCause::NonCanonical,
            saved,
        );
    }
    let (translation, host) = match resolve(state, address, write, false) {
        Ok(resolved) => resolved,
        Err(cause) => return abandon(state, frame, address, write, cause, saved),
    };
    // EL1 and PAN share a root. SMAP guards a leaf if a read with PAN set
    // would be denied by either stage, or by stage 1's PAN rules.
    let guarded = state.contexts.pan_flag
        && state.current % VIEWS == EL1_VIEW
        && (!translation.readable || translation.pan_sensitive);
    let rights = PRESENT
        | NX
        | if translation.writable { WRITABLE } else { 0 }
        | if guarded { tables::USER } else { 0 };
    let root = state.roots[state.root_of(state.current)];
    let virtual_page = address & !(PAGE - 1);
    let half = if address >> 63 == 0 { LOWER } else { UPPER };
    if half == LOWER
        && translation.global
        && state.lower_globals.as_ref().is_some_and(|span| {
            let (first, last) = shadow::leaf(address, translation.size);
            span.widen(first, last)
        })
    {
        return;
    }
    state.pool.half = half;
    // A failed allocation can already have installed intermediate tables.
    state.legacy_translations_dirty = true;
    let slot = match tables::map_page_slot(&mut state.pool, root, virtual_page, host, rights) {
        Ok(slot) => slot,
        Err(_) => {
            // The half's frames are exhausted: start it over and map again.
            // The lower half's kept halves hold frames until it is emptied.
            state.exhausted += 1;
            if half == LOWER {
                state.reset_lower();
            } else {
                state.reset_shadow(Cause::Exhausted);
            }
            state.pool.half = half;
            state.legacy_translations_dirty = true;
            match tables::map_page_slot(&mut state.pool, root, virtual_page, host, rights) {
                Ok(slot) => slot,
                Err(_) => super::super::interrupts::fatal(frame),
            }
        }
    };
    // A not-present fault has no cached translation to invalidate. Every
    // shadow removal already invalidates that view or marks it stale.
    // A protection fault can replace a present mapping.
    if frame.error & 1 != 0 {
        invlpg(virtual_page);
    }
    if half == LOWER {
        let entry = tables::index(virtual_page, layout::top_level());
        state.record_lower(root, entry);
    }
    state.extents[half].record(address, translation.size, translation.global);
    // The rest of the leaf's window has the same translation and rights:
    // map its RAM pages that are not watched now, as a TLB entry covers
    // the whole leaf. Pages that were absent need no flush. A Tagged leaf
    // whose tags are checked maps only the passed page.
    if tag_checks[half] && translation.attributes == mmu::attributes::TAGGED {
        state.fills += 1;
        state.fill_ticks += rdtsc().wrapping_sub(entered);
        return;
    }
    const { assert!(FILL_WINDOW.is_power_of_two() && FILL_WINDOW <= tables::LARGE) };
    let (first, last) = shadow::window(address, translation.size, FILL_WINDOW);
    if last - first >= PAGE {
        let physical = translation
            .physical
            .wrapping_sub(address)
            .wrapping_add(first);
        // Contiguous backing is common. Look up the first page once, then
        // fall back to page lookups past the end of that physical run.
        let (first_host, available) = state.run(physical).unwrap_or((0, 0));
        let table = slot.wrapping_sub(tables::index(virtual_page, 0));
        let watch = &state.regime.as_ref().expect("checked above").watch;
        let watched = watch.overlaps(first, last);
        let mut page = first;
        loop {
            if page != virtual_page && (!watched || !watch.blocks(page)) {
                let offset = page - first;
                let host = if offset < available {
                    first_host.checked_add(offset)
                } else {
                    state.host_physical(physical.wrapping_add(offset))
                };
                if let Some(host) = host {
                    // SAFETY: the power-of-two prefill window is contained
                    // in one leaf table. The just-mapped slot belongs to
                    // that live table, exclusively managed by this host.
                    unsafe {
                        table
                            .add(tables::index(page, 0))
                            .write_volatile((host & tables::ADDRESS) | rights)
                    };
                    state.prefills += 1;
                }
            }
            if page >= last & !(PAGE - 1) {
                break;
            }
            page += PAGE;
        }
    }
    state.fills += 1;
    state.fill_ticks += rdtsc().wrapping_sub(entered);
}

/// #GP or #UD inside native code ends the block; elsewhere it is fatal.
pub(crate) fn native_exception(
    frame: &mut Frame,
    saved: &super::super::interrupts::SavedState<'_>,
) {
    let state = state();
    if !in_code(state, frame.rip) {
        super::super::interrupts::fatal(frame);
    }
    let cause = if frame.vector == 13 {
        FaultCause::NonCanonical
    } else {
        FaultCause::InvalidOpcode
    };
    abandon(state, frame, 0, false, cause, saved);
}
