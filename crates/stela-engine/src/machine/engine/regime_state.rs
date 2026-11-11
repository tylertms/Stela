//! Derived regime state: what System register writes and EL changes make the
//! engine forget.

use super::*;

impl<H: Host, D: Devices> Machine<H, D> {
    /// A write of `register`: a [`stored`] one reaches only its own cells,
    /// a timer channel's changes only its deadline, a PAC key's everything
    /// derived except the read cells, and any other everything derived.
    pub(super) fn register_written(&mut self, register: SystemRegister) {
        if timer::channel_register(register) {
            self.timers = None;
            return;
        }
        if pointer_key(register) {
            return self.forget_pointer_derived();
        }
        if !stored(register) {
            return self.derived_changed();
        }
        use registers::encodings as e;
        let encoding = match register {
            SystemRegister::TPIDR_EL1 => e::TPIDR_EL1,
            SystemRegister::TPIDR_EL2 => e::TPIDR_EL2,
            _ => return,
        };
        if let Some(index) = MIRRORED.iter().position(|&mirrored| mirrored == encoding) {
            if !self.cpu.degraded.contains_key(&encoding.selector()) {
                let value = self.cpu.registers.get(register, 0);
                self.mirrors.update(index, value);
            } else {
                self.mirrors.close(index);
            }
        }
    }

    /// The PE's bank of cells ([`Mirrors`]).
    pub(super) fn mirror_state(&self) -> usize {
        let pstate = self.cpu.pstate;
        mirror_state(pstate.el, pstate.guarded)
    }

    /// Forget state derived from System registers and PSTATE.
    pub(super) fn derived_changed(&mut self) {
        self.forget_derived();
        self.mirrors.transitions.forget();
        self.device_lines = None;
        self.mirrors.close_all();
        // Under step timing, the System exit being handled is charged with it.
        self.timed_closed |= self.timed_word.is_some();
    }

    /// An exception entry or return, GENTER or GEXIT moved the PE to another
    /// EL or GXF state: what derives from PSTATE goes, but the mirrored reads
    /// keep their cells per state ([`Self::cells_moved`]).
    pub(super) fn transitioned(&mut self) {
        // The timer deadline is an absolute count derived from timer state.
        // Polling still applies the new PSTATE masks and interrupt routing.
        self.forget_pstate_derived();
        self.cells_moved();
    }

    /// The cells of the PE's state take over: an open DAIF cell reads the
    /// current masks and an open counter cell the count.
    pub(super) fn cells_moved(&mut self) {
        let pstate = self.cpu.pstate;
        self.mirrors.transition(self.mirror_state());
        let daif = &mut self.mirrors[DAIF_MIRROR];
        if daif.readable != 0 {
            daif.value = daif_register(pstate.daif);
        }
        for index in COUNTER_MIRRORS {
            if self.mirrors[index].readable == 0 {
                continue;
            }
            let read = decode_register(read_word(MIRRORED[index])).expect("an MRS word");
            let physical = self.now();
            self.mirrors[index] = match self.cpu.apple_access(read, physical) {
                RegisterOutcome::Read(value) => register_read::Cell {
                    readable: 1,
                    value: self.degraded_read(read, value),
                },
                _ => register_read::Cell::default(),
            };
        }
        self.sync_host_regime(step_timing::SyncSource::Transition);
    }

    /// What [`Self::derived_changed`] forgets but the mirrored reads and
    /// the device lines.
    pub(super) fn forget_derived(&mut self) {
        self.timers = None;
        self.mirrors.transitions.forget();
        self.forget_pstate_derived();
    }

    /// [`Self::forget_derived`] after a write of PAC's inputs alone
    /// (`Changes::pointer`): GENTER and GEXIT go between EL2 and its guarded
    /// level, where every PAC input is the same register (APCTL's EL2
    /// level, SCTLR_EL2, the EL2 keys, TCR_EL2, no trap), so the service the
    /// engine reopens after either is the one native code keeps across it,
    /// and their records stay. SVC and ERET records, whose two levels' PAC
    /// use may differ, go.
    pub(super) fn forget_pointer_derived(&mut self) {
        self.timers = None;
        self.mirrors
            .transitions
            .forget_but(&[transition::GUARDED_ENTER, transition::GUARDED_EXIT]);
        self.forget_pstate_derived();
    }

    fn forget_pstate_derived(&mut self) {
        self.forget_state_derived();
        self.reopen_pointers();
    }

    /// What [`Self::forget_pstate_derived`] forgets but the PAC service.
    pub(super) fn forget_state_derived(&mut self) {
        self.floating_point = None;
        self.memory_checks = None;
        self.native_system = None;
        self.debug = None;
    }

    /// The PAC service after a derived-state change: what native code had
    /// open reopens under the new keys, version, layout, enables and traps,
    /// exactly as the PAC exits that opened it would now decide, so the
    /// change costs no exit of its own. PACM changes the next instruction's
    /// operands, which no thunk applies: everything stays closed.
    pub(super) fn reopen_pointers(&mut self) {
        if !self.pointer.any_open() || self.cpu.pstate.pacm {
            self.pointer.close();
            return;
        }
        self.revalidate_pointers();
    }

    /// Bring the PAC service to the current keys, version, layout, enables
    /// and traps, opening every key's tag word they allow.
    pub(super) fn revalidate_pointers(&mut self) {
        let Ok(version) = self.pointer_version() else {
            self.pointer.close();
            return;
        };
        let inputs = PointerInputs {
            keys: self.pointer_keys(),
            version,
            tcr: self.cpu.regime_tcr(),
        };
        let stripping = self.pointer.revalidate(inputs);
        let trapped = self.pointer_authentication_trapped();
        let enables = self.pointer_key_enables();
        for (key, block) in [
            (
                stela_contracts::pauth::Key::Ia,
                PointerAuthenticationKey::Ia,
            ),
            (
                stela_contracts::pauth::Key::Ib,
                PointerAuthenticationKey::Ib,
            ),
            (
                stela_contracts::pauth::Key::Da,
                PointerAuthenticationKey::Da,
            ),
            (
                stela_contracts::pauth::Key::Db,
                PointerAuthenticationKey::Db,
            ),
        ] {
            let enabled = enables & pointer_authentication::key_enable_bit(block) != 0;
            self.pointer.reopen(key, enabled, trapped);
        }
        self.pointer.reopen_generic(trapped);
        for (data, open) in [false, true].into_iter().zip(stripping) {
            if open {
                self.pointer.reopen_stripping(data);
            }
        }
    }

    /// Whether loads and stores through SP check its 16-byte alignment (Arm
    /// `CheckSPAlignment`): SCTLR_ELx.SA0 at EL0 and SA otherwise, of the
    /// current translation regime.
    pub(super) fn stack_alignment_checked(&mut self) -> bool {
        self.memory_checks().stack_alignment
    }

    /// The checks native code makes for the current regime's SCTLR_ELx: SP
    /// alignment, A (Arm `AlignmentEnforced`) and nAA; and, where
    /// ID_AA64PFR0_EL1.CSV3 discloses FEAT_CSV3, the guest range check at
    /// every access (`identification::Speculation::faulting_data`).
    pub(super) fn memory_checks(&mut self) -> MemoryChecks {
        *self.memory_checks.get_or_insert_with(|| {
            let sctlr = self.cpu.regime_sctlr();
            let bit = if self.cpu.pstate.el == 0 { 4 } else { 3 };
            MemoryChecks::system(
                sctlr & (1 << bit) != 0,
                sctlr & mmu::alignment::SCTLR_A != 0,
                sctlr & mmu::alignment::SCTLR_NAA == 0,
                self.cpu
                    .field(SystemRegister::ID_AA64PFR0_EL1, CSV3_SHIFT, 4)
                    != 0,
            )
            // EL0's one-sided check: EL0 code addresses the lower range.
            .with_range(
                self.host
                    .placement()
                    .check()
                    .one_sided(self.cpu.pstate.el == 0 && !self.cpu.pstate.guarded),
            )
        })
    }

    /// Whether this PE's memory accesses go through its canonical memory:
    /// a machine held for the parallel PEs keeps it but accesses RAM as
    /// they do.
    pub(super) fn canonical_memory_serves(&self) -> bool {
        self.cpu.has_canonical_memory() && self.serial_hold.is_none()
    }

    pub(super) fn native_system(&mut self) -> NativeSystem {
        *self.native_system.get_or_insert_with(|| NativeSystem {
            zero_block: system::native_zero_block(&self.cpu, &self.profile_missing),
            cache_clean: system::native_cache_clean(&self.cpu),
            exclusive: self.parallel.is_some(),
            pointer: Some(super::pointer::CELLS),
            mirrored: inline_mirrored_reads(),
            amx: self.cpu.amx.generation().is_some(),
            self_loops: self.self_loops,
            loop_counts: self.step_timing.is_some(),
            capture_every_nzcv: self.capture_every_nzcv,
        })
    }

    /// Run self-loops inside their blocks ([`NativeSystem::self_loops`]),
    /// the default, or through their links, from the next compilation on;
    /// the blocks compiled so far stay. For tests that compare the two.
    #[cfg(any(test, feature = "test-support"))]
    pub fn set_self_loops(&mut self, on: bool) {
        self.self_loops = on;
        self.native_system = None;
    }

    /// Have every flag-setting instruction store NZCV as it runs
    /// ([`NativeSystem::capture_every_nzcv`]), or leave the default, from
    /// the next compilation on. For tests that compare the two.
    #[cfg(any(test, feature = "test-support"))]
    pub fn set_capture_every_nzcv(&mut self, on: bool) {
        self.capture_every_nzcv = on;
        self.native_system = None;
    }

    pub(super) fn sync_regime(&mut self) {
        self.derived_changed();
        self.sync_host_regime(step_timing::SyncSource::Derived);
    }

    /// The host's view of the regime (shadow root, requester, watchpoints and
    /// tag checks) and the PMU's and prediction contexts follow the PE, with
    /// state derived from System registers left as it is.
    /// `source` asked for it ([`step_timing::RegimeSyncs::sources`]).
    pub(super) fn sync_host_regime(&mut self, source: step_timing::SyncSource) {
        self.sync_monitor();
        let watch = Watch {
            pass: self.watch.pass,
            ..Watch::from_ranges(&debug::watched(&self.cpu))
        };
        if watch.ranges != self.watch.ranges {
            // Newly watched pages must leave the shadow tables.
            self.advance_translation_generation();
            self.host.invalidate_translations();
        }
        self.watch = watch;
        self.tag_checks = self.cpu.tag_checks();
        self.cpu.note_ctrr_locks();
        let regime = Regime {
            controls: self.cpu.translating_controls(self.cpu.regime()),
            requester: self.cpu.requester(),
            watch,
            tag_checks: self.tag_checks,
        };
        // Native device reads key the requester's state and PAN themselves.
        let same = self.synced_regime == Some(regime);
        if !self.synced_regime.is_some_and(|synced| {
            synced.controls == regime.controls
                && synced.watch == regime.watch
                && synced.tag_checks == regime.tag_checks
        }) {
            self.forget_device_reads();
        }
        self.synced_regime = Some(regime);
        self.publish_translating(&regime.controls);
        let syncing = self.timing_start();
        self.host.set_regime(&regime);
        let synced = self.timing_since(syncing);
        let context = PredictionContext::of(&self.cpu);
        let mut isolated = None;
        if self.prediction_context != Some(context) {
            self.prediction_context = Some(context);
            self.statistics.prediction_isolations += 1;
            let isolating = self.timing_start();
            self.host.isolate_predictions();
            isolated = Some(self.timing_since(isolating));
        }
        if self.step_timing.is_some() {
            let regimes = &mut self.system_exits.regimes;
            regimes.syncs += 1;
            regimes.sync_ticks += synced;
            regimes.unchanged += u64::from(same && isolated.is_none());
            regimes.sources[source as usize] += 1;
            if let Some(ticks) = isolated {
                regimes.isolations += 1;
                regimes.isolation_ticks += ticks;
            }
        }
    }
}

/// The [`MIRRORED`] registers whose MRS blocks complete inside themselves
/// (`x86_64::mirrored_read`): those whose thunk reads the cell's value
/// alone (`register_read::thunk`), with the kernel's thread pointer first.
/// The guarded, count, counter and DAIF mirrors keep their thunks.
fn inline_mirrored_reads() -> stela_compiler::x86_64::mirrored_read::Reads {
    use registers::encodings::{CURRENT_EL, MPIDR_EL1, TPIDR_EL1, TPIDR_GL2};
    use stela_compiler::x86_64::mirrored_read::{Read, Reads};
    let read = |encoding| {
        let index = MIRRORED
            .iter()
            .position(|&mirrored| mirrored == encoding)
            .expect("a mirrored register");
        debug_assert!(
            !GUARDED_MIRRORS.contains(&index)
                && !COUNT_MIRRORS.contains(&index)
                && !COUNTER_MIRRORS.contains(&index)
                && index != DAIF_MIRROR
        );
        let cell = cell_place(index);
        debug_assert_eq!(cell.anchor, stela_compiler::Anchor::Cells);
        Some(Read {
            word: read_word(encoding),
            cell: u32::try_from(cell.offset).expect("a cell offset"),
        })
    };
    Reads {
        reads: [
            read(TPIDR_EL1),
            read(TPIDR_GL2),
            read(MPIDR_EL1),
            read(CURRENT_EL),
        ],
    }
}
