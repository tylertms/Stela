use super::super::LocalRegisters;
use super::*;
use core::sync::atomic::AtomicUsize;

struct Kicks(Arc<AtomicUsize>);
impl Kick for Kicks {
    fn kick(&self, _pe: usize) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

fn hub(pes: usize) -> (Arc<Hub>, Arc<AtomicUsize>) {
    let kicks = Arc::new(AtomicUsize::new(0));
    let hub = Hub::new(pes, Kicks(Arc::clone(&kicks)));
    for pe in 0..pes {
        hub.set_live(pe, true);
    }
    // Every PE but the first, which a test's machine may be, runs a block
    // (`Hub::enter_active`): a broadcast waits for it.
    for pe in 1..pes {
        assert!(hub.enter_active(pe));
    }
    (hub, kicks)
}

fn tlbi(kind: crate::machine::system::TlbiKind, operand: u64) -> Maintenance {
    use crate::machine::system::{Regime, Tlbi, TlbiAttr, TlbiLevel};
    Maintenance::Tlb(tlbi::Scope::new(
        &Tlbi {
            kind,
            regime: Regime::EL10,
            vmid: None,
            broadcast: Broadcast::ISH,
            level: TlbiLevel::Any,
            attr: TlbiAttr::AllAttr,
            operand,
        },
        tlbi::Context::default(),
    ))
}

/// A TLBI of another context leaves its mail for the PE's next read, and
/// neither interrupts it nor waits for it.
#[test]
fn a_tlbi_passes_by_a_pe_translating_another_context() {
    use crate::machine::system::TlbiKind::{ASID, VA, VAA};
    const ASID_5: u64 = 5 << 48;
    let (hub, _) = hub(2);
    let inbox = &hub.inboxes[1];
    let word = context_word(mmu::Regime::El10, 7);
    let epoch = || inbox.chain.interrupt_epoch().load(Ordering::Relaxed);
    assert_eq!(
        hub.broadcast(0, &[tlbi(ASID, ASID_5)]).len(),
        1,
        "unpublished"
    );
    inbox.translating.store(word, Ordering::Relaxed);
    let interrupts = epoch();
    let passes = [ASID_5, ASID_5 | 0x1234];
    for (kind, operand) in [ASID, VA].into_iter().zip(passes) {
        assert!(hub.broadcast(0, &[tlbi(kind, operand)]).is_empty());
    }
    assert_eq!(epoch(), interrupts);
    assert!(hub.pending(1));
    assert_eq!(hub.take(1).unwrap().maintenance.len(), 3);
    assert_eq!(
        hub.broadcast(0, &[tlbi(ASID, 7 << 48)]).len(),
        1,
        "its context"
    );
    assert_eq!(
        hub.broadcast(0, &[tlbi(VAA, 0x1234)]).len(),
        1,
        "every ASID"
    );
    assert!(inbox.lower_globals.widen(0x123_0000, 0x123_ffff));
    assert!(!inbox.lower_globals.widen(0x123_4000, 0x123_7fff));
    assert_eq!(
        hub.broadcast(0, &[tlbi(VA, ASID_5 | 0x1234)]).len(),
        1,
        "its global page"
    );
    assert!(hub.broadcast(0, &[tlbi(VA, ASID_5 | 0x4321)]).is_empty());
}

/// What a PE translates is published before its next mail read.
#[test]
fn a_pe_publishes_the_context_it_translates() {
    use super::super::tests::machine;
    let mut machine = machine(&[0xd503_201f]);
    let (hub, _) = hub(1);
    machine.enter_parallel(Arc::clone(&hub)).unwrap();
    assert_eq!(
        hub.inboxes[0].translating.load(Ordering::Relaxed),
        ANY_CONTEXT
    );
    machine.sync_host_regime(super::super::step_timing::SyncSource::Other);
    let controls = machine.cpu.controls(machine.cpu.regime());
    assert_eq!(
        hub.inboxes[0].translating.load(Ordering::Relaxed),
        context_word(controls.regime, mmu::current_asid(&controls))
    );
}

#[test]
fn duplicate_safe_attachment_refuses_without_replacing_the_live_owner() {
    use super::super::tests::{BASE, board};
    use crate::machine::NoDevices;
    use stela_contracts::identity::Identity;
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let mut first = board(&[], NoDevices, identities.clone());
    let mut second = board(&[], NoDevices, identities);
    let (hub, _) = hub(1);
    first.enter_parallel(Arc::clone(&hub)).unwrap();
    let epoch = first.attached.as_ref().unwrap().epoch;
    let untouched = second.chain.snapshot();
    assert_eq!(
        second.enter_parallel(Arc::clone(&hub)),
        Err(ParallelAdmissionError::PeOwned { pe: 0 })
    );
    assert!(second.parallel.is_none() && second.attached.is_none());
    assert_eq!(second.chain.snapshot(), untouched);
    assert_eq!(
        first.enter_parallel(Arc::clone(&hub)),
        Err(ParallelAdmissionError::MachineAttached)
    );
    first.chain.budget().store(17, Ordering::Relaxed);
    hub.interrupt(0);
    assert_eq!(first.chain.budget().load(Ordering::Relaxed), 17);
    let pending = first.chain.snapshot();
    assert_ne!(pending.interrupt_epoch, pending.observed_interrupt_epoch);
    drop(first);
    second.enter_parallel(Arc::clone(&hub)).unwrap();
    assert!(second.attached.as_ref().unwrap().epoch > epoch);
    assert_eq!(second.cpu.pc, BASE);
}

#[test]
fn an_interrupt_borrow_survives_detach_and_reuse_without_a_raw_pointer_lease() {
    use std::sync::mpsc;
    let (hub, _) = hub(1);
    let weak = Arc::downgrade(&hub.inboxes[0].chain);
    let (attached, mut control) = Attached::new(&hub, 0, false).unwrap();
    control.initialize(11, 14, [12, 13], [0xffff, 0xfff]);
    control.begin(9, 99);
    let (ready_tx, ready_rx) = mpsc::channel();
    let (resume_tx, resume_rx) = mpsc::channel();
    let foreign = Arc::clone(&hub);
    let reader = std::thread::spawn(move || {
        let epoch = foreign.inboxes[0].chain.interrupt_epoch();
        ready_tx.send(()).unwrap();
        resume_rx.recv().unwrap();
        epoch.fetch_add(1, Ordering::Release);
    });
    ready_rx.recv().unwrap();
    drop(attached);
    assert_ne!(
        hub.inboxes[0].owner.load(Ordering::Acquire),
        0,
        "the control owner retains the claim"
    );
    drop(control);
    assert_eq!(hub.inboxes[0].owner.load(Ordering::Acquire), 0);
    assert!(
        weak.upgrade().is_some(),
        "hub retains the physical allocation"
    );
    let (next, mut control) = Attached::new(&hub, 0, false).unwrap();
    control.initialize(21, 24, [22, 23], [0xffff, 0xfff]);
    control.begin(2, 77);
    resume_tx.send(()).unwrap();
    reader.join().unwrap();
    let snapshot = control.snapshot();
    assert_eq!(
        (
            snapshot.budget,
            snapshot.retired,
            snapshot.hits,
            snapshot.linked
        ),
        (77, 0, 0, 0)
    );
    assert_eq!(
        (snapshot.block, snapshot.table, snapshot.contexts),
        (2, 21, [22, 23])
    );
    assert_ne!(snapshot.interrupt_epoch, snapshot.observed_interrupt_epoch);
    assert_eq!(hub.inboxes[0].owner.load(Ordering::Acquire), next.epoch);
    drop(control);
    drop(next);
    drop(hub);
    assert!(
        weak.upgrade().is_none(),
        "last hub/owner releases the allocation"
    );
}

#[test]
fn a_populated_machine_keeps_its_code_as_it_adopts_a_hub_control() {
    use super::super::tests::{BASE, machine};
    use stela_compiler::block::ExitKind;
    let mut machine = machine(&[0x9100_0400, 0x1400_0002, 0, 0x9100_0400, 0x17ff_fffc]);
    let direct = |next_pc| super::super::NativeExit::Exit {
        next_pc,
        kind: ExitKind::Direct as u64,
    };
    machine.host.exits = vec![direct(BASE + 12), direct(BASE), direct(BASE + 12)];
    machine.step().unwrap();
    machine.step().unwrap();
    machine.step().unwrap();
    assert_eq!(machine.statistics.links, 2);
    let (blocks, installed) = (machine.code.blocks.len(), machine.host.installed.len());
    let contexts = machine.chain.contexts();
    let old_address = machine.chain.address();
    let old_storage = machine.chain.weak_storage();
    let (hub, _) = hub(1);
    machine.enter_parallel(Arc::clone(&hub)).unwrap();
    assert_ne!(machine.chain.address(), old_address);
    assert!(old_storage.upgrade().is_none());
    // Code finds the control through the frame, which the next entry sets.
    assert_eq!(machine.chain.contexts(), contexts);
    assert_eq!(
        (machine.code.blocks.len(), machine.host.installed.len()),
        (blocks, installed)
    );
    assert_eq!(
        machine.anchors()[stela_compiler::Anchor::Control],
        machine.chain.address()
    );
}

#[test]
fn repeated_owner_reuse_keeps_remote_epoch_writes_disjoint() {
    use core::sync::atomic::AtomicBool;
    use std::sync::Barrier;
    let (hub, _) = hub(1);
    let started = Arc::new(Barrier::new(2));
    let done = Arc::new(AtomicBool::new(false));
    let foreign = Arc::clone(&hub);
    let worker_started = Arc::clone(&started);
    let worker_done = Arc::clone(&done);
    let interrupt = std::thread::spawn(move || {
        worker_started.wait();
        while !worker_done.load(Ordering::Acquire) {
            foreign.interrupt(0);
        }
    });
    started.wait();
    let mut last = 0;
    for index in 0..1000 {
        let (attached, mut control) = Attached::new(&hub, 0, false).unwrap();
        assert!(attached.epoch > last);
        last = attached.epoch;
        control.initialize(index, index + 3, [index + 1, index + 2], [0xffff, 0xfff]);
        control.begin(index as usize, 31);
        let state = control.snapshot();
        assert_eq!(
            (state.block, state.table, state.itlb, state.contexts),
            (index, index, index + 3, [index + 1, index + 2])
        );
        drop(attached);
        drop(control);
    }
    done.store(true, Ordering::Release);
    interrupt.join().unwrap();
    assert_eq!(hub.inboxes[0].owner.load(Ordering::Acquire), 0);
}

#[test]
fn parallel_pes_refuse_independent_idle_leaps() {
    use super::super::tests::{BASE, board};
    use {crate::machine::engine::NoDevices, stela_contracts::identity::Identity};

    let identities = (0..2)
        .map(|pe| Identity {
            midr: 0x611f_0320,
            mpidr: 0x8000_0000 | pe << 8,
            clock: None,
            caches: None,
        })
        .collect::<Vec<_>>();
    let mut home = board(&[], NoDevices, identities.clone());
    let prepared = board(&[], NoDevices, identities);
    let mut worker = home
        .worker_profile()
        .start(prepared.cpu, prepared.host, prepared.devices, 1)
        .unwrap();
    assert!(Arc::ptr_eq(&home.admit_table, &worker.admit_table));
    home.set_idle_skip(true).unwrap();
    let (hub, _) = hub(2);
    home.enter_parallel(Arc::clone(&hub)).unwrap();
    worker
        .start_parallel(hub, home.cpu.core_locks.clone(), BASE)
        .unwrap();
    for machine in [&mut home, &mut worker] {
        assert!(!machine.idle_skip);
        assert_eq!(
            machine.set_idle_skip(true),
            Err("idle skipping requires all PEs to share one scheduler")
        );
        assert!(!machine.idle_skip);
        machine.set_idle_skip(false).unwrap();
        machine.host.now = 1000;
    }
    assert_eq!(home.now(), worker.now());
    home.host.now = 2000;
    worker.host.now = 2000;
    assert_eq!(home.now(), worker.now());
}

#[test]
fn edited_cpu_profile_keeps_its_own_admission_policy() {
    use super::super::tests::machine_on;
    use crate::machine::{identification, registers::encodings::ID_AA64ISAR1_EL1};
    use stela_contracts::Chip;

    const SMMLA: u32 = 0x4e9b_a781;
    let chip = Chip::AppleM2;
    let mut home = machine_on(chip, &[]);
    assert_eq!(home.admit_table.admit(SMMLA), SMMLA);
    home.profile.fixed =
        identification::derive(chip, home.target, true, identification::Speculation::NONE);
    for (register, value) in &mut home.profile.fixed {
        if *register == ID_AA64ISAR1_EL1 {
            *value &= !(0xf << 52);
        }
    }
    let worker = home.worker_profile();
    assert!(!Arc::ptr_eq(&home.admit_table, &worker.admit_table));
    assert_eq!(worker.admit_table.admit(SMMLA), 0);
}

#[test]
fn source_observation_uses_only_the_attached_owner_and_remains_stale() {
    use super::super::tests::{BASE, board};
    use crate::machine::engine::NoDevices;
    use stela_contracts::identity::Identity;

    let identities = (0..2)
        .map(|pe| Identity {
            midr: 0x611f_0320,
            mpidr: 0x8000_0000 | pe << 8,
            clock: None,
            caches: None,
        })
        .collect::<Vec<_>>();
    let mut home = board(&[], NoDevices, identities.clone());
    let prepared = board(&[], NoDevices, identities);
    let mut worker = Machine::new_at(
        prepared.cpu,
        prepared.host,
        prepared.devices,
        prepared.profile,
        prepared.target,
        1,
    )
    .unwrap();
    let (hub, kicks) = hub(2);
    home.enter_parallel(Arc::clone(&hub)).unwrap();
    worker
        .start_parallel(Arc::clone(&hub), home.cpu.core_locks.clone(), BASE)
        .unwrap();
    let reads = worker.host.counter_reads;
    worker.observe_parallel_source(SourcePhase::SliceEnter);
    assert_eq!(
        worker.host.counter_reads, reads,
        "disabled path reads no counter"
    );
    assert_eq!(hub.source_observation(1), None, "default off");
    hub.set_source_observations(true);
    worker.cpu.pc = BASE + 4;
    worker.cpu.state.sp = 0x8000;
    worker.cpu.state.x[29] = 0x8010;
    worker.cpu.state.x[30] = BASE + 8;
    worker.cpu.pstate.daif = 3;
    worker.host.now = 1000;
    let before = worker.cpu.state.clone();
    let counts = worker.counts();
    let sent = kicks.load(Ordering::Relaxed);
    worker.observe_parallel_source(SourcePhase::BroadcastEnter);
    let first = hub.source_observation(1).unwrap();
    assert_eq!(first.sequence, 2);
    assert_eq!(
        first.source,
        MaterializedSource {
            host_counter: 1000,
            phase: SourcePhase::BroadcastEnter,
            pc: BASE + 4,
            sp: 0x8000,
            fp: 0x8010,
            lr: BASE + 8,
            el: 2,
            daif: 3,
        }
    );
    assert_eq!(worker.cpu.state, before);
    assert_eq!(worker.counts(), counts);
    assert_eq!(kicks.load(Ordering::Relaxed), sent);
    assert_eq!(hub.source_observation(0), None);
    assert_eq!(hub.source_observation(2), None);
    assert_eq!(hub.source_observation(usize::MAX), None);

    // A caller cannot select another slot: attachment is private and
    // publication refuses a mismatched current-PE image.
    worker.current = 0;
    worker.observe_parallel_source(SourcePhase::SliceExit);
    assert_eq!(hub.source_observation(0), None);
    assert_eq!(hub.source_observation(1), Some(first));
    worker.current = 1;
    worker.cpu.pc = BASE + 16;
    worker.host.now = 2000;
    assert_eq!(
        hub.source_observation(1),
        Some(first),
        "no implicit refresh"
    );
    worker.observe_parallel_source(SourcePhase::SliceExit);
    let second = hub.source_observation(1).unwrap();
    assert_eq!(second.sequence, 4);
    assert_eq!(second.source.pc, BASE + 16);
    assert_eq!(second.source.host_counter, 2000);
    hub.set_source_observations(false);
    worker.observe_parallel_source(SourcePhase::ParkEnter);
    assert_eq!(hub.source_observation(1), None);
    hub.set_source_observations(true);
    assert_eq!(hub.source_observation(1), Some(second));

    // Real boundary hooks publish once at entry and once on return, not
    // on each wait-loop spin. A PE that stops reading mail needs not
    // acknowledge what was posted to it.
    home.post(Maintenance::Code(None));
    home.send_outgoing();
    hub.set_live(1, false);
    home.complete_maintenance().unwrap();
    let broadcast = hub.source_observation(0).unwrap();
    assert_eq!(broadcast.sequence, 4);
    assert_eq!(broadcast.source.phase, SourcePhase::BroadcastExit);
    home.device_transfer(0).unwrap();
    let dma = hub.source_observation(0).unwrap();
    assert_eq!(dma.sequence, 8);
    assert_eq!(dma.source.phase, SourcePhase::DeviceAdvanceExit);
    assert_eq!(hub.source_observation(1), Some(second));

    drop(worker);
    assert_eq!(hub.source_observation(1), None, "detached owner");
    let (_new_owner, _control) = Attached::new(&hub, 1, false).unwrap();
    assert_eq!(hub.source_observation(1), None, "replacement has no copy");
}

/// A machine of PE 0 of two in `hub`, running `words`.
fn first_of_two(
    words: &[u32],
    hub: &Arc<Hub>,
) -> Machine<super::super::tests::FakeHost, crate::machine::NoDevices> {
    use stela_contracts::identity::Identity;
    let identities = (0..2)
        .map(|pe| Identity {
            midr: 0x611f_0320,
            mpidr: 0x8000_0000 | pe << 8,
            clock: None,
            caches: None,
        })
        .collect::<Vec<_>>();
    let mut machine = super::super::tests::board(words, crate::machine::NoDevices, identities);
    machine.enter_parallel(Arc::clone(hub)).unwrap();
    machine
}

#[test]
fn broadcast_profile_counts_posts_and_completion_waits_for_the_attached_owner() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[0xd503_201f], &hub);
    // Unobserved, a post reads no clock; the completion reads its two.
    let reads = machine.host.counter_reads;
    machine.post(Maintenance::Code(None));
    assert_eq!(machine.host.counter_reads, reads);
    machine.send_outgoing();
    hub.set_live(1, false);
    machine.complete_maintenance().unwrap();
    assert_eq!(machine.host.counter_reads - reads, 2);
    assert!(hub.broadcast_profile(0).is_none());

    hub.set_source_observations(true);
    hub.set_live(1, true);
    let reads = machine.host.counter_reads;
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.send_outgoing();
    hub.set_live(1, false);
    machine.complete_maintenance().unwrap();
    // A clock for the posts, and the completion's two and its two source
    // observations; accounting adds no native notification.
    assert_eq!(machine.host.counter_reads - reads, 5);
    let observed = hub.broadcast_profile(0).unwrap();
    let code = observed.kinds[BroadcastKind::Code.index()];
    assert_eq!(
        (code.started, code.completed, code.recipients, code.polls),
        (2, 2, 2, 0)
    );
    let completion = observed.kinds[BroadcastKind::Completion.index()];
    assert_eq!(
        (
            completion.started,
            completion.completed,
            completion.recipients
        ),
        (1, 1, 1)
    );
    assert_eq!((observed.active, observed.dropped_updates), (None, false));
    // PE 1 posted nothing.
    let idle = hub.broadcast_profile(1).unwrap();
    assert!(idle.kinds.iter().all(|counts| counts.started == 0));

    // With nothing posted, a completion waits for nothing.
    let reads = machine.host.counter_reads;
    machine.complete_maintenance().unwrap();
    assert_eq!(machine.host.counter_reads, reads);

    hub.set_live(1, true);
    let attached = machine.attached.take().unwrap();
    machine.post(Maintenance::Code(None));
    machine.send_outgoing();
    hub.set_live(1, false);
    machine.complete_maintenance().unwrap();
    assert_eq!(hub.broadcast_profile(0), Some(observed));
    machine.attached = Some(attached);
}

/// A PE waiting for others' acknowledgements answers the host at each turn
/// of the wait: a processor that waits on this one, as a TLB shootdown
/// does, gets its answer, so the two waits cannot deadlock.
#[test]
fn a_completion_wait_answers_the_host_each_turn() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.send_outgoing();
    let other = Arc::clone(&hub);
    // PE 1 acknowledges only once this one has relaxed three times.
    machine.host.on_relax = Some(Box::new(move |turns| {
        if turns == 3 {
            let mail = other.take(1).expect("posted mail");
            other.acknowledge(1, mail.serial);
        }
    }));
    machine.complete_maintenance().unwrap();
    assert_eq!(machine.host.relaxes, 3);
}

/// A wait for a PE that does not apply posted maintenance past its bound
/// is reported once, naming that PE and where it is, and goes on: it ends
/// normally once the PE applies the maintenance.
#[test]
fn a_completion_wait_past_its_bound_is_reported_once_and_goes_on() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.send_outgoing();
    let limit = machine.profile.counter_frequency / super::MAINTENANCE_WAIT_DIVISOR;
    // Each turn reads the counter a step later: the bound passes at about
    // the eighth turn, and PE 1 applies at the 40th.
    machine.host.counter_step = limit / 8;
    let other = Arc::clone(&hub);
    machine.host.on_relax = Some(Box::new(move |turns| {
        if turns == 40 {
            let mail = other.take(1).expect("posted mail");
            other.acknowledge(1, mail.serial);
        }
    }));
    let degraded = machine.statistics.degraded;
    machine.complete_maintenance().unwrap();
    assert_eq!(machine.host.relaxes, 40, "waited until PE 1 applied");
    assert!(machine.posted.is_empty());
    assert_eq!(machine.statistics.degraded, degraded + 1);
    let reports = machine.take_unmodeled();
    assert_eq!(reports.len(), 1, "once: {reports:?}");
    let RequirementKind::MaintenanceOverrun {
        pe,
        serial,
        unapplied,
        state,
        ticks,
    } = reports[0].kind
    else {
        panic!("expected a maintenance overrun, got {:?}", reports[0].kind);
    };
    assert_eq!(
        (pe, serial, unapplied, state),
        (1, 1, 1, super::super::PeState::Running)
    );
    assert!((limit..2 * limit).contains(&ticks), "{ticks} of {limit}");
    assert!(
        reports[0]
            .to_string()
            .contains("maintenance overrun: PE 1 (running)")
    );

    // A wait within its bound reports nothing.
    machine.host.on_relax = None;
    machine.host.counter_step = 0;
    machine.post(Maintenance::Code(Some(0x8000)));
    machine.send_outgoing();
    let mail = hub.take(1).expect("posted mail");
    hub.acknowledge(1, mail.serial);
    machine.complete_maintenance().unwrap();
    assert!(machine.take_unmodeled().is_empty());
}

#[test]
fn maintenance_waits_for_the_dsb_and_reaches_a_pe_in_one_piece_of_mail() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.post(Maintenance::Code(Some(0x8000)));
    assert!(hub.take(1).is_none());

    machine.send_outgoing();
    hub.set_live(1, false);
    machine.complete_maintenance().unwrap();
    let mail = hub.take(1).unwrap();
    assert_eq!(mail.maintenance.len(), 2);
    assert_eq!(mail.serial, 1);
}

#[test]
fn a_broadcast_passes_by_a_pe_outside_its_run() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    // PE 0 runs a block.
    assert!(hub.enter_active(0));
    let posted = hub.broadcast(1, &[Maintenance::Code(None)]);
    assert!(!hub.applied(&mut posted.clone()));
    machine.outside_run(|_| assert!(hub.applied(&mut posted.clone())));
    assert!(!hub.applied(&mut posted.clone()));
    hub.leave_active(0);
}

#[test]
fn a_full_list_of_maintenance_goes_out_without_a_dsb() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    for page in 0..OUTGOING as u64 {
        machine.post(Maintenance::Code(Some(page << 14)));
    }
    assert_eq!(hub.take(1).unwrap().maintenance.len(), OUTGOING);
}

#[test]
fn a_page_invalidation_the_pe_has_yet_to_read_covers_the_same_page_again() {
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.post(Maintenance::Code(Some(0x8000)));
    machine.post(Maintenance::Code(None));
    machine.post(Maintenance::Code(Some(0xc000)));
    machine.send_outgoing();
    let mail = hub.take(1).unwrap();
    let pages: Vec<_> = mail
        .maintenance
        .iter()
        .map(|maintenance| match maintenance {
            Maintenance::Code(page) => *page,
            _ => unreachable!(),
        })
        .collect();
    assert_eq!(pages, [Some(0x4000), Some(0x8000), None]);
    // The PE's acknowledgment of this mail is the one waited for.
    assert_eq!((mail.serial, machine.posted.as_slice()), (1, &[(1, 1)][..]));
}

/// A PE that reads no mail for a long time, asleep or powered down, holds
/// at most [`MAIL_TLB_SCOPES`] TLB scopes and [`MAIL_SCOPES`] code pages:
/// past them, scopes that merge with none of the last kept ones and code
/// pages make one invalidation of everything of that kind.
#[test]
fn mail_for_a_pe_that_does_not_read_it_stays_bounded() {
    use crate::machine::system::TlbiKind::ASID;
    let (hub, _) = hub(2);
    for page in 0..10_000u64 {
        hub.broadcast(
            0,
            &[
                tlbi(ASID, (page % 200) << 48),
                Maintenance::Code(Some(page << 14)),
            ],
        );
    }
    let mail = hub.take(1).unwrap();
    let count =
        |kind: fn(&Maintenance) -> bool| mail.maintenance.iter().filter(|m| kind(m)).count();
    assert!(
        mail.maintenance.len() <= MAIL_TLB_SCOPES + MAIL_SCOPES + 2,
        "{}",
        mail.maintenance.len()
    );
    assert_eq!(count(|m| matches!(m, Maintenance::TlbAll)), 1);
    assert_eq!(count(|m| matches!(m, Maintenance::Code(None))), 1);
    // What follows a whole invalidation is covered by it.
    assert_eq!(count(|m| matches!(m, Maintenance::Tlb(_))), 0);
    assert_eq!(count(|m| matches!(m, Maintenance::Code(Some(_)))), 0);
}

/// A run of TLBIs by page, as XNU's and SPTM's loops send them, merges
/// into one range a sleeping PE applies at once: no invalidation of
/// everything, and the kernel's contexts outlive it.
/// A PE counts how long its TLB mail waited, from the first post it had
/// not read to the read, and how long its native run, when one was going,
/// took of that to return; then the apply. Later posts of the same mail
/// keep the first's time.
#[test]
fn a_pe_times_its_tlb_mail_from_the_first_post_to_the_read() {
    use super::super::reports::MailWait;
    use crate::machine::system::TlbiKind::VAA;
    let clock = Arc::new(AtomicUsize::new(1));
    let hub = Hub::new(2, Ticks(Arc::clone(&clock)));
    for pe in 0..2 {
        hub.set_live(pe, true);
    }
    // NOP.
    let mut machine = first_of_two(&[0xd503_201f], &hub);
    let waits = |machine: &Machine<_, _>| machine.statistics.mail_waits;
    // Mail read between runs: all of its wait is the engine's.
    hub.broadcast(1, &[tlbi(VAA, 0x10_0000)]);
    hub.broadcast(1, &[tlbi(VAA, 0x10_0001)]);
    machine.read_mail().unwrap();
    let counted = waits(&machine);
    assert_eq!(counted[MailWait::Count as usize], 1);
    assert!(counted[MailWait::Latency as usize] >= 2);
    assert_eq!(counted[MailWait::Native as usize], 0);
    assert_eq!(
        counted[MailWait::Exit as usize],
        counted[MailWait::Latency as usize]
    );
    assert!(counted[MailWait::Apply as usize] >= 1);
    assert_eq!(
        counted[MailWait::Longest as usize],
        counted[MailWait::Latency as usize]
    );
    // Mail posted while a native run goes on: its return counts apart.
    let pc = machine.cpu.pc;
    let Ok(super::super::Fetched::Block(index)) = machine.fetch_block(pc, false) else {
        panic!("a block");
    };
    hub.broadcast(1, &[tlbi(VAA, 0x20_0000)]);
    machine.host.exits = vec![super::super::NativeExit::Exit {
        next_pc: pc + 4,
        kind: stela_compiler::block::ExitKind::Fallthrough as u64,
    }];
    let budget = machine.chain_budget();
    machine.execute(index, budget).unwrap();
    machine.read_mail().unwrap();
    let after = waits(&machine);
    assert_eq!(after[MailWait::Count as usize], 2);
    let native = after[MailWait::Native as usize];
    assert!(native >= 1);
    assert!(native < after[MailWait::Latency as usize] - counted[MailWait::Latency as usize]);
    // Nothing waits after the read.
    machine.read_mail().unwrap();
    assert_eq!(waits(&machine), after);
}

#[test]
fn a_run_of_tlbis_by_page_merges_into_one_range() {
    use crate::machine::system::TlbiKind::{VA, VAA};
    let (hub, _) = hub(2);
    for page in 0..10_000u64 {
        hub.broadcast(0, &[tlbi(VAA, 0x10_0000 + page)]);
    }
    // Pages of one ASID in the upper half, one apart: another range.
    for page in 0..100u64 {
        hub.broadcast(0, &[tlbi(VA, 7 << 48 | (0xfff_ff00_0000 + 2 * page))]);
    }
    let mail = hub.take(1).unwrap();
    assert_eq!(mail.merged, 10_000 - 1 + 100 - 1);
    let [Maintenance::Tlb(lower), Maintenance::Tlb(upper)] = mail.maintenance[..] else {
        panic!("two ranges: {:?}", mail.maintenance);
    };
    assert_eq!(
        lower.span(),
        Some((0x1_0000_0000, 0x1_0000_0000 + 9_999 * 0x1000))
    );
    assert_eq!(lower.named_asid(), None);
    let upper_first = mmu::tlbi::canonical(0xfff_ff00_0000 << 12);
    assert_eq!(
        upper.span(),
        Some((upper_first, upper_first + 198 * 0x1000))
    );
    assert_eq!(upper.named_asid(), Some(7));
}

/// Pages of two processes at the same VAs (the shared cache's) stay two
/// ranges, one per ASID, rather than one of every ASID that would drop
/// those VAs in every process's lookup contexts; a page of every ASID
/// merges with neither. A full mailbox may merge them all the same.
#[test]
fn mail_merges_tlbis_of_different_asids_only_when_full() {
    use crate::machine::system::TlbiKind::{VA, VAA};
    let (first, _) = hub(2);
    for page in 0..8u64 {
        for asid in [7u64, 8] {
            first.broadcast(0, &[tlbi(VA, asid << 48 | (0x18_0000 + page))]);
        }
    }
    first.broadcast(0, &[tlbi(VAA, 0x18_0004)]);
    let mail = first.take(1).unwrap();
    let scopes: Vec<_> = mail
        .maintenance
        .iter()
        .map(|posted| match posted {
            Maintenance::Tlb(scope) => (scope.named_asid(), scope.span()),
            other => panic!("{other:?}"),
        })
        .collect();
    let range = Some((0x1_8000_0000, 0x1_8000_0000 + 7 * 0x1000));
    assert_eq!(
        scopes,
        [
            (Some(7), range),
            (Some(8), range),
            (None, Some((0x1_8000_4000, 0x1_8000_4000)))
        ]
    );
    assert_eq!(mail.merged, 2 * 7);
    // Past the kept scopes, any of the last ones takes the next.
    let (hub, _) = hub(2);
    for page in 0..super::MAIL_TLB_SCOPES as u64 {
        hub.broadcast(0, &[tlbi(VAA, 0x10_0000 + 32 * page)]);
    }
    hub.broadcast(0, &[tlbi(VA, 7 << 48 | 0x10_0001)]);
    let mail = hub.take(1).unwrap();
    assert_eq!(mail.maintenance.len(), super::MAIL_TLB_SCOPES);
}

/// Whatever TLBIs a PE's mail keeps or merges, every entry one of them
/// removes, the kept scopes remove: by [`tlbi::Scope::covers`], and by
/// what the PE's ITLB and shadow apply ([`tlbi::Scope::everywhere`],
/// [`tlbi::Scope::reach`], [`tlbi::Scope::named_asid`]).
#[test]
fn merged_mail_removes_every_entry_its_tlbis_remove() {
    use crate::machine::system::TlbiKind::{RVA, RVAA, VA, VAA};
    let mut seed = 0x1234_5678_9abc_def0u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    for round in 0..40 {
        let mut mail = Mail::default();
        let mut sent = Vec::new();
        let count = [20, 300, 2000][round % 3];
        for _ in 0..count {
            let draw = next();
            let asid = (draw >> 8) % 4;
            // Pages of four places in each half, 4 KiB apart.
            let place = (draw >> 16) % 4;
            let upper = if draw >> 20 & 1 != 0 {
                0xf_ffff_0000_0000
            } else {
                0
            };
            let page = upper | place << 24 | ((draw >> 24) % 64);
            let kind = [VA, VAA, RVA, RVAA][(draw % 4) as usize];
            let operand = match kind {
                // TG 4 KiB, SCALE 0, NUM 1: four pages from BaseADDR.
                RVA | RVAA => {
                    let base = (canonical_page(page) >> 12) & ((1 << 37) - 1);
                    asid << 48 | 1 << 46 | 1 << 39 | base
                }
                _ => asid << 48 | page & ((1 << 44) - 1),
            };
            let Maintenance::Tlb(scope) = tlbi(kind, operand) else {
                unreachable!()
            };
            sent.push(scope);
            mail.keep(Maintenance::Tlb(scope));
        }
        let all = mail
            .maintenance
            .iter()
            .any(|kept| matches!(kept, Maintenance::TlbAll));
        let kept: Vec<tlbi::Scope> = mail
            .maintenance
            .iter()
            .filter_map(|kept| match kept {
                Maintenance::Tlb(scope) => Some(*scope),
                _ => None,
            })
            .collect();
        assert!(kept.len() <= MAIL_TLB_SCOPES);
        assert_eq!(kept.len() + mail.merged as usize, sent.len(), "{all}");
        // Entries at and around every sent page, of every size and ASID.
        for scope in &sent {
            let (first, last) = scope.span().unwrap();
            for address in [first, last, first.wrapping_sub(0x1000), last + 0x1000] {
                for (size_bits, granule_bits, level) in [(12, 12, 3), (14, 14, 3), (21, 12, 2)] {
                    for asid in [None, Some(0), Some(1), Some(3)] {
                        let entry = tlbi::Entry {
                            regime: mmu::Regime::El10,
                            vmid: None,
                            asid,
                            address: address & ((1 << 56) - 1),
                            size_bits,
                            granule_bits,
                            level,
                            stage1: true,
                            stage2: false,
                            table: false,
                        };
                        if !scope.covers(&entry) {
                            continue;
                        }
                        assert!(
                            all || kept.iter().any(|kept| kept.covers(&entry)),
                            "{scope:?} removes {entry:x?}, no kept scope does"
                        );
                    }
                }
            }
            // What the ITLB and shadow apply: a span holding the sent one,
            // of its ASID or of every ASID.
            let holds = |kept: &tlbi::Scope| {
                let (kept_first, kept_last) = kept.span().unwrap();
                kept_first <= first
                    && last <= kept_last
                    && kept
                        .named_asid()
                        .is_none_or(|asid| scope.named_asid() == Some(asid))
            };
            assert!(all || kept.iter().any(holds), "{scope:?}");
        }
    }
}

/// The canonical VA of 4 KiB page number `page`.
fn canonical_page(page: u64) -> u64 {
    mmu::tlbi::canonical(page << 12)
}

#[test]
fn an_ic_ivau_reaches_other_pes_only_for_compiled_pages_once_per_dsb() {
    const PAGE: u64 = 0x4000_0000;
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    // No PE compiled from it: the IC IVAU of each line stops here.
    machine.post_code_page(PAGE);
    assert!(machine.outgoing.is_empty());
    // The next lines of a page add nothing to its first one.
    hub.compiling(PAGE + 0x100);
    machine.post_code_page(PAGE + 0x40);
    machine.post_code_page(PAGE + 0x80);
    assert_eq!(machine.outgoing.len(), 1);
    // A DSB ends the run: what follows may have been written since.
    machine.send_outgoing();
    let mail = hub.take(1).unwrap();
    hub.acknowledge(1, mail.serial);
    machine.complete_maintenance().unwrap();
    machine.post_code_page(PAGE + 0x40);
    assert_eq!(machine.outgoing.len(), 1);
}

#[test]
fn maintenance_posted_to_another_pe_completes_at_the_next_dsb() {
    let (hub, _) = hub(2);
    // DSB ISH.
    let mut machine = first_of_two(&[0xd503_3b9f], &hub);
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.send_outgoing();
    assert_eq!(machine.posted, [(1, 1)]);
    // PE 1 reads its mail at its next return to the engine.
    let mail = hub.take(1).unwrap();
    hub.acknowledge(1, mail.serial);
    machine.host.exits = vec![super::super::NativeExit::Exit {
        next_pc: super::super::tests::BASE,
        kind: stela_compiler::block::ExitKind::System as u64,
    }];
    machine.step().unwrap();
    assert!(machine.posted.is_empty());
    assert_eq!(machine.cpu.pc, super::super::tests::BASE + 4);
    // A PE outside its run loop reads its mail before it runs again: the
    // completion does not wait for it.
    machine.post(Maintenance::Code(None));
    hub.set_outside(1, true);
    machine.complete_maintenance().unwrap();
    assert!(machine.posted.is_empty());
    assert!(hub.take(1).is_some());
}

#[test]
fn notification_after_drain_survives_native_budget_reset() {
    use super::super::tests::machine;
    let mut machine = machine(&[0xd503_201f]);
    let (hub, _) = hub(1);
    machine.enter_parallel(Arc::clone(&hub)).unwrap();
    hub.post(0, |mail| mail.event = true);
    machine.read_mail().unwrap();
    let drained = machine.chain.snapshot();
    assert_eq!(drained.interrupt_epoch, drained.observed_interrupt_epoch);
    // This is the old lost-notification window: the owner has drained,
    // then a peer posts before the owner initializes its next budget.
    hub.post(0, |mail| mail.event = true);
    machine.chain.begin(0, 4096);
    let entered = machine.chain.snapshot();
    assert_eq!(entered.budget, 4096);
    assert_ne!(entered.interrupt_epoch, entered.observed_interrupt_epoch);
    assert_eq!(entered.observed_interrupt_epoch, drained.interrupt_epoch);
    machine.read_mail().unwrap();
    let next = machine.chain.snapshot();
    assert_eq!(next.interrupt_epoch, next.observed_interrupt_epoch);
}

#[test]
fn epoch_exhaustion_is_sticky_across_drain_detach_and_reuse() {
    use super::super::tests::machine;
    let mut machine = machine(&[0xd503_201f]);
    let (hub, _) = hub(1);
    machine.enter_parallel(Arc::clone(&hub)).unwrap();
    hub.inboxes[0]
        .chain
        .interrupt_epoch()
        .store(u64::MAX - 1, Ordering::Relaxed);
    hub.post(0, |mail| mail.event = true);
    machine.read_mail().unwrap();
    hub.interrupt(0);
    let exhausted = machine.chain.snapshot();
    assert_eq!(exhausted.interrupt_epoch, u64::MAX);
    assert_eq!(exhausted.observed_interrupt_epoch, u64::MAX - 1);
    drop(machine);
    let (_, mut next) = Attached::new(&hub, 0, false).unwrap();
    next.initialize(11, 14, [12, 13], [0xffff, 0xfff]);
    next.acknowledge_interrupt(u64::MAX);
    next.begin(0, 4096);
    let reused = next.snapshot();
    assert_eq!(reused.interrupt_epoch, u64::MAX);
    assert_eq!(reused.observed_interrupt_epoch, u64::MAX - 1);
}

#[test]
fn mail_reaches_running_pes_and_kicks_only_a_sleeping_one() {
    let (hub, kicks) = hub(3);
    hub.event(0);
    assert_eq!(kicks.load(Ordering::Relaxed), 0, "no PE sleeps");
    assert!(hub.take(0).is_none(), "SEV reaches the others");
    assert!(hub.take(1).is_some_and(|mail| mail.event));
    // A PE that sleeps halts only without mail, and mail kicks it.
    let mut halts = 0;
    hub.sleep(1, hub.lines_epoch(1), true, || halts += 1);
    assert_eq!(halts, 1);
    hub.sleep(2, hub.lines_epoch(2), true, || halts += 1);
    assert_eq!(halts, 1, "PE 2 has mail");
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    hub.start(1, 0x8000);
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
}

#[test]
fn a_broadcast_is_applied_once_every_running_pe_acknowledged_it() {
    let (hub, kicks) = hub(3);
    let (attached, control) = Attached::new(&hub, 1, false).unwrap();
    control.budget().store(4096, Ordering::Relaxed);
    let mut posted = hub.broadcast(0, &[Maintenance::Code(None)]);
    assert_eq!(posted, [(1, 1), (2, 1)]);
    assert!(!hub.applied(&mut posted));
    // The running PE leaves native code at its next chained transfer.
    drop(attached);
    assert_eq!(control.budget().load(Ordering::Relaxed), 4096);
    let interrupted = control.snapshot();
    assert_ne!(
        interrupted.interrupt_epoch,
        interrupted.observed_interrupt_epoch
    );
    let mail = hub.take(1).unwrap();
    assert_eq!((mail.maintenance.len(), mail.serial), (1, 1));
    hub.acknowledge(1, mail.serial);
    assert!(!hub.applied(&mut posted));
    assert_eq!(posted, [(2, 1)]);
    // A PE that sleeps is neither woken nor waited for: it reads its
    // mail before it runs again.
    hub.inboxes[2].sleeping.store(true, Ordering::SeqCst);
    assert!(hub.applied(&mut posted));
    assert_eq!(kicks.load(Ordering::Relaxed), 0);
    hub.inboxes[2].sleeping.store(false, Ordering::SeqCst);
    assert!(hub.take(2).is_some_and(|mail| mail.serial == 1));
    // A PE that no longer reads mail holds no broadcast up.
    let mut posted = hub.broadcast(0, &[Maintenance::Code(None)]);
    hub.set_live(2, false);
    hub.acknowledge(1, 2);
    assert!(hub.applied(&mut posted));
    // Serials keep counting after a take.
    assert_eq!(hub.broadcast(0, &[Maintenance::Code(None)]), [(1, 3)]);
}

#[test]
fn a_host_call_ends_a_run_and_wakes_a_sleeping_pe() {
    let (hub, kicks) = hub(2);
    let (attached, control) = Attached::new(&hub, 1, false).unwrap();
    hub.call_host(1);
    let interrupted = control.snapshot();
    assert_ne!(
        interrupted.interrupt_epoch,
        interrupted.observed_interrupt_epoch
    );
    assert_eq!(kicks.load(Ordering::Relaxed), 0);
    assert!(hub.take_host_call(1));
    assert!(!hub.take_host_call(1), "taken once");
    drop(attached);
    // A sleep does not start over a call, and a sleeper is woken for one.
    hub.call_host(1);
    let mut halted = false;
    hub.sleep(1, 0, true, || halted = true);
    assert!(!halted);
    assert!(hub.take_host_call(1));
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    hub.call_host(1);
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    hub.inboxes[1].sleeping.store(false, Ordering::SeqCst);
}

/// A line change publishes the receiver's pins before it moves the
/// receiver's epoch, so a running receiver reads them at once; a sleeping
/// one is kicked only after the producer releases the board lock, which no
/// other PE then waits behind.
#[test]
fn a_line_notification_publishes_pins_first_and_kicks_after_the_unlock() {
    #[derive(Default)]
    struct Latched(Lines);
    impl Devices for Latched {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Ok(0)
        }
        fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            Ok(())
        }
        fn lines(&mut self, pe: usize, _: u64) -> Lines {
            if pe == 1 { self.0 } else { Lines::default() }
        }
        fn pins(&mut self, pe: usize) -> Option<Lines> {
            Some(self.lines(pe, 0))
        }
    }
    struct ReadPins {
        worker: Arc<Mutex<Option<SharedDevices<Latched>>>>,
        observed: Arc<Mutex<Vec<Lines>>>,
    }
    impl Kick for ReadPins {
        fn kick(&self, pe: usize) {
            // The kick follows the unlock. Latched pins are lockless.
            let mut worker = self.worker.lock().unwrap();
            let worker = worker.as_mut().unwrap();
            assert!(worker.board.devices.try_lock().is_ok());
            let lines = worker.lines(pe, 0);
            self.observed.lock().unwrap().push(lines);
        }
    }
    let worker = Arc::new(Mutex::new(None));
    let observed = Arc::new(Mutex::new(Vec::new()));
    let hub = Hub::new(
        2,
        ReadPins {
            worker: Arc::clone(&worker),
            observed: Arc::clone(&observed),
        },
    );
    for pe in 0..2 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Latched::default(), Arc::clone(&hub));
    *worker.lock().unwrap() = Some(home.worker());
    let changes = [
        Lines {
            irq: true,
            fiq: false,
        },
        Lines {
            irq: false,
            fiq: true,
        },
        Lines::default(),
    ];
    for lines in changes {
        hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
        home.serve(|devices| devices.0 = lines);
    }
    worker.lock().unwrap().take();
    assert_eq!(*observed.lock().unwrap(), changes);
}

/// A write of 1 requests a restart, of 2 starts PE 1 at 0x800 and of 3
/// leaves bytes for the host; any other value sets the DMA deadline, 0
/// clearing it. Also raises PE 1's IRQ from count `irq_at`.
#[derive(Default)]
struct Pending {
    reset: Option<Reset>,
    dma: Option<u64>,
    irq_at: Option<u64>,
    starts: Vec<Start>,
    service: bool,
    register_calls: usize,
}

/// The one IMPLEMENTATION DEFINED register [`Pending`] serves.
const SERVED: Encoding = Encoding::new(3, 5, 15, 0, 1);

impl Devices for Pending {
    fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
        Ok(0)
    }
    fn write(&mut self, _: usize, _: u64, _: u8, value: u64, _: u64) -> Result<(), DeviceError> {
        match value {
            1 => {
                self.reset = Some(Reset {
                    transition: stela_contracts::bus::Transition::Restart,
                    reason: "test".into(),
                })
            }
            2 => self.starts.push(Start {
                cpu: 1,
                address: 0x800,
            }),
            3 => self.service = true,
            0 => self.dma = None,
            deadline => self.dma = Some(deadline),
        }
        Ok(())
    }
    fn lines(&mut self, pe: usize, now: u64) -> Lines {
        Lines {
            irq: pe == 1 && self.irq_at.is_some_and(|at| now >= at),
            ..Lines::default()
        }
    }
    fn deadline(&mut self, now: u64) -> Option<u64> {
        self.irq_at.filter(|&at| at > now)
    }
    fn take_reset(&mut self) -> Option<Reset> {
        self.reset.take()
    }
    fn take_start(&mut self) -> Option<Start> {
        self.starts.pop()
    }
    fn dma_deadline(&self) -> Option<u64> {
        self.dma
    }
    fn take_host_service(&mut self) -> bool {
        core::mem::take(&mut self.service)
    }
    fn system_register(
        &mut self,
        _: usize,
        encoding: Encoding,
        _: Option<u64>,
        _: u64,
    ) -> Option<Result<u64, &'static str>> {
        self.register_calls += 1;
        (encoding == SERVED).then_some(Ok(7))
    }
    fn serves_register(&self, encoding: Encoding) -> bool {
        encoding == SERVED
    }
}

/// Counts its clock's reads: each board lock reads it once when taken and
/// once when released.
struct Ticks(Arc<AtomicUsize>);
impl Kick for Ticks {
    fn kick(&self, _pe: usize) {}
    fn ticks(&self) -> u64 {
        self.0.fetch_add(1, Ordering::SeqCst) as u64
    }
}

/// Counts the host work releases of the devices' lock run
/// ([`Kick::released`]).
struct Releases(Arc<AtomicUsize>);
impl Kick for Releases {
    fn kick(&self, _pe: usize) {}
    fn released(&self) -> bool {
        self.0.fetch_add(1, Ordering::SeqCst);
        false
    }
    fn release_due(&self) -> bool {
        true
    }
}

/// The host work a release of the devices' lock leaves runs only on a PE
/// outside its run loop or asleep, where no broadcast waits for it: one
/// inside its loop, or a call that names no PE, leaves it for later, so a
/// slow host device never keeps a running PE from its mail.
#[test]
fn released_host_work_runs_only_outside_the_run_loop() {
    let released = Arc::new(AtomicUsize::new(0));
    let hub = Hub::new(2, Releases(Arc::clone(&released)));
    for pe in 0..2 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Signalling::default(), Arc::clone(&hub));
    released.store(0, Ordering::SeqCst);
    let mut worker = home.worker();

    // Inside its run loop: the access takes the lock and leaves the work.
    worker.write(1, 0, 4, 0, 0).unwrap();
    assert_eq!(released.load(Ordering::SeqCst), 0, "inside the run loop");
    // A call that names no PE leaves it too.
    home.with(|_| ());
    assert_eq!(released.load(Ordering::SeqCst), 0, "no PE named");

    // Outside its run loop, as a device site's access runs, it does it.
    hub.set_outside(1, true);
    worker.write(1, 0, 4, 0, 0).unwrap();
    assert_eq!(released.load(Ordering::SeqCst), 1, "outside");
    hub.set_outside(1, false);

    // Asleep, as a halted PE serving the host, it does it.
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    worker.write(1, 0, 4, 0, 0).unwrap();
    assert_eq!(released.load(Ordering::SeqCst), 2, "asleep");
    hub.inboxes[1].sleeping.store(false, Ordering::SeqCst);
    worker.write(1, 0, 4, 0, 0).unwrap();
    assert_eq!(released.load(Ordering::SeqCst), 2, "inside again");
}

/// What follows a write, [`Devices::settle_write`]: the lock, the DMA
/// under it, the call and the lock after, in that order, from one board
/// lock.
#[test]
fn a_shared_board_settles_a_write_under_one_lock() {
    use super::super::{LockedMemory, Memory};
    use crate::machine::mmu::MemoryLocks;
    const LOCK: MemoryLocks = [Some((0x1000, 0x1fff)), None, None, None];
    const AFTER: MemoryLocks = [Some((0x1000, 0x2fff)), None, None, None];
    #[derive(Default)]
    struct Settling {
        steps: Vec<&'static str>,
    }
    impl Devices for Settling {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Ok(0)
        }
        fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            Ok(())
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            Lines::default()
        }
        fn memory_lock(&self) -> MemoryLocks {
            if self.steps.contains(&"transfer") {
                AFTER
            } else {
                LOCK
            }
        }
        fn transfer(
            &mut self,
            cpu: usize,
            memory: &mut dyn Memory,
            _: u64,
        ) -> Result<(), DeviceError> {
            assert_eq!(cpu, 1);
            assert!(memory.write(0x800, &[1]));
            assert!(!memory.write(0x1800, &[1]), "the lock before the DMA");
            self.steps.push("transfer");
            Ok(())
        }
    }
    /// RAM below 0x4000 that refuses writes the lock covers.
    struct Locked(MemoryLocks);
    impl Memory for Locked {
        fn read(&mut self, _: u64, _: &mut [u8]) -> bool {
            true
        }
        fn write(&mut self, physical: u64, _: &[u8]) -> bool {
            !self
                .0
                .into_iter()
                .flatten()
                .any(|(first, last)| (first..=last).contains(&physical))
        }
    }
    impl LockedMemory for Locked {
        fn under(&mut self, lock: MemoryLocks) -> &mut dyn Memory {
            self.0 = lock;
            self
        }
    }
    let reads = Arc::new(AtomicUsize::new(0));
    let hub = Hub::new(2, Ticks(Arc::clone(&reads)));
    let mut worker = SharedDevices::new(Settling::default(), hub).worker();
    let before = reads.load(Ordering::SeqCst);
    let settled = worker.settle_write(1, 0, &mut Locked([None; 4]));
    assert_eq!(reads.load(Ordering::SeqCst) - before, 2, "one lock");
    assert!(settled.transfer.is_ok() && settled.call.is_none());
    assert_eq!((settled.lock, settled.after), (LOCK, AFTER));
    assert_eq!(worker.with(|devices| devices.steps.clone()), ["transfer"]);
}

/// A handle counts the ticks its calls waited for the board's lock, once.
#[test]
fn a_handle_counts_the_ticks_it_waited_for_the_lock() {
    let reads = Arc::new(AtomicUsize::new(0));
    let hub = Hub::new(2, Ticks(reads));
    let home = SharedDevices::new(Pending::default(), hub);
    let mut worker = home.worker();
    worker.write(1, 0, 4, 0, 0).unwrap();
    assert_eq!(worker.take_lock_wait(), 0, "uncontended");
    let board = Arc::clone(&home.board);
    let held = board.devices.lock().unwrap();
    let waiter = std::thread::spawn(move || {
        worker.write(1, 0, 4, 0, 0).unwrap();
        (worker.take_lock_wait(), worker.take_lock_wait())
    });
    // Bounded: the waiter blocks on the lock within five seconds.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while board.waits[Call::Access as usize].load(Ordering::Relaxed) == 0 {
        assert!(
            std::time::Instant::now() < deadline,
            "the waiter never waited"
        );
        std::thread::yield_now();
    }
    drop(held);
    let (waited, again) = waiter.join().unwrap();
    assert!(waited > 0);
    assert_eq!(again, 0);
}

#[test]
fn only_the_registers_the_devices_serve_reach_them() {
    let (hub, _) = hub(2);
    let mut worker = SharedDevices::new(Pending::default(), hub).worker();
    assert_eq!(worker.system_register(1, SERVED, None, 0), Some(Ok(7)));
    let other = Encoding::new(3, 6, 15, 1, 0);
    assert_eq!(worker.system_register(1, other, Some(1), 0), None);
    assert!(worker.serves_register(SERVED) && !worker.serves_register(other));
    assert_eq!(worker.with(|devices| devices.register_calls), 1);
}

/// A fast-IPI write (IPI_RR_GLOBAL_EL1) moves only device lines: the
/// step reads them again, and nothing derived from the PE's registers goes.
#[test]
fn a_fast_ipi_write_keeps_what_derives_from_the_pes_registers() {
    use super::super::tests::board;
    use stela_contracts::identity::Identity;
    const MSR: u32 = 0xd51d_f029; // msr s3_5_c15_c0_1, x9
    let (hub, _) = hub(2);
    let worker = SharedDevices::new(Pending::default(), hub).worker();
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let mut machine = board(&[MSR], worker, identities);
    machine.native_system();
    assert!(machine.native_system.is_some());
    machine.device_lines = Some((Lines::default(), u64::MAX, machine.current));
    let pc = machine.cpu.pc;
    machine.cpu.state.x[9] = 1 << 16;
    machine.system_instruction(MSR).unwrap();
    assert_eq!(machine.cpu.pc, pc + 4);
    assert_eq!(machine.devices.with(|devices| devices.register_calls), 1);
    assert_eq!(machine.device_lines, None, "lines are read again");
    assert!(machine.native_system.is_some(), "derived state stays");
}

/// A board whose PEs signal each other with a word each, as Apple's fast
/// IPIs do: a write of `SERVED` sets the word of the PE its value names
/// through the PE's own handle, without the lock, unless the value has bit
/// 8 set, which takes the lock and raises it there.
#[derive(Default)]
struct Signalling {
    words: Arc<[AtomicU64; 2]>,
    /// Each CPU's deferred-request flag, which only native completion reads.
    armed: Arc<[AtomicBool; 2]>,
    signalled: u64,
    register_calls: usize,
    /// The SoC sleeps ([`Devices::waits_on_host`]).
    asleep: bool,
}

struct SignalWords {
    words: Arc<[AtomicU64; 2]>,
    pe: usize,
}

impl LocalRegisters for SignalWords {
    fn register(&self, _: Encoding, _: Option<u64>) -> Option<u64> {
        None
    }
    fn storage(&self, _: Encoding) -> Option<u64> {
        None
    }
    fn send(&self, encoding: Encoding, value: u64) -> Option<Option<usize>> {
        if encoding != SERVED || value & 1 << 8 != 0 {
            return None;
        }
        let target = (value & 1) as usize;
        Some((self.words[target].swap(1, Ordering::AcqRel) == 0).then_some(target))
    }
    fn fiq(&self) -> bool {
        self.words[self.pe].load(Ordering::Acquire) != 0
    }
}

impl Devices for Signalling {
    fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
        Ok(0)
    }
    fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
        Ok(())
    }
    fn lines(&mut self, pe: usize, _: u64) -> Lines {
        self.pins(pe).unwrap()
    }
    /// No latched line: the FIQ is each PE's word ([`Devices::own_fiq`]).
    fn pins(&mut self, _: usize) -> Option<Lines> {
        Some(Lines::default())
    }
    fn fast_ipi_words(&self) -> Vec<crate::machine::fast_ipi::Word> {
        (0..2)
            .map(|pe| crate::machine::fast_ipi::Word {
                cluster: 0,
                cpu: pe as u8,
                status: self.words[pe].as_ptr() as u64,
                armed: self.armed[pe].as_ptr() as u64,
            })
            .collect()
    }
    fn take_signalled(&mut self, pe: usize) -> bool {
        let raised = self.signalled & 1 << pe != 0;
        self.signalled &= !(1 << pe);
        raised
    }
    fn system_register(
        &mut self,
        _: usize,
        _: Encoding,
        write: Option<u64>,
        _: u64,
    ) -> Option<Result<u64, &'static str>> {
        self.register_calls += 1;
        let target = (write? & 1) as usize;
        if self.words[target].swap(1, Ordering::AcqRel) == 0 {
            self.signalled |= 1 << target;
        }
        Some(Ok(0))
    }
    fn serves_register(&self, encoding: Encoding) -> bool {
        encoding == SERVED
    }
    fn waits_on_host(&self) -> bool {
        self.asleep
    }
    fn local_registers(&self, pe: usize) -> Option<Arc<dyn LocalRegisters>> {
        Some(Arc::new(SignalWords {
            words: Arc::clone(&self.words),
            pe,
        }))
    }
}

/// A signal its handle completes takes no lock and wakes its target, which
/// reads the FIQ live from its own word, never in its lines; one the board
/// raises under the lock wakes the target through the call's notification.
#[test]
fn a_signal_without_the_lock_wakes_its_target_and_raises_its_fiq() {
    let (hub, kicks) = hub(2);
    let home = SharedDevices::new(Signalling::default(), Arc::clone(&hub));
    let mut sender = home.worker();
    let mut target = home.worker();
    // Each handle takes its PE's registers once, under the lock.
    sender.local_registers_of(0);
    target.local_registers_of(1);
    let calls = |devices: &SharedDevices<Signalling>| devices.with(|board| board.register_calls);
    let epoch = hub.inboxes[1].lines.load(Ordering::SeqCst);
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    let board = Arc::clone(&home.board);
    let held = board.devices.lock().unwrap();
    // The lock is held elsewhere: the send must not need it.
    assert_eq!(sender.system_register(0, SERVED, Some(1), 0), Some(Ok(0)));
    drop(held);
    assert_ne!(hub.inboxes[1].lines.load(Ordering::SeqCst), epoch);
    assert_eq!(kicks.load(Ordering::Relaxed), 1, "the sleeping target");
    assert!(target.own_fiq(1) && !target.lines(1, 0).fiq);
    assert!(!sender.own_fiq(0));
    assert_eq!(calls(&home), 0);
    // Pending already: no line changes, no wake.
    let epoch = hub.inboxes[1].lines.load(Ordering::SeqCst);
    assert_eq!(sender.system_register(0, SERVED, Some(1), 0), Some(Ok(0)));
    assert_eq!(hub.inboxes[1].lines.load(Ordering::SeqCst), epoch);
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    // Held low while the SoC sleeps, as the latched pins are.
    board.asleep.store(true, Ordering::Release);
    assert!(!target.own_fiq(1));
    board.asleep.store(false, Ordering::Release);
    // Under the lock: the call's notification wakes the target, though its
    // latched pins did not change.
    home.with(|devices| devices.words[1].store(0, Ordering::Release));
    let epoch = hub.inboxes[1].lines.load(Ordering::SeqCst);
    assert_eq!(
        sender.system_register(0, SERVED, Some(1 << 8 | 1), 0),
        Some(Ok(0))
    );
    assert_eq!(calls(&home), 1);
    assert_ne!(hub.inboxes[1].lines.load(Ordering::SeqCst), epoch);
    assert_eq!(kicks.load(Ordering::Relaxed), 2);
    assert!(target.own_fiq(1));
}

/// A PE's own FIQ is read live at every look past its cached lines: an
/// acknowledgement between the cache fill and the next look delivers no
/// FIQ, and a send between them is not lost, while the cache holds.
#[test]
fn a_pes_own_fiq_is_read_live_past_its_cached_lines() {
    use super::super::tests::board;
    use stela_contracts::identity::Identity;
    struct Own {
        word: Arc<AtomicBool>,
        queries: usize,
    }
    impl Devices for Own {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Ok(0)
        }
        fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            Ok(())
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            self.queries += 1;
            Lines::default()
        }
        fn own_fiq(&mut self, _: usize) -> bool {
            self.word.load(Ordering::SeqCst)
        }
    }
    let word = Arc::new(AtomicBool::new(true));
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let devices = Own {
        word: Arc::clone(&word),
        queries: 0,
    };
    let mut machine = board(&[], devices, identities);
    assert!(machine.device_lines_of(0, 0).fiq);
    word.store(false, Ordering::SeqCst);
    assert!(!machine.device_lines_of(0, 1).fiq, "acknowledged: no FIQ");
    word.store(true, Ordering::SeqCst);
    assert!(machine.device_lines_of(0, 2).fiq, "sent: not lost");
    assert_eq!(machine.devices.queries, 1, "the cached lines held");
}

/// The board's frozen fast-IPI table names each PE's own word and the
/// hub's counters for it: a request completed by it raises the target's
/// FIQ, tells it, and has a halted target woken through the board.
#[test]
fn the_fast_ipi_table_completes_a_request_through_the_hubs_words() {
    use crate::machine::fast_ipi::{Register, Sent};
    let (hub, kicks) = hub(2);
    let home = SharedDevices::new(Signalling::default(), Arc::clone(&hub));
    let mut target = home.worker();
    let table = home.fast_ipi_table().expect("a table");
    assert!(Arc::ptr_eq(&table, &target.fast_ipi_table().unwrap()));
    let entry = *table.target(0, 1).unwrap();
    assert_eq!(entry.lines, hub.inboxes[1].lines.as_ptr() as u64);
    assert_eq!(entry.sleeping, hub.inboxes[1].sleeping.as_ptr() as u64);
    assert_eq!(
        entry.interrupt,
        hub.inboxes[1].chain.interrupt_epoch().as_ptr() as u64
    );
    assert_eq!(entry.pe, 1);
    let epoch = hub.inboxes[1].lines.load(Ordering::SeqCst);
    let interrupt = hub.inboxes[1]
        .chain
        .interrupt_epoch()
        .load(Ordering::SeqCst);
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    assert_eq!(table.send(0, Register::RequestLocal, 1), Sent::Wake(1));
    assert!(target.own_fiq(1) && !target.lines(1, 0).fiq);
    assert_eq!(hub.inboxes[1].lines.load(Ordering::SeqCst), epoch + 1);
    assert_eq!(
        hub.inboxes[1]
            .chain
            .interrupt_epoch()
            .load(Ordering::SeqCst),
        interrupt + 1
    );
    assert_eq!(kicks.load(Ordering::Relaxed), 0, "the engine wakes it");
    target.wake_signalled(1);
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    // The engine's own path then finds it pending and changes nothing.
    assert_eq!(table.send(0, Register::RequestLocal, 1), Sent::Done);
    assert_eq!(hub.inboxes[1].lines.load(Ordering::SeqCst), epoch + 1);
}

/// A board that signals CPUs by number, which may name one no PE runs: a
/// write of `SERVED` with bit 8 clear completes on the sender's handle for
/// the CPU its low byte names; with bit 8 set it raises that CPU under the
/// lock.
struct Numbered {
    raised: Vec<bool>,
}

struct NumberedHandle;

impl LocalRegisters for NumberedHandle {
    fn register(&self, _: Encoding, _: Option<u64>) -> Option<u64> {
        None
    }
    fn storage(&self, _: Encoding) -> Option<u64> {
        None
    }
    fn send(&self, encoding: Encoding, value: u64) -> Option<Option<usize>> {
        (encoding == SERVED && value & 1 << 8 == 0).then_some(Some((value & 0xff) as usize))
    }
}

impl Devices for Numbered {
    fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
        Ok(0)
    }
    fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
        Ok(())
    }
    fn lines(&mut self, _: usize, _: u64) -> Lines {
        Lines::default()
    }
    fn latched_pins(&mut self, _: usize) -> Option<Lines> {
        Some(Lines::default())
    }
    fn take_signalled(&mut self, pe: usize) -> bool {
        self.raised.get_mut(pe).is_some_and(core::mem::take)
    }
    fn system_register(
        &mut self,
        _: usize,
        _: Encoding,
        write: Option<u64>,
        _: u64,
    ) -> Option<Result<u64, &'static str>> {
        self.raised[(write? & 0xff) as usize] = true;
        Some(Ok(0))
    }
    fn serves_register(&self, encoding: Encoding) -> bool {
        encoding == SERVED
    }
    fn local_registers(&self, _: usize) -> Option<Arc<dyn LocalRegisters>> {
        Some(Arc::new(NumberedHandle))
    }
}

/// A signal without the lock to a CPU the board has but no PE runs is
/// refused (the engine reports it once and goes on) and wakes no one; it
/// does not index past the PEs. One to a PE still wakes it.
#[test]
fn a_signal_to_a_cpu_without_a_pe_is_refused_and_dropped() {
    let (hub, kicks) = hub(2);
    let home = SharedDevices::new(
        Numbered {
            raised: vec![false; 8],
        },
        Arc::clone(&hub),
    );
    let mut sender = home.worker();
    for pe in 0..2 {
        hub.inboxes[pe].sleeping.store(true, Ordering::SeqCst);
    }
    let epochs = |hub: &Hub| -> Vec<u64> {
        hub.inboxes
            .iter()
            .map(|inbox| inbox.lines.load(Ordering::SeqCst))
            .collect()
    };
    let before = epochs(&hub);
    assert!(matches!(
        sender.system_register(0, SERVED, Some(5), 0),
        Some(Err(_))
    ));
    assert_eq!(kicks.load(Ordering::Relaxed), 0);
    assert_eq!(epochs(&hub), before);
    let woken = |home: &SharedDevices<Numbered>| {
        home.board.woken[Call::Register as usize].load(Ordering::Relaxed)
    };
    assert_eq!(woken(&home), 0);
    assert_eq!(sender.system_register(0, SERVED, Some(1), 0), Some(Ok(0)));
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    assert_eq!(woken(&home), 1, "counted in DEVICE WAKES register=");
}

/// A PE numbered past 63 whose FIQ a locked call raised is told and woken
/// as any other: the board keeps a raise for every PE.
#[test]
fn a_locked_signal_wakes_a_pe_past_63() {
    let (hub, kicks) = hub(70);
    let home = SharedDevices::new(
        Numbered {
            raised: vec![false; 70],
        },
        Arc::clone(&hub),
    );
    let mut sender = home.worker();
    hub.inboxes[66].sleeping.store(true, Ordering::SeqCst);
    let epoch = hub.inboxes[66].lines.load(Ordering::SeqCst);
    assert_eq!(
        sender.system_register(0, SERVED, Some(1 << 8 | 66), 0),
        Some(Ok(0))
    );
    assert_ne!(hub.inboxes[66].lines.load(Ordering::SeqCst), epoch);
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    assert!(!home.with(|devices| devices.raised[66]), "taken once");
}

/// A signal still pending when the SoC wakes reaches its PE: its FIQ was
/// held low while the SoC slept, so the PE, asleep in WFI, is told and woken
/// when the SoC's sleep ends, though no latched pin changed.
#[test]
fn a_signal_pending_when_the_soc_wakes_wakes_its_target() {
    let (hub, kicks) = hub(2);
    let home = SharedDevices::new(Signalling::default(), Arc::clone(&hub));
    let mut sender = home.worker();
    let mut target = home.worker();
    home.with(|devices| devices.asleep = true);
    assert_eq!(sender.system_register(0, SERVED, Some(1), 0), Some(Ok(0)));
    assert!(!target.own_fiq(1), "held low while the SoC sleeps");
    // The target looked at its lines and sleeps again.
    let epoch = hub.inboxes[1].lines.load(Ordering::SeqCst);
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    let before = kicks.load(Ordering::Relaxed);
    home.with(|devices| devices.asleep = false);
    assert_ne!(hub.inboxes[1].lines.load(Ordering::SeqCst), epoch);
    assert_eq!(kicks.load(Ordering::Relaxed), before + 1);
    assert!(target.own_fiq(1));
}

/// Invariant: a shared board publishes only latched pins, never a PE's own
/// signal word, so a PE that clears its word without the lock never reads
/// a stale FIQ that an unrelated locked call published while the word was
/// set (`Board::publish` and `Held::notify` use `latched_pins`).
#[test]
fn a_signal_cleared_without_the_lock_leaves_no_stale_published_fiq() {
    let (hub, _) = hub(2);
    let home = SharedDevices::new(Signalling::default(), Arc::clone(&hub));
    let mut sender = home.worker();
    let mut target = home.worker();
    let words = home.with(|devices| Arc::clone(&devices.words));
    assert_eq!(sender.system_register(0, SERVED, Some(1), 0), Some(Ok(0)));
    assert!(target.own_fiq(1));
    // Unrelated calls under the lock while the word is set: each release
    // publishes every PE's pins, and a write notifies the others.
    home.with(|_| ());
    sender.write(0, 0, 4, 0, 0).unwrap();
    assert_eq!(
        home.board.pins[1].load(Ordering::Acquire),
        shared_devices::pack(Some(Lines::default()))
    );
    // The target acknowledges on its own word.
    words[1].store(0, Ordering::Release);
    assert!(!target.own_fiq(1) && !target.lines(1, 0).fiq);
}

/// No lost wake-up: a PE that saw no FIQ and halts (`Hub::sleep`) is kicked
/// by a send that races with its halting, in every interleaving the
/// rounds reach. Each halt is bounded: a lost wake-up fails the test.
#[test]
fn a_signal_racing_with_a_halt_always_wakes_the_target() {
    struct Flag(Arc<AtomicBool>);
    impl Kick for Flag {
        fn kick(&self, _: usize) {
            self.0.store(true, Ordering::SeqCst);
        }
    }
    const ROUNDS: usize = 2_000;
    let kicked = Arc::new(AtomicBool::new(false));
    let hub = Hub::new(2, Flag(Arc::clone(&kicked)));
    for pe in 0..2 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Signalling::default(), Arc::clone(&hub));
    let mut sender = home.worker();
    let mut target = home.worker();
    let words = home.with(|devices| Arc::clone(&devices.words));
    let start = std::sync::Barrier::new(2);
    let done = std::sync::Barrier::new(2);
    let lost = AtomicBool::new(false);
    std::thread::scope(|scope| {
        scope.spawn(|| {
            for round in 0..ROUNDS {
                start.wait();
                // Vary where the send lands against the target's halt.
                for _ in 0..(round * 7) % 64 {
                    core::hint::spin_loop();
                }
                assert_eq!(sender.system_register(0, SERVED, Some(1), 0), Some(Ok(0)));
                done.wait();
            }
        });
        for _ in 0..ROUNDS {
            start.wait();
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            loop {
                let epoch = hub.inboxes[1].lines.load(Ordering::SeqCst);
                if target.own_fiq(1) {
                    break;
                }
                hub.sleep(1, epoch, true, || {
                    while !kicked.swap(false, Ordering::SeqCst) {
                        if std::time::Instant::now() > deadline {
                            lost.store(true, Ordering::SeqCst);
                            return;
                        }
                        std::thread::yield_now();
                    }
                });
                if lost.load(Ordering::SeqCst) {
                    break;
                }
            }
            words[1].store(0, Ordering::Release);
            kicked.store(false, Ordering::SeqCst);
            done.wait();
        }
    });
    assert!(
        !lost.load(Ordering::SeqCst),
        "a send left its target halted"
    );
}

/// `cargo test -p stela-engine --release --lib signal_send_costs --
/// --ignored --nocapture`: ns per signal sent through the board, under the
/// devices' lock (as every fast IPI was) or through the sender's handle,
/// alone and while another thread takes the lock for 2 us in every 4 us as
/// device accesses do. The target's word is cleared after each send, so
/// every send raises a line and bumps the target's epoch (no kick: it
/// does not sleep).
#[test]
#[ignore = "manual: opt-in"]
fn signal_send_costs() {
    const SENDS: u32 = 200_000;
    for contended in [false, true] {
        for locked in [true, false] {
            let (hub, _) = hub(2);
            let home = SharedDevices::new(Signalling::default(), hub);
            let mut sender = home.worker();
            sender.local_registers_of(0);
            let words = home.with(|devices| Arc::clone(&devices.words));
            let value = if locked { 1 << 8 | 1 } else { 1 };
            let stop = AtomicBool::new(false);
            let ns = std::thread::scope(|scope| {
                if contended {
                    scope.spawn(|| {
                        while !stop.load(Ordering::Relaxed) {
                            home.with(|_| {
                                let until =
                                    std::time::Instant::now() + std::time::Duration::from_micros(2);
                                while std::time::Instant::now() < until {
                                    core::hint::spin_loop();
                                }
                            });
                            let until =
                                std::time::Instant::now() + std::time::Duration::from_micros(2);
                            while std::time::Instant::now() < until {
                                core::hint::spin_loop();
                            }
                        }
                    });
                }
                let started = std::time::Instant::now();
                for _ in 0..SENDS {
                    sender.system_register(0, SERVED, Some(value), 0);
                    words[1].store(0, Ordering::Release);
                }
                let ns = started.elapsed().as_nanos() / u128::from(SENDS);
                stop.store(true, Ordering::Relaxed);
                ns
            });
            std::println!("SIGNAL contended={contended} locked={locked} ns_per_send={ns}");
        }
    }
}

#[test]
fn bytes_for_the_host_wake_one_sleeping_pe_and_reach_one_machine_once() {
    let (hub, kicks) = hub(3);
    let home = SharedDevices::new(Pending::default(), Arc::clone(&hub));
    let mut worker = home.worker();
    // With no PE asleep, the next to wait takes it, and none halts
    // meanwhile.
    worker.write(1, 0, 4, 3, 0).unwrap();
    assert_eq!(kicks.load(Ordering::Relaxed), 0);
    let mut halts = 0;
    hub.sleep(2, hub.lines_epoch(2), true, || halts += 1);
    assert_eq!(halts, 0, "the service is due");
    let mut home = home;
    assert!(home.take_host_service());
    assert!(!worker.take_host_service(), "taken once");
    // One PE asleep waiting for an interrupt wakes for it, and a request
    // already due wakes none.
    for pe in [1, 2] {
        hub.inboxes[pe].serves.store(true, Ordering::SeqCst);
        hub.inboxes[pe].sleeping.store(true, Ordering::SeqCst);
    }
    worker.write(1, 0, 4, 3, 0).unwrap();
    worker.write(1, 0, 4, 3, 0).unwrap();
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    assert!(worker.take_host_service());
}

#[test]
fn the_home_machine_sees_any_pes_reset_and_dma_without_the_lock() {
    let (hub, _) = hub(2);
    let mut home = SharedDevices::new(
        Pending {
            dma: Some(7),
            ..Pending::default()
        },
        hub,
    );
    let mut worker = home.worker();
    assert_eq!(home.dma_deadline(), Some(7), "as the board was made");
    worker.write(1, 0, 4, 90, 0).unwrap();
    assert_eq!(
        (home.dma_deadline(), worker.dma_deadline()),
        (Some(90), None)
    );
    worker.write(1, 0, 4, 0, 0).unwrap();
    assert_eq!(home.dma_deadline(), None);
    // A reset another PE's access requested reaches the home machine
    // once, and never the worker.
    worker.write(1, 0, 4, 1, 0).unwrap();
    assert!(worker.take_reset().is_none());
    assert!(
        home.take_reset()
            .is_some_and(|reset| reset.reason == "test")
    );
    assert!(home.take_reset().is_none());
    // A start reaches whichever machine asks first, once.
    home.write(0, 0, 4, 2, 0).unwrap();
    worker.write(1, 0, 4, 2, 0).unwrap();
    let start = worker.take_start().unwrap();
    assert_eq!((start.cpu, start.address), (1, 0x800));
    assert!(home.take_start().is_some());
    assert!(home.take_start().is_none() && worker.take_start().is_none());
    assert_eq!(home.waits(), [0; Call::ALL.len()]);
}

#[test]
fn only_the_home_machine_advances_the_devices_with_time_and_tells_the_others() {
    let (hub, _) = hub(2);
    let mut home = SharedDevices::new(
        Pending {
            irq_at: Some(20),
            ..Pending::default()
        },
        Arc::clone(&hub),
    );
    let mut worker = home.worker();
    assert_eq!(worker.deadline(10), None, "the worker waits to be told");
    assert_eq!(home.deadline(10), Some(20));
    let epoch = hub.lines_epoch(1);
    assert_eq!(home.deadline(25), None);
    assert_eq!(hub.lines_epoch(1), epoch + 1, "PE 1's IRQ rose");
    assert!(worker.lines(1, 25).irq);
    // Asking again changes nothing.
    home.deadline(30);
    assert_eq!(hub.lines_epoch(1), epoch + 1);
    // A call from a PE whose counter read lagged cannot undo that advance.
    assert!(!home.lines_and_deadline(0, 10).0.irq);
    assert_eq!(hub.lines_epoch(1), epoch + 1);
    assert!(worker.lines(1, 10).irq);
}

/// Latched pins: PE 1's IRQ rises once an advance with time reaches
/// `irq_at`; counts the advances.
#[derive(Default)]
struct Timed {
    irq_at: u64,
    raised: bool,
    advances: usize,
}

impl Devices for Timed {
    fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
        Ok(0)
    }
    fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
        Ok(())
    }
    fn lines(&mut self, pe: usize, now: u64) -> Lines {
        self.deadline(now);
        self.pins(pe).unwrap_or_default()
    }
    fn pins(&mut self, pe: usize) -> Option<Lines> {
        Some(Lines {
            irq: pe == 1 && self.raised,
            ..Lines::default()
        })
    }
    fn deadline(&mut self, now: u64) -> Option<u64> {
        self.advances += 1;
        self.raised |= now >= self.irq_at;
        (!self.raised).then_some(self.irq_at)
    }
}

#[test]
fn a_pe_advances_the_devices_once_their_deadline_passed_while_home_is_outside() {
    let (hub, _) = hub(2);
    let mut home = SharedDevices::new(
        Timed {
            irq_at: 20,
            ..Timed::default()
        },
        Arc::clone(&hub),
    );
    let mut worker = home.worker();
    assert_eq!(home.deadline(10), Some(20));
    let epoch = hub.lines_epoch(1);
    let advances = |home: &SharedDevices<Timed>| home.with(|devices| devices.advances);
    // Home runs: it advances the devices itself, so the worker does not.
    let before = advances(&home);
    assert_eq!(worker.deadline(25), None);
    assert_eq!(advances(&home), before);
    let (lines, until) = worker.lines_and_deadline(1, 25);
    assert!(!lines.irq);
    assert!(until.is_some_and(|until| until > 25), "looks again later");
    // Home leaves its run loop: the worker, awake past the deadline, brings
    // the devices up to its count and tells every PE what changed.
    hub.set_outside(0, true);
    assert_eq!(worker.deadline(15), None, "not yet due");
    assert_eq!(advances(&home), before);
    assert_eq!(worker.deadline(25), None, "it still waits for no device");
    assert_eq!(advances(&home), before + 1);
    assert_eq!(hub.lines_epoch(1), epoch + 1, "PE 1's IRQ rose");
    assert!(worker.lines(1, 25).irq);
    // Nothing more is due: no PE advances them again.
    assert_eq!(worker.deadline(40), None);
    assert!(worker.lines_and_deadline(1, 40).0.irq);
    assert_eq!(advances(&home), before + 1);
    // While another call holds the devices, the worker leaves them.
    home.with(|devices| {
        devices.raised = false;
        devices.irq_at = 50;
    });
    hub.set_outside(0, false);
    assert_eq!(home.deadline(45), Some(50));
    hub.set_outside(0, true);
    let held = home.board.devices.lock().unwrap_or_else(|e| e.into_inner());
    assert_eq!(worker.deadline(60), None);
    drop(held);
    assert_eq!(advances(&home), before + 2);
    assert_eq!(worker.deadline(60), None);
    assert_eq!(advances(&home), before + 3);
    assert!(worker.lines(1, 60).irq);
}

#[test]
fn a_pe_without_a_machine_waits_for_its_first_start() {
    let (hub, _) = hub(2);
    hub.event(0);
    hub.start(1, 0x1000);
    let mut halts = 0;
    assert_eq!(hub.first_start(1, || halts += 1), 0x1000);
    assert_eq!(halts, 0);
}

/// A call looks only at the PEs whose pins the devices say may have
/// changed: the others' pins are neither read nor published, and the PE
/// whose line rose is told once and reads it without the lock.
#[test]
fn a_call_looks_only_at_the_pes_whose_pins_changed() {
    #[derive(Default)]
    struct Masked {
        pins: [Lines; 4],
        changed: u64,
        reads: [usize; 4],
    }
    impl Devices for Masked {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Ok(0)
        }
        fn write(
            &mut self,
            _: usize,
            _: u64,
            _: u8,
            value: u64,
            _: u64,
        ) -> Result<(), DeviceError> {
            let pins = &mut self.pins[value as usize];
            if !pins.irq {
                pins.irq = true;
                self.changed |= 1 << value;
            }
            Ok(())
        }
        fn lines(&mut self, pe: usize, _: u64) -> Lines {
            self.pins[pe]
        }
        fn pins(&mut self, pe: usize) -> Option<Lines> {
            Some(self.pins[pe])
        }
        fn latched_pins(&mut self, pe: usize) -> Option<Lines> {
            self.reads[pe] += 1;
            Some(self.pins[pe])
        }
        fn take_pins_changed(&mut self) -> u64 {
            core::mem::take(&mut self.changed)
        }
    }
    let (hub, _) = hub(4);
    let home = SharedDevices::new(Masked::default(), Arc::clone(&hub));
    let mut worker = home.worker();
    let told: Vec<u64> = (0..4)
        .map(|pe| hub.inboxes[pe].lines.load(Ordering::SeqCst))
        .collect();
    worker.write(1, 0, 4, 2, 0).unwrap();
    let reads = home.with(|devices| core::mem::take(&mut devices.reads));
    // PE 1 updates its own; PE 2 is read to tell it and to publish it.
    assert_eq!((reads[0], reads[1], reads[3]), (0, 1, 0), "{reads:?}");
    assert!(reads[2] >= 1, "{reads:?}");
    let now: Vec<u64> = (0..4)
        .map(|pe| hub.inboxes[pe].lines.load(Ordering::SeqCst))
        .collect();
    assert_eq!(now, [told[0], told[1], told[2] + 1, told[3]]);
    assert!(worker.lines(2, 0).irq, "published for a lock-free read");
    // A call that changed no pins reads none of the others'.
    worker.write(1, 0, 4, 2, 0).unwrap();
    let reads = home.with(|devices| devices.reads);
    assert_eq!((reads[0], reads[2], reads[3]), (0, 0, 0), "{reads:?}");
}

/// A write and its settlement take the board's lock once: one release
/// publishes, and another PE whose line the write raised is told once. The
/// copy the settlement deferred runs after that release, and the transfer
/// that publishes what it came to follows it.
#[test]
fn a_write_settles_under_one_lock_and_tells_another_pe_once() {
    use super::super::tests::board;
    use super::super::{Memory, UnlockedDma};
    use core::cell::Cell;
    use stela_contracts::identity::Identity;
    type Log = Arc<Mutex<Vec<&'static str>>>;
    struct Copy(Log);
    impl UnlockedDma for Copy {
        fn run(self: Box<Self>, _: &mut dyn Memory) {
            self.0.lock().unwrap().push("copy");
        }
    }
    struct Raising {
        log: Log,
        irq: bool,
        copies: usize,
        left: Vec<Box<dyn UnlockedDma>>,
        publishes: Cell<usize>,
    }
    impl Devices for Raising {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Ok(0)
        }
        fn write(&mut self, cpu: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            assert_eq!(cpu, 0);
            self.log.lock().unwrap().push("write");
            self.irq = true;
            Ok(())
        }
        fn lines(&mut self, pe: usize, _: u64) -> Lines {
            Lines {
                irq: pe == 1 && self.irq,
                ..Lines::default()
            }
        }
        fn transfer(&mut self, _: usize, _: &mut dyn Memory, _: u64) -> Result<(), DeviceError> {
            self.log.lock().unwrap().push("transfer");
            if self.copies != 0 {
                self.copies -= 1;
                self.left.push(Box::new(Copy(Arc::clone(&self.log))));
            }
            Ok(())
        }
        fn take_unlocked(&mut self) -> Vec<Box<dyn UnlockedDma>> {
            core::mem::take(&mut self.left)
        }
        fn dma_deadline(&self) -> Option<u64> {
            // Read once by each release's publication.
            self.publishes.set(self.publishes.get() + 1);
            None
        }
    }
    let (hub, _) = hub(2);
    let log: Log = Arc::default();
    let devices = Raising {
        log: Arc::clone(&log),
        irq: false,
        copies: 1,
        left: Vec::new(),
        publishes: Cell::new(0),
    };
    let worker = SharedDevices::new(devices, Arc::clone(&hub)).worker();
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let mut machine = board(&[], worker, identities);
    let publishes = |machine: &mut Machine<_, SharedDevices<Raising>>| {
        machine.devices.with(|devices| devices.publishes.replace(0))
    };
    publishes(&mut machine);
    let told = hub.inboxes[1].lines.load(Ordering::SeqCst);
    let (devices, processors) = machine.processor_write_parts();
    let written = devices
        .write_settled(0, 0x30, 4, 1, 0, processors)
        .expect("a plain write");
    assert_eq!(written.written, Ok(()));
    assert!(
        written
            .settled
            .is_some_and(|settled| settled.transfer.is_ok())
    );
    assert_eq!(
        *log.lock().unwrap(),
        ["write", "transfer", "copy", "transfer"]
    );
    // The write's lock and the copy's publishing transfer: two releases,
    // past the one of the call that counted last.
    assert_eq!(publishes(&mut machine), 1 + 2);
    assert_eq!(hub.inboxes[1].lines.load(Ordering::SeqCst), told + 1);
    // Without a copy, one lock in all; PE 1's line is already up.
    log.lock().unwrap().clear();
    let (devices, processors) = machine.processor_write_parts();
    assert!(
        devices
            .write_settled(0, 0x30, 4, 1, 0, processors)
            .is_some_and(|written| written.written.is_ok())
    );
    assert_eq!(*log.lock().unwrap(), ["write", "transfer"]);
    assert_eq!(publishes(&mut machine), 1 + 1);
    assert_eq!(hub.inboxes[1].lines.load(Ordering::SeqCst), told + 1);
}

#[test]
fn a_shared_board_classifies_and_serves_a_scalar_access_in_one_call() {
    use super::super::tests::board;
    use stela_contracts::identity::Identity;
    const CANONICAL: u64 = 0x10;
    const SAMPLER: u64 = 0x20;
    #[derive(Default)]
    struct Classified {
        reads: usize,
        writes: usize,
    }
    impl Devices for Classified {
        fn pc_sample_target(&self, physical: u64, _: u8) -> Option<usize> {
            (physical == SAMPLER).then_some(1)
        }
        fn requires_canonical_read(&self, _: usize, physical: u64, _: DeviceReadShape) -> bool {
            physical == CANONICAL
        }
        fn requires_canonical_write(&self, _: usize, physical: u64, _: u8) -> bool {
            physical == CANONICAL
        }
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            self.reads += 1;
            Ok(7)
        }
        fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            self.writes += 1;
            Ok(())
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            Lines::default()
        }
    }
    let (hub, _) = hub(2);
    let mut worker = SharedDevices::new(Classified::default(), hub).worker();
    assert_eq!(worker.read_scalar(1, 0x30, 4, 0), ScalarRead::Served(Ok(7)));
    assert_eq!(
        worker.read_scalar(1, CANONICAL, 4, 0),
        ScalarRead::Canonical
    );
    assert_eq!(worker.read_scalar(1, SAMPLER, 4, 0), ScalarRead::Sample(1));
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let mut machine = board(&[], worker, identities);
    let (devices, processors) = machine.processor_write_parts();
    assert!(
        devices
            .write_settled(0, CANONICAL, 4, 1, 0, processors)
            .is_none()
    );
    let (devices, processors) = machine.processor_write_parts();
    let written = devices
        .write_settled(0, 0x30, 4, 1, 0, processors)
        .expect("a plain write");
    assert_eq!(written.written, Ok(()));
    assert!(
        written
            .settled
            .is_some_and(|settled| settled.transfer.is_ok())
    );
    // Only the plain accesses reached the registers.
    assert_eq!(
        machine
            .devices
            .with(|devices| (devices.reads, devices.writes)),
        (1, 1)
    );
}

#[test]
fn board_grace_follows_run_loop_flags() {
    let (hub, _) = hub(2);
    let grace = &hub.board.grace;
    assert!(!grace.inside(0) && !grace.inside(1));
    // Entering the run loop enters the section; parking leaves it.
    hub.set_outside(0, false);
    hub.set_outside(1, false);
    assert!(grace.inside(0) && grace.inside(1));
    let snapshot = grace.snapshot();
    assert!(!grace.passed(&snapshot, None));
    hub.set_parked(1, true);
    assert!(!grace.inside(1));
    assert!(!grace.passed(&snapshot, None), "PE 0 has not moved on");
    // A loop top passes; parked and outside together stay outside.
    hub.pass(0);
    assert!(grace.passed(&snapshot, None));
    hub.set_outside(1, true);
    hub.set_parked(1, false);
    assert!(!grace.inside(1));
    hub.pass(1);
    hub.set_outside(1, false);
    assert!(grace.inside(1));
    // A PE that was outside at the snapshot never holds it up.
    hub.set_outside(0, true);
    let snapshot = grace.snapshot();
    hub.pass(1);
    assert!(grace.passed(&snapshot, None));
}

#[test]
fn board_wait_runs_out_and_reports_once() {
    use super::super::board::Waited;
    let (hub, _) = hub(2);
    hub.set_outside(0, false);
    hub.set_outside(1, false);
    let snapshot = hub.board.grace.snapshot();
    let mut now = 0u64;
    let mut counter = || {
        now += 10;
        now
    };
    // PE 1 stays inside: PE 0's wait ends at its deadline, back inside.
    assert_eq!(
        hub.board.wait(0, &snapshot, 100, &mut counter),
        Waited::Overdue { first: true }
    );
    assert!(hub.board.grace.inside(0));
    assert_eq!(
        hub.board.wait(0, &snapshot, 100, &mut counter),
        Waited::Overdue { first: false }
    );
    hub.pass(1);
    assert_eq!(
        hub.board.wait(0, &snapshot, 100, &mut counter),
        Waited::Passed
    );
}

/// A PE that slept through hundreds of other PEs' TLBIs by page, of many
/// processes' pages scattered over the lower half, keeps its kernel
/// translations: its mail merges them into lower-half ranges instead of an
/// invalidation of everything, so the wake path's pages stay in its ITLB.
#[test]
fn a_sleeping_pe_keeps_its_kernel_translations_through_a_pile_of_page_tlbis() {
    use crate::machine::system::TlbiKind::VA;
    const KERNEL: u64 = 0xffff_fe00_1000_0000;
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[], &hub);
    let el = machine.cpu.pstate.el;
    let variant = machine.current_variant(false, false);
    let contexts = machine.contexts(el, variant);
    machine.chain.set_contexts(contexts);
    for page in 0..32 {
        let pc = KERNEL + page * FETCH_PAGE;
        machine.fetched_through(1, pc, FETCH_PAGE, true);
        machine.translate_page(pc, stela_compiler::x86_64::chain::block_key(page << 12, 2));
    }
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    for _ in 0..600 {
        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        let (asid, page) = ((seed >> 56) % 200 + 1, (seed >> 20) & 0xff_ffff);
        hub.broadcast(1, &[tlbi(VA, asid << 48 | page)]);
    }
    machine.read_mail().unwrap();
    assert_eq!(machine.statistics.itlb_drops[4], 0, "no whole invalidation");
    assert!(machine.statistics.itlb_sources[super::super::ItlbSource::MailMerged as usize] > 0);
    assert_eq!(machine.contexts(el, variant), contexts);
    for page in 0..32 {
        assert!(machine.current_key(KERNEL + page * FETCH_PAGE).is_some());
    }
}

#[test]
fn passive_registers_are_read_without_the_devices_lock() {
    use super::super::PassiveRegisters;
    struct Word(AtomicU64);
    impl PassiveRegisters for Word {
        fn read(&self, physical: u64, bytes: u8) -> Option<u64> {
            (physical == 0x40 && bytes == 8).then(|| self.0.load(Ordering::Relaxed))
        }
    }
    #[derive(Default)]
    struct Counted {
        word: Option<Arc<Word>>,
        reads: usize,
    }
    impl Devices for Counted {
        fn passive_registers(&self) -> Option<Arc<dyn PassiveRegisters>> {
            self.word
                .as_ref()
                .map(|word| Arc::clone(word) as Arc<dyn PassiveRegisters>)
        }
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            self.reads += 1;
            Ok(3)
        }
        fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            Ok(())
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            Lines::default()
        }
    }
    let word = Arc::new(Word(AtomicU64::new(9)));
    let (hub, _) = hub(2);
    let home = SharedDevices::new(
        Counted {
            word: Some(Arc::clone(&word)),
            reads: 0,
        },
        hub,
    );
    let mut worker = home.worker();
    // While another call holds the devices' lock, the passive word reads.
    let held = home.board.lock(Call::Other);
    assert_eq!(worker.read_scalar(1, 0x40, 8, 0), ScalarRead::Served(Ok(9)));
    word.0.store(11, Ordering::Relaxed);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(11));
    drop(held);
    // Any other width or address takes the lock to the model.
    assert_eq!(worker.read_scalar(1, 0x40, 4, 0), ScalarRead::Served(Ok(3)));
    assert_eq!(worker.read_scalar(1, 0x48, 8, 0), ScalarRead::Served(Ok(3)));
    assert_eq!(home.with(|devices| devices.reads), 2);
}

#[test]
fn plain_reads_are_served_again_without_the_lock_while_their_epoch_holds() {
    /// A model at 0x40 whose 8-byte read is plain, and a bump of its epoch
    /// on every write, as a bus moves it.
    #[derive(Default)]
    struct Plain {
        epoch: Arc<AtomicU64>,
        value: u64,
        reads: usize,
    }
    impl Devices for Plain {
        fn plain_epoch(&self, physical: u64, bytes: u8) -> Option<(Arc<AtomicU64>, u64)> {
            (physical == 0x40 && bytes == 8)
                .then(|| (Arc::clone(&self.epoch), self.epoch.load(Ordering::SeqCst)))
        }
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            self.reads += 1;
            Ok(self.value)
        }
        fn write(
            &mut self,
            _: usize,
            _: u64,
            _: u8,
            value: u64,
            _: u64,
        ) -> Result<(), DeviceError> {
            self.epoch.fetch_add(1, Ordering::SeqCst);
            self.value = value;
            Ok(())
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            Lines::default()
        }
    }
    let (hub, _) = hub(2);
    let home = SharedDevices::new(Plain::default(), hub);
    let mut worker = home.worker();
    let reads = |home: &SharedDevices<Plain>| home.with(|devices| devices.reads);
    // Each plain read it serves is native code's to serve again.
    let plain = |worker: &mut SharedDevices<Plain>| {
        worker.take_plain_read().map(|read| {
            (
                read.epoch.load(Ordering::SeqCst),
                read.seen,
                read.value,
                read.until,
            )
        })
    };
    assert_eq!(worker.read_scalar(1, 0x40, 8, 0), ScalarRead::Served(Ok(0)));
    assert_eq!(plain(&mut worker), Some((0, 0, 0, u64::MAX)));
    assert_eq!(plain(&mut worker), None);
    assert_eq!(reads(&home), 1);
    // Served again while another call holds the devices' lock.
    let held = home.board.lock(Call::Other);
    assert_eq!(worker.read_scalar(1, 0x40, 8, 1), ScalarRead::Served(Ok(0)));
    assert_eq!(plain(&mut worker), Some((0, 0, 0, u64::MAX)));
    drop(held);
    assert_eq!(reads(&home), 1);
    // A write by another PE moves the epoch: the next read reaches the model.
    let mut other = home.worker();
    other.write(0, 0x40, 8, 5, 2).unwrap();
    assert_eq!(worker.read_scalar(1, 0x40, 8, 3), ScalarRead::Served(Ok(5)));
    assert_eq!(worker.read_scalar(1, 0x40, 8, 4), ScalarRead::Served(Ok(5)));
    assert_eq!(reads(&home), 2);
    // Other reads, and any read once a device deadline passed, reach it.
    assert_eq!(worker.read_scalar(1, 0x48, 8, 5), ScalarRead::Served(Ok(5)));
    assert_eq!(plain(&mut worker), None);
    assert_eq!(worker.read_scalar(1, 0x40, 4, 5), ScalarRead::Served(Ok(5)));
    assert_eq!(plain(&mut worker), None);
    assert_eq!(reads(&home), 4);
    home.board.time.store(6, Ordering::Release);
    assert_eq!(worker.read_scalar(1, 0x40, 8, 6), ScalarRead::Served(Ok(5)));
    assert_eq!(reads(&home), 5);
    // Kept with the deadline it found.
    assert_eq!(plain(&mut worker), Some((1, 1, 5, 6)));
}

/// The copies a transfer leaves ([`Devices::take_unlocked`]) run on the PE
/// that settled the write, after it released the devices' lock, each once,
/// with PEs settling at once; and what each came to is published by a
/// transfer on that PE before its call returns, with no other PE or tick
/// coming by (otherwise a completion can wait for an idle home machine).
#[test]
fn transfers_leave_their_copies_to_run_outside_the_lock() {
    use super::super::{LockedMemory, Memory, UnlockedDma};
    use crate::machine::mmu::MemoryLocks;
    use std::sync::OnceLock;
    type Shared = Arc<OnceLock<Arc<shared_devices::Board<Copying>>>>;
    struct Copy {
        board: Shared,
        address: u64,
        ran: Arc<AtomicUsize>,
        alone: Arc<AtomicBool>,
    }
    impl UnlockedDma for Copy {
        fn run(self: Box<Self>, memory: &mut dyn Memory) {
            let board = self.board.get().expect("the board is built");
            // Free unless another PE took it since: checked while one runs.
            if self.alone.load(Ordering::SeqCst) {
                assert!(board.devices.try_lock().is_ok(), "outside the lock");
            }
            assert!(memory.write(self.address, &[1]));
            self.ran.fetch_add(1, Ordering::SeqCst);
        }
    }
    /// A device whose write starts a copy and whose transfer publishes
    /// every copy that ran.
    struct Copying {
        board: Shared,
        deferred: bool,
        wants: [bool; 4],
        left: Vec<Box<dyn UnlockedDma>>,
        ran: Arc<AtomicUsize>,
        published: usize,
        alone: Arc<AtomicBool>,
    }
    impl Devices for Copying {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Ok(0)
        }
        fn write(&mut self, cpu: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            self.wants[cpu] = true;
            Ok(())
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            Lines::default()
        }
        fn defer_dma(&mut self) {
            self.deferred = true;
        }
        fn take_unlocked(&mut self) -> Vec<Box<dyn UnlockedDma>> {
            core::mem::take(&mut self.left)
        }
        fn transfer(&mut self, cpu: usize, _: &mut dyn Memory, _: u64) -> Result<(), DeviceError> {
            assert!(self.deferred, "a shared board defers its copies");
            self.published = self.ran.load(Ordering::SeqCst);
            if core::mem::take(&mut self.wants[cpu]) {
                self.left.push(Box::new(Copy {
                    board: Arc::clone(&self.board),
                    address: cpu as u64,
                    ran: Arc::clone(&self.ran),
                    alone: Arc::clone(&self.alone),
                }));
            }
            Ok(())
        }
    }
    /// RAM that counts each PE's byte.
    struct Counted(Arc<[AtomicUsize; 4]>);
    impl Memory for Counted {
        fn read(&mut self, _: u64, _: &mut [u8]) -> bool {
            true
        }
        fn write(&mut self, physical: u64, _: &[u8]) -> bool {
            self.0[physical as usize].fetch_add(1, Ordering::SeqCst);
            true
        }
    }
    impl LockedMemory for Counted {
        fn under(&mut self, _: MemoryLocks) -> &mut dyn Memory {
            self
        }
    }
    let board: Shared = Arc::new(OnceLock::new());
    let ran = Arc::new(AtomicUsize::new(0));
    let alone = Arc::new(AtomicBool::new(true));
    let devices = Copying {
        board: Arc::clone(&board),
        deferred: false,
        wants: [false; 4],
        left: Vec::new(),
        ran: Arc::clone(&ran),
        published: 0,
        alone: Arc::clone(&alone),
    };
    let hub = Hub::new(4, Ticks(Arc::new(AtomicUsize::new(0))));
    let home = SharedDevices::new(devices, hub);
    assert!(board.set(Arc::clone(&home.board)).is_ok());
    let written = Arc::new([const { AtomicUsize::new(0) }; 4]);
    // One PE, every other one idle: the copy ran and its result is
    // published before the call returns.
    let mut first = home.worker();
    first.write(1, 0, 4, 0, 0).unwrap();
    assert!(
        first
            .settle_write(1, 0, &mut Counted(Arc::clone(&written)))
            .transfer
            .is_ok()
    );
    assert_eq!(ran.load(Ordering::SeqCst), 1, "before the call returned");
    assert_eq!(
        home.with(|devices| devices.published),
        1,
        "published at once"
    );
    alone.store(false, Ordering::SeqCst);
    written[1].store(0, Ordering::SeqCst);
    ran.store(0, Ordering::SeqCst);
    const ROUNDS: usize = 200;
    let threads: Vec<_> = (1..4)
        .map(|cpu| {
            let mut worker = home.worker();
            let written = Arc::clone(&written);
            std::thread::spawn(move || {
                for _ in 0..ROUNDS {
                    worker.write(cpu, 0, 4, 0, 0).unwrap();
                    let settled = worker.settle_write(cpu, 0, &mut Counted(Arc::clone(&written)));
                    assert!(settled.transfer.is_ok());
                }
            })
        })
        .collect();
    for thread in threads {
        thread.join().unwrap();
    }
    assert_eq!(ran.load(Ordering::SeqCst), 3 * ROUNDS);
    for cpu in 1..4 {
        assert_eq!(written[cpu].load(Ordering::SeqCst), ROUNDS, "PE {cpu}");
    }
    assert_eq!(
        home.with(|devices| (devices.left.len(), devices.published)),
        (0, 3 * ROUNDS),
        "every copy published by its own call"
    );
}

/// The word a cell probe counts a transfer through the cell at host
/// address `cell` in, as [`chain::lookup_dispatcher`]'s does from the
/// control it runs with: the cell's `heat` word among the control's
/// stamped cells, or else the control's own `heat`, which nothing reads.
fn probed_heat(control: &chain::ChainControl, cell: u64) -> Option<*const AtomicU32> {
    let index = cell.wrapping_sub(control.cells) >> 3;
    (index < (chain::CELL_LIVE / 8) as u64)
        .then(|| (control.cells + chain::CELL_HEAT as u64 + index) as *const AtomicU32)
}

/// A PE gone parallel runs on its hub control: its cell probes count heat
/// in the board's stamped cells, where the sweep reads it, so a hot exit
/// out of the page gets a stamped tail, and its stamps are counted.
#[test]
fn a_parallel_pe_promotes_the_exits_its_cell_probes_made_hot() {
    use super::super::tests::{NATIVE_CODE_STRIDE, machine};
    use stela_compiler::block::ExitKind;
    const ADD: u32 = 0x9100_0400; // add x0, x0, #1
    const RET: u32 = 0xd65f_03c0;
    let mut words = vec![0; 1026];
    words[0] = 0x1400_0400; // b .+0x1000
    words[1024] = ADD;
    words[1025] = RET;
    let mut machine = machine(&words);
    let base = machine.cpu.pc;
    let far = base + 0x1000;
    let exit = |next_pc, kind: ExitKind| super::super::NativeExit::Exit {
        next_pc,
        kind: kind as u64,
    };
    machine.host.exits = vec![exit(far, ExitKind::Direct), exit(base, ExitKind::Return)];
    let (hub, _) = hub(1);
    machine.enter_parallel(Arc::clone(&hub)).unwrap();
    machine.step().unwrap();
    machine.step().unwrap();
    let first = &machine.code.blocks[0];
    let code = &machine.host.installed[first.code.0 / NATIVE_CODE_STRIDE - 1];
    let site = chain::direct_patch((first.code.0 + code.len() - chain::DIRECT_EXIT_BYTES) as u64);
    let (number, cells, stamps_held) = {
        let mut cache = machine.code.cache();
        let number = cache.targets.site_cell_number(site).expect("a cell");
        let control = machine.chain.snapshot();
        (
            number,
            cache.targets.stamped_cells(),
            cache.targets.holds_stamps(control.stamps),
        )
    };
    assert!((number as usize) < chain::STAMPED_CELLS);
    let control = machine.chain.snapshot();
    // The cell probe's transfers through the exit's cell.
    let cell = cells + (number as u64) * size_of::<chain::LookupEntry>() as u64;
    let patches = machine.host.patches.len();
    for _ in 0..chain::HOT_CELL {
        if let Some(heat) = probed_heat(&control, cell) {
            // SAFETY: a heat word of the board's stamped cells, an atomic,
            // which live while the board does.
            unsafe { &*heat }.fetch_add(1, Ordering::Relaxed);
        }
    }
    for _ in 0..chain::STAMPED_CELLS / 256 {
        machine.promote_hot_exits();
    }
    assert_eq!(machine.host.patches.len(), patches + 2, "a stamped tail");
    assert_eq!(
        machine.host.patches[patches].1.len(),
        chain::STAMPED_TAIL_BYTES
    );
    assert_eq!(machine.statistics.stamped_tails, 1);
    assert_eq!(machine.statistics.stamped_in_use, 1);
    assert_eq!(control.cells, cells, "the board's stamped cells");
    assert!(stamps_held, "stamps of the board's table");
    // What the stamped tails count is read from the PE's own stamps.
    assert_eq!(machine.chain.stamp_counts(), [0; chain::STAMP_COUNTS]);
}

/// Every native entry starts the PE's jump cache over (`chain::lookup_tail`):
/// what its table probes found under the last run's ITLB, contexts and
/// blocks holds in no later run.
#[test]
fn each_native_entry_moves_the_jump_cache_serial() {
    let mut machine = super::super::tests::machine(&[0xd65f_03c0]);
    let stamps = machine.chain.stamps().0;
    assert_ne!(stamps, 0, "every PE's control has stamps before it runs");
    let at = (stamps as i64 + i64::from(chain::JUMP_SERIAL)) as *const u64;
    // SAFETY: the serial in front of the PE's live stamps; no native code
    // runs meanwhile.
    let serial = || unsafe { at.read() };
    let before = serial();
    assert_ne!(before, 0);
    machine.chain.begin(0, 16);
    assert_eq!(serial(), before + 1);
    machine.chain.begin(0, 16);
    assert_eq!(serial(), before + 2);
}

/// A PE that joins a board other than the one it was made on takes stamps
/// of the board's table, not of its first board's, which goes.
#[test]
fn a_pe_that_joins_the_board_takes_stamps_of_its_table() {
    use super::super::board;
    use super::super::tests::machine;
    let mut memory = vec![0u8; 4 * 4096];
    let window = super::super::CodeWindow {
        exec: memory.as_mut_ptr() as usize,
        write: memory.as_mut_ptr() as usize,
        bytes: memory.len(),
    };
    let arena = || {
        Arc::new(board::Code::new(
            board::slot_capacity(Some(window.bytes)),
            chain::MIN_LOOKUP_ENTRIES,
            Some(window),
        ))
    };
    let shared = arena();
    let (hub, _) = hub(2);
    hub.share_code(&shared);
    let mut second = machine(&[0xd65f_03c0]);
    second.code = arena();
    second.attach_stamps();
    let (_attached, control) = Attached::new(&hub, 1, false).unwrap();
    second.join_code(&hub);
    assert!(Arc::ptr_eq(&second.code, &shared));
    second.adopt_parallel_control(control);
    let control = second.chain.snapshot();
    let mut cache = shared.cache();
    assert_eq!(control.cells, cache.targets.stamped_cells());
    assert!(cache.targets.holds_stamps(control.stamps));
}

/// A WFE of a PE that is not home ends inside the scheduler's entry when
/// its next own wake is within the halt-poll: at its event stream edge, or
/// earlier on another PE's SEV, with mail that came during the spin read
/// before the guest runs again. A wake past the poll, and the home PE,
/// take the wait passes.
#[test]
fn a_short_wfe_completes_in_its_exit_without_the_wait_passes() {
    use super::super::tests::{BASE, FakeHost, machine};
    use crate::machine::NoDevices;
    use crate::machine::system::Register as SystemRegister;
    use crate::machine::system::TlbiKind::VAA;
    const WFE: u32 = 0xd503_205f;
    let start = |events: u64| {
        let (hub, _) = hub(2);
        let mut machine = machine(&[]);
        machine.enter_parallel(Arc::clone(&hub)).unwrap();
        hub.home.store(1, Ordering::Relaxed);
        machine.host.counter_step = 1;
        machine.idle_poll = 300;
        // CNTHCTL_EL2.EVNTEN with EVNTI `events`: edges 2^(events+1) apart.
        machine
            .cpu
            .registers
            .set(SystemRegister::CNTHCTL_EL2, 0, 1 << 2 | events << 4);
        (hub, machine)
    };
    let wfe = |machine: &mut Machine<FakeHost, NoDevices>| {
        machine.cpu.pc = BASE;
        machine.system_instruction(WFE).unwrap();
        machine.schedule()
    };
    // Edges 128 counts apart: the spin reaches the next one.
    let (_hub, mut machine) = start(6);
    let edge = crate::machine::timer::next_event(&machine.cpu, machine.host.now).unwrap();
    wfe(&mut machine).unwrap();
    assert!(machine.waiting.is_none() && !machine.cpu.event);
    assert_eq!(machine.cpu.pc, BASE + 4);
    let statistics = &machine.statistics;
    assert_eq!((statistics.waits, statistics.short_waits), (1, 1));
    assert_eq!(statistics.halt_ticks, 0);
    assert!(machine.host.timers.is_empty(), "no halt was armed");
    assert!(machine.host.now >= edge && machine.host.now < edge + 100);
    // Another PE's TLBI and SEV during the spin: both are read in it and
    // the SEV ends the WFE before the edge.
    let (hub, mut machine) = start(7);
    let edge = crate::machine::timer::next_event(&machine.cpu, machine.host.now).unwrap();
    let sender = Arc::clone(&hub);
    machine.host.on_relax = Some(Box::new(move |turn| {
        if turn == 3 {
            sender.broadcast(1, &[tlbi(VAA, 0x1234)]);
            sender.event(1);
        }
    }));
    wfe(&mut machine).unwrap();
    assert!(machine.waiting.is_none());
    assert!(!hub.pending(0), "the mail was read");
    assert_eq!(machine.statistics.short_waits, 1);
    assert!(machine.host.now < edge, "the SEV ended it");
    // Edges 65536 counts apart, past the poll: the wait passes take it.
    let (_hub, mut machine) = start(15);
    wfe(&mut machine).unwrap();
    assert!(machine.waiting.is_none());
    assert_eq!(machine.statistics.short_waits, 0);
    assert!(machine.statistics.waits > 0);
    // The home PE always takes the passes.
    let (hub, mut machine) = start(6);
    hub.home.store(0, Ordering::Relaxed);
    wfe(&mut machine).unwrap();
    assert_eq!(machine.statistics.short_waits, 0);
}

/// A device whose IRQ rises at `irq_at`, which a write sets; counts its
/// advances where reading the count takes no lock.
struct Armed {
    irq_at: u64,
    advances: Arc<AtomicUsize>,
}

impl Devices for Armed {
    fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
        Ok(0)
    }
    fn write(&mut self, _: usize, _: u64, _: u8, value: u64, _: u64) -> Result<(), DeviceError> {
        self.irq_at = value;
        Ok(())
    }
    fn lines(&mut self, _: usize, now: u64) -> Lines {
        Lines {
            irq: now >= self.irq_at,
            ..Lines::default()
        }
    }
    fn deadline(&mut self, now: u64) -> Option<u64> {
        self.advances.fetch_add(1, Ordering::Relaxed);
        (now < self.irq_at).then_some(self.irq_at)
    }
}

/// The home machine's wait pass advances the devices once: the deadline
/// its cached lines came with stands in for a second advance, until
/// another PE's call reaches the devices, such as one that arms an earlier
/// deadline, which the next pass then sees.
#[test]
fn home_reuses_its_lines_deadline_until_another_pe_reaches_the_devices() {
    use super::super::tests::board;
    use stela_contracts::identity::Identity;
    let (hub, _) = hub(2);
    let count = Arc::new(AtomicUsize::new(0));
    let home = SharedDevices::new(
        Armed {
            irq_at: 100,
            advances: Arc::clone(&count),
        },
        hub,
    );
    let mut worker = home.worker();
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let mut machine = board(&[], home, identities);
    let advances = |_: &Machine<_, SharedDevices<Armed>>| count.load(Ordering::Relaxed);
    assert!(!machine.device_lines_of(0, 10).irq);
    let before = advances(&machine);
    assert_eq!(machine.next_deadline(10), Some(100));
    assert_eq!(machine.next_deadline(20), Some(100));
    assert_eq!(advances(&machine), before, "no second advance");
    // PE 1 arms an earlier deadline: the next pass asks the devices.
    worker.write(1, 0x30, 4, 50, 20).unwrap();
    assert_eq!(machine.next_deadline(20), Some(50));
    assert_eq!(advances(&machine), before + 1);
    // Past the cached deadline the devices are asked again too.
    machine.device_lines = None;
    assert!(!machine.device_lines_of(0, 30).irq);
    let before = advances(&machine);
    assert_eq!(machine.next_deadline(40), Some(50));
    assert_eq!(advances(&machine), before);
    assert_eq!(machine.next_deadline(60), None);
    assert_eq!(advances(&machine), before + 1);
}

/// Quiescent maintenance: the post-then-entry order. Mail posted while a
/// PE is between blocks counts as applied at once, and the PE's next entry
/// finds it and does not enter until it has read it.
#[test]
fn mail_posted_between_blocks_is_applied_at_once_and_blocks_the_next_entry() {
    let (hub, _) = hub(2);
    hub.leave_active(1);
    let mut posted = hub.broadcast(0, &[Maintenance::Code(None)]);
    assert_eq!(posted.len(), 1, "reached and interrupted");
    assert!(
        hub.applied(&mut posted),
        "a PE between blocks is not waited for"
    );
    assert!(!hub.enter_active(1), "the entry finds the mail");
    assert_eq!(hub.inboxes[1].active.load(Ordering::SeqCst) & 1, 0);
    let mail = hub.take(1).unwrap();
    hub.acknowledge(1, mail.serial);
    assert!(hub.enter_active(1), "read, it enters");
}

/// The entry-then-post order: a PE running a block is waited for until it
/// applies the mail or leaves the block, whichever is first.
#[test]
fn mail_posted_while_a_pe_runs_a_block_waits_for_it_to_apply_or_leave() {
    let (hub, _) = hub(3);
    let mut posted = hub.broadcast(0, &[Maintenance::Code(None)]);
    assert_eq!(posted.len(), 2);
    assert!(!hub.applied(&mut posted));
    // PE 1 applies it while it runs; PE 2 leaves its block without it.
    let mail = hub.take(1).unwrap();
    hub.acknowledge(1, mail.serial);
    assert!(!hub.applied(&mut posted));
    assert_eq!(posted, [(2, 1)]);
    hub.leave_active(2);
    assert!(hub.applied(&mut posted));
    // Left without reading it, PE 2 reads it before its next block.
    assert!(!hub.enter_active(2));
    let mail = hub.take(2).unwrap();
    hub.acknowledge(2, mail.serial);
    assert!(hub.enter_active(2));
}

/// A broadcast's DSB does not wait for a PE between blocks, however long
/// it stays there (fetching, compiling, scheduling): the wait takes no
/// turn at all, and the mail stays for the PE's next entry.
#[test]
fn a_completion_does_not_wait_for_a_pe_between_blocks() {
    let (hub, _) = hub(2);
    hub.leave_active(1);
    let mut machine = first_of_two(&[], &hub);
    machine.post(Maintenance::Code(Some(0x4000)));
    machine.send_outgoing();
    machine.complete_maintenance().unwrap();
    assert_eq!(machine.host.relaxes, 0, "no turn of the wait");
    assert!(machine.posted.is_empty());
    assert!(hub.pending(1));
    assert!(!hub.enter_active(1));
}

/// A machine that mail reached after its fetch does not run what it
/// fetched: it reads the mail first and fetches again, so it never runs
/// a block the mail retired.
#[test]
fn a_machine_reads_mail_that_came_after_its_fetch_before_it_runs_the_block() {
    use super::super::tests::BASE;
    use stela_compiler::block::ExitKind;
    let (hub, _) = hub(2);
    // NOP; NOP.
    let mut machine = first_of_two(&[0xd503_201f, 0xd503_201f], &hub);
    let exit = |next_pc| super::super::NativeExit::Exit {
        next_pc,
        kind: ExitKind::Direct as u64,
    };
    // The block runs once and comes back to itself: it stays installed.
    machine.host.exits = vec![exit(BASE), exit(BASE + 8)];
    machine.step().unwrap();
    assert_eq!(machine.cpu.pc, BASE);
    // PE 1 discards every block while PE 0 is between blocks; PE 0's next
    // fetch still finds the old block.
    let mut posted = hub.broadcast(1, &[Maintenance::Code(None)]);
    let posted_serial = posted[0].1;
    assert!(hub.applied(&mut posted));
    machine.step().unwrap();
    assert_eq!(machine.cpu.pc, BASE, "nothing ran");
    assert_eq!(machine.host.exits.len(), 1);
    assert_eq!(hub.inboxes[0].active.load(Ordering::SeqCst) & 1, 0);
    // Its schedule reads the mail, and the next step runs a fresh block.
    machine.read_mail().unwrap();
    assert!(!hub.pending(0));
    assert_eq!(hub.inboxes[0].applied.load(Ordering::SeqCst), posted_serial);
    machine.step().unwrap();
    assert_eq!(machine.cpu.pc, BASE + 8);
    assert!(machine.host.exits.is_empty());
    assert_eq!(hub.inboxes[0].active.load(Ordering::SeqCst) & 1, 0);
}

/// Racing posts against a PE that enters and leaves blocks: whenever the
/// broadcaster has counted a post as applied, the PE, inside any block it
/// runs, has applied it. Bounded by counts, not by time.
#[test]
fn a_pe_inside_a_block_has_applied_every_post_counted_as_applied() {
    const POSTS: u64 = 5_000;
    let (hub, _) = hub(2);
    hub.leave_active(1);
    let required = AtomicU64::new(0);
    let done = AtomicBool::new(false);
    std::thread::scope(|scope| {
        scope.spawn(|| {
            let mut applied = 0;
            let mut entries = 0u64;
            while !done.load(Ordering::SeqCst) {
                if let Some(mail) = hub.take(1) {
                    applied = mail.serial;
                    hub.acknowledge(1, mail.serial);
                    continue;
                }
                if hub.enter_active(1) {
                    entries += 1;
                    assert!(required.load(Ordering::SeqCst) <= applied);
                    for _ in 0..(entries % 16) {
                        core::hint::spin_loop();
                    }
                    assert!(required.load(Ordering::SeqCst) <= applied);
                    hub.leave_active(1);
                }
            }
        });
        for page in 1..=POSTS {
            let mut posted = hub.broadcast(0, &[Maintenance::Code(Some(page << 14))]);
            let serial = posted[0].1;
            while !hub.applied(&mut posted) {
                std::thread::yield_now();
            }
            required.store(serial, Ordering::SeqCst);
        }
        done.store(true, Ordering::SeqCst);
    });
}

/// A DSB completes natively while this PE has no maintenance outstanding,
/// in any state and across derived state changes, and exits to the engine
/// once a post or an open code page cell leaves it something to complete;
/// the engine's DSB completes that and reopens the native path.
#[test]
fn native_dsb_completes_only_without_outstanding_maintenance() {
    use super::super::mirrors::{BARRIER, CODE_PAGE};
    use super::super::tests::{BASE, FakeHost};
    use super::super::transitions::tests::run_native;
    use crate::machine::NoDevices;
    use crate::machine::exception::Pstate;
    use stela_compiler::x86_64::register_read;
    use stela_compiler::{GuestAddress, block::ExitKind};
    const DSB_ISH: u32 = 0xd503_3b9f;
    let (hub, _) = hub(2);
    let mut machine = first_of_two(&[DSB_ISH], &hub);
    machine.cpu.pstate = Pstate::reset(1);
    let thunk = register_read::barrier_thunk(GuestAddress(BASE), BARRIER);
    let native = |machine: &mut Machine<FakeHost, NoDevices>| {
        let exit = run_native(machine, &thunk.code);
        if exit == (BASE + 4, ExitKind::Fallthrough as u64) {
            true
        } else {
            assert_eq!(exit, (BASE, ExitKind::System as u64));
            false
        }
    };
    let engine = |machine: &mut Machine<FakeHost, NoDevices>| {
        machine.cpu.pc = BASE;
        machine.system_instruction(DSB_ISH).unwrap();
        assert_eq!(machine.cpu.pc, BASE + 4);
    };
    assert!(!native(&mut machine), "no DSB completed yet");
    engine(&mut machine);
    assert!(native(&mut machine));
    // Neither derived state nor the PE's state decides it.
    machine.derived_changed();
    assert!(native(&mut machine), "derived state changed");
    machine.cpu.pstate.el = 0;
    assert!(native(&mut machine), "at EL0");
    machine.cpu.pstate.el = 1;
    // Maintenance while the other PE runs no block goes at once, and it
    // will read it before it runs one: the DSB has nothing to complete.
    hub.leave_active(1);
    machine.post(Maintenance::Code(None));
    assert!(machine.outgoing.is_empty() && machine.posted.is_empty());
    assert!(hub.take(1).is_some());
    assert!(native(&mut machine), "nobody to wait for");
    // Maintenance while it runs a block: the DSB must send it and wait.
    assert!(hub.enter_active(1));
    machine.post(Maintenance::Code(None));
    assert!(!native(&mut machine), "maintenance posted");
    assert!(hub.take(1).is_none(), "kept for the DSB");
    hub.set_live(1, false);
    engine(&mut machine);
    assert!(machine.outgoing.is_empty() && machine.posted.is_empty());
    assert!(native(&mut machine));
    hub.set_live(1, true);
    // A code page cell the DSB that ends the run of lines must close.
    machine.mirrors.open_code_page(0x4000);
    assert!(!native(&mut machine), "code page cell open");
    engine(&mut machine);
    assert_eq!(machine.mirrors[CODE_PAGE].readable, 0);
    assert!(native(&mut machine));
}

/// Host work a guest exit's device call left (a storage doorbell's disk
/// step) goes to a PE that sleeps waiting for an interrupt, and the caller
/// goes on with its guest; a PE asleep in WFE is not woken for it, and with
/// no PE to take it the caller does the work itself.
#[test]
fn released_host_work_goes_to_a_pe_that_sleeps_waiting_for_an_interrupt() {
    #[derive(Default)]
    struct Work {
        kicked: Mutex<Vec<usize>>,
        due: AtomicBool,
        done: AtomicUsize,
    }
    struct Released(Arc<Work>);
    impl Kick for Released {
        fn kick(&self, pe: usize) {
            self.0.kicked.lock().unwrap().push(pe);
        }
        fn released(&self) -> bool {
            if self.0.due.swap(false, Ordering::SeqCst) {
                self.0.done.fetch_add(1, Ordering::SeqCst);
            }
            false
        }
        fn release_due(&self) -> bool {
            self.0.due.load(Ordering::SeqCst)
        }
    }
    let work = Arc::new(Work::default());
    let hub = Hub::new(3, Released(Arc::clone(&work)));
    for pe in 0..3 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Pending::default(), Arc::clone(&hub));
    let mut worker = home.worker();
    // PE 1 is in a guest exit; PE 2 sleeps in WFE.
    hub.inboxes[1].outside.store(true, Ordering::SeqCst);
    hub.inboxes[2].sleeping.store(true, Ordering::SeqCst);
    work.due.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work.done.load(Ordering::SeqCst), 1, "the caller did it");
    assert!(work.kicked.lock().unwrap().is_empty());
    assert!(!hub.service.load(Ordering::SeqCst));
    let releases = |home: &SharedDevices<Pending>| {
        let times = home.lock_times()[Call::Access as usize];
        (times.stepped, times.handed)
    };
    assert_eq!(releases(&home), (1, 0), "counted as done on the caller");
    assert_eq!(home.board.released[1].load(Ordering::SeqCst), 0, "no ticks");
    // PE 2 now sleeps waiting for an interrupt: it is woken for the work.
    hub.inboxes[2].serves.store(true, Ordering::SeqCst);
    work.due.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work.done.load(Ordering::SeqCst), 1, "the caller went on");
    assert!(work.due.load(Ordering::SeqCst), "left for the service");
    assert_eq!(*work.kicked.lock().unwrap(), [2]);
    assert!(hub.take_service());
    assert_eq!(releases(&home), (1, 1), "counted as handed off");
}

/// Released host work that only starts commands (a doorbell write) runs
/// on the releasing PE at once, even with a PE asleep to take it: its wake
/// would cost more, and the command would start only after it.
#[test]
fn released_work_that_only_starts_commands_runs_on_the_releasing_pe() {
    #[derive(Default)]
    struct Work {
        kicked: Mutex<Vec<usize>>,
        due: AtomicBool,
        inline: AtomicBool,
        done: AtomicUsize,
    }
    struct Released(Arc<Work>);
    impl Kick for Released {
        fn kick(&self, pe: usize) {
            self.0.kicked.lock().unwrap().push(pe);
        }
        fn released(&self) -> bool {
            if self.0.due.swap(false, Ordering::SeqCst) {
                self.0.done.fetch_add(1, Ordering::SeqCst);
            }
            false
        }
        fn release_due(&self) -> bool {
            self.0.due.load(Ordering::SeqCst)
        }
        fn release_inline(&self) -> bool {
            self.0.inline.load(Ordering::SeqCst)
        }
    }
    let work = Arc::new(Work::default());
    let hub = Hub::new(3, Released(Arc::clone(&work)));
    for pe in 0..3 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Pending::default(), Arc::clone(&hub));
    let mut worker = home.worker();
    let releases = |home: &SharedDevices<Pending>| {
        let times = home.lock_times()[Call::Access as usize];
        (times.stepped, times.handed)
    };
    // PE 1 is in a guest exit; PE 2 sleeps waiting for an interrupt.
    hub.inboxes[1].outside.store(true, Ordering::SeqCst);
    hub.inboxes[2].sleeping.store(true, Ordering::SeqCst);
    hub.inboxes[2].serves.store(true, Ordering::SeqCst);
    work.due.store(true, Ordering::SeqCst);
    work.inline.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work.done.load(Ordering::SeqCst), 1, "started on the caller");
    assert!(work.kicked.lock().unwrap().is_empty(), "no PE woken");
    assert!(!hub.service.load(Ordering::SeqCst));
    assert_eq!(releases(&home), (1, 0));
    // Work that also takes completions still goes to the sleeping PE.
    work.due.store(true, Ordering::SeqCst);
    work.inline.store(false, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work.done.load(Ordering::SeqCst), 1);
    assert_eq!(*work.kicked.lock().unwrap(), [2]);
    assert_eq!(releases(&home), (1, 1));
}

/// A host interrupt a busy PE took at its release (a host disk's completion
/// signal) wakes a PE that sleeps waiting for an interrupt, at that release
/// on the model clock rather than at the busy PE's slice end; with none
/// asleep the busy PE leaves it to the host's service, and work its own
/// release handed on already is not handed twice.
#[test]
fn a_busy_pes_completion_signal_wakes_a_sleeping_pe_at_its_release() {
    #[derive(Default)]
    struct Signals {
        kicked: Mutex<Vec<(usize, u64)>>,
        signal: AtomicBool,
        due: AtomicBool,
        now: AtomicU64,
    }
    struct Signalled(Arc<Signals>);
    impl Kick for Signalled {
        fn kick(&self, pe: usize) {
            let now = self.0.now.load(Ordering::SeqCst);
            self.0.kicked.lock().unwrap().push((pe, now));
        }
        fn ticks(&self) -> u64 {
            self.0.now.load(Ordering::SeqCst)
        }
        fn release_due(&self) -> bool {
            self.0.due.load(Ordering::SeqCst)
        }
        fn signal_due(&self) -> bool {
            self.0.signal.swap(false, Ordering::SeqCst)
        }
    }
    let signals = Arc::new(Signals::default());
    let hub = Hub::new(3, Signalled(Arc::clone(&signals)));
    for pe in 0..3 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Pending::default(), Arc::clone(&hub));
    let mut worker = home.worker();
    let handed = |home: &SharedDevices<Pending>| {
        let times = home.lock_times()[Call::Access as usize];
        (times.handed, times.signals)
    };
    // PE 1 runs its guest; nobody sleeps: the signal waits for the service.
    signals.now.store(1_000, Ordering::SeqCst);
    signals.signal.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 1_000), Ok(0));
    assert!(signals.kicked.lock().unwrap().is_empty());
    assert!(!hub.service.load(Ordering::SeqCst));
    // PE 2 sleeps waiting for an interrupt: the next signal wakes it at the
    // release that took it, at 2_500, not at PE 1's slice end.
    hub.inboxes[2].sleeping.store(true, Ordering::SeqCst);
    hub.inboxes[2].serves.store(true, Ordering::SeqCst);
    signals.now.store(2_500, Ordering::SeqCst);
    signals.signal.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 2_500), Ok(0));
    assert_eq!(*signals.kicked.lock().unwrap(), [(2, 2_500)]);
    assert!(hub.take_service());
    assert_eq!(handed(&home), (0, 1));
    // A release that handed its own work on already wakes nobody twice.
    hub.inboxes[1].outside.store(true, Ordering::SeqCst);
    signals.due.store(true, Ordering::SeqCst);
    signals.signal.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 3_000), Ok(0));
    assert_eq!(signals.kicked.lock().unwrap().len(), 2);
    assert_eq!(handed(&home), (1, 1));
}

/// With no PE asleep, a release outside the run loop takes a completion
/// signal's step itself, once for the signal; a stream of signals gets at
/// most one step a release, none at a release that did its own host work
/// already, and none from a PE inside its run loop.
#[test]
fn a_stream_of_signals_gets_at_most_one_step_a_release() {
    #[derive(Default)]
    struct Signals {
        kicked: Mutex<Vec<usize>>,
        due: AtomicBool,
        released: AtomicUsize,
        stepped: AtomicUsize,
    }
    struct Stream(Arc<Signals>);
    impl Kick for Stream {
        fn kick(&self, pe: usize) {
            self.0.kicked.lock().unwrap().push(pe);
        }
        fn released(&self) -> bool {
            self.0.due.store(false, Ordering::SeqCst);
            self.0.released.fetch_add(1, Ordering::SeqCst);
            false
        }
        fn release_due(&self) -> bool {
            self.0.due.load(Ordering::SeqCst)
        }
        // A new signal comes before every release.
        fn signal_due(&self) -> bool {
            true
        }
        fn signal_step(&self) -> bool {
            self.0.stepped.fetch_add(1, Ordering::SeqCst);
            false
        }
    }
    let signals = Arc::new(Signals::default());
    let hub = Hub::new(3, Stream(Arc::clone(&signals)));
    for pe in 0..3 {
        hub.set_live(pe, true);
    }
    let home = SharedDevices::new(Pending::default(), Arc::clone(&hub));
    let mut worker = home.worker();
    let steps = |home: &SharedDevices<Pending>| {
        let times = home.lock_times()[Call::Access as usize];
        (times.stepped, times.signals, times.signal_steps)
    };
    let work = |signals: &Signals| {
        (
            signals.released.load(Ordering::SeqCst),
            signals.stepped.load(Ordering::SeqCst),
        )
    };
    // PE 1 runs in its loop: it leaves the signal to the host's service.
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work(&signals), (0, 0));
    // In a guest exit with nobody asleep, each release steps once.
    hub.inboxes[1].outside.store(true, Ordering::SeqCst);
    for release in 1..=3 {
        assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
        assert_eq!(work(&signals), (0, release));
    }
    assert_eq!(steps(&home), (0, 0, 3));
    // A release that did its own host work takes no signal step too.
    signals.due.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work(&signals), (1, 3));
    assert_eq!(steps(&home), (1, 0, 3));
    // A PE asleep to take it is woken instead.
    hub.inboxes[2].sleeping.store(true, Ordering::SeqCst);
    hub.inboxes[2].serves.store(true, Ordering::SeqCst);
    assert_eq!(worker.read(1, 0x40, 8, 0), Ok(0));
    assert_eq!(work(&signals), (1, 3));
    assert_eq!(*signals.kicked.lock().unwrap(), [2]);
    assert_eq!(steps(&home), (1, 1, 3));
}

/// A core whose halt waits on a doorbell never polls before it halts; any
/// other polls as Linux's haltpoll governor fits it, at most `most`.
#[test]
fn the_halt_poll_is_none_on_a_doorbell_and_bounded_elsewhere() {
    let most = 200;
    for (poll, blocked) in [(0, 10), (200, 10), (100, 1000), (0, 1000)] {
        assert_eq!(idle_poll_after(poll, blocked, most, true), 0);
    }
    // A short wait that halted doubles the poll, to at least a quarter.
    assert_eq!(idle_poll_after(0, 10, most, false), 50);
    assert_eq!(idle_poll_after(50, 10, most, false), 100);
    assert_eq!(idle_poll_after(200, 10, most, false), 200);
    // Long waits halve it, and below a quarter it stops.
    assert_eq!(idle_poll_after(200, 1000, most, false), 100);
    assert_eq!(idle_poll_after(50, 1000, most, false), 0);
}

/// A WFE that waits in the engine, taking no trap, opens its state's cell
/// for native WFE to spin in, only with PEs on cores of their own; one that
/// traps opens none, and a change of derived state closes it. What native
/// spins counted reaches the PE's levels as native code returns.
#[test]
fn engine_waits_open_native_event_spins_only_where_other_pes_run() {
    use super::super::mirrors::{EVENT_WAITS, mirror_state};
    use super::super::tests::{BASE, machine};
    use crate::machine::exception::Pstate;
    use crate::machine::system::Register as SystemRegister;
    use stela_compiler::x86_64::register_read::EventSpin;
    const WFE: u32 = 0xd503_205f;
    const HCR_TWE: u64 = 1 << 14;
    let el1 = mirror_state(1, false);
    let mut machine = machine(&[WFE]);
    machine.cpu.pstate = Pstate::reset(1);
    machine.derived_changed();
    let wait = |machine: &mut Machine<_, _>| {
        machine.waiting = None;
        machine.cpu.pc = BASE;
        machine.system_instruction(WFE).unwrap();
    };
    wait(&mut machine);
    assert!(machine.waiting.is_some());
    assert_eq!(
        machine.mirrors.banks[el1][EVENT_WAITS].readable, 0,
        "serialized"
    );
    let (hub, _) = hub(1);
    machine.enter_parallel(hub).unwrap();
    wait(&mut machine);
    assert!(machine.waiting.is_some());
    assert_eq!(machine.mirrors.banks[el1][EVENT_WAITS].readable, 1);
    machine.derived_changed();
    assert_eq!(machine.mirrors.banks[el1][EVENT_WAITS].readable, 0);
    let hcr = machine.cpu.read(SystemRegister::HCR_EL2);
    machine
        .cpu
        .registers
        .set(SystemRegister::HCR_EL2, 0, hcr | HCR_TWE);
    machine.derived_changed();
    wait(&mut machine);
    assert!(machine.waiting.is_none());
    assert_eq!(machine.cpu.pstate.el, 2, "trapped");
    assert_eq!(machine.mirrors.banks[el1][EVENT_WAITS].readable, 0);

    machine.set_step_timing(true);
    machine.mirrors.spin = EventSpin {
        completed: 2,
        missed: 3,
        held: 1,
        counts: 96,
        start: 7,
        turns: 9,
    };
    machine.take_event_spins();
    let levels = machine.step_timing().unwrap().levels;
    assert_eq!(
        [
            levels.wfe_spun,
            levels.wfe_spin_missed,
            levels.wfe_held,
            levels.wfe_spin_counts
        ],
        [2, 3, 1, 96]
    );
    assert_eq!(machine.mirrors.spin, EventSpin::default());
}

/// With the board's fast-IPI table named to native code, a plain fast-IPI
/// write the engine completes opens its state's cell for native writes,
/// which a change of derived state closes; the PEs native requests found
/// halted are woken through the board when native code returns.
#[test]
fn plain_fast_ipi_writes_open_native_ones_and_native_wakes_reach_the_board() {
    use super::super::mirrors::{FAST_IPI_WRITES, mirror_state};
    use super::super::tests::board;
    use stela_contracts::identity::Identity;
    const MSR: u32 = 0xd51d_f029; // msr s3_5_c15_c0_1, x9
    let (hub, kicks) = hub(2);
    let worker = SharedDevices::new(Signalling::default(), Arc::clone(&hub)).worker();
    let identities = vec![Identity {
        midr: 0x611f_0320,
        mpidr: 0x8000_0000,
        clock: None,
        caches: None,
    }];
    let mut machine = board(&[MSR], worker, identities);
    let state = mirror_state(machine.cpu.pstate.el, machine.cpu.pstate.guarded);
    let write = |machine: &mut Machine<_, _>| {
        let pc = machine.cpu.pc;
        machine.cpu.state.x[9] = 1;
        machine.system_instruction(MSR).unwrap();
        assert_eq!(machine.cpu.pc, pc + 4);
        machine.cpu.pc = pc;
    };
    write(&mut machine);
    assert_eq!(
        machine.mirrors.banks[state][FAST_IPI_WRITES].readable, 0,
        "no table named"
    );
    assert_eq!(machine.enter_fast_ipi(), Some(()));
    let ipi = machine.mirrors.fast_ipi;
    let table = machine.devices.fast_ipi_table().unwrap();
    assert_eq!((ipi.base, ipi.pe), (table.base(), 0));
    assert_eq!(ipi.status, table.target(0, 0).unwrap().status);
    write(&mut machine);
    assert_eq!(machine.mirrors.banks[state][FAST_IPI_WRITES].readable, 1);
    machine.derived_changed();
    assert_eq!(machine.mirrors.banks[state][FAST_IPI_WRITES].readable, 0);
    hub.inboxes[1].sleeping.store(true, Ordering::SeqCst);
    machine.mirrors.fast_ipi.wakes = 1 << 1;
    machine.take_fast_ipi_wakes();
    assert_eq!(kicks.load(Ordering::Relaxed), 1);
    assert_eq!(machine.mirrors.fast_ipi.wakes, 0);
}

/// A register a read leaves plain, as an event register whose last event the
/// read took: the device that raises events at 0x50 moves its epoch, as a
/// bus does, before the event is there to read at 0x40.
#[derive(Default)]
struct Events {
    epoch: Arc<AtomicU64>,
    pending: u64,
    raised: u64,
    reads: usize,
}
impl Devices for Events {
    fn plain_epoch(&self, physical: u64, bytes: u8) -> Option<(Arc<AtomicU64>, u64)> {
        (physical == 0x40 && bytes == 4 && self.pending == 0)
            .then(|| (Arc::clone(&self.epoch), self.epoch.load(Ordering::SeqCst)))
    }
    fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
        self.reads += 1;
        if self.pending == 0 {
            return Ok(0);
        }
        // Taking an event changes the model: the epoch moves first.
        self.epoch.fetch_add(1, Ordering::SeqCst);
        self.pending -= 1;
        Ok(self.raised - self.pending)
    }
    fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
        self.epoch.fetch_add(1, Ordering::SeqCst);
        self.pending += 1;
        self.raised += 1;
        Ok(())
    }
    fn lines(&mut self, _: usize, _: u64) -> Lines {
        Lines::default()
    }
}

/// The read that takes the last event leaves the next read served without
/// the lock; an event raised since reaches the next read.
#[test]
fn a_read_that_leaves_its_register_plain_serves_the_next_read_without_the_lock() {
    let (hub, _) = hub(2);
    let home = SharedDevices::new(Events::default(), hub);
    let mut worker = home.worker();
    let mut other = home.worker();
    let reads = |home: &SharedDevices<Events>| home.with(|devices| devices.reads);
    other.write(0, 0x50, 4, 0, 0).unwrap();
    assert_eq!(worker.read_scalar(1, 0x40, 4, 1), ScalarRead::Served(Ok(1)));
    // The claim and the read after it under the same lock.
    assert_eq!(reads(&home), 2);
    assert_eq!(worker.read_scalar(1, 0x40, 4, 2), ScalarRead::Served(Ok(0)));
    assert_eq!(reads(&home), 2, "served without the devices");
    // An event raised since moved the epoch: the next read takes it.
    other.write(0, 0x50, 4, 0, 3).unwrap();
    assert_eq!(worker.read_scalar(1, 0x40, 4, 4), ScalarRead::Served(Ok(2)));
    // Two pending: the first claim leaves the register not plain.
    other.write(0, 0x50, 4, 0, 5).unwrap();
    other.write(0, 0x50, 4, 0, 5).unwrap();
    let before = reads(&home);
    assert_eq!(worker.read_scalar(1, 0x40, 4, 6), ScalarRead::Served(Ok(3)));
    assert_eq!(reads(&home), before + 1);
    assert_eq!(worker.read_scalar(1, 0x40, 4, 7), ScalarRead::Served(Ok(4)));
    assert_eq!(worker.read_scalar(1, 0x40, 4, 8), ScalarRead::Served(Ok(0)));
    assert_eq!(reads(&home), before + 3);
}

/// Events raised on one core while another reads: no event is lost or
/// taken twice, and a read that began after a raise returned never finds
/// none while that event is untaken.
#[test]
fn events_raised_while_their_register_is_served_plain_are_never_lost_or_late() {
    const EVENTS: u64 = 20_000;
    let (hub, _) = hub(2);
    let home = SharedDevices::new(Events::default(), hub);
    let raised = Arc::new(AtomicU64::new(0));
    let mut raiser = home.worker();
    let mut reader = home.worker();
    let raising = {
        let raised = Arc::clone(&raised);
        std::thread::spawn(move || {
            for _ in 0..EVENTS {
                raiser.write(0, 0x50, 4, 0, 0).unwrap();
                raised.fetch_add(1, Ordering::SeqCst);
            }
        })
    };
    let mut taken = 0u64;
    while taken < EVENTS {
        let before = raised.load(Ordering::SeqCst);
        match reader.read_scalar(1, 0x40, 4, 0) {
            ScalarRead::Served(Ok(0)) => assert!(
                taken >= before,
                "a read found none with {} raised before it untaken",
                before - taken
            ),
            ScalarRead::Served(Ok(event)) => {
                taken += 1;
                assert_eq!(event, taken, "events are taken once, in order");
            }
            other => panic!("{other:?}"),
        }
    }
    raising.join().unwrap();
    assert_eq!(reader.read_scalar(1, 0x40, 4, 0), ScalarRead::Served(Ok(0)));
    assert_eq!(home.with(|devices| devices.pending), 0);
}

/// Counts its kicks and reads a counter the test moves, as a host's
/// doorbells and its 24 MHz count do.
struct Served {
    kicks: Arc<AtomicUsize>,
    host: Arc<AtomicU64>,
}
impl Kick for Served {
    fn kick(&self, _pe: usize) {
        self.kicks.fetch_add(1, Ordering::SeqCst);
    }
    fn count(&self) -> Option<u64> {
        Some(self.host.load(Ordering::SeqCst))
    }
}

/// A counter frequency at which the stall limit is 1200 counts.
const SERVED_FREQUENCY: u64 = 24_000;
const SERVED_STALL: u64 = SERVED_FREQUENCY * super::super::clock::STALL_MILLIS / 1000;

type Clock = super::super::clock::Clock;

/// Two PEs on a hub whose shared count follows a host counter from 1000,
/// each with its own reader; both run.
fn serving_pair() -> (Arc<Hub>, Arc<AtomicUsize>, Arc<AtomicU64>, Clock, Clock) {
    let kicks = Arc::new(AtomicUsize::new(0));
    let host = Arc::new(AtomicU64::new(1000));
    let hub = Hub::new(
        2,
        Served {
            kicks: Arc::clone(&kicks),
            host: Arc::clone(&host),
        },
    );
    for pe in 0..2 {
        hub.set_live(pe, true);
    }
    Clock::default().seed(&hub.clock, 1000, SERVED_FREQUENCY);
    let (mut first, mut second) = (Clock::default(), Clock::default());
    first.attach(Arc::clone(&hub.clock), 0);
    second.attach(Arc::clone(&hub.clock), 1);
    (hub, kicks, host, first, second)
}

/// The reported shape: PE 1's guest waits for an interrupt and its core
/// serves the host in a part that takes seconds, while PE 0 runs and sends
/// it a fast IPI. The IPI kicks its core at once, the service sees it at
/// its next part, and the count PE 0 reads goes on at most a stall limit
/// past the IPI however long the part takes: PE 0's XNU never sees PE 1
/// leave an IPI unanswered for longer. Once PE 1 looks again, the count
/// goes on from there and the held time is lag the slew repays.
#[test]
fn an_ipi_to_a_pe_that_serves_the_host_kicks_it_and_holds_the_count_for_it() {
    let (hub, kicks, host, mut first, mut second) = serving_pair();
    first.idle(|| 1000, 0, None);
    first.busy(|| 1000, 0, None);
    second.idle(|| 1000, 0, None);
    // PE 1 starts a service turn with nothing pending.
    let epoch = hub.lines_epoch(1);
    assert_eq!(hub.set_serving(1, Some(epoch)), None);
    let watch = GuestWatch::new(Some(Arc::clone(&hub)), 1, epoch, None);
    assert_eq!(watch.pending(1000), None);
    // A wake with no guest work for it (a SEV, which a WFI does not take)
    // neither kicks nor marks it.
    hub.event(0);
    assert_eq!(kicks.load(Ordering::SeqCst), 0);
    // PE 0's fast IPI at host 2000: one kick, and the turn's next look
    // finds it.
    host.store(2000, Ordering::SeqCst);
    hub.lines_changed_unwoken(1);
    hub.wake(1);
    assert_eq!(kicks.load(Ordering::SeqCst), 1, "the serving PE is kicked");
    assert_eq!(watch.pending(2000), Some(super::Pending::Lines));
    // A second IPI meanwhile kicks it no more.
    hub.lines_changed_unwoken(1);
    hub.wake(1);
    assert_eq!(kicks.load(Ordering::SeqCst), 1);
    // PE 1 stays out for five seconds of host time; PE 0 runs and marks
    // its progress every 10 counts.
    let ipi = 2000;
    let mut retired = 0;
    let end = 2000 + 5 * SERVED_FREQUENCY;
    for now in (2010..end).step_by(10) {
        retired += 100;
        first.mark(|| now, retired, None);
        let count = first.read(|| now);
        assert!(count <= ipi + SERVED_STALL, "at {now}: count {count}");
    }
    assert_eq!(
        first.read(|| end),
        ipi + SERVED_STALL,
        "held a stall limit on"
    );
    assert_eq!(
        watch.waited(end),
        Some((super::Pending::Lines, Some(end - ipi))),
        "the wait is measured from the first wake"
    );
    // PE 1 is back in its loop: the count goes on from where it held.
    assert_eq!(hub.set_serving(1, None), Some(epoch));
    second.idle(|| end, 0, None);
    first.mark(|| end + 10, retired + 100, None);
    assert_eq!(first.read(|| end + 10), ipi + SERVED_STALL + 10);
    assert_eq!(hub.clock.take_stalled(), end - (ipi + SERVED_STALL));
    // A PE that neither serves nor sleeps is not kicked: it reads its
    // lines at its next return to the engine.
    hub.lines_changed_unwoken(1);
    hub.wake(1);
    assert_eq!(kicks.load(Ordering::SeqCst), 1);
}

/// PE 1's own timer comes due while it serves: the service sees it at its
/// next part, and the count holds a stall limit past the deadline, as for
/// an IPI. Its next wait pass clears the mark.
#[test]
fn a_timer_due_on_a_pe_that_serves_the_host_ends_its_service_and_holds_the_count() {
    let (hub, kicks, _host, mut first, mut second) = serving_pair();
    first.idle(|| 1000, 0, None);
    first.busy(|| 1000, 0, None);
    second.idle(|| 1000, 0, None);
    let deadline = 3000;
    second.serving(Some(deadline));
    let epoch = hub.lines_epoch(1);
    hub.set_serving(1, Some(epoch));
    let watch = GuestWatch::new(Some(Arc::clone(&hub)), 1, epoch, Some(deadline));
    assert_eq!(watch.pending(2999), None);
    assert_eq!(watch.pending(3000), Some(super::Pending::Timer));
    assert_eq!(watch.waited(3500), Some((super::Pending::Timer, Some(500))));
    // Before its deadline it holds nothing; past it a stall limit on.
    let mut retired = 0;
    for now in (1010..10_000).step_by(10) {
        retired += 100;
        first.mark(|| now, retired, None);
        assert_eq!(
            first.read(|| now),
            now.min(deadline + SERVED_STALL),
            "at {now}"
        );
    }
    assert_eq!(
        kicks.load(Ordering::SeqCst),
        0,
        "its own timer needs no kick"
    );
    hub.set_serving(1, None);
    second.look(|| 10_000);
    first.mark(|| 10_010, retired + 100, None);
    assert_eq!(first.read(|| 10_010), deadline + SERVED_STALL + 10);
}

/// Guest work already there when a PE begins to serve marks the count, as
/// a wake would, without kicking its own core; a forwarded access waits
/// only for the home machine, a stop only for the others.
#[test]
fn guest_work_before_a_service_begins_marks_the_count_without_a_kick() {
    let (hub, kicks, host, _first, mut second) = serving_pair();
    second.idle(|| 1000, 0, None);
    let epoch = hub.lines_epoch(1);
    host.store(1500, Ordering::SeqCst);
    hub.lines_changed_unwoken(1);
    hub.set_serving(1, Some(epoch));
    assert_eq!(kicks.load(Ordering::SeqCst), 0);
    let watch = GuestWatch::new(Some(Arc::clone(&hub)), 1, epoch, None);
    assert_eq!(watch.waited(1600), Some((super::Pending::Lines, Some(100))));
    hub.set_serving(1, None);
    let epoch = hub.lines_epoch(1);
    hub.requested.store(true, Ordering::SeqCst);
    assert_eq!(hub.guest_pending(1, epoch), None, "PE 1 is not home");
    assert_eq!(
        hub.guest_pending(0, hub.lines_epoch(0)),
        Some(super::Pending::Forward)
    );
    hub.requested.store(false, Ordering::SeqCst);
    hub.stopped.store(true, Ordering::SeqCst);
    assert_eq!(hub.guest_pending(1, epoch), Some(super::Pending::Stop));
    assert_eq!(
        hub.guest_pending(0, hub.lines_epoch(0)),
        None,
        "home runs on"
    );
}
