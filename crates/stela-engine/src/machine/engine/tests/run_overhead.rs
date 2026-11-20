//! Host timing of the run loop's fixed cost per native run: the schedule,
//! the step's bookkeeping, the fetch of a block the table already holds and
//! the completion of its exit, with native code that costs nothing (the
//! fake host answers each run with the next of a cycle of exits). A guest
//! loop that ends a run at every turn, by a BR back (Indirect) or by an MRS
//! the engine completes and then the BR (System), on one PE alone, on one PE
//! of a parallel hub, and on several PEs of one hub at once, each on its
//! own thread. The counter reads the time stamp counter, as a host's does.

use super::super::parallel::{Hub, Kick};
use super::*;
use stela_contracts::identity::Identity;

const MRS: u32 = 0xd538_d080; // mrs x0, tpidr_el1
const BR: u32 = 0xd61f_0020; // br x1
/// Runs a timing makes, after as many again to warm up.
const RUNS: u64 = 2_000_000;

/// A hub whose kicks wake nobody: every PE runs.
struct Quiet;
impl Kick for Quiet {
    fn kick(&self, _: usize) {}
}

/// A machine moved to the thread that runs it: the fake host's raw chain
/// pointer and callbacks keep it from being `Send`.
struct Moved(Machine<FakeHost, NoDevices>);
// SAFETY: each machine runs on one thread at a time, and its fake host's
// chain pointer and callbacks are unset.
unsafe impl Send for Moved {}

impl Moved {
    /// [`time`] on the thread that took it whole.
    fn time(mut self) -> f64 {
        time(&mut self.0)
    }
}

/// The guest loop's words, and the cycle of exits its runs end with.
fn guest(system: bool) -> ([u32; 2], Vec<NativeExit>) {
    let back = NativeExit::Exit {
        next_pc: BASE,
        kind: ExitKind::Indirect as u64,
    };
    if system {
        let mrs = NativeExit::Exit {
            next_pc: BASE,
            kind: ExitKind::System as u64,
        };
        ([MRS, BR], vec![mrs, back])
    } else {
        ([BR, BR], vec![back])
    }
}

/// Ready `machine` to loop at BASE.
fn prepare(machine: &mut Machine<FakeHost, NoDevices>, cycle: Vec<NativeExit>) {
    machine.host.cycle = cycle;
    machine.host.stamped = true;
    machine.cpu.pc = BASE;
    machine.cpu.state.x[1] = BASE;
}

/// Nanoseconds per run of `machine`'s loop over [`RUNS`] runs, after as
/// many to warm up.
fn time(machine: &mut Machine<FakeHost, NoDevices>) -> f64 {
    let blocks = machine.statistics.blocks;
    assert_eq!(machine.run(blocks + RUNS), Stop::Budget);
    let blocks = machine.statistics.blocks;
    let started = std::time::Instant::now();
    assert_eq!(machine.run(blocks + RUNS), Stop::Budget);
    let runs = machine.statistics.blocks - blocks;
    assert_eq!(runs, RUNS, "one block a run");
    started.elapsed().as_nanos() as f64 / runs as f64
}

/// `pes` machines of one parallel hub, at BASE.
fn parallel(pes: usize, system: bool) -> Vec<Machine<FakeHost, NoDevices>> {
    let (words, cycle) = guest(system);
    let identities = (0..pes as u64)
        .map(|pe| Identity {
            midr: 0x611f_0320,
            mpidr: 0x8000_0000 | pe << 8,
            clock: None,
            caches: None,
        })
        .collect::<Vec<_>>();
    let mut home = board(&words, NoDevices, identities.clone());
    let mut workers = (1..pes)
        .map(|pe| {
            let prepared = board(&words, NoDevices, identities.clone());
            home.worker_profile()
                .start(prepared.cpu, prepared.host, prepared.devices, pe)
                .unwrap()
        })
        .collect::<Vec<_>>();
    let hub = Hub::new(pes, Quiet);
    home.enter_parallel(Arc::clone(&hub)).unwrap();
    for worker in &mut workers {
        worker
            .start_parallel(Arc::clone(&hub), home.cpu.core_locks.clone(), BASE)
            .unwrap();
    }
    let mut machines = vec![home];
    machines.extend(workers);
    for machine in &mut machines {
        prepare(machine, cycle.clone());
    }
    machines
}

/// `cargo test -p stela-engine --release --lib run_loop_costs --
/// --ignored --nocapture`
#[test]
#[ignore = "manual: opt-in"]
fn run_loop_costs() {
    for system in [false, true] {
        let exit = if system { "mrs+br" } else { "br" };
        let (words, cycle) = guest(system);
        let mut machine = super::machine(&words);
        prepare(&mut machine, cycle);
        let ns = time(&mut machine);
        std::println!("RUNS exit={exit} pes=1 serial ns_per_run={ns:.1}");
        for pes in [1, 2, 4] {
            let machines = parallel(pes, system);
            let start = std::sync::Barrier::new(pes);
            let each = std::thread::scope(|scope| {
                let start = &start;
                let threads = machines
                    .into_iter()
                    .map(|machine| {
                        let moved = Moved(machine);
                        scope.spawn(move || {
                            start.wait();
                            moved.time()
                        })
                    })
                    .collect::<Vec<_>>();
                threads
                    .into_iter()
                    .map(|thread| thread.join().unwrap())
                    .collect::<Vec<_>>()
            });
            let mean = each.iter().sum::<f64>() / each.len() as f64;
            std::println!(
                "RUNS exit={exit} pes={pes} parallel ns_per_run={mean:.1} each={each:.1?}"
            );
        }
    }
}
