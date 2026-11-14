use super::*;
use alloc::string::ToString;
use core::sync::atomic::AtomicU64;

static NOW: AtomicU64 = AtomicU64::new(0);

fn clock() -> u64 {
    NOW.load(Ordering::Relaxed)
}

fn at(tick: u64) {
    NOW.store(tick, Ordering::Relaxed);
}

#[test]
fn every_phase_has_one_slot_within_the_limit() {
    for operation in Timed::ALL {
        let phases = operation.phases();
        assert!(phases.len() <= PHASES);
        for (slot, phase) in phases.iter().enumerate() {
            assert_eq!(phase.place(), (operation, slot));
        }
    }
}

#[test]
fn a_call_counts_its_own_phases_only() {
    let mut recorder = Recorder::default();
    recorder.set_clock(clock);
    at(100);
    recorder.begin(Timed::NewTexture);
    at(110);
    let start = recorder.since(Phase::TexturePlan, 100);
    assert_eq!(start, 110);
    at(150);
    recorder.since(Phase::TextureAllocate, start);
    // A commit's phase while a texture is timed is not counted.
    recorder.add(Phase::CommitSubmit, 1_000);
    recorder.add(Phase::TextureAllocate, 5);
    recorder.detail(
        Timed::Commit,
        Detail::Commit {
            stream: 1,
            uploads: 2,
        },
    );
    at(200);
    let call = recorder.take().expect("a timed call");
    assert_eq!(call.operation, Timed::NewTexture);
    assert_eq!(call.total, 100);
    assert_eq!(call.phases[0], 10);
    assert_eq!(call.phases[3], 45);
    assert_eq!(call.other(), 45);
    assert_eq!(call.detail, Detail::None);
    assert_eq!(call.longest_phase(), ("alloc", 45));
    assert_eq!(recorder.take(), None);
}

#[test]
fn without_a_call_nothing_is_counted() {
    let mut recorder = Recorder::default();
    recorder.add(Phase::BytesCopy, 7);
    assert!(!recorder.timing(Timed::GetBytes));
    assert_eq!(recorder.take(), None);
}

#[test]
fn totals_sum_calls_and_limit_slow_lines() {
    let totals = Totals::new();
    let mut call = CallPhases::new(Timed::GetBytes);
    call.total = 300;
    call.phases[1] = 200;
    for _ in 0..SLOW_SHOWN {
        assert!(totals.add(&call, 300));
    }
    assert!(!totals.add(&call, 300));
    call.total = 100;
    assert!(!totals.add(&call, 300));
    // A zero threshold shows none.
    assert!(!totals.add(&call, 0));
    let taken: alloc::vec::Vec<_> = totals.take().collect();
    assert_eq!(taken.len(), 1);
    let taken = taken[0];
    let count = SLOW_SHOWN + 3;
    assert_eq!(taken.operation, Timed::GetBytes);
    assert_eq!(taken.count, count);
    assert_eq!(taken.total, 300 * (SLOW_SHOWN + 1) + 200);
    assert_eq!(taken.longest, 300);
    assert_eq!(taken.phases[1], 200 * count);
    assert_eq!(taken.slow, SLOW_SHOWN + 1);
    assert_eq!(taken.other(), taken.total - 200 * count);
    // The next period starts empty and shows slow lines again.
    assert_eq!(totals.take().count(), 0);
    call.total = 300;
    assert!(totals.add(&call, 300));
}

#[test]
fn the_phases_line_names_every_phase() {
    let taken = Taken {
        operation: Timed::NewTexture,
        count: 2,
        total: 9_000,
        longest: 6_000,
        phases: [1_000, 0, 2_000, 3_000, 0, 1_000, 0, 1_000],
        slow: 1,
    };
    assert_eq!(
        taken.line(1_000_000).to_string(),
        "GPU PHASES new_texture count=2 total_us=9000 plan_us=1000 recycle_us=0 create_us=2000 \
         alloc_us=3000 bind_us=0 views_us=1000 descriptors_us=0 cpu_copy_us=1000 other_us=1000 \
         longest_us=6000 slow=1"
    );
}

#[test]
fn the_slow_line_names_the_longest_phase_and_the_work() {
    let mut call = CallPhases::new(Timed::NewRenderPipeline);
    call.total = 5_000_000;
    call.phases = [100_000, 1_000_000, 500_000, 0, 3_000_000, 0, 0, 0];
    call.detail = Detail::Pipeline {
        vertex: Name::new("vertex_main"),
        fragment: None,
    };
    assert_eq!(
        call.slow_line(1_000_000_000).to_string(),
        "GPU SLOW new_render_pipeline took_us=5000 most=create/3000us specialize_us=100 \
         vertex_us=1000 fragment_us=500 create_us=3000 other_us=400 vertex=vertex_main fragment=-"
    );
    let mut call = CallPhases::new(Timed::NewTexture);
    call.total = 10;
    call.detail = Detail::Texture {
        format: PixelFormat::Bgra8Unorm,
        size: [4096, 2048, 1],
        slices: 1,
        levels: 1,
        bytes: 1 << 25,
        recycled: false,
    };
    assert_eq!(
        call.slow_line(1_000_000).to_string(),
        "GPU SLOW new_texture took_us=10 most=other/10us other_us=10 format=Bgra8Unorm \
         size=4096x2048x1 slices=1 levels=1 bytes=33554432 recycled=false"
    );
}

#[test]
fn a_name_is_cut_at_a_character() {
    let long = "é".repeat(30);
    let name = Name::new(&long);
    assert_eq!(name.as_str(), "é".repeat(20));
}

#[test]
fn the_exit_line_checks_every_exit_against_the_target() {
    // A clock of a million ticks a second: a tick is a microsecond.
    const HZ: u64 = 1_000_000;
    let watch = ExitWatch::new();
    assert_eq!(watch.take(), None, "no exits, no line");
    watch.note(EXIT_TARGET_MICROS - 1, 0x203, false, HZ);
    watch.note(300, 0x40b, true, HZ);
    let exits = watch.take().unwrap();
    assert!(exits.within_target());
    assert_eq!(
        exits.line(HZ).to_string(),
        "GPU EXIT longest_us=4999@0x203 over=0/2 target_us=5000 check=ok gave_up=1"
    );
    // The next period starts empty; one exit at the target marks it.
    watch.note(EXIT_TARGET_MICROS, 0x103, false, HZ);
    watch.note(138_606, 0x40b, false, HZ);
    watch.note(10, 0x3, false, HZ);
    let exits = watch.take().unwrap();
    assert!(!exits.within_target());
    assert_eq!(
        exits.line(HZ).to_string(),
        "GPU EXIT longest_us=138606@0x40b over=2/3 target_us=5000 check=over gave_up=0"
    );
}

#[test]
fn a_long_copy_is_reported_each_time_and_other_phases_once() {
    let mut call = CallPhases::new(Timed::GetBytes);
    call.total = 300;
    // The copy took longest.
    call.phases[1] = 200;
    call.phases[4] = 50;
    assert_eq!(call.longest_phase(), ("copy", 200));
    assert!(call.reported_each_time());
    // The write took longest.
    call.phases[4] = 250;
    assert_eq!(call.longest_phase(), ("write", 250));
    assert!(!call.reported_each_time());
    // Time no phase accounts for.
    call.total = 2000;
    assert_eq!(call.longest_phase().0, "other");
    assert!(!call.reported_each_time());
    let mut commit = CallPhases::new(Timed::Commit);
    commit.total = 10;
    commit.phases[0] = 10;
    assert!(!commit.reported_each_time());
}
