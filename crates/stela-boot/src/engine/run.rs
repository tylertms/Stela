//! The run loop: slices, host service between them, and the heartbeat that
//! reports the running PEs.

use super::*;

/// Exit kinds by `ExitKind` value, then native faults by cause
/// (`step_timing::fault_slot`), then System exits at device sites, then the
/// `fault-check` faults by outcome (`step_timing::Check`), as the PROFILE
/// line names them.
pub(super) const EXIT_NAMES: [&str; step_timing::EXIT_SLOTS] = [
    "fallthrough",
    "direct",
    "indirect",
    "call",
    "return",
    "trap",
    "pointer-auth",
    "clock",
    "thread-register",
    "floating-point",
    "scalable",
    "system",
    "amx",
    "fault-guest",
    "fault-device",
    "fault-check",
    "fault-watched",
    "fault-translation",
    "device-site",
    "check-tagged",
    "check-alignment",
    "check-abort",
    "check-translated",
];

/// `--profile`'s heartbeat lines: of `elapsed` ticks, the share inside the
/// engine (`sliced`) and in each of its phases, the rest of the engine's
/// time unattributed, and each exit kind's count, share and mean cost.
pub(super) fn profile_report(
    timing: &step_timing::StepTiming,
    sliced: u64,
    elapsed: u64,
    tsc_hz: u64,
) {
    let share = |ticks: u64| {
        let permille = u128::from(ticks) * 1000 / u128::from(elapsed.max(1));
        format!("{}.{}%", permille / 10, permille % 10)
    };
    let mut line = format!(
        "PROFILE seconds={} engine={} host_service={}",
        elapsed / tsc_hz.max(1),
        share(sliced),
        share(elapsed.saturating_sub(sliced)),
    );
    let mut attributed = 0u64;
    for phase in step_timing::Phase::ALL {
        let ticks = timing.phases[phase as usize];
        attributed = attributed.saturating_add(ticks);
        line.push_str(&format!(" {}={}", phase.name(), share(ticks)));
    }
    line.push_str(&format!(
        " unattributed={}\n",
        share(sliced.saturating_sub(attributed))
    ));
    output::arguments(format_args!("{line}"));
    let mut exits: Vec<(usize, (u64, u64))> = timing
        .exits
        .iter()
        .copied()
        .enumerate()
        .filter(|(_, (count, _))| *count > 0)
        .collect();
    exits.sort_by_key(|exit| core::cmp::Reverse(exit.1.1));
    let mut line = String::from("PROFILE exits");
    for (slot, (count, ticks)) in exits {
        let mean_ns =
            u128::from(ticks) * 1_000_000_000 / u128::from(count) / u128::from(tsc_hz.max(1));
        line.push_str(&format!(
            " {}={}x/{}/{}ns",
            EXIT_NAMES[slot],
            count,
            share(ticks),
            mean_ns
        ));
    }
    output::arguments(format_args!("{line}\n"));
}

/// `PES SYSTEM count/ns/closes`: every PE's System exits since the last
/// beat by key (`step_timing::system_key_name`), the eight most frequent,
/// then the sixteen costliest, each with its total handling time and the exits
/// that closed every mirror cell; `other` holds keys past the table's. Then
/// every key's exits by class (`step_timing::SystemClass`), the keys past
/// the table's as `other`; pointer and floating-point keys have lines of
/// their own.
fn system_exits_report(exits: &step_timing::SystemExits, tsc_hz: u64) {
    const SHOWN: usize = 8;
    const SHOWN_BY_TIME: usize = 16;
    let nanos = |ticks: u64| u128::from(ticks) * 1_000_000_000 / u128::from(tsc_hz.max(1));
    let item = |name: &str, exit: &step_timing::SystemExit| {
        format!(
            " {name}={}/{}/{}",
            exit.count,
            nanos(exit.ticks),
            exit.closes
        )
    };
    let (numerical, keys): (Vec<step_timing::SystemExit>, Vec<_>) = exits
        .counted()
        .copied()
        .partition(|exit| step_timing::numerical_key(exit.key));
    let (mut pointer, mut keys): (Vec<step_timing::SystemExit>, Vec<_>) = keys
        .into_iter()
        .partition(|exit| step_timing::pointer_key(exit.key));
    let mut line = String::from("PES SYSTEM count/ns/closes by_count");
    keys.sort_unstable_by_key(|exit| core::cmp::Reverse(exit.count));
    for exit in keys.iter().take(SHOWN) {
        line.push_str(&item(&step_timing::system_key_name(exit.key), exit));
    }
    if let Some(rest) = rest(&keys, SHOWN) {
        line.push_str(&item("rest", &rest));
    }
    line.push_str(" by_time");
    keys.sort_unstable_by_key(|exit| core::cmp::Reverse(exit.ticks));
    for exit in keys.iter().take(SHOWN_BY_TIME) {
        line.push_str(&item(&step_timing::system_key_name(exit.key), exit));
    }
    if exits.other.count != 0 {
        line.push_str(&item("other", &exits.other));
    }
    // Every key's exits by class, so that nothing hides past the shown
    // ones; the keys past the table's count as `other`.
    let mut classes = [step_timing::SystemExit::default(); step_timing::SystemClass::COUNT];
    let unkeyed = core::iter::once((step_timing::SystemClass::Other, &exits.other));
    let keyed = keys
        .iter()
        .map(|exit| (step_timing::SystemClass::of(exit.key), exit));
    for (class, exit) in keyed.chain(unkeyed) {
        let total = &mut classes[class as usize];
        total.count += exit.count;
        total.ticks += exit.ticks;
        total.closes += exit.closes;
    }
    line.push_str(" by_class");
    for (name, total) in step_timing::SystemClass::NAMES.iter().zip(&classes) {
        if total.count != 0 {
            line.push_str(&item(name, total));
        }
    }
    output::arguments(format_args!("{line}\n"));
    // `PES POINTER count/ns/closes`: the pointer authentication exits by
    // word, the eight most frequent.
    if !pointer.is_empty() {
        let mut line = String::from("PES POINTER count/ns/closes by_count");
        pointer.sort_unstable_by_key(|exit| core::cmp::Reverse(exit.count));
        for exit in pointer.iter().take(SHOWN) {
            line.push_str(&item(&step_timing::system_key_name(exit.key), exit));
        }
        if let Some(rest) = rest(&pointer, SHOWN) {
            line.push_str(&item("rest", &rest));
        }
        output::arguments(format_args!("{line}\n"));
    }
    numerical_exits_report(numerical, &exits.floating, tsc_hz);
    numerical_causes_report(&exits.floating_words);
    compression_report(&exits.compression, tsc_hz);
}

/// `PES WKDM compress=count ns_each whole/read/codec/write`: the WKDMC and
/// WKDMD instructions completed since the last beat, the mean nanoseconds
/// of their handling and of its parts (`step_timing::Compression`).
fn compression_report(compression: &[step_timing::Compression; 2], tsc_hz: u64) {
    if compression.iter().all(|operation| operation.count == 0) {
        return;
    }
    let mut line = String::from("PES WKDM count ns_each whole/read/codec/write");
    for (name, operation) in ["compress", "decompress"].iter().zip(compression) {
        let each = |ticks: u64| {
            u128::from(ticks) * 1_000_000_000
                / u128::from(tsc_hz.max(1))
                / u128::from(operation.count.max(1))
        };
        line.push_str(&format!(
            " {name}={} {}/{}/{}/{}",
            operation.count,
            each(operation.whole),
            each(operation.read),
            each(operation.codec),
            each(operation.write),
        ));
    }
    output::arguments(format_args!(
        "{line}
"
    ));
}

/// The exits of `exits` past the first `shown`, summed, if any.
fn rest(exits: &[step_timing::SystemExit], shown: usize) -> Option<step_timing::SystemExit> {
    let rest = exits.get(shown..).filter(|rest| !rest.is_empty())?;
    Some(step_timing::SystemExit {
        key: 0,
        count: rest.iter().map(|exit| exit.count).sum(),
        ticks: rest.iter().map(|exit| exit.ticks).sum(),
        closes: rest.iter().map(|exit| exit.closes).sum(),
    })
}

/// `PES FLOATING CAUSES`: the sixteen floating-point exit words with the
/// most exits since the last beat, each with its exits by reason
/// (`step_timing::FloatingReason`, `mode` overlapping the others), how many
/// found FPSR.IXC clear, and its exits by the classes their operands
/// showed (`step_timing::Operands`, overlapping), then the other words'
/// summed as `rest`; nothing when there were none.
fn numerical_causes_report(words: &step_timing::FloatingWords) {
    const SHOWN: usize = 16;
    if words.is_empty() {
        return;
    }
    let mut counted: Vec<&step_timing::FloatingWord> = words.counted().collect();
    counted.sort_unstable_by_key(|word| core::cmp::Reverse(word.count()));
    let join = |counts: &[u32]| {
        counts
            .iter()
            .map(|count| format!("{count}"))
            .collect::<Vec<_>>()
            .join("/")
    };
    let mut line = format!(
        "PES FLOATING CAUSES reasons:ixc_clear:operands {}:ixc_clear:{} by_count",
        step_timing::FloatingReason::NAMES.join("/"),
        step_timing::Operands::NAMES.join("/")
    );
    let mut rest = step_timing::FloatingWord::default();
    for word in counted.iter().skip(SHOWN) {
        rest.merge(word);
    }
    let rest = (rest.count() != 0).then_some(("rest".into(), &rest));
    let other = (words.other.count() != 0).then(|| ("other".into(), &words.other));
    let named = counted
        .into_iter()
        .take(SHOWN)
        .map(|word| (step_timing::system_key_name(word.key), word));
    for (name, word) in named.chain(rest).chain(other) {
        line.push_str(&format!(
            " {name}={}:{}:{}",
            join(&word.reasons),
            word.inexact_clear,
            join(&word.operands)
        ));
    }
    output::arguments(format_args!("{line}\n"));
}

/// `PES FLOATING count/ns`: every PE's floating-point exits since the last
/// beat by what their completion found (`step_timing::FloatingReason`), then
/// by instruction word, the sixteen most frequent, each with its total
/// handling time, and the other words' as `rest`; nothing when there were
/// none.
fn numerical_exits_report(
    mut words: Vec<step_timing::SystemExit>,
    reasons: &[u64; step_timing::FloatingReason::COUNT],
    tsc_hz: u64,
) {
    const SHOWN: usize = 16;
    if words.is_empty() && reasons.iter().all(|&count| count == 0) {
        return;
    }
    let nanos = |ticks: u64| u128::from(ticks) * 1_000_000_000 / u128::from(tsc_hz.max(1));
    words.sort_unstable_by_key(|exit| core::cmp::Reverse(exit.count));
    let count: u64 = words.iter().map(|exit| exit.count).sum();
    let mut line = format!("PES FLOATING count/ns exits={count} words={}", words.len());
    for (name, count) in step_timing::FloatingReason::NAMES.iter().zip(reasons) {
        line.push_str(&format!(" {name}={count}"));
    }
    line.push_str(" by_count");
    for exit in words.iter().take(SHOWN) {
        line.push_str(&format!(
            " {}={}/{}",
            step_timing::system_key_name(exit.key),
            exit.count,
            nanos(exit.ticks)
        ));
    }
    if let Some(rest) = rest(&words, SHOWN) {
        line.push_str(&format!(" rest={}/{}", rest.count, nanos(rest.ticks)));
    }
    output::arguments(format_args!("{line}\n"));
}

/// `PES DEVICE count/ns/lock_ns`: every PE's device accesses the engine
/// completed since the last beat, by 4 KiB page, the eight costliest, each
/// named by the model mapped there (`name@page`) with its total handling
/// time and the part of it spent waiting for the board's lock; `other`
/// holds pages past the table's, and `forwarded` the accesses sent to the
/// home machine with their PEs' wait for its answer. `PES DEVICE MODELS`
/// sums the pages by model, and `PES DEVICE REGISTERS` gives the sixteen
/// costliest registers by direction (`name@address:r` or `:w`, or
/// `name.REGISTER:r` summed over the model's pages where it names them).
/// `PES BARRIER`: the DSBs that completed maintenance their PE posted, the
/// share that waited for no PE, the time each took, and by what the last PE
/// they waited for was doing (native code or an exit, by slot and word), the
/// eight that took longest.
fn barrier_waits_report(waits: &step_timing::BarrierWaits, tsc_hz: u64) {
    const SHOWN: usize = 8;
    if waits.waits == 0 {
        return;
    }
    let each = |count: u64, ticks: u64| {
        u128::from(ticks) * 1_000_000_000 / u128::from(count.max(1)) / u128::from(tsc_hz.max(1))
    };
    let mut line = format!(
        "PES BARRIER waits={} quick={} ns_each={} blockers count/ns_each",
        waits.waits,
        waits.quick,
        each(waits.waits, waits.ticks)
    );
    let mut blockers: Vec<_> = waits
        .blockers
        .iter()
        .filter(|blocker| blocker.count != 0)
        .collect();
    blockers.sort_unstable_by_key(|blocker| core::cmp::Reverse(blocker.ticks));
    for blocker in blockers.into_iter().take(SHOWN) {
        let name = if blocker.key == step_timing::BLOCKER_ENTERING {
            String::from("entering")
        } else if blocker.key == step_timing::BLOCKER_RETURNED {
            String::from("returned")
        } else if blocker.key & step_timing::BLOCKER_EXIT == 0 {
            String::from("native")
        } else {
            let slot = ((blocker.key & !step_timing::BLOCKER_EXIT) >> 32) as usize;
            let word = blocker.key as u32;
            let slot_name = EXIT_NAMES.get(slot).copied().unwrap_or("exit");
            if word == 0 || word == step_timing::DEVICE_SITE_KEY {
                String::from(slot_name)
            } else {
                format!("{slot_name}.{}", step_timing::system_key_name(word))
            }
        };
        line.push_str(&format!(
            " {name}={}/{}",
            blocker.count,
            each(blocker.count, blocker.ticks)
        ));
    }
    if waits.other.count != 0 {
        line.push_str(&format!(
            " other={}/{}",
            waits.other.count,
            each(waits.other.count, waits.other.ticks)
        ));
    }
    output::arguments(format_args!("{line}\n"));
}

/// `PES REGIME`: the host's regime changes and prediction isolations, how
/// many and the time each took.
fn regime_syncs_report(regimes: &step_timing::RegimeSyncs, tsc_hz: u64) {
    if regimes.syncs == 0 {
        return;
    }
    let each = |count: u64, ticks: u64| {
        u128::from(ticks) * 1_000_000_000 / u128::from(count.max(1)) / u128::from(tsc_hz.max(1))
    };
    let mut sources = String::new();
    for (name, count) in step_timing::SyncSource::NAMES.iter().zip(regimes.sources) {
        sources.push_str(&format!(" {name}={count}"));
    }
    output::arguments(format_args!(
        "PES REGIME syncs={} ns_each={} isolations={} ns_each={} unchanged={} by{sources}\n",
        regimes.syncs,
        each(regimes.syncs, regimes.sync_ticks),
        regimes.isolations,
        each(regimes.isolations, regimes.isolation_ticks),
        regimes.unchanged,
    ));
}

fn device_exits_report<H: Host>(
    machine: &Machine<H, parallel::Board>,
    exits: &step_timing::DeviceExits,
    tsc_hz: u64,
) {
    const SHOWN: usize = 8;
    if exits.is_empty() {
        return;
    }
    let nanos = |ticks: u64| u128::from(ticks) * 1_000_000_000 / u128::from(tsc_hz.max(1));
    let lock_nanos = |ticks: u64| u128::from(ticks) * 1_000_000_000 / u128::from(host::COUNTER_HZ);
    let mut pages: Vec<step_timing::DeviceExit> = exits.counted().copied().collect();
    pages.sort_unstable_by_key(|exit| core::cmp::Reverse(exit.ticks));
    let count: u64 = pages.iter().map(|exit| exit.count).sum::<u64>() + exits.other.count;
    let mut line = format!(
        "PES DEVICE count/ns/lock_ns accesses={count} forwarded={}/{}",
        exits.forwarded,
        nanos(exits.forwarded_ticks)
    );
    for exit in pages.iter().take(SHOWN) {
        let physical = u64::from(exit.page) << 12;
        let name = machine
            .devices
            .with(|devices| stela_engine::machine::Devices::device_name(devices, physical));
        line.push_str(&format!(
            " {}@{physical:#x}={}/{}/{}",
            name.unwrap_or("unmapped"),
            exit.count,
            nanos(exit.ticks),
            lock_nanos(exit.waited)
        ));
    }
    if exits.other.count != 0 {
        let other = exits.other;
        line.push_str(&format!(
            " other={}/{}/{}",
            other.count,
            nanos(other.ticks),
            lock_nanos(other.waited)
        ));
    }
    output::arguments(format_args!("{line}\n"));
    // The same pages by the model mapped there: a model of many pages, such
    // as a gate for each Metal connection, counts once.
    let mut models: Vec<(&'static str, u64, u64)> = Vec::new();
    for exit in &pages {
        let name = machine.devices.with(|devices| {
            stela_engine::machine::Devices::device_name(devices, u64::from(exit.page) << 12)
        });
        let name = name.unwrap_or("unmapped");
        match models.iter_mut().find(|(model, ..)| *model == name) {
            Some((_, count, ticks)) => {
                *count += exit.count;
                *ticks += exit.ticks;
            }
            None => models.push((name, exit.count, exit.ticks)),
        }
    }
    models.sort_unstable_by_key(|&(_, _, ticks)| core::cmp::Reverse(ticks));
    let mut line = String::from("PES DEVICE MODELS count/ns");
    for &(name, count, ticks) in models.iter().take(SHOWN) {
        line.push_str(&format!(" {name}={count}/{}", nanos(ticks)));
    }
    output::arguments(format_args!("{line}\n"));
    // The same accesses by register and direction, the sixteen costliest:
    // which registers translated code could complete itself. A register
    // its model names counts as `model.NAME`, summed over the model's pages
    // (a gate's CALL over every connection).
    const REGISTERS: usize = 16;
    let mut registers: Vec<(String, step_timing::DeviceRegisterExit)> = Vec::new();
    for exit in exits.counted_registers() {
        let physical = exit.register & !step_timing::DEVICE_REGISTER_WRITE;
        let direction = if exit.register & step_timing::DEVICE_REGISTER_WRITE != 0 {
            'w'
        } else {
            'r'
        };
        let (name, register) = machine.devices.with(|devices| {
            (
                stela_engine::machine::Devices::device_name(devices, physical),
                stela_engine::machine::Devices::register_name(devices, physical),
            )
        });
        let name = name.unwrap_or("unmapped");
        let label = match register {
            Some(register) => format!("{name}.{register}:{direction}"),
            None => format!("{name}@{physical:#x}:{direction}"),
        };
        match registers.iter_mut().find(|(known, _)| *known == label) {
            Some((_, sum)) => {
                sum.count += exit.count;
                sum.ticks += exit.ticks;
                sum.waited += exit.waited;
                sum.access += exit.access;
                sum.settle += exit.settle;
                sum.transfer += exit.transfer;
                sum.copies += exit.copies;
                sum.released += exit.released;
                sum.longest = sum.longest.max(exit.longest);
            }
            None => registers.push((label, *exit)),
        }
    }
    registers.sort_unstable_by_key(|(_, exit)| core::cmp::Reverse(exit.ticks));
    // A write's time also splits into its access (the lock and the model's
    // write) and the settle after it, and the settle into the transfer under
    // the lock and the deferred copies after its release; the longest exit
    // ends each entry.
    let mut line = String::from(
        "PES DEVICE REGISTERS count/ns/lock_ns[/access_ns/settle_ns/transfer_ns/copies_ns/released_ns]/longest_ns",
    );
    for (label, exit) in registers.iter().take(REGISTERS) {
        line.push_str(&format!(
            " {label}={}/{}/{}",
            exit.count,
            nanos(exit.ticks),
            lock_nanos(exit.waited)
        ));
        if label.ends_with(":w") {
            line.push_str(&format!(
                "/{}/{}/{}/{}/{}",
                nanos(exit.access),
                nanos(exit.settle),
                nanos(exit.transfer),
                nanos(exit.copies),
                lock_nanos(exit.released)
            ));
        }
        line.push_str(&format!("/{}", nanos(exit.longest)));
    }
    let other = exits.other_registers;
    if other.count != 0 {
        line.push_str(&format!(
            " other={}/{}/{}",
            other.count,
            nanos(other.ticks),
            lock_nanos(other.waited)
        ));
    }
    output::arguments(format_args!("{line}\n"));
    // A device write's exit, split ([`step_timing::DeviceWrites`]).
    let writes = exits.writes;
    if writes.count != 0 {
        let each = |ticks: u64| nanos(ticks) / u128::from(writes.count);
        let parts = writes.access + writes.settle + writes.call;
        output::arguments(format_args!(
            "PES DEVICE WRITES count={} ns_each exit={} access={} settle={} transfer={} copies={} released={} call={} rest={}\n",
            writes.count,
            each(writes.exit),
            each(writes.access),
            each(writes.settle),
            each(writes.transfer),
            each(writes.copies),
            lock_nanos(writes.released) / u128::from(writes.count),
            each(writes.call),
            each(writes.exit.saturating_sub(parts)),
        ));
    }
}

/// `PES HOT level samples=N other=M`: where native code ran since the last
/// beat, from every PE's sampled runs (`step_timing::HotBlocks`: each
/// transfer of native code equally likely), by level, the most sampled
/// blocks first as `pc:count`. EL1 and guarded PCs are offsets from the
/// sampling PE's vector base (`v+offset`, `v-offset`), which moves with the
/// kernel's slide, so runs compare across slides; EL0 PCs as they are.
/// Every [`HOT_TOTAL_BEATS`] beats, the samples since the first beat
/// ([`hot_total_report`]).
fn hot_blocks_report<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    dynamic: &mut step_timing::DynamicSites,
    total: &mut step_timing::HotBlocks,
    whole: bool,
) {
    use stela_engine::machine::engine::step_timing::Level;
    const SHOWN: usize = 24;
    let hot = parallel::hot_samples(machine);
    dynamic.add(&hot.dynamic);
    dynamic.untracked += hot.dynamic_other;
    total.merge(&hot);
    if whole {
        hot_total_report(total);
    }
    if hot.is_empty() {
        return;
    }
    for (level, name) in [
        (Level::Kernel, "kernel"),
        (Level::Guarded, "guarded"),
        (Level::User, "user"),
    ] {
        let samples = hot.samples[level as usize];
        if samples == 0 {
            continue;
        }
        let mut line = format!("PES HOT {name} samples={samples} other={}", hot.other);
        for block in hot.top(level, SHOWN) {
            push_hot_block(&mut line, level, &block);
        }
        output::arguments(format_args!("{line}\n"));
    }
}

/// Beats between [`hot_total_report`]s.
const HOT_TOTAL_BEATS: u64 = 16;

/// ` pc:count` for a [`hot_blocks_report`] line.
fn push_hot_block(line: &mut String, level: step_timing::Level, block: &step_timing::HotBlock) {
    if level == step_timing::Level::User {
        line.push_str(&format!(" {:#x}:{}", block.pc, block.count));
    } else if (block.pc as i64) >= 0 {
        line.push_str(&format!(" v+{:#x}:{}", block.pc, block.count));
    } else {
        line.push_str(&format!(
            " v-{:#x}:{}",
            block.pc.wrapping_neg(),
            block.count
        ));
    }
}

/// `PES HOT SHARE level samples=N blocks=B top24= top100= top1000= top8192=`
/// in per mille of the level's samples since the first beat, then
/// `PES HOT TOTAL level` with its most sampled blocks: how concentrated
/// native time is, which the per-beat lines cannot show. Samples that found
/// the table full count in `samples` and in no top share.
fn hot_total_report(total: &step_timing::HotBlocks) {
    use stela_engine::machine::engine::step_timing::Level;
    const SHOWN: usize = 128;
    for (level, name) in [
        (Level::Kernel, "kernel"),
        (Level::Guarded, "guarded"),
        (Level::User, "user"),
    ] {
        let samples = total.samples[level as usize];
        if samples == 0 {
            continue;
        }
        let blocks = total.top(level, usize::MAX);
        let mut line = format!(
            "PES HOT SHARE {name} samples={samples} other={} blocks={}",
            total.other,
            blocks.len()
        );
        for shown in [24, 100, 1000, step_timing::HOT_BLOCKS] {
            let counted: u64 = blocks.iter().take(shown).map(|block| block.count).sum();
            line.push_str(&format!(" top{shown}={}", counted * 1000 / samples));
        }
        output::arguments(format_args!("{line}\n"));
        let mut line = format!("PES HOT TOTAL {name}");
        for block in blocks.iter().take(SHOWN) {
            push_hot_block(&mut line, level, block);
        }
        output::arguments(format_args!("{line}\n"));
    }
}

/// The end of the `PES STAMPED` line under step timing: for RET, BLR, BR
/// and their PAC forms, the sites sampled at least twice since the first beat, those
/// that had one target in every sample, and the samples of each group, as
/// `form=sites/mono/samples/mono_samples`, then the samples of sites past
/// those tracked: whether one-target inline caches would serve the hot
/// sites.
fn dynamic_sites(dynamic: &step_timing::DynamicSites) -> String {
    use step_timing::DynamicForm;
    let mut line = String::new();
    for form in DynamicForm::ALL {
        let seen = dynamic.seen(form);
        let name = match form {
            DynamicForm::Ret => "ret",
            DynamicForm::Blr => "blr",
            DynamicForm::Br => "br",
            DynamicForm::Reta => "reta",
            DynamicForm::Blra => "blra",
            DynamicForm::Bra => "bra",
        };
        line.push_str(&format!(
            " {name}={}/{}/{}/{}",
            seen.sites, seen.monomorphic, seen.samples, seen.monomorphic_samples
        ));
    }
    line.push_str(&format!(" untracked={}", dynamic.untracked));
    line
}

/// `DEVICE TRANSFERS calls/us/longest_us`: since the last beat, each part
/// of the board's transfers (`stela_soc::machine::TRANSFER_PARTS`), which
/// run under the devices' lock after every device write and at the home
/// machine's DMA deadline. The first beat starts the clock and reports
/// nothing.
fn transfer_times_report<H: Host>(machine: &Machine<H, parallel::Board>, tsc_hz: u64) {
    let times = machine.devices.with(|devices| {
        devices.time_transfers(Some(host::rdtsc));
        devices.take_transfer_times()
    });
    if times.iter().all(|time| time.calls == 0) {
        return;
    }
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    let mut line = String::from("DEVICE TRANSFERS calls/us/longest_us");
    for (name, time) in stela_soc::machine::TRANSFER_PARTS.iter().zip(&times) {
        line.push_str(&format!(
            " {name}={}/{}/{}",
            time.calls,
            micros(time.ticks),
            micros(time.longest)
        ));
    }
    output::arguments(format_args!("{line}\n"));
}

/// `DEVICE WRITE PARTS calls/entry_us/model_us/service_us/longest_us`:
/// since the last beat, the board's scalar device writes by register page
/// (`stela_soc::machine::WRITE_PARTS`), the most costly first: the model's
/// own work under the devices' lock, apart from the lock, the transfer and
/// the release a write's exit also holds.
fn write_times_report<H: Host>(machine: &Machine<H, parallel::Board>, tsc_hz: u64) {
    let times = machine.devices.with(|devices| devices.take_write_times());
    let mut pages: Vec<_> = times
        .pages
        .iter()
        .map(|page| (Some(page.page), *page))
        .chain(core::iter::once((None, times.other)))
        .filter(|(_, page)| page.calls != 0)
        .collect();
    if pages.is_empty() {
        return;
    }
    pages.sort_unstable_by_key(|(_, page)| core::cmp::Reverse(page.parts.iter().sum::<u64>()));
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    let mut line = String::from("DEVICE WRITE PARTS calls/entry_us/model_us/service_us/longest_us");
    for (address, page) in &pages {
        let name = match address {
            Some(address) => {
                let device = machine
                    .devices
                    .with(|devices| stela_engine::machine::Devices::device_name(devices, *address))
                    .unwrap_or("unmapped");
                format!("{device}@{address:#x}")
            }
            None => String::from("other"),
        };
        line.push_str(&format!(
            " {name}={}/{}/{}/{}/{}",
            page.calls,
            micros(page.parts[0]),
            micros(page.parts[1]),
            micros(page.parts[2]),
            micros(page.longest)
        ));
    }
    output::arguments(format_args!("{line}\n"));
}

/// `DEVICE AIC rises/pins_raised/claimed/empty`: since the last beat, how
/// often the AIC's IRQ output rose, the CPU pins those rises raised, and
/// the event reads that claimed an IRQ or found none
/// (`stela_soc::aic::AicCounts`): pins per rise is the fan-out, and the
/// empty reads mostly the CPUs that lost the claim.
fn aic_counts_report<H: Host>(machine: &Machine<H, parallel::Board>) {
    let Some(counts) = machine.devices.with(|devices| devices.take_aic_counts()) else {
        return;
    };
    if counts == stela_soc::aic::AicCounts::default() {
        return;
    }
    output::arguments(format_args!(
        "DEVICE AIC rises={} pins_raised={} claimed={} empty={}\n",
        counts.rises, counts.pins_raised, counts.claimed, counts.empty
    ));
}

/// Completed intervals of this owned home machine only. Other PEs' state
/// is not inspected; an unfinished phase has no completed maximum yet.
pub(super) fn profile_lifetime_maxima(timing: &step_timing::StepTiming, tsc_hz: u64, report: &str) {
    for phase in step_timing::Phase::ALL {
        let maximum = timing.lifetime_maxima[phase as usize];
        if maximum.ticks == 0 {
            continue;
        }
        output::arguments(format_args!(
            "PROFILE MAX observer=home report={report} phase={} scope=lifetime counter_hz={tsc_hz} ticks={} finished_ticks={} pc_after={:#x} el_after={} daif_after={:#x} snapshot=materialized-after completed-only=true precise_pc=false cost_bound=false\n",
            phase.name(),
            maximum.ticks,
            maximum.finished_ticks,
            maximum.pc_after,
            maximum.el_after,
            maximum.daif_after,
        ));
    }
}

pub(super) fn level_name(guarded: bool) -> &'static str {
    if guarded { "GL" } else { "EL" }
}

/// Blocks between checks of whether the screen or a heartbeat is due.
pub(super) const SLICE: u64 = 1 << 18;

/// How often the waits of every PE return to the run loop to take host
/// inputs, at least (`Machine::set_wait_limit`).
pub(super) const WAIT_POLLS_PER_SECOND: u64 = 20;

/// How often the boot PE's slices end, at least, for its host service, the
/// GPUs' pump and the heartbeat's stages: 4 ms, whether its guest runs or
/// waits (`start`).
pub(super) const HOME_SLICES_PER_SECOND: u64 = 250;

/// Seconds between heartbeats, which locate a guest that runs without output.
pub(super) const BEAT_SECONDS: u64 = 10;

/// Run up to `budget` blocks (0: no limit), showing what macOS prints last
/// as the running step's detail, reporting the PE's position
/// every [`BEAT_SECONDS`]. The caller supplies
/// its admitted execution path; reporting and host polling never select a
/// different memory or native-entry path.
pub(super) fn run<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    budget: u64,
    tsc_hz: u64,
    boot_progress: &mut progress::BootProgress,
    mut execute: impl FnMut(&mut Machine<H, parallel::Board>, u64) -> Stop,
) -> Stop {
    let mut heartbeat = Heartbeat::new(machine, host::rdtsc());
    let mut service = ServiceTimes::new(heartbeat.beat.at);
    boot_progress.start(heartbeat.beat.at, machine.statistics.blocks);
    let mut drained_at = host::rdtsc();
    let mut cursor = super::cursor::Cursor::new(drained_at);
    // While this runs, processors step the staged disks (`storage::released`).
    let _stepping = super::storage::Stepping::start();
    super::display::set_tick_rate(tsc_hz);
    loop {
        let slice = machine.statistics.blocks + SLICE;
        let limit = if budget == 0 {
            slice
        } else {
            budget.min(slice)
        };
        let slice_started = host::rdtsc();
        service.slice_start(slice_started);
        machine.observe_parallel_source(parallel::SourcePhase::SliceEnter);
        let stop = execute(machine, limit);
        machine.observe_parallel_source(parallel::SourcePhase::SliceExit);
        let service_now = host::rdtsc();
        let waited = matches!(stop, Stop::Waiting) || machine.yielded_for_interrupt(&stop);
        service.slice_end(service_now, if waited { "wait" } else { "guest" });
        heartbeat.sliced_ticks += service_now.wrapping_sub(slice_started);
        // The host's work between slices runs outside the boot PE's run: it
        // uses none of its translations, and another PE's maintenance
        // broadcast need not wait for it to end (it reads its mail first).
        let (waiting, quiet) = host_work(machine, |machine| {
            // One hold of the devices' lock, which every PE's device accesses
            // take, for everything they left for the reports. A PE whose access
            // waits on the host (a disk's MMIO) can hold it for milliseconds:
            // the reports then wait for a later slice, a few at most.
            let drained = if service_now.wrapping_sub(drained_at) < DRAIN_MILLIS * tsc_hz / 1000 {
                machine.devices.try_with(Drained::take)
            } else {
                Some(machine.devices.with(Drained::take))
            };
            if let Some(drained) = drained {
                drained_at = service_now;
                report_drained(machine, drained, boot_progress, service_now, tsc_hz);
            }
            service.parts.lap(ServiceParts::REPORTS, host::rdtsc());
            // A wait the limit or the host's inputs ended goes on like a slice.
            let waiting = matches!(stop, Stop::Waiting);
            // The boot PE holds no lock another PE spins on while it waits for
            // an interrupt or runs at EL0 (`service`).
            let idle = machine.yielded_for_interrupt(&stop);
            let quiet = idle || machine.cpu.pstate.el == 0;
            let requested = machine.devices.hub().service_unclaimed();
            // It serves whole only while it waits itself: a PE in guest code
            // that serves holds its guest's thread, and at EL1 maybe a lock
            // other PEs time (`clock`). Waiting PEs serve; with none for a while, it takes one
            // bounded step between slices.
            if !machine.runs_parallel() || idle {
                let watch = machine.guest_watch();
                machine.exclude_host_time(|_| service::serve(true, false, &watch));
            } else if service::starved(tsc_hz, requested) {
                let watch = machine.guest_watch();
                machine.exclude_host_time(|_| service::serve(true, true, &watch));
            }
            service.parts.lap(ServiceParts::SERVICE, host::rdtsc());
            // The GPUs' work waits for a boot PE that holds no lock, as the
            // service does, for a bounded time (`gpu::pump_due`).
            super::gpu::interrupt_window();
            let pump = super::gpu::pump_due(calm(machine, quiet));
            // The frames the DCP model presented reach the screen outside the
            // devices' lock its presents hold, from the other PEs while they
            // run (`display::Turn`): the boot PE takes only the first and
            // those late, so the GPUs' work it alone can do is its share.
            let turn = if machine.runs_parallel() {
                super::display::Turn::Home
            } else {
                super::display::Turn::Alone
            };
            // One stop for all of it.
            machine.exclude_host_time(|devices| {
                if pump {
                    super::gpu::pump();
                }
                service.parts.lap(ServiceParts::PUMP, host::rdtsc());
                super::display::flush(turn);
                service.parts.lap(ServiceParts::DISPLAY, host::rdtsc());
                // A staged guest disk nobody else stepped lately: one step
                // with guest code to run, more while idle. What it completed
                // the home machine's next transfer takes.
                if super::storage::serve_staged(idle) {
                    devices.call_home();
                }
                // The translation pack's writes, a bounded step, only while
                // this PE's guest waits.
                if idle {
                    super::translations::step();
                }
                service.parts.lap(ServiceParts::DISK, host::rdtsc());
            });
            if service::shutdown_pending() {
                let watch = machine.guest_watch();
                machine.exclude_host_time(|_| service::serve(true, false, &watch));
            }
            // A GPU compilation no other PE took (`gate::run_job`), while
            // this PE's guest waits, so the count need not wait for it.
            #[cfg(feature = "nvk")]
            if idle {
                super::gpu::gate::run_job(super::gpu::gate::Taker::Boot);
            }
            service.parts.lap(ServiceParts::SERVICE, host::rdtsc());
            (waiting, quiet)
        });

        // A wall-time service yield may precede even the final block limit.
        // Only the executed count can establish budget completion.
        let budget_reached = budget != 0 && machine.statistics.blocks >= budget;
        let sliced = waiting || matches!(stop, Stop::Budget) && !budget_reached;
        let now = host::rdtsc();
        if !sliced {
            service.report(tsc_hz);
            output::report_log_stores();
            return stop;
        }
        cursor.look(now);
        heartbeat.sample(machine, now, tsc_hz);
        // The heartbeat's reports wait for a quiet boot PE, as the service
        // does, for one more interval at most.
        let due = now - heartbeat.beat.at;
        if heartbeat.stage.is_some() {
            // The beat's later reports, one bounded stage per boundary.
            host_work(machine, |machine| heartbeat.step(machine, tsc_hz));
        } else if due >= BEAT_SECONDS * tsc_hz && (quiet || due >= 2 * BEAT_SECONDS * tsc_hz) {
            host_work(machine, |machine| {
                heartbeat.beat(machine, &mut service, now, tsc_hz)
            });
            output::arguments(format_args!("{}", cursor.report(now, tsc_hz)));
        }
    }
}

/// The longest the boot PE's reports wait for a free devices' lock before
/// they wait on it.
const DRAIN_MILLIS: u64 = 50;

/// Run the host's `work` between slices (the boot PE's reports, service,
/// pump and beat; an AP's reports, service and serial drain) outside the
/// PE's run, so that maintenance broadcasts do not wait for it. It uses
/// none of the PE's translations, which the PE's mail updates before its
/// next block.
pub(super) fn host_work<H: Host, R>(
    machine: &mut Machine<H, parallel::Board>,
    work: impl FnOnce(&mut Machine<H, parallel::Board>) -> R,
) -> R {
    // Out of its run loop, the PE stays reachable for guest work: a wake
    // with an interrupt for it kicks its core, and the shared count holds
    // for it a stall limit past that or its own timer's deadline.
    machine.serve_host(work)
}

/// What the boot PE runs at a slice's end, for the GPUs' pump: `quiet` as
/// for the service, else whether its IRQs are masked, as XNU's spinlocks
/// and interrupt handlers run, or it runs at a guarded level.
fn calm<H: Host>(machine: &Machine<H, parallel::Board>, quiet: bool) -> super::gpu::Calm {
    use stela_engine::machine::exception::DAIF_I;
    let pstate = &machine.cpu.pstate;
    if quiet {
        super::gpu::Calm::Quiet
    } else if pstate.daif & DAIF_I != 0 || pstate.guarded {
        super::gpu::Calm::Masked
    } else {
        super::gpu::Calm::Unmasked
    }
}

/// Report what the PEs' device accesses left since the last slice: the
/// guest's console and the installer's step, the boot's progress, and the
/// unmodeled requests and device logs.
pub(super) fn report_drained<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    drained: Drained,
    boot_progress: &mut progress::BootProgress,
    now: u64,
    tsc_hz: u64,
) {
    let lines = drained.lines;
    if let Some(step) = lines.iter().rev().find_map(|line| installer_step(line)) {
        crate::status::progress(&step);
        boot_progress.installer(step);
    }
    if boot_progress.due(now, tsc_hz) {
        let physical = observed_instruction(machine);
        if let Some(update) = boot_progress.sample(physical, now, tsc_hz, machine.statistics.blocks)
        {
            if let Some(component) = update.first_component {
                output::arguments(format_args!(
                    "BOOT PROGRESS observed={} blocks={}\n",
                    component.name(),
                    machine.statistics.blocks,
                ));
            }
            crate::status::progress(&update.detail);
        }
    }
    crate::status::stream(lines.iter().map(String::as_str));
    crate::status::tick();
    unmodeled_report(machine, drained.unmodeled);
    watchdog_report(machine, drained.watchdog);
    for line in drained.sep {
        output::arguments(format_args!("SEP OBSERVE {line}\n"));
    }
    parallel::report_diagnostics(machine);
    for log in drained.usb {
        use stela_soc::machine::UsbLog;
        match log {
            UsbLog::Descriptor(descriptor) => output::arguments(format_args!(
                "USB descriptor type={} index={} lang={:#x} bytes={:02x?}\n",
                descriptor.kind, descriptor.index, descriptor.language, descriptor.bytes
            )),
            UsbLog::MuxControl(bytes) => {
                output::arguments(format_args!("USB mux control bytes={:02x?}\n", bytes))
            }
            UsbLog::MuxRejected(bytes) => {
                output::arguments(format_args!("USB mux rejected bytes={:02x?}\n", bytes))
            }
            UsbLog::QueryTypeReply(bytes) => output::arguments(format_args!(
                "USB QueryType reply bytes={:02x?} text={}\n",
                bytes,
                String::from_utf8_lossy(&bytes)
            )),
        }
    }
    for (port, line) in drained.usb_transfers {
        output::arguments(format_args!("USB transfer port={port} {line}\n"));
    }
    for (port, line) in drained.usb_host {
        output::arguments(format_args!("XHCI port={port} {line}\n"));
    }

    for log in drained.smc {
        output::arguments(format_args!(
            "SMC key-write op={:#x} id={} flags={:#x} key={:#010x} command={:x?} request={:#018x} reply={:x?} dropped={}\n",
            log.request as u8,
            (log.request >> 12) & 0xf,
            (log.request >> 8) & 0xf,
            (log.request >> 32) as u32,
            log.system_event,
            log.request,
            log.reply,
            log.dropped_before
        ));
    }

    for entry in drained.display {
        display::print_log(&entry);
    }

    for log in drained.power {
        use stela_soc::machine::PowerLog;
        match log {
            PowerLog::Sleep => output::arguments(format_args!(
                "POWER SoC sleep: every PE is down after slpa\n"
            )),
            PowerLog::Wake(source) => output::arguments(format_args!("POWER SoC wake: {source}\n")),
            PowerLog::Masked(source) => output::arguments(format_args!(
                "POWER SoC wake by the {source} masked by the AP; still asleep\n"
            )),
        }
    }
}

/// The host service between slices: how many boundaries, the longest gap
/// between two slices and the longest service, by part, since the last beat.
pub(super) struct ServiceTimes {
    /// When the last slice ended.
    pub(super) serviced: u64,
    pub(super) boundaries: u64,
    pub(super) gap: u64,
    /// When the running slice started, and of the longest gap the slice's
    /// part and whether the boot PE ran guest code or waited in it.
    pub(super) slice_started: u64,
    pub(super) gap_slice: u64,
    pub(super) gap_kind: &'static str,
    /// With every PE on a core of its own, the boot PE's guest time passes
    /// meanwhile.
    pub(super) longest: u64,
    pub(super) parts: ServiceParts,
}

impl ServiceTimes {
    pub(super) fn new(now: u64) -> Self {
        Self {
            serviced: now,
            boundaries: 0,
            gap: 0,
            slice_started: now,
            gap_slice: 0,
            gap_kind: "-",
            longest: 0,
            parts: ServiceParts::new(now),
        }
    }

    /// A slice starts at `now`, ending the service since the last one.
    pub(super) fn slice_start(&mut self, now: u64) {
        self.longest = self.longest.max(now.wrapping_sub(self.serviced));
        self.parts.lap(ServiceParts::BEAT, now);
        self.slice_started = now;
    }

    /// The slice ended at `now`, starting the next service; `kind` is what
    /// the boot PE did in it: `guest` code, or `wait` for an interrupt.
    pub(super) fn slice_end(&mut self, now: u64, kind: &'static str) {
        self.parts.start(now);
        let gap = now.wrapping_sub(self.serviced);
        if gap > self.gap {
            self.gap = gap;
            self.gap_slice = now.wrapping_sub(self.slice_started);
            self.gap_kind = kind;
        }
        self.serviced = now;
        self.boundaries = self.boundaries.saturating_add(1);
    }

    pub(super) fn report(&self, tsc_hz: u64) {
        output::arguments(format_args!(
            "HOST SERVICE boundaries={} maximum_gap_us={} gap=host_us/slice_us/kind:{}/{}/{} maximum_service_us={} gpu_interrupts={} linux_irqs={} irq_quarantined={} irq_fault={:?} {} {}\n",
            self.boundaries,
            u128::from(self.gap) * 1_000_000 / u128::from(tsc_hz),
            u128::from(self.gap.saturating_sub(self.gap_slice)) * 1_000_000 / u128::from(tsc_hz),
            u128::from(self.gap_slice) * 1_000_000 / u128::from(tsc_hz),
            self.gap_kind,
            u128::from(self.longest) * 1_000_000 / u128::from(tsc_hz),
            super::gpu::interrupts(),
            stela_linux::service::serviced_irqs(),
            crate::native_irq::CONTROLLER.quarantine_count(),
            crate::native_irq::CONTROLLER.controller_failure(),
            self.parts.report(tsc_hz),
            service::report(tsc_hz),
        ));
        if let Some(line) = super::gpu::pump_report(tsc_hz) {
            output::arguments(format_args!("{line}\n"));
        }
        #[cfg(feature = "nvk")]
        super::gpu::gate::report_calls(tsc_hz);
    }

    /// Start the next beat's maxima.
    pub(super) fn restart(&mut self) {
        self.gap = 0;
        self.gap_slice = 0;
        self.gap_kind = "-";
        self.longest = 0;
        self.parts.longest = [0; ServiceParts::PARTS];
        self.parts.total = [0; ServiceParts::PARTS];
        self.parts.sliced = 0;
    }
}

/// The counts a heartbeat reports the change in, as of the last beat.
#[derive(Clone, Copy)]
pub(super) struct Beat {
    /// The TSC at the beat.
    pub(super) at: u64,
    /// TSC ticks spent in compiled code.
    pub(super) native: u64,
    pub(super) blocks: u64,
    pub(super) compiled: u64,
    pub(super) chained: u64,
    /// Exceptions, interrupts and waits.
    pub(super) events: (u64, u64, u64),
}

impl Beat {
    pub(super) fn at<H: Host>(machine: &Machine<H, parallel::Board>, at: u64) -> Self {
        let statistics = &machine.statistics;
        Self {
            at,
            native: host::native_ticks(),
            blocks: statistics.blocks,
            compiled: statistics.compiled,
            chained: statistics.chained,
            events: (
                statistics.exceptions,
                statistics.interrupts,
                statistics.waits,
            ),
        }
    }
}

/// What the heartbeat compares against and samples between beats.
pub(super) struct Heartbeat {
    pub(super) beat: Beat,
    /// Guest instructions retired by the last beat.
    pub(super) retired: u128,
    /// Each PE's counts at the last beat.
    pub(super) pe_counts: Vec<parallel::Counts>,
    /// The shared count at the last beat, 0 before the first.
    pub(super) clock_count: u64,
    /// Lock waits at the last beat: the devices', by call, and the heap's.
    pub(super) lock_waits: ([u64; Call::ALL.len()], u64),
    /// The devices' lock times at the last beat, by call.
    pub(super) lock_times: [stela_engine::machine::engine::LockTimes; Call::ALL.len()],
    /// The board's shared code at the last beat.
    pub(super) board: stela_engine::machine::engine::CodeReport,
    /// Counter ticks spent compiling, the guest instructions compiled and
    /// the blocks taken from the translation store, at the last beat.
    pub(super) translation: (u64, u64, u64),
    /// At most one call-chain sample per second, bounded by the heartbeat
    /// interval. Host-service yields can be much more frequent than sampling.
    pub(super) samples: Vec<[u64; CHAIN]>,
    pub(super) sampled: u64,
    /// The running PE's PC at service boundaries, at most once a millisecond.
    pub(super) hot: BTreeMap<u64, HotPc>,
    pub(super) hot_sampled: u64,
    /// Samples taken at EL0.
    pub(super) user: u32,
    pub(super) beats: u64,
    pub(super) exits: ExitsSeen,
    pub(super) translated: TranslatedSeen,
    pub(super) shadows: ShadowSeen,
    pub(super) occupancy: Option<stela_engine::machine::engine::Occupancy>,
    /// Ticks inside `execute` since the last beat, and the step timing then.
    pub(super) sliced_ticks: u64,
    pub(super) timing: Option<step_timing::StepTiming>,
    /// The beat's reports still to run, one stage per service boundary
    /// ([`Self::step`]), and what its stages took so far.
    pub(super) stage: Option<BeatStage>,
    pub(super) cost: BeatCost,
    /// The hardware counters at the last `PES PMU` line (`host::pmu`).
    pub(super) pmu: host::pmu::Seen,
    /// The RET, BLR and BR sites sampled since the first beat
    /// (`hot_blocks_report`).
    pub(super) dynamic: step_timing::DynamicSites,
    /// The hot blocks sampled since the first beat (`hot_total_report`).
    pub(super) hot_total: step_timing::HotBlocks,
}

/// The heartbeat's reports after its first part ([`Heartbeat::beat`]), in
/// order. Each runs at its own service boundary, so none holds the boot PE
/// long; the walks of every block go on in steps of [`STEP_MICROS`].
pub(super) enum BeatStage {
    /// The exits by instruction word, then their change since the last beat
    /// and the exit, translated and shadow lines.
    Exits(stela_engine::machine::engine::ExitWordsWalk),
    ExitsDiff(WordsDiff),
    /// The PEs and stale code, then the beat's samples and the timers.
    Stale(BeatSamples),
    Samples(BeatSamples),
    /// The coprocessors, storage and USB, the survey and the occupancy.
    Devices,
    Suspension,
    /// The home PE's code bookkeeping, every [`FOOTPRINT_BEATS`] beats.
    Footprint(stela_engine::machine::engine::FootprintWalk),
}

/// The call chains and PCs the heartbeat sampled, and the samples at EL0.
pub(super) struct BeatSamples {
    samples: Vec<[u64; CHAIN]>,
    hot: BTreeMap<u64, HotPc>,
    user: u32,
}

/// The longest one heartbeat stage, or one step of a walk of every block,
/// runs on the boot PE between two slices.
const STEP_MICROS: u64 = 300;

impl Heartbeat {
    pub(super) fn new<H: Host>(machine: &Machine<H, parallel::Board>, now: u64) -> Self {
        Self {
            beat: Beat::at(machine, now),
            retired: machine.retired_instructions(),
            pe_counts: parallel::counts(machine),
            clock_count: 0,
            lock_waits: ([0; Call::ALL.len()], 0),
            lock_times: Default::default(),
            board: Default::default(),
            translation: (
                machine.statistics.compile_ticks,
                machine.statistics.compiled_instructions,
                machine.statistics.stored,
            ),
            samples: Vec::with_capacity(BEAT_SECONDS as usize),
            sampled: now,
            hot: BTreeMap::new(),
            hot_sampled: now,
            user: 0,
            beats: 0,
            exits: ExitsSeen::default(),
            translated: TranslatedSeen::default(),
            shadows: ShadowSeen::default(),
            occupancy: None,
            sliced_ticks: 0,
            timing: machine.step_timing().copied(),
            stage: None,
            cost: BeatCost::new(),
            pmu: host::pmu::Seen::new(),
            dynamic: Default::default(),
            hot_total: Default::default(),
        }
    }

    /// Sample the running PE's PC and, once a second, its call chain.
    pub(super) fn sample<H: Host>(
        &mut self,
        machine: &mut Machine<H, parallel::Board>,
        now: u64,
        tsc_hz: u64,
    ) {
        if now.wrapping_sub(self.hot_sampled) >= tsc_hz / 1000 {
            let pc = machine.cpu.pc;
            self.hot
                .entry(pc)
                .or_insert_with(|| {
                    let mut code = [0; HOT_CODE_BYTES];
                    if !machine.read_virtual(pc.wrapping_sub(HOT_CODE_BEFORE), &mut code) {
                        code.fill(0);
                    }
                    HotPc { count: 0, code }
                })
                .count += 1;
            self.hot_sampled = now;
        }
        if now.wrapping_sub(self.sampled) >= tsc_hz && self.samples.len() < BEAT_SECONDS as usize {
            let (pc, frame) = (machine.cpu.pc, machine.cpu.state.x[29]);
            self.samples.push(call_chain(machine, pc, frame));
            self.user += u32::from(machine.cpu.pstate.el == 0);
            self.sampled = now;
        }
    }

    /// Report the interval since the last beat, which ends `now`: the
    /// position, service, processors, performance and translation lines,
    /// which compare against the last beat; the rest follow in stages
    /// ([`Self::step`]).
    pub(super) fn beat<H: Host>(
        &mut self,
        machine: &mut Machine<H, parallel::Board>,
        service: &mut ServiceTimes,
        now: u64,
        tsc_hz: u64,
    ) {
        let mut at = now;
        let mut mark = |cost: &mut BeatCost, name: &'static str| {
            let now = host::rdtsc();
            cost.add(name, now.wrapping_sub(at));
            at = now;
        };
        let mut cost = BeatCost::new();
        let next = Beat::at(machine, now);
        let elapsed = now - self.beat.at;
        beat_report(machine, &self.beat, &next, elapsed);
        mark(&mut cost, "position");
        service.report(tsc_hz);
        mark(&mut cost, "service");
        self.report_processors(machine, &next, elapsed, tsc_hz);
        mark(&mut cost, "processors");
        if let (Some(now), Some(then)) = (machine.step_timing().copied(), self.timing) {
            profile_report(&now.since(&then), self.sliced_ticks, elapsed, tsc_hz);
            self.timing = Some(now);
            // Hardware counters by phase, per PE: `host::pmu::report`.
            host::pmu::report(&mut self.pmu);
        }
        self.sliced_ticks = 0;
        #[cfg(feature = "native-profile")]
        native_profile_report(machine);
        service.restart();
        let compiled = next.compiled - self.beat.compiled;
        self.beat = next;
        mark(&mut cost, "performance");
        self.translation = translation_report(machine, self.translation, compiled);
        mark(&mut cost, "translation");
        cost.step(at.wrapping_sub(now));
        self.cost = cost;
        self.beats += 1;
        self.stage = Some(BeatStage::Exits(Default::default()));
    }

    /// Run the beat's next stage, or the next step of its walk, for about
    /// [`STEP_MICROS`] at most; after the last one, the cost line.
    pub(super) fn step<H: Host>(&mut self, machine: &mut Machine<H, parallel::Board>, tsc_hz: u64) {
        let Some(stage) = self.stage.take() else {
            return;
        };
        let started = host::rdtsc();
        let budget = tsc_hz / 1_000_000 * STEP_MICROS;
        let more = || host::rdtsc().wrapping_sub(started) < budget;
        let (name, next) = match stage {
            // The walk of every block costs about 16 ms a beat on a booted
            // desktop: it runs only while profiling.
            BeatStage::Exits(_) if machine.step_timing().is_none() => (
                "exits",
                Some(BeatStage::ExitsDiff(WordsDiff::unwalked(core::mem::take(
                    &mut self.exits.words,
                )))),
            ),
            BeatStage::Exits(mut walk) => {
                let next = if machine.exit_words_step(&mut walk, more) {
                    BeatStage::ExitsDiff(WordsDiff::new(machine.exit_words_finish(walk)))
                } else {
                    BeatStage::Exits(walk)
                };
                ("exits", Some(next))
            }
            BeatStage::ExitsDiff(mut diff) => {
                if self.exits.diff_step(&mut diff, more) {
                    self.exits.report(machine, diff, tsc_hz);
                    self.translated.report(machine);
                    self.shadows.report();
                    let samples = BeatSamples {
                        samples: core::mem::take(&mut self.samples),
                        hot: core::mem::take(&mut self.hot),
                        user: core::mem::take(&mut self.user),
                    };
                    ("exits", Some(BeatStage::Stale(samples)))
                } else {
                    ("exits", Some(BeatStage::ExitsDiff(diff)))
                }
            }
            BeatStage::Stale(samples) => {
                peers_report(machine);
                let stale = machine.stale_code();
                if stale.stale > 0 || stale.admitted > 0 {
                    output::arguments(format_args!("STALE {stale:?}\n"));
                }
                ("stale", Some(BeatStage::Samples(samples)))
            }
            BeatStage::Samples(mut samples) => {
                sample_report(&mut samples.samples, samples.user);
                hot_report(&mut samples.hot);
                timer_report(machine);
                ("samples", Some(BeatStage::Devices))
            }
            BeatStage::Devices => {
                for state in machine
                    .devices
                    .with(|devices| devices.coprocessor_diagnostics())
                {
                    output::arguments(format_args!("ASC {state}\n"));
                }
                let (now, _) = machine.timer_snapshot();
                for state in machine
                    .devices
                    .with(|devices| devices.storage_diagnostics(now))
                {
                    output::arguments(format_args!("ANS {state}\n"));
                }
                for state in machine.devices.with(|devices| devices.usb_diagnostics()) {
                    output::arguments(format_args!("USB {state}\n"));
                }
                survey_report(machine);
                occupancy_report(machine, &mut self.occupancy);
                let next = if self.beats.is_multiple_of(SUSPENSION_BEATS) {
                    Some(BeatStage::Suspension)
                } else if self.beats.is_multiple_of(FOOTPRINT_BEATS) {
                    Some(BeatStage::Footprint(Default::default()))
                } else {
                    None
                };
                ("devices", next)
            }
            BeatStage::Suspension => {
                suspension_report(machine);
                let next = self
                    .beats
                    .is_multiple_of(FOOTPRINT_BEATS)
                    .then(|| BeatStage::Footprint(Default::default()));
                ("suspension", next)
            }
            BeatStage::Footprint(mut walk) => {
                if machine.footprint_step(&mut walk, more) {
                    footprint_report(machine, &machine.footprint_finish(&walk));
                    ("footprint", None)
                } else {
                    ("footprint", Some(BeatStage::Footprint(walk)))
                }
            }
        };
        let took = host::rdtsc().wrapping_sub(started);
        self.cost.add(name, took);
        self.cost.step(took);
        self.stage = next;
        if self.stage.is_none() {
            self.cost.report(tsc_hz);
        }
    }

    /// The guest instructions retired in the interval and, with each PE on
    /// a host core of its own, each PE's rate and time and the lock waits.
    pub(super) fn report_processors<H: Host>(
        &mut self,
        machine: &mut Machine<H, parallel::Board>,
        next: &Beat,
        elapsed: u64,
        tsc_hz: u64,
    ) {
        // Each PE on its own host core counts for itself.
        parallel::report_source_observations(machine);
        if machine.runs_parallel() {
            let count = machine.devices.hub().clock(host::fresh_counter);
            // Guest time against host time, and the host time the count
            // waited for PEs the host stopped in the kernel too long.
            let wall =
                u128::from(elapsed) * u128::from(host::COUNTER_HZ) / u128::from(tsc_hz.max(1));
            let guest = count.wrapping_sub(self.clock_count);
            let stalled = machine.devices.hub().clock_stalled();
            // What the count still lags host time by after holds, and how
            // much of the lag it made up since the last beat.
            let (lag, repaid) = machine.devices.hub().clock_lag();
            if self.clock_count != 0 {
                let rate = u128::from(guest) * 1000 / wall.max(1);
                let millis = |counts: u64| u128::from(counts) * 1000 / u128::from(host::COUNTER_HZ);
                output::arguments(format_args!(
                    "CLOCK RATE guest_per_wall={}.{:03} stalled_ms={} lag_ms={} repaid_ms={}\n",
                    rate / 1000,
                    rate % 1000,
                    millis(stalled),
                    millis(lag),
                    millis(repaid),
                ));
            }
            self.clock_count = count;
            output::arguments(format_args!("CLOCK count={count:#x}\n"));
            // Per PE: its longest gap with guest work and no instruction
            // retired since the last beat, and the engine phase that took
            // most of it (`host` outside the step loop, `?` without
            // `--profile`); one past 50 ms held the count (`stalled_ms`).
            let mut stalls = String::from("PES STALLS longest_ms/phase");
            for (pe, (ticks, phase)) in machine.devices.hub().clock_stalls().iter().enumerate() {
                let millis = u128::from(*ticks) * 1000 / u128::from(host::COUNTER_HZ);
                stalls.push_str(&format!(" {pe}:{millis}/{phase}"));
            }
            output::arguments(format_args!("{stalls}\n"));
        }
        let per_pe = parallel::counts(machine);
        let retired = per_pe
            .iter()
            .map(|counts| u128::from(counts.retired))
            .sum::<u128>();
        let instructions = retired.saturating_sub(self.retired);
        // Per PE: its stamped transfers ([`stela_compiler::x86_64::chain::StampCount`]):
        // hits, misses by cause (ITLB drop epoch or none yet, context,
        // interruption), stamps written and cells found changed since their
        // link, then exits it gave a stamped tail; after every PE, the most
        // exits any PE saw with one, and the RET/BLR/BR sites
        // `hot_blocks_report` sampled.
        let mut stamped_line = None;
        if per_pe.len() > 1 {
            let interval_ticks =
                u128::from(elapsed.max(1)) * u128::from(host::COUNTER_HZ) / u128::from(tsc_hz);
            let percent = |ticks: u64| u128::from(ticks) * 100 / interval_ticks.max(1);
            let mut rates = String::from("PES RETIRED millions_per_second");
            // Per PE: idle, monitor-polling and synchronizing shares of
            // the interval, its waits, and the blocks it compiled and
            // installed from another PE.
            let mut shares =
                String::from("PES TIME idle%/spin%/sync%/waits/compiled/shared/halt%/short");
            // Per PE: the interval's share compiling, reading and hashing
            // fetch pages, finding shared blocks, installing blocks, walking
            // fetch translations and refilling dropped or remade pages.
            let mut fetches =
                String::from("PES FETCH compile%/page%/shared%/install%/walk%/refill%");
            // Per PE: the blocks it holds, its block table's slots, the
            // pages its index finds blocks in, and the code arena parts it
            // released for heap pressure since the last beat; on a shared
            // board the first three are the board's, and `board_code` adds.
            let mut code = String::from("PES CODE live/slots/pages/releases");
            // Per PE: device transfers whose deadline came and device line
            // queries, each as a count and, with `--profile`, a share of the
            // interval.
            let mut device_work = String::from("PES DEVICES transfers/transfer%/queries/query%");
            // Per PE: the fetches its engine made, by where they found their
            // block (the block table through the ITLB, there after a fresh
            // translation, the jump cache, the block index, a compile or
            // install), then the pages its ITLB took and its whole clears.
            let mut lookups = String::from(
                "PES LOOKUP found/translated/jump/indexed/installed/itlb_fills/itlb_rekeys",
            );
            // Per PE: how its fetches began: through the ITLB and the block
            // table without a walk, at BTYPE zero or with a BTYPE the entry
            // admitted, or leaving that path because the ITLB lacked the
            // page, the block table the PC, the entry refused the BTYPE, or
            // the fetch was of one instruction.
            let mut paths = String::from(
                "PES FETCHPATH fast/fast_btype/itlb_miss/table_miss/btype_refused/single",
            );
            // Per PE: its ITLB fills by why the page was missing (a fill
            // took its entry's place, a TLBI by VA dropped it, its context
            // was forgotten, no trace of it); the live entries fills took
            // the place of, the entries TLBIs by VA dropped, the contexts
            // that left a half's most recent, the salts taken from another
            // owner, the whole invalidations its mail kept for too many
            // scoped TLBIs; and its halves' forgotten contexts: for an ASID or
            // non-global TLBI, a TLBI of everything, a TLBI by VA that
            // reached entries, other changes.
            let mut itlb = String::from(concat!(
                "PES ITLB evicted/forgotten/context/cold",
                "/drop_evict/drop_tlbi/space_lru/salt_reuse/mail_all",
                "/forget_asid/forget_all/forget_va/forget_other",
            ));
            // Per PE: where its ITLB's fills, drops and forgotten contexts
            // came from: fills of the lower and upper half, cold fills of
            // a page taken before in the same address space, entries TLBIs
            // by VA dropped in one ASID's contexts and in every context,
            // TLBIs of everything that forgot every context (its own, and
            // another PE's or its mail's), ones that reached nothing of the
            // regime it translates in (forgetting other regimes' contexts,
            // or none), contexts made, TLBIs its mail merged, and pages
            // walked again at once for contexts made anew.
            // Per PE: GENTER completed natively and by the engine, then
            // GEXIT's, SVC's and ERET's.
            let mut transitions =
                String::from("PES TRANSITIONS genter/engine/gexit/engine/svc/engine/eret/engine");
            // All PEs: why SVC, ERET, GENTER and GEXIT took the engine path.
            let mut transition_misses = [0u64; 36];
            // All PEs: why runs ended at fallthrough, direct, indirect, call
            // and return exits.
            let mut branch_misses = [0u64; 40];
            // All PEs: their TLB mail's waits, the sums over the interval
            // and the longest of the run.
            let mut mail_waits = [0u64; stela_engine::machine::engine::MAIL_WAITS];
            let mut itlb_sources = String::from(concat!(
                "PES ITLBSRC fills_lower/fills_upper/cold_seen/drop_asid/drop_every",
                "/all_local/all_received/all_forgot/all_kept/made/mail_merged/refilled",
                "/drop_upper/drop_no_asid/drop_global",
            ));
            // Per PE: its installs at EL0 and then above EL0 by where the
            // block came from: compiled in a page the store keys (offered
            // to it), compiled where it keys none (one instruction, a
            // device-site page), taken from the pack, taken from another
            // PE, rebased from an entry proven at another address during
            // the boot.
            let mut store = String::from(concat!(
                "PES STORE user_compiled/user_unstorable/user_pack/user_shared/user_rebased",
                "/kernel_compiled/kernel_unstorable/kernel_pack/kernel_shared/kernel_rebased",
            ));
            // Per PE: stamped tail hits and misses by cause, stamps written,
            // changed cells, table probe transfers (jump cache misses), stamped
            // tails made, then every probe transfer (jump cache and table).
            let mut stamped = String::from(concat!(
                "PES STAMPED hits/miss_epoch/miss_context/miss_interrupt/restamped/changed",
                "/filled/promoted/probe_hits",
            ));
            let mut stamped_in_use = 0;
            let mut predicted = 0;
            let mut pac_fills = 0;
            let deltas = |line: &mut String, pe: usize, now: &[u64], before: &[u64]| {
                line.push_str(&format!(" {pe}:"));
                for (slot, (now, before)) in now.iter().zip(before).enumerate() {
                    if slot > 0 {
                        line.push('/');
                    }
                    line.push_str(&format!("{}", now.saturating_sub(*before)));
                }
            };
            // Per PE with `--profile`: the interval's share in each engine
            // phase, in step timing's order.
            let mut phases = String::from("PES PHASES");
            for phase in step_timing::Phase::ALL {
                phases.push_str(if phase == step_timing::Phase::Schedule {
                    " "
                } else {
                    "/"
                });
                phases.push_str(phase.name());
                phases.push('%');
            }
            // Per PE with `--profile`: why blocks were installed (evicted
            // here within 10 s, installed on another PE then, never here
            // before), the KiB of block code installed and evicted, and the
            // interval's share evicting. On a shared board one census
            // counts for the board: evicted by any PE, never on the board
            // before, and never elsewhere.
            let mut installs = String::from(
                "PES INSTALLS installs/thrash/elsewhere/first/in_kb/out_kb/evict%/insns/upper_kb/upper_insns",
            );
            let tsc_percent = |ticks: u64| u128::from(ticks) * 100 / u128::from(elapsed.max(1));
            // With `--profile`: the exits of every PE together, by kind, as
            // `PROFILE exits` gives the home machine's.
            let mut exits = [(0u64, 0u64); step_timing::EXIT_SLOTS];
            // Per PE with `--profile`: its busy time (every phase but
            // schedule) at EL0, in the kernel and in guarded mode, in
            // interrupt handlers (an IRQ or FIQ until the next exception
            // return) and in kernel spans that reached a WFI without running
            // EL0 code (idle paths, wake-ups with nothing for user code,
            // kernel threads), as shares of the interval; then its WFIs and
            // those that completed at once, its WFEs that reached the engine
            // and those that completed at once, and its interrupts; then the
            // WFEs native code completed after spinning for the event, the
            // spins that missed, the WFEs an exclusive monitor kept from
            // spinning and the time spins took.
            let mut levels = String::from(
                "PES LEVELS user%/kernel%/guarded%/irq%/before_wfi% wfi/wfi_now/wfe/wfe_now/interrupts/wfe_spun/spin_miss/wfe_held/spin_ns",
            );
            let mut totals = parallel::Counts::default();
            for (pe, (now, before)) in per_pe.iter().zip(&mut self.pe_counts).enumerate() {
                let rate = u128::from(now.retired.saturating_sub(before.retired))
                    .saturating_mul(u128::from(tsc_hz))
                    / u128::from(elapsed.max(1))
                    / 1_000_000;
                rates.push_str(&format!(" {pe}:{rate}"));
                shares.push_str(&format!(
                    " {pe}:{}/{}/{}/{}/{}/{}/{}/{}",
                    percent(now.sleep_ticks.saturating_sub(before.sleep_ticks)),
                    percent(now.spin_ticks.saturating_sub(before.spin_ticks)),
                    percent(now.sync_ticks.saturating_sub(before.sync_ticks)),
                    now.waits.saturating_sub(before.waits),
                    now.compiled.saturating_sub(before.compiled),
                    now.shared.saturating_sub(before.shared),
                    percent(now.halt_ticks.saturating_sub(before.halt_ticks)),
                    now.short_waits.saturating_sub(before.short_waits),
                ));
                fetches.push_str(&format!(
                    " {pe}:{}/{}/{}/{}/{}/{}",
                    percent(now.compile_ticks.saturating_sub(before.compile_ticks)),
                    percent(now.page_ticks.saturating_sub(before.page_ticks)),
                    percent(now.shared_ticks.saturating_sub(before.shared_ticks)),
                    percent(now.install_ticks.saturating_sub(before.install_ticks)),
                    percent(now.walk_ticks.saturating_sub(before.walk_ticks)),
                    percent(now.refill_ticks.saturating_sub(before.refill_ticks)),
                ));
                code.push_str(&format!(
                    " {pe}:{}/{}/{}/{}",
                    now.live_blocks,
                    now.block_slots,
                    now.indexed_pages,
                    now.code_releases.saturating_sub(before.code_releases),
                ));
                device_work.push_str(&format!(
                    " {pe}:{}/{}/{}/{}",
                    now.due_transfers.saturating_sub(before.due_transfers),
                    tsc_percent(now.transfer_ticks.saturating_sub(before.transfer_ticks)),
                    now.line_queries.saturating_sub(before.line_queries),
                    tsc_percent(now.line_ticks.saturating_sub(before.line_ticks)),
                ));
                lookups.push_str(&format!(" {pe}:"));
                for (slot, (now, before)) in now
                    .fetches
                    .iter()
                    .chain(&now.itlb)
                    .zip(before.fetches.iter().chain(&before.itlb))
                    .enumerate()
                {
                    if slot > 0 {
                        lookups.push('/');
                    }
                    lookups.push_str(&format!("{}", now.saturating_sub(*before)));
                }
                deltas(&mut paths, pe, &now.fetch_paths, &before.fetch_paths);
                let concat = |counts: &parallel::Counts| {
                    let mut all = [0u64; 13];
                    all[..4].copy_from_slice(&counts.itlb_misses);
                    all[4..9].copy_from_slice(&counts.itlb_drops);
                    all[9..].copy_from_slice(&counts.lookup_forgets);
                    all
                };
                deltas(&mut itlb, pe, &concat(now), &concat(before));
                deltas(
                    &mut itlb_sources,
                    pe,
                    &now.itlb_sources,
                    &before.itlb_sources,
                );
                deltas(&mut transitions, pe, &now.transitions, &before.transitions);
                for (total, (now, before)) in transition_misses.iter_mut().zip(
                    now.transition_misses
                        .as_flattened()
                        .iter()
                        .zip(before.transition_misses.as_flattened()),
                ) {
                    *total += now.saturating_sub(*before);
                }
                {
                    use stela_engine::machine::engine::MailWait::{Longest, LongestNative};
                    let longest = Longest as usize;
                    for (total, (now, before)) in mail_waits[..longest]
                        .iter_mut()
                        .zip(now.mail_waits.iter().zip(&before.mail_waits))
                    {
                        *total += now.saturating_sub(*before);
                    }
                    if now.mail_waits[longest] > mail_waits[longest] {
                        mail_waits[longest] = now.mail_waits[longest];
                        mail_waits[LongestNative as usize] = now.mail_waits[LongestNative as usize];
                    }
                }
                for (total, (now, before)) in branch_misses.iter_mut().zip(
                    now.branch_misses
                        .as_flattened()
                        .iter()
                        .zip(before.branch_misses.as_flattened()),
                ) {
                    *total += now.saturating_sub(*before);
                }
                let promoted = stela_compiler::x86_64::chain::STAMP_COUNTS + 1;
                deltas(
                    &mut stamped,
                    pe,
                    &now.stamped[..promoted],
                    &before.stamped[..promoted],
                );
                stamped.push_str(&format!(
                    "/{}",
                    now.stamped[promoted + 2].saturating_sub(before.stamped[promoted + 2])
                ));
                stamped_in_use = stamped_in_use.max(now.stamped[promoted]);
                predicted += now.stamped[promoted + 1].saturating_sub(before.stamped[promoted + 1]);
                pac_fills += now.pac_fills.saturating_sub(before.pac_fills);
                deltas(
                    &mut store,
                    pe,
                    now.store_sources.as_flattened(),
                    before.store_sources.as_flattened(),
                );
                let (cause, was) = (&now.installs, &before.installs);
                installs.push_str(&format!(
                    " {pe}:{}/{}/{}/{}/{}/{}/{}/{}/{}/{}",
                    cause.installs.saturating_sub(was.installs),
                    cause.thrash.saturating_sub(was.thrash),
                    cause.elsewhere.saturating_sub(was.elsewhere),
                    cause.first.saturating_sub(was.first),
                    cause.installed_bytes.saturating_sub(was.installed_bytes) / 1024,
                    cause.evicted_bytes.saturating_sub(was.evicted_bytes) / 1024,
                    percent(cause.evict_ticks.saturating_sub(was.evict_ticks)),
                    cause
                        .guest_instructions
                        .saturating_sub(was.guest_instructions),
                    cause.upper_bytes.saturating_sub(was.upper_bytes) / 1024,
                    cause
                        .upper_instructions
                        .saturating_sub(was.upper_instructions),
                ));
                phases.push_str(&format!(" {pe}:"));
                for (slot, (now, before)) in now.phases.iter().zip(before.phases).enumerate() {
                    if slot > 0 {
                        phases.push('/');
                    }
                    phases.push_str(&format!("{}", tsc_percent(now.saturating_sub(before))));
                }
                let level = now.levels.since(&before.levels);
                levels.push_str(&format!(
                    " {pe}:{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}/{}",
                    tsc_percent(level.ticks[step_timing::Level::User as usize]),
                    tsc_percent(level.ticks[step_timing::Level::Kernel as usize]),
                    tsc_percent(level.ticks[step_timing::Level::Guarded as usize]),
                    tsc_percent(level.interrupt_ticks),
                    tsc_percent(level.kernel_before_wfi),
                    level.wfi,
                    level.wfi_at_once,
                    level.wfe,
                    level.wfe_at_once,
                    level.interrupts,
                    level.wfe_spun,
                    level.wfe_spin_missed,
                    level.wfe_held,
                    u128::from(level.wfe_spin_counts) * 1_000_000_000
                        / u128::from(host::COUNTER_HZ),
                ));
                for (total, (now, before)) in
                    exits.iter_mut().zip(now.exits.iter().zip(before.exits))
                {
                    total.0 += now.0.saturating_sub(before.0);
                    total.1 += now.1.saturating_sub(before.1);
                }
                totals.broadcasts += now.broadcasts.saturating_sub(before.broadcasts);
                totals.forwards += now.forwards.saturating_sub(before.forwards);
                *before = *now;
            }
            // Lock waits since the last beat: the devices', by call, and
            // the heap's.
            let waits = (machine.devices.waits(), crate::ALLOCATOR.waits());
            let mut devices = String::new();
            for (call, (now, before)) in
                Call::ALL.iter().zip(waits.0.iter().zip(&self.lock_waits.0))
            {
                devices.push_str(&format!(" {}={}", call.name(), now - before));
            }
            output::arguments(format_args!(
                "{rates} broadcasts={} forwards={} device_waits{devices} heap_waits={}\n",
                totals.broadcasts,
                totals.forwards,
                waits.1 - self.lock_waits.1
            ));
            // The devices' lock since the last beat, by call: contended
            // calls, the milliseconds they waited, the milliseconds calls
            // held it, and the longest hold in microseconds.
            let times = machine.devices.lock_times();
            let millis = |ticks: u64| u128::from(ticks) * 1000 / u128::from(host::COUNTER_HZ);
            let mut locks = String::from("DEVICE LOCK waits/waited_ms/held_ms/longest_us");
            for (index, call) in Call::ALL.iter().enumerate() {
                locks.push_str(&format!(
                    " {}={}/{}/{}/{}",
                    call.name(),
                    waits.0[index] - self.lock_waits.0[index],
                    millis(
                        times[index]
                            .waited
                            .wrapping_sub(self.lock_times[index].waited)
                    ),
                    millis(times[index].held.wrapping_sub(self.lock_times[index].held)),
                    u128::from(times[index].longest) * 1_000_000 / u128::from(host::COUNTER_HZ),
                ));
            }
            // By call since the last beat: the calls that changed another
            // PE's lines, and the microseconds they spent waking those PEs
            // after releasing the lock (a sleeping PE's core takes an
            // interrupt), which the calling PE's exit pays.
            let mut wakes = String::from("DEVICE WAKES calls/us");
            for (index, call) in Call::ALL.iter().enumerate() {
                wakes.push_str(&format!(
                    " {}={}/{}",
                    call.name(),
                    times[index]
                        .woken
                        .wrapping_sub(self.lock_times[index].woken),
                    u128::from(times[index].wake.wrapping_sub(self.lock_times[index].wake))
                        * 1_000_000
                        / u128::from(host::COUNTER_HZ),
                ));
            }
            // By call since the last beat: the releases that did the host's
            // work (a staged disk's step) on the calling core and its
            // microseconds there, those that handed it to a sleeping PE,
            // those that handed it a host disk's completion signal, and those
            // that took that signal's step themselves with no PE asleep.
            let mut releases =
                String::from("DEVICE RELEASES stepped/us/handed/signals/signal_steps");
            for (index, call) in Call::ALL.iter().enumerate() {
                let (now, then) = (&times[index], &self.lock_times[index]);
                releases.push_str(&format!(
                    " {}={}/{}/{}/{}/{}",
                    call.name(),
                    now.stepped.wrapping_sub(then.stepped),
                    u128::from(now.step.wrapping_sub(then.step)) * 1_000_000
                        / u128::from(host::COUNTER_HZ),
                    now.handed.wrapping_sub(then.handed),
                    now.signals.wrapping_sub(then.signals),
                    now.signal_steps.wrapping_sub(then.signal_steps),
                ));
            }
            self.lock_times = times;
            output::arguments(format_args!("{locks}\n"));
            output::arguments(format_args!("{wakes}\n"));
            output::arguments(format_args!("{releases}\n"));
            self.lock_waits = waits;
            output::arguments(format_args!("{shares}\n"));
            output::arguments(format_args!("{fetches}\n"));
            if let Some(board) = machine.devices.hub().code_report() {
                code.push_str(&board_code(&board, &self.board, tsc_hz));
                self.board = board;
            }
            output::arguments(format_args!("{code}\n"));
            output::arguments(format_args!("{device_work}\n"));
            output::arguments(format_args!("{lookups}\n"));
            output::arguments(format_args!("{paths}\n"));
            output::arguments(format_args!("{itlb}\n"));
            output::arguments(format_args!("{itlb_sources}\n"));
            output::arguments(format_args!("{transitions}\n"));
            let [svc, eret, genter, gexit] = [0, 1, 2, 3].map(|kind| {
                transition_misses[kind * 9..kind * 9 + 9]
                    .iter()
                    .map(|count| format!("{count}"))
                    .collect::<Vec<_>>()
                    .join("/")
            });
            output::arguments(format_args!(
                "PES TRANSITION MISSES never/forgotten/frame/deadline/epoch/check/interrupt/other/monitor svc={svc} eret={eret} genter={genter} gexit={gexit}\n"
            ));
            let [fallthrough, direct, indirect, call, ret] = [0, 1, 2, 3, 4].map(|kind| {
                branch_misses[kind * 8..kind * 8 + 8]
                    .iter()
                    .map(|count| format!("{count}"))
                    .collect::<Vec<_>>()
                    .join("/")
            });
            output::arguments(format_args!(
                "PES BRANCH MISSES spent/interrupt/unlinked/itlb/table/btype/found/transitioned fallthrough={fallthrough} direct={direct} indirect={indirect} call={call} return={ret}\n"
            ));
            {
                use stela_engine::machine::engine::MailWait as W;
                let count = mail_waits[W::Count as usize].max(1);
                let nanos =
                    |ticks: u64| u128::from(ticks) * 1_000_000_000 / u128::from(tsc_hz.max(1));
                let each = |part: W| nanos(mail_waits[part as usize] / count);
                output::arguments(format_args!(
                    "PES MAIL tlb={} ns_each latency={} native={} exit={} apply={} longest_us={}/native_us={}\n",
                    mail_waits[W::Count as usize],
                    each(W::Latency),
                    each(W::Native),
                    each(W::Exit),
                    each(W::Apply),
                    nanos(mail_waits[W::Longest as usize]) / 1000,
                    nanos(mail_waits[W::LongestNative as usize]) / 1000,
                ));
            }
            output::arguments(format_args!("{store}\n"));
            stamped.push_str(&format!(" in_use={stamped_in_use}"));
            stamped_line = Some((stamped, predicted, pac_fills));
            if machine.step_timing().is_some() {
                output::arguments(format_args!("{phases}\n"));
                output::arguments(format_args!("{levels}\n"));
                output::arguments(format_args!("{installs}\n"));
                let mut slots: Vec<usize> =
                    (0..exits.len()).filter(|&slot| exits[slot].0 > 0).collect();
                slots.sort_by_key(|&slot| core::cmp::Reverse(exits[slot].1));
                // Shares are of one PE's interval: 100% is one PE's whole time.
                let mut line = String::from("PES EXITS count/pe%/ns");
                for slot in slots {
                    let (count, ticks) = exits[slot];
                    line.push_str(&format!(
                        " {}={}/{}/{}",
                        EXIT_NAMES[slot],
                        count,
                        tsc_percent(ticks),
                        u128::from(ticks) * 1_000_000_000
                            / u128::from(count)
                            / u128::from(tsc_hz.max(1)),
                    ));
                }
                output::arguments(format_args!("{line}\n"));
            }
        }
        if machine.step_timing().is_some() {
            let exits = parallel::system_exits(machine);
            system_exits_report(&exits, tsc_hz);
            barrier_waits_report(&exits.barriers, tsc_hz);
            regime_syncs_report(&exits.regimes, tsc_hz);
            device_exits_report(machine, &exits.devices, tsc_hz);
            transfer_times_report(machine, tsc_hz);
            aic_counts_report(machine);
            write_times_report(machine, tsc_hz);
            hot_blocks_report(
                machine,
                &mut self.dynamic,
                &mut self.hot_total,
                self.beats.is_multiple_of(HOT_TOTAL_BEATS),
            );
        }
        if let Some((mut line, predicted, pac_fills)) = stamped_line {
            if machine.step_timing().is_some() {
                line.push_str(&dynamic_sites(&self.dynamic));
            }
            // Exits of PAC branch thunks given a predicted tail this beat.
            line.push_str(&format!(" predicted={predicted}"));
            // PAC results computed this beat, all PEs: memo misses.
            line.push_str(&format!(" pac_fills={pac_fills}"));
            output::arguments(format_args!("{line}\n"));
        }
        // Self-loops since boot, every PE together: compiled, then with
        // `--profile` their entries and the instructions their iterations
        // retired past each entry's first (0/0 without it).
        let loops = per_pe.iter().fold([0u64; 3], |sum, counts| {
            core::array::from_fn(|at| sum[at].wrapping_add(counts.loops[at]))
        });
        output::arguments(format_args!(
            "GUEST PERFORMANCE retired={retired} interval_retired={instructions} instructions_per_second={} instructions_per_block_milli={} loops={}/{}/{}\n",
            instructions.saturating_mul(u128::from(tsc_hz)) / u128::from(elapsed.max(1)),
            instructions.saturating_mul(1000) / u128::from((next.blocks - self.beat.blocks).max(1)),
            loops[0],
            loops[1],
            loops[2],
        ));
        self.retired = retired;
    }
}

/// The board's shared code since `was`, for `PES CODE`: the window in MiB,
/// its parts and the ring's generation; parts taken away (`retired`), of
/// them for want of a block slot (`forced`); installs that lost to another
/// PE's (`lost`); waits for a part's grace and those that ran out; the
/// longest whole retirement since (lock-held microseconds of its steps, and
/// how many); the block table entries jump cache hits would add (taken,
/// refused under the lock, refused without it, put off); and the code
/// lock's acquisitions, those that found it held, the microseconds
/// they waited and the longest wait.
fn board_code(
    now: &stela_engine::machine::engine::CodeReport,
    was: &stela_engine::machine::engine::CodeReport,
    tsc_hz: u64,
) -> String {
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    // By what took the lock: acquisitions, microseconds held and waited,
    // and the longest hold.
    let mut sites = String::from(" lock_sites=acquired/held_us/waited_us/longest_held_us");
    for site in stela_engine::machine::engine::LockSite::ALL {
        let (now, was) = (now.lock.sites[site as usize], was.lock.sites[site as usize]);
        sites.push_str(&format!(
            " {}:{}/{}/{}/{}",
            site.name(),
            now.acquired.saturating_sub(was.acquired),
            micros(now.held.saturating_sub(was.held)),
            micros(now.waited.saturating_sub(was.waited)),
            micros(now.longest_held),
        ));
    }
    // Block table entries jump cache hits would add: taken, refused under
    // the lock, refused without it, put off.
    let (targets, was_targets) = (now.targets, was.targets);
    let targets = format!(
        " targets=inserted/raced/unindexed/deferred:{}/{}/{}/{}",
        targets.inserted.saturating_sub(was_targets.inserted),
        targets.raced.saturating_sub(was_targets.raced),
        targets.unindexed.saturating_sub(was_targets.unindexed),
        targets.deferred.saturating_sub(was_targets.deferred),
    );
    // Installs: never on the board before; again after their block went
    // with its ring part, an IC IVAU, a part taken for want of a slot or
    // anything else; IC IVAU ones of unchanged bytes; again within one
    // turn of the ring, four, or later; at a frame offset that held a
    // block of another PC, or of the same PC in another variant; ghosts
    // that took another block's ghost slot (re-installs seen as new).
    let churn = now.churn.since(&was.churn);
    let churn = format!(
        " churn=new/ring/ivau/slot/other:{}/{}/{}/{}/{} same_bytes={} \
         ages=turn/four/older:{}/{}/{} aliases=pc/variant:{}/{} ghosts_overwritten={} \
         rebased=hits/kept/unstorable/kib:{}/{}/{}/{}",
        churn.new,
        churn.again[0],
        churn.again[1],
        churn.again[2],
        churn.again[3],
        churn.same_bytes,
        churn.ages[0],
        churn.ages[1],
        churn.ages[2],
        churn.aliases,
        churn.variants,
        churn.overwritten,
        now.rebases.hits.saturating_sub(was.rebases.hits),
        now.rebases.kept.saturating_sub(was.rebases.kept),
        now.rebases
            .unstorable
            .saturating_sub(was.rebases.unstorable),
        now.rebases.bytes / 1024,
    );
    // The parts young and tenured code hold, installs sent to the tenured
    // generation, and parts taken away: the oldest young one, the tenured
    // one entered least, a tenured one past its age, one mostly holes.
    let outs: [u64; 4] = core::array::from_fn(|why| now.outs[why].saturating_sub(was.outs[why]));
    let churn = format!(
        "{churn} gens=young/tenured:{}/{} promoted={} out=young/cold/aged/holes:{}/{}/{}/{}",
        now.generations[0],
        now.generations[1],
        now.promoted.saturating_sub(was.promoted),
        outs[0],
        outs[1],
        outs[2],
        outs[3],
    );
    format!(
        " board={}M/{}/{} retired={} forced={} lost={} redone/refused={}/{} \
         grace_waits={}/{}{churn} \
         retire_longest=us/steps:{}/{}{targets} \
         lock=acquired/contended/waited_us/longest_us:{}/{}/{}/{}{sites}",
        now.window >> 20,
        now.parts,
        now.generation,
        now.retired.saturating_sub(was.retired),
        now.forced.saturating_sub(was.forced),
        now.lost.saturating_sub(was.lost),
        now.redone.saturating_sub(was.redone),
        now.refused.saturating_sub(was.refused),
        now.grace_waits.saturating_sub(was.grace_waits),
        now.overdue.saturating_sub(was.overdue),
        micros(now.retire_ticks),
        now.retire_steps,
        now.lock.acquired.saturating_sub(was.lock.acquired),
        now.lock.contended.saturating_sub(was.lock.contended),
        micros(now.lock.waited.saturating_sub(was.lock.waited)),
        micros(now.lock.longest),
    )
}

/// `HEAP small=310M large=62M 128K:352/73M cores=300M/20M classes=16:8M,…`:
/// the heap's live bytes in size-class blocks and free-list allocations,
/// the latter by power of two where a size holds a megabyte or more, each
/// tagged core's share, the tagged cores' bytes by size class, the failed
/// allocations so far and whether the heap is low.
fn heap_report(heap: &stela_boot_core::heap::Usage) {
    let large: usize = heap.large.iter().map(|&(_, bytes)| bytes).sum();
    let mut line = format!(
        "HEAP small={}M large={}M",
        heap.live.saturating_sub(large) >> 20,
        large >> 20
    );
    for (bucket, &(count, bytes)) in heap.large.iter().enumerate() {
        if bytes >= 1 << 20 {
            let size = 4096usize << bucket;
            let size = if size >= 1 << 20 {
                format!("{}M", size >> 20)
            } else {
                format!("{}K", size >> 10)
            };
            line.push_str(&format!(" {size}:{count}/{}M", bytes >> 20));
        }
    }
    let cores: Vec<String> = crate::ALLOCATOR
        .owned()
        .map(|own| format!("{}M", own >> 20))
        .collect();
    line.push_str(&format!(" cores={}", cores.join("/")));
    let classes: Vec<String> = crate::ALLOCATOR
        .class_live()
        .iter()
        .enumerate()
        .map(|(class, &bytes)| format!("{}:{}M", 16 << class, bytes >> 20))
        .collect();
    line.push_str(&format!(" classes={}", classes.join(",")));
    // Allocations that found no memory, which fallible callers survive, and
    // whether the heap is low enough that the engine gives memory back.
    line.push_str(&format!(
        " failures={} low={}",
        crate::ALLOCATOR.failures(),
        crate::ALLOCATOR.low()
    ));
    output::arguments(format_args!("{line}\n"));
    owner_report(heap.live);
    super::large_blocks::report();
}

/// `HEAP OWNERS linux=40/52M libc=900/1350M rest=600M`: the heap bytes
/// the Linux environment and the C library (Mesa, NAK, LLVM) hold now and
/// at most, and the rest of the live heap (engine, devices, tables).
fn owner_report(live: usize) {
    let (linux, linux_peak) = stela_linux::heap::usage();
    #[cfg(feature = "nvk")]
    let (libc, libc_peak) = super::gpu::client::LIBC_HEAP.usage();
    #[cfg(not(feature = "nvk"))]
    let (libc, libc_peak) = (0, 0);
    output::arguments(format_args!(
        "HEAP OWNERS linux={}/{}M libc={}/{}M rest={}M\n",
        linux >> 20,
        linux_peak >> 20,
        libc >> 20,
        libc_peak >> 20,
        live.saturating_sub(linux + libc) >> 20,
    ));
}

/// `(maintenance=9,memory=2)`: the causes of the discards so far, or
/// nothing before the first.
fn discard_causes(counts: &[u64]) -> String {
    let causes: Vec<String> = stela_engine::machine::engine::Discard::ALL
        .iter()
        .zip(counts)
        .filter(|&(_, &count)| count != 0)
        .map(|(cause, count)| format!("{}={count}", cause.name()))
        .collect();
    if causes.is_empty() {
        String::new()
    } else {
        format!("({})", causes.join(","))
    }
}

/// The heartbeat's position and engine lines: the boot PE's place, the
/// ticks per block and compiled share, compilation, discards, chaining and
/// events over the interval, the heap and the shadow and code pools.
pub(super) fn beat_report<H: Host>(
    machine: &Machine<H, parallel::Board>,
    last: &Beat,
    next: &Beat,
    elapsed: u64,
) {
    let cpu = &machine.cpu;
    let blocks = (next.blocks - last.blocks).max(1);
    let heap = crate::ALLOCATOR.usage();
    let shadow = host::statistics();
    output::arguments(format_args!(
        "BEAT blocks={} pc={:#x} {}{} lr={:#x} ticks/block={} native={}% \
         compiled={} discards={}{} evictions={} unlinks={} chained={}% exceptions={} interrupts={} \
         waits={} \
         heap={}M/{}M carved={}M peak={}M largest={}M extents={} carves={} \
         shadow_peak={}M/{}M/{}M/{}M exhausted={} code_peak={}M/{}M\n",
        next.blocks,
        cpu.pc,
        level_name(cpu.pstate.guarded),
        cpu.pstate.el,
        cpu.state.x[30],
        elapsed / blocks,
        (next.native - last.native) * 100 / elapsed.max(1),
        next.compiled - last.compiled,
        machine.statistics.code_discards.iter().sum::<u64>(),
        discard_causes(&machine.statistics.code_discards),
        machine.statistics.code_evictions,
        machine.statistics.unlinks,
        (next.chained - last.chained) * 100 / blocks,
        next.events.0 - last.events.0,
        next.events.1 - last.events.1,
        next.events.2 - last.events.2,
        heap.live >> 20,
        heap.total >> 20,
        heap.carved >> 20,
        heap.peak >> 20,
        heap.largest >> 20,
        heap.extents,
        heap.carves,
        shadow.shadow_peak[0] >> 20,
        shadow.shadow_peak[1] >> 20,
        shadow.shadow_bytes[0] >> 20,
        shadow.shadow_bytes[1] >> 20,
        shadow.exhausted,
        shadow.code_peak >> 20,
        shadow.code_bytes >> 20,
    ));
    heap_report(&heap);
    output::arguments(format_args!(
        "REGISTERS pe={} pc={:#x} x19={:#x} x20={:#x} x21={:#x} x22={:#x} x23={:#x} lr={:#x}\n",
        machine.processor(),
        cpu.pc,
        cpu.state.x[19],
        cpu.state.x[20],
        cpu.state.x[21],
        cpu.state.x[22],
        cpu.state.x[23],
        cpu.state.x[30],
    ));
}

/// `native-profile`'s heartbeat lines: the BSP's reference ticks, the word
/// and work profiles, the board's and the home PE's native statistics.
#[cfg(feature = "native-profile")]
pub(super) fn native_profile_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    if machine.runs_parallel() && machine.step_timing().is_some() {
        parallel::report_stored_pages(machine, host::fresh_counter());
    }
    output::arguments(format_args!(
        "NATIVE PROFILE bsp_reference_ticks {:?}\n",
        host::profile_snapshot()
    ));
    host::profile_word_report();
    host::profile_work_report();
    output::arguments(format_args!(
        "NATIVE BOARD {:?}\n",
        machine.devices.with(|devices| devices.advance_statistics())
    ));
    let pe = machine.processor();
    output::arguments(format_args!(
        "NATIVE DATA TLB pe={pe} {:?}\n",
        machine.native_data_tlb_statistics()
    ));
    output::arguments(format_args!(
        "NATIVE EPOCH pe={pe} {:?}\n",
        machine.native_epoch_statistics()
    ));
    output::arguments(format_args!(
        "NATIVE SOURCE pe={pe} {:?}\n",
        machine.native_source_statistics()
    ));
}

/// Observe the current instruction's source range after all native owners
/// have recovered. This walker can read RAM descriptors only: diagnostic
/// failures neither update guest tables nor raise a guest exception.
pub(super) fn observed_instruction<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
) -> Option<u64> {
    use stela_engine::machine::mmu;
    use stela_engine::machine::mmu::Access;
    use stela_engine::machine::mmu::Outcome;
    use stela_engine::machine::mmu::TableMemory;
    struct ReadOnlyTables<'a, H>(&'a mut H);
    impl<H: Host> TableMemory for ReadOnlyTables<'_, H> {
        fn read_descriptor(&mut self, physical: u64) -> Option<u64> {
            let mut bytes = [0; size_of::<u64>()];
            self.0
                .read_ram(physical, &mut bytes)
                .then(|| u64::from_le_bytes(bytes))
        }

        fn update_descriptor(&mut self, _: u64, _: u64, _: u64) -> Option<bool> {
            None
        }
    }
    match mmu::translate(
        &machine.cpu.controls(machine.cpu.regime()),
        machine.cpu.pc,
        Access::Execute,
        machine.cpu.requester(),
        &mut ReadOnlyTables(&mut machine.host),
    ) {
        Outcome::Mapped(translation) => Some(translation.physical),
        _ => None,
    }
}

/// The PC and return addresses a call chain sample keeps.
pub(super) const CHAIN: usize = 6;

/// `pc` and the return addresses of the frame records from `frame`,
/// without their PACs; zero past the chain's end.
pub(super) fn call_chain<const N: usize, H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    pc: u64,
    mut frame: u64,
) -> [u64; N] {
    let mut chain = [0; N];
    chain[0] = pc;
    for slot in &mut chain[1..] {
        let mut record = [0u8; 16];
        if frame == 0 || !frame.is_multiple_of(8) || !machine.read_virtual(frame, &mut record) {
            break;
        }
        let link = u64::from_le_bytes(record[8..].try_into().expect("8 bytes"));
        *slot = machine.strip_instruction_pointer(link);
        frame = u64::from_le_bytes(record[..8].try_into().expect("8 bytes"));
    }
    chain
}

/// Heartbeats between reports of the suspended guest threads.
pub(super) const SUSPENSION_BEATS: u64 = 6;

/// Heartbeats between reports of the home PE's code bookkeeping, which
/// walk its blocks.
const FOOTPRINT_BEATS: u64 = 6;

/// `ENGINE MEMORY pe=0 blocks=36M owners=12M …`: the heap bytes of the home
/// PE's code bookkeeping, by structure ([`Machine::footprint`]).
fn footprint_report<H: Host>(
    machine: &Machine<H, parallel::Board>,
    footprint: &stela_engine::machine::engine::Footprint,
) {
    output::arguments(format_args!(
        "ENGINE MEMORY pe={} blocks={}M owners={}M successors={}M rows={}M index={}M lookup={}M pages={}M\n",
        machine.processor(),
        footprint.blocks >> 20,
        footprint.owners >> 20,
        footprint.successors >> 20,
        footprint.rows >> 20,
        footprint.index >> 20,
        footprint.lookup >> 20,
        footprint.pages >> 20,
    ));
}

/// The return addresses a suspended thread's chain keeps.
pub(super) const SUSPENSION_CHAIN: usize = 12;

/// The suspended guest threads grouped by where they wait: each distinct
/// chain of the link register and the frame records, with its thread count
/// and the seconds since the most recent of them stopped. A guest that idles
/// without progress shows here the wait it never leaves.
pub(super) fn suspension_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    const SHOWN: usize = 32;
    let now = machine.host.counter();
    let frequency = stela_engine::machine::timer::system_frequency(&machine.cpu).max(1);
    let suspended: Vec<_> = machine.suspended().values().copied().collect();
    let mut chains: Vec<([u64; SUSPENSION_CHAIN], u64)> = suspended
        .iter()
        .map(|suspension| {
            let link = machine.strip_instruction_pointer(suspension.link);
            (call_chain(machine, link, suspension.frame), suspension.at)
        })
        .collect();
    chains.sort_unstable();
    // Each chain's thread count and the latest suspension among them.
    let mut groups: Vec<(u64, usize, [u64; SUSPENSION_CHAIN])> = chains
        .chunk_by(|a, b| a.0 == b.0)
        .map(|run| {
            (
                run.iter().map(|&(_, at)| at).max().unwrap_or(0),
                run.len(),
                run[0].0,
            )
        })
        .collect();
    groups.sort_unstable_by(|a, b| b.cmp(a));
    output::arguments(format_args!(
        "SUSPENDED threads={} chains={}\n",
        suspended.len(),
        groups.len()
    ));
    for (at, count, chain) in groups.into_iter().take(SHOWN) {
        output::arguments(format_args!(
            "SUSPENSION x{count} age={}s",
            now.saturating_sub(at) / frequency
        ));
        for address in chain.into_iter().take_while(|&address| address != 0) {
            output::arguments(format_args!(" {address:#x}"));
        }
        output::arguments(format_args!("\n"));
    }
}

/// The share of the heartbeat's time-spaced samples at EL0 and the call
/// chains they found most often, which show where a guest without output
/// spends its time.
pub(super) fn sample_report(samples: &mut Vec<[u64; CHAIN]>, user: u32) {
    const SHOWN: usize = 3;
    samples.sort_unstable();
    let mut counts: Vec<(usize, [u64; CHAIN])> = samples
        .chunk_by(|a, b| a == b)
        .map(|run| (run.len(), run[0]))
        .collect();
    counts.sort_unstable_by(|a, b| b.cmp(a));
    output::arguments(format_args!("SAMPLES el0={user}/{}\n", samples.len()));
    for (count, chain) in counts.into_iter().take(SHOWN) {
        output::arguments(format_args!("SAMPLE x{count}"));
        for address in chain.into_iter().take_while(|&address| address != 0) {
            output::arguments(format_args!(" {address:#x}"));
        }
        output::arguments(format_args!("\n"));
    }
    samples.clear();
}

/// The longest each part of the service between two slices took since the
/// last beat: with every PE on a core of its own, the boot PE's guest time
/// passes meanwhile.
pub(super) struct ServiceParts {
    pub(super) at: u64,
    pub(super) longest: [u64; Self::PARTS],
    /// Each part's ticks in all since the last beat.
    pub(super) total: [u64; Self::PARTS],
    /// The guest's slices' ticks since the last beat.
    pub(super) sliced: u64,
}

impl ServiceParts {
    const PARTS: usize = 6;
    const REPORTS: usize = 0;
    const SERVICE: usize = 1;
    const BEAT: usize = 2;
    /// The host's Linux work, which the GPU's interrupts and timers bring.
    const PUMP: usize = 3;
    /// The firmware framebuffer's writes of presented frames.
    const DISPLAY: usize = 4;
    /// The staged guest disks' steps the boot PE takes ([`super::storage`]).
    const DISK: usize = 5;
    const NAMES: [&str; Self::PARTS] = ["reports", "service", "beat", "pump", "display", "disk"];

    pub(super) fn new(now: u64) -> Self {
        Self {
            at: now,
            longest: [0; Self::PARTS],
            total: [0; Self::PARTS],
            sliced: 0,
        }
    }

    pub(super) fn start(&mut self, now: u64) {
        self.sliced += now.wrapping_sub(self.at);
        self.at = now;
    }

    /// The part `part` ended at `now`.
    pub(super) fn lap(&mut self, part: usize, now: u64) {
        let spent = now.wrapping_sub(self.at);
        self.longest[part] = self.longest[part].max(spent);
        self.total[part] += spent;
        self.at = now;
    }

    pub(super) fn report(&self, tsc_hz: u64) -> String {
        let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz);
        let mut line = String::from("longest_us");
        for (name, ticks) in Self::NAMES.iter().zip(self.longest) {
            line.push_str(&format!(" {name}={}", micros(ticks)));
        }
        line.push_str(&format!(" total_us sliced={}", micros(self.sliced)));
        for (name, ticks) in Self::NAMES.iter().zip(self.total) {
            line.push_str(&format!(" {name}={}", micros(ticks)));
        }
        line
    }
}

/// What each of the heartbeat's reports took in all, which holds the boot
/// PE's guest up, and its longest stage or step.
pub(super) struct BeatCost {
    pub(super) parts: Vec<(&'static str, u64)>,
    pub(super) steps: u32,
    pub(super) longest: u64,
}

impl BeatCost {
    pub(super) fn new() -> Self {
        Self {
            parts: Vec::new(),
            steps: 0,
            longest: 0,
        }
    }

    /// The report `name` took `ticks` more.
    pub(super) fn add(&mut self, name: &'static str, ticks: u64) {
        match self.parts.iter_mut().find(|(part, _)| *part == name) {
            Some((_, total)) => *total += ticks,
            None => self.parts.push((name, ticks)),
        }
    }

    /// One stage or step, run at a service boundary of its own, took `ticks`.
    pub(super) fn step(&mut self, ticks: u64) {
        self.steps += 1;
        self.longest = self.longest.max(ticks);
    }

    /// `BEAT COST us name=total … steps=N longest_step=us`.
    pub(super) fn report(&self, tsc_hz: u64) {
        let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
        let mut line = String::from("BEAT COST us");
        for (name, ticks) in &self.parts {
            line.push_str(&format!(" {name}={}", micros(*ticks)));
        }
        line.push_str(&format!(
            " steps={} longest_step={}",
            self.steps,
            micros(self.longest)
        ));
        output::arguments(format_args!("{line}\n"));
    }
}
