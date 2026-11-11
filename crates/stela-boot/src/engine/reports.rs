//! What a stopped or stalled machine reports: exits, frames, memory,
//! translations and registers.

use super::*;

/// One sampled PC: how often it was seen and the code around it, which names
/// a user-space image the serial log has no map of.
pub(super) struct HotPc {
    pub(super) count: u32,
    /// `HOT_CODE_BYTES` from `HOT_CODE_BEFORE` before the PC; zero if unread.
    pub(super) code: [u8; HOT_CODE_BYTES],
}

pub(super) const HOT_CODE_BYTES: usize = 32;
pub(super) const HOT_CODE_BEFORE: u64 = 16;

/// The PCs the running PE was at most often at the heartbeat's service
/// boundaries, sampled at most once a millisecond: where guest time goes.
pub(super) fn hot_report(hot: &mut BTreeMap<u64, HotPc>) {
    const SHOWN: usize = 16;
    const SHOWN_CODE: usize = 6;
    let total: u32 = hot.values().map(|sample| sample.count).sum();
    let mut counts: Vec<(u32, u64)> = hot.iter().map(|(&pc, s)| (s.count, pc)).collect();
    counts.sort_unstable_by(|a, b| b.cmp(a));
    output::arguments(format_args!("HOT samples={total}"));
    for &(count, pc) in counts.iter().take(SHOWN) {
        output::arguments(format_args!(" {pc:#x}:{count}"));
    }
    output::arguments(format_args!("\n"));
    for &(_, pc) in counts.iter().take(SHOWN_CODE) {
        output::arguments(format_args!("HOT CODE {pc:#x} "));
        for byte in hot[&pc].code {
            output::arguments(format_args!("{byte:02x}"));
        }
        output::arguments(format_args!("\n"));
    }
    hot.clear();
}

/// The counter, PSTATE.DAIF and each generic timer's CTL and CVAL, which
/// show why a PE that should sleep in WFI does not.
pub(super) fn timer_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    use stela_engine::machine::timer::CHANNELS;
    let (now, timers) = machine.timer_snapshot();
    output::arguments(format_args!(
        "TIMERS now={now:#x} daif={:#x}",
        machine.cpu.pstate.daif
    ));
    for (channel, (control, compare)) in CHANNELS.into_iter().zip(timers) {
        output::arguments(format_args!(
            " {:?}={:#x}/{:#x}",
            channel.control, control, compare
        ));
    }
    output::arguments(format_args!("\n"));
}

/// Whether the guest has yet been seen to reach addresses outside the
/// host's native ranges.
#[derive(Default)]
pub(super) struct TranslatedSeen {
    pub(super) reported: bool,
}

impl TranslatedSeen {
    /// Once per boot, the first guest PC and address that needed the
    /// translated variant, or that fell in the four-level guard, with the
    /// counts so far: such a guest runs correctly, through the translation
    /// helper, but slower, and may call for a different placement.
    pub(super) fn report<H: Host>(&mut self, machine: &Machine<H, parallel::Board>) {
        let host = host::statistics();
        let first = machine.first_translated();
        if self.reported || (first.is_none() && host.guard_faults == 0) {
            return;
        }
        self.reported = true;
        let (pc, address) = first.unwrap_or_default();
        output::arguments(format_args!(
            "TRANSLATED pc={pc:#x} address={address:#x} pages={} helper_calls={} \
             helper_fills={} bounces={} guard_faults={}\n",
            machine.statistics.translated_pages,
            host.helper_calls,
            host.helper_fills,
            host.bounces,
            host.guard_faults,
        ));
    }
}

/// The host's shadow counters at the last heartbeat.
#[derive(Default)]
pub(super) struct ShadowSeen {
    pub(super) counts: [u64; 13],
    pub(super) causes: [u64; 7],
    pub(super) fields: [u64; 9],
}

impl ShadowSeen {
    /// Since the last heartbeat: the shadow faults filled and the pages
    /// mapped beside them; the resets of the whole shadow, and of those
    /// the ones each cause led to (a half running out of frames, SPRR,
    /// TTBR0 or other controls, tag checks, invalidations and TLBIs of
    /// everything, and for other controls the fields that differed); the
    /// resets of the lower half, its switches to another TTBR0, those that
    /// found the half kept, and the kept halves dropped; the switches
    /// between the roots of SPRR_PERM_EL0, SPRR_PERM_EL1 and E0PD1 values
    /// and those that found none kept; the TLBIs that unmapped ranges
    /// instead; and the root loads for a change of view.
    pub(super) fn report(&mut self) {
        let host = host::statistics();
        let counts = [
            host.fills,
            host.prefills,
            host.resets,
            host.exhausted,
            host.lower_resets,
            host.lower_switches,
            host.lower_found,
            host.kept_dropped,
            host.bank_switches,
            host.bank_misses,
            host.scoped,
            host.view_switches,
            host.fill_ns,
        ];
        let [
            fills,
            prefills,
            resets,
            exhausted,
            lower_resets,
            lower_switches,
            lower_found,
            kept_dropped,
            bank_switches,
            bank_misses,
            scoped,
            view_switches,
            fill_ns,
        ] = core::array::from_fn(|index| counts[index] - self.counts[index]);
        self.counts = counts;
        let fill_ns_each = fill_ns / fills.max(1);
        let [_, sprr, table, controls, tags, invalidated, maintained] =
            core::array::from_fn(|index| host.reset_causes[index] - self.causes[index]);
        self.causes = host.reset_causes;
        let [
            sctlr,
            tcr,
            ttbr0,
            ttbr1,
            mair,
            sprr_field,
            ctrr,
            regime,
            stage2,
        ] = core::array::from_fn(|index| host.control_fields[index] - self.fields[index]);
        self.fields = host.control_fields;
        output::arguments(format_args!(
            "SHADOW fills={fills} prefills={prefills} resets={resets} exhausted={exhausted} \
             sprr={sprr} table={table} controls={controls} (sctlr={sctlr} tcr={tcr} \
             ttbr0={ttbr0} ttbr1={ttbr1} mair={mair} sprr={sprr_field} ctrr={ctrr} \
             regime={regime} stage2={stage2}) tags={tags} \
             invalidated={invalidated} maintained={maintained} \
             lower_resets={lower_resets} lower_switches={lower_switches} \
             lower_found={lower_found} kept_dropped={kept_dropped} \
             bank_switches={bank_switches} bank_misses={bank_misses} scoped={scoped} \
             view_switches={view_switches} fill_ns_each={fill_ns_each}\n"
        ));
        if let Some((from, to)) = host.last_controls
            && controls > 0
        {
            output::arguments(format_args!(
                "SHADOW LAST tcr={:#x}->{:#x} ttbr0={:#x}->{:#x} ttbr1={:#x}->{:#x} \
                 sprr={:#x}/{:#x}/{:#x}/{:#x}->{:#x}/{:#x}/{:#x}/{:#x} regime={:?}\n",
                from.tcr,
                to.tcr,
                from.ttbr0,
                to.ttbr0,
                from.ttbr1,
                to.ttbr1,
                from.sprr.config,
                from.sprr.el1,
                from.sprr.el0,
                from.sprr.fields[0] | from.sprr.fields[1] << 32,
                to.sprr.config,
                to.sprr.el1,
                to.sprr.el0,
                to.sprr.fields[0] | to.sprr.fields[1] << 32,
                to.regime,
            ));
        }
    }
}

/// The engine's entries into compiled code, its exits by kind and the
/// exits by instruction word at the last heartbeat.
#[derive(Default)]
pub(super) struct ExitsSeen {
    pub(super) entries: u64,
    pub(super) kinds: [u64; EXIT_KINDS],
    pub(super) words: BTreeMap<u32, u64>,
    /// Exits sent to lookup dispatchers, lookup hits, and the events that drop cached translations
    /// and lookups: translation control writes, TLBIs, TTBR0 writes and
    /// IC IVAU.
    pub(super) lookups: [u64; 2],
    pub(super) drops: [u64; 4],
    /// `Statistics::lookup_misses` and `lookup_forgets`.
    pub(super) misses: [u64; 5],
    pub(super) forgets: [u64; 4],
    /// `Statistics::native_transitions`.
    pub(super) transitions: [u64; 4],
    /// `Machine::device_registers`.
    pub(super) registers: BTreeMap<u64, DeviceRegister>,
    /// `Machine::system_words`.
    pub(super) system: BTreeMap<u32, DeviceRegister>,
}

/// The `ExitKind`s `Statistics::exit_kinds` counts.
pub(super) const EXIT_KINDS: usize = 13;

/// The exits by word since the last heartbeat, found in steps
/// ([`ExitsSeen::diff_step`]): the counts now, the last word compared, and
/// the words whose count changed with the change.
pub(super) struct WordsDiff {
    words: BTreeMap<u32, u64>,
    after: Option<u32>,
    top: Vec<(u64, u32)>,
    /// Whether `words` came from a walk of the blocks: without `--profile`
    /// the beat skips it ([`Self::unwalked`]).
    walked: bool,
}

impl WordsDiff {
    /// `words` is [`Machine::exit_words`], gathered in steps.
    pub(super) fn new(words: BTreeMap<u32, u64>) -> Self {
        Self {
            words,
            after: None,
            top: Vec::new(),
            walked: true,
        }
    }

    /// No walk this beat: `last` is [`ExitsSeen::words`], kept as it is, so
    /// no word shows a change and the next walk compares with the last one.
    pub(super) fn unwalked(last: BTreeMap<u32, u64>) -> Self {
        Self {
            words: last,
            after: None,
            top: Vec::new(),
            walked: false,
        }
    }
}

impl ExitsSeen {
    /// Compare the counts now with the last heartbeat's from the word after
    /// the last one compared, both maps side by side in word order, asking
    /// `more` every 256 words; whether every word was compared.
    pub(super) fn diff_step(&self, diff: &mut WordsDiff, mut more: impl FnMut() -> bool) -> bool {
        use core::ops::Bound::{Excluded, Unbounded};
        if !diff.walked {
            return true;
        }
        let start = diff.after.map_or(Unbounded, Excluded);
        let mut last = self.words.range((start, Unbounded)).peekable();
        for (compared, (&word, &count)) in diff.words.range((start, Unbounded)).enumerate() {
            if compared != 0 && compared % 256 == 0 && !more() {
                return false;
            }
            while last.next_if(|&(&seen, _)| seen < word).is_some() {}
            let before = last
                .next_if(|&(&seen, _)| seen == word)
                .map_or(0, |(_, &count)| count);
            let count = count.saturating_sub(before);
            if count != 0 {
                diff.top.push((count, word));
            }
            diff.after = Some(word);
        }
        true
    }

    /// Since the last heartbeat: the native runs the engine started (those
    /// that did not end in a counted exit ran out of their chain budget),
    /// the exits by `ExitKind`, and the instruction words that most often
    /// left compiled code, so the engine's exit causes can be ranked, and
    /// the device registers whose native faults cost the most, each with its
    /// accesses and their handling time in microseconds.
    /// `diff` is the words' compared in full ([`Self::diff_step`]).
    pub(super) fn report<H: Host>(
        &mut self,
        machine: &Machine<H, parallel::Board>,
        diff: WordsDiff,
        tsc_hz: u64,
    ) {
        const SHOWN: usize = 24;
        const REGISTERS: usize = 16;
        let statistics = &machine.statistics;
        let entries = statistics.blocks - statistics.chained;
        let kinds = statistics.exit_kinds;
        let lookups = [statistics.lookups, statistics.lookup_hits];
        let misses = statistics.lookup_misses;
        let forgets = statistics.lookup_forgets;
        let transitions = statistics.native_transitions;
        let drops = [
            statistics.translation_changes,
            statistics.tlb_maintenance,
            statistics.table_base_changes,
            statistics.code_invalidations,
        ];
        let WordsDiff {
            words,
            mut top,
            walked,
            ..
        } = diff;
        // Only the shown words in order; the rest only add to the share.
        if top.len() > SHOWN {
            top.select_nth_unstable_by(SHOWN, |a, b| b.cmp(a));
        }
        let shown_len = top.len().min(SHOWN);
        top[..shown_len].sort_unstable_by(|a, b| b.cmp(a));
        let exits: u64 = kinds
            .iter()
            .zip(&self.kinds)
            .map(|(now, last)| now - last)
            .sum();
        output::arguments(format_args!(
            "EXITS entries={} exits={exits} kinds=",
            entries - self.entries
        ));
        for (index, (now, last)) in kinds.iter().zip(&self.kinds).enumerate() {
            let separator = if index == 0 { "" } else { "," };
            output::arguments(format_args!("{separator}{}", now - last));
        }
        output::arguments(format_args!(
            " thunks={} hits={} controls={} tlbi={} ttbr0={} ivau={}",
            lookups[0] - self.lookups[0],
            lookups[1] - self.lookups[1],
            drops[0] - self.drops[0],
            drops[1] - self.drops[1],
            drops[2] - self.drops[2],
            drops[3] - self.drops[3],
        ));
        // Unlinked, another PC in the slot, another context, no budget,
        // a refused BTYPE.
        for (index, (now, last)) in misses.iter().zip(&self.misses).enumerate() {
            let separator = if index == 0 { " misses=" } else { "," };
            output::arguments(format_args!("{separator}{}", now - last));
        }
        // Halves forgotten by an ASIDE1, a TLBI of everything, table scans
        // for a wide TLBI by VA, halves forgotten by anything else.
        for (index, (now, last)) in forgets.iter().zip(&self.forgets).enumerate() {
            let separator = if index == 0 { " forgets=" } else { "," };
            output::arguments(format_args!("{separator}{}", now - last));
        }
        // SVC, ERET, GENTER and GEXIT completed natively.
        for (index, (now, last)) in transitions.iter().zip(&self.transitions).enumerate() {
            let separator = if index == 0 { " native=" } else { "," };
            output::arguments(format_args!("{separator}{}", now - last));
        }
        if !walked {
            // The walk of every block for the exits by word runs with
            // `--profile` only.
            output::arguments(format_args!(" words=profile-only"));
        }
        // The share of the exits by word the shown words make up.
        let counted: u64 = top.iter().map(|&(count, _)| count).sum();
        let shown: u64 = top.iter().take(SHOWN).map(|&(count, _)| count).sum();
        output::arguments(format_args!(
            " shown={}%",
            (shown * 100).checked_div(counted).unwrap_or(100)
        ));
        for (count, word) in top.into_iter().take(SHOWN) {
            output::arguments(format_args!(" {word:#010x}:{count}"));
        }
        output::arguments(format_args!("\n"));
        let registers = machine.device_registers().clone();
        let mut top: Vec<_> = registers
            .iter()
            .map(|(physical, register)| {
                let last = self.registers.get(physical).copied().unwrap_or_default();
                let ticks = register.ticks - last.ticks;
                (ticks, register.accesses - last.accesses, *physical)
            })
            .filter(|&(_, count, _)| count != 0)
            .collect();
        top.sort_unstable_by(|a, b| b.cmp(a));
        let accesses: u64 = top.iter().map(|&(_, count, _)| count).sum();
        let shown: u64 = top.iter().take(REGISTERS).map(|&(_, count, _)| count).sum();
        output::arguments(format_args!(
            "DEVICE REGISTERS accesses={accesses} registers={} shown={}%",
            top.len(),
            (shown * 100).checked_div(accesses).unwrap_or(100)
        ));
        let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
        for (ticks, count, physical) in top.into_iter().take(REGISTERS) {
            output::arguments(format_args!(" {physical:#x}:{count}/{}us", micros(ticks)));
        }
        output::arguments(format_args!("\n"));
        let system = machine.system_words().clone();
        let mut top: Vec<_> = system
            .iter()
            .map(|(word, timed)| {
                let last = self.system.get(word).copied().unwrap_or_default();
                (
                    timed.ticks - last.ticks,
                    timed.accesses - last.accesses,
                    *word,
                )
            })
            .filter(|&(_, count, _)| count != 0)
            .collect();
        top.sort_unstable_by(|a, b| b.cmp(a));
        output::arguments(format_args!("SYSTEM WORDS"));
        for (ticks, count, word) in top.into_iter().take(REGISTERS) {
            output::arguments(format_args!(" {word:#010x}:{count}/{}us", micros(ticks)));
        }
        output::arguments(format_args!("\n"));
        *self = Self {
            system,
            entries,
            kinds,
            words,
            lookups,
            drops,
            misses,
            forgets,
            transitions,
            registers,
        };
    }
}

/// The instruction words that most often left compiled code for the engine.
pub(super) fn exit_report<H: Host>(machine: &Machine<H, parallel::Board>) {
    const SHOWN: usize = 16;
    let mut words: Vec<_> = machine
        .exit_words()
        .iter()
        .map(|(&word, &count)| (count, word))
        .collect();
    words.sort_unstable_by(|a, b| b.cmp(a));
    for (count, word) in words.into_iter().take(SHOWN) {
        output::arguments(format_args!("EXIT {word:#010x} x{count}\n"));
    }
}

/// The frame records from `frame` on (each the caller's X29 and the return
/// address, Arm's AAPCS64 frame chain), read through the running PE's
/// translation regime, which another PE's kernel frames share. Deep enough
/// to reach a kernel caller through nested lock and interrupt frames.
pub(super) fn frames<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    label: &str,
    mut frame: u64,
) {
    for depth in 0..64 {
        let mut record = [0u8; 16];
        if frame == 0 || !frame.is_multiple_of(8) || !machine.read_virtual(frame, &mut record) {
            break;
        }
        let next = u64::from_le_bytes(record[..8].try_into().expect("8 bytes"));
        let link = u64::from_le_bytes(record[8..].try_into().expect("8 bytes"));
        output::arguments(format_args!("{label} {depth} fp={frame:#x} lr={link:#x}\n"));
        frame = next;
    }
}

/// The guest virtual ranges `--dump` names, 16 bytes a line, through the
/// stopped PE's translation regime.
pub(super) fn dump_report<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    dumps: &[(Base, u64)],
) {
    for &(base, bytes) in dumps {
        let address = base.resolve(&machine.cpu.state.x);
        for line in (address..address + bytes).step_by(16) {
            let mut data = [0u8; 16];
            if !machine.read_virtual(line, &mut data) {
                output::arguments(format_args!("DUMP {line:#x}: unmapped\n"));
                break;
            }
            output::arguments(format_args!("DUMP {line:#x}:"));
            for word in data.as_chunks::<8>().0 {
                output::arguments(format_args!(" {:#018x}", u64::from_le_bytes(*word)));
            }
            output::arguments(format_args!("\n"));
        }
    }
}

pub(super) fn stack_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    frames(machine, "FRAME", machine.cpu.state.x[29]);
    let sp = machine.cpu.state.sp;
    for line in 0..32u64 {
        let mut bytes = [0u8; 16];
        if !machine.read_virtual(sp + line * 16, &mut bytes) {
            break;
        }
        output::arguments(format_args!("STACK {:#x}:", sp + line * 16));
        for byte in bytes {
            output::arguments(format_args!(" {byte:02x}"));
        }
        output::arguments(format_args!("  "));
        for byte in bytes {
            let shown = if byte.is_ascii_graphic() || byte == b' ' {
                byte
            } else {
                b'.'
            };
            output::arguments(format_args!("{}", char::from(shown)));
        }
        output::arguments(format_args!("\n"));
    }
    text_report(machine);
}

/// The format strings another PE's stack points to, from its last
/// materialized SP up: a PE inside `panic` keeps the panic's format and
/// arguments in its frames even when the console never prints them (a PE
/// that never answers the debugger cross-call holds it back). Only words
/// that point to at least 12 printable characters with a `%` are shown,
/// three per PE from 128 words.
pub(super) fn stack_text_report<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    pe: usize,
    sp: u64,
) {
    const WORDS: u64 = 128;
    const SHOWN: usize = 3;
    const LIMIT: usize = 160;
    let mut shown = 0;
    for word in 0..WORDS {
        let mut value = [0u8; 8];
        if !machine.read_virtual(sp.wrapping_add(word * 8), &mut value) {
            break;
        }
        let address = u64::from_le_bytes(value);
        if address >> 48 != 0xffff {
            continue;
        }
        let mut text = Vec::new();
        'read: while text.len() < LIMIT {
            let mut byte = [0u8; 1];
            if !machine.read_virtual(address.wrapping_add(text.len() as u64), &mut byte) {
                break;
            }
            let byte = byte[0];
            if byte == 0 || !(byte.is_ascii_graphic() || b" \t\n".contains(&byte)) {
                break 'read;
            }
            text.push(byte);
        }
        if text.len() >= 12 && text.contains(&b'%') {
            let text = core::str::from_utf8(&text).expect("ASCII");
            output::arguments(format_args!(
                "PE {pe} STACK TEXT sp+{:#x} {address:#x}: {text:?}\n",
                word * 8
            ));
            shown += 1;
            if shown == SHOWN {
                break;
            }
        }
    }
}

/// The text a general-purpose register points to, when it reads as at least
/// eight printable characters: a guest's panic message often sits in one at
/// a stop, while the console that would print it is off.
pub(super) fn text_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    const LIMIT: usize = 512;
    for register in 0..29 {
        let address = machine.cpu.state.x[register];
        let mut text = Vec::new();
        'read: while text.len() < LIMIT {
            let mut chunk = [0u8; 16];
            if !machine.read_virtual(address.wrapping_add(text.len() as u64), &mut chunk) {
                break;
            }
            for byte in chunk {
                if byte == 0 || !(byte.is_ascii_graphic() || b" \t\n".contains(&byte)) {
                    break 'read;
                }
                text.push(byte);
            }
        }
        if text.len() >= 8 {
            let text = core::str::from_utf8(&text).expect("ASCII");
            output::arguments(format_args!("TEXT x{register} {address:#x}: {text:?}\n"));
        }
    }
}

/// What the devices left for the boot PE's reports since the last slice:
/// the guest's console lines, the unsupported requests they answered, the
/// SEP's `--sep-observe` journal (a run keeps going past each observed
/// request and may end at the host timeout) and their logs.
pub(super) struct Drained {
    pub(super) lines: Vec<String>,
    pub(super) unmodeled: Vec<String>,
    pub(super) sep: Vec<String>,
    pub(super) usb: Vec<stela_soc::machine::UsbLog>,
    pub(super) usb_host: Vec<(u32, String)>,
    /// The controllers' recent transfer steps, after the guest reported on the
    /// mux control channel, which carries its errors.
    pub(super) usb_transfers: Vec<(u32, String)>,
    pub(super) smc: Vec<stela_soc::machine::SmcWriteLog>,
    pub(super) display: Vec<stela_soc::machine::DisplayLog>,
    pub(super) power: Vec<stela_soc::machine::PowerLog>,
    /// The watchdog mechanism's arms and disarms, and how many more were
    /// not kept.
    pub(super) watchdog: (Vec<stela_soc::wdt::MechanismEvent>, u32),
}

impl Drained {
    pub(super) fn take(devices: &mut AppleDevices) -> Self {
        let usb: Vec<_> = core::iter::from_fn(|| devices.take_usb_log()).collect();
        // Lost or refused mux data: what the core transferred before it.
        let mux_error = usb.iter().any(|log| {
            matches!(
                log,
                stela_soc::machine::UsbLog::MuxControl(_)
                    | stela_soc::machine::UsbLog::MuxRejected(_)
            )
        });
        Self {
            // The console that adds them is a device another PE can reach.
            lines: GUEST_LINES
                .0
                .try_borrow_mut()
                .map(|mut lines| core::mem::take(&mut *lines))
                .unwrap_or_default(),
            unmodeled: devices.take_unmodeled(),
            sep: devices.sep_observations(),
            usb_transfers: if mux_error {
                devices.usb_transfer_history()
            } else {
                Vec::new()
            },
            usb,
            usb_host: core::iter::from_fn(|| devices.take_usb_host_trace()).collect(),
            smc: core::iter::from_fn(|| devices.take_smc_log()).collect(),
            display: core::iter::from_fn(|| devices.take_display_log()).collect(),
            power: core::iter::from_fn(|| devices.take_power_log()).collect(),
            watchdog: devices.take_watchdog_mechanism_log(),
        }
    }
}

/// One line per watchdog mechanism write: XNU arms it on its halt, restart
/// and panic path, so the arm dates the start of a reset that follows. The
/// writing PE's pc and lr are its last materialized ones, close to the write
/// but not exact.
pub(super) fn watchdog_report<H: Host>(
    machine: &Machine<H, parallel::Board>,
    (events, dropped): (Vec<stela_soc::wdt::MechanismEvent>, u32),
) {
    let hub = machine.devices.hub();
    for event in events {
        let (pc, lr) = hub
            .source_observation(event.cpu)
            .map_or((0, 0), |observed| (observed.source.pc, observed.source.lr));
        // XNU moves the deadline forward every few seconds while it is well; only
        // an arm whose margin differs from the last one is news.
        if let Some(seconds) = event.deadline {
            let margin = u64::from(seconds)
                .wrapping_mul(COUNTER_HZ)
                .wrapping_sub(event.at)
                / COUNTER_HZ;
            if WATCHDOG_MARGIN
                .swap(margin, Ordering::Relaxed)
                .abs_diff(margin)
                <= 1
            {
                continue;
            }
        } else {
            WATCHDOG_MARGIN.store(u64::MAX, Ordering::Relaxed);
        }
        match event.deadline {
            Some(seconds) => output::arguments(format_args!(
                "WDT MECHANISM armed deadline_s={seconds} counter={} pe={} source_pc={pc:#x} source_lr={lr:#x}\n",
                event.at, event.cpu
            )),
            None => output::arguments(format_args!(
                "WDT MECHANISM disarmed counter={} pe={} source_pc={pc:#x} source_lr={lr:#x}\n",
                event.at, event.cpu
            )),
        }
    }
    if dropped > 0 {
        output::arguments(format_args!("WDT MECHANISM writes_not_kept={dropped}\n"));
    }
}

/// The margin in seconds of the last reported watchdog arm.
static WATCHDOG_MARGIN: AtomicU64 = AtomicU64::new(u64::MAX);
/// The guest counter's rate.
const COUNTER_HZ: u64 = 24_000_000;

/// The distinct unsupported operations `UNMODELED` lines have reported.
pub(super) static UNMODELED_REPORTED: AtomicU64 = AtomicU64::new(0);

/// Each unsupported operation answered degraded for the first time since
/// the last report: machine operations and device accesses, then
/// coprocessor requests answered with their firmware's error reply or left
/// unanswered.
pub(super) fn unmodeled_report<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    requests: Vec<String>,
) {
    static ORDER_NOTED: AtomicBool = AtomicBool::new(false);
    let requirements = machine.take_unmodeled();
    UNMODELED_REPORTED.fetch_add(
        (requirements.len() + requests.len()) as u64,
        Ordering::Relaxed,
    );
    if (!requirements.is_empty() || !requests.is_empty())
        && !ORDER_NOTED.swap(true, Ordering::Relaxed)
    {
        output::arguments(format_args!(
            "LOG ORDER these errors and trace/watch/device records are buffered; \
             guest console is immediate. Timestamps mark printing, not cross-stream event order.\n"
        ));
    }
    for requirement in requirements {
        let line = alloc::format!("{requirement}");
        if first_report(&line) {
            output::arguments(format_args!("UNMODELED {line}\n"));
        }
    }
    for line in requests {
        if first_report(&line) {
            output::arguments(format_args!("UNMODELED {line}\n"));
        }
    }
}

/// Lines [`first_report`] remembers; past these every line is printed.
const REPORTED_LINES: usize = 512;
/// The FNV-1a hashes of the `UNMODELED` lines printed so far, zero for an
/// empty place.
static REPORTED: [AtomicU64; REPORTED_LINES] = [const { AtomicU64::new(0) }; REPORTED_LINES];

/// Whether `line` was not printed before in this boot. Each PE and each
/// report window finds the same unsupported operation anew, and the log
/// held one operation's line once for each of them.
fn first_report(line: &str) -> bool {
    let hash = line
        .bytes()
        .fold(0xcbf2_9ce4_8422_2325u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3)
        })
        .max(1);
    for place in &REPORTED {
        match place.compare_exchange(0, hash, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return true,
            Err(kept) if kept == hash => return false,
            Err(_) => {}
        }
    }
    true
}

/// Each device register a survey answered for the first time since the last
/// report: guest PA, width, direction, the PC and link register that first
/// reached it, its count so far, the last value written and the refusal it replaced.
pub(super) fn survey_report<H: Host>(machine: &mut Machine<H, parallel::Board>) {
    for record in machine.take_survey() {
        output::arguments(format_args!(
            "SURVEY pa={:#x} bytes={} {} pc={:#x} lr={:#x} count={} value={:#x}: {}\n",
            record.physical,
            record.bytes,
            if record.write { "write" } else { "read" },
            record.pc,
            record.link,
            record.count,
            record.value,
            record.detail
        ));
    }
}

/// The running regime's TTBR1 occupancy (`Machine::upper_occupancy`)
/// whenever it differs from the last one shown: the level-1 ranges its
/// tables map and the outermost mapped 32 MiB, which decide whether a
/// four-level host's upper window can lie below the guest's mappings.
pub(super) fn occupancy_report<H: Host>(
    machine: &mut Machine<H, parallel::Board>,
    last: &mut Option<stela_engine::machine::engine::Occupancy>,
) {
    let Some(now) = machine.upper_occupancy() else {
        return;
    };
    if last.as_ref() == Some(&now) {
        return;
    }
    output::arguments(format_args!(
        "OCCUPANCY ttbr1={:#x} lowest={:#x} highest={:#x} ranges=",
        now.root,
        now.lowest.unwrap_or(0),
        now.highest.unwrap_or(0)
    ));
    for (first, last) in &now.ranges {
        output::arguments(format_args!(" {first:#x}-{last:#x}"));
    }
    output::arguments(format_args!("\n"));
    *last = Some(now);
}

pub(super) fn general_registers(cpu: &Cpu) {
    for (index, pair) in cpu.state.x.chunks(4).enumerate() {
        output::arguments(format_args!("X{}:", index * 4));
        for value in pair {
            output::arguments(format_args!(" {value:#018x}"));
        }
        output::arguments(format_args!("\n"));
    }
}

/// The translation cost since the last beat, `last` its compile ticks,
/// compiled instructions and stored blocks then, for the `compiled` blocks
/// compiled since: nanoseconds per block and per guest
/// instruction, and the blocks the translation store gave. The new `last`.
pub(super) fn translation_report<H: Host>(
    machine: &Machine<H, parallel::Board>,
    last: (u64, u64, u64),
    compiled: u64,
) -> (u64, u64, u64) {
    const NANOSECONDS: u128 = 1_000_000_000;
    let statistics = &machine.statistics;
    let now = (
        statistics.compile_ticks,
        statistics.compiled_instructions,
        statistics.stored,
    );
    let nanoseconds =
        u128::from(now.0.wrapping_sub(last.0)) * NANOSECONDS / u128::from(host::COUNTER_HZ);
    let instructions = now.1 - last.1;
    if compiled != 0 {
        output::arguments(format_args!(
            "TRANSLATION blocks={compiled} instructions={instructions} ns/block={} \
             ns/instruction={} stored={}\n",
            nanoseconds / u128::from(compiled),
            nanoseconds / u128::from(instructions.max(1)),
            now.2 - last.2,
        ));
    }
    now
}

/// Where every PE is: `R` running, `-` held in reset, `d` powered down, `P`
/// on its own machine, or a parked PE's PC and its wait (`i` WFI, `e` WFE,
/// `p` preempted), with the switch count.
pub(super) fn peers_report<H: Host>(machine: &Machine<H, parallel::Board>) {
    output::arguments(format_args!("PES switches={}", machine.statistics.switches));
    for (number, state) in machine.peer_states().into_iter().enumerate() {
        match state {
            PeerState::Running => output::arguments(format_args!(" {number}:R")),
            PeerState::Off => output::arguments(format_args!(" {number}:-")),
            PeerState::Remote => output::arguments(format_args!(" {number}:P")),
            PeerState::Down => output::arguments(format_args!(" {number}:d")),
            PeerState::Parked { cpu, wfi } => {
                let wait = match wfi {
                    Some(true) => 'i',
                    Some(false) => 'e',
                    None => 'p',
                };
                output::arguments(format_args!(" {number}:{:#x}/{wait}", cpu.pc));
            }
        }
    }
    output::arguments(format_args!("\n"));
}

/// A PE's PC and its EL1 and EL2 exception registers.
pub(super) struct Registers<'a>(pub(super) &'a Cpu);

impl fmt::Display for Registers<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let cpu = self.0;
        write!(
            f,
            "pc={:#x} el={} sp={:#x} daif={:#x} nzcv={:#x}",
            cpu.pc, cpu.pstate.el, cpu.state.sp, cpu.pstate.daif, cpu.state.nzcv,
        )?;
        let banks = [
            (
                1,
                [
                    Register::ELR_EL1,
                    Register::SPSR_EL1,
                    Register::ESR_EL1,
                    Register::FAR_EL1,
                    Register::VBAR_EL1,
                    Register::SCTLR_EL1,
                ],
            ),
            (
                2,
                [
                    Register::ELR_EL2,
                    Register::SPSR_EL2,
                    Register::ESR_EL2,
                    Register::FAR_EL2,
                    Register::VBAR_EL2,
                    Register::SCTLR_EL2,
                ],
            ),
        ];
        for (el, bank) in banks {
            let [elr, spsr, esr, far, vbar, sctlr] = bank.map(|register| cpu.read(register));
            write!(
                f,
                " el{el}: elr={elr:#x} spsr={spsr:#x} esr={esr:#x} far={far:#x} vbar={vbar:#x} sctlr={sctlr:#x}"
            )?;
        }
        for (level, bank) in cpu.apple.guarded.iter().enumerate() {
            write!(
                f,
                " gl{}: elr={:#x} spsr={:#x} esr={:#x} far={:#x} vbar={:#x}",
                level + 1,
                bank.elr,
                bank.spsr,
                bank.esr,
                bank.far,
                bank.vbar
            )?;
        }
        write!(
            f,
            " guarded={} cntkctl={:#x} cnthctl={:#x}",
            cpu.pstate.guarded,
            cpu.read(Register::CNTKCTL_EL1),
            cpu.read(Register::CNTHCTL_EL2)
        )
    }
}
