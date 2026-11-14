//! The automatic repair of a missing Metal plugin, `StelaGPU.bundle` in
//! `/Library/GPUBundles` on the Data volume.
//!
//! The StelaGPU kext writes the plugin's files from the copy it carries, but
//! the kernel gives a kext no way to create a directory (`files.h`:
//! `VNOP_MKDIR` is kernel-private and not in the kernel's symbol table the
//! kext is linked against), so on a
//! freshly installed system, whose Data volume holds only Apple's empty
//! `/Library/GPUBundles`, it reports the plugin missing. Nothing else can
//! create them before the first boot: the Data volume is encrypted, Stela's
//! APFS code only reads, and the restore runs Apple's own programs. A root
//! process of a normal boot is refused too: `/Library/GPUBundles` is
//! restricted, and the GPU policy's SIP grant keeps filesystem protection on.
//!
//! So a normal boot that finds the plugin missing records
//! [`BUNDLE_REPAIR_PENDING`] in NVRAM at its stop and restarts. The loader
//! turns that into one single-user boot with filesystem protection off
//! (`--bundle-repair`) and records [`BUNDLE_REPAIR_ATTEMPTED`] before it
//! boots, so neither a failure nor a power loss in the middle repeats the
//! repair by itself. In that boot the engine types [`steps`] at the guest's
//! root console ([`Driver`]) once the shell's prompt shows, as the manual
//! remedy does: it mounts the Data
//! volume, creates the plugin's directories root-owned and 0755, marks them
//! restricted, syncs, remounts it read-only and restarts quickly
//! ([`REBOOT`]). The next boot's kext then creates the
//! files in place. Only mounting and the directories are required: a
//! failing optional step is logged and the repair goes on. A failed
//! required step still syncs and restarts, and a repair whose shell stops
//! answering restarts the PC, so the next normal boot always tries the kext
//! once. The repair boot's stop records what it reported ([`Outcome`],
//! [`store_result`]). A boot that finds the plugin installed removes the
//! record; one that finds it missing after an attempt stops ([`decide`]),
//! naming that result ([`stop_reason`]), and `stela repair` turns the
//! attempt back into a request ([`again`]). The manual single-user remedy
//! is no advice: the guest's console takes input only from the PC's COM1,
//! which most PCs lack, and no `stela` command asks for that boot.
//!
//! A repair whose shell never took input, because no prompt came or the
//! shell left before the first step was typed ([`Failure::unanswered`]),
//! asks for one more maintenance boot: it records
//! [`BUNDLE_REPAIR_RETRY`] ([`retry`]), which the loader turns into
//! [`BUNDLE_REPAIR_RETRIED`] as it does a request, and a second such
//! failure is final like any other.
//!
//! A finished erase install knows the plugin is missing without a boot to
//! find it: it records [`BUNDLE_REPAIR_INSTALLED`] ([`Change::Installed`]),
//! which makes the first macOS boot the maintenance boot and is removed
//! before that boot starts. If that repair fails, the next normal boot finds
//! the plugin missing with no record and asks for the repair as above.
//!
//! The manual remedy's `xattr -w com.apple.rootless ...` is not a step: it
//! failed with EPERM in every manual installation (runs 20261106-070021,
//! 20261106-083218, 20261107-060417, and in this repair's first run), even
//! with filesystem protection off, and Metal loaded the plugin each time
//! without it. Whether Metal needs the restricted flag is unproven: every
//! manual installation set it, so the repair tries it but does not depend
//! on it.
use alloc::{
    format,
    string::{String, ToString},
    vec::Vec,
};
use stela_loader::boot_selection::{
    BUNDLE_REPAIR, BUNDLE_REPAIR_ATTEMPTED, BUNDLE_REPAIR_INSTALLED, BUNDLE_REPAIR_PENDING,
    BUNDLE_REPAIR_RESULT, BUNDLE_REPAIR_RETRIED, BUNDLE_REPAIR_RETRY, lookup,
};
use stela_soc_data::nor::nvram::{Partition, Variables};

/// The plugin's parent on the Data volume, as the kext looks for it.
pub const PLUGINS: &str = "/System/Volumes/Data/Library/GPUBundles";

/// The plugin's directories, parents first: the bundle layout
/// `cargo xtask guest` builds (`Contents/Info.plist`,
/// `Contents/MacOS/StelaGPU`), whose files the kext writes.
pub const DIRECTORIES: [&str; 3] = [
    "StelaGPU.bundle",
    "StelaGPU.bundle/Contents",
    "StelaGPU.bundle/Contents/MacOS",
];

/// What the kext found of the plugin (the gate's `BUNDLE` report).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Report {
    Current,
    Updated,
    Missing,
    Failed,
}

/// Which boot reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Boot {
    /// A multi-user boot, whose Metal clients need the plugin.
    Normal,
    /// A single-user boot an operator asked for (`--single-user`).
    SingleUser,
    /// The automatic repair's maintenance boot (`--bundle-repair`).
    Repair,
}

/// [`BUNDLE_REPAIR`] as NVRAM holds it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Record {
    None,
    Pending,
    Attempted,
    /// An erase install's request, which the loader removes when it takes
    /// it.
    Installed,
    /// A repair whose shell never took input asks for one more.
    Retry,
    /// The loader took [`Record::Retry`]: the second and last attempt.
    Retried,
    /// A value Stela never writes.
    Other,
}

impl Record {
    pub fn read(variables: &Variables) -> Self {
        match lookup(variables, BUNDLE_REPAIR)
            .map(|value| value.strip_suffix(b"\0").unwrap_or(value))
        {
            None => Self::None,
            Some(BUNDLE_REPAIR_PENDING) => Self::Pending,
            Some(BUNDLE_REPAIR_ATTEMPTED) => Self::Attempted,
            Some(BUNDLE_REPAIR_INSTALLED) => Self::Installed,
            Some(BUNDLE_REPAIR_RETRY) => Self::Retry,
            Some(BUNDLE_REPAIR_RETRIED) => Self::Retried,
            Some(_) => Self::Other,
        }
    }

    /// The record as NVRAM spells it, for messages.
    pub fn name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Pending => "pending",
            Self::Attempted => "attempted",
            Self::Installed => "installed",
            Self::Retry => "retry",
            Self::Retried => "retried",
            Self::Other => "a value Stela never writes",
        }
    }
}

/// The most bytes of [`BUNDLE_REPAIR_RESULT`] kept and shown.
pub const RESULT_BYTES: usize = 160;

/// What the last repair boot reported ([`BUNDLE_REPAIR_RESULT`]), as
/// printable text of at most [`RESULT_BYTES`]: the guest can write NVRAM
/// too.
pub fn result(variables: &Variables) -> Option<String> {
    let value = lookup(variables, BUNDLE_REPAIR_RESULT)?;
    let text: String = value
        .iter()
        .take(RESULT_BYTES)
        .filter(|byte| (b' '..=b'~').contains(*byte))
        .map(|&byte| char::from(byte))
        .collect();
    (!text.is_empty()).then_some(text)
}

/// Record `text` as the repair boot's result; whether `variables` changed.
pub fn store_result(variables: &mut Variables, text: &str) -> Result<bool, String> {
    let value = &text.as_bytes()[..text.len().min(RESULT_BYTES)];
    if lookup(variables, BUNDLE_REPAIR_RESULT) == Some(value) {
        return Ok(false);
    }
    variables
        .set(Partition::System, BUNDLE_REPAIR_RESULT, value)
        .map_err(|error| format!("{error:?}"))?;
    Ok(true)
}

/// What NVRAM's record becomes at the stop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    /// Ask the next boot to repair.
    Request,
    /// The plugin is installed: forget any repair.
    Forget,
    /// An erase install finished: the first macOS boot repairs.
    Installed,
    /// The repair's shell never took input: one more maintenance boot.
    Retry,
}

impl Change {
    /// Every change, in the order the engine keeps them by number.
    pub const ALL: [Self; 4] = [Self::Request, Self::Forget, Self::Installed, Self::Retry];

    /// Apply to `variables`; whether they changed, when the bank must be
    /// saved.
    pub fn apply(self, variables: &mut Variables) -> Result<bool, String> {
        let (record, value) = match self {
            Self::Request => (Record::Pending, BUNDLE_REPAIR_PENDING),
            Self::Installed => (Record::Installed, BUNDLE_REPAIR_INSTALLED),
            Self::Retry => (Record::Retry, BUNDLE_REPAIR_RETRY),
            Self::Forget => {
                let mut changed = false;
                for partition in [Partition::System, Partition::Common] {
                    for name in [BUNDLE_REPAIR, BUNDLE_REPAIR_RESULT] {
                        changed |= variables.remove(partition, name).is_some();
                    }
                }
                return Ok(changed);
            }
        };
        if Record::read(variables) == record {
            return Ok(false);
        }
        variables
            .set(Partition::System, BUNDLE_REPAIR, value)
            .map_err(|error| format!("{error:?}"))?;
        Ok(true)
    }
}

/// What a repair boot whose shell never took input ([`Failure::unanswered`])
/// changes, given the record this boot opened with: the first attempt, or
/// an erase install's whose record the loader removed, asks for one more;
/// the retry itself asks for nothing.
pub fn retry(record: Record) -> Option<Change> {
    matches!(record, Record::Attempted | Record::None).then_some(Change::Retry)
}

/// Let the automatic repair run again (`stela repair`): an attempt that did
/// not help, or a value Stela never writes, becomes a request for the next
/// macOS boot. A request still waiting, or no record, is left as it is: a
/// normal boot that finds the plugin missing asks by itself. Whether
/// `variables` changed.
pub fn again(variables: &mut Variables) -> Result<bool, String> {
    match Record::read(variables) {
        Record::Attempted | Record::Retried | Record::Other => Change::Request.apply(variables),
        Record::None | Record::Pending | Record::Installed | Record::Retry => Ok(false),
    }
}

/// What the engine does with a report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    /// The plugin is in place: boot on, applying the change at the stop.
    Continue(Option<Change>),
    /// A single-user boot logs the report; its console may install it.
    Log,
    /// Record the request and restart into the repair.
    Repair,
    /// Stop ([`stop_reason`]); `attempted` when the automatic repair
    /// already ran and did not help, otherwise it cannot run.
    StopMissing { attempted: bool },
    /// The kext failed to write the plugin; its lines say why.
    StopFailed,
}

/// Decide what `report` means for `boot`, given NVRAM's `record` and
/// whether this boot can write NVRAM at its stop (`persistent`).
pub fn decide(report: Report, boot: Boot, record: Record, persistent: bool) -> Decision {
    match (report, boot) {
        (Report::Current | Report::Updated, _) => {
            Decision::Continue((record != Record::None).then_some(Change::Forget))
        }
        (Report::Missing | Report::Failed, Boot::SingleUser | Boot::Repair) => Decision::Log,
        (Report::Failed, Boot::Normal) => Decision::StopFailed,
        (Report::Missing, Boot::Normal) => match record {
            Record::None | Record::Other if persistent => Decision::Repair,
            // A request this boot did not take means NVRAM was not read
            // for it; another request would loop.
            Record::None | Record::Other | Record::Pending | Record::Installed | Record::Retry => {
                Decision::StopMissing { attempted: false }
            }
            Record::Attempted | Record::Retried => Decision::StopMissing { attempted: true },
        },
    }
}

/// What a user can do once an attempt did not help, with the `stela` of
/// this release: the record lives on the PC's own disk, which only a
/// system started on that PC reaches.
const AGAIN: &str = "start another system on this PC (Linux, Windows or a live USB), plug in \
                     the stick and run stela repair, which lets the repair run again at the next \
                     start; stela log saves the boot logs for a report";

/// The stop's reason for a normal boot that finds the plugin missing
/// ([`Decision::StopMissing`]): what the last repair boot reported
/// (`result`, [`result`]), or why no repair can run, given NVRAM's `record`
/// and whether it was read (`persistent`).
pub fn stop_reason(
    attempted: bool,
    record: Record,
    persistent: bool,
    result: Option<&str>,
) -> String {
    const MISSING: &str = "StelaGPU.bundle is not installed in /Library/GPUBundles and a kext \
                           cannot create its directories";
    if attempted {
        let reported = match result {
            Some(result) => format!("its boot reported: {result}"),
            None => "its boot left no result, so it ended before its stop (its BUNDLE REPAIR \
                     lines in the log say how far it got)"
                .into(),
        };
        return format!(
            "StelaGPU.bundle is still not installed in /Library/GPUBundles after the automatic \
             single-user repair; {reported}. To try again, {AGAIN}"
        );
    }
    if !persistent {
        return format!(
            "{MISSING}, and the machine state's NVRAM was not read at this start, so no \
             automatic repair can be recorded (its NVRAM lines in the log say why); stela log \
             saves the boot logs for a report"
        );
    }
    format!(
        "{MISSING}, and NVRAM holds a repair record ({}) this start did not take, as when NVRAM \
         is not read before the boot (its NVRAM lines in the log say why); stela log saves the \
         boot logs for a report",
        record.name()
    )
}

/// One console command and the seconds it may take.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Step {
    pub command: String,
    pub seconds: u64,
    /// A failure ends the repair; otherwise it is logged and the next step
    /// goes on.
    pub required: bool,
}

/// The repair's commands, in order, each answered by its exit status.
pub fn steps() -> Vec<Step> {
    let directories = DIRECTORIES
        .iter()
        .map(|directory| format!("{PLUGINS}/{directory}"))
        .collect::<Vec<_>>()
        .join(" ");
    let step = |command: String, seconds| Step {
        command,
        seconds,
        required: true,
    };
    let optional = |command: String, seconds| Step {
        command,
        seconds,
        required: false,
    };
    Vec::from([
        // The Data volume, as the single-user banner says for a machine
        // with a SEP.
        step("/sbin/mount -P 1".into(), 120),
        step("/usr/libexec/init_data_protection".into(), 180),
        step("/sbin/mount -P 2".into(), 180),
        step(format!("/bin/test -d {PLUGINS}"), 30),
        // Each named last, so `-m` gives each its mode.
        step(format!("/bin/mkdir -p -m 0755 {directories}"), 30),
        step(format!("/usr/sbin/chown root:wheel {directories}"), 30),
        optional(format!("/usr/bin/chflags restricted {directories}"), 30),
        step("/bin/sync".into(), 120),
        // Commits the volume and marks it clean, so the quick restart below
        // leaves nothing for the next mount to replay. Optional: the sync
        // already put every change on disk, and a busy volume refuses it.
        optional("/sbin/mount -u -r /System/Volumes/Data".into(), 60),
    ])
}

/// The command that ends the repair; the guest's restart restarts the PC.
///
/// `-q` is RB_QUICK: XNU's `reboot_kernel` then only syncs and goes to
/// `PEHaltRestart`, skipping the process kill and `vfs_unmountall`. The
/// power tree's shutdown still runs there, so the NVMe driver still sends
/// its shutdown notification and the controller flushes. The full path
/// stalled in single-user mode: run 20261111-150448 stopped in
/// `vfs_unmountall(system)` at `unmount(0) /System/Volumes/xarts`, before
/// APFS began that unmount, until the 300 s bound ran out.
pub const REBOOT: &str = "/sbin/reboot -q";
/// What ends a repair whose required step failed: what was written stays.
pub const SYNC_AND_REBOOT: &str = "/bin/sync; /sbin/reboot -q";
/// The line the single-user shell's banner starts with.
pub const BANNER: &str = "*** Single-user boot ***";
/// How long the single-user banner may take to appear, from the start.
pub const BANNER_SECONDS: u64 = 1800;
/// The end of the shell's prompt: launchd starts the shell after the
/// banner, and the shell prints its prompt, root's `-sh-3.2# ` (`PS1`'s
/// `\s-\v\$ `), once it reads its terminal, so typed input is neither
/// flushed nor taken by anyone else. It starts a line and ends it without
/// a line feed.
pub const PROMPT: &str = "# ";
/// How long the prompt may take to appear after the banner.
pub const PROMPT_SECONDS: u64 = 60;
/// What launchd logs when the single-user shell ends: `was signaled: ...`
/// or `exited ...`. Before the restart, it means launchd goes on to a
/// multi-user boot that no step typed after it reaches.
pub const SHELL_LEFT: &str = "Single-user shell ";
const SHELL_LEFT_HOW: [&str; 2] = ["signaled", "exited"];
/// How long the guest may take to restart after [`REBOOT`] in all.
pub const RESTART_SECONDS: u64 = 120;
/// How long the guest's console may stay silent after [`REBOOT`]: the
/// restart path logs each stage, and a stalled one prints nothing more.
pub const QUIET_SECONDS: u64 = 30;

/// Why the repair failed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Failure {
    /// No single-user banner in time.
    NoShell,
    /// The banner, but no shell prompt in time.
    NoPrompt,
    /// launchd reported the shell gone: before any step was typed
    /// (`None`), or while step `index` ran.
    ShellLeft { index: Option<usize> },
    /// Step `index` printed no exit status in time.
    Timeout { index: usize },
    /// Step `index` exited with `status`.
    Exit { index: usize, status: u32 },
    /// The guest did not restart after the repair.
    NoRestart,
}

impl Failure {
    /// Whether the shell never took input, so no step ran: what one more
    /// maintenance boot may get past ([`retry`]). A boot that never showed
    /// the banner is not: it would stall the same way.
    pub fn unanswered(&self) -> bool {
        matches!(self, Self::NoPrompt | Self::ShellLeft { index: None })
    }
}

impl core::fmt::Display for Failure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoShell => write!(f, "no single-user shell within {BANNER_SECONDS} s"),
            Self::NoPrompt => write!(
                f,
                "the single-user shell showed no prompt within {PROMPT_SECONDS} s; nothing was typed"
            ),
            Self::ShellLeft { index: None } => write!(
                f,
                "the single-user shell ended before it took input; nothing was typed"
            ),
            Self::ShellLeft { index: Some(index) } => {
                write!(f, "the single-user shell ended during step {}", index + 1)
            }
            Self::Timeout { index } => {
                write!(f, "step {} printed no exit status in time", index + 1)
            }
            Self::Exit { index, status } => {
                write!(f, "step {} exited with status {status}", index + 1)
            }
            Self::NoRestart => write!(
                f,
                "macOS did not restart (console silent for {QUIET_SECONDS} s, or over {RESTART_SECONDS} s in all)"
            ),
        }
    }
}

/// How a repair boot ended, for its stop to record ([`store_result`]) and a
/// later stop to name.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Every required step passed and the restart was typed.
    Done,
    /// The first failure: a required step's, or the shell's or restart's.
    Failed(Failure),
}

impl Outcome {
    /// One number for an atomic: the kind in the low byte, a step's index
    /// in the next and a status in the high half; never 0.
    pub fn encode(&self) -> u64 {
        let (kind, index, status) = match self {
            Self::Done => (1, 0, 0),
            Self::Failed(Failure::NoShell) => (2, 0, 0),
            Self::Failed(Failure::NoPrompt) => (3, 0, 0),
            Self::Failed(Failure::ShellLeft { index: None }) => (4, 0, 0),
            Self::Failed(Failure::ShellLeft { index: Some(index) }) => (5, *index, 0),
            Self::Failed(Failure::Timeout { index }) => (6, *index, 0),
            Self::Failed(Failure::Exit { index, status }) => (7, *index, *status),
            Self::Failed(Failure::NoRestart) => (8, 0, 0),
        };
        kind | ((index.min(0xff) as u64) << 8) | (u64::from(status) << 32)
    }

    /// The outcome [`Outcome::encode`] gave `code`.
    pub fn decode(code: u64) -> Option<Self> {
        let index = ((code >> 8) & 0xff) as usize;
        let status = (code >> 32) as u32;
        Some(match code & 0xff {
            1 => Self::Done,
            2 => Self::Failed(Failure::NoShell),
            3 => Self::Failed(Failure::NoPrompt),
            4 => Self::Failed(Failure::ShellLeft { index: None }),
            5 => Self::Failed(Failure::ShellLeft { index: Some(index) }),
            6 => Self::Failed(Failure::Timeout { index }),
            7 => Self::Failed(Failure::Exit { index, status }),
            8 => Self::Failed(Failure::NoRestart),
            _ => return None,
        })
    }

    /// The text NVRAM keeps: the failure with its step's command up to the
    /// plugin's paths, so it fits [`RESULT_BYTES`].
    pub fn text(&self, steps: &[Step]) -> String {
        let step = |index: usize| {
            let command = steps.get(index).map_or("?", |step| {
                step.command
                    .split(" /System/Volumes/Data/Library")
                    .next()
                    .unwrap_or(&step.command)
            });
            format!("step {} ({command})", index + 1)
        };
        match self {
            Self::Done => "every required step passed and the directories were created".into(),
            Self::Failed(Failure::ShellLeft { index: Some(index) }) => {
                format!("the single-user shell ended during {}", step(*index))
            }
            Self::Failed(Failure::Timeout { index }) => {
                format!("{} printed no exit status in time", step(*index))
            }
            Self::Failed(Failure::Exit { index, status }) => {
                format!("{} exited with status {status}", step(*index))
            }
            Self::Failed(failure) => failure.to_string(),
        }
    }
}

/// What the driver did, for the log.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    Shell,
    /// The shell's prompt showed: typing starts.
    Prompt,
    Sent {
        index: usize,
        command: String,
    },
    Passed {
        index: usize,
    },
    /// An optional step failed; the repair goes on.
    Skipped {
        index: usize,
        status: u32,
    },
    /// The restart was typed; `repaired` when every required step passed.
    Rebooting {
        repaired: bool,
    },
    Failed(Failure),
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum State {
    Banner {
        deadline: u64,
    },
    Prompt {
        deadline: u64,
    },
    Step {
        index: usize,
        deadline: u64,
    },
    /// `quiet` moves with each console line, never past `deadline`.
    Rebooting {
        quiet: u64,
        deadline: u64,
    },
    Failed(Failure),
}

/// The most of the console's current line [`Driver::output`] keeps.
const TAIL_BYTES: usize = 64;

/// Types [`steps`] at the guest's console, one at a time, each followed by
/// an `echo` of its exit status under a marker, and goes on only when that
/// line says 0. It types nothing before the shell's prompt shows, and ends
/// when launchd reports the shell gone. Times are milliseconds on any
/// monotonic clock.
pub struct Driver {
    steps: Vec<Step>,
    state: State,
    input: Vec<u8>,
    sent: usize,
    events: Vec<Event>,
    /// The console's current line, as [`Driver::output`] saw it.
    tail: Vec<u8>,
    /// How the repair went, once known: the first failure, or the end.
    outcome: Option<Outcome>,
}

const MILLIS: u64 = 1000;

impl Driver {
    pub fn new(steps: Vec<Step>, now: u64) -> Self {
        Self {
            steps,
            state: State::Banner {
                deadline: now + BANNER_SECONDS * MILLIS,
            },
            input: Vec::new(),
            sent: 0,
            events: Vec::new(),
            tail: Vec::with_capacity(TAIL_BYTES),
            outcome: None,
        }
    }

    fn marker(index: usize) -> String {
        format!("STELA-REPAIR-{}-rc=", index + 1)
    }

    fn send(&mut self, index: usize, now: u64) {
        let step = &self.steps[index];
        let line = format!("{}; echo {}$?\n", step.command, Self::marker(index));
        self.input.extend_from_slice(line.as_bytes());
        self.events.push(Event::Sent {
            index,
            command: step.command.clone(),
        });
        self.state = State::Step {
            index,
            deadline: now + step.seconds * MILLIS,
        };
    }

    fn fail(&mut self, failure: Failure) {
        self.outcome
            .get_or_insert_with(|| Outcome::Failed(failure.clone()));
        self.events.push(Event::Failed(failure.clone()));
        self.state = State::Failed(failure);
    }

    /// One byte the guest's console printed, for what shows before a line
    /// ends: the shell's prompt.
    pub fn output(&mut self, byte: u8, now: u64) {
        if matches!(byte, b'\n' | b'\r') {
            self.tail.clear();
            return;
        }
        if self.tail.len() == TAIL_BYTES {
            self.tail.remove(0);
        }
        self.tail.push(byte);
        if matches!(self.state, State::Prompt { .. }) && Self::prompt(&self.tail) {
            self.events.push(Event::Prompt);
            self.advance(0, now);
        }
    }

    /// Whether the console's current line is a prompt: one word ending in
    /// [`PROMPT`], as `-sh-3.2# `, not kernel text with `# ` in it.
    fn prompt(line: &[u8]) -> bool {
        line.strip_suffix(PROMPT.as_bytes())
            .is_some_and(|word| !word.iter().any(u8::is_ascii_whitespace))
    }

    /// A complete line the guest's console printed.
    pub fn line(&mut self, text: &str, now: u64) {
        let left = text
            .find(SHELL_LEFT)
            .is_some_and(|at| SHELL_LEFT_HOW.iter().any(|how| text[at..].contains(how)));
        match self.state {
            State::Banner { .. } if text.contains(BANNER) => {
                self.events.push(Event::Shell);
                self.state = State::Prompt {
                    deadline: now + PROMPT_SECONDS * MILLIS,
                };
            }
            State::Prompt { .. } if left => self.fail(Failure::ShellLeft { index: None }),
            State::Step { index, .. } if left => {
                self.fail(Failure::ShellLeft { index: Some(index) })
            }
            State::Step { index, .. } => {
                let marker = Self::marker(index);
                // The shell's echo of the command holds the marker before
                // `$?`; only the `echo`'s output has digits after it.
                let Some(at) = text.find(&marker) else {
                    return;
                };
                let digits: String = text[at + marker.len()..]
                    .chars()
                    .take_while(char::is_ascii_digit)
                    .collect();
                let Ok(status) = digits.parse::<u32>() else {
                    return;
                };
                if status == 0 {
                    self.events.push(Event::Passed { index });
                } else if self.steps[index].required {
                    // The shell answers: keep what was written and restart,
                    // so the next boot tries the kext.
                    let failure = Failure::Exit { index, status };
                    self.outcome
                        .get_or_insert_with(|| Outcome::Failed(failure.clone()));
                    self.events.push(Event::Failed(failure));
                    return self.reboot(SYNC_AND_REBOOT, false, now);
                } else {
                    self.events.push(Event::Skipped { index, status });
                }
                self.advance(index + 1, now);
            }
            State::Rebooting { deadline, .. } => {
                self.state = State::Rebooting {
                    quiet: deadline.min(now + QUIET_SECONDS * MILLIS),
                    deadline,
                };
            }
            _ => {}
        }
    }

    /// Send step `index`, or after the last, the restart.
    fn advance(&mut self, index: usize, now: u64) {
        if index < self.steps.len() {
            return self.send(index, now);
        }
        self.reboot(REBOOT, true, now);
    }

    fn reboot(&mut self, command: &str, repaired: bool, now: u64) {
        if repaired {
            self.outcome.get_or_insert(Outcome::Done);
        }
        self.input.extend_from_slice(command.as_bytes());
        self.input.push(b'\n');
        self.events.push(Event::Rebooting { repaired });
        self.state = State::Rebooting {
            quiet: now + QUIET_SECONDS * MILLIS,
            deadline: now + RESTART_SECONDS * MILLIS,
        };
    }

    /// Advance the clock: every deadline.
    pub fn tick(&mut self, now: u64) {
        match self.state {
            State::Banner { deadline } if now >= deadline => self.fail(Failure::NoShell),
            State::Prompt { deadline } if now >= deadline => self.fail(Failure::NoPrompt),
            State::Step { index, deadline } if now >= deadline => {
                self.fail(Failure::Timeout { index })
            }
            State::Rebooting { quiet, .. } if now >= quiet => self.fail(Failure::NoRestart),
            _ => {}
        }
    }

    /// The next byte to type, if any. A failed repair types nothing more.
    pub fn next_byte(&mut self) -> Option<u8> {
        if self.failed() {
            return None;
        }
        let byte = *self.input.get(self.sent)?;
        self.sent += 1;
        if self.sent == self.input.len() {
            self.input.clear();
            self.sent = 0;
        }
        Some(byte)
    }

    /// What happened since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        core::mem::take(&mut self.events)
    }

    /// Whether the repair stopped without a guest restart on its way: the
    /// shell did not answer or left, or macOS did not restart. The engine
    /// then restarts the PC itself.
    pub fn failed(&self) -> bool {
        matches!(self.state, State::Failed(_))
    }

    /// How the repair went, once known: the first failure, which may come
    /// before a restart the guest makes itself, or the end of the steps.
    pub fn outcome(&self) -> Option<&Outcome> {
        self.outcome.as_ref()
    }

    /// Why the repair stopped, once [`Driver::failed`].
    pub fn failure(&self) -> Option<&Failure> {
        match &self.state {
            State::Failed(failure) => Some(failure),
            _ => None,
        }
    }
}

/// The log line for `event`.
pub fn describe(event: &Event) -> String {
    match event {
        Event::Shell => "BUNDLE REPAIR single-user shell up".to_string(),
        Event::Prompt => "BUNDLE REPAIR shell prompt shown; typing the steps".to_string(),
        Event::Sent { index, command } => {
            format!("BUNDLE REPAIR step {} sent: {command}", index + 1)
        }
        Event::Passed { index } => format!("BUNDLE REPAIR step {} ok", index + 1),
        Event::Skipped { index, status } => format!(
            "BUNDLE REPAIR step {} exited with status {status}; optional, going on",
            index + 1
        ),
        Event::Rebooting { repaired: true } => {
            "BUNDLE REPAIR directories created; restarting".to_string()
        }
        Event::Rebooting { repaired: false } => {
            "BUNDLE REPAIR syncing and restarting; the next boot tries the kext once".to_string()
        }
        Event::Failed(failure) => format!("BUNDLE REPAIR failed: {failure}"),
    }
}

#[cfg(test)]
mod tests;
