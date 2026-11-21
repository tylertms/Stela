//! The startup menu (`stela_loader::mount_interface`): when the boot volume
//! has `choices.cfg`, `stela.efi` lists on the framebuffer, in Stela's look
//! ([`ui`]), the entries that can start now, then the PC firmware's own
//! boot entries (`firmware::entries`), with one highlighted. A macOS entry
//! is left out until a macOS is installed or while Install macOS has not
//! prepared its SSD or the disk it chose is gone (`chosen_disk::Stale`),
//! and Install macOS once it has (`selection::ready`). Up and Down move the
//! highlight, Enter starts the highlighted entry, and an entry's key (`1`-`9`,
//! then `a`-`z`) starts it at once; a firmware entry restarts the PC into it.
//! When the first entry, the one `stela.cfg` holds, is the only Mac that
//! can start now and the PC has no other system on its disks, it starts at
//! once unless a key is held or pressed within [`GLANCE`], as a Mac shows
//! its startup options only when asked (`mount_interface::opening`). When
//! there is a choice to make, such as Windows or Linux beside the Mac, a
//! countdown from the file's timeout starts the first entry; any key stops
//! it. A timeout of 0 opens the menu only when a key is already pressed.
//! A file that cannot be read or parsed is reported and
//! `stela.cfg` boots, so a damaged file never stops the default boot. A
//! machine that can only be set up waits at the menu with no countdown and
//! its setup entry highlighted ([`hold`]).
use crate::{
    console::{self, Framebuffer, font},
    firmware::entries::{self, Entry},
    output,
    selection::{self, Readiness},
    status,
    ui::{self, Canvas},
};
use alloc::{format, string::String, vec::Vec};
use core::time::Duration;
use stela_boot_core::chosen_disk::{Adopted, adopt};
use stela_contracts::refuse;
use stela_loader::mount_interface::{
    CHOICES_FILE, CHOSEN_DISK, ChoiceKind, Choices, DISK_FILE, Opening, opening, with_disk,
};
use stela_runtime::pc::{disk_plan::Plan, disk_writes::Pending};
use uefi::{
    CString16,
    boot::{self, EventType, TimerTrigger, Tpl},
    fs::{FileSystem, PathBuf},
    proto::console::text::{Key, ScanCode},
};

const TICK: Duration = Duration::from_secs(1);
/// How long a key may still come to open the menu when the first entry
/// starts without one: longer than a held key's repeat and a USB keyboard's
/// polling, short enough not to be a wait.
const GLANCE: Duration = Duration::from_millis(250);
/// On the USB, asks Install macOS to take the PC's one blank disk unasked.
const AUTO_FILE: &str = "\\EFI\\stela\\auto-install";

pub(crate) enum Press {
    Up,
    Down,
    Left,
    Right,
    Home,
    End,
    Delete,
    Escape,
    Enter,
    Character(char),
    Other,
}

/// What a wait for a key ended with.
pub(crate) enum Wait {
    Key(Press),
    /// The key event fired without a key.
    Nothing,
    Timeout,
    /// The console cannot be waited on.
    NoInput,
}

/// The next key, waiting up to `timeout` (forever with `None`).
pub(crate) fn key(timeout: Option<Duration>) -> Wait {
    let Ok(keys) = uefi::system::with_stdin(|input| input.wait_for_key_event()) else {
        return Wait::NoInput;
    };
    let timer = match timeout {
        Some(duration) => {
            // SAFETY: a plain timer event without a notification function.
            let Ok(timer) =
                (unsafe { boot::create_event(EventType::TIMER, Tpl::APPLICATION, None, None) })
            else {
                return Wait::NoInput;
            };
            if boot::set_timer(&timer, TimerTrigger::Relative(duration)).is_err() {
                let _ = boot::close_event(timer);
                return Wait::NoInput;
            }
            Some(timer)
        }
        None => None,
    };
    let mut events = Vec::from([keys]);
    // SAFETY: the copy is only waited on and closed once, below.
    events.extend(timer.as_ref().map(|t| unsafe { t.unsafe_clone() }));
    let signaled = boot::wait_for_event(&events).ok();
    if let Some(timer) = timer {
        let _ = boot::close_event(timer);
    }
    match signaled {
        Some(0) => pressed().map_or(Wait::Nothing, Wait::Key),
        Some(_) => Wait::Timeout,
        None => Wait::NoInput,
    }
}

/// A key already waiting, without waiting.
fn pressed() -> Option<Press> {
    Some(
        match uefi::system::with_stdin(|input| input.read_key()).ok()?? {
            Key::Printable(character) => match char::from(character) {
                '\r' | '\n' => Press::Enter,
                character => Press::Character(character),
            },
            Key::Special(ScanCode::UP) => Press::Up,
            Key::Special(ScanCode::DOWN) => Press::Down,
            Key::Special(ScanCode::LEFT) => Press::Left,
            Key::Special(ScanCode::RIGHT) => Press::Right,
            Key::Special(ScanCode::HOME) => Press::Home,
            Key::Special(ScanCode::END) => Press::End,
            Key::Special(ScanCode::DELETE) => Press::Delete,
            Key::Special(ScanCode::ESCAPE) => Press::Escape,
            Key::Special(_) => Press::Other,
        },
    )
}

/// The volume's startup choices; `None` when it has none or they cannot be
/// read, which is reported.
fn read(volume: &mut FileSystem) -> Option<Choices> {
    let path = CString16::try_from(CHOICES_FILE).map(PathBuf::from).ok()?;
    if !matches!(volume.try_exists(&path), Ok(true)) {
        return None;
    }
    let choices = volume
        .read(&path)
        .map_err(|_| refuse!("cannot read choices.cfg"))
        .and_then(|bytes| String::from_utf8(bytes).map_err(|_| refuse!("choices.cfg is not UTF-8")))
        .and_then(|text| Ok(Choices::parse(&text)?));
    match choices {
        Ok(choices) => Some(choices),
        Err(error) => {
            output::arguments(format_args!("CHOICES refused: {error}\n"));
            None
        }
    }
}

/// The partition Install macOS chose (`SERIAL:GUID`), when it chose one.
pub fn chosen_disk(volume: &mut FileSystem) -> Option<String> {
    let path = CString16::try_from(DISK_FILE).map(PathBuf::from).ok()?;
    let text = String::from_utf8(volume.read(&path).ok()?).ok()?;
    let disk = text.trim();
    match disk.parse::<stela_runtime::pc::storage::Target>() {
        Ok(_) => Some(disk.into()),
        Err(_) => {
            output::arguments(format_args!("CHOSEN DISK refused: {disk:?}\n"));
            None
        }
    }
}

/// The partition the macOS choices start from, and whether it was found on
/// this PC's disks and not read from the record: the recorded one while
/// macOS starts from it, else the one of Stela's macOS partitions on the
/// PC's disks that macOS starts from (`chosen_disk::adopt`). A USB made new
/// on a PC that already holds macOS then offers that macOS, where it
/// offered only Install macOS, which erases it.
fn startup_disk(
    volume: &mut FileSystem,
    choices: &Choices,
    pending: &Pending,
) -> (Option<String>, bool) {
    let recorded = chosen_disk(volume);
    let mut pending = pending.clone();
    pending.replace_plan(None);
    let starts = |disk: Option<&str>| {
        choices.entries.iter().any(|entry| {
            entry.kind() == ChoiceKind::Mac
                && with_disk(&entry.options, disk).is_some_and(|options| {
                    selection::ready(&options, ChoiceKind::Mac, &pending) == Readiness::Ready
                })
        })
    };
    // Only choices that take the chosen disk can take another.
    let takes = choices
        .entries
        .iter()
        .any(|entry| entry.kind() == ChoiceKind::Mac && unchosen(entry.options.as_bytes()));
    // A record is kept whatever it starts: a disk taken out for now or an
    // install not finished yet is still the one this USB was set up for, and
    // Install macOS's picker is where another is chosen.
    if !takes || recorded.is_some() {
        return (recorded, false);
    }
    let found = crate::installer::installed();
    let ready: Vec<bool> = found.iter().map(|disk| starts(Some(disk))).collect();
    match adopt(false, &ready) {
        Adopted::One(index) => {
            output::arguments(format_args!(
                "CHOSEN DISK found: the USB holds no record and one partition on this PC \
                 starts macOS; the macOS choice starts from it\n"
            ));
            (found.get(index).cloned(), true)
        }
        Adopted::Several(count) => {
            output::arguments(format_args!(
                "CHOSEN DISK not found: {count} partitions on this PC start macOS and the \
                 USB holds no record of one\n"
            ));
            (recorded, false)
        }
        Adopted::None | Adopted::Recorded => (recorded, false),
    }
}

/// The disk a choice of `kind` takes ([`chosen_disk::disk_for`]).
fn disk_for(kind: ChoiceKind, disk: Option<&str>, found: bool) -> Option<&str> {
    stela_boot_core::chosen_disk::disk_for(kind == ChoiceKind::Mac, disk, found)
}

/// Record `disk` as the partition macOS starts from, once a macOS choice
/// found on the PC's disks is taken.
fn record(volume: &mut FileSystem, disk: &str) {
    let written = CString16::try_from(DISK_FILE)
        .map(PathBuf::from)
        .ok()
        .is_some_and(|path| volume.write(&path, format!("{disk}\n")).is_ok());
    match written {
        true => output::arguments(format_args!("CHOSEN DISK {disk}\n")),
        false => output::arguments(format_args!(
            "CHOSEN DISK not recorded: the USB could not be written\n"
        )),
    }
}

/// `options` with the chosen disk in place of [`CHOSEN_DISK`], or as they
/// are while none is chosen, which the default boot then holds for.
pub fn resolve(volume: &mut FileSystem, options: Vec<u8>, pending: &Pending) -> Vec<u8> {
    let Ok(text) = core::str::from_utf8(&options) else {
        return options;
    };
    // The disk the startup choices take ([`startup_disk`]), so that the
    // default boot is checked on the macOS this PC holds.
    let disk = match read(volume) {
        Some(choices) => startup_disk(volume, &choices, pending).0,
        None => chosen_disk(volume),
    };
    match with_disk(text, disk.as_deref()) {
        Some(resolved) => resolved.into_bytes(),
        None => options,
    }
}

/// Whether `options` still name a disk Install macOS has not chosen.
pub fn unchosen(options: &[u8]) -> bool {
    core::str::from_utf8(options)
        .is_ok_and(|text| text.split_ascii_whitespace().any(|w| w == CHOSEN_DISK))
}

/// The heading over the firmware's entries.
const PC: &str = "This PC";

/// The SSD (`--storage`) a startup choice's options name.
fn storage(options: &str) -> Option<&str> {
    let mut words = options.split_ascii_whitespace();
    words.find(|word| *word == "--storage")?;
    words.next()
}

/// One entry of the menu.
enum Item {
    /// Entry `n` of the startup choices.
    Choice(usize, ChoiceKind),
    /// One of the PC firmware's own boot entries.
    Firmware(Entry),
}

/// The menu.
struct Menu<'a> {
    choices: &'a Choices,
    /// The partition Install macOS chose, when it chose one, or the one
    /// found on this PC's disks ([`startup_disk`]).
    disk: Option<String>,
    /// The partition was found, and the record does not name it yet.
    found: bool,
    items: Vec<Item>,
    selected: usize,
    /// Seconds before the first entry starts, while the countdown runs.
    countdown: Option<u32>,
    /// The first entry starts unless a key asks for the menu.
    glance: bool,
    /// Why the last firmware entry chosen did not start.
    notice: Option<&'static str>,
    /// Why the machine can only be set up, shown while nothing else is.
    held: Option<&'a str>,
}

impl<'a> Menu<'a> {
    /// The menu of `choices`: those that can start now, then the firmware's
    /// entries. An entry is checked on its disk as choosing it leaves the
    /// `pending` work: the updates done and no plan, which a choice
    /// replaces. `held` is why the machine can only be set up.
    fn new(
        choices: &'a Choices,
        held: Option<&'a str>,
        (disk, found): (Option<String>, bool),
        pending: &Pending,
    ) -> Self {
        let mut pending = pending.clone();
        pending.replace_plan(None);
        let pending = &pending;
        // A choice on a disk not chosen yet waits for Install macOS.
        let resolved: Vec<Option<String>> = choices
            .entries
            .iter()
            .map(|entry| {
                with_disk(
                    &entry.options,
                    disk_for(entry.kind(), disk.as_deref(), found),
                )
            })
            .collect();
        let readiness: Vec<Readiness> = choices
            .entries
            .iter()
            .zip(&resolved)
            .map(|(entry, options)| match options {
                Some(options) => selection::ready(options, entry.kind(), pending),
                None => Readiness::Unprepared,
            })
            .collect();
        // The SSDs a macOS choice is known to start from,
        // which Install macOS has prepared already.
        let prepared: Vec<Option<&str>> = choices
            .entries
            .iter()
            .zip(&readiness)
            .zip(&resolved)
            .filter(|((_, r), _)| **r == Readiness::Ready)
            .map(|(_, options)| options.as_deref().and_then(storage))
            .collect();
        let mut items = Vec::new();
        for (n, entry) in choices.entries.iter().enumerate() {
            let kind = entry.kind();
            let left_out = match (kind, readiness[n]) {
                (ChoiceKind::Mac, Readiness::NotInstalled) => Some("no macOS is installed"),
                (ChoiceKind::Mac, Readiness::Unprepared) => Some("its SSD is not prepared"),
                (ChoiceKind::Mac, Readiness::Stale(stale)) => Some(stale.reason()),
                // Install macOS that chooses its disk is always offered.
                (ChoiceKind::Setup, _)
                    if !unchosen(entry.options.as_bytes())
                        && prepared.contains(&storage(&entry.options)) =>
                {
                    Some("its SSD is prepared")
                }
                _ => None,
            };
            match left_out {
                Some(reason) => output::arguments(format_args!(
                    "CHOICE {} ({}) left out: {reason}\n",
                    n + 1,
                    entry.name
                )),
                None => items.push(Item::Choice(n, kind)),
            }
        }
        // A setup without its restore choice still lists its own.
        if items.is_empty() {
            items.extend(
                choices
                    .entries
                    .iter()
                    .enumerate()
                    .map(|(n, entry)| Item::Choice(n, entry.kind())),
            );
        }
        items.extend(entries::list().into_iter().map(Item::Firmware));
        let selected = match held {
            Some(_) => items
                .iter()
                .position(|item| matches!(item, Item::Choice(_, ChoiceKind::Setup)))
                .unwrap_or(0),
            None => 0,
        };
        // The first entry starts by itself only when it starts the Mac,
        // with no countdown when that Mac is the only one ready.
        let first_mac = match items.first() {
            Some(Item::Choice(n, ChoiceKind::Mac)) => Some(readiness[*n] == Readiness::Ready),
            _ => None,
        };
        let ready_macs = items
            .iter()
            .filter(|item| {
                matches!(item, Item::Choice(n, ChoiceKind::Mac) if readiness[*n] == Readiness::Ready)
            })
            .count();
        // The PC's other systems, such as Windows or Linux, which the
        // menu offers beside the Mac.
        let other_systems = items
            .iter()
            .filter(|item| matches!(item, Item::Firmware(entry) if entry.on_disk))
            .count();
        let opens = match held {
            Some(_) => Opening::Wait,
            None => opening(
                choices.timeout_seconds,
                first_mac,
                ready_macs,
                other_systems,
            ),
        };
        Self {
            choices,
            disk,
            found,
            countdown: match opens {
                Opening::Countdown(seconds) => Some(seconds),
                Opening::Glance | Opening::Wait => None,
            },
            glance: opens == Opening::Glance,
            items,
            selected,
            notice: None,
            held,
        }
    }

    fn name(&self, item: &'a Item) -> &'a str {
        match item {
            Item::Choice(n, _) => &self.choices.entries[*n].name,
            Item::Firmware(entry) => &entry.name,
        }
    }

    /// The entries under the header, the highlighted one raised with an
    /// orange mark, then the countdown or a notice.
    fn draw(&self, canvas: &mut Canvas) {
        const ROW: usize = 36;
        let (width, height) = canvas.size();
        canvas.fill(0, 0, width, height, ui::BACKGROUND);
        let top = canvas.header() + 24;
        let column = width.saturating_sub(32).min(440);
        let left = (width - column) / 2;
        let fits = (height.saturating_sub(top + 120) / ROW).max(1);
        let first = stela_boot_core::screen::window(self.selected, self.items.len(), fits);
        let mut y = top;
        let mut firmware = false;
        for index in (first..self.items.len()).take(fits) {
            let item = &self.items[index];
            if !firmware && matches!(item, Item::Firmware(_)) {
                firmware = true;
                y += 16;
                canvas.text(left + 16, y, PC, 1, ui::DIM);
                y += 28;
            }
            if index == self.selected {
                canvas.panel(left, y, column, ROW - 4, 8, ui::RAISED);
                canvas.panel(left + 6, y + 9, 3, ROW - 22, 1, ui::ORANGE);
            }
            canvas.text(left + 20, y + 8, self.name(item), 1, ui::TEXT);
            y += ROW;
        }
        let (status, colour) = match (self.notice, self.countdown, self.held) {
            (Some(notice), _, _) => (String::from(notice), ui::ORANGE),
            (None, Some(seconds), _) => (
                format!("{} in {seconds}", self.name(&self.items[0])),
                ui::DIM,
            ),
            (None, None, Some(held)) => (String::from(held), ui::DIM),
            (None, None, None) => return,
        };
        // A status of two lines raises its first line by one row.
        let lines = status.lines().count();
        for (n, line) in status.lines().enumerate() {
            let y = height.saturating_sub(56 + (lines - 1 - n) * 24);
            canvas.text_centred(width / 2, y, line, 1, colour);
        }
    }

    /// The chosen item; `None` without console input.
    fn run(&mut self) -> Option<usize> {
        console::release();
        let (screen, font) = Framebuffer::from_firmware().zip(font::console())?;
        let mut canvas = Canvas::new(screen, font);
        loop {
            self.draw(&mut canvas);
            canvas.show();
            let wait = key(self.countdown.map(|_| TICK));
            if let Wait::Key(_) = wait {
                self.countdown = None;
                self.notice = None;
            }
            let items = self.items.len();
            match wait {
                Wait::Key(Press::Up) => self.selected = (self.selected + items - 1) % items,
                Wait::Key(Press::Down) => self.selected = (self.selected + 1) % items,
                Wait::Key(Press::Enter) => return Some(self.selected),
                Wait::Key(Press::Character(key)) => {
                    let chosen = (0..items).find(|&n| Choices::key(n) == Some(key));
                    if chosen.is_some() {
                        return chosen;
                    }
                }
                Wait::Key(_) | Wait::Nothing => {}
                Wait::Timeout => match self.countdown {
                    Some(1) => return Some(0),
                    Some(seconds) => self.countdown = Some(seconds - 1),
                    None => {}
                },
                Wait::NoInput => return None,
            }
        }
    }
}

/// What boots: the options, and the change Install macOS's disk picker
/// planned for the disk they name, which Stela's drivers make after the
/// firmware has exited (`pc::disk_plan`). The plan belongs to these options
/// alone: another choice made later replaces both, and nothing of a
/// dropped plan was written.
pub struct Chosen {
    pub options: Vec<u8>,
    pub plan: Option<Plan>,
}

impl From<Vec<u8>> for Chosen {
    /// Options that change no disk before they boot.
    fn from(options: Vec<u8>) -> Self {
        Self {
            options,
            plan: None,
        }
    }
}

/// Log the chosen entry; its options, on `disk`.
fn chosen(choices: &Choices, index: usize, disk: Option<&str>) -> Vec<u8> {
    let entry = &choices.entries[index];
    output::arguments(format_args!("CHOICE {} ({})\n", index + 1, entry.name));
    status::info(&format!("Starting {}", entry.name));
    with_disk(&entry.options, disk)
        .unwrap_or_else(|| entry.options.clone())
        .into_bytes()
}

/// Whether a key asks for the menu: one already pressed or held, or one
/// pressed within [`GLANCE`].
fn asked() -> bool {
    pressed().is_some() || matches!(key(Some(GLANCE)), Wait::Key(_))
}

/// `menu` until one of `choices` is chosen; its options. Install macOS that
/// chooses its disk always opens the disk picker first
/// ([`installer::choose`]), where the partition chosen before can be chosen
/// again, and returns to the menu when the picker is left. The picker gives
/// a plan and writes nothing to the disk; the plan goes with the options. A
/// firmware entry chosen restarts the PC into it. `None` without console
/// input to choose with.
fn menu(volume: &mut FileSystem, choices: &Choices, mut menu: Menu<'_>) -> Option<Chosen> {
    // An unattended install starts the setup entry at once, once, and only
    // while the machine can only be set up: a later menu never reinstalls.
    let mut auto = menu.held.is_some()
        && CString16::try_from(AUTO_FILE)
            .map(PathBuf::from)
            .is_ok_and(|path| volume.try_exists(&path).unwrap_or(false));
    loop {
        let setup = menu.items.iter().position(|item| {
            matches!(item, Item::Choice(n, ChoiceKind::Setup)
                if unchosen(choices.entries[*n].options.as_bytes()))
        });
        let index = match setup.filter(|_| auto) {
            Some(index) => index,
            None => menu.run()?,
        };
        let n = match &menu.items[index] {
            Item::Choice(n, ChoiceKind::Setup)
                if unchosen(choices.entries[*n].options.as_bytes()) =>
            {
                *n
            }
            Item::Choice(n, kind) => {
                if let (true, ChoiceKind::Mac, Some(disk)) = (menu.found, kind, &menu.disk) {
                    record(volume, disk);
                }
                let disk = disk_for(*kind, menu.disk.as_deref(), menu.found);
                return Some(chosen(choices, *n, disk).into());
            }
            Item::Firmware(entry) => {
                output::arguments(format_args!("CHOICE firmware entry ({})\n", entry.name));
                menu.notice = Some(entries::start(entry.number));
                continue;
            }
        };
        let Some(plan) = crate::installer::choose(core::mem::take(&mut auto)) else {
            continue;
        };
        let disk = format!("{}:{}", plan.serial, plan.partition.guid);
        // The record goes to the USB before the disk is changed: a boot
        // that stops in between finds no such partition and chooses again
        // (`chosen_disk::Stale`).
        let path = CString16::try_from(DISK_FILE).map(PathBuf::from).ok()?;
        if volume.write(&path, format!("{disk}\n")).is_err() {
            menu.notice = Some("The chosen disk could not be recorded on the USB.");
            continue;
        }
        output::arguments(format_args!("CHOSEN DISK {disk}\n"));
        return Some(Chosen {
            options: chosen(choices, n, Some(&disk)),
            plan: Some(plan),
        });
    }
}

/// What to boot: the chosen entry's options, or `options` (`stela.cfg`).
/// `held` is why the default boot can only wait for the setup restore, when
/// it can. `pending` is the disk work decided so far, which the entries'
/// disks are read with (`selection::ready`).
pub fn choose(
    volume: &mut FileSystem,
    options: Vec<u8>,
    held: Option<&str>,
    pending: &Pending,
) -> Chosen {
    if let Some(reason) = held {
        return hold(volume, reason, pending).unwrap_or(options.into());
    }
    let Some(choices) = read(volume) else {
        return options.into();
    };
    let (disk, found) = startup_disk(volume, &choices, pending);
    // The first choice as it starts with no menu, recording a found disk
    // when it is a macOS choice.
    let first = |volume: &mut FileSystem, n: usize| {
        let kind = choices.entries.get(n).map(|entry| entry.kind());
        if let (true, Some(ChoiceKind::Mac), Some(disk)) = (found, kind, &disk) {
            record(volume, disk);
        }
        let disk = match kind {
            Some(kind) => disk_for(kind, disk.as_deref(), found),
            None => disk.as_deref(),
        };
        chosen(&choices, n, disk)
    };
    // Only a key pressed before the menu opens asks for it.
    if choices.timeout_seconds == 0 && pressed().is_none() {
        return first(volume, 0).into();
    }
    let opened = Menu::new(&choices, None, (disk.clone(), found), pending);
    if opened.glance
        && !asked()
        && let Some(Item::Choice(n, _)) = opened.items.first()
    {
        return first(volume, *n).into();
    }
    menu(volume, &choices, opened)
        .unwrap_or_else(|| chosen(&choices, 0, chosen_disk(volume).as_deref()).into())
}

/// Wait at the menu after `reason`, with no countdown, as LLB waits in
/// recovery mode for a restore host: the machine can only be set up. The
/// setup entry starts highlighted; any entry may be chosen, and one that
/// cannot boot either returns here. Returns what the chosen entry boots;
/// `None` when the volume has no choices or no console input to choose with.
pub fn hold(volume: &mut FileSystem, reason: &str, pending: &Pending) -> Option<Chosen> {
    let choices = read(volume)?;
    output::arguments(format_args!("HOLD {reason}\n"));
    let disk = startup_disk(volume, &choices, pending);
    let held = Menu::new(&choices, Some(reason), disk, pending);
    menu(volume, &choices, held)
}
