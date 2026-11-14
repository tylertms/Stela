//! Plain `stela`: look at the plugged-in sticks and offer the one right next
//! step, with a short list of other actions. Each action is the explicit
//! command a script would run (`setup::command`), with the same consent
//! rules: every question can be declined, and declining changes nothing.
//!
//! A stick is classified with the quick checks `status` makes (no piece is
//! read whole): no Stela, an interrupted first create, an interrupted macOS
//! switch, a newer or older Stela than this one, problems, or healthy.
//! Without a terminal nothing is asked: the report names the command for
//! each stick, and the run fails when one is due, unless `--yes` accepts
//! it: then each due command runs as it would with `--yes`. On a terminal,
//! `--yes` accepts the questions an action asks, as a command's does.
use super::{
    Action, Environment, Record, Reported, Request, builds, doctor, host, read_record, release,
    sources, ui::Ui, wizard,
};
use anyhow::{Context, Result};
use std::{
    fmt::Write as _,
    path::{Path, PathBuf},
};

/// The smallest stick an installer is made on: one sold as 8 GB.
pub(super) const NETWORK_MIN: u64 = 7 * 1024 * 1024 * 1024;

/// What a stick holds, as plain `stela` sees it.
pub(super) enum State {
    /// No Stela setup.
    Empty,
    /// A first create that stopped part way.
    Interrupted,
    /// A macOS switch that stopped after the old copy was removed: the
    /// build it switches to.
    Replacing(String),
    /// Made or last updated by a newer Stela: its version.
    Newer(String),
    /// Made or last updated by an older Stela.
    Older,
    /// What `status` finds wrong.
    Damaged(Vec<String>),
    Healthy,
}

/// A plugged-in stick.
pub(super) struct Stick {
    pub(super) disk: host::Disk,
    /// The mounted volume that holds its Stela setup.
    pub(super) volume: Option<PathBuf>,
    pub(super) record: Option<Record>,
    pub(super) state: State,
}

impl Stick {
    /// Its whole disk as a user types it.
    pub(super) fn path(&self) -> String {
        format!("{}{}", host::DISK_PREFIX, self.disk.identifier)
    }

    /// One line naming it: `STELA · 64.0 GB · /dev/sdb`.
    pub(super) fn name(&self) -> String {
        format!(
            "{} · {} · {}",
            self.disk.name,
            host::size_text(self.disk.bytes),
            self.path()
        )
    }

    /// What it holds, in a few words.
    fn summary(&self) -> String {
        let stela = || match self.record.as_ref().and_then(|r| r.release.clone()) {
            Some(version) => format!("Stela {version}"),
            None => "Stela".into(),
        };
        match &self.state {
            State::Empty if self.disk.bytes != 0 && self.disk.bytes < NETWORK_MIN => {
                "too small: an installer needs 8 GB or more".into()
            }
            State::Empty if self.disk.partitions.is_empty() => "empty".into(),
            State::Empty => "no Stela on it".into(),
            State::Interrupted => "a create stopped part way".into(),
            State::Replacing(_) => format!("{}, a macOS switch stopped part way", stela()),
            State::Newer(_) => format!("{}, newer than this stela", stela()),
            State::Older => format!("{}, update available", stela()),
            State::Damaged(_) => format!("{}, needs repair", stela()),
            State::Healthy => stela(),
        }
    }

    /// The stick with its state: `STELA · 64.0 GB · /dev/sdb · Stela 0.3.0,
    /// update available`.
    pub(super) fn line(&self) -> String {
        format!("{} · {}", self.name(), self.summary())
    }

    /// What its record says it installs: `Stela 0.3.0 · Offline · macOS
    /// 27.0.1 (26A434)`.
    fn details(&self) -> Option<String> {
        let record = self.record.as_ref()?;
        Some(format!(
            "{} · {} · macOS {}",
            match &record.release {
                Some(version) => format!("Stela {version}"),
                None => "Stela 0.3 or older".into(),
            },
            record.install_kind().name(),
            macos_name(&record.build)
        ))
    }

    /// The command that is due for it, if any; with the stick named when
    /// `several` fit.
    pub(super) fn next(&self, several: bool) -> Option<String> {
        match &self.state {
            State::Empty if self.disk.bytes != 0 && self.disk.bytes < NETWORK_MIN => None,
            // Without a terminal, erasing needs the stick typed back.
            State::Empty => Some(format!(
                "stela create {} --confirm-disk {}",
                self.path(),
                match self.disk.bytes {
                    0 => super::quoted(&self.disk.name),
                    bytes => host::size_text(bytes).replace(' ', ""),
                }
            )),
            _ => self.due().map(|action| match several {
                true => format!("stela {} {}", super::command_words(&action), self.path()),
                false => format!("stela {}", super::command_words(&action)),
            }),
        }
    }

    /// What a Stela stick needs, if anything: `repair` finishes or puts
    /// back, `update` brings an older one to this Stela.
    fn due(&self) -> Option<Action> {
        match &self.state {
            State::Interrupted | State::Replacing(_) | State::Damaged(_) => Some(Action::Repair),
            State::Older => Some(Action::UpdateStela),
            State::Empty | State::Newer(_) | State::Healthy => None,
        }
    }
}

/// `action` on `stick`, with the options plain `stela` was given.
fn maintenance(request: &Request, stick: &Stick, action: Action) -> Request {
    Request {
        action,
        usb: stick.volume.clone().unwrap_or_default(),
        dry_run: request.dry_run,
        yes: request.yes,
        efi: request.efi.clone(),
        ..Request::default()
    }
}

/// `27.2 (26B123)`, or the build alone when Stela does not know it.
pub(super) fn macos_name(build: &str) -> String {
    match builds::SUPPORTED.iter().find(|entry| entry.build == build) {
        Some(entry) => format!("{} ({build})", entry.version),
        None => build.into(),
    }
}

/// Every removable disk of this computer, classified; Stela sticks first.
pub(super) fn sticks(environment: &Environment<'_>) -> Result<Vec<Stick>> {
    let mut sticks: Vec<Stick> = environment
        .host
        .disks()?
        .into_iter()
        .filter(host::Disk::erasable)
        .map(|disk| classify(environment, disk))
        .collect();
    sticks.sort_by_key(|stick| stick.record.is_none());
    Ok(sticks)
}

/// What `disk` holds.
fn classify(environment: &Environment<'_>, disk: host::Disk) -> Stick {
    let volume = super::setup_volume(&disk).or_else(|| {
        // Hosts often leave a stick's EFI partition unmounted: it holds the
        // record.
        let esp = disk.partitions.iter().find(|p| p.content == "EFI")?;
        let mount = environment.host.mount(&esp.identifier).ok()?;
        mount.join(super::RECORD).is_file().then_some(mount)
    });
    let record = volume
        .as_ref()
        .and_then(|volume| read_record(environment, volume).ok());
    let state = match &record {
        None => State::Empty,
        Some(record) if !record.complete => State::Interrupted,
        Some(record) => match (&record.replacing, release::newer(record)) {
            (Some(next), _) => State::Replacing(next.clone()),
            (None, Some(version)) => State::Newer(version),
            (None, None) if release::older(record) => State::Older,
            (None, None) => {
                let usb = volume.clone().unwrap_or_default();
                match doctor::status(environment, &usb, false) {
                    Ok(_) => State::Healthy,
                    Err(error) => match error.downcast::<Reported>() {
                        Ok(reported) => State::Damaged(problems(&reported.report)),
                        Err(error) => State::Damaged(vec![format!("{error:#}")]),
                    },
                }
            }
        },
    };
    Stick {
        disk,
        volume,
        record,
        state,
    }
}

/// The problems a `status` report lists.
fn problems(report: &str) -> Vec<String> {
    report
        .lines()
        .filter_map(|line| line.strip_prefix("  - "))
        .map(str::to_owned)
        .collect()
}

/// A run that found an action due without a terminal to offer it at: the
/// report said which, so the run fails without another message.
#[derive(Debug)]
pub(crate) struct Due;

impl std::fmt::Display for Due {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("an action is due")
    }
}

impl std::error::Error for Due {}

/// Without a terminal: each stick, what it holds and the command that is
/// due, changing nothing.
pub(super) fn report(sticks: &[Stick]) -> (String, bool) {
    let mut text = String::new();
    if sticks.is_empty() {
        text.push_str("No USB stick found. Plug in a stick of 8 GB or more.\n");
        return (text, false);
    }
    let several = sticks.iter().filter(|s| s.record.is_some()).count() > 1;
    let mut due = false;
    for stick in sticks {
        let _ = writeln!(text, "{}", stick.line());
        if let Some(details) = stick.details() {
            let _ = writeln!(text, "  {details}");
        }
        match &stick.state {
            State::Newer(version) => {
                let _ = writeln!(
                    text,
                    "  This stela is {}. Download stela {version} or newer to change this stick.",
                    release::CURRENT
                );
            }
            State::Older => {
                let _ = writeln!(text, "  This stela is {}.", release::CURRENT);
            }
            State::Damaged(problems) => {
                for problem in problems {
                    let _ = writeln!(text, "  - {problem}");
                }
            }
            _ => {}
        }
        match stick.next(several) {
            Some(command) => {
                // Only a stick that holds Stela can be due: a blank one may
                // be meant for anything.
                due |= stick.record.is_some();
                let _ = writeln!(text, "Next: {command}");
            }
            // A newer stick's line named the download that changes it.
            None if matches!(stick.state, State::Healthy) => {
                let _ = writeln!(text, "Nothing to do.");
            }
            None => {}
        }
    }
    (text, due)
}

/// Plain `stela`.
pub(super) fn run(environment: &Environment<'_>, request: Request) -> Result<()> {
    let ui = environment.ui;
    if !ui.interactive() {
        let sticks = sticks(environment)?;
        let (text, due) = report(&sticks);
        print!("{text}");
        if !due {
            return Ok(());
        }
        // `--yes` accepts what is due: each stick's command runs as
        // `stela update --yes` or `stela repair --yes` would.
        if !request.yes {
            return Err(Due.into());
        }
        for stick in &sticks {
            if let Some(action) = stick.due() {
                perform(environment, maintenance(&request, stick, action))?;
            }
        }
        return Ok(());
    }
    ui.intro(&format!("Stela {}", release::CURRENT))?;
    match look(environment, request) {
        // Escape at a question stops there; what ran before it stays done.
        Err(error) if super::ui::cancelled(&error) => ui.cancel("Stopped."),
        // A no to an action's own question, as a command takes it.
        result => super::closed(ui, result),
    }
}

/// Find the sticks, choose one, and act on it.
fn look(environment: &Environment<'_>, request: Request) -> Result<()> {
    let ui = environment.ui;
    let sticks = loop {
        let sticks = sticks(environment)?;
        if !sticks.is_empty() {
            break sticks;
        }
        ui.warning("No USB stick found. Plug in a stick of 8 GB or more.")?;
        let rows = super::ui::rows(&[("Look again", ""), ("Quit", "")]);
        if ui.select("Look again?", &rows, 0)? == 1 {
            return quit(ui);
        }
    };
    let Some(stick) = choose(ui, sticks)? else {
        return quit(ui);
    };
    act(environment, request, stick)
}

/// The stick to act on: the one plugged in, or the one chosen.
pub(super) fn choose(ui: &dyn Ui, mut sticks: Vec<Stick>) -> Result<Option<Stick>> {
    if sticks.len() == 1 {
        return Ok(sticks.pop());
    }
    let mut rows: Vec<_> = sticks
        .iter()
        .enumerate()
        .map(|(value, stick)| super::wizard::menu::Item {
            value,
            label: stick.line(),
            hint: String::new(),
            enabled: true,
        })
        .collect();
    rows.push(super::wizard::menu::Item {
        value: usize::MAX,
        label: "Quit".into(),
        hint: String::new(),
        enabled: true,
    });
    match ui.select("Which stick?", &rows, 0)? {
        usize::MAX => Ok(None),
        index => Ok(Some(sticks.swap_remove(index))),
    }
}

fn quit(ui: &dyn Ui) -> Result<()> {
    ui.cancel("Nothing was changed.")
}

/// What can be done with a Stela stick from plain `stela`.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Choice {
    /// Finish an interrupted create.
    Finish,
    /// Finish an interrupted macOS switch.
    FinishSwitch,
    Update,
    Repair,
    /// Use the newest tested macOS for new installs.
    Macos,
    Settings,
    /// `status --full`: read every macOS piece.
    Check,
    Status,
    Logs,
    Erase,
    /// The other choices.
    Other,
    Quit,
}

/// What a stick's choices say, worked out once.
struct Offers {
    /// What finishing a create downloads, when known.
    finish_bytes: Option<u64>,
    finish: String,
    update: String,
    macos: Option<(&'static builds::Build, MacosText)>,
}

impl Offers {
    fn of(environment: &Environment<'_>, stick: &Stick) -> Self {
        let record = stick.record.as_ref();
        let finish_bytes = match (&stick.state, record) {
            (State::Interrupted, Some(record)) => {
                super::resume_bytes(environment, record, None, None).filter(|&b| b > 0)
            }
            _ => None,
        };
        let finish = match finish_bytes {
            Some(bytes) => format!(
                "Keeps what is written, checks it, and downloads the rest ({})",
                host::size_text(bytes)
            ),
            None => "Keeps what is written, checks it, and writes the rest".into(),
        };
        let update = match stick.state {
            State::Older => update_text(environment, record),
            _ => String::new(),
        };
        let macos = record
            .filter(|_| settled(stick))
            .and_then(|record| Some((newer_macos(record)?, record)))
            .map(|(newest, record)| (newest, macos_text(environment, record, newest.build)));
        Self {
            finish_bytes,
            finish,
            update,
            macos,
        }
    }
}

/// A complete stick that this stela may change.
fn settled(stick: &Stick) -> bool {
    matches!(
        stick.state,
        State::Older | State::Damaged(_) | State::Healthy
    )
}

/// Every choice for `stick` in its state, in the menu's order: what its
/// state calls for first, then the same rows for every state, each where
/// it applies.
fn choices(stick: &Stick, offers: &Offers) -> Vec<Choice> {
    let mut all = match &stick.state {
        State::Interrupted => vec![Choice::Finish],
        State::Replacing(_) => vec![Choice::FinishSwitch],
        State::Older => vec![Choice::Update, Choice::Repair],
        State::Damaged(_) | State::Healthy => vec![Choice::Repair],
        State::Empty | State::Newer(_) => Vec::new(),
    };
    // Only an Online installer's install uses the stick's network settings.
    let offline = stick
        .record
        .as_ref()
        .is_some_and(|r| r.active_medium().is_some_and(|m| !m.remote));
    if offers.macos.is_some() {
        all.push(Choice::Macos);
    }
    if settled(stick) && !offline {
        all.push(Choice::Settings);
    }
    if offline {
        all.push(Choice::Check);
    }
    all.extend([Choice::Status, Choice::Logs]);
    // A newer Stela's stick is changed by that Stela only.
    if !matches!(stick.state, State::Newer(_)) {
        all.push(Choice::Erase);
    }
    all.push(Choice::Quit);
    all
}

/// The menu row of `choice`; `first` for the short question a state asks
/// first, where a row is named for that question.
fn row(choice: Choice, stick: &Stick, offers: &Offers, first: bool) -> super::wizard::menu::Item {
    let (label, hint): (String, &str) = match choice {
        Choice::Finish if first => ("Finish it".into(), offers.finish.as_str()),
        Choice::Finish => ("Finish the create".into(), offers.finish.as_str()),
        Choice::FinishSwitch => {
            let next = match &stick.state {
                State::Replacing(next) => macos_name(next),
                _ => String::new(),
            };
            (
                format!("Finish the switch to macOS {next}"),
                "Writes the rest of the new copy",
            )
        }
        Choice::Update if first => ("Update".into(), ""),
        Choice::Update => (
            format!("Update to Stela {}", release::CURRENT),
            offers.update.as_str(),
        ),
        Choice::Repair => (
            "Repair".into(),
            "Puts back what is damaged or missing; asks before a download over 1 GB",
        ),
        Choice::Macos => match &offers.macos {
            Some((newest, offer)) => (
                format!("Use macOS {} for new installs", newest.version),
                offer.text.as_str(),
            ),
            None => unreachable!("offered with a build"),
        },
        Choice::Settings => (
            "Change Wi-Fi or network settings".into(),
            "For an install that downloads macOS",
        ),
        Choice::Check => (
            "Check every file".into(),
            "Reads every macOS piece and changes nothing; takes a few minutes",
        ),
        Choice::Status => ("Show status".into(), "Changes nothing"),
        Choice::Logs => ("Save the boot logs for a bug report".into(), ""),
        Choice::Erase if first => ("Start over".into(), "Erases the stick"),
        Choice::Erase => ("Erase it and make a new installer".into(), ""),
        Choice::Other => ("Other choices".into(), ""),
        Choice::Quit => ("Quit".into(), ""),
    };
    super::wizard::menu::Item {
        value: choice as usize,
        label,
        hint: hint.into(),
        enabled: true,
    }
}

/// Ask `question` with `choices`, Enter taking `initial`.
fn ask(
    ui: &dyn Ui,
    question: &str,
    stick: &Stick,
    offers: &Offers,
    choices: &[Choice],
    initial: Choice,
    first: bool,
) -> Result<Choice> {
    let rows: Vec<_> = choices
        .iter()
        .map(|&choice| row(choice, stick, offers, first))
        .collect();
    let value = ui.select(question, &rows, initial as usize)?;
    Ok(*choices
        .iter()
        .find(|&&choice| choice as usize == value)
        .expect("a row's value is its choice"))
}

/// Offer the next step for `stick`, then the other choices.
fn act(environment: &Environment<'_>, request: Request, stick: Stick) -> Result<()> {
    let ui = environment.ui;
    ui.info(&stick.name())?;
    if let Some(details) = stick.details() {
        ui.info(&details)?;
    }
    let offers = Offers::of(environment, &stick);
    // The question a state asks first, and its choices.
    let first = |question: &str, choices: &[Choice]| -> Result<Choice> {
        let mut choices = choices.to_vec();
        choices.extend([Choice::Other, Choice::Quit]);
        ask(ui, question, &stick, &offers, &choices, choices[0], true)
    };
    let choice = match &stick.state {
        State::Empty if stick.disk.bytes != 0 && stick.disk.bytes < NETWORK_MIN => {
            ui.warning(&format!(
                "This stick holds {}; an installer needs one of 8 GB or more.",
                host::size_text(stick.disk.bytes)
            ))?;
            return quit(ui);
        }
        State::Empty => return create(environment, request, &stick),
        State::Interrupted => {
            let record = stick.record.as_ref().expect("a setup has its record");
            ui.warning(&format!(
                "A create stopped part way: {}.",
                record.resume_summary()
            ))?;
            first("Finish the create?", &[Choice::Finish, Choice::Erase])?
        }
        State::Replacing(next) => {
            ui.warning(&format!(
                "A switch to macOS {} stopped before its copy was complete: this stick cannot \
                 install macOS until it is.",
                macos_name(next)
            ))?;
            first("Finish the switch?", &[Choice::FinishSwitch])?
        }
        State::Newer(version) => {
            ui.warning(&format!(
                "This stick has Stela {version}, newer than this stela ({}). Download stela \
                 {version} or newer to change it.",
                release::CURRENT
            ))?;
            Choice::Other
        }
        State::Older => {
            ui.info(&offers.update)?;
            first(
                &format!("Update this stick to Stela {}?", release::CURRENT),
                &[Choice::Update],
            )?
        }
        State::Damaged(problems) => {
            ui.warning(&format!(
                "This stick needs repair:\n{}",
                problems.join("\n")
            ))?;
            first("Repair this stick?", &[Choice::Repair])?
        }
        State::Healthy => {
            ui.info("This stick is ready.")?;
            Choice::Other
        }
    };
    let choice = match choice {
        // Quit is the choice Enter takes: no row that writes the stick is.
        Choice::Other => ask(
            ui,
            "What next?",
            &stick,
            &offers,
            &choices(&stick, &offers),
            Choice::Quit,
            false,
        )?,
        choice => choice,
    };
    run_choice(environment, request, &stick, offers, choice)
}

/// Do `choice` for `stick`.
fn run_choice(
    environment: &Environment<'_>,
    request: Request,
    stick: &Stick,
    offers: Offers,
    choice: Choice,
) -> Result<()> {
    let maintenance = |action: Action| maintenance(&request, stick, action);
    let volume = stick.volume.clone().unwrap_or_default();
    match choice {
        // The choice named what it downloads, when that was known: that
        // download is not asked about again, any other is.
        Choice::Finish => perform(
            environment,
            Request {
                accepted_download: offers.finish_bytes,
                ..maintenance(Action::Repair)
            },
        ),
        Choice::FinishSwitch | Choice::Repair => perform(environment, maintenance(Action::Repair)),
        Choice::Update => {
            act_on(environment, maintenance(Action::UpdateStela))?;
            offer_macos(environment, &request, &volume)
        }
        Choice::Macos => {
            let (newest, offer) = offers.macos.expect("offered with a build");
            change_macos(environment, &request, &volume, newest.build, offer.named)
        }
        Choice::Settings => {
            let mut settings = maintenance(Action::Configure);
            match super::settings::ask(environment, &mut settings, stick.record.as_ref())? {
                true => perform(environment, settings),
                // The settings' own question said that nothing was changed.
                false => Ok(()),
            }
        }
        // Reads every piece and changes nothing, as its row says.
        Choice::Check => perform(
            environment,
            Request {
                full: true,
                ..maintenance(Action::Status)
            },
        ),
        Choice::Status => perform(environment, maintenance(Action::Status)),
        Choice::Logs => perform(environment, log_request(&volume)),
        Choice::Erase => create(environment, request, stick),
        Choice::Other | Choice::Quit => quit(environment.ui),
    }
}

/// What an update does, with what it downloads.
fn update_text(environment: &Environment<'_>, record: Option<&Record>) -> String {
    let download = record
        .filter(|record| record.generation() != environment.generation())
        .and_then(|record| {
            let lookup = sources::Lookup {
                need: sources::Need::BootFiles,
                build: &record.build,
                model: &record.model,
                ipsw: None,
                download_dir: None,
                medium: record.active_medium(),
            };
            sources::locate(environment, &lookup).ok()
        });
    match download {
        None => "Replaces stela.efi; its boot files stay.".into(),
        Some(source) => match sources::download_bytes(&source) {
            0 => "Replaces stela.efi and prepares the boot files again.".into(),
            bytes => format!(
                "Replaces stela.efi and prepares the boot files again (downloads {}).",
                host::size_text(bytes)
            ),
        },
    }
}

/// The newest tested macOS, when it is newer than the one `record`
/// installs.
fn newer_macos(record: &Record) -> Option<&'static builds::Build> {
    let newest = builds::DEFAULT;
    (record.build != newest.build
        && builds::build_major(&record.build) <= builds::build_major(newest.build))
    .then_some(newest)
}

/// After an update: offer the newest tested macOS for new installs, saying
/// what it downloads and removes. The default is no when it downloads more
/// than 1 GB.
fn offer_macos(environment: &Environment<'_>, request: &Request, volume: &Path) -> Result<()> {
    let ui = environment.ui;
    let record = read_record(environment, volume)?;
    let Some(newest) = newer_macos(&record) else {
        return ui.outro(DONE);
    };
    let offer = macos_text(environment, &record, newest.build);
    ui.info(&offer.text)?;
    let rows = super::ui::rows(&[("Yes", ""), ("Not now", "")]);
    let question = format!(
        "macOS {} is available. Use it for new installs?",
        newest.version
    );
    match ui.select(&question, &rows, usize::from(offer.large))? {
        0 => change_macos(environment, request, volume, newest.build, offer.named),
        _ => ui.outro(DONE),
    }
}

/// What switching `record`'s stick to `build` downloads and removes.
struct MacosText {
    text: String,
    /// It downloads more than 1 GB.
    large: bool,
    /// The text names the download and the copy removed for room: its
    /// source was found.
    named: bool,
}

/// What switching `record`'s stick to `build` downloads and removes.
fn macos_text(environment: &Environment<'_>, record: &Record, build: &str) -> MacosText {
    let need = sources::need(record, build, true).unwrap_or(sources::Need::BootFiles);
    let lookup = sources::Lookup {
        need,
        build,
        model: &record.model,
        ipsw: None,
        download_dir: None,
        medium: record.medium_of(build),
    };
    let mut text = String::new();
    let mut large = false;
    let mut named = false;
    match sources::locate(environment, &lookup) {
        Ok(source) => {
            named = true;
            let bytes = sources::download_bytes(&source);
            large = bytes > super::LARGE_DOWNLOAD;
            if bytes > 0 {
                let _ = write!(text, "Downloads {}. ", host::size_text(bytes));
            }
            if need.whole()
                && sources::check_room(environment, record, build, &source).is_err()
                && let Some(old) = record.active_medium().filter(|m| !m.remote)
            {
                let _ = write!(
                    text,
                    "The stick has room for one copy of macOS: {}'s copy is removed first, and \
                     the stick cannot install macOS until the new copy is complete. ",
                    macos_name(&old.build)
                );
            }
        }
        Err(error) => {
            let _ = write!(text, "{error:#}. ");
        }
    }
    text.push_str("The installed macOS is not changed; it updates itself.");
    MacosText { text, large, named }
}

/// Switch the stick to `build`: when the question that offered it named
/// what it downloads and removes (`named`), the switch asks nothing more;
/// otherwise it asks as `stela update --macos` does.
fn change_macos(
    environment: &Environment<'_>,
    request: &Request,
    volume: &Path,
    build: &str,
    named: bool,
) -> Result<()> {
    perform(
        environment,
        Request {
            action: Action::UpdateOs,
            usb: volume.to_path_buf(),
            build: build.into(),
            dry_run: request.dry_run,
            efi: request.efi.clone(),
            yes: request.yes || named,
            ..Request::default()
        },
    )
}

/// `log` of the stick at `volume`, saved to a file in this folder.
fn log_request(volume: &Path) -> Request {
    Request {
        action: Action::BootLog,
        usb: volume.to_path_buf(),
        output: Some(crate::boot_log::default_output()),
        ..Request::default()
    }
}

/// Make an installer on `stick` with the wizard, erasing it.
fn create(environment: &Environment<'_>, mut request: Request, stick: &Stick) -> Result<()> {
    if !wizard::configure(environment, &mut request, stick)? {
        return Ok(());
    }
    request.action = Action::Create;
    super::create_installer(environment, request)
}

/// Run `request`, show what it did, and close the run with one line.
pub(super) fn perform(environment: &Environment<'_>, request: Request) -> Result<()> {
    let closing = act_on(environment, request)?;
    environment.ui.outro(&closing)
}

/// How a run that changed the stick closes.
const DONE: &str = "Done. Check it any time with `stela status`.";

/// Run `request` and show what it did, or save it to the request's output
/// file; and the line that closes the run: where the file went, what
/// `status` found, or that it is done.
fn act_on(environment: &Environment<'_>, mut request: Request) -> Result<String> {
    let output = request.output.take();
    let status = matches!(request.action, Action::Status);
    let dry_run = request.dry_run;
    let result = match request.dispatch(environment) {
        // A `status` that finds problems did what it was asked: its report
        // names them and its summary closes, with no error after it.
        Err(error) if status => match error.downcast::<Reported>() {
            Ok(reported) => {
                print!("{}", reported.report);
                return Ok(reported.summary);
            }
            Err(error) => Err(error),
        },
        result => result,
    };
    let (report, result) = super::split_report(result);
    if let (Some(path), Ok(())) = (&output, &result) {
        std::fs::write(path, &report)
            .with_context(|| format!("cannot write {}", path.display()))?;
        #[cfg(unix)]
        super::hand_to_invoker(path);
        return Ok(format!(
            "Saved the boot logs to {}",
            crate::boot_log::shown(path).display()
        ));
    }
    print!("{report}");
    result?;
    Ok(match (status, dry_run) {
        (true, _) => "No problems found.",
        (false, true) => "Done.",
        (false, false) => DONE,
    }
    .into())
}
