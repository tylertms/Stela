use super::*;
use stela_soc_data::nor::nvram::empty_bank;

fn variables(record: Option<&[u8]>) -> Variables {
    let mut variables = Variables::parse(&empty_bank()).unwrap();
    if let Some(value) = record {
        variables
            .set(Partition::System, BUNDLE_REPAIR, value)
            .unwrap();
    }
    variables
}

#[test]
fn a_normal_boot_repairs_once_and_then_stops_with_the_manual_remedy() {
    let missing = |record| decide(Report::Missing, Boot::Normal, record, true);
    assert_eq!(missing(Record::None), Decision::Repair);
    assert_eq!(missing(Record::Other), Decision::Repair);
    assert_eq!(
        missing(Record::Attempted),
        Decision::StopMissing { attempted: true }
    );
    // A request the loader did not take means NVRAM was not read for it.
    assert_eq!(
        missing(Record::Pending),
        Decision::StopMissing { attempted: false }
    );
    assert_eq!(
        missing(Record::Installed),
        Decision::StopMissing { attempted: false }
    );
    // Without NVRAM to write, no repair can be asked for.
    assert_eq!(
        decide(Report::Missing, Boot::Normal, Record::None, false),
        Decision::StopMissing { attempted: false }
    );
    assert_eq!(
        decide(Report::Failed, Boot::Normal, Record::None, true),
        Decision::StopFailed
    );
}

#[test]
fn single_user_boots_only_log_and_an_installed_plugin_forgets_the_repair() {
    for boot in [Boot::SingleUser, Boot::Repair] {
        for report in [Report::Missing, Report::Failed] {
            assert_eq!(decide(report, boot, Record::Attempted, true), Decision::Log);
        }
    }
    for boot in [Boot::Normal, Boot::SingleUser, Boot::Repair] {
        for report in [Report::Current, Report::Updated] {
            assert_eq!(
                decide(report, boot, Record::None, true),
                Decision::Continue(None)
            );
            assert_eq!(
                decide(report, boot, Record::Attempted, false),
                Decision::Continue(Some(Change::Forget))
            );
        }
    }
}

#[test]
fn the_record_round_trips_through_nvram() {
    let mut vars = variables(None);
    assert_eq!(Record::read(&vars), Record::None);
    assert!(Change::Request.apply(&mut vars).unwrap());
    assert!(!Change::Request.apply(&mut vars).unwrap());
    let saved = Variables::parse(&vars.encode().unwrap()).unwrap();
    assert_eq!(Record::read(&saved), Record::Pending);
    assert_eq!(
        Record::read(&variables(Some(b"attempted\0"))),
        Record::Attempted
    );
    assert_eq!(Record::read(&variables(Some(b"x"))), Record::Other);
    // An erase install's request replaces whatever an earlier system left.
    let mut installed = variables(Some(BUNDLE_REPAIR_ATTEMPTED));
    assert!(Change::Installed.apply(&mut installed).unwrap());
    assert!(!Change::Installed.apply(&mut installed).unwrap());
    assert_eq!(Record::read(&installed), Record::Installed);
    assert!(Change::Forget.apply(&mut installed).unwrap());
    let mut attempted = variables(Some(BUNDLE_REPAIR_ATTEMPTED));
    attempted
        .set(Partition::Common, BUNDLE_REPAIR, BUNDLE_REPAIR_ATTEMPTED)
        .unwrap();
    assert!(Change::Forget.apply(&mut attempted).unwrap());
    assert_eq!(Record::read(&attempted), Record::None);
    assert!(!Change::Forget.apply(&mut attempted).unwrap());
}

#[test]
fn the_steps_create_the_directories_the_manual_remedy_does() {
    let steps = steps();
    let commands: Vec<&str> = steps.iter().map(|step| step.command.as_str()).collect();
    assert_eq!(
        &commands[..3],
        [
            "/sbin/mount -P 1",
            "/usr/libexec/init_data_protection",
            "/sbin/mount -P 2"
        ]
    );
    let all = "/System/Volumes/Data/Library/GPUBundles/StelaGPU.bundle \
               /System/Volumes/Data/Library/GPUBundles/StelaGPU.bundle/Contents \
               /System/Volumes/Data/Library/GPUBundles/StelaGPU.bundle/Contents/MacOS";
    assert!(commands.contains(&format!("/bin/mkdir -p -m 0755 {all}").as_str()));
    assert!(commands.contains(&format!("/usr/sbin/chown root:wheel {all}").as_str()));
    // The rootless attribute fails with EPERM even with protection off,
    // and Metal loads the plugin without it.
    assert!(!commands.iter().any(|command| command.contains("xattr")));
    let chflags = format!("/usr/bin/chflags restricted {all}");
    assert!(commands.contains(&chflags.as_str()));
    // Only the restricted flag and the read-only remount after the sync are
    // optional.
    let remount = "/sbin/mount -u -r /System/Volumes/Data";
    for step in &steps {
        assert_eq!(
            step.required,
            step.command != chflags && step.command != remount,
            "{}",
            step.command
        );
    }
    assert_eq!(&commands[commands.len() - 2..], ["/bin/sync", remount]);
    // The restart skips the unmount walk that stalled; the sync stays.
    assert_eq!(REBOOT, "/sbin/reboot -q");
    assert!(SYNC_AND_REBOOT.starts_with("/bin/sync; ") && SYNC_AND_REBOOT.ends_with(REBOOT));
    assert!(
        steps
            .iter()
            .all(|step| step.seconds > 0 && step.seconds <= 180)
    );
}

/// A fake single-user console: echoes what is typed after its prompt and
/// answers each command's `echo` with `status(command)`.
struct Console {
    pending: Vec<u8>,
    lines: Vec<String>,
}

impl Console {
    fn new() -> Self {
        Self {
            pending: Vec::new(),
            lines: Vec::new(),
        }
    }

    /// Take what the driver types and print the guest's lines for each
    /// whole line typed.
    fn run(&mut self, driver: &mut Driver, status: &dyn Fn(&str) -> Option<u32>) -> Vec<String> {
        while let Some(byte) = driver.next_byte() {
            self.pending.push(byte);
        }
        let mut out = Vec::new();
        while let Some(end) = self.pending.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.pending.drain(..=end).collect();
            let line = String::from_utf8(line[..line.len() - 1].to_vec()).unwrap();
            self.lines.push(line.clone());
            out.push(format!("-sh-3.2# {line}"));
            out.push("nx_volume_group_update:819: disk3s1 inode 1 already in stitched list".into());
            if let Some((command, echo)) = line.split_once("; echo ")
                && let Some(status) = status(command)
            {
                out.push(echo.replace("$?", &status.to_string()));
            }
        }
        out
    }
}

const BOOT: [&str; 4] = [
    "com.apple.xpc.launchd (boot) <Notice>: Entering single-user mode.",
    "*** Single-user boot ***",
    "Root device is mounted read-only",
    "\t$ /sbin/mount -P 1",
];

/// The shell's prompt, which ends its line without a line feed.
const PROMPT_TEXT: &[u8] = b"-sh-3.2# ";

fn print(driver: &mut Driver, bytes: &[u8], now: u64) {
    for &byte in bytes {
        driver.output(byte, now);
    }
}

fn start(driver: &mut Driver, now: &mut u64) {
    for line in BOOT {
        print(driver, line.as_bytes(), *now);
        print(driver, b"\r\n", *now);
        driver.line(line, *now);
    }
    // Nothing is typed before the shell's prompt shows, however long it
    // takes.
    driver.tick(*now + PROMPT_SECONDS * 1000 - 1);
    assert_eq!(driver.next_byte(), None);
    *now += 1000;
    print(driver, PROMPT_TEXT, *now);
}

fn pump(
    driver: &mut Driver,
    console: &mut Console,
    now: &mut u64,
    status: &dyn Fn(&str) -> Option<u32>,
) {
    for _ in 0..64 {
        let lines = console.run(driver, status);
        if lines.is_empty() {
            break;
        }
        for line in lines {
            *now += 10;
            driver.line(&line, *now);
        }
    }
}

#[test]
fn the_driver_types_each_step_after_the_last_succeeded_then_restarts() {
    let mut now = 1_000;
    let mut driver = Driver::new(steps(), now);
    let mut console = Console::new();
    start(&mut driver, &mut now);
    pump(&mut driver, &mut console, &mut now, &|_| Some(0));
    assert!(!driver.failed());
    let expected: Vec<String> = steps()
        .iter()
        .enumerate()
        .map(|(index, step)| format!("{}; echo STELA-REPAIR-{}-rc=$?", step.command, index + 1))
        .chain([REBOOT.to_string()])
        .collect();
    assert_eq!(console.lines, expected);
    let events = driver.take_events();
    assert_eq!(events.first(), Some(&Event::Shell));
    assert_eq!(events.last(), Some(&Event::Rebooting { repaired: true }));
    assert_eq!(
        events
            .iter()
            .filter(|event| matches!(event, Event::Passed { .. }))
            .count(),
        steps().len()
    );
    // The restart ends the boot; no restart in time is a failure.
    driver.tick(now + RESTART_SECONDS * 1000);
    assert_eq!(driver.take_events(), [Event::Failed(Failure::NoRestart)]);
}

#[test]
fn a_failing_required_step_syncs_and_restarts_without_the_rest() {
    let mut now = 0;
    let mut driver = Driver::new(steps(), now);
    let mut console = Console::new();
    start(&mut driver, &mut now);
    let status = |command: &str| Some(u32::from(command.starts_with("/sbin/mount -P 2")) * 66);
    pump(&mut driver, &mut console, &mut now, &status);
    // The guest restarts itself, so the engine need not.
    assert!(!driver.failed());
    assert_eq!(console.lines.len(), 4);
    assert_eq!(console.lines[3], SYNC_AND_REBOOT);
    let events = driver.take_events();
    assert!(events.contains(&Event::Failed(Failure::Exit {
        index: 2,
        status: 66
    })));
    assert_eq!(events.last(), Some(&Event::Rebooting { repaired: false }));
    // A later status line types nothing more.
    driver.line("STELA-REPAIR-4-rc=0", now);
    assert_eq!(driver.next_byte(), None);
    assert!(driver.take_events().is_empty());
    // No restart in time: the engine restarts the PC.
    driver.tick(now + RESTART_SECONDS * 1000);
    assert!(driver.failed());
    assert_eq!(driver.take_events(), [Event::Failed(Failure::NoRestart)]);
}

#[test]
fn a_failing_optional_step_is_logged_and_the_repair_goes_on() {
    let mut now = 0;
    let mut driver = Driver::new(steps(), now);
    let mut console = Console::new();
    start(&mut driver, &mut now);
    let status = |command: &str| Some(u32::from(command.starts_with("/usr/bin/chflags")));
    pump(&mut driver, &mut console, &mut now, &status);
    assert!(!driver.failed());
    let chflags = steps().iter().position(|step| !step.required).unwrap();
    let events = driver.take_events();
    assert!(events.contains(&Event::Skipped {
        index: chflags,
        status: 1
    }));
    assert!(!events.iter().any(|event| matches!(event, Event::Failed(_))));
    assert_eq!(events.last(), Some(&Event::Rebooting { repaired: true }));
    assert_eq!(console.lines.len(), steps().len() + 1);
    assert_eq!(console.lines.last().map(String::as_str), Some(REBOOT));
}

#[test]
fn no_banner_and_a_silent_step_time_out() {
    let mut driver = Driver::new(steps(), 0);
    driver.line("panic(cpu 0 caller 0x0): something else", 1);
    driver.tick(BANNER_SECONDS * 1000 - 1);
    assert!(!driver.failed());
    driver.tick(BANNER_SECONDS * 1000);
    assert_eq!(driver.take_events(), [Event::Failed(Failure::NoShell)]);
    assert_eq!(driver.next_byte(), None);

    // A step whose command never returns (no `echo` output: the console
    // only echoes the typed line).
    let mut now = 0;
    let mut driver = Driver::new(steps(), now);
    let mut console = Console::new();
    start(&mut driver, &mut now);
    let status = |command: &str| (command != "/usr/libexec/init_data_protection").then_some(0);
    pump(&mut driver, &mut console, &mut now, &status);
    assert!(!driver.failed());
    driver.tick(now + 180 * 1000);
    assert!(driver.failed());
    assert!(
        driver
            .take_events()
            .contains(&Event::Failed(Failure::Timeout { index: 1 }))
    );
}

#[test]
fn only_the_current_step_s_status_line_counts() {
    let mut now = 0;
    let mut driver = Driver::new(steps(), now);
    start(&mut driver, &mut now);
    while driver.next_byte().is_some() {}
    driver.take_events();
    // The shell's echo of the command, another step's marker and a marker
    // whose step number only starts the same are all ignored.
    for line in [
        "-sh-3.2# /sbin/mount -P 1; echo STELA-REPAIR-1-rc=$?",
        "STELA-REPAIR-2-rc=0",
        "STELA-REPAIR-11-rc=0",
    ] {
        driver.line(line, now);
    }
    assert!(driver.take_events().is_empty());
    // Kernel text before the status on the same line does not hide it.
    driver.line("nx_volume_group_update: x STELA-REPAIR-1-rc=0", now);
    assert_eq!(driver.take_events()[0], Event::Passed { index: 0 });
}

#[test]
fn power_loss_in_the_middle_leaves_no_request_to_repeat() {
    // The loader recorded the attempt before the repair boot; a boot after
    // a power loss mid-repair reads it and, the plugin still missing, stops.
    let mut vars = variables(Some(BUNDLE_REPAIR_PENDING));
    let selection = stela_loader::boot_selection::select(&vars).unwrap();
    assert!(selection.bundle_repair);
    stela_loader::boot_selection::consume(&mut vars, &selection).unwrap();
    let record = Record::read(&vars);
    assert_eq!(record, Record::Attempted);
    assert_eq!(
        decide(Report::Missing, Boot::Normal, record, true),
        Decision::StopMissing { attempted: true }
    );
    // Once installed by hand, the record goes and a later loss repairs again.
    if let Decision::Continue(Some(change)) = decide(Report::Current, Boot::Normal, record, true) {
        change.apply(&mut vars).unwrap();
    }
    assert_eq!(
        decide(Report::Missing, Boot::Normal, Record::read(&vars), true),
        Decision::Repair
    );
}

#[test]
fn events_read_as_log_lines() {
    assert_eq!(
        describe(&Event::Passed { index: 0 }),
        "BUNDLE REPAIR step 1 ok"
    );
    assert_eq!(
        describe(&Event::Failed(Failure::Exit {
            index: 4,
            status: 1
        })),
        "BUNDLE REPAIR failed: step 5 exited with status 1"
    );
}

/// A driver whose every step passed and that has typed the restart.
fn rebooting(now: &mut u64) -> Driver {
    let mut driver = Driver::new(steps(), *now);
    let mut console = Console::new();
    start(&mut driver, now);
    pump(&mut driver, &mut console, now, &|_| Some(0));
    assert_eq!(
        driver.take_events().last(),
        Some(&Event::Rebooting { repaired: true })
    );
    driver
}

#[test]
fn a_restart_that_stops_printing_fails_after_the_quiet_bound_and_a_busy_one_at_the_cap() {
    // A stall after the last console line, as at the xarts unmount.
    let mut now = 0;
    let mut driver = rebooting(&mut now);
    let last = now + 1000;
    driver.line("unmount(0) /System/Volumes/xarts", last);
    driver.tick(last + QUIET_SECONDS * 1000 - 1);
    assert!(!driver.failed());
    driver.tick(last + QUIET_SECONDS * 1000);
    assert!(driver.failed());
    assert_eq!(driver.take_events(), [Event::Failed(Failure::NoRestart)]);
    assert_eq!(driver.next_byte(), None);
    // A restart that keeps printing still ends at the cap.
    let mut now = 0;
    let mut driver = rebooting(&mut now);
    let cap = now + RESTART_SECONDS * 1000;
    while now + 10_000 < cap {
        now += 10_000;
        driver.line("IOPlatformHaltRestartAction -> AppleT8112PMGR", now);
        driver.tick(now);
        assert!(!driver.failed());
    }
    // The restart was typed just before the console's last echo lines.
    driver.tick(cap - 1000);
    assert!(!driver.failed());
    driver.tick(cap);
    assert!(driver.failed());
    const { assert!(QUIET_SECONDS < RESTART_SECONDS) };
}

#[test]
fn a_prompt_before_the_banner_or_in_the_middle_of_a_line_types_nothing() {
    let mut driver = Driver::new(steps(), 0);
    print(&mut driver, b"-sh-3.2# ", 1);
    assert_eq!(driver.next_byte(), None);
    print(&mut driver, b"\n", 1);
    driver.line(BANNER, 2);
    // The banner's own help lines end in `$ ` commands, not a prompt.
    print(&mut driver, b"\t$ /sbin/mount -P 1\r\n", 3);
    print(
        &mut driver,
        b"apfs: disk3s1 # 4 -sh-3.2# in kernel text\n",
        3,
    );
    assert_eq!(driver.next_byte(), None);
    // A kernel line that broke into the prompt leaves its end on a line
    // of its own.
    print(&mut driver, b"-sh-3.2", 4);
    print(&mut driver, b"nx_volume_group_update:819: disk3s1\r\n# ", 4);
    assert_eq!(
        driver.take_events(),
        [
            Event::Shell,
            Event::Prompt,
            Event::Sent {
                index: 0,
                command: "/sbin/mount -P 1".into()
            }
        ]
    );
    assert_eq!(driver.next_byte(), Some(b'/'));
}

#[test]
fn a_shell_interrupted_before_its_prompt_ends_the_repair_at_once_as_unanswered() {
    // The sequence both reports of issue 74 logged.
    let mut driver = Driver::new(steps(), 0);
    driver.line("*** Single-user boot ***", 36_736);
    driver.line(
        "com.apple.xpc.launchd|2026-11-14 16:24:07.756689 <Notice>: Single-user shell was signaled: Interrupt: 2",
        36_781,
    );
    assert!(driver.failed());
    let failure = Failure::ShellLeft { index: None };
    assert_eq!(driver.failure(), Some(&failure));
    assert!(failure.unanswered());
    assert_eq!(
        driver.take_events(),
        [Event::Shell, Event::Failed(failure.clone())]
    );
    assert_eq!(
        describe(&Event::Failed(failure)),
        "BUNDLE REPAIR failed: the single-user shell ended before it took input; nothing was typed"
    );
    // The multi-user boot launchd goes on to gets nothing typed, prompt or
    // not.
    print(&mut driver, b"-sh-3.2# ", 41_000);
    driver.tick(161_000);
    assert_eq!(driver.next_byte(), None);
    assert!(driver.take_events().is_empty());
}

#[test]
fn no_prompt_in_time_is_unanswered_and_a_shell_lost_mid_step_is_not() {
    let mut driver = Driver::new(steps(), 0);
    driver.line(BANNER, 1000);
    driver.tick(1000 + PROMPT_SECONDS * 1000 - 1);
    assert!(!driver.failed());
    driver.tick(1000 + PROMPT_SECONDS * 1000);
    assert_eq!(driver.failure(), Some(&Failure::NoPrompt));
    assert!(Failure::NoPrompt.unanswered());
    assert_eq!(driver.next_byte(), None);

    let mut now = 0;
    let mut driver = Driver::new(steps(), now);
    let mut console = Console::new();
    start(&mut driver, &mut now);
    let status = |command: &str| (command != "/sbin/mount -P 2").then_some(0);
    pump(&mut driver, &mut console, &mut now, &status);
    driver.line(
        "com.apple.xpc.launchd <Notice>: Single-user shell exited with status 1",
        now,
    );
    let failure = Failure::ShellLeft { index: Some(2) };
    assert_eq!(driver.failure(), Some(&failure));
    assert!(!failure.unanswered());
    // Neither are a missing banner, a silent step or a failed one.
    for failure in [
        Failure::NoShell,
        Failure::Timeout { index: 0 },
        Failure::Exit {
            index: 0,
            status: 1,
        },
        Failure::NoRestart,
    ] {
        assert!(!failure.unanswered(), "{failure}");
    }
}

#[test]
fn the_restart_ending_the_shell_is_no_failure() {
    let mut now = 0;
    let mut driver = rebooting(&mut now);
    driver.line(
        "com.apple.xpc.launchd <Notice>: Single-user shell was signaled: Killed: 9",
        now + 10,
    );
    assert!(!driver.failed());
}

#[test]
fn an_unanswered_repair_asks_for_one_more_and_stela_repair_rearms_a_finished_one() {
    // The first attempt, or an erase install's whose record the loader
    // removed, asks once more; the retry itself does not.
    assert_eq!(retry(Record::Attempted), Some(Change::Retry));
    assert_eq!(retry(Record::None), Some(Change::Retry));
    assert_eq!(retry(Record::Retried), None);
    let mut vars = variables(Some(BUNDLE_REPAIR_ATTEMPTED));
    assert!(Change::Retry.apply(&mut vars).unwrap());
    assert!(!Change::Retry.apply(&mut vars).unwrap());
    assert_eq!(Record::read(&vars), Record::Retry);
    let selection = stela_loader::boot_selection::select(&vars).unwrap();
    assert!(selection.bundle_repair);
    stela_loader::boot_selection::consume(&mut vars, &selection).unwrap();
    assert_eq!(Record::read(&vars), Record::Retried);
    // A normal boot after the second attempt stops as after the first.
    let missing = |record| decide(Report::Missing, Boot::Normal, record, true);
    assert_eq!(
        missing(Record::Retried),
        Decision::StopMissing { attempted: true }
    );
    assert_eq!(
        missing(Record::Retry),
        Decision::StopMissing { attempted: false }
    );
    assert_eq!(
        decide(Report::Current, Boot::Normal, Record::Retried, true),
        Decision::Continue(Some(Change::Forget))
    );
    // `stela repair`: a finished attempt becomes a request again.
    for value in [BUNDLE_REPAIR_ATTEMPTED, BUNDLE_REPAIR_RETRIED, &b"x"[..]] {
        let mut vars = variables(Some(value));
        assert!(again(&mut vars).unwrap());
        assert_eq!(Record::read(&vars), Record::Pending);
        assert!(!again(&mut vars).unwrap());
    }
    for value in [
        None,
        Some(BUNDLE_REPAIR_PENDING),
        Some(BUNDLE_REPAIR_INSTALLED),
        Some(BUNDLE_REPAIR_RETRY),
    ] {
        let mut vars = variables(value);
        let before = Record::read(&vars);
        assert!(!again(&mut vars).unwrap());
        assert_eq!(Record::read(&vars), before);
    }
}

#[test]
fn every_change_has_its_number() {
    for (number, change) in Change::ALL.into_iter().enumerate() {
        assert_eq!(change as usize, number);
    }
}
