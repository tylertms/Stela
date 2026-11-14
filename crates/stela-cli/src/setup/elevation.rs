//! Request disk-write rights before asking for installer preferences or
//! touching a disk, and keep the window the elevated copy runs in readable
//! when it finishes.
use anyhow::Result;

#[cfg(windows)]
static RELAUNCHED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// How a command gets the rights it needs before it downloads or changes
/// anything.
#[derive(Debug, PartialEq, Eq)]
pub(super) enum Rights {
    /// It has them, or needs none.
    Enough,
    /// It runs again with them: through `sudo` in the same terminal, or on
    /// Windows in a new administrator window.
    Relaunch,
    /// It stops and says how to run it with them ([`refused`]).
    Refuse,
}

/// What [`rights`] decides from.
pub(super) struct Situation {
    pub(super) windows: bool,
    /// Root on Unix, an elevated administrator on Windows.
    pub(super) administrator: bool,
    pub(super) dry_run: bool,
    /// The wizard, plain `stela`, `log` and a `status` that mounts the
    /// stick's EFI partition, whose output a new window may carry.
    pub(super) own_window: bool,
    /// Someone answers at a terminal: a new window reaches them, and a
    /// script that waits for the output and exit status gets neither.
    pub(super) interactive: bool,
    /// It may write to a disk, as every command but `status` and `log`
    /// does.
    pub(super) writes_disks: bool,
    /// It reads what Windows lets only an administrator read: every disk
    /// raw for `log`, and for `status` an EFI partition that has no drive
    /// letter, which only an administrator can give it.
    pub(super) admin_reads: bool,
}

/// A command's way to the rights it needs. A dry run changes nothing and
/// needs none. On Unix, `sudo` runs the same command in the same terminal.
/// On Windows, a relaunch opens a new window, which loses a scripted
/// command's output and exit status, so only a windowed command run at a
/// terminal is relaunched; any other command that writes a disk, or reads
/// what only an administrator may, stops before it downloads or changes
/// anything.
pub(super) fn rights(situation: Situation) -> Rights {
    let Situation {
        windows,
        administrator,
        dry_run,
        own_window,
        interactive,
        writes_disks,
        admin_reads,
    } = situation;
    if administrator || dry_run {
        Rights::Enough
    } else if !windows {
        Rights::Relaunch
    } else if !(writes_disks || admin_reads) {
        Rights::Enough
    } else if own_window && interactive {
        Rights::Relaunch
    } else {
        Rights::Refuse
    }
}

/// Why a command stopped at [`Rights::Refuse`], and how to run it: one that
/// writes disks (`writes`), or one that only reads them.
pub(super) fn refused(writes: bool) -> String {
    format!(
        "this command {}, which Windows allows only to an administrator. Open Terminal or \
         Command Prompt with Run as administrator and run the same command there. Nothing was \
         downloaded or changed.",
        match writes {
            true => "writes to disks",
            false => "reads the stick's EFI partition or every disk",
        }
    )
}

/// Whether this process has the rights to write disks: root.
#[cfg(not(windows))]
pub(super) fn is_administrator() -> bool {
    // SAFETY: reads this process's credentials; no pointer arguments.
    unsafe { libc::geteuid() == 0 }
}

/// Whether this process has the rights to write disks: an elevated
/// administrator.
#[cfg(windows)]
pub(super) fn is_administrator() -> bool {
    // SAFETY: queries the calling process's token; no pointer arguments.
    unsafe { windows_sys::Win32::UI::Shell::IsUserAnAdmin() != 0 }
}

/// Becomes root through `sudo` in this terminal: returns only when sudo
/// could not start.
#[cfg(not(windows))]
pub(super) fn relaunch() -> Result<()> {
    use anyhow::Context;
    use std::os::unix::process::CommandExt;
    let error = std::process::Command::new("sudo")
        .arg("--")
        .args(kept_environment(std::env::vars_os()))
        .arg(std::env::current_exe()?)
        .args(std::env::args_os().skip(1))
        .exec();
    Err(error).context("stela writes disks as root, and sudo could not start")
}

/// The proxy settings that stela's downloads need behind a proxy, as
/// `NAME=value` words for sudo, which otherwise clears them from the
/// environment of the command it runs.
#[cfg(not(windows))]
fn kept_environment(
    variables: impl Iterator<Item = (std::ffi::OsString, std::ffi::OsString)>,
) -> Vec<std::ffi::OsString> {
    const PROXY: [&str; 4] = ["http_proxy", "https_proxy", "all_proxy", "no_proxy"];
    variables
        .filter(|(name, value)| {
            !value.is_empty()
                && name
                    .to_str()
                    .is_some_and(|name| PROXY.contains(&name.to_ascii_lowercase().as_str()))
        })
        .map(|(name, value)| {
            let mut word = name;
            word.push("=");
            word.push(value);
            word
        })
        .collect()
}

#[cfg(test)]
mod rights_tests {
    use super::{Rights, Situation, rights};

    fn decide(windows: bool, administrator: bool, dry_run: bool, own_window: bool) -> Rights {
        rights(Situation {
            windows,
            administrator,
            dry_run,
            own_window,
            interactive: true,
            writes_disks: true,
            admin_reads: false,
        })
    }

    #[test]
    fn unix_runs_every_real_command_through_sudo() {
        assert_eq!(decide(false, false, false, false), Rights::Relaunch);
        assert_eq!(decide(false, false, false, true), Rights::Relaunch);
        assert_eq!(decide(false, true, false, false), Rights::Enough);
        assert_eq!(decide(false, false, true, false), Rights::Enough);
    }

    #[test]
    fn windows_relaunches_only_windowed_commands_and_refuses_other_disk_writes() {
        assert_eq!(decide(true, false, false, true), Rights::Relaunch);
        assert_eq!(decide(true, false, false, false), Rights::Refuse);
        assert_eq!(decide(true, true, false, false), Rights::Enough);
        assert_eq!(decide(true, true, false, true), Rights::Enough);
        assert_eq!(decide(true, false, true, false), Rights::Enough);
        // `status` writes no disk and runs with the rights it has.
        let status = |own_window, interactive, admin_reads| {
            rights(Situation {
                windows: true,
                administrator: false,
                dry_run: false,
                own_window,
                interactive,
                writes_disks: false,
                admin_reads,
            })
        };
        assert_eq!(status(false, true, false), Rights::Enough);
        assert_eq!(status(false, false, false), Rights::Enough);
        // Unless its EFI partition has no drive letter to read it through:
        // a new window at a terminal, refused without one.
        assert_eq!(status(true, true, true), Rights::Relaunch);
        assert_eq!(status(true, false, true), Rights::Refuse);
    }

    #[test]
    fn windows_never_relaunches_a_run_without_a_terminal() {
        // Plain `stela --yes` in a script: refused, not relaunched into a
        // window whose output and exit status the script never gets.
        let home = |interactive| {
            rights(Situation {
                windows: true,
                administrator: false,
                dry_run: false,
                own_window: true,
                interactive,
                writes_disks: true,
                admin_reads: false,
            })
        };
        assert_eq!(home(false), Rights::Refuse);
        assert_eq!(home(true), Rights::Relaunch);
        assert!(super::refused(true).contains("writes to disks"));
        assert!(super::refused(false).contains("reads the stick's EFI partition"));
    }
}

#[cfg(all(test, not(windows)))]
mod unix_tests {
    #[test]
    fn sudo_keeps_the_proxy_settings_alone() {
        let variables = [
            ("HTTPS_PROXY", "http://proxy.test:3128"),
            ("no_proxy", "localhost"),
            ("http_proxy", ""),
            ("HOME", "/home/someone"),
            ("PROXY_PASSWORD", "x"),
        ]
        .map(|(name, value)| (name.into(), value.into()));
        let kept = super::kept_environment(variables.into_iter());
        assert_eq!(
            kept,
            ["HTTPS_PROXY=http://proxy.test:3128", "no_proxy=localhost"]
                .map(std::ffi::OsString::from)
        );
    }
}

#[cfg(not(windows))]
pub(crate) fn hold_window(_: bool) {}

/// Windows closes a console window when its last process exits. The elevated
/// copy and a double-clicked stela.exe own theirs, so wait for a key there
/// before the result or error disappears; a terminal's shell keeps its window.
#[cfg(windows)]
pub(crate) fn hold_window(success: bool) {
    use std::{io::IsTerminal, sync::atomic::Ordering};
    use windows_sys::Win32::System::Console::GetConsoleProcessList;
    if RELAUNCHED.load(Ordering::Relaxed) || !std::io::stdin().is_terminal() {
        return;
    }
    let mut processes = [0u32; 2];
    // SAFETY: writes at most two process IDs into the local buffer.
    if unsafe { GetConsoleProcessList(processes.as_mut_ptr(), 2) } != 1 {
        return;
    }
    let prompt = if success {
        "Press any key to close this window."
    } else {
        "Stela stopped with an error. Press any key to close this window."
    };
    eprintln!("\n{}", console::style(prompt).for_stderr().dim());
    let _ = console::Term::stderr().read_key();
}

/// Runs this command again in a new administrator window, through the UAC
/// consent dialog.
#[cfg(windows)]
pub(super) fn relaunch() -> Result<()> {
    use anyhow::ensure;
    use std::{os::windows::ffi::OsStrExt, ptr};
    use windows_sys::Win32::UI::{Shell::ShellExecuteW, WindowsAndMessaging::SW_SHOWNORMAL};
    let exe: Vec<u16> = std::env::current_exe()?
        .as_os_str()
        .encode_wide()
        .chain(Some(0))
        .collect();
    let cwd: Vec<u16> = std::env::current_dir()?
        .as_os_str()
        .encode_wide()
        .chain(Some(0))
        .collect();
    let parameters = parameters(std::env::args_os().skip(1));
    let verb: Vec<u16> = "runas".encode_utf16().chain(Some(0)).collect();
    // SAFETY: all strings are NUL-terminated and live through this synchronous
    // launch request. Windows owns the UAC consent dialog and the new console.
    let result = unsafe {
        ShellExecuteW(
            ptr::null_mut(),
            verb.as_ptr(),
            exe.as_ptr(),
            parameters.as_ptr(),
            cwd.as_ptr(),
            SW_SHOWNORMAL,
        )
    };
    ensure!(
        result as isize > 32,
        "administrator access was cancelled or unavailable; your stick was not changed"
    );
    RELAUNCHED.store(true, std::sync::atomic::Ordering::Relaxed);
    Ok(())
}

#[cfg(windows)]
fn parameters(args: impl Iterator<Item = std::ffi::OsString>) -> Vec<u16> {
    use std::os::windows::ffi::OsStrExt;
    let mut result = Vec::new();
    for arg in args {
        if !result.is_empty() {
            result.push(b' ' as u16);
        }
        result.push(b'"' as u16);
        let mut slashes = 0;
        for unit in arg.encode_wide() {
            if unit == b'\\' as u16 {
                slashes += 1;
                continue;
            }
            result.extend(std::iter::repeat_n(
                b'\\' as u16,
                if unit == b'"' as u16 {
                    slashes * 2 + 1
                } else {
                    slashes
                },
            ));
            result.push(unit);
            slashes = 0;
        }
        result.extend(std::iter::repeat_n(b'\\' as u16, slashes * 2));
        result.push(b'"' as u16);
    }
    result.push(0);
    result
}

#[cfg(all(test, windows))]
mod tests {
    use super::*;
    #[test]
    fn elevation_preserves_argument_quoting() {
        let arguments = [
            "create",
            "--ipsw",
            "C:\\my images\\mac.ipsw",
            "a\"b",
            "C:\\tail\\",
            "--profile",
        ];
        let quoted = parameters(arguments.into_iter().map(Into::into));
        assert_eq!(
            String::from_utf16(&quoted[..quoted.len() - 1]).unwrap(),
            "\"create\" \"--ipsw\" \"C:\\my images\\mac.ipsw\" \"a\\\"b\" \"C:\\tail\\\\\" \"--profile\""
        );
    }
}
