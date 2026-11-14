//! The `stela` command line makes and maintains the USB stick that boots
//! Stela to install macOS on a PC, and reads its boot logs. Research and
//! development commands belong to `stela-dev`, which uses this library's
//! boot-bundle and restore-image tools.
use clap::Parser;
use std::process::ExitCode;

pub mod apfs;
pub mod args;
pub mod boot;
mod boot_log;
pub mod efi;
pub use stela_gpu_maintenance as gpu;
mod net;
mod progress;
pub mod restore;
mod setup;

pub use setup::InstallMedium;

/// The help's examples, with this computer's spelling of a whole disk.
fn examples() -> String {
    let disk = setup::host::DISK_EXAMPLE;
    format!(
        "Examples:
  stela                 Choose what to do, step by step
  stela update          Update the plugged-in stick
  stela log             Save the boot logs to log.txt
  stela create {disk} --confirm-disk 64.0GB --installer offline"
    )
}

/// The help's layout, with this computer's spelling of a whole disk.
fn help_template() -> String {
    let disk = setup::host::DISK_EXAMPLE;
    format!(
        "{{about}}

{{usage-heading}} {{usage}}

Run stela without a command to see the plugged-in sticks and choose what
to do next.

Commands:
{{subcommands}}

STICK is the stick's whole disk ({disk}) or one of its mounted volumes.
It can be left out when only one stick fits.

Options:
{{options}}{{after-help}}"
    )
}

const ABOUT: &str = "Make and maintain the USB stick that installs macOS on a PC with Stela";

#[derive(Parser)]
#[command(
    name = "stela",
    version,
    about = ABOUT,
    override_usage = "stela [COMMAND [STICK]] [OPTIONS]",
    help_template = help_template(),
    after_help = examples(),
)]
pub(crate) struct Cli {
    #[command(subcommand)]
    command: Option<setup::Command>,
    /// Show what would change, and change nothing
    #[arg(short = 'n', long, global = true)]
    dry_run: bool,
    /// Accept questions without a terminal
    #[arg(short = 'y', long, global = true)]
    yes: bool,
}

impl Cli {
    /// The request the command line names: without a command, the home
    /// screen, which looks at the sticks and offers the next step.
    fn request(self) -> Result<setup::Request, clap::Error> {
        let global = setup::Global {
            dry_run: self.dry_run,
            yes: self.yes,
        };
        match self.command {
            Some(command) => command.request(global),
            None => Ok(setup::Request::home(global)),
        }
    }
}

/// Parse the command line and run it, reporting errors on stderr.
/// What a command line of 0.3 or older is told: such a line runs nothing,
/// so this is not an alias.
fn replaced(words: &[std::ffi::OsString]) -> Option<String> {
    let words: Vec<_> = words.iter().map(|word| word.to_string_lossy()).collect();
    if let Some(first) = words.iter().find(|word| !word.starts_with('-'))
        && ["setup", "boot-log", "configure"].contains(&first.as_ref())
    {
        return Some(format!(
            "stela {first} was replaced in 0.4: run stela, or see stela --help"
        ));
    }
    words
        .iter()
        .any(|word| *word == "--usb" || word.starts_with("--usb="))
        .then(|| {
            "--usb was replaced in 0.4: name the stick after the command, or see stela --help"
                .into()
        })
}

pub fn main() -> ExitCode {
    let words: Vec<_> = std::env::args_os().skip(1).collect();
    if let Some(text) = replaced(&words) {
        eprintln!("{text}");
        return ExitCode::from(2);
    }
    let result = Cli::parse()
        .request()
        .unwrap_or_else(|error| error.exit())
        .run();
    let code = match result {
        Ok(()) => ExitCode::SUCCESS,
        // The report named what is due.
        Err(error) if error.is::<setup::home::Due>() => ExitCode::FAILURE,
        Err(error) => {
            eprintln!(
                "{} {error:#}",
                console::style("error:").for_stderr().red().bold()
            );
            ExitCode::FAILURE
        }
    };
    setup::hold_window(code == ExitCode::SUCCESS);
    code
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn every_help_line_fits_80_columns() {
        let mut cli = Cli::command();
        cli.build();
        let mut pages = vec![
            cli.render_help().to_string(),
            cli.render_long_help().to_string(),
        ];
        for command in cli.get_subcommands_mut() {
            if !command.is_hide_set() {
                pages.push(command.render_help().to_string());
                pages.push(command.render_long_help().to_string());
            }
        }
        for page in pages {
            for line in page.lines() {
                assert!(line.chars().count() <= 80, "{line:?} in:\n{page}");
            }
        }
    }

    #[test]
    fn the_help_example_confirms_the_stick_it_names_as_stela_shows_it() {
        let examples = examples();
        let typed = examples
            .split("--confirm-disk ")
            .nth(1)
            .and_then(|rest| rest.split_whitespace().next())
            .unwrap();
        let bytes = 64_000_000_000;
        assert_eq!(typed, setup::host::size_text(bytes).replace(' ', ""));
        assert!(setup::host::names_disk("Flash Drive", bytes, typed));
    }

    #[test]
    fn a_0_3_command_line_is_named_and_runs_nothing() {
        let said = |words: &[&str]| {
            replaced(
                &words
                    .iter()
                    .map(std::ffi::OsString::from)
                    .collect::<Vec<_>>(),
            )
        };
        for words in [
            &["setup"][..],
            &["-n", "setup", "update", "stela"],
            &["boot-log", "--output", "log.txt"],
            &["configure", "--wifi", "Home"],
        ] {
            let text = said(words).unwrap();
            assert!(text.ends_with("was replaced in 0.4: run stela, or see stela --help"));
        }
        assert!(
            said(&["status", "--usb", "/Volumes/STELA"])
                .unwrap()
                .starts_with("--usb")
        );
        assert!(said(&["status", "--usb=/Volumes/STELA"]).is_some());
        for words in [
            &[][..],
            &["status"],
            &["log", "setup"],
            &["settings", "--usb4-allow", "none"],
        ] {
            assert_eq!(said(words), None, "{words:?}");
        }
    }

    pub(crate) fn parse(words: &[&str]) -> Result<setup::Request, clap::Error> {
        Cli::try_parse_from(std::iter::once("stela").chain(words.iter().copied()))?.request()
    }

    #[test]
    fn help_lists_the_six_commands_and_hides_the_rest() {
        let help = Cli::command().render_help().to_string();
        for command in ["create", "update", "repair", "status", "settings", "log"] {
            assert!(
                help.contains(&format!("\n  {command} ")),
                "{command}: {help}"
            );
        }
        for hidden in [
            "uninstall",
            "nvram",
            "state",
            "host-firmware",
            "setup",
            "boot-log",
        ] {
            assert!(!help.contains(hidden), "{hidden}: {help}");
        }
        assert!(
            help.contains("-n, --dry-run") && help.contains("-y, --yes"),
            "{help}"
        );
        assert!(help.starts_with(ABOUT), "{help}");
        Cli::command().debug_assert();
    }

    #[test]
    fn old_spellings_are_refused() {
        // 0.4 drops the `setup` word and the commands and flags it replaced.
        for words in [
            &["setup"][..],
            &["setup", "status"],
            &["boot-log"],
            &["configure"],
            &["status", "--usb", "/Volumes/STELA"],
            &["repair", "--fetch"],
            &["repair", "--verify"],
            &["repair", "--build", "26B123"],
            &["update", "--build", "26B123"],
            &["create", "--build", "26B123"],
            &["settings", "--no-wifi"],
            &["settings", "--no-dns"],
            &["settings", "--no-ipv4"],
            &["settings", "--automatic-dns"],
            &["settings", "--no-usb4-approve"],
            &[
                "settings",
                "--usb4-approve",
                "00000000-0000-0000-0000-000000000000",
            ],
            &["host-firmware", "fetch", "--gpu", "ga104"],
            &["compile-block"],
            &["gpu"],
        ] {
            assert!(parse(words).is_err(), "{words:?}");
        }
    }

    #[test]
    fn each_command_takes_only_its_own_options() {
        for words in [
            &[][..],
            &["-n"],
            &["--yes"],
            &["status"],
            &["status", "/dev/sdb", "--full"],
            &["repair", "--ipsw", "a.ipsw", "--full"],
            &["repair", "--erase-target", "--initialize-nor", "-n"],
            &["update", "/Volumes/STELA", "--macos", "27.2"],
            &["update", "--efi", "stela.efi"],
            &[
                "create",
                "/dev/sdb",
                "--installer",
                "online",
                "--wifi",
                "none",
            ],
            &[
                "create",
                "--ipsw",
                "a.ipsw",
                "--profile",
                "--macos",
                "latest",
            ],
            &[
                "settings", "--wifi", "Home", "--ipv4", "auto", "--dns", "auto",
            ],
            &["settings", "--usb4-allow", "none", "--chooser-timeout", "0"],
            &["settings", "--profile", "off"],
            &["log"],
            &["log", "disk.img", "--offset", "0", "--in-target"],
            &["log", "-o", "report.txt"],
            &["uninstall", "/Volumes/STELA"],
            &["nvram", "next-boot", "recovery", "--install", "macOS"],
            &["nvram", "reset", "--nor", "sdc3"],
            &["state", "backup", "state.bin"],
            &[
                "state",
                "restore",
                "state.bin",
                "/Volumes/STELA",
                "--nor",
                "sdc3",
            ],
            &["host-firmware", "package"],
        ] {
            parse(words).unwrap_or_else(|error| panic!("{words:?}: {error}"));
        }
        for words in [
            &["status", "--target", "sdc2"][..],
            &["status", "--ipsw", "a.ipsw"],
            &["uninstall", "--ipsw", "a.ipsw"],
            &["settings", "--installer", "online"],
            &["update", "--full"],
            &["log", "--partition", "x"],
            &["create", "--gateway", "192.168.1.1"],
            &[
                "settings",
                "--usb4-allow",
                "none",
                "--usb4-allow",
                "00000000-0000-0000-0000-000000000001",
            ],
            &["create", "--macos", "27/2"],
            &["doctor"],
        ] {
            assert!(parse(words).is_err(), "{words:?}");
        }
    }

    #[test]
    fn global_options_reach_every_command() {
        let request = parse(&["repair", "-n", "--yes"]).unwrap();
        assert!(request.dry_run && request.yes);
        let request = parse(&["--dry-run", "status"]).unwrap();
        assert!(request.dry_run && !request.yes);
        let request = parse(&["-y"]).unwrap();
        assert!(request.yes);
    }

    #[test]
    fn macos_takes_a_version_a_build_or_latest() {
        let build = |words: &[&str]| parse(words).unwrap().build;
        let newest = stela_formats::builds::DEFAULT;
        assert_eq!(build(&["create"]), newest.build);
        assert_eq!(build(&["create", "--macos", "latest"]), newest.build);
        assert_eq!(build(&["create", "--macos", newest.version]), newest.build);
        assert_eq!(build(&["create", "--macos", "26a434"]), "26A434");
    }
}
