//! The macOS builds the native mount admits.
//! Device models implement each tested build's AP-to-coprocessor protocols.
//! Every other build of the same macOS major version ([`MAJOR`]) is admitted
//! with the protocol values of the nearest tested build at or below it
//! ([`find`]); [`tested`] tells the two apart so callers can say so. Setup and
//! boot-bundle preparation reject builds of other macOS versions with
//! [`Unsupported`].
use alloc::{string::String, vec::Vec};
use core::fmt;

/// One supported build and the protocol values its firmware fixes.
#[derive(Debug, PartialEq, Eq)]
pub struct Build {
    /// The build, such as `26A434`.
    pub build: &'static str,
    /// Its product version, such as `27.0.1`.
    pub version: &'static str,
    /// The interface version of the DCP link library, which the
    /// coprocessor announces and the AP hashes into the interface CRC
    /// (`platform::apple::dcp::link`).
    pub dcp_link_version: u32,
    /// The RTKit management protocol version every RTKit coprocessor offers
    /// in its HELLO and the AP's RTBuddy accepts
    /// (`platform::apple::rtkit`).
    pub rtkit_version: u64,
}

/// Every supported build, newest first.
pub const SUPPORTED: &[Build] = &[
    Build {
        build: "26B123",
        version: "27.2",
        // IOMobileGraphicsFamily-DCP's `link_version__`, at the 26B123
        // Mac14,3 kernelcache's file offset 0x5c06768.
        dcp_link_version: 213,
        rtkit_version: 12,
    },
    Build {
        build: "26A434",
        version: "27.0.1",
        // The same global, at the 26A434 kernelcache's file offset 0x5bd9bd8.
        dcp_link_version: 211,
        rtkit_version: 12,
    },
];

/// The native firmware build setup selects when none is named: the newest
/// entry of [`SUPPORTED`] whose GFX firmware interface the GPU model has
/// checked.
///
/// This is not simply the newest entry. 26B123's G14G kext writes a later
/// initdata revision (`stela_soc_data::agx::generation::UNCHECKED_INITDATA`)
/// that the GFX model refuses, so a guest that boots it stalls at the
/// installer's "load sep os" stage with repeated GFX firmware DMA refusals.
/// 26B123 stays selectable by name (`--macos 26B123`) for work on that
/// revision. Once its layouts are checked, remove it from
/// `UNCHECKED_INITDATA` and point this back at it.
pub const DEFAULT: &Build = &SUPPORTED[1];

/// Builds of [`SUPPORTED`] that may be selected by name but whose GFX
/// firmware interface the GPU model has not checked, so new installs do not
/// pick them by default. Kept in step with
/// `stela_soc_data::agx::generation::UNCHECKED_INITDATA`.
pub const GPU_UNCHECKED: &[&str] = &["26B123"];

/// `build` is a tested build whose GPU interface the model has checked.
pub fn gpu_checked(build: &str) -> bool {
    tested(build) && !GPU_UNCHECKED.contains(&build)
}

/// The macOS major version Stela supports: every build of it is admitted.
pub const MAJOR: u32 = 27;

/// The major version of a product version such as `27.0.1`.
pub fn version_major(version: &str) -> Option<u32> {
    version.split('.').next()?.trim().parse().ok()
}

/// An Apple build such as `26B123` in order: its Darwin major (the leading
/// digits), its train letter, its number and any suffix letters.
fn order(build: &str) -> Option<(u32, u8, u32, &str)> {
    let digits = build.find(|c: char| !c.is_ascii_digit())?;
    let darwin = build[..digits].parse().ok()?;
    let train = *build.as_bytes().get(digits)?;
    if !train.is_ascii_uppercase() {
        return None;
    }
    let rest = &build[digits + 1..];
    let number_end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    let number = rest[..number_end].parse().ok()?;
    let suffix = &rest[number_end..];
    suffix
        .bytes()
        .all(|b| b.is_ascii_lowercase())
        .then_some((darwin, train, number, suffix))
}

/// The macOS major version of an Apple build such as `26B123`: Darwin 20
/// to 24 are macOS 11 to 15, and Darwin 25 onward macOS 26 onward.
pub fn build_major(build: &str) -> Option<u32> {
    let (darwin, ..) = order(build)?;
    match darwin {
        20..=24 => Some(darwin - 9),
        25.. => Some(darwin + 1),
        _ => None,
    }
}

/// `build` is one of the tested builds of [`SUPPORTED`].
pub fn tested(build: &str) -> bool {
    SUPPORTED.iter().any(|entry| entry.build == build)
}

/// The entry of `table` whose protocol values `build` uses: its own, or
/// for another build of a macOS major version `table` holds, the nearest
/// tested build at or below it (the oldest one when it is older than all).
pub fn find_in<'t>(table: &'t [Build], build: &str) -> Result<&'t Build, Unsupported> {
    if let Some(entry) = table.iter().find(|entry| entry.build == build) {
        return Ok(entry);
    }
    let unsupported = || Unsupported {
        build: build.into(),
        majors: {
            let mut majors: Vec<u32> = table
                .iter()
                .filter_map(|entry| version_major(entry.version))
                .collect();
            majors.dedup();
            majors
        },
        tested: table.iter().map(|entry| entry.build).collect(),
    };
    let (Some(major), Some(key)) = (build_major(build), order(build)) else {
        return Err(unsupported());
    };
    let mut same: Vec<&Build> = table
        .iter()
        .filter(|entry| version_major(entry.version) == Some(major))
        .filter(|entry| order(entry.build).is_some())
        .collect();
    same.sort_by_key(|entry| order(entry.build));
    same.iter()
        .rev()
        .find(|entry| order(entry.build) <= Some(key))
        .or(same.first())
        .copied()
        .ok_or_else(unsupported)
}

/// `build`'s entry of [`SUPPORTED`], as [`find_in`] chooses it.
pub fn find(build: &str) -> Result<&'static Build, Unsupported> {
    find_in(SUPPORTED, build)
}

/// A build of a macOS version the table does not hold.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Unsupported {
    pub build: String,
    /// The macOS major versions the table holds.
    pub majors: Vec<u32>,
    /// The tested builds.
    pub tested: Vec<&'static str>,
}

impl fmt::Display for Unsupported {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "macOS build {} is not supported", self.build)?;
        if let Some(major) = build_major(&self.build) {
            write!(f, " (macOS {major})")?;
        }
        let majors: Vec<String> = self.majors.iter().map(|m| alloc::format!("{m}")).collect();
        write!(
            f,
            "; Stela supports macOS {} (tested: {})",
            majors.join(", "),
            self.tested.join(", ")
        )
    }
}

impl core::error::Error for Unsupported {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tested_builds_keep_their_own_protocol_values() {
        assert_eq!(find("26A434").unwrap().dcp_link_version, 211);
        assert_eq!(find("26A434").unwrap().rtkit_version, 12);
        assert_eq!(find("26B123").unwrap().dcp_link_version, 213);
        assert_eq!(DEFAULT.build, "26A434");
        assert!(tested("26B123") && tested("26A434") && !tested("26B5040"));
        for entry in SUPPORTED {
            assert_eq!(version_major(entry.version), Some(MAJOR), "{}", entry.build);
            assert_eq!(build_major(entry.build), Some(MAJOR), "{}", entry.build);
        }
    }

    #[test]
    fn the_default_is_the_newest_build_whose_gpu_interface_is_checked() {
        assert!(gpu_checked(DEFAULT.build));
        assert!(!gpu_checked("26B123"));
        let newest_checked = SUPPORTED
            .iter()
            .find(|entry| gpu_checked(entry.build))
            .unwrap();
        assert_eq!(DEFAULT, newest_checked);
        for build in GPU_UNCHECKED {
            assert!(tested(build), "{build}");
        }
    }

    #[test]
    fn every_macos_27_build_is_admitted_with_the_nearest_tested_values() {
        // Older than every tested build: the oldest tested one.
        assert_eq!(find("26A5").unwrap().build, "26A434");
        assert_eq!(find("26A400").unwrap().build, "26A434");
        // Between them, betas included: the tested one at or below.
        assert_eq!(find("26A440a").unwrap().build, "26A434");
        assert_eq!(find("26B50").unwrap().build, "26A434");
        assert_eq!(find("26B5040a").unwrap().build, "26B123");
        assert_eq!(find("26B123a").unwrap().build, "26B123");
        // Newer than every tested build: the newest.
        assert_eq!(find("26C55").unwrap().build, "26B123");
        assert_eq!(find("26E1000").unwrap().build, "26B123");
    }

    #[test]
    fn builds_of_other_macos_versions_are_refused_naming_macos_27() {
        assert_eq!(build_major("25A354"), Some(26));
        assert_eq!(build_major("24G84"), Some(15));
        assert_eq!(build_major("20A2411"), Some(11));
        assert_eq!(build_major("27A100"), Some(28));
        for build in [
            "25A354", "25G5", "27A100", "24G84", "STELA1", "26", "26b1", "",
        ] {
            assert!(find(build).is_err(), "{build}");
        }
        assert_eq!(
            find("25A354").unwrap_err().to_string(),
            "macOS build 25A354 is not supported (macOS 26); Stela supports macOS 27 \
             (tested: 26B123, 26A434)"
        );
        assert_eq!(
            find("27A100").unwrap_err().to_string(),
            "macOS build 27A100 is not supported (macOS 28); Stela supports macOS 27 \
             (tested: 26B123, 26A434)"
        );
    }
}
