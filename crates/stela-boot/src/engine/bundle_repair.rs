//! The engine's part of the automatic Metal plugin repair
//! (`stela_boot_core::bundle_repair`): what this boot is, what NVRAM's
//! record said when the machine state opened, what the kext's report decides,
//! the record's change written at the stop, and the console driver of a
//! repair boot.
use super::output;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use stela_boot_core::bundle_repair::{self, Boot, Change, Driver, Record};
#[cfg(feature = "nvk")]
use stela_boot_core::bundle_repair::{Decision, Report};
use stela_soc_data::nor::{Image, nvram::Variables};

static BOOT: AtomicU8 = AtomicU8::new(0);
static RECORD: AtomicU8 = AtomicU8::new(0);
/// The machine state is open, so the stop can write NVRAM.
static PERSISTENT: AtomicBool = AtomicBool::new(false);
static CHANGE: AtomicU8 = AtomicU8::new(0);

const BOOTS: [Boot; 3] = [Boot::Normal, Boot::SingleUser, Boot::Repair];
const RECORDS: [Record; 7] = [
    Record::None,
    Record::Pending,
    Record::Attempted,
    Record::Installed,
    Record::Retry,
    Record::Retried,
    Record::Other,
];

/// Record which boot this is, before the guest starts.
pub(super) fn boot(boot: Boot) {
    let index = BOOTS.iter().position(|b| *b == boot).unwrap_or(0);
    BOOT.store(index as u8, Ordering::Relaxed);
}

fn this_boot() -> Boot {
    BOOTS[usize::from(BOOT.load(Ordering::Relaxed)) % BOOTS.len()]
}

/// Read the record from the bank the machine state opened with.
pub(super) fn nvram(bank: &[u8]) {
    let record = match Variables::parse(bank) {
        Ok(variables) => Record::read(&variables),
        Err(error) => {
            output::arguments(format_args!(
                "BUNDLE REPAIR NVRAM unreadable ({error:?}); no repair can be recorded\n"
            ));
            return;
        }
    };
    let index = RECORDS.iter().position(|r| *r == record).unwrap_or(0);
    RECORD.store(index as u8, Ordering::Relaxed);
    PERSISTENT.store(true, Ordering::Relaxed);
    if record != Record::None {
        output::arguments(format_args!("BUNDLE REPAIR NVRAM record {record:?}\n"));
    }
}

fn this_record() -> Record {
    RECORDS[usize::from(RECORD.load(Ordering::Relaxed)) % RECORDS.len()]
}

/// Keep `change` for the stop.
fn keep(change: Change) {
    CHANGE.store(1 + change as u8, Ordering::Relaxed);
}

/// What the kext's `report` decides for this boot; a change to NVRAM's
/// record is kept for the stop. Only the GPU gate reports.
#[cfg(feature = "nvk")]
pub(super) fn decide(report: Report) -> Decision {
    let record = this_record();
    let decision = bundle_repair::decide(
        report,
        this_boot(),
        record,
        PERSISTENT.load(Ordering::Relaxed),
    );
    let change = match decision {
        Decision::Repair => Some(Change::Request),
        Decision::Continue(change) => change,
        _ => None,
    };
    if let Some(change) = change {
        keep(change);
    }
    decision
}

/// This repair boot's shell never took input
/// (`bundle_repair::Failure::unanswered`): the stop asks for one more
/// maintenance boot, unless this was it. Whether it does.
pub(super) fn unanswered() -> bool {
    if !PERSISTENT.load(Ordering::Relaxed) {
        return false;
    }
    let Some(change) = bundle_repair::retry(this_record()) else {
        output::arguments(format_args!(
            "BUNDLE REPAIR nothing ran in the second attempt either; stela repair lets it run again\n"
        ));
        return false;
    };
    keep(change);
    true
}

/// An erase install finished: its stop asks the first macOS boot to be the
/// repair, which a fresh Data volume always needs.
pub(super) fn installed() {
    if PERSISTENT.load(Ordering::Relaxed) {
        keep(Change::Installed);
    }
}

/// Whether the stop has a change to store.
pub(super) fn change_pending() -> bool {
    CHANGE.load(Ordering::Relaxed) != 0
}

/// Apply the kept change to `image`'s boot bank at the stop, the guest no
/// longer running: the sectors to store, or none when nothing changes.
pub(super) fn at_stop(image: &mut Image) -> Vec<(usize, Vec<u8>)> {
    let Some(change) = usize::from(CHANGE.swap(0, Ordering::Relaxed))
        .checked_sub(1)
        .and_then(|number| Change::ALL.get(number).copied())
    else {
        return Vec::new();
    };
    let result = (|| -> Result<Vec<(usize, Vec<u8>)>, alloc::string::String> {
        let (_, proxy) = image.boot_bank().map_err(|e| alloc::format!("{e:?}"))?;
        let mut variables = Variables::parse(&proxy).map_err(|e| alloc::format!("{e:?}"))?;
        if !change.apply(&mut variables)? {
            return Ok(Vec::new());
        }
        let bank = variables.encode().map_err(|e| alloc::format!("{e:?}"))?;
        let (_, sectors) = image
            .save_bank(&bank)
            .map_err(|e| alloc::format!("{e:?}"))?;
        let size = stela_soc_data::nor::SECTOR_SIZE;
        Ok(sectors
            .into_iter()
            .map(|offset| (offset, image.bytes()[offset..offset + size].to_vec()))
            .collect())
    })();
    match result {
        Ok(sectors) => {
            if !sectors.is_empty() {
                output::arguments(format_args!("BUNDLE REPAIR NVRAM {change:?} stored\n"));
            }
            sectors
        }
        Err(error) => {
            output::arguments(format_args!(
                "BUNDLE REPAIR NVRAM {change:?} not stored: {error}\n"
            ));
            Vec::new()
        }
    }
}

/// The console driver of a repair boot, started with the guest.
pub(super) fn driver(now_millis: u64) -> Option<Driver> {
    (this_boot() == Boot::Repair).then(|| {
        output::arguments(format_args!(
            "BUNDLE REPAIR maintenance boot: single-user, filesystem protection off\n"
        ));
        Driver::new(bundle_repair::steps(), now_millis)
    })
}
