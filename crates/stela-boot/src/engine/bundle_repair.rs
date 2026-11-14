//! The engine's part of the automatic Metal plugin repair
//! (`stela_boot_core::bundle_repair`): what this boot is, what NVRAM's
//! record said when the machine state opened, what the kext's report decides,
//! the record's change written at the stop, the console driver of a
//! repair boot and what it reported, which its stop records for a later
//! stop to name.
use super::{nor::HostNor, output};
use alloc::{string::String, vec::Vec};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use stela_boot_core::bundle_repair::{self, Boot, Change, Driver, Outcome, Record};
#[cfg(feature = "nvk")]
use stela_boot_core::bundle_repair::{Decision, Report};
use stela_formats::boot_bundle::Nvram;
use stela_soc_data::nor::{Image, nvram::Variables};

static BOOT: AtomicU8 = AtomicU8::new(0);
static RECORD: AtomicU8 = AtomicU8::new(0);
/// The machine state is open, so the stop can write NVRAM.
static PERSISTENT: AtomicBool = AtomicBool::new(false);
static CHANGE: AtomicU8 = AtomicU8::new(0);
/// This repair boot's outcome (`Outcome::encode`), 0 while none is known.
static OUTCOME: AtomicU64 = AtomicU64::new(0);
/// What the last repair boot reported, as NVRAM held it when the machine
/// state opened.
static REPORTED: output::Global<Option<String>> = output::Global(RefCell::new(None));

const BOOTS: [Boot; 3] = [Boot::Normal, Boot::SingleUser, Boot::Repair];
const RECORDS: [Record; 8] = [
    Record::None,
    Record::Pending,
    Record::Attempted,
    Record::Installed,
    Record::Retry,
    Record::Retried,
    Record::Outdated,
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
    let variables = match Variables::parse(bank) {
        Ok(variables) => variables,
        Err(error) => {
            output::arguments(format_args!(
                "BUNDLE REPAIR NVRAM unreadable ({error:?}); no repair can be recorded\n"
            ));
            return;
        }
    };
    let record = Record::read(&variables);
    let index = RECORDS.iter().position(|r| *r == record).unwrap_or(0);
    RECORD.store(index as u8, Ordering::Relaxed);
    PERSISTENT.store(true, Ordering::Relaxed);
    if record != Record::None {
        output::arguments(format_args!("BUNDLE REPAIR NVRAM record {record:?}\n"));
    }
    let reported = bundle_repair::result(&variables);
    if let Some(reported) = &reported {
        output::arguments(format_args!(
            "BUNDLE REPAIR NVRAM result of the last repair: {reported}\n"
        ));
    }
    if let Ok(mut kept) = REPORTED.0.try_borrow_mut() {
        *kept = reported;
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

/// The stop's reason for a normal boot that finds the plugin missing
/// (`Decision::StopMissing`): what the last repair reported, or why none
/// can run.
#[cfg(feature = "nvk")]
pub(super) fn missing(attempted: bool) -> String {
    let reported = REPORTED.0.try_borrow().ok().and_then(|kept| kept.clone());
    bundle_repair::stop_reason(
        attempted,
        this_record(),
        PERSISTENT.load(Ordering::Relaxed),
        reported.as_deref(),
    )
}

/// How this repair boot went, as its console driver knows it: kept for the
/// stop, which records it, whether the guest or the engine restarts.
pub(super) fn outcome(outcome: &Outcome) {
    OUTCOME.store(outcome.encode(), Ordering::Relaxed);
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
        || (PERSISTENT.load(Ordering::Relaxed) && OUTCOME.load(Ordering::Relaxed) != 0)
}

/// Save `edit`'s change to `image`'s boot bank into its next bank: the
/// sectors to store, or none when nothing changes.
fn save(
    image: &mut Image,
    edit: impl FnOnce(&mut Variables) -> Result<bool, String>,
) -> Result<Vec<(usize, Vec<u8>)>, String> {
    let (_, proxy) = image.boot_bank().map_err(|e| alloc::format!("{e:?}"))?;
    let mut variables = Variables::parse(&proxy).map_err(|e| alloc::format!("{e:?}"))?;
    if !edit(&mut variables)? {
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
}

/// A repair boot whose machine state opened with the request still in it
/// (`bundle_repair::untaken`): the loader's take was not saved, so the
/// engine stores it now, before the guest starts, waiting as long as the
/// stop does, and hands the guest the bank with it (`nvram`). A take that
/// cannot be stored stops the boot with its reason: the request would
/// make every start a repair boot.
pub(super) fn take(nor: &mut HostNor, image: &mut Image, nvram: &mut Nvram) {
    let record = this_record();
    if !bundle_repair::untaken(this_boot(), record) {
        return;
    }
    output::arguments(format_args!(
        "BUNDLE REPAIR NVRAM still holds the request ({record:?}) this repair boot took; \
         storing the take\n"
    ));
    let sectors = save(image, bundle_repair::take)
        .unwrap_or_else(|error| output::fatal(bundle_repair::untaken_reason(record, &error)));
    nor.store(sectors);
    if let Err(why) = nor.finish() {
        output::fatal(bundle_repair::untaken_reason(record, &why));
    }
    let Ok((bank, proxy)) = image.boot_bank() else {
        output::fatal(bundle_repair::untaken_reason(
            record,
            "the stored image has no boot bank",
        ));
    };
    let taken = Variables::parse(&proxy)
        .map(|variables| Record::read(&variables))
        .unwrap_or(Record::Other);
    let index = RECORDS.iter().position(|r| *r == taken).unwrap_or(0);
    RECORD.store(index as u8, Ordering::Relaxed);
    *nvram = Nvram { bank, proxy };
    output::arguments(format_args!(
        "BUNDLE REPAIR NVRAM take stored; record {taken:?}\n"
    ));
}

/// The change the stop's store carries ([`at_stop`]), until [`stored`]
/// learns whether it is durable.
static STORING: output::Global<Option<String>> = output::Global(RefCell::new(None));
/// The restart's refusal: a change to the record that is not durable.
static UNSTORED: output::Global<Option<String>> = output::Global(RefCell::new(None));

fn refuse_restart(what: &str, why: &str) {
    output::arguments(format_args!(
        "BUNDLE REPAIR NVRAM {what} not stored: {why}\n"
    ));
    if let Ok(mut unstored) = UNSTORED.0.try_borrow_mut() {
        *unstored = Some(bundle_repair::unstored_reason(what, why));
    }
}

/// Apply the kept change to `image`'s boot bank at the stop, the guest no
/// longer running: the sectors to store, or none when nothing changes.
/// Whether they are stored is known only once the store is durable
/// ([`stored`]).
pub(super) fn at_stop(image: &mut Image) -> Vec<(usize, Vec<u8>)> {
    let change = usize::from(CHANGE.swap(0, Ordering::Relaxed))
        .checked_sub(1)
        .and_then(|number| Change::ALL.get(number).copied());
    let reported = Outcome::decode(OUTCOME.swap(0, Ordering::Relaxed))
        .filter(|_| PERSISTENT.load(Ordering::Relaxed))
        .map(|outcome| outcome.text(&bundle_repair::steps()));
    if change.is_none() && reported.is_none() {
        return Vec::new();
    }
    let what = match (change, &reported) {
        (Some(change), Some(_)) => alloc::format!("{change:?} and the repair's result"),
        (Some(change), None) => alloc::format!("{change:?}"),
        (None, _) => "the repair's result".into(),
    };
    let result = save(image, |variables| {
        let mut changed = false;
        if let Some(change) = change {
            changed |= change.apply(variables)?;
        }
        if let Some(reported) = &reported {
            changed |= bundle_repair::store_result(variables, reported)?;
        }
        Ok(changed)
    });
    match result {
        Ok(sectors) => {
            if !sectors.is_empty()
                && let Ok(mut storing) = STORING.0.try_borrow_mut()
            {
                *storing = Some(what);
            }
            sectors
        }
        Err(error) => {
            refuse_restart(&what, &error);
            Vec::new()
        }
    }
}

/// The stop's NOR stores ended: `Ok` once every one is durable, else why
/// not. The change [`at_stop`] stored is logged as stored only now; one
/// that is not durable refuses the restart ([`restart_refusal`]).
pub(super) fn stored(durable: &Result<(), String>) {
    let Some(what) = STORING.0.try_borrow_mut().ok().and_then(|mut s| s.take()) else {
        return;
    };
    match durable {
        Ok(()) => output::arguments(format_args!("BUNDLE REPAIR NVRAM {what} stored\n")),
        Err(why) => refuse_restart(&what, why),
    }
}

/// Why the PC must not restart: the stop could not store a change to the
/// repair's record, and the next start, not knowing it, could repeat this
/// boot at every restart. `None` when nothing was lost.
pub(super) fn restart_refusal() -> Option<String> {
    UNSTORED
        .0
        .try_borrow()
        .ok()
        .and_then(|unstored| unstored.clone())
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
