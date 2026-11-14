//! A stop the USB's boot log cannot keep leaves its last lines in the PC's
//! firmware (`stela_boot_core::stop_record`): [`keep`] writes them with the
//! firmware's variable services, which still run after its boot services
//! have ended, and [`carry`] copies them into the log at the next start
//! and deletes them.
//!
//! [`keep`] runs once a boot, on the first processor that stops, and only
//! from a stop: a boot that does not stop writes no variable. The loader
//! calls the services where the firmware left them. The engine reaches
//! them in the loader's root, which keeps the firmware's runtime regions
//! mapped by identity, as `SetVirtualAddressMap` is never called.
use crate::output;
use alloc::{boxed::Box, string::String};
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicU8, AtomicU64, Ordering},
};
use stela_boot_core::stop_record::{
    ATTRIBUTES, HEADER_NAME, Header, Kept, PIECES, Space, Tail, VENDOR, carried_notice, fitting,
    piece_name, pieces, read, stop_reason,
};
use uefi::{
    CStr16, Guid,
    runtime::{self, VariableVendor},
    table::Revision,
};

/// `EFI_RUNTIME_SERVICES.SetVariable` (UEFI 2.10 section 8.2.3).
type SetVariable =
    unsafe extern "efiapi" fn(*const u16, *const Guid, u32, usize, *const u8) -> usize;
/// `EFI_RUNTIME_SERVICES.QueryVariableInfo` (section 8.2.4).
type QueryVariableInfo = unsafe extern "efiapi" fn(u32, *mut u64, *mut u64, *mut u64) -> usize;
/// `EFI_SUCCESS`.
const SUCCESS: usize = 0;

/// What a stop writes its record with, which the loader hands the engine.
#[derive(Clone, Copy, Default)]
#[repr(C)]
pub(crate) struct Services {
    /// The firmware's `SetVariable` and `QueryVariableInfo` entries; zero
    /// without both.
    set_variable: u64,
    query_variable_info: u64,
    /// This boot's number in the USB's log; 0 without one.
    boot: u64,
    /// One more than the firmware entry this start came from
    /// (`BootCurrent`, section 3.3); 0 where the firmware names none.
    entry: u64,
}

impl Services {
    /// The firmware's, read while its boot services run. Firmware older
    /// than UEFI 2.0 has no `QueryVariableInfo`, and gets no record.
    pub(crate) fn capture() -> Self {
        let services = Self::capture_without_heap();
        if services.set_variable == 0 {
            return services;
        }
        let entry = runtime::get_variable_boxed(
            uefi::cstr16!("BootCurrent"),
            &VariableVendor::GLOBAL_VARIABLE,
        )
        .ok()
        .and_then(|(value, _)| <[u8; 2]>::try_from(&*value).ok())
        .map_or(0, |value| u64::from(u16::from_le_bytes(value)) + 1);
        Self { entry, ..services }
    }

    /// [`Self::capture`] without the firmware entry this start came from,
    /// whose read needs the heap: for a stop before the heap exists.
    pub(crate) fn capture_without_heap() -> Self {
        let Some(table) = uefi::table::system_table_raw() else {
            return Self::default();
        };
        // SAFETY: the firmware's live system table, before boot services end.
        let runtime = unsafe { table.as_ref().runtime_services };
        if runtime.is_null() {
            return Self::default();
        }
        // SAFETY: as above; its runtime services table.
        let runtime = unsafe { &*runtime };
        if runtime.header.revision < Revision::EFI_2_00 {
            return Self::default();
        }
        Self {
            set_variable: runtime.set_variable as usize as u64,
            query_variable_info: runtime.query_variable_info as usize as u64,
            boot: 0,
            entry: 0,
        }
    }
}

static SET_VARIABLE: AtomicU64 = AtomicU64::new(0);
static QUERY_VARIABLE_INFO: AtomicU64 = AtomicU64::new(0);
static BOOT: AtomicU64 = AtomicU64::new(0);
static ENTRY: AtomicU64 = AtomicU64::new(0);
/// The root the firmware's services are called in; zero for the tables
/// the processor has, the firmware's own.
static ROOT: AtomicU64 = AtomicU64::new(0);

/// Give a later stop the firmware's services, called in `root`: `None`
/// while this image runs on the firmware's tables. Until then, and with
/// services the firmware lacks, a stop writes nothing.
pub(crate) fn arm(services: Services, root: Option<u64>) {
    ROOT.store(root.unwrap_or(0), Ordering::Relaxed);
    BOOT.store(services.boot, Ordering::Relaxed);
    ENTRY.store(services.entry, Ordering::Relaxed);
    QUERY_VARIABLE_INFO.store(services.query_variable_info, Ordering::Relaxed);
    SET_VARIABLE.store(services.set_variable, Ordering::Release);
}

/// From now on a stop writes nothing: this image is about to run on
/// tables that do not map the firmware's runtime regions.
pub(crate) fn disarm() {
    SET_VARIABLE.store(0, Ordering::Release);
}

/// This boot's number in the USB's log, once it is open.
pub(crate) fn set_boot(boot: u64) {
    BOOT.store(boot, Ordering::Relaxed);
}

/// What [`arm`] was given, with the boot's number, for the engine.
pub(crate) fn services() -> Services {
    Services {
        set_variable: SET_VARIABLE.load(Ordering::Acquire),
        query_variable_info: QUERY_VARIABLE_INFO.load(Ordering::Relaxed),
        boot: BOOT.load(Ordering::Relaxed),
        entry: ENTRY.load(Ordering::Relaxed),
    }
}

/// Ask the firmware to start the entry this start came from at the next
/// start (`BootNext`), for a restart macOS asked for: the firmware's own
/// order may put another system first, and a restart must come back here.
/// The entry's number where the firmware took it; nothing is asked where the
/// firmware named no entry or its services are out of reach.
pub(crate) fn return_at_restart() -> Option<u16> {
    /// `EFI_VARIABLE_NON_VOLATILE | BOOTSERVICE_ACCESS | RUNTIME_ACCESS`.
    const BOOT_NEXT_ATTRIBUTES: u32 = 0x7;
    let entry = u16::try_from(ENTRY.load(Ordering::Relaxed).checked_sub(1)?).ok()?;
    let set = SET_VARIABLE.load(Ordering::Acquire) as usize;
    if set == 0 {
        return None;
    }
    // SAFETY: the firmware's entry, as `Services::capture` read it from its
    // runtime services table; code addresses fit a pointer.
    let set = unsafe { core::mem::transmute::<usize, SetVariable>(set) };
    let name = uefi::cstr16!("BootNext");
    let value = entry.to_le_bytes();
    let store = || {
        // SAFETY: a runtime service, in the tables its regions are mapped
        // in; the name, GUID and data outlive the call.
        unsafe {
            set(
                name.as_ptr().cast(),
                &VariableVendor::GLOBAL_VARIABLE.0,
                BOOT_NEXT_ATTRIBUTES,
                value.len(),
                value.as_ptr(),
            )
        }
    };
    let status = match ROOT.load(Ordering::Relaxed) {
        0 => store(),
        // SAFETY: the loader's live root, which `arm` was given with the
        // engine's handoff; the restart resumes nothing here.
        root => unsafe { crate::engine::in_loader_root_at_stop(root, store) },
    };
    (status == SUCCESS).then_some(entry)
}

/// No stop has tried to write the record.
const IDLE: u8 = 0;
/// A processor is writing it, or stopped again inside the write.
const WRITING: u8 = 1;
/// The firmware took the whole record.
const WRITTEN: u8 = 2;
/// Nothing was written, or not all of it.
const SKIPPED: u8 = 3;
static STATE: AtomicU8 = AtomicU8::new(IDLE);

struct Lines(UnsafeCell<Tail>);
// SAFETY: reached only by the one processor that moved `STATE` from `IDLE`.
unsafe impl Sync for Lines {}
/// The record's text, outside the stack a stop runs on.
static LINES: Lines = Lines(UnsafeCell::new(Tail::new()));

/// Write the stop's record, the newest lines `fill` gives, into the
/// firmware's variables; whether the firmware holds it whole. Only the
/// first call writes, on whichever processor makes it: the firmware's
/// services are not called by two processors at once, and a fault inside
/// one comes back here and finds the write begun, so it is not tried
/// again. Later calls say what the first did.
///
/// Nothing is written without the services ([`arm`]), without lines, or
/// where the record would leave less than the reserve of the firmware's store
/// free (`Space::takes`).
pub(crate) fn keep(fill: impl FnOnce(&mut Tail)) -> bool {
    if STATE
        .compare_exchange(IDLE, WRITING, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return STATE.load(Ordering::Acquire) == WRITTEN;
    }
    let written = write(fill);
    STATE.store(if written { WRITTEN } else { SKIPPED }, Ordering::Release);
    written
}

fn write(fill: impl FnOnce(&mut Tail)) -> bool {
    let (set, query) = (
        SET_VARIABLE.load(Ordering::Acquire) as usize,
        QUERY_VARIABLE_INFO.load(Ordering::Relaxed) as usize,
    );
    if set == 0 || query == 0 {
        return false;
    }
    // SAFETY: the firmware's entries, as `Services::capture` read them from
    // its runtime services table; code addresses fit a pointer.
    let (set, query) = unsafe {
        (
            core::mem::transmute::<usize, SetVariable>(set),
            core::mem::transmute::<usize, QueryVariableInfo>(query),
        )
    };
    // SAFETY: this processor alone moved `STATE` from `IDLE`.
    let lines = unsafe { &mut *LINES.0.get() };
    fill(lines);
    let text = lines.text();
    let boot = BOOT.load(Ordering::Relaxed);
    let store = || {
        let mut space = Space {
            maximum: 0,
            remaining: 0,
            largest: 0,
        };
        // SAFETY: a runtime service, in the tables its regions are mapped
        // in; the results are this frame's.
        let asked = unsafe {
            query(
                ATTRIBUTES,
                &mut space.maximum,
                &mut space.remaining,
                &mut space.largest,
            )
        };
        if asked != SUCCESS {
            return false;
        }
        // A store too short for all of it keeps the newest lines that fit.
        let text = fitting(&space, text);
        let Some(header) = Header::of(boot, text) else {
            return false;
        };
        for (index, piece) in pieces(text).enumerate() {
            let name = piece_name(index);
            // SAFETY: as above; the name, GUID and data outlive the call.
            let status = unsafe {
                set(
                    name.as_ptr(),
                    &VENDOR,
                    ATTRIBUTES,
                    piece.len(),
                    piece.as_ptr(),
                )
            };
            if status != SUCCESS {
                return false;
            }
        }
        // Last, so pieces without it, or under an older one, are no record.
        let header = header.encode();
        // SAFETY: as above.
        let status = unsafe {
            set(
                HEADER_NAME.as_ptr(),
                &VENDOR,
                ATTRIBUTES,
                header.len(),
                header.as_ptr(),
            )
        };
        status == SUCCESS
    };
    match ROOT.load(Ordering::Relaxed) {
        0 => store(),
        // SAFETY: the loader's live root, which `arm` was given with the
        // engine's handoff.
        root => unsafe { crate::engine::in_loader_root_at_stop(root, store) },
    }
}

/// Copy the record an earlier start left on this PC into the log, between
/// marker lines, tell the screen that start stopped, why and where its
/// lines are (`carried_notice`), and delete its variables once the USB's
/// log holds the lines. Variables that are no whole record are reported in
/// one line and deleted. Called once the USB's log is open, while boot services run;
/// a PC that holds no record, as one the stick was moved to, gets no line
/// and no write.
pub(crate) fn carry() {
    let vendor = VariableVendor(VENDOR);
    let value = |name: &[u16]| -> Option<Box<[u8]>> {
        let name = CStr16::from_u16_with_nul(name).ok()?;
        runtime::get_variable_boxed(name, &vendor)
            .ok()
            .map(|(value, _)| value)
    };
    let header = value(&HEADER_NAME);
    let values: [Option<Box<[u8]>>; PIECES] =
        core::array::from_fn(|index| value(&piece_name(index)));
    // The stop's reason, for the screen: a stop after the GPU's reset left
    // none of its own.
    let reason = match read(header.as_deref(), &values) {
        Kept::Absent => return,
        Kept::Damaged(why) => {
            output::arguments(format_args!(
                "PREVIOUS BOOT left a stop record in this PC's firmware that cannot be read: \
                 {why}\n"
            ));
            None
        }
        Kept::Record { boot, text } => {
            output::arguments(format_args!(
                "PREVIOUS BOOT stopped (boot {boot} of its USB's log) where the USB's log was \
                 out of reach or the screen may have been dead; this PC's firmware kept its last \
                 lines:\n"
            ));
            for line in text.lines() {
                output::arguments(format_args!("PREVIOUS BOOT {line}\n"));
            }
            output::arguments(format_args!("PREVIOUS BOOT stop record ends\n"));
            Some(stop_reason(&text).map(String::from))
        }
    };
    // On the stick first: the variables are the only other copy.
    output::flush_log();
    let in_log = output::usb_log_kept();
    if let Some(reason) = reason {
        crate::status::warning(&carried_notice(reason.as_deref(), in_log));
    }
    if !in_log {
        output::arguments(format_args!(
            "PREVIOUS BOOT stop record stays in this PC's firmware: the USB's log did not take \
             it\n"
        ));
        return;
    }
    let names = core::iter::once((HEADER_NAME.as_slice(), header.is_some())).chain(
        PIECE_NAMES
            .iter()
            .zip(&values)
            .map(|(name, value)| (name.as_slice(), value.is_some())),
    );
    let mut refused = None;
    // The header first: pieces without it are no record.
    for (name, _) in names.filter(|&(_, held)| held) {
        let deleted = CStr16::from_u16_with_nul(name)
            .map_err(|_| uefi::Status::INVALID_PARAMETER)
            .and_then(|name| {
                runtime::delete_variable(name, &vendor).map_err(|error| error.status())
            });
        if let Err(status) = deleted {
            refused.get_or_insert(status);
        }
    }
    match refused {
        None => output::arguments(format_args!(
            "PREVIOUS BOOT stop record cleared from this PC's firmware\n"
        )),
        Some(status) => output::arguments(format_args!(
            "PREVIOUS BOOT stop record not cleared from this PC's firmware: {status:?}. The \
             next start copies it again\n"
        )),
    }
}

/// Every piece's name, for [`carry`]'s borrow of them.
const PIECE_NAMES: [[u16; 11]; PIECES] = {
    let mut names = [[0; 11]; PIECES];
    let mut index = 0;
    while index < PIECES {
        names[index] = piece_name(index);
        index += 1;
    }
    names
};
